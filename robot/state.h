#pragma once
// The blackboard: everything one thread produces and another consumes.
//
// Each slot has exactly one writer and holds only the latest value -- an old
// scan or a stale camera setpoint is worthless, so nothing queues up behind a
// slow reader. That is what lets the lidar run at 10 Hz, SLAM at its own pace,
// the control loop at command rate and telemetry at 10 Hz without any of them
// waiting on another.
//
// Nothing here talks to hardware, to sockets or to the operator. Modules that
// do take a reference to this and to RobotOptions, never to each other.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include "protocol.h"
#include "control/auto/map_types.h"
#include "control/safety/obstacle_check.h"
#include "systems/rpi/lidar/lidar_distances.h"
#include "systems/rpi/lidar/scan.h"

// Who is driving. The control loop is the only writer: a button on the operator
// side is a *request* (RobotState::explore_requested), never the mode itself, so
// there is one place that decides and one place that acts.
enum class ControlMode : uint8_t {
    Manual = 0,
    Explore = 1,
};

// Latest obstacle verdict: the lidar thread writes, the control loop and
// telemetry read.
class ObstacleState {
public:
    // Called only by the lidar module, once per scan. `commanded_mm_s` is the
    // operator's (or explorer's) currently commanded wheel speed, already in
    // mm/s -- it acts as a floor under the measured closing speed so the zone
    // widens the instant speed is asked for, before two scans exist to measure
    // it. The braking-zone policy lives in obstacle_check.h; this method owns the
    // one piece of state that policy needs: the previous scan, to difference.
    void update(const LidarDistances& distances, const BrakingZone& zone, float commanded_mm_s) {
        const auto now = std::chrono::steady_clock::now();

        SectorSpeeds closing{};  // zero-initialised: "gap steady" until proven otherwise
        const float dt =
            have_prev_ ? std::chrono::duration<float>(now - prev_time_).count() : 0.0f;
        // A gap in the scan stream (lidar reopened, thread stalled) makes the
        // difference meaningless and would fake a huge speed: drop the running
        // estimate and re-seed from the next pair. The upper bound clears the
        // slowest supported lidar period (1 s) with scheduling headroom.
        if (have_prev_ && dt > 0.005f && dt < 1.5f) {
            for (size_t s = 0; s < proto::SECTOR_COUNT; ++s) {
                const float d_now = distanceAt(distances, static_cast<int>(s));
                const float d_prev = distanceAt(prev_distances_, static_cast<int>(s));
                float raw = 0.0f;
                if (std::isfinite(d_now) && d_now > 0.0f && std::isfinite(d_prev) && d_prev > 0.0f)
                    raw = (d_prev - d_now) / dt;  // > 0 when the gap is shrinking
                // The nearest point in a 60-degree wedge hops between scans, so
                // the raw derivative is noisy: smooth it before it drives STOP.
                closing_ema_[s] = kSpeedSmoothing * raw + (1.0f - kSpeedSmoothing) * closing_ema_[s];
                closing[s] = closing_ema_[s];
            }
        } else {
            closing_ema_.fill(0.0f);
        }
        const float floor = commanded_mm_s > 0.0f ? commanded_mm_s : 0.0f;
        for (float& c : closing) c = std::max(c, floor);

        const SectorStatuses next = evaluate(distances, zone, closing);
        prev_distances_ = distances;
        prev_time_ = now;
        have_prev_ = true;

        std::lock_guard<std::mutex> lock(mutex_);
        statuses_ = next;
        updated_at_ = now;
        have_sample_ = true;
    }

    // Anything older than max_age reads as Unknown. A lidar thread that died or
    // hung inside its driver would otherwise keep serving its last verdict
    // forever, and a stale "clear" is exactly the reading that drives a robot
    // into a wall through the drive gate.
    SectorStatuses statuses(std::chrono::milliseconds max_age) const {
        const auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lock(mutex_);
        if (!have_sample_ || now - updated_at_ > max_age) return unknownStatuses();
        return statuses_;
    }

private:
    // How much of a fresh derivative to trust each scan: lower is smoother but
    // lags a real approach; 0.4 keeps the response within a couple of scans.
    static constexpr float kSpeedSmoothing = 0.4f;

    mutable std::mutex mutex_;
    SectorStatuses statuses_ = unknownStatuses();
    std::chrono::steady_clock::time_point updated_at_{};
    bool have_sample_ = false;

    // Touched only by update(), i.e. only by the lidar thread -- outside the
    // mutex, which guards just the published verdict above.
    LidarDistances prev_distances_{};
    std::chrono::steady_clock::time_point prev_time_{};
    bool have_prev_ = false;
    SectorSpeeds closing_ema_{};
};

// Full revolutions from the lidar, for SLAM only. Separate from ObstacleState on
// purpose: that one is the safety path and must keep working even if nothing
// ever reads this.
class ScanState {
public:
    using Clock = std::chrono::steady_clock;

    // Written only by the lidar thread.
    void publish(LaserScan scan) {
        std::lock_guard<std::mutex> lock(mutex_);
        scan_ = std::move(scan);
    }

    // Hands over the scan only if it is newer than the one the caller last saw,
    // so SLAM never matches the same revolution twice and never spins on an
    // empty queue. Pass a default-constructed time_point to take whatever is
    // there.
    bool since(Clock::time_point after, LaserScan& out) const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (scan_.points.empty() || scan_.stamp <= after) return false;
        out = scan_;
        return true;
    }

private:
    mutable std::mutex mutex_;
    LaserScan scan_;
};

// What SLAM knows. Published as an immutable snapshot behind a shared_ptr: the
// grid is 160 KB, and copying that under a lock on every planner cycle would
// stall SLAM for no reason. A reader takes the pointer in nanoseconds and then
// works on its own frozen copy for as long as it likes.
class MapState {
public:
    // Written only by the SLAM thread.
    void publish(std::shared_ptr<const MapSnapshot> snapshot) {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_ = std::move(snapshot);
    }

    // Null until SLAM has produced its first map.
    std::shared_ptr<const MapSnapshot> snapshot() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return snapshot_;
    }

private:
    mutable std::mutex mutex_;
    std::shared_ptr<const MapSnapshot> snapshot_;
};

// What autonomy wants the robot to do. The explorer writes, the control loop
// reads -- and the control loop still runs it through the safety gate, exactly
// like an operator command.
class AutonomyState {
public:
    using Clock = std::chrono::steady_clock;

    // Written only by the explorer thread.
    void publish(const proto::DriveCommand& command) {
        const auto now = Clock::now();
        std::lock_guard<std::mutex> lock(mutex_);
        command_ = command;
        updated_at_ = now;
        have_command_ = true;
    }

    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        have_command_ = false;
    }

    // STOP when the explorer has gone quiet for longer than max_age. An
    // autonomy thread that hung must not leave the robot driving on its last
    // command; this is the same reasoning as the lidar staleness cutoff.
    proto::DriveCommand command(std::chrono::milliseconds max_age) const {
        const auto now = Clock::now();
        std::lock_guard<std::mutex> lock(mutex_);
        if (!have_command_ || now - updated_at_ > max_age)
            return {proto::Direction::STOP, 0.0f};
        return command_;
    }

private:
    mutable std::mutex mutex_;
    proto::DriveCommand command_{proto::Direction::STOP, 0.0f};
    Clock::time_point updated_at_{};
    bool have_command_ = false;
};

struct RobotState {
    std::atomic<bool> running{true};
    std::atomic<bool> failed{false};

    ObstacleState obstacles;  // lidar -> control loop, telemetry
    ScanState scans;          // lidar -> SLAM
    MapState map;             // SLAM  -> explorer, map channel
    AutonomyState autonomy;   // explorer -> control loop

    // Operator asks (map channel), control loop decides and publishes.
    std::atomic<bool> explore_requested{false};
    std::atomic<ControlMode> mode{ControlMode::Manual};

    // Magnitude of the wheel speed the control loop last commanded, 0..1, 0 for
    // STOP. Written by the control loop, read by the lidar thread: it feeds the
    // speed-scaled braking zone as a floor, so the red zone widens the moment
    // the operator asks for speed rather than a scan later.
    std::atomic<float> drive_speed{0.0f};

    // Written by the control loop after a successful servo write, read by
    // telemetry: what the camera was actually commanded to, not what was asked.
    void setAppliedCamera(const proto::CameraState& camera) {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        applied_camera_ = camera;
    }
    void clearAppliedCamera() {
        setAppliedCamera({proto::UNKNOWN_TELEMETRY_VALUE, proto::UNKNOWN_TELEMETRY_VALUE});
    }
    proto::CameraState appliedCamera() const {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        return applied_camera_;
    }

    // A worker that cannot continue stops the whole robot rather than leaving
    // the others running against half-initialized hardware.
    void abort() {
        failed.store(true);
        running.store(false);
    }

private:
    mutable std::mutex camera_mutex_;
    proto::CameraState applied_camera_{proto::UNKNOWN_TELEMETRY_VALUE,
                                       proto::UNKNOWN_TELEMETRY_VALUE};
};

// With no explicit destination, video follows the active command connection.
class VideoTarget {
public:
    explicit VideoTarget(std::string fixed_host) : fixed_host_(std::move(fixed_host)) {}
    void setPeer(const std::string& peer) {
        std::lock_guard<std::mutex> lock(mutex_);
        peer_ = peer;
    }
    void clearPeer() {
        std::lock_guard<std::mutex> lock(mutex_);
        peer_.clear();
    }
    std::string host() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return fixed_host_.empty() ? peer_ : fixed_host_;
    }

private:
    const std::string fixed_host_;
    mutable std::mutex mutex_;
    std::string peer_;
};

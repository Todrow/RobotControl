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

#include <atomic>
#include <chrono>
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
    // Called only by the lidar module, once per scan.
    void update(const LidarDistances& distances, const ObstacleThresholds& thresholds) {
        const SectorStatuses next = evaluate(distances, thresholds);
        const auto now = std::chrono::steady_clock::now();
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
    mutable std::mutex mutex_;
    SectorStatuses statuses_ = unknownStatuses();
    std::chrono::steady_clock::time_point updated_at_{};
    bool have_sample_ = false;
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

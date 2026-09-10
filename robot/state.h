#pragma once
// The blackboard: everything one thread produces and another consumes.
//
// Each slot has exactly one writer and holds only the latest value -- an old
// scan or a stale camera setpoint is worthless, so nothing queues up behind a
// slow reader. That is what lets the lidar run at 10 Hz, the control loop at
// command rate and telemetry at 10 Hz without any of them waiting on another.
//
// Nothing here talks to hardware, to sockets or to the operator. Modules that
// do take a reference to this and to RobotOptions, never to each other.

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <utility>

#include "protocol.h"
#include "control/safety/obstacle_check.h"
#include "systems/rpi/lidar/lidar_distances.h"

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

struct RobotState {
    std::atomic<bool> running{true};
    std::atomic<bool> failed{false};
    ObstacleState obstacles;

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

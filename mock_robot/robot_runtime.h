#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <utility>

#include "protocol.h"
#include "actuators/drive_config.h"
#include "actuators/servo_config.h"
#include "lidar/lidar_config.h"
#include "utils/lidar_distances.h"
#include "utils/obstacle_check.h"

enum class VideoSource { Camera, Test, Disabled };

struct RobotOptions {
    uint16_t command_port = proto::COMMAND_PORT;
    uint16_t telemetry_port = proto::TELEMETRY_PORT;
    uint16_t video_port = proto::VIDEO_PORT;
    std::string video_host;
    VideoSource video_source = VideoSource::Camera;
    int camera_index = 0;
    int width = 1280;
    int height = 720;
    int framerate = 30;
    int bitrate = 4000000;
    DriveOptions drive_uart;
    ServoOptions servos;
    LidarOptions lidar;
};

// Latest obstacle verdict: the lidar thread writes, the telemetry thread reads.
// A single slot, last write wins -- an old scan is worthless, so nothing queues
// up behind a slow reader.
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
    // into a wall once this feeds the drive gate.
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

struct RobotRuntime {
    std::atomic<bool> running{true};
    std::atomic<bool> failed{false};
    ObstacleState obstacles;

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

void runCommandListener(const RobotOptions&, RobotRuntime&, VideoTarget&);
void runLidarReader(const RobotOptions&, RobotRuntime&);
void runTelemetrySender(const RobotOptions&, RobotRuntime&);
void runVideoSender(const RobotOptions&, RobotRuntime&, const VideoTarget&);

#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <utility>

#include "protocol.h"
#include "drive_config.h"
#include "servo_config.h"

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
    int bitrate = 2000000;
    DriveOptions drive_uart;
    ServoOptions servos;
};

struct RobotRuntime {
    std::atomic<bool> running{true};
    std::atomic<bool> failed{false};

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
void runTelemetrySender(const RobotOptions&, RobotRuntime&);
void runVideoSender(const RobotOptions&, RobotRuntime&, const VideoTarget&);

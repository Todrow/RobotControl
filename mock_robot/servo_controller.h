#pragma once

#include <string>

#include "protocol.h"
#include "servo_config.h"

// Initialize, apply and release from the command thread only.
class ServoController {
public:
    explicit ServoController(const ServoOptions& options);
    ~ServoController();

    ServoController(const ServoController&) = delete;
    ServoController& operator=(const ServoController&) = delete;

    bool initialize();
    bool apply(const proto::CameraState& camera);
    bool release() noexcept;

private:
    struct Channel {
        int number = 0;
        std::string path;
        std::string period_path;
        std::string duty_path;
        std::string polarity_path;
        std::string enable_path;
        bool exported = false;
        bool period_valid = false;
        bool enabled = false;
        bool requires_release = false;
        int pulse_us = -1;
    };

    void cleanup() noexcept;
#if defined(__linux__)
    bool prepareChannel(Channel& channel);
#endif

    const ServoOptions options_;
    std::string chip_path_;
    std::string export_path_;
    std::string unexport_path_;
    Channel pitch_;
    Channel yaw_;
    bool initialized_ = false;
};

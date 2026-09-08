#pragma once

#include <string>

#include "drive_config.h"
#include "protocol.h"

struct WheelPower {
    int right = 0;
    int left = 0;
};

// Signed power: positive moves forward, negative moves backward.
// LEFT/RIGHT turn in place; malformed commands produce a stopped setpoint.
WheelPower driveWheelPower(const proto::DriveCommand& command) noexcept;
std::string formatWheelCommand(const WheelPower& power);

// Initialize, apply and stop from the command thread only.
class DriveController {
public:
    explicit DriveController(const DriveOptions& options);
    ~DriveController();

    DriveController(const DriveController&) = delete;
    DriveController& operator=(const DriveController&) = delete;

    bool initialize();
    bool apply(const proto::DriveCommand& command);
    bool stop() noexcept;

private:
    void closeDevice() noexcept;

    const DriveOptions options_;
    int fd_ = -1;
    bool initialized_ = false;
    bool incomplete_frame_ = false;
};

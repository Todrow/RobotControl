#pragma once

#include <string>

#include "drive_config.h"
#include "protocol.h"

struct WheelPower {
    int right = 0;
    int left = 0;
};

// Share of the commanded speed left on the inner wheel of a diagonal; the outer
// wheel keeps the full speed. Lower means a tighter arc.
constexpr float kDiagonalInnerFactor = 0.5f;

// Signed power: positive moves forward, negative moves backward.
// LEFT/RIGHT turn in place. Diagonals drive both wheels the same way and slow
// down the wheel on the side of the turn, so the robot follows an arc; while
// reversing this steers like a car in reverse -- S+D takes the tail to the
// right, which rotates the hull the opposite way to a plain RIGHT command.
// Malformed commands produce a stopped setpoint.
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

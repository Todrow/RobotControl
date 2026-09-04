#pragma once

#include <algorithm>
#include <cmath>
#include <string>

struct ServoAxisConfig {
    int channel = 0;
    int min_us = 500;
    int center_us = 1500;
    int max_us = 2500;
    bool inverted = false;
};

struct ServoOptions {
    bool enabled = false;
    // Empty means the Pi 4 PWM0 device (DT node pwm@7e20c000), not any PWM chip.
    std::string pwm_chip;
    ServoAxisConfig pitch{0, 500, 1500, 2500, false};
    ServoAxisConfig yaw{1, 500, 1500, 2500, false};
};

inline bool validServoAxis(const ServoAxisConfig& axis) {
    return axis.channel >= 0 && axis.channel <= 1 &&
           axis.min_us >= 500 && axis.max_us <= 2500 &&
           axis.min_us < axis.center_us && axis.center_us < axis.max_us;
}

// CameraState contains absolute normalized positions, not angular velocities.
// Return false for malformed values; never turn NaN into a hardware setpoint.
inline bool servoPulseWidth(float value, const ServoAxisConfig& axis, int& pulse_us) {
    if (!std::isfinite(value) || !validServoAxis(axis)) return false;
    const double normalized = std::clamp(static_cast<double>(value), -1.0, 1.0) *
                              (axis.inverted ? -1.0 : 1.0);
    const int span = normalized < 0.0 ? axis.center_us - axis.min_us
                                      : axis.max_us - axis.center_us;
    pulse_us = std::clamp(static_cast<int>(std::lround(axis.center_us + normalized * span)),
                          axis.min_us, axis.max_us);
    return true;
}

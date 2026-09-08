#pragma once

#include "utils/obstacle_check.h"

// How the sector distances are produced. `Device` measures with the real lidar
// (LDROBOT STL-19P on whichever serial port answers); `Simulated` publishes
// moving fake numbers so the operator's display and the thresholds can be
// exercised without hardware; `Disabled` publishes nothing, leaving every
// sector Unknown.
enum class LidarSource { Device, Simulated, Disabled };

struct LidarOptions {
    LidarSource source = LidarSource::Device;
    int period_ms = 100;   // How often the module publishes a fresh sample.
    int max_age_ms = 500;  // Older than this and the sample stops being trusted.
    ObstacleThresholds thresholds;
};

inline bool validLidarOptions(const LidarOptions& options) noexcept {
    return options.period_ms > 0 && options.max_age_ms >= options.period_ms &&
           validThresholds(options.thresholds);
}

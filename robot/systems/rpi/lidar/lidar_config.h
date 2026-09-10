#pragma once

#include "control/safety/obstacle_check.h"

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
    // When set, the command listener refuses a drive command that would push the
    // hull into a sector whose verdict is Red; turning in place and reversing
    // away stay available. Clear it (--obstacle-no-enforce) for report-only.
    bool enforce = true;
    // The real device path (readDistances) clamps every sector to this: a reading
    // further out is dropped and a sector with no echo in range reads as exactly
    // this value. It has to sit above the widest the speed-scaled zone can grow
    // (red_max_mm + yellow_margin_mm), or a clear sector reads Red the moment the
    // closing speed pushes the zone past the clamp. SLAM uses its own, far wider
    // range and is unaffected.
    int sector_range_mm = 1500;
    // The red/yellow bands are speed-scaled: see BrakingZone in obstacle_check.h.
    BrakingZone zone;
};

inline bool validLidarOptions(const LidarOptions& options) noexcept {
    return options.period_ms > 0 && options.max_age_ms >= options.period_ms &&
           validBrakingZone(options.zone) &&
           static_cast<float>(options.sector_range_mm) >=
               options.zone.red_max_mm + options.zone.yellow_margin_mm;
}

#pragma once

#include <string>

// LiDAR (LDROBOT STL-19P) configuration, in the DriveOptions style.
// The STL-19P is handled as LD_19 by the LDROBOT SDK at 230400 baud.
struct LidarOptions {
    bool enabled = false;
    std::string device = "/dev/ttyUSB0";  // USB-serial (CP2102) on the Pi
    float stop_distance_m = 0.10f;         // block a direction closer than this
    float min_valid_m = 0.02f;             // closer than this is treated as noise
    // Front sector is centred on the LiDAR 0 deg mark, back sector on 180 deg;
    // each spans +/- half_width_deg. Adjust half_width to taste.
    float front_offset_deg = 0.0f;
    float half_width_deg = 45.0f;
};

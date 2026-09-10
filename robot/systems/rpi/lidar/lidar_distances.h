#pragma once
// Distance from the robot to the nearest obstacle in each sector, in
// millimetres. This is the lidar module's only output: it measures, it does not
// decide anything. Thresholds live in obstacle_check.h.
#include "protocol.h"

// Field order matches proto::Sector so distanceAt() below stays the single place
// that knows the mapping between names and wire indices.
struct LidarDistances {
    float forward       = proto::UNKNOWN_TELEMETRY_VALUE;
    float forward_right = proto::UNKNOWN_TELEMETRY_VALUE;
    float back_right    = proto::UNKNOWN_TELEMETRY_VALUE;
    float back          = proto::UNKNOWN_TELEMETRY_VALUE;
    float back_left     = proto::UNKNOWN_TELEMETRY_VALUE;
    float forward_left  = proto::UNKNOWN_TELEMETRY_VALUE;
};

// NaN means "not measured": no lidar, the sector was outside the scan, or the
// reading was rejected. It must never be reported as 0, which would read as an
// obstacle touching the robot -- and never as a large number, which would read
// as a clear path.
inline float distanceAt(const LidarDistances& distances, int sector) {
    switch (sector) {
        case proto::SECTOR_FORWARD:       return distances.forward;
        case proto::SECTOR_FORWARD_RIGHT: return distances.forward_right;
        case proto::SECTOR_BACK_RIGHT:    return distances.back_right;
        case proto::SECTOR_BACK:          return distances.back;
        case proto::SECTOR_BACK_LEFT:     return distances.back_left;
        case proto::SECTOR_FORWARD_LEFT:  return distances.forward_left;
        default:                          return proto::UNKNOWN_TELEMETRY_VALUE;
    }
}

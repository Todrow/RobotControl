#pragma once
// Threshold policy: turns measured millimetres into the three-state verdict that
// travels in telemetry. Pure functions only -- no state, no I/O, no threads, so
// this is testable without a robot.
#include <array>
#include <cmath>

#include "protocol.h"
#include "utils/lidar_distances.h"

using SectorStatuses = std::array<proto::SectorStatus, proto::SECTOR_COUNT>;

// Two thresholds, so three bands: [0, red) is Red, [red, yellow) is Yellow, and
// anything further is Green. Tune on the real robot via the CLI, not by rebuilding.
struct ObstacleThresholds {
    float red_mm = 180.0f;
    float yellow_mm = 230.0f;
};

inline bool validThresholds(const ObstacleThresholds& thresholds) noexcept {
    return thresholds.red_mm > 0.0f && thresholds.yellow_mm > thresholds.red_mm;
}

inline SectorStatuses unknownStatuses() {
    SectorStatuses statuses;
    statuses.fill(proto::SectorStatus::Unknown);
    return statuses;
}

// A non-finite or non-positive reading is missing data, not a clear path, so it
// stays Unknown instead of falling through to Green.
inline proto::SectorStatus classify(float millimetres,
                                    const ObstacleThresholds& thresholds) noexcept {
    if (!std::isfinite(millimetres) || millimetres <= 0.0f) return proto::SectorStatus::Unknown;
    if (millimetres < thresholds.red_mm) return proto::SectorStatus::Red;
    if (millimetres < thresholds.yellow_mm) return proto::SectorStatus::Yellow;
    return proto::SectorStatus::Green;
}

// CALL SITE RULE: this runs in exactly one place, ObstacleState::update() in
// robot_runtime.h. Telemetry -- and later the drive gate -- read the published
// verdict instead of re-deriving it. A second caller means a second copy of the
// thresholds, and sooner or later the two disagree: the operator sees a clear
// sector while the robot refuses to move, or worse, the reverse.
inline SectorStatuses evaluate(const LidarDistances& distances,
                               const ObstacleThresholds& thresholds) {
    SectorStatuses statuses = unknownStatuses();
    for (int sector = 0; sector < proto::SECTOR_COUNT; ++sector)
        statuses[static_cast<size_t>(sector)] = classify(distanceAt(distances, sector), thresholds);
    return statuses;
}

#pragma once
// Threshold policy: turns measured millimetres into the three-state verdict that
// travels in telemetry. Pure functions only -- no state, no I/O, no threads, so
// this is testable without a robot.
#include <array>
#include <cmath>

#include "protocol.h"
#include "systems/rpi/lidar/lidar_distances.h"

using SectorStatuses = std::array<proto::SectorStatus, proto::SECTOR_COUNT>;

// Per-sector closing speed in mm/s, indexed by proto::Sector: positive when the
// gap to that sector's nearest obstacle is shrinking.
using SectorSpeeds = std::array<float, proto::SECTOR_COUNT>;

// Two thresholds, so three bands: [0, red) is Red, [red, yellow) is Yellow, and
// anything further is Green. Tune on the real robot via the CLI, not by rebuilding.
struct ObstacleThresholds {
    float red_mm = 250.0f;
    float yellow_mm = 300.0f;
};

inline bool validThresholds(const ObstacleThresholds& thresholds) noexcept {
    return thresholds.red_mm > 0.0f && thresholds.yellow_mm > thresholds.red_mm;
}

// Speed-scaled braking zone. The red threshold is not a fixed distance any more:
// it grows with how fast the gap to the obstacle in a sector is closing, so a
// fast approach trips STOP earlier and a slow crawl still lets the hull get
// close. This struct is pure policy -- the closing speed itself is measured
// elsewhere (ObstacleState::update, from consecutive lidar scans and the
// operator's commanded speed) and passed in.
//
//   red_eff    = clamp(red_base_mm + lookahead_s * closing_mm_s, red_base, red_max)
//   yellow_eff = red_eff + yellow_margin_mm
//
// lookahead_s is "how many seconds of the current closing motion to keep as
// margin": at 0.9 s and a 400 mm/s approach the zone grows by 360 mm.
struct BrakingZone {
    float red_base_mm = 250.0f;      // the zone at rest / when the gap is not closing
    float red_max_mm = 700.0f;       // clamp: the zone never grows past this
    float yellow_margin_mm = 80.0f;  // yellow threshold = effective red + this
    float lookahead_s = 0.9f;        // seconds of closing motion kept as margin
    float max_speed_mm_s = 500.0f;   // wheel speed at full throttle; converts the
                                     // 0..1 commanded speed into mm/s for the floor
};

inline bool validBrakingZone(const BrakingZone& zone) noexcept {
    return zone.red_base_mm > 0.0f && zone.red_max_mm >= zone.red_base_mm &&
           zone.yellow_margin_mm > 0.0f && zone.lookahead_s >= 0.0f &&
           zone.max_speed_mm_s > 0.0f;
}

// The effective thresholds for one sector, given the speed (mm/s, positive when
// the gap is shrinking) at which its nearest obstacle is closing. A non-positive
// speed -- stable gap, or backing away -- leaves the zone at red_base_mm.
inline ObstacleThresholds thresholdsFor(const BrakingZone& zone, float closing_mm_s) noexcept {
    const float extra = closing_mm_s > 0.0f ? zone.lookahead_s * closing_mm_s : 0.0f;
    float red = zone.red_base_mm + extra;
    if (red > zone.red_max_mm) red = zone.red_max_mm;
    return {red, red + zone.yellow_margin_mm};
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

// CALL SITE RULE: these run in exactly one place, ObstacleState::update() in
// state.h. Telemetry and the drive gate read the published verdict instead of
// re-deriving it. A second caller means a second copy of the thresholds, and
// sooner or later the two disagree: the operator sees a clear sector while the
// robot refuses to move, or worse, the reverse.
inline SectorStatuses evaluate(const LidarDistances& distances,
                               const ObstacleThresholds& thresholds) {
    SectorStatuses statuses = unknownStatuses();
    for (int sector = 0; sector < proto::SECTOR_COUNT; ++sector)
        statuses[static_cast<size_t>(sector)] = classify(distanceAt(distances, sector), thresholds);
    return statuses;
}

// Same, but each sector is classified against its own speed-scaled thresholds:
// the red zone in a sector whose obstacle is closing fast reaches further out
// than in one whose gap is steady.
inline SectorStatuses evaluate(const LidarDistances& distances, const BrakingZone& zone,
                               const SectorSpeeds& closing_mm_s) {
    SectorStatuses statuses = unknownStatuses();
    for (int sector = 0; sector < proto::SECTOR_COUNT; ++sector) {
        const ObstacleThresholds thresholds =
            thresholdsFor(zone, closing_mm_s[static_cast<size_t>(sector)]);
        statuses[static_cast<size_t>(sector)] = classify(distanceAt(distances, sector), thresholds);
    }
    return statuses;
}

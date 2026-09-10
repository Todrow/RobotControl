#pragma once
// SLAM: where am I, and what does the room look like.
//
// Scan matching only -- this robot has no encoders and no IMU, so the lidar is
// the sole source of motion information. Each revolution is slid and rotated
// over the map already built until it fits best; that offset *is* the movement
// since the last scan. Then the scan is burned into the map from the pose that
// won, and the next revolution matches against the improved map.
//
// What this deliberately is not: there is no pose graph and no loop closure.
// Drift accumulates and never gets corrected -- drive a long loop and the far
// end will not line up with the near end. That is the honest limit of one file
// of scan matching, and it is fine for exploring a room. Fixing it is a
// different class of problem (graph optimisation), not a tweak to this one.
//
// Where it fails, concretely: a long blank corridor. Sliding a scan along a
// featureless wall costs nothing in match score, so the robot cannot tell how
// far it has travelled and the map compresses. Corners, doorways and furniture
// are what make matching work.

#include <cstdint>
#include <vector>

#include "control/auto/map_types.h"
#include "options.h"
#include "state.h"
#include "systems/rpi/lidar/scan.h"

class Slam {
public:
    explicit Slam(const SlamOptions& options) : options_(options) {}

    // Feeds one revolution. Returns false when the scan was rejected: too few
    // points, or it did not match the map well enough to be trusted. A rejected
    // scan changes nothing -- neither pose nor map -- because writing a badly
    // located scan into the grid corrupts the map permanently, and a corrupt map
    // makes every later match worse.
    bool update(const LaserScan& scan);

    const Pose2D& pose() const noexcept { return pose_; }
    const OccupancyGrid& grid() const noexcept { return grid_; }
    float matchScore() const noexcept { return score_; }
    bool started() const noexcept { return started_; }

private:
    // Best pose near `seed` on a grid of offsets: `radius_xy` steps of `step_m`
    // in x and y, `radius_theta` steps of `step_rad` in heading. `coarse` picks
    // the low-resolution field for the wide first pass.
    Pose2D searchAround(const LaserScan& scan, const Pose2D& seed, float step_m, float step_rad,
                        int radius_xy, int radius_theta, bool coarse, float& best_score) const;

    // Mean likelihood under the scan points, 0..1, measured only where the map
    // knows something. Returns 0 when too little of the scan overlaps the known
    // map to judge anything.
    float scoreAt(const LaserScan& scan, const Pose2D& pose, bool coarse) const;

    // Ray-cast every point: cells along the beam become free, the endpoint
    // becomes occupied, and the gaps between diverging beams are filled in.
    void integrate(const LaserScan& scan, const Pose2D& pose);

    // Clear the cells between the robot and a world point, without ever placing
    // a wall.
    void traceFree(int rx, int ry, float wx, float wy);

    // Rebuild both search fields from the grid, after integration.
    void rebuildFields();

    const SlamOptions options_;
    Pose2D pose_;
    OccupancyGrid grid_;

    // Likelihood field, 0..255 per cell, rebuilt from the grid after every
    // accepted scan. Scoring against raw log-odds does not work: a wall cell
    // holds +12 after one sighting and only saturates near +127 after a dozen,
    // so an honest match would score near zero early on -- exactly when the
    // robot most needs to keep tracking. Worse, a scan taken 5 cm further along
    // lands in the *next* cell and scores nothing at all. The field fixes both:
    // it normalises against the "decided" threshold rather than saturation, and
    // it spreads each wall over its neighbours so a near miss still counts.
    std::vector<uint8_t> field_;
    // 4x4 max-pooled copy of the field for the wide search: 16x less work, and
    // broad peaks a coarse step cannot fall between.
    std::vector<uint8_t> coarse_;

    // Which cells the map has an opinion about at all (free or occupied, as
    // opposed to never seen). Scoring counts a scan point only where the map can
    // actually agree or disagree with it -- see scoreAt(). Same 4x4 pooling for
    // the coarse pass, so a 20 cm search step does not fall into a hole.
    std::vector<uint8_t> known_;
    std::vector<uint8_t> coarse_known_;

    float score_ = 0.0f;
    bool started_ = false;
};

// SLAM thread: reads scans from RobotState, publishes pose + map snapshots.
void runSlam(const RobotOptions& options, RobotState& state);

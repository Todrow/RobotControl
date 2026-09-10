#include "control/auto/slam.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>

#include "systems/rpi/link/socket_utils.h"

namespace {

constexpr float kPi = 3.14159265358979f;

// Side of one coarse cell, in fine cells. 4 * 5 cm = 20 cm blocks.
constexpr int kCoarseFactor = 4;
constexpr int kCoarseCells = OccupancyGrid::kCells / kCoarseFactor;

// How much a wall bleeds into its neighbours in the likelihood field. Without
// this, a scan taken half a cell further along scores zero and the match is
// lost; with too much, everything scores well and the peak stops being sharp
// enough to locate the robot. One cell of strong bleed and one of weak is
// enough for 5 cm cells.
constexpr int kFieldPeak = 255;
constexpr int kFieldNear = 150;  // the 8 cells touching a wall
constexpr int kFieldFar = 60;    // the ring beyond those

float wrapAngle(float radians) noexcept {
    while (radians > kPi) radians -= 2.0f * kPi;
    while (radians < -kPi) radians += 2.0f * kPi;
    return radians;
}

}  // namespace

// Mean likelihood under the scan, once it is placed at `pose`, normalised 0..1.
//
// Only walls contribute. Scoring free cells negatively sounds sensible and is
// actively harmful here: a scan hanging over unexplored space would then beat a
// scan sitting on a wall it matches imperfectly, and the robot would rather
// believe it is somewhere it has never been.
float Slam::scoreAt(const LaserScan& scan, const Pose2D& pose, bool coarse) const {
    const float cos_t = std::cos(pose.theta);
    const float sin_t = std::sin(pose.theta);
    int total = 0;
    int counted = 0;
    for (const ScanPoint& point : scan.points) {
        const float wx = pose.x + point.x * cos_t - point.y * sin_t;
        const float wy = pose.y + point.x * sin_t + point.y * cos_t;
        int cx = 0;
        int cy = 0;
        if (!OccupancyGrid::toCell(wx, wy, cx, cy)) continue;
        ++counted;
        if (coarse) {
            const int bx = cx / kCoarseFactor;
            const int by = cy / kCoarseFactor;
            total += coarse_[static_cast<size_t>(by) * kCoarseCells + bx];
        } else {
            total += field_[static_cast<size_t>(cy) * OccupancyGrid::kCells + cx];
        }
    }
    if (counted == 0) return 0.0f;
    return static_cast<float>(total) / (static_cast<float>(counted) * kFieldPeak);
}

Pose2D Slam::searchAround(const LaserScan& scan, const Pose2D& seed, float step_m, float step_rad,
                          int radius_xy, int radius_theta, bool coarse, float& best_score) const {
    Pose2D best = seed;
    best_score = scoreAt(scan, seed, coarse);
    for (int dt = -radius_theta; dt <= radius_theta; ++dt) {
        for (int dy = -radius_xy; dy <= radius_xy; ++dy) {
            for (int dx = -radius_xy; dx <= radius_xy; ++dx) {
                if (dx == 0 && dy == 0 && dt == 0) continue;
                Pose2D candidate;
                candidate.x = seed.x + static_cast<float>(dx) * step_m;
                candidate.y = seed.y + static_cast<float>(dy) * step_m;
                candidate.theta = wrapAngle(seed.theta + static_cast<float>(dt) * step_rad);
                const float score = scoreAt(scan, candidate, coarse);
                if (score > best_score) {
                    best_score = score;
                    best = candidate;
                }
            }
        }
    }
    return best;
}

// Bresenham along every beam. Everything the beam passed through is free, the
// endpoint is a wall.
//
// The free-space marking is what makes exploration possible at all: frontiers
// are the boundary between free and unknown, so without carving out free space
// there is nothing to explore toward.
void Slam::integrate(const LaserScan& scan, const Pose2D& pose) {
    const float cos_t = std::cos(pose.theta);
    const float sin_t = std::sin(pose.theta);
    int rx = 0;
    int ry = 0;
    if (!OccupancyGrid::toCell(pose.x, pose.y, rx, ry)) return;

    for (const ScanPoint& point : scan.points) {
        const float wx = pose.x + point.x * cos_t - point.y * sin_t;
        const float wy = pose.y + point.x * sin_t + point.y * cos_t;
        int ex = 0;
        int ey = 0;
        const bool endpoint_inside = OccupancyGrid::toCell(wx, wy, ex, ey);

        int x = rx;
        int y = ry;
        const int step_x = ex > rx ? 1 : (ex < rx ? -1 : 0);
        const int step_y = ey > ry ? 1 : (ey < ry ? -1 : 0);
        const int delta_x = std::abs(ex - rx);
        const int delta_y = std::abs(ey - ry);
        int error = delta_x - delta_y;
        // Guard against a runaway walk if the endpoint is far outside the grid.
        int guard = 4 * OccupancyGrid::kCells;

        while ((x != ex || y != ey) && guard-- > 0) {
            if (!OccupancyGrid::inside(x, y)) break;
            grid_.observe(x, y, OccupancyGrid::kMissDelta);
            const int doubled = 2 * error;
            if (doubled > -delta_y) {
                error -= delta_y;
                x += step_x;
            }
            if (doubled < delta_x) {
                error += delta_x;
                y += step_y;
            }
        }
        // A beam that flew off the map hit nothing: mark the free space it
        // crossed, but never invent a wall at the edge of the grid.
        if (endpoint_inside) grid_.observe(ex, ey, OccupancyGrid::kHitDelta);
    }
}

void Slam::rebuildFields() {
    field_.assign(static_cast<size_t>(OccupancyGrid::kCells) * OccupancyGrid::kCells, 0);

    const auto raise = [this](int cx, int cy, int value) {
        if (!OccupancyGrid::inside(cx, cy)) return;
        const size_t index = static_cast<size_t>(cy) * OccupancyGrid::kCells + cx;
        if (value > field_[index]) field_[index] = static_cast<uint8_t>(value);
    };

    for (int cy = 0; cy < OccupancyGrid::kCells; ++cy) {
        for (int cx = 0; cx < OccupancyGrid::kCells; ++cx) {
            const int8_t evidence = grid_.at(cx, cy);
            if (evidence <= 0) continue;
            // Normalised against "decided", not against saturation: a cell seen
            // once already counts for most of its final weight, so tracking
            // works from the second scan instead of after a dozen.
            const int strength =
                std::min<int>(evidence, OccupancyGrid::kOccupiedAt) * kFieldPeak /
                OccupancyGrid::kOccupiedAt;
            raise(cx, cy, strength);
            for (int oy = -2; oy <= 2; ++oy) {
                for (int ox = -2; ox <= 2; ++ox) {
                    if (ox == 0 && oy == 0) continue;
                    const bool touching = std::abs(ox) <= 1 && std::abs(oy) <= 1;
                    raise(cx + ox, cy + oy,
                          strength * (touching ? kFieldNear : kFieldFar) / kFieldPeak);
                }
            }
        }
    }

    coarse_.assign(static_cast<size_t>(kCoarseCells) * kCoarseCells, 0);
    for (int cy = 0; cy < OccupancyGrid::kCells; ++cy) {
        for (int cx = 0; cx < OccupancyGrid::kCells; ++cx) {
            const uint8_t value = field_[static_cast<size_t>(cy) * OccupancyGrid::kCells + cx];
            if (value == 0) continue;
            const size_t index =
                static_cast<size_t>(cy / kCoarseFactor) * kCoarseCells + (cx / kCoarseFactor);
            // Maximum, not mean: the wide search must find a wall that fills only
            // one cell of the block, and averaging would dilute it away.
            if (value > coarse_[index]) coarse_[index] = value;
        }
    }
}

bool Slam::update(const LaserScan& scan) {
    if (static_cast<int>(scan.points.size()) < options_.min_points) return false;

    if (!started_) {
        // The first revolution defines the map frame. There is nothing to match
        // against, so the robot is at the origin by definition.
        pose_ = Pose2D{};
        integrate(scan, pose_);
        rebuildFields();
        started_ = true;
        score_ = 1.0f;
        return true;
    }

    // Three passes, wide and blurred first, then narrow and sharp. A single pass
    // fine enough to be accurate would have to cover the whole window at 1 cm
    // steps -- hundreds of times the work for the same answer.
    const float coarse_step = OccupancyGrid::kResolution * kCoarseFactor;  // 20 cm
    const int coarse_radius =
        std::max(1, static_cast<int>(options_.search_radius_m / coarse_step + 0.5f));
    const float coarse_step_rad = 6.0f * kPi / 180.0f;
    const int coarse_turns =
        std::max(1, static_cast<int>(options_.search_radius_deg / 6.0f + 0.5f));

    float score = 0.0f;
    Pose2D best = searchAround(scan, pose_, coarse_step, coarse_step_rad, coarse_radius,
                               coarse_turns, true, score);
    best = searchAround(scan, best, OccupancyGrid::kResolution, 2.0f * kPi / 180.0f, 4, 3, false,
                        score);
    best = searchAround(scan, best, OccupancyGrid::kResolution / 4.0f, 0.5f * kPi / 180.0f, 3, 3,
                        false, score);

    score_ = score;
    if (score < options_.min_score) {
        // Lost. Keep the last good pose and leave the map untouched: a scan
        // written at the wrong place is permanent damage, while a skipped scan
        // costs one cycle and the next one may match fine.
        return false;
    }

    pose_ = best;
    integrate(scan, pose_);
    rebuildFields();
    return true;
}

void runSlam(const RobotOptions& options, RobotState& state) {
    if (!options.slam.enabled) {
        std::printf("[slam] disabled; no map is built\n");
        return;
    }
    if (options.lidar.source == LidarSource::Disabled) {
        std::printf("[slam] no lidar; nothing to map\n");
        return;
    }
    std::printf("[slam] scan matching only (no odometry); %dx%d cells at %.0f cm\n",
                OccupancyGrid::kCells, OccupancyGrid::kCells,
                static_cast<double>(OccupancyGrid::kResolution * 100.0f));

    Slam slam(options.slam);
    LaserScan scan;
    auto last_stamp = std::chrono::steady_clock::time_point{};
    int rejected_in_a_row = 0;

    while (state.running.load()) {
        if (!state.scans.since(last_stamp, scan)) {
            // No new revolution yet. Poll well inside one lidar period so a scan
            // is never left sitting while the robot keeps moving.
            net::sleepUntil(net::Clock::now() + std::chrono::milliseconds(20), state.running);
            continue;
        }
        last_stamp = scan.stamp;

        if (slam.update(scan)) {
            rejected_in_a_row = 0;
        } else if (slam.started() && ++rejected_in_a_row % 20 == 1) {
            std::fprintf(stderr, "[slam] scan rejected, match %.2f; pose held\n",
                         static_cast<double>(slam.matchScore()));
        }
        if (!slam.started()) continue;

        // Publish a frozen copy. Readers hold the shared_ptr for as long as they
        // like without blocking the next cycle.
        auto snapshot = std::make_shared<MapSnapshot>();
        snapshot->pose = slam.pose();
        snapshot->grid = slam.grid();
        snapshot->match_score = slam.matchScore();
        state.map.publish(std::move(snapshot));
    }
}

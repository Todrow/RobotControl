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

// How far a wall bleeds, in metres, converted to whatever that is in cells.
constexpr float kBlurMetres = 0.10f;
const int kBlurCells =
    std::max(1, static_cast<int>(kBlurMetres / OccupancyGrid::kResolution + 0.5f));

// Fewest scan points that have to fall on known ground before a score means
// anything. Below this the robot is essentially somewhere new and the previous
// pose is the best estimate available.
constexpr int kMinMatchedPoints = 25;

// Two neighbouring beams whose ranges differ by more than this are looking at
// different surfaces, so nothing is interpolated between them.
constexpr float kSameSurfaceStepM = 0.25f;
// Ceiling on interpolated beams per gap, so one distant wall cannot turn a
// single scan into tens of thousands of ray-casts.
constexpr int kMaxInterpolatedBeams = 8;

float wrapAngle(float radians) noexcept {
    while (radians > kPi) radians -= 2.0f * kPi;
    while (radians < -kPi) radians += 2.0f * kPi;
    return radians;
}

}  // namespace

// How well the scan agrees with the map, 0..1, once it is placed at `pose`.
//
// Only points landing on ground the map has already seen are counted. This is
// the difference between "I do not know where I am" and "I am looking somewhere
// new", and getting it wrong is what made the robot stop every time it turned
// toward unexplored space: those points cannot agree or disagree with anything,
// so counting them in the denominator halved the score for doing exactly what an
// explorer is supposed to do.
//
// Points on known-free cells still count, and score badly -- that is a genuine
// disagreement (a wall where the map says floor) and the matcher must feel it.
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
        if (coarse) {
            const int bx = cx / kCoarseFactor;
            const int by = cy / kCoarseFactor;
            const size_t index = static_cast<size_t>(by) * kCoarseCells + bx;
            if (!coarse_known_[index]) continue;
            ++counted;
            total += coarse_[index];
        } else {
            const size_t index = static_cast<size_t>(cy) * OccupancyGrid::kCells + cx;
            if (!known_[index]) continue;
            ++counted;
            total += field_[index];
        }
    }
    // Too little overlap to judge. Reporting a confident score off a handful of
    // points would let a chance alignment of five readings move the robot.
    if (counted < kMinMatchedPoints) return 0.0f;
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

// Clear every cell between the robot and a world point. Free space only: this
// never places a wall, which is what makes it safe for the interpolated beams
// below.
void Slam::traceFree(int rx, int ry, float wx, float wy) {
    int ex = 0;
    int ey = 0;
    OccupancyGrid::toCell(wx, wy, ex, ey);

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
}

// Ray-cast every beam, then fill in the gaps between them.
//
// The free-space marking is what makes exploration possible at all: frontiers
// are the boundary between free and unknown, so without carving out free space
// there is nothing to explore toward.
//
// One line per beam is not enough, though. 360 beams diverge as they travel: at
// 5 m two neighbours are 9 cm apart while a cell is 5 cm, so the cells between
// them are never touched and stay Unknown inside an area the robot can see
// perfectly well. Every one of those holes is a frontier, which is what had the
// explorer chasing targets half a metre away instead of driving down the
// corridor in front of it.
//
// So neighbouring beams that clearly hit the same surface get extra beams
// interpolated between them. The extras only ever clear space, never place a
// wall. Where two neighbours disagree wildly about range -- the edge of a
// doorway -- nothing is interpolated, because there really is unseen space
// behind that edge, and inventing floor there would be a lie the planner acts on.
void Slam::integrate(const LaserScan& scan, const Pose2D& pose) {
    const float cos_t = std::cos(pose.theta);
    const float sin_t = std::sin(pose.theta);
    int rx = 0;
    int ry = 0;
    if (!OccupancyGrid::toCell(pose.x, pose.y, rx, ry)) return;

    const size_t count = scan.points.size();
    std::vector<float> bearing(count);
    std::vector<float> range(count);
    for (size_t i = 0; i < count; ++i) {
        bearing[i] = std::atan2(scan.points[i].y, scan.points[i].x);
        range[i] = std::sqrt(scan.points[i].x * scan.points[i].x +
                             scan.points[i].y * scan.points[i].y);
    }
    // Sorted by bearing, so "neighbouring beam" means what it says whatever
    // order the driver handed the points over in.
    std::vector<size_t> order(count);
    for (size_t i = 0; i < count; ++i) order[i] = i;
    std::sort(order.begin(), order.end(),
              [&bearing](size_t a, size_t b) { return bearing[a] < bearing[b]; });

    const auto toWorld = [&](float px, float py, float& wx, float& wy) {
        wx = pose.x + px * cos_t - py * sin_t;
        wy = pose.y + px * sin_t + py * cos_t;
    };

    for (size_t i = 0; i < count; ++i) {
        const ScanPoint& point = scan.points[order[i]];
        float wx = 0.0f;
        float wy = 0.0f;
        toWorld(point.x, point.y, wx, wy);
        int ex = 0;
        int ey = 0;
        const bool endpoint_inside = OccupancyGrid::toCell(wx, wy, ex, ey);
        traceFree(rx, ry, wx, wy);
        // A beam that flew off the map hit nothing: mark the free space it
        // crossed, but never invent a wall at the edge of the grid.
        if (endpoint_inside) grid_.observe(ex, ey, OccupancyGrid::kHitDelta);
    }

    for (size_t i = 0; i + 1 < count; ++i) {
        const size_t a = order[i];
        const size_t b = order[i + 1];
        if (std::fabs(range[a] - range[b]) > kSameSurfaceStepM) continue;
        const float gap = std::fabs(wrapAngle(bearing[b] - bearing[a]));
        const float arc = gap * std::max(range[a], range[b]);
        const int extra = std::min(kMaxInterpolatedBeams,
                                   static_cast<int>(arc / (OccupancyGrid::kResolution * 0.8f)));
        if (extra <= 0) continue;
        // Stop one cell short of the surface, so an interpolated beam can never
        // erase the wall its own neighbours just placed.
        const float shrink = 1.0f - OccupancyGrid::kResolution / std::max(range[a], 0.2f);
        for (int step = 1; step <= extra; ++step) {
            const float t = static_cast<float>(step) / static_cast<float>(extra + 1);
            const float px = scan.points[a].x + (scan.points[b].x - scan.points[a].x) * t;
            const float py = scan.points[a].y + (scan.points[b].y - scan.points[a].y) * t;
            float wx = 0.0f;
            float wy = 0.0f;
            toWorld(px * shrink, py * shrink, wx, wy);
            traceFree(rx, ry, wx, wy);
        }
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
            // Bleed a fixed DISTANCE, not a fixed number of cells: the point is
            // that a scan taken 10 cm further along still finds the wall, and
            // that is a property of the room, not of the grid. Spelling it in
            // cells would double the blur when the cell size doubles and turn
            // the match peak to mush.
            for (int oy = -kBlurCells; oy <= kBlurCells; ++oy) {
                for (int ox = -kBlurCells; ox <= kBlurCells; ++ox) {
                    if (ox == 0 && oy == 0) continue;
                    const bool touching = std::abs(ox) <= 1 && std::abs(oy) <= 1;
                    raise(cx + ox, cy + oy,
                          strength * (touching ? kFieldNear : kFieldFar) / kFieldPeak);
                }
            }
        }
    }

    coarse_.assign(static_cast<size_t>(kCoarseCells) * kCoarseCells, 0);
    known_.assign(static_cast<size_t>(OccupancyGrid::kCells) * OccupancyGrid::kCells, 0);
    coarse_known_.assign(static_cast<size_t>(kCoarseCells) * kCoarseCells, 0);
    for (int cy = 0; cy < OccupancyGrid::kCells; ++cy) {
        for (int cx = 0; cx < OccupancyGrid::kCells; ++cx) {
            const size_t fine = static_cast<size_t>(cy) * OccupancyGrid::kCells + cx;
            const size_t block =
                static_cast<size_t>(cy / kCoarseFactor) * kCoarseCells + (cx / kCoarseFactor);
            // "Known" means any evidence at all, not the free/occupied verdict: a
            // cell a beam has crossed once already says something a scan point can
            // contradict, and waiting for it to cross the verdict threshold would
            // leave the denominator empty exactly when the map is youngest.
            if (grid_.at(cx, cy) != 0) {
                known_[fine] = 1;
                coarse_known_[block] = 1;
            }
            const uint8_t value = field_[fine];
            if (value == 0) continue;
            // Maximum, not mean: the wide search must find a wall that fills only
            // one cell of the block, and averaging would dilute it away.
            if (value > coarse_[block]) coarse_[block] = value;
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

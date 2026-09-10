#include "control/auto/explore.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>

#include "systems/rpi/link/socket_utils.h"

namespace {

constexpr float kPi = 3.14159265358979f;

// How far ahead on the path to aim. Too short and the robot wobbles chasing the
// point under its own nose; too long and it cuts corners into walls.
constexpr float kLookaheadM = 0.35f;

// Below this the match score means SLAM has lost track. Driving on a pose that
// is wrong is worse than not driving: the robot would map garbage and plan
// against it.
constexpr float kTrustedScore = 0.30f;

float wrapAngle(float radians) noexcept {
    while (radians > kPi) radians -= 2.0f * kPi;
    while (radians < -kPi) radians += 2.0f * kPi;
    return radians;
}

proto::DriveCommand stop() noexcept { return {proto::Direction::STOP, 0.0f}; }

}  // namespace

void Explorer::reset() {
    path_.clear();
    next_point_ = 0;
    finished_ = false;
    planned_at_ = {};
}

void Explorer::buildDrivable(const OccupancyGrid& grid) {
    const int cells = OccupancyGrid::kCells;
    drivable_.assign(static_cast<size_t>(cells) * cells, 0);

    // Pass one: free cells are candidates.
    for (int cy = 0; cy < cells; ++cy)
        for (int cx = 0; cx < cells; ++cx)
            if (grid.cell(cx, cy) == Cell::Free)
                drivable_[static_cast<size_t>(cy) * cells + cx] = 1;

    // Pass two: erase everything within the robot radius of a wall. Doing this
    // on the map instead of inside the planner means the path is safe by
    // construction -- there is no "did we remember to check clearance" question
    // at any later step.
    const int margin = static_cast<int>(options_.robot_radius_m / OccupancyGrid::kResolution + 0.5f);
    if (margin <= 0) return;
    std::vector<uint8_t> inflated = drivable_;
    for (int cy = 0; cy < cells; ++cy) {
        for (int cx = 0; cx < cells; ++cx) {
            if (grid.cell(cx, cy) != Cell::Occupied) continue;
            for (int oy = -margin; oy <= margin; ++oy) {
                for (int ox = -margin; ox <= margin; ++ox) {
                    if (ox * ox + oy * oy > margin * margin) continue;
                    const int nx = cx + ox;
                    const int ny = cy + oy;
                    if (!OccupancyGrid::inside(nx, ny)) continue;
                    inflated[static_cast<size_t>(ny) * cells + nx] = 0;
                }
            }
        }
    }
    drivable_.swap(inflated);
}

bool Explorer::planPath(const MapSnapshot& snapshot) {
    const int cells = OccupancyGrid::kCells;
    buildDrivable(snapshot.grid);

    int start_x = 0;
    int start_y = 0;
    if (!OccupancyGrid::toCell(snapshot.pose.x, snapshot.pose.y, start_x, start_y)) return false;

    // The robot sits where it sits: if inflation erased its own cell (hugging a
    // wall), refusing to plan would strand it. Let the search start there anyway
    // and rely on the gate for safety on the way out.
    std::vector<int> parent(static_cast<size_t>(cells) * cells, -1);
    std::deque<int> queue;
    const int start = start_y * cells + start_x;
    parent[static_cast<size_t>(start)] = start;
    queue.push_back(start);

    int goal = -1;
    while (!queue.empty() && goal < 0) {
        const int current = queue.front();
        queue.pop_front();
        const int cx = current % cells;
        const int cy = current / cells;

        // A frontier: known-free here, unknown next door. Reached breadth-first,
        // so the first one found is the closest by travel distance.
        bool frontier = false;
        for (int oy = -1; oy <= 1 && !frontier; ++oy)
            for (int ox = -1; ox <= 1 && !frontier; ++ox)
                if (snapshot.grid.cell(cx + ox, cy + oy) == Cell::Unknown) frontier = true;
        // Ignore frontiers we are already standing on; the robot must actually
        // travel somewhere or it would declare victory on the spot.
        if (frontier && current != start) {
            const float dx = static_cast<float>(cx - start_x) * OccupancyGrid::kResolution;
            const float dy = static_cast<float>(cy - start_y) * OccupancyGrid::kResolution;
            if (std::sqrt(dx * dx + dy * dy) > options_.goal_tolerance_m) {
                goal = current;
                break;
            }
        }

        for (int oy = -1; oy <= 1; ++oy) {
            for (int ox = -1; ox <= 1; ++ox) {
                if (ox == 0 && oy == 0) continue;
                const int nx = cx + ox;
                const int ny = cy + oy;
                if (!OccupancyGrid::inside(nx, ny)) continue;
                const int next = ny * cells + nx;
                if (parent[static_cast<size_t>(next)] >= 0) continue;
                if (!drivable_[static_cast<size_t>(next)]) continue;
                parent[static_cast<size_t>(next)] = current;
                queue.push_back(next);
            }
        }
    }

    path_.clear();
    next_point_ = 0;
    if (goal < 0) return false;

    for (int node = goal; node != start; node = parent[static_cast<size_t>(node)]) {
        float wx = 0.0f;
        float wy = 0.0f;
        OccupancyGrid::toWorld(node % cells, node / cells, wx, wy);
        path_.push_back({wx, wy});
    }
    std::reverse(path_.begin(), path_.end());
    return !path_.empty();
}

proto::DriveCommand Explorer::followPath(const Pose2D& pose) {
    // Drop points already behind us, then aim at the first one far enough ahead.
    while (next_point_ < path_.size()) {
        const float dx = path_[next_point_].x - pose.x;
        const float dy = path_[next_point_].y - pose.y;
        if (std::sqrt(dx * dx + dy * dy) >= kLookaheadM) break;
        ++next_point_;
    }
    if (next_point_ >= path_.size()) {
        // Arrived: the next cycle re-plans toward the next frontier.
        path_.clear();
        next_point_ = 0;
        return stop();
    }

    const float dx = path_[next_point_].x - pose.x;
    const float dy = path_[next_point_].y - pose.y;
    const float bearing = wrapAngle(std::atan2(dy, dx) - pose.theta);
    const float degrees = bearing * 180.0f / kPi;

    // Turning in place first, then arcing, then straight. Pivoting costs a
    // moment but keeps the robot off the wall it would otherwise clip while
    // swinging round; arcs are for fine corrections only.
    if (std::fabs(degrees) > options_.turn_in_place_deg)
        return {degrees > 0.0f ? proto::Direction::LEFT : proto::Direction::RIGHT, options_.speed};
    if (std::fabs(degrees) > 10.0f)
        return {degrees > 0.0f ? proto::Direction::FORWARD_LEFT : proto::Direction::FORWARD_RIGHT,
                options_.speed};
    return {proto::Direction::FORWARD, options_.speed};
}

proto::DriveCommand Explorer::step(const MapSnapshot& snapshot) {
    if (snapshot.match_score < kTrustedScore) {
        // SLAM is lost. Stop and let it re-acquire; the pose it would give us is
        // not worth planning against.
        path_.clear();
        next_point_ = 0;
        return stop();
    }

    const auto now = std::chrono::steady_clock::now();
    const bool due =
        path_.empty() || now - planned_at_ >= std::chrono::milliseconds(options_.replan_period_ms);
    if (due) {
        planned_at_ = now;
        if (!planPath(snapshot)) {
            // Nothing reachable left to see.
            finished_ = true;
            return stop();
        }
        finished_ = false;
    }
    return followPath(snapshot.pose);
}

void runExplorer(const RobotOptions& options, RobotState& state) {
    Explorer explorer(options.explore);
    bool was_exploring = false;
    bool announced_finish = false;

    while (state.running.load()) {
        if (state.mode.load() != ControlMode::Explore) {
            if (was_exploring) {
                // Leaving autonomy: drop the plan and the command, so a later
                // run starts fresh instead of resuming a stale path.
                explorer.reset();
                state.autonomy.clear();
                was_exploring = false;
                announced_finish = false;
            }
            net::sleepUntil(net::Clock::now() + std::chrono::milliseconds(50), state.running);
            continue;
        }
        if (!was_exploring) {
            explorer.reset();
            std::printf("[explore] started\n");
            was_exploring = true;
        }

        const auto snapshot = state.map.snapshot();
        if (!snapshot) {
            // No map yet: SLAM has not seen its first scan.
            state.autonomy.publish({proto::Direction::STOP, 0.0f});
            net::sleepUntil(net::Clock::now() + std::chrono::milliseconds(50), state.running);
            continue;
        }

        state.autonomy.publish(explorer.step(*snapshot));
        if (explorer.finished() && !announced_finish) {
            std::printf("[explore] no reachable frontier left; area mapped\n");
            announced_finish = true;
        } else if (!explorer.finished()) {
            announced_finish = false;
        }

        // Faster than the command period so the control loop always has a fresh
        // command, but slow enough that planning does not dominate the CPU.
        net::sleepUntil(net::Clock::now() + std::chrono::milliseconds(50), state.running);
    }
    state.autonomy.clear();
}

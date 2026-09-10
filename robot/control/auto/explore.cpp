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

// How far from a frontier cell to look for somewhere the robot can actually
// stand. Has to cover the inflation radius, or every doorway looks unreachable.
constexpr int kGoalReach = 6;

// Stop counting unknown space behind a frontier once this much has been found:
// past here it is plainly a real place to go, and flooding an entire unexplored
// floor to learn that would be wasted work.
constexpr int kRevealedCap = 300;

float wrapAngle(float radians) noexcept {
    while (radians > kPi) radians -= 2.0f * kPi;
    while (radians < -kPi) radians += 2.0f * kPi;
    return radians;
}

proto::DriveCommand stop() noexcept { return {proto::Direction::STOP, 0.0f}; }

bool isFrontier(const OccupancyGrid& grid, int cx, int cy) {
    if (grid.cell(cx, cy) != Cell::Free) return false;
    for (int oy = -1; oy <= 1; ++oy)
        for (int ox = -1; ox <= 1; ++ox)
            if (grid.cell(cx + ox, cy + oy) == Cell::Unknown) return true;
    return false;
}

}  // namespace

void Explorer::reset() {
    path_.clear();
    next_point_ = 0;
    have_goal_ = false;
    finished_ = false;
    planned_at_ = {};
    goal_chosen_at_ = {};
}

void Explorer::buildDrivable(const OccupancyGrid& grid) {
    const int cells = OccupancyGrid::kCells;
    drivable_.assign(static_cast<size_t>(cells) * cells, 0);

    for (int cy = 0; cy < cells; ++cy)
        for (int cx = 0; cx < cells; ++cx)
            if (grid.cell(cx, cy) == Cell::Free)
                drivable_[static_cast<size_t>(cy) * cells + cx] = 1;

    // Erase everything within the robot radius of a wall. Doing this on the map
    // instead of inside the planner means the path is safe by construction --
    // there is no "did we remember to check clearance" question at a later step.
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
    // wall), refusing to plan would strand it. Start the search there anyway and
    // rely on the gate for safety on the way out.
    std::vector<int> parent(static_cast<size_t>(cells) * cells, -1);
    std::vector<int> steps(static_cast<size_t>(cells) * cells, 0);
    std::deque<int> queue;
    const int start = start_y * cells + start_x;
    parent[static_cast<size_t>(start)] = start;
    queue.push_back(start);

    // One full sweep, recording every reachable cell. Frontiers are collected as
    // we go; the choice between them happens afterwards, when their sizes are
    // known -- a decision that cannot be made while still walking.
    std::vector<int> frontier_cells;
    while (!queue.empty()) {
        const int current = queue.front();
        queue.pop_front();
        const int cx = current % cells;
        const int cy = current / cells;
        if (isFrontier(snapshot.grid, cx, cy)) frontier_cells.push_back(current);

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
                steps[static_cast<size_t>(next)] = steps[static_cast<size_t>(current)] + 1;
                queue.push_back(next);
            }
        }
    }

    const auto reached = [&](int node) { return parent[static_cast<size_t>(node)] >= 0; };
    const auto travel = [&](int node) {
        return static_cast<float>(steps[static_cast<size_t>(node)]) * OccupancyGrid::kResolution;
    };

    int goal = -1;

    // Keep the previous goal if it is still worth going to. Re-deciding every
    // cycle is what makes a robot turn one way, then the other, and never arrive.
    if (have_goal_) {
        int gx = 0;
        int gy = 0;
        const bool timed_out =
            std::chrono::steady_clock::now() - goal_chosen_at_ >
            std::chrono::milliseconds(options_.goal_timeout_ms);
        if (!timed_out && OccupancyGrid::toCell(goal_.x, goal_.y, gx, gy)) {
            const int node = gy * cells + gx;
            if (reached(node) && isFrontier(snapshot.grid, gx, gy) &&
                travel(node) > options_.goal_tolerance_m)
                goal = node;
        }
    }

    if (goal < 0) {
        // Group the frontier cells, so a single stray unknown cell cannot become
        // a destination. Flood fill over the frontier set itself.
        std::vector<uint8_t> is_frontier(static_cast<size_t>(cells) * cells, 0);
        for (int node : frontier_cells) is_frontier[static_cast<size_t>(node)] = 1;

        std::vector<uint8_t> seen(static_cast<size_t>(cells) * cells, 0);
        std::vector<uint8_t> counted_unknown(static_cast<size_t>(cells) * cells, 0);
        int best = -1;
        float best_travel = 0.0f;
        int fallback = -1;
        float fallback_travel = 0.0f;

        for (int seed : frontier_cells) {
            if (seen[static_cast<size_t>(seed)]) continue;
            std::deque<int> group_queue{seed};
            seen[static_cast<size_t>(seed)] = 1;
            // What matters is how much unseen space this frontier opens up, not
            // how many cells its border is made of. A single unknown cell in the
            // middle of the floor is ringed by eight border cells, which sails
            // past any threshold counted the naive way -- and that artefact is
            // exactly what the robot kept driving at.
            std::deque<int> unknown_queue;
            int revealed = 0;
            int nearest = -1;
            float nearest_travel = 0.0f;

            // The frontier cell itself is usually NOT somewhere the robot can
            // stand: it sits against the unknown, and inflation has cleared a
            // robot radius around every wall nearby. So the goal is the nearest
            // reachable cell within arm's reach of the frontier. Without this the
            // planner rejects every real doorway and concludes the room is fully
            // explored while staring straight at the way out.
            const auto consider = [&](int candidate) {
                if (!reached(candidate)) return;
                const float distance = travel(candidate);
                if (nearest < 0 || distance < nearest_travel) {
                    nearest = candidate;
                    nearest_travel = distance;
                }
            };

            while (!group_queue.empty()) {
                const int node = group_queue.front();
                group_queue.pop_front();
                {
                    const int fx = node % cells;
                    const int fy = node / cells;
                    for (int oy = -kGoalReach; oy <= kGoalReach; ++oy) {
                        for (int ox = -kGoalReach; ox <= kGoalReach; ++ox) {
                            const int nx = fx + ox;
                            const int ny = fy + oy;
                            if (!OccupancyGrid::inside(nx, ny)) continue;
                            consider(ny * cells + nx);
                        }
                    }
                }
                const int cx = node % cells;
                const int cy = node / cells;
                for (int oy = -1; oy <= 1; ++oy) {
                    for (int ox = -1; ox <= 1; ++ox) {
                        const int nx = cx + ox;
                        const int ny = cy + oy;
                        if (!OccupancyGrid::inside(nx, ny)) continue;
                        const int next = ny * cells + nx;
                        if (snapshot.grid.cell(nx, ny) == Cell::Unknown) {
                            if (!counted_unknown[static_cast<size_t>(next)]) {
                                counted_unknown[static_cast<size_t>(next)] = 1;
                                unknown_queue.push_back(next);
                                ++revealed;
                            }
                            continue;
                        }
                        if (seen[static_cast<size_t>(next)] || !is_frontier[static_cast<size_t>(next)])
                            continue;
                        seen[static_cast<size_t>(next)] = 1;
                        group_queue.push_back(next);
                    }
                }
            }

            // How much is actually back there? Flood the unknown region behind
            // the frontier, stopping once it is obviously worth going to. A
            // doorway opens onto a whole room and hits the cap immediately; a
            // gap between two lidar beams is one or two cells and stops there.
            // Counting only the cells touching the frontier cannot tell those
            // apart -- a 0.6 m doorway touches eleven, which is the same order
            // as a handful of artefacts side by side.
            while (!unknown_queue.empty() && revealed < kRevealedCap) {
                const int node = unknown_queue.front();
                unknown_queue.pop_front();
                const int cx = node % cells;
                const int cy = node / cells;
                for (int oy = -1; oy <= 1; ++oy) {
                    for (int ox = -1; ox <= 1; ++ox) {
                        const int nx = cx + ox;
                        const int ny = cy + oy;
                        if (!OccupancyGrid::inside(nx, ny)) continue;
                        if (snapshot.grid.cell(nx, ny) != Cell::Unknown) continue;
                        const int next = ny * cells + nx;
                        if (counted_unknown[static_cast<size_t>(next)]) continue;
                        counted_unknown[static_cast<size_t>(next)] = 1;
                        unknown_queue.push_back(next);
                        ++revealed;
                    }
                }
            }

            if (revealed < kMinRevealedCells || nearest < 0) continue;
            if (nearest_travel < options_.goal_tolerance_m) continue;
            if (nearest_travel >= options_.min_goal_distance_m) {
                if (best < 0 || nearest_travel < best_travel) {
                    best = nearest;
                    best_travel = nearest_travel;
                }
            } else if (fallback < 0 || nearest_travel > fallback_travel) {
                // Too close to be a good goal, but better than declaring the job
                // done: kept only if nothing further away is reachable.
                fallback = nearest;
                fallback_travel = nearest_travel;
            }
        }
        goal = best >= 0 ? best : fallback;
        if (goal >= 0) {
            OccupancyGrid::toWorld(goal % cells, goal / cells, goal_.x, goal_.y);
            have_goal_ = true;
            goal_chosen_at_ = std::chrono::steady_clock::now();
        }
    }

    path_.clear();
    next_point_ = 0;
    if (goal < 0) {
        have_goal_ = false;
        return false;
    }

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
        // Arrived. Drop the goal too, so the next cycle picks a fresh one rather
        // than re-planning to a place already reached.
        path_.clear();
        next_point_ = 0;
        have_goal_ = false;
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
    if (snapshot.match_score < min_match_score_) {
        // SLAM does not trust its own pose, so neither do we. Stop and let it
        // re-acquire rather than plan against a position that may be wrong.
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
            // Nothing reachable left worth seeing.
            finished_ = true;
            return stop();
        }
        finished_ = false;
    }
    return followPath(snapshot.pose);
}

void runExplorer(const RobotOptions& options, RobotState& state) {
    Explorer explorer(options.explore, options.slam.min_score);
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

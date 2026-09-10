#pragma once
// Exploration: drive to the nearest place the robot has not seen yet.
//
// A frontier is a free cell that touches an unknown one -- the edge of what is
// known. Driving to one and looking around turns unknown into known, which
// creates new frontiers further out. Repeat until none are left and the room is
// mapped. That is the whole idea; everything below is bookkeeping around it.
//
// "Nearest" is measured along a drivable path, not in a straight line. The cell
// two metres away through a wall is not nearer than the one five metres away
// down the corridor, and a straight-line choice would have the robot grinding
// against that wall forever.
//
// Two rules keep it from spinning on the spot, which is what a naive
// nearest-frontier explorer does:
//
//  - A frontier is judged by how much unseen space lies behind it, and small
//    ones are thrown away. Isolated unknown cells are almost always artefacts --
//    a gap between two lidar beams, a cell the robot has not looked at twice yet
//    -- not places worth driving to. Chasing them is what makes a robot turn
//    towards its own shadow every second.
//  - Once a goal is chosen it is kept until it is reached, becomes unreachable,
//    or stops being a frontier. Re-deciding every cycle means the robot turns
//    towards a new target before it has made any progress towards the last one,
//    and it never actually goes anywhere.
//
// This produces a DriveCommand and nothing else. It never touches the drive
// itself: the command goes into RobotState and the control loop runs it through
// the same safety gate as an operator command. Autonomy cannot bypass the gate,
// because autonomy does not know the gate exists.

#include <chrono>
#include <cstdint>
#include <vector>

#include "control/auto/map_types.h"
#include "options.h"
#include "protocol.h"
#include "state.h"

struct WorldPoint {
    float x = 0.0f;
    float y = 0.0f;
};

class Explorer {
public:
    // `min_match_score` is the SLAM threshold: the explorer refuses to drive on
    // a pose SLAM itself would not accept. Passed in rather than hard-coded so
    // there is exactly one number, not two that can drift apart.
    Explorer(const ExploreOptions& options, float min_match_score)
        : options_(options), min_match_score_(min_match_score) {}

    // How much unseen space a frontier has to open up before it is worth driving
    // to, in SQUARE METRES. This is the unknown area flooded from behind the
    // frontier, not the frontier's own size: a stray unknown cell has an
    // eight-cell border, so counting the border would wave it straight through,
    // while a real doorway opens onto a whole room.
    //
    // In metres rather than cells so that changing the grid resolution does not
    // silently change what counts as worth exploring.
    static constexpr float kMinRevealedArea = 0.06f;
    // Stop measuring once this much has been found: past here it is plainly a
    // real place to go, and flooding an entire unexplored floor to learn that
    // would be wasted work.
    static constexpr float kRevealedCapArea = 0.75f;
    // How far from a frontier cell to look for somewhere the robot can actually
    // stand. Has to cover the inflation radius, or every doorway looks
    // unreachable.
    static constexpr float kGoalReachM = 0.35f;

    // One cycle against the latest map. Returns what the robot should do now:
    // STOP when there is nowhere left to go, when the map is not trustworthy, or
    // when no route to any frontier exists.
    proto::DriveCommand step(const MapSnapshot& snapshot);

    // True once a full search found no reachable frontier worth going to: the
    // reachable area is mapped. Not the same as "no unknown cells exist" -- a
    // room behind a closed door stays unknown forever, and that is correct.
    bool finished() const noexcept { return finished_; }

    const std::vector<WorldPoint>& path() const noexcept { return path_; }
    bool haveGoal() const noexcept { return have_goal_; }
    WorldPoint goal() const noexcept { return goal_; }

    void reset();

private:
    // Breadth-first from the robot over drivable cells. Records the tree of
    // parents, finds every frontier cluster, and picks one. Fills path_.
    bool planPath(const MapSnapshot& snapshot);

    // Cells within the robot radius of a wall are not drivable, so a path is
    // never planned through a gap the hull does not fit through.
    void buildDrivable(const OccupancyGrid& grid);

    // Steering toward a point a fixed distance ahead on the path.
    proto::DriveCommand followPath(const Pose2D& pose);

    const ExploreOptions options_;
    const float min_match_score_;

    std::vector<uint8_t> drivable_;
    std::vector<WorldPoint> path_;
    size_t next_point_ = 0;

    // The goal survives between re-plans; see the header comment.
    WorldPoint goal_;
    bool have_goal_ = false;
    // How far the goal was when it was chosen. The goal is not reconsidered
    // until half of that has been covered -- the standard cure for a frontier
    // explorer that swaps targets before it has made progress on either.
    float goal_distance_at_choice_ = 0.0f;
    std::chrono::steady_clock::time_point goal_chosen_at_{};

    std::chrono::steady_clock::time_point planned_at_{};
    bool finished_ = false;
};

// Explorer thread: reads the map, publishes drive commands into RobotState.
void runExplorer(const RobotOptions& options, RobotState& state);

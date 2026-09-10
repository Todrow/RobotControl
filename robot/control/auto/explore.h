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
// This produces a DriveCommand and nothing else. It never touches the drive
// itself: the command goes into RobotState and the control loop runs it through
// the same safety gate as an operator command. Autonomy cannot bypass the gate,
// because autonomy does not know the gate exists.

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
    explicit Explorer(const ExploreOptions& options) : options_(options) {}

    // One cycle against the latest map. Returns what the robot should do now:
    // STOP when there is nowhere left to go, when the map is not trustworthy, or
    // when no route to any frontier exists.
    proto::DriveCommand step(const MapSnapshot& snapshot);

    // True once a full search found no reachable frontier: the reachable area is
    // mapped. Not the same as "no frontiers exist" -- an unreachable room behind
    // a closed door stays unknown forever, and that is the correct outcome.
    bool finished() const noexcept { return finished_; }

    const std::vector<WorldPoint>& path() const noexcept { return path_; }

    void reset();

private:
    // Breadth-first from the robot over drivable cells; stops at the first
    // frontier found, which is by construction the nearest one by path. Fills
    // path_ and returns false when nothing is reachable.
    bool planPath(const MapSnapshot& snapshot);

    // Cells within the robot radius of a wall are not drivable, so a path is
    // never planned through a gap the hull does not fit through.
    void buildDrivable(const OccupancyGrid& grid);

    // Steering toward a point a fixed distance ahead on the path.
    proto::DriveCommand followPath(const Pose2D& pose);

    const ExploreOptions options_;
    std::vector<uint8_t> drivable_;
    std::vector<WorldPoint> path_;
    size_t next_point_ = 0;
    std::chrono::steady_clock::time_point planned_at_{};
    bool finished_ = false;
};

// Explorer thread: reads the map, publishes drive commands into RobotState.
void runExplorer(const RobotOptions& options, RobotState& state);

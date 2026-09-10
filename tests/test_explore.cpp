// Standalone explorer test: hand-built maps, no robot and no SLAM.
//   c++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Icommon -Irobot
//       tests/test_explore.cpp robot/control/auto/explore.cpp -o /tmp/t -pthread && /tmp/t
//
// Checks the two things that made the real robot spin on the spot: that a single
// stray unknown cell is not treated as somewhere worth driving to, and that a
// goal once chosen is kept instead of being re-picked every cycle.
#include "control/auto/explore.h"

#include <cmath>
#include <cstdio>
#include <string>

namespace {

int failures = 0;

void check(bool condition, const std::string& what) {
    if (condition) return;
    std::fprintf(stderr, "FAIL: %s\n", what.c_str());
    ++failures;
}

// Paint a rectangle of world metres into the grid, hard enough that the cells
// pass the evidence threshold immediately.
void fill(OccupancyGrid& grid, float x0, float y0, float x1, float y1, bool occupied) {
    for (float y = y0; y <= y1; y += OccupancyGrid::kResolution * 0.5f) {
        for (float x = x0; x <= x1; x += OccupancyGrid::kResolution * 0.5f) {
            int cx = 0;
            int cy = 0;
            if (!OccupancyGrid::toCell(x, y, cx, cy)) continue;
            grid.observe(cx, cy, occupied ? 127 : -127);
        }
    }
}

// A room the robot has fully mapped, with one wide opening in the +x wall that
// leads to unknown space, and one single unknown cell left over inside.
MapSnapshot roomWithDoorAndSpeck() {
    MapSnapshot snapshot;
    snapshot.match_score = 1.0f;
    snapshot.pose = Pose2D{};
    fill(snapshot.grid, -3.0f, -2.0f, 3.0f, 2.0f, false);  // known floor
    fill(snapshot.grid, -3.1f, -2.2f, -3.0f, 2.2f, true);  // walls
    fill(snapshot.grid, -3.1f, 2.0f, 3.1f, 2.2f, true);
    fill(snapshot.grid, -3.1f, -2.2f, 3.1f, -2.0f, true);
    // +x wall with a 0.6 m doorway around y = 0, beyond which nothing is known.
    fill(snapshot.grid, 3.0f, -2.2f, 3.1f, -0.3f, true);
    fill(snapshot.grid, 3.0f, 0.3f, 3.1f, 2.2f, true);

    // One lone unknown cell right next to the robot: the artefact that used to
    // capture the planner.
    int cx = 0;
    int cy = 0;
    OccupancyGrid::toCell(0.4f, 0.0f, cx, cy);
    snapshot.grid.observe(cx, cy, -127);  // force it back to zero below
    while (snapshot.grid.cell(cx, cy) != Cell::Unknown) snapshot.grid.observe(cx, cy, 1);
    return snapshot;
}

float distance(const WorldPoint& a, float x, float y) {
    return std::sqrt((a.x - x) * (a.x - x) + (a.y - y) * (a.y - y));
}

}  // namespace

int main() {
    ExploreOptions options;
    Explorer explorer(options, 0.22f);

    MapSnapshot snapshot = roomWithDoorAndSpeck();

    // Sanity: the speck really is an isolated unknown cell next to the robot.
    int cx = 0;
    int cy = 0;
    OccupancyGrid::toCell(0.4f, 0.0f, cx, cy);
    check(snapshot.grid.cell(cx, cy) == Cell::Unknown, "the test speck must be unknown");

    const proto::DriveCommand first = explorer.step(snapshot);
    check(!explorer.finished(), "a doorway to unknown space is somewhere to go");
    check(explorer.haveGoal(), "a goal should have been chosen");
    check(first.direction != proto::Direction::STOP, "the robot should set off");

    // The goal must be the doorway at x = +3, not the speck 40 cm away.
    const WorldPoint goal = explorer.goal();
    std::printf("goal: (%+.2f, %+.2f)\n", static_cast<double>(goal.x),
                static_cast<double>(goal.y));
    check(distance(goal, 0.4f, 0.0f) > 1.0f, "a lone unknown cell must not become a goal");
    check(goal.x > 2.0f, "the goal should be the doorway");

    // And it must stay the goal across cycles, even as the robot moves.
    const WorldPoint chosen = goal;
    for (int step = 0; step < 5; ++step) {
        snapshot.pose.x += 0.25f;
        explorer.step(snapshot);
        check(distance(explorer.goal(), chosen.x, chosen.y) < 0.01f,
              "the goal must not be re-picked while it is still valid");
    }

    // Two equally good ways out, one ahead and one behind. Nearest-frontier has
    // no reason to prefer either and flips between them as the map updates;
    // cost-utility pays for the pivot, so it commits to the one in front.
    {
        MapSnapshot both;
        both.match_score = 1.0f;
        both.pose = Pose2D{};  // facing +x
        fill(both.grid, -3.0f, -2.0f, 3.0f, 2.0f, false);
        fill(both.grid, -3.1f, 2.0f, 3.1f, 2.2f, true);
        fill(both.grid, -3.1f, -2.2f, 3.1f, -2.0f, true);
        // Doorway at +x.
        fill(both.grid, 3.0f, -2.2f, 3.1f, -0.3f, true);
        fill(both.grid, 3.0f, 0.3f, 3.1f, 2.2f, true);
        // Mirror-image doorway at -x, the same size and the same distance.
        fill(both.grid, -3.1f, -2.2f, -3.0f, -0.3f, true);
        fill(both.grid, -3.1f, 0.3f, -3.0f, 2.2f, true);

        Explorer chooser(options, 0.22f);
        chooser.step(both);
        check(chooser.haveGoal(), "one of the two doorways must be chosen");
        std::printf("with two equal exits, goal x = %+.2f\n",
                    static_cast<double>(chooser.goal().x));
        check(chooser.goal().x > 0.0f, "the exit ahead must beat the one behind");
    }

    // A map with nowhere left to go: the same room, sealed.
    MapSnapshot sealed;
    sealed.match_score = 1.0f;
    fill(sealed.grid, -3.0f, -2.0f, 3.0f, 2.0f, false);
    fill(sealed.grid, -3.1f, -2.2f, -3.0f, 2.2f, true);
    fill(sealed.grid, 3.0f, -2.2f, 3.1f, 2.2f, true);
    fill(sealed.grid, -3.1f, 2.0f, 3.1f, 2.2f, true);
    fill(sealed.grid, -3.1f, -2.2f, 3.1f, -2.0f, true);
    Explorer closed(options, 0.22f);
    const proto::DriveCommand nowhere = closed.step(sealed);
    check(closed.finished(), "a sealed room is fully explored");
    check(nowhere.direction == proto::Direction::STOP, "and the robot must stop");

    // A pose SLAM does not trust must not be driven on.
    MapSnapshot lost = roomWithDoorAndSpeck();
    lost.match_score = 0.10f;
    Explorer unsure(options, 0.22f);
    check(unsure.step(lost).direction == proto::Direction::STOP,
          "a distrusted pose must not be driven on");

    if (failures != 0) {
        std::fprintf(stderr, "Explorer test failed: %d check(s)\n", failures);
        return 1;
    }
    std::printf("Explorer tests passed\n");
    return 0;
}

// Standalone SLAM test: a virtual room, a known trajectory, synthetic scans.
//   c++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -Icommon -Irobot
//       tests/test_slam.cpp robot/control/auto/slam.cpp -o /tmp/t && /tmp/t
//
// No robot, no lidar, no threads. The point is to check the thing that cannot be
// checked by reading the code: whether scan matching actually tracks the robot,
// or quietly drifts away. The trajectory is known exactly, so every estimate has
// a right answer to be compared against.
#include "control/auto/slam.h"

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

namespace {

constexpr float kPi = 3.14159265358979f;

struct Segment {
    float ax, ay, bx, by;
};

// A 6x4 m room with a box in it. The box matters: a bare rectangle looks the
// same from symmetric positions, and the matcher would have no way to tell two
// of them apart. Real rooms have furniture; a test room needs some too.
std::vector<Segment> room() {
    return {
        {-3.0f, -2.0f, 3.0f, -2.0f}, {3.0f, -2.0f, 3.0f, 2.0f},
        {3.0f, 2.0f, -3.0f, 2.0f},   {-3.0f, 2.0f, -3.0f, -2.0f},
        // Box near the right-hand wall.
        {1.2f, 0.3f, 1.6f, 0.3f},    {1.6f, 0.3f, 1.6f, 0.7f},
        {1.6f, 0.7f, 1.2f, 0.7f},    {1.2f, 0.7f, 1.2f, 0.3f},
    };
}

// Distance from `origin` along `angle` to the nearest wall, or -1 if the ray
// escapes (it cannot in a closed room, but the caller should not assume).
float castRay(const std::vector<Segment>& walls, float ox, float oy, float angle) {
    const float dx = std::cos(angle);
    const float dy = std::sin(angle);
    float nearest = -1.0f;
    for (const Segment& wall : walls) {
        const float ex = wall.bx - wall.ax;
        const float ey = wall.by - wall.ay;
        const float denominator = dx * ey - dy * ex;
        if (std::fabs(denominator) < 1e-9f) continue;  // parallel
        const float t = ((wall.ax - ox) * ey - (wall.ay - oy) * ex) / denominator;
        const float u = ((wall.ax - ox) * dy - (wall.ay - oy) * dx) / denominator;
        if (t <= 0.0f || u < 0.0f || u > 1.0f) continue;
        if (nearest < 0.0f || t < nearest) nearest = t;
    }
    return nearest;
}

// One revolution as the lidar would report it: 360 beams, robot frame,
// x forward and y left.
LaserScan scanAt(const std::vector<Segment>& walls, const Pose2D& pose) {
    LaserScan scan;
    scan.points.reserve(360);
    for (int degree = 0; degree < 360; ++degree) {
        const float bearing = static_cast<float>(degree) * kPi / 180.0f;
        const float range = castRay(walls, pose.x, pose.y, pose.theta + bearing);
        if (range <= 0.0f || range > 12.0f) continue;
        scan.points.push_back({range * std::cos(bearing), range * std::sin(bearing)});
    }
    scan.stamp = std::chrono::steady_clock::now();
    return scan;
}

int failures = 0;

void check(bool condition, const std::string& what) {
    if (condition) return;
    std::fprintf(stderr, "FAIL: %s\n", what.c_str());
    ++failures;
}

float wrap(float radians) {
    while (radians > kPi) radians -= 2.0f * kPi;
    while (radians < -kPi) radians += 2.0f * kPi;
    return radians;
}

}  // namespace

int main() {
    const std::vector<Segment> walls = room();
    SlamOptions options;
    Slam slam(options);

    // Straight run with a steady turn: 5 cm and 3 degrees per revolution is
    // about what this robot covers in one lidar period at exploration speed.
    Pose2D truth;
    std::vector<Pose2D> path;
    for (int step = 0; step < 40; ++step) {
        path.push_back(truth);
        truth.x += 0.05f * std::cos(truth.theta);
        truth.y += 0.05f * std::sin(truth.theta);
        truth.theta = wrap(truth.theta + 3.0f * kPi / 180.0f);
    }

    float worst_position = 0.0f;
    float worst_heading = 0.0f;
    int accepted = 0;
    auto spent = std::chrono::steady_clock::duration::zero();
    for (const Pose2D& pose : path) {
        const LaserScan scan = scanAt(walls, pose);
        const auto started = std::chrono::steady_clock::now();
        if (slam.update(scan)) ++accepted;
        spent += std::chrono::steady_clock::now() - started;
        const float dx = slam.pose().x - pose.x;
        const float dy = slam.pose().y - pose.y;
        worst_position = std::max(worst_position, std::sqrt(dx * dx + dy * dy));
        worst_heading =
            std::max(worst_heading, std::fabs(wrap(slam.pose().theta - pose.theta)));
    }

    const double per_scan_ms = std::chrono::duration<double, std::milli>(spent).count() /
                               static_cast<double>(path.size());
    std::printf("accepted %d/%zu scans, worst error: %.3f m, %.2f deg, %.1f ms/scan\n",
                accepted, path.size(), static_cast<double>(worst_position),
                static_cast<double>(worst_heading * 180.0f / kPi), per_scan_ms);
    // The lidar publishes every 100 ms. Matching has to fit well inside that
    // on a Raspberry Pi 4, which is several times slower than this desktop.
    check(per_scan_ms < 40.0, "one scan must match well inside the lidar period");

    check(accepted >= static_cast<int>(path.size()) - 2, "almost every scan should match");
    // One grid cell is 5 cm, so anything under ~2 cells is as good as this
    // representation can be.
    check(worst_position < 0.12f, "position must track the true trajectory");
    check(worst_heading < 5.0f * kPi / 180.0f, "heading must track the true trajectory");

    // The map has to be usable, not merely present: walls solid, middle open,
    // and the far side of a wall never invented as free space.
    const OccupancyGrid& grid = slam.grid();
    int wall_cx = 0, wall_cy = 0, open_cx = 0, open_cy = 0, outside_cx = 0, outside_cy = 0;
    OccupancyGrid::toCell(3.0f, 0.0f, wall_cx, wall_cy);
    OccupancyGrid::toCell(0.0f, 0.0f, open_cx, open_cy);
    OccupancyGrid::toCell(4.0f, 0.0f, outside_cx, outside_cy);
    check(grid.cell(wall_cx, wall_cy) == Cell::Occupied, "the right-hand wall must be occupied");
    check(grid.cell(open_cx, open_cy) == Cell::Free, "the middle of the room must be free");
    check(grid.cell(outside_cx, outside_cy) == Cell::Unknown,
          "space behind a wall must stay unknown");

    // A scan of nothing must not move the robot or touch the map.
    const Pose2D before = slam.pose();
    check(!slam.update(LaserScan{}), "an empty scan must be rejected");
    check(slam.pose().x == before.x && slam.pose().y == before.y, "a rejected scan must not move");

    if (failures != 0) {
        std::fprintf(stderr, "SLAM test failed: %d check(s)\n", failures);
        return 1;
    }
    std::printf("SLAM tests passed\n");
    return 0;
}

#pragma once
// One full revolution of the lidar, in the robot's own frame.
//
// This is the second, separate product of the lidar module. The six sector
// verdicts (lidar_distances.h -> obstacle_check.h) stay exactly as they were:
// they are the safety path, they are clamped to half a metre, and nothing here
// may change how they behave. A scan is the mapping path -- full range, every
// point, no thresholds applied -- and only SLAM reads it.
//
// Keeping them apart matters: if SLAM dies or falls behind, the obstacle gate
// keeps working off its own untouched pipeline.
//
// Frame: x forward, y to the LEFT, metres, origin at the lidar. The device
// reports angles clockwise from the nose, so y is negated on the way in --
// see scanFromPolar(). Angles in the rest of the autonomy code are radians,
// counter-clockwise positive, which is the usual robotics convention and what
// every trig function here expects.

#include <chrono>
#include <vector>

struct ScanPoint {
    float x = 0.0f;
    float y = 0.0f;
};

struct LaserScan {
    std::vector<ScanPoint> points;
    std::chrono::steady_clock::time_point stamp{};

    bool empty() const noexcept { return points.empty(); }
    size_t size() const noexcept { return points.size(); }
};

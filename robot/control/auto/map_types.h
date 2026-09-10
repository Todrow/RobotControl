#pragma once
// The two things SLAM produces: where the robot is, and what it has seen.
//
// Data only -- no algorithm. SLAM fills these in (slam.cpp), the explorer reads
// them (explore.cpp), the map channel serialises them (systems/rpi/link), and
// state.h stores them. Keeping the types here means state.h does not have to
// depend on the SLAM implementation to hold its output.

#include <cstdint>
#include <vector>

// Robot pose in the map frame: x forward-ish, y left, theta counter-clockwise
// from the +x axis, radians. The map frame is fixed at wherever the robot was
// when SLAM started, so at t=0 the pose is exactly zero.
struct Pose2D {
    float x = 0.0f;
    float y = 0.0f;
    float theta = 0.0f;
};

// What one cell is known to be. Unknown is not Free: it is the whole point of
// exploring, and treating it as free would send the robot through walls it has
// not seen yet.
enum class Cell : uint8_t {
    Unknown = 0,
    Free = 1,
    Occupied = 2,
};

// Occupancy grid in log-odds, fixed size, robot starts at the centre.
//
// int8_t per cell rather than float: 160 KB instead of 640 KB, it survives being
// copied around, and the saturating add below is all the update rule a grid this
// simple needs. 0 means "no evidence either way" -- the initial state of every
// cell, and exactly what Unknown means.
class OccupancyGrid {
public:
    // 400 x 400 at 5 cm is 20 x 20 m, and a doorway is ~8 cells wide -- enough
    // resolution for the planner to see the gap.
    static constexpr int kCells = 400;
    static constexpr float kResolution = 0.05f;

    // Beyond these the cell counts as decided. Deliberately asymmetric: a cell
    // must be seen free a few times before the planner will drive through it,
    // but one solid hit is enough to call it a wall.
    static constexpr int8_t kOccupiedAt = 20;
    static constexpr int8_t kFreeAt = -20;

    // Per-observation evidence. Hits count for more than misses, so a thin wall
    // seen edge-on is not erased by the beams passing beside it.
    static constexpr int kHitDelta = 12;
    static constexpr int kMissDelta = -4;

    OccupancyGrid() : logodds_(static_cast<size_t>(kCells) * kCells, 0) {}

    static constexpr int centre() noexcept { return kCells / 2; }

    // World metres -> cell index. Returns false when the point falls outside the
    // grid; callers must check rather than clamp, or distant readings would pile
    // up as phantom walls along the border.
    static bool toCell(float x, float y, int& cx, int& cy) noexcept {
        const int ix = static_cast<int>(x / kResolution + (x >= 0 ? 0.5f : -0.5f)) + centre();
        const int iy = static_cast<int>(y / kResolution + (y >= 0 ? 0.5f : -0.5f)) + centre();
        cx = ix;
        cy = iy;
        return inside(ix, iy);
    }

    // Cell index -> the world coordinates of its centre.
    static void toWorld(int cx, int cy, float& x, float& y) noexcept {
        x = static_cast<float>(cx - centre()) * kResolution;
        y = static_cast<float>(cy - centre()) * kResolution;
    }

    static bool inside(int cx, int cy) noexcept {
        return cx >= 0 && cx < kCells && cy >= 0 && cy < kCells;
    }

    int8_t at(int cx, int cy) const noexcept {
        if (!inside(cx, cy)) return 0;
        return logodds_[index(cx, cy)];
    }

    Cell cell(int cx, int cy) const noexcept {
        const int8_t value = at(cx, cy);
        if (value >= kOccupiedAt) return Cell::Occupied;
        if (value <= kFreeAt) return Cell::Free;
        return Cell::Unknown;
    }

    // Saturating, so a cell stared at for a minute does not become impossible to
    // ever change its mind about -- a door that opens has to be able to go free.
    void observe(int cx, int cy, int delta) noexcept {
        if (!inside(cx, cy)) return;
        int value = logodds_[index(cx, cy)] + delta;
        if (value > 127) value = 127;
        if (value < -127) value = -127;
        logodds_[index(cx, cy)] = static_cast<int8_t>(value);
    }

    const std::vector<int8_t>& raw() const noexcept { return logodds_; }

private:
    static size_t index(int cx, int cy) noexcept {
        return static_cast<size_t>(cy) * kCells + static_cast<size_t>(cx);
    }
    std::vector<int8_t> logodds_;
};

// What SLAM publishes each cycle. Pose and map travel together on purpose: the
// planner that reads a pose from one moment and a map from another would build
// a path from a place the robot is not.
struct MapSnapshot {
    Pose2D pose;
    OccupancyGrid grid;
    // How well the last scan matched the map, 0..1. Low means SLAM is lost and
    // the pose should not be trusted; the explorer refuses to drive on it.
    float match_score = 0.0f;
};

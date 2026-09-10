#pragma once
// The autonomy channel: map and pose out, mode requests in. TCP, port 5004.
//
// Separate from protocol.h on purpose. That one carries fixed-size structs with
// no framing at all -- a reader takes exactly sizeof(T) bytes -- which works
// only while every message has one known size. A map does not: it is compressed,
// so its length changes every frame. Rather than retrofit framing onto the
// working teleoperation path and risk breaking it, this channel carries its own
// length-prefixed framing and leaves DesiredState and Telemetry untouched.
//
// Byte order is native little-endian, same assumption protocol.h already makes.
// Both ends are rebuilt together; there is no negotiation.

#include <cstdint>
#include <cstring>
#include <vector>

namespace mapproto {

constexpr uint16_t MAP_PORT = 5004;

// Robot -> control. One complete map, sent a couple of times a second.
constexpr uint16_t MSG_MAP = 1;
// Control -> robot. One byte: 1 starts exploring, 0 stops.
constexpr uint16_t MSG_MODE = 2;

// Every frame starts with this. `length` counts the payload only.
#pragma pack(push, 1)
struct FrameHeader {
    uint16_t type;
    uint32_t length;
};

// Fixed part of MSG_MAP; the run-length encoded cells follow it.
struct MapHeader {
    float pose_x;
    float pose_y;
    float pose_theta;   // radians, counter-clockwise
    float match_score;  // 0..1; below ~0.3 the pose is not trustworthy
    float resolution;   // metres per cell
    uint16_t cells;     // grid is cells x cells, origin at its centre
    uint32_t runs;      // number of (value, count) pairs that follow
    // What the robot is ACTUALLY doing, not what was last asked of it. The
    // operator can cancel autonomy just by touching the stick, and the button on
    // screen has to follow the robot rather than the other way round.
    uint8_t exploring;
};

// One run of identical cells. 0 = unknown, 1 = free, 2 = occupied.
struct CellRun {
    uint8_t value;
    uint16_t count;
};
#pragma pack(pop)

static_assert(sizeof(FrameHeader) == 6);
static_assert(sizeof(MapHeader) == 27);
static_assert(sizeof(CellRun) == 3);

constexpr uint8_t CELL_UNKNOWN = 0;
constexpr uint8_t CELL_FREE = 1;
constexpr uint8_t CELL_OCCUPIED = 2;

// A partly explored 400x400 map is mostly one long run of unknown, so this
// turns 160 KB into a few kilobytes. Worst case -- a checkerboard -- it would
// expand to 3 bytes per cell, which is why the reader below refuses a run count
// that could not fit the grid.
inline std::vector<CellRun> encodeRuns(const std::vector<uint8_t>& cells) {
    std::vector<CellRun> runs;
    if (cells.empty()) return runs;
    uint8_t value = cells[0];
    uint32_t count = 1;
    for (size_t i = 1; i < cells.size(); ++i) {
        // 65535 is the widest run the count field holds; break and start another.
        if (cells[i] == value && count < 65535) {
            ++count;
            continue;
        }
        runs.push_back({value, static_cast<uint16_t>(count)});
        value = cells[i];
        count = 1;
    }
    runs.push_back({value, static_cast<uint16_t>(count)});
    return runs;
}

// Expands into exactly `expected` cells, or returns false. A frame that decodes
// to the wrong size is corrupt, and painting it would put walls in places the
// robot never saw.
inline bool decodeRuns(const std::vector<CellRun>& runs, size_t expected,
                       std::vector<uint8_t>& cells) {
    cells.clear();
    cells.reserve(expected);
    for (const CellRun& run : runs) {
        if (cells.size() + run.count > expected) return false;
        cells.insert(cells.end(), run.count, run.value);
    }
    return cells.size() == expected;
}

}  // namespace mapproto

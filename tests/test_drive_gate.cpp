// Drive-gate policy tests: a per-sector verdict and a Direction in, the command
// the drive should actually run out. Pure functions, so no lidar, no sockets and
// no Raspberry Pi are involved.
//   c++ -std=c++17 -Wall -Wextra -Wpedantic -Icommon -Imock_robot tests/test_drive_gate.cpp -o /tmp/t && /tmp/t
#include <cstdio>
#include <string>

#include "protocol.h"
#include "utils/drive_gate.h"

namespace {

int failures = 0;

void check(bool condition, const std::string& what) {
    if (condition) return;
    std::fprintf(stderr, "FAIL: %s\n", what.c_str());
    ++failures;
}

SectorStatuses allGreen() {
    SectorStatuses statuses;
    statuses.fill(proto::SectorStatus::Green);
    return statuses;
}

SectorStatuses withRed(int sector) {
    SectorStatuses statuses = allGreen();
    statuses[static_cast<size_t>(sector)] = proto::SectorStatus::Red;
    return statuses;
}

bool passesThrough(proto::Direction direction, const SectorStatuses& sectors) {
    const proto::DriveCommand in{direction, 0.7f};
    const proto::DriveCommand out = gateDriveCommand(in, sectors);
    return out.direction == direction && out.speed == in.speed;
}

bool stopped(proto::Direction direction, const SectorStatuses& sectors) {
    const proto::DriveCommand out = gateDriveCommand({direction, 0.7f}, sectors);
    return out.direction == proto::Direction::STOP && out.speed == 0.0f;
}

// A clear scan changes nothing, whatever the heading.
void testClearPathPassesEveryDirection() {
    const SectorStatuses clear = allGreen();
    for (int raw = 0; raw <= static_cast<int>(proto::DIRECTION_MAX); ++raw)
        check(passesThrough(static_cast<proto::Direction>(raw), clear),
              "clear path leaves the command untouched");
}

// Each straight heading is blocked only by the sector it drives into.
void testStraightHeadingsBlockedByTheirSector() {
    check(stopped(proto::Direction::FORWARD, withRed(proto::SECTOR_FORWARD)),
          "FORWARD into a Red forward sector becomes STOP");
    check(passesThrough(proto::Direction::FORWARD, withRed(proto::SECTOR_BACK)),
          "FORWARD is indifferent to a Red sector behind it");
    check(stopped(proto::Direction::BACKWARD, withRed(proto::SECTOR_BACK)),
          "BACKWARD into a Red back sector becomes STOP");
    check(passesThrough(proto::Direction::BACKWARD, withRed(proto::SECTOR_FORWARD)),
          "BACKWARD is indifferent to a Red sector ahead");
}

// A diagonal is blocked by its straight sector or by its corner.
void testDiagonalsBlockedByEitherSector() {
    check(stopped(proto::Direction::FORWARD_RIGHT, withRed(proto::SECTOR_FORWARD)),
          "FORWARD_RIGHT stops on a Red forward sector");
    check(stopped(proto::Direction::FORWARD_RIGHT, withRed(proto::SECTOR_FORWARD_RIGHT)),
          "FORWARD_RIGHT stops on a Red forward-right corner");
    check(passesThrough(proto::Direction::FORWARD_RIGHT, withRed(proto::SECTOR_FORWARD_LEFT)),
          "FORWARD_RIGHT ignores the opposite corner");
    check(stopped(proto::Direction::BACKWARD_LEFT, withRed(proto::SECTOR_BACK)),
          "BACKWARD_LEFT stops on a Red back sector");
    check(stopped(proto::Direction::BACKWARD_LEFT, withRed(proto::SECTOR_BACK_LEFT)),
          "BACKWARD_LEFT stops on a Red back-left corner");
    check(passesThrough(proto::Direction::BACKWARD_LEFT, withRed(proto::SECTOR_BACK_RIGHT)),
          "BACKWARD_LEFT ignores the opposite corner");
}

// Pivots and STOP move the hull into no sector, so a boxed-in robot can escape.
void testPivotsAndStopAreNeverBlocked() {
    SectorStatuses everywhere;
    everywhere.fill(proto::SectorStatus::Red);
    check(passesThrough(proto::Direction::LEFT, everywhere), "LEFT pivot is never blocked");
    check(passesThrough(proto::Direction::RIGHT, everywhere), "RIGHT pivot is never blocked");
    check(passesThrough(proto::Direction::STOP, everywhere), "STOP is never blocked");
}

// Only Red stops the robot; Yellow, Green and Unknown all keep moving.
void testOnlyRedBlocks() {
    for (const auto status : {proto::SectorStatus::Unknown, proto::SectorStatus::Green,
                              proto::SectorStatus::Yellow}) {
        SectorStatuses sectors = allGreen();
        sectors[proto::SECTOR_FORWARD] = status;
        check(passesThrough(proto::Direction::FORWARD, sectors),
              "a non-Red forward sector still allows FORWARD");
        check(sectorForbidsMotion(status) == false, "only Red forbids motion");
    }
    check(sectorForbidsMotion(proto::SectorStatus::Red), "Red forbids motion");
}

}  // namespace

int main() {
    testClearPathPassesEveryDirection();
    testStraightHeadingsBlockedByTheirSector();
    testDiagonalsBlockedByEitherSector();
    testPivotsAndStopAreNeverBlocked();
    testOnlyRedBlocks();
    if (failures != 0) {
        std::fprintf(stderr, "%d drive gate test(s) failed\n", failures);
        return 1;
    }
    std::printf("Drive gate tests passed\n");
    return 0;
}

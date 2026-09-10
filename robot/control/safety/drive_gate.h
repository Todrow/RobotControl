#pragma once
// Safety gate: the published per-sector verdict decides which drive commands the
// robot is allowed to run. A command that would push the hull into a sector the
// lidar has flagged Red is turned into a STOP before it reaches the UART.
//
// Pure functions only -- no state, no I/O, no threads -- so the Direction ->
// sector mapping is testable without a robot. The verdict itself and the
// thresholds behind it live in obstacle_check.h; this file never re-derives
// them, it only decides how a heading relates to the six sectors.
#include <array>

#include "protocol.h"
#include "control/safety/obstacle_check.h"

// A sector counts as "dangerous" only when its verdict is Red: an obstacle
// nearer than the red threshold. Yellow is a warning the operator still drives
// through. Unknown and Green both allow motion -- an unmeasured sector (lidar
// disabled, stale, or rejected) must never freeze the robot in place, and
// Unknown is exactly what every sector reads as when there is no lidar at all.
inline bool sectorForbidsMotion(proto::SectorStatus status) noexcept {
    return status == proto::SectorStatus::Red;
}

// True when `direction` drives the hull toward a Red sector. STOP and the two
// in-place pivots translate the hull into no sector, so they are always allowed:
// a robot boxed in on every side must still be able to turn away. A diagonal is
// blocked when either the straight sector ahead of it or its corner is Red.
inline bool motionForbidden(proto::Direction direction, const SectorStatuses& sectors) noexcept {
    const auto red = [&sectors](int sector) {
        return sectorForbidsMotion(sectors[static_cast<size_t>(sector)]);
    };
    switch (direction) {
        case proto::Direction::FORWARD:
            return red(proto::SECTOR_FORWARD);
        case proto::Direction::BACKWARD:
            return red(proto::SECTOR_BACK);
        case proto::Direction::FORWARD_RIGHT:
            return red(proto::SECTOR_FORWARD) || red(proto::SECTOR_FORWARD_RIGHT);
        case proto::Direction::FORWARD_LEFT:
            return red(proto::SECTOR_FORWARD) || red(proto::SECTOR_FORWARD_LEFT);
        case proto::Direction::BACKWARD_RIGHT:
            return red(proto::SECTOR_BACK) || red(proto::SECTOR_BACK_RIGHT);
        case proto::Direction::BACKWARD_LEFT:
            return red(proto::SECTOR_BACK) || red(proto::SECTOR_BACK_LEFT);
        case proto::Direction::STOP:
        case proto::Direction::LEFT:
        case proto::Direction::RIGHT:
            return false;
    }
    return false;
}

// The command the drive should actually execute: the caller's command unchanged,
// or a full STOP when that command would drive the hull into a Red sector. The
// operator's other options -- reverse, pivot in place -- are left untouched, so
// this stops the robot without trapping it.
inline proto::DriveCommand gateDriveCommand(const proto::DriveCommand& command,
                                            const SectorStatuses& sectors) noexcept {
    if (motionForbidden(command.direction, sectors)) return {proto::Direction::STOP, 0.0f};
    return command;
}

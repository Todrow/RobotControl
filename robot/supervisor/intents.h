#pragma once
// What the operator actually asked for.
//
// The link hands over bytes; this is where those bytes stop being a frame and
// become an intent. Nothing downstream of here re-checks the numbers, so the
// validation below is the only thing standing between a corrupt or hostile
// frame and the motors -- keep it total, and reject rather than clamp: a frame
// that fails any check is not a weaker command, it is a broken sender.

#include <cmath>

#include "protocol.h"

// One operator command, already known to be well-formed.
struct OperatorIntent {
    proto::DriveCommand drive;
    proto::CameraState camera;
};

inline bool validDesiredState(const proto::DesiredState& state) noexcept {
    return state.drive_cmd.direction <= proto::DIRECTION_MAX &&
           std::isfinite(state.drive_cmd.speed) && state.drive_cmd.speed >= 0.0f &&
           state.drive_cmd.speed <= 1.0f &&
           std::isfinite(state.camera.pitch) && std::fabs(state.camera.pitch) <= 1.0f &&
           std::isfinite(state.camera.yaw) && std::fabs(state.camera.yaw) <= 1.0f;
}

// Only call after validDesiredState() has accepted the frame.
inline OperatorIntent intentFrom(const proto::DesiredState& state) noexcept {
    return {state.drive_cmd, state.camera};
}

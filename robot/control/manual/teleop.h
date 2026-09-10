#pragma once
// Manual mode: the operator drives, the robot obeys.
//
// The mapping itself is deliberately empty -- a teleoperated robot must do
// exactly what the stick says, with no smoothing or second-guessing in between,
// or the operator ends up fighting the robot. Everything that may override the
// operator lives in control/safety, downstream of here and shared with every
// other mode, so it can never be bypassed by adding a mode.
//
// The state this does keep is about the session, not the vehicle: whether the
// operator has ever commanded anything, and whether the latest command differs
// from the last one. Both feed decisions the control loop has to make -- what to
// log, and whether a dropped link was a real driving session worth recovering
// from.

#include "protocol.h"
#include "supervisor/intents.h"

// Fixed-width labels, so consecutive log lines stay aligned.
const char* directionName(proto::Direction direction) noexcept;

class Teleop {
public:
    // The operator's command, unchanged. Safety runs after this, not here.
    proto::DriveCommand drive(const OperatorIntent& intent) const noexcept {
        return intent.drive;
    }
    proto::CameraState camera(const OperatorIntent& intent) const noexcept {
        return intent.camera;
    }

    // True when this intent differs from the last one accepted -- the first
    // intent of a session always counts. Records it either way.
    bool accept(const OperatorIntent& intent) noexcept;

    // Whether any valid command arrived during this session. A link that drops
    // before the first one was never a driving session.
    bool everCommanded() const noexcept { return have_previous_; }

    // Call when the session ends; the next one starts with no history.
    void reset() noexcept { have_previous_ = false; }

private:
    OperatorIntent previous_{};
    bool have_previous_ = false;
};

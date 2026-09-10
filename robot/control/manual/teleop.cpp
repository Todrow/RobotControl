#include "control/manual/teleop.h"

#include <cmath>

const char* directionName(proto::Direction direction) noexcept {
    switch (direction) {
        case proto::Direction::STOP: return "STOP    ";
        case proto::Direction::FORWARD: return "FORWARD ";
        case proto::Direction::BACKWARD: return "BACKWARD";
        case proto::Direction::LEFT: return "LEFT    ";
        case proto::Direction::RIGHT: return "RIGHT   ";
        case proto::Direction::FORWARD_RIGHT: return "FWD-RGHT";
        case proto::Direction::FORWARD_LEFT: return "FWD-LEFT";
        case proto::Direction::BACKWARD_RIGHT: return "BCK-RGHT";
        case proto::Direction::BACKWARD_LEFT: return "BCK-LEFT";
    }
    return "?       ";
}

bool Teleop::accept(const OperatorIntent& intent) noexcept {
    // The controller repeats the full state every COMMAND_PERIOD_MS, so most
    // intents are identical to the last one. Only real changes are worth a log
    // line; the tolerance is below what the operator's input can resolve.
    constexpr float kEpsilon = 0.001f;
    const bool changed =
        !have_previous_ || intent.drive.direction != previous_.drive.direction ||
        std::fabs(intent.drive.speed - previous_.drive.speed) > kEpsilon ||
        std::fabs(intent.camera.pitch - previous_.camera.pitch) > kEpsilon ||
        std::fabs(intent.camera.yaw - previous_.camera.yaw) > kEpsilon;
    // Only a change moves the reference forward. Comparing against the last
    // *logged* intent, not the last received one, keeps a slow drift below the
    // tolerance from creeping past it one imperceptible step at a time.
    if (changed) {
        previous_ = intent;
        have_previous_ = true;
    }
    return changed;
}

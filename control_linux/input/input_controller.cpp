#include "input_controller.h"

#include <algorithm>

#include "../state/desired_state_slot.h"

namespace {

constexpr float kMouseSensitivity = 0.004f;  // full -1..1 sweep in ~500 px

enum WasdBit : unsigned {
    kBitW = 1u << 0,
    kBitA = 1u << 1,
    kBitS = 1u << 2,
    kBitD = 1u << 3,
};

unsigned wasdBit(int vk) {
    switch (vk) {
        case vkey::W: return kBitW;
        case vkey::S: return kBitS;
        case vkey::A: return kBitA;
        case vkey::D: return kBitD;
        default: return 0;
    }
}

// Each axis is decided independently and opposite keys cancel it, so a held
// combination always maps to exactly one direction and no key order is kept.
proto::Direction wasdDirection(unsigned held) {
    const bool forward = (held & kBitW) && !(held & kBitS);
    const bool backward = (held & kBitS) && !(held & kBitW);
    const bool right = (held & kBitD) && !(held & kBitA);
    const bool left = (held & kBitA) && !(held & kBitD);

    if (forward) {
        if (right) return proto::Direction::FORWARD_RIGHT;
        if (left) return proto::Direction::FORWARD_LEFT;
        return proto::Direction::FORWARD;
    }
    if (backward) {
        if (right) return proto::Direction::BACKWARD_RIGHT;
        if (left) return proto::Direction::BACKWARD_LEFT;
        return proto::Direction::BACKWARD;
    }
    if (right) return proto::Direction::RIGHT;
    if (left) return proto::Direction::LEFT;
    return proto::Direction::STOP;
}

proto::Direction arrowDirection(int vk) {
    switch (vk) {
        case vkey::Up: return proto::Direction::FORWARD;
        case vkey::Down: return proto::Direction::BACKWARD;
        case vkey::Left: return proto::Direction::LEFT;
        case vkey::Right: return proto::Direction::RIGHT;
        default: return proto::Direction::STOP;
    }
}

bool isArrow(int vk) { return arrowDirection(vk) != proto::Direction::STOP; }

}  // namespace

InputController::InputController(DesiredStateSlot& slot) : slot_(slot) {}

void InputController::write() {
    float speed = 0.0f;
    if (direction_ != proto::Direction::STOP) {
        speed = power_;
        if (source_ == Source::Wasd && !shift_) speed *= kNoShiftFactor;
    }
    slot_.setDrive(direction_, speed);
}

void InputController::drive(proto::Direction direction, Source source) {
    direction_ = direction;
    source_ = source;
    write();
}

void InputController::stopDrive() {
    direction_ = proto::Direction::STOP;
    source_ = Source::None;
    write();
}

void InputController::applyWasd() {
    const proto::Direction direction = wasdDirection(held_wasd_);
    // STOP here means nothing is held any more, or the held keys cancel out.
    if (direction == proto::Direction::STOP)
        stopDrive();
    else
        drive(direction, Source::Wasd);
}

bool InputController::keyPress(int vk, bool autorepeat) {
    const unsigned bit = wasdBit(vk);
    if (bit == 0 && !isArrow(vk)) return false;
    if (autorepeat) return true;  // holding a key must not re-toggle anything

    if (bit != 0) {
        held_wasd_ |= bit;
        latched_arrow_ = 0;  // WASD took over; the next arrow press starts fresh
        applyWasd();
        return true;
    }

    if (latched_arrow_ == vk) {
        latched_arrow_ = 0;
        stopDrive();
    } else {
        latched_arrow_ = vk;
        drive(arrowDirection(vk), Source::Arrow);
    }
    return true;
}

bool InputController::keyRelease(int vk) {
    if (isArrow(vk)) return true;  // toggle mode: release does nothing
    const unsigned bit = wasdBit(vk);
    if (bit == 0) return false;

    // A release without a matching press (focus came back mid-keystroke) must
    // not disturb an arrow that is currently latched.
    if (held_wasd_ & bit) {
        held_wasd_ &= ~bit;
        latched_arrow_ = 0;
        applyWasd();  // the remaining keys keep driving; none of them means STOP
    }
    return true;
}

void InputController::releaseKeys() {
    held_wasd_ = 0;
    latched_arrow_ = 0;
    stopDrive();
}

void InputController::setPower(float power) {
    power_ = std::clamp(power, 0.0f, 1.0f);
    if (direction_ != proto::Direction::STOP) write();  // takes effect while driving
}

void InputController::setShift(bool down) {
    if (down == shift_) return;
    shift_ = down;
    if (direction_ != proto::Direction::STOP) write();
}

void InputController::mouseDelta(int dx, int dy) {
    yaw_ = std::clamp(yaw_ + dx * kMouseSensitivity, -1.0f, 1.0f);
    pitch_ = std::clamp(pitch_ - dy * kMouseSensitivity, -1.0f, 1.0f);  // mouse up = pitch up
    slot_.setCamera(pitch_, yaw_);
}

void InputController::resetCamera() {
    pitch_ = 0.0f;
    yaw_ = 0.0f;
    slot_.setCamera(pitch_, yaw_);
}

#include "input_controller.h"

#include <algorithm>

#include "../state/desired_state_slot.h"

namespace {

constexpr float kMouseSensitivity = 0.004f;  // full -1..1 sweep in ~500 px

proto::Direction wasdDirection(int vk) {
    switch (vk) {
        case vkey::W: return proto::Direction::FORWARD;
        case vkey::S: return proto::Direction::BACKWARD;
        case vkey::A: return proto::Direction::LEFT;
        case vkey::D: return proto::Direction::RIGHT;
        default: return proto::Direction::STOP;
    }
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

bool isWasd(int vk) { return wasdDirection(vk) != proto::Direction::STOP; }
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

bool InputController::keyPress(int vk, bool autorepeat) {
    if (!isWasd(vk) && !isArrow(vk)) return false;
    if (autorepeat) return true;  // holding a key must not re-toggle anything

    if (isWasd(vk)) {
        held_wasd_ = vk;
        latched_arrow_ = 0;  // WASD took over; the next arrow press starts fresh
        drive(wasdDirection(vk), Source::Wasd);
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
    if (!isWasd(vk)) return false;

    if (held_wasd_ == vk) {
        held_wasd_ = 0;
        latched_arrow_ = 0;
        stopDrive();
    }
    return true;
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

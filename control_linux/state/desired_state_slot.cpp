#include "desired_state_slot.h"

DesiredStateSlot::DesiredStateSlot() {
    state_.drive_cmd = {proto::Direction::STOP, 0.0f};
    state_.camera = {0.0f, 0.0f};
}

void DesiredStateSlot::setDrive(proto::Direction direction, float speed) {
    std::lock_guard<std::mutex> lock(mtx_);
    state_.drive_cmd.direction = direction;
    state_.drive_cmd.speed = speed;
}

void DesiredStateSlot::setCamera(float pitch, float yaw) {
    std::lock_guard<std::mutex> lock(mtx_);
    state_.camera.pitch = pitch;
    state_.camera.yaw = yaw;
}

proto::DesiredState DesiredStateSlot::get() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return state_;
}

#pragma once
#include "../../common/protocol.h"

class DesiredStateSlot;

// Windows virtual-key codes. Unlike Qt::Key they do not follow the active
// keyboard layout, so WASD keeps working on a Cyrillic layout.
namespace vkey {
constexpr int W = 0x57;
constexpr int A = 0x41;
constexpr int S = 0x53;
constexpr int D = 0x44;
constexpr int Left = 0x25;
constexpr int Up = 0x26;
constexpr int Right = 0x27;
constexpr int Down = 0x28;
}  // namespace vkey

// Keyboard/mouse -> DesiredStateSlot.
//   WASD    momentary: held down = drive, released = STOP.
//           Speed is power * 0.5, or power * 1.0 while Shift is down.
//   arrows  toggle:    press = drive at full power, press the same arrow = STOP
//   mouse   relative:  dx/dy accumulate into yaw/pitch, clamped to -1..1
// Both key sources write the same slot, so the newest write wins. Power lives
// here, on the control side only -- the protocol carries the resulting speed.
class InputController {
public:
    static constexpr float kNoShiftFactor = 0.5f;

    explicit InputController(DesiredStateSlot& slot);

    // Return true when the key belongs to the drive controls.
    bool keyPress(int vk, bool autorepeat);
    bool keyRelease(int vk);
    void mouseDelta(int dx, int dy);
    void resetCamera();

    void setPower(float power);  // 0.0 .. 1.0
    float power() const { return power_; }
    void setShift(bool down);

private:
    enum class Source { None, Wasd, Arrow };

    void drive(proto::Direction direction, Source source);
    void stopDrive();
    void write();  // re-send the current direction at the current speed

    DesiredStateSlot& slot_;
    float power_ = 1.0f;
    bool shift_ = false;
    proto::Direction direction_ = proto::Direction::STOP;
    Source source_ = Source::None;
    int held_wasd_ = 0;      // WASD key currently held, 0 = none
    int latched_arrow_ = 0;  // arrow currently driving, 0 = none
    float pitch_ = 0.0f;
    float yaw_ = 0.0f;
};

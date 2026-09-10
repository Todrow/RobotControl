#pragma once
// What the robot does on its own when the operator link goes away mid-drive.
//
// Losing the link at speed is the one situation where nobody is commanding the
// robot but it is still moving: stop, wait, back off a little, stop for good.
// Backing off buys the operator room to reconnect instead of leaving the hull
// pressed against whatever it was driving into.
//
// A pure state machine: no sockets, no UART, no printing, no clock of its own --
// the caller passes the time in and carries out the action it asks for. That is
// what makes the timing testable without a robot, and it is also why the same
// machine will serve autonomy later: it decides *what* should happen, and the
// control loop stays the only place that touches the drive.

#include <chrono>

#include "protocol.h"

enum class RecoveryPhase { Idle, Paused, Reversing };

// What the drive should be told at this instant.
enum class RecoveryAction {
    Wait,     // Nothing due yet.
    Reverse,  // Send (or repeat) the backing-off command.
    Stop,     // The manoeuvre is over.
};

class DisconnectRecovery {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    // Stay stopped this long before backing off, so a controller that comes
    // straight back never sees the robot move on its own.
    static constexpr auto kPause = std::chrono::milliseconds(1000);
    // A deliberately short nudge: enough to unstick the hull, not enough to
    // drive anywhere.
    static constexpr auto kReverseDuration = std::chrono::milliseconds(300);
    static constexpr float kReverseSpeed = 0.20f;
    // The ESP32 stops the motors after esp32_uart::COMMAND_TIMEOUT_MS of
    // silence, so the reverse command has to be repeated while it runs.
    static constexpr auto kHeartbeat = std::chrono::milliseconds(50);

    bool active() const noexcept { return phase_ != RecoveryPhase::Idle; }
    bool reversing() const noexcept { return phase_ == RecoveryPhase::Reversing; }
    // Only meaningful while active(): when the caller must look in again.
    TimePoint nextActionAt() const noexcept { return next_action_; }

    proto::DriveCommand reverseCommand() const noexcept {
        return {proto::Direction::BACKWARD, kReverseSpeed};
    }

    // Arm the manoeuvre after a session that was actually driving. `stopped_at`
    // is the moment the wheels were commanded to rest.
    void arm(TimePoint stopped_at) noexcept;

    RecoveryAction due(TimePoint now) const noexcept;

    // Call right after a Reverse action reached the drive. Repeats keep the
    // ESP32 fed but never extend the pulse: only the first write starts it.
    void reverseWritten(TimePoint written_at) noexcept;

    // Call after a Stop action reached the drive.
    void finish() noexcept { phase_ = RecoveryPhase::Idle; }

    // A reconnect or a shutdown cancels the manoeuvre. The caller still has to
    // stop the drive if reversing() was true.
    void cancel() noexcept { phase_ = RecoveryPhase::Idle; }

private:
    RecoveryPhase phase_ = RecoveryPhase::Idle;
    TimePoint next_action_{};
    TimePoint reverse_stop_at_{};
};

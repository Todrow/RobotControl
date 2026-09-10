// socket_utils.h first: on Windows it has to pull in winsock2.h ahead of
// anything that could reach windows.h.
#include "systems/rpi/link/socket_utils.h"

#include "control_loop.h"

#include <algorithm>
#include <cstdio>

#include "control/manual/teleop.h"
#include "control/safety/drive_gate.h"
#include "supervisor/intents.h"
#include "supervisor/recovery.h"
#include "systems/rpi/drive/drive_controller.h"
#include "systems/rpi/link/command_link.h"
#include "systems/rpi/servos/servo_controller.h"

namespace {

// How long the loop may sit in accept() before looking at the clock again.
constexpr auto kAcceptPoll = std::chrono::milliseconds(200);
// Shorter while driving itself: with no operator connected this is also how
// often the ESP32 gets fed, and it stops the motors after 500 ms of silence.
constexpr auto kAutonomousPoll = std::chrono::milliseconds(50);
// A controller that goes quiet this long has lost the link. Far above
// proto::COMMAND_PERIOD_MS, so ordinary scheduling jitter never trips it.
constexpr auto kReceiveTimeout = std::chrono::milliseconds(500);
// An autonomy command older than this reads as STOP: the explorer thread has
// stalled, and a robot must not keep driving on a command nobody is refreshing.
constexpr auto kAutonomyMaxAge = std::chrono::milliseconds(400);

// The safety gate, applied to every command regardless of where it came from --
// operator, link-loss manoeuvre or autonomy. Reads the verdict the lidar module
// published -- never a second copy of the thresholds -- with the same staleness
// cutoff telemetry uses, so a sector going Red (or its sample ageing out) takes
// effect within one pass.
proto::DriveCommand gated(const proto::DriveCommand& command, const RobotOptions& options,
                          const RobotState& state) {
    if (!options.lidar.enforce) return command;
    const SectorStatuses sectors =
        state.obstacles.statuses(std::chrono::milliseconds(options.lidar.max_age_ms));
    return gateDriveCommand(command, sectors);
}

// Carry out whatever the link-loss manoeuvre is due for. False means the drive
// failed and the robot has to come down.
bool serviceRecovery(DisconnectRecovery& recovery, DriveController& drive,
                     const RobotOptions& options, RobotState& state) {
    switch (recovery.due(net::Clock::now())) {
        case RecoveryAction::Wait:
            return true;
        case RecoveryAction::Stop: {
            state.drive_speed.store(0.0f);
            const bool okay = drive.stop();
            recovery.finish();
            std::printf("[cmd] disconnect recovery: STOP; waiting for reconnect\n");
            return okay;
        }
        case RecoveryAction::Reverse: {
            const bool starting = !recovery.reversing();
            const proto::DriveCommand reverse = recovery.reverseCommand();
            state.drive_speed.store(reverse.speed);
            // Still gated: never back the robot into a Red sector behind it.
            const bool okay = drive.apply(gated(reverse, options, state));
            recovery.reverseWritten(net::Clock::now());
            if (starting)
                std::printf("[cmd] disconnect recovery: BACKWARD %.0f%% for %lld ms\n",
                            DisconnectRecovery::kReverseSpeed * 100.0f,
                            static_cast<long long>(DisconnectRecovery::kReverseDuration.count()));
            return okay;
        }
    }
    return true;
}

// The operator asked for autonomy, or dropped it. Publishing the mode here, in
// the one thread that acts on it, keeps "who is driving" a single decision
// rather than a race between the button and the explorer.
void applyModeRequest(RobotState& state) {
    const ControlMode wanted =
        state.explore_requested.load() ? ControlMode::Explore : ControlMode::Manual;
    if (state.mode.load() == wanted) return;
    state.mode.store(wanted);
    std::printf("[cmd] mode: %s\n", wanted == ControlMode::Explore ? "EXPLORE" : "MANUAL");
}

// Any real movement command from the operator takes the robot back. Grabbing the
// stick has to work instantly and without thinking -- that is the whole point of
// having a human in the loop -- so this cancels autonomy rather than asking it
// to stop politely. A STOP or a camera-only frame is not a grab: the controller
// sends those continuously whether or not anyone is touching the keys.
bool operatorTookOver(const OperatorIntent& intent, RobotState& state) {
    if (state.mode.load() != ControlMode::Explore) return false;
    if (intent.drive.direction == proto::Direction::STOP || intent.drive.speed <= 0.0f)
        return false;
    state.explore_requested.store(false);
    state.mode.store(ControlMode::Manual);
    std::printf("[cmd] mode: MANUAL (operator took over)\n");
    return true;
}

}  // namespace

void runControlLoop(const RobotOptions& options, RobotState& state, VideoTarget& video_target) {
    CommandLink link(options.command_port);
    if (!link.open()) {
        state.abort();
        return;
    }

    DriveController drive(options.drive_uart);
    ServoController servos(options.servos);
    if (!drive.initialize() || !servos.initialize()) {
        state.abort();
        return;
    }

    DisconnectRecovery recovery;
    Teleop teleop;

    while (state.running.load()) {
        applyModeRequest(state);
        const bool autonomous = state.mode.load() == ControlMode::Explore;

        // Wake up for whichever comes first: a controller, or the next thing the
        // robot has to do on its own. Reconnecting has to stay possible
        // throughout the link-loss manoeuvre and throughout autonomous driving.
        auto accept_deadline = net::Clock::now() + (autonomous ? kAutonomousPoll : kAcceptPoll);
        if (recovery.active()) accept_deadline = std::min(accept_deadline, recovery.nextActionAt());
        const auto accepted = link.waitForClient(accept_deadline, state.running);

        if (accepted == net::IoResult::Timeout) {
            if (!state.running.load()) break;
            if (autonomous) {
                // Driving itself with nobody connected. Same gate, same UART,
                // same everything -- the only difference is where the command
                // came from.
                const proto::DriveCommand self = state.autonomy.command(kAutonomyMaxAge);
                state.drive_speed.store(
                    self.direction == proto::Direction::STOP ? 0.0f : self.speed);
                if (!drive.apply(gated(self, options, state))) {
                    state.abort();
                    break;
                }
            } else if (!serviceRecovery(recovery, drive, options, state)) {
                state.abort();
                break;
            }
            continue;
        }
        if (accepted == net::IoResult::Stopped) break;
        if (accepted != net::IoResult::Ok) {
            std::fprintf(stderr, "[cmd] accept failed (socket error %d)\n", net::lastError());
            state.abort();
            break;
        }

        // A new TCP connection cancels the old manoeuvre immediately, even
        // before its first command: never keep reversing while a receive waits.
        if (recovery.active()) {
            const bool stopped = !recovery.reversing() || drive.stop();
            recovery.cancel();
            if (!stopped) {
                state.abort();
                break;
            }
            std::printf("[cmd] disconnect recovery cancelled by reconnect\n");
        }
        video_target.setPeer(link.peer());
        std::printf("[cmd] control connected: %s\n", link.peer().c_str());

        teleop.reset();
        bool connection_lost = false;
        bool gate_blocking = false;
        while (state.running.load()) {
            proto::DesiredState frame{};
            const auto received =
                link.receive(frame, net::Clock::now() + kReceiveTimeout, state.running);
            if (received != net::IoResult::Ok) {
                connection_lost = received == net::IoResult::Timeout ||
                                  received == net::IoResult::Closed ||
                                  received == net::IoResult::Error;
                if (received == net::IoResult::Timeout)
                    std::fprintf(stderr, "[cmd] no complete command within %lld ms\n",
                                 static_cast<long long>(kReceiveTimeout.count()));
                break;
            }
            if (!validDesiredState(frame)) {
                std::fprintf(stderr, "[cmd] invalid command; closing connection\n");
                break;
            }
            const OperatorIntent intent = intentFrom(frame);
            operatorTookOver(intent, state);
            applyModeRequest(state);
            const bool driving_itself = state.mode.load() == ControlMode::Explore;

            // The camera always follows the operator, in either mode: watching
            // where the robot is going is not the same as steering it.
            if (!servos.apply(teleop.camera(intent))) {
                state.clearAppliedCamera();
                state.abort();
                break;
            }
            if (options.servos.enabled) state.setAppliedCamera(teleop.camera(intent));

            const proto::DriveCommand requested =
                driving_itself ? state.autonomy.command(kAutonomyMaxAge) : teleop.drive(intent);
            // The braking-zone floor tracks what was asked for, not what the gate
            // allowed: holding the stick into an obstacle keeps the zone wide so
            // it cannot flicker Red/clear at the boundary.
            state.drive_speed.store(
                requested.direction == proto::Direction::STOP ? 0.0f : requested.speed);
            const proto::DriveCommand allowed = gated(requested, options, state);
            const bool blocked_now = allowed.direction != requested.direction;
            if (blocked_now != gate_blocking) {
                gate_blocking = blocked_now;
                if (blocked_now)
                    std::fprintf(stderr,
                                 "[cmd] obstacle gate: %s refused, Red sector ahead; STOP\n",
                                 directionName(requested.direction));
                else
                    std::printf("[cmd] obstacle gate: path clear; operator control resumed\n");
            }
            // Forward every accepted command, including repeats, as a UART frame.
            if (!drive.apply(allowed)) {
                state.abort();
                break;
            }
            if (teleop.accept(intent) && !driving_itself) {
                const WheelPower wheels = driveWheelPower(requested);
                std::printf("[cmd] %s speed=%.2f  R:%d%%|L:%d%%  pitch=%+.3f yaw=%+.3f\n",
                            directionName(requested.direction), requested.speed,
                            wheels.right, wheels.left, intent.camera.pitch, intent.camera.yaw);
            }
        }

        // The operator is gone. Release the camera either way, but only stop the
        // wheels if the operator was the one driving them: in autonomy the robot
        // is meant to carry on without a controller, and stopping here would
        // fight the explorer for the UART.
        state.clearAppliedCamera();
        const bool still_autonomous = state.mode.load() == ControlMode::Explore;
        if (!still_autonomous) state.drive_speed.store(0.0f);
        const bool drive_stopped = still_autonomous || drive.stop();
        const auto stopped_at = net::Clock::now();
        video_target.clearPeer();
        const bool servos_released = servos.release();
        if (!drive_stopped || !servos_released) state.abort();
        std::printf("[cmd] control disconnected: %s%s\n", link.peer().c_str(),
                    still_autonomous ? " (still exploring)" : "");

        // Arm only once after a real driving session, and never in autonomy --
        // the manoeuvre would back the robot into whatever the explorer was
        // steering around. Startup, invalid input, silent reconnects, device
        // failures and shutdown stay stop-only.
        if (!still_autonomous && options.drive_uart.enabled && connection_lost &&
            teleop.everCommanded() && state.running.load()) {
            recovery.arm(stopped_at);
            std::printf("[cmd] disconnect recovery: STOP; waiting %lld ms before reverse\n",
                        static_cast<long long>(DisconnectRecovery::kPause.count()));
        }
        link.disconnect();
    }

    // Also release outputs if shutdown or an accept/UART error interrupts the
    // reverse pulse. This one is unconditional: the robot is coming down.
    state.clearAppliedCamera();
    state.drive_speed.store(0.0f);
    const bool drive_stopped = drive.stop();
    const bool servos_released = servos.release();
    if (!drive_stopped || !servos_released) state.abort();
    video_target.clearPeer();
}

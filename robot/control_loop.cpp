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
// A controller that goes quiet this long has lost the link. Far above
// proto::COMMAND_PERIOD_MS, so ordinary scheduling jitter never trips it.
constexpr auto kReceiveTimeout = std::chrono::milliseconds(500);

// The safety gate, applied to every command regardless of where it came from.
// Reads the verdict the lidar module published -- never a second copy of the
// thresholds -- with the same staleness cutoff telemetry uses, so a sector going
// Red (or its sample ageing out) takes effect within one pass.
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
            const bool okay = drive.stop();
            recovery.finish();
            std::printf("[cmd] disconnect recovery: STOP; waiting for reconnect\n");
            return okay;
        }
        case RecoveryAction::Reverse: {
            const bool starting = !recovery.reversing();
            // Still gated: never back the robot into a Red sector behind it.
            const bool okay = drive.apply(gated(recovery.reverseCommand(), options, state));
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
        // Wake up for whichever comes first: a controller, or the next step of
        // the manoeuvre. Reconnecting has to stay possible throughout the pause,
        // the reverse pulse and the final stop.
        auto accept_deadline = net::Clock::now() + kAcceptPoll;
        if (recovery.active()) accept_deadline = std::min(accept_deadline, recovery.nextActionAt());
        const auto accepted = link.waitForClient(accept_deadline, state.running);

        if (accepted == net::IoResult::Timeout) {
            if (!state.running.load()) break;
            if (!serviceRecovery(recovery, drive, options, state)) {
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

            // Apply each valid absolute camera position; the controller avoids
            // redundant PWM writes. Logging thresholds must not filter motion.
            if (!servos.apply(teleop.camera(intent))) {
                state.clearAppliedCamera();
                state.abort();
                break;
            }
            if (options.servos.enabled) state.setAppliedCamera(teleop.camera(intent));

            const proto::DriveCommand requested = teleop.drive(intent);
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
            if (teleop.accept(intent)) {
                const WheelPower wheels = driveWheelPower(requested);
                std::printf("[cmd] %s speed=%.2f  R:%d%%|L:%d%%  pitch=%+.3f yaw=%+.3f\n",
                            directionName(requested.direction), requested.speed,
                            wheels.right, wheels.left, intent.camera.pitch, intent.camera.yaw);
            }
        }

        // Also stop on malformed input, timeout, peer loss and server shutdown.
        // Attempt both releases even if either device reports an error.
        state.clearAppliedCamera();
        const bool drive_stopped = drive.stop();
        const auto stopped_at = net::Clock::now();
        video_target.clearPeer();
        const bool servos_released = servos.release();
        if (!drive_stopped || !servos_released) state.abort();
        std::printf("[cmd] control disconnected: %s\n", link.peer().c_str());

        // Arm only once after a real driving session. Startup, invalid input,
        // silent reconnects, device failures and shutdown stay stop-only.
        if (options.drive_uart.enabled && connection_lost && teleop.everCommanded() &&
            state.running.load()) {
            recovery.arm(stopped_at);
            std::printf("[cmd] disconnect recovery: STOP; waiting %lld ms before reverse\n",
                        static_cast<long long>(DisconnectRecovery::kPause.count()));
        }
        link.disconnect();
    }

    // Also release outputs if shutdown or an accept/UART error interrupts the
    // reverse pulse.
    state.clearAppliedCamera();
    const bool drive_stopped = drive.stop();
    const bool servos_released = servos.release();
    if (!drive_stopped || !servos_released) state.abort();
    video_target.clearPeer();
}

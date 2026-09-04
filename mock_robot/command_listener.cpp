#include "socket_utils.h"

#include <cmath>
#include <cstdio>

#include "robot_runtime.h"
#include "drive_controller.h"
#include "servo_controller.h"

namespace {

constexpr auto kDisconnectPause = std::chrono::milliseconds(1000);
constexpr auto kDisconnectReverseDuration = std::chrono::milliseconds(300);
constexpr float kDisconnectReverseSpeed = 0.20f;
constexpr auto kDisconnectHeartbeat = std::chrono::milliseconds(50);

enum class RecoveryPhase { None, Paused, Reversing };

const char* directionName(proto::Direction direction) {
    switch (direction) {
        case proto::Direction::STOP: return "STOP    ";
        case proto::Direction::FORWARD: return "FORWARD ";
        case proto::Direction::BACKWARD: return "BACKWARD";
        case proto::Direction::LEFT: return "LEFT    ";
        case proto::Direction::RIGHT: return "RIGHT   ";
    }
    return "?       ";
}

bool validState(const proto::DesiredState& state) {
    return state.drive_cmd.direction <= proto::Direction::RIGHT &&
           std::isfinite(state.drive_cmd.speed) && state.drive_cmd.speed >= 0.0f &&
           state.drive_cmd.speed <= 1.0f &&
           std::isfinite(state.camera.pitch) && std::fabs(state.camera.pitch) <= 1.0f &&
           std::isfinite(state.camera.yaw) && std::fabs(state.camera.yaw) <= 1.0f;
}

// Apply the LiDAR permission gate to a drive command. If motion in the
// requested direction is blocked, replace it with STOP so the wheels get a zero
// setpoint. Directions with no guard (left/right today) always pass through.
proto::DriveCommand gateByPermissions(const proto::DriveCommand& command,
                                      const MotionPermissions& permissions) {
    bool allowed = true;
    switch (command.direction) {
        case proto::Direction::FORWARD:  allowed = permissions.forward.load();  break;
        case proto::Direction::BACKWARD: allowed = permissions.backward.load(); break;
        case proto::Direction::LEFT:     allowed = permissions.left.load();     break;
        case proto::Direction::RIGHT:    allowed = permissions.right.load();    break;
        case proto::Direction::STOP:     allowed = true;                        break;
    }
    if (allowed) return command;
    return {proto::Direction::STOP, 0.0f};
}

}  // namespace

void runCommandListener(const RobotOptions& options, RobotRuntime& runtime, VideoTarget& video_target) {
    net::Socket server = net::listenOn(options.command_port);
    if (!server) {
        std::fprintf(stderr, "[cmd] cannot listen on %u (socket error %d)\n",
                     static_cast<unsigned>(options.command_port), net::lastError());
        runtime.failed.store(true);
        runtime.running.store(false);
        return;
    }

    DriveController drive(options.drive_uart);
    ServoController servos(options.servos);
    if (!drive.initialize() || !servos.initialize()) {
        runtime.failed.store(true);
        runtime.running.store(false);
        return;
    }

    RecoveryPhase recovery = RecoveryPhase::None;
    net::Deadline next_recovery_action{};
    net::Deadline reverse_stop_at{};

    while (runtime.running.load()) {
        net::Socket client;
        std::string peer;
        auto accept_deadline = net::Clock::now() + std::chrono::milliseconds(200);
        if (recovery != RecoveryPhase::None)
            accept_deadline = std::min(accept_deadline, next_recovery_action);
        const auto accepted = net::acceptClient(server.get(), client, peer,
            accept_deadline, runtime.running);
        if (accepted == net::IoResult::Timeout) {
            // Service the manoeuvre between accept polls, so reconnects remain
            // possible throughout the pause, reverse pulse and final stop.
            const auto now = net::Clock::now();
            if (!runtime.running.load()) break;
            if (recovery == RecoveryPhase::None || now < next_recovery_action) continue;

            bool okay = true;
            if (recovery == RecoveryPhase::Reversing && now >= reverse_stop_at) {
                okay = drive.stop();
                recovery = RecoveryPhase::None;
                std::printf("[cmd] disconnect recovery: STOP; waiting for reconnect\n");
            } else {
                // Repeat the short reverse command as an ESP32 heartbeat.
                // Only the first write starts the duration; repeats never extend it.
                // The disconnect back-off still respects the LiDAR guard: if the
                // rear is blocked, gating turns the reverse pulse into a stop.
                okay = drive.apply(gateByPermissions(
                    {proto::Direction::BACKWARD, kDisconnectReverseSpeed}, runtime.permissions));
                const auto written_at = net::Clock::now();
                if (recovery == RecoveryPhase::Paused) {
                    recovery = RecoveryPhase::Reversing;
                    reverse_stop_at = written_at + kDisconnectReverseDuration;
                    std::printf("[cmd] disconnect recovery: BACKWARD %.0f%% for %lld ms\n",
                                kDisconnectReverseSpeed * 100.0f,
                                static_cast<long long>(kDisconnectReverseDuration.count()));
                }
                next_recovery_action = std::min(reverse_stop_at, written_at + kDisconnectHeartbeat);
            }
            if (!okay) {
                runtime.failed.store(true);
                runtime.running.store(false);
                break;
            }
            continue;
        }
        if (accepted == net::IoResult::Stopped) break;
        if (accepted != net::IoResult::Ok) {
            std::fprintf(stderr, "[cmd] accept failed (socket error %d)\n", net::lastError());
            runtime.failed.store(true);
            runtime.running.store(false);
            break;
        }
        // A new TCP connection cancels the old manoeuvre immediately, even
        // before its first command. Never keep reversing while recvExact waits.
        if (recovery != RecoveryPhase::None) {
            const bool stopped = recovery != RecoveryPhase::Reversing || drive.stop();
            recovery = RecoveryPhase::None;
            if (!stopped) {
                runtime.failed.store(true);
                runtime.running.store(false);
                break;
            }
            std::printf("[cmd] disconnect recovery cancelled by reconnect\n");
        }
        video_target.setPeer(peer);
        std::printf("[cmd] control connected: %s\n", peer.c_str());

        // The Windows controller repeats the complete state every COMMAND_PERIOD_MS.
        proto::DesiredState previous{};
        bool have_previous = false;
        bool connection_lost = false;
        bool prev_gated = false;
        while (runtime.running.load()) {
            proto::DesiredState state{};
            const auto received = net::recvExact(client.get(), &state, sizeof(state),
                net::Clock::now() + std::chrono::milliseconds(500), runtime.running);
            if (received != net::IoResult::Ok) {
                connection_lost = received == net::IoResult::Timeout ||
                                  received == net::IoResult::Closed ||
                                  received == net::IoResult::Error;
                if (received == net::IoResult::Timeout)
                    std::fprintf(stderr, "[cmd] no complete command within 500 ms\n");
                break;
            }
            if (!validState(state)) {
                std::fprintf(stderr, "[cmd] invalid command; closing connection\n");
                break;
            }
            // Apply each valid absolute camera position; the controller avoids
            // redundant PWM writes. Logging thresholds must not filter motion.
            if (!servos.apply(state.camera)) {
                runtime.failed.store(true);
                runtime.running.store(false);
                break;
            }
            // LiDAR permission gate: if the requested direction is blocked by an
            // obstacle, the wheels receive STOP instead. The operator's raw
            // command is preserved for logging; only what we send is gated.
            const proto::DriveCommand gated =
                gateByPermissions(state.drive_cmd, runtime.permissions);
            const bool is_gated = gated.direction != state.drive_cmd.direction;

            // Forward every complete command, including repeats, as a UART frame.
            if (!drive.apply(gated)) {
                runtime.failed.store(true);
                runtime.running.store(false);
                break;
            }
            const bool changed =
                !have_previous || state.drive_cmd.direction != previous.drive_cmd.direction ||
                std::fabs(state.drive_cmd.speed - previous.drive_cmd.speed) > 0.001f ||
                std::fabs(state.camera.pitch - previous.camera.pitch) > 0.001f ||
                std::fabs(state.camera.yaw - previous.camera.yaw) > 0.001f ||
                is_gated != prev_gated;
            if (changed) {
                const WheelPower wheels = driveWheelPower(gated);
                std::printf("[cmd] %s speed=%.2f  R:%d%%|L:%d%%  pitch=%+.3f yaw=%+.3f%s\n",
                            directionName(state.drive_cmd.direction), state.drive_cmd.speed,
                            wheels.right, wheels.left,
                            state.camera.pitch, state.camera.yaw,
                            is_gated ? "  [BLOCKED by LiDAR]" : "");
                previous = state;
                have_previous = true;
            }
            prev_gated = is_gated;
        }
        // Also stop on malformed input, timeout, peer loss and server shutdown.
        // Attempt both releases even if either device reports an error.
        const bool drive_stopped = drive.stop();
        const auto stopped_at = net::Clock::now();
        video_target.clearPeer();
        const bool servos_released = servos.release();
        if (!drive_stopped || !servos_released) {
            runtime.failed.store(true);
            runtime.running.store(false);
        }
        std::printf("[cmd] control disconnected: %s\n", peer.c_str());
        // Arm only once after a real control session. Startup, invalid input,
        // silent reconnects, device failures and shutdown must remain stop-only.
        if (options.drive_uart.enabled && connection_lost && have_previous && runtime.running.load()) {
            recovery = RecoveryPhase::Paused;
            next_recovery_action = stopped_at + kDisconnectPause;
            std::printf("[cmd] disconnect recovery: STOP; waiting %lld ms before reverse\n",
                        static_cast<long long>(kDisconnectPause.count()));
        }
    }
    // Also release outputs if shutdown or an accept/UART error interrupts reverse.
    const bool drive_stopped = drive.stop();
    const bool servos_released = servos.release();
    if (!drive_stopped || !servos_released) {
        runtime.failed.store(true);
        runtime.running.store(false);
    }
    video_target.clearPeer();
}

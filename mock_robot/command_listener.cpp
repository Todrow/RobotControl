#include "socket_utils.h"

#include <cmath>
#include <cstdio>

#include "robot_runtime.h"
#include "drive_controller.h"
#include "servo_controller.h"

namespace {

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

    while (runtime.running.load()) {
        net::Socket client;
        std::string peer;
        const auto accepted = net::acceptClient(server.get(), client, peer,
            net::Clock::now() + std::chrono::milliseconds(200), runtime.running);
        if (accepted == net::IoResult::Timeout) continue;
        if (accepted == net::IoResult::Stopped) break;
        if (accepted != net::IoResult::Ok) {
            std::fprintf(stderr, "[cmd] accept failed (socket error %d)\n", net::lastError());
            runtime.failed.store(true);
            runtime.running.store(false);
            break;
        }
        video_target.setPeer(peer);
        std::printf("[cmd] control connected: %s\n", peer.c_str());

        // The Windows controller repeats the complete state every 50 ms.
        proto::DesiredState previous{};
        bool have_previous = false;
        while (runtime.running.load()) {
            proto::DesiredState state{};
            const auto received = net::recvExact(client.get(), &state, sizeof(state),
                net::Clock::now() + std::chrono::milliseconds(500), runtime.running);
            if (received != net::IoResult::Ok) {
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
            // Forward every complete command, including repeats, as a UART frame.
            if (!drive.apply(state.drive_cmd)) {
                runtime.failed.store(true);
                runtime.running.store(false);
                break;
            }
            const bool changed =
                !have_previous || state.drive_cmd.direction != previous.drive_cmd.direction ||
                std::fabs(state.drive_cmd.speed - previous.drive_cmd.speed) > 0.001f ||
                std::fabs(state.camera.pitch - previous.camera.pitch) > 0.001f ||
                std::fabs(state.camera.yaw - previous.camera.yaw) > 0.001f;
            if (changed) {
                const WheelPower wheels = driveWheelPower(state.drive_cmd);
                std::printf("[cmd] %s speed=%.2f  R:%d%%|L:%d%%  pitch=%+.3f yaw=%+.3f\n",
                            directionName(state.drive_cmd.direction), state.drive_cmd.speed,
                            wheels.right, wheels.left,
                            state.camera.pitch, state.camera.yaw);
                previous = state;
                have_previous = true;
            }
        }
        // Also stop on malformed input, timeout, peer loss and server shutdown.
        // Attempt both releases even if either device reports an error.
        const bool drive_stopped = drive.stop();
        video_target.clearPeer();
        const bool servos_released = servos.release();
        if (!drive_stopped || !servos_released) {
            runtime.failed.store(true);
            runtime.running.store(false);
        }
        std::printf("[cmd] control disconnected: %s\n", peer.c_str());
    }
    video_target.clearPeer();
}

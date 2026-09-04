#include "socket_utils.h"

#include <cmath>
#include <cstdio>
#include <random>

#include "robot_runtime.h"
#include "lidar_guard.h"

void runTelemetrySender(const RobotOptions& options, RobotRuntime& runtime,
                        const LidarGuard& lidar) {
    net::Socket server = net::listenOn(options.telemetry_port);
    if (!server) {
        std::fprintf(stderr, "[tlm] cannot listen on %u (socket error %d)\n",
                     static_cast<unsigned>(options.telemetry_port), net::lastError());
        runtime.failed.store(true);
        runtime.running.store(false);
        return;
    }

    // Preserve the existing simulator; these values do not come from Pi sensors.
    std::mt19937 rng(1234);
    std::uniform_real_distribution<float> noise(-0.4f, 0.4f);

    while (runtime.running.load()) {
        net::Socket client;
        std::string peer;
        const auto accepted = net::acceptClient(server.get(), client, peer,
            net::Clock::now() + std::chrono::milliseconds(200), runtime.running);
        if (accepted == net::IoResult::Timeout) continue;
        if (accepted == net::IoResult::Stopped) break;
        if (accepted != net::IoResult::Ok) {
            std::fprintf(stderr, "[tlm] accept failed (socket error %d)\n", net::lastError());
            runtime.failed.store(true);
            runtime.running.store(false);
            break;
        }
        std::printf("[tlm] control connected: %s (telemetry: %s)\n", peer.c_str(),
                    lidar.enabled() ? "obstacles REAL, rest simulated" : "SIMULATED");

        float t = 0.0f;
        float battery = 100.0f;
        auto next = net::Clock::now();
        while (runtime.running.load()) {
            proto::Telemetry telemetry{};
            telemetry.cpu_temp = 48.0f + 6.0f * std::sin(t) + noise(rng);
            battery = std::fmax(0.0f, battery - 0.02f);
            telemetry.battery_level = std::fmax(0.0f, std::fmin(100.0f, battery + noise(rng) * 0.2f));
            for (int i = 0; i < proto::LIDAR_POINTS; ++i) {
                telemetry.points[i].angle = i * (360.0f / proto::LIDAR_POINTS);
                telemetry.points[i].distance =
                    2.5f + 1.5f * std::sin(t + i * 0.5f) + noise(rng) * 0.1f;
                telemetry.points[i].intensity = 0.5f + 0.4f * std::cos(t * 0.7f + i);
            }
            // Real obstacle flags from the LiDAR guard (0 when disabled/clear).
            telemetry.obstacles = lidar.obstacleMask();

            const auto sent = net::sendExact(client.get(), &telemetry, sizeof(telemetry),
                net::Clock::now() + std::chrono::seconds(1), runtime.running);
            if (sent != net::IoResult::Ok) {
                if (sent == net::IoResult::Timeout)
                    std::fprintf(stderr, "[tlm] could not send a complete frame within 1 second\n");
                break;
            }
            t += 0.05f;
            next += std::chrono::milliseconds(proto::TELEMETRY_PERIOD_MS);
            // A slow peer must not cause a burst of old samples when it resumes.
            if (next < net::Clock::now()) next = net::Clock::now();
            net::sleepUntil(next, runtime.running);
        }
        std::printf("[tlm] control disconnected: %s\n", peer.c_str());
    }
}

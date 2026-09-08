#include "util/socket_utils.h"

#include <cstdio>

#include "robot_runtime.h"
#include "system_telemetry.h"

void runTelemetrySender(const RobotOptions& options, RobotRuntime& runtime) {
    net::Socket server = net::listenOn(options.telemetry_port);
    if (!server) {
        std::fprintf(stderr, "[tlm] cannot listen on %u (socket error %d)\n",
                     static_cast<unsigned>(options.telemetry_port), net::lastError());
        runtime.failed.store(true);
        runtime.running.store(false);
        return;
    }

    const SystemTelemetrySource sensors;
    std::printf("[tlm] system sensors; unavailable values are NaN; camera is applied PWM setpoint\n");

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
        std::printf("[tlm] control connected: %s\n", peer.c_str());

        auto next = net::Clock::now();
        while (runtime.running.load()) {
            proto::Telemetry telemetry = proto::unknownTelemetry();
            const auto system = sensors.sample();
            telemetry.cpu_temp = system.cpu_temp;
            telemetry.battery_level = system.battery_level;
            telemetry.camera = runtime.appliedCamera();

            const auto sent = net::sendExact(client.get(), &telemetry, sizeof(telemetry),
                net::Clock::now() + std::chrono::seconds(1), runtime.running);
            if (sent != net::IoResult::Ok) {
                if (sent == net::IoResult::Timeout)
                    std::fprintf(stderr, "[tlm] could not send a complete frame within 1 second\n");
                break;
            }
            next += std::chrono::milliseconds(proto::TELEMETRY_PERIOD_MS);
            // A slow peer must not cause a burst of old samples when it resumes.
            if (next < net::Clock::now()) next = net::Clock::now();
            net::sleepUntil(next, runtime.running);
        }
        std::printf("[tlm] control disconnected: %s\n", peer.c_str());
    }
}

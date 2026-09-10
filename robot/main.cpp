// socket_utils.h first: on Windows it has to pull in winsock2.h ahead of
// anything that could reach windows.h.
#include "systems/rpi/link/socket_utils.h"

#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <memory>
#include <string>
#include <thread>

#include <gst/gst.h>

#include "control_loop.h"
#include "options.h"
#include "state.h"
#include "systems/rpi/camera/camera_stream.h"
#include "systems/rpi/lidar/lidar_reader.h"
#include "systems/rpi/link/telemetry_link.h"
#include "systems/rpi/logging/logger.h"

// Assembly and nothing else: parse the options, start one thread per system,
// wait, shut everything down. Every decision the robot makes lives in
// control_loop.cpp; every device it touches lives under systems/.
namespace {

volatile std::sig_atomic_t stop_requested = 0;

void requestStop(int) {
    // Signal handlers must not lock, log, or call GStreamer.
    stop_requested = 1;
}

std::filesystem::path logDirectory() {
#ifdef _WIN32
    const char* home = std::getenv("USERPROFILE");
#else
    const char* home = std::getenv("HOME");
#endif
    const std::filesystem::path base = home ? std::filesystem::path(home) : std::filesystem::path(".");
    return base / "Documents" / "RobotControl";
}

// A worker that throws takes the robot down rather than leaving the others
// running against half-initialized hardware.
template <typename Function>
void runSafely(const char* name, RobotState& state, Function function) {
    try {
        function();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "[%s] worker failed: %s\n", name, error.what());
        state.abort();
    } catch (...) {
        std::fprintf(stderr, "[%s] worker failed with an unknown exception\n", name);
        state.abort();
    }
}

}  // namespace

int main(int argc, char* argv[]) {
    // Keep connection/camera diagnostics visible when redirected to a service log.
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--help") {
            printUsage(argv[0]);
            return 0;
        }
    }

#ifdef _WIN32
    WSADATA wsa{};
    const int startup_error = WSAStartup(MAKEWORD(2, 2), &wsa);
    if (startup_error != 0) {
        std::fprintf(stderr, "WSAStartup failed: %d\n", startup_error);
        return 1;
    }
#endif
    RobotOptions options;
    if (!parseOptions(argc, argv, options)) {
#ifdef _WIN32
        WSACleanup();
#endif
        return 1;
    }
    GError* error = nullptr;
    if (!gst_init_check(nullptr, nullptr, &error)) {
        std::fprintf(stderr, "GStreamer initialization failed: %s\n",
                     error ? error->message : "unknown error");
        if (error) g_error_free(error);
#ifdef _WIN32
        WSACleanup();
#endif
        return 1;
    }
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);

    std::unique_ptr<Logger> logger;
    try {
        const std::filesystem::path log_dir = logDirectory();
        std::filesystem::create_directories(log_dir);
        const std::filesystem::path log_path = log_dir / "robot.log";
        logger = std::make_unique<Logger>(log_path.string());
        logger->info("robot starting: cmd=", options.command_port,
                     " tlm=", options.telemetry_port, " video=", options.video_port);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "Logger initialization failed: %s\n", error.what());
    }

    RobotState state;
    VideoTarget video_target(options.video_host);
    std::printf("robot: cmd=%u tlm=%u video=%u (Ctrl+C to quit)\n",
                static_cast<unsigned>(options.command_port),
                static_cast<unsigned>(options.telemetry_port),
                static_cast<unsigned>(options.video_port));
    std::printf("Lidar: %s. Red sectors %s drive commands toward them.\n",
                options.lidar.source == LidarSource::Device ? "STL-19P device"
                : options.lidar.source == LidarSource::Simulated ? "simulated (PLACEHOLDER distances)"
                                                                : "disabled",
                options.lidar.enforce ? "BLOCK" : "do NOT block (report-only)");
    std::printf("Drive UART: %s. Camera servos: %s. Telemetry: system sensors / NaN if unavailable.\n",
                options.drive_uart.enabled ? "ENABLED" : "disabled (use --drive-uart)",
                options.servos.enabled ? "ENABLED" : "disabled (use --servos)");
    if (logger) {
        logger->info("Drive UART: ", options.drive_uart.enabled ? "ENABLED" : "disabled",
                     ". Camera servos: ", options.servos.enabled ? "ENABLED" : "disabled",
                     ". Obstacle enforcement: ", options.lidar.enforce ? "ENABLED" : "disabled");
    }

    std::thread control;
    std::thread telemetry;
    std::thread video;
    std::thread lidar;
    try {
        control = std::thread([&] {
            runSafely("cmd", state, [&] { runControlLoop(options, state, video_target); });
        });
        telemetry = std::thread([&] {
            runSafely("tlm", state, [&] { runTelemetryLink(options, state); });
        });
        video = std::thread([&] {
            runSafely("video", state, [&] { runCameraStream(options, state, video_target); });
        });
        lidar = std::thread([&] {
            runSafely("lidar", state, [&] { runLidarReader(options, state); });
        });
        while (state.running.load() && !stop_requested)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } catch (const std::exception& exception) {
        std::fprintf(stderr, "Cannot start worker threads: %s\n", exception.what());
        state.failed.store(true);
    }
    state.running.store(false);
    if (control.joinable()) control.join();
    if (telemetry.joinable()) telemetry.join();
    if (video.joinable()) video.join();
    if (lidar.joinable()) lidar.join();

    gst_deinit();
#ifdef _WIN32
    WSACleanup();
#endif
    return state.failed.load() ? 1 : 0;
}

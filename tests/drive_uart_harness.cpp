#include "robot_runtime.h"

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <thread>

namespace {
volatile std::sig_atomic_t stopping = 0;
void requestStop(int) { stopping = 1; }
}

// Run the real command listener without camera, telemetry or GStreamer. The
// parent test owns the PTY and sends ordinary DesiredState frames over TCP.
int main(int argc, char** argv) {
    if (argc != 3) return 2;
    std::signal(SIGTERM, requestStop);
    std::signal(SIGINT, requestStop);
    std::setvbuf(stdout, nullptr, _IONBF, 0);

    RobotOptions options;
    options.command_port = static_cast<uint16_t>(std::strtoul(argv[1], nullptr, 10));
    options.drive_uart.enabled = true;
    options.drive_uart.device = argv[2];
    RobotRuntime runtime;
    VideoTarget target("");
    std::thread listener([&] { runCommandListener(options, runtime, target); });
    while (runtime.running.load() && !stopping)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    runtime.running.store(false);
    listener.join();
    return runtime.failed.load() ? 1 : 0;
}

#include "control_loop.h"

#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <thread>

namespace {
volatile std::sig_atomic_t stopping = 0;
void requestStop(int) { stopping = 1; }
}

// Run the real control loop without camera, telemetry or GStreamer. The parent
// test owns the PTY and sends ordinary DesiredState frames over TCP, so this
// exercises the whole path the robot uses: link -> supervisor -> control ->
// drive, including the safety gate and the link-loss manoeuvre.
int main(int argc, char** argv) {
    if (argc != 3) return 2;
    std::signal(SIGTERM, requestStop);
    std::signal(SIGINT, requestStop);
    std::setvbuf(stdout, nullptr, _IONBF, 0);

    RobotOptions options;
    options.command_port = static_cast<uint16_t>(std::strtoul(argv[1], nullptr, 10));
    options.drive_uart.enabled = true;
    options.drive_uart.device = argv[2];
    RobotState state;
    VideoTarget target("");
    std::thread loop([&] { runControlLoop(options, state, target); });
    while (state.running.load() && !stopping)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    state.running.store(false);
    loop.join();
    return state.failed.load() ? 1 : 0;
}

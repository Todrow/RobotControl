#pragma once
// Every knob the robot server has, plus the command line that fills them in.
//
// Options are read once at startup and then only read, never written, so every
// thread may hold a const reference to the same struct. Keeping the parser here
// instead of in main.cpp leaves main.cpp as what it should be: wiring.

#include <cstdint>
#include <string>

#include "protocol.h"
#include "systems/rpi/drive/drive_config.h"
#include "systems/rpi/lidar/lidar_config.h"
#include "systems/rpi/servos/servo_config.h"

enum class VideoSource { Camera, Test, Disabled };

struct RobotOptions {
    uint16_t command_port = proto::COMMAND_PORT;
    uint16_t telemetry_port = proto::TELEMETRY_PORT;
    uint16_t video_port = proto::VIDEO_PORT;
    std::string video_host;
    VideoSource video_source = VideoSource::Camera;
    int camera_index = 0;
    int width = 1280;
    int height = 720;
    int framerate = 30;
    int bitrate = 4000000;
    DriveOptions drive_uart;
    ServoOptions servos;
    LidarOptions lidar;
};

void printUsage(const char* executable);

// Fills `options` from argv. Prints its own diagnostics and returns false on the
// first bad value, leaving `options` partially written -- the caller exits.
bool parseOptions(int argc, char* argv[], RobotOptions& options);

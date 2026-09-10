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

// Scan matching has no odometry to fall back on, so every number here is about
// how much movement it is allowed to believe in between two revolutions.
struct SlamOptions {
    bool enabled = true;
    // A revolution with fewer usable points than this says nothing about where
    // the robot is -- matching it would be fitting noise.
    int min_points = 60;
    // Mean evidence under the scan, 0..1, below which the match is rejected:
    // pose stays put and the map is left alone. Too low and a bad match poisons
    // the map; too high and SLAM refuses to work in a sparse room.
    float min_score = 0.22f;
    // Half-width of the widest search, in metres and degrees. This is the most
    // the robot may have moved since the last scan; with no odometry it is a
    // pure guess, so it has to cover the fastest the robot can drive in one
    // lidar period, with margin.
    float search_radius_m = 0.60f;
    float search_radius_deg = 30.0f;
};

// The exploration policy: drive to the nearest place the robot has not seen.
struct ExploreOptions {
    // Autonomy drives slowly. The safety gate reacts within one command period,
    // but stopping distance is still real, and SLAM matches better at low speed.
    float speed = 0.35f;
    // Half the widest part of the hull, plus margin. Cells this close to a wall
    // are removed from the planner's map, so a path is never planned through a
    // gap the robot cannot fit through. MEASURE THIS on the real robot.
    float robot_radius_m = 0.20f;
    // Close enough to the goal to call it reached and pick the next one.
    float goal_tolerance_m = 0.25f;
    // Frontiers nearer than this are ignored while anything further exists. A
    // goal half a metre away is reached before the robot has finished turning
    // towards it, so the robot spends its time turning rather than travelling.
    // Only if nothing further is reachable does it settle for a close one.
    float min_goal_distance_m = 1.2f;
    // Give up on a goal that has not been reached in this long and pick another.
    // Without it a goal the robot cannot quite get to would hold it forever.
    int goal_timeout_ms = 25000;
    // How often to rebuild the route. The GOAL is not reconsidered this often --
    // only the path to it -- so a longer period here just means the route is a
    // little staler, not that the robot dithers.
    int replan_period_ms = 700;
    // Steering: how far off the path heading before turning in place instead of
    // driving an arc.
    float turn_in_place_deg = 35.0f;
};

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
    SlamOptions slam;
    ExploreOptions explore;
    // Map + autonomy commands travel on their own TCP channel, so the existing
    // command and telemetry protocols stay byte-for-byte what they were.
    uint16_t map_port = 5004;
};

void printUsage(const char* executable);

// Fills `options` from argv. Prints its own diagnostics and returns false on the
// first bad value, leaving `options` partially written -- the caller exits.
bool parseOptions(int argc, char* argv[], RobotOptions& options);

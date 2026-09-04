#pragma once
// Wire protocol shared by control and the robot side. Fixed-size structs only:
// no framing, no CRC, no sequence numbers -- a reader takes exactly sizeof(T)
// bytes off the socket and reinterprets them.
#include <cstdint>

namespace proto {

constexpr uint16_t COMMAND_PORT   = 5001;  // control -> robot, DesiredState
constexpr uint16_t TELEMETRY_PORT = 5002;  // robot -> control, Telemetry
constexpr uint16_t VIDEO_PORT     = 5003;  // robot -> control, RTP/H264 over UDP

constexpr int COMMAND_PERIOD_MS   = 5;
constexpr int TELEMETRY_PERIOD_MS = 100;

constexpr int LIDAR_POINTS = 12;

enum class Direction : uint8_t { STOP = 0, FORWARD = 1, BACKWARD = 2, LEFT = 3, RIGHT = 4 };

struct DriveCommand {
    Direction direction;
    float speed;  // 0.0 .. 1.0
};

struct CameraState {
    float pitch;  // -1.0 .. 1.0
    float yaw;    // -1.0 .. 1.0
};

#pragma pack(push, 1)
struct DesiredState {
    DriveCommand drive_cmd;
    CameraState camera;
};

struct LidarPoint {
    float angle;
    float distance;
    float intensity;
};

struct Telemetry {
    float cpu_temp;
    float battery_level;
    LidarPoint points[LIDAR_POINTS];
    CameraState camera;
};
#pragma pack(pop)

// DriveCommand is declared outside the packed region, so it keeps 3 bytes of
// padding after `direction`; the asserts below pin the resulting on-wire layout
// that any peer implementation (Raspberry/Arduino) has to reproduce byte for byte.
static_assert(sizeof(DriveCommand) == 8);
static_assert(sizeof(CameraState) == 8);
static_assert(sizeof(DesiredState) == 16);
static_assert(sizeof(LidarPoint) == 12);
static_assert(sizeof(Telemetry) == 152);

}  // namespace proto

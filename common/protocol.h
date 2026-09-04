#pragma once
// Wire protocol shared by control and the robot side. Fixed-size structs only:
// no framing, no CRC, no sequence numbers -- a reader takes exactly sizeof(T)
// bytes off the socket and reinterprets them.
#include <cstddef>
#include <cstdint>
#include <limits>
#include <type_traits>

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
    float cpu_temp;       // Degrees Celsius; NaN if unavailable.
    float battery_level;  // Driver-reported percent, 0..100; NaN if unavailable.
    LidarPoint points[LIDAR_POINTS];
    // Last successfully applied PWM setpoint, in the command's -1..1 coordinates.
    // This is not a measured physical angle. NaN while PWM is inactive/unknown.
    CameraState camera;
};
#pragma pack(pop)

// Missing sensors must not look like real zero readings. Receivers must check
// individual floats with isfinite() before displaying/using them. No fake lidar.
inline constexpr float UNKNOWN_TELEMETRY_VALUE = std::numeric_limits<float>::quiet_NaN();

inline Telemetry unknownTelemetry() {
    Telemetry telemetry{};
    telemetry.cpu_temp = UNKNOWN_TELEMETRY_VALUE;
    telemetry.battery_level = UNKNOWN_TELEMETRY_VALUE;
    for (auto& point : telemetry.points)
        point = {UNKNOWN_TELEMETRY_VALUE, UNKNOWN_TELEMETRY_VALUE, UNKNOWN_TELEMETRY_VALUE};
    telemetry.camera = {UNKNOWN_TELEMETRY_VALUE, UNKNOWN_TELEMETRY_VALUE};
    return telemetry;
}

// DriveCommand is declared outside the packed region, so it keeps 3 bytes of
// padding after `direction`; the asserts below pin the resulting on-wire layout
// that any peer implementation (Raspberry/Arduino) has to reproduce byte for byte.
// Native little-endian IEEE-754 peers only. The camera extension changes Telemetry
// from 152 to 160 bytes: BOTH endpoints must be rebuilt; there is no negotiation.
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
static_assert(std::is_trivially_copyable_v<DesiredState> && std::is_standard_layout_v<DesiredState>);
static_assert(std::is_trivially_copyable_v<Telemetry> && std::is_standard_layout_v<Telemetry>);
static_assert(sizeof(DriveCommand) == 8);
static_assert(offsetof(DriveCommand, direction) == 0);
static_assert(offsetof(DriveCommand, speed) == 4);
static_assert(sizeof(CameraState) == 8);
static_assert(offsetof(CameraState, pitch) == 0);
static_assert(offsetof(CameraState, yaw) == 4);
static_assert(sizeof(DesiredState) == 16);
static_assert(offsetof(DesiredState, drive_cmd) == 0);
static_assert(offsetof(DesiredState, camera) == 8);
static_assert(sizeof(LidarPoint) == 12);
static_assert(offsetof(LidarPoint, angle) == 0);
static_assert(offsetof(LidarPoint, distance) == 4);
static_assert(offsetof(LidarPoint, intensity) == 8);
static_assert(sizeof(Telemetry) == 160);
static_assert(offsetof(Telemetry, cpu_temp) == 0);
static_assert(offsetof(Telemetry, battery_level) == 4);
static_assert(offsetof(Telemetry, points) == 8);
static_assert(offsetof(Telemetry, camera) == 152);

}  // namespace proto

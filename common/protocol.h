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

// Obstacle sectors, 60 degrees each, clockwise from the robot's nose. The value
// is the index into Telemetry::sectors; both endpoints must agree on the order.
constexpr int SECTOR_COUNT = 6;

enum Sector : uint8_t {
    SECTOR_FORWARD       = 0,
    SECTOR_FORWARD_RIGHT = 1,
    SECTOR_BACK_RIGHT    = 2,
    SECTOR_BACK          = 3,
    SECTOR_BACK_LEFT     = 4,
    SECTOR_FORWARD_LEFT  = 5,
};

// The robot compares each measured distance against two thresholds and sends
// only the verdict; raw millimetres never reach control. Unknown is not "clear":
// it means the sector was not measured (no lidar, stale or rejected reading).
enum class SectorStatus : uint8_t {
    Unknown = 0,
    Green   = 1,
    Yellow  = 2,
    Red     = 3,
};

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

struct Telemetry {
    float cpu_temp;       // Degrees Celsius; NaN if unavailable.
    float battery_level;  // Driver-reported percent, 0..100; NaN if unavailable.
    // Threshold verdict per sector, indexed by Sector. Unknown while the lidar
    // module publishes nothing or its last sample went stale.
    SectorStatus sectors[SECTOR_COUNT];
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
    for (auto& sector : telemetry.sectors) sector = SectorStatus::Unknown;
    telemetry.camera = {UNKNOWN_TELEMETRY_VALUE, UNKNOWN_TELEMETRY_VALUE};
    return telemetry;
}

// DriveCommand is declared outside the packed region, so it keeps 3 bytes of
// padding after `direction`; the asserts below pin the resulting on-wire layout
// that any peer implementation (Raspberry/Arduino) has to reproduce byte for byte.
// Native little-endian IEEE-754 peers only. Replacing the unused lidar point
// array with six sector verdicts changes Telemetry from 160 to 22 bytes: BOTH
// endpoints must be rebuilt; there is no negotiation.
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
static_assert(sizeof(SectorStatus) == 1);
static_assert(sizeof(Telemetry) == 22);  // 8 + 6 sectors + CameraState(8)
static_assert(offsetof(Telemetry, sectors) == 8);
static_assert(offsetof(Telemetry, camera) == 14);

}  // namespace proto

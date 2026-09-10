#include "systems/rpi/link/socket_utils.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "systems/rpi/lidar/lidar_reader.h"
#include "systems/rpi/lidar/lidar_distances.h"
#include "systems/rpi/lidar/scan.h"

#ifdef _WIN32
#include <windows.h>
// <windows.h> (pulled in by socket_utils.h too) defines ERROR as a macro, and
// the SDK has an enumerator by that name.
#undef ERROR

#include "ldlidar_driver/ldlidar_driver_win.h"
#else
#include <dirent.h>
#include <cstring>

#include "ldlidar_driver/ldlidar_driver_linux.h"
#endif

namespace {

#ifdef _WIN32
using LidarDriver = ldlidar::LDLidarDriverWinInterface;
#else
using LidarDriver = ldlidar::LDLidarDriverLinuxInterface;
#endif

// LDROBOT STL-19P; the SDK drives it as LD_19 at 230400 bit/s.
constexpr ldlidar::LDType kLidarType = ldlidar::LDType::LD_19;
constexpr uint32_t kBaudrate = 230400;

// The circle is split into six 60-degree zones centred on the sector directions,
// so together they cover it without gaps. Zone i is proto::Sector i: forward,
// forward-right, back-right, back, back-left, forward-left.
constexpr float kZoneHalfWidthDeg = 30.0f;
constexpr float kZoneCenterDeg[proto::SECTOR_COUNT] = {0.0f, 60.0f, 120.0f, 180.0f, 240.0f, 300.0f};

// Where the lidar's own zero points. 0 = the arrow printed on its case looks
// forward; turn the device on the robot or change this, not both.
constexpr float kFrontOffsetDeg = 0.0f;

// Set to true if left and right come out mirrored, i.e. the angle grows the
// other way round on this unit.
constexpr bool kMirrorAngle = false;

// Closer than this is noise around the axis. The far clamp is per-run
// (LidarOptions::sector_range_mm, passed into readDistances): a reading past it
// is dropped and a sector with no echo in range reads as exactly that value, so
// every sector always carries a number.
constexpr float kMinValidMm = 20.0f;

// SLAM needs the opposite of the sector clamp above: everything the device can
// actually see. At 500 mm no map would ever form -- the robot would be walking
// around inside a half-metre bubble. The STL-19P is specified to 12 m; readings
// past that are noise, not walls.
constexpr float kScanMaxRangeMm = 12000.0f;

// How long to wait for the port to name itself / for the first scan.
constexpr int64_t kCommTimeoutMs = 2000;
// Consecutive failed reads before the device is closed and reopened. At the
// default period that is a couple of seconds of silence.
constexpr int kMaxReadFailures = 20;

float normAngle(float a) {
    while (a < 0.0f) a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    return a;
}

// Raw lidar angle -> robot angle: front offset first, then the mirror.
float toRobotAngle(float raw) {
    float a = raw - kFrontOffsetDeg;
    if (kMirrorAngle) a = -a;
    return normAngle(a);
}

// Which sector the angle falls into, or -1 if it falls outside every zone.
int sectorOfAngle(float robot_angle) {
    for (int sector = 0; sector < proto::SECTOR_COUNT; ++sector) {
        float rel = normAngle(robot_angle - kZoneCenterDeg[sector]);
        if (rel > 180.0f) rel -= 360.0f;
        if (std::fabs(rel) <= kZoneHalfWidthDeg) return sector;
    }
    return -1;
}

// ---------------------------------------------------------------------------
//  Port discovery. Nothing is configured: every serial device the system
//  offers is tried in turn, and the one that answers with a parsable scan is
//  the lidar. A port that belongs to something else fails WaitLidarComm and is
//  released again.
// ---------------------------------------------------------------------------
std::vector<std::string> candidatePorts() {
    std::vector<std::string> ports;
#ifdef _WIN32
    // HARDWARE\DEVICEMAP\SERIALCOMM lists exactly the COM ports that exist now.
    HKEY key = nullptr;
    if (RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM", 0, KEY_READ, &key) !=
        ERROR_SUCCESS)
        return ports;
    for (DWORD index = 0;; ++index) {
        char name[256];
        char value[256];
        DWORD name_len = sizeof name;
        DWORD value_len = sizeof value;
        DWORD type = 0;
        const LONG status = RegEnumValueA(key, index, name, &name_len, nullptr, &type,
                                          reinterpret_cast<LPBYTE>(value), &value_len);
        if (status != ERROR_SUCCESS) break;
        if (type == REG_SZ && value_len > 0) ports.emplace_back(value);
    }
    RegCloseKey(key);
#else
    // USB-serial adapters (the lidar's CP210x among them) show up as ttyUSB*,
    // CDC-ACM ones as ttyACM*. The Pi's own GPIO UART is deliberately not
    // touched: that one belongs to the drive.
    if (DIR* dev = opendir("/dev")) {
        while (const dirent* entry = readdir(dev)) {
            if (std::strncmp(entry->d_name, "ttyUSB", 6) == 0 ||
                std::strncmp(entry->d_name, "ttyACM", 6) == 0)
                ports.emplace_back(std::string("/dev/") + entry->d_name);
        }
        closedir(dev);
    }
    std::sort(ports.begin(), ports.end());
#endif
    return ports;
}

bool connectPort(LidarDriver* driver, const std::string& port) {
#ifdef _WIN32
    PortParams params;  // PortParams/BaudRate live in the global namespace.
    params.baudrate = BaudRate::Baud230400;
    std::string name = port;
    return driver->Connect(kLidarType, name, params);
#else
    return driver->Connect(kLidarType, port, kBaudrate);
#endif
}

void closeLidar(LidarDriver* driver) {
    if (driver == nullptr) return;
    driver->Stop();
    driver->Disconnect();
    LidarDriver::Destory(driver);
}

// Opens the first port that actually answers as a lidar. Returns nullptr if no
// port did; the caller retries, so a lidar plugged in later is still found.
LidarDriver* openLidar(std::string& opened_port) {
    for (const std::string& port : candidatePorts()) {
        LidarDriver* driver = LidarDriver::Create();
        driver->RegisterGetTimestampFunctional([]() -> uint64_t {
            return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                             std::chrono::system_clock::now().time_since_epoch())
                                             .count());
        });
        driver->EnablePointCloudDataFilter(true);  // drops the trailing false points

        if (connectPort(driver, port)) {
            if (driver->WaitLidarComm(kCommTimeoutMs) && driver->Start()) {
                opened_port = port;
                return driver;
            }
            driver->Disconnect();
        }
        LidarDriver::Destory(driver);
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
//  One revolution of points -> the nearest obstacle per sector, in millimetres.
//  A zone with no echo within range reads as max_range_mm rather than NaN: the
//  reading is real, it just says "nothing closer than the clamp".
// ---------------------------------------------------------------------------
// The same revolution as readDistances(), kept whole and turned into Cartesian
// points in the robot frame. The device measures angles clockwise from the nose,
// so y is negated: the rest of the autonomy code uses x forward, y left and
// counter-clockwise angles, and the conversion belongs here, once, rather than
// inside every trig call in SLAM.
LaserScan buildScan(const ldlidar::Points2D& scan) {
    LaserScan out;
    out.points.reserve(scan.size());
    for (const ldlidar::PointData& point : scan) {
        const float millimetres = static_cast<float>(point.distance);
        if (millimetres < kMinValidMm || millimetres > kScanMaxRangeMm) continue;
        const float radians = toRobotAngle(point.angle) * 3.14159265358979f / 180.0f;
        const float metres = millimetres / 1000.0f;
        out.points.push_back({metres * std::cos(radians), -metres * std::sin(radians)});
    }
    out.stamp = std::chrono::steady_clock::now();
    return out;
}

LidarDistances readDistances(const ldlidar::Points2D& scan, float max_range_mm) {
    std::array<float, proto::SECTOR_COUNT> nearest;
    nearest.fill(max_range_mm);
    for (const ldlidar::PointData& point : scan) {
        const float millimetres = static_cast<float>(point.distance);
        if (millimetres < kMinValidMm || millimetres > max_range_mm) continue;
        const int sector = sectorOfAngle(toRobotAngle(point.angle));
        if (sector < 0) continue;
        nearest[static_cast<size_t>(sector)] =
            std::min(nearest[static_cast<size_t>(sector)], millimetres);
    }

    LidarDistances distances;
    distances.forward       = nearest[proto::SECTOR_FORWARD];
    distances.forward_right = nearest[proto::SECTOR_FORWARD_RIGHT];
    distances.back_right    = nearest[proto::SECTOR_BACK_RIGHT];
    distances.back          = nearest[proto::SECTOR_BACK];
    distances.back_left     = nearest[proto::SECTOR_BACK_LEFT];
    distances.forward_left  = nearest[proto::SECTOR_FORWARD_LEFT];
    return distances;
}

// ---------------------------------------------------------------------------
//  No hardware is read here. Each sector sweeps 150..1150 mm on its own offset,
//  so every threshold band shows up in the operator's display and the colours
//  can be checked without a lidar attached.
// ---------------------------------------------------------------------------
LidarDistances simulatedDistances(float phase) {
    const auto sweep = [phase](int sector) {
        return 650.0f + 500.0f * std::sin(phase + static_cast<float>(sector) * 1.05f);
    };
    LidarDistances distances;
    distances.forward       = sweep(proto::SECTOR_FORWARD);
    distances.forward_right = sweep(proto::SECTOR_FORWARD_RIGHT);
    distances.back_right    = sweep(proto::SECTOR_BACK_RIGHT);
    distances.back          = sweep(proto::SECTOR_BACK);
    distances.back_left     = sweep(proto::SECTOR_BACK_LEFT);
    distances.forward_left  = sweep(proto::SECTOR_FORWARD_LEFT);
    return distances;
}

// ---------------------------------------------------------------------------
//  Simulated room, for --lidar-source sim.
//
//  Exists so the whole autonomy path -- SLAM, the explorer, the map channel and
//  the operator display -- can be run and watched on a desktop with no robot,
//  no lidar and no serial port. It is a debugging aid and nothing more: the
//  robot only turns on the spot here, so it exercises the pipeline, not the
//  driving. The room has a doorway on purpose, otherwise it would be sealed and
//  the explorer would correctly report that there is nowhere left to go.
// ---------------------------------------------------------------------------
struct SimSegment {
    float ax, ay, bx, by;
};

const SimSegment kSimRoom[] = {
    {-2.5f, -2.0f, 2.5f, -2.0f},  {2.5f, -2.0f, 2.5f, 2.0f},
    {2.5f, 2.0f, 0.6f, 2.0f},     // wall stops short: doorway from 0.6 to -0.6
    {-0.6f, 2.0f, -2.5f, 2.0f},   {-2.5f, 2.0f, -2.5f, -2.0f},
    {1.0f, 0.2f, 1.4f, 0.2f},     {1.4f, 0.2f, 1.4f, 0.6f},
    {1.4f, 0.6f, 1.0f, 0.6f},     {1.0f, 0.6f, 1.0f, 0.2f},
};

float simCastRay(float ox, float oy, float angle) {
    const float dx = std::cos(angle);
    const float dy = std::sin(angle);
    float nearest = -1.0f;
    for (const SimSegment& wall : kSimRoom) {
        const float ex = wall.bx - wall.ax;
        const float ey = wall.by - wall.ay;
        const float denominator = dx * ey - dy * ex;
        if (std::fabs(denominator) < 1e-9f) continue;
        const float t = ((wall.ax - ox) * ey - (wall.ay - oy) * ex) / denominator;
        const float u = ((wall.ax - ox) * dy - (wall.ay - oy) * dx) / denominator;
        if (t <= 0.0f || u < 0.0f || u > 1.0f) continue;
        if (nearest < 0.0f || t < nearest) nearest = t;
    }
    return nearest;
}

LaserScan simulatedScan(float heading) {
    LaserScan scan;
    scan.points.reserve(360);
    for (int degree = 0; degree < 360; ++degree) {
        const float bearing = static_cast<float>(degree) * 3.14159265358979f / 180.0f;
        const float range = simCastRay(0.0f, 0.0f, heading + bearing);
        if (range <= 0.0f || range > kScanMaxRangeMm / 1000.0f) continue;
        scan.points.push_back({range * std::cos(bearing), range * std::sin(bearing)});
    }
    scan.stamp = std::chrono::steady_clock::now();
    return scan;
}

void runSimulated(const RobotOptions& options, RobotState& state) {
    float phase = 0.0f;
    float heading = 0.0f;
    auto next = net::Clock::now();
    while (state.running.load()) {
        const float commanded_mm_s = state.drive_speed.load() * options.lidar.zone.max_speed_mm_s;
        state.obstacles.update(simulatedDistances(phase), options.lidar.zone, commanded_mm_s);
        // The scan comes from the virtual room, not from the sector sweep above:
        // those placeholder distances are a display test and would make no sense
        // as a map.
        state.scans.publish(simulatedScan(heading));
        heading += 0.02f;
        if (heading > 6.2831853f) heading -= 6.2831853f;

        phase += 0.06f;
        if (phase > 6.2831853f) phase -= 6.2831853f;
        next += std::chrono::milliseconds(options.lidar.period_ms);
        // A late cycle must not turn into a burst of catch-up samples.
        if (next < net::Clock::now()) next = net::Clock::now();
        net::sleepUntil(next, state.running);
    }
}

}  // namespace

void runLidarReader(const RobotOptions& options, RobotState& state) {
    if (options.lidar.source == LidarSource::Disabled) {
        // Publishing nothing is not the same as publishing "clear": every sector
        // stays Unknown, and control shows it as unknown rather than as free space.
        std::printf("[lidar] disabled; all sectors report Unknown\n");
        return;
    }

    if (options.lidar.source == LidarSource::Simulated) {
        std::printf("[lidar] SIMULATED distances; red %.0f..%.0f mm (speed-scaled), every %d ms\n",
                    static_cast<double>(options.lidar.zone.red_base_mm),
                    static_cast<double>(options.lidar.zone.red_max_mm),
                    options.lidar.period_ms);
        runSimulated(options, state);
        return;
    }

    std::printf("[lidar] STL-19P, range %d mm; red %.0f..%.0f mm (speed-scaled), every %d ms\n",
                options.lidar.sector_range_mm,
                static_cast<double>(options.lidar.zone.red_base_mm),
                static_cast<double>(options.lidar.zone.red_max_mm),
                options.lidar.period_ms);

    // A scan is a full revolution, so never wait less than one; the loop below
    // owns the pacing, this is only the ceiling on one blocking read.
    const int64_t read_timeout_ms = std::max(200, options.lidar.period_ms * 2);

    LidarDriver* driver = nullptr;
    std::string port;
    int read_failures = 0;
    ldlidar::Points2D scan;
    auto next = net::Clock::now();

    while (state.running.load()) {
        if (driver == nullptr) {
            driver = openLidar(port);
            if (driver == nullptr) {
                // No lidar on any port yet. Publishing nothing leaves every
                // sector Unknown, which is what an unmeasured sector is.
                std::fprintf(stderr, "[lidar] no lidar found on any serial port; retrying\n");
                net::sleepUntil(net::Clock::now() + std::chrono::seconds(2), state.running);
                continue;
            }
            std::printf("[lidar] running on %s\n", port.c_str());
            read_failures = 0;
            next = net::Clock::now();
        }

        if (driver->GetLaserScanData(scan, read_timeout_ms) == ldlidar::LidarStatus::NORMAL) {
            read_failures = 0;
            const float commanded_mm_s =
                state.drive_speed.load() * options.lidar.zone.max_speed_mm_s;
            state.obstacles.update(
                readDistances(scan, static_cast<float>(options.lidar.sector_range_mm)),
                options.lidar.zone, commanded_mm_s);
            // Second, independent product: the full revolution for SLAM. The
            // verdicts above are unaffected by anything that happens to it.
            state.scans.publish(buildScan(scan));
        } else if (++read_failures >= kMaxReadFailures) {
            // Unplugged, unpowered or wedged. Drop the handle and look for it
            // again; the last sample ages out to Unknown on its own.
            std::fprintf(stderr, "[lidar] %s stopped sending data; reopening\n", port.c_str());
            closeLidar(driver);
            driver = nullptr;
            continue;
        }

        next += std::chrono::milliseconds(options.lidar.period_ms);
        // A late cycle must not turn into a burst of catch-up samples.
        if (next < net::Clock::now()) next = net::Clock::now();
        net::sleepUntil(next, state.running);
    }

    closeLidar(driver);
}

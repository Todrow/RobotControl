#include "lidar_guard.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>

#include "protocol.h"

#ifdef HAVE_LIDAR_SDK
#if defined(__linux__)
#include "ldlidar_driver/ldlidar_driver_linux.h"
#else
#include "ldlidar_driver/ldlidar_driver_win.h"
#endif
#endif  // HAVE_LIDAR_SDK

namespace {

#ifdef HAVE_LIDAR_SDK
uint64_t timestampNs() {
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch())
            .count());
}

float normalizeAngle(float a) {
    while (a < 0.0f) a += 360.0f;
    while (a >= 360.0f) a -= 360.0f;
    return a;
}

// True when `angle` lies within +/- half_width of a sector centre.
bool inSector(float angle, float centre, float half_width) {
    float rel = normalizeAngle(angle - centre);
    if (rel > 180.0f) rel -= 360.0f;
    return std::fabs(rel) <= half_width;
}
#endif  // HAVE_LIDAR_SDK

}  // namespace

LidarGuard::LidarGuard(const LidarOptions& options, MotionPermissions& permissions)
    : options_(options), permissions_(permissions) {}

LidarGuard::~LidarGuard() { stop(); }

bool LidarGuard::start() {
    if (!options_.enabled) return true;  // inert
#ifndef HAVE_LIDAR_SDK
    std::fprintf(stderr,
                 "[lidar] --lidar requested but built without ldlidar_sdk; guard inactive\n");
    return false;
#else
    running_.store(true);
    worker_ = std::thread([this] { run(); });
    return true;
#endif
}

void LidarGuard::stop() noexcept {
    running_.store(false);
    if (worker_.joinable()) worker_.join();
    // Leave everything permitted when the guard goes away.
    permissions_.allowAll();
    obstacle_mask_.store(0);
}

#ifdef HAVE_LIDAR_SDK
void LidarGuard::run() {
#if defined(__linux__)
    auto* drv = ldlidar::LDLidarDriverLinuxInterface::Create();
#else
    auto* drv = ldlidar::LDLidarDriverWinInterface::Create();
#endif

    drv->RegisterGetTimestampFunctional(std::bind(&timestampNs));
    drv->EnablePointCloudDataFilter(true);

#if defined(__linux__)
    std::string device = options_.device;
    const bool ok = drv->Connect(ldlidar::LDType::LD_19, device, 230400);
#else
    std::string device = options_.device;
    PortParams params;
    params.baudrate = BaudRate::Baud230400;
    const bool ok = drv->Connect(ldlidar::LDType::LD_19, device, params);
#endif

    auto destroy = [&]() {
#if defined(__linux__)
        ldlidar::LDLidarDriverLinuxInterface::Destory(drv);
#else
        ldlidar::LDLidarDriverWinInterface::Destory(drv);
#endif
    };

    if (!ok) {
        std::fprintf(stderr, "[lidar] connect failed on %s; guard inactive\n",
                     options_.device.c_str());
        destroy();
        return;
    }
    if (!drv->WaitLidarComm(3500) || !drv->Start()) {
        std::fprintf(stderr, "[lidar] no data / start failed on %s; guard inactive\n",
                     options_.device.c_str());
        drv->Disconnect();
        destroy();
        return;
    }

    std::printf("[lidar] STL-19P guard active on %s: stop at %.0f cm, sector +/-%.0f deg\n",
                options_.device.c_str(), options_.stop_distance_m * 100.0f,
                options_.half_width_deg);

    ldlidar::Points2D scan;
    const float min_valid_mm = options_.min_valid_m * 1000.0f;
    const float stop_mm = options_.stop_distance_m * 1000.0f;
    const float front_centre = options_.front_offset_deg;
    const float back_centre = normalizeAngle(options_.front_offset_deg + 180.0f);

    while (running_.load()) {
        const ldlidar::LidarStatus st = drv->GetLaserScanData(scan, 1500);
        if (st == ldlidar::LidarStatus::NORMAL) {
            bool front_blocked = false;
            bool back_blocked = false;
            for (const auto& p : scan) {
                if (p.distance < min_valid_mm) continue;  // noise near the hub
                if (p.distance > stop_mm) continue;       // far enough, ignore
                if (inSector(p.angle, front_centre, options_.half_width_deg))
                    front_blocked = true;
                else if (inSector(p.angle, back_centre, options_.half_width_deg))
                    back_blocked = true;
            }

            // Publish permissions (writer side). left/right stay allowed for now.
            permissions_.forward.store(!front_blocked);
            permissions_.backward.store(!back_blocked);

            uint8_t mask = proto::OBSTACLE_NONE;
            if (front_blocked) mask |= proto::OBSTACLE_FRONT;
            if (back_blocked) mask |= proto::OBSTACLE_BACK;
            obstacle_mask_.store(mask);
        } else if (st == ldlidar::LidarStatus::DATA_TIME_OUT) {
            // Lost the LiDAR: fail safe -- block both driving directions.
            permissions_.forward.store(false);
            permissions_.backward.store(false);
            obstacle_mask_.store(proto::OBSTACLE_FRONT | proto::OBSTACLE_BACK);
        }
    }

    drv->Stop();
    drv->Disconnect();
    destroy();
}
#else   // no SDK: never called (start() returns false), but must exist to link
void LidarGuard::run() {}
#endif  // HAVE_LIDAR_SDK

#pragma once

#include <atomic>
#include <cstdint>
#include <thread>

#include "lidar_config.h"
#include "motion_permissions.h"

// Background LiDAR guard for the LDROBOT STL-19P (built on ldlidar_sdk).
//
// One worker thread owns the SDK driver, continuously reads scans, and checks
// the front and back sectors. When something is closer than stop_distance_m in
// a sector it clears the matching MotionPermissions flag (forward/backward), so
// the command thread refuses to drive that way. It also publishes an obstacle
// bitmask (proto::ObstacleFlags) for telemetry.
//
// If disabled or the LiDAR fails to open, the guard stays inert: every
// direction remains permitted and the obstacle mask stays 0, so the robot
// behaves exactly as before.
class LidarGuard {
public:
    LidarGuard(const LidarOptions& options, MotionPermissions& permissions);
    ~LidarGuard();

    LidarGuard(const LidarGuard&) = delete;
    LidarGuard& operator=(const LidarGuard&) = delete;

    bool start();          // connect + spawn worker; true if disabled (no-op)
    void stop() noexcept;

    bool enabled() const noexcept { return options_.enabled; }
    uint8_t obstacleMask() const noexcept { return obstacle_mask_.load(); }

private:
    void run();

    const LidarOptions options_;
    MotionPermissions& permissions_;
    std::thread worker_;
    std::atomic<bool> running_{false};
    std::atomic<uint8_t> obstacle_mask_{0};
};

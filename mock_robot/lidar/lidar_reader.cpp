#include "utils/socket_utils.h"

#include <cmath>
#include <cstdio>

#include "robot_runtime.h"
#include "utils/lidar_distances.h"

namespace {

// ---------------------------------------------------------------------------
//  THE LIDAR DRIVER GOES HERE -- this function is the entire hand-off point.
//
//  Replace readDistances() with real measurements and nothing else in this file
//  (or anywhere else) has to change: the worker loop below already owns the
//  thread, the timing and the publishing.
//
//  Contract:
//    * return the distance to the NEAREST obstacle in each of the six sectors,
//      in MILLIMETRES;
//    * a sector you did not measure, or whose reading you rejected as noise,
//      must stay NaN -- see the comment in utils/lidar_distances.h for why 0 and
//      "some large number" are both wrong;
//    * do not block for longer than one period. If the device needs a long
//      blocking read, do it here anyway: this thread is yours alone and nothing
//      else waits on it, but the sample will age and start reading as Unknown
//      after lidar.max_age_ms;
//    * do not touch RobotRuntime, sockets or the drive. Measuring is the whole
//      job; the threshold policy lives in utils/obstacle_check.h.
//
//  Device setup/teardown (opening the port, starting the SDK) belongs at the top
//  and bottom of runLidarReader() below, where the two marked spots are.
// ---------------------------------------------------------------------------
LidarDistances readDistances(float phase) {
    // PLACEHOLDER: no hardware is read. Each sector sweeps 150..1150 mm on its
    // own offset, so every threshold band shows up in the operator's display and
    // the colours can be checked without a lidar attached.
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

}  // namespace

void runLidarReader(const RobotOptions& options, RobotRuntime& runtime) {
    if (options.lidar.source == LidarSource::Disabled) {
        // Publishing nothing is not the same as publishing "clear": every sector
        // stays Unknown, and control shows it as unknown rather than as free space.
        std::printf("[lidar] disabled; all sectors report Unknown\n");
        return;
    }

    // DEVICE SETUP GOES HERE (open the port, start the SDK, bail out on failure).

    std::printf("[lidar] SIMULATED distances; red <%.0f mm, yellow <%.0f mm, every %d ms\n",
                static_cast<double>(options.lidar.thresholds.red_mm),
                static_cast<double>(options.lidar.thresholds.yellow_mm),
                options.lidar.period_ms);

    float phase = 0.0f;
    auto next = net::Clock::now();
    while (runtime.running.load()) {
        runtime.obstacles.update(readDistances(phase), options.lidar.thresholds);

        phase += 0.06f;
        if (phase > 6.2831853f) phase -= 6.2831853f;
        next += std::chrono::milliseconds(options.lidar.period_ms);
        // A late cycle must not turn into a burst of catch-up samples.
        if (next < net::Clock::now()) next = net::Clock::now();
        net::sleepUntil(next, runtime.running);
    }

    // DEVICE TEARDOWN GOES HERE (stop the SDK, close the port).
}

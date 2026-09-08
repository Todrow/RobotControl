// Threshold policy tests: measured millimetres in, three-state verdict out.
// Pure functions, so no lidar, no sockets and no Raspberry Pi are involved.
//   c++ -std=c++17 -Icommon -Imock_robot tests/test_obstacle_check.cpp -o /tmp/t && /tmp/t
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>

#include "protocol.h"
#include "utils/obstacle_check.h"

namespace {

int failures = 0;

void check(bool condition, const std::string& what) {
    if (condition) return;
    std::fprintf(stderr, "FAIL: %s\n", what.c_str());
    ++failures;
}

void testBands() {
    const ObstacleThresholds thresholds{300.0f, 800.0f};
    check(classify(299.0f, thresholds) == proto::SectorStatus::Red, "Below red is Red");
    check(classify(300.0f, thresholds) == proto::SectorStatus::Yellow,
          "The red threshold itself is already Yellow");
    check(classify(799.0f, thresholds) == proto::SectorStatus::Yellow, "Below yellow is Yellow");
    check(classify(800.0f, thresholds) == proto::SectorStatus::Green,
          "The yellow threshold itself is already Green");
    check(classify(5000.0f, thresholds) == proto::SectorStatus::Green, "Far away is Green");
}

// The dangerous failure is a missing reading that looks like a clear path.
void testMissingReadingsAreNeverGreen() {
    const ObstacleThresholds thresholds{300.0f, 800.0f};
    const float nan_value = std::numeric_limits<float>::quiet_NaN();
    const float infinity = std::numeric_limits<float>::infinity();
    check(classify(nan_value, thresholds) == proto::SectorStatus::Unknown, "NaN is Unknown");
    check(classify(infinity, thresholds) == proto::SectorStatus::Unknown, "Infinity is Unknown");
    check(classify(0.0f, thresholds) == proto::SectorStatus::Unknown, "Zero is Unknown, not Red");
    check(classify(-1.0f, thresholds) == proto::SectorStatus::Unknown, "Negative is Unknown");
}

// Field names must line up with the wire indices control reads.
void testSectorMapping() {
    const ObstacleThresholds thresholds{300.0f, 800.0f};
    LidarDistances distances;  // every field starts unmeasured
    distances.forward = 100.0f;
    distances.forward_right = 500.0f;
    distances.back = 2000.0f;

    const SectorStatuses statuses = evaluate(distances, thresholds);
    check(statuses[proto::SECTOR_FORWARD] == proto::SectorStatus::Red, "forward maps to index 0");
    check(statuses[proto::SECTOR_FORWARD_RIGHT] == proto::SectorStatus::Yellow,
          "forward_right maps to index 1");
    check(statuses[proto::SECTOR_BACK_RIGHT] == proto::SectorStatus::Unknown,
          "back_right was never measured");
    check(statuses[proto::SECTOR_BACK] == proto::SectorStatus::Green, "back maps to index 3");
    check(statuses[proto::SECTOR_BACK_LEFT] == proto::SectorStatus::Unknown,
          "back_left was never measured");
    check(statuses[proto::SECTOR_FORWARD_LEFT] == proto::SectorStatus::Unknown,
          "forward_left was never measured");
    check(unknownStatuses()[proto::SECTOR_FORWARD] == proto::SectorStatus::Unknown,
          "The default verdict is Unknown");
}

void testThresholdValidation() {
    check(validThresholds({300.0f, 800.0f}), "Ordered thresholds are valid");
    check(!validThresholds({800.0f, 300.0f}), "Swapped thresholds are rejected");
    check(!validThresholds({300.0f, 300.0f}), "Equal thresholds leave no yellow band");
    check(!validThresholds({0.0f, 800.0f}), "A zero red threshold is rejected");
}

}  // namespace

int main() {
    testBands();
    testMissingReadingsAreNeverGreen();
    testSectorMapping();
    testThresholdValidation();
    if (failures != 0) {
        std::fprintf(stderr, "%d obstacle check test(s) failed\n", failures);
        return 1;
    }
    std::printf("Obstacle check tests passed\n");
    return 0;
}

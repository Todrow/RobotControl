// Threshold policy tests: measured millimetres in, three-state verdict out.
// Pure functions, so no lidar, no sockets and no Raspberry Pi are involved.
//   c++ -std=c++17 -Icommon -Irobot tests/test_obstacle_check.cpp -o /tmp/t && /tmp/t
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>

#include "protocol.h"
#include "control/safety/obstacle_check.h"

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

// The speed-scaled red zone: at rest it is red_base_mm, it grows by
// lookahead_s * closing_speed while the gap shrinks, and it is clamped at
// red_max_mm. Yellow always trails red by exactly yellow_margin_mm.
void testBrakingZoneScaling() {
    const BrakingZone zone{200.0f, 600.0f, 50.0f, 1.0f, 500.0f};

    const ObstacleThresholds at_rest = thresholdsFor(zone, 0.0f);
    check(at_rest.red_mm == 200.0f, "Zero closing speed leaves red at red_base_mm");
    check(at_rest.yellow_mm == 250.0f, "Yellow is red + yellow_margin_mm");

    const ObstacleThresholds opening = thresholdsFor(zone, -300.0f);
    check(opening.red_mm == 200.0f, "A widening gap never shrinks the zone below red_base_mm");

    // 1.0 s lookahead * 250 mm/s closing = +250 mm, below the 600 mm ceiling.
    const ObstacleThresholds closing = thresholdsFor(zone, 250.0f);
    check(std::fabs(closing.red_mm - 450.0f) < 1e-3f, "Red grows by lookahead_s * closing_speed");
    check(std::fabs(closing.yellow_mm - 500.0f) < 1e-3f, "Yellow tracks the grown red");

    const ObstacleThresholds clamped = thresholdsFor(zone, 100000.0f);
    check(clamped.red_mm == 600.0f, "Red is clamped at red_max_mm");
    check(clamped.yellow_mm == 650.0f, "Yellow stays margin above the clamped red");
}

// A sector whose obstacle is rushing in is Red at a distance that is safely
// Green when nothing is moving -- that is the whole point of the feature.
void testSpeedScaledEvaluate() {
    const BrakingZone zone{200.0f, 600.0f, 50.0f, 1.0f, 500.0f};
    LidarDistances distances;
    distances.forward = 400.0f;
    distances.back = 400.0f;

    SectorSpeeds steady{};  // all zero
    check(evaluate(distances, zone, steady)[proto::SECTOR_FORWARD] == proto::SectorStatus::Green,
          "400 mm with nothing closing is Green");

    SectorSpeeds forward_rushing{};
    forward_rushing[proto::SECTOR_FORWARD] = 300.0f;  // zone grows to 500 mm
    const SectorStatuses statuses = evaluate(distances, zone, forward_rushing);
    check(statuses[proto::SECTOR_FORWARD] == proto::SectorStatus::Red,
          "400 mm with a 300 mm/s approach is Red");
    check(statuses[proto::SECTOR_BACK] == proto::SectorStatus::Green,
          "The sector that is not closing is judged on its own speed");
}

void testBrakingZoneValidation() {
    check(validBrakingZone({200.0f, 600.0f, 50.0f, 1.0f, 500.0f}), "A well-ordered zone is valid");
    check(validBrakingZone({200.0f, 200.0f, 50.0f, 0.0f, 500.0f}),
          "red_max == red_base and zero lookahead are allowed (feature off)");
    check(!validBrakingZone({200.0f, 150.0f, 50.0f, 1.0f, 500.0f}),
          "red_max_mm below red_base_mm is rejected");
    check(!validBrakingZone({0.0f, 600.0f, 50.0f, 1.0f, 500.0f}), "A zero red base is rejected");
    check(!validBrakingZone({200.0f, 600.0f, 0.0f, 1.0f, 500.0f}),
          "A zero yellow margin leaves no yellow band");
    check(!validBrakingZone({200.0f, 600.0f, 50.0f, -1.0f, 500.0f}),
          "A negative lookahead is rejected");
    check(!validBrakingZone({200.0f, 600.0f, 50.0f, 1.0f, 0.0f}),
          "A zero max speed cannot scale the commanded floor");
}

}  // namespace

int main() {
    testBands();
    testMissingReadingsAreNeverGreen();
    testSectorMapping();
    testThresholdValidation();
    testBrakingZoneScaling();
    testSpeedScaledEvaluate();
    testBrakingZoneValidation();
    if (failures != 0) {
        std::fprintf(stderr, "%d obstacle check test(s) failed\n", failures);
        return 1;
    }
    std::printf("Obstacle check tests passed\n");
    return 0;
}

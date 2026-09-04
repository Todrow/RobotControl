// Standalone Linux test; no Raspberry Pi sensors are required.
// g++ -std=c++17 -Wall -Wextra -Wpedantic -Icommon -Imock_robot tests/test_system_telemetry.cpp mock_robot/system_telemetry.cpp -o /tmp/test_system_telemetry
#include "protocol.h"
#include "system_telemetry.h"

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <system_error>

namespace fs = std::filesystem;

namespace {

void check(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void checkValue(float actual, float expected, const std::string& message) {
    check(std::isfinite(actual) && std::fabs(actual - expected) < 0.001f, message);
}

class SysfsFixture {
public:
    SysfsFixture() {
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        for (int attempt = 0; attempt < 100; ++attempt) {
            const auto candidate = fs::temp_directory_path() /
                ("robotcontrol-telemetry-test-" + std::to_string(stamp) + "-" +
                 std::to_string(attempt));
            if (fs::create_directory(candidate)) {
                root = candidate;
                return;
            }
        }
        throw std::runtime_error("Could not create a unique sysfs fixture directory");
    }

    ~SysfsFixture() {
        std::error_code ignored;
        fs::remove_all(root, ignored);
    }

    SysfsFixture(const SysfsFixture&) = delete;
    SysfsFixture& operator=(const SysfsFixture&) = delete;

    void write(const std::string& relative, const std::string& value) const {
        const auto path = root / relative;
        fs::create_directories(path.parent_path());
        std::ofstream file(path);
        check(static_cast<bool>(file), "Cannot open fixture file: " + relative);
        file << value;
        file.close();
        check(static_cast<bool>(file), "Cannot write fixture file: " + relative);
    }

    void thermal(int index, const std::string& type, const std::string& temp) const {
        const auto zone = "class/thermal/thermal_zone" + std::to_string(index) + "/";
        write(zone + "type", type + "\n");
        write(zone + "temp", temp + "\n");
    }

    void battery(const std::string& name, const std::string& capacity) const {
        const auto supply = "class/power_supply/" + name + "/";
        write(supply + "type", "Battery\n");
        write(supply + "capacity", capacity + "\n");
    }

    fs::path root;
};

void testCpuDiscoveryAndValidation() {
    SysfsFixture fixture;
    SystemTelemetrySource source(fixture.root.string());
    check(std::isnan(source.sample().cpu_temp), "Absent CPU sensor must be unknown");

    fixture.thermal(0, "cpu-thermal", "48500");
    checkValue(source.sample().cpu_temp, 48.5f, "CPU millidegrees must become degrees");
    fixture.thermal(1, "cpu_thermal", "53000");
    fixture.thermal(2, "bcm2835_thermal", "51000");
    fixture.thermal(3, "gpu-thermal", "99000");
    checkValue(source.sample().cpu_temp, 53.0f, "Use maximum valid CPU temperature, ignoring GPU");
    fixture.write("class/thermal/thermal_zone1/temp", "invalid\n");
    checkValue(source.sample().cpu_temp, 51.0f, "Ignore a malformed CPU alongside valid CPU sensors");
    fs::remove_all(fixture.root / "class/thermal/thermal_zone0");
    fs::remove_all(fixture.root / "class/thermal/thermal_zone1");
    checkValue(source.sample().cpu_temp, 51.0f, "Recognize bcm2835 CPU sensor");

    for (const std::string invalid : {"", "48500mC", "48.5", "48500 50000", "nan",
                                      "-273151", "250001", "999999999999999999999"}) {
        fixture.write("class/thermal/thermal_zone2/temp", invalid + "\n");
        check(std::isnan(source.sample().cpu_temp), "Reject invalid CPU value: " + invalid);
    }
    for (const auto value : {0, -10000, -273150, 250000}) {
        fixture.write("class/thermal/thermal_zone2/temp", std::to_string(value) + "\n");
        checkValue(source.sample().cpu_temp, static_cast<float>(value) / 1000.0f,
                   "Accept an in-range CPU temperature");
    }
}

void testBatteryDiscoveryAndValidation() {
    SysfsFixture fixture;
    SystemTelemetrySource source(fixture.root.string());
    check(std::isnan(source.sample().battery_level), "Absent battery must be unknown");

    fixture.battery("BAT0", "0");
    checkValue(source.sample().battery_level, 0.0f, "An empty battery is a real reading");
    fixture.write("class/power_supply/BAT0/capacity", "100\n");
    fixture.write("class/power_supply/BAT0/scope", "System\n");
    fixture.write("class/power_supply/BAT0/present", "1\n");
    checkValue(source.sample().battery_level, 100.0f, "Accept a full, present system battery");
    fixture.write("class/power_supply/BAT0/scope", "corrupt\n");
    check(std::isnan(source.sample().battery_level), "Reject a malformed battery scope");
    fixture.write("class/power_supply/BAT0/scope", "Unknown\n");
    checkValue(source.sample().battery_level, 100.0f, "An unknown scope is a valid driver value");
    fixture.write("class/power_supply/BAT0/scope", "System\n");

    fixture.battery("mouse", "70");
    fixture.write("class/power_supply/mouse/scope", "Device\n");
    fixture.write("class/power_supply/AC/type", "Mains\n");
    fixture.write("class/power_supply/AC/capacity", "60\n");
    checkValue(source.sample().battery_level, 100.0f, "Ignore device batteries and mains supplies");

    for (const std::string invalid : {"", "-1", "101", "50%", "50.0", "50 60",
                                      "999999999999999999999"}) {
        fixture.write("class/power_supply/BAT0/capacity", invalid + "\n");
        check(std::isnan(source.sample().battery_level), "Reject invalid battery capacity: " + invalid);
    }
    fixture.write("class/power_supply/BAT0/capacity", "42\n");
    for (const std::string absent : {"0", "invalid", "", "2"}) {
        fixture.write("class/power_supply/BAT0/present", absent + "\n");
        check(std::isnan(source.sample().battery_level), "Ignore battery with invalid/absent presence: " + absent);
    }
    fs::remove(fixture.root / "class/power_supply/BAT0/present");
    checkValue(source.sample().battery_level, 42.0f, "Presence attribute is optional");
    fixture.battery("BAT1", "80");
    check(std::isnan(source.sample().battery_level), "Multiple system batteries are ambiguous");
    fs::remove_all(fixture.root / "class/power_supply/BAT0");
    fs::remove_all(fixture.root / "class/power_supply/BAT1");
    check(std::isnan(source.sample().battery_level), "Device battery must never supply robot capacity");
}

void testDisappearingSensors() {
    SysfsFixture fixture;
    SystemTelemetrySource source(fixture.root.string());
    fixture.thermal(0, "cpu-thermal", "48500");
    fixture.battery("BAT0", "42");
    const auto present = source.sample();
    checkValue(present.cpu_temp, 48.5f, "Initial CPU reading");
    checkValue(present.battery_level, 42.0f, "Initial battery reading");

    fs::remove(fixture.root / "class/thermal/thermal_zone0/temp");
    fs::remove(fixture.root / "class/power_supply/BAT0/capacity");
    const auto missingFiles = source.sample();
    check(std::isnan(missingFiles.cpu_temp) && std::isnan(missingFiles.battery_level),
          "Removed sensor files must not retain stale readings");
    fs::remove_all(fixture.root / "class");
    const auto missingDirectories = source.sample();
    check(std::isnan(missingDirectories.cpu_temp) && std::isnan(missingDirectories.battery_level),
          "Removed sysfs directories must remain unknown");
}

void testWireRepresentation() {
    static_assert(sizeof(proto::Telemetry) == 160, "Telemetry wire size");
    static_assert(offsetof(proto::Telemetry, camera) == 152, "Camera wire offset");
    static_assert(sizeof(proto::DesiredState) == 16, "DesiredState wire size");
    check(std::isnan(proto::UNKNOWN_TELEMETRY_VALUE), "Unknown wire value must be NaN");
    auto telemetry = proto::unknownTelemetry();
    check(std::isnan(telemetry.cpu_temp) && std::isnan(telemetry.battery_level),
          "Unknown telemetry must initialize system readings");
    for (const auto& point : telemetry.points) {
        check(std::isnan(point.angle) && std::isnan(point.distance) && std::isnan(point.intensity),
              "Unknown telemetry must initialize every lidar value");
    }
    check(std::isnan(telemetry.camera.pitch) && std::isnan(telemetry.camera.yaw),
          "Unknown telemetry must initialize both camera axes");

    telemetry.cpu_temp = 48.5f;
    telemetry.battery_level = 0.0f;
    std::array<unsigned char, sizeof(proto::Telemetry)> wire{};
    std::memcpy(wire.data(), &telemetry, wire.size());
    proto::Telemetry decoded{};
    std::memcpy(&decoded, wire.data(), wire.size());
    checkValue(decoded.cpu_temp, 48.5f, "CPU survives raw wire roundtrip");
    checkValue(decoded.battery_level, 0.0f, "Real zero survives raw wire roundtrip");
    check(std::isnan(decoded.points[0].distance) && std::isnan(decoded.camera.yaw),
          "Unknown values survive raw wire roundtrip");
    telemetry.camera = {0.25f, -0.5f};
    std::memcpy(wire.data(), &telemetry, wire.size());
    float pitch = 0.0f;
    float yaw = 0.0f;
    std::memcpy(&pitch, wire.data() + 152, sizeof(pitch));
    std::memcpy(&yaw, wire.data() + 156, sizeof(yaw));
    checkValue(pitch, 0.25f, "Camera pitch occupies bytes 152..155");
    checkValue(yaw, -0.5f, "Camera yaw occupies bytes 156..159");
}

}  // namespace

int main() {
    try {
        testCpuDiscoveryAndValidation();
        testBatteryDiscoveryAndValidation();
        testDisappearingSensors();
        testWireRepresentation();
        std::cout << "System telemetry tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "System telemetry test failed: " << error.what() << '\n';
        return 1;
    }
}

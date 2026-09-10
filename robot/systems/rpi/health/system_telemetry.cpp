#include "systems/rpi/health/system_telemetry.h"

#include <limits>
#include <utility>

#if defined(__linux__)
#include <array>
#include <cerrno>
#include <charconv>
#include <filesystem>
#include <string_view>
#include <system_error>

#include <fcntl.h>
#include <unistd.h>
#endif

namespace {

float unavailable() {
    return std::numeric_limits<float>::quiet_NaN();
}

#if defined(__linux__)
namespace fs = std::filesystem;

enum class AttributeStatus { value, missing, error };

bool isSpace(char value) {
    return value == ' ' || value == '\t' || value == '\n' ||
           value == '\r' || value == '\f' || value == '\v';
}

// Sysfs attributes are short text files. Limit reads, reject truncation and
// distinguish an absent optional attribute from an unreadable one.
AttributeStatus readAttribute(const fs::path& path, std::string& value) {
    value.clear();
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
        return errno == ENOENT ? AttributeStatus::missing : AttributeStatus::error;
    }

    constexpr size_t kMaxAttributeLength = 128;
    std::array<char, kMaxAttributeLength + 1> buffer{};
    size_t length = 0;
    bool valid = true;
    while (length < buffer.size()) {
        const ssize_t count = ::read(fd, buffer.data() + length, buffer.size() - length);
        if (count > 0) {
            length += static_cast<size_t>(count);
        } else if (count == 0) {
            break;
        } else if (errno != EINTR) {
            valid = false;
            break;
        }
    }
    ::close(fd);
    if (!valid || length > kMaxAttributeLength) return AttributeStatus::error;

    size_t first = 0;
    while (first < length && isSpace(buffer[first])) ++first;
    while (length > first && isSpace(buffer[length - 1])) --length;
    if (first == length) return AttributeStatus::error;
    value.assign(buffer.data() + first, length - first);
    return AttributeStatus::value;
}

bool parseInteger(const std::string& text, int& value) {
    const char* end = text.data() + text.size();
    const auto result = std::from_chars(text.data(), end, value);
    return result.ec == std::errc{} && result.ptr == end;
}

bool isCpuZone(const std::string& type) {
    return type == "cpu-thermal" || type == "cpu_thermal" ||
           type == "bcm2835_thermal";
}

float cpuTemperature(const fs::path& root) {
    std::error_code error;
    fs::directory_iterator entry(root / "class/thermal", error);
    const fs::directory_iterator end;
    if (error) return unavailable();

    bool found = false;
    int maximum = 0;
    for (; entry != end; entry.increment(error)) {
        if (error) return unavailable();
        const fs::path& zone = entry->path();
        const std::string name = zone.filename().string();
        constexpr std::string_view prefix = "thermal_zone";
        if (name.compare(0, prefix.size(), prefix) != 0) continue;

        std::string text;
        if (readAttribute(zone / "type", text) != AttributeStatus::value ||
            !isCpuZone(text)) continue;
        int millidegrees = 0;
        if (readAttribute(zone / "temp", text) != AttributeStatus::value ||
            !parseInteger(text, millidegrees) ||
            millidegrees < -273150 || millidegrees > 250000) continue;

        if (!found || millidegrees > maximum) maximum = millidegrees;
        found = true;
    }
    if (error || !found) return unavailable();
    return static_cast<float>(maximum) / 1000.0f;
}

float batteryLevel(const fs::path& root) {
    std::error_code error;
    fs::directory_iterator entry(root / "class/power_supply", error);
    const fs::directory_iterator end;
    if (error) return unavailable();

    fs::path battery;
    for (; entry != end; entry.increment(error)) {
        if (error) return unavailable();
        const fs::path& supply = entry->path();
        std::string text;
        if (readAttribute(supply / "type", text) != AttributeStatus::value ||
            text != "Battery") continue;

        const auto scope = readAttribute(supply / "scope", text);
        if (scope == AttributeStatus::error ||
            (scope == AttributeStatus::value && text != "System" && text != "Unknown")) continue;

        const auto present = readAttribute(supply / "present", text);
        int is_present = 0;
        if (present == AttributeStatus::error ||
            (present == AttributeStatus::value &&
             (!parseInteger(text, is_present) || is_present != 1))) continue;

        // Do not select one arbitrarily or combine unlike battery capacities.
        if (!battery.empty()) return unavailable();
        battery = supply;
    }
    if (error || battery.empty()) return unavailable();

    std::string text;
    int percent = 0;
    if (readAttribute(battery / "capacity", text) != AttributeStatus::value ||
        !parseInteger(text, percent) || percent < 0 || percent > 100) {
        return unavailable();
    }
    return static_cast<float>(percent);
}
#endif

}  // namespace

SystemTelemetrySource::SystemTelemetrySource(std::string sysfs_root)
    : sysfs_root_(std::move(sysfs_root)) {}

SystemTelemetry SystemTelemetrySource::sample() const {
#if defined(__linux__)
    const fs::path root(sysfs_root_);
    return {cpuTemperature(root), batteryLevel(root)};
#else
    return {unavailable(), unavailable()};
#endif
}

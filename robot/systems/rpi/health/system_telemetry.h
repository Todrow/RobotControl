#pragma once

#include <string>

struct SystemTelemetry {
    float cpu_temp;
    float battery_level;
};

// Missing or invalid sensors produce NaN on every sample, never a stale value.
class SystemTelemetrySource {
public:
    explicit SystemTelemetrySource(std::string sysfs_root = "/sys");
    SystemTelemetry sample() const;

private:
    std::string sysfs_root_;
};

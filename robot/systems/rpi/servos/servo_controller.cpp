#include "servo_controller.h"

#include <cstdio>

#if defined(__linux__)
#include <cerrno>
#include <chrono>
#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <initializer_list>
#include <system_error>
#include <thread>

#include <fcntl.h>
#include <unistd.h>

namespace {

constexpr unsigned long long kPeriodNs = 20000000;  // 50 Hz hardware PWM.
constexpr const char* kPwmClass = "/sys/class/pwm";

bool fileError(const std::string& path, const char* operation, int error) noexcept {
    std::fprintf(stderr, "[servo] %s %s: %s (errno %d)\n",
                 operation, path.c_str(), std::strerror(error), error);
    return false;
}

// Sysfs attributes already exist. Never create files, and never leak descriptors
// into the concurrently launched camera process.
bool writeAttribute(const std::string& path, const char* value,
                    bool* committed = nullptr) noexcept {
    if (committed) *committed = false;
    const int fd = ::open(path.c_str(), O_WRONLY | O_CLOEXEC);
    if (fd < 0) return fileError(path, "open", errno);

    bool okay = true;
    const size_t size = std::strlen(value);
    size_t offset = 0;
    while (offset < size) {
        const ssize_t count = ::write(fd, value + offset, size - offset);
        if (count < 0 && errno == EINTR) continue;
        if (count <= 0) {
            const int error = count < 0 ? errno : EIO;
            fileError(path, "write", error);
            okay = false;
            break;
        }
        offset += static_cast<size_t>(count);
    }
    // An export or period write can take effect even if close subsequently fails.
    if (committed) *committed = offset == size;
    if (::close(fd) != 0) {
        fileError(path, "close", errno);
        okay = false;
    }
    return okay;
}

bool writeNumber(const std::string& path, unsigned long long value,
                 bool* committed = nullptr) noexcept {
    char text[32];
    std::snprintf(text, sizeof(text), "%llu\n", value);
    return writeAttribute(path, text, committed);
}

bool readNumber(const std::string& path, unsigned long long& value) noexcept {
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
    if (fd < 0) return fileError(path, "open", errno);

    char text[64];
    size_t size = 0;
    bool okay = true;
    while (size < sizeof(text) - 1) {
        const ssize_t count = ::read(fd, text + size, sizeof(text) - 1 - size);
        if (count < 0 && errno == EINTR) continue;
        if (count < 0) {
            fileError(path, "read", errno);
            okay = false;
            break;
        }
        if (count == 0) break;
        size += static_cast<size_t>(count);
    }
    if (::close(fd) != 0) {
        fileError(path, "close", errno);
        okay = false;
    }
    if (!okay) return false;
    if (size == sizeof(text) - 1) return fileError(path, "read number", EOVERFLOW);
    text[size] = '\0';
    char* end = nullptr;
    errno = 0;
    const unsigned long long parsed = std::strtoull(text, &end, 10);
    if (text[0] < '0' || text[0] > '9' || end == text || errno == ERANGE)
        return fileError(path, "parse number", EINVAL);
    while (*end && std::isspace(static_cast<unsigned char>(*end))) ++end;
    if (*end) return fileError(path, "parse number", EINVAL);
    value = parsed;
    return true;
}

bool isPwmChipPath(const std::string& path) {
    const std::string prefix = std::string(kPwmClass) + "/pwmchip";
    if (path.compare(0, prefix.size(), prefix) != 0 || path.size() == prefix.size())
        return false;
    for (size_t i = prefix.size(); i < path.size(); ++i)
        if (path[i] < '0' || path[i] > '9') return false;
    return true;
}

bool findPi4Chip(std::string& chip) {
    std::error_code error;
    std::filesystem::directory_iterator entry(kPwmClass, error);
    if (error) return fileError(kPwmClass, "list", error.value());
    const std::filesystem::directory_iterator end;
    for (; entry != end; entry.increment(error)) {
        if (error) return fileError(kPwmClass, "list", error.value());
        const std::string path = entry->path().string();
        if (!isPwmChipPath(path)) continue;
        std::error_code node_error;
        const auto node = std::filesystem::canonical(entry->path() / "device/of_node", node_error);
        if (node_error || node.filename() != "pwm@7e20c000") continue;
        if (!chip.empty()) {
            std::fprintf(stderr, "[servo] multiple PWM chips for pwm@7e20c000; specify --servo-pwm-chip\n");
            return false;
        }
        chip = path;
    }
    if (error) return fileError(kPwmClass, "list", error.value());
    if (chip.empty()) {
        std::fprintf(stderr,
            "[servo] Pi 4 PWM0 (pwm@7e20c000) is unavailable; enable the pwm-2chan overlay\n");
        return false;
    }
    return true;
}

bool unclaimedChannel(const std::string& path) {
    std::error_code error;
    const bool exists = std::filesystem::exists(path, error);
    if (error) return fileError(path, "inspect", error.value());
    if (exists) {
        std::fprintf(stderr, "[servo] %s is already exported; refusing to take over its output\n",
                     path.c_str());
        return false;
    }
    return true;
}

}  // namespace
#endif

ServoController::ServoController(const ServoOptions& options) : options_(options) {}

ServoController::~ServoController() {
    cleanup();
}

bool ServoController::initialize() {
    if (!options_.enabled) return true;
    if (initialized_) return true;
#if defined(__linux__)
    if (pitch_.exported || yaw_.exported) {
        std::fprintf(stderr, "[servo] previous PWM cleanup failed; cannot initialize again\n");
        return false;
    }
    if (!validServoAxis(options_.pitch) || !validServoAxis(options_.yaw) ||
        options_.pitch.channel == options_.yaw.channel) {
        std::fprintf(stderr, "[servo] invalid calibration or duplicate PWM channels\n");
        return false;
    }

    chip_path_ = options_.pwm_chip;
    if (chip_path_.empty()) {
        if (!findPi4Chip(chip_path_)) return false;
    } else if (!isPwmChipPath(chip_path_)) {
        std::fprintf(stderr, "[servo] --servo-pwm-chip must be /sys/class/pwm/pwmchipN\n");
        return false;
    }
    unsigned long long count = 0;
    if (!readNumber(chip_path_ + "/npwm", count)) return false;
    if (count != 2) {
        std::fprintf(stderr, "[servo] %s has %llu channels; expected the two-channel Pi 4 PWM0\n",
                     chip_path_.c_str(), count);
        return false;
    }
    export_path_ = chip_path_ + "/export";
    unexport_path_ = chip_path_ + "/unexport";
    auto paths = [&](Channel& channel, int number) {
        channel.number = number;
        channel.path = chip_path_ + "/pwm" + std::to_string(number);
        channel.period_path = channel.path + "/period";
        channel.duty_path = channel.path + "/duty_cycle";
        channel.polarity_path = channel.path + "/polarity";
        channel.enable_path = channel.path + "/enable";
    };
    paths(pitch_, options_.pitch.channel);
    paths(yaw_, options_.yaw.channel);
    if (!unclaimedChannel(pitch_.path) || !unclaimedChannel(yaw_.path)) return false;
    if (!prepareChannel(pitch_) || !prepareChannel(yaw_)) {
        cleanup();
        return false;
    }
    initialized_ = true;
    std::printf("[servo] %s: pitch=pwm%d yaw=pwm%d, 50 Hz; waiting for camera commands\n",
                chip_path_.c_str(), pitch_.number, yaw_.number);
    return true;
#else
    std::fprintf(stderr, "[servo] GPIO servos require Raspberry Pi OS (Linux)\n");
    return false;
#endif
}

#if defined(__linux__)
bool ServoController::prepareChannel(Channel& channel) {
    // Recheck just before export: an EBUSY from export also leaves ownership false.
    if (!unclaimedChannel(channel.path)) return false;
    if (!writeNumber(export_path_, static_cast<unsigned>(channel.number), &channel.exported))
        return false;

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    while (true) {
        bool ready = true;
        for (const std::string* path : {&channel.enable_path, &channel.period_path,
                                       &channel.duty_path, &channel.polarity_path}) {
            std::error_code error;
            const bool exists = std::filesystem::exists(*path, error);
            if (error) return fileError(*path, "inspect", error.value());
            ready = ready && exists;
        }
        if (ready) break;
        if (std::chrono::steady_clock::now() >= deadline) {
            std::fprintf(stderr, "[servo] PWM attributes did not appear under %s within 500 ms\n",
                         channel.path.c_str());
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    unsigned long long enabled = 0;
    unsigned long long period = 0;
    if (!readNumber(channel.enable_path, enabled) ||
        !readNumber(channel.period_path, period)) return false;
    if (enabled > 1) return fileError(channel.enable_path, "validate enable", EINVAL);
    channel.enabled = enabled == 1;
    channel.period_valid = period > 0;
    channel.requires_release = channel.enabled;
    if (channel.enabled) {
        if (!writeAttribute(channel.enable_path, "0\n")) return false;
        channel.enabled = false;
    }
    // A fresh PWM has period=0: set a valid period before touching its duty cycle.
    // If a period is already configured, first clear a possibly larger old duty.
    if (channel.period_valid && !writeAttribute(channel.duty_path, "0\n")) return false;
    bool period_committed = false;
    const bool period_okay = writeNumber(channel.period_path, kPeriodNs, &period_committed);
    channel.period_valid = channel.period_valid || period_committed;
    if (!period_okay || !writeAttribute(channel.polarity_path, "normal\n") ||
        !writeAttribute(channel.duty_path, "0\n")) return false;
    channel.requires_release = false;
    channel.pulse_us = -1;
    return true;
}
#endif

bool ServoController::apply(const proto::CameraState& camera) {
    if (!options_.enabled) return true;
#if defined(__linux__)
    if (!initialized_) {
        std::fprintf(stderr, "[servo] camera command received before PWM initialization\n");
        return false;
    }
    int pitch_us = 0;
    int yaw_us = 0;
    if (!servoPulseWidth(camera.pitch, options_.pitch, pitch_us) ||
        !servoPulseWidth(camera.yaw, options_.yaw, yaw_us)) {
        std::fprintf(stderr, "[servo] invalid camera position; stopping servo pulses\n");
        release();
        return false;
    }
    pitch_.requires_release = true;
    yaw_.requires_release = true;
    const auto setPulse = [](Channel& channel, int pulse_us) {
        if (channel.pulse_us == pulse_us) return true;
        if (!writeNumber(channel.duty_path, static_cast<unsigned long long>(pulse_us) * 1000))
            return false;
        channel.pulse_us = pulse_us;
        return true;
    };
    const auto enable = [](Channel& channel) {
        if (channel.enabled) return true;
        if (!writeAttribute(channel.enable_path, "1\n")) return false;
        channel.enabled = true;
        return true;
    };
    // Prepare both absolute setpoints before enabling either output on connect.
    if (!setPulse(pitch_, pitch_us) || !setPulse(yaw_, yaw_us) ||
        !enable(pitch_) || !enable(yaw_)) {
        release();
        return false;
    }
    return true;
#else
    (void)camera;
    std::fprintf(stderr, "[servo] GPIO servos require Raspberry Pi OS (Linux)\n");
    return false;
#endif
}

bool ServoController::release() noexcept {
    if (!options_.enabled) return true;
#if defined(__linux__)
    bool okay = true;
    for (Channel* channel : {&pitch_, &yaw_}) {
        channel->pulse_us = -1;  // Reconnect must reapply even the same positions.
        if (!channel->exported || !channel->period_valid || !channel->requires_release) continue;
        // Zero duty before disabling: disabled PWM output levels are driver-specific.
        const bool duty_okay = writeAttribute(channel->duty_path, "0\n");
        const bool disabled = writeAttribute(channel->enable_path, "0\n");
        if (disabled) channel->enabled = false;
        if (duty_okay && disabled) channel->requires_release = false;
        okay = duty_okay && disabled && okay;
    }
    return okay;
#else
    return true;
#endif
}

void ServoController::cleanup() noexcept {
    if (!options_.enabled) return;
    release();
#if defined(__linux__)
    for (Channel* channel : {&pitch_, &yaw_}) {
        if (!channel->exported) continue;
        bool unexported = false;
        writeNumber(unexport_path_, static_cast<unsigned>(channel->number), &unexported);
        if (unexported) {
            channel->exported = false;
            channel->period_valid = false;
            channel->enabled = false;
            channel->requires_release = false;
        }
    }
#endif
    initialized_ = false;
}

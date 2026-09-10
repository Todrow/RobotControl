// socket_utils.h must come first on Windows: it pulls winsock2.h ahead of any
// header that could reach windows.h and break the ordering.
#include "systems/rpi/link/socket_utils.h"

#include "options.h"

#include <charconv>
#include <cstdio>
#include <string>

void printUsage(const char* executable) {
    std::printf(
        "Usage: %s [options]\n"
        "  --command-port PORT    TCP command listener (default 5001)\n"
        "  --telemetry-port PORT  TCP system telemetry listener (default 5002)\n"
        "  --video-port PORT      RTP/H264 UDP destination port (default 5003)\n"
        "  --video-host IPV4      Fixed Windows receiver; default: active command peer\n"
        "  --video-source SOURCE camera | test | none (default camera)\n"
        "  --camera INDEX        Camera index from rpicam-hello (0..255; default 0)\n"
        "  --width PIXELS        Even width, 16..1920 (default 1280)\n"
        "  --height PIXELS       Even height, 16..1080 (default 720)\n"
        "  --fps FPS             Frame rate, 1..30 (default 30)\n"
        "  --bitrate BITS         H264 bitrate, 100000..25000000 (default 4000000)\n"
        "  --drive-uart          Enable Linux UART wheel commands to ESP32\n"
        "  --drive-uart-device P UART device (default /dev/serial0)\n"
        "  --drive-uart-baud N   9600|19200|38400|57600|115200|230400 (default 115200)\n"
        "  --lidar-source SOURCE real | sim | none (default real: the STL-19P lidar)\n"
        "  --lidar-period MS     Sector publish period, 10..1000 (default 100)\n"
        "  --lidar-max-age MS    Samples older than this read as Unknown (default 500)\n"
        "  --obstacle-red MM     Red below this distance, 10..10000 (default 100)\n"
        "  --obstacle-yellow MM  Yellow below this distance, 10..10000 (default 150)\n"
        "  --obstacle-no-enforce Report Red sectors only; do not block drive commands\n"
        "  --servos              Enable Linux hardware PWM camera servos\n"
        "  --servo-pwm-chip PATH  /sys/class/pwm/pwmchipN; default: auto-detect Pi 4 PWM0\n"
        "  --pitch-channel N     PWM channel 0 or 1 (default 0: BCM12, physical pin 32)\n"
        "  --yaw-channel N       PWM channel 0 or 1 (default 1: BCM13, physical pin 33)\n"
        "  --pitch-min-us US      Pitch negative endpoint (default 500)\n"
        "  --pitch-center-us US   Pitch neutral pulse (default 1500)\n"
        "  --pitch-max-us US      Pitch positive endpoint (default 2500)\n"
        "  --yaw-min-us US        Yaw negative endpoint (default 500)\n"
        "  --yaw-center-us US     Yaw neutral pulse (default 1500)\n"
        "  --yaw-max-us US        Yaw positive endpoint (default 2500)\n"
        "  --pitch-invert         Reverse pitch movement\n"
        "  --yaw-invert           Reverse yaw movement\n"
        "  --help                 Show this help\n"
        "Wheel UART requires --drive-uart: R:<percent>%%|L:<percent>%% followed by LF.\n"
        "Signed power: -100..100; LEFT/RIGHT pivot in place. UART: 8N1, no flow control.\n"
        "Camera servos require --servos and pwm-2chan setup.\n"
        "Lidar sectors: forward, forward-right, back-right, back, back-left, forward-left.\n"
        "Lidar: --lidar-source real finds the STL-19P on the serial ports by itself and\n"
        "reports 500 mm for anything further away; sim publishes placeholder distances,\n"
        "none leaves every sector Unknown. A Red sector blocks drive commands heading\n"
        "into it (STOP substituted) unless --obstacle-no-enforce is given; turning in\n"
        "place and reversing away are never blocked.\n"
        "Servo pulse limits: 500 <= min < center < max <= 2500 microseconds; 50 Hz.\n"
        "Telemetry: Linux CPU/battery sensors, applied PWM camera setpoint; NaN if unavailable.\n"
        "Camera: rpicam-vid on Raspberry Pi; ksvideosrc on Windows.\n",
        executable);
}

// Local to the parser: every numeric option shares one range check.
static bool integerOption(const std::string& name, const std::string& value, int minimum, int maximum,
                   int& result) {
    const auto parsed = std::from_chars(value.data(), value.data() + value.size(), result);
    if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() ||
        result < minimum || result > maximum) {
        std::fprintf(stderr, "%s must be an integer in %d..%d\n", name.c_str(), minimum, maximum);
        return false;
    }
    return true;
}

bool parseOptions(int argc, char* argv[], RobotOptions& options) {
    for (int i = 1; i < argc; ++i) {
        const std::string name = argv[i];
        if (name == "--drive-uart") {
            options.drive_uart.enabled = true;
            continue;
        }
        if (name == "--servos") {
            options.servos.enabled = true;
            continue;
        }
        if (name == "--obstacle-no-enforce") {
            options.lidar.enforce = false;
            continue;
        }
        if (name == "--pitch-invert" || name == "--yaw-invert") {
            (name == "--pitch-invert" ? options.servos.pitch : options.servos.yaw).inverted = true;
            continue;
        }
        if (i + 1 >= argc) {
            std::fprintf(stderr, "Missing value for %s (use --help)\n", name.c_str());
            return false;
        }
        const std::string value = argv[++i];
        int number = 0;
        if (name == "--command-port" || name == "--telemetry-port" || name == "--video-port") {
            if (!integerOption(name, value, 1, 65535, number)) return false;
            const auto port = static_cast<uint16_t>(number);
            if (name == "--command-port") options.command_port = port;
            else if (name == "--telemetry-port") options.telemetry_port = port;
            else options.video_port = port;
        } else if (name == "--video-host") {
            if (!net::isIPv4(value)) {
                std::fprintf(stderr, "--video-host must be a numeric IPv4 address\n");
                return false;
            }
            options.video_host = value;
        } else if (name == "--video-source") {
            if (value == "camera") options.video_source = VideoSource::Camera;
            else if (value == "test") options.video_source = VideoSource::Test;
            else if (value == "none") options.video_source = VideoSource::Disabled;
            else {
                std::fprintf(stderr, "--video-source must be camera, test or none\n");
                return false;
            }
        } else if (name == "--lidar-source") {
            if (value == "real") options.lidar.source = LidarSource::Device;
            else if (value == "sim") options.lidar.source = LidarSource::Simulated;
            else if (value == "none") options.lidar.source = LidarSource::Disabled;
            else {
                std::fprintf(stderr, "--lidar-source must be real, sim or none\n");
                return false;
            }
        } else if (name == "--lidar-period") {
            if (!integerOption(name, value, 10, 1000, options.lidar.period_ms)) return false;
        } else if (name == "--lidar-max-age") {
            if (!integerOption(name, value, 10, 10000, options.lidar.max_age_ms)) return false;
        } else if (name == "--obstacle-red" || name == "--obstacle-yellow") {
            if (!integerOption(name, value, 10, 10000, number)) return false;
            auto& threshold = name == "--obstacle-red" ? options.lidar.thresholds.red_mm
                                                       : options.lidar.thresholds.yellow_mm;
            threshold = static_cast<float>(number);
        } else if (name == "--camera") {
            if (!integerOption(name, value, 0, 255, options.camera_index)) return false;
        } else if (name == "--width") {
            if (!integerOption(name, value, 16, 1920, options.width)) return false;
        } else if (name == "--height") {
            if (!integerOption(name, value, 16, 1080, options.height)) return false;
        } else if (name == "--fps") {
            if (!integerOption(name, value, 1, 30, options.framerate)) return false;
        } else if (name == "--bitrate") {
            if (!integerOption(name, value, 100000, 25000000, options.bitrate)) return false;
        } else if (name == "--drive-uart-device") {
            if (value.empty()) {
                std::fprintf(stderr, "--drive-uart-device must not be empty\n");
                return false;
            }
            options.drive_uart.device = value;
        } else if (name == "--drive-uart-baud") {
            if (!integerOption(name, value, 9600, 230400, number)) return false;
            if (!validDriveBaud(number)) {
                std::fprintf(stderr, "--drive-uart-baud must be 9600, 19200, 38400, 57600, 115200 or 230400\n");
                return false;
            }
            options.drive_uart.baud = number;
        } else if (name == "--servo-pwm-chip") {
            const std::string prefix = "/sys/class/pwm/pwmchip";
            if (value.compare(0, prefix.size(), prefix) != 0 || value.size() == prefix.size() ||
                value.find_first_not_of("0123456789", prefix.size()) != std::string::npos) {
                std::fprintf(stderr, "--servo-pwm-chip must be /sys/class/pwm/pwmchipN\n");
                return false;
            }
            options.servos.pwm_chip = value;
        } else if (name == "--pitch-channel" || name == "--yaw-channel") {
            auto& axis = name == "--pitch-channel" ? options.servos.pitch : options.servos.yaw;
            if (!integerOption(name, value, 0, 1, axis.channel)) return false;
        } else if (name == "--pitch-min-us" || name == "--pitch-center-us" ||
                   name == "--pitch-max-us" || name == "--yaw-min-us" ||
                   name == "--yaw-center-us" || name == "--yaw-max-us") {
            if (!integerOption(name, value, 500, 2500, number)) return false;
            auto& axis = name.compare(0, 8, "--pitch-") == 0 ? options.servos.pitch : options.servos.yaw;
            if (name.find("-min-us") != std::string::npos) axis.min_us = number;
            else if (name.find("-center-us") != std::string::npos) axis.center_us = number;
            else axis.max_us = number;
        } else {
            std::fprintf(stderr, "Unknown option %s (use --help)\n", name.c_str());
            return false;
        }
    }
    if (options.command_port == options.telemetry_port) {
        std::fprintf(stderr, "Command and telemetry TCP ports must be different\n");
        return false;
    }
    if (options.width % 2 != 0 || options.height % 2 != 0) {
        std::fprintf(stderr, "H264 width and height must both be even\n");
        return false;
    }
    if (!validServoAxis(options.servos.pitch) || !validServoAxis(options.servos.yaw)) {
        std::fprintf(stderr, "Each servo needs 500 <= min-us < center-us < max-us <= 2500\n");
        return false;
    }
    if (!validLidarOptions(options.lidar)) {
        std::fprintf(stderr,
                     "Lidar needs 0 < --obstacle-red < --obstacle-yellow and "
                     "--lidar-max-age >= --lidar-period\n");
        return false;
    }
    if (options.servos.pitch.channel == options.servos.yaw.channel) {
        std::fprintf(stderr, "Pitch and yaw must use different PWM channels\n");
        return false;
    }
#ifdef _WIN32
    if (options.drive_uart.enabled) {
        std::fprintf(stderr, "--drive-uart is supported on the Raspberry Pi Linux server only\n");
        return false;
    }
    if (options.servos.enabled) {
        std::fprintf(stderr, "--servos is supported on the Raspberry Pi Linux server only\n");
        return false;
    }
#endif
    return true;
}

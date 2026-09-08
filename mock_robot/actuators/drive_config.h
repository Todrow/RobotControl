#pragma once

#include <string>

struct DriveOptions {
    bool enabled = false;
    std::string device = "/dev/serial0";
    int baud = 115200;
};

inline bool validDriveBaud(int baud) noexcept {
    switch (baud) {
        case 9600:
        case 19200:
        case 38400:
        case 57600:
        case 115200:
        case 230400:
            return true;
        default:
            return false;
    }
}

#pragma once

#include <string>

#include "systems/esp32/uart_protocol.h"

struct DriveOptions {
    bool enabled = false;
    std::string device = "/dev/serial0";
    int baud = esp32_uart::DEFAULT_BAUD;
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

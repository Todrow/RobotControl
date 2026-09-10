#include "drive_controller.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#if defined(__linux__)
#include <cerrno>
#include <chrono>
#include <cstring>

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace {

// Line length and percentage range come from the ESP32 contract, never from a
// second copy of the numbers here: systems/esp32/uart_protocol.h is the one file
// that both this driver and the firmware read.
constexpr size_t kFrameCapacity = static_cast<size_t>(esp32_uart::MAX_LINE_LENGTH);

size_t formatFrame(char* frame, const WheelPower& power) noexcept {
    const int count = std::snprintf(frame, kFrameCapacity, "R:%d%%|L:%d%%\n",
                                    std::clamp(power.right, esp32_uart::MIN_PERCENT,
                                               esp32_uart::MAX_PERCENT),
                                    std::clamp(power.left, esp32_uart::MIN_PERCENT,
                                               esp32_uart::MAX_PERCENT));
    return count > 0 ? static_cast<size_t>(count) : 0;
}

#if defined(__linux__)
using Clock = std::chrono::steady_clock;
constexpr auto kWriteTimeout = std::chrono::milliseconds(100);

bool uartError(const std::string& device, const char* operation, int error) noexcept {
    std::fprintf(stderr, "[drive] %s %s: %s (errno %d)\n",
                 operation, device.c_str(), std::strerror(error), error);
    return false;
}

speed_t uartBaud(int baud) noexcept {
    switch (baud) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default: return B0;
    }
}

bool writeFrame(int fd, const std::string& device, const char* frame, size_t size,
                bool& incomplete_frame, Clock::time_point deadline) noexcept {
    size_t offset = 0;
    while (offset < size) {
        // Check even after partial writes or EINTR; neither may extend the deadline.
        const auto now = Clock::now();
        if (now >= deadline) return uartError(device, "write timeout", ETIMEDOUT);

        const ssize_t count = ::write(fd, frame + offset, size - offset);
        if (count > 0) {
            incomplete_frame = true;
            offset += static_cast<size_t>(count);
            continue;
        }
        if (count == 0) return uartError(device, "write", EIO);
        const int error = errno;
        if (error == EINTR) continue;
        if (error != EAGAIN && error != EWOULDBLOCK)
            return uartError(device, "write", error);

        const auto remaining = deadline - Clock::now();
        if (remaining <= Clock::duration::zero())
            return uartError(device, "write timeout", ETIMEDOUT);
        // Round upward so a sub-millisecond remainder does not busy-spin.
        const auto wait_ms = std::chrono::ceil<std::chrono::milliseconds>(remaining).count();
        pollfd output{fd, POLLOUT, 0};
        const int ready = ::poll(&output, 1, static_cast<int>(wait_ms));
        if (ready < 0) {
            if (errno == EINTR) continue;
            return uartError(device, "poll", errno);
        }
        if (ready == 0) return uartError(device, "write timeout", ETIMEDOUT);
        if (output.revents & (POLLERR | POLLHUP | POLLNVAL))
            return uartError(device, "UART disconnected", EIO);
    }
    incomplete_frame = false;
    // Success means queued to the UART driver; this protocol has no ESP32 ACK.
    return true;
}

bool waitForOutput(int fd, const std::string& device) noexcept {
    const auto deadline = Clock::now() + kWriteTimeout;
    while (Clock::now() < deadline) {
        int pending = 0;
        if (::ioctl(fd, TIOCOUTQ, &pending) != 0) {
            if (errno == EINTR) continue;
            return uartError(device, "query output before close", errno);
        }
        if (pending == 0) return true;
        const auto remaining = deadline - Clock::now();
        if (remaining <= Clock::duration::zero()) break;
        const auto wait_ms = std::chrono::ceil<std::chrono::milliseconds>(remaining).count();
        // POLLOUT means buffer space is available, not that the queue is empty.
        // Brief sleeps avoid spinning while waiting for the UART driver to drain.
        if (::poll(nullptr, 0, static_cast<int>(wait_ms > 2 ? 2 : wait_ms)) < 0 &&
            errno != EINTR)
            return uartError(device, "wait before close", errno);
    }
    return uartError(device, "output timeout before close", ETIMEDOUT);
}
#endif

}  // namespace

WheelPower driveWheelPower(const proto::DriveCommand& command) noexcept {
    if (!std::isfinite(command.speed) || command.speed < 0.0f || command.speed > 1.0f)
        return {};
    const int percent = static_cast<int>(std::lround(command.speed * 100.0f));
    const int inner = static_cast<int>(std::lround(command.speed * 100.0f * kDiagonalInnerFactor));
    switch (command.direction) {
        case proto::Direction::FORWARD: return {percent, percent};
        case proto::Direction::BACKWARD: return {-percent, -percent};
        case proto::Direction::LEFT: return {percent, -percent};
        case proto::Direction::RIGHT: return {-percent, percent};
        // The wheel on the side of the turn is the inner one in all four cases.
        case proto::Direction::FORWARD_RIGHT: return {inner, percent};
        case proto::Direction::FORWARD_LEFT: return {percent, inner};
        case proto::Direction::BACKWARD_RIGHT: return {-inner, -percent};
        case proto::Direction::BACKWARD_LEFT: return {-percent, -inner};
        case proto::Direction::STOP: return {};
    }
    return {};
}

std::string formatWheelCommand(const WheelPower& power) {
    char frame[kFrameCapacity];
    const size_t size = formatFrame(frame, power);
    return std::string(frame, size);
}

DriveController::DriveController(const DriveOptions& options) : options_(options) {}

DriveController::~DriveController() {
    if (fd_ >= 0) {
        stop();
#if defined(__linux__)
        // Give the final STOP a bounded opportunity to leave the driver queue.
        // Unlike tcdrain(), this cannot wait indefinitely on broken hardware;
        // an empty queue still does not provide an acknowledgement from ESP32.
        waitForOutput(fd_, options_.device);
#endif
    }
    closeDevice();
}

bool DriveController::initialize() {
    if (!options_.enabled) return true;
    if (initialized_) return true;
    if (!validDriveBaud(options_.baud) || options_.device.empty() ||
        options_.device.find('\0') != std::string::npos) {
        std::fprintf(stderr, "[drive] invalid UART device or baud rate\n");
        return false;
    }
#if defined(__linux__)
    fd_ = ::open(options_.device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) return uartError(options_.device, "open", errno);

    termios settings{};
    if (::tcgetattr(fd_, &settings) != 0) {
        uartError(options_.device, "tcgetattr", errno);
        closeDevice();
        return false;
    }
    // Raw UART, 8 data bits, no parity, one stop bit, no hardware/software flow control.
    settings.c_iflag = 0;
    settings.c_oflag = 0;
    settings.c_lflag = 0;
    settings.c_cflag &= ~(CSIZE | PARENB | PARODD | CSTOPB | CRTSCTS);
#if defined(CMSPAR)
    settings.c_cflag &= ~CMSPAR;
#endif
    settings.c_cflag |= CS8 | CLOCAL | CREAD;
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    const speed_t baud = uartBaud(options_.baud);
    if (::cfsetispeed(&settings, baud) != 0 || ::cfsetospeed(&settings, baud) != 0 ||
        ::tcsetattr(fd_, TCSANOW, &settings) != 0 || ::tcflush(fd_, TCIOFLUSH) != 0) {
        uartError(options_.device, "configure raw UART", errno);
        closeDevice();
        return false;
    }
    initialized_ = true;
    if (!stop()) {
        closeDevice();
        return false;
    }
    std::printf("[drive] UART %s at %d baud, 8N1\n", options_.device.c_str(), options_.baud);
    return true;
#else
    std::fprintf(stderr, "[drive] GPIO UART is supported only on Linux\n");
    return false;
#endif
}

bool DriveController::apply(const proto::DriveCommand& command) {
    if (!options_.enabled) return true;
    if (!initialized_) {
        std::fprintf(stderr, "[drive] UART is not initialized\n");
        return false;
    }
    const WheelPower power = driveWheelPower(command);
    if (power.right == 0 && power.left == 0) return stop();
#if defined(__linux__)
    // Do not suppress duplicates: the ESP32 can use repeated commands as a heartbeat.
    char frame[kFrameCapacity];
    const size_t size = formatFrame(frame, power);
    return writeFrame(fd_, options_.device, frame, size, incomplete_frame_, Clock::now() + kWriteTimeout);
#else
    return false;
#endif
}

bool DriveController::stop() noexcept {
    if (!options_.enabled || fd_ < 0) return true;
#if defined(__linux__)
    const auto deadline = Clock::now() + kWriteTimeout;
    int pending = 0;
    const bool queue_known = ::ioctl(fd_, TIOCOUTQ, &pending) == 0;
    const bool resync = !queue_known || pending > 0 || incomplete_frame_;
    bool flushed = true;
    if (resync) {
        // Drop queued motion before STOP. A dropped or partially written frame
        // may already have reached the receiver, so terminate it with a newline.
        while (::tcflush(fd_, TCOFLUSH) != 0) {
            const int error = errno;
            if (error == EINTR && Clock::now() < deadline) continue;
            flushed = uartError(options_.device, "flush before STOP", error);
            break;
        }
    }
    constexpr char stop_frame[] = "\nR:0%|L:0%\n";
    const char* frame = stop_frame + (resync ? 0 : 1);
    const size_t size = sizeof(stop_frame) - 1 - (resync ? 0 : 1);
    const bool written = writeFrame(fd_, options_.device, frame, size, incomplete_frame_, deadline);
    return flushed && written;
#else
    return false;
#endif
}

void DriveController::closeDevice() noexcept {
#if defined(__linux__)
    if (fd_ >= 0 && ::close(fd_) != 0) uartError(options_.device, "close", errno);
#endif
    fd_ = -1;
    initialized_ = false;
    incomplete_frame_ = false;
}

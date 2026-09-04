#pragma once

#if !defined(_WIN32) && !defined(_GNU_SOURCE)
#define _GNU_SOURCE  // accept4 on Raspberry Pi OS/glibc.
#endif

// Include this before GStreamer or any other header that may include windows.h.
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#else
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <thread>
#include <utility>

namespace net {

#ifdef _WIN32
using SocketHandle = SOCKET;
constexpr SocketHandle INVALID_HANDLE = INVALID_SOCKET;
using SocketLength = int;
#else
using SocketHandle = int;
constexpr SocketHandle INVALID_HANDLE = -1;
using SocketLength = socklen_t;
#endif

using Clock = std::chrono::steady_clock;
using Deadline = Clock::time_point;
enum class IoResult { Ok, Closed, Timeout, Stopped, Error };

inline int lastError() {
#ifdef _WIN32
    return WSAGetLastError();
#else
    return errno;
#endif
}

inline bool interrupted(int error) {
#ifdef _WIN32
    return error == WSAEINTR;
#else
    return error == EINTR;
#endif
}

inline bool wouldBlock(int error) {
#ifdef _WIN32
    return error == WSAEWOULDBLOCK;
#else
    return error == EAGAIN || error == EWOULDBLOCK;
#endif
}

class Socket {
public:
    Socket() = default;
    explicit Socket(SocketHandle handle) : handle_(handle) {}
    ~Socket() { reset(); }
    Socket(const Socket&) = delete;
    Socket& operator=(const Socket&) = delete;
    Socket(Socket&& other) noexcept : handle_(other.release()) {}
    Socket& operator=(Socket&& other) noexcept {
        if (this != &other) reset(other.release());
        return *this;
    }
    SocketHandle get() const { return handle_; }
    explicit operator bool() const { return handle_ != INVALID_HANDLE; }

    void reset(SocketHandle handle = INVALID_HANDLE) {
        if (handle_ != INVALID_HANDLE) {
            const int saved_error = lastError();
#ifdef _WIN32
            closesocket(handle_);
            WSASetLastError(saved_error);
#else
            // Do not retry close() after EINTR: the descriptor may be reused.
            ::close(handle_);
            errno = saved_error;
#endif
        }
        handle_ = handle;
    }

private:
    SocketHandle release() { return std::exchange(handle_, INVALID_HANDLE); }
    SocketHandle handle_ = INVALID_HANDLE;
};

inline bool makeNonBlocking(SocketHandle socket) {
#ifdef _WIN32
    u_long mode = 1;
    return ioctlsocket(socket, FIONBIO, &mode) == 0 &&
           SetHandleInformation(reinterpret_cast<HANDLE>(socket), HANDLE_FLAG_INHERIT, 0) != 0;
#else
    const int flags = fcntl(socket, F_GETFL, 0);
    const int descriptor_flags = fcntl(socket, F_GETFD, 0);
    return flags >= 0 && descriptor_flags >= 0 &&
           fcntl(socket, F_SETFL, flags | O_NONBLOCK) == 0 &&
           fcntl(socket, F_SETFD, descriptor_flags | FD_CLOEXEC) == 0;
#endif
}

inline Socket listenOn(uint16_t port) {
#ifdef _WIN32
    Socket server(::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP));
#else
    // Atomic close-on-exec is required while the camera thread starts rpicam-vid.
    Socket server(::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC | SOCK_NONBLOCK, IPPROTO_TCP));
#endif
    if (!server || !makeNonBlocking(server.get())) return {};
    const int reuse = 1;
    if (setsockopt(server.get(), SOL_SOCKET, SO_REUSEADDR,
                   reinterpret_cast<const char*>(&reuse), sizeof(reuse)) != 0) return {};

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(port);
    if (::bind(server.get(), reinterpret_cast<const sockaddr*>(&address), sizeof(address)) != 0 ||
        ::listen(server.get(), 1) != 0) return {};
    return server;
}

inline IoResult waitReady(SocketHandle socket, bool writing, Deadline deadline,
                          const std::atomic<bool>& running) {
    while (running.load()) {
        const auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now());
        if (remaining.count() <= 0) return IoResult::Timeout;
        const int timeout_ms = static_cast<int>(std::min<int64_t>(remaining.count(), 100));
#ifdef _WIN32
        fd_set ready;
        FD_ZERO(&ready);
        FD_SET(socket, &ready);
        timeval timeout{};
        timeout.tv_usec = timeout_ms * 1000;
        const int result = ::select(0, writing ? nullptr : &ready, writing ? &ready : nullptr,
                                    nullptr, &timeout);
#else
        pollfd descriptor{};
        descriptor.fd = socket;
        descriptor.events = writing ? POLLOUT : POLLIN;
        const int result = ::poll(&descriptor, 1, timeout_ms);
        if (result > 0 && (descriptor.revents & POLLNVAL)) {
            errno = EBADF;
            return IoResult::Error;
        }
#endif
        if (result > 0) return running.load() ? IoResult::Ok : IoResult::Stopped;
        if (result < 0 && !interrupted(lastError())) return IoResult::Error;
    }
    return IoResult::Stopped;
}

inline IoResult acceptClient(SocketHandle server, Socket& client, std::string& peer,
                             Deadline deadline, const std::atomic<bool>& running) {
    while (running.load()) {
        const IoResult ready = waitReady(server, false, deadline, running);
        if (ready != IoResult::Ok) return ready;
        sockaddr_in address{};
        SocketLength length = sizeof(address);
#ifdef _WIN32
        Socket accepted(::accept(server, reinterpret_cast<sockaddr*>(&address), &length));
#else
        Socket accepted(::accept4(server, reinterpret_cast<sockaddr*>(&address), &length,
                                  SOCK_CLOEXEC | SOCK_NONBLOCK));
#endif
        if (!accepted) {
            const int error = lastError();
            if (interrupted(error) || wouldBlock(error)) continue;
#ifdef _WIN32
            if (error == WSAECONNRESET || error == WSAECONNABORTED) continue;
#else
            // Linux can report pending network errors from the new peer here.
            // They do not mean that the listening socket is broken.
            if (error == ECONNABORTED || error == EPROTO || error == ECONNRESET ||
                error == ENETDOWN || error == ENOPROTOOPT || error == EHOSTDOWN ||
                error == ENONET || error == EHOSTUNREACH || error == EOPNOTSUPP ||
                error == ENETUNREACH) continue;
#endif
            return IoResult::Error;
        }
        if (!makeNonBlocking(accepted.get())) return IoResult::Error;
        char host[INET_ADDRSTRLEN]{};
        if (!inet_ntop(AF_INET, &address.sin_addr, host, sizeof(host))) return IoResult::Error;
        peer = host;
        client = std::move(accepted);
        return IoResult::Ok;
    }
    return IoResult::Stopped;
}

inline IoResult recvExact(SocketHandle socket, void* data, size_t size, Deadline deadline,
                          const std::atomic<bool>& running) {
    auto* bytes = static_cast<char*>(data);
    size_t received = 0;
    while (received < size) {
        const IoResult ready = waitReady(socket, false, deadline, running);
        if (ready != IoResult::Ok) return ready;
        const int chunk = static_cast<int>(std::min<size_t>(size - received, std::numeric_limits<int>::max()));
        const auto count = ::recv(socket, bytes + received, chunk, 0);
        if (count == 0) return IoResult::Closed;
        if (count < 0) {
            const int error = lastError();
            if (interrupted(error) || wouldBlock(error)) continue;
            return IoResult::Error;
        }
        received += static_cast<size_t>(count);
    }
    return IoResult::Ok;
}

inline IoResult sendExact(SocketHandle socket, const void* data, size_t size, Deadline deadline,
                          const std::atomic<bool>& running) {
    const auto* bytes = static_cast<const char*>(data);
    size_t sent = 0;
    while (sent < size) {
        const IoResult ready = waitReady(socket, true, deadline, running);
        if (ready != IoResult::Ok) return ready;
        const int chunk = static_cast<int>(std::min<size_t>(size - sent, std::numeric_limits<int>::max()));
#ifdef _WIN32
        const auto count = ::send(socket, bytes + sent, chunk, 0);
#else
        const auto count = ::send(socket, bytes + sent, chunk, MSG_NOSIGNAL);
#endif
        if (count == 0) return IoResult::Closed;
        if (count < 0) {
            const int error = lastError();
            if (interrupted(error) || wouldBlock(error)) continue;
            return IoResult::Error;
        }
        sent += static_cast<size_t>(count);
    }
    return IoResult::Ok;
}

inline void sleepUntil(Deadline deadline, const std::atomic<bool>& running) {
    while (running.load()) {
        const auto now = Clock::now();
        if (now >= deadline) return;
        std::this_thread::sleep_until(std::min(deadline, now + std::chrono::milliseconds(50)));
    }
}

inline bool isIPv4(const std::string& host) {
    in_addr address{};
    return inet_pton(AF_INET, host.c_str(), &address) == 1;
}

}  // namespace net

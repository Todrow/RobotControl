#include "tcp_connection.h"

#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>

static_assert(sizeof(SOCKET) == sizeof(uintptr_t));
static constexpr uintptr_t kInvalid = ~uintptr_t(0);

TcpConnection::~TcpConnection() { close(); }

bool TcpConnection::connectTo(const std::string& host, uint16_t port, int connect_timeout_ms) {
    close();

    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    addrinfo* res = nullptr;
    if (getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res) != 0) return false;

    SOCKET s = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (s == INVALID_SOCKET) {
        freeaddrinfo(res);
        return false;
    }

    // Non-blocking connect + select, so a dead peer fails fast instead of
    // stalling the reconnect loop on the OS default timeout.
    u_long nb = 1;
    ioctlsocket(s, FIONBIO, &nb);
    int rc = ::connect(s, res->ai_addr, static_cast<int>(res->ai_addrlen));
    freeaddrinfo(res);

    bool ok = (rc == 0);
    if (!ok && WSAGetLastError() == WSAEWOULDBLOCK) {
        fd_set w, e;
        FD_ZERO(&w); FD_SET(s, &w);
        FD_ZERO(&e); FD_SET(s, &e);
        timeval tv{connect_timeout_ms / 1000, (connect_timeout_ms % 1000) * 1000};
        ok = select(0, nullptr, &w, &e, &tv) > 0 && FD_ISSET(s, &w);
    }
    if (!ok) {
        closesocket(s);
        return false;
    }

    nb = 0;
    ioctlsocket(s, FIONBIO, &nb);
    DWORD to = kIoTimeoutMs;
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&to), sizeof(to));
    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&to), sizeof(to));
    BOOL nodelay = TRUE;
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, reinterpret_cast<const char*>(&nodelay), sizeof(nodelay));

    std::lock_guard<std::mutex> lock(mtx_);
    sock_ = s;
    connected_.store(true);
    return true;
}

void TcpConnection::closeLocked() {
    if (sock_ != kInvalid) {
        closesocket(static_cast<SOCKET>(sock_));
        sock_ = kInvalid;
    }
    connected_.store(false);
}

void TcpConnection::close() {
    std::lock_guard<std::mutex> lock(mtx_);
    closeLocked();
}

TcpConnection::Result TcpConnection::sendAll(const void* data, size_t len) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (sock_ == kInvalid) return Result::Timeout;

    const char* p = static_cast<const char*>(data);
    size_t sent = 0;
    while (sent < len) {
        int n = ::send(static_cast<SOCKET>(sock_), p + sent, static_cast<int>(len - sent), 0);
        if (n > 0) {
            sent += static_cast<size_t>(n);
            continue;
        }
        if (WSAGetLastError() == WSAETIMEDOUT) {
            if (sent == 0) return Result::Timeout;
            continue;  // partially sent: the peer is just slow, finish the struct
        }
        closeLocked();
        return Result::Error;
    }
    return Result::Ok;
}

TcpConnection::Result TcpConnection::recvAll(void* data, size_t len) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (sock_ == kInvalid) return Result::Timeout;

    char* p = static_cast<char*>(data);
    size_t got = 0;
    while (got < len) {
        int n = ::recv(static_cast<SOCKET>(sock_), p + got, static_cast<int>(len - got), 0);
        if (n > 0) {
            got += static_cast<size_t>(n);
            continue;
        }
        if (n == 0) {  // orderly shutdown by peer
            closeLocked();
            return Result::Error;
        }
        if (WSAGetLastError() == WSAETIMEDOUT) {
            if (got == 0) return Result::Timeout;
            continue;  // mid-struct: the rest is still on its way
        }
        closeLocked();
        return Result::Error;
    }
    return Result::Ok;
}

#include "tcp_connection.h"

#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace {
constexpr int kInvalid = -1;

// recv()/send() with SO_RCVTIMEO/SO_SNDTIMEO report a hit deadline as
// EAGAIN/EWOULDBLOCK; treat both spellings as the same timeout.
bool isWouldBlock(int e) { return e == EAGAIN || e == EWOULDBLOCK; }
}  // namespace

TcpConnection::~TcpConnection() { close(); }

bool TcpConnection::connectTo(const std::string& host, uint16_t port, int connect_timeout_ms) {
    close();

    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    addrinfo* res = nullptr;
    if (getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res) != 0) return false;

    int s = socket(res->ai_family, res->ai_socktype, res->ai_protocol);
    if (s < 0) {
        freeaddrinfo(res);
        return false;
    }

    // Non-blocking connect + select, so a dead peer fails fast instead of
    // stalling the reconnect loop on the OS default timeout.
    const int flags = fcntl(s, F_GETFL, 0);
    fcntl(s, F_SETFL, flags | O_NONBLOCK);
    int rc = ::connect(s, res->ai_addr, res->ai_addrlen);
    freeaddrinfo(res);

    bool ok = (rc == 0);
    if (!ok && errno == EINPROGRESS) {
        fd_set w, e;
        FD_ZERO(&w); FD_SET(s, &w);
        FD_ZERO(&e); FD_SET(s, &e);
        timeval tv{};
        tv.tv_sec = connect_timeout_ms / 1000;
        tv.tv_usec = (connect_timeout_ms % 1000) * 1000;
        int sel;
        do {
            sel = select(s + 1, nullptr, &w, &e, &tv);
        } while (sel < 0 && errno == EINTR);
        if (sel > 0 && FD_ISSET(s, &w)) {
            // A writable non-blocking socket may still hold a connect error.
            int soerr = 0;
            socklen_t len = sizeof(soerr);
            ok = getsockopt(s, SOL_SOCKET, SO_ERROR, &soerr, &len) == 0 && soerr == 0;
        }
    }
    if (!ok) {
        ::close(s);
        return false;
    }

    fcntl(s, F_SETFL, flags);  // back to blocking
    timeval to{};
    to.tv_sec = kIoTimeoutMs / 1000;
    to.tv_usec = (kIoTimeoutMs % 1000) * 1000;
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &to, sizeof(to));
    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &to, sizeof(to));
    int nodelay = 1;
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

    std::lock_guard<std::mutex> lock(mtx_);
    sock_ = s;
    connected_.store(true);
    return true;
}

void TcpConnection::closeLocked() {
    if (sock_ != kInvalid) {
        ::close(sock_);
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
        // MSG_NOSIGNAL: a dropped peer must return an error here, not raise
        // SIGPIPE and kill the process.
        ssize_t n = ::send(sock_, p + sent, len - sent, MSG_NOSIGNAL);
        if (n > 0) {
            sent += static_cast<size_t>(n);
            continue;
        }
        if (n < 0 && errno == EINTR) continue;
        if (n < 0 && isWouldBlock(errno)) {
            // A partial frame cannot be resumed by the next sendAll call.
            // Reconnect to restore the stream's frame boundary.
            if (sent != 0) closeLocked();
            return Result::Timeout;
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
        ssize_t n = ::recv(sock_, p + got, len - got, 0);
        if (n > 0) {
            got += static_cast<size_t>(n);
            continue;
        }
        if (n == 0) {  // orderly shutdown by peer
            closeLocked();
            return Result::Error;
        }
        if (errno == EINTR) continue;
        if (isWouldBlock(errno)) {
            // The caller discards this partial frame on timeout.
            // Reconnect to restore the stream's frame boundary.
            if (got != 0) closeLocked();
            return Result::Timeout;
        }
        closeLocked();
        return Result::Error;
    }
    return Result::Ok;
}

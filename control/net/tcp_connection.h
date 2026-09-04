#pragma once
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

// Thin Winsock2 client socket. All calls are mutex-guarded so the reconnect
// thread may close/reopen the socket while a worker thread is in send/recv.
// Socket timeouts (kIoTimeoutMs) keep that mutex from being held for long.
class TcpConnection {
public:
    enum class Result { Ok, Timeout, Error };

    TcpConnection() = default;
    ~TcpConnection();
    TcpConnection(const TcpConnection&) = delete;
    TcpConnection& operator=(const TcpConnection&) = delete;

    bool connectTo(const std::string& host, uint16_t port, int connect_timeout_ms = 1000);
    void close();
    bool connected() const { return connected_.load(); }

    // Both close the socket themselves on Result::Error, so the failing worker
    // never races the reconnect thread by closing a freshly opened socket.
    // A socket that is already down reports Timeout, not Error.
    Result sendAll(const void* data, size_t len);
    // Blocks until exactly len bytes are read. Timeout is only reported when
    // nothing at all arrived; a partially read struct keeps waiting.
    Result recvAll(void* data, size_t len);

private:
    static constexpr int kIoTimeoutMs = 200;

    void closeLocked();  // mtx_ must be held

    mutable std::mutex mtx_;
    uintptr_t sock_ = ~uintptr_t(0);  // INVALID_SOCKET
    std::atomic<bool> connected_{false};
};

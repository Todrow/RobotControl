#include "telemetry_receiver.h"

#include <chrono>

#include "tcp_connection.h"

TelemetryReceiver::TelemetryReceiver(TcpConnection& conn, QObject* parent)
    : QObject(parent), conn_(conn) {
    qRegisterMetaType<proto::Telemetry>();
}

TelemetryReceiver::~TelemetryReceiver() { stop(); }

void TelemetryReceiver::start() {
    if (running_.exchange(true)) return;
    thread_ = std::thread(&TelemetryReceiver::loop, this);
}

void TelemetryReceiver::stop() {
    running_.store(false);
    if (thread_.joinable()) thread_.join();
}

bool TelemetryReceiver::pop(proto::Telemetry& out) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (queue_.empty()) return false;
    out = queue_.front();
    queue_.pop_front();
    return true;
}

void TelemetryReceiver::loop() {
    while (running_.load()) {
        if (!conn_.connected()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(proto::TELEMETRY_PERIOD_MS));
            continue;
        }

        proto::Telemetry t{};
        switch (conn_.recvAll(&t, sizeof(t))) {
            case TcpConnection::Result::Ok: {
                {
                    std::lock_guard<std::mutex> lock(mtx_);
                    if (queue_.size() >= kMaxQueue) queue_.pop_front();
                    queue_.push_back(t);
                }
                emit telemetryReceived(t);  // connected as QueuedConnection
                break;
            }
            case TcpConnection::Result::Timeout:
            case TcpConnection::Result::Error:
                break;  // the connection closed itself; the manager reconnects
        }
    }
}

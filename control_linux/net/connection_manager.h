#pragma once
#include <QObject>
#include <QString>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>

#include "command_sender.h"
#include "tcp_connection.h"
#include "telemetry_receiver.h"

class DesiredStateSlot;

// Owns both TCP links and keeps them alive: a background loop retries whichever
// socket is down every kRetryMs and reports status changes as Qt signals.
class ConnectionManager : public QObject {
    Q_OBJECT

public:
    explicit ConnectionManager(DesiredStateSlot& slot, QObject* parent = nullptr);
    ~ConnectionManager() override;

    void start(const QString& host, uint16_t command_port, uint16_t telemetry_port);
    void stop();
    bool running() const { return running_.load(); }

    TelemetryReceiver* telemetry() { return &telemetry_rx_; }

signals:
    void commandStatusChanged(bool connected);
    void telemetryStatusChanged(bool connected);

private:
    static constexpr int kRetryMs = 1500;
    static constexpr int kConnectTimeoutMs = 700;

    void loop();

    TcpConnection command_conn_;
    TcpConnection telemetry_conn_;
    CommandSender sender_;
    TelemetryReceiver telemetry_rx_;

    std::mutex mtx_;
    std::string host_;
    uint16_t command_port_ = 0;
    uint16_t telemetry_port_ = 0;

    std::atomic<bool> running_{false};
    std::thread thread_;
};

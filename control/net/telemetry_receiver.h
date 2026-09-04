#pragma once
#include <QMetaType>
#include <QObject>
#include <atomic>
#include <deque>
#include <mutex>
#include <thread>

#include "../../common/protocol.h"

class TcpConnection;

// RX thread: reads whole Telemetry structs, keeps the last kMaxQueue of them in
// a FIFO (oldest dropped on overflow) and signals each one to the GUI thread.
class TelemetryReceiver : public QObject {
    Q_OBJECT

public:
    static constexpr size_t kMaxQueue = 100;

    explicit TelemetryReceiver(TcpConnection& conn, QObject* parent = nullptr);
    ~TelemetryReceiver() override;

    void start();
    void stop();

    bool pop(proto::Telemetry& out);  // FIFO drain for non-signal consumers

signals:
    void telemetryReceived(proto::Telemetry telemetry);

private:
    void loop();

    TcpConnection& conn_;
    std::mutex mtx_;
    std::deque<proto::Telemetry> queue_;
    std::atomic<bool> running_{false};
    std::thread thread_;
};

Q_DECLARE_METATYPE(proto::Telemetry)

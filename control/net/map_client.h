#pragma once
#include <QMetaType>
#include <QObject>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "../../common/map_protocol.h"
#include "tcp_connection.h"

// One decoded map, ready to paint. Cells are row-major, `size` per side, with
// the grid origin at its centre -- the same layout the robot sends.
struct MapFrame {
    std::vector<uint8_t> cells;
    int size = 0;
    float resolution = 0.05f;
    float pose_x = 0.0f;
    float pose_y = 0.0f;
    float pose_theta = 0.0f;
    float match_score = 0.0f;
    bool exploring = false;
};

// The autonomy link: receives maps, sends the explore request.
//
// Its own connection on its own port, deliberately separate from the command
// and telemetry links: if this one drops, or the robot was built without it,
// teleoperation carries on untouched. Reconnects on its own like
// ConnectionManager does, so the operator can start the robot and the laptop in
// either order.
class MapClient : public QObject {
    Q_OBJECT

public:
    explicit MapClient(QObject* parent = nullptr);
    ~MapClient() override;

    void start(const QString& host, uint16_t port);
    void stop();

    // Ask the robot to start or stop exploring. Queued and sent by the worker
    // thread; a request made while disconnected is sent on reconnect, so the
    // button always means what it says.
    void requestExplore(bool enabled);

signals:
    void mapReceived(MapFrame map);
    void statusChanged(bool connected);

private:
    static constexpr int kRetryMs = 1500;
    static constexpr int kConnectTimeoutMs = 700;

    void loop();
    bool readFrame();
    void sendPendingRequest();

    TcpConnection conn_;

    std::mutex mtx_;
    std::string host_;
    uint16_t port_ = 0;
    bool want_explore_ = false;
    bool request_pending_ = false;

    std::atomic<bool> running_{false};
    std::atomic<bool> connected_{false};
    std::thread thread_;
};

Q_DECLARE_METATYPE(MapFrame)

#include "map_client.h"

#include <chrono>
#include <cstring>

MapClient::MapClient(QObject* parent) : QObject(parent) { qRegisterMetaType<MapFrame>(); }

MapClient::~MapClient() { stop(); }

void MapClient::start(const QString& host, uint16_t port) {
    {
        std::lock_guard<std::mutex> lock(mtx_);
        host_ = host.toStdString();
        port_ = port;
    }
    if (running_.exchange(true)) return;
    thread_ = std::thread(&MapClient::loop, this);
}

void MapClient::stop() {
    running_.store(false);
    if (thread_.joinable()) thread_.join();
    conn_.close();
    connected_.store(false);
}

void MapClient::requestExplore(bool enabled) {
    std::lock_guard<std::mutex> lock(mtx_);
    want_explore_ = enabled;
    request_pending_ = true;
}

void MapClient::sendPendingRequest() {
    bool enabled = false;
    {
        std::lock_guard<std::mutex> lock(mtx_);
        if (!request_pending_) return;
        enabled = want_explore_;
        request_pending_ = false;
    }
    mapproto::FrameHeader header{mapproto::MSG_MODE, 1};
    uint8_t body = enabled ? 1 : 0;
    uint8_t bytes[sizeof(header) + 1];
    std::memcpy(bytes, &header, sizeof(header));
    bytes[sizeof(header)] = body;
    if (conn_.sendAll(bytes, sizeof(bytes)) != TcpConnection::Result::Ok) {
        // The socket closed itself; the reconnect below picks it up and the
        // request is re-armed so the button press is not silently lost.
        std::lock_guard<std::mutex> lock(mtx_);
        request_pending_ = true;
    }
}

bool MapClient::readFrame() {
    mapproto::FrameHeader header{};
    const auto got = conn_.recvAll(&header, sizeof(header));
    if (got == TcpConnection::Result::Timeout) return true;  // nothing yet
    if (got != TcpConnection::Result::Ok) return false;

    if (header.type != mapproto::MSG_MAP || header.length < sizeof(mapproto::MapHeader))
        return false;

    mapproto::MapHeader map{};
    if (conn_.recvAll(&map, sizeof(map)) != TcpConnection::Result::Ok) return false;

    // Sanity before allocating: a corrupt length must not turn into a huge
    // allocation, and a grid that is not the size we expect cannot be painted.
    const size_t expected_cells = static_cast<size_t>(map.cells) * map.cells;
    const size_t declared = static_cast<size_t>(map.runs) * sizeof(mapproto::CellRun);
    if (map.cells == 0 || map.cells > 2048 || map.runs == 0 || map.runs > expected_cells ||
        declared != header.length - sizeof(map))
        return false;

    std::vector<mapproto::CellRun> runs(map.runs);
    if (conn_.recvAll(runs.data(), declared) != TcpConnection::Result::Ok) return false;

    MapFrame view;
    if (!mapproto::decodeRuns(runs, expected_cells, view.cells)) return false;
    view.size = map.cells;
    view.resolution = map.resolution;
    view.pose_x = map.pose_x;
    view.pose_y = map.pose_y;
    view.pose_theta = map.pose_theta;
    view.match_score = map.match_score;
    view.exploring = map.exploring != 0;
    emit mapReceived(view);  // connected as QueuedConnection
    return true;
}

void MapClient::loop() {
    auto next_attempt = std::chrono::steady_clock::now();
    while (running_.load()) {
        if (!conn_.connected()) {
            if (connected_.exchange(false)) emit statusChanged(false);
            if (std::chrono::steady_clock::now() < next_attempt) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }
            next_attempt = std::chrono::steady_clock::now() + std::chrono::milliseconds(kRetryMs);

            std::string host;
            uint16_t port = 0;
            {
                std::lock_guard<std::mutex> lock(mtx_);
                host = host_;
                port = port_;
            }
            if (host.empty() || port == 0) continue;
            if (conn_.connectTo(host, port, kConnectTimeoutMs)) {
                connected_.store(true);
                emit statusChanged(true);
                // Re-assert whatever the operator last asked for: the robot may
                // have restarted, and the button on screen has to stay true.
                std::lock_guard<std::mutex> lock(mtx_);
                request_pending_ = true;
            }
            continue;
        }

        sendPendingRequest();
        if (!readFrame()) conn_.close();
    }
}

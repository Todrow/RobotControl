#include "connection_manager.h"

#include <algorithm>
#include <chrono>

#include "../state/desired_state_slot.h"

ConnectionManager::ConnectionManager(DesiredStateSlot& slot, QObject* parent)
    : QObject(parent), sender_(command_conn_, slot), telemetry_rx_(telemetry_conn_, this) {}

ConnectionManager::~ConnectionManager() { stop(); }

void ConnectionManager::start(const QString& host, uint16_t command_port, uint16_t telemetry_port) {
    stop();
    {
        std::lock_guard<std::mutex> lock(mtx_);
        host_ = host.toStdString();
        command_port_ = command_port;
        telemetry_port_ = telemetry_port;
    }
    running_.store(true);
    sender_.start();
    telemetry_rx_.start();
    thread_ = std::thread(&ConnectionManager::loop, this);
}

void ConnectionManager::stop() {
    const bool was_running = running_.exchange(false);
    if (thread_.joinable()) thread_.join();
    sender_.stop();
    telemetry_rx_.stop();
    command_conn_.close();
    telemetry_conn_.close();
    if (was_running) {
        emit commandStatusChanged(false);
        emit telemetryStatusChanged(false);
    }
}

void ConnectionManager::loop() {
    bool command_up = false;
    bool telemetry_up = false;
    auto next_try = std::chrono::steady_clock::now();

    while (running_.load()) {
        const auto attempt_start = std::chrono::steady_clock::now();
        if (attempt_start >= next_try) {
            std::string host;
            uint16_t cp, tp;
            {
                std::lock_guard<std::mutex> lock(mtx_);
                host = host_;
                cp = command_port_;
                tp = telemetry_port_;
            }
            if (!command_conn_.connected()) command_conn_.connectTo(host, cp, kConnectTimeoutMs);
            if (running_.load() && !telemetry_conn_.connected())
                telemetry_conn_.connectTo(host, tp, kConnectTimeoutMs);
            // Cadence counted from the start of the attempt, so a slow refusal
            // does not stretch the retry interval; never busy-retry either.
            next_try = std::max(attempt_start + std::chrono::milliseconds(kRetryMs),
                                std::chrono::steady_clock::now() + std::chrono::milliseconds(250));
        }

        if (command_conn_.connected() != command_up) {
            command_up = !command_up;
            emit commandStatusChanged(command_up);
        }
        if (telemetry_conn_.connected() != telemetry_up) {
            telemetry_up = !telemetry_up;
            emit telemetryStatusChanged(telemetry_up);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

#include "command_sender.h"

#include <chrono>

#include "../../common/protocol.h"
#include "../state/desired_state_slot.h"
#include "tcp_connection.h"

CommandSender::CommandSender(TcpConnection& conn, DesiredStateSlot& slot)
    : conn_(conn), slot_(slot) {}

CommandSender::~CommandSender() { stop(); }

void CommandSender::start() {
    if (running_.exchange(true)) return;
    thread_ = std::thread(&CommandSender::loop, this);
}

void CommandSender::stop() {
    running_.store(false);
    if (thread_.joinable()) thread_.join();
}

void CommandSender::loop() {
    using clock = std::chrono::steady_clock;
    const auto period = std::chrono::milliseconds(proto::COMMAND_PERIOD_MS);
    auto next = clock::now();

    while (running_.load()) {
        if (conn_.connected()) {
            const proto::DesiredState st = slot_.get();
            // On error the connection closes itself; the manager reconnects.
            conn_.sendAll(&st, sizeof(st));
        }
        next += period;
        // A long stall (socket timeout, debugger break) must not turn into a
        // burst of catch-up sends.
        const auto now = clock::now();
        if (next < now) next = now + period;
        std::this_thread::sleep_until(next);
    }
}

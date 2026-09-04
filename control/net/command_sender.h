#pragma once
#include <atomic>
#include <thread>

class DesiredStateSlot;
class TcpConnection;

// TX thread: every COMMAND_PERIOD_MS it snapshots the slot and sends one
// DesiredState, unconditionally. Runs for the whole session; while the socket
// is down it idles and lets ConnectionManager do the reconnecting.
class CommandSender {
public:
    CommandSender(TcpConnection& conn, DesiredStateSlot& slot);
    ~CommandSender();

    void start();
    void stop();

private:
    void loop();

    TcpConnection& conn_;
    DesiredStateSlot& slot_;
    std::atomic<bool> running_{false};
    std::thread thread_;
};

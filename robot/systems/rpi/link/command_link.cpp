#include "systems/rpi/link/command_link.h"

#include <cstdio>

bool CommandLink::open() {
    server_ = net::listenOn(port_);
    if (!server_) {
        std::fprintf(stderr, "[cmd] cannot listen on %u (socket error %d)\n",
                     static_cast<unsigned>(port_), net::lastError());
        return false;
    }
    return true;
}

net::IoResult CommandLink::waitForClient(net::Deadline deadline, const std::atomic<bool>& running) {
    net::Socket client;
    std::string peer;
    const auto accepted = net::acceptClient(server_.get(), client, peer, deadline, running);
    if (accepted == net::IoResult::Ok) {
        client_ = std::move(client);
        peer_ = std::move(peer);
    }
    return accepted;
}

net::IoResult CommandLink::receive(proto::DesiredState& frame, net::Deadline deadline,
                                   const std::atomic<bool>& running) {
    return net::recvExact(client_.get(), &frame, sizeof(frame), deadline, running);
}

void CommandLink::disconnect() noexcept {
    client_ = net::Socket();
    peer_.clear();
}

#pragma once
// The command channel to the operator's laptop, as a device: it accepts one
// connection at a time and moves fixed-size frames across it.
//
// Deliberately stupid. It does not know what a DesiredState means, never looks
// at a field, and never touches the drive -- it reports Ok, Timeout, Closed,
// Stopped or Error and lets the control loop decide what any of that means.
// Everything that used to sit between the socket and the motors now lives
// upstream in supervisor/ and control/, which is what makes a second command
// source (autonomy) possible without a second copy of the safety rules.

#include <atomic>
#include <cstdint>
#include <string>

#include "protocol.h"
#include "systems/rpi/link/socket_utils.h"

class CommandLink {
public:
    explicit CommandLink(uint16_t port) : port_(port) {}

    // Starts listening. Prints its own diagnostic and returns false on failure.
    bool open();

    // Waits for a controller to connect, giving up at `deadline` so the caller
    // can service other work between polls. Ok means connected(); Timeout means
    // nothing arrived yet.
    net::IoResult waitForClient(net::Deadline deadline, const std::atomic<bool>& running);

    // Reads exactly one frame from the connected controller.
    net::IoResult receive(proto::DesiredState& frame, net::Deadline deadline,
                          const std::atomic<bool>& running);

    bool connected() const noexcept { return static_cast<bool>(client_); }
    const std::string& peer() const noexcept { return peer_; }

    void disconnect() noexcept;

private:
    const uint16_t port_;
    net::Socket server_;
    net::Socket client_;
    std::string peer_;
};

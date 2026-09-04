#pragma once
#include <mutex>

#include "../../common/protocol.h"

// Single-slot mailbox: the input layer overwrites it, the TX thread reads the
// latest value every COMMAND_PERIOD_MS. Last write wins, nothing is queued.
class DesiredStateSlot {
public:
    DesiredStateSlot();

    void setDrive(proto::Direction direction, float speed);
    void setCamera(float pitch, float yaw);
    proto::DesiredState get() const;

private:
    mutable std::mutex mtx_;
    proto::DesiredState state_{};
};

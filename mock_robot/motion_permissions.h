#pragma once

#include <atomic>

// Thread-safe "may I drive this way?" flags, shared between the LiDAR guard
// thread (writer) and the command thread (reader). Four directions are kept so
// the design already fits a future 4-way robot; today only forward/backward are
// updated by the LiDAR (left/right stay allowed).
//
// Each flag is an independent atomic<bool>, so no lock is needed: the guard
// thread writes, the command thread reads, and a torn read is impossible for a
// bool. `true` means motion in that direction is permitted.
struct MotionPermissions {
    std::atomic<bool> forward{true};
    std::atomic<bool> backward{true};
    std::atomic<bool> left{true};
    std::atomic<bool> right{true};

    void allowAll() noexcept {
        forward.store(true);
        backward.store(true);
        left.store(true);
        right.store(true);
    }
};

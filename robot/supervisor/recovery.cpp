#include "supervisor/recovery.h"

#include <algorithm>

void DisconnectRecovery::arm(TimePoint stopped_at) noexcept {
    phase_ = RecoveryPhase::Paused;
    next_action_ = stopped_at + kPause;
}

RecoveryAction DisconnectRecovery::due(TimePoint now) const noexcept {
    if (phase_ == RecoveryPhase::Idle || now < next_action_) return RecoveryAction::Wait;
    // The pulse has run its course; anything else is another heartbeat.
    if (phase_ == RecoveryPhase::Reversing && now >= reverse_stop_at_) return RecoveryAction::Stop;
    return RecoveryAction::Reverse;
}

void DisconnectRecovery::reverseWritten(TimePoint written_at) noexcept {
    if (phase_ == RecoveryPhase::Paused) {
        phase_ = RecoveryPhase::Reversing;
        reverse_stop_at_ = written_at + kReverseDuration;
    }
    // Whichever comes first: the end of the pulse, or the next heartbeat.
    next_action_ = std::min(reverse_stop_at_, written_at + kHeartbeat);
}

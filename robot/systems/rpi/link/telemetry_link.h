#pragma once
// The telemetry channel to the laptop: one TCP connection, one fixed-size
// proto::Telemetry every proto::TELEMETRY_PERIOD_MS.
//
// Read-only with respect to the robot. It samples the health sensors and copies
// whatever the other threads have already published; it decides nothing and
// commands nothing, so a stalled or missing controller can never affect driving.

#include "options.h"
#include "state.h"

void runTelemetryLink(const RobotOptions& options, RobotState& state);

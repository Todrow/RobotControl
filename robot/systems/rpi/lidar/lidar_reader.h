#pragma once
// The STL-19P scanner: it measures, it does not decide anything.
//
// The thread turns raw scans into a per-sector distance and publishes the
// verdict into RobotState::obstacles. Thresholds are applied in exactly one
// place (control/safety/obstacle_check.h, called from ObstacleState::update);
// both the control loop and telemetry read that published verdict rather than
// re-deriving it, so the operator and the robot can never disagree about which
// sector is blocked.

#include "options.h"
#include "state.h"

void runLidarReader(const RobotOptions& options, RobotState& state);

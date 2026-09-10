#pragma once
// The autonomy channel to the operator: map and pose out, mode requests in.
//
// A system like any other -- it moves bytes and nothing else. It does not decide
// whether the robot explores: an incoming byte only sets RobotState's
// explore_requested, and the control loop is still the one place that turns a
// request into a mode. Losing this connection therefore does not stop
// exploration, which is deliberate: the robot keeps mapping while the operator
// walks out of Wi-Fi range.

#include "options.h"
#include "state.h"

void runMapLink(const RobotOptions& options, RobotState& state);

#pragma once
// The one place that commands the drive and the camera servos.
//
//   link  ->  supervisor  ->  control  ->  systems
//
// Each pass: take what the operator sent, decide what it means, decide what to
// do about it, then act. Manual mode is the only mode today; when autonomy
// arrives it becomes a second source of DriveCommand right before the safety
// gate, and everything downstream -- the gate, the UART, the link-loss
// manoeuvre -- keeps working unchanged, because none of it knows or cares who
// produced the command.
//
// Single writer by construction: the socket thread no longer reaches the motors
// on its own, so nothing can race the loop for the UART.

#include "options.h"
#include "state.h"

void runControlLoop(const RobotOptions& options, RobotState& state, VideoTarget& video_target);

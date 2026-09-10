#pragma once
// The camera: an H264 RTP stream pushed to whichever host is driving.
//
// rpicam-vid feeds GStreamer on the Raspberry Pi, ksvideosrc on Windows. With
// no fixed --video-host the destination follows the active command connection,
// which is why this takes the VideoTarget slot rather than a plain address.

#include "options.h"
#include "state.h"

void runCameraStream(const RobotOptions& options, RobotState& state,
                     const VideoTarget& target);

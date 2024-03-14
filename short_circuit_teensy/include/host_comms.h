#pragma once

#include "ros_comms.h"

namespace host_comms {

void init();

void poll();

// Returns the milliseconds elapsed since the last time we successfully received a message
uint32_t message_age();

double steering_angle();

void send_debug_info(ros_comms::DebugInfo info);

void send_nand_odometry(double x, double y);
    
}
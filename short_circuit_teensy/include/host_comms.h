#pragma once

#include "ros_comms.h"

namespace host_comms {

void init();

void poll();

void send_debug_info(ros_comms::DebugInfo info);

void send_nand_odometry(double x, double y);
    
}
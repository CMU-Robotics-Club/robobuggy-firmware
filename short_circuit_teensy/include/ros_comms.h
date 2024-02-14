#pragma once

#include "brake.h"

#include <cstdint>

namespace ros_comms {
    void init();

    float steering_angle();

    brake::Status brake_status();

    struct DebugInfo {
        float rc_steering_angle;
        float steering_angle;
        float battery_voltage;
        bool operator_ready;
        bool steering_alarm;
        brake::Status brake_command;
        bool use_autonomous_steering;
        std::uint8_t rc_uplink_quality;
        std::uint8_t nand_fix;
    };

    void publish_debug_info(DebugInfo info);

    void publish_nand_odometry(double x, double y);

    // Call this once per loop iteration
    void spin_once();
}
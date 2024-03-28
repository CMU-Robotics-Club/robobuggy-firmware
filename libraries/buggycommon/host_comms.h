#pragma once

#include <cstdint>
#include <brake.h>

namespace host_comms {

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
	uint8_t padding[2];
};

enum AlarmStatus : uint8_t {
	Ok      = 0,
	Warning = 1,
	Error   = 2
};

void init();

void poll();

// Returns the milliseconds elapsed since the last time we successfully received a message
uint32_t message_age();

double steering_angle();

AlarmStatus alarm_status();

void send_debug_info(DebugInfo info);

void send_nand_odometry(double x, double y);
    
}
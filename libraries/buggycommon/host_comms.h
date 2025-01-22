#pragma once

#include <cstdint>
#include <brake.h>
#include <steering.h>

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

struct NANDDebugInfo {
	// 64 bits
	double heading_rate; // positive when accelerating in CCW direction (done)
	double encoder_pos; // encoder position 
	// 32 bits
	int timestamp; //teensy timestamp (done)
	float rc_steering_angle; // steering angle sent by TX12 (done)
	float steering_angle; // steering angle commanded by software (done)
	float true_stepper_pos; // actual stepper position (done)
	int rfm69_timeout_cnt;// # of times RFM69 has timeout
	// 8 bits
	bool operator_ready;// operator ready (done)
	brake::Status brake_status;// brake status (char) (done)
	bool use_auton_steering;// use auton steer (done)
	bool tx12_connected;// TX12 connected (done)
	steering::Status steering_alarm; // unsigned char (done)
	uint8_t rc_uplink;// RC uplink quality (done)
	uint8_t padding[2];
};

struct NANDUKF {
	// 64 bits
	double eastern; // position eastern
	double northern; // position northern
	double heading; // current heading, radians, value of 0 pointing east, increasing counterclockwise
	double heading_rate; // positive when accelerating in CCW direction
	double front_speed; // speed of the front wheel
	// 32 bits
	int timestamp; // teensy timestamp
};

struct NANDRawGPS {
	// 64 bits
	double eastern;// position eastern
	double northern;// position northern
	double accuracy;// accuracy of gps position
	uint64_t gps_time;// GPS time
	// 32 bits
	int gps_seq_num;// gps seqence number
	int timestamp;// teensy timestamp
	// 8 bits
	uint8_t fix_type;// RTK fix type
	uint8_t padding[3];
};

struct SCDebugInfo {
	// 64 bits
	double encoder_pos;// Front wheel position ( = 0, no encoder on SC)
	// 32 bits
	float rc_steering_angle;// RC steering angle
	float steering_angle;// Steering angle commanded by software
	float true_stepper_position;// true stepper position
	int missed_packets;// # missed packets since last debug packet (not yet)
	int timestamp;// Teensy timestamp (not yet)
	// 8 bits
	bool tx12_connected;// TX12 Connected 
	bool operator_ready;// Operator ready
	steering::Status stepper_alarm;// Stepper alarm (unsigned char)
	brake::Status brake_status;// Brake status (unsigned char)
	bool use_auton_steering;// Use auton steer (bool)
	uint8_t rc_uplink_quality;// RC Uplink Quality (uint8_t)
	uint8_t padding[2]; // 32 bit aligned struct
};

struct SCSensors {
	// 64 bits
	double front_speed;// Front wheel speed ( = 0, no encoder on SC)
	// 32 bits
	float true_stepper_position;
	int timestamp;// teensy timestamp
};

struct SCRadioRx {
	// 64 bits
	double nand_east;
	double nand_north;
	// 32 bits
	uint32_t gps_seq;
	// 8 bits
	uint8_t nand_fix;
	uint8_t padding[3];
};

struct Roundtrip {
	// 32 bits
	int time;
	int soft_time;
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

int software_time();

AlarmStatus alarm_status();

void send_debug_info(DebugInfo info);

void send_nand_odometry(double x, double y, uint32_t radio_seq, uint32_t gps_seq);

void send_bnya_telemetry(
	double x, double y,
	double velocity,
	double steering,
	double heading,
	double heading_rate
);

void send_timestamp(Roundtrip time);

void sc_send_sensors(SCSensors s);

void sc_send_debug_info(SCDebugInfo info);    

void sc_send_nand_pos(SCRadioRx nand);
}
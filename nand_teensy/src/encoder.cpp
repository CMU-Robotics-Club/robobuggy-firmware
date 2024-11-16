#include "encoder.h"
#include "AS5600.h"

#include <Arduino.h>

#include <Wire.h>

#define ENCODER_I2C Wire
#define ENCODER_ADDR 0x36 // not used in this file, but this is the hard coded I2C address for the encoder

#define DIAMETER_M (0.180)
#define HISTORY_LEN 1000

namespace encoder {

AS5600 as5600(&ENCODER_I2C);
float positions_rad[HISTORY_LEN] = {0};
unsigned long long times_us[HISTORY_LEN] = {0};
int history_index = 0;
int prev_time = 0; // time most recent I2C call took

int prev_time_millis() {
	return prev_time;
}

double front_speed() {
	// return  as5600.getCumulativePosition() * 2.0 * M_PI / 4096.0;
	int t1 = millis();
	positions_rad[history_index] = as5600.getCumulativePosition() * 2.0 * M_PI / 4096.0;
	int tF = millis() - t1;
	prev_time = tF;
	times_us[history_index] = micros();

	int prev_index = (history_index + 1) % HISTORY_LEN;
	float speed = -DIAMETER_M / 2.0 * 1e6 * (positions_rad[history_index] - positions_rad[prev_index]) / (times_us[history_index] - times_us[prev_index]);
	// Serial.println(times_us[history_index] - times_us[prev_index]);
	// Serial.println(positions_rad[history_index] - positions_rad[prev_index]);

	history_index = (history_index + 1) % HISTORY_LEN;
	return speed; // v=r*omega
	// when getAngularSpeed returns radians per second, builtin_front_speed returns meters per second
	//NOTE: current orientation of the magnet and therefore sign of the getAngularSpeed function
	//		is currently untested, so the wheel rolling forward may result in a negative angular
	//		speed value.
}

double rear_speed(double steering_angle) {
	steering_angle *= M_PI / 180.0;
	return front_speed() * cos(steering_angle);
}

uint16_t e_angle() {
	return as5600.readAngle();
}

uint16_t e_raw_angle() {
	return as5600.rawAngle();
}

bool e_m_strong() {
	return as5600.magnetTooStrong();
}

bool e_m_weak() {
	return as5600.magnetTooWeak();
}

void init() {
	//need to set direction (need to test once mounted)
	as5600.begin();
	while(!as5600.isConnected()) {
		Serial.println("Encoder not connected");
	}
}

}
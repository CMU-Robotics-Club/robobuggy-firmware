#include "encoder.h"
#include "AS5600.h"

#include <Arduino.h>

#include <Wire.h>

#define ENCODER_I2C Wire1

#define RADIUS_M (0.180)

namespace encoder {

AS5600 as5600(&ENCODER_I2C);

double front_speed() {
	return as5600.getAngularSpeed(AS5600_MODE_RADIANS)*RADIUS_M; // v=r*omega
	// when getAngularSpeed returns radians per second, builtin_front_speed returns meters per second
	//NOTE: current orientation of the magnet and therefore sign of the getAngularSpeed function
	//		is currently untested, so the wheel rolling forward may result in a negative angular
	//		speed value.
}

double rear_speed(double steering_angle) {
	steering_angle *= M_PI / 180.0;
	return front_speed() * cos(steering_angle);
}

void init() {
	//need to set direction (need to test once mounted)
	as5600.begin();
	while(!as5600.isConnected()) {
		Serial.println("Encoder not connected");
	}
}

}
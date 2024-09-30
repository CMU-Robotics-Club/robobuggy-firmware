#include "encoder.h"
#include "AS5600.h"

#include <Arduino.h>

#include <Wire.h>

#define ENCODER_I2C Wire1

#define ENCODER_PIN 40
#define NUM_BUCKETS 20
#define BUCKET_INTERVAL_MS 40
#define PPR 5
#define CIRCUMFERENCE_M (0.565)
#define RADIUS_M (0.180)

namespace encoder {

volatile int TOTAL_STEPS = 0;
volatile int STEPS[NUM_BUCKETS];
volatile int bucket = 0;
AS5600 as5600(&ENCODER_I2C);

double front_speed() {
	return as5600.getAngularSpeed(AS5600_MODE_RADIANS)*RADIUS_M; // v=r*omega
	// when getAngularSpeed returns radians per second, builtin_front_speed returns meters per second
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
	//if not connected, serial print in while(1)
}

}
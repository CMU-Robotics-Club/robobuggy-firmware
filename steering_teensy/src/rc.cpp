#include "rc.h"

#include <Arduino.h>
#include <ArduinoCRSF.h>

namespace rc {

#define RC_SERIAL Serial6
#define RC_BAUDRATE 115200

#define CHANNEL_LEFT_X  4
#define CHANNEL_LEFT_Y  3
#define CHANNEL_RIGHT_X 1
#define CHANNEL_RIGHT_Y 2
#define CHANNEL_SWITCH_E 5
#define CHANNEL_SWITCH_F 6
#define CHANNEL_SWITCH_B 7
#define CHANNEL_SWITCH_C 8
#define CHANNEL_BUTTON_A 9
#define CHANNEL_BUTTON_D 10

ArduinoCRSF rc_controller;

void init() {
	RC_SERIAL.begin(115200);
	if (!RC_SERIAL) {
		while (1) {
			Serial.println("RC serial port init failed!");
			delay(1000);
		}
	}
	rc_controller.begin(RC_SERIAL);
}

void update() {
	rc_controller.update();
}

const crsfLinkStatistics_t& link_statistics() {
	return *rc_controller.getLinkStatistics();
}

bool connected() {
	return rc_controller.isLinkUp();
}

#define RC_STEERING_DEGREES 30.0

bool operator_ready() {
	if (connected()) {
		return (rc_controller.getChannel(CHANNEL_BUTTON_A) > 1500) || (rc_controller.getChannel(CHANNEL_BUTTON_D) > 1500);
	} else {
		return false;
	}
}

float steering_angle() {
	if (connected()) {
		int raw_width = rc_controller.getChannel(CHANNEL_RIGHT_X);

		// Scaled to -1.0 to 1.0, left positive
		float analog = -1.0 * (raw_width - 1500.0) / 500.0;

		return analog * RC_STEERING_DEGREES;
	} else {
		return 0.0;
	}
}

bool use_autonomous_steering() {
	bool auto_switch = (rc_controller.getChannel(CHANNEL_SWITCH_E) > 1750);
	return operator_ready() && auto_switch;
}

} // namespace rc
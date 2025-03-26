#pragma once

#include <ArduinoCRSF.h>

namespace rc {
	void init(HardwareSerial &serial);

	void update();

	const crsfLinkStatistics_t& link_statistics();

	bool connected();

	// Returns true if the controller is connected and the dead man's switches are held down.
	bool operator_ready();

	// Returns the commanded steering angle in degrees.
	// If the controller has disconnected, returns 0.0 (center).
	float steering_angle();

	// Returns true if the auto steering angle should be used instead of the RC input.
	// This is only true if the operator is ready and the auto switch is activated.
	bool use_autonomous_steering();

	// Returns true if the set offset switch has just been flipped
	// Only triggers on rising edge of switch
	bool temp_offset_switch();

	// returns true if swtich F on TX12 (channel 6) is in high position
	bool offset_switch();
        
        // returns true if button A on TX12 (channel 11) is pressed down
	bool offset_button();
}

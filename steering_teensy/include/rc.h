#pragma once

#include <ArduinoCRSF.h>

namespace rc {
	void init();

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
}
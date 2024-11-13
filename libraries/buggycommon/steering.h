#pragma once

// Steering is left-positive and measured in degrees from straight ahead.
namespace steering {
	// This begins a background timer interrupt that will send pulses on the stepper pins.
	void init(int pulse_pin, int dir_pin, int alarm_pin, int left_stepper_switch, int right_stepper_switch, float steps_per_degree, int center_step_offset);

	// Determine the left and right angle limits and then center the wheel.
	// This will block until the calibration is done.
	void calibrate();

	// Set the position (in degrees from center, left-positive) that the stepper will try to achieve.
	void set_goal_angle(float degrees);

	// Store the current steering position as the temporary zero steering point
	void set_offset(float degrees);

	float current_angle_degrees();

	// Returns true if the alarm pin has been triggered,
	// meaning that the stepper driver is no longer responding and we have lost steering control.
	// A full power cycle of both the Teensy and driver is required to clear this condition.
	bool alarm_triggered();

	int left_step_limit();
	int right_step_limit();
}

#pragma once

namespace encoder {

void init();

void update();

int steps();

double front_speed();

/**
 * @brief Calculates the speed of the buggy's rear, given the angle at which it's steering
 * (and implicitly the speed at which the front is moving)
 * 
 * @param steering_angle Steering angle IN DEGREES.
 */
double rear_speed(double steering_angle);

}
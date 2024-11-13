#pragma once

namespace encoder {

void init();

void update();

int steps();

double front_speed();

double rear_speed(double steering_angle);

}
#pragma once

#include <stdint.h>

namespace encoder {

void init();

int steps();
int prev_time_millis();

uint16_t e_raw_angle();
uint16_t e_angle();
bool e_m_strong();
bool e_m_weak();

double front_speed();

double rear_speed(double steering_angle);

}
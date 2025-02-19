#pragma once

#include <stdint.h>

namespace encoder {

void init();

int prev_time_millis();

double front_speed();
double avg_speed();
double rear_speed(double steering_angle);

float get_front_pos();
uint16_t get_diagnostics();

uint16_t rawRot();
uint16_t state();
uint16_t gain();
double rotRad();

struct encoder_spi read_pkt(uint16_t rd_pkt);

}
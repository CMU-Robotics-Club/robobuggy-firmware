#include <Arduino.h>
#include <cstdint>
#include <Wire.h>
#pragma once

namespace encoder {
    struct angles {
        uint16_t raw_angle;
        uint16_t angle;
    };
    typedef struct angles angles_t;
    struct angle_time {
        angles_t a;
        int time; // millis
    };
    void init();
    void get_pos();
    void get_ang(angles_t *a);
    double get_speed();
    size_t get_buf_writes();
}
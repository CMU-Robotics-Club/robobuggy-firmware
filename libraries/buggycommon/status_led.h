#pragma once
#include <stdint.h>

namespace status_led {

struct Rgb {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

void init(int pin1, int pin2, int pin3);

void set_color(Rgb rgb);

}
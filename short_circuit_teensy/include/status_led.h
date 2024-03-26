#pragma once
#include <stdint.h>

namespace status_led {

struct Rgb {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

void init();

void set_color(Rgb rgb);

}
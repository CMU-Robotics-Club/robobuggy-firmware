#pragma once
#include <stdint.h>
#include <OctoWS2811.h>

namespace status_led
{

    struct Rgb
    {
        uint8_t r;
        uint8_t g;
        uint8_t b;
    };

    extern Rgb green;
    extern Rgb dark_green;
    extern Rgb light_green;
    extern Rgb red;
    extern Rgb dark_red;
    extern Rgb light_red;
    extern Rgb orange;
    extern Rgb yellow;
    extern Rgb blue;
    extern Rgb black;

    void init(OctoWS2811 *leds, int leds_per_strip, int num_led_pins);

    void set_color(Rgb rgb);

}
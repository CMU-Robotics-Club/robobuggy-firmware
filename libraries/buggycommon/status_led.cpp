#include "status_led.h"

#include <OctoWS2811.h>

// #define DATA_PIN 19

namespace status_led {

static DMAMEM int display_memory[8 * 1];
static int drawing_memory[8 * 1];

static uint8_t pin_list[1] = { 0 };

static OctoWS2811 leds(0, nullptr, nullptr);

void init(int pin) {
    pin_list[0] = pin;
    leds = OctoWS2811(1, display_memory, drawing_memory, WS2811_GRB | WS2811_800kHz, 1, pin_list);

    leds.begin();
}

void set_color(Rgb color) {
    static Rgb last_color = { 0x12, 0x34, 0x56 };

    if (color.r != last_color.r || color.g != last_color.g || color.b != last_color.b) {
        last_color = color;
        leds.setPixel(0, color.r, color.g, color.b);
        leds.show();
    }
}

} // namespace status_led
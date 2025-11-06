#include "status_led.h"

#include <OctoWS2811.h>

// #define DATA_PIN 19
#define NUM_LEDS 120
#define NUM_PINS 1

namespace status_led
{

    static DMAMEM int display_memory[8 * NUM_LEDS];
    static int drawing_memory[8 * NUM_LEDS];

    static uint8_t pin_list[NUM_PINS] = {0};

    static OctoWS2811 leds(0, nullptr, nullptr);

    void init(int pin)
    {
        pin_list[0] = pin;
        leds = OctoWS2811(NUM_LEDS, display_memory, drawing_memory, WS2811_GRB | WS2811_800kHz, NUM_PINS, pin_list);

        leds.begin();
    }

    void set_color(Rgb color)
    {
        static Rgb last_color = {0x12, 0x34, 0x56};

        if (color.r != last_color.r || color.g != last_color.g || color.b != last_color.b)
        {
            last_color = color;
            for (int i = 0; i < NUM_LEDS; i++)
                leds.setPixel(i, color.r, color.g, color.b);
            leds.show();
        }
    }

} // namespace status_led
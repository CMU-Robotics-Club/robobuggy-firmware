#include "status_led.h"

namespace status_led
{

    // Color definitions
    Rgb green = {0x00, 0xFF, 0x00};
    Rgb dark_green = {0x00, 0xD0, 0x00};
    Rgb light_green = {0x00, 0xFF, 0x20};
    Rgb red = {0xFF, 0x00, 0x00};
    Rgb dark_red = {0xD0, 0x00, 0x00};
    Rgb light_red = {0xFF, 0x20, 0x00};
    Rgb orange = {0xFF, 0x80, 0x00};
    Rgb yellow = {0x80, 0x80, 0x00};
    Rgb blue = {0x00, 0x00, 0xFF};
    Rgb black = {0x00, 0x00, 0x00};

    int _leds_per_strip = 0;
    int _num_led_pins = 0;
    OctoWS2811 *_leds;

    void init(OctoWS2811 *leds, int leds_per_strip, int num_led_pins)
    {
        _leds_per_strip = leds_per_strip;
        _num_led_pins = num_led_pins;
        _leds = leds;
        _leds->begin();
        _leds->show();
    }

    void set_color(Rgb color)
    {
        static Rgb last_color = {0x12, 0x34, 0x56};

        if (color.r != last_color.r || color.g != last_color.g || color.b != last_color.b)
        {
            last_color = color;
            for (int i = 0; i < _leds_per_strip * _num_led_pins; i++)
            {
                _leds->setPixel(i, color.r, color.g, color.b);
            }
            _leds->show();
        }
    }

} // namespace status_led
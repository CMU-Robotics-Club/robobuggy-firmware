#include "status_led.h"

#include <OctoWS2811.h>

#define LEDS_PER_STRIP 30 // LEDs per strip
#define NUM_PINS 3 // number of LED strips
#define BYTES_PER_LED 3 // change to 4 if using RGBW

namespace status_led
{
    // From PaulStoffregen/OctoWS2811/examples/Teensy4_PinList:
    // These buffers need to be large enough for all the pixels.
    // The total number of pixels is "ledsPerStrip * numPins".
    // Each pixel needs 3 bytes, so multiply by 3.  An "int" is
    // 4 bytes, so divide by 4.  The array is created using "int"
    // so the compiler will align it to 32 bit memory.
    static DMAMEM int displayMemory[LEDS_PER_STRIP * NUM_PINS * BYTES_PER_LED / 4];
    static int drawingMemory[LEDS_PER_STRIP * NUM_PINS * BYTES_PER_LED / 4];

    static byte pinList[NUM_PINS] = {0};

    static OctoWS2811 leds(0, nullptr, nullptr);

    void init(int pin1, int pin2, int pin3)
    {
        pinList[0] = pin1;
        pinList[1] = pin2;
        pinList[2] = pin3;
        leds = OctoWS2811(LEDS_PER_STRIP, displayMemory, drawingMemory, WS2811_GRB | WS2811_800kHz, NUM_PINS, pinList);
        leds.begin();
        leds.show();
    }

    void set_color(Rgb color)
    {
        static Rgb last_color = {0x12, 0x34, 0x56};

        if (color.r != last_color.r || color.g != last_color.g || color.b != last_color.b)
        {
            last_color = color;
            for (int i = 0; i < LEDS_PER_STRIP * NUM_PINS; i++) {
                leds.setPixel(i, color.r, color.g, color.b);
            }
            leds.show();
        }
    }

} // namespace status_led
#include <Arduino.h>
#include <cstdint>
#include <Wire.h>

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
    void pos(angles_t *a);
    double get_speed();
}
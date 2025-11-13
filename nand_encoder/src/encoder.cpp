#include <Arduino.h>
#include "encoder.h"

#define I2C_ADDRESS 0x36
#define RAW_REGISTER 0x0C
#define FIX_REGISTER 0x0E
#define QUANTITY 2
#define STOP true
#define BUFFER_SIZE 100
#define RAW_SCALE 4096.0
#define RAW_SCALE_INT 4096
#define CIRCUMFERENCE 0.5186

namespace encoder
{
    /**
     * @brief Former value read from encoder, used for calculating speed.
     */
    unsigned long last_value;
    /**
     * @brief Timestamp (in microseconds) of when last_value was measured.
     */
    unsigned long last_micros;

    bool forward = true; // true = value increases.  false = value decreases.

    void init()
    {
        Wire.begin();
    }

    uint16_t to_degrees(uint16_t value)
    {
        return value * 360 / 4096;
    }

    uint16_t get_ticks()
    {
        uint16_t raw_value = 0;
        uint16_t value = 0;
        // TODO the encoder provides both a "raw angle" and an "angle".
        // based on the current configuration, they are the same value, but it could be something to look into later.

        Wire.beginTransmission(I2C_ADDRESS);
        Wire.write(RAW_REGISTER); // write register address
        Wire.endTransmission();
        Wire.requestFrom(I2C_ADDRESS, QUANTITY, STOP);
        raw_value |= (Wire.read() & 0x0F) << 8; // RAW_ANGLE[11:8]
        raw_value |= Wire.read();               // RAW_ANGLE[7:0]
        Wire.beginTransmission(I2C_ADDRESS);
        Wire.write(FIX_REGISTER); // write register address
        Wire.endTransmission();
        Wire.requestFrom(I2C_ADDRESS, QUANTITY, STOP);
        value |= (Wire.read() & 0x0F) << 8; // ANGLE[11:8]
        value |= Wire.read();               // ANGLE[7:0]

        return value;
    }

    void update()
    {
        last_value = get_ticks();
        last_micros = micros();
    }

    uint16_t get_degrees()
    {
        return to_degrees(get_ticks());
    }

    double get_degrees_per_second()
    {
        unsigned long last_angle = to_degrees(last_value);
        unsigned long current_angle = (unsigned long)to_degrees(get_ticks());
        unsigned long current_micros = micros();

        unsigned long d_angle = current_angle - last_angle;
        unsigned long d_time = current_micros - last_micros;
        Serial.printf("d_angle: %lu degrees\t d_time: %lu microseconds\n", d_angle, d_time);

        double speed = d_angle / d_time;
        speed /= 1000000;
        return speed;
    }

}
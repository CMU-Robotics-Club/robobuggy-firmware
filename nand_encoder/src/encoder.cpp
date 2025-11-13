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

    void init()
    {
        Wire.begin();
    }

    float to_degrees_int(uint16_t value)
    {
        return (value * 360.0) / 4096.0;
    }

    float to_degrees_float(float value)
    {
        return (value * 360.0) / 4096.0;
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

    float get_degrees()
    {
        return to_degrees_int(get_ticks());
    }

    float get_degrees_per_second()
    {
        // save current encoder value and time
        unsigned long current_value = (unsigned long)get_ticks();
        unsigned long current_micros = micros();

        // determine which direction we are moving, using the assumption that
        // the wheel has NOT moved more than 180 degrees (aka 2048 encoder ticks) since the last time this function was called
        bool forward; // true = value increases over time.  false = value decreases over time.
        // there are a couple of different cases to consider:
        if (current_value >= last_value)
        {
            forward = true;
            // check if it "wrapped around"
            if ((last_value + 4096) - current_value < 2048)
            {
                forward = false;
            }
        }
        else if (current_value < last_value)
        {
            forward = false;
            // check if it "wrapped around"
            if ((current_value + 4096) - last_value < 2048)
            {
                forward = true;
            }
        }

        // calculate (signed) change in angle based on direction determined
        float d_angle;
        if (forward)
        {
            if (last_value > current_value)
            { // the value wrapped around
                d_angle = (current_value + 4096) - last_value;
            }
            else
            {
                d_angle = current_value - last_value;
            }
        }
        else
        {
            if (current_value > last_value)
            { // the value wrapped around
                d_angle = (last_value + 4096) - current_value;
            }
            else
            {
                d_angle = last_value - current_value;
            }
            d_angle *= -1;
        }
        d_angle = to_degrees_float(d_angle);

        float d_time = current_micros - last_micros;
        float speed = (d_angle * 1000000) / d_time;

        // Serial.print("d_angle: ");
        // Serial.print(d_angle);
        // Serial.print("\tspeed: ");
        // Serial.print(speed);
        // Serial.println();

        // update state for the next time this function is called
        last_value = current_value;
        last_micros = current_micros;

        return speed;
    }

}
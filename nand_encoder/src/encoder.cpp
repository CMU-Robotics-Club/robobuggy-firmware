/**
 * @file encoder.cpp
 * @brief Implementation for the encoder namespace on the XIAO
 *
 * This file implements functions in the encoder namespace to communicate with
 * the AS5600 over I2C, which outputs the current wheel position. Further
 * functions are used to convert to degrees and compute instantaneous speed.
 *
 * @author Delaynie McMillan
 * @author Alden Grover
 * @author Anna Delale-O'Connor
 * @date 1/12/2026
 *
 * @author Sanjay Ravishankar
 * @date 1/31/2026 - Changed function signatures to add error reporting,
 * added some documentation, cleaned up code a bit
 */

#include <Arduino.h>
#include "encoder.h"

#define I2C_ADDRESS 0x36
#define RAW_REGISTER 0x0C
#define FIX_REGISTER 0x0E
#define QUANTITY 2
#define STOP true
#define BUFFER_SIZE 100
#define RAW_SCALE 4096.0
#define RAW_SCALE_INT 4096 // 12-bit ADC
#define CIRCUMFERENCE 0.5186

namespace encoder
{
    /**
     * @brief Former value read from encoder, used for calculating speed.
     */
    unsigned long last_value = 0;
    /**
     * @brief Timestamp (in microseconds) of when last_value was measured.
     */
    unsigned long last_micros = 0;

    bool init()
    {
        // Serial.println("starting wire");
        Wire.begin();
        // Serial.println("we started wire");
        Wire.beginTransmission(I2C_ADDRESS);
        return Wire.endTransmission() == 0;
    }

    float to_degrees(uint16_t value)
    {
        return (value * 360.0) / 4096.0;
    }

    float to_degrees(float value)
    {
        return (value * 360.0) / 4096.0;
    }

    /**
     * @details Reads 2 bytes from the FIX register to get the encoder
     * angle in the range [0-4096) (12-bit resolution). If any of the
     * I2C operations fail, we return false; this should be registered
     * as a comm error.
     */
    bool get_ticks(uint16_t *position_ptr)
    {
        if (!position_ptr)
            return false;

        /** TODO: the encoder provides both a "raw angle" and an "angle".
         * based on the current configuration, they are the same value,
         * but it could be something to look into later.
         * For now the raw angle read is commented out.
         */

        /*
        uint16_t raw_value = 0;
        Wire.beginTransmission(I2C_ADDRESS);
        Wire.write(RAW_REGISTER); // write register address
        if (Wire.endTransmission() != 0)
            return false;
        if (Wire.requestFrom(I2C_ADDRESS, QUANTITY, STOP) != QUANTITY)
            return false;
        raw_value |= (Wire.read() & 0x0F) << 8; // RAW_ANGLE[11:8]
        raw_value |= Wire.read();               // RAW_ANGLE[7:0]
        */

        uint16_t value = 0;
        Wire.beginTransmission(I2C_ADDRESS);
        Wire.write(FIX_REGISTER); // write register address
        if (Wire.endTransmission() != 0)
            return false;
        if (Wire.requestFrom(I2C_ADDRESS, QUANTITY, STOP) != QUANTITY) // Writes to buffer
            return false;
        value |= (Wire.read() & 0x0F) << 8; // ANGLE[11:8]
        value |= Wire.read();               // ANGLE[7:0]
        *position_ptr = value;
        return true;
    }

    bool get_degrees(float *degrees_ptr)
    {
        if (!degrees_ptr)
            return false;
        uint16_t ticks = 0;
        if (!get_ticks(&ticks))
            return false;
        *degrees_ptr = to_degrees(ticks);
        return true;
    }

    bool get_degrees_per_second(float *dps_ptr)
    {
        if (!dps_ptr)
            return false;
        uint16_t ticks = 0;
        if (!get_ticks(&ticks))
            return false;
        // save current encoder value and time
        unsigned long current_value = (unsigned long)ticks;
        unsigned long current_micros = micros();
        // determine which direction we are moving, using the assumption that
        // the wheel has NOT moved more than 180 degrees (aka 2048 encoder ticks) since the last time this function was called
        bool forward; // true = value increases over time.  false = value decreases over time.
        // there are a couple of different cases to consider:
        /**
         * TODO: clean up if statements and use the DIR pin on the AS5600
         * to know direction rather than relying on the 180deg assumption
         */
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
        d_angle = to_degrees(d_angle);
        float d_time = current_micros - last_micros;
        if (d_time == 0) // Divide by zero guard
            return false;
        float speed = (d_angle * 1000000) / d_time;
        *dps_ptr = speed;
        // Serial.print("d_angle: ");
        // Serial.print(d_angle);
        // Serial.print("\tspeed: ");
        // Serial.print(speed);
        // Serial.println();
        // update state for the next time this function is called
        last_value = current_value;
        last_micros = current_micros;
        return true;
    }
}
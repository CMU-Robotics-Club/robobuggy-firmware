#include <Arduino.h>
#include <cstdint>
#include <Wire.h>
#pragma once

namespace encoder
{

    /**
     * @brief struct to hold information about errors, both in encoder and I2C; includes whether
     * position written to, most recent packet sent, and error code.
     */
    struct errorHold{
        boolean isWritten;
        uint32_t i2cError;
        uint32_t recentPacket;
        // what should be error for AS5600? Might be values in STATUS register, 0x0B
        // for STATUS, error if bit 3 or bit 4 is high

    };

    /**
     * @brief Initializes the encoder module and I2C communication.
     * Must be called once during setup before using other encoder functions.
     */
    void init();

    /**
     * @brief Gets the current rotational position of the encoder, in units of tick count.
     * @return The current encoder position; a number in [0, 4096], inclusive.
     */
    uint16_t get_ticks();

    /**
     * @brief Gets the current rotational position of the encoder in degrees.
     * @return The rotation angle in degrees [0, 360] inclusive.
     */
    float get_degrees();

    /**
     * @brief Gets the current rotational speed.
     * IMPORTANT: This function must be called periodically for accurate readings.
     * This function assumes that the wheel has not rotated *more* than 180 degrees since the last call of this function.
     * @return The speed value in units of degrees per second.
     */
    float get_degrees_per_second();
}
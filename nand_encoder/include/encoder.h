#include <Arduino.h>
#include <cstdint>
#include <Wire.h>
#pragma once

namespace encoder
{

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
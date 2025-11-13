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
     * @brief Updates the encoder state by reading current values from the sensor.
     * This is for calculating the derivative needed for get_speed().
     * Should be called regularly (e.g., in the main loop) to maintain accurate readings.
     */
    void update();

    /**
     * @brief Gets the current encoder tick count.
     * @return The current encoder position; a number in [0, 4096], inclusive.
     */
    uint16_t get_ticks();

    /**
     * @brief Gets the current rotation angle in degrees.
     * @return The rotation angle in degrees [0, 360] inclusive.
     */
    uint16_t get_degrees();

    /**
     * @brief Gets the current rotational speed.
     * @return The speed value in units of degrees per second.
     */
    double get_degrees_per_second();
}
/**
 * @file encoder.h
 * @brief Header file for the encoder namespace on the XIAO
 *
 * This file declares functions in the encoder namespace that allow I2C 
 * communication with the AS5600 encoder module and compute speed.
 *
 * @author Delaynie McMillan
 * @author Alden Grover
 * @author Anna Delale-O'Connor
 * @date 1/12/2026
 * 
 * @author Sanjay Ravishankar
 * @date 1/31/2026 - Changed function signatures to add error reporting
 */

#pragma once
#include <Arduino.h>
#include <cstdint>
#include <Wire.h>

namespace encoder
{
    /**
     * @brief Initializes the encoder module and I2C communication
     * Must be called once during setup before using other encoder functions
     * @return If the AS5600 can be read over I2C
     */
    bool init();

    /**
     * @brief Gets the current rotational position of the encoder, in units of tick count
     * @param[in,out] position: The current encoder position; a number in [0, 4096)
     * @return If the operation worked
     */
    bool get_ticks(uint16_t *position_ptr);

    /**
     * @brief Gets the current rotational position of the encoder in degrees
     * @param[in,out] degrees_ptr: The float pointer for the rotation angle in degrees [0, 360)
     * @return If the operation worked
     */
    bool get_degrees(float *degrees_ptr);

    /**
     * @brief Gets the current rotational speed
     * @param[in,out] dps_ptr: The float pointer for the speed value in units of degrees per second
     * @return If the operation worked
     *
     * IMPORTANT: This function must be called periodically for accurate readings.
     * This function assumes that the wheel has not rotated *more* than 180 degrees since the last call of this function.
     */
    bool get_degrees_per_second(float *dps_ptr);
}
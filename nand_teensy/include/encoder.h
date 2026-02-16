/**
 * @file encoder.h
 * @brief Header file for the encoder namespace on the Teensy
 * 
 * This file declares functions in the encoder namespace that create 
 * an interface for the encoder microcontroller and to read encoder data.
 * See nand_teensy/examples/encoder-test.cpp for usage.
 * 
 * @author Sanjay Ravishankar
 * @date 1/31/2026-2/16/2026
 */

#pragma once
#include <Arduino.h>
#define COMM_SERIAL Serial6
#define COMM_BAUDRATE 115200

namespace encoder
{
    /**
     * @brief Initializes serial communication with the XIAO
     * Must be called during setup before other encoder functions
     */
    void init();

    /**
     * @brief State machine for encoder communication
     * Must be called in the loop
     */
    void poll();

    /**
     * @brief Gets the speed as a float
     * @param[in,out] s: The float pointer for the current front wheel 
     * speed in degrees/sec
     * @return If the operation worked
     */
    bool front_speed(float *s);

    /**
     * @brief Computes the rear wheel speed given the steering angle
     * @param[in] steering_angle: The current steering angle in degrees
     * @return The rear wheel speed in degrees/sec
     */
    double rear_speed(double steering_angle);

    /**
     * @brief Used for making sure packets are being received 
     * @return How long ago the last packet was received in ms
     */
    long lastPacket();
}
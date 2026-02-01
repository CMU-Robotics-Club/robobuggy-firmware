/**
 * @file ratelimit.h
 * @brief Implementation for RateLimit class on the XIAO
 *
 * This file defines the RateLimit class for a simple rate limiter
 *
 * @author Alden Grover
 * @author Anna Delale-O'Connor
 * @date 1/12/2026
 *
 * @author Sanjay Ravishankar
 * @date 1/31/2026 - moved code from main.cpp to here
 */

#pragma once
#include <Arduino.h>

class RateLimit
{
public:
    unsigned long period;

    /**
     * @brief Constructor for RateLimit class
     * @param[in] _period: Rate limit period (in ms)
     */
    RateLimit(unsigned long _period) : period(_period), last_time(millis()) {}

    /**
     * @brief Checks rate limit state
     * @returns Whether the rate limited operation should be performed
     */
    bool ready()
    {
        unsigned long cur_time = millis();
        if (cur_time - last_time > period)
        {
            last_time = cur_time;
            return true;
        }
        return false;
    }

    /**
     * @brief Resets the rate limit timer
     */
    void reset()
    {
        last_time = millis();
    }

private:
    unsigned long last_time;
};
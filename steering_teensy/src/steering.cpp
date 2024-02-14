#include "steering.h"
#include <Arduino.h>

namespace steering
{

#define STEPS_PER_REV 1000 // steps per rotation
#define PUL 27             // pin for stepper pulse
#define DIR 38             // pin for stepper direction
#define ALARM_PIN 39

#define LIMIT_SWITCH_RIGHT 7
#define LIMIT_SWITCH_LEFT 8

    inline bool at_left_limit()
    {
        return !digitalRead(LIMIT_SWITCH_LEFT);
    }

    inline bool at_right_limit()
    {
        return !digitalRead(LIMIT_SWITCH_RIGHT);
    }

    volatile int current_position = 0;
    volatile int goal_position = 0;

    /**
     * @brief Sends pulses to the stepper driver, to make current_position line up with goal_position.
     * The stepper driver excepts normally high, with low pulses.
     * However, this pin goes into a MOSFET, so the polarity is inverted.
     */
    static void pulse_interrupt_handler()
    {

        if (current_position < goal_position)
        {
            if (at_left_limit())
            {
                return;
            }
            digitalWrite(DIR, LOW);
            delayMicroseconds(5);
            digitalWrite(PUL, HIGH);
            delayMicroseconds(5);
            digitalWrite(PUL, LOW);
            ++current_position;
        }
        else if (current_position > goal_position)
        {
            if (at_right_limit())
            {
                return;
            }
            digitalWrite(DIR, HIGH);
            delayMicroseconds(5);
            digitalWrite(PUL, HIGH);
            delayMicroseconds(5);
            digitalWrite(PUL, LOW);
            --current_position;
        }
        else
        {
            return;
        }
    }

    IntervalTimer pulse_timer;

    int LEFT_STEPPER_LIMIT = 0;
    int RIGHT_STEPPER_LIMIT = 0;

    /**
     * @brief Initializes hardware.  Should be called in the main setup() function.
     */
    void init()
    {
        pinMode(LIMIT_SWITCH_LEFT, INPUT_PULLUP);
        pinMode(LIMIT_SWITCH_RIGHT, INPUT_PULLUP);

        pinMode(ALARM_PIN, INPUT_PULLUP);

        pinMode(PUL, OUTPUT);
        pinMode(DIR, OUTPUT);

        pulse_timer.begin(pulse_interrupt_handler, 50);
        pulse_timer.priority(255);
    }

    /**
     * @brief The steering motor performs the calibration sequence by rotating to both of the limit switches,
     * and then updating the step counter.
     * This function must be called after init() but before moving the buggy.
     */
    void calibrate()
    {
        Serial.println("Beginning calibration...");
        while (!at_left_limit())
        {
            delay(1);
            ++goal_position;
        }
        LEFT_STEPPER_LIMIT = goal_position;
        Serial.printf("Determined left limit (%d)\n", LEFT_STEPPER_LIMIT);

        while (!at_right_limit())
        {
            delay(1);
            --goal_position;
        }
        RIGHT_STEPPER_LIMIT = goal_position;
        Serial.printf("Determined right limit (%d)\n", RIGHT_STEPPER_LIMIT);

        int offset = (LEFT_STEPPER_LIMIT + RIGHT_STEPPER_LIMIT) / 2;
        goal_position -= offset;
        current_position -= offset;
        LEFT_STEPPER_LIMIT -= offset;
        RIGHT_STEPPER_LIMIT -= offset;

        Serial.println("Calibration finished.");

        // Center the steering again
        set_goal_angle(0.0);
    }

    // Start with steps per revolution of the stepper,
    // divide by 360 to get steps per degree of the stepper,
    // multiply by the gear ratio to get steps per degree of the gearbox,
    // and finally multiply by the belt ratio to get steps per degree of the wheel.
    const float STEPS_PER_DEGREE = (STEPS_PER_REV / 360.0) * 10.0 * (34.0 / 18.0);

    void set_goal_angle(float degrees)
    {
        goal_position = (int)(STEPS_PER_DEGREE * degrees);
    }

    /**
     * @brief Checks whether or not the "alarm pin" has been triggered.
     * The pin triggers once the stepper goes into overcurrent protection and is then unresponsive to any movement commands.
     */
    bool alarm_triggered()
    {
        // static to make sure it persists between calls
        static bool fault = false;

        if (!digitalRead(ALARM_PIN))
        {
            fault = true;
        }

        return fault;
    }

    int left_step_limit() { return LEFT_STEPPER_LIMIT; }
    int right_step_limit() { return RIGHT_STEPPER_LIMIT; }

} // namespace steering
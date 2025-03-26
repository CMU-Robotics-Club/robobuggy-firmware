#include "steering.h"
#include <Arduino.h>

namespace steering
{
    static int pulse_pin = -1;
    static int dir_pin   = -1;
    static int alarm_pin = -1;
    static int left_stepper_switch_pin = -1;
    static int right_stepper_switch_pin = -1;
    static float steps_per_degree = 0.0;
    static float temporary_offset = 0.0;
    static int center_step_offset = 0;


#define STEPS_PER_REV 1000 // steps per rotation


    inline bool at_left_limit()
    {
        return !digitalRead(left_stepper_switch_pin);
    }

    inline bool at_right_limit()
    {
        return !digitalRead(right_stepper_switch_pin);
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
            digitalWrite(pulse_pin, HIGH);
            delayMicroseconds(5);
            digitalWrite(pulse_pin, LOW);
            ++current_position;
        }
        else if (current_position > goal_position)
        {
            if (at_right_limit())
            {
                return;
            }
            digitalWrite(pulse_pin, HIGH);
            delayMicroseconds(5);
            digitalWrite(pulse_pin, LOW);
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
    void init(
        int pulse_pin_,
        int dir_pin_,
        int alarm_pin_,
        int left_stepper_switch_pin_,
        int right_stepper_switch_pin_,
        float steps_per_degree_,
        int center_step_offset_
    ) {
        pulse_pin = pulse_pin_;
        dir_pin = dir_pin_;
        alarm_pin = alarm_pin_;
        left_stepper_switch_pin = left_stepper_switch_pin_;
        right_stepper_switch_pin = right_stepper_switch_pin_;
        steps_per_degree = steps_per_degree_;
        center_step_offset = center_step_offset_;

        pinMode(left_stepper_switch_pin, INPUT_PULLUP);
        pinMode(right_stepper_switch_pin, INPUT_PULLUP);

        pinMode(alarm_pin, INPUT_PULLUP);

        pinMode(pulse_pin, OUTPUT);
        pinMode(dir_pin, OUTPUT);

        pulse_timer.begin(pulse_interrupt_handler, 50);
        pulse_timer.priority(255);
    }

    static void set_goal_step(int step) {
        static int last_dir = 0;

        cli();

        goal_position = step;

        int dir = 0;
        if (current_position < goal_position) {
            digitalWrite(dir_pin, LOW);
            dir = 1;
        } else if (current_position > goal_position) { 
            digitalWrite(dir_pin, HIGH);
            dir = -1;
        }

        if (last_dir != dir && dir != 0) {
            // Delay if we needed to change directions
            delayMicroseconds(5);
        }

        sei();

        last_dir = dir;
    }

    void set_goal_angle(float degrees)
    {
        set_goal_step(steps_per_degree * (degrees+temporary_offset));
    }

    void set_offset(float degrees) 
    {
        temporary_offset = degrees+temporary_offset;
    }

    int32_t load_offset() {
        uint32_t value = (
            (EEPROM.read(0)) |
            (EEPROM.read(1) << 8) | 
            (EEPROM.read(2) << 16) | 
            (EEPROM.read(3) << 24));
        return (value==0xFFFFFFFF)?0:(int32_t)value;
    }


    /**
     * @brief The steering motor performs the calibration sequence by rotating to both of the limit switches,
     * and then updating the step counter.
     * This function must be called after init() but before moving the buggy
     */
    void calibrate()
    {
        // TODO: This only works the first time we start up
        // If we ever want to have a recalibrate button,
        // we need to check the current steering angle
        int goal = 0;

        Serial.println("Beginning calibration...");
        while (!at_left_limit())
        {
            delay(1);
            set_goal_step(++goal);
        }
        LEFT_STEPPER_LIMIT = goal;
        Serial.printf("Determined left limit (%d)\n", LEFT_STEPPER_LIMIT);

        while (!at_right_limit())
        {
            delay(1);
            set_goal_step(--goal);
        }
        RIGHT_STEPPER_LIMIT = goal;
        Serial.printf("Determined right limit (%d)\n", RIGHT_STEPPER_LIMIT);

        int offset = load_offset();
        goal_position -= offset;
        current_position -= offset;
        LEFT_STEPPER_LIMIT -= offset;
        RIGHT_STEPPER_LIMIT -= offset;

        Serial.println("Calibration finished.");

        // Center the steering again
        set_goal_angle(0.0);
    }

	void save_offsest(uint32_t offset) {
		EEPROM.write(0, (uint8_t)offset);
		EEPROM.write(1, (uint8_t)(offset >> 8));
		EEPROM.write(2, (uint8_t)(offset >> 16));
		EEPROM.write(3, (uint8_t)(offset >> 24));
	}

	void make_offset() {
		if(!offset_switch() && !offset_button()) return;
		int32_t offset = current_position;
        goal_position -= offset;
        current_position -= offset;
        LEFT_STEPPER_LIMIT -= offset;
        RIGHT_SETPPER_LIMIT -= offset;
        save_offset((uint32_t)offset);
	}

    float current_angle_degrees()
    {
        cli();
        int pos = current_position;
        sei();
        return pos / steps_per_degree;
    }

    float current_angle_rads() {
        return radians(current_angle_degrees());
    }

    bool avg_read(int pin, bool prev) {
        int read[10];
        for(int i=0;i<10;i++) {
            read[i] = digitalRead(pin);
        }
        
        bool fault = read[0];
        for(int i=1;i<10;i++) {
            if(fault ^ read[i]) return prev;
        }
        return fault;
    }
    /**
     * @brief Checks whether or not the "alarm pin" has been triggered.
     * The pin triggers once the stepper goes into overcurrent protection and is then unresponsive to any movement commands.
     */
    Status alarm_triggered()
    {
        // static to make sure it persists between calls
        static Status fault = steering::Status::no_alarm;
        static bool current_val = true;

        current_val = avg_read(alarm_pin, current_val);
        if (!current_val)
        {
            fault = steering::Status::alarm;
        }

        return fault;
    }

    int left_step_limit() { return LEFT_STEPPER_LIMIT; }
    int right_step_limit() { return RIGHT_STEPPER_LIMIT; }

} // namespace steering

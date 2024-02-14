#include "ros_comms.h"

#define USE_TEENSY_HW_SERIAL // Must be before <ros.h>
#define ROS_BAUD 1000000

#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <sensor_msgs/BatteryState.h>
#include <nav_msgs/Odometry.h>

namespace
{

    volatile float ROS_STEERING_ANGLE = 0.0;
    void update_steering(const std_msgs::Float64 &cmd_msg) { ROS_STEERING_ANGLE = cmd_msg.data; }

    volatile float ROS_BRAKE = 1.0;
    void update_brake(const std_msgs::Float64 &cmd_msg) { ROS_BRAKE = cmd_msg.data; }

} // anonymous namespace

namespace ros_comms
{

    ros::NodeHandle nh;
    ros::Subscriber<std_msgs::Float64> steer("buggy/input/steering", update_steering);
    ros::Subscriber<std_msgs::Float64> brake("buggy/input/brake", update_brake);

    sensor_msgs::BatteryState battery_msg;
    ros::Publisher battery("buggy/battery", &battery_msg);

    diagnostic_msgs::DiagnosticStatus dbg_msg;
    ros::Publisher debug("TeensyStateIn_T", &dbg_msg);

    nav_msgs::Odometry odometry_msg;
    ros::Publisher nand_nav("/NAND/nav/odom", &odometry_msg);

    void init()
    {
        nh.getHardware()->setBaud(ROS_BAUD);
        nh.initNode();
        nh.subscribe(steer);
        nh.subscribe(brake);
        nh.advertise(debug);
        nh.advertise(battery);
        nh.advertise(nand_nav);

        // The charging status as reported
        battery_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
        //  The battery health metric
        battery_msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
        // The battery chemistry
        battery_msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;

        // Set unused battery parameters
        battery_msg.temperature = NAN;     // Temperature in Degrees Celsius (If unmeasured NaN)
        battery_msg.current = NAN;         // Negative when discharging (A)  (If unmeasured NaN)
        battery_msg.charge = NAN;          // Current charge in Ah  (If unmeasured NaN)
        battery_msg.capacity = NAN;        // Capacity in Ah (last full capacity)  (If unmeasured NaN)
        battery_msg.design_capacity = NAN; // Capacity in Ah (design capacity)  (If unmeasured NaN)
        battery_msg.percentage = NAN;      // Charge percentage on 0 to 1 range  (If unmeasured NaN)
        battery_msg.present = true;        // True if the battery is present
        battery_msg.location = "Buggy";    // The location into which the battery is inserted. (slot number or plug)
        battery_msg.serial_number = "";    // The best approximation of the battery serial number
    }

    /**
     * @return float The angle (in degrees) to which the steering is being told to go by auton.
     * Zero is stright forward.
     * A positive value is to the left of center, a negative value is to the right.
     */
    float steering_angle()
    {
        return ROS_STEERING_ANGLE;
    }

    /**
     * @brief The brake status commanded by the auton.
     */
    brake::Status brake_status()
    {
        // TODO: check polarity of ROS_BRAKE so we can use it here
        return brake::Status::Rolling;
    }

    void publish_debug_info(Debug info)
    {
        diagnostic_msgs::KeyValue dbg_values[10];

        int next_value_index = 0;
        auto add_value = [&](const char *name, char buf[])
        {
            dbg_values[next_value_index].key = name;
            dbg_values[next_value_index].value = buf;
            ++next_value_index;
        };

        char c_rc_steering_angle[32];
        snprintf(c_rc_steering_angle, 32, "%f", info.rc_steering_angle);
        add_value("RC Steering Angle (Degrees)", c_rc_steering_angle);

        char c_steering_angle[32];
        snprintf(c_steering_angle, 32, "%f", info.steering_angle);
        add_value("Commanded Steering Angle (Degrees)", c_steering_angle);

        char c_operator_ready[32];
        snprintf(c_operator_ready, 32, "%d", (int)info.operator_ready);
        add_value("Operator Ready", c_operator_ready);

        char c_steering_alarm[32];
        snprintf(c_steering_alarm, 32, "%d", (int)info.steering_alarm);
        add_value("Steering Alarm", c_steering_alarm);

        char c_brake_command[32];
        snprintf(c_brake_command, 32, "%d", (int)info.brake_command);
        add_value("Brake Command Rolling", c_brake_command);

        char c_auto_steering[32];
        snprintf(c_auto_steering, 32, "%d", (int)info.use_autonomous_steering);
        add_value("Autonomous Steering Enabled", c_auto_steering);

        char c_uplink_link_quality[32];
        snprintf(c_uplink_link_quality, 32, "%d", (int)info.uplink_link_quality);
        add_value("RC Uplink Quality", c_uplink_link_quality);

        char c_nand_fix[32];
        snprintf(c_nand_fix, 32, "%d", (int)info.nand_fix);
        add_value("NAND Fix Type", c_nand_fix);

        dbg_msg.name = "Steering Teensy Log";
        dbg_msg.level = diagnostic_msgs::DiagnosticStatus::OK;
        dbg_msg.message = "buggy yeet";
        dbg_msg.values = &dbg_values[0];
        dbg_msg.values_length = next_value_index;
        debug.publish(&dbg_msg);

        battery_msg.voltage = info.battery_voltage;
        battery.publish(&battery_msg);
    }

    void publish_nand_odometry(double x, double y)
    {
        odometry_msg.pose.pose.position.x = x;
        odometry_msg.pose.pose.position.y = y;
        odometry_msg.pose.pose.position.z = 0.0;

        memset(odometry_msg.pose.covariance, 0, sizeof(odometry_msg.pose.covariance));

        odometry_msg.twist.twist.angular.x = 0.0;
        odometry_msg.twist.twist.angular.y = 0.0;
        odometry_msg.twist.twist.angular.z = 0.0;

        odometry_msg.twist.twist.linear.x = 0.0;
        odometry_msg.twist.twist.linear.y = 0.0;
        odometry_msg.twist.twist.linear.z = 0.0;

        memset(odometry_msg.twist.covariance, 0, sizeof(odometry_msg.twist.covariance));

        nand_nav.publish(&odometry_msg);
    }

    /**
     * @brief Calls the internal node handler's spinOnce() function.
     * This function needs to get called periodically.
     * Typically once every main loop().
     */
    void spin_once()
    {
        nh.spinOnce();
    }

} // namespace ros_telemetry
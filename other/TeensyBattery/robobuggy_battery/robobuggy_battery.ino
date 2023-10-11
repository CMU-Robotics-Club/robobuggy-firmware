/**
 * Robobuggy Battery Interface Firmware
 *  - based on the below example sketches
 * 
 * 1. rosserial ADC Example
 * 
 * Also see the ROS Battery State message definition for more reference
 * http://docs.ros.org/en/api/sensor_msgs/html/msg/BatteryState.html
 */

#define USE_TEENSY_HW_SERIAL
#define ROS_BAUD 4608000L

#include <math.h>

#include <ros.h>
#include <sensor_msgs/BatteryState.h>

ros::NodeHandle nh;
sensor_msgs::BatteryState battery_msg;
ros::Publisher p("BatteryIn_T", &battery_msg);

void setup()
{ 
  pinMode(13, OUTPUT);
  nh.getHardware()->setBaud(ROS_BAUD);
  nh.initNode();

  // The charging status as reported
  battery_msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  // The battery health metric
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
  
  nh.advertise(p);
}

//We average the analog reading to elminate some of the noise
float averageAnalog(int pin){
  float v=0;
  for(int i=0; i<4; i++) v+= analogRead(pin);
  return v/4;
}

void loop()
{
  // TODO: Update when specs of voltage divider are known
  battery_msg.voltage = averageAnalog(0) / 1024 * 3.3 * (5); 
  
  p.publish(&battery_msg);

  nh.spinOnce();
  delay(1000);
}

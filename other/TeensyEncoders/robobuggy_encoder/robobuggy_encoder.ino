/**
 * Robobuggy Encoder Interface Firmware
 *  - based on the below two example sketches
 * 
 * 1. Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *  
 * 2. rosserial ADC Example
 */

#define USE_TEENSY_HW_SERIAL
#define ROS_BAUD 4608000L

#include <ros.h>
#include "buggy/EncoderStatus.h"
#include <Encoder.h>

ros::NodeHandle nh;
buggy::EncoderStatus es;
ros::Publisher p("EncoderIn_T", &es);

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
//   avoid using pins with LEDs attached
Encoder left_wheel_encoder(3, 4);
Encoder right_wheel_encoder(5, 6);
Encoder front_wheel_encoder(7, 8);

void setup() {
  //Serial.begin(115200);
  pinMode(13, OUTPUT);
  
  nh.getHardware()->setBaud(ROS_BAUD);
  nh.initNode();
  nh.advertise(p);
}

void loop() {
  
  long left_position = left_wheel_encoder.read();
  long right_position = right_wheel_encoder.read();
  long front_position = front_wheel_encoder.read();

  es.left_wheel_encoder = left_position / ((double) 4.0 * 720);
  es.right_wheel_encoder = right_position / ((double) 4.0 * 720);
  es.front_wheel_encoder = front_position / ((double) 4.0 * 720);

  //Serial.printf("%ld, %f\n", left_position, es.left_wheel_encoder);

  p.publish(&es);
  nh.spinOnce();
  delay(10);
}

#include <Arduino.h>

#define RFM69_CS 10
#define RFM69_INT 36
#define RFM69_RST 37

#include "ArduinoCRSF.h"
#include "buggyradio.h"
#include "steering.h"
#include "rc.h"
#include "brake.h"
#include "ros_comms.h"

/* ============= */
/* Board Config  */
/* ============= */

// Teensy Pins
#define VOLTAGE_PIN 27

#define ROS_DEBUG_INTERVAL_MILLIS 100


void setup()
{
  Serial.begin(115200);
  pinMode(VOLTAGE_PIN, INPUT);

  rc::init();
  brake::init();
  steering::init();

  radio_init(RFM69_CS, RFM69_INT, RFM69_RST);

  delay(2000);
  steering::calibrate();
}

void loop()
{
  /* ================================================ */
  /* Handle RC/autonomous control of steering/braking */
  /* ================================================ */

  rc::update();

  float steering_command = rc::use_autonomous_steering() ? ros_comms::steering_angle() : rc::steering_angle();
  brake::Status brake_command = rc::operator_ready() ? brake::Status::Rolling : brake::Status::Stopped;

  steering::set_goal_angle(steering_command);
  if (steering::alarm_triggered()) {
    // Stop the buggy immediately if we lose steering
    brake_command = brake::Status::Stopped;
  }

  brake::set(brake_command);

  /* ========================== */
  /* Publish NAND odometry data */
  /* ========================== */

  static uint8_t nand_fix = 0xFF;
  if (radio_available()) {
    uint8_t buf[256] = { 0 };
    if (auto length = radio_receive(buf)) {
      Packet *p = (Packet *)buf;
      if (p->tag == GPS_X_Y) {
        Serial.printf("X: %lf Y: %lf T: %llu F: %u\n", p->gps_x_y.x, p->gps_x_y.y, p->gps_x_y.time, (unsigned)p->gps_x_y.fix);
        ros_comms::publish_nand_odometry(p->gps_x_y.x, p->gps_x_y.y);
        nand_fix = p->gps_x_y.fix;
      }
    }
  }

  /* ================= */
  /* ROS Debug Logging */
  /* ================= */

  // Log data to ROS 10 times per second
  static unsigned last_log = millis();
  unsigned time = millis();
  if (time - last_log > ROS_DEBUG_INTERVAL_MILLIS) {
    last_log = time;

    auto link_stats = rc::link_statistics();

    float battery_voltage = analogRead(VOLTAGE_PIN) / 1024.0 * 50.0;

    ros_comms::Debug debug {
      rc::steering_angle(),
      steering_command,
      battery_voltage,
      rc::operator_ready(),
      steering::alarm_triggered(),
      brake_command,
      rc::use_autonomous_steering(),
      link_stats.uplink_Link_quality,
      nand_fix,
    };

    ros_comms::publish_debug_info(debug);
  }

  ros_comms::spin_once();
  delay(1);
}

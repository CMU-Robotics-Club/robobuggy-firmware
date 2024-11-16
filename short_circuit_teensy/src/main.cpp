#include <Arduino.h>

#define RFM69_CS 10
#define RFM69_INT 36
#define RFM69_RST 37

#include "ArduinoCRSF.h"
#include "buggyradio.h"
#include "steering.h"
#include "rc.h"
#include "brake.h"
#include "host_comms.h"
#include "status_led.h"

/* ============= */
/* Board Config  */
/* ============= */

// Teensy Pins
#define VOLTAGE_PIN 27

#define ROS_DEBUG_INTERVAL_MILLIS 100

#define RC_SERIAL Serial6

#define STEERING_PULSE_PIN 27             // pin for stepper pulse
#define STEERING_DIR_PIN 38             // pin for stepper direction
#define STEERING_ALARM_PIN 40
#define LIMIT_SWITCH_RIGHT_PIN 8
#define LIMIT_SWITCH_LEFT_PIN 7
#define CENTER_STEP_OFFSET 350

#define STATUS_LED_PIN 19

// Start with steps per revolution of the stepper,
// divide by 360 to get steps per degree of the stepper,
// multiply by the gear ratio to get steps per degree of the gearbox,
// and finally multiply by the belt ratio to get steps per degree of the wheel.
const float STEPS_PER_DEGREE = (1000.0 / 360.0) * 10.0 * (34.0 / 18.0);

#define BRAKE_RELAY_PIN 26

void setup()
{
  Serial.begin(115200);
  if (CrashReport) {
    Serial.print(CrashReport);
  }

  host_comms::init();

  pinMode(VOLTAGE_PIN, INPUT);

  rc::init(RC_SERIAL);
  brake::init(BRAKE_RELAY_PIN);
  steering::init(STEERING_PULSE_PIN, STEERING_DIR_PIN, STEERING_ALARM_PIN, LIMIT_SWITCH_LEFT_PIN, LIMIT_SWITCH_RIGHT_PIN, STEPS_PER_DEGREE, CENTER_STEP_OFFSET);
  status_led::init(STATUS_LED_PIN);

  radio_init(RFM69_CS, RFM69_INT, RFM69_RST);

  delay(2000);
  steering::calibrate();
}

using status_led::Rgb;
using host_comms::AlarmStatus;

#define LOG_COUNT 100

template<typename T, size_t Count> class History {
public:
  History() : length(0) {}

  void push(T t) {
    if (length >= Count) {
      // History is full, discard an element 
      for (size_t i = 0; i < Count - 1; ++i) {
        data[i] = std::move(data[i + 1]);
      }
      data[Count - 1] = std::move(t);
    } else {
      data[length++] = std::move(t);
    }
  }

  T max() {
    if (length == 0) {
      return {};
    } else {
      T m = data[0];
      for (size_t i = 1; i < length; ++i) {
        if (m < data[i]) {
          m = data[i];
        }
      }
      return m;
    }
  }

  double avg() {
    double a = 0.0;
    for (size_t i = 0; i < length; ++i) {
      a += data[i];
    }
    return a / length;
  }

private:
  T data[Count];
  size_t length;
};

void loop()
{
  static History<uint32_t, LOG_COUNT> loop_time {};
  /* ================================================ */
  /* Handle RC/autonomous control of steering/braking */
  /* ================================================ */

  uint32_t loop_start = millis();

  rc::update();

  host_comms::poll();

  Rgb blue   = { 0x00, 0x00, 0xFF };
  Rgb orange = { 0xFF, 0x80, 0x00 };
  Rgb red    = { 0xFF, 0x00, 0x00 };
  Rgb black  = { 0x00, 0x00, 0x00 };
  Rgb green  = { 0x00, 0xFF, 0x00 };

  // Orange by default
  Rgb status_color = orange;

  float steering_command = rc::use_autonomous_steering() ? host_comms::steering_angle() : rc::steering_angle();
  steering::set_goal_angle(steering_command);

  if (rc::temp_offset_switch()) steering::set_offset(rc::steering_angle()); 

  if (rc::use_autonomous_steering()) {
    if (host_comms::message_age() > 1000) {
      // We're in autonomous but we haven't recently received any messages, very concerning!
      // Solid red
      status_color = { 0xFF, 0x00, 0x00 };
    } else {

      switch (host_comms::alarm_status()) {
      case AlarmStatus::Ok:
        // Blue
        status_color = { 0x00, 0x00, 0xFF };
        break;
      case AlarmStatus::Warning:
        // Blink blue/orange
        status_color = ((millis() % 1000) < 500) ? blue : orange;
        break;
      case AlarmStatus::Error:
        // Blink blue/red
        status_color = ((millis() % 1000) < 500) ? blue : red;
        break;
      default:
        break;
      }
    }
  } else {
    // Teleop mode

    if (host_comms::message_age() <= 1000) {
      switch (host_comms::alarm_status()) {
      case AlarmStatus::Ok:
        // Green
        status_color = green;
        break;
      case AlarmStatus::Warning:
        // Blink green/orange
        status_color = ((millis() % 1000) < 500) ? green : orange;
        break;
      case AlarmStatus::Error:
        // Blink green/red
        status_color = ((millis() % 1000) < 500) ? green : red;
        break;
      default:
        break;
      }
    }
  }
  //Serial.printf("alarm: %i, connect: %i | ",steering::alarm_triggered(),!rc::connected());
  if (steering::alarm_triggered() || !rc::connected()) {
    // Blink red really fast, we have lost steering
    status_color = ((millis() % 300) < 150) ? red : black;
  }

  status_led::set_color(status_color);

  brake::Status brake_command = brake::Status::Stopped;
  //Serial.printf("operator: %i, alarm: %i\n",rc::operator_ready(),!steering::alarm_triggered());
  if (rc::operator_ready() && !steering::alarm_triggered()) {
    // Only roll if:
    // 1. The person holding the controller is holding down the buttons actively
    // 2. The steering servo is still working
    brake_command = brake::Status::Rolling;
  }

  brake::set(brake_command);

  /* ========================== */
  /* Publish NAND odometry data */
  /* ========================== */

  static int last_time = 0;

  static uint8_t nand_fix = 0xFF;
  if (radio_available()) {
    uint8_t buf[256] = { 0 };
    // todo: comments
    if (std::optional<uint8_t> length = radio_receive(buf)) {
      Packet *p = (Packet *)buf;
      if (p->tag == GPS_X_Y) {
        Serial.printf("S: %u X: %lf Y: %lf T: %u F: %u\n", p->seq, p->gps_x_y.x, p->gps_x_y.y, p->gps_x_y.gps_seq, (unsigned)p->gps_x_y.fix);
        Serial.printf("RSSI: %i dBm\n", (int)radio_last_rssi());
        host_comms::send_nand_odometry(p->gps_x_y.x, p->gps_x_y.y, p->seq, p->gps_x_y.gps_seq);
        nand_fix = p->gps_x_y.fix;

        if (last_time + 1 != p->seq) {
          Serial.printf("====== MISSED %d PACKETS =========\n", p->seq - last_time + 1);
        }
        last_time = p->seq;
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

    host_comms::DebugInfo info {
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

    host_comms::send_debug_info(info);
  }

  /*
  uint32_t loop_dur = millis() - loop_start;

  loop_time.push(loop_dur);

  static int x = 0;
  if (++x >= 10000) {
    x = 0;

    Serial.printf("LOOP TIME: %f\n", loop_time.avg());
    Serial.printf("MAX LOOP TIME: %u\n", loop_time.max());
  }
  */
}

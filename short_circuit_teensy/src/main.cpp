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
#define STATUS_LED_PIN 19

/**
 * @brief The number of stepper steps in one degree.
 * 
 * To calculate: start with steps per revolution of the stepper,
 * divide by 360 to get steps per degree of the stepper,
 * multiply by the gear ratio to get steps per degree of the gearbox,
 * and finally multiply by the belt ratio to get steps per degree of the wheel.
 */
const float STEPS_PER_DEGREE = (1000.0 / 360.0) * 10.0 * (34.0 / 18.0);

#define BRAKE_RELAY_PIN 26

/**
 * @brief Class for ensuring periodic tasks requiring different periods 
 * get executed when they're supposed to.
 * 
 * aka "rtos kernel at home"
 */
class RateLimit {
public:
  int period; // milliseconds

  RateLimit(int _period) : period(_period), last_time(millis()) {}

  bool ready() {
    int cur_time = millis();
    if (cur_time - last_time > period) {
      last_time = cur_time;
      return true;
    } else {
      return false;
    }
  }
  
  void reset() {
    last_time = millis();
  }

private:
  int last_time = millis();
};

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
  steering::init(STEERING_PULSE_PIN, STEERING_DIR_PIN, STEERING_ALARM_PIN, LIMIT_SWITCH_LEFT_PIN, LIMIT_SWITCH_RIGHT_PIN, STEPS_PER_DEGREE);
  status_led::init(STATUS_LED_PIN);

  radio_init(RFM69_CS, RFM69_INT, RFM69_RST);

  delay(2000);
  steering::calibrate();
}

using status_led::Rgb;
using host_comms::AlarmStatus;

#define LOG_COUNT 100

/**
 * @brief Helper class for storing and analyzing data
 * 
 * aka "circular buffer at home"
 */
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

/**
 * @brief The rate at which debug packets are sent to software.
 */
RateLimit debug_pkt_send_rate {100};
/**
 * @brief The rate at which sensor packets are sent to software.
 */
RateLimit sensor_pkt_send_rate {50};
/**
 * @brief The rate at which timing packets are sent to software.
 */
RateLimit timing_packet_send_rate{100};

elapsedMicros elapsed_loop_micros;
void loop()
{
  elapsed_loop_micros = 0;

  /* ================================================ */
  /* Handle RC/autonomous control of steering/braking */
  /* ================================================ */

  rc::update();

  host_comms::poll();

  Rgb blue   = { 0x00, 0x00, 0xFF };
  Rgb orange = { 0xFF, 0x80, 0x00 };
  Rgb red    = { 0xFF, 0x00, 0x00 };
  Rgb black  = { 0x00, 0x00, 0x00 };
  Rgb green  = { 0x00, 0xFF, 0x00 };

  // Orange by default
  Rgb status_color = orange;

  float rc_ang = rc::steering_angle();

  float steering_command = rc::use_autonomous_steering() ? host_comms::steering_angle() : rc_ang;
  steering::set_goal_angle(steering_command);
  if (millis() % 1000 == 0) Serial.printf("STEERING COMMAND %f\n", STEPS_PER_DEGREE*steering_command);

  if (rc::offset_button() && rc::offset_switch()) steering::update_offset();

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
  if ((steering::alarm_triggered()==steering::Status::alarm) || !rc::connected()) {
    // Blink red really fast, we have lost steering
    status_color = ((millis() % 300) < 150) ? red : black;
  }

  status_led::set_color(status_color);

  brake::Status brake_command = brake::Status::Stopped;
  //Serial.printf("operator: %i, alarm: %i\n",rc::operator_ready(),!steering::alarm_triggered());
  if (rc::operator_ready() && !(steering::alarm_triggered()==steering::Status::alarm)) {
    // Only roll if:
    // 1. The person holding the controller is holding down the buttons actively
    // 2. The steering servo is still working
    brake_command = brake::Status::Rolling;
  }

  brake::set(brake_command);

  /* =============================== */
  /* Receive Radio Packets from NAND */
  /* =============================== */

  /**
   * @brief The index of the most-recently-received radio packet from NAND
   * (Packets sent from NAND are all marked with an index)
   */
  static int prev_radio_pkt_idx = 0;
  /**
   * @brief The quantity of radio packets from NAND we have failed to receive.
   */
  // TODO: what if SC is powered on after NAND has alr sent a bunch of packets?
  // Will this value count packets sent while SC is off as missed?
  static int num_radio_pkts_missed = 0;

  static uint8_t nand_fix = 0xFF;
  if (radio_available()) {
    uint8_t buf[256] = { 0 }; // buffer of data received from NAND by the radio
    if (std::optional<uint8_t> length = radio_receive(buf)) {
      Packet *p = (Packet *)buf;
      if (p->tag == GPS_X_Y) {
        //Serial.printf("S: %u X: %lf Y: %lf T: %u F: %u\n", p->seq, p->gps_x_y.x, p->gps_x_y.y, p->gps_x_y.gps_seq, (unsigned)p->gps_x_y.fix);
        //Serial.printf("RSSI: %i dBm\n", (int)radio_last_rssi());
        host_comms::SCRadioRx nand_pkt;
        nand_pkt.nand_east = p->gps_x_y.x;
        nand_pkt.nand_north = p->gps_x_y.y;
        nand_pkt.gps_seq = p->gps_x_y.gps_seq;
        nand_pkt.nand_fix = p->gps_x_y.fix;
        host_comms::sc_send_nand_pos(nand_pkt);
        //host_comms::send_nand_odometry(p->gps_x_y.x, p->gps_x_y.y, p->seq, p->gps_x_y.gps_seq);
        //nand_fix = p->gps_x_y.fix;

        if (prev_radio_pkt_idx + 1 != p->seq) {
          Serial.printf("====== MISSED %d PACKETS =========\n", p->seq - prev_radio_pkt_idx + 1);
          num_radio_pkts_missed = p->seq - prev_radio_pkt_idx + 1;
        }
        prev_radio_pkt_idx = p->seq;
      }
    }
  }
  /* ================= */
  /* ROS Debug Logging */
  /* ================= */

  if(debug_pkt_send_rate.ready()) {
    host_comms::SCDebugInfo debug_pkt;
    debug_pkt.encoder_pos = 0.0f;
    debug_pkt.true_stepper_position = steering::current_angle_degrees();
    debug_pkt.brake_status = brake_command;
    debug_pkt.rc_steering_angle = rc_ang;
    debug_pkt.steering_angle = host_comms::steering_angle();
    debug_pkt.operator_ready = rc::operator_ready();
    debug_pkt.rc_uplink_quality = rc::link_statistics().uplink_Link_quality;
    debug_pkt.stepper_alarm = steering::alarm_triggered();
    debug_pkt.tx12_connected = rc::connected();
    debug_pkt.use_auton_steering = rc::use_autonomous_steering();
    debug_pkt.brake_status = brake::state();
    debug_pkt.missed_packets = num_radio_pkts_missed;
    debug_pkt.timestamp = millis();
    host_comms::sc_send_debug_info(debug_pkt);
  }

  if(sensor_pkt_send_rate.ready()) {
    host_comms::SCSensors sensor_pkt;
    sensor_pkt.front_speed = 0.0f;
    sensor_pkt.true_stepper_position = steering::current_angle_degrees();
    sensor_pkt.timestamp = millis();
    host_comms::sc_send_sensors(sensor_pkt);
  }

  // software timestamp
  if(timing_packet_send_rate.ready()) {
    host_comms::Roundtrip timing_pkt;
    timing_pkt.time = millis();
    timing_pkt.soft_time = host_comms::software_time();
    timing_pkt.cycle_time = (int64_t) elapsed_loop_micros;
    host_comms::send_timestamp(timing_pkt);
  }
  if (elapsed_loop_micros > 3000) {
    Serial.printf("Long cycle time (microseconds): %lu\n", (int64_t)elapsed_loop_micros);
  }

}

/*
  Read NMEA sentences over I2C using Ublox module SAM-M8Q, NEO-M8P, etc
  By: Nathan Seidle
  SparkFun Electronics
  Date: August 22nd, 2018
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example reads the NMEA characters over I2C and pipes them to MicroNMEA
  This example will output your current long/lat and satellites in view

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  ZED-F9P RTK2: https://www.sparkfun.com/products/15136
  NEO-M8P RTK: https://www.sparkfun.com/products/15005
  SAM-M8Q: https://www.sparkfun.com/products/15106

  For more MicroNMEA info see https://github.com/stevemarple/MicroNMEA

  Hardware Connections:
  Plug a Qwiic cable into the GPS and a BlackBoard
  If you don't have a platform with a Qwiic connection use the SparkFun Qwiic Breadboard Jumper (https://www.sparkfun.com/products/14425)
  Open the serial monitor at 115200 baud to see the output
  Go outside! Wait ~25 seconds and you should see your lat/long
*/

#include "gps.h"
#include "buggyradio.h"
#include "steering.h"
#include "rc.h"
#include "brake.h"
#include "encoder.h"
#include "sd_logging.h"
#include "host_comms.h"
#include "status_led.h"
#include "ukf.h"

#include <Arduino.h>
#include <Wire.h> //Needed for I2C to GPS

#include <Adafruit_BNO08x.h>

using status_led::Rgb;

#define RFM69_CS 10
#define RFM69_INT 36
#define RFM69_RST 37

#define RC_SERIAL Serial6
#define BRAKE_RELAY_PIN 26

#define STEERING_PULSE_PIN 27             // pin for stepper pulse
#define STEERING_DIR_PIN 38             // pin for stepper direction
#define STEERING_ALARM_PIN 39
#define LIMIT_SWITCH_RIGHT_PIN 7
#define LIMIT_SWITCH_LEFT_PIN 8
#define CENTER_STEP_OFFSET 0

// Start with steps per revolution of the stepper,
// divide by 360 to get steps per degree of the stepper,
// multiply by the gear ratio to get steps per degree of the gearbox,
// and finally multiply by the belt ratio to get steps per degree of the wheel.
const float STEPS_PER_DEGREE = (1000.0 / 360.0) * 10.0 * (32.0 / 15.0);

#define BNO_085_INT 20

#define RADIO_TX_PERIOD_MS 100

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

//#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_u-blox_GNSS
//SFE_UBLOX_GPS myGPS;

/**  @file

 @brief Universal Transverse Mercator transforms.

 Functions to convert (spherical) latitude and longitude to and
 from (Euclidean) UTM coordinates.

 @author Chuck Gantz- chuck.gantz@globalstar.com
 */

#include <cmath>
#include <stdio.h>
#include <stdlib.h>
// #include "ofMathConstants.h"

//@requires power>=0;
uint64_t positivePow(uint64_t base, uint64_t power)
{

  uint64_t result = 1;
  while (power > 0)
  {
    result *= base;
    power -= 1;
  }
  return result;
}

void setReports(void) {
  Serial.println("Setting desired reports");
  /*if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
    Serial.println("Could not enable accelerometer");
  }*/
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.println("Could not enable gyroscope");
  }
  /*if (!bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
    Serial.println("Could not enable magnetic field calibrated");
  }
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
    Serial.println("Could not enable linear acceleration");
  }
  if (!bno08x.enableReport(SH2_GRAVITY)) {
    Serial.println("Could not enable gravity vector");
  }
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
    Serial.println("Could not enable rotation vector");
  }
  if (!bno08x.enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR)) {
    Serial.println("Could not enable geomagnetic rotation vector");
  }
  if (!bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
    Serial.println("Could not enable game rotation vector");
  }
  if (!bno08x.enableReport(SH2_RAW_ACCELEROMETER)) {
    Serial.println("Could not enable raw accelerometer");
  }
  if (!bno08x.enableReport(SH2_RAW_GYROSCOPE)) {
    Serial.println("Could not enable raw gyroscope");
  }
  if (!bno08x.enableReport(SH2_RAW_MAGNETOMETER)) {
    Serial.println("Could not enable raw magnetometer");
  }
  */
}

#define STATUS_LED_PIN 16

void setup()
{
  Serial.begin(115200);
  Serial.println("NAND Booting Up!");

  // Workaround to set the status LED pin as an output
  pinMode(29, OUTPUT);
  digitalWrite(29, LOW);

  pinMode(STATUS_LED_PIN, OUTPUT);

  if (CrashReport) {
    Serial.print(CrashReport);
  }

  rc::init(RC_SERIAL);
  brake::init(BRAKE_RELAY_PIN);
  steering::init(STEERING_PULSE_PIN, STEERING_DIR_PIN, STEERING_ALARM_PIN, LIMIT_SWITCH_LEFT_PIN, LIMIT_SWITCH_RIGHT_PIN, STEPS_PER_DEGREE, CENTER_STEP_OFFSET);

  status_led::init(STATUS_LED_PIN);

  Wire.begin();
  Wire.setClock(400000);

  while (!bno08x.begin_I2C()) {
    Serial.println("BNO085 not detected over I2C. Retrying...");
    delay(1000);
  }

  //setReports();

  gps_init();

  radio_init(RFM69_CS, RFM69_INT, RFM69_RST);

  host_comms::init();

  encoder::init();

  // sd_logging::init();

  delay(1000);

  steering::calibrate();
}

bool led_state = false;

uint64_t last_gps_time = 0;
uint64_t last_local_time = millis();

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

void serial_log(int time_ms, double speed_mps, double steering_rad, state_vector_t state_est, state_cov_matrix_t state_cov) {
  Serial.printf("%9.3f ", time_ms / 1000.0);
  Serial.printf("% 6.3f ", speed_mps);
  Serial.printf("% 6.3f ", degrees(steering_rad));
  Serial.printf("% 12.3f ", state_est(0, 0));
  Serial.printf("% 12.3f ", state_est(1, 0));
  Serial.printf("% 7.3f ", degrees(state_est(2, 0)));
  Serial.printf("%6.3e ", state_cov(0, 0));
  Serial.printf("%6.3e ", state_cov(1, 1));
  Serial.printf("%6.3e ", state_cov(2, 2));
  Serial.println();
}

void loop()
{
  #if 0
  int fileNum = 0;
  char fileName[100];
  while (true) {
    snprintf(fileName, 100, "log%d.csv", fileNum);

    if (!SD.exists(fileName)) {
      break;
    }

    ++fileNum;
  }
  File f = SD.open(fileName, FILE_WRITE);
  
  if (!f) {
    while (1)
      Serial.println("File not created. Freezing");
  }

  f.printf("------- BEGIN NEW LOG --------\n");

  #endif

  /*             CSV Format                  */
  /* timestamp, type of data, <rest of data> */

  RateLimit radio_tx_limit { 200 };

  GpsUpdate last_gps_data { 0 };
  bool fresh_gps_data = false;
  int gps_sequence_number = 0;

  History<uint32_t, 10> gps_time_history {};
  History<uint32_t, 10> imu_update_history {};
  History<uint32_t, 10> radio_send_history {};

  RateLimit imu_poll_limit { 5 };
  
  RateLimit debug_limit { 50 };
  RateLimit bnya_telem_limit { 10 };

  Rgb  dark_green = { 0x00, 0xD0, 0x00 };
  Rgb light_green = { 0x00, 0xFF, 0x20 };

  Rgb  dark_red = { 0xD0, 0x00, 0x00 };
  Rgb light_red = { 0xFF, 0x20, 0x00 };

  Rgb black = { 0x00, 0x00, 0x00 };

  UKF filter(
    // Wheelbase (meters)
    1.2,
    // Zeroth sigma point weight
    1.0 / 3.0,
    // Process noise,
    state_cov_matrix_t{
        {0.0001, 0.0, 0.0},
        {0.0, 0.0001, 0.0},
        {0.0, 0.0, 0.01}},
    // GPS noise,
    measurement_cov_matrix_t{
        {0.01, 0.0},
        {0.0, 0.01}});
  bool kalman_init = false;
  uint32_t last_predict_timestamp;

  double heading_rate = 0.0;

  while (1) {
    unsigned long loop_update_elapsed_ms = millis();
    bool debug_time = debug_limit.ready();
    host_comms::NANDDebugInfo debug_packet;
    /* ================================================ */
    /* Handle RC/autonomous control of steering/braking */
    /* ================================================ */

    Rgb rgb;
    if (kalman_init) {
      rgb = ((millis() % 1000) > 500) ? dark_green : light_green;
    } else {
      rgb = Rgb { 0x80, 0x80, 0x00 };
    }

    rc::update();

    host_comms::poll();

    float steering_command = rc::use_autonomous_steering() ? host_comms::steering_angle() : rc::steering_angle();
    steering::set_goal_angle(steering_command);

    if (rc::temp_offset_switch()) steering::set_offset(rc::steering_angle()); 

    brake::Status brake_command = brake::Status::Stopped;
    if (rc::operator_ready() && !(steering::alarm_triggered()==steering::Status::alarm)) {
      // Only roll if:
      // 1. The person holding the controller is holding down the buttons actively
      // 2. The steering servo is still working
      brake_command = brake::Status::Rolling;

      if (rc::use_autonomous_steering()) {
        rgb = Rgb { 0x00, 0x00, 0xFF };
      }
    }
    brake::set(brake_command);
    if(debug_time) {
      debug_packet.brake_status = brake_command;
      debug_packet.rc_steering_angle = rc::steering_angle();
      debug_packet.steering_angle = host_comms::steering_angle();
      debug_packet.true_stepper_pos = steering::current_angle_degrees();
      debug_packet.operator_ready = rc::operator_ready();
      debug_packet.use_auton_steering = rc::use_autonomous_steering();
      debug_packet.tx12_connected = rc::connected();
      debug_packet.rc_uplink = rc::link_statistics().uplink_Link_quality;
      debug_packet.steering_alarm = steering::alarm_triggered();
      debug_packet.timestamp = millis();
    }
    /*if (debug_limit.ready()) {
      auto link_stats = rc::link_statistics();

      host_comms::DebugInfo info {
        rc::steering_angle(),
        steering::current_angle_degrees(),
        0.0,
        rc::operator_ready(),
        steering::alarm_triggered(),
        brake_command,
        rc::use_autonomous_steering(),
        link_stats.uplink_Link_quality,
        0
      };
      host_comms::send_debug_info(info);
    }*/

    uint32_t cur_time = micros();
    double dt = ((double)(cur_time - last_predict_timestamp)) / 1e6;
    filter.set_speed(encoder::rear_speed(steering::current_angle_degrees()));
    int i2c_time = encoder::prev_time_millis();
    if(i2c_time>=5) {
      Serial.printf("First encoder time: %d\n",i2c_time);
    }
    if (kalman_init) {
      filter.predict(input_vector_t{steering::current_angle_rads()}, dt);
    }
    last_predict_timestamp = cur_time;

    int gps_t1 = millis();
    elapsedMillis gps_update_elapsed = {};
    if (auto gps_coord = gps_update()) {
      int gps_tF = millis()-gps_t1;
      if(gps_tF>=5){
        Serial.printf("GPS Time: %dms\n", gps_tF);
      }

      if (!kalman_init && gps_coord->accuracy < 50.0) {
        Serial.println("GPS accuracy OK, initializing filter");
        filter.curr_state_est(0, 0) = gps_coord->x;
        filter.curr_state_est(1, 0) = gps_coord->y;
        filter.curr_state_est(2, 0) = -M_PI_2;

        kalman_init = true;
      }

      last_gps_data = *gps_coord;
      fresh_gps_data = true;
      ++gps_sequence_number;

      gps_time_history.push(gps_update_elapsed);
      if (kalman_init) {
        filter.set_gps_noise(gps_coord->accuracy);
        filter.update(measurement_vector_t{gps_coord->x, gps_coord->y});
      }

      serial_log(millis(), encoder::rear_speed(steering::current_angle_degrees()), steering::current_angle_rads(), filter.curr_state_est, filter.curr_state_cov);
      i2c_time = encoder::prev_time_millis();
      if(i2c_time>=5) {
        Serial.printf("Second encoder time :%d\n",i2c_time);
      }
      // serial_log(millis(), encoder::front_speed(), encoder::e_raw_angle(), filter.curr_state_est, filter.curr_state_cov);

      // Serial.printf("Maximum GPS update time: %d\n", gps_time_history.max());
      // Serial.printf("Average GPS update time: %f\n", gps_time_history.avg());
    }
    else
    {
      int gps_tF = millis()-gps_t1;
      if(gps_tF>=5){
        Serial.printf("GPS not resolved: %dms\n", gps_tF);
      }
    }

    /*if (bnya_telem_limit.ready()) {
      host_comms::send_bnya_telemetry(
        filter.curr_state_est(0, 0), filter.curr_state_est(1, 0),
        encoder::rear_speed(steering::current_angle_degrees()),
        steering::current_angle_degrees(),
        filter.curr_state_est(2, 0),
        heading_rate
      );
    }*/

    i2c_time = encoder::prev_time_millis();
    if(i2c_time>=5) {
      Serial.printf("Third encoder time: %d\n",i2c_time);
    }

    static int last_failed = millis();

    int t1 = millis();

    elapsedMillis radio_send_elapsed = {};
    if (radio_tx_limit.ready() || fresh_gps_data) {
      fresh_gps_data = false;
      radio_tx_limit.reset();
      int radio_t1 = millis();
      if (!radio_send_gps(last_gps_data.x, last_gps_data.y, gps_sequence_number, last_gps_data.fix)) {
        int radio_tF = millis() - radio_t1;
        if(radio_tF>5) Serial.printf("Radio failed: %d\n",radio_tF);
        last_failed = millis();
      }
      int radio_tF = millis() - radio_t1;
        if(radio_tF>5) Serial.printf("Radio success: %d\n",radio_tF);

      radio_send_history.push(radio_send_elapsed);

      static int aaa =0;
      ++aaa;

      /*
      Serial.printf("SEQ: %d\n", gps_sequence_number);
      Serial.printf("SEQ      : %d\n", aaa);
      Serial.printf("Maximum radio send time: %d\n", radio_send_history.max());
      Serial.printf("Average radio send time: %f\n", radio_send_history.avg());
      */
    }

    int tF = millis()-t1;
    if(tF>=5){
      Serial.printf("Time took for Radio is %dms\n", tF);
    }



    if (millis() - last_failed < 300) {
      rgb = ((millis() % 500) > 250) ? dark_red : light_red;
    }

    if (!rc::connected() || (steering::alarm_triggered()==steering::Status::alarm)) {
      rgb = ((millis() % 500) > 250) ? dark_red : black;
    }

    if (imu_poll_limit.ready()) {
      if (bno08x.wasReset()) {
        Serial.print("sensor was reset ");
        setReports();
      }

      elapsedMillis imu_update_elapsed = {};
      int imu_t1 = millis();
      if (bno08x.getSensorEvent(&sensorValue)) {
        int imu_tF = millis() - imu_t1;
        if(imu_tF > 5) Serial.printf("IMU got event, time: %d\n",imu_tF);
        //Serial.println("Logging IMU event");

        if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
          double x = sensorValue.un.gyroscope.x;
          double y = sensorValue.un.gyroscope.y;
          double z = sensorValue.un.gyroscope.z;

          heading_rate = z;
          if(debug_time) debug_packet.heading_rate = heading_rate;
        }
      } else {
        int imu_tF = millis() - imu_t1;
        if(imu_tF > 5) Serial.printf("IMU no event, time: %d\n",imu_tF);
      }
    }

    status_led::set_color(rgb);

    unsigned long now_ms = millis();
    if (now_ms - loop_update_elapsed_ms > 10) {
      Serial.println(now_ms - loop_update_elapsed_ms);
    }

  }
}

// This function gets called from the SparkFun Ublox Arduino Library
// As each NMEA character comes in you can specify what to do with it
// Useful for passing to other libraries like tinyGPS, MicroNMEA, or even
// a buffer, radio, etc.

/*
void SFE_UBLOX_GPS::processNMEA(char incoming)
{
  // Take the incoming char from the Ublox I2C port and pass it on to the MicroNMEA lib
  // for sentence cracking
  nmea.process(incoming);
}
*/
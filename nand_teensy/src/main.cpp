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

#define RFM69_CS 10
#define RFM69_INT 36
#define RFM69_RST 37

#define RC_SERIAL Serial2
#define BRAKE_RELAY_PIN 22

#define WHEEL_RADIUS_METERS 0.09

#define STEERING_PULSE_PIN 23 // pin for stepper pulse
#define STEERING_DIR_PIN 38   // pin for stepper direction
#define STEERING_ALARM_PIN 21
#define LIMIT_SWITCH_RIGHT_PIN 32
#define LIMIT_SWITCH_LEFT_PIN 31

// Start with steps per revolution of the stepper,
// divide by 360 to get steps per degree of the stepper,
// multiply by the gear ratio to get steps per degree of the gearbox,
// and finally multiply by the belt ratio to get steps per degree of the wheel.
const float STEPS_PER_DEGREE = (1000.0 / 360.0) * 10.0 * (32.0 / 15.0);

#define BNO_085_INT 34

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

// #include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_u-blox_GNSS
// SFE_UBLOX_GPS myGPS;

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

/**
 * @brief Configure the IMU report type
 */
void setReports(void)
{
  Serial.println("Setting desired reports");
  /*if (!bno08x.enableReport(SH2_ACCELEROMETER)) {
    Serial.println("Could not enable accelerometer");
  }*/
  if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED))
  {
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

// From PaulStoffregen/OctoWS2811/examples/Teensy4_PinList:
// These buffers need to be large enough for all the pixels.
// The total number of pixels is "ledsPerStrip * numPins".
// Each pixel needs 3 bytes, so multiply by 3.  An "int" is
// 4 bytes, so divide by 4.  The array is created using "int"
// so the compiler will align it to 32 bit memory.
#define LEDS_PER_STRIP 20 // LEDs per strip
#define NUM_LED_PINS 2    // number of LED strips
#define BYTES_PER_LED 3   // change to 4 if using RGBW
static DMAMEM int displayMemory[LEDS_PER_STRIP * NUM_LED_PINS * BYTES_PER_LED / 4];
static int drawingMemory[LEDS_PER_STRIP * NUM_LED_PINS * BYTES_PER_LED / 4];

#define STATUS_LED_PIN1 40
#define STATUS_LED_PIN2 41
static byte pinList[NUM_LED_PINS] = {STATUS_LED_PIN1, STATUS_LED_PIN2};

static OctoWS2811 leds = OctoWS2811(LEDS_PER_STRIP, displayMemory, drawingMemory, WS2811_GRB | WS2811_800kHz, NUM_LED_PINS, pinList);

void setup()
{
  Serial.begin(115200);
  Serial.println("NAND Booting Up!");

  Serial.println("Encoder starting initalization");
  encoder::init();
  Serial.println("Encoder initalized");
  // Serial.printf("Diagnostic: %i\n",encoder::get_diagnostics());

  if (CrashReport)
  {
    Serial.print(CrashReport);
  }

  rc::init(RC_SERIAL);
  brake::init(BRAKE_RELAY_PIN);
  steering::init(STEERING_PULSE_PIN, STEERING_DIR_PIN, STEERING_ALARM_PIN, LIMIT_SWITCH_LEFT_PIN, LIMIT_SWITCH_RIGHT_PIN, STEPS_PER_DEGREE);

  pinMode(STATUS_LED_PIN1, OUTPUT);
  pinMode(STATUS_LED_PIN2, OUTPUT);
  status_led::init(&leds, LEDS_PER_STRIP, NUM_LED_PINS);

  encoder::init();

  // Configuration for I2C bus
  Wire.begin();
  Wire.setClock(400000);

  while (!bno08x.begin_I2C())
  {
    Serial.println("BNO085 not detected over I2C. Retrying...");
    delay(1000);
  }

  // setReports();

  gps_init();

  radio_init(RFM69_CS, RFM69_INT, RFM69_RST);

  host_comms::init();

  delay(1000);

  steering::calibrate();
}

bool led_state = false;

class RateLimit
{
public:
  int period; // milliseconds

  RateLimit(int _period) : period(_period), last_time(millis()) {}

  bool ready()
  {
    int cur_time = millis();
    if (cur_time - last_time > period)
    {
      last_time = cur_time;
      return true;
    }
    else
    {
      return false;
    }
  }

  void reset()
  {
    last_time = millis();
  }

private:
  int last_time = millis();
};

template <typename T, size_t Count>
class History
{
public:
  History() : length(0) {}

  void push(T t)
  {
    if (length >= Count)
    {
      // History is full, discard an element
      for (size_t i = 0; i < Count - 1; ++i)
      {
        data[i] = std::move(data[i + 1]);
      }
      data[Count - 1] = std::move(t);
    }
    else
    {
      data[length++] = std::move(t);
    }
  }

  T max()
  {
    if (length == 0)
    {
      return {};
    }
    else
    {
      T m = data[0];
      for (size_t i = 1; i < length; ++i)
      {
        if (m < data[i])
        {
          m = data[i];
        }
      }
      return m;
    }
  }

  double avg()
  {
    double a = 0.0;
    for (size_t i = 0; i < length; ++i)
    {
      a += data[i];
    }
    return a / length;
  }

private:
  T data[Count];
  size_t length;
};

/**
 * @brief Helper function for printing UKF data
 */
void serial_log(double speed_deg_per_sec, double steering_deg, state_vector_t state_est, state_cov_matrix_t state_cov)
{
  double heading = degrees(state_est(2, 0));
  heading += 360;
  fmod(heading, 360);
  heading += 360;

  Serial.printf("time: %9.3f sec ", millis() / 1000.0);
  Serial.printf("speed: % 6.3f ", speed_deg_per_sec);
  Serial.printf("steering: % 6.3f deg ", steering_deg);
  Serial.printf("x: % 12.3f ", state_est(0, 0));
  Serial.printf("y: % 12.3f ", state_est(1, 0));
  Serial.printf("heading: % 7.3f deg ", degrees(state_est(2, 0)));
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

  RateLimit radio_tx_limit{200};
  RateLimit control_latency_print_limit{50};

  /**
   * @brief Variable for storing the most recent data from the GPS
   */
  GpsUpdate last_gps_data{0};
  bool fresh_gps_data = false;
  /**
   * @brief Counter/index for tracking the number of GPS readings
   */
  int gps_sequence_number = 0;

  History<uint32_t, 10> gps_time_history{};
  History<uint32_t, 10> imu_update_history{};
  History<uint32_t, 10> radio_send_history{};

  RateLimit print_limit{100};
  RateLimit imu_poll_limit{5};
  RateLimit encoder_poll_limit{20};
  RateLimit timing_pkt_send_rate{100};
  RateLimit ukf_pkt_send_rate{10};
  RateLimit debug_pkt_send_rate{50};
  RateLimit gps_pkt_send_rate{250};

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
  uint32_t last_predict_timestamp; // the timestamp at which the UKF predict step was run most recently

  double heading_rate = 0.0;
  double speed_deg_per_sec = 0.0;

  elapsedMicros elapsed_loop_micros;

  while (1)
  {
    elapsed_loop_micros = 0;

    // Status LED
    status_led::Rgb rgb;
    if (kalman_init)
    {
      rgb = ((millis() % 1000) > 500) ? status_led::dark_green : status_led::light_green;
    }
    else
    {
      rgb = status_led::yellow;
    }

    rc::update();

    host_comms::poll();
    static int rfm69_timeout = 0;

    float steering_command = rc::use_autonomous_steering() ? host_comms::steering_angle() : rc::steering_angle();
    steering::set_goal_angle(steering_command);

    if (control_latency_print_limit.ready())
    {
      uint32_t control_latency = micros() - host_comms::ukf_steering_timestamp();
      // Serial.printf("UKF Timestamp: %lu\tControl latency: %lu\n", host_comms::ukf_steering_timestamp(), control_latency);
    }

    if (rc::offset_button() && rc::offset_switch())
      steering::update_offset();

    brake::Status brake_command = brake::Status::Stopped;
    if (rc::operator_ready() && !(steering::alarm_triggered() == steering::Status::alarm))
    {
      // Only roll if:
      // 1. The person holding the controller is holding down the buttons actively
      // 2. The steering servo is still working
      brake_command = brake::Status::Rolling;

      if (rc::use_autonomous_steering())
      {
        rgb = status_led::blue;
      }
    }
    brake::set(brake_command);

    elapsedMicros imu_elapsed_micros = 0;
    // IMU code
    if (imu_poll_limit.ready())
    {
      if (bno08x.wasReset())
      {
        Serial.print("sensor was reset ");
        setReports();
      }

      elapsedMillis imu_update_elapsed = {};
      int imu_t1 = millis();
      if (bno08x.getSensorEvent(&sensorValue))
      {
        int imu_tF = millis() - imu_t1;
        if (imu_tF > 5)
          Serial.printf("IMU got event, time: %d\n", imu_tF);
        // Serial.println("Logging IMU event");

        if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED)
        {
          double x = sensorValue.un.gyroscope.x;
          double y = sensorValue.un.gyroscope.y;
          double z = sensorValue.un.gyroscope.z;

          heading_rate = z;
        }
      }
      else
      {
        int imu_tF = millis() - imu_t1;
        if (imu_tF > 5)
          Serial.printf("IMU no event, time: %d\n", imu_tF);
      }
    }

    if (imu_elapsed_micros > 10)
    {
      // Serial.printf("IMU Time:\t %lu\n", (uint64_t)imu_elapsed_micros);
    }

    if (encoder_poll_limit.ready())
    {
      encoder::poll();
      long last_encoder_packet = encoder::lastPacket();
      if (last_encoder_packet > 100)
      {
        Serial.printf("Have not received encoder packet in %lu ms!\n", last_encoder_packet);
        speed_deg_per_sec = -1;
      }
      if (encoder::front_speed(&speed_deg_per_sec))
      {
        double speed_m_per_sec = radians(speed_deg_per_sec) * WHEEL_RADIUS_METERS;
        filter.set_speed(speed_m_per_sec);
      };
    }

    if (debug_pkt_send_rate.ready())
    {
      host_comms::NANDDebugInfo debug_packet;
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
      debug_packet.heading_rate = heading_rate;
      debug_packet.rfm69_timeout_cnt = rfm69_timeout;
      debug_packet.front_wheel_speed = speed_deg_per_sec;
      host_comms::nand_send_debug(debug_packet);
    }

    uint32_t cur_time = micros(); // timing variable for UKF
    double dt = ((double)(cur_time - last_predict_timestamp)) / 1e6;
    // filter.set_speed(encoder::rear_speed(steering::current_angle_degrees()));
    // int i2c_time = encoder::prev_time_millis();
    /*if(i2c_time>=5) {
      Serial.printf("First encoder time: %d\n",i2c_time);
    }*/
    if (kalman_init)
    {
      filter.predict(input_vector_t{steering::current_angle_rads()}, dt);
    }
    last_predict_timestamp = cur_time;

    elapsedMicros gps_update_elapsed = 0; // timer for timing how long reading from the GPS takes
    if (auto gps_coord = gps_update())
    {
      if (!kalman_init && gps_coord->accuracy < 50.0)
      {
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
      if (kalman_init)
      {
        filter.set_gps_noise(gps_coord->accuracy);
        filter.update(measurement_vector_t{gps_coord->x, gps_coord->y});
      }

      /*i2c_time = encoder::prev_time_millis();
      if(i2c_time>=5) {
        Serial.printf("Second encoder time :%d\n",i2c_time);
      }*/
      // serial_log(millis(), encoder::front_speed(), encoder::e_raw_angle(), filter.curr_state_est, filter.curr_state_cov);

      // Serial.printf("Maximum GPS update time: %d\n", gps_time_history.max());
      // Serial.printf("Average GPS update time: %f\n", gps_time_history.avg());
    }
    else
    {
      int gps_tF = gps_update_elapsed;
      if (gps_tF >= 5000)
      {
        Serial.printf("GPS not resolved: %dms\n", gps_tF);
      }
    }
    if (gps_pkt_send_rate.ready())
    {
      host_comms::NANDRawGPS raw_gps_packet;
      raw_gps_packet.eastern = last_gps_data.x;
      raw_gps_packet.northern = last_gps_data.y;
      raw_gps_packet.accuracy = last_gps_data.accuracy;
      raw_gps_packet.gps_SIV = last_gps_data.gps_SIV;
      raw_gps_packet.gps_seq_num = gps_sequence_number;
      raw_gps_packet.timestamp = millis();
      raw_gps_packet.gps_fix = last_gps_data.gps_fix;
      raw_gps_packet.rtk_fix = last_gps_data.rtk_fix;
      host_comms::nand_send_raw_gps(raw_gps_packet);
    }

    if (gps_update_elapsed > 4000)
    {
      // Serial.printf("GPS read and send:\t%lu\n", (uint64_t)gps_update_elapsed);
    }

    /*i2c_time = encoder::prev_time_millis();
    if(i2c_time>=5) {
      Serial.printf("Third encoder time: %d\n",i2c_time);
    }*/

    // TODO cleanup
    /**
     * @brief The timestamp (millis) at which the most recent radio send failure occured.
     */
    static int last_failed = millis();

    int t1 = millis();

    elapsedMicros radio_send_elapsed = 0;
    if (radio_tx_limit.ready() || fresh_gps_data)
    {
      fresh_gps_data = false;
      radio_tx_limit.reset();
      int radio_t1 = millis();
      if (!radio_send_gps(last_gps_data.x, last_gps_data.y, gps_sequence_number, last_gps_data.rtk_fix, (uint8_t)rc::use_autonomous_steering()))
      {
        int radio_tF = millis() - radio_t1;
        if (radio_tF > 5)
          Serial.printf("Radio failed: %d\n", radio_tF);
        last_failed = millis();
        ++rfm69_timeout;
      }
      else
      {
        // Serial.printf("Sent GPS data over RFM69 radio!\n");
      }

      radio_send_history.push(radio_send_elapsed);

      static int aaa = 0;
      ++aaa;

      /*
      Serial.printf("SEQ: %d\n", gps_sequence_number);
      Serial.printf("SEQ      : %d\n", aaa);
      Serial.printf("Maximum radio send time: %d\n", radio_send_history.max());
      Serial.printf("Average radio send time: %f\n", radio_send_history.avg());*/
    }

    // send bogus gps data over the RFM69 radio if we do not have fresh gps data!!
    if (radio_tx_limit.ready() && !fresh_gps_data)
    {
      radio_tx_limit.reset();
      int radio_t1 = millis();
      if (!radio_send_gps(0, 0, gps_sequence_number, 213, 0))
      {
        int radio_tF = millis() - radio_t1;
        Serial.printf("Radio failed: %d\n", radio_tF);
        last_failed = millis();
      }
      else
        Serial.printf("Sending fake GPS data over RFM69 radio.  Fresh GPS data not available.\n");

      radio_send_history.push(radio_send_elapsed);
    }

    int tF = millis() - t1;
    if (tF >= 5)
    {
      Serial.printf("Time took for Radio is %dms\n", tF);
    }

    if (millis() - last_failed < 300)
    {
      rgb = ((millis() % 500) > 250) ? status_led::dark_red : status_led::light_red;
    }

    if (!rc::connected() || (steering::alarm_triggered() == steering::Status::alarm))
    {
      rgb = ((millis() % 500) > 250) ? status_led::dark_red : status_led::black;
    }

    if (radio_send_elapsed > 10)
    {
      // Serial.printf("Radio send (us):\t%lu\n", (uint64_t)radio_send_elapsed);
    }

    // send all of the bnyah serial packets

    status_led::set_color(rgb);

    // send packet with UKF info
    if (ukf_pkt_send_rate.ready())
    {
      host_comms::NANDUKF ukf_packet;
      ukf_packet.eastern = filter.curr_state_est(0, 0);
      ukf_packet.northern = filter.curr_state_est(1, 0);
      ukf_packet.heading = filter.curr_state_est(2, 0);

      if (kalman_init)
      {
        ukf_packet.eastern_cov = filter.curr_state_cov(0, 0);
        ukf_packet.northern_cov = filter.curr_state_cov(1, 1);
        ukf_packet.heading_cov = filter.curr_state_cov(2, 2);
        ukf_packet.speed_cov = speed_deg_per_sec;
      }
      else
      {
        ukf_packet.eastern_cov = std::numeric_limits<double>::infinity();
        ukf_packet.northern_cov = std::numeric_limits<double>::infinity();
        ukf_packet.heading_cov = std::numeric_limits<double>::infinity();
        ukf_packet.speed_cov = std::numeric_limits<double>::infinity();
      }

      ukf_packet.heading_rate = heading_rate;
      ukf_packet.front_speed = speed_deg_per_sec; // filter.curr_state_est(3, 0);
      ukf_packet.timestamp = (uint32_t)micros();
      host_comms::nand_send_ukf(ukf_packet);
    }

    if (timing_pkt_send_rate.ready())
    {
      host_comms::Roundtrip rt_packet;
      rt_packet.time = millis();
      rt_packet.cycle_time = (int64_t)elapsed_loop_micros;
      rt_packet.soft_time = host_comms::software_time();
      host_comms::send_timestamp(rt_packet);
    }

    if (print_limit.ready())
    {
      serial_log(speed_deg_per_sec, steering::current_angle_degrees(), filter.curr_state_est, filter.curr_state_cov);
    }

    if (elapsed_loop_micros > 10000)
    {
      // Serial.printf("Cycle time (microseconds): %lu\n", (int64_t)elapsed_loop_micros);
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

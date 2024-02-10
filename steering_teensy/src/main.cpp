#include <Arduino.h>

#define RFM69_CS 10
#define RFM69_INT 36
#define RFM69_RST 37


#include "ArduinoCRSF.h"
#include "buggyradio.h"
#include "steering.h"
#include "rc.h"

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

/* ============= */
/* Board Config  */
/* ============= */

// Teensy Pins
#define VOLTAGE_PIN 27
#define BRAKE_RELAY_PIN 26
#define INTERRUPT_PIN 41

// Width of RC steering and brake pulses.
volatile int v_rcSteeringWidth = 0;
volatile int v_rcThrottleWidth = 0;

/**
 * @brief Timestamp of the most recent rising OR falling edge on the steering pin.
 */
volatile unsigned long rcSteeringLastEdge = 0;
/**
 * @brief Timestamp of the most recent rising OR falling edge on the brake pin.
 */
volatile unsigned long rcThrottleLastEdge = 0;

/**
 * @brief Timestamp of the most recent rising edge on the steering pin.
 */
unsigned long rcSteeringUptime = 0;
/**
 * @brief Timestamp of the most recent rising edge on the steering pin.
 */
unsigned long rcThrottleUptime = 0;

// For averaging the past 5 consecutive steering pulse widths.
#define RC_SAMPLING_WINDOW 5
int rcSteeringSamples[RC_SAMPLING_WINDOW] = {0};
int rcSteeringSampleIndex = 0;

// TODO make sure i implemented these interrupt handlers correctly lol

#if 0

/**
 * @brief This method gets called every time STEERING_PIN switches from low to high or from high to low.
 * It updates rcSteeringLastEdge to current timestamp.
 * If the change is a rising edge, rcSteeringUptime is also updated to the current time.
 * If the change is a falling edge, we are measuring the width of the last high pulse that just ended,
 * and storing that value in v_rcSteeringWidth.
 *
 */
void steeringInterruptHandler()
{
  rcSteeringLastEdge = millis();

  if (digitalRead(STEERING_PIN))
  {
    rcSteeringUptime = micros();
  }
  else
  {
    int width = micros() - rcSteeringUptime;
    if (10 <= width && width <= 2000)
    { // Filtering out blips and long pauses in the signal.
      v_rcSteeringWidth = width;
    }

    if (width > 2000)
    {
      digitalWrite(INTERRUPT_PIN, HIGH);
      digitalWrite(INTERRUPT_PIN, LOW);
    }
  }
}


/**
 * @brief See description for the steeringInterrupHandler method.
 */
void throttleInterruptHandler()
{

  rcThrottleLastEdge = millis();

  if (digitalRead(THROTTLE_PIN))
  { // The pin has changed from low to high, so we're resetting the uptime timer.
    rcThrottleUptime = micros();
  }
  else
  {
    int width = micros() - rcThrottleUptime;
    if (10 <= width && width <= 2000)
    { // Filtering out blips and long pauses in the signal.
      v_rcThrottleWidth = width;
    }

    if (width > 2000)
    {
      digitalWrite(INTERRUPT_PIN, HIGH);
      digitalWrite(INTERRUPT_PIN, LOW);
    }
  }
}

#endif


volatile float rosSteeringAngle = 0.0;
volatile float rosBrake = 1.0;

/**
 * @brief Simple wrapper to pull ROS number and store it in rosSteeringAngle.
 * Note: a value > 0 is left.  < 0 is right.
 * @param cmd_msg idk lol.  i simply copied this from the old version.
 */
void rosSteeringCallback(const std_msgs::Float64 &cmd_msg)
{
  rosSteeringAngle = cmd_msg.data;
}

/**
 * @brief Simple wrapper to pull ROS number and store it in rosBrake.
 * Note: digital brake control.  if 0, brake off.  if 1, brake on.
 * @param cmd_msg idk lol.  i simply copied this from the old version.
 */
void rosBrakeCallback(const std_msgs::Float64 &cmd_msg)
{
  rosBrake = cmd_msg.data;
}

ros::NodeHandle nh;
ros::Subscriber<std_msgs::Float64> steer("buggy/input/steering", rosSteeringCallback);
ros::Subscriber<std_msgs::Float64> brake("buggy/input/brake", rosBrakeCallback);

sensor_msgs::BatteryState battery_msg;
ros::Publisher battery("buggy/battery", &battery_msg);

diagnostic_msgs::DiagnosticStatus rosLogger;
diagnostic_msgs::KeyValue rosLogValues[10];
ros::Publisher debug("TeensyStateIn_T", &rosLogger);

nav_msgs::Odometry odometryMessage;
ros::Publisher nand_nav("/NAND/nav/odom", &odometryMessage);

uint8_t nand_fix = 0xFF;

// Every 100 cycles, publish debug data to ROS
int rosLogCounter = 0;

void setup()
{
  Serial.begin(115200);

  radio_init(RFM69_CS, RFM69_INT, RFM69_RST);

  rc::init();

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

  pinMode(BRAKE_RELAY_PIN, OUTPUT);
  digitalWrite(BRAKE_RELAY_PIN, LOW);
  pinMode(INTERRUPT_PIN, OUTPUT);
  pinMode(VOLTAGE_PIN, INPUT);

  steering::init();
  delay(2000);
  steering::calibrate();
}

void loop()
{

#if 0
  int rcSteeringWidth = v_rcSteeringWidth;
  int rcThrottleWidth = v_rcThrottleWidth;

  bool rcSteeringTimeout = (millis() - rcSteeringLastEdge) >= 50.0;
  bool rcThrottleTimeout = (millis() - rcThrottleLastEdge) >= 50.0;
  bool rcTimeout = rcSteeringTimeout || rcThrottleTimeout;

  // Capture most recent RC steering sample.
  rcSteeringSampleIndex++;
  rcSteeringSampleIndex %= RC_SAMPLING_WINDOW;
  rcSteeringSamples[rcSteeringSampleIndex] = rcSteeringWidth;

  // Calculating the average of the past 5 RC steering samples.
  float rcSteeringAvg = 0.0;
  for (int i = 0; i < RC_SAMPLING_WINDOW; i++)
  {
    rcSteeringAvg += rcSteeringSamples[i];
  }
  rcSteeringAvg /= RC_SAMPLING_WINDOW;

  // Determining the state of the throttle trigger on the RC controller.
  bool autoMode = rcThrottleWidth < (RC_THROTTLE_CENTER - RC_THROTTLE_DEADZONE);
  bool teleMode = (RC_THROTTLE_CENTER + RC_THROTTLE_DEADZONE) < rcThrottleWidth;
  bool brakeMode = !autoMode && !teleMode;
#endif

  rc::update();

  auto link_stats = rc::link_statistics();

  float steering_command = rc::use_autonomous_steering() ? rosSteeringAngle : rc::steering_angle();
  bool brake_command = rc::operator_ready();

  steering::set_goal_angle(steering_command);

  if (steering::alarm_triggered()) {
    brake_command = false;
  }

  digitalWrite(BRAKE_RELAY_PIN, brake_command);

  if (radio_available()) {
    uint8_t buf[256] = { 0 };
    if (auto length = radio_receive(buf)) {
      Packet *p = (Packet *)buf;
      if (p->tag == GPS_X_Y) {
        Serial.printf("X: %lf Y: %lf T: %llu F: %u\n", p->gps_x_y.x, p->gps_x_y.y, p->gps_x_y.time, (unsigned)p->gps_x_y.fix);

        odometryMessage.pose.pose.position.x = p->gps_x_y.x;
        odometryMessage.pose.pose.position.y = p->gps_x_y.y;
        odometryMessage.pose.pose.position.z = 0.0;

        memset(odometryMessage.pose.covariance, 0, sizeof(odometryMessage.pose.covariance));
        
        odometryMessage.twist.twist.angular.x = 0.0;
        odometryMessage.twist.twist.angular.y = 0.0;
        odometryMessage.twist.twist.angular.z = 0.0;

        odometryMessage.twist.twist.linear.x = 0.0;
        odometryMessage.twist.twist.linear.y = 0.0;
        odometryMessage.twist.twist.linear.z = 0.0;

        memset(odometryMessage.twist.covariance, 0, sizeof(odometryMessage.twist.covariance));

        nand_nav.publish(&odometryMessage);

        nand_fix = p->gps_x_y.fix;
      }
    }
  }

  // Logging data to ROS
  if (rosLogCounter == 0)
  {
    rosLogger.name = "Steering Teensy Log";
    rosLogger.level = diagnostic_msgs::DiagnosticStatus::OK;
    rosLogger.message = "buggy yeet";
    rosLogger.values = &rosLogValues[0];
    rosLogger.values_length = sizeof(rosLogValues) / sizeof(diagnostic_msgs::KeyValue);

    char c_steeringCommand[32];
    String(String(steering_command) + " deg").toCharArray(c_steeringCommand, 32);

    char c_brakeCommand[32];
    String(brake_command).toCharArray(c_brakeCommand, 32);

    char c_presentLoad[32];
    String("xxx").toCharArray(c_presentLoad, 32);

    char c_current[32];
    //String(String(dynamixelCurrentToMilliAmps(current)) + " mA").toCharArray(c_current, 32);
    //snprintf(c_current, 32, "%hu", current);
    snprintf(c_current, 32, "xxx");

    char c_leftSteeringLimit[32];
    String(String(steering::left_step_limit()) + " steps").toCharArray(c_leftSteeringLimit, 32);

    char c_rightSteeringLimit[32];
    String(String(steering::right_step_limit()) + " steps").toCharArray(c_rightSteeringLimit, 32);

    char c_rcSteeringInput[32];
    String(String(rc::steering_angle())).toCharArray(c_rcSteeringInput, 32);

    char c_uplinkQuality[32];
    String(link_stats.uplink_Link_quality).toCharArray(c_uplinkQuality, 32);

    char c_autoMode[32];
    String(rc::use_autonomous_steering()).toCharArray(c_autoMode, 32);

    char c_rcSteeringWidth[32];
    //String(rcSteeringWidth).toCharArray(c_rcSteeringWidth, 32);
    String(rc::steering_angle()).toCharArray(c_rcSteeringWidth, 32);

    char c_nandFix[32];
    String(nand_fix).toCharArray(c_nandFix, 32);

    rosLogValues[0].key = "steeringAngleCommand";
    rosLogValues[0].value = c_steeringCommand;
    rosLogValues[1].key = "brakeCommand";
    rosLogValues[1].value = c_brakeCommand;
    rosLogValues[2].key = "present load";
    rosLogValues[2].value = c_presentLoad;
    rosLogValues[3].key = "current milliamps";
    rosLogValues[3].value = c_current;
    /*rosLogValues[4].key = "left dynamixel limit";
    rosLogValues[4].value = c_leftSteeringLimit;
    rosLogValues[5].key = "right dynamixel limit";
    rosLogValues[5].value = c_rightSteeringLimit;*/
    rosLogValues[4].key = "uplink quality";
    rosLogValues[4].value = c_uplinkQuality;
    rosLogValues[5].key = "auto mode";
    rosLogValues[5].value = c_autoMode;
    rosLogValues[6].key = "rc steering input percent";
    rosLogValues[6].value = c_rcSteeringInput;
    rosLogValues[7].key = "rc steering input width";
    rosLogValues[7].value = c_rcSteeringWidth;
    rosLogValues[8].key = "nand fix type";
    rosLogValues[8].value = c_nandFix;

    debug.publish(&rosLogger);

    battery_msg.voltage = analogRead(VOLTAGE_PIN) / 1024.0 * 50.0;
    battery.publish(&battery_msg);
  }

  rosLogCounter++;
  rosLogCounter %= 100;

  delay(1);
  nh.spinOnce();
}

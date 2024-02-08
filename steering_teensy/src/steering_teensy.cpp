#include <Arduino.h>

#define RFM69_CS 2
#define RFM69_INT 3
#define RFM69_RST 4

#include "ArduinoCRSF.h"
#include "buggyradio.h"

#define USE_TEENSY_HW_SERIAL // Must be before <ros.h>
#define ROS_BAUD 1000000

#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt32.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <sensor_msgs/BatteryState.h>

/* ============= */
/* Board Config  */
/* ============= */

// Teensy Pins
#define VOLTAGE_PIN 27
#define BRAKE_RELAY_PIN 26
#define INTERRUPT_PIN 41
#define ALARM_PIN 39

#define STEPS_PER_REV 1000 // steps per rotation
#define PUL 27 // pin for stepper pulse
#define DIR 38 // pin for stepper direction

#define LIMIT_SWITCH_RIGHT 7
#define LIMIT_SWITCH_LEFT 8

// RC Controller PWM Pins
#define RC_SERIAL Serial6
#define RC_BAUDRATE 115200

#define CHANNEL_LEFT_X  4
#define CHANNEL_LEFT_Y  3
#define CHANNEL_RIGHT_X 1
#define CHANNEL_RIGHT_Y 2
#define CHANNEL_SWITCH_E 5
#define CHANNEL_SWITCH_F 6
#define CHANNEL_SWITCH_B 7
#define CHANNEL_SWITCH_C 8
#define CHANNEL_BUTTON_A 9
#define CHANNEL_BUTTON_D 10

ArduinoCRSF rc_controller;
//#define STEERING_PIN 26
//#define THROTTLE_PIN 27

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

/** inturrupt function for steering*/

volatile int cPos = 0;
volatile int gPos = 0;

// gPos++ turns left, gPos-- turns right
void pulse(){ 
  if(cPos < gPos){
    if(!digitalRead(LIMIT_SWITCH_LEFT)){
      return;
    }
    digitalWrite(DIR, LOW);
    delayMicroseconds(5);
    digitalWrite(PUL, HIGH);
    delayMicroseconds(5);
    digitalWrite(PUL, LOW);
    cPos++;
  } else if(cPos > gPos){
    if(!digitalRead(LIMIT_SWITCH_RIGHT)){
      return;
    }
    digitalWrite(DIR, HIGH);
    delayMicroseconds(5);
    digitalWrite(PUL, HIGH);
    delayMicroseconds(5);
    digitalWrite(PUL,LOW);
    cPos--;
  } else {
    return;
  }
}

IntervalTimer step;

// Positive direction -> going left
int LEFT_STEPPER_LIMIT = 0;
int RIGHT_STEPPER_LIMIT = 0;

// Adjust the left and right steering limits
void calibrate_steering() {
  while (digitalRead(LIMIT_SWITCH_LEFT)) {
    delay(1);
    ++gPos;
    Serial.printf("gPos: %d\n",gPos);
  }
  LEFT_STEPPER_LIMIT = gPos;
  while (digitalRead(LIMIT_SWITCH_RIGHT)) {
    delay(1);
    --gPos;
  }
  RIGHT_STEPPER_LIMIT = gPos;

  int offset = (LEFT_STEPPER_LIMIT + RIGHT_STEPPER_LIMIT) / 2;

  gPos -= offset;
  cPos -= offset;
  LEFT_STEPPER_LIMIT -= offset;
  RIGHT_STEPPER_LIMIT -= offset;

  // Center the steering again
  gPos = 0;
}

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

// Every 100 cycles, publish debug data to ROS
int rosLogCounter = 0;

/**
 * @brief
 */
inline int getCenter()
{
  return 0;
}

/**
 * @brief
 */
inline int getStepperRange()
{
  return LEFT_STEPPER_LIMIT - RIGHT_STEPPER_LIMIT;
}

const float RC_STEERING_DEGREES = 30.0;

float rcToDegrees(int pulse_width) {
  // Scales it to -1.0 to 1.0
  float displacement = -2.0 * (pulse_width - 1500.0) / 1000.0;

  // Scale to the range of +/-RC_STEERING_DEGREES
  displacement = displacement * RC_STEERING_DEGREES;

  return displacement;
}

const float DEGREES_TO_STEPS = (1000 / 360.0) * (34.0 / 18.0) * 10.0;

void setGoalSteeringAngle(float degrees) {
  gPos = (int)(DEGREES_TO_STEPS * degrees);
}

void setup()
{
  Serial.begin(115200);

  radio_init();

  RC_SERIAL.begin(RC_BAUDRATE);
  if (!RC_SERIAL) {
    while (1) Serial.println("CRSF serial initialization failed");
  }
  rc_controller.begin(RC_SERIAL);

  /*pinMode(STEERING_PIN, INPUT);
  pinMode(THROTTLE_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(STEERING_PIN), steeringInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN), throttleInterruptHandler, CHANGE);*/

  nh.getHardware()->setBaud(ROS_BAUD);
  nh.initNode();
  nh.subscribe(steer);
  nh.subscribe(brake);
  nh.advertise(debug);
  nh.advertise(battery);

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

  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(ALARM_PIN, INPUT_PULLUP);

  pinMode(LIMIT_SWITCH_LEFT,INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_RIGHT,INPUT_PULLUP);

  step.begin(pulse, 50);
  step.priority(255);

  delay(2000);

  Serial.println("starting calibrate_steering");
  calibrate_steering();
  Serial.println("finished with calibrate steering");

  /*Serial.print("Left limit is ");
  Serial.println(LEFT_STEPPER_LIMIT);
  Serial.print("Right limit is ");
  Serial.println(RIGHT_STEPPER_LIMIT);*/
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

  rc_controller.update();

  auto link_stats = rc_controller.getLinkStatistics();

  bool rcTimeout = !rc_controller.isLinkUp();
  bool buggyEnabled = (rc_controller.getChannel(CHANNEL_BUTTON_A) > 1500) || (rc_controller.getChannel(CHANNEL_BUTTON_D) > 1500);
  bool autoMode = (rc_controller.getChannel(CHANNEL_SWITCH_E) > 1750) && buggyEnabled;

  int rcSteeringAvg = rc_controller.getChannel(CHANNEL_RIGHT_X);

  // Controlling hardware thru RC.
  float steeringCommand = rcTimeout ? getCenter() : rcToDegrees(rcSteeringAvg);
  bool brakeCommand = (buggyEnabled && !rcTimeout);

  // If auton is enabled, it will set inputs to ROS inputs.
  if (autoMode && !rcTimeout)
  {
    steeringCommand = rosSteeringAngle;
    //brakeCommand = 0.5 < rosBrake;
  }

  static bool dynamixel_shutdown = false;

  setGoalSteeringAngle(steeringCommand);

  Serial.println(rcSteeringAvg);

  if (!digitalRead(ALARM_PIN)) {
    brakeCommand = false;
  }

  digitalWrite(BRAKE_RELAY_PIN, brakeCommand);

  if (radio_available()) {
    uint8_t buf[256] = { 0 };
    radio_receive(buf, 256);

    Serial.println(buf);
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
    String(String(steeringCommand) + " deg").toCharArray(c_steeringCommand, 32);

    char c_brakeCommand[32];
    String(brakeCommand).toCharArray(c_brakeCommand, 32);

    char c_presentLoad[32];
    String("xxx").toCharArray(c_presentLoad, 32);

    char c_current[32];
    //String(String(dynamixelCurrentToMilliAmps(current)) + " mA").toCharArray(c_current, 32);
    //snprintf(c_current, 32, "%hu", current);
    snprintf(c_current, 32, "xxx");

    char c_leftSteeringLimit[32];
    String(String(LEFT_STEPPER_LIMIT) + " steps").toCharArray(c_leftSteeringLimit, 32);

    char c_rightSteeringLimit[32];
    String(String(RIGHT_STEPPER_LIMIT) + " steps").toCharArray(c_rightSteeringLimit, 32);

    char c_rcSteeringInput[32];
    String(String(rcSteeringAvg)).toCharArray(c_rcSteeringInput, 32);

    char c_uplinkQuality[32];
    String(link_stats->uplink_Link_quality).toCharArray(c_uplinkQuality, 32);

    char c_autoMode[32];
    String(autoMode).toCharArray(c_autoMode, 32);

    char c_rcSteeringWidth[32];
    //String(rcSteeringWidth).toCharArray(c_rcSteeringWidth, 32);
    String(rcSteeringAvg).toCharArray(c_rcSteeringWidth, 32);

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

    debug.publish(&rosLogger);

    battery_msg.voltage = analogRead(VOLTAGE_PIN) / 1024.0 * 50.0;
    battery.publish(&battery_msg);
  }

  rosLogCounter++;
  rosLogCounter %= 100;

  delay(1);
  nh.spinOnce();
}

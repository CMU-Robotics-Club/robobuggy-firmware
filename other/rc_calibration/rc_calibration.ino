#define STEERING_PIN 26
#define THROTTLE_PIN 27

int LEFT_DYNAMIXEL_LIMIT = 1460;
int RIGHT_DYNAMIXEL_LIMIT = 890;
int DYNAMIXEL_CENTER = (LEFT_DYNAMIXEL_LIMIT + RIGHT_DYNAMIXEL_LIMIT) / 2.0;
int DYNAMIXEL_RANGE = LEFT_DYNAMIXEL_LIMIT - RIGHT_DYNAMIXEL_LIMIT;

volatile int v_rcSteeringWidth = 0;
volatile int v_rcThrottleWidth = 0;

volatile unsigned long rcSteeringLastEdge = 0;
volatile unsigned long rcThrottleLastEdge = 0;
unsigned long rcSteeringUptime = 0;
unsigned long rcThrottleUptime = 0;
int rcSteeringSamples[5] = { 0 };
int rcSteeringSampleIndex = 0;

void steeringInterruptHandler() {
  rcSteeringLastEdge = millis();

  if (digitalRead(STEERING_PIN)) {
    rcSteeringUptime = micros();
  } else {
    int width = micros() - rcSteeringUptime;
    if (10 <= width && width <= 2000) {  // Filtering out blips and long pauses in the signal.
      v_rcSteeringWidth = width;
    }
  }
}

void throttleInterruptHandler() {

  rcThrottleLastEdge = millis();

  if (digitalRead(THROTTLE_PIN)) {  // The pin has changed from low to high, so we're resetting the uptime timer.
    rcThrottleUptime = micros();
  } else {
    int width = micros() - rcThrottleUptime;
    if (10 <= width && width <= 2000) {  // Filtering out blips and long pauses in the signal.
      v_rcThrottleWidth = width;
    }
  }
}

int mapPWM(int pulseWidth) {
  // Scaling from milliseconds to a [0, 1] range
  float displacement = (pulseWidth - 1000.0) / 650.0;

  // Skewing
  float midpoint = 0.51;
  float output;
  if (displacement > midpoint) {
    output = (displacement - midpoint) / (1.0 - midpoint) * 0.5 + 0.5;
  } else {
    output = 0.5 - (midpoint - displacement) / (midpoint)*0.5;
  }

  // Scaling from [0, 1] to [-1, 1]
  output = (output * 2.0) - 1.0;

  // Quadratic steering scale
  // (known to the programmers of Saints Robotics as "odd square")
  output *= abs(output);

  return output;
}

void setup() {
  pinMode(STEERING_PIN, INPUT);
  pinMode(THROTTLE_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(STEERING_PIN), steeringInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN), throttleInterruptHandler, CHANGE);

  Serial.begin(115200);
}

void loop() {
  int rcSteeringWidth = v_rcSteeringWidth;
  int rcThrottleWidth = v_rcThrottleWidth;

  rcSteeringSampleIndex++;
  rcSteeringSampleIndex %= 5;
  rcSteeringSamples[rcSteeringSampleIndex] = rcSteeringWidth;

  float rcSteeringAvg = 0.0;
  for (int i = 0; i < 5; i++) {
    rcSteeringAvg += rcSteeringSamples[i];
  }
  rcSteeringAvg /= 5;

  Serial.print("Steering width: ");
  Serial.print(rcSteeringWidth);
  Serial.print(" Throttle width: ");
  Serial.println(rcThrottleWidth);
}

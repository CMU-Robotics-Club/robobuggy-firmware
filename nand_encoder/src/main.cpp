#include <Arduino.h>
#include "encoder.h"
#include <HardwareSerial.h>

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

RateLimit enc_read_limit{100};
RateLimit serial_send_limit{100};

float speed;

void setup()
{
  encoder::init();
  Serial.begin(115200);
}

void loop()
{
  if (enc_read_limit.ready())
  {
    speed = encoder::get_degrees_per_second();
  }
  if (serial_send_limit.ready())
  {
    Serial.print("encoder speed: ");
    Serial.println(speed);
  }
}
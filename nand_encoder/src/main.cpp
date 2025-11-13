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

void setup()
{
  encoder::init();
  Serial.begin(115200);
}

void loop()
{
  bool writes_good = false;
  if (enc_read_limit.ready())
  {
    encoder::get_degrees_per_second();
    encoder::update();
  }
  if (serial_send_limit.ready())
  {
    Serial.print("encoder speed: ");
    Serial.println(encoder::get_degrees_per_second());
  }
}
#include <Arduino.h>
#include "encoder.h"
#include <HardwareSerial.h>

#define ENCODER_READ_PERIOD_MS 1
#define SERIAL_PRINT_PERIOD_MS 100

#define FILTER_BUFFER_SIZE 128

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

RateLimit enc_read_limit{ENCODER_READ_PERIOD_MS};
RateLimit serial_send_limit{SERIAL_PRINT_PERIOD_MS};

float speed;
float speed_filter[FILTER_BUFFER_SIZE];
int filter_idx;
float speed_filter_sum;

void setup()
{
  encoder::init();
  Serial.begin(115200);
  filter_idx = 0;
  speed_filter_sum = 0;
}

void loop()
{
  if (enc_read_limit.ready())
  {
    speed = encoder::get_degrees_per_second();
    speed_filter_sum += speed;
    speed_filter_sum -= speed_filter[filter_idx];
    speed_filter[filter_idx] = speed;
    filter_idx++;
    filter_idx %= FILTER_BUFFER_SIZE;
  }
  if (serial_send_limit.ready())
  {
    Serial.print("encoder pos: ");
    Serial.print(encoder::get_degrees());
    Serial.print("\tencoder speed: ");
    Serial.print(speed);
    Serial.print("\tfiltered speed: ");
    Serial.println(speed_filter_sum/FILTER_BUFFER_SIZE);
  }
}
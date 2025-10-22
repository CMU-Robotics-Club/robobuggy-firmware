#include <Arduino.h>
#include "encoder.h"
#include <HardwareSerial.h>

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

RateLimit enc_read_limit {10};
RateLimit serial_send_limit {100};

void setup() {
  encoder::init();
  Serial.begin(115200);
}

void loop() {
  if(enc_read_limit.ready()) {
    encoder::get_pos();
  }
  if(serial_send_limit.ready()) {
    // Serial.printf("%c%c%c%c",0xAA,0xFF,0x00,0x55);
    Serial.println(encoder::get_speed());
    Serial.flush();
  }
}
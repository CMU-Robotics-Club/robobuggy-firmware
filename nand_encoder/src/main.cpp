#include <Arduino.h>
#include <cstdint>
#include <array>
#include <cstring>
#include "encoder.h"
#include <HardwareSerial.h>

#define ENCODER_READ_PERIOD_MS 1
#define SERIAL_PRINT_PERIOD_MS 100

#define FILTER_BUFFER_SIZE 128

// Packet networking
#define COMM_SERIAL Serial1
#define COMM_BAUDRATE 115200
#define SYNC_LEN 4

const std::array<uint8_t, SYNC_LEN> SYNC_WORD = {0xAA, 0xFF, 0x00, 0x55};

enum class EncoderMessageHeader : uint8_t
{
  ErrorInfo = 'E',
  SpeedInfo = 'S'
} header;

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
float filtered_speed;
float speed_filter[FILTER_BUFFER_SIZE];

int filter_idx;
float speed_filter_sum;

// Packet networking
void write_speed_packet()
{
  COMM_SERIAL.write(SYNC_WORD.data(), SYNC_LEN); // Sync word

  filtered_speed = speed_filter_sum / FILTER_BUFFER_SIZE;
  header = EncoderMessageHeader::SpeedInfo;
  COMM_SERIAL.write(header); // Message type
  byte* bytes = (byte*)&filtered_speed;
  COMM_SERIAL.write(bytes, sizeof(filtered_speed)); // Message
}

void setup()
{
  // pinMode(3, OUTPUT);
  // digitalWrite(3, HIGH);
  Serial.begin(115200);
  COMM_SERIAL.begin(COMM_BAUDRATE);
  Serial.print("init enc");
  encoder::init();
  filter_idx = 0;
  speed_filter_sum = 0;
}

void loop()
{
  // Serial1.println("encoder ready");
  // Serial.println("we're alive");
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
    Serial.println(speed_filter_sum / FILTER_BUFFER_SIZE);

    write_speed_packet();
    // Add error packet later

    /*
    Serial1.print("encoder pos: ");
    Serial1.print(encoder::get_degrees());
    Serial1.print("\tencoder speed: ");
    Serial1.print(speed);
    Serial1.print("\tfiltered speed: ");
    Serial1.println(speed_filter_sum / FILTER_BUFFER_SIZE);
    */
  }
}
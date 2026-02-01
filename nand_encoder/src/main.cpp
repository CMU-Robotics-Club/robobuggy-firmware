/**
 * @file main.cpp
 * @brief Main XIAO code
 *
 * This file implements setup and loop for the XIAO, taking care of
 * using the encoder interface and writing packets to the Teensy over UART.
 * It also runs a moving average filter on the speed to send a temporally
 * smoothed value in packets.
 *
 * @author Delaynie McMillan
 * @author Alden Grover
 * @author Anna Delale-O'Connor
 * @date 1/12/2026
 *
 * @author Sanjay Ravishankar
 * @date 1/19/2026 - Added speed packet communication
 *
 * @author Sanjay Ravishankar
 * @date 1/31/2026 - Added error packet communication
 */

#include <Arduino.h>
#include "encoder.h"
#include "ratelimit.h"

#define ENCODER_READ_PERIOD_MS 1
#define SERIAL_PRINT_PERIOD_MS 25
#define FILTER_BUFFER_SIZE 128
#define COMM_SERIAL Serial1
#define COMM_BAUDRATE 115200
#define SYNC_LEN 4

enum EncoderMessageHeader : uint8_t
{
  ErrorInfo = 'E',
  SpeedInfo = 'S'
} header;
enum Error : uint8_t
{
  None,
  FailedInit = 'I',
  FailedComm = 'C'
} error;
RateLimit enc_read_limit{ENCODER_READ_PERIOD_MS};
RateLimit serial_send_limit{SERIAL_PRINT_PERIOD_MS};
const uint8_t SYNC_WORD[SYNC_LEN] = {0xAA, 0xFF, 0x00, 0x55};
float position;
float speed;
float filtered_speed;
float speed_filter[FILTER_BUFFER_SIZE];
int filter_idx;
float speed_filter_sum;

/**
 * Packet networking
 * Writes sync word, message header, and message
 * TODO: maybe turn into state machine to reduce blocking
 */
void write_speed_packet()
{
  COMM_SERIAL.write(SYNC_WORD, SYNC_LEN);
  COMM_SERIAL.write(EncoderMessageHeader::SpeedInfo);
  COMM_SERIAL.write((byte *)&filtered_speed, sizeof(filtered_speed));
}

void write_error_packet()
{
  COMM_SERIAL.write(SYNC_WORD, SYNC_LEN);
  COMM_SERIAL.write(EncoderMessageHeader::ErrorInfo);
  COMM_SERIAL.write(error);
}

/**
 * @note Blocking if encoder fails to initialize purposes.
 * This should probably not be the case outside of testing.
 * TODO: maybe implement handshake with Teensy
 */
void setup()
{
  Serial.begin(115200);
  COMM_SERIAL.begin(COMM_BAUDRATE);
  error = Error::None;
  Serial.println("Initializing encoder...");
  while (!encoder::init())
  {
    error = Error::FailedInit;
    write_error_packet();
    Serial.println("ERROR: Failed init");
  }
  Serial.println("Encoder initialized");
  filter_idx = 0;
  speed_filter_sum = 0;
}

/**
 * @details Applies rate limiters to encoder reading and packet
 * communicaton. Also computes filtered_speed over a moving average.
 */
void loop()
{
  if (enc_read_limit.ready())
  {
    if (encoder::get_degrees(&position) && encoder::get_degrees_per_second(&speed))
    {
      error = Error::None;
      speed_filter_sum += speed;
      speed_filter_sum -= speed_filter[filter_idx];
      speed_filter[filter_idx] = speed;
      filter_idx++;
      filter_idx %= FILTER_BUFFER_SIZE;
    }
    else
    {
      error = Error::FailedComm;
    }
  }
  if (serial_send_limit.ready())
  {
    if (error == Error::None)
    {
      Serial.print("pos: ");
      Serial.print(position);
      Serial.print("\tspeed: ");
      Serial.print(speed);
      Serial.print("\tfiltered speed: ");

      // Why is the line here and not in the enc_read_limit.ready() if statement?
      filtered_speed = speed_filter_sum / FILTER_BUFFER_SIZE;

      Serial.println(filtered_speed);
      write_speed_packet();
    }
    else
    {
      Serial.println("ERROR: Failed comm");
      write_error_packet();
    }
  }
}
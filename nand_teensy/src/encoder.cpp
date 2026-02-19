/**
 * @file encoder.cpp
 * @brief Implementation for the encoder namespace on the Teensy
 *
 * This file implements a state machine to read UART packets from
 * the Seeed Studio XIAO connected to the AS5600 encoder module.
 * It also implements getter functions for the encoder data.
 * See nand_teensy/examples/encoder-test.cpp for usage.
 *
 * @author Alden Grover
 * @author Garrison Chan
 * @author Anna Delale-O'Connor
 * @author Sanjay Ravishankar
 * @date 1/12/2026-2/16/2026
 */

#include <Arduino.h>
#include "encoder.h"

namespace encoder
{
  elapsedMillis lastPacketReceived;
  enum class State
  {
    SyncWord,
    Header,
    Payload
  };
  State state = State::SyncWord;
  const uint8_t SYNC_WORD[] = {0xAA, 0xFF, 0x00, 0x55};
  const uint8_t SYNC_LEN = sizeof(SYNC_WORD); // Should be 4
  uint8_t sync_index = 0;
  uint8_t payload_index = 0;
  uint8_t byte;
  char header = '\0';
  uint8_t payload[4] = {0}; // For speed
  // Filtered speed in degrees/sec
  // Float instead of double since XIAO SAMD doesn't have double precision
  float speed = 0;
  char error = '\0';

  /**
   * TODO: maybe implement handshake with XIAO
   */
  void init()
  {
    Serial.println("[encoder.cpp] Establishing serial connection to XIAO...");
    COMM_SERIAL.begin(COMM_BAUDRATE);
  }

  /**
   * @details Polls the serial buffer and steps through the state machine
   * for each byte that was read. There are three possible states: sync word,
   * header, and payload. The current state must be finished before moving
   * on to the next.
   *
   * States:
   * - Sync word: check that the 4 sync word bytes are received in the correct
   *   order, accounting for overlapping or incorrect packets.
   * - Header: check that the header is 'S' for speed or 'E' for error.
   * - Payload:
   *     - Speed packet: read 4 bytes into a payload array and then memcpy into
   *       the speed float for a speed packet.
   *     - Error packet: check that the error is 'I' for if the XIAO could not
   *       connect to the AS5600 over I2C at all (init error), or 'C' if the
   *       I2C failed after initialization (comm error).
   *
   * @note Print statements for errors/warning may want to be removed after debug
   */
  void poll()
  {
    while (COMM_SERIAL.available())
    {
      byte = COMM_SERIAL.read();
      switch (state)
      {
      case State::SyncWord:
        if (byte == SYNC_WORD[sync_index]) // Bytes in sync
        {
          sync_index++;
          if (sync_index == SYNC_LEN) // Current byte match
          {
            // Full match
            sync_index = 0;
            state = State::Header;
          }
        }
        else if (byte == SYNC_WORD[0]) // Sync word was cut off and restarted
        {
          sync_index = 1;
          Serial.println("[encoder.cpp] Warning: Sync word was cut off");
        }
        else // Mismatch, restart state machine
        {
          sync_index = 0;
          Serial.println("[encoder.cpp] Error: Received invalid sync byte");
        }
        break;
      case State::Header:
        header = (char)byte;
        if (header == 'S' || header == 'E') // Valid header
          state = State::Payload;
        else // Invalid header, restart state machine
        {
          state = State::SyncWord;
          Serial.println("[encoder.cpp] Error: Received invalid header byte");
        }
        break;
      case State::Payload:
        if (header == 'S') // Speed packet
        {
          payload[payload_index++] = byte;
          if (payload_index == 4) // Payload received, reset state machine
          {
            payload_index = 0;
            error = '\0';
            header = '\0';
            memcpy(&speed, payload, sizeof(speed));
            state = State::SyncWord;
          }
        }
        else if (header == 'E') // Error packet
        {
          error = (char)byte;
          if (error == 'I')
            Serial.println("[encoder.cpp] Error packet: Failed init");
          else if (error == 'C')
            Serial.println("[encoder.cpp] Error packet: Failed comm");
          else // Invalid header
            Serial.println("[encoder.cpp] Error: received invalid error byte");
          state = State::SyncWord;
        };
        lastPacketReceived = 0;
        break;
      default:
        break;
      }
    }
  }

  bool front_speed(double *s)
  {
    if (!(s && error == '\0'))
      return false;
    *s = (double)speed;
    return true;
  }

  /**
   * TODO: Add valid input range to steering_angle (like -90 to 90 degrees)
   */
  bool rear_speed(double *s, double steering_angle)
  {
    if (!(s && error == '\0'))
      return false;
    double front_speed_val;
    if (front_speed(&front_speed_val))
    {
      steering_angle *= M_PI / 180.0;
      *s = front_speed_val * cos(steering_angle);
      return true;
    }
    return false;
  }
  
  long lastPacket()
  {
    return lastPacketReceived;
  }
}
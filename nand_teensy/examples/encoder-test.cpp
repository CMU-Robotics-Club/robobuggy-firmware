/**
 * @file encoder-test.cpp
 * @brief Test file for the encoder namespace on the Teensy
 * 
 * This file tests the encoder namespace by printing the speed to the serial
 * monitor every 50ms. It also checks that packets are being received and prints
 * an error if they aren't.
 * 
 * Packet structure:
 - Sync word: Denotes the start of each packet (uint8_t, 4 bytes)
 - Header: The type of message ('S' for speed, 'E' for error) (char, 1 byte)
 - Payload:
    - Speed in deg/sec (float, 4 bytes), or
    - Error code ('I' for init error, 'C' for comm error) (char, 1 byte)
 * 
 * @author Sanjay Ravishankar
 * @date 2/16/2026
 */

#include <Arduino.h>
#include "encoder.h"

elapsedMillis elapsedTime = 0;
double speed;

void setup()
{
  Serial.begin(115200);
  encoder::init();
}

// Test that the Teensy receives speed properly
void loop()
{
  encoder::poll();
  if (encoder::lastPacket() > 100) // ms
  {
    Serial.println("Error, packets not found!");
    return;
  }
  if (elapsedTime >= 50) // Delay interval of 50ms to simulate other operations
  {
    if (encoder::front_speed(&speed))
      Serial.println(speed, 3); // Format to 3 decimal places
    elapsedTime = 0;
  }
}

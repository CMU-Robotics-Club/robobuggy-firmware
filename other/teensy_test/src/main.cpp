/**
 * @file main.cpp
 * @brief Main test file for reading encoder data on the Teensy
 * 
 * This file imports the encoder interface to test init, poll and get_speed.
 * 
 * @author Sanjay Ravishankar
 * @date 1/31/2026
 */

#include <Arduino.h>
#include "encoder.h"

elapsedMillis elapsedTime = 0;
float speed;

void setup()
{
  Serial.begin(115200);
  encoder::init();
}

// Test that the Teensy receives speed properly
void loop()
{
  encoder::poll();
  if (elapsedTime >= 50) // Delay interval of 50ms to simulate other operations
  {
    if (encoder::get_speed(&speed))
      Serial.println(speed, 3); // Format to 3 decimal places
    elapsedTime = 0;
  }
}

#include <Arduino.h>
#include <Wire.h>

// Include header file: digit_display.h
#include "digit_display.h"

// Initialization，i2c address 0x70 (DigitDisplay::kDeviceI2cAddress)
DigitDisplay digital_display(DigitDisplay::kDeviceI2cAddress);

// DEFINE SEVEN SEGMENT DISPLAY PINS

#define DATA 1
#define SDA 18
#define SCL 19

// DEFINE CURRENT READING PIN

#define CURRENT_READ_PIN A1

// DEFINE GLOBAL VARS;

float charge_capacity = 100.0; // Full charge = 10Ah = 10A * 3600s = 3.6e4 Coulombs

void setup() 
{
  Serial.begin(115200);

  digital_display.setup(); // Initialize, clear display

  digital_display.SetBrightness(15); // Set brightness to max val (0 - 15)

  digital_display.ShowColon(false);

  // Set output pins
  pinMode(DATA, OUTPUT);

  // Set clock, data to zero
  digitalWrite(DATA, 0);
}

void loop()
{
  // Set up time
  long time = micros();
  long prev = time;
  
  // Reads voltage, which is a representation of the current
  int current_reading_raw = analogRead(CURRENT_READ_PIN);
  float current_reading = ((current_reading_raw / 1023 * 3.3) - 1.833) * (4.5/3.3) * 10; // in amps (TRUST THE PROCESS)

  // Implement 50 us delay
  while (time - prev < 50)
  {
    time = micros();
  }
  float lost_charge = current_reading * 5 * 1e-5; // Obtains the amount of charge lost
  charge_capacity -= lost_charge / 3.6e4 * 100; // Converts lost charge to percentage of charge_capacity
  digital_display.ShowNumber(charge_capacity);
}
#include <Arduino.h>

// DEFINE LCD PINS

#define DATA 1
#define CLOCK 2

// DEFINE VOLTAGE READING PIN

#define VOLTAGE_READ_PIN A1

void setup() {
  pinMode(VOLTAGE_READ_PIN, INPUT);
  pinMode(DATA, OUTPUT);
  pinMode(CLOCK, OUTPUT);
}

void loop() {
  int voltage_input = analogRead(VOLTAGE_READ_PIN);
  float voltage_total = voltage_input / 1023 * 40 * 100; // Division by 1023 and multiplication by 40 to convert teensy reading to actual voltage
  for (int i = 0; i < 4; i++)                            // Multiplication by 100 to "round" voltage value for later processing
  { 
    byte byte_to_send = getByte(voltage_total, i);
    sendByte(byte_to_send);
  }
}

byte getByte(float voltage, int i) {
  int digit = (int)(voltage / pow(10, 3 - i)) % (int)(pow(10, i));
  
  // Creates a byte reflecting the output of the display
  switch (digit) { // Assumes segment is displayed when the bit is low
    case 0:
      return 0x02;
    case 1:
      return 0x9E;
    case 2:
      return 0x24;
    case 3:
      return 0x0C;
    case 4:
      return 0x98;
    case 5:
      return 0x48;
    case 6:
      return 0x40;
    case 7:
      return 0x1E;
    case 8:
      return 0x00;
    case 9:
      return 0x08;
    default:
      return 0x60; // Returns the letter E for Error
  }
}

void sendByte(byte byte_to_send) {
  long prev = 0;
  long current = micros();

  // Sets clock and data to zero
  digitalWrite(CLOCK, 0);
  digitalWrite(DATA, 0);

  // Sends each bit individually to SIPO (sends fromm left to right due to shifting)
  for (int i = 0; i < 8; i++) {
    bool bit = byte_to_send & (1 << i);
    prev = current;
    while(current - prev < 10) {
      while(current - prev < 5) {
        digitalWrite(DATA, bit);
        digitalWrite(CLOCK, 1);
      }
      digitalWrite(CLOCK, 0);
      digitalWrite(DATA, 0);
    }
  }
}
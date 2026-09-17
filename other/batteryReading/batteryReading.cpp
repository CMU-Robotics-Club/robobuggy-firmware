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
  float voltage_total = voltage_input / 1023 * 40 * 100; /* Division by 1023 and multiplication by 40 to convert teensy reading to actual voltage,
                                                                  Multiply by 100 and cast to int to "round" voltage to 2 decimal places */ 
  for (int i = 0; i < 4; i++) {
    byte byte_to_send = getByte(voltage_total, i);
    sendByte(byte_to_send);
  }
}

byte getByte(float voltage, int i) {
  int digit = (int)(voltage / pow(10, 3 - i)) % (int)(pow(10, i));
  switch (digit) {
    case 0:
      return 0x03;
    case 1:
      return 0x9F;
    case 2:
      return 0x25;
    case 3:
      return 0x0D;
    case 4:
      return 0x99;
    case 5:
      return 0x49;
    case 6:
      return 0x41;
    case 7:
      return 0x1F;
    case 8:
      return 0x01;
    case 9:
      return 0x09;
    default:
      return 0x61; // Returns the letter E for Error
  }
}

void sendByte(byte byte_to_send) {
  long prev = 0;
  long current = micros();

  // Sets clock and data to zero
  digitalWrite(CLOCK, 0);
  digitalWrite(DATA, 0);

  // Sends each bit individually to SIPO (does not send bit in 0th position since the bit in 0th position is not used)
  for (int i = 7; i > 0; i--) {
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
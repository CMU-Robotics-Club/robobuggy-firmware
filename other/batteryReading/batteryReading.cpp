#include <Arduino.h>

// DEFINE LCD PINS

#define DATA 1
#define CLOCK_SSD 2 // CLOCK_SSD --> Clock for the Serial Input Parallel Output (SIPO) moduels connected to the Seven Segment Displays (SSDs)
#define CLOCK_SIPO 3 // CLOCK_SIPO --> Clock for the Serial Input Parallel Output (SIPO) connected to other SIPOs

// DEFINE VOLTAGE READING PIN

#define VOLTAGE_READ_PIN A1

void setup() 
{
  pinMode(VOLTAGE_READ_PIN, INPUT);
  pinMode(DATA, OUTPUT);
  pinMode(CLOCK_SSD, OUTPUT);
  pinMode(CLOCK_SIPO, OUTPUT);

  // Sets clock and data to zero
  digitalWrite(DATA, 0);
  digitalWrite(CLOCK_SSD, 0);
  digitalWrite(CLOCK_SIPO, 0);
}

void loop()
{
  int voltage_input = analogRead(VOLTAGE_READ_PIN);
  float voltage_total = voltage_input / 1023 * 40 * 100; // Division by 1023 and multiplication by 40 to convert teensy reading to actual voltage
  byte bytes_to_send[8];                                 // Multiplication by 100 to "round" voltage value for later processing
  for (int i = 0; i < 8; i++)                            
  { 
    bytes_to_send[i] = getByte(voltage_total, i);
  }
}

byte getByte(float voltage, int i) 
{
  int digit = (int)(voltage / pow(10, 3 - i)) % (int)(pow(10, i));
  
  // Creates a byte reflecting the output of the display
  switch (digit) // Assumes segment is displayed when the bit is low
  { 
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

void sendByte(byte bytes_to_send[8]) 
{
  long prev = 0;
  long current = micros();

  // Sends each bit individually to the SIPOs connected to SSDs via "main" SIPO(sends fromm left to right due to shifting)
  for (int i = 0; i < 8; i++) 
  {
    
    // Sends a single bit from each byte to the "main" SIPO (the one connected to the other SIPOS)
    for (int j = 7; j > -1; j++)
    {
      bool bit = bytes_to_send[j] & (1 << i);
      prev = current;

      // Part where bits get sent from the Teensy to the main SIPO
      while(current - prev < 10)
      {
        while(current - prev < 5) 
        {
          digitalWrite(DATA, bit);
          digitalWrite(CLOCK_SIPO, 1);
        }
        digitalWrite(CLOCK_SIPO, 0);
        digitalWrite(DATA, 0);
      }
    }
    // Part where bits get sent from the main SIPO to the other SIPOs
    
    prev = current;
    while(current - prev > 10)
    {
      while (current - prev > 5)
      {
        digitalWrite(CLOCK_SIPO, 1);
        digitalWrite(CLOCK_SSD, 1);
      }
      digitalWrite(CLOCK_SSD, 0);
      digitalWrite(CLOCK_SIPO, 0);
    }
  }

  // Part where bits get sent from the other SIPOs to the seven segment displays
  prev = current;
  while(current - prev > 10)
  {
    while (current - prev > 5)
    {
      digitalWrite(CLOCK_SSD, 1);
    }
    digitalWrite(CLOCK_SSD, 0);
  }
}
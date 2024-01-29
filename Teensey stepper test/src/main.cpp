#include <Arduino.h>

#define STEPS 1600 // steps per rotation

#define PUL 6 // pin for stepper pulse
#define DIR 10 // pin for stepper direction
#define ENA 4 // not used
#define ALM 5 // not used


volatile int cPos = 0;
volatile int gPos = 0;

// NOTE: The directions for DIR still need to be checked. May have to switch (DIR,LOW) and (DIR,HIGH)
void pulse(){ 
  if(cPos < gPos){
    digitalWrite(DIR, LOW);
    delayMicroseconds(5);
    digitalWrite(PUL, HIGH);
    delayMicroseconds(5);
    digitalWrite(PUL, LOW);
    cPos++;
  } else if(cPos > gPos){
    digitalWrite(DIR, HIGH);
    delayMicroseconds(5);
    digitalWrite(PUL, HIGH);
    delayMicroseconds(5);
    digitalWrite(PUL,LOW);
    cPos--;
  } else {
    return;
  }
}

IntervalTimer step;

void setup() {
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);

  cPos = 0; // initial position, arbitrary
  gPos = 16000; // assuming steps per rotation = 1600

  step.begin(pulse, 100);
  step.priority(255);
}

void loop() {} //need to make a loop that takes the remote control data and change gPos
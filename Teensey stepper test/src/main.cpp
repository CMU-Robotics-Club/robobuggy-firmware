#include <Arduino.h>

#define STEPS 1600

#define PUL 6
#define DIR 10
#define ENA 4 // not used
#define ALM 5 // not used


volatile int cPos = 0;
volatile int gPos = 0;

void blink(){
  digitalToggle(LED_BUILTIN);
}

void pulse(){
  if(cPos == gPos){
    return;
  } else if(cPos < gPos){
    digitalWrite(DIR, LOW);
    delayMicroseconds(5);
    digitalWrite(PUL, HIGH);
    delayMicroseconds(5);
    digitalWrite(PUL, LOW);
    cPos++;
    return;
  } else if(cPos > gPos){
    digitalWrite(DIR, HIGH);
    delayMicroseconds(5);
    digitalWrite(PUL, HIGH);
    delayMicroseconds(5);
    digitalWrite(PUL,LOW);
    cPos--;
    return;
  }
  return;
}

IntervalTimer ledBlink;
IntervalTimer step;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);

  ledBlink.begin(blink, 1000);
  ledBlink.priority(0);

  cPos = 0;
  gPos = 64000;

  step.begin(pulse, 100);
  step.priority(255);
}

void loop() {
  
}

// put function definitions here:

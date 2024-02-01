#include <Arduino.h>

#define STEPS 1600 // steps per rotation

#define PUL 27 // pin for stepper pulse
#define DIR 38 // pin for stepper direction
#define ENA 4 // not used
#define ALM 5 // not used

#define LIMIT_SWITCH_RIGHT 7
#define LIMIT_SWITCH_LEFT 8

volatile int cPos = 0;
volatile int gPos = 0;

// NOTE: The directions for DIR still need to be checked. May have to switch (DIR,LOW) and (DIR,HIGH)
void pulse(){ 
  if(cPos < gPos){
    if(!digitalRead(LIMIT_SWITCH_LEFT)){
      return;
    }
    digitalWrite(DIR, LOW);
    delayMicroseconds(5);
    digitalWrite(PUL, HIGH);
    delayMicroseconds(5);
    digitalWrite(PUL, LOW);
    cPos++;
  } else if(cPos > gPos){
    if(!digitalRead(LIMIT_SWITCH_RIGHT)){
      return;
    }
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

int left_limit = 0;
int right_limit = 0;

void calib_steer(){
  while(digitalRead(LIMIT_SWITCH_LEFT)){
    delay(1);
    gPos = gPos+1;
  }
  left_limit = gPos;
  while(digitalRead(LIMIT_SWITCH_RIGHT)){
    delay(1);
    gPos-=1;
  }
  right_limit = gPos;
  gPos = (right_limit+left_limit)/2;
}

void setup() {
  Serial.begin(115200);
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);

  pinMode(LIMIT_SWITCH_LEFT,INPUT_PULLUP);
  pinMode(LIMIT_SWITCH_RIGHT,INPUT_PULLUP);

  cPos = 0; // initial position, arbitrary
  gPos = 0; // assuming steps per rotation = 1600

  step.begin(pulse, 50);
  step.priority(255);

  calib_steer();
}

void loop() {
} //need to make a loop that takes the remote control data and change gPos
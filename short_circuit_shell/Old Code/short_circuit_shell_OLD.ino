// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// Released under the GPLv3 license to match the rest of the
// Adafruit NeoPixel library

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif
#include "RoboCore_MMA8452Q.h"

// --------------------------------------------------
// Variables

MMA8452Q accelerometer;

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN        6 // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 200 // Popular NeoPixel ring size

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

#define DELAYVAL 10 // Time (in milliseconds) to pause between pixels
int delayVal=20;
float accerationArray[50]={0};

//original function that calculate the delay time of each LED
//not using for program

int getDelay(float acceration){
  if (acceration<=0){
    return 100;
  }
  int delayVal=100-int(log(acceration*10)/log(50)*100);
  return delayVal;

}

//calculates the delay value for LED flashes
int getDelay2(float acceration){
  if (-acceration>3/9.8){
    Serial.println(acceration);
    return 1;
  }
  return 500;
}

int getPusher(int pusher){
  return pusher;
}

//gets the filtered acceration
float getAcceration(){
  accelerometer.read();
  //float totalAcceration=pow(accelerometer.x*accelerometer.x+accelerometer.y*accelerometer.y+accelerometer.z*accelerometer.z,0.5);
  float totalAcceration=accelerometer.x*-1;
  updateFilterArray(totalAcceration,accerationArray);
  float sum=filter(accerationArray);
  return sum;
}

//returns the average of the first 50 elements of acceration array
//acts as digital low pass filter
float filter(float* accerationArray){
  float sum=0;
  for (int i=0;i<50;i++){;
    sum+=accerationArray[i];
  }

  return sum/50;

}

//updates the acceration array
void updateFilterArray(float newVal,float* accerationArray ){
  for (int i=1;i<50;i++){
    accerationArray[i-1]=accerationArray[i];
  }
  accerationArray[49]=newVal;

}


void setup() {
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  Serial.begin(9600);
  accelerometer.init();
}

void loop() {
  pixels.clear(); // Set all pixel colors to 'off'

  // The first NeoPixel in a strand is #0, second is 1, all the way up
  // to the count of pixels minus one.
  for(int i=0; i<20; i++) { // For each pixel...
  //calculates time
    int time=int(millis());
    // pixels.Color() takes RGB values, from 0,0,0 up to 255,255,255
    // Here we're using a moderately bright green color:
    
    if (time%delayVal==0){ //checks if it is right time to update LED pattern
      for (int j=i;j<150;j+=20) {
       //Serial.println(delayVal);
       pixels.setPixelColor(j, pixels.Color(0,255,90,90));
       pixels.show();   // Send the updated pixel colors to the hardware.

       //takes acceration reading
       if (time%5==0) {
        float acceration=getAcceration();    
        delayVal=getDelay2(acceration);
        
        //Serial.println("");
        //Serial.print(",");
        //Serial.println(acceration);
      }
      }
      Serial.println(int(millis())-time);
    }
    else{
      i-=1;
    }
    //takes acceration reading
    if (time%5==0) {
        float acceration=getAcceration();    
        delayVal=getDelay2(acceration);
        
        //Serial.println("");
        //Serial.print(",");
        //Serial.println(acceration);
      }
    /*
    for (int i=0;i<50;i++){
        Serial.print(accerationArray[i]);
        Serial.print(",");
    }
    */

    delay(1); // Pause before next pass through loop
    
  }
}

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
#define NUMPIXELS 144 // Popular NeoPixel ring size

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

#define DELAYVAL 0 // Time (in milliseconds) to pause between pixels

#define WIDTH 40
int startpos = 0-WIDTH;


void setup() {

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  Serial.begin(9600);
}

void runStripe(){
   pixels.clear();
   for (int i=0; i < NUMPIXELS; i++) {
    if ((startpos <= i) && (i < startpos + WIDTH) && (i >= 0)) {
      pixels.setPixelColor(i, pixels.Color(0,255,90,100));
    }
   }
   pixels.show();
   startpos += 1;
   if (startpos >= NUMPIXELS) startpos = 0-WIDTH;
}

void loop() {
  runStripe();
  delay(DELAYVAL);
}

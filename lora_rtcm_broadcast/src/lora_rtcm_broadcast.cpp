/*
   RadioLib SX127x Transmit with Frequency Hopping Example

   This example transmits packets using SX1276 LoRa radio module.
   Each packet contains up to 256 bytes of data, in the form of:
    - Arduino String
    - null-terminated char array (C-string)
    - arbitrary binary data (byte array)

   Other modules from SX127x/RFM9x family can also be used.

   For default module settings, see the wiki page
   https://github.com/jgromes/RadioLib/wiki/Default-configuration#sx127xrfm9x---lora-modem

   For full API reference, see the GitHub Pages
   https://jgromes.github.io/RadioLib/

   SX127x supports FHSS or Frequency Hopping Spread Spectrum.
   Once a hopping period is set and a transmission is started, the radio
   will begin triggering interrupts every hop period where the radio frequency
   is changed to the next channel.
*/

#include <Arduino.h>
#include <RadioLib.h>
#include <CircularBuffer.h>
#include <rtcmstreamsplitter.h>

#define LORA_HEADER "W3VC/1"
#define LORA_HEADER_LENGTH 6
#define LORA_PAYLOAD_LENGTH (255 - LORA_HEADER_LENGTH)
#define LORA_FIXED_FREQ 902.5 // MHz (902.5 to 927.5 valid in US)

typedef struct {
  byte length;
  byte header[LORA_HEADER_LENGTH];
  byte data[LORA_PAYLOAD_LENGTH];
} RadioMessage;

// SX1276 has the following connections:
// NSS pin:   10
// DIO0 pin:  26
// RESET pin: 25
// DIO1 pin:  27
SX1276 radio = new Module(10, 26, 25, 27);

// Create circular buffer to dispatch data
CircularBuffer<RadioMessage, 8> cbuffer;

// RTCM Stream Splitter
RTCMStreamSplitter splitter;

// flag to indicate that tx is in use
volatile bool transmittingFlag = false;

// flag to indicate frequency must be changed
volatile bool fhssChangeFlag = false;

// the channel frequencies can be generated randomly or hard coded
// NOTE: The frequency list MUST be the same on both sides!
float channels[] = {       902.3, 902.5, 902.7, 902.9, 
                    903.1, 903.3, 903.5, 903.7, 903.9,
                    904.1, 904.3, 904.5, 904.7, 904.9,
                    905.1, 905.3, 905.5, 905.7, 905.9,
                    906.1, 906.3, 906.5, 906.7, 906.9,
                    907.1, 907.3, 907.5, 907.7, 907.9,
                    908.1, 908.3, 908.5, 908.7, 908.9,
                    909.1, 909.3, 909.5, 909.7, 909.9,
                    910.1, 910.3, 910.5, 910.7, 910.9,
                    911.1, 911.3, 911.5, 911.7, 911.9,
                    912.1, 912.3, 912.5, 912.7, 912.9,
                    913.1, 913.3, 913.5, 913.7, 913.9,
                    914.1, 914.3, 914.5, 914.7, 914.9 };
int numberOfChannels = sizeof(channels) / sizeof(float);
uint8_t channel_indices[sizeof(channels) / sizeof(float)];

// counter to keep track of how many frequency hops were performed
int hopsCompleted = 0;

// counter that increments with each sent packet
int packetCounter = 0;

// save transmission state between loops
int transmissionState = RADIOLIB_ERR_NONE;

// timestamp of last received byte over serial
unsigned long lastByteMicros = 0;

// timestamp of start of last transmission
unsigned long lastTxMillis = 0;

//declare reset function at address 0
void(* resetFunc) (void) = 0;

// this function is called when a complete packet
// is transmitted by the module
void setTxFlag(void) {
  radio.finishTransmit();
  transmittingFlag = false;
}

// this function is called when FhssChangeChannel interrupt occurs
// (at the beginning of each transmission)
void setFHSSFlag(void) {
  fhssChangeFlag = true;
}

void setup_radio() {

  // begin radio on home channel
  Serial.print(F("[SX1276] Initializing ... "));

  #ifndef LORA_FIXED_FREQ
  int state = radio.begin(channels[channel_indices[0]], 125.0, 7, 5, RADIOLIB_SX127X_SYNC_WORD, 17, 8, 0);
  #else
  int state = radio.begin(LORA_FIXED_FREQ, 250.0, 7, 8, RADIOLIB_SX127X_SYNC_WORD, 17, 8, 0);
  #endif

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  // set output power to 10 dBm (accepted range is -3 - 17 dBm)
  // NOTE: 20 dBm value allows high power operation, but transmission
  //       duty cycle MUST NOT exceed 1%
  /*
  if (radio.setOutputPower(10) == RADIOLIB_ERR_INVALID_OUTPUT_POWER) {
    Serial.println(F("Selected output power is invalid for this module!"));
    while (true);
  }
  */

  // set the CRC to be used
  state = radio.setCRC(true);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }

  // set hop period in symbols
  // this will also enable FHSS
  #ifndef LORA_FIXED_FREQ
  state = radio.setFHSSHoppingPeriod(9);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  }
  #endif

  // set the function to call when transmission is finished
  radio.setDio0Action(setTxFlag);

  // set the function to call when we need to change frequency
  #ifndef LORA_FIXED_FREQ
  radio.setDio1Action(setFHSSFlag);
  #endif

  // set the control pins
  radio.setRfSwitchPins(8, 9);

  // set default values
  hopsCompleted = 0;
  transmittingFlag = false;
}

void setup() {
  Serial.begin(57600);

  // generate LFSR indexes (psuedorandom non-repeating [0, 63])
  memset(&channel_indices[0], 0, sizeof(channel_indices));
  channel_indices[1] = 29;
  for (int i = 2; i < numberOfChannels; i++) {
    bool mask = channel_indices[i-1] & 0x1;
    channel_indices[i] = channel_indices[i-1] >> 1;
    if (mask)
      channel_indices[i] ^= 0x30;
  }

  radio.reset();
  delay(1000);
  setup_radio();
}

void parse_rtcm(byte nextByte) {
  unsigned int type = splitter.inputByte(nextByte);
  if (type > 0) {
    unsigned int length = splitter.outputStreamLength;
    if (length > LORA_PAYLOAD_LENGTH) {
      Serial.println(F("[SX1276] RTCM Packet Larger than Max Packet"));
      return;
    }

    RadioMessage message = {
      .length = (byte) length,
    };
    memcpy(&message.header, (uint8_t*)LORA_HEADER, LORA_HEADER_LENGTH);
    memcpy(&message.data, &splitter.outputStream, length);
    cbuffer.push(message);
  }
}

void loop() {

  // check if the serial input has given bytes
  while (Serial.available() > 0) {
    char tempByte;
    int numBytesRead = Serial.readBytes(&tempByte, 1);
    if (numBytesRead > 0) {
      parse_rtcm(tempByte);
      lastByteMicros = micros();
    }
  }

  // check if the radio is malfunctioning
  bool txFrozen = (millis() - lastTxMillis > 1000) && transmittingFlag;
  bool hopFrozen = (hopsCompleted == 1) && !transmittingFlag;
  if (txFrozen || hopFrozen) {
    radio.reset();
    delay(50);
    //resetFunc();
    setup_radio();
  }
  
  // check if the transmission flag is set
  if (!transmittingFlag && (cbuffer.size() > 0)) {
    // reset flag
    lastTxMillis = millis();
    transmittingFlag = true;

    if (transmissionState == RADIOLIB_ERR_NONE) {
      // packet was successfully sent
      Serial.println(F("transmission finished!"));
    } else {
      Serial.print(F("failed, code "));
      Serial.println(transmissionState);
    }

    Serial.print(F("[SX1276] Items waiting in queue: "));
    Serial.println(cbuffer.size());

    #ifndef LORA_FIXED_FREQ
    // The channel is automatically reset to 0 upon completion
    Serial.print(F("[SX1276] Radio is on channel: "));
    Serial.println(radio.getFHSSChannel());

    // print the number of hops it took
    Serial.print(F("[SX1276] Hops completed: "));
    Serial.println(hopsCompleted);

    // reset the counter
    hopsCompleted = 0;

    // return to home channel before the next transaction
    radio.setFrequency(channels[radio.getFHSSChannel() % numberOfChannels]);
    #endif

    // increment the packet counter
    packetCounter++;

    // load packet
    RadioMessage message = cbuffer.shift();

    // send another packet
    Serial.print(F("[SX1276] Sending another packet of size "));
    Serial.print(message.length+LORA_HEADER_LENGTH);
    Serial.println(F("... "));
    transmissionState = radio.startTransmit(&message.header[0], message.length+LORA_HEADER_LENGTH);
  }

  #ifndef LORA_FIXED_FREQ
  // check if we need to do another frequency hop
  if (fhssChangeFlag == true) {
    // we do, change it now
    int state = radio.setFrequency(channels[radio.getFHSSChannel() % numberOfChannels]);
    if (state != RADIOLIB_ERR_NONE) {
      Serial.print(F("[SX1276] Failed to change frequency, code "));
      Serial.println(state);
    }

    // increment the counter
    hopsCompleted++;

    // clear the FHSS interrupt
    radio.clearFHSSInt();

    // we're ready to do another hop, clear the flag
    fhssChangeFlag = false;
  }
  #endif
}

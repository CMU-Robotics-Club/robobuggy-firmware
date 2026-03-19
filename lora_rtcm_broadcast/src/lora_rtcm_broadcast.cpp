/*
   RadioLib SX127x Transmit RTCM data received via serial stream.
*/

#include <Arduino.h>
#include <RadioLib.h>
#include <CircularBuffer.h>
#include <rtcmstreamsplitter.h>

#define LORA_HEADER "W3VC/1"
#define LORA_HEADER_LENGTH 6
#define LORA_PAYLOAD_LENGTH (255 - LORA_HEADER_LENGTH)
#define LORA_FIXED_FREQ 902.5 // MHz (902.5 to 927.5 valid in US)

#define LORA_TRANSMIT_RATE_MS 250
#define RTCM_BUFFER_SIZE 1

typedef struct
{
  byte length;
  byte header[LORA_HEADER_LENGTH];
  byte data[LORA_PAYLOAD_LENGTH];
} RadioMessage;

// SX1276 has the following connections:
// NSS pin:   10
// DIO0 pin:  26
// RESET pin: 25
// DIO1 pin:  27

// DON'T FORGET TO INCLUDE THE OTHER SPI CONNECTIONS:
// These pinouts are implictly defined *somewhere* deep in the Arduino libraries.
// SCLK pin:  13
// SDI pin:   11
// SDO pin:   12

// Also the RX and TX enable pins:
// You can see where these pins are defined in the call to the radio.setRfSwitchPins(8, 9) function below.
// RXEN:      8
// TXEN:      9
SX1276 radio = new Module(10, 26, 25, 27);

// Create circular buffer to dispatch data
CircularBuffer<RadioMessage, RTCM_BUFFER_SIZE> cbuffer;

// RTCM Stream Splitter
RTCMStreamSplitter splitter;

// flag to indicate that tx is in use
volatile bool transmittingFlag = false;

// counter that increments with each sent packet
int packetCounter = 0;

// save transmission state between loops
int transmissionState = RADIOLIB_ERR_NONE;

// timestamp of last received byte over serial
unsigned long lastByteMillis = 0;

// timestamp of start of last transmission
unsigned long lastTxMillis = 0;

// declare reset function at address 0
void (*resetFunc)(void) = 0;

/**
 * @brief
 * this function is called when a complete packet
 * is transmitted by the module
 */
void setTxFlag(void)
{
  radio.finishTransmit();
  transmittingFlag = false;
}

// this function is called when FhssChangeChannel interrupt occurs
// (at the beginning of each transmission)
void setFHSSFlag(void)
{
  // ignore.  we are not doing frequency hopping.
  return;
}

/**
 * @brief Initialize the radio.
 *
 */
void setup_radio()
{

  // begin radio on home channel
  cbuffer.clear();
  packetCounter = 0;
  lastByteMillis = millis(); // reset the timer here
  Serial.print("[SX1276] Initializing ... ");

#ifndef LORA_FIXED_FREQ
  int state = radio.begin(channels[channel_indices[0]], 125.0, 7, 5, RADIOLIB_SX127X_SYNC_WORD, 17, 8, 0);
#else
  int state = radio.begin(LORA_FIXED_FREQ, 250.0, 7, 8, RADIOLIB_SX127X_SYNC_WORD, 17, 8, 0);
#endif

  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.println(F("success!"));
  }
  else
  {

    Serial.printf("setup failed, code %d", state);
    Serial.println();
    Serial.println("Retrying setup in 5 seconds...");
    delay(5000);
    return;
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
  if (state == RADIOLIB_ERR_NONE)
  {
    Serial.println(F("success!"));
  }
  else
  {

    Serial.printf("CRC setup failed, code %d", state);
    Serial.println("Retrying setup in 5 seconds...");
    delay(5000);
    return;
  }

  // set the function to call when transmission is finished
  radio.setDio0Action(setTxFlag);

// set the function to call when we need to change frequency
#ifndef LORA_FIXED_FREQ
  radio.setDio1Action(setFHSSFlag);
#endif

  // set the control pins
  radio.setRfSwitchPins(8, 9);

  transmittingFlag = false;
}

void setup()
{
  Serial.begin(57600);

  radio.reset();
  delay(1000);
  setup_radio();
}

void parse_rtcm(byte nextByte)
{
  unsigned int type = splitter.inputByte(nextByte);
  if (type > 0)
  {
    unsigned int length = splitter.outputStreamLength;
    if (length > LORA_PAYLOAD_LENGTH)
    {
      Serial.println(F("RTCM Packet Larger than Max Packet"));
      return;
    }

    RadioMessage message = {
        .length = (byte)length,
    };
    memcpy(&message.header, (uint8_t *)LORA_HEADER, LORA_HEADER_LENGTH);
    memcpy(&message.data, &splitter.outputStream, length);
    cbuffer.push(message);
  }
}

void loop()
{

  // check if the serial input has given bytes
  while (Serial.available() > 0)
  {
    char tempByte;
    int numBytesRead = Serial.readBytes(&tempByte, 1);
    if (numBytesRead > 0)
    {
      parse_rtcm(tempByte);
      lastByteMillis = millis();
    }
  }

  // check if the radio is malfunctioning
  bool txFrozen = (millis() - lastTxMillis > 500) && transmittingFlag;
  bool noSerial = (millis() - lastByteMillis) > 3000;
  if (txFrozen || noSerial)
  {
    Serial.printf("Problem detected. Time since last serial message = %d ms.  Restarting radio in 3 seconds...", millis() - lastByteMillis);
    Serial.println();
    delay(3000);
    radio.reset();
    delay(50);
    setup_radio();
  }

  // check if the transmission flag is set
  // check if there is data to transmit
  // check if it's been at least LORA_TRANSMIT_RATE_MS since the beginning of the last transmission.
  if (!transmittingFlag && (cbuffer.size() > 0) && (millis() - lastTxMillis > LORA_TRANSMIT_RATE_MS))
  {
    // reset flag
    transmittingFlag = true;

    if (transmissionState == RADIOLIB_ERR_NONE)
    {
      // packet was successfully sent
      Serial.println(F("transmission finished!"));
    }
    else
    {

      Serial.printf("transmit failed, code %d", transmissionState);
      Serial.println();
    }

    // get packet from buffer
    RadioMessage message = cbuffer.pop();
    // delete, and do not send, old packets
    cbuffer.clear();

    // send packet
    Serial.printf("[%lu ms]\tSending packet number %d of size %d...", millis(), packetCounter, message.length + LORA_HEADER_LENGTH);
    Serial.println();
    lastTxMillis = millis();
    // increment the packet counter
    packetCounter++;
    transmissionState = radio.startTransmit(&message.header[0], message.length + LORA_HEADER_LENGTH);
  }
}

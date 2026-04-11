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

#define LORA_HEADER "W3VC/1"
#define LORA_HEADER_LENGTH 6
#define LORA_PAYLOAD_LENGTH (255 - LORA_HEADER_LENGTH)
#define LORA_FIXED_FREQ 902.5

#define USE_USBCON
#define GPS_SERIAL Serial1

#include <Arduino.h>
#include <RadioLib.h>

char debug_chars[64];

typedef struct
{
  byte length;
  byte header[LORA_HEADER_LENGTH];
  byte data[LORA_PAYLOAD_LENGTH];
} RadioMessage;

// SX1276 has the following connections:
// NSS pin:   8
// DIO0 pin:  3
// RESET pin: 4
// DIO1 pin:  14
SX1276 radio = new Module(8, 3, 4, 14);

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// flag to indicate frequency must be changed
volatile bool fhssChangeFlag = false;

unsigned long pkt_rx_timestamp_ms; // timestamp in ms when last rtcm packet was received.
unsigned long pkt_rx_duration_ms;  // the number of ms it took to receive the most recent rtcm packet.

// the channel frequencies can be generated randomly or hard coded
// NOTE: The frequency list MUST be the same on both sides!
float channels[] = {902.3, 902.5, 902.7, 902.9,
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
                    914.1, 914.3, 914.5, 914.7, 914.9};
int numberOfChannels = sizeof(channels) / sizeof(float);
uint8_t channel_indices[sizeof(channels) / sizeof(float)];

// counter to keep track of how many frequency hops were performed
int hopsCompleted = 0;

// cache for transmitting over radio
RadioMessage message;

// this function is called when a complete packet
// is received by the module
void setRxFlag(void)
{
  pkt_rx_duration_ms = millis() - pkt_rx_timestamp_ms;
  receivedFlag = true;
  digitalWrite(LED_BUILTIN, HIGH);
}

// this function is called when FhssChangeChannel interrupt occurs
// (at the beginning of each transmission)
void setFHSSFlag(void)
{
  fhssChangeFlag = true;
}

// Helper function to extract a 30-bit field from bit position in RTCM payload
uint32_t extractBits30(const byte *data, int startBit)
{
  uint32_t value = 0;
  for (int i = 0; i < 30; i++)
  {
    int byteIdx = (startBit + i) / 8;
    int bitIdx = 7 - ((startBit + i) % 8);
    value = (value << 1) | ((data[byteIdx] >> bitIdx) & 1);
  }
  return value;
}

// Helper function to extract a 6-bit field from bit position in RTCM payload
uint8_t extractBits6(const byte *data, int startBit)
{
  uint8_t value = 0;
  for (int i = 0; i < 6; i++)
  {
    int byteIdx = (startBit + i) / 8;
    int bitIdx = 7 - ((startBit + i) % 8);
    value = (value << 1) | ((data[byteIdx] >> bitIdx) & 1);
  }
  return value;
}

// Helper function to extract and display time information from RTCM messages
void displayRTCMTime(uint16_t msgID, const byte *rtcmData, size_t length)
{
  // GPS observation messages (1001-1004, 1005-1006, 1030)
  if ((msgID >= 1001 && msgID <= 1006) || msgID == 1030)
  {
    if (length >= 7) // Need at least 7 bytes for header + time field
    {
      // GPS Time of Week (TOW) is 30 bits starting at bit position 12 (after the 12-bit message ID)
      uint32_t tow_ms = extractBits30(rtcmData, 12);

      // Convert TOW from milliseconds to hours, minutes, seconds
      unsigned long total_seconds = tow_ms / 1000;
      unsigned int hours = (total_seconds / 3600) % 24;
      unsigned int minutes = (total_seconds / 60) % 60;
      unsigned int seconds = total_seconds % 60;
      unsigned int milliseconds = tow_ms % 1000;

      Serial.printf("GPS Time of Week: %02d:%02d:%02d.%03d (TOW: %lu ms)\n",
                    hours, minutes, seconds, milliseconds, tow_ms);

      // Number of satellites (6 bits at position 42)
      uint8_t numSats = extractBits6(rtcmData, 42);
      Serial.printf("Number of Satellites: %d\n", numSats);
    }
  }
  // GLONASS observation messages (1009-1012)
  else if (msgID >= 1009 && msgID <= 1012)
  {
    if (length >= 7)
    {
      // GLONASS Time of Day (TOD) is 27 bits starting at bit position 12
      // Convert to hours, minutes, seconds
      uint32_t tod_ms = 0;
      for (int i = 0; i < 27; i++)
      {
        int byteIdx = (12 + i) / 8;
        int bitIdx = 7 - ((12 + i) % 8);
        tod_ms = (tod_ms << 1) | ((rtcmData[byteIdx] >> bitIdx) & 1);
      }

      unsigned long total_seconds = tod_ms / 1000;
      unsigned int hours = (total_seconds / 3600) % 24;
      unsigned int minutes = (total_seconds / 60) % 60;
      unsigned int seconds = total_seconds % 60;
      unsigned int milliseconds = tod_ms % 1000;

      Serial.printf("GLONASS Time of Day: %02d:%02d:%02d.%03d (TOD: %lu ms)\n",
                    hours, minutes, seconds, milliseconds, tod_ms);

      // Number of satellites (6 bits at position 39)
      uint8_t numSats = extractBits6(rtcmData, 39);
      Serial.printf("Number of Satellites: %d\n", numSats);
    }
  }
}

// Function to display RTCM data in human-readable format
void displayRTCMData(const byte *rtcmData, size_t length)
{
  if (length < 3)
  {
    Serial.println("RTCM: Data too short");
    return;
  }

  // RTCM structure:
  // Byte 0: Preamble (0xD3)
  // Bytes 1-2: Reserved (6 bits) + Length (10 bits)
  // Bytes 3+: Message ID (12 bits) + Payload
  // Last 3 bytes: CRC24

  Serial.println("=== RTCM Data ===");

  // Check for RTCM preamble
  if (rtcmData[0] != 0xD3)
  {
    Serial.printf("Warning: Invalid RTCM preamble (0x%02X, expected 0xD3)\n", rtcmData[0]);
  }

  // Extract message length (10 bits from bytes 1-2)
  uint16_t msgLength = ((rtcmData[1] & 0x03) << 8) | rtcmData[2];
  Serial.printf("Message Length: %d bytes\n", msgLength);

  if (msgLength + 6 > length)
  {
    Serial.printf("Warning: Declared length (%d) exceeds available data (%zu)\n", msgLength + 6, length);
    return;
  }

  // Extract message ID (first 12 bits of payload)
  uint16_t msgID = ((rtcmData[3] << 4) | (rtcmData[4] >> 4)) & 0x0FFF;
  Serial.printf("Message ID (Type): %d\n", msgID);

  // Display message type description
  const char *msgType = "Unknown";
  switch (msgID)
  {
  case 1001:
    msgType = "L1-only GPS RTK Observation Data";
    break;
  case 1002:
    msgType = "Extended L1-only GPS RTK Observation Data";
    break;
  case 1003:
    msgType = "L1/L2 GPS RTK Observation Data";
    break;
  case 1004:
    msgType = "Extended L1/L2 GPS RTK Observation Data";
    break;
  case 1005:
    msgType = "Stationary RTK Reference Station ARP";
    break;
  case 1006:
    msgType = "Stationary RTK Reference Station ARP with Antenna Height";
    break;
  case 1007:
    msgType = "Antenna Descriptor";
    break;
  case 1008:
    msgType = "Antenna Descriptor with Serial Number";
    break;
  case 1009:
    msgType = "L1-only GLONASS RTK Observation Data";
    break;
  case 1010:
    msgType = "Extended L1-only GLONASS RTK Observation Data";
    break;
  case 1011:
    msgType = "L1/L4 GLONASS RTK Observation Data";
    break;
  case 1012:
    msgType = "Extended L1/L4 GLONASS RTK Observation Data";
    break;
  case 1013:
    msgType = "System Parameter Message";
    break;
  case 1014:
    msgType = "Network Auxiliary Station Data";
    break;
  case 1015:
    msgType = "GPS Ionospheric Correction Differences";
    break;
  case 1016:
    msgType = "GPS Geometric Correction Differences";
    break;
  case 1017:
    msgType = "GPS Combined Correction Differences";
    break;
  case 1019:
    msgType = "GPS Ephemerides";
    break;
  case 1020:
    msgType = "GLONASS Ephemerides";
    break;
  case 1021:
    msgType = "Helmert/Similarity Transformation Parameters";
    break;
  case 1022:
    msgType = "Moledenski-Badekas Transformation Parameters";
    break;
  case 1023:
    msgType = "Residuals, Ellipsoidal Grid Representation";
    break;
  case 1024:
    msgType = "Residuals, Plane Grid Representation";
    break;
  case 1025:
    msgType = "Projection Parameters, Cassini-Soldner";
    break;
  case 1026:
    msgType = "Projection Parameters, Transverse Mercator";
    break;
  case 1027:
    msgType = "Projection Parameters, Transverse Mercator";
    break;
  case 1029:
    msgType = "Unicode Text String";
    break;
  case 1030:
    msgType = "GPS Network Geometric Station Data";
    break;
  case 1031:
    msgType = "Glonass Network Geometric Station Data";
    break;
  case 1032:
    msgType = "Combined GPS and Glonass Network Geometric Station Data";
    break;
  case 1033:
    msgType = "Receiver and Software Descriptor";
    break;
  case 1034:
    msgType = "GPS Network Combination RTK Observation Data";
    break;
  case 1035:
    msgType = "Glonass Network Combination RTK Observation Data";
    break;
  case 1087:
    msgType = "BeiDou RTK Observation Data";
    break;
  case 1127:
    msgType = "Galileo RTK Observation Data";
    break;
  case 1230:
    msgType = "GLONASS L1 and L2 Code-Phase Biases";
    break;

  default:
    if (msgID >= 4000 && msgID <= 4095)
      msgType = "Reserved (4000-4095)";
    else if (msgID >= 1000 && msgID <= 1299)
      msgType = "Observation Data";
    break;
  }
  Serial.printf("Type Description: %s\n", msgType);

  // Extract and display time information if available
  displayRTCMTime(msgID, &rtcmData[3], length - 3);

  // Display raw hex data
  Serial.printf("Hex Data: ");
  for (size_t i = 0; i < length && i < 60; i++)
  {
    Serial.printf("%02X ", rtcmData[i]);
    if ((i + 1) % 16 == 0 && i + 1 < length)
      Serial.print("\n              ");
  }
  if (length > 60)
    Serial.print("...");
  Serial.println();

  // Display CRC24 if available
  if (length >= 3)
  {
    uint32_t crc24 = ((rtcmData[length - 3] << 16) | (rtcmData[length - 2] << 8) | rtcmData[length - 1]) & 0xFFFFFF;
    Serial.printf("CRC24: 0x%06X\n", crc24);
  }

  Serial.println("==================\n");
}

void setup()
{
  Serial.begin(115200);

  // Set up RS232 data
  GPS_SERIAL.begin(115200);

  // generate LFSR indexes (psuedorandom non-repeating [0, 63])
  memset(&channel_indices[0], 0, sizeof(channel_indices));
  channel_indices[1] = 29;
  for (int i = 2; i < numberOfChannels; i++)
  {
    bool mask = channel_indices[i - 1] & 0x1;
    channel_indices[i] = channel_indices[i - 1] >> 1;
    if (mask)
      channel_indices[i] ^= 0x30;
  }

  // begin radio on home channel
  Serial.println("Initializing...");
#ifndef LORA_FIXED_FREQ
  int state = radio.begin(channels[channel_indices[0]], 125.0, 7, 5, RADIOLIB_SX127X_SYNC_WORD, 17, 8, 0);
#else
  int state = radio.begin(LORA_FIXED_FREQ, 250.0, 7, 8, RADIOLIB_SX127X_SYNC_WORD, 10, 8, 0);
#endif
  if (state != RADIOLIB_ERR_NONE)
  {
    snprintf(&debug_chars[0], 64, "failed, code %d", state);
    Serial.printf("failed, code %d\n", state);
    delay(1000);
    while (true)
      ;
  }

  // set the CRC to be used
  state = radio.setCRC(true);
  if (state != RADIOLIB_ERR_NONE)
  {
    snprintf(&debug_chars[0], 64, "failed, code %d", state);
    Serial.printf("failed, code %d\n", state);
    delay(1000);
    while (true)
      ;
  }

// set hop period in symbols
// this will also enable FHSS
#ifndef LORA_FIXED_FREQ
  state = radio.setFHSSHoppingPeriod(9);
  if (state != RADIOLIB_ERR_NONE)
  {
    snprintf(&debug_chars[0], 64, "failed, code %d", state);
    nh.logerror(debug_chars);
    while (true)
      ;
  }
#endif

  // set the function to call when reception is finished
  radio.setDio0Action(setRxFlag);

// set the function to call when we need to change frequency
#ifndef LORA_FIXED_FREQ
  radio.setDio1Action(setFHSSFlag);
#endif

  // start listening for LoRa packets
  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE)
  {
    snprintf(&debug_chars[0], 64, "failed, code %d", state);
    Serial.println(debug_chars);
    while (true)
      ;
  }

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  // check if the reception flag is set
  if (receivedFlag == true)
  {
    // we're ready to receive more packets, clear the flag
    receivedFlag = false;

    // you can read received data
    unsigned int length = radio.getPacketLength();
    int state = radio.readData(&message.header[0], length);
    message.length = length > LORA_HEADER_LENGTH ? length - LORA_HEADER_LENGTH : 0;
    float snr = radio.getSNR();
    float rssi = radio.getRSSI();

    Serial.printf("Packet length = %d\tSNR = %f\t RSSI = %f", length, snr, rssi);
    Serial.println();
    // Serial.println("Data:");
    // for (int i = 0; i < message.length - LORA_HEADER_LENGTH; i++)
    // {
    //   Serial.printf("%x ", message.data[i]);
    // }
    // Serial.println();

    // Serial.printf("Received %d bytes in %lu ms. %2.3f ms per byte.", message.length + LORA_HEADER_LENGTH, pkt_rx_duration_ms, (float)((float)pkt_rx_duration_ms / (float)(message.length+LORA_HEADER_LENGTH)));
    // Serial.println();

    // put the module back to listen mode
    radio.startReceive();
    pkt_rx_timestamp_ms = millis();

    if (state == RADIOLIB_ERR_NONE && message.length > 0)
    {
      // packet was successfully received
      displayRTCMData(&message.data[0], message.length);
      GPS_SERIAL.write(&message.data[0], message.length);
      digitalWrite(LED_BUILTIN, LOW);
    }

#ifndef LORA_FIXED_FREQ
    // reset the counter
    hopsCompleted = 0;
#endif
  }

#ifndef LORA_FIXED_FREQ
  // check if we need to do another frequency hop
  if (fhssChangeFlag == true)
  {
    // we do, change it now
    int state = radio.setFrequency(channels[radio.getFHSSChannel() % numberOfChannels]);
    if (state != RADIOLIB_ERR_NONE)
    {
      snprintf(&debug_chars[0], 64, "[SX1276] Failed to change frequency, code %d", state);
      nh.logerror(debug_chars);
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

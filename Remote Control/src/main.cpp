#include <ArduinoCRSF.h>
#include <HardwareSerial.h>

#define PIN_RX 24
#define PIN_TX 25

// Set up a new Serial object
HardwareSerial& crsfSerial = Serial6;
ArduinoCRSF crsf;

void setup()
{
  Serial.begin(115200);
  Serial.println("COM Serial initialized");
  
  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1);
  if (!crsfSerial) while (1) Serial.println("Invalid crsfSerial configuration");

  crsf.begin(crsfSerial);
}

//Use crsf.getChannel(x) to get us channel values (1-16).
void printChannels()
{
  for (int ChannelNum = 1; ChannelNum <= 16; ChannelNum++)
  {
    Serial.print(crsf.getChannel(ChannelNum));
    Serial.print(", ");
  }
  Serial.println(" ");
}

void loop()
{
    // Must call crsf.update() in loop() to process data
    crsf.update();
    printChannels();
}
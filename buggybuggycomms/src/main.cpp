#include <Arduino.h>

#include <SPI.h>
#include <RH_RF69.h>

/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 434.0
#define RFM69_CS 2
#define RFM69_INT 3
#define RFM69_RST 4

RH_RF69 rf69(RFM69_CS, RFM69_INT);

int packetNum = 0;

void transmit_location()
{
  char radiopacket[25] = "location data here";
  itoa(packetNum++, radiopacket+13, 10);
  Serial.print("Sending "); Serial.println(radiopacket);
  // Send a message!
  rf69.send((uint8_t *)radiopacket, strlen(radiopacket));
  rf69.waitPacketSent();
}

void transmit_steering_angle()
{
  char radiopacket[25] = "steering angle data here";
  itoa(packetNum++, radiopacket+13, 10);
  Serial.print("Sending "); Serial.println(radiopacket);
  // Send a message!
  rf69.send((uint8_t *)radiopacket, strlen(radiopacket));
  rf69.waitPacketSent();
}

void setup() {
  Serial.begin(115200);
  //while (!Serial) delay(1); // Wait for Serial Console (comment out line if no computer)

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, LOW);

  Serial.println("Feather RFM69 TX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM69_RST, HIGH);
  delay(10);
  digitalWrite(RFM69_RST, LOW);
  delay(10);

  if (!rf69.init()) {
	while (1) {
		Serial.println("RFM69 radio init failed");
		delay(100);
	}
  }
  Serial.println("RFM69 radio init OK!");
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
  // No encryption
  if (!rf69.setFrequency(RF69_FREQ)) {
    while (1) {
      Serial.println("setFrequency failed");
    }
  }
  if (!rf69.setModemConfig(RH_RF69::GFSK_Rb250Fd250)) {
    while (1) {
      Serial.println("setModem failed");
    }
  }

  // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
  // ishighpowermodule flag set like this:
  rf69.setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

  Serial.print("RFM69 radio @");  Serial.print((int)RF69_FREQ);  Serial.println(" MHz");
}

#define TRANSMIT

void loop() {
  delay(1000);  // Wait 1 second between transmits, could also 'sleep' here!

  #ifdef TRANSMIT

  if(packetNum%2==0){
    transmit_location();
  } else{
    transmit_steering_angle();
  }
  packetNum++;
  #else

  // Now wait for a reply
  uint8_t buf[RH_RF69_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf69.waitAvailableTimeout(800)) {
    // Should be a reply message for us now
    if (rf69.recv(buf, &len)) {
      Serial.print("Got a reply: ");
      Serial.println((char*)buf);
    } else {
      Serial.println("Receive failed");
    }
  } else {
    Serial.println("No reply, is another RFM69 listening?");
  }

  Serial.println(rf69.rssiRead());

  #endif
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}
#include "buggyradio.h"

#include <Arduino.h>

#include <SPI.h>
#include <RH_RF69.h>


/************ Radio Setup ***************/

// Change to 434.0 or other frequency, must match RX's freq!
#define RF69_FREQ 434.0

RH_RF69 *rf69 = nullptr;

// put function declarations here:
int myFunction(int, int);

void radio_init(int pin_cs, int pin_int, int pin_rst) {
	pinMode(pin_rst, OUTPUT);
	digitalWrite(pin_rst, LOW);

	digitalWrite(pin_rst, HIGH);
	delay(10);
	digitalWrite(pin_rst, LOW);
	delay(10);

	rf69 = new RH_RF69(pin_cs, pin_int);

	if (!rf69->init()) {
		while (1) {
			Serial.println("RFM69 radio init failed");
			delay(100);
		}
	}

    // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM (for low power module)
    // No encryption
    if (!rf69->setFrequency(RF69_FREQ)) {
        while (1) {
            Serial.println("setFrequency failed");
            delay(100);
        }
    }

    if (!rf69->setModemConfig(RH_RF69::GFSK_Rb250Fd250)) {
        while (1) {
            Serial.println("setModem failed");
            delay(100);
        }
    }

    // If you are using a high power RF69 eg RFM69HW, you *must* set a Tx power with the
    // ishighpowermodule flag set like this:
    rf69->setTxPower(20, true);  // range from 14-20 for power, 2nd arg must be true for 69HCW

	Serial.println("RFM69 radio init OK!");
}

void radio_transmit(const uint8_t *data, size_t size) {
	rf69->waitPacketSent();
	rf69->send(data, size);
}

void radio_send_gps(double x, double y, uint64_t gps_time, uint8_t fix) {
    Packet p{};
    p.tag = GPS_X_Y;
    p.gps_x_y.x = x;
    p.gps_x_y.y = y;
    p.gps_x_y.time = gps_time;
    p.gps_x_y.fix = fix;
    radio_transmit((uint8_t*)&p, sizeof(p));
}

void radio_send_steering(double angle) {
    Packet p{};
    p.tag = STEER_ANGLE;
    p.steer_angle.degrees = angle;
    radio_transmit((uint8_t*)&p, sizeof(p));
}

std::optional<uint8_t> radio_receive(uint8_t *data) {
	uint8_t len = 255;
	if (rf69->recv(data, &len)) {
		return { len };
	} else {
		return std::nullopt;
	}
}

bool radio_available() {
	return rf69->available();
}
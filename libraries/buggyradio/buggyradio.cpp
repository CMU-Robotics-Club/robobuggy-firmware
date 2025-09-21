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

    if (!rf69->setModemConfig(RH_RF69::GFSK_Rb4_8Fd9_6)) {
    /*	GFSK_Rb2Fd5,	    < GFSK, Whitening, Rb = 2kbs,    Fd = 5kHz
        GFSK_Rb2_4Fd4_8,    < GFSK, Whitening, Rb = 2.4kbs,  Fd = 4.8kHz
        GFSK_Rb4_8Fd9_6,    < GFSK, Whitening, Rb = 4.8kbs,  Fd = 9.6kHz
        GFSK_Rb9_6Fd19_2,   < GFSK, Whitening, Rb = 9.6kbs,  Fd = 19.2kHz */
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

bool radio_transmit(const uint8_t *data, size_t size) {
	if (rf69->waitPacketSent(10)) {
        rf69->send(data, size);
        return true;
    } else {
        Serial.println("================ MISSED PACKET ===============");
        return false;
    }
}

static int radio_seq_number = 0;

bool radio_send_gps(double x, double y, uint32_t gps_seq_number, uint8_t fix) {
    Packet p{};
    p.tag = GPS_X_Y;
    p.seq = radio_seq_number++;
    p.gps_x_y.x = x;
    p.gps_x_y.y = y;
    p.gps_x_y.gps_seq = gps_seq_number;
    p.gps_x_y.fix = fix;
    return radio_transmit((uint8_t*)&p, sizeof(p));
}

void radio_send_steering(double angle) {
    Packet p{};
    p.tag = STEER_ANGLE;
    p.seq = radio_seq_number++;
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

int16_t radio_last_rssi() {
    return rf69->lastRssi();
}

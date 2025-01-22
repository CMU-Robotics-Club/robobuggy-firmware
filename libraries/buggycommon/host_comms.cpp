#include "host_comms.h"
#include "crc.h"
#include <Arduino.h>

#include <array>

#define SYNC_LEN 4

#define COMM_SERIAL Serial1
#define COMM_BAUDRATE 1000000

#define MAX_PAYLOAD_SIZE 100

namespace {

const std::array<uint8_t, SYNC_LEN> SYNC_WORD = { 0xAA, 0xFF, 0x00, 0x55 };

#define PACK_MSG_TYPE(c1, c2) ( ((uint16_t)c1) | (((uint16_t)c2) << 8) )

enum MessageType : uint16_t {
    NAND_DebugInfo = PACK_MSG_TYPE('N','D'),
    NAND_UKFPacket = PACK_MSG_TYPE('N','U'),
    NAND_RawGPS = PACK_MSG_TYPE('N','G'),
    SC_DebugInfo = PACK_MSG_TYPE('S','D'),
    SC_Sensors = PACK_MSG_TYPE('S','S'),
    SC_Nand_Pos = PACK_MSG_TYPE('N','P'),
    Timestamp = PACK_MSG_TYPE('R','T'),
    Soft_Angle = PACK_MSG_TYPE('S','T'),
    Soft_Alarm = PACK_MSG_TYPE('A','L'),
    Soft_Timestamp = PACK_MSG_TYPE('T','M'),
    Soft_Brake = PACK_MSG_TYPE('B','R')
};

void write_and_checksum(const std::uint8_t *data, std::size_t size, Crc16 &crc) {
    COMM_SERIAL.write(data, size);
    crc.update(data, size);
}

void write_and_checksum(uint16_t data, Crc16 &crc) {
    write_and_checksum((const uint8_t *)&data, sizeof(data), crc);
}

void write_and_checksum(uint32_t data, Crc16 &crc) {
    write_and_checksum((const uint8_t *)&data, sizeof(data), crc);
}

void write_and_checksum(double data, Crc16 &crc) {
    write_and_checksum((const uint8_t *)&data, sizeof(data), crc);
}

void read_and_checksum(std::uint8_t *data, std::size_t size, Crc16 &crc) {
    COMM_SERIAL.readBytes(data, size);
    crc.update(data, size);
}

class Parser {
    // The current state is what we want to see next,
    // so e.g. RxState::Header means we've seen 4 sync bytes and want to see the message type and length
    enum class State {
        Sync0,
        Sync1,
        Sync2,
        Sync3,
        Header,
        Payload,

    } state;

public:
    uint8_t msg_buf[MAX_PAYLOAD_SIZE] = { 0 };

    uint16_t msg_type;

    // Includes only the body of the message
    uint16_t msg_len;

    Crc16 checksum;

    Parser() : state(State::Sync0) {}

    // Returns true if a packet finished
    bool update() {
        int c;

        while (1) {
            switch (state) {
            case State::Sync0:
                c = COMM_SERIAL.read();
                if (c == SYNC_WORD[0]) {
                    state = State::Sync1;
                    break;
                } else if (c != -1) {
                    Serial.printf("Invalid 1st byte of sync %x\n", (unsigned)c);
                    state = State::Sync0;
                    return false;
                }
                return false;
            case State::Sync1:
                c = COMM_SERIAL.read();
                if (c == SYNC_WORD[1]) {
                    state = State::Sync2;
                    break;
                } else if (c != -1) {
                    Serial.printf("Invalid 2nd byte of sync %x\n", (unsigned)c);
                    state = State::Sync0;
                    return false;
                }
                return false;
            case State::Sync2:
                c = COMM_SERIAL.read();
                if (c == SYNC_WORD[2]) {
                    state = State::Sync3;
                    break;
                } else if (c != -1) {
                    Serial.printf("Invalid 3rd byte of sync %x\n", (unsigned)c);
                    state = State::Sync0;
                    return false;
                }
                return false;
            case State::Sync3:
                c = COMM_SERIAL.read();
                if (c == SYNC_WORD[3]) {
                    state = State::Header;
                    break;
                } else if (c != -1) {
                    Serial.println("Invalid 4th byte of sync");
                    state = State::Sync0;
                    return false;
                }
                return false;
            case State::Header:
                checksum.accum = 0;
                if (COMM_SERIAL.available() >= (int)(sizeof(msg_type) + sizeof(msg_len))) {
                    read_and_checksum((uint8_t *)&msg_type, sizeof(msg_type), checksum);
                    read_and_checksum((uint8_t *)&msg_len,  sizeof(msg_len),  checksum);
                    if (msg_len > MAX_PAYLOAD_SIZE) {
                        // Yikes
                        Serial.println("Invalid length");
                        state = State::Sync0;
                        return false;
                    }
                    state = State::Payload;
                    break;
                }
                return false;
            case State::Payload:
                if (COMM_SERIAL.available() >= (int)(msg_len + sizeof(uint16_t))) {
                    read_and_checksum(msg_buf, msg_len, checksum);

                    uint16_t rx_checksum;
                    COMM_SERIAL.readBytes((uint8_t *)&rx_checksum, sizeof(rx_checksum));

                    // Whether the checksum is right or wrong, it's still the end of the packet
                    state = State::Sync0;

                    if (rx_checksum == checksum.accum) {
                        return true;
                    }

                    Serial.println("Invalid checksum byte");
                }
                return false;
            }
        }
    }
};

}

namespace host_comms {

static uint8_t rxbuf[1024];

void init() {
    COMM_SERIAL.begin(COMM_BAUDRATE);
    COMM_SERIAL.setTimeout(0);
    COMM_SERIAL.addMemoryForRead(rxbuf, sizeof(rxbuf));
}

static uint32_t LAST_MESSAGE = 0;

static double STEERING_ANGLE = 0.0;
static int SOFT_TIME = 0;

static AlarmStatus ALARM_STATUS = AlarmStatus::Ok;

void poll() {
    static Parser parser = {};

    while (parser.update()) {
        // We got a new packet
        if (parser.msg_type == MessageType::Soft_Angle) {
            memcpy(&STEERING_ANGLE, parser.msg_buf, sizeof(STEERING_ANGLE));
            LAST_MESSAGE = millis();
        } else if (parser.msg_type == MessageType::Soft_Brake) {
            // TODO: decide on a format
            // NOTE: At the moment, we have decided against software having the ability to brake
            Serial.println("Brake packet");

            LAST_MESSAGE = millis();
        } else if (parser.msg_type == MessageType::Soft_Alarm) {
            ALARM_STATUS = (AlarmStatus)parser.msg_buf[0];
            LAST_MESSAGE = millis();
        } else if (parser.msg_type==MessageType::Soft_Timestamp) {
            SOFT_TIME = (int)parser.msg_buf[0];
            LAST_MESSAGE = millis();
        } else {
            Serial.println("Received an unknown packet");
        }
    }
}

uint32_t message_age() {
    return millis() - LAST_MESSAGE;
}

double steering_angle() {
    return STEERING_ANGLE;
}

AlarmStatus alarm_status() {
    return ALARM_STATUS;
}

int software_time() {
    return SOFT_TIME;
}

void nand_send_debug(NANDDebugInfo info) {
    Crc16 checksum = {};

    COMM_SERIAL.write(SYNC_WORD.data(), SYNC_WORD.size());
    write_and_checksum(MessageType::NAND_DebugInfo, checksum);
    write_and_checksum((uint16_t)sizeof(info), checksum);
    write_and_checksum(reinterpret_cast<uint8_t *>(&info), sizeof(info), checksum);
    COMM_SERIAL.write(reinterpret_cast<uint8_t *>(&checksum.accum), sizeof(checksum.accum));
}

void nand_send_ukf(NANDUKF info) {
    Crc16 checksum = {};

    COMM_SERIAL.write(SYNC_WORD.data(), SYNC_WORD.size());
    write_and_checksum(MessageType::NAND_UKFPacket, checksum);
    write_and_checksum((uint16_t)sizeof(info), checksum);
    write_and_checksum(reinterpret_cast<uint8_t *>(&info), sizeof(info), checksum);
    COMM_SERIAL.write(reinterpret_cast<uint8_t *>(&checksum.accum), sizeof(checksum.accum));
}

void nand_send_raw_gps(NANDRawGPS info) {
    Crc16 checksum = {};

    COMM_SERIAL.write(SYNC_WORD.data(), SYNC_WORD.size());
    write_and_checksum(MessageType::NAND_RawGPS, checksum);
    write_and_checksum((uint16_t)sizeof(info), checksum);
    write_and_checksum(reinterpret_cast<uint8_t *>(&info), sizeof(info), checksum);
    COMM_SERIAL.write(reinterpret_cast<uint8_t *>(&checksum.accum), sizeof(checksum.accum));
}

void sc_send_debug_info(SCDebugInfo info) {
    Crc16 checksum = {};

    COMM_SERIAL.write(SYNC_WORD.data(), SYNC_WORD.size());
    write_and_checksum(MessageType::SC_DebugInfo, checksum);
    write_and_checksum((uint16_t)sizeof(info), checksum);
    write_and_checksum(reinterpret_cast<uint8_t *>(&info), sizeof(info), checksum);
    COMM_SERIAL.write(reinterpret_cast<uint8_t *>(&checksum.accum), sizeof(checksum.accum));
}

void sc_send_sensors(SCSensors info) {
    Crc16 checksum = {};

    COMM_SERIAL.write(SYNC_WORD.data(), SYNC_WORD.size());
    write_and_checksum(MessageType::SC_Sensors, checksum);
    write_and_checksum((uint16_t)sizeof(info), checksum);
    write_and_checksum(reinterpret_cast<uint8_t *>(&info), sizeof(info), checksum);
    COMM_SERIAL.write(reinterpret_cast<uint8_t *>(&checksum.accum), sizeof(checksum.accum));
}

void send_timestamp(Roundtrip time) {
    Crc16 checksum = {};

    COMM_SERIAL.write(SYNC_WORD.data(), SYNC_WORD.size());
    write_and_checksum(MessageType::Timestamp, checksum);
    write_and_checksum((uint16_t)sizeof(time), checksum);
    write_and_checksum(reinterpret_cast<uint8_t *>(&time), sizeof(time), checksum);
    COMM_SERIAL.write(reinterpret_cast<uint8_t *>(&checksum.accum), sizeof(checksum.accum));
}

void sc_send_nand_pos(SCRadioRx nand) {
    Crc16 checksum = {};

    COMM_SERIAL.write(SYNC_WORD.data(), SYNC_WORD.size());
    write_and_checksum(MessageType::SC_Nand_Pos, checksum);
    write_and_checksum((uint16_t)sizeof(nand), checksum);
    write_and_checksum(reinterpret_cast<uint8_t *>(&nand), sizeof(nand), checksum);
    COMM_SERIAL.write(reinterpret_cast<uint8_t *>(&checksum.accum), sizeof(checksum.accum));
}

/*
void send_debug_info(DebugInfo info) {
    Crc16 checksum = {};

    COMM_SERIAL.write(SYNC_WORD.data(), SYNC_WORD.size());
    write_and_checksum(MessageType::DebugInfo, checksum);
    write_and_checksum((uint16_t)sizeof(info), checksum);
    write_and_checksum(reinterpret_cast< uint8_t* >(&info), sizeof(info), checksum);
    COMM_SERIAL.write(reinterpret_cast< uint8_t* >(&checksum.accum), sizeof(checksum.accum));
}

void send_nand_odometry(double x, double y, uint32_t radio_seq_num, uint32_t gps_seq_num) {
    Crc16 checksum = {};

    COMM_SERIAL.write(SYNC_WORD.data(), SYNC_WORD.size());
    write_and_checksum(MessageType::Odometry, checksum);
    write_and_checksum((uint16_t)(2 * sizeof(double) + sizeof(uint32_t) + sizeof(uint32_t)), checksum);
    write_and_checksum(reinterpret_cast< uint8_t* >(&x), sizeof(x), checksum);
    write_and_checksum(reinterpret_cast< uint8_t* >(&y), sizeof(y), checksum);
    write_and_checksum(radio_seq_num, checksum);
    write_and_checksum(gps_seq_num,   checksum);
    COMM_SERIAL.write(reinterpret_cast< uint8_t* >(&checksum.accum), sizeof(checksum.accum));
}

void send_bnya_telemetry(
	double x, double y,
	double velocity,
	double steering,
	double heading,
	double heading_rate
) {
    Crc16 checksum = {};

    COMM_SERIAL.write(SYNC_WORD.data(), SYNC_WORD.size());
    write_and_checksum(MessageType::BnyaTelem, checksum);
    write_and_checksum((uint16_t)(6 * sizeof(double)), checksum);
    write_and_checksum(x, checksum);
    write_and_checksum(y, checksum);
    write_and_checksum(velocity, checksum);
    write_and_checksum(steering, checksum);
    // The UKF coordinate system puts x north and y east,
    // which means that positive angle turns the buggy to the right.
    //
    // The autonomous coordinate system expects x east and y north,
    // which means that positive angle turns the buggy to the left.
    //
    // This converts the UKF heading to the autonomous heading.
    write_and_checksum(heading, checksum);
    write_and_checksum(heading_rate, checksum);
    COMM_SERIAL.write(reinterpret_cast< uint8_t* >(&checksum.accum), sizeof(checksum.accum));
}
*/
}

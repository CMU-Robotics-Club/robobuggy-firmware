#include "host_comms.h"
#include "crc.h"
#include <Arduino.h>

#include <array>

#define SYNC_LEN 4

#define COMM_SERIAL Serial1

#define MAX_PAYLOAD_SIZE 100

namespace {

const std::array<uint8_t, SYNC_LEN> SYNC_WORD = { 0xAA, 0xFF, 0x00, 0x55 };

#define PACK_MSG_TYPE(c1, c2) ( ((uint16_t)c1) | (((uint16_t)c2) << 8) )

enum MessageType : uint16_t {
    DebugInfo = PACK_MSG_TYPE('D', 'B'),
    Odometry  = PACK_MSG_TYPE('O', 'D'),
    Steering  = PACK_MSG_TYPE('S', 'T'),
    Brake     = PACK_MSG_TYPE('B', 'R')
};

void write_and_checksum(const std::uint8_t *data, std::size_t size, Crc16 &crc) {
    COMM_SERIAL.write(data, size);
    crc.update(data, size);
}

void write_and_checksum(uint16_t data, Crc16 &crc) {
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

void init() {
    COMM_SERIAL.begin(1000000);
    COMM_SERIAL.setTimeout(0);
}

static uint32_t LAST_MESSAGE = 0;

static double STEERING_ANGLE = 0.0;

void poll() {
    static Parser parser = {};

    while (parser.update()) {
        // We got a new packet
        if (parser.msg_type == MessageType::Steering) {
            memcpy(&STEERING_ANGLE, parser.msg_buf, sizeof(STEERING_ANGLE));
            LAST_MESSAGE = millis();
        } else if (parser.msg_type == MessageType::Brake) {
            Serial.println("Brake packet");
            // TODO: decide on a format
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


void send_debug_info(ros_comms::DebugInfo info) {
    Crc16 checksum = {};

    COMM_SERIAL.write(SYNC_WORD.data(), SYNC_WORD.size());
    write_and_checksum(MessageType::DebugInfo, checksum);
    write_and_checksum((uint16_t)sizeof(info), checksum);
    write_and_checksum(reinterpret_cast< uint8_t* >(&info), sizeof(info), checksum);
    COMM_SERIAL.write(reinterpret_cast< uint8_t* >(&checksum.accum), sizeof(checksum.accum));
}

void send_nand_odometry(double x, double y) {
    Crc16 checksum = {};

    COMM_SERIAL.write(SYNC_WORD.data(), SYNC_WORD.size());
    write_and_checksum(MessageType::Odometry, checksum);
    write_and_checksum((uint16_t)(2 * sizeof(double)), checksum);
    write_and_checksum(reinterpret_cast< uint8_t* >(&x), sizeof(x), checksum);
    write_and_checksum(reinterpret_cast< uint8_t* >(&y), sizeof(y), checksum);
    COMM_SERIAL.write(reinterpret_cast< uint8_t* >(&checksum.accum), sizeof(checksum.accum));
}

}

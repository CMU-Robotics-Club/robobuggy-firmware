#pragma once 

#include <Arduino.h>
#include <cstdint>
#include <Wire.h>

#define ENCODER_SERIAL Serial6
#define ENCODER_BAUDRATE 115200
#define SYNC_LEN 4

//NEW CODE FOR ENCODER, modeling after bnyahserial
namespace{

    const std::array<uint8_t, SYNC_LEN> SYNC_WORD = { 0xAA, 0xFF, 0x00, 0x55 };
    enum class State {
        Sync0,
        Sync1,
        Sync2,
        Sync3,
        Header,
        Payload
    } state;
    
    enum EncoderMessageHeader : uint8_t {
        ErrorInfo = 'E',
        SpeedInfo = 'S'
    };

    float speed;
   

    void poll() {
        int c;
        
        while (1) {
            switch (state) {
            case State::Sync0:
                c = ENCODER_SERIAL.read();
                if (c == SYNC_WORD[0]) {
                    state = State::Sync1;
                    break;
                } else if (c != -1) {
                    Serial.printf("Invalid 1st byte of sync %x\n", (unsigned)c);
                    state = State::Sync0;
                    return;
                }
                return;
            case State::Sync1:
                c = ENCODER_SERIAL.read();
                if (c == SYNC_WORD[1]) {
                    state = State::Sync2;
                    break;
                } else if (c != -1) {
                    Serial.printf("Invalid 2nd byte of sync %x\n", (unsigned)c);
                    state = State::Sync0;
                    return;
                }
                return;
            case State::Sync2:
                c = ENCODER_SERIAL.read();
                if (c == SYNC_WORD[2]) {
                    state = State::Sync3;
                    break;
                } else if (c != -1) {
                    Serial.printf("Invalid 3rd byte of sync %x\n", (unsigned)c);
                    state = State::Sync0;
                    return;
                }
                return;
            case State::Sync3:
                c = ENCODER_SERIAL.read();
                if (c == SYNC_WORD[3]) {
                    state = State::Header;
                    break;
                } else if (c != -1) {
                    Serial.printf("Invalid 4th byte of sync %x\n", (unsigned)c);
                    state = State::Sync0;
                    return;
                }
                return;
            case State::Header:
                c = ENCODER_SERIAL.read();
                switch (c) {
                    case EncoderMessageHeader::ErrorInfo:
                        break;
                    case EncoderMessageHeader::SpeedInfo:
                        ENCODER_SERIAL.readBytes(&speed, 4);
                        break;
                    default:
                        Serial.printf("Invalid header byte: %x\n", (unsigned)c);
                        state = State::Sync0;
                        return;
                }
            }
        }
    }

    float get_speed() {};





}


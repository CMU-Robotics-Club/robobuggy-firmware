#pragma once
#include <cstdint>
#include <cstddef>
#include <optional>

using std::uint8_t;
using std::size_t;

enum PacketType : uint32_t {
    GPS_X_Y,
    STEER_ANGLE
};

struct Packet {
    PacketType tag;
    union {
        struct { double x; double y; uint64_t time; uint8_t fix; uint8_t _pad[7]; } gps_x_y;
        struct { double degrees; } steer_angle;
    };
};

void radio_init(int pin_cs, int pin_int, int pin_rst);

bool radio_transmit(const uint8_t *data, size_t size);

bool radio_send_gps(double x, double y, uint64_t gps_time, uint8_t fix);

void radio_send_steering(double angle);

// Returns true on success
std::optional<uint8_t> radio_receive(uint8_t *data);

bool radio_available();

int16_t radio_last_rssi();
#pragma once
#include <cstdint>
#include <cstddef>
#include <optional>

using std::uint8_t;
using std::size_t;

void radio_init(int pin_cs, int pin_int, int pin_rst);

void radio_transmit(const uint8_t *data, size_t size);

void radio_send_gps(double x, double y, uint64_t gps_time);

void radio_send_steering(double angle);

// Returns true on success
std::optional<uint8_t> radio_receive(uint8_t *data);

bool radio_available();
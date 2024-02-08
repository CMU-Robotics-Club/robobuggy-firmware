#pragma once
#include <cstdint>
#include <cstddef>

using std::uint8_t;
using std::size_t;

void radio_init();

void radio_transmit(const uint8_t *data, size_t size);

void radio_send_gps(double x, double y, uint64_t gps_time);

void radio_send_steering(double angle);

// Returns true on success
bool radio_receive(uint8_t *data, size_t size);

bool radio_available();
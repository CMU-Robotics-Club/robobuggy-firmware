#pragma once
#include <cstdint>
#include <cstddef>

using std::uint8_t;
using std::size_t;

void radio_init();

void radio_transmit(const uint8_t *data, size_t size);

// Returns true on success
bool radio_receive(uint8_t *data, size_t size);

bool radio_available();
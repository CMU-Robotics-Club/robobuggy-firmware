#pragma once

#include <cstdint>
#include <cstddef>

using std::uint16_t;
using std::uint8_t;
using std::size_t;

struct Crc16 {
    Crc16() = default;

    void update(std::uint8_t data);
    void update(const std::uint8_t data[], std::size_t len);

    std::uint16_t accum = 0;
};
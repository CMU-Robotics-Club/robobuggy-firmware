#pragma once

#include <cstdint>
#include <optional>

struct GpsUpdate {
    double x;
    double y;
    uint64_t gps_time;
    uint8_t fix;
    uint8_t _pad[7];
};

using std::uint64_t;

void gps_init();

std::optional<GpsUpdate> gps_update();

uint64_t gps_time_millis();
#pragma once

#include <cstdint>
#include <optional>

struct GpsUpdate {
    double x;
    double y;
    uint64_t gps_time;
};

using std::uint64_t;

void gps_init();

std::optional<GpsUpdate> gps_update();

uint64_t gps_time_millis();
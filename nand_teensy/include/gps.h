#pragma once

#include <cstdint>
#include <optional>

struct GpsUpdate {
    double x;
    double y;
    double accuracy;
    uint8_t gps_SIV; // // Number of satellites used in fix (satellites-in-view)
    uint8_t gps_fix; // 0=no fix, 1=dead reckoning, 2=2D, 3=3D, 4=GNSS, 5=Time fix
    uint8_t rtk_fix; // 0, 1 or 2 for no RTK, float RTK, or fixed RTK solution, respectively
    uint8_t _pad[5];
};

using std::uint64_t;

void gps_init();

std::optional<GpsUpdate> gps_update();

uint64_t gps_time_millis();
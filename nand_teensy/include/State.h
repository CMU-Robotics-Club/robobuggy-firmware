#include "gps.h"
#include "ukf.h"


struct State {
    bool fresh_gps_data;
    GpsUpdate last_gps_data;
    int gps_sequence_number;
    elapsedMicros gps_update_elapsed;

    double heading_rate;
    elapsedMicros imu_elapsed;

    int rfm69_timeout;
    int radio_last_failed;
    elapsedMicros radio_send_elapsed;

    bool kalman_init;
    uint32_t last_predict_timestamp;
    
    bool use_autonomous_steering;
};
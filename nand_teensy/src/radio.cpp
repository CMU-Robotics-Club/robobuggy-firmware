#include <Arduino.h>
#include "RateLimit.h"
#include "History.h"

#include "buggyradio.h"
#include "gps.h"

void send_position_to_sc(RateLimit radio_tx_limit, 
                         bool fresh_gps_data, GpsUpdate last_gps_data, int gps_sequence_number,
                         bool use_autonomous_steering, 
                         int &rfm69_timeout, int &last_failed, 
                         History<uint32_t, 10> radio_send_history, elapsedMicros radio_send_elapsed) 
{
    int t1 = millis();
    if (radio_tx_limit.ready() || fresh_gps_data)
    {
        fresh_gps_data = false;
        radio_tx_limit.reset();
        int radio_t1 = millis();
        if (!radio_send_gps(last_gps_data.x, last_gps_data.y, gps_sequence_number, last_gps_data.fix, (uint8_t)use_autonomous_steering))
        {
        int radio_tF = millis() - radio_t1;
        if (radio_tF > 5)
            Serial.printf("Radio failed: %d\n", radio_tF);
        last_failed = millis();
        ++rfm69_timeout;
        }

        radio_send_history.push(radio_send_elapsed);

        static int aaa = 0;
        ++aaa;
    }

    // send bogus gps data over the RFM69 radio if we do not have fresh gps data!!
    if (radio_tx_limit.ready() && !fresh_gps_data)
    {
        radio_tx_limit.reset();
        int radio_t1 = millis();
        if (!radio_send_gps(0, 0, gps_sequence_number, 213, 0))
        {
        int radio_tF = millis() - radio_t1;
        Serial.printf("Radio failed: %d\n", radio_tF);
        last_failed = millis();
        }
        else
        Serial.printf("BNYAAAAHH RADIO SENT!!!!!!!!");

        radio_send_history.push(radio_send_elapsed);
    }

    int tF = millis() - t1;
    if (tF >= 5)
    {
        Serial.printf("Time took for Radio is %dms\n", tF);
    }
}
#include "brake.h"

#include <Arduino.h>

namespace brake {

int brake_relay_pin = -1;

void init(int brake_relay_pin_) {
    brake_relay_pin = brake_relay_pin_;

    pinMode(brake_relay_pin, OUTPUT);
}

void set(Status status) {
    digitalWrite(brake_relay_pin, (int)status);
}

} // namespace brake
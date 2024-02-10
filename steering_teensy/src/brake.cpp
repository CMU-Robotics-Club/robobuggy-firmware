#include "brake.h"

#include <Arduino.h>

namespace brake {

#define BRAKE_RELAY_PIN 26

void init() {
    pinMode(BRAKE_RELAY_PIN, OUTPUT);
}

void set(Status status) {
    digitalWrite(BRAKE_RELAY_PIN, (int)status);
}

} // namespace brake
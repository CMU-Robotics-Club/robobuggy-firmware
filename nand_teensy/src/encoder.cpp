#include "encoder.h"

#include <Arduino.h>

#define ENCODER_PIN 40

namespace encoder {

volatile int STEPS = 0;

static void on_step() {
	++STEPS;
}

void init() {
	pinMode(ENCODER_PIN, INPUT_PULLUP);
	attachInterrupt(ENCODER_PIN, on_step, RISING);
}

int steps() {
	cli();
	int steps = STEPS;
	sei();
	return steps;
}

}
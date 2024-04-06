#include "encoder.h"

#include <Arduino.h>

#define ENCODER_PIN 40
#define NUM_BUCKETS 20
#define BUCKET_INTERVAL_MS 40
#define PPR 5
#define CIRCUMFERENCE_M (0.565)

namespace encoder {

volatile int TOTAL_STEPS = 0;
volatile int STEPS[NUM_BUCKETS];
volatile int bucket = 0;

static void on_step() {
	++STEPS[bucket];
	++TOTAL_STEPS;
}

void update() {
	static int last_bucket = bucket;

	cli();
	bucket = (millis() % (NUM_BUCKETS * BUCKET_INTERVAL_MS)) / BUCKET_INTERVAL_MS;
	if (bucket != last_bucket) {
		STEPS[bucket] = 0;
	}
	last_bucket = bucket;
	sei();


}

double front_speed() {
	double speed = 0.0;

	cli();
	for (int i = 0; i < NUM_BUCKETS; i++)
		speed += STEPS[i];
	sei();

	return (speed * CIRCUMFERENCE_M * 1000.0) / (PPR * BUCKET_INTERVAL_MS * NUM_BUCKETS);
}

double rear_speed(double steering_angle) {
	steering_angle *= M_PI / 180.0;

	return front_speed() * cos(steering_angle);
}

void init() {
	pinMode(ENCODER_PIN, INPUT_PULLUP);
	attachInterrupt(ENCODER_PIN, on_step, RISING);
}

int steps() {
	cli();
	int steps = TOTAL_STEPS;
	sei();
	return steps;
}

}
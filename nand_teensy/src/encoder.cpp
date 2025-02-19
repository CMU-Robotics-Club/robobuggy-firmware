#include "encoder.h"
// #include "AS5600.h"

#include <Arduino.h>

#include <SPI.h>


// #define ENCODER_I2C Wire
// #define ENCODER_ADDR 0x36 // not used in this file, but this is the hard coded I2C address for the encoder
#define CS_ENCODER 2 //TODO: which pin

// SPI settings for encoder
#define SPI_SPEED 5000000
SPISettings settings_enc(SPI_SPEED, MSBFIRST, SPI_MODE1); 

#define DIAMETER_M (0.180)
#define HISTORY_LEN 1000
#define NUM_VALS_AVG 5


namespace encoder {

	struct encoder_spi{
		uint16_t pkt_val;
		uint16_t error_val;
		bool recv_error;	
		bool parity_error;
	};


float positions_rad[HISTORY_LEN] = {0};
unsigned long long times_us[HISTORY_LEN] = {0};
int history_index = 0;
int prev_time = 0; // time most recent call took place

int prev_time_millis() {
	return prev_time;
}

double front_speed() {
	int t1 = millis();
	positions_rad[history_index] = get_front_pos();
	int tF = millis() - t1;
	prev_time = tF;
	times_us[history_index] = micros();

	int prev_index = (history_index + 1) % HISTORY_LEN;
	float speed = -DIAMETER_M / 2.0 * 1e6 * (positions_rad[history_index] - positions_rad[prev_index]) / (times_us[history_index] - times_us[prev_index]);
	// Serial.println(times_us[history_index] - times_us[prev_index]);
	// Serial.println(positions_rad[history_index] - positions_rad[prev_index]);

	history_index = prev_index;
	return speed; // v=r*omega
	// when getAngularSpeed returns radians per second, builtin_front_speed returns meters per second
	//NOTE: current orientation of the magnet and therefore sign of the getAngularSpeed function
	//		is currently untested, so the wheel rolling forward may result in a negative angular
	//		speed value.
}

//TODO: finish changing
double avg_speed() {
	int t1 = millis();
	positions_rad[history_index] = get_front_pos();
	int tF = millis() - t1;
	prev_time = tF;
	times_us[history_index] = micros();

	int prev_index = (history_index + 1) % HISTORY_LEN;
	
	float speed = -DIAMETER_M / 2.0 * 1e6 * (positions_rad[history_index] - positions_rad[prev_index]) / (times_us[history_index] - times_us[prev_index]);
	// Serial.println(times_us[history_index] - times_us[prev_index]);
	// Serial.println(positions_rad[history_index] - positions_rad[prev_index]);

	history_index = prev_index;
	return speed; // v=r*omega
}

float get_front_pos() {
	struct encoder_spi angle_pkt;
	uint16_t read_angle_pkt = 0x3FFF;

	angle_pkt = read_pkt(read_angle_pkt);

	// TODO: error handling
	
	float rad_val = value * 2.0 * M_PI / 16384;
	return rad_val;
}

uint16_t get_diagnostics(){
	struct encoder_spi diagnostics_pkt;
	uint16_t read_diagnostics_pkt = 0x7FFD;
	
	diagnostics_pkt = read_pkt(read_diagnostics_pkt);

	Serial.println("Diagnostics packet:");
	Serial.println(diagnostics_pkt);

	// TODO: error handling

}

struct encoder_spi read_pkt(uint16_t rd_pkt) {
	uint16_t clr_errorflag_pkt = 1<<14 | 1; //0x8001
	uint16_t errorflag_mask = 1<<14;

	uint16_t recv_error = 0;
	uint16_t parity_error = 0;
	uint16_t value = 0;
	uint16_t error_value = 0;
	struct encoder_spi out_struct;

	SPI.beginTransaction(settings_enc);
	digitalWrite (CS_ENCODER, LOW);

	//read
	SPI.transfer16(rd_pkt);
	value = SPI.transfer16(clr_errorflag_pkt); // error_value returned next frame (if applicable)

	if (value & errorflag_mask){
		recv_error = 1;
		error_value = SPI.transfer16(0); // 0x0001 - framing, 0x0002 - command invalid, 0x0003 - parity error
	}

	digitalWrite (CS_ENCODER, HIGH);
	SPI.endTransaction();

	parity_error = value ^ (value>>8);
	parity_error = parity_error ^ (parity_error>>4);
	parity_error = parity_error ^ (parity_error>>2);
	parity_error = (parity_error ^ (parity_error>>1)) & 0x01;

	out_struct = {value, error_value, recv_error, parity_error};


	return out_struct;
}

double rear_speed(double steering_angle) {
	steering_angle *= M_PI / 180.0;
	return front_speed() * cos(steering_angle);
}

// uint16_t e_angle() {
// 	//return as5600.readAngle();
// }

// uint16_t e_raw_angle() {
// 	//return as5600.rawAngle();
// }


void init() {
	// Init SPI
	pinMode(CS_ENCODER, OUTPUT);
	// Main SPI pins enabled by default
	SPI.begin(); 
	
}

}
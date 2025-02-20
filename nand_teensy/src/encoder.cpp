#include "encoder.h"
#include "AS5048A.h"
// #include "AS5600.h"

#include <Arduino.h>

#include <SPI.h>


#define CS_ENCODER 2 //TODO: which pin

// SPI settings for encoder
#define SPI_SPEED 500000
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
// AS5048A enc { CS_ENCODER , true};

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

bool calc_parity(uint16_t value) {
	uint16_t parity_error = value ^ (value>>8);
	parity_error = parity_error ^ (parity_error>>4);
	parity_error = parity_error ^ (parity_error>>2);
	parity_error = (parity_error ^ (parity_error>>1)) & 0x01;
	return parity_error;
}

int raw_front_pos() {
	struct encoder_spi angle_pkt;
	uint16_t read_angle_pkt = 0x3FFF;
	read_angle_pkt = read_angle_pkt | (0x1 << 14); // read command
	bool parity = calc_parity(read_angle_pkt);
	read_angle_pkt = read_angle_pkt | ((parity?0x1:0x0) << 15); // parity calculation

	angle_pkt = read_pkt(read_angle_pkt);

	// TODO: error handling

	int value = angle_pkt.pkt_val;// & 0x3FFF;
	return value;
}

float get_front_pos() {
	int value = encoder::raw_front_pos();
	float rad_val = (value<<1) * M_PI / 16384;
	return rad_val;
}
/*
uint16_t rawRot() {
	return enc.getRawRotation();
}

uint16_t state() {
	return enc.getState();
}

uint16_t gain() {
	return enc.getGain();
}



double rotRad() {
	return enc.getRotationInRadians();
}*/



uint16_t get_diagnostics(){
	struct encoder_spi diagnostics_pkt;
	uint16_t read_diagnostics_pkt = 0x7FFD;
	
	diagnostics_pkt = read_pkt(read_diagnostics_pkt);

	Serial.println("Diagnostics packet:");
	Serial.printf("%i\n",diagnostics_pkt.pkt_val);
	Serial.printf("%i\n",diagnostics_pkt.error_val);
	Serial.printf("%i\n",diagnostics_pkt.recv_error);
	Serial.printf("%i\n",diagnostics_pkt.parity_error);
	return -1;
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
	digitalWrite(CS_ENCODER, HIGH);
	digitalWrite(CS_ENCODER, LOW);
	value = SPI.transfer16(clr_errorflag_pkt); // error_value returned next frame (if applicable)
	digitalWrite(CS_ENCODER, HIGH);

	if (value & errorflag_mask){
		recv_error = 1;
		digitalWrite(CS_ENCODER, LOW);
		error_value = SPI.transfer16(0); // 0x0001 - framing, 0x0002 - command invalid, 0x0003 - parity error
		digitalWrite(CS_ENCODER, HIGH);
	}

	digitalWrite (CS_ENCODER, HIGH);
	SPI.endTransaction();

	parity_error = value ^ (value>>8);
	parity_error = parity_error ^ (parity_error>>4);
	parity_error = parity_error ^ (parity_error>>2);
	parity_error = (parity_error ^ (parity_error>>1)) & 0x01;

	out_struct = {value, error_value, *(bool *)(&recv_error), *(bool *)(&parity_error)};


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
//	enc.begin();
	// Main SPI pins enabled by default
	SPI.begin(); 
	
}

}
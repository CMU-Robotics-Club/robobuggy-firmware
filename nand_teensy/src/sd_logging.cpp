#include "sd_logging.h"

#include <Arduino.h>
#include <SD.h>
#include "TeensyThreads.h"
#define buf_size 20000

namespace sd_logging {

static const bool DO_LOGGING = true;

static File STEERING_FILE {};
static File GPS_FILE {};
static File ENCODER_FILE {};
static File FILTER_FILE {};
static File COVARIANCE_FILE {};

volatile char steering_buf   [buf_size]; //Garrison
volatile char gps_buf        [buf_size]; //Ashley
volatile char encoder_buf    [buf_size]; //Gyan
volatile char filter_buf     [buf_size]; //Kevin
volatile char covarience_buf [buf_size]; //Nnenna

volatile size_t steering_size = 0;

volatile size_t filter_size = 0;
volatile size_t covarience_size = 0;

Threads::Mutex steering_m;
Threads::Mutex gps_m;
Threads::Mutex encoder_m;
Threads::Mutex filter_m;
Threads::Mutex covarience_m;

void sd_thread() {
	
}

void steering_sd(){
	steering_m.lock();
	char thread_buf [buf_size];
	size_t cnt = snprintf(thread_buf, steering_size, (const char *)&steering_buf);
	steering_size = 0;
	steering_m.unlock();
	STEERING_FILE.write(thread_buf, cnt);
}

void log_steering(double angle) {
	if (!DO_LOGGING) {
		return;
	}

	steering_m.lock();
		steering_size += snprintf((char *)&steering_buf[steering_size], buf_size - steering_size, "%lu,%f\n", millis(), angle);
	steering_m.unlock();
}

/*
 filter_buf
 filter_size
 filter_m
*/
void multithread_filter() {
	//make new buffer
	char copy_buf[20000];
	//lock
	filter_m.lock();
	//copy filter_buf to new buffer
	int copy_num = snprintf(copy_buf, filter_size, (const char *)filter_buf);
	//set filter_size to 0
	filter_size = 0;
	//unlock
	filter_m.unlock();
	//write
	FILTER_FILE.write(copy_buf, copy_num);
}
void multithread_covarience() {
	char temp_buf[20000];
	covarience_m.lock();
	int copy_num = snprintf(temp_buf, 20000, (const char *)covarience_buf);
	covarience_size = 0;
	covarience_m.unlock();
	COVARIANCE_FILE.write(temp_buf, copy_num);
}

void init() {
	if (!DO_LOGGING) {
		return;
	}

	while (!SD.begin(BUILTIN_SDCARD)) {
		Serial.println("SD card not detected");
		delay(1000);
	}

	int file_num = 0;
	char steering_file_name[100];
	char gps_file_name[100];
	char encoder_file_name[100];
	char filter_file_name[100];
	char covar_file_name[100];
	while (true) {
		snprintf(steering_file_name, sizeof(steering_file_name), "log%d-steering.csv", file_num);
		snprintf(gps_file_name,      sizeof(gps_file_name),      "log%d-gps.csv",      file_num);
		snprintf(encoder_file_name,  sizeof(encoder_file_name),  "log%d-encoder.csv",  file_num);
		snprintf(filter_file_name,   sizeof(filter_file_name),   "log%d-filter.csv",   file_num);
		snprintf(covar_file_name,    sizeof(covar_file_name),    "log%d-covar.csv",    file_num);

		if (!SD.exists(steering_file_name)) {
			break;
		}

		++file_num;
	}

	STEERING_FILE   = SD.open(steering_file_name, FILE_WRITE);
	GPS_FILE        = SD.open(gps_file_name, FILE_WRITE);
	ENCODER_FILE    = SD.open(encoder_file_name, FILE_WRITE);
	FILTER_FILE     = SD.open(filter_file_name, FILE_WRITE);
	COVARIANCE_FILE = SD.open(covar_file_name, FILE_WRITE);

	STEERING_FILE.write("timestamp,steering\n");
	GPS_FILE.write("timestamp,pos_x,pos_y,accuracy\n");
	ENCODER_FILE.write("timestamp,speed\n");
	FILTER_FILE.write("timestamp,pos_x,pos_y,heading\n");
	COVARIANCE_FILE.write("timestamp,c1,c2,c3,c4,c5,c6,c7,c8,c9\n");

	threads.addThread(sd_thread);
	threads.setSliceMillis(1);

}

void log_gps(double x, double y, double accuracy) {
	if (!DO_LOGGING) {
		return;
	}

	char buf[100];
	size_t cnt = snprintf(buf, sizeof(buf), "%lu,%f,%f,%f\n", millis(), x, y, accuracy);
	GPS_FILE.write(buf, cnt);
}

void log_speed(double speed) {
	if (!DO_LOGGING) {
		return;
	}

	char buf[100];
	size_t cnt = snprintf(buf, sizeof(buf), "%lu,%f\n", millis(), speed);
	ENCODER_FILE.write(buf, cnt);
}

void log_filter_state(double x, double y, double heading) {
	if (!DO_LOGGING) {
		return;
	}

	char buf[100];
	size_t cnt = snprintf(buf, sizeof(buf), "%lu,%f,%f,%f\n", millis(), x, y, heading);
	//FILTER_FILE.write(buf, cnt);
	
	//lock
	//copy buf to filter_buf (offset by filter_size) - &filter_buf[filter_size]
	//increment filter_size by cnt
	//unlock
}

void log_covariance(const state_cov_matrix_t &cov) {
	if (!DO_LOGGING) {
		return;
	}

	char buf[200];
	size_t cnt = snprintf(buf, sizeof(buf),
		"%lu,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", millis(),
		cov(0, 0), cov(0, 1), cov(0, 2),
		cov(1, 0), cov(1, 1), cov(1, 2),
		cov(2, 0), cov(2, 1), cov(2, 2)
	);
	COVARIANCE_FILE.write(buf, cnt);
}

void flush_files() {
	if(!DO_LOGGING){
		return;
	}
	STEERING_FILE.flush();
	GPS_FILE.flush();
	ENCODER_FILE.flush();
	FILTER_FILE.flush();
	COVARIANCE_FILE.flush();
}

} // namespace sd
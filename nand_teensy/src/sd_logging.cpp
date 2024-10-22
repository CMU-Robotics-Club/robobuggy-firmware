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
volatile size_t gps_size = 0;
volatile size_t encoder_size = 0;
volatile size_t filter_size = 0;
volatile size_t covarience_size = 0;

Threads::Mutex steering_m;
Threads::Mutex gps_m;
Threads::Mutex encoder_m;
Threads::Mutex filter_m;
Threads::Mutex covarience_m;

void multithread_steering(){
	char temp_buf [buf_size];
	steering_m.lock();
	size_t cnt = snprintf(temp_buf, steering_size, (const char *)steering_buf);
	steering_size = 0;
	steering_m.unlock();
	STEERING_FILE.write(temp_buf, cnt);
}

void multithread_gps(){
	char temp_buf[buf_size];
	gps_m.lock();
	size_t copy_num = snprintf(temp_buf, gps_size, (const char *)gps_buf);
	gps_size = 0;
	gps_m.unlock();
	GPS_FILE.write(temp_buf, copy_num);
}

void multithread_encoder() {
	char temp_buf[buf_size];
	encoder_m.lock();
	size_t copy_num = snprintf(temp_buf, encoder_size, (const char *)encoder_buf);
	encoder_size = 0;
	encoder_m.unlock();
	ENCODER_FILE.write(temp_buf, copy_num);
}

void multithread_filter() {
	char copy_buf[buf_size];
	filter_m.lock();
	size_t copy_num = snprintf(copy_buf, filter_size, (const char *)filter_buf);
	filter_size = 0;
	filter_m.unlock();
	FILTER_FILE.write(copy_buf, copy_num);
}
void multithread_covarience() {
	char temp_buf[buf_size];
	covarience_m.lock();
	size_t copy_num = snprintf(temp_buf, covarience_size, (const char *)covarience_buf);
	covarience_size = 0;
	covarience_m.unlock();
	COVARIANCE_FILE.write(temp_buf, copy_num);
}

void sd_thread(int arg) {
	if(!DO_LOGGING) {
		return;
	}
	multithread_steering();
	multithread_gps();
	multithread_encoder();
	multithread_filter();
	multithread_covarience();

	flush_files();
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

	threads.addThread(sd_thread, 0, buf_size*2, NULL);
	threads.setSliceMillis(1);

}

void log_steering(double angle) {
	if (!DO_LOGGING) {
		return;
	}

	steering_m.lock();
		steering_size += snprintf((char *)&steering_buf[steering_size], buf_size - steering_size, "%lu,%f\n", millis(), angle);
	steering_m.unlock();
}

void log_gps(double x, double y, double accuracy) {
	if (!DO_LOGGING) {
		return;
	}

	gps_m.lock();
		gps_size += snprintf((char *)&gps_buf[gps_size], buf_size - gps_size, "%lu,%f,%f,%f",millis(),x,y,accuracy);
	gps_m.unlock();
}

void log_speed(double speed) {
	if (!DO_LOGGING) {
		return;
	}

	encoder_m.lock();
		encoder_size += snprintf((char *)&encoder_buf[encoder_size], buf_size - encoder_size, "%lu,%f",millis(),speed);
	encoder_m.unlock();
}

void log_filter_state(double x, double y, double heading) {
	if (!DO_LOGGING) {
		return;
	}

	filter_m.lock();
		filter_size += snprintf((char *)&filter_buf[encoder_size], buf_size - filter_size, "%lu,%f,%f,%f",millis(),x,y,heading);
	filter_m.unlock();
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

	covarience_m.lock();
		covarience_size += snprintf((char *)&covarience_buf[covarience_size], buf_size - filter_size, "%lu,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", millis(),
																										cov(0, 0), cov(0, 1), cov(0, 2),
																										cov(1, 0), cov(1, 1), cov(1, 2),
																										cov(2, 0), cov(2, 1), cov(2, 2));
	covarience_m.unlock();
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
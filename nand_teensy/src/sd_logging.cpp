#include "sd_logging.h"

#include <Arduino.h>
#include <SD.h>

namespace sd_logging {

static const bool DO_LOGGING = true;

static File FILE {};

static File STEERING_FILE {};
static File GPS_FILE {};
static File ENCODER_FILE {};
static File FILTER_FILE {};
static File COVARIANCE_FILE {};

void init() {
	if (!DO_LOGGING) {
		return;
	}

	while (!SD.begin(BUILTIN_SDCARD)) {
		Serial.println("SD card not detected");
		delay(1000);
	}

	int file_num = 0;
	/*char steering_file_name[100];
	char gps_file_name[100];
	char encoder_file_name[100];
	char filter_file_name[100];
	char covar_file_name[100];*/
	char file_name[100];
	while (true) {
		/*snprintf(steering_file_name, sizeof(steering_file_name), "log%d-steering.csv", file_num);
		snprintf(gps_file_name,      sizeof(gps_file_name),      "log%d-gps.csv",      file_num);
		snprintf(encoder_file_name,  sizeof(encoder_file_name),  "log%d-encoder.csv",  file_num);
		snprintf(filter_file_name,   sizeof(filter_file_name),   "log%d-filter.csv",   file_num);
		snprintf(covar_file_name,    sizeof(covar_file_name),    "log%d-covar.csv",    file_num);*/
		snprintf(file_name, 		 sizeof(file_name), 		 "log%d.csv", 		   file_num);

		if (!SD.exists(file_name)) {
			break;
		}

		++file_num;
	}

	/*STEERING_FILE   = SD.open(steering_file_name, FILE_WRITE);
	GPS_FILE        = SD.open(gps_file_name, FILE_WRITE);
	ENCODER_FILE    = SD.open(encoder_file_name, FILE_WRITE);
	FILTER_FILE     = SD.open(filter_file_name, FILE_WRITE);
	COVARIANCE_FILE = SD.open(covar_file_name, FILE_WRITE);*/
	FILE 			= SD.open(file_name, FILE_WRITE);

	/*STEERING_FILE.write("timestamp,steering\n");
	GPS_FILE.write("timestamp,pos_x,pos_y,accuracy\n");
	ENCODER_FILE.write("timestamp,speed\n");
	FILTER_FILE.write("timestamp,pos_x,pos_y,heading\n");
	COVARIANCE_FILE.write("timestamp,c1,c2,c3,c4,c5,c6,c7,c8,c9\n");*/

	FILE.write("timestamp,steering.steering,gps.pos_x,gps.pos_y,gps.accuracy,encoder.speed,filter.pos_x,filter.pos_y,filter.heading,\
	covariance.c1,covariance.c2,covariance.c3,covariance.c4,covariance.c5,covariance.c6,covariance.c7,covariance.c8,covariance.c9\n");

	/*threads.addThread(sd_thread, 0, buf_size*2, NULL);
	threads.setSliceMillis(1);*/

}

void log_steering(double angle) {
	if (!DO_LOGGING) {
		return;
	}
	char buf[100];
	size_t cnt = snprintf(buf, sizeof(buf), "%lu,%f,,,,,,,,,,,,,,,,\n", millis(), angle);
	FILE.write(buf,cnt);
}

void log_gps(double x, double y, double accuracy) {
	if (!DO_LOGGING) {
		return;
	}
	char buf[100];
	size_t cnt = snprintf(buf, sizeof(buf), "%lu,,%f,%f,%f,,,,,,,,,,,,,\n", millis(), x,y,accuracy);
	FILE.write(buf,cnt);
}

void log_speed(double speed) {
	if (!DO_LOGGING) {
		return;
	}
	char buf[100];
	size_t cnt = snprintf(buf, sizeof(buf), "%lu,,,,,%f,,,,,,,,,,,,\n", millis(), speed);
	FILE.write(buf,cnt);
}

void log_filter_state(double x, double y, double heading) {
	if (!DO_LOGGING) {
		return;
	}
	char buf[100];
	size_t cnt = snprintf(buf, sizeof(buf), "%lu,,,,,,%f,%f,%f,,,,,,,,,\n", millis(), x,y,heading);
	FILE.write(buf,cnt);
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
	char buf[100];
	size_t cnt = snprintf(buf, sizeof(buf), "%lu,,,,,,,,,,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",millis(),cov(0, 0), cov(0, 1), cov(0, 2),cov(1, 0), cov(1, 1), cov(1, 2),cov(2, 0), cov(2, 1), cov(2, 2));
	FILE.write(buf,cnt);
}

void flush_files() {
	if(!DO_LOGGING){
		return;
	}
	FILE.flush();
}

} // namespace sd
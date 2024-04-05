#include "sd_logging.h"

#include <Arduino.h>
#include <SD.h>


namespace sd_logging {

static const bool DO_LOGGING = false;

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


}

void log_steering(double angle) {
	if (!DO_LOGGING) {
		return;
	}

	char buf[100];
	size_t cnt = snprintf(buf, sizeof(buf), "%lu,%f\n", millis(), angle);
	STEERING_FILE.write(buf, cnt);
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
	FILTER_FILE.write(buf, cnt);
}

void log_covariance(const state_cov_matrix_t &cov) {
	if (!DO_LOGGING) {
		return;
	}

	char buf[200];
	size_t cnt = snprintf(buf, sizeof(buf),
		"%lu,%f,%f,%f,%f,%f,%f,%f,%f,%f", millis(),
		cov(0, 0), cov(0, 1), cov(0, 2),
		cov(1, 0), cov(1, 1), cov(1, 2),
		cov(2, 0), cov(2, 1), cov(2, 2)
	);
	COVARIANCE_FILE.write(buf, cnt);
}

void flush_files() {
	STEERING_FILE.flush();
	GPS_FILE.flush();
	ENCODER_FILE.flush();
	FILTER_FILE.flush();
	COVARIANCE_FILE.flush();
}

} // namespace sd
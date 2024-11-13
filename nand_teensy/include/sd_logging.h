#pragma once

#include "ukf.h"

namespace sd_logging {
	void init();

	void log_steering(double angle);

	void log_gps(double x, double y, double accuracy);

	void log_speed(double speed);

	void log_filter_state(double x, double y, double heading);

	void log_covariance(const state_cov_matrix_t &cov);

	void flush_files();
}
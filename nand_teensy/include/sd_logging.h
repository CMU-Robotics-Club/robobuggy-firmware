#pragma once

namespace sd_logging {
	void init();

	void log_steering(double angle);

	void log_gps(double x, double y, double accuracy);

	void log_speed(double speed);

	void flush_files();
}
#pragma once

namespace sd_logging {
	void init();

	void log_steering(double angle);

	void log_gps(double x, double y);

	void log_encoder(int steps);
}
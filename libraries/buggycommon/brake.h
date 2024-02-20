#pragma once

namespace brake {
	enum class Status {
		Stopped,
		Rolling,
	};

	void init(int brake_pin);

	void set(Status status);
}
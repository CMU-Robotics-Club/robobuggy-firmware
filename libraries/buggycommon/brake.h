#pragma once

namespace brake {
	enum class Status : unsigned char {
		Stopped,
		Rolling,
	};

	void init(int brake_pin);

	void set(Status status);

	Status state();
}
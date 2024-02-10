#pragma once

namespace brake {
	enum class Status {
		Stopped,
		Rolling,
	};

	void init();

	void set(Status status);
}
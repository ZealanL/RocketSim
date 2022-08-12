#pragma once
#include "../BaseInc.h"

// Stores all control inputs to a car
struct CarControls {
	// Driving control
	float throttle, steer;

	// Air orientation control
	float pitch, yaw, roll;

	// Boolean action inputs
	bool boost, jump, handbrake;

	// Maybe someday...
	// bool useItem;

	CarControls() = default;

	// Makes all values range-valid (clamps from -1 to 1)
	void ClampFix() {
		throttle	= CLAMP(throttle,	-1, 1);
		steer		= CLAMP(steer,		-1, 1);
		pitch		= CLAMP(pitch,		-1, 1);
		yaw			= CLAMP(yaw,		-1, 1);
		roll		= CLAMP(roll,		-1, 1);
	}
};
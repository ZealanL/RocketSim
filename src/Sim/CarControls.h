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

	CarControls() {
		// Initialize everything as zero
		memset(this, 0, sizeof(CarControls));
	}

	// Makes all values range-valid (clamps from -1 to 1)
	void ClampFix() {
		throttle	= RS_CLAMP(throttle,	-1, 1);
		steer		= RS_CLAMP(steer,		-1, 1);
		pitch		= RS_CLAMP(pitch,		-1, 1);
		yaw			= RS_CLAMP(yaw,		-1, 1);
		roll		= RS_CLAMP(roll,		-1, 1);
	}
};
#pragma once
#include "../BaseInc.h"

struct WheelPair {
	// Radius of both wheels
	float wheelRadius;

	// How far the wheels can be pulled away before they stop moving
	float maxSuspensionExtendDist;

	// How much force do these wheels push with
	float drivePushForce;

	// How fast do these wheels spin
	float spinRate;
};

struct CarConfig {
	// Area of the hitbox (relative to car origin)
	Vec hitboxMin, hitboxMax;

	// Front and back wheel pairs
	WheelPair wheelPairs[2]; 
};
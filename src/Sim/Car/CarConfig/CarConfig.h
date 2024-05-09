#pragma once
#include "../../../BaseInc.h"

RS_NS_START

struct WheelPairConfig {
	// Radius of both wheels
	float wheelRadius;

	// How far out the suspension rests
	float suspensionRestLength;

	// Where the wheel actually connects (suspension start position)
	// NOTE: Y should ALWAYS be positive. It will be automatically negated when creating the second wheel. 
	Vec connectionPointOffset;
};

#define WHEEL_PAIR_CONFIG_SERIALIZATION_FIELDS(name) \
name.connectionPointOffset, name.suspensionRestLength, name.wheelRadius

struct CarConfig {
	// Full size of hitbox (NOT the half-size/extent)
	Vec hitboxSize;

	// Offset of the hitbox (from it's origin)
	// NOTE: Does not effect car's center of mass, that's always at local (0,0,0)
	Vec hitboxPosOffset;

	WheelPairConfig frontWheels, backWheels;

	// abs(yaw or pitch or roll) input will need to be >= this in order to flip
	float dodgeDeadzone = 0.5f;
};

#define CAR_CONFIG_SERIALIZATION_FIELDS(name) \
name.dodgeDeadzone, name.hitboxPosOffset, name.hitboxSize, \
WHEEL_PAIR_CONFIG_SERIALIZATION_FIELDS(name.frontWheels), \
WHEEL_PAIR_CONFIG_SERIALIZATION_FIELDS(name.backWheels) \

// Global car configurations for all car type presets
// NOTE: CAR_CONFIG_PLANK is the batmobile preset
RSAPI const extern CarConfig
	CAR_CONFIG_OCTANE, CAR_CONFIG_DOMINUS, CAR_CONFIG_PLANK, CAR_CONFIG_BREAKOUT, CAR_CONFIG_HYBRID, CAR_CONFIG_MERC;

RS_NS_END
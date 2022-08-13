#pragma once
#include "CarConfig/CarConfig.h"
#include "../btVehicleRL/btVehicleRL.h"
#include "../CarControls.h"

struct CarState {
	// Position in world space
	Vec pos;

	EulerAngle angles;

	// Linear velocity
	Vec vel;

	// Angular velocity (axis-angle)
	Vec angVel;

	bool isOnGround;
	bool hasJumped, hasDoubleJumped;

	// Goes from 0 to 100
	float boost;

	// This is a state variable due to the supersonic maintain time (see RLConst.h)
	bool isSupersonic;

	// The most recent controls this car was simulated with
	// Required for jumping/flipping, the last value of the jump button is required, because jumping occurs on press only
	CarControls controls;
};

enum class Team {
	BLUE = 0,
	ORANGE = 1
};

class Car {
public:
	CarConfig config;
	Team team;

	CarState GetState();
	void SetState(const CarState& state);

	btVehicleRL _bulletVehicle;
	
	// NOTE: Not all values are updated because they are unneeded for internal simulation
	// Those values are only updated when GetState() is called
	CarState _internalState;
};
#pragma once
#include "../../BaseInc.h"

RS_NS_START

struct PhysState {
	// Position in world space (UU)
	Vec pos = {};
	
	// Rotation matrix (column-major)
	RotMat rotMat = RotMat::GetIdentity();

	// Linear velocity
	Vec vel = {};

	// Angular velocity (rad/s)
	Vec angVel = {};
};

RS_NS_END
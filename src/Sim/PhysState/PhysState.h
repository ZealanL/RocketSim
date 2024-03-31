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

	// Gets a copy of this state rotated 180 degrees around Z axis
	// This is achieved by multiplying all vectors by (-1, -1, 1)
	PhysState GetInvertedY() const;
};

RS_NS_END
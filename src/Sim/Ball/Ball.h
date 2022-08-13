#pragma once
#include "../../BaseInc.h"

struct BallState {
	// Position in world space
	Vec pos;

	// Linear velocity
	Vec vel;

	// Angular velocity (axis-angle)
	Vec angVel;
};

class Ball {
public:
	BallState GetState();
	void SetState(const BallState& state);

	btRigidBody* _rigidBody;
	btSphereShape* _collisionShape;
};
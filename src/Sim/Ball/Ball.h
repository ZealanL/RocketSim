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

	// No copy/move constructor
	Ball(const Ball& other) = delete;
	Ball(Ball&& other) = delete;

	btRigidBody* _rigidBody;
	btSphereShape* _collisionShape;

	float GetRadius() {
		return _collisionShape->getRadius();
	}

	// For construction by Arena
	static Ball* _AllocBall();
	
	~Ball();

private:
	Ball() {}
};
#pragma once
#include "../../BaseInc.h"

#include "../../RLConst.h"

struct BallState {
	// Position in world space
	Vec pos = { 0, 0, 93.15f };

	// Linear velocity
	Vec vel = { 0, 0, 0 };

	// Angular velocity (axis-angle)
	Vec angVel = { 0, 0, 0 };
};

class Ball {
public:
	RSAPI BallState GetState();
	RSAPI void SetState(const BallState& state);

	// No copy/move constructor
	Ball(const Ball& other) = delete;
	Ball(Ball&& other) = delete;

	btRigidBody* _rigidBody;
	btSphereShape* _collisionShape;

	// If non-zero, added to velocity at the end of an Arena step
	Vec _velocityImpulseCache = { 0,0,0 };

	// For construction by Arena
	static Ball* _AllocBall();
	void _BulletSetup(btDynamicsWorld* bulletWorld, float radius);

	void _ApplyPhysicsRounding();
	void _LimitVelocities();

	RSAPI float GetRadius() {
		if (!_collisionShape) {
			return -1;
		} else {
			return _collisionShape->getRadius();
		}
	}

	~Ball();

private:
	Ball() {
		
	}
};
#pragma once
#include "../../BaseInc.h"

#include "../../RLConst.h"
#include "../../DataStream/DataStreamIn.h"
#include "../../DataStream/DataStreamOut.h"

struct BallHitInfo {
	uint32_t carID = NULL; // ID of the car that hit the ball
	Vec relativePosOnBall; // Position of the hit relative to the ball's position
	Vec ballPos; // World position of the ball when the hit occured
	Vec extraHitVel; // Extra velocity added to base collision velocity
	uint64_t tickCountWhenHit; // Arena tick count when the hit occured

	void Serialize(DataStreamOut& out);
	void Deserialize(DataStreamIn& in);
};

#define BALLHITINFO_SERIALIZATION_FIELDS \
carID, relativePosOnBall, ballPos, extraHitVel, tickCountWhenHit

struct BallState {
	// Position in world space
	Vec pos = { 0, 0, RLConst::BALL_REST_Z };

	// Linear velocity
	Vec vel = { 0, 0, 0 };
	 
	// Angular velocity (axis-angle)
	Vec angVel = { 0, 0, 0 };

	// Information from the most recent car-ball hit
	// Does ever not reset automatically
	BallHitInfo ballHitInfo = BallHitInfo();

	void Serialize(DataStreamOut& out);
	void Deserialize(DataStreamIn& in);
};

#define BALLSTATE_SERIALIZATION_FIELDS \
pos, vel, angVel

class Ball {
public:

	BallState _internalState;
	RSAPI BallState GetState();
	RSAPI void SetState(const BallState& state);

	// No copy/move constructor
	Ball(const Ball& other) = delete;
	Ball(Ball&& other) = delete;

	btRigidBody* _rigidBody;
	btSphereShape* _collisionShape;

	// For construction by Arena
	static Ball* _AllocBall();
	void _BulletSetup(btDynamicsWorld* bulletWorld, float radius);

	Vec _velocityImpulseCache = { 0,0,0 };
	void _FinishPhysicsTick();

	float GetRadiusBullet() {
		if (!_collisionShape) {
			return -1;
		} else {
			return _collisionShape->getRadius();
		}
	}

	float GetRadius() {
		return GetRadiusBullet() * BT_TO_UU;
	}

	~Ball();

private:
	Ball() {}
};
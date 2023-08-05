#pragma once
#include "../../BaseInc.h"

#include "../../RLConst.h"
#include "../../DataStream/DataStreamIn.h"
#include "../../DataStream/DataStreamOut.h"

#include "../MutatorConfig/MutatorConfig.h"

#include "../../../libsrc/bullet3-2.82/BulletDynamics/Dynamics/btRigidBody.h"
#include "../../../libsrc/bullet3-2.82/BulletCollision/CollisionShapes/btSphereShape.h"

struct BallState {
	// Position in world space
	Vec pos = { 0, 0, RLConst::BALL_REST_Z };

	// Linear velocity
	Vec vel = { 0, 0, 0 };
	 
	// Angular velocity (axis-angle)
	Vec angVel = { 0, 0, 0 };

	bool Matches(const BallState& other, float marginPos = 0.8, float marginVel = 0.4, float marginAngVel = 0.02) const;

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

	btRigidBody _rigidBody;
	btSphereShape _collisionShape;

	// For construction by Arena
	static Ball* _AllocBall() { return new Ball(); }

	// For removal by Arena
	static void _DestroyBall(Ball* ball) { delete ball; }

	void _BulletSetup(class btDynamicsWorld* bulletWorld, const MutatorConfig& mutatorConfig);

	Vec _velocityImpulseCache = { 0,0,0 };
	void _FinishPhysicsTick(const MutatorConfig& mutatorConfig);

	// Returns radius in BulletPhysics units
	float GetRadiusBullet();

	// Returns radius in Unreal Engine units (uu)
	float GetRadius() {
		return GetRadiusBullet() * BT_TO_UU;
	}

	Ball(const Ball& other) = delete;
	Ball& operator=(const Ball& other) = delete;

	~Ball() {}

private:
	Ball() {}
};
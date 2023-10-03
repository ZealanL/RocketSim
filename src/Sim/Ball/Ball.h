#pragma once
#include "../../BaseInc.h"

#include "../../RLConst.h"
#include "../../DataStream/DataStreamIn.h"
#include "../../DataStream/DataStreamOut.h"

#include "../MutatorConfig/MutatorConfig.h"

#include "../../../libsrc/bullet3-3.24/BulletDynamics/Dynamics/btRigidBody.h"
#include "../../../libsrc/bullet3-3.24/BulletCollision/CollisionShapes/btSphereShape.h"

struct BallState {
	// Position in world space
	Vec pos = { 0, 0, RLConst::BALL_REST_Z };

	RotMat rotMat = RotMat::GetIdentity();

	// Linear velocity
	Vec vel = { 0, 0, 0 };
	 
	// Angular velocity (axis-angle)
	Vec angVel = { 0, 0, 0 };

	struct HeatseekerInfo {
		// Which net the ball should seek towards
		// When 0, no net
		float yTargetDir = 0;

		float curTargetSpeed = RLConst::Heatseeker::INITIAL_TARGET_SPEED;
		float timeSinceHit = 0;
	};

	HeatseekerInfo hsInfo;

	bool Matches(const BallState& other, float marginPos = 0.8, float marginVel = 0.4, float marginAngVel = 0.02) const;

	void Serialize(DataStreamOut& out);
	void Deserialize(DataStreamIn& in);
};

#define BALLSTATE_SERIALIZATION_FIELDS \
pos, rotMat, vel, angVel, \
hsInfo.yTargetDir, hsInfo.curTargetSpeed, hsInfo.timeSinceHit

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
	float GetRadiusBullet() const;

	// Returns radius in Unreal Engine units (uu)
	float GetRadius() const {
		return GetRadiusBullet() * BT_TO_UU;
	}

	void _PreTickUpdate(GameMode gameMode, float tickTime);
	void _OnHit(GameMode gameMode, struct Car* car);
	void _OnWorldCollision(GameMode gameMode, Vec normal);
		
	Ball(const Ball& other) = delete;
	Ball& operator=(const Ball& other) = delete;

	~Ball() {}

private:
	Ball() {}
};
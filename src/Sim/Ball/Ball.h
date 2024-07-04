#pragma once
#include "../PhysState/PhysState.h"

#include "../../RLConst.h"
#include "../../DataStream/DataStreamIn.h"
#include "../../DataStream/DataStreamOut.h"

#include "../MutatorConfig/MutatorConfig.h"

#include "../../../libsrc/bullet3-3.24/BulletDynamics/Dynamics/btRigidBody.h"
#include "../../../libsrc/bullet3-3.24/BulletCollision/CollisionShapes/btSphereShape.h"

class btDynamicsWorld;

RS_NS_START

struct BallState : public PhysState {
	// Incremented every update, reset when SetState() is called
	// Used for telling if a stateset occured
	// Not serialized
	uint64_t updateCounter = 0;

	struct HeatseekerInfo {
		// Which net the ball should seek towards
		// When 0, no net
		float yTargetDir = 0;

		float curTargetSpeed = RLConst::Heatseeker::INITIAL_TARGET_SPEED;
		float timeSinceHit = 0;
	};

	HeatseekerInfo hsInfo;

	BallState() : PhysState() {
		pos.z = RLConst::BALL_REST_Z;
	}

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
	btCollisionShape* _collisionShape;

	// For construction by Arena
	static Ball* _AllocBall() { return new Ball(); }

	// For removal by Arena
	static void _DestroyBall(Ball* ball) { delete ball; }

	void _BulletSetup(GameMode gameMode, btDynamicsWorld* bulletWorld, const MutatorConfig& mutatorConfig, bool noRot);

	bool _groundStickApplied = false;
	Vec _velocityImpulseCache = { 0,0,0 };
	void _FinishPhysicsTick(const MutatorConfig& mutatorConfig);

	RSAPI bool IsSphere() const;

	// Returns radius in BulletPhysics units
	RSAPI float GetRadiusBullet() const;

	// Returns radius in Unreal Engine units (uu)
	float GetRadius() const {
		return GetRadiusBullet() * BT_TO_UU;
	}

	void _PreTickUpdate(GameMode gameMode, float tickTime);
	void _OnHit(GameMode gameMode, class Car* car);
	void _OnWorldCollision(GameMode gameMode, Vec normal, float tickTime);
		
	Ball(const Ball& other) = delete;
	Ball& operator=(const Ball& other) = delete;

	~Ball() {
		delete _collisionShape;
	}

private:
	Ball() {}
};

RS_NS_END
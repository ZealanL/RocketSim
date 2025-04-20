#pragma once
#include "../PhysState/PhysState.h"

#include "../../RLConst.h"
#include "../../DataStream/DataStreamIn.h"
#include "../../DataStream/DataStreamOut.h"

#include "../MutatorConfig/MutatorConfig.h"

#include "../../../libsrc/bullet3-3.24/BulletDynamics/Dynamics/btRigidBody.h"
#include "../../../libsrc/bullet3-3.24/BulletCollision/CollisionShapes/btSphereShape.h"
#include "../Arena/DropshotTiles/DropshotTiles.h"

class btDynamicsWorld;

RS_NS_START

struct BallState : public PhysState {
	// Incremented every update, reset when SetState() is called
	// Used for telling if a stateset occured
	// Not serialized
	uint64_t tickCountSinceUpdate = 0;

	struct HeatseekerInfo {
		// Which net the ball should seek towards
		// When 0, no net
		float yTargetDir = 0;

		float curTargetSpeed = RLConst::Heatseeker::INITIAL_TARGET_SPEED;
		float timeSinceHit = 0;
	};
	HeatseekerInfo hsInfo;

	struct DropshotInfo {
		// Charge level number, which controls the radius of damage when hitting tiles
		// 1 = damages r=1 -> 1 tile
		// 2 = damages r=2 -> 7 tiles
		// 3 = damages r=3 -> 19 tiles
		int chargeLevel = 1;

		float accumulatedHitForce = 0; // Resets when a tile is damaged
		float yTargetDir = 0; // Which side of the field the ball can damage (0=none, -1=blue, 1=orange)
		
		bool hasDamaged = false;
		uint64_t lastDamageTick; // Only valid if hasDamaged
	};
	DropshotInfo dsInfo;

	BallState() : PhysState() {
		pos.z = RLConst::BALL_REST_Z;
	}

	bool Matches(const BallState& other, float marginPos = 0.8, float marginVel = 0.4, float marginAngVel = 0.02) const;

	void Serialize(DataStreamOut& out);
	void Deserialize(DataStreamIn& in);
};

#define BALLSTATE_SERIALIZATION_FIELDS \
pos, rotMat, vel, angVel, \
hsInfo.yTargetDir, hsInfo.curTargetSpeed, hsInfo.timeSinceHit, \
dsInfo.chargeLevel, dsInfo.accumulatedHitForce, dsInfo.yTargetDir, dsInfo.hasDamaged, dsInfo.lastDamageTick \

class Ball {
public:

	BallState _internalState;
	BallState GetState();
	void SetState(const BallState& state);

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

	bool IsSphere() const;

	// Returns radius in BulletPhysics units
	float GetRadiusBullet() const;

	// Returns radius in Unreal Engine units (uu)
	float GetRadius() const {
		return GetRadiusBullet() * BT_TO_UU;
	}

	// Returns mass
	float GetMass() const;

	void _PreTickUpdate(GameMode gameMode, float tickTime);
	void _OnHit(
		class Car* car, Vec relPos,
		float& outFriction, float& outRestitution,
		GameMode gameMode, const MutatorConfig& mutatorConfig, uint64_t tickCount
	);
	void _OnWorldCollision(GameMode gameMode, Vec normal, float tickTime);
	// Returns true if the tiles state was modified
	bool _OnDropshotTileCollision(
		DropshotTilesState& tilesState, int tileTotalIndex, const btCollisionObject* tileObj, 
		uint64_t tickCount, float tickTime
	);
		
	Ball(const Ball& other) = delete;
	Ball& operator=(const Ball& other) = delete;

	~Ball() {
		delete _collisionShape;
	}

private:
	Ball() {}
};

RS_NS_END
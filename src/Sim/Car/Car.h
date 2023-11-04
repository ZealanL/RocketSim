#pragma once
#include "CarConfig/CarConfig.h"
#include "../btVehicleRL/btVehicleRL.h"
#include "../CarControls.h"
#include "../BallHitInfo/BallHitInfo.h"
#include "../MutatorConfig/MutatorConfig.h"

#include "../../../libsrc/bullet3-3.24/BulletDynamics/Dynamics/btRigidBody.h"
#include "../../../libsrc/bullet3-3.24/BulletDynamics/Vehicle/btDefaultVehicleRaycaster.h"
#include "../../../libsrc/bullet3-3.24/BulletCollision/CollisionShapes/btBoxShape.h"
#include "../../../libsrc/bullet3-3.24/BulletCollision/CollisionShapes/btCompoundShape.h"
#include "../../../src/Sim/btVehicleRL/btVehicleRL.h"

struct CarState {

	// Incremented every update, reset when SetState() is called
	// Used for telling if a stateset occured
	// Not serialized
	uint64_t updateCounter = 0;

	// Position in world space (UU)
	Vec pos = { 0, 0, 17 };
	
	RotMat rotMat = RotMat::GetIdentity();

	// Linear velocity
	Vec vel = { 0, 0, 0 };

	// Angular velocity (rad/s)
	Vec angVel = { 0, 0, 0 };

	bool isOnGround = true;
	bool hasJumped = false, hasDoubleJumped = false, hasFlipped = false;
	Vec lastRelDodgeTorque = { 0, 0, 0 };

	// Active during the duration of a jump or flip
	float jumpTime = 0, flipTime = 0;

	// True during a flip (not a jump, and not after a flip)
	bool isFlipping = false;

	// True during a jump (not double jumps or a flip)
	bool isJumping = false;

	// Time spent in the air once !isJumping
	float airTimeSinceJump = 0;

	// Goes from 0 to 100
	float boost = RLConst::BOOST_SPAWN_AMOUNT;

	// Added to replicate minimum boosting time
	// NOTE: Will be used even when we have no boost
	float timeSpentBoosting = 0;

	bool isSupersonic = false;

	// Time spent supersonic, for checking with the supersonic maintain time (see RLConst.h)
	float supersonicTime = 0;

	// This is a state variable due to the rise/fall rate of handbrake inputs (see RLConst.h)
	float handbrakeVal = 0;

	bool isAutoFlipping = false;
	float autoFlipTimer = 0; // Counts down when auto-flipping
	float autoFlipTorqueScale = 0;

	struct {
		bool hasContact = false;
		Vec contactNormal;
	} worldContact;

	struct {
		uint32_t otherCarID = 0;
		float cooldownTimer = 0;
	} carContact;

	bool isDemoed = false;
	float demoRespawnTimer = 0;

	BallHitInfo ballHitInfo = BallHitInfo();

	// Controls from last tick, set to this->controls after simulation
	CarControls lastControls = CarControls();

	void Serialize(DataStreamOut& out) const;
	void Deserialize(DataStreamIn& in);
};

#define CARSTATE_SERIALIZATION_FIELDS \
pos, rotMat, vel, angVel, isOnGround, hasJumped, hasDoubleJumped, hasFlipped, \
lastRelDodgeTorque, jumpTime, isFlipping, flipTime, isJumping, airTimeSinceJump, \
boost, timeSpentBoosting, supersonicTime, handbrakeVal, isAutoFlipping, \
autoFlipTimer, autoFlipTorqueScale, isDemoed, demoRespawnTimer, lastControls, \
worldContact.hasContact, worldContact.contactNormal, \
carContact.otherCarID, carContact.cooldownTimer

enum class Team : byte {
	BLUE = 0,
	ORANGE = 1
};

#define RS_OPPOSITE_TEAM(team) ((team) == Team::BLUE ? Team::ORANGE : Team::BLUE)
#define RS_TEAM_FROM_Y(y) ((y) < 0 ? Team::BLUE : Team::ORANGE)

class Car {
public:
	// Configuration for this car
	CarConfig config;
	Team team;

	// Each car is given a unique ID when created by the arena
	// Will always be >0 (unless you somehow reach integer overflow)
	uint32_t id;

	// The controls to simulate the car with
	CarControls controls;

	RSAPI CarState GetState();
	RSAPI void SetState(const CarState& state);

	void Demolish(float respawnDelay = RLConst::DEMO_RESPAWN_TIME);

	// Respawn the car, called after we have been demolished and waited for the respawn timer
	void Respawn(GameMode gameMode, int seed = -1, float boostAmount = RLConst::BOOST_SPAWN_AMOUNT);

	btVehicleRL _bulletVehicle;
	btDefaultVehicleRaycaster _bulletVehicleRaycaster;
	btRigidBody _rigidBody;
	btCompoundShape _compoundShape;
	btBoxShape _childHitboxShape;

	// NOTE: Not all values are updated because they are unneeded for internal simulation
	// Those values are only updated when GetState() is called
	CarState _internalState;

	// Get the forward direction as a unit vector
	Vec GetForwardDir() const {
		return _internalState.rotMat.forward;
	}

	// Get the rightward direction as a unit vector
	Vec GetRightDir() const {
		return _internalState.rotMat.right;
	}

	// Get the upward direction as a unit vector
	Vec GetUpDir() const {
		return _internalState.rotMat.up;
	}

	void _PreTickUpdate(GameMode gameMode, float tickTime, const MutatorConfig& mutatorConfig, struct SuspensionCollisionGrid* grid);
	void _PostTickUpdate(GameMode gameMode, float tickTime, const MutatorConfig& mutatorConfig);

	Vec _velocityImpulseCache = { 0,0,0 };
	void _FinishPhysicsTick(const MutatorConfig& mutatorConfig);

	void _BulletSetup(GameMode gameMode, class btDynamicsWorld* bulletWorld, const MutatorConfig& mutatorConfig);
	
	// For construction by Arena
	static Car* _AllocateCar() { return new Car(); }

	RSAPI void Serialize(DataStreamOut& out);
	void _Deserialize(DataStreamIn& in);

	Car(const Car& other) = delete;
	Car& operator=(const Car& other) = delete;

	~Car() {}

private:
	void _UpdateWheels(float tickTime, const MutatorConfig& mutatorConfig, int numWheelsInContact, float forwardSpeed_UU);
	void _UpdateBoost(float tickTime, const MutatorConfig& mutatorConfig, float forwardSpeed_UU);
	void _UpdateJump(float tickTime, const MutatorConfig& mutatorConfig, bool jumpPressed);
	void _UpdateAirTorque(float tickTime, const MutatorConfig& mutatorConfig, bool doAirControl);
	void _UpdateDoubleJumpOrFlip(float tickTime, const MutatorConfig& mutatorConfig, bool jumpPressed, float forwardSpeed_UU);
	void _UpdateAutoFlip(float tickTime, const MutatorConfig& mutatorConfig, bool jumpPressed);
	void _UpdateAutoRoll(float tickTime, const MutatorConfig& mutatorConfig, int numWheelsInContact);

	Car() {}
};
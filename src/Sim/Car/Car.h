#pragma once
#include "CarConfig/CarConfig.h"
#include "../btVehicleRL/btVehicleRL.h"
#include "../CarControls.h"

struct CarState {
	// Position in world space
	Vec pos;

	Angle angles;

	// Linear velocity
	Vec vel;

	// Angular velocity (rad/s)
	Vec angVel;

	bool isOnGround;
	bool hasJumped, hasDoubleJumped, hasFlipped;
	Vec lastRelDodgeTorque;

	// Active during the duration of a jump or flip
	float jumpTimer, flipTimer;

	// True during a jump (not double jumps or a flip)
	bool isJumping;

	// Time spent in the air once !isJumping
	float airTimeSinceJump;

	// Goes from 0 to 100
	float boost;

	// Added to replicate minimum boosting time
	// NOTE: Will be used even when we have no boost
	float timeSpentBoosting;

	bool isSupersonic;

	// Time spent supersonic, for checking with the supersonic maintain time (see RLConst.h)
	float supersonicTime;

	// This is a state variable due to the rise/fall rate of handbrake inputs (see RLConst.h)
	float handbrakeVal;

	// Controls from last tick, set to this->controls after simulation
	CarControls lastControls;
};

enum class Team {
	BLUE = 0,
	ORANGE = 1
};

class Car {
public:
	CarConfig config;
	Team team;

	// Each car is given a unique ID when created by the arena
	// Will always be >0 (unless you somehow reach integer overflow)
	uint32_t id;

	// The controls to simulate the car with
	CarControls controls;

	// No copy/move constructors
	Car(const Car& other) = delete;
	Car(Car&& other) = delete;

	RSAPI CarState GetState();
	RSAPI void SetState(const CarState& state);

	btVehicleRL* _bulletVehicle;
	btVehicleRaycaster* _bulletVehicleRaycaster;
	btRigidBody* _rigidBody;
	btCompoundShape* _compoundShape;
	btBoxShape* _childHitboxShape;

	// NOTE: Not all values are updated because they are unneeded for internal simulation
	// Those values are only updated when GetState() is called
	CarState _internalState;

	~Car();

	void _PreTickUpdate(float tickTime);

	void _ApplyPhysicsRounding();

	void _PostTickUpdate(float tickTime);

	// For construction by Arena
	static Car* _AllocateCar();
	void _BulletSetup(btDynamicsWorld* bulletWorld);

private:
	Car() {};
};
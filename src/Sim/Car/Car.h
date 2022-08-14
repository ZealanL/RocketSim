#pragma once
#include "CarConfig/CarConfig.h"
#include "../btVehicleRL/btVehicleRL.h"
#include "../CarControls.h"

struct CarState {
	// Position in world space
	Vec pos;

	EulerAngle angles;

	// Linear velocity
	Vec vel;

	// Angular velocity (axis-angle)
	Vec angVel;

	bool isOnGround;
	bool hasJumped, hasDoubleJumped;

	// Goes from 0 to 100
	float boost;

	// This is a state variable due to the supersonic maintain time (see RLConst.h)
	bool isSupersonic;

	// The controls to simulate the car with
	CarControls controls;

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

	// No copy/move constructors
	Car(const Car& other) = delete;
	Car(Car&& other) = delete;

	CarState GetState();
	void SetState(const CarState& state);

	btVehicleRL* _bulletVehicle;
	btVehicleRaycaster* _bulletVehicleRaycaster;
	btRigidBody* _rigidBody;
	btCompoundShape* _compoundShape;
	btBoxShape* _childHitboxShape;

	// NOTE: Not all values are updated because they are unneeded for internal simulation
	// Those values are only updated when GetState() is called
	CarState _internalState;

	~Car();

	// For construction by Arena
	static Car* _AllocateCar();

private:
	Car() {};
};
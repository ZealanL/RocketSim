#pragma once
#include "BaseInc.h"

// Constant/default values from the game

namespace RLConst {
	constexpr float CAR_MASS = 180.f;
	constexpr float BALL_MASS = CAR_MASS / 6.f; // Ref: https://www.reddit.com/r/RocketLeague/comments/bmje9l/comment/emxkwrl/?context=3

	constexpr float BALL_MAX_SPEED = 6000.f;
	constexpr float CAR_MAX_SPEED = 2300.f;

	constexpr float BOOST_MAX = 100.f;
	constexpr float BOOST_USED_PER_SECOND = BOOST_MAX / 3;
	constexpr float BOOST_MIN_TIME = 0.1f; // Minimum time we can be boosting for
	
	// Speed needed to begin being supersonic
	constexpr float SUPERSONIC_START_SPEED = 2200.f; 
	
	// We can still be technically supersonic at this speed if we slow down after reaching supersonic, 
	// until the timer runs out (see timer's time below)
	constexpr float SUPERSONIC_MAINTAIN_MIN_SPEED = SUPERSONIC_START_SPEED - 100.f;

	// How long we can maintain supersonic status while (speed >= SUPERSONIC_MAINTAIN_MIN_SPEED and < SUPERSONIC_START_SPEED)
	constexpr float SUPERSONIC_MAINTAIN_MAX_TIME = 1.f; 

	// Powerslide is actually an analog value from 0-1
	// When held, powerslide goes up by 0.05, and it drops by 0.02 each tick once released
	// Thanks to Rangler for this one :P
	constexpr float 
		POWERSLIDE_RATIO_RISE = 0.05f, 
		POWERSLIDE_RATIO_FALL = 0.02f;

	constexpr float
		THROTTLE_TORQUE_AMOUNT = CAR_MASS * 400.f,
		BRAKE_TORQUE_AMOUNT = CAR_MASS * (14.25f + (1.f/3.f));
		
	constexpr float
		STOPPING_FORWARD_VEL = 25.f,
		COASTING_BRAKE_FACTOR = 0.15f,
		THROTTLE_DEADZONE = 0.001f;

	// Rocket League uses BulletPhysics, so I'd imagine they use a variation of the btRaycastVehicle
	// These are those vehicle's settings
	namespace BTVehicle {
		// TODO: These values might change from car to car...? Need to check!
		constexpr float 
			SUSPENSION_FORCE_SCALE_FRONT = 54.f + (1.f / 4.f) + (1.5f / 100.f),
			SUSPENSION_FORCE_SCALE_BACK = 36.f - (1.f / 4.f);

		constexpr float
			SUSPENSION_STIFFNESS = 500.f,
			WHEELS_DAMPING_COMPRESSION = 25.f,
			WHEELS_DAMPING_RELAXATION = 40.f,
			MAX_SUSPENSION_TRAVEL = 12.f, // TODO: Are we sure this is the same for all cars?
			FRICTION_SLIP_AMOUNT = 2.f; // TODO: Where did I get this number?
	}

	// TODO: Got these from https://github.com/samuelpmish/RLUtilities/blob/develop/src/simulation/ball.cc, but I should check them just in case?
	constexpr float
		BALL_COLLISION_RADIUS_NORMAL = 93.15f, // Soccar, Hoops, etc.
		BALL_COLLISION_RADIUS_DROPSHOT = 103.6f;

	constexpr float SOCCAR_GOAL_SCORE_BASE_THRESHOLD_Y = 5121.75f;
	constexpr float SOCCAR_BALL_SCORE_THRESHOLD_Y = SOCCAR_GOAL_SCORE_BASE_THRESHOLD_Y + BALL_COLLISION_RADIUS_NORMAL;

	inline float 
}
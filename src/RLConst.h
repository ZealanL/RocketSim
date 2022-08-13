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

	// Rocket League uses BulletPhysics, so I'd imagine they use a variation of the btRaycastVehicle
	// These are those vehicle's settings
	namespace BTVehicle {

	}

	constexpr float
		BALL_COLLISION_RADIUS_NORMAL = 93.15f, // Soccar, Hoops, etc.
		BALL_COLLISION_RADIUS_DROPSHOT = 103.6f;
}
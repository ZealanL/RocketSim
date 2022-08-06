#pragma once
#include "BaseInc.h"

// Constant/default values from the game

namespace RLConst {
	constexpr float BALL_MAX_SPEED = 6000.f;
	constexpr float CAR_MAX_SPEED = 2300.f;

	constexpr float BOOST_MAX = 100.f;
	constexpr float BOOST_USED_PER_SECOND = BOOST_MAX / 3;
	constexpr float BOOST_FORCE = 178500.f;
	constexpr float BOOST_MIN_TIME = 0.1f; // Minimum time we can be boosting for

	
	// Speed needed to begin being supersonic
	constexpr float SUPERSONIC_START_SPEED = 2200.f; 
	
	// We can still be technically supersonic at this speed if we slow down after reaching supersonic, 
	// until the timer runs out (see timer's time below)
	constexpr float SUPERSONIC_MAINTAIN_MIN_SPEED = SUPERSONIC_START_SPEED - 100.f;

	// How long we can maintain supersonic status while (speed >= SUPERSONIC_MAINTAIN_MIN_SPEED and < SUPERSONIC_START_SPEED)
	constexpr float SUPERSONIC_MAINTAIN_MAX_TIME = 1.f; 
}
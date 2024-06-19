#pragma once
#include "../../RLConst.h"
#include "../GameMode.h"

#include "../../DataStream/DataStreamIn.h"
#include "../../DataStream/DataStreamOut.h"

RS_NS_START

enum class DemoMode : byte {
	NORMAL,
	ON_CONTACT,
	DISABLED
};

RSAPI struct MutatorConfig {

	Vec gravity = Vec(0, 0, RLConst::GRAVITY_Z);

	float
		carMass = RLConst::CAR_MASS_BT,

		// Friction between car and world (arena)
		carWorldFriction = RLConst::CARWORLD_COLLISION_FRICTION,


		carWorldRestitution = RLConst::CARWORLD_COLLISION_RESTITUTION,

		ballMass,
		ballMaxSpeed = RLConst::BALL_MAX_SPEED,
		ballDrag = RLConst::BALL_DRAG,

		// Friction between car and world (arena)
		ballWorldFriction,

		// Restitution between ball and world (arena)
		ballWorldRestitution,

		jumpAccel = RLConst::JUMP_ACCEL,
		jumpImmediateForce = RLConst::JUMP_IMMEDIATE_FORCE,

		boostAccelGround = RLConst::BOOST_ACCEL_GROUND,
		boostAccelAir = RLConst::BOOST_ACCEL_AIR,
		boostUsedPerSecond = RLConst::BOOST_USED_PER_SECOND,

		respawnDelay = RLConst::DEMO_RESPAWN_TIME,
		bumpCooldownTime = RLConst::BUMP_COOLDOWN_TIME,

		boostPadCooldown_Big = RLConst::BoostPads::COOLDOWN_BIG,
		boostPadCooldown_Small = RLConst::BoostPads::COOLDOWN_SMALL,

		carSpawnBoostAmount = RLConst::BOOST_SPAWN_AMOUNT;

	float
		ballHitExtraForceScale = 1,
		bumpForceScale = 1;

	float
		ballRadius;

	bool
		unlimitedFlips = false,
		unlimitedDoubleJumps = false;

	DemoMode demoMode = DemoMode::NORMAL;
	bool enableTeamDemos = false;

	// Only used if the game mode has soccar goals (i.e. soccar, heatseeker, snowday)
	float goalBaseThresholdY = RLConst::SOCCAR_GOAL_SCORE_BASE_THRESHOLD_Y;

	MutatorConfig(GameMode gameMode);

	void Serialize(DataStreamOut& out) const;
	void Deserialize(DataStreamIn& in);
};

#define MUTATOR_CONFIG_SERIALIZATION_FIELDS \
gravity, carMass, carWorldFriction, carWorldRestitution, ballMass, \
ballMaxSpeed, ballDrag, ballWorldFriction, ballWorldRestitution, jumpAccel, \
jumpImmediateForce, boostAccelGround, boostAccelAir, boostUsedPerSecond, respawnDelay, \
carSpawnBoostAmount, bumpCooldownTime, boostPadCooldown_Big, boostPadCooldown_Small, \
ballHitExtraForceScale, bumpForceScale, ballRadius, unlimitedFlips, unlimitedDoubleJumps, \
demoMode, enableTeamDemos, goalBaseThresholdY

RS_NS_END
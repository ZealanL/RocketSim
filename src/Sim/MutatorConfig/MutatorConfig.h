#pragma once
#include "../../RLConst.h"

#include "../../DataStream/DataStreamIn.h"
#include "../../DataStream/DataStreamOut.h"

enum class DemoMode : byte {
	NORMAL,
	ON_CONTACT,
	DISABLED
};

struct MutatorConfig {

	Vec gravity = Vec(0, 0, RLConst::GRAVITY_Z);

	float
		carMass = RLConst::CAR_MASS_BT,

		// Friction between car and world (arena)
		carWorldFriction = RLConst::CARWORLD_COLLISION_FRICTION,


		carWorldRestitution = RLConst::CARWORLD_COLLISION_RESTITUTION,

		ballMass = RLConst::BALL_MASS_BT,
		ballMaxSpeed = RLConst::BALL_MAX_SPEED,
		ballDrag = RLConst::BALL_DRAG,

		// Friction between car and world (arena)
		ballWorldFriction = RLConst::BALL_FRICTION,

		// Restitution between ball and world (arena)
		ballWorldRestitution = RLConst::BALL_RESTITUTION,

		jumpAccel = RLConst::JUMP_ACCEL,
		jumpImmediateForce = RLConst::JUMP_IMMEDIATE_FORCE,

		boostForce = RLConst::BOOST_FORCE,
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
		ballRadius = RLConst::BALL_COLLISION_RADIUS_NORMAL;

	DemoMode demoMode = DemoMode::NORMAL;
	bool enableTeamDemos = false;

	MutatorConfig() = default;

	RSAPI void Serialize(DataStreamOut& out) const;
	RSAPI void Deserialize(DataStreamIn& in);
};

#define MUTATOR_CONFIG_SERIALIZATION_FIELDS \
gravity, carMass, carWorldFriction, carWorldRestitution, ballMass, \
ballMaxSpeed, ballDrag, ballWorldFriction, ballWorldRestitution, jumpAccel, \
jumpImmediateForce, boostForce, boostUsedPerSecond, respawnDelay, \
carSpawnBoostAmount, bumpCooldownTime, boostPadCooldown_Big, boostPadCooldown_Small, \
ballHitExtraForceScale, bumpForceScale, ballRadius, demoMode, enableTeamDemos
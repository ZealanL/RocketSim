#ifdef RS_PYBIND
#include "PYB.h"

PYB_INIT_F(MutatorConfig) {

	pyb::enum_<DemoMode>(m, "DemoMode")
		.value("DISABLED", DemoMode::DISABLED)
		.value("NORMAL", DemoMode::NORMAL)
		.value("ON_CONTACT", DemoMode::ON_CONTACT);


#define PYB_CUR_CLASS MutatorConfig
	PYB_CLASS(MutatorConfig)
		PYB_DEFAULT_INITS()
		PYBP(ballDrag)
		PYBP(ballHitExtraForceScale)
		PYBP(ballMass)
		PYBP(ballMaxSpeed)
		PYBP(ballRadius)
		PYBP(ballWorldFriction)
		PYBP(ballWorldRestitution)
		PYBP(boostAccel)
		PYBP(boostPadCooldown_Big)
		PYBP(boostPadCooldown_Small)
		PYBP(boostUsedPerSecond)
		PYBP(bumpCooldownTime)
		PYBP(bumpForceScale)
		PYBP(carMass)
		PYBP(carSpawnBoostAmount)
		PYBP(carWorldFriction)
		PYBP(carWorldRestitution)
		PYBP(demoMode)
		PYBP(enablePhysicsRounding)
		PYBP(enableTeamDemos)
		PYBP(gravity)
		PYBP(jumpAccel)
		PYBP(jumpImmediateForce)
		PYBP(respawnDelay)
		PYBP(unlimitedDoubleJumps)
		PYBP(unlimitedFlips)
		PYB_SERIALS()
		;
}
#endif
#ifdef RS_PYBIND
#include "PYB.h"

PYB_INIT_F(Car) {
#define PYB_CUR_CLASS CarControls
	PYB_CLASS()
		PYB_DEFAULT_INITS()
		PYBP(boost)
		PYBP(handbrake)
		PYBP(jump)
		PYBP(pitch)
		PYBP(roll)
		PYBP(steer)
		PYBP(throttle)
		PYBP(yaw)
		;

#define PYB_CUR_CLASS CarState
	PYB_CLASS()
		PYB_DEFAULT_INITS()
		PYBP(airTimeSinceJump)
		PYBP(angVel)
		PYBP(autoFlipTimer)
		PYBP(autoFlipTorqueScale)
		PYBP(ballHitInfo)
		PYBP(boost)
		PYBP(carContact)
		PYBP(demoRespawnTimer)
		PYBP(flipTime)
		PYBP(handbrakeVal)
		PYBP(hasDoubleJumped)
		PYBP(hasFlipped)
		PYBP(hasJumped)
		PYBP(isAutoFlipping)
		PYBP(isDemoed)
		PYBP(isFlipping)
		PYBP(isOnGround)
		PYBP(isSupersonic)
		PYBP(jumpTime)
		PYBP(lastControls)
		PYBP(lastRelDodgeTorque)
		PYBP(pos)
		PYBP(rotMat)
		PYBP(supersonicTime)
		PYBP(timeSpentBoosting)
		PYBP(vel)

		// TODO: Add worldContact struct properly
		.def("get_has_world_contact", [](const CarState& carState) { return carState.worldContact.hasContact; })
		.def("get_world_contact_normal", [](const CarState& carState) { return carState.worldContact.contactNormal; })
		;

#define PYB_CUR_CLASS WheelPairConfig
	PYB_CLASS()
		PYB_DEFAULT_INITS()
		PYBP(connectionPointOffset)
		PYBP(suspensionRestLength)
		PYBP(wheelRadius)
		;

#define PYB_CUR_CLASS CarConfig
	PYB_CLASS()
		PYB_DEFAULT_INITS()
		PYBP(backWheels)
		PYBP(dodgeDeadzone)
		PYBP(frontWheels)
		PYBP(hitboxPosOffset)

		.def_readonly_static("BREAKOUT", &CAR_CONFIG_BREAKOUT)
		.def_readonly_static("DOMINUS", &CAR_CONFIG_DOMINUS)
		.def_readonly_static("HYBRID", &CAR_CONFIG_HYBRID)
		.def_readonly_static("MERC", &CAR_CONFIG_MERC)
		.def_readonly_static("OCTANE", &CAR_CONFIG_OCTANE)
		.def_readonly_static("PLANK", &CAR_CONFIG_PLANK)
		;

#define PYB_CUR_CLASS Car
	PYB_CLASS()
		PYBP(config)
		PYBP(controls)
		PYBP(id)
		PYBP(team)
		.def_readonly("vel_impulse_cache", &Car::_velocityImpulseCache)
		.def("get_state", &Car::GetState)
		.def("set_state", &Car::SetState)
		;
}
#endif
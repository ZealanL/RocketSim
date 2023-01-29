#pragma once

// BulletLink.h: Includes needed bullet headers, and defines some convenient typedefs/wrappers/etc.

// Bullet v3 (https://github.com/bulletphysics/bullet3)
#include "../libsrc/bullet3-3.24/btBulletCollisionCommon.h"
#include "../libsrc/bullet3-3.24/btBulletDynamicsCommon.h"
#include "../libsrc/bullet3-3.24/BulletDynamics/Dynamics/btRigidBody.h"

typedef btVector3 Vec;

//  BulletPhysics Units (1m) to Unreal Units (2cm) conversion scale
#define BT_TO_UU (50.f)

// Unreal Units (2cm) to BulletPhysics Units (1m) conversion scale
#define UU_TO_BT (1.f/50.f)

#define TICKRATE 120
#define TICKTIME (1.f/TICKRATE)

static_assert(TICKRATE <= 120, "Tick rate is too high, must be from 1-120");
static_assert(TICKRATE >= 1, "Tick rate is too low, must be from 1-120");

// NOTE: Values should be in radians
struct Angle {
	float yaw, pitch, roll;
};

// For easy printing of bullet vectors
#include <ostream>
static inline std::ostream& operator <<(std::ostream& stream, const btVector3& v) {
	stream << "[ " << v.x() << ", " << v.y() << ", " << v.z() << " ]";
	return stream;
}
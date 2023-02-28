#pragma once

// BulletLink.h: Includes needed bullet headers, and defines some convenient typedefs/wrappers/etc.

// Bullet v3 (https://github.com/bulletphysics/bullet3)
#include "../libsrc/bullet3-3.24/btBulletCollisionCommon.h"
#include "../libsrc/bullet3-3.24/btBulletDynamicsCommon.h"
#include "../libsrc/bullet3-3.24/BulletDynamics/Dynamics/btRigidBody.h"

#include <ostream>

typedef btVector3 Vec;

//  BulletPhysics Units (1m) to Unreal Units (2cm) conversion scale
#define BT_TO_UU (50.f)

// Unreal Units (2cm) to BulletPhysics Units (1m) conversion scale
#define UU_TO_BT (1.f/50.f)

// Has the same order of application as Rotators in Rocket League (YPR)
// NOTE: Values are in radians
struct Angle {
	float yaw, pitch, roll;

	Angle(float yaw = 0, float pitch = 0, float roll = 0) : yaw(yaw), pitch(pitch), roll(roll) {}

	Angle(btMatrix3x3 mat);
	btMatrix3x3 ToMatrix() const;

	// Limits yaw/pitch/roll to [-pi,pi]/[-pi/2,pi/2]/[-pi,pi] while still representing the same rotation
	void NormalizeFix();
};

// For printing btVector3
static inline std::ostream& operator <<(std::ostream& stream, const btVector3& v) {
	stream << "[ " << v.x() << ", " << v.y() << ", " << v.z() << " ]";
	return stream;
}

// For printing Angle
static inline std::ostream& operator <<(std::ostream& stream, const Angle& ang) {
	stream << "(YPR)[ " << ang.yaw << ", " << ang.pitch << ", " << ang.roll << " ]";
	return stream;
}

// Enum values for Bullet btCollisionObject userinfo usage
enum : int {
	BT_USERINFO_NONE,

	BT_USERINFO_TYPE_CAR,
	BT_USERINFO_TYPE_BALL,
	BT_USERINFO_TYPE_BOOSTPAD
};

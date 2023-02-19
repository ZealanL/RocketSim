#include "BulletLink.h"

#include "Framework.h"

#define _USE_MATH_DEFINES
#include <math.h>

Angle::Angle(btMatrix3x3 mat) {
	// Based off of QuatToRotator in https://github.com/bakkesmodorg/BakkesModSDK/blob/master/include/bakkesmod/wrappers/wrapperstructs.h

	Vec
		forward = mat.getColumn(0),
		right = mat.getColumn(1),
		up = mat.getColumn(2);

	pitch = asinf(RS_CLAMP(forward.z(), -1, 1));

	float horizonMagnitude = (forward * Vec(1, 1, 0)).length();
	yaw = asinf(RS_CLAMP(forward.y() / horizonMagnitude, -1, 1));
	if (yaw >= 0) {
		if (forward.x() < 0)
			yaw = M_PI - yaw;
	} else {
		if (forward.x() < 0)
			yaw = -M_PI - yaw;
	}

	Vec vert = (up.z() < 0) ? Vec(0, 0, -1) : Vec(0, 0, 1);
	Vec horizonRight = (forward.cross(vert) * -1).normalized();

	float horizonRightDot = horizonRight.dot(right);
	roll = acosf(RS_CLAMP(horizonRightDot, -1, 1));
	float upSin = asinf(RS_CLAMP(up.z(), -1, 1));

	if (right.z() >= 0) {
		if (up.z() >= 0) {
			roll = -roll;
		} else {
			roll = -M_PI + roll;
		}
	} else {
		if (up.z() < 0)
			roll = M_PI - roll;
	}
}

btMatrix3x3 Angle::ToMatrix() {
	Vec forward, right, up;

	float
		sy = sinf(yaw),		cy = cosf(yaw),
		sp = sinf(pitch),	cp = cosf(pitch),
		sr = sinf(roll),	cr = cosf(roll);

	return btMatrix3x3(
		(cp * cy), -(-1 * sr * sp * cy + -1 * cr * -sy), (cr * sp * cy + -sr * -sy),
		(cp * sy), -(-1 * sr * sp * sy + -1 * cr * cy), (cr * sp * sy + -sr * cy),
		-(-sp), (-1 * sr * cp), -(cr * cp)
	);
}
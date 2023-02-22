#include "BulletLink.h"

#include "Framework.h"

#define _USE_MATH_DEFINES
#include <math.h>

Angle::Angle(btMatrix3x3 mat) {
	mat.getEulerYPR(yaw, pitch, roll);
	pitch *= -1;
	roll *= -1;
}

btMatrix3x3 Angle::ToMatrix() {
	btMatrix3x3 mat;
	mat.setEulerYPR(yaw, -pitch, -roll);
	return mat;
}
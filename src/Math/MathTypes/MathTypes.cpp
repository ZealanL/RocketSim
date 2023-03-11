#include "MathTypes.h"

#include "../Math.h"

#define VEC_OP_VEC(op) \
Vec Vec::operator op(const Vec& other) const { return Vec(x op other.x, y op other.y, z op other.z); } \
Vec& Vec::operator op##=(const Vec& other) { return *this = *this op other; }

#define VEC_OP_FLT(op) \
Vec Vec::operator op(float val) const { return Vec(x op val, y op val, z op val); } \
Vec operator op(float val, const Vec& vec) { return Vec(val op vec.x, val op vec.y, val op vec.z); } \
Vec& Vec::operator op##=(float val) { return *this = *this op val; }

VEC_OP_VEC(+)
VEC_OP_VEC(-)
VEC_OP_VEC(*)
VEC_OP_VEC(/)

VEC_OP_FLT(*)
VEC_OP_FLT(/)

#define MAT_OP_EACH_MAT(op) \
RotMat RotMat::operator op(const RotMat& other) const { \
	RotMat result; \
	for (int i = 0; i < 3; i++) \
		for (int j = 0; j < 3; j++) \
			result[i][j] = (*this)[i][j] op other[i][j]; \
	return result; \
} \
RotMat& RotMat::operator op##=(const RotMat& other) { \
	return *this = *this op other; \
}

MAT_OP_EACH_MAT(+)
MAT_OP_EACH_MAT(-)

#undef MAT_OP_EACH_MAT

#define MAT_OP_EACH_FLT(op) \
RotMat RotMat::operator op(float val) const { \
	RotMat result; \
	for (int i = 0; i < 3; i++) \
		for (int j = 0; j < 3; j++) \
			result[i][j] = (*this)[i][j] op val; \
	return result; \
} \
RotMat& RotMat::operator op##=(float val) { \
	return *this = *this op val; \
}

MAT_OP_EACH_FLT(*)
MAT_OP_EACH_FLT(/)

#undef MAT_OP_EACH_FLT

Angle::Angle(RotMat mat) {
	// TODO: Don't use btMatrix3x3
	btMatrix3x3 bulletMat = mat;
	bulletMat.getEulerYPR(yaw, pitch, roll);
	pitch *= -1;
	roll *= -1;
}

RotMat Angle::ToMatrix() const {
	// TODO: Don't use btMatrix3x3
	btMatrix3x3 mat;
	mat.setEulerYPR(yaw, -pitch, -roll);
	return mat;
}

Vec Angle::GetForwardVector() const {
	float
		cy = cosf(yaw),
		cp = cosf(-pitch),
		sp = sinf(-pitch);
	return Vec(cy * cp, cy * sp, -sp);
}

void Angle::NormalizeFix() {
	yaw = Math::WrapNormalizeFloat(yaw, M_PI);
	pitch = Math::WrapNormalizeFloat(pitch, M_PI / 2);
	roll = Math::WrapNormalizeFloat(roll, M_PI);
}
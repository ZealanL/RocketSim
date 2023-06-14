#include "MathTypes.h"

#include "../Math.h"

#define VEC_OP_VEC(op) \
Vec Vec::operator op(const Vec& other) const { return Vec(x op other.x, y op other.y, z op other.z, _w op other._w); } \
Vec& Vec::operator op##=(const Vec& other) { return *this = *this op other; }

#define VEC_OP_FLT(op) \
Vec Vec::operator op(float val) const { return Vec(x op val, y op val, z op val, _w op val); } \
Vec operator op(float val, const Vec& vec) { return Vec(val op vec.x, val op vec.y, val op vec.z, val op vec._w); } \
Vec& Vec::operator op##=(float val) { return *this = *this op val; }

VEC_OP_VEC(+)
VEC_OP_VEC(-)
VEC_OP_VEC(*)
VEC_OP_VEC(/)

VEC_OP_FLT(*)
VEC_OP_FLT(/)

////////////////////////////////////

#define MAT_OP_EACH_MAT(op) \
RotMat RotMat::operator op(const RotMat& other) const { \
	RotMat result; \
	for (int i = 0; i < 3; i++) \
		for (int j = 0; j < 4; j++) \
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
		for (int j = 0; j < 4; j++) \
			result[i][j] = (*this)[i][j] op val; \
	return result; \
} \
RotMat& RotMat::operator op##=(float val) { \
	return *this = *this op val; \
}

MAT_OP_EACH_FLT(*)
MAT_OP_EACH_FLT(/)

#undef MAT_OP_EACH_FLT

RSAPI RotMat RotMat::LookAt(Vec forwardDir, Vec upDir) {
	Vec 
		f = forwardDir.Normalized(),
		tr = upDir.Cross(f),
		u = f.Cross(tr).Normalized(),
		r = u.Cross(f).Normalized();

	return RotMat(f, r, u);
}

//////////////////////////////////////

Angle Angle::FromRotMat(RotMat mat) {
	Angle result;

	// TODO: Don't use btMatrix3x3
	btMatrix3x3 bulletMat = mat;
	bulletMat.getEulerYPR(result.yaw, result.pitch, result.roll);
	result.pitch *= -1;
	result.roll *= -1;
	return result;
}

RotMat Angle::ToRotMat() const {
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
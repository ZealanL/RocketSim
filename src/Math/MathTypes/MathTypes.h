#pragma once
#include "../../BaseInc.h"

RS_NS_START

// RocketSim 3D vector struct
struct RS_ALIGN_16 Vec {
	float x, y, z;

	float _w; // 4th component to get compiler to use SIMD operations

	constexpr Vec() {
		x = y = z = _w = 0;
	}

	constexpr Vec(float x, float y, float z, float _w = 0) : x(x), y(y), z(z), _w(_w) {}

	Vec(const btVector3& bulletVec) {
		*(btVector3*)this = bulletVec;
	}

	bool IsZero() const {
		return (x == 0 && y == 0 && z == 0 && _w == 0);
	}

	// Makes a copy with zeroed z
	Vec To2D() const {
		return Vec(x, y, 0);
	}

	float LengthSq() const {
		return (x * x + y * y + z * z + _w * _w);
	}

	float Length() const {
		float lengthSq = LengthSq();
		if (lengthSq > 0) {
			return sqrtf(lengthSq);
		} else {
			return 0;
		}
	}

	float LengthSq2D() const {
		return (x * x + y * y);
	}

	float Length2D() const {
		float lengthSq2D = LengthSq2D();
		if (lengthSq2D > 0) {
			return sqrtf(lengthSq2D);
		} else {
			return 0;
		}
	}

	float Dot(const Vec& other) const {
		return (x * other.x + y * other.y + z * other.z + _w * other._w);
	}

	Vec Cross(const Vec& other) const {
		return Vec(
			(y * other.z) - (z * other.y),
			(z * other.x) - (x * other.z),
			(x * other.y) - (y * other.x)
		);
	}

	float DistSq(const Vec& other) const {
		return (*this - other).LengthSq();
	}

	float Dist(const Vec& other) const {
		return sqrtf(DistSq(other));
	}

	float DistSq2D(const Vec& other) const {
		float dx = this->x - other.x;
		float dy = this->y - other.y;
		return (dx * dx + dy * dy);
	}

	float Dist2D(const Vec& other) const {
		return sqrtf(DistSq2D(other));
	}

	// NOTE: Safe
	Vec Normalized() const {
		float length = Length();
		if (length > FLT_EPSILON * FLT_EPSILON) {
			return *this / length;
		} else {
			return Vec();
		}
	}

	float& operator[](uint32_t index) {
		assert(index >= 0 && index < 3);
		return ((float*)this)[index];
	}

	float operator[](uint32_t index) const {
		assert(index >= 0 && index < 3);
		return ((float*)this)[index];
	}

	operator btVector3() const {
		return *(btVector3*)(this);
	}

	RSAPI Vec operator+(const Vec& other) const;
	RSAPI Vec operator-(const Vec& other) const;
	RSAPI Vec operator*(const Vec& other) const;
	RSAPI Vec operator/(const Vec& other) const;

	RSAPI Vec& operator+=(const Vec& other);
	RSAPI Vec& operator-=(const Vec& other);
	RSAPI Vec& operator*=(const Vec& other);
	RSAPI Vec& operator/=(const Vec& other);

	RSAPI Vec operator*(float val) const;
	RSAPI Vec operator/(float val) const;

	RSAPI Vec& operator*=(float val);
	RSAPI Vec& operator/=(float val);

	bool operator<(const Vec& other) const {
		return (x < other.x) && (y < other.y) && (z < other.z);
	}

	bool operator>(const Vec& other) const {
		return (x > other.x) && (y > other.y) && (z > other.z);
	}

	Vec operator-() const {
		return Vec(-x, -y, -z, -_w);
	}

	bool operator==(const Vec& other) const {
		return
			(x == other.x) &&
			(y == other.y) &&
			(z == other.z) &&
			(_w == other._w);
	}

	bool operator!=(const Vec& other) const {
		return !(*this == other);
	}

	friend std::ostream& operator<<(std::ostream& stream, const Vec& vec) {
		stream << "[ " << vec.x << ", " << vec.y << ", " << vec.z << " ]";
		return stream;
	}
};

// Vec needs to be equal in both size and structure layout to btVector3, because they are type-punned to and from
static_assert(sizeof(Vec) == sizeof(btVector3), "RocketSim Vec size must match btVector3 size");

// RocketSim 3x3 rotation matrix struct
// NOTE: Column-major
struct RS_ALIGN_16 RotMat {
	Vec forward, right, up;

	RotMat() {
		forward = right = up = Vec();
	}

	RotMat(Vec forward, Vec right, Vec up) : forward(forward), right(right), up(up) {}

	RotMat(const btMatrix3x3& bulletMat) {
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				// NOTE: btMatrix3x3 is row-major, whereas we are column-major
				(*this)[i][j] = bulletMat[j][i];
			}
		}
	}

	static RotMat GetIdentity() {
		return RotMat(
			Vec(1, 0, 0),
			Vec(0, 1, 0),
			Vec(0, 0, 1)
		);
	}

	// NOTE: up does not have to be at a right angle from forward
	static RotMat LookAt(Vec forwardDir, Vec upDir) {
		Vec
			f = forwardDir.Normalized(),
			tr = upDir.Cross(f),
			u = f.Cross(tr).Normalized(),
			r = u.Cross(f).Normalized();

		return RotMat(f, r, u);
	}

	Vec operator[](uint32_t index) const {
		assert(index >= 0 && index < 3);
		return ((Vec*)(this))[index];
	}

	Vec& operator[](uint32_t index) {
		assert(index >= 0 && index < 3);
		return ((Vec*)(this))[index];
	}

	operator btMatrix3x3() const {
		btMatrix3x3 result;
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				// NOTE: btMatrix3x3 is row-major, whereas we are column-major
				result[i][j] = (*this)[j][i];
			}
		}
		return result;
	}

	RSAPI RotMat operator+(const RotMat& other) const;
	RSAPI RotMat operator-(const RotMat& other) const;

	RSAPI RotMat& operator+=(const RotMat& other);
	RSAPI RotMat& operator-=(const RotMat& other);

	RSAPI RotMat operator*(float val) const;
	RSAPI RotMat operator/(float val) const;

	RSAPI RotMat& operator*=(float val);
	RSAPI RotMat& operator/=(float val);

	bool operator==(const RotMat& other) const {
		return
			(forward == other.forward) &&
			(right == other.right) &&
			(up == other.up);
	}

	bool operator!=(const RotMat& other) const {
		return !(*this == other);
	}

	Vec Dot(const Vec& vec) const {
		return Vec(
			vec.Dot(forward),
			vec.Dot(right),
			vec.Dot(up)
		);
	}

	RotMat Dot(const RotMat& other) const {
		RotMat result;

		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				for (int k = 0; k < 3; k++)
					result[i][j] += (*this)[i][j] * other[k][j];

		return result;
	}

	RotMat Transpose() const {
		RotMat result;
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				result[i][j] = (*this)[j][i];
		return result;
	}

	friend std::ostream& operator<<(std::ostream& stream, const RotMat& mat) {
		stream << "(FRU)(" << std::endl;
		for (int i = 0; i < 3; i++)
			stream << "\t" << mat[i] << (i < 2 ? "," : "") << std::endl;
		stream << ")";
		return stream;
	}
};

// RocketSim euler angle struct
// Has the same order of application as Rotators in Rocket League (YPR), values are in radians
struct Angle {
	float yaw, pitch, roll;

	Angle(float yaw = 0, float pitch = 0, float roll = 0) : yaw(yaw), pitch(pitch), roll(roll) {}

	RSAPI static Angle FromRotMat(RotMat mat);
	RSAPI RotMat ToRotMat() const;

	RSAPI static Angle FromVec(const Vec& forward);
	RSAPI Vec GetForwardVec() const;

	// Limits yaw/pitch/roll to [-pi,pi]/[-pi/2,pi/2]/[-pi,pi] while still representing the same rotation
	RSAPI void NormalizeFix();

	bool operator==(const Angle& other) const {
		return (yaw == other.yaw) && (pitch == other.pitch) && (roll == other.roll);
	}

	Angle GetDeltaTo(const Angle& other) const {
		Angle delta = Angle(other.yaw - yaw, other.pitch - pitch, other.roll - roll);
		delta.NormalizeFix();
		return delta;
	}

	Angle operator+(const Angle& other) const {
		Angle combined = Angle(other.yaw + yaw, other.pitch + pitch, other.roll + roll);
		combined.NormalizeFix();
		return combined;
	}

	Angle operator-(const Angle& other) const {
		return other.GetDeltaTo(*this);
	}

	float operator[](size_t index) const {
		assert(index < 3);
		// TODO: Kind of lame? Unsure
		return (index == 0) ? yaw : ((index == 1) ? pitch : roll);
	}

	float& operator[](size_t index) {
		assert(index < 3);
		return (index == 0) ? yaw : ((index == 1) ? pitch : roll);
	}

	friend std::ostream& operator<<(std::ostream& stream, const Angle& ang) {
		stream << "(YPR)[ " << ang.yaw << ", " << ang.pitch << ", " << ang.roll << " ]";
		return stream;
	}
};

RS_NS_END
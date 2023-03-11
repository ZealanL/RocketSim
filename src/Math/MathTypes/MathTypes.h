#pragma once
#include "../../BaseInc.h"

// RocketSim 3D vector struct
struct Vec {
	float x, y, z;

	Vec() {
		x = y = z = 0;
	}

	Vec(float x, float y, float z) : x(x), y(y), z(z) {}

	Vec(const btVector3& bulletVec) {
		for (int i = 0; i < 3; i++)
			(*this)[i] = bulletVec[i];
	}

	bool IsZero() const {
		return (x == 0 && y == 0 && z == 0);
	}

	float LengthSq() const {
		return (x * x + y * y + z * z);
	}

	float Length() const {
		float lengthSq = LengthSq();
		if (lengthSq > 0) {
			return sqrtf(lengthSq);
		} else {
			return 0;
		}
	}

	float Dot(const Vec& other) const {
		return (x * other.x + y * other.y + z * other.z);
	}

	Vec Cross(const Vec& other) const {
		return Vec(
			 (y * other.z) - (z * other.y),
			-(x * other.z) - (z * other.x),
			 (x * other.y) - (y * other.x)
		);
	}

	float DistSq(const Vec& other) const {
		return (*this - other).LengthSq();
	}

	float Dist(const Vec& other) const {
		return sqrtf(DistSq(other));
	}

	Vec Normalized() const {
		float length = Length();
		if (length > 0) {
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
		return btVector3(x, y, z);
	}

	Vec operator+(const Vec& other) const;
	Vec operator-(const Vec& other) const;
	Vec operator*(const Vec& other) const;
	Vec operator/(const Vec& other) const;

	Vec& operator+=(const Vec& other);
	Vec& operator-=(const Vec& other);
	Vec& operator*=(const Vec& other);
	Vec& operator/=(const Vec& other);

	Vec operator*(float val) const;
	Vec operator/(float val) const;

	Vec& operator*=(float val);
	Vec& operator/=(float val);

	Vec operator-() const {
		return Vec(-x, -y, -z);
	}

	friend std::ostream& operator<<(std::ostream& stream, const Vec& vec) {
		stream << "[ " << vec.x << ", " << vec.y << ", " << vec.z << " ]";
		return stream;
	}
};

// RocketSim 3x3 rotation matrix struct
// NOTE: Column-major
struct RotMat {
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

	RotMat operator+(const RotMat& other) const;
	RotMat operator-(const RotMat& other) const;

	RotMat& operator+=(const RotMat& other);
	RotMat& operator-=(const RotMat& other);

	RotMat operator*(float val) const;
	RotMat operator/(float val) const;

	RotMat& operator*=(float val);
	RotMat& operator/=(float val);

	Vec Dot(const Vec& vec) const {
		return Vec(
			vec.Dot(forward),
			vec.Dot(right),
			vec.Dot(up)
		);
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

	static Angle FromRotMat(RotMat mat);
	RotMat ToRotMat() const;

	Vec GetForwardVector() const;

	// Limits yaw/pitch/roll to [-pi,pi]/[-pi/2,pi/2]/[-pi,pi] while still representing the same rotation
	void NormalizeFix();

	friend std::ostream& operator<<(std::ostream& stream, const Angle& ang) {
		stream << "(YPR)[ " << ang.yaw << ", " << ang.pitch << ", " << ang.roll << " ]";
		return stream;
	}
};
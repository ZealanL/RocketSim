#pragma once
#include "../BaseInc.h"

RS_NS_START

struct LinearPieceCurve {
	std::map<float, float> valueMappings;

	float GetOutput(float input, float defaultOutput = 1) const;
};

namespace Math {
	btVector3 RoundVec(btVector3 vec, float precision);

	// NOTE: min is inclusive, max is exclusive
	// Seed will be used if not -1
	int RandInt(int min, int max, int seed = -1);

	float RandFloat(float min = 0, float max = 1);

	std::default_random_engine& GetRandEngine();

	float WrapNormalizeFloat(float val, float minmax);

	// Simulates aggressive UE3 rotator rounding when converting from a UE3 rotator to vector
	Angle RoundAngleUE3(Angle ang);

	// Rounds to the nearest 1 / scale, halves up
	inline float RoundHalfUp(float val, float scale) {
		float y = val * scale;
		y = y + 0.5f; // (A separate statement, so it's never fused into a multiply-add)
		return floorf(y) / scale;
	}
}

RS_NS_END
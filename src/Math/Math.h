#pragma once
#include "../BaseInc.h"

RS_NS_START

struct LinearPieceCurve {
	std::map<float, float> valueMappings;

	RSAPI float GetOutput(float input, float defaultOutput = 1) const;
};

namespace Math {
	RSAPI btVector3 RoundVec(btVector3 vec, float precision);

	// NOTE: min is inclusive, max is exclusive
	// Seed will be used if not -1
	RSAPI int RandInt(int min, int max, int seed = -1);

	RSAPI float RandFloat(float min = 0, float max = 1);

	RSAPI std::default_random_engine& GetRandEngine();

	RSAPI float WrapNormalizeFloat(float val, float minmax);

	// Simulates aggressive UE3 rotator rounding when converting from a UE3 rotator to vector
	RSAPI Angle RoundAngleUE3(Angle ang);
}

RS_NS_END
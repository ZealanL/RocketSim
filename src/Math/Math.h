#pragma once
#include "../BaseInc.h"

struct LinearPieceCurve {
	map<float, float> valueMappings;

	float GetOutput(float input, float defaultOutput = 1) const;
};

namespace Math {
	btVector3 RoundVec(btVector3 vec, float precision);
}
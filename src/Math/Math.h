#pragma once
#include "../BaseInc.h"

struct LinearPieceCurve {
	map<float, float> valueMappings;

	float GetOutput(float input) const;
};

namespace Math {
	
}
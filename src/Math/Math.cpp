#include "Math.h"

float LinearPieceCurve::GetOutput(float input, float defaultOutput) const {
	float output = input;

	if (!valueMappings.empty()) {

		// Make sure it isnt before/at the first value mapping
		auto& firstValPair = *valueMappings.begin();
		if (input <= firstValPair.first)
			return firstValPair.second;

		for (auto itr = std::next(valueMappings.begin()); itr != valueMappings.end(); itr++) {
			if (itr->first > input) {
				// Found the point bigger than it! Get surrounding
				auto& afterPair = *itr;
				auto& beforePair = *std::prev(itr);

				float rangeBetween = afterPair.first - beforePair.first;
				float valDiffBetween = afterPair.second - beforePair.second;
				float linearInterpFactor = (input - beforePair.first) / rangeBetween;
				return beforePair.second + valDiffBetween * linearInterpFactor;
			}
		}

		// Must be beyond the largest input mapping, return that
		return std::prev(valueMappings.end())->second;
	} else {
		return defaultOutput;
	}

	return output;
}

btVector3 Math::RoundVec(btVector3 vec, float precision) {
	vec.x() = roundf(vec.x() / precision) * precision;
	vec.y() = roundf(vec.y() / precision) * precision;
	vec.z() = roundf(vec.z() / precision) * precision;
	return vec;
}
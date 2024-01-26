#include "Math.h"

RS_NS_START

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

int Math::RandInt(int min, int max, int seed) {
	if (seed != -1) {
		std::default_random_engine tempEngine = std::default_random_engine(seed);
		return min + (tempEngine() % (max - min));
	} else {
		std::default_random_engine& randEngine = GetRandEngine();
		return min + (randEngine() % (max - min));
	}
}

float Math::RandFloat(float min, float max) {
	std::default_random_engine& randEngine = GetRandEngine();
	return min + ((randEngine() / (float)randEngine.max()) * (max - min));
}

std::default_random_engine& Math::GetRandEngine() {
	static thread_local auto hashThreadID = std::hash<std::thread::id>();
	static thread_local uint64_t seed = RS_CUR_MS() + hashThreadID(std::this_thread::get_id());
	static thread_local std::default_random_engine randEngine = std::default_random_engine(seed);
	return randEngine;
}

float Math::WrapNormalizeFloat(float val, float minmax) {
	float result = fmod(val, minmax * 2);
	if (result > minmax)
		result -= minmax * 2;
	else if (result < -minmax)
		result += minmax * 2;
	return result;
}

RS_NS_END
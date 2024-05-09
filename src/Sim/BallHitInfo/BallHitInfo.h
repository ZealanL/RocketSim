#pragma once

#include "../../RLConst.h"
#include "../../DataStream/DataStreamIn.h"
#include "../../DataStream/DataStreamOut.h"

RS_NS_START

struct BallHitInfo {
	// If false, all other fields within this struct should not be trusted
	bool isValid = false;

	Vec relativePosOnBall; // Position of the hit relative to the ball's position
	Vec ballPos; // World position of the ball when the hit occured
	Vec extraHitVel; // Extra velocity added to base collision velocity

	// Arena tick count when the hit occured
	uint64_t tickCountWhenHit = ~0ULL; 

	// Arena tick count when the last extra ball-car hit impulse was applied
	// This is needed since the extra ball-car hit impulse cannot be applied on two consecutive ticks
	uint64_t tickCountWhenExtraImpulseApplied = ~0ULL;

	void Serialize(DataStreamOut& out) const;
	void Deserialize(DataStreamIn& in);
};

// NOTE: Does not include isValid
#define BALLHITINFO_SERIALIZATION_FIELDS \
relativePosOnBall, ballPos, extraHitVel, tickCountWhenHit, tickCountWhenExtraImpulseApplied

RS_NS_END
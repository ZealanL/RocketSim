#include "BallPredTracker.h"

BallPredTracker::BallPredTracker(Arena* arena, size_t numPredTicks) : numPredTicks(numPredTicks) {
	// Make ball pred arena
	this->ballPredArena = Arena::Create(arena->gameMode, ArenaMemWeightMode::LIGHT, arena->GetTickRate());
	this->ballPredArena->tickCount = arena->tickCount;

	predData.reserve(numPredTicks);
	UpdatePred(arena);
}

BallPredTracker::~BallPredTracker() {
	delete this->ballPredArena;
}

void BallPredTracker::UpdatePred(Arena* arena) {
	uint64_t curTickCount = arena->tickCount;
	uint64_t lastPredTickCount = ballPredArena->tickCount;

	if (lastPredTickCount > curTickCount) {
		// How many ball pred ticks are now old
		uint64_t predStartOffset = numPredTicks - (lastPredTickCount - curTickCount);

		// Data may be re-usable
		if (predStartOffset == numPredTicks) {
			// Already predicted for this tick
			return;
		}

		if (predStartOffset < predData.size()) {
			BallState curBallState = arena->ball->GetState();
			if (curBallState.Matches(predData[predStartOffset])) {
				// Pred matches real ball, continue prediction from here
				ballPredArena->ball->SetState(curBallState);
				predData.erase(predData.begin(), predData.begin() + predStartOffset);
				predData.resize(numPredTicks);
				for (uint64_t i = predStartOffset; i < numPredTicks; i++) {
					ballPredArena->Step();
					predData[i] = ballPredArena->ball->GetState();
				}
				return;
			} else {
				// Doesn't match
			}
		}
	}

	// Full re-simulation required
	ForceUpdateAllPred(arena);
}

void BallPredTracker::ForceUpdateAllPred(Arena* arena) {
	ballPredArena->ball->SetState(arena->ball->GetState());
	predData.resize(numPredTicks);
	for (size_t i = 0; i < numPredTicks; i++) {
		ballPredArena->Step();
		predData[i] = ballPredArena->ball->GetState();
	}
}

BallState BallPredTracker::GetBallStateForTime(float predTime) const {
	return BallState();
}

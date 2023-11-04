#pragma once
#include "../Arena/Arena.h"

// An external tool struct that predicts the ball of a given arena
struct BallPredTracker {
	Arena* ballPredArena;
	std::vector<BallState> predData;
	size_t numPredTicks;

	BallPredTracker(Arena* arena, size_t numPredTicks);
	~BallPredTracker();

	// No copying
	BallPredTracker(const BallPredTracker& other) = delete;
	BallPredTracker& operator=(const BallPredTracker& other) = delete;

	// Update the prediction data, does not need to be called every tick
	void UpdatePred(Arena* arena);

	// Forcefully re-predicts all ticks
	void ForceUpdateAllPred(Arena* arena);

	// Get the predicted ball state at a given future time delta
	BallState GetBallStateForTime(float predTime) const;
};
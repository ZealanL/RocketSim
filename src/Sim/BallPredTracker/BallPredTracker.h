#pragma once
#include "../Arena/Arena.h"

RS_NS_START

// An external tool struct that predicts the ball of a given arena
struct BallPredTracker {
	Arena* ballPredArena;
	std::vector<BallState> predData;
	size_t numPredTicks;

	// arena: The arena you want to predict the ball for (BallPredTracker will make a copy of it without the cars)
	// You do not need to make another arena for BallPredTracker, it does that itself
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

RS_NS_END
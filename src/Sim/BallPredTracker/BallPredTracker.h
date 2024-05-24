#pragma once
#include "../Arena/Arena.h"

RS_NS_START

// An external tool struct that predicts the ball of a given arena
struct BallPredTracker {
	Arena* ballPredArena;
	std::vector<BallState> predData;
	size_t numPredTicks;

	int lastUpdateTickCount;

	// arena: The arena you want to predict the ball for (BallPredTracker will make a copy of it without the cars)
	// You do not need to make another arena for BallPredTracker, it does that itself
	BallPredTracker(Arena* arena, size_t numPredTicks);
	~BallPredTracker();

	// No copying
	BallPredTracker(const BallPredTracker& other) = delete;
	BallPredTracker& operator=(const BallPredTracker& other) = delete;

	// Update the prediction data from the arena the ball is in, does not need to be called every tick
	// The arena is needed for the current ball state, as well as the tick count to determine time since last update
	void UpdatePredFromArena(Arena* arena);

	// An alternate version of UpdatePred which doesn't require the arena, 
	//	but instead you manually provide the current ball state and the ticks since this tracker was last updated
	void UpdatePredManual(const BallState& curBallState, int ticksSinceLastUpdate);

	// Forcefully re-predicts all ticks
	void ForceUpdateAllPred(const BallState& initialBallState);

	// Get the predicted ball state at a given future time delta
	BallState GetBallStateForTime(float predTime) const;
};

RS_NS_END
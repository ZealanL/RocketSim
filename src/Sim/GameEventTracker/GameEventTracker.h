#pragma once
#include "../Arena/Arena.h"

RS_NS_START

typedef std::function<void(class Arena* arena, Car* shooter, Car* passer, void* userInfo)> ShotEventFn;
typedef std::function<void(class Arena* arena, Car* scorer, Car* passer, void* userInfo)> GoalEventFn;
typedef std::function<void(class Arena* arena, Car* saver, void* userInfo)> SaveEventFn;

struct GameEventTrackerConfig {
	// NOTE: These are not the same values as the game, they are just what makes sense to me
	// You should probably change them to what you want/need
	
	// Minimum speed towards net in UU/S to be considered a shot
	// For reference: 
	// 50kph = 1388uu/s
	// 82.8kph = 2300uu/s (car max speed)
	// 100kph = 2777uu/s
	float shotMinSpeed = 1750;

	// Time since the shooting car last touched the ball for a shot to be considered
	float shotTouchMinDelay = 0.3f;

	// Added margin for predicting if the ball is scoring or not
	// By default it errs on the side of scoring (not quite to the extent of normal RL), but this value can be made negative to reverse these effects
	float predScoreExtraMargin = 0;

	// Minimum time between shot events
	// Prevents a scenario in which you can quickly farm shots by repeatedly hitting the ball towards the net and blocking it
	float shotEventCooldown = 1.0f;

	// Minimum time to score (or hit backwall/post/crossbar/whatever) for a shot to be counted
	float shotMinScoreTime = 2.0f;

	// Maximum time between a car hitting the ball and it going in the opposing net for it to be counted as a goal
	float goalMaxTouchTime = 4.0f;

	// Maximum time between the touch of the shooting car and the passing car
	float passMaxTouchTime = 2.0f;
};

#define GAMEEVENTTRACKER_CONFIG_SERIALIZATION_FIELDS \
	

// An external tool struct that tracks saves, shots, assists, and goals
// When Update() is called and one of these events occurs, the associated callback will be called (if a callback is registered)
// Note that bumps and demos are not tracked as they are already trackable through arena callbacks
struct GameEventTracker {
	GameEventTrackerConfig config = {};

	struct {
		ShotEventFn func = NULL;
		void* userInfo = NULL;
	} _shotCallback;
	void SetShotCallback(ShotEventFn callbackFn, void* userInfo = NULL) {
		_shotCallback.func = callbackFn;
		_shotCallback.userInfo = userInfo;
	}

	struct {
		GoalEventFn func = NULL;
		void* userInfo = NULL;
	} _goalCallback;
	void SetGoalCallback(GoalEventFn callbackFn, void* userInfo = NULL) {
		_goalCallback.func = callbackFn;
		_goalCallback.userInfo = userInfo;
	}

	struct {
		SaveEventFn func = NULL;
		void* userInfo = NULL;
	} _saveCallback;
	void SetSaveCallback(SaveEventFn callbackFn, void* userInfo = NULL) {
		_saveCallback.func = callbackFn;
		_saveCallback.userInfo = userInfo;
	}

	float _shotCooldown = 0;
	bool _ballShot = false;
	Team _ballShotGoalTeam = {};
	bool _ballScoredLast = false;

	// Used to check if the ball was stateset since last Update()
	// If the update count decreased, reset persistent info
	// If the update count is the same, the update is skipped
	uint64_t _lastBallUpdateCount = 0;

	// Doesn't need to be every tick, but should be called pretty frequently, otherwise events may be missed
	// If you're running an ML bot with tick-skip, update it at that interval
	RSAPI void Update(Arena* arena);

	// Automatically detect when the ball's state is set and clear persistent info
	// Turn this off if it is happening when you don't want it, and you are going to call ResetPersistentInfo() manually 
	bool autoStateSetDetection = true;

	// Resets info that is maintained between ticks
	// Automatically called from Update() when the ball's state has been set since last update
	// Call this whenever you set the arena to a new state if you want to be extra safe
	RSAPI void ResetPersistentInfo();
};

RS_NS_END
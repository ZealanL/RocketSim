#include "GameEventTracker.h"

RS_NS_START

bool GetShooterPasser(Arena* arena, Team team, Car*& shooterOut, bool findPasser, Car*& passerOut, uint64_t maxShooterTicks, uint64_t maxPasserTicks) {
	shooterOut = passerOut = NULL;
	// TODO: Instead of looping over cars to find who hit it last, use persistent info

	for (Car* car : arena->_cars) {
		if (car->team != team)
			continue;

		if (!car->_internalState.ballHitInfo.isValid)
			continue;

		if (car->_internalState.ballHitInfo.tickCountWhenHit + maxShooterTicks >= arena->tickCount) {
			if (!shooterOut || car->_internalState.ballHitInfo.tickCountWhenHit > shooterOut->_internalState.ballHitInfo.tickCountWhenHit) {
				shooterOut = car;
			}
		}
	}

	if (shooterOut && findPasser) { // Look for passer
		uint64_t shootTick = shooterOut->_internalState.ballHitInfo.tickCountWhenHit;

		// TODO: Repetitive code
		for (Car* car : arena->_cars) {
			if (car->team != team)
				continue;

			if (!car->_internalState.ballHitInfo.isValid)
				continue;

			if (car == shooterOut)
				continue;

			if (car->_internalState.ballHitInfo.tickCountWhenHit + maxPasserTicks >= shootTick) {
				if (!passerOut || car->_internalState.ballHitInfo.tickCountWhenHit > passerOut->_internalState.ballHitInfo.tickCountWhenHit) {
					passerOut = car;
				}
			}
		}
	}

	return shooterOut != NULL;
}

void GameEventTracker::Update(Arena* arena) {
	bool scored = arena->IsBallScored();
	
	float tickrate = arena->GetTickRate();
	uint64_t ballUpdateCount = arena->ball->_internalState.updateCounter;

	if (ballUpdateCount > _lastBallUpdateCount || !autoStateSetDetection) {
		// Game is continuing

		uint64_t deltaTicks = ballUpdateCount - _lastBallUpdateCount;

		// Time since last update
		float deltaTime = deltaTicks * arena->tickTime;

		// Goal event
		if (scored && !_ballScoredLast) {
			Car* shooter;
			Car* passer;
			if (GetShooterPasser(
				arena,
				RS_TEAM_FROM_Y(-arena->ball->_rigidBody.getWorldTransform().m_origin.y()),
				shooter, true, passer,
				config.goalMaxTouchTime * tickrate,
				config.passMaxTouchTime * tickrate
			)) {

				if (_goalCallback.func)
					_goalCallback.func(arena, shooter, passer, _goalCallback.userInfo);
			}
		} else {
			if (!_ballShot) { // Ball is not currently shot

				if (_shotCooldown > 0) {
					_shotCooldown = RS_MAX(_shotCooldown - deltaTime, 0);
					// Can't make a shot yet
				} else {

					float speedSq = (arena->ball->_rigidBody.m_linearVelocity * BT_TO_UU).length2();
					if (speedSq >= config.shotMinSpeed * config.shotMinSpeed) {
						Team goalTeam;
						if (arena->IsBallProbablyGoingIn(config.shotMinScoreTime, config.predScoreExtraMargin, &goalTeam)) {
							Team shooterTeam = RS_OPPOSITE_TEAM(goalTeam);

							uint64_t shotMinTouchDelayTicks = config.shotTouchMinDelay * tickrate;

							Car* shooter;
							Car* passer;
							if (GetShooterPasser(
								arena,
								shooterTeam,
								shooter, true, passer,
								deltaTicks + shotMinTouchDelayTicks,
								config.passMaxTouchTime * tickrate
							)) {

								uint64_t ticksSinceHit = arena->tickCount - shooter->_internalState.ballHitInfo.tickCountWhenHit;
								if (ticksSinceHit >= shotMinTouchDelayTicks) {

									// This is officially now a shot!
									_ballShot = true;
									_ballShotGoalTeam = goalTeam;
									_shotCooldown = config.shotEventCooldown;
									if (_shotCallback.func)
										_shotCallback.func(arena, shooter, passer, _shotCallback.userInfo);
								}
							}
						}
					}
				}
			} else {
				// Ball is currently shot

				bool willScore = arena->IsBallProbablyGoingIn(config.shotMinScoreTime, config.predScoreExtraMargin);
				if (!willScore) {
					// Ball is no longer going in
					// Maybe it missed, or maybe it was saved

					Car* saver;
					Car* _unused;
					if (GetShooterPasser(
						arena,
						_ballShotGoalTeam,
						saver, false, _unused,
						deltaTicks,
						0
					)) {

						// A car from the team the ball has just hit the ball
						// Since it's no longer scoring, this was a save
						if (_saveCallback.func)
							_saveCallback.func(arena, saver, _saveCallback.userInfo);
					} else {
						// It just stopped going in (probably missed)
					}
					
					_ballShot = false;
				}
			}
		}
	} else if (ballUpdateCount == _lastBallUpdateCount) {
		// Skip this update
		return;
	} else {
		// Ball update count decreased
		// Reset persistent info
		ResetPersistentInfo();
	}

	_ballScoredLast = scored;
	_lastBallUpdateCount = ballUpdateCount;
}

void GameEventTracker::ResetPersistentInfo() {
	_ballScoredLast = false;
	_ballShot = false;
	_shotCooldown = 0;

	// _ballShotGoalTeam doesn't need to be reset
}

RS_NS_END
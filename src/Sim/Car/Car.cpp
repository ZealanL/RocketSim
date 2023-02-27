#include "Car.h"
#include "../../RLConst.h"

// Update our internal state from bullet and return it
CarState Car::GetState() {

	btTransform rbTransform = _rigidBody->getWorldTransform();

	_internalState.pos = rbTransform.getOrigin() * BT_TO_UU;

	_internalState.angles = Angle(rbTransform.getBasis());

	_internalState.vel = _rigidBody->getLinearVelocity() * BT_TO_UU;

	_internalState.angVel = _rigidBody->getAngularVelocity();

	return _internalState;
}

// Update our bullet stuff to this new state, replace our internal state with it
void Car::SetState(const CarState& state) {
	btTransform rbTransform = _rigidBody->getWorldTransform();

	rbTransform.setOrigin(state.pos * UU_TO_BT);

	rbTransform.setBasis(state.angles.ToMatrix());

	_rigidBody->setWorldTransform(rbTransform);

	_rigidBody->setLinearVelocity(state.vel * UU_TO_BT);
	_rigidBody->setAngularVelocity(state.angVel);

	_velocityImpulseCache = { 0, 0, 0 };

	_internalState = state;
}

void Car::Demolish() {
	_internalState.isDemoed = true;
	_internalState.demoRespawnTimer = RLConst::DEMO_RESPAWN_TIME;
}

void Car::Respawn() {
	using namespace RLConst;

	CarState newState = CarState();
	
	int spawnPosIndex = RS_RAND(0, CAR_RESPAWN_LOCATION_AMOUNT);
	CarSpawnPos spawnPos = CAR_RESPAWN_LOCATIONS[spawnPosIndex];

	newState.pos = Vec(spawnPos.x, spawnPos.y * (team == Team::BLUE ? 1 : -1), CAR_RESPAWN_Z);
	newState.angles = Angle(spawnPos.yawAng, 0.f, 0.f);

	this->SetState(newState);
}

void Car::_PreTickUpdate(float tickTime) {
	using namespace RLConst;

#ifndef RS_MAX_SPEED
	// Fix inputs
	controls.ClampFix();
#endif

	assert(_bulletVehicle->getNumWheels() == 4);

	{
		if (_internalState.isDemoed) {
			_internalState.demoRespawnTimer = RS_MAX(_internalState.demoRespawnTimer - tickTime, 0);
			if (_internalState.demoRespawnTimer == 0)
				Respawn();
		}
		
		if (_internalState.isDemoed) {
			// Disable rigidbody simulation
			_rigidBody->m_activationState1 = DISABLE_SIMULATION;
			_rigidBody->m_collisionFlags |= btCollisionObject::CF_NO_CONTACT_RESPONSE;

			// Put car far away from anything going on in the arena
			_rigidBody->m_worldTransform.m_origin = Vec(0, 0, -1000);

			// Don't bother updating anything
			return;
		} else {
			// Prevent the car's RB from becoming inactive
			_rigidBody->m_activationState1 = ACTIVE_TAG;
			_rigidBody->m_collisionFlags &= ~btCollisionObject::CF_NO_CONTACT_RESPONSE;
		}
	}

	_bulletVehicle->updateVehicle(tickTime);

	_internalState.worldContact.hasContact = false;

	btMatrix3x3 basis = _rigidBody->getWorldTransform().getBasis();

	Vec
		forwardDir = basis.getColumn(0),
		rightDir = basis.getColumn(1),
		upDir = basis.getColumn(2);

	float forwardSpeed = _bulletVehicle->getForwardSpeed();
	float absForwardSpeed = abs(forwardSpeed);

	bool jumpPressed = controls.jump && !_internalState.lastControls.jump;

	int numWheelsInContact = 0;
	for (int i = 0; i < 4; i++)
		numWheelsInContact += _bulletVehicle->m_wheelInfo[i].m_raycastInfo.m_isInContact;

	{ // Increase/decrease handbrake value from input
		if (controls.handbrake) {
			_internalState.handbrakeVal += POWERSLIDE_RISE_RATE * tickTime;
		} else {
			_internalState.handbrakeVal -= POWERSLIDE_FALL_RATE * tickTime;
		}
		_internalState.handbrakeVal = RS_CLAMP(_internalState.handbrakeVal, 0, 1);
	}

	{ // Update steering
		float absForwardSpeedUU = absForwardSpeed * BT_TO_UU;
		float steerAngle = STEER_ANGLE_FROM_SPEED_CURVE.GetOutput(absForwardSpeedUU);

		if (_internalState.handbrakeVal) {
			steerAngle +=
				(POWERSLIDE_STEER_ANGLE_FROM_SPEED_CURVE.GetOutput(absForwardSpeedUU) - steerAngle)
				* _internalState.handbrakeVal;
		}

		steerAngle *= controls.steer;
		_bulletVehicle->m_wheelInfo[0].m_steerAngle = steerAngle;
		_bulletVehicle->m_wheelInfo[1].m_steerAngle = steerAngle;
	}

	{ // Update friction
		for (int i = 0; i < 4; i++) {
			auto& wheel = _bulletVehicle->m_wheelInfo[i];
			if (wheel.m_raycastInfo.m_groundObject) {

				Vec
					vel = _rigidBody->getLinearVelocity(),
					angularVel = _rigidBody->getAngularVelocity();

				Vec
					latDir = wheel.m_worldTransform.getBasis().getColumn(1),
					longDir = latDir.cross(wheel.m_raycastInfo.m_contactNormalWS);

				float frictionCurveInput = 0;

				Vec wheelDelta = wheel.m_raycastInfo.m_hardPointWS - _rigidBody->getWorldTransform().getOrigin();

				auto crossVec = (angularVel.cross(wheelDelta) + vel) * BT_TO_UU;

				float baseFriction = abs(crossVec.dot(latDir));

				// Significant friction results in lateral slip
				if (baseFriction > 5)
					frictionCurveInput = baseFriction / (abs(crossVec.dot(longDir)) + baseFriction);

				float latFriction = LAT_FRICTION_CURVE.GetOutput(frictionCurveInput);
				float longFriction = LONG_FRICTION_CURVE.GetOutput(frictionCurveInput);

				if (_internalState.handbrakeVal) {
					float handbrakeAmount = _internalState.handbrakeVal;

					latFriction *= (HANDBRAKE_LAT_FRICTION_FACTOR_CURVE.GetOutput(latFriction) - 1) * handbrakeAmount + 1;
					longFriction *= (HANDBRAKE_LONG_FRICTION_FACTOR_CURVE.GetOutput(frictionCurveInput) - 1) * handbrakeAmount + 1;
				} else {
					longFriction = 1; // If we aren't powersliding, it's not scaled down
				}

				bool isContactSticky = controls.throttle != 0;

				if (isContactSticky) {
					// Keep current friction values
				} else {
					// Scale friction down with non-sticky friction curve
					float nonStickyScale = NON_STICKY_FRICTION_FACTOR_CURVE.GetOutput(wheel.m_raycastInfo.m_contactNormalWS.z());
					latFriction *= nonStickyScale;
					longFriction *= nonStickyScale;
				}

				wheel.m_latFriction = latFriction;
				wheel.m_longFriction = longFriction;
			}
		}
	}

	{ // Update boosting timer
		if (_internalState.timeSpentBoosting > 0) {
			if (!controls.boost && _internalState.timeSpentBoosting >= BOOST_MIN_TIME) {
				_internalState.timeSpentBoosting = 0;
			} else {
				_internalState.timeSpentBoosting += tickTime;
			}
		} else {
			if (controls.boost) {
				// Start boosting (even if we dont have any)
				_internalState.timeSpentBoosting = tickTime;
			}
		}
	}

	float realThrottle = controls.throttle;
	float realBrake = 0;

	if (controls.boost && _internalState.boost > 0)
		realThrottle = 1;

	{ // Update throttle/brake forces
		float driveSpeedScale = DRIVE_SPEED_TORQUE_FACTOR_CURVE.GetOutput(absForwardSpeed * BT_TO_UU);

		if (controls.handbrake > 0) {
			// Real throttle is unchanged from the input throttle when powersliding
		} else {
			float absThrottle = abs(realThrottle);

			if (absThrottle >= THROTTLE_DEADZONE) {
				if (absForwardSpeed > 0 && RS_SGN(realThrottle) != RS_SGN(forwardSpeed)) {
					// Full brake is applied if we are trying to drive in the opposite direction
					realBrake = 1;

					if (absForwardSpeed > 0.01f) {
						// Kill actual throttle (we can't throttle and break at the same time, even backwards)
						realThrottle = 0;
					}
				}
			} else {
				// No throttle, we are coasting
				realThrottle = 0;

				// Apply coasting brake, we full-break when coasting very slowly
				bool shouldFullStop = (absForwardSpeed < (STOPPING_FORWARD_VEL* UU_TO_BT));
				realBrake = shouldFullStop ? 1 : COASTING_BRAKE_FACTOR;
			}
		}

		if (numWheelsInContact < 3)
			driveSpeedScale /= 4;

		float driveEngineForce = realThrottle * (THROTTLE_TORQUE_AMOUNT * UU_TO_BT) * driveSpeedScale;
		float driveBrakeForce = realBrake * (BRAKE_TORQUE_AMOUNT * UU_TO_BT);
		for (int i = 0; i < 4; i++) {
			_bulletVehicle->m_wheelInfo[i].m_engineForce = driveEngineForce;
			_bulletVehicle->m_wheelInfo[i].m_brake = driveBrakeForce;
		}
	}

	if (numWheelsInContact >= 3) { // Grounded, apply sticky forces
		Vec upwardsDir = _bulletVehicle->getUpwardsDirFromWheelContacts();

		bool fullStick = (realThrottle != 0) || (absForwardSpeed > 25);

		float stickyForceScale = 0.5f;
		if (fullStick)
			stickyForceScale += 1 - abs(upwardsDir.z());

		_rigidBody->applyCentralForce(upwardsDir * stickyForceScale * (GRAVITY_Z * UU_TO_BT) * CAR_MASS_BT);

	} else { // Not grounded, apply air control
		btMatrix3x3 basis = _rigidBody->getWorldTransform().getBasis();
		btVector3
			dirPitch_right = -rightDir,
			dirYaw_up = upDir,
			dirRoll_forward = -forwardDir;

		bool doAirControl = false;
		if (_internalState.hasFlipped && _internalState.flipTime < FLIP_TORQUE_TIME) {

			btVector3 relDodgeTorque = _internalState.lastRelDodgeTorque;

			if (!_internalState.lastRelDodgeTorque.isZero()) {
				// Flip cancel check
				float pitchScale = 1;
				if (relDodgeTorque.y() != 0 && controls.pitch != 0) {
					if (RS_SGN(relDodgeTorque.y()) == RS_SGN(controls.pitch)) {
						pitchScale = 0;
						doAirControl = true;
					}
				}

				relDodgeTorque.y() *= pitchScale;

				btVector3 dodgeTorque = _rigidBody->getWorldTransform().getBasis() * (relDodgeTorque * btVector3(FLIP_TORQUE_X, FLIP_TORQUE_Y, 0));
				_rigidBody->setAngularVelocity(
					_rigidBody->getAngularVelocity() + dodgeTorque * tickTime
				);
			} else {
				// Stall, allow air control
				doAirControl = true;
			}
		} else {
			doAirControl = true;
		}

		doAirControl &= !_internalState.isAutoFlipping;
		if (doAirControl) {
			// Net torque to apply to the car
			btVector3 torque;

			float pitchTorqueScale = 1;
			if (controls.pitch || controls.yaw || controls.roll) {

				if (_internalState.hasFlipped && _internalState.flipTime < FLIP_PITCHLOCK_TIME)
					pitchTorqueScale = 0;

				// TODO: Use actual dot product operator functions (?)
				torque = (controls.pitch * dirPitch_right * pitchTorqueScale * CAR_AIR_CONTROL_TORQUE.x()) +
					(controls.yaw * dirYaw_up * CAR_AIR_CONTROL_TORQUE.y()) +
					(controls.roll * dirRoll_forward * CAR_AIR_CONTROL_TORQUE.z());
			} else {
				torque = { 0, 0, 0 };
			}

			auto angVel = _rigidBody->getAngularVelocity();

			// TODO: Use actual dot product operator functions (?)
			float
				dampPitch = dirPitch_right.dot(angVel) * CAR_AIR_CONTROL_DAMPING.x() * (1 - abs(doAirControl ? (controls.pitch * pitchTorqueScale) : 0)),
				dampYaw = dirYaw_up.dot(angVel) * CAR_AIR_CONTROL_DAMPING.y() * (1 - abs(doAirControl ? controls.yaw : 0)),
				dampRoll = dirRoll_forward.dot(angVel) * CAR_AIR_CONTROL_DAMPING.z();

			btVector3 damping =
				(dirYaw_up * dampYaw) +
				(dirPitch_right * dampPitch) +
				(dirRoll_forward * dampRoll);

			_rigidBody->setAngularVelocity(
				_rigidBody->getAngularVelocity() + (torque - damping) * CAR_TORQUE_SCALE * tickTime
			);
		}
	}

	if (!_internalState.isOnGround) {
		// Throttle in air
		if (controls.throttle != 0)
			_rigidBody->applyCentralImpulse(forwardDir * controls.throttle * THROTTLE_AIR_FORCE * tickTime);
	}

	// Apply boosting force and consume boost
	if (_internalState.boost > 0 && _internalState.timeSpentBoosting > 0) {
		_internalState.boost = RS_MAX(_internalState.boost - BOOST_USED_PER_SECOND * tickTime, 0);

		float forceScale = 1;
		if (_internalState.isOnGround && forwardSpeed > (BOOST_FORCE_GROUND_DECAY_MIN_VEL * UU_TO_BT))
			forceScale = (1 - BOOST_FORCE_GROUND_DECAY_AMOUNT);

		_rigidBody->applyCentralImpulse(forwardDir * BOOST_FORCE * forceScale * tickTime);
	}
}

void Car::_PostTickUpdate(float tickTime) {

	if (_internalState.isDemoed)
		return;

	int numWheelsInContact = 0;
	for (int i = 0; i < 4; i++)
		numWheelsInContact += _bulletVehicle->m_wheelInfo[i].m_raycastInfo.m_isInContact;

	{ // Update isOnGround
		_internalState.isOnGround = numWheelsInContact >= 3;
	}

	bool jumpPressed = controls.jump && !_internalState.lastControls.jump;

	{ // Update jump
		using namespace RLConst;
		if (_internalState.isOnGround && !_internalState.isJumping) {
			if (_internalState.hasJumped && _internalState.jumpTime < JUMP_MIN_TIME + JUMP_RESET_TIME_PAD) {
				// Don't reset the jump just yet, we might still be leaving the ground
				// This fixes the bug where jump is reset before we actually leave the ground after a minimum-time jump
				// TODO: RL does something similar to this time-pad, but not exactly the same
			} else {
				_internalState.hasJumped = false;
			}
		}

		if (_internalState.isJumping) {
			if (_internalState.jumpTime < JUMP_MIN_TIME || controls.jump && _internalState.jumpTime < JUMP_MAX_TIME) {
				// Continue jump
				_internalState.isJumping = true;
			} else {
				// We can't keep jumping any longer
				_internalState.isJumping = false;
			}
		} else if (_internalState.isOnGround && jumpPressed) {
			// Start jumping
			_internalState.isJumping = true;
			_internalState.jumpTime = 0;
			btVector3 jumpStartForce = _bulletVehicle->getUpVector() * JUMP_IMMEDIATE_FORCE * UU_TO_BT;
			_rigidBody->applyCentralImpulse(jumpStartForce * CAR_MASS_BT);
		}

		if (_internalState.isJumping) {
			_internalState.hasJumped = true;
			_internalState.jumpTime += tickTime;

			// Apply extra long-jump force
			btVector3 extraJumpForce = _bulletVehicle->getUpVector() * JUMP_ACCEL;

			if (_internalState.jumpTime < JUMP_MIN_TIME) {
				extraJumpForce *= 0.75f;
			}

			_rigidBody->applyCentralImpulse(extraJumpForce * CAR_MASS_BT * UU_TO_BT * tickTime);
		}
	}

	{ // Update flip/double jump
		using namespace RLConst;
		if (_internalState.isOnGround) {
			_internalState.hasDoubleJumped = false;
			_internalState.hasFlipped = false;
			_internalState.airTimeSinceJump = false;
		} else {
			if (_internalState.hasJumped && !_internalState.isJumping) {
				_internalState.airTimeSinceJump += tickTime;
			} else {
				_internalState.airTimeSinceJump = 0;
			}

			if (jumpPressed && _internalState.airTimeSinceJump < DOUBLEJUMP_MAX_DELAY) {
				if (!_internalState.hasDoubleJumped && !_internalState.hasFlipped) {
					bool shouldFlip = RS_MAX(RS_MAX(abs(controls.yaw), abs(controls.pitch)), abs(controls.roll)) >= config.dodgeDeadzone;

					if (shouldFlip) {
						// Begin flipping
						_internalState.flipTime = 0;
						_internalState.hasFlipped = true;

						// Apply initial dodge vel and set later dodge vel
						// Replicated based on https://github.com/samuelpmish/RLUtilities/blob/develop/src/simulation/car.cc
						{
							btVector3 forwardDir = _bulletVehicle->getForwardVector();
							float forwardSpeed = forwardDir.dot(_rigidBody->getLinearVelocity()) * BT_TO_UU;
							float forwardSpeedRatio = abs(forwardSpeed) / CAR_MAX_SPEED;

							btVector3 dodgeDir = btVector3(-controls.pitch, controls.yaw + controls.roll, 0);

							if (abs(controls.yaw + controls.roll) < 0.1f && abs(controls.pitch) < 0.1f) {
								dodgeDir = { 0, 0, 0 };
							} else {
								dodgeDir.safeNormalize();
							}

							_internalState.lastRelDodgeTorque = btVector3(-dodgeDir.y(), dodgeDir.x(), 0);

							if (abs(dodgeDir.x()) < 0.1f) dodgeDir.x() = 0;
							if (abs(dodgeDir.y()) < 0.1f) dodgeDir.y() = 0;

							if (!dodgeDir.fuzzyZero()) {
								bool shouldDodgeBackwards;

								if (abs(forwardSpeed) < 100.0f) {
									shouldDodgeBackwards = dodgeDir.x() < 0.0f;
								} else {
									shouldDodgeBackwards = (dodgeDir.x() >= 0.0f) != (forwardSpeed >= 0.0f);
								}

								btVector3 initalDodgeVel = dodgeDir * FLIP_INITIAL_VEL_SCALE;

								float maxSpeedScaleX =
									shouldDodgeBackwards ? FLIP_BACKWARD_IMPULSE_MAX_SPEED_SCALE : FLIP_FORWARD_IMPULSE_MAX_SPEED_SCALE;

								initalDodgeVel.x() *= ((maxSpeedScaleX - 1) * forwardSpeedRatio) + 1.f;
								initalDodgeVel.y() *= ((FLIP_SIDE_IMPULSE_MAX_SPEED_SCALE - 1) * forwardSpeedRatio) + 1.f;

								if (shouldDodgeBackwards)
									initalDodgeVel.x() *= FLIP_BACKWARD_IMPULSE_SCALE_X;

								float forwardAng = atan2f(forwardDir.y(), forwardDir.x());

								btVector3
									xVelDir = { cosf(forwardAng), -sinf(forwardAng), 0.f },
									yVelDir = { sinf(forwardAng), cosf(forwardAng), 0.f };

								btVector3 finalDeltaVel = {
									initalDodgeVel.dot(xVelDir),
									initalDodgeVel.dot(yVelDir),
									0.f
								};

								_rigidBody->applyCentralImpulse(finalDeltaVel * UU_TO_BT * CAR_MASS_BT);
							}
						}

					} else {
						// Double jump, add upwards velocity
						btVector3 jumpStartForce = _bulletVehicle->getUpVector() * JUMP_IMMEDIATE_FORCE * UU_TO_BT;
						_rigidBody->applyCentralImpulse(jumpStartForce * CAR_MASS_BT);

						_internalState.hasDoubleJumped = true;
					}
				}
			}
		}

		if (_internalState.hasFlipped) {
			// Replicated from https://github.com/samuelpmish/RLUtilities/blob/develop/src/mechanics/dodge.cc
			_internalState.flipTime += tickTime;
			if (_internalState.flipTime <= FLIP_TORQUE_TIME) {

				btVector3 vel = _rigidBody->getLinearVelocity();
				if (_internalState.flipTime >= FLIP_Z_DAMP_START && (vel.z() < 0 || _internalState.flipTime < FLIP_Z_DAMP_END)) {
					vel.z() *= powf(1 - FLIP_Z_DAMP_120, tickTime / (1 / 120.f));
					_rigidBody->setLinearVelocity(vel);
				}
			}
		}
	}

	{ // Update auto-flip jump

		auto basis = _rigidBody->getWorldTransform().getBasis();

		using namespace RLConst;
		if (
			jumpPressed &&
			_internalState.worldContact.hasContact &&
			_internalState.worldContact.contactNormal.z() > CAR_AUTOFLIP_NORMZ_THRESH
			) {

			Vec upDir = basis.getColumn(2);

			Angle angles = Angle(basis);
			_internalState.autoFlipTimer = CAR_AUTOFLIP_TIME * (abs(angles.roll) / M_PI);
			_internalState.autoFlipTorqueScale = (angles.roll > 0) ? 1 : -1;
			_internalState.isAutoFlipping = true;

			_rigidBody->applyCentralImpulse(-upDir * CAR_AUTOFLIP_IMPULSE * CAR_MASS_BT * UU_TO_BT);
		}

		if (_internalState.isAutoFlipping) {
			if (_internalState.autoFlipTimer <= 0) {
				_internalState.isAutoFlipping = false;
				_internalState.autoFlipTimer = 0;
			} else {
				Vec forwardDir = basis.getColumn(0);

				_rigidBody->applyTorqueImpulse(forwardDir * CAR_AUTOFLIP_TORQUE * _internalState.autoFlipTorqueScale);

				_internalState.autoFlipTimer -= tickTime;
			}
		}
	}

	// Update auto-roll
	if (controls.throttle && ((numWheelsInContact > 0 && numWheelsInContact < 4) || _internalState.worldContact.hasContact)) {
		
		auto basis = _rigidBody->getWorldTransform().getBasis();

		Vec
			forwardDir = basis.getColumn(0),
			rightDir = basis.getColumn(1),
			upDir = basis.getColumn(2);
		
		Vec groundUpDir;
		if (numWheelsInContact > 0) {
			groundUpDir = _bulletVehicle->getUpwardsDirFromWheelContacts();
		} else {
			groundUpDir = _internalState.worldContact.contactNormal;
		}

		Vec groundDownDir = -groundUpDir;

		Vec
			crossRightDir = groundUpDir.cross(forwardDir),
			crossForwardDir = groundDownDir.cross(crossRightDir);

		float
			rightTorqueFactor = 1 - RS_CLAMP(rightDir.dot(crossRightDir), 0, 1),
			forwardTorqueFactor = 1 - RS_CLAMP(forwardDir.dot(crossForwardDir), 0, 1);

		Vec
			torqueDirRight = forwardDir * (rightDir.dot(groundUpDir) >= 0 ? -1 : 1),
			torqueDirForward = rightDir * (forwardDir.dot(groundUpDir) >= 0 ? -1 : 1);

		Vec torqueRight = torqueDirRight * rightTorqueFactor;
		Vec torqueForward = torqueDirForward * forwardTorqueFactor;

		_rigidBody->m_linearVelocity += groundDownDir * RLConst::CAR_AUTOROLL_FORCE * UU_TO_BT * tickTime;
		_rigidBody->m_angularVelocity += (torqueForward + torqueRight) * RLConst::CAR_AUTOROLL_TORQUE * tickTime;
	}

	{ // Update supersonic
		float speedSquared = (_rigidBody->getLinearVelocity() * BT_TO_UU).length2();

		if (_internalState.isSupersonic && _internalState.supersonicTime < RLConst::SUPERSONIC_MAINTAIN_MAX_TIME) {
			_internalState.isSupersonic =
				(speedSquared >= RLConst::SUPERSONIC_MAINTAIN_MIN_SPEED * RLConst::SUPERSONIC_MAINTAIN_MIN_SPEED);
		} else {
			_internalState.isSupersonic =
				(speedSquared >= RLConst::SUPERSONIC_START_SPEED * RLConst::SUPERSONIC_START_SPEED);
		}

		if (_internalState.isSupersonic) {
			_internalState.supersonicTime += tickTime;
		} else {
			_internalState.supersonicTime = 0;
		}
	}

	// Update car contact cooldown timer
	if (_internalState.carContact.cooldownTimer > 0)
		_internalState.carContact.cooldownTimer = RS_MAX(_internalState.carContact.cooldownTimer - tickTime, 0);

	_internalState.lastControls = controls;
}

void Car::_FinishPhysicsTick() {
	using namespace RLConst;

	if (_internalState.isDemoed)
		return;

	// Add velocity cache
	if (!_velocityImpulseCache.isZero()) {
		_rigidBody->m_linearVelocity += _velocityImpulseCache;
		_velocityImpulseCache = { 0,0,0 };
	}

	{ // Limit velocities
		btVector3
			vel = _rigidBody->m_linearVelocity,
			angVel = _rigidBody->m_angularVelocity;

		if (vel.length2() > (CAR_MAX_SPEED * UU_TO_BT) * (CAR_MAX_SPEED * UU_TO_BT))
			vel = vel.normalized() * (CAR_MAX_SPEED * UU_TO_BT);

		if (angVel.length2() > (CAR_MAX_ANG_SPEED * CAR_MAX_ANG_SPEED))
			angVel = angVel.normalized() * CAR_MAX_ANG_SPEED;

		_rigidBody->m_linearVelocity = vel;
		_rigidBody->m_angularVelocity = angVel;
	}

	{ // Round physics values to match RL
		_rigidBody->m_worldTransform.m_origin =
			Math::RoundVec(_rigidBody->m_worldTransform.m_origin, 0.01 * UU_TO_BT);

		_rigidBody->m_linearVelocity =
			Math::RoundVec(_rigidBody->m_linearVelocity, 0.01 * UU_TO_BT);

		_rigidBody->m_angularVelocity =
			Math::RoundVec(_rigidBody->m_angularVelocity, 0.00001);
	}
}

Car* Car::_AllocateCar() {
	return new Car();
}

void Car::_BulletSetup(btDynamicsWorld* bulletWorld) {
	{ // Set up rigidbody and collision shapes
		_childHitboxShape = new btBoxShape((config.hitboxSize * UU_TO_BT) / 2);
		_compoundShape = new btCompoundShape();

		btTransform hitboxOffsetTransform = btTransform();
		hitboxOffsetTransform.setIdentity();
		hitboxOffsetTransform.setOrigin(config.hitboxPosOffset * UU_TO_BT);
		_compoundShape->addChildShape(hitboxOffsetTransform, _childHitboxShape);

		btVector3 localInertia(0, 0, 0);
		_childHitboxShape->calculateLocalInertia(RLConst::CAR_MASS_BT, localInertia);

		btRigidBody::btRigidBodyConstructionInfo rbInfo
			= btRigidBody::btRigidBodyConstructionInfo(RLConst::CAR_MASS_BT, NULL, _compoundShape, localInertia);

		btTransform carTransform = btTransform();
		carTransform.setIdentity();
		rbInfo.m_startWorldTransform = carTransform;

		_rigidBody = new btRigidBody(rbInfo);
		_rigidBody->setUserIndex(BT_USERINFO_TYPE_CAR);
		_rigidBody->setUserPointer(this);

		_rigidBody->m_collisionFlags |= btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK;

		_rigidBody->m_friction = RLConst::CAR_COLLISION_FRICTION;
		_rigidBody->m_restitution = RLConst::CAR_COLLISION_RESTITUTION;

		// Disable gyroscopic force (shoutout to Allah for this one)
		_rigidBody->m_rigidbodyFlags = 0;
	}

	// Add rigidbody to world
	bulletWorld->addRigidBody(_rigidBody);

	{ // Set up actual vehicle stuff
		_bulletVehicleRaycaster = new btDefaultVehicleRaycaster(bulletWorld);

		btVehicleRL::btVehicleTuning tuning = btVehicleRL::btVehicleTuning();

		_bulletVehicle = new btVehicleRL(tuning, _rigidBody, _bulletVehicleRaycaster, bulletWorld);

		// Match RL with X forward, Y right, Z up
		_bulletVehicle->setCoordinateSystem(1, 2, 0);

		// Set up wheel directions with RL coordinate system
		btVector3 wheelDirectionCS(0, 0, -1), wheelAxleCS(0, -1, 0);

		{ // Set up wheels
			for (int i = 0; i < 4; i++) {
				bool front = i < 2;
				bool left = i % 2;

				float radius = front ? config.frontWheels.wheelRadius : config.backWheels.wheelRadius;
				btVector3 wheelRayStartOffset =
					front ? config.frontWheels.connectionPointOffset : config.backWheels.connectionPointOffset;

				if (left)
					wheelRayStartOffset.y() *= -1;

				float suspensionRestLength =
					front ? config.frontWheels.suspensionRestLength : config.backWheels.suspensionRestLength;

				suspensionRestLength -= RLConst::BTVehicle::MAX_SUSPENSION_TRAVEL;

				_bulletVehicle->addWheel(
					wheelRayStartOffset * UU_TO_BT,
					wheelDirectionCS, wheelAxleCS,
					suspensionRestLength * UU_TO_BT,
					radius * UU_TO_BT, tuning, true);

				{ // Fix wheel info data
					using namespace RLConst::BTVehicle;

					btWheelInfoRL& wheelInfo = _bulletVehicle->m_wheelInfo[i];
					wheelInfo.m_suspensionStiffness = SUSPENSION_STIFFNESS;
					wheelInfo.m_wheelsDampingCompression = WHEELS_DAMPING_COMPRESSION;
					wheelInfo.m_wheelsDampingRelaxation = WHEELS_DAMPING_RELAXATION;
					wheelInfo.m_maxSuspensionTravelCm = (MAX_SUSPENSION_TRAVEL * UU_TO_BT) * 100; // Same for all cars (hopefully)
					wheelInfo.m_maxSuspensionForce = FLT_MAX; // Don't think there's a limit
					wheelInfo.m_bIsFrontWheel = front;
					wheelInfo.m_suspensionForceScale = front ? SUSPENSION_FORCE_SCALE_FRONT : SUSPENSION_FORCE_SCALE_BACK;
				}
			}
		}
	}
}

Car::~Car() {
	// Remove from world
	_bulletVehicle->m_dynamicsWorld->removeCollisionObject(_rigidBody);

	delete _bulletVehicle;
	delete _bulletVehicleRaycaster;
	delete _rigidBody;
	delete _compoundShape;
	delete _childHitboxShape;
}
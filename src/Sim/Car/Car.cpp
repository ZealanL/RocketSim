#include "Car.h"
#include "../../RLConst.h"

// Update our internal state from bullet and return it
CarState Car::GetState() {

	btTransform rbTransform = _rigidBody->getWorldTransform();

	_internalState.pos = rbTransform.getOrigin() * BT_TO_UU;

	rbTransform.getRotation().getEulerZYX(_internalState.angles.yaw, _internalState.angles.pitch, _internalState.angles.roll);

	_internalState.vel = _rigidBody->getLinearVelocity() * BT_TO_UU;

	_internalState.angVel = _rigidBody->getAngularVelocity();

	return _internalState;
}

// Update our bullet stuff to this new state, replace our internal state with it
void Car::SetState(const CarState& state) {
	btTransform rbTransform = _rigidBody->getWorldTransform();

	rbTransform.setOrigin(state.pos * UU_TO_BT);

	btQuaternion quat;
	quat.setEulerZYX(state.angles.yaw, state.angles.pitch, state.angles.roll);
	rbTransform.setRotation(quat);

	_rigidBody->setWorldTransform(rbTransform);

	_rigidBody->setLinearVelocity(state.vel * UU_TO_BT);
	_rigidBody->setAngularVelocity(state.angVel);

	_internalState = state;
}

void Car::_PreTickUpdate(float tickTime) {

#ifndef RS_MAX_SPEED
	// Fix inputs
	controls.ClampFix();
#endif

	assert(_bulletVehicle->getNumWheels() == 4);

	// Prevent the car's RB from becoming inactive
	_rigidBody->setActivationState(ACTIVE_TAG);

	_bulletVehicle->updateVehicle(tickTime);

	Vec forwardDir = _bulletVehicle->getForwardVector();

	float forwardSpeed = _bulletVehicle->getForwardSpeed();
	float absForwardSpeed = abs(forwardSpeed);

	bool jumpPressed = controls.jump && !_internalState.lastControls.jump;

	int numWheelsInContact = 0;
	for (int i = 0; i < 4; i++)
		numWheelsInContact += _bulletVehicle->m_wheelInfo[i].m_raycastInfo.m_isInContact;

	{ // Increase/decrease handbrake value from input
		if (controls.handbrake) {
			_internalState.handbrakeVal += RLConst::POWERSLIDE_RISE_RATE * tickTime;
		} else {
			_internalState.handbrakeVal -= RLConst::POWERSLIDE_FALL_RATE * tickTime;
		}
		_internalState.handbrakeVal = RS_CLAMP(_internalState.handbrakeVal, 0, 1);
	}

	{ // Update steering
		float absForwardSpeedUU = absForwardSpeed * BT_TO_UU;
		float steerAngle = RLConst::STEER_ANGLE_FROM_SPEED_CURVE.GetOutput(absForwardSpeedUU);

		if (_internalState.handbrakeVal) {
			steerAngle +=
				(RLConst::POWERSLIDE_STEER_ANGLE_FROM_SPEED_CURVE.GetOutput(absForwardSpeedUU) - steerAngle)
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

				float latFriction = RLConst::LAT_FRICTION_CURVE.GetOutput(frictionCurveInput);
				float longFriction = RLConst::LONG_FRICTION_CURVE.GetOutput(frictionCurveInput);

				if (_internalState.handbrakeVal) {
					float handbrakeAmount = _internalState.handbrakeVal;

					latFriction *= (RLConst::HANDBRAKE_LAT_FRICTION_FACTOR_CURVE.GetOutput(latFriction) - 1) * handbrakeAmount + 1;
					longFriction *= (RLConst::HANDBRAKE_LONG_FRICTION_FACTOR_CURVE.GetOutput(frictionCurveInput) - 1) * handbrakeAmount + 1;
				} else {
					longFriction = 1; // If we aren't powersliding, it's not scaled down
				}

				bool isContactSticky = controls.throttle != 0;

				if (isContactSticky) {
					// Keep current friction values
				} else {
					// Scale friction down with non-sticky friction curve
					float nonStickyScale = RLConst::NON_STICKY_FRICTION_FACTOR_CURVE.GetOutput(wheel.m_raycastInfo.m_contactNormalWS.z());
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
			if (!controls.boost && _internalState.timeSpentBoosting >= RLConst::BOOST_MIN_TIME) {
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

	if (_internalState.timeSpentBoosting && _internalState.boost > 0)
		realThrottle = 1;

	{ // Update throttle/brake forces
		float driveSpeedScale = RLConst::DRIVE_SPEED_TORQUE_FACTOR_CURVE.GetOutput(absForwardSpeed * BT_TO_UU);

		if (controls.handbrake > 0) {
			// Real throttle is unchanged from the input throttle when powersliding
		} else {
			float absThrottle = abs(realThrottle);

			if (absThrottle >= RLConst::THROTTLE_DEADZONE) {
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
				bool shouldFullStop = (absForwardSpeed < (RLConst::STOPPING_FORWARD_VEL* UU_TO_BT));
				realBrake = shouldFullStop ? 1 : RLConst::COASTING_BRAKE_FACTOR;
			}
		}

		if (numWheelsInContact < 3)
			driveSpeedScale /= 4;

		float driveEngineForce = realThrottle * (RLConst::THROTTLE_TORQUE_AMOUNT * UU_TO_BT) * driveSpeedScale;
		float driveBrakeForce = realBrake * (RLConst::BRAKE_TORQUE_AMOUNT * UU_TO_BT);
		for (int i = 0; i < 4; i++) {
			_bulletVehicle->m_wheelInfo[i].m_engineForce = driveEngineForce;
			_bulletVehicle->m_wheelInfo[i].m_brake = driveBrakeForce;
		}
	}

	if (numWheelsInContact >= 3) { // Grounded, apply sticky forces
		Vec downwardsDir = _bulletVehicle->getDownwardsDirFromWheelContacts();

		bool fullStick = (realThrottle != 0) || (absForwardSpeed > 25);

		float stickyForceScale = 0.5f;
		if (fullStick)
			stickyForceScale += 1 - abs(downwardsDir.z());

		_rigidBody->applyCentralForce(downwardsDir * stickyForceScale * (-RLConst::GRAVITY_Z * UU_TO_BT) * RLConst::CAR_MASS_BT);

	} else { // Not grounded, apply air control
		using namespace RLConst;

		btMatrix3x3 basis = _rigidBody->getWorldTransform().getBasis();
		btVector3
			dirPitch_right = -basis.getColumn(1),
			dirYaw_up = basis.getColumn(2),
			dirRoll_forward = -basis.getColumn(0);

		bool doAirControl = false;
		if (_internalState.hasFlipped && _internalState.flipTimer < FLIP_TORQUE_TIME) {

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

				btVector3 dodgeTorque = _rigidBody->getWorldTransform().getBasis() * (relDodgeTorque * btVector3(FLIP_TORQUE_X, FLIP_TORQUE_Y, 0));;
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

		if (doAirControl) {
			// Net torque to apply to the car
			btVector3 torque;

			float pitchTorqueScale = 1;
			if (controls.pitch || controls.yaw || controls.roll) {

				if (_internalState.hasFlipped && _internalState.flipTimer < FLIP_PITCHLOCK_TIME)
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

		// Throttle in air
		if (controls.throttle != 0)
			_rigidBody->applyCentralImpulse(forwardDir * controls.throttle * RLConst::THROTTLE_AIR_FORCE * tickTime);
	}

	// Apply boosting force and consume boost
	if (_internalState.boost > 0 && _internalState.timeSpentBoosting > 0) {
		_internalState.boost = RS_MAX(_internalState.boost - RLConst::BOOST_USED_PER_SECOND * tickTime, 0);

		_rigidBody->applyCentralImpulse(forwardDir * RLConst::BOOST_FORCE * tickTime);
	}
}

void Car::_ApplyPhysicsRounding() {
	_internalState = GetState();

	_internalState.pos = Math::RoundVec(_internalState.pos, 0.01);
	_internalState.vel = Math::RoundVec(_internalState.vel, 0.01);
	_internalState.angVel = Math::RoundVec(_internalState.angVel, 0.00001);
	
	SetState(_internalState);
}

void Car::_PostTickUpdate(float tickTime) {
	{ // Update isOnGround
		int wheelsWithContact = 0;
		for (int i = 0; i < 4; i++)
			wheelsWithContact += _bulletVehicle->m_wheelInfo[i].m_raycastInfo.m_isInContact;

		_internalState.isOnGround = wheelsWithContact >= 3;
	}

	bool jumpPressed = controls.jump && !_internalState.lastControls.jump;

	{ // Update jump
		using namespace RLConst;
		if (_internalState.isOnGround && !_internalState.isJumping)
			_internalState.hasJumped = false;

		if (_internalState.isJumping) {
			if (_internalState.jumpTimer < JUMP_MIN_TIME || controls.jump && _internalState.jumpTimer < JUMP_MAX_TIME) {
					// Continue jump
				_internalState.isJumping = true;
			} else {
				// We can't keep jumping any longer
				_internalState.isJumping = _internalState.jumpTimer < JUMP_MIN_TIME;
			}
		} else if (_internalState.isOnGround && jumpPressed) {
			// Start jumping
			_internalState.isJumping = true;
			_internalState.jumpTimer = 0;
			btVector3 jumpStartForce = _bulletVehicle->getUpVector() * JUMP_IMMEDIATE_FORCE * UU_TO_BT;
			_rigidBody->applyCentralImpulse(jumpStartForce * CAR_MASS_BT);
		}

		if (_internalState.isJumping) {
			_internalState.hasJumped = true;
			_internalState.jumpTimer += tickTime;

			// Apply extra long-jump force
			btVector3 extraJumpForce = _bulletVehicle->getUpVector() * JUMP_ACCEL;

			if (_internalState.jumpTimer < JUMP_MIN_TIME) {
				extraJumpForce *= 0.75f;
			}

			_rigidBody->applyCentralImpulse(extraJumpForce * CAR_MASS_BT * UU_TO_BT * tickTime);
		} else {
			_internalState.jumpTimer = 0;
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
			
			if (_internalState.hasJumped && jumpPressed && _internalState.airTimeSinceJump < DOUBLEJUMP_MAX_DELAY) {
				if (!_internalState.hasDoubleJumped && !_internalState.hasFlipped) {
					bool shouldFlip = RS_MAX(RS_MAX(abs(controls.yaw), abs(controls.pitch)), abs(controls.roll)) >= config.dodgeDeadzone;

					if (shouldFlip) {
						// Begin flipping
						_internalState.flipTimer = 0;
						_internalState.hasFlipped = true;

						// Apply initial dodge vel and set later dodge vel
						// Replicated based on https://github.com/samuelpmish/RLUtilities/blob/develop/src/simulation/car.cc
						{
							btVector3 forwardDir = _bulletVehicle->getForwardVector();
							float forwardSpeed = forwardDir.dot(_rigidBody->getLinearVelocity()) * BT_TO_UU;
							float forwardSpeedRatio = abs(forwardSpeed) / CAR_MAX_SPEED;

							btVector3 dodgeDir = btVector3(-controls.pitch, controls.yaw + controls.roll, 0).normalize();
							_internalState.lastRelDodgeTorque = btVector3(-dodgeDir.y(), dodgeDir.x(), 0);

							if (abs(dodgeDir.x()) < 0.1f) dodgeDir.x() = 0;
							if (abs(dodgeDir.y()) < 0.1f) dodgeDir.y() = 0;

							if (!dodgeDir.fuzzyZero()) {
								bool shouldDodgeBackwards;

								if (abs(forwardSpeed) < 100.0f)
									shouldDodgeBackwards = dodgeDir.x() < 0.0f;
								else
									shouldDodgeBackwards = (dodgeDir.x() >= 0.0f) != (dodgeDir.x() > 0.0f);

								btVector3 initalDodgeVel = dodgeDir * FLIP_INITIAL_VEL_SCALE;

								if (shouldDodgeBackwards)
									initalDodgeVel.x() *= (16.f / 15.f) * (1.5F * forwardSpeedRatio + 1.f);
								initalDodgeVel.y() *= (0.9f * forwardSpeedRatio) + 1.f;

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
			_internalState.flipTimer += tickTime;
			if (_internalState.flipTimer <= FLIP_TORQUE_TIME) {

				btVector3 vel = _rigidBody->getLinearVelocity();
				if (_internalState.flipTimer >= FLIP_Z_DAMP_START && (vel.z() < 0 || _internalState.flipTimer < FLIP_Z_DAMP_END)) {
					vel.z() *= powf(1 - FLIP_Z_DAMP_120, tickTime / (1 / 120.f));
					_rigidBody->setLinearVelocity(vel);
				}
				
			}
			
		}
	}
	_internalState.lastControls = controls;

	{ // Limit velocities
		using namespace RLConst;
		btVector3 
			vel = _rigidBody->getLinearVelocity(), 
			angVel = _rigidBody->getAngularVelocity();

		if (vel.length2() > (CAR_MAX_SPEED * UU_TO_BT) * (CAR_MAX_SPEED * UU_TO_BT))
			vel = vel.normalize() * (CAR_MAX_SPEED * UU_TO_BT);
		_rigidBody->setLinearVelocity(vel);

		angVel = angVel / RS_MAX(1, angVel.length() / CAR_MAX_ANG_SPEED);
		_rigidBody->setAngularVelocity(angVel);
	}
}

Car* Car::_AllocateCar() {
	return new Car();
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
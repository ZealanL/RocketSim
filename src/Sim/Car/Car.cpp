#include "Car.h"
#include "../../RLConst.h"
#include "../SuspensionCollisionGrid/SuspensionCollisionGrid.h"

#include "../../../libsrc/bullet3-3.24/BulletDynamics/Dynamics/btDynamicsWorld.h"

RS_NS_START

// Update our internal state from bullet and return it
CarState Car::GetState() {
	_internalState.pos = _rigidBody.getWorldTransform().m_origin * BT_TO_UU;

	// NOTE: rotMat already updated at the start of Car::_PostTickUpdate()

	_internalState.vel = _rigidBody.m_linearVelocity * BT_TO_UU;

	_internalState.angVel = _rigidBody.m_angularVelocity;

	return _internalState;
}

// Update our bullet stuff to this new state, replace our internal state with it
void Car::SetState(const CarState& state) {
	btTransform rbTransform;
	rbTransform.setOrigin(state.pos * UU_TO_BT);
	rbTransform.setBasis(state.rotMat);

	_rigidBody.getWorldTransform() = rbTransform;

	_rigidBody.m_linearVelocity = state.vel * UU_TO_BT;
	_rigidBody.m_angularVelocity = state.angVel;

	_velocityImpulseCache = { 0, 0, 0 };

	_internalState = state;
	_internalState.updateCounter = 0;
}

void Car::Demolish(float respawnDelay) {
	_internalState.isDemoed = true;
	_internalState.demoRespawnTimer = respawnDelay;
}

void Car::Respawn(GameMode gameMode, int seed, float boostAmount) {
	using namespace RLConst;

	CarState newState = CarState();

	int spawnPosIndex = Math::RandInt(0, CAR_RESPAWN_LOCATION_AMOUNT, seed);
	CarSpawnPos spawnPos = ((gameMode == GameMode::HOOPS) ? CAR_RESPAWN_LOCATIONS_HOOPS : CAR_RESPAWN_LOCATIONS_SOCCAR)[spawnPosIndex];

	newState.pos = Vec(spawnPos.x, spawnPos.y * (team == Team::BLUE ? 1 : -1), CAR_RESPAWN_Z);
	newState.rotMat = Angle(spawnPos.yawAng + (team == Team::BLUE ? 0 : M_PI), 0.f, 0.f).ToRotMat();

	newState.boost = boostAmount;
	this->SetState(newState);
}

void Car::_PreTickUpdate(GameMode gameMode, float tickTime, const MutatorConfig& mutatorConfig, SuspensionCollisionGrid* grid) {
	using namespace RLConst;

#ifndef RS_MAX_SPEED
	// Fix inputs
	controls.ClampFix();
#endif

	assert(_bulletVehicle.getNumWheels() == 4);

	{ // Update simulation state
		if (_internalState.isDemoed) {
			_internalState.demoRespawnTimer = RS_MAX(_internalState.demoRespawnTimer - tickTime, 0);
			if (_internalState.demoRespawnTimer == 0)
				Respawn(gameMode, -1, mutatorConfig.carSpawnBoostAmount);

			// Disable rigidbody simulation
			_rigidBody.m_activationState1 = DISABLE_SIMULATION;
			_rigidBody.m_collisionFlags |= btCollisionObject::CF_NO_CONTACT_RESPONSE;

			// Don't bother updating anything
		} else {
			// Prevent the car's RB from becoming inactive
			_rigidBody.m_activationState1 = ACTIVE_TAG;
			_rigidBody.m_collisionFlags &= ~btCollisionObject::CF_NO_CONTACT_RESPONSE;
		}
	}

	if (_internalState.isDemoed)
		return; // No other updates need to occur

	// Do first part of the btVehicleRL update (update wheel transforms, do traces, calculate friction impulses) 
	_bulletVehicle.updateVehicleFirst(tickTime, grid);


	btMatrix3x3 basis = _rigidBody.getWorldTransform().m_basis;

	bool jumpPressed = controls.jump && !_internalState.lastControls.jump;

	int numWheelsInContact = 0;
	for (int i = 0; i < 4; i++)
		numWheelsInContact += _bulletVehicle.m_wheelInfo[i].m_raycastInfo.m_isInContact;

	float forwardSpeed_UU = _bulletVehicle.getForwardSpeed() * BT_TO_UU;
	_UpdateWheels(tickTime, mutatorConfig, numWheelsInContact, forwardSpeed_UU);

	if (numWheelsInContact < 3) {
		_UpdateAirTorque(tickTime, mutatorConfig, numWheelsInContact == 0);
	} else {
		_internalState.isFlipping = false;
	}

	_UpdateJump(tickTime, mutatorConfig, jumpPressed);
	_UpdateAutoFlip(tickTime, mutatorConfig, jumpPressed);

	_UpdateDoubleJumpOrFlip(tickTime, mutatorConfig, jumpPressed, forwardSpeed_UU);

	if (controls.throttle && ((numWheelsInContact > 0 && numWheelsInContact < 4) || _internalState.worldContact.hasContact))
		_UpdateAutoRoll(tickTime, mutatorConfig, numWheelsInContact);

	_internalState.worldContact.hasContact = false;

	// Complete the btVehicleRL update (does suspension and applies wheel forces)
	_bulletVehicle.updateVehicleSecond(tickTime);

	_UpdateBoost(tickTime, mutatorConfig, forwardSpeed_UU);
}

void Car::_PostTickUpdate(GameMode gameMode, float tickTime, const MutatorConfig& mutatorConfig) {

	if (_internalState.isDemoed)
		return;

	_internalState.rotMat = _rigidBody.getWorldTransform().m_basis;

	// Update wheelsWithContact
	int numWheelsInContact = 0;
	for (int i = 0; i < 4; i++) {
		bool inContact = _bulletVehicle.m_wheelInfo[i].m_raycastInfo.m_isInContact;
		_internalState.wheelsWithContact[i] = inContact;
		numWheelsInContact += inContact;
	}

	{ // Update isOnGround
		_internalState.isOnGround = numWheelsInContact >= 3;
	}

	{ // Update supersonic
		float speedSquared = (_rigidBody.m_linearVelocity * BT_TO_UU).length2();

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

void Car::_FinishPhysicsTick(const MutatorConfig& mutatorConfig) {
	using namespace RLConst;

	if (_internalState.isDemoed)
		return;

	// Add velocity cache
	if (!_velocityImpulseCache.IsZero()) {
		_rigidBody.m_linearVelocity += _velocityImpulseCache;
		_velocityImpulseCache = { 0,0,0 };
	}

	{ // Limit velocities
		btVector3
			vel = _rigidBody.m_linearVelocity,
			angVel = _rigidBody.m_angularVelocity;

		if (vel.length2() > (CAR_MAX_SPEED * UU_TO_BT) * (CAR_MAX_SPEED * UU_TO_BT))
			vel = vel.normalized() * (CAR_MAX_SPEED * UU_TO_BT);

		if (angVel.length2() > (CAR_MAX_ANG_SPEED * CAR_MAX_ANG_SPEED))
			angVel = angVel.normalized() * CAR_MAX_ANG_SPEED;

		_rigidBody.m_linearVelocity = vel;
		_rigidBody.m_angularVelocity = angVel;
	}

	_internalState.updateCounter++;
}

void Car::_BulletSetup(GameMode gameMode, btDynamicsWorld* bulletWorld, const MutatorConfig& mutatorConfig) {
	// Set up rigidbody and collision shapes
	_childHitboxShape = btBoxShape((config.hitboxSize * UU_TO_BT) / 2);
	_compoundShape = btCompoundShape(false, 1);

	btTransform hitboxOffsetTransform = btTransform();
	hitboxOffsetTransform.setIdentity();
	hitboxOffsetTransform.setOrigin(config.hitboxPosOffset * UU_TO_BT);
	_compoundShape.addChildShape(hitboxOffsetTransform, &_childHitboxShape);

	btVector3 localInertia(0, 0, 0);
	_childHitboxShape.calculateLocalInertia(RLConst::CAR_MASS_BT, localInertia);

	btRigidBody::btRigidBodyConstructionInfo rbInfo
		= btRigidBody::btRigidBodyConstructionInfo(RLConst::CAR_MASS_BT, NULL, &_compoundShape, localInertia);

	btTransform carTransform = btTransform();
	carTransform.setIdentity();
	rbInfo.m_startWorldTransform = carTransform;

	_rigidBody = btRigidBody(rbInfo);
	_rigidBody.setUserIndex(BT_USERINFO_TYPE_CAR);
	_rigidBody.setUserPointer(this);

	_rigidBody.m_collisionFlags |= btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK;

	_rigidBody.m_friction = RLConst::CAR_COLLISION_FRICTION;
	_rigidBody.m_restitution = RLConst::CAR_COLLISION_RESTITUTION;

	// Disable gyroscopic force
	_rigidBody.m_rigidbodyFlags = 0;

	// Add rigidbody to world
	bulletWorld->addRigidBody(&_rigidBody);

	{ // Set up actual vehicle stuff
		_bulletVehicleRaycaster = btDefaultVehicleRaycaster(bulletWorld);

		btVehicleRL::btVehicleTuning tuning = btVehicleRL::btVehicleTuning();

		_bulletVehicle = btVehicleRL(tuning, &_rigidBody, &_bulletVehicleRaycaster, bulletWorld);

		// Match RL with X forward, Y right, Z up
		_bulletVehicle.setCoordinateSystem(1, 2, 0);

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

				_bulletVehicle.addWheel(
					wheelRayStartOffset * UU_TO_BT,
					wheelDirectionCS, wheelAxleCS,
					suspensionRestLength * UU_TO_BT,
					radius * UU_TO_BT, tuning, true);

				{ // Fix wheel info data
					using namespace RLConst::BTVehicle;

					btWheelInfoRL& wheelInfo = _bulletVehicle.m_wheelInfo[i];
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

	_internalState.boost = mutatorConfig.carSpawnBoostAmount;
}

bool CarState::HasFlipOrJump() const {
	return 
		isOnGround || 
		(!hasFlipped && !hasDoubleJumped && airTimeSinceJump < RLConst::DOUBLEJUMP_MAX_DELAY);
}

bool CarState::HasFlipReset() const {
	return !isOnGround && HasFlipOrJump() && !hasJumped;
}

bool CarState::GotFlipReset() const {
	return !isOnGround && !hasJumped;
}

void CarState::Serialize(DataStreamOut& out) const {
	ballHitInfo.Serialize(out);

	out.WriteMultiple(
		CARSTATE_SERIALIZATION_FIELDS
	);
}

void CarState::Deserialize(DataStreamIn& in) {

	ballHitInfo.Deserialize(in);

	in.ReadMultiple(
		CARSTATE_SERIALIZATION_FIELDS
	);
}

void Car::Serialize(DataStreamOut& out) {
	out.WriteMultiple(CAR_CONTROLS_SERIALIZATION_FIELDS(controls));
	out.WriteMultiple(CAR_CONFIG_SERIALIZATION_FIELDS(config));
	GetState().Serialize(out);
}

void Car::_Deserialize(DataStreamIn& in) {
	in.ReadMultiple(CAR_CONTROLS_SERIALIZATION_FIELDS(controls));
	in.ReadMultiple(CAR_CONFIG_SERIALIZATION_FIELDS(config));
	CarState newState;
	newState.Deserialize(in);
	_internalState = newState;
}

void Car::_UpdateWheels(float tickTime, const MutatorConfig& mutatorConfig, int numWheelsInContact, float forwardSpeed_UU) {
	using namespace RLConst;

	float absForwardSpeed_UU = abs(forwardSpeed_UU);

	bool wheelsHaveWorldContact = false;
	for (int i = 0; i < 4; i++)
		wheelsHaveWorldContact |= _bulletVehicle.m_wheelInfo[i].m_isInContactWithWorld;

	{ // Increase/decrease handbrake value from input
		if (controls.handbrake) {
			_internalState.handbrakeVal += POWERSLIDE_RISE_RATE * tickTime;
		} else {
			_internalState.handbrakeVal -= POWERSLIDE_FALL_RATE * tickTime;
		}
		_internalState.handbrakeVal = RS_CLAMP(_internalState.handbrakeVal, 0, 1);
	}

	float realThrottle = controls.throttle;
	float realBrake = 0;

	if (controls.boost && _internalState.boost > 0)
		realThrottle = 1;

	{ // Update throttle/brake forces
		float driveSpeedScale = DRIVE_SPEED_TORQUE_FACTOR_CURVE.GetOutput(absForwardSpeed_UU);

		float engineThrottle = realThrottle;

		if (controls.handbrake) {
			// Real throttle is unchanged from the input throttle when powersliding
		} else {
			float absThrottle = abs(realThrottle);

			if (absThrottle >= THROTTLE_DEADZONE) {
				if (absForwardSpeed_UU > STOPPING_FORWARD_VEL && RS_SGN(realThrottle) != RS_SGN(forwardSpeed_UU)) {
					// Full brake is applied if we are trying to drive in the opposite direction
					realBrake = 1;

					if (absForwardSpeed_UU > 0.01f) {
						// Kill actual throttle (we can't throttle and brake at the same time, even backwards)
						engineThrottle = 0;
					}
				}
			} else {
				// No throttle, we are coasting
				engineThrottle = 0;

				// Apply coasting brake, we full-break when coasting very slowly
				bool shouldFullStop = (absForwardSpeed_UU < STOPPING_FORWARD_VEL);
				realBrake = shouldFullStop ? 1 : COASTING_BRAKE_FACTOR;
			}
		}

		if (numWheelsInContact < 3)
			driveSpeedScale /= 4;

		float driveEngineForce = engineThrottle * (THROTTLE_TORQUE_AMOUNT * UU_TO_BT) * driveSpeedScale;
		float driveBrakeForce = realBrake * (BRAKE_TORQUE_AMOUNT * UU_TO_BT);
		for (int i = 0; i < 4; i++) {
			_bulletVehicle.m_wheelInfo[i].m_engineForce = driveEngineForce;
			_bulletVehicle.m_wheelInfo[i].m_brake = driveBrakeForce;
		}
	}

	{ // Update steering
		float steerAngle = STEER_ANGLE_FROM_SPEED_CURVE.GetOutput(absForwardSpeed_UU);

		if (_internalState.handbrakeVal) {
			steerAngle +=
				(POWERSLIDE_STEER_ANGLE_FROM_SPEED_CURVE.GetOutput(absForwardSpeed_UU) - steerAngle)
				* _internalState.handbrakeVal;
		}

		steerAngle *= controls.steer;
		_bulletVehicle.m_wheelInfo[0].m_steerAngle = steerAngle;
		_bulletVehicle.m_wheelInfo[1].m_steerAngle = steerAngle;
	}

	{ // Update friction
		for (int i = 0; i < 4; i++) {
			auto& wheel = _bulletVehicle.m_wheelInfo[i];
			if (wheel.m_raycastInfo.m_groundObject) {

				btVector3
					vel = _rigidBody.m_linearVelocity,
					angularVel = _rigidBody.m_angularVelocity;

				btVector3
					latDir = wheel.m_worldTransform.getBasis().getColumn(1),
					longDir = latDir.cross(wheel.m_raycastInfo.m_contactNormalWS);

				float frictionCurveInput = 0;

				btVector3 wheelDelta = wheel.m_raycastInfo.m_hardPointWS - _rigidBody.getWorldTransform().m_origin;

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

				bool isContactSticky = realThrottle != 0;

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

	// Update sticky forces if at least 1 wheel has world contact
	if (wheelsHaveWorldContact) {
		btVector3 upwardsDir = _bulletVehicle.getUpwardsDirFromWheelContacts();

		bool fullStick = (realThrottle != 0) || (absForwardSpeed_UU > STOPPING_FORWARD_VEL);

		float stickyForceScale = 0.5f;
		if (fullStick)
			stickyForceScale += 1 - abs(upwardsDir.z());

		_rigidBody.applyCentralForce(upwardsDir * stickyForceScale * (GRAVITY_Z * UU_TO_BT) * CAR_MASS_BT);
	}
}

void Car::_UpdateBoost(float tickTime, const MutatorConfig& mutatorConfig, float forwardSpeed_UU) {
	using namespace RLConst;

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

	// Apply boosting force and consume boost
	if (_internalState.boost > 0 && _internalState.timeSpentBoosting > 0) {
		_internalState.boost = RS_MAX(_internalState.boost - mutatorConfig.boostUsedPerSecond * tickTime, 0);

		float forceScale = 1;
		if (_internalState.isOnGround && forwardSpeed_UU > BOOST_ACCEL_GROUND_DECAY_MIN_VEL)
			forceScale = (1 - BOOST_ACCEL_GROUND_DECAY_AMOUNT);

		_rigidBody.applyCentralForce(GetForwardDir() * mutatorConfig.boostAccel * forceScale * CAR_MASS_BT);
	}
}

void Car::_UpdateJump(float tickTime, const MutatorConfig& mutatorConfig, bool jumpPressed) {
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
		btVector3 jumpStartForce = GetUpDir() * mutatorConfig.jumpImmediateForce * UU_TO_BT * CAR_MASS_BT;
		_rigidBody.applyCentralImpulse(jumpStartForce);
	}

	if (_internalState.isJumping) {
		_internalState.hasJumped = true;

		// Apply extra long-jump force
		btVector3 totalJumpForce = GetUpDir() * mutatorConfig.jumpAccel;

		if (_internalState.jumpTime < JUMP_MIN_TIME) {
			// TODO: Either move to RLConst or preferably don't use this system at all
			constexpr float JUMP_PRE_MIN_ACCEL_SCALE = 0.62f;
			totalJumpForce *= JUMP_PRE_MIN_ACCEL_SCALE;
		}

		_rigidBody.applyCentralForce(totalJumpForce * UU_TO_BT * CAR_MASS_BT);
		_internalState.jumpTime += tickTime;
	}
}

void Car::_UpdateAirTorque(float tickTime, const MutatorConfig& mutatorConfig, bool updateAirControl) {
	using namespace RLConst;

	btVector3
		dirPitch_right = -GetRightDir(),
		dirYaw_up = GetUpDir(),
		dirRoll_forward = -GetForwardDir();

	bool doAirControl = false;
	if (_internalState.isFlipping)
		_internalState.isFlipping = _internalState.hasFlipped && _internalState.flipTime < FLIP_TORQUE_TIME;

	if (_internalState.isFlipping) {

		btVector3 relDodgeTorque = _internalState.flipRelTorque;

		if (!_internalState.flipRelTorque.IsZero()) {
			// Flip cancel check
			float pitchScale = 1;
			if (relDodgeTorque.y() != 0 && controls.pitch != 0) {
				if (RS_SGN(relDodgeTorque.y()) == RS_SGN(controls.pitch)) {

#ifndef RS_MAX_SPEED
					pitchScale = 1 - RS_MIN(abs(controls.pitch), 1); // Sanity clamp
#else
					pitchScale = 1 - abs(controls.pitch); // No sanity check
#endif
					doAirControl = true;
				}
			}

			relDodgeTorque.y() *= pitchScale;

			btVector3 dodgeTorque = relDodgeTorque * btVector3(FLIP_TORQUE_X, FLIP_TORQUE_Y, 0);
			_rigidBody.applyTorque(_rigidBody.m_invInertiaTensorWorld.inverse() * _rigidBody.getWorldTransform().m_basis * dodgeTorque);
		} else {
			// Stall, allow air control
			doAirControl = true;
		}
	} else {
		doAirControl = true;
	}

	doAirControl &= !_internalState.isAutoFlipping;
	doAirControl &= updateAirControl;
	if (doAirControl) {

		float pitchTorqueScale = 1;

		btVector3 torque;
		if (controls.pitch || controls.yaw || controls.roll) {

			if (_internalState.isFlipping) {
				pitchTorqueScale = 0;
			} else if (_internalState.hasFlipped) {
				// Extra pitch lock after flip has finished
				if (_internalState.flipTime < FLIP_TORQUE_TIME + FLIP_PITCHLOCK_EXTRA_TIME)
					pitchTorqueScale = 0;
			}

			// TODO: Use actual dot product operator functions (?)
			torque = (controls.pitch * dirPitch_right * pitchTorqueScale * CAR_AIR_CONTROL_TORQUE.x) +
				(controls.yaw * dirYaw_up * CAR_AIR_CONTROL_TORQUE.y) +
				(controls.roll * dirRoll_forward * CAR_AIR_CONTROL_TORQUE.z);
		} else {
			torque = { 0, 0, 0 };
		}

		auto angVel = _rigidBody.m_angularVelocity;

		// TODO: Use actual dot product operator functions (?)
		float
			dampPitch = dirPitch_right.dot(angVel) * CAR_AIR_CONTROL_DAMPING.x * (1 - abs(doAirControl ? (controls.pitch * pitchTorqueScale) : 0)),
			dampYaw = dirYaw_up.dot(angVel) * CAR_AIR_CONTROL_DAMPING.y * (1 - abs(doAirControl ? controls.yaw : 0)),
			dampRoll = dirRoll_forward.dot(angVel) * CAR_AIR_CONTROL_DAMPING.z;

		btVector3 damping =
			(dirYaw_up * dampYaw) +
			(dirPitch_right * dampPitch) +
			(dirRoll_forward * dampRoll);
		_rigidBody.applyTorque(_rigidBody.m_invInertiaTensorWorld.inverse() * (torque - damping) * CAR_TORQUE_SCALE);
	}

	if (controls.throttle != 0)
		_rigidBody.applyCentralForce(GetForwardDir() * controls.throttle * THROTTLE_AIR_FORCE * CAR_MASS_BT);
}

void Car::_UpdateDoubleJumpOrFlip(float tickTime, const MutatorConfig& mutatorConfig, bool jumpPressed, float forwardSpeed_UU) {
	using namespace RLConst;

	if (_internalState.isOnGround) {
		_internalState.hasDoubleJumped = false;
		_internalState.hasFlipped = false;
		_internalState.airTime = 0;
		_internalState.airTimeSinceJump = 0;
		_internalState.flipTime = 0;
	} else {
		_internalState.airTime += tickTime;

		if (_internalState.hasJumped && !_internalState.isJumping) {
			_internalState.airTimeSinceJump += tickTime;
		} else {
			_internalState.airTimeSinceJump = 0;
		}

		if (jumpPressed && _internalState.airTimeSinceJump < DOUBLEJUMP_MAX_DELAY) {
			float inputMagnitude = abs(controls.yaw) + abs(controls.pitch) + abs(controls.roll);
			bool isFlipInput = inputMagnitude >= config.dodgeDeadzone;

			bool canUse;

			if (isFlipInput) {
				canUse = (!_internalState.hasDoubleJumped && !_internalState.hasFlipped) || mutatorConfig.unlimitedFlips;
			} else {
				canUse = (!_internalState.hasDoubleJumped && !_internalState.hasFlipped) || mutatorConfig.unlimitedDoubleJumps;
			}

			if (_internalState.isAutoFlipping)
				canUse = false;

			if (canUse) {
				if (isFlipInput) {
					// Begin flipping
					_internalState.flipTime = 0;
					_internalState.hasFlipped = true;
					_internalState.isFlipping = true;

					// Apply initial dodge vel and set later dodge vel
					// Replicated based on https://github.com/samuelpmish/RLUtilities/blob/develop/src/simulation/car.cc
					{
						float forwardSpeedRatio = abs(forwardSpeed_UU) / CAR_MAX_SPEED;

						btVector3 dodgeDir = btVector3(-controls.pitch, controls.yaw + controls.roll, 0);

						if (abs(controls.yaw + controls.roll) < 0.1f && abs(controls.pitch) < 0.1f) {
							dodgeDir = { 0, 0, 0 };
						} else {
							dodgeDir = dodgeDir.safeNormalized();
						}

						_internalState.flipRelTorque = btVector3(-dodgeDir.y(), dodgeDir.x(), 0);

						if (abs(dodgeDir.x()) < 0.1f) dodgeDir.x() = 0;
						if (abs(dodgeDir.y()) < 0.1f) dodgeDir.y() = 0;

						if (!dodgeDir.fuzzyZero()) {
							bool shouldDodgeBackwards;

							if (abs(forwardSpeed_UU) < 100.0f) {
								shouldDodgeBackwards = dodgeDir.x() < 0.0f;
							} else {
								shouldDodgeBackwards = (dodgeDir.x() >= 0.0f) != (forwardSpeed_UU >= 0.0f);
							}

							btVector3 initalDodgeVel = dodgeDir * FLIP_INITIAL_VEL_SCALE;

							float maxSpeedScaleX =
								shouldDodgeBackwards ? FLIP_BACKWARD_IMPULSE_MAX_SPEED_SCALE : FLIP_FORWARD_IMPULSE_MAX_SPEED_SCALE;

							initalDodgeVel.x() *= ((maxSpeedScaleX - 1) * forwardSpeedRatio) + 1.f;
							initalDodgeVel.y() *= ((FLIP_SIDE_IMPULSE_MAX_SPEED_SCALE - 1) * forwardSpeedRatio) + 1.f;

							if (shouldDodgeBackwards)
								initalDodgeVel.x() *= FLIP_BACKWARD_IMPULSE_SCALE_X;

							btVector3 forwardDir = GetForwardDir();
							float forwardAng = atan2f(forwardDir.y(), forwardDir.x());

							btVector3
								xVelDir = { cosf(forwardAng), -sinf(forwardAng), 0.f },
								yVelDir = { sinf(forwardAng), cosf(forwardAng), 0.f };

							btVector3 finalDeltaVel = {
								initalDodgeVel.dot(xVelDir),
								initalDodgeVel.dot(yVelDir),
								0.f
							};

							_rigidBody.applyCentralImpulse(finalDeltaVel * UU_TO_BT * CAR_MASS_BT);
						}
					}

				} else {
					// Double jump, add upwards velocity
					btVector3 jumpStartForce = GetUpDir() * JUMP_IMMEDIATE_FORCE * UU_TO_BT * CAR_MASS_BT;
					_rigidBody.applyCentralImpulse(jumpStartForce);

					_internalState.hasDoubleJumped = true;
				}
			}
		}
	}

	if (_internalState.isFlipping) {
		_internalState.flipTime += tickTime;
		if (_internalState.flipTime <= FLIP_TORQUE_TIME) {
			if (_internalState.flipTime >= FLIP_Z_DAMP_START && (_rigidBody.m_linearVelocity.z() < 0 || _internalState.flipTime < FLIP_Z_DAMP_END)) {
				_rigidBody.m_linearVelocity.z() *= powf(1 - FLIP_Z_DAMP_120, tickTime / (1 / 120.f));
			}
		}
	} else if (_internalState.hasFlipped) {
		// Increment flip time even after we are done flipping
		// We need to count the time after for FLIP_PITCHLOCK_EXTRA_TIME to work
		_internalState.flipTime += tickTime;
	}
}

void Car::_UpdateAutoFlip(float tickTime, const MutatorConfig& mutatorConfig, bool jumpPressed) {
	using namespace RLConst;

	// TODO: Improve accuracy

	if (
		jumpPressed &&
		_internalState.worldContact.hasContact &&
		_internalState.worldContact.contactNormal.z > CAR_AUTOFLIP_NORMZ_THRESH
		) {

		// TODO: Slow :(
		Angle angles = Angle::FromRotMat(_internalState.rotMat);

		float absRoll = abs(angles.roll);
		if (absRoll > CAR_AUTOFLIP_ROLL_THRESH) {
			_internalState.autoFlipTimer = CAR_AUTOFLIP_TIME * (absRoll / M_PI);
			_internalState.autoFlipTorqueScale = (angles.roll > 0) ? 1 : -1;
			_internalState.isAutoFlipping = true;

			_rigidBody.applyCentralImpulse(-GetUpDir() * CAR_AUTOFLIP_IMPULSE * UU_TO_BT * CAR_MASS_BT);
		}
	}

	if (_internalState.isAutoFlipping) {
		if (_internalState.autoFlipTimer <= 0) {
			_internalState.isAutoFlipping = false;
			_internalState.autoFlipTimer = 0;
		} else {
			_rigidBody.m_angularVelocity += GetForwardDir() * CAR_AUTOFLIP_TORQUE * _internalState.autoFlipTorqueScale * tickTime;

			_internalState.autoFlipTimer -= tickTime;
		}
	}
}

void Car::_UpdateAutoRoll(float tickTime, const MutatorConfig& mutatorConfig, int numWheelsInContact) {
	using namespace RLConst;

	btVector3 groundUpDir;
	if (numWheelsInContact > 0) {
		groundUpDir = _bulletVehicle.getUpwardsDirFromWheelContacts();
	} else {
		groundUpDir = _internalState.worldContact.contactNormal;
	}

	btVector3 groundDownDir = -groundUpDir;

	btVector3
		forwardDir = GetForwardDir(),
		rightDir = GetRightDir(),
		upDir = GetUpDir();
	
	btVector3
		crossRightDir = groundUpDir.cross(forwardDir),
		crossForwardDir = groundDownDir.cross(crossRightDir);

	float
		rightTorqueFactor = 1 - RS_CLAMP(rightDir.dot(crossRightDir), 0, 1),
		forwardTorqueFactor = 1 - RS_CLAMP(forwardDir.dot(crossForwardDir), 0, 1);

	Vec
		torqueDirRight = forwardDir * (rightDir.dot(groundUpDir) >= 0 ? -1 : 1),
		torqueDirForward = rightDir * (forwardDir.dot(groundUpDir) >= 0 ? 1 : -1);

	Vec torqueRight = torqueDirRight * rightTorqueFactor;
	Vec torqueForward = torqueDirForward * forwardTorqueFactor;

	_rigidBody.applyCentralForce(groundDownDir * RLConst::CAR_AUTOROLL_FORCE * UU_TO_BT * CAR_MASS_BT);
	_rigidBody.applyTorque(_rigidBody.m_invInertiaTensorWorld.inverse() * (torqueForward + torqueRight) * RLConst::CAR_AUTOROLL_TORQUE);
}

RS_NS_END
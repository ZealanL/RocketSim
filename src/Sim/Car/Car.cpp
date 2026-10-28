#include "Car.h"
#include "../../RLConst.h"

#include "../../../libsrc/bullet3-3.24/BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "../CollisionMasks.h"

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
	_rigidBody.updateInertiaTensor();

	_velocityImpulseCache = { 0, 0, 0 };

	_internalState = state;
	_internalState.tickCountSinceUpdate = 0;
}

void Car::Demolish(float respawnDelay) {
	_internalState.isDemoed = true;
	_internalState.demoRespawnTimer = respawnDelay;
}

void Car::Respawn(GameMode gameMode, int seed, float boostAmount) {
	using namespace RLConst;

	_RespawnAt(gameMode, Math::RandInt(0, CAR_RESPAWN_LOCATION_AMOUNT, seed), boostAmount);
}

void Car::_RespawnAt(GameMode gameMode, int spawnPosIndex, float boostAmount) {
	using namespace RLConst;

	CarState newState = CarState();

	const CarSpawnPos* spawnPosArray;
	switch (gameMode) {
	case GameMode::HOOPS:
		spawnPosArray = CAR_RESPAWN_LOCATIONS_HOOPS;
		break;
	case GameMode::DROPSHOT:
		spawnPosArray = CAR_RESPAWN_LOCATIONS_DROPSHOT;
		break;
	default:
		spawnPosArray = CAR_RESPAWN_LOCATIONS_SOCCAR;
	}

	CarSpawnPos spawnPos = spawnPosArray[spawnPosIndex];

	newState.pos = Vec(spawnPos.x, spawnPos.y * (team == Team::BLUE ? 1 : -1), CAR_RESPAWN_Z);
	newState.rotMat = Angle(spawnPos.yawAng + (team == Team::BLUE ? 0 : M_PI), 0.f, 0.f).ToRotMat();

	newState.boost = boostAmount;
	this->SetState(newState);
}

// The game's 8-bit axis input (negatives /128, positives /127)
static float QuantizeAxisInput(float x) {
	float c = RS_CLAMP(x, -1.f, 1.f);
	float y = c < 0 ? RS_MAX(c * 128.f, -128.f) : RS_MIN(c * 127.f, 127.f);
	float w = ((y + 128.f) + (y + 128.f)) + 0.5f;
	int byte = ((int)rintf(w) >> 1) & 0xFF;
	float s = (float)byte - 128.f;
	return byte < 0x80 ? s * (1.f / 128.f) : s / 127.f;
}

void Car::_PreTickUpdate(GameMode gameMode, float tickTime, const MutatorConfig& mutatorConfig, int respawnPosIndex) {
	using namespace RLConst;

#ifndef RS_MAX_SPEED
	// Fix inputs
	controls.ClampFix();
#endif

	assert(_bulletVehicle.getNumWheels() == 4 || _bulletVehicle.getNumWheels() == 3);

	{ // Update simulation state
		if (_internalState.isDemoed) {
			_internalState.demoRespawnTimer = RS_MAX(_internalState.demoRespawnTimer - tickTime, 0);
			if (_internalState.demoRespawnTimer == 0)
				_RespawnAt(gameMode, respawnPosIndex, mutatorConfig.carSpawnBoostAmount);

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

	_planeContactsTick++;

	if (_internalState.isDemoed)
		return; // No other updates need to occur

	// The physics reads the game's quantized inputs; the raw controls come back at the end of the tick
	const CarControls rawControls = controls;
	controls.throttle = QuantizeAxisInput(controls.throttle);
	controls.steer = QuantizeAxisInput(controls.steer);
	controls.pitch = QuantizeAxisInput(controls.pitch);
	controls.yaw = QuantizeAxisInput(controls.yaw);
	controls.roll = QuantizeAxisInput(controls.roll);

	{ // Update supersonic, as the game does before the physics step: supersonic at the start speed, then until the speed
		// drops below the maintain speed or the car has spent SUPERSONIC_MAINTAIN_MAX_TIME below the start speed
		float speedSquared = (_rigidBody.m_linearVelocity * BT_TO_UU).length2();
		if (speedSquared >= SUPERSONIC_START_SPEED * SUPERSONIC_START_SPEED) {
			_internalState.isSupersonic = true;
			_internalState.supersonicTime = 0;
		} else if (_internalState.isSupersonic) {
			_internalState.supersonicTime += tickTime;
			if (_internalState.supersonicTime >= SUPERSONIC_MAINTAIN_MAX_TIME
				|| speedSquared < SUPERSONIC_MAINTAIN_MIN_SPEED * SUPERSONIC_MAINTAIN_MIN_SPEED) {
				_internalState.isSupersonic = false;
				_internalState.supersonicTime = 0;
			}
		}
	}

	// Last tick's wheel contacts
	int prevNumWheels = 0;
	for (int i = 0; i < _bulletVehicle.m_wheelInfo.size(); i++)
		prevNumWheels += _internalState.wheelsWithContact[i] ? 1 : 0;
	const bool wasOnGround = prevNumWheels >= 3;

	bool jumpPressed = controls.jump && !_internalState.lastControls.jump;

	// The game's jump availability: a jump needs the car on the ground (last tick's wheel contacts) and bJumped clear,
	// a dodge or double jump needs it off the ground. bJumped clears when the car lands, or on the ground once the
	// jump has been inactive for 0.125 s
	const bool wasJumping = _internalState.isJumping;
	const bool airJumpUsedBefore = _internalState.hasDoubleJumped || _internalState.hasFlipped;
	if (_internalState.tickCountSinceUpdate == 0) {
		_gameJumped = _internalState.hasJumped;
		_gameDoubleJumped = _internalState.hasDoubleJumped || _internalState.hasFlipped;
		_gameOnGround = wasOnGround;
		_jumpInactiveTime = _internalState.isJumping ? 0 : _internalState.jumpTime;
	}
	if (wasOnGround && !_gameOnGround)
		_gameJumped = _gameDoubleJumped = false;
	_gameOnGround = wasOnGround;
	if (_gameJumped && wasOnGround && !_internalState.isJumping && _jumpInactiveTime >= 0.125f)
		_gameJumped = _gameDoubleJumped = false;
	// The dodge window closes after a double jump or dodge, or once the jump has been inactive for DOUBLEJUMP_MAX_DELAY
	const bool airJumpWindowClosed = _gameDoubleJumped || (_gameJumped && _jumpInactiveTime > DOUBLEJUMP_MAX_DELAY);
	const bool airJumpPressed = jumpPressed && !wasOnGround && !airJumpWindowClosed;

	// The game starts the jump before the wheel pass, from last tick's ground state
	bool jumpStarted = false;
	if (jumpPressed && _internalState.isOnGround && !_internalState.isJumping && !_gameJumped) {
		_StartJump(mutatorConfig);
		jumpStarted = true;
	}

	// Last tick's world-contact wheels and their normals (on the first tick after a state set, this tick's, below)
	int prevWorldWheels = 0;
	btVector3 prevWorldWheelNormalSum = btVector3(0, 0, 0);
	if (_internalState.tickCountSinceUpdate != 0) {
		prevWorldWheels = _prevWorldWheels;
		prevWorldWheelNormalSum = _prevWorldWheelNormalSum;
	}

	const bool hasFlippedAtStart = _internalState.hasFlipped;
	// With no flip active, the game locks air pitch until the flip restart timer reaches FLIP_PITCHLOCK_EXTRA_TIME (a zero
	// timer means unknown: no lock)
	const float pitchLockRestartTime = (!_internalState.hasFlipped && !_internalState.isFlipping && _internalState.flipTime > 0) ?
		_internalState.flipTime : -1.f;

	// The game updates the dodge and double jump before the wheels, with the start-of-tick forward speed
	float forwardSpeed_UU = _bulletVehicle.getForwardSpeed() * BT_TO_UU;
	_UpdateDoubleJumpOrFlip(tickTime, mutatorConfig, airJumpPressed && !jumpStarted, forwardSpeed_UU, rawControls.pitch);

	// This tick's handbrake and steer angle go into the wheel transforms
	_UpdateHandbrakeAndSteering(tickTime, abs(forwardSpeed_UU));

	// Do first part of the btVehicleRL update (update wheel transforms, do traces)
	_bulletVehicle.updateVehicleFirst(tickTime);

	// Update wheelsWithContact
	int numWheelsInContact = 0;
	for (int i = 0; i < _bulletVehicle.m_wheelInfo.size(); i++) {
		bool inContact = _bulletVehicle.m_wheelInfo[i].m_raycastInfo.m_isInContact;
		_internalState.wheelsWithContact[i] = inContact;
		numWheelsInContact += inContact;
	}

	{ // This tick's world-contact wheels and normals, for the next tick
		btVector3 sum = btVector3(0, 0, 0);
		int worldWheels = 0;
		for (int i = 0; i < _bulletVehicle.m_wheelInfo.size(); i++)
			if (_bulletVehicle.m_wheelInfo[i].m_raycastInfo.m_isInContact && _bulletVehicle.m_wheelInfo[i].m_isInContactWithWorld) {
				sum += _bulletVehicle.m_wheelInfo[i].m_raycastInfo.m_contactNormalWS;
				worldWheels++;
			}
		_prevWorldWheelNormalSum = sum;
		_prevWorldWheels = worldWheels;
		if (_internalState.tickCountSinceUpdate == 0) {
			prevWorldWheels = worldWheels;
			prevWorldWheelNormalSum = sum;
		}
	}

	{ // Update isOnGround
		_internalState.isOnGround = numWheelsInContact >= 3;
	}

	_UpdateWheels(tickTime, mutatorConfig, numWheelsInContact, forwardSpeed_UU, prevWorldWheels, prevWorldWheelNormalSum, rawControls.throttle);

	// The game's air control and air throttle need no wheel contact last tick (on the first tick after a state set, the
	// state's or this tick's contacts)
	int airControlPrevWheels = prevNumWheels;
	if (_internalState.tickCountSinceUpdate == 0)
		airControlPrevWheels = RS_MAX(numWheelsInContact, prevNumWheels);

	// The game's air-control and dodge-torque stage counts last tick's wheel contacts, except on the first tick after a
	// state set and on the tick a dodge starts
	int airWheelsInContact = RS_MAX(numWheelsInContact, prevNumWheels);
	if (_internalState.tickCountSinceUpdate != 0 && !(_internalState.hasFlipped && !hasFlippedAtStart))
		airWheelsInContact = prevNumWheels;

	const bool wasFlipping = _internalState.isFlipping;
	if (airWheelsInContact < 3) {
		_UpdateAirTorque(tickTime, mutatorConfig, airControlPrevWheels == 0, rawControls.pitch, pitchLockRestartTime);
	} else {
		_internalState.isFlipping = false;
		if (airControlPrevWheels == 0) // Touchdown on 3 or 4 wheels: still in the air last tick
			_UpdateAirTorque(tickTime, mutatorConfig, true, rawControls.pitch, pitchLockRestartTime);
	}

	// The game applies no auto-roll on the tick the flip ends
	const bool flipEnded = airWheelsInContact < 3 && wasFlipping && !_internalState.isFlipping;

	_UpdateJump(tickTime, mutatorConfig, jumpPressed);
	_UpdateAutoFlip(tickTime, mutatorConfig, jumpPressed);

	{ // A started jump, dodge or double jump sets bJumped
		bool airJumpUsed = !airJumpUsedBefore && (_internalState.hasDoubleJumped || _internalState.hasFlipped);
		if ((_internalState.isJumping && !wasJumping) || airJumpUsed)
			_gameJumped = true;
		if (airJumpUsed)
			_gameDoubleJumped = true;
		_jumpInactiveTime = _internalState.isJumping ? 0 : _jumpInactiveTime + tickTime;
	}

	// The game's auto-roll counts and aims with last tick's world-contact wheels, and not while dodging. On the first tick
	// after a state set with 1-3 wheel contacts, it uses the state's contacts
	int autoRollWheels = prevWorldWheels;
	bool autoRollActive = autoRollWheels < 4 && (autoRollWheels > 0 || _internalState.worldContact.hasContact);
	if (_internalState.tickCountSinceUpdate == 0 && prevNumWheels > 0 && prevNumWheels < 4) {
		const bool sticky = prevWorldWheels > 0;
		autoRollActive = sticky || _internalState.worldContact.hasContact;
		autoRollWheels = sticky ? prevNumWheels : 0;
	}
	if (controls.throttle && !_internalState.isFlipping && !flipEnded && autoRollActive)
		_UpdateAutoRoll(tickTime, mutatorConfig, autoRollWheels, prevWorldWheelNormalSum);

	_internalState.worldContact.hasContact = false;

	// Complete the btVehicleRL update (does suspension, then computes and applies wheel friction)
	_bulletVehicle.updateVehicleSecond(tickTime);

	_UpdateBoost(tickTime, mutatorConfig, forwardSpeed_UU);

	controls = rawControls;
}

void Car::_PostTickUpdate(GameMode gameMode, float tickTime, const MutatorConfig& mutatorConfig) {

	if (_internalState.isDemoed)
		return;

	_internalState.rotMat = _rigidBody.getWorldTransform().m_basis;

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

	_internalState.tickCountSinceUpdate++;
}

void Car::_LimitVelocities() {
	using namespace RLConst;

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

// The game's rounding of a velocity component: truncated to 1 / scale, plus 0.1 / scale away from zero
static float RoundVelocity(float val, float scale) {
	float k = truncf(val * scale);
	return k == 0.0f ? 0.0f : (k + copysignf(0.1f, k)) / scale;
}

void Car::_TickStartUpdate(float tickTime) {
	using namespace RLConst;

	if (_internalState.isDemoed)
		return;

	{ // The game limits and rounds the car's state at the start of every tick
		_LimitVelocities();

		btVector3 pos = _rigidBody.getWorldTransform().m_origin * BT_TO_UU;
		btVector3 vel = _rigidBody.m_linearVelocity * BT_TO_UU;
		btVector3 angVel = _rigidBody.m_angularVelocity;
		_rigidBody.getWorldTransform().m_origin =
			btVector3(Math::RoundHalfUp(pos.x(), 100), Math::RoundHalfUp(pos.y(), 100), Math::RoundHalfUp(pos.z(), 100)) * UU_TO_BT;
		_rigidBody.m_linearVelocity =
			btVector3(RoundVelocity(vel.x(), 100), RoundVelocity(vel.y(), 100), RoundVelocity(vel.z(), 100)) * UU_TO_BT;
		_rigidBody.m_angularVelocity =
			btVector3(RoundVelocity(angVel.x(), 10000), RoundVelocity(angVel.y(), 10000), RoundVelocity(angVel.z(), 10000));
	}

	_startVel = _rigidBody.m_linearVelocity;

	// The game's flip z-damping runs before any car's update, while airborne (last tick's ground state) with the flip
	// time in [FLIP_Z_DAMP_START, FLIP_TORQUE_TIME] (our flip time advances in the dodge update, so it trails by a tick)
	if (_internalState.isOnGround || !_internalState.isFlipping)
		return;
	float flipTime = _internalState.flipTime + tickTime;
	if (!(flipTime >= FLIP_Z_DAMP_START && flipTime <= FLIP_TORQUE_TIME))
		return;
	if (_rigidBody.m_linearVelocity.z() < 0 || flipTime < FLIP_Z_DAMP_END)
		_rigidBody.m_linearVelocity.z() *= powf(1 - FLIP_Z_DAMP_120, tickTime / (1 / 120.f));
}

void Car::_PlaneContactsCallback(btPersistentManifold* manifold, const btCollisionObject* carObj, const btCollisionObject* planeObj, bool store) {
	if (carObj->getUserIndex() != BT_USERINFO_TYPE_CAR)
		return;
	Car* car = (Car*)carObj->getUserPointer();

	// One slot per plane
	PlaneContacts* slot = nullptr;
	for (auto& entry : car->_planeContacts) {
		if (entry.plane == planeObj || !entry.plane) {
			slot = &entry;
			break;
		}
	}
	if (!slot)
		return;

	if (store) {
		slot->plane = planeObj;
		slot->tick = car->_planeContactsTick;
		slot->numPoints = manifold->getNumContacts();
		for (int i = 0; i < slot->numPoints; i++)
			slot->points[i] = manifold->getContactPoint(i);
	} else if (slot->plane == planeObj && slot->tick + 1 == car->_planeContactsTick) {
		// Only last tick's points: a pair not processed last tick starts empty
		for (int i = 0; i < slot->numPoints; i++)
			manifold->addManifoldPoint(slot->points[i], true);
	}
}

void Car::_BulletSetup(GameMode gameMode, btDynamicsWorld* bulletWorld, const MutatorConfig& mutatorConfig) {
	// Set up rigidbody and collision shapes
	btVector3 halfExtents = (config.hitboxSize * UU_TO_BT) / 2;
	_childHitboxShape = btBoxShape(halfExtents);
	{ // The game's box keeps its full half extents with the lowered margin
		float margin = _childHitboxShape.getMargin();
		_childHitboxShape.setImplicitShapeDimensions(halfExtents - btVector3(margin, margin, margin));
	}
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

	_rigidBody.m_collisionFlags |= btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK | btCollisionObject::CF_ROCKETSIM_CAR;

	_rigidBody.m_friction = RLConst::CAR_COLLISION_FRICTION;
	_rigidBody.m_restitution = RLConst::CAR_COLLISION_RESTITUTION;

	// Disable gyroscopic force
	_rigidBody.m_rigidbodyFlags = 0;
	
	// We want our car and our suspension rays to collide with the dropshot floor
	int extraCollisionMask = CollisionMasks::DROPSHOT_FLOOR;

	// Add rigidbody to world
	bulletWorld->addRigidBody(
		&_rigidBody, btBroadphaseProxy::DefaultFilter | extraCollisionMask, btBroadphaseProxy::AllFilter
	);

	{ // Set up actual vehicle stuff
		_bulletVehicleRaycaster = btDefaultVehicleRaycaster(bulletWorld);

		btVehicleRL::btVehicleTuning tuning = btVehicleRL::btVehicleTuning();

		_bulletVehicle = btVehicleRL(tuning, &_rigidBody, &_bulletVehicleRaycaster, bulletWorld, extraCollisionMask);

		// Match RL with X forward, Y right, Z up
		_bulletVehicle.setCoordinateSystem(1, 2, 0);

		// Set up wheel directions with RL coordinate system
		btVector3 wheelDirectionCS(0, 0, -1), wheelAxleCS(0, -1, 0);

		{ // Set up wheels
			int numWheels = 4;//config.threeWheels ? 3 : 4;
			for (int i = 0; i < numWheels; i++) {
				bool front = i < 2;
				if (numWheels == 3)
					front = i == 0;

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
					wheelInfo.m_maxSuspensionTravelCm = (SUSPENSION_TRAVEL_LIMIT * UU_TO_BT) * 100; // Same for all cars (hopefully)
					wheelInfo.m_maxSuspensionForce = FLT_MAX; // Don't think there's a limit
					wheelInfo.m_bIsFrontWheel = front;
				}
			}
		}

		{ // The game's suspension force scale is half the car's mass times the other axle's share of the wheelbase
			float mass = 1.f / _rigidBody.getInvMass();
			float frontX = _bulletVehicle.m_wheelInfo[0].m_chassisConnectionPointCS.x();
			float backX = -_bulletVehicle.m_wheelInfo[2].m_chassisConnectionPointCS.x();
			float front, back;
			if (frontX < 0 || backX < 0) {
				front = back = (mass * 0.5f) * 0.5f;
			} else {
				float backShare = (frontX / (backX + frontX)) * mass;
				front = (mass - backShare) * 0.5f;
				back = backShare * 0.5f;
			}
			for (int i = 0; i < _bulletVehicle.m_wheelInfo.size(); i++)
				_bulletVehicle.m_wheelInfo[i].m_suspensionForceScale = _bulletVehicle.m_wheelInfo[i].m_bIsFrontWheel ? front : back;
		}

		// The game ignores wheel hits closer to the hardpoint than half the car's body box height minus 1 uu (the body box
		// is the hitbox 1.25 uu thinner on each face)
		float minHitDist = (config.hitboxSize.z * 0.5f - 2.25f) * UU_TO_BT;
		_bulletVehicle.m_minHitDistSq = minHitDist * minHitDist;
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

void Car::_UpdateHandbrakeAndSteering(float tickTime, float absForwardSpeed_UU) {
	using namespace RLConst;

	{ // Increase/decrease handbrake value from input
		if (controls.handbrake) {
			_internalState.handbrakeVal += POWERSLIDE_RISE_RATE * tickTime;
		} else {
			_internalState.handbrakeVal -= POWERSLIDE_FALL_RATE * tickTime;
		}
		_internalState.handbrakeVal = RS_CLAMP(_internalState.handbrakeVal, 0, 1);
	}

	{ // Update steering
		float steerAngle = (config.threeWheels ? STEER_ANGLE_FROM_SPEED_CURVE_THREEWHEEL : STEER_ANGLE_FROM_SPEED_CURVE).GetOutput(absForwardSpeed_UU);

		if (_internalState.handbrakeVal) {
			steerAngle +=
				(POWERSLIDE_STEER_ANGLE_FROM_SPEED_CURVE.GetOutput(absForwardSpeed_UU) - steerAngle)
				* _internalState.handbrakeVal;
		}

		steerAngle *= controls.steer;
		_bulletVehicle.m_wheelInfo[0].m_steerAngle = steerAngle;
		_bulletVehicle.m_wheelInfo[1].m_steerAngle = steerAngle;
	}
}

void Car::_UpdateWheels(float tickTime, const MutatorConfig& mutatorConfig, int numWheelsInContact, float forwardSpeed_UU,
	int prevWorldWheels, const btVector3& prevWorldWheelNormalSum, float rawThrottle) {
	using namespace RLConst;

	float absForwardSpeed_UU = abs(forwardSpeed_UU);

	// The game's drive reads the throttle input (boosting doesn't set it)
	float realThrottle = controls.throttle;
	float realBrake = 0;

	{ // Update throttle/brake forces
		float driveSpeedScale = DRIVE_SPEED_TORQUE_FACTOR_CURVE.GetOutput(absForwardSpeed_UU);

		float engineThrottle = realThrottle;

		if (controls.handbrake) {
			// Real throttle is unchanged from the input throttle when powersliding
		} else {
			float absThrottle = abs(realThrottle);

			if (absThrottle > THROTTLE_DEADZONE) {
				if (absForwardSpeed_UU > STOPPING_FORWARD_VEL && RS_SGN(realThrottle) != RS_SGN(forwardSpeed_UU)) {
					// Full brake is applied if we are trying to drive in the opposite direction
					realBrake = 1;

					if (absForwardSpeed_UU > BRAKING_NO_THROTTLE_SPEED_THRESH) {
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
		for (int i = 0; i < _bulletVehicle.m_wheelInfo.size(); i++) {
			_bulletVehicle.m_wheelInfo[i].m_engineForce = driveEngineForce;
			_bulletVehicle.m_wheelInfo[i].m_brake = driveBrakeForce;
		}
	}

	{ // Update friction
		for (int i = 0; i < _bulletVehicle.m_wheelInfo.size(); i++) {
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

				float latFriction = (config.threeWheels ? LAT_FRICTION_CURVE_THREEWHEEL : LAT_FRICTION_CURVE).GetOutput(frictionCurveInput);
				float longFriction = LONG_FRICTION_CURVE.GetOutput(frictionCurveInput);

				if (_internalState.handbrakeVal) {
					float handbrakeAmount = _internalState.handbrakeVal;

					latFriction *= (HANDBRAKE_LAT_FRICTION_FACTOR_CURVE.GetOutput(frictionCurveInput) - 1) * handbrakeAmount + 1;
					longFriction *= (HANDBRAKE_LONG_FRICTION_FACTOR_CURVE.GetOutput(frictionCurveInput) - 1) * handbrakeAmount + 1;
				} else {
					longFriction = 1; // If we aren't powersliding, it's not scaled down
				}

				// (A wheel on a car or the ball is never sticky)
				bool isContactSticky = realThrottle != 0 && wheel.m_isInContactWithWorld;

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

	// Update sticky forces if at least 1 wheel had world contact last tick, along those wheels' contact normals
	if (prevWorldWheels > 0) {
		btVector3 upwardsDir = (prevWorldWheelNormalSum == btVector3(0, 0, 0)) ? _bulletVehicle.getUpVector() : prevWorldWheelNormalSum.safeNormalized();

		bool fullStick = (rawThrottle != 0) || (absForwardSpeed_UU > STOPPING_FORWARD_VEL);

		float stickyForceScale = config.threeWheels ? 0 : 0.5f;
		if (fullStick)
			stickyForceScale += 1 - abs(upwardsDir.z());

		_rigidBody.applyCentralForce(upwardsDir * stickyForceScale * (GRAVITY_Z * UU_TO_BT) * CAR_MASS_BT);
	}
}

void Car::_UpdateBoost(float tickTime, const MutatorConfig& mutatorConfig, float forwardSpeed_UU) {
	using namespace RLConst;

	// As the game, the first boost tick is free, and while boosting the boost is paid first, stopping without pushing
	// once the tank is empty
	bool push;
	if (_internalState.isBoosting) {
		_internalState.boost = RS_MAX(_internalState.boost - mutatorConfig.boostUsedPerSecond * tickTime, 0);
		bool minTimeDone = !controls.boost && _internalState.boostingTime >= BOOST_MIN_TIME;
		push = !(_internalState.boost == 0 || minTimeDone);
	} else {
		push = controls.boost && _internalState.boost > 0;
	}
	_internalState.isBoosting = push;

	if (push) {
		// In the air, the game's air control throttle adds THROTTLE_AIR_ACCEL to the boost (see _UpdateAirTorque)
		float boostAccel = _internalState.isOnGround ? mutatorConfig.boostAccelGround : mutatorConfig.boostAccelAir - THROTTLE_AIR_ACCEL;
		_rigidBody.applyCentralForce(boostAccel * UU_TO_BT * GetForwardDir() * CAR_MASS_BT);
		_internalState.boostingTime += tickTime;
		_internalState.timeSinceBoosted = 0;
	} else {
		_internalState.boostingTime = 0;
		_internalState.timeSinceBoosted += tickTime;

		// Recharge boost
		if (mutatorConfig.rechargeBoostEnabled)
			if (_internalState.timeSinceBoosted >= mutatorConfig.rechargeBoostDelay)
				_internalState.boost += mutatorConfig.rechargeBoostPerSecond * tickTime;
	}

	_internalState.boost = RS_MIN(RS_MAX(_internalState.boost, 0), RLConst::BOOST_MAX);
}

void Car::_StartJump(const MutatorConfig& mutatorConfig) {
	using namespace RLConst;
	_internalState.isJumping = true;
	_internalState.jumpTime = 0;
	btVector3 jumpStartForce = GetUpDir() * mutatorConfig.jumpImmediateForce * UU_TO_BT * CAR_MASS_BT;
	_rigidBody.applyCentralImpulse(jumpStartForce);
	_LimitVelocities(); // The game limits the velocities after the jump impulse
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
			_internalState.jumpTime = 0;
		}
	}

	if (_internalState.isJumping) {
		if (_internalState.jumpTime < JUMP_MIN_TIME || (controls.jump && _internalState.jumpTime < JUMP_MAX_TIME)) {
			// Continue jump
			_internalState.isJumping = true;
		} else {
			// We can't keep jumping any longer
			_internalState.isJumping = false;
		}
	}

	if (_internalState.isJumping) {
		_internalState.hasJumped = true;

		// Apply extra long-jump force
		btVector3 totalJumpForce = GetUpDir() * mutatorConfig.jumpAccel;

		_rigidBody.applyCentralForce(totalJumpForce * UU_TO_BT * CAR_MASS_BT);
	}

	if (_internalState.isJumping || _internalState.hasJumped) {
		_internalState.jumpTime += tickTime;
	}
}

void Car::_UpdateAirTorque(float tickTime, const MutatorConfig& mutatorConfig, bool updateAirControl, float rawPitch, float pitchLockRestartTime) {
	using namespace RLConst;

	btVector3
		dirPitch_right = -GetRightDir(),
		dirYaw_up = GetUpDir(),
		dirRoll_forward = -GetForwardDir();

	if (_internalState.isFlipping)
		_internalState.isFlipping = _internalState.hasFlipped && _internalState.flipTime < FLIP_TORQUE_TIME;

	if (_internalState.isFlipping) {

		btVector3 relDodgeTorque = _internalState.flipRelTorque;

		if (!_internalState.flipRelTorque.IsZero()) {
			// Flip cancel check: the game cancels once the dodge has run for FLIP_TORQUE_MIN_TIME, with its quantization of
			// the negated raw pitch
			const float cancelPitch = (_internalState.flipTime >= FLIP_TORQUE_MIN_TIME) ? -QuantizeAxisInput(-rawPitch) : 0;
			float pitchScale = 1;
			if (relDodgeTorque.y() != 0 && cancelPitch != 0) {
				if (RS_SGN(relDodgeTorque.y()) == RS_SGN(cancelPitch)) {

#ifndef RS_MAX_SPEED
					pitchScale = 1 - RS_MIN(abs(cancelPitch), 1); // Sanity clamp
#else
					pitchScale = 1 - abs(cancelPitch); // No sanity check
#endif
				}
			}

			relDodgeTorque.y() *= pitchScale;

			btVector3 dodgeTorque = relDodgeTorque * btVector3(FLIP_TORQUE_X, FLIP_TORQUE_Y, 0);
			_rigidBody.applyTorque(_rigidBody.m_invInertiaTensorWorld.inverse() * _rigidBody.getWorldTransform().m_basis * dodgeTorque);
		}
	}

	// The game keeps air control running during a dodge
	bool doAirControl = !_internalState.isAutoFlipping && updateAirControl;
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
			if (pitchLockRestartTime >= 0 && pitchLockRestartTime < FLIP_PITCHLOCK_EXTRA_TIME)
				pitchTorqueScale = 0;

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

	// While boosting (last tick's state), the game's air throttle is full forward
	float airThrottle = _internalState.isBoosting ? 1.f : controls.throttle;
	if (airThrottle != 0 && updateAirControl)
		_rigidBody.applyCentralForce(GetForwardDir() * airThrottle * THROTTLE_AIR_ACCEL * UU_TO_BT * CAR_MASS_BT);
}

// The flip time as the game's flip restart timer, once a landing clears hasFlipped: a mid-flip landing restarts it at 0,
// otherwise it counts from the end of the flip. A zero timer with no flip active stays zero and means unknown
float Car::_GetFlipRestartTime(float tickTime) const {
	using namespace RLConst;
	if (!_internalState.hasFlipped)
		return _internalState.flipTime;
	if (_internalState.isFlipping)
		return 0;
	if (_internalState.flipTime < FLIP_TORQUE_TIME)
		return 0;
	int k = (int)rintf(_internalState.flipTime / tickTime) - (int)rintf(FLIP_TORQUE_TIME / tickTime);
	if (k < 1)
		k = 1;
	if (k > 64)
		return (float)k * tickTime;
	float t = 0;
	for (int i = 0; i < k; i++)
		t += tickTime;
	return t;
}

void Car::_UpdateDoubleJumpOrFlip(float tickTime, const MutatorConfig& mutatorConfig, bool jumpPressed, float forwardSpeed_UU, float rawPitch) {
	using namespace RLConst;

	float tickTimeScale = tickTime / (1 / 120.f);

	if (_internalState.isOnGround) {
		_internalState.hasDoubleJumped = false;
		_internalState.flipTime = _GetFlipRestartTime(tickTime);
		_internalState.hasFlipped = false;
		_internalState.airTime = 0;
		_internalState.airTimeSinceJump = 0;
	}

	// A press while the game's car is off the ground (by last tick's contacts) takes the air branch
	if (!_internalState.isOnGround || jumpPressed) {
		_internalState.airTime += tickTime;

		if (_internalState.hasJumped && !_internalState.isJumping) {
			_internalState.airTimeSinceJump += tickTime;
		} else {
			_internalState.airTimeSinceJump = 0;
		}

		if (jumpPressed && _internalState.airTimeSinceJump < DOUBLEJUMP_MAX_DELAY) {
			// The game's dodge input quantizes the negated raw pitch
			CarControls controls = this->controls;
			controls.pitch = -QuantizeAxisInput(-rawPitch);
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

						_internalState.flipRelTorque = btVector3(-dodgeDir.y() / tickTimeScale, dodgeDir.x() / tickTimeScale, 0);

						if (abs(dodgeDir.x()) < 0.1f) dodgeDir.x() = 0;
						if (abs(dodgeDir.y()) < 0.1f) dodgeDir.y() = 0;

						if (!dodgeDir.fuzzyZero()) {
							bool shouldDodgeBackwards;

							if (abs(forwardSpeed_UU) < 100.0f) {
								shouldDodgeBackwards = dodgeDir.x() < 0.0f;
							} else {
								shouldDodgeBackwards = (dodgeDir.x() >= 0.0f) != (forwardSpeed_UU >= 0.0f);
							}

							Vec initalDodgeVel = dodgeDir * FLIP_INITIAL_VEL_SCALE;

							float maxSpeedScaleX =
								shouldDodgeBackwards ? FLIP_BACKWARD_IMPULSE_MAX_SPEED_SCALE : FLIP_FORWARD_IMPULSE_MAX_SPEED_SCALE;

							initalDodgeVel.x *= ((maxSpeedScaleX - 1) * forwardSpeedRatio) + 1.f;
							initalDodgeVel.y *= ((FLIP_SIDE_IMPULSE_MAX_SPEED_SCALE - 1) * forwardSpeedRatio) + 1.f;

							if (shouldDodgeBackwards)
								initalDodgeVel.x *= FLIP_BACKWARD_IMPULSE_SCALE_X;

							Vec forwardDir2D = GetForwardDir().To2D().Normalized();
							Vec rightDir2D = Vec(-forwardDir2D.y, forwardDir2D.x, 0.f);

							Vec finalDeltaVel = initalDodgeVel.x * forwardDir2D + initalDodgeVel.y * rightDir2D;

							_rigidBody.applyCentralImpulse(finalDeltaVel * UU_TO_BT * CAR_MASS_BT);
							_LimitVelocities(); // The game limits the velocities after the dodge impulse
						}
					}

				} else {
					// Double jump, add upwards velocity
					btVector3 jumpStartForce = GetUpDir() * JUMP_IMMEDIATE_FORCE * UU_TO_BT * CAR_MASS_BT;
					_rigidBody.applyCentralImpulse(jumpStartForce);
					_LimitVelocities(); // The game limits the velocities after the double jump impulse

					_internalState.hasDoubleJumped = true;
				}
			}

			// The game's flip clock reads 0 on the dodge tick
			if (_internalState.isFlipping && _internalState.flipTime == 0)
				_internalState.flipTime = -tickTime;
		}
	}

	// (The z-damping is in _TickStartUpdate)
	if (_internalState.isFlipping || _internalState.hasFlipped || _internalState.flipTime != 0) {
		// Increment flip time even after we are done flipping, as the game's flip restart timer
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

		// The game gates on the car's up axis against the contact normal, not the roll
		float absRoll = abs(angles.roll);
		if (_internalState.worldContact.contactNormal.Dot(GetUpDir()) < CAR_AUTOFLIP_UP_DOT_THRESH) {
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

void Car::_UpdateAutoRoll(float tickTime, const MutatorConfig& mutatorConfig, int numWheelsInContact, const btVector3& wheelNormalSum) {
	using namespace RLConst;

	btVector3 groundUpDir;
	if (numWheelsInContact > 0) {
		groundUpDir = (wheelNormalSum == btVector3(0, 0, 0)) ? _bulletVehicle.getUpVector() : wheelNormalSum.safeNormalized();
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

#include "btVehicleRL.h"
#include "../../../libsrc/bullet3-3.24/BulletDynamics/ConstraintSolver/btContactConstraint.h"
#define ROLLING_INFLUENCE_FIX

btVehicleRL::btVehicleRL(const btVehicleTuning& tuning, btRigidBody* chassis, btVehicleRaycaster* raycaster, btDynamicsWorld* world)
	: m_vehicleRaycaster(raycaster), m_pitchControl(0),  m_dynamicsWorld(world) {
	m_chassisBody = chassis;
	m_indexRightAxis = 0;
	m_indexUpAxis = 2;
	m_indexForwardAxis = 1;
	defaultInit(tuning);
}

void btVehicleRL::defaultInit(const btVehicleTuning& tuning) {
	(void)tuning;
	m_steeringValue = 0;
}

btVehicleRL::~btVehicleRL() {
}

//
// basically most of the code is general for 2 or 4 wheel vehicles, but some of it needs to be reviewed
//
btWheelInfoRL& btVehicleRL::addWheel(const Vec& connectionPointCS, const Vec& wheelDirectionCS0, const Vec& wheelAxleCS, float suspensionRestLength, float wheelRadius, const btVehicleTuning& tuning, bool isFrontWheel) {
	btWheelInfoConstructionInfo ci;

	ci.m_chassisConnectionCS = connectionPointCS;
	ci.m_wheelDirectionCS = wheelDirectionCS0;
	ci.m_wheelAxleCS = wheelAxleCS;
	ci.m_suspensionRestLength = suspensionRestLength;
	ci.m_wheelRadius = wheelRadius;
	ci.m_suspensionStiffness = tuning.m_suspensionStiffness;
	ci.m_wheelsDampingCompression = tuning.m_suspensionCompression;
	ci.m_wheelsDampingRelaxation = tuning.m_suspensionDamping;
	ci.m_frictionSlip = tuning.m_frictionSlip;
	ci.m_bIsFrontWheel = isFrontWheel;
	ci.m_maxSuspensionTravelCm = tuning.m_maxSuspensionTravelCm;
	ci.m_maxSuspensionForce = tuning.m_maxSuspensionForce;

	m_wheelInfo.push_back(btWheelInfoRL(ci));

	btWheelInfoRL& wheel = m_wheelInfo[getNumWheels() - 1];

	updateWheelTransformsWS(wheel);
	updateWheelTransform(getNumWheels() - 1);
	return wheel;
}

const btTransform& btVehicleRL::getWheelTransformWS(int wheelIndex) const {
	btAssert(wheelIndex < getNumWheels());
	const btWheelInfoRL& wheel = m_wheelInfo[wheelIndex];
	return wheel.m_worldTransform;
}

void btVehicleRL::updateWheelTransform(int wheelIndex) {
	btWheelInfoRL& wheel = m_wheelInfo[wheelIndex];
	updateWheelTransformsWS(wheel);
	Vec up = -wheel.m_raycastInfo.m_wheelDirectionWS;
	const Vec& right = wheel.m_raycastInfo.m_wheelAxleWS;
	Vec fwd = up.cross(right);
	fwd = fwd.normalize();
	//	up = right.cross(fwd);
	//	up.normalize();

	btQuaternion steeringOrn(up, wheel.m_steerAngle);
	btMatrix3x3 steeringMat(steeringOrn);

	btMatrix3x3 basis2;
	basis2[0][m_indexRightAxis] = -right[0];
	basis2[1][m_indexRightAxis] = -right[1];
	basis2[2][m_indexRightAxis] = -right[2];

	basis2[0][m_indexUpAxis] = up[0];
	basis2[1][m_indexUpAxis] = up[1];
	basis2[2][m_indexUpAxis] = up[2];

	basis2[0][m_indexForwardAxis] = fwd[0];
	basis2[1][m_indexForwardAxis] = fwd[1];
	basis2[2][m_indexForwardAxis] = fwd[2];
	wheel.m_worldTransform.setBasis(steeringMat * basis2);
	wheel.m_worldTransform.setOrigin(
		wheel.m_raycastInfo.m_hardPointWS + wheel.m_raycastInfo.m_wheelDirectionWS * wheel.m_raycastInfo.m_suspensionLength);
}

void btVehicleRL::resetSuspension() {
	int i;
	for (i = 0; i < m_wheelInfo.size(); i++) {
		btWheelInfoRL& wheel = m_wheelInfo[i];
		wheel.m_raycastInfo.m_suspensionLength = wheel.getSuspensionRestLength();
		wheel.m_suspensionRelativeVelocity = 0;

		wheel.m_raycastInfo.m_contactNormalWS = -wheel.m_raycastInfo.m_wheelDirectionWS;
		wheel.m_clippedInvContactDotSuspension = 1;
		wheel.m_extraPushback = 0;
	}
}

void btVehicleRL::updateWheelTransformsWS(btWheelInfoRL& wheel) {
	wheel.m_raycastInfo.m_isInContact = false;

	btTransform chassisTrans = getChassisWorldTransform();
	wheel.m_raycastInfo.m_hardPointWS = chassisTrans(wheel.m_chassisConnectionPointCS);
	wheel.m_raycastInfo.m_wheelDirectionWS = chassisTrans.getBasis() * wheel.m_wheelDirectionCS;
	wheel.m_raycastInfo.m_wheelAxleWS = chassisTrans.getBasis() * wheel.m_wheelAxleCS;
}



float btVehicleRL::rayCast(btWheelInfoRL& wheel) {
	updateWheelTransformsWS(wheel);

	float depth = -1;

	float suspensionTravel = wheel.m_maxSuspensionTravelCm / 100;
	float magicSubtractionNumber = 0.05f; // TODO: Add to RLConst
	float realRayLength = wheel.getSuspensionRestLength() + suspensionTravel + wheel.m_wheelsRadius - magicSubtractionNumber;

	Vec source = wheel.m_raycastInfo.m_hardPointWS;
	Vec target = source + (wheel.m_raycastInfo.m_wheelDirectionWS * realRayLength);
	wheel.m_raycastInfo.m_contactPointWS = target;

	btVehicleRaycaster::btVehicleRaycasterResult rayResults;

	btAssert(m_vehicleRaycaster);

	void* object = m_vehicleRaycaster->castRay(source, target, rayResults);

	wheel.m_raycastInfo.m_groundObject = 0;

	if (object) {
		bool hitGround = abs(rayResults.m_hitPointInWorld.z()) < 0.01f
			&& rayResults.m_hitNormalInWorld.z() > 0.995f;
		if (hitGround)
			rayResults.m_hitPointInWorld.z() = 0; // Make sure ground is at exactly 0

		wheel.m_raycastInfo.m_contactPointWS = rayResults.m_hitPointInWorld;
		float fraction = rayResults.m_distFraction;
		depth = realRayLength * rayResults.m_distFraction;
		wheel.m_raycastInfo.m_contactNormalWS = rayResults.m_hitNormalInWorld;
		wheel.m_raycastInfo.m_isInContact = true;

		wheel.m_raycastInfo.m_groundObject = &getFixedBody();  ///@todo for driving on dynamic/movable objects!;
		//wheel.m_raycastInfo.m_groundObject = object;

		float wheelTraceLenSq = (wheel.m_raycastInfo.m_hardPointWS - wheel.m_raycastInfo.m_contactPointWS).dot(getUpVector());
		wheel.m_raycastInfo.m_suspensionLength = wheelTraceLenSq - wheel.m_wheelsRadius;

		//clamp on max suspension travel
		float minSuspensionLen = wheel.getSuspensionRestLength() - suspensionTravel;
		float maxSuspensionLen = wheel.getSuspensionRestLength() + suspensionTravel;
		wheel.m_raycastInfo.m_suspensionLength =
			std::clamp(
				wheel.m_raycastInfo.m_suspensionLength,
				wheel.getSuspensionRestLength() - suspensionTravel,
				wheel.getSuspensionRestLength() + suspensionTravel
			);

		float denominator = wheel.m_raycastInfo.m_contactNormalWS.dot(wheel.m_raycastInfo.m_wheelDirectionWS);

		Vec chassis_velocity_at_contactPoint;
		Vec relpos = wheel.m_raycastInfo.m_contactPointWS - getRigidBody()->getCenterOfMassPosition();

		chassis_velocity_at_contactPoint = getRigidBody()->getVelocityInLocalPoint(relpos);

		float projVel = wheel.m_raycastInfo.m_contactNormalWS.dot(chassis_velocity_at_contactPoint);

		if (denominator >= 0.1) {
			wheel.m_suspensionRelativeVelocity = 0;
			wheel.m_clippedInvContactDotSuspension = 10;
		} else {
			float inv = -1 / denominator;
			wheel.m_suspensionRelativeVelocity = projVel * inv;
			wheel.m_clippedInvContactDotSuspension = inv;
		}

		{ // Compute m_extraPushback

			// Temporarily disable factors
			// This will prevent resolveSingleCollision from actually applying force to us
			// TODO: This is sorta hacky, we should just make our own version of resolveSingleCollision or something
			m_chassisBody->setLinearFactor({ 0,0,0 });
			m_chassisBody->setAngularFactor({ 0,0,0 });

			float susRayDeltaDist = wheelTraceLenSq - (realRayLength - wheel.m_wheelsRadius);
			
			float collisionResult = resolveSingleCollision(m_chassisBody, (btCollisionObject*)object, rayResults.m_hitPointInWorld, rayResults.m_hitNormalInWorld, m_dynamicsWorld->getSolverInfo(), susRayDeltaDist);
			float pushBackScale = (1 / 1.5f);
			wheel.m_extraPushback = (collisionResult * pushBackScale) / getNumWheels();

			// Restore factors 
			m_chassisBody->setLinearFactor({ 1,1,1 });
			m_chassisBody->setAngularFactor({ 1,1,1 });
		}

	} else {
		wheel.m_raycastInfo.m_suspensionLength = wheel.getSuspensionRestLength() + suspensionTravel;
		wheel.m_suspensionRelativeVelocity = 0;
		wheel.m_raycastInfo.m_contactNormalWS = -wheel.m_raycastInfo.m_wheelDirectionWS;
		wheel.m_clippedInvContactDotSuspension = 1;
		wheel.m_extraPushback = 0;
	}

	return depth;
}

const btTransform& btVehicleRL::getChassisWorldTransform() const {
	return getRigidBody()->getCenterOfMassTransform();
}

void btVehicleRL::updateVehicle(float step) {

	for (int i = 0; i < getNumWheels(); i++)
		updateWheelTransform(i);

	//
	// simulate suspension
	//

	int i = 0;
	for (i = 0; i < m_wheelInfo.size(); i++) {
		//float depth;
		//depth =
		rayCast(m_wheelInfo[i]);
	}

	calcFrictionImpulses(step);

	updateSuspension(step);
	applyFrictionImpulses(step);
}

void btVehicleRL::setSteeringValue(float steering, int wheel) {
	btAssert(wheel >= 0 && wheel < getNumWheels());

	btWheelInfoRL& wheelInfo = getWheelInfo(wheel);
	wheelInfo.m_steering = steering;
}

float btVehicleRL::getSteeringValue(int wheel) const {
	return getWheelInfo(wheel).m_steering;
}

void btVehicleRL::applyEngineForce(float force, int wheel) {
	btAssert(wheel >= 0 && wheel < getNumWheels());
	btWheelInfoRL& wheelInfo = getWheelInfo(wheel);
	wheelInfo.m_engineForce = force;
}

const btWheelInfoRL& btVehicleRL::getWheelInfo(int index) const {
	btAssert((index >= 0) && (index < getNumWheels()));

	return m_wheelInfo[index];
}

btWheelInfoRL& btVehicleRL::getWheelInfo(int index) {
	btAssert((index >= 0) && (index < getNumWheels()));

	return m_wheelInfo[index];
}

void btVehicleRL::setBrake(float brake, int wheelIndex) {
	btAssert((wheelIndex >= 0) && (wheelIndex < getNumWheels()));
	getWheelInfo(wheelIndex).m_brake = brake;
}

void btVehicleRL::updateSuspension(float deltaTime) {
	float chassisMass = 1 / m_chassisBody->getInvMass();

	for (int i = 0; i < getNumWheels(); i++) {
		btWheelInfoRL& wheel_info = m_wheelInfo[i];

		if (wheel_info.m_raycastInfo.m_isInContact) {
			float force =
				(wheel_info.getSuspensionRestLength() - wheel_info.m_raycastInfo.m_suspensionLength)
				* wheel_info.m_suspensionStiffness * wheel_info.m_clippedInvContactDotSuspension;
			
			float dampingVelScale = (wheel_info.m_suspensionRelativeVelocity < 0) ? wheel_info.m_wheelsDampingCompression : wheel_info.m_wheelsDampingRelaxation;

			wheel_info.m_wheelsSuspensionForce = force - (dampingVelScale * wheel_info.m_suspensionRelativeVelocity);
			wheel_info.m_wheelsSuspensionForce *= wheel_info.m_suspensionForceScale;
			
			// RL never uses downwards suspension forces
			if (wheel_info.m_wheelsSuspensionForce < 0)
				wheel_info.m_wheelsSuspensionForce = 0;

		} else {
			wheel_info.m_wheelsSuspensionForce = 0;
		}
	}

	for (int i = 0; i < getNumWheels(); i++) {
		//apply suspension force
		btWheelInfoRL& wheel = m_wheelInfo[i];
		if (wheel.m_wheelsSuspensionForce != 0) {
			Vec contactPointOffset = wheel.m_raycastInfo.m_contactPointWS - getRigidBody()->getCenterOfMassPosition();
			float baseForceScale = (wheel.m_wheelsSuspensionForce * deltaTime) + wheel.m_extraPushback;
			Vec force = wheel.m_raycastInfo.m_contactNormalWS * baseForceScale;
			m_chassisBody->applyImpulse(force, contactPointOffset);
		}
	}
}

void btVehicleRL::calcFrictionImpulses(float timeStep) {

	float frictionScale = m_chassisBody->getMass() / 3;

	// Determine impulses
	for (int i = 0; i < 4; i++) {
		btWheelInfoRL& wheel = m_wheelInfo[i];

		btRigidBody* groundObject = (btRigidBody*)wheel.m_raycastInfo.m_groundObject;
		if (groundObject) {
			// Axle direction (includes steering turn)
			btVector3 axleDir = wheel.m_worldTransform.getBasis().getColumn(m_indexRightAxis);

			btVector3 surfNormalWS = wheel.m_raycastInfo.m_contactNormalWS;
			float proj = axleDir.dot(surfNormalWS);
			axleDir -= surfNormalWS * proj;
			axleDir = axleDir.normalized();

			// Wheel forwards direction
			btVector3 forwardDir = surfNormalWS.cross(axleDir).normalized();

			float sideImpulse;

			// Get sideways friction force
			resolveSingleBilateral(
				*m_chassisBody, wheel.m_raycastInfo.m_contactPointWS,
				*groundObject, wheel.m_raycastInfo.m_contactPointWS,
				0,
				axleDir,
				sideImpulse,
				timeStep
			);

			float rollingFriction;
			if (wheel.m_engineForce == 0) {
				if (wheel.m_brake) {
					// Simplified variation of calcRollingFriction()
					btVector3 contactPoint = wheel.m_raycastInfo.m_contactPointWS;
					btVector3 carRelContactPoint = contactPoint - m_chassisBody->getCenterOfMassPosition();
					btVector3 groundObRelContactPoint = contactPoint - groundObject->getCenterOfMassPosition();

					btVector3
						v1 = m_chassisBody->getVelocityInLocalPoint(carRelContactPoint),
						v2 = groundObject->getVelocityInLocalPoint(carRelContactPoint);
					btVector3 contactVel = v1 - v2;
					float relVel = contactVel.dot(forwardDir);

					// TODO: No idea where this number comes from or how it was calculated lol
					constexpr float ROLLING_FRICTION_SCALE_MAGIC = 113.73963f;

					rollingFriction = CLAMP(-relVel * ROLLING_FRICTION_SCALE_MAGIC, -wheel.m_brake, wheel.m_brake);
				} else {
					// Don't apply friction when driving with no brake
					rollingFriction = 0;
				}
			} else {
				// Engine force already accounts for our mass, so we will cancel out the friction scale multiplication at the end
				rollingFriction = -wheel.m_engineForce / frictionScale;
			}

			btVector3 totalFrictionForce = (forwardDir * rollingFriction * wheel.m_longFriction) + (axleDir * sideImpulse * wheel.m_latFriction);
			wheel.m_impulse = totalFrictionForce * frictionScale;
		} else {
			wheel.m_impulse = { 0,0,0 };
		}
	}
}

void btVehicleRL::applyFrictionImpulses(float timeStep) {
	// Apply impulses
	btVector3 upDir = m_chassisBody->getWorldTransform().getBasis().getColumn(m_indexUpAxis);
	for (int i = 0; i < 4; i++) {
		btWheelInfoRL& wheel = m_wheelInfo[i];
		{
			btVector3 wheelContactOffset = wheel.m_raycastInfo.m_contactPointWS - m_chassisBody->getWorldTransform().getOrigin();
			float contactUpDot = upDir.dot(wheelContactOffset);
			btVector3 wheelRelPos = wheelContactOffset - upDir * contactUpDot;
			m_chassisBody->applyImpulse(wheel.m_impulse * timeStep, wheelRelPos);
		}
	}
}

Vec btVehicleRL::getDownwardsDirFromWheelContacts() {
	Vec sumContactDir = Vec(0, 0, 0);
	for (int i = 0; i < 4; i++)
		if (m_wheelInfo[i].m_isInContactWithWorld)
			sumContactDir += m_wheelInfo[i].m_raycastInfo.m_contactNormalWS;

	if (sumContactDir == Vec(0, 0, 0)) {
		// No wheels had world contact, just return downwards direction
		return -this->getUpVector();
	} else {
		// Average normal of contact
		return sumContactDir.normalized();
	}
}

float btVehicleRL::getForwardSpeed() {
	return m_chassisBody->getLinearVelocity().dot(getForwardVector());
}
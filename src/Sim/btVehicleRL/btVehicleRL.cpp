#include "btVehicleRL.h"
#include "../../RLConst.h"
#define ROLLING_INFLUENCE_FIX

#include "../SuspensionCollisionGrid/SuspensionCollisionGrid.h"

#include "../../../libsrc/bullet3-3.24/BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "../../../libsrc/bullet3-3.24/BulletDynamics/ConstraintSolver/btContactConstraint.h"

RS_NS_START

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
btWheelInfoRL& btVehicleRL::addWheel(const btVector3& connectionPointCS, const btVector3& wheelDirectionCS0, const btVector3& wheelAxleCS, float suspensionRestLength, float wheelRadius, const btVehicleTuning& tuning, bool isFrontWheel) {
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

// See: I20 or I21
void btVehicleRL::updateWheelTransform(int wheelIndex) {
	btWheelInfoRL& wheel = m_wheelInfo[wheelIndex];
	updateWheelTransformsWS(wheel);
	btVector3 up = -wheel.m_raycastInfo.m_wheelDirectionWS;
	const btVector3& right = wheel.m_raycastInfo.m_wheelAxleWS;
	btVector3 fwd = up.cross(right);
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

// See: I20 or I21
void btVehicleRL::updateWheelTransformsWS(btWheelInfoRL& wheel) {
	wheel.m_raycastInfo.m_isInContact = false;
	wheel.m_isInContactWithWorld = false;

	btTransform chassisTrans = getChassisWorldTransform();
	wheel.m_raycastInfo.m_hardPointWS = chassisTrans(wheel.m_chassisConnectionPointCS);
	wheel.m_raycastInfo.m_wheelDirectionWS = chassisTrans.getBasis() * wheel.m_wheelDirectionCS;
	wheel.m_raycastInfo.m_wheelAxleWS = chassisTrans.getBasis() * wheel.m_wheelAxleCS;
}

float btVehicleRL::rayCast(btWheelInfoRL& wheel, SuspensionCollisionGrid* grid) {
	updateWheelTransformsWS(wheel);

	float depth = -1;

	float suspensionTravel = wheel.m_maxSuspensionTravelCm / 100;
	float realRayLength = wheel.getSuspensionRestLength() + suspensionTravel + wheel.m_wheelsRadius - RLConst::BTVehicle::SUSPENSION_SUBTRACTION;

	// See: I21
	btVector3 source = wheel.m_raycastInfo.m_hardPointWS;
	btVector3 target = source + (wheel.m_raycastInfo.m_wheelDirectionWS * realRayLength);
	wheel.m_raycastInfo.m_contactPointWS = target;
	wheel.m_raycastInfo.m_groundObject = NULL;

	// See: I22
	btVehicleRaycaster::btVehicleRaycasterResult rayResults;
	
	btAssert(m_vehicleRaycaster);
	btCollisionObject* object;
	if (grid) {
		object = grid->CastSuspensionRay(m_vehicleRaycaster, source, target, m_chassisBody, rayResults);
	} else {
		object = (btCollisionObject*)m_vehicleRaycaster->castRay(source, target, m_chassisBody, rayResults);
	}

	// See: I23
	if (object) {
		wheel.m_raycastInfo.m_contactPointWS = rayResults.m_hitPointInWorld;
		float fraction = rayResults.m_distFraction;
		depth = realRayLength * rayResults.m_distFraction;
		wheel.m_raycastInfo.m_contactNormalWS = rayResults.m_hitNormalInWorld;
		wheel.m_raycastInfo.m_isInContact = true;
		wheel.m_isInContactWithWorld = object->isStaticObject();

		wheel.m_raycastInfo.m_groundObject = object;

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

		float denominator = wheel.m_raycastInfo.m_contactNormalWS.dot(getUpVector());

		btVector3 relpos = wheel.m_raycastInfo.m_contactPointWS - m_chassisBody->getWorldTransform().m_origin;
		wheel.m_velAtContactPoint = m_chassisBody->getVelocityInLocalPoint(relpos);

		float projVel = wheel.m_raycastInfo.m_contactNormalWS.dot(wheel.m_velAtContactPoint);

		if (denominator > 0.1) {
			float inv = 1 / denominator;
			wheel.m_suspensionRelativeVelocity = projVel * inv;
			wheel.m_clippedInvContactDotSuspension = inv;
		} else {
			// Denominator is too tiny to be meaningful
			wheel.m_suspensionRelativeVelocity = 0;
			wheel.m_clippedInvContactDotSuspension = 10;
		}

		if (object->isStaticObject()) { // Compute m_extraPushback when colliding with static object
			float rayPushbackThresh = (wheel.m_suspensionRestLength1 + wheel.m_wheelsRadius) - RLConst::BTVehicle::SUSPENSION_SUBTRACTION;
			if (wheelTraceLenSq < rayPushbackThresh) {

				float wheelTraceDistDelta = wheelTraceLenSq - rayPushbackThresh;
				float collisionResult = resolveSingleCollision(
					m_chassisBody,
					object,
					rayResults.m_hitPointInWorld,
					rayResults.m_hitNormalInWorld,
					m_dynamicsWorld->getSolverInfo(),
					wheelTraceDistDelta,
					false
				);

				wheel.m_extraPushback = collisionResult / getNumWheels();
			}
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

void btVehicleRL::updateVehicleFirst(float step, SuspensionCollisionGrid* grid) {

	for (int i = 0; i < getNumWheels(); i++)
		updateWheelTransform(i);

	//
	// simulate suspension
	//

	int i = 0;
	for (i = 0; i < m_wheelInfo.size(); i++) {
		//float depth;
		//depth =
		rayCast(m_wheelInfo[i], grid);
	}

	calcFrictionImpulses(step);
}

void btVehicleRL::updateVehicleSecond(float step) {
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

// See: I24
void btVehicleRL::updateSuspension(float deltaTime) {
	
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
		btWheelInfoRL& wheel = m_wheelInfo[i];
		if (wheel.m_wheelsSuspensionForce != 0) {
			btVector3 contactPointOffset = wheel.m_raycastInfo.m_contactPointWS - getRigidBody()->getCenterOfMassPosition();
			float baseForceScale = (wheel.m_wheelsSuspensionForce * deltaTime) + wheel.m_extraPushback;
			btVector3 force = wheel.m_raycastInfo.m_contactNormalWS * baseForceScale;
			m_chassisBody->applyImpulse(force, contactPointOffset);
		}
	}
}

// See: I25
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
			axleDir = axleDir.safeNormalized();

			// Wheel forwards direction
			btVector3 forwardDir = surfNormalWS.cross(axleDir).safeNormalized();

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

					// This will round off small rolling friction amounts when at sub-80 TPS to prevent stuttering (to an extent)
					// TODO: Improve and clarify
					if (timeStep > (1 / 80.f)) {
						float threshold = -(1 / (timeStep * 150.f)) + 0.8f;
						if (abs(relVel) < threshold)
							relVel = 0;
					}

					// TODO: No idea where this number comes from or how it was calculated lol
					constexpr float ROLLING_FRICTION_SCALE_MAGIC = 113.73963f;

					rollingFriction = RS_CLAMP(-relVel * ROLLING_FRICTION_SCALE_MAGIC, -wheel.m_brake, wheel.m_brake);
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

// See: I25
void btVehicleRL::applyFrictionImpulses(float timeStep) {
	// Apply impulses
	btVector3 upDir = m_chassisBody->getWorldTransform().getBasis().getColumn(m_indexUpAxis);
	for (int i = 0; i < 4; i++) {
		btWheelInfoRL& wheel = m_wheelInfo[i];
		if (!wheel.m_impulse.isZero()) {
			btVector3 wheelContactOffset = wheel.m_raycastInfo.m_contactPointWS - m_chassisBody->getWorldTransform().getOrigin();
			float contactUpDot = upDir.dot(wheelContactOffset);
			btVector3 wheelRelPos = wheelContactOffset - upDir * contactUpDot;
			m_chassisBody->applyImpulse(wheel.m_impulse * timeStep, wheelRelPos);
		}
	}
}

btVector3 btVehicleRL::getUpwardsDirFromWheelContacts() {
	btVector3 sumContactDir = btVector3(0, 0, 0);
	for (int i = 0; i < 4; i++)
		if (m_wheelInfo[i].m_raycastInfo.m_isInContact)
			sumContactDir += m_wheelInfo[i].m_raycastInfo.m_contactNormalWS;

	if (sumContactDir == btVector3(0, 0, 0)) {
		// No wheels had world contact, just return basic upward direction
		return this->getUpVector();
	} else {
		// Average normal of contact
		return sumContactDir.safeNormalized();
	}
}

float btVehicleRL::getForwardSpeed() {
	return m_chassisBody->getLinearVelocity().dot(getForwardVector());
}

RS_NS_END
#include "btVehicleRL.h"
#include "../../../libsrc/bullet3-3.24/BulletDynamics/ConstraintSolver/btContactConstraint.h"
#define ROLLING_INFLUENCE_FIX

btVehicleRL::btVehicleRL(const btVehicleTuning& tuning, btRigidBody* chassis, btVehicleRaycaster* raycaster)
	: m_vehicleRaycaster(raycaster),
	m_pitchControl(0) {
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
btWheelInfoRL& btVehicleRL::addWheel(const Vec& connectionPointCS, const Vec& wheelDirectionCS0, const Vec& wheelAxleCS, btScalar suspensionRestLength, btScalar wheelRadius, const btVehicleTuning& tuning, bool isFrontWheel) {
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

	//rotate around steering over de wheelAxleWS
	btScalar steering = wheel.m_steering;

	btQuaternion steeringOrn(up, steering);  //wheel.m_steering);
	btMatrix3x3 steeringMat(steeringOrn);

	btQuaternion rotatingOrn(right, -wheel.m_rotation);
	btMatrix3x3 rotatingMat(rotatingOrn);

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

	wheel.m_worldTransform.setBasis(steeringMat * rotatingMat * basis2);
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
		//wheel_info.setContactFriction(0);
		wheel.m_clippedInvContactDotSuspension = 1;
	}
}

void btVehicleRL::updateWheelTransformsWS(btWheelInfoRL& wheel) {
	wheel.m_raycastInfo.m_isInContact = false;

	btTransform chassisTrans = getChassisWorldTransform();
	wheel.m_raycastInfo.m_hardPointWS = chassisTrans(wheel.m_chassisConnectionPointCS);
	wheel.m_raycastInfo.m_wheelDirectionWS = chassisTrans.getBasis() * wheel.m_wheelDirectionCS;
	wheel.m_raycastInfo.m_wheelAxleWS = chassisTrans.getBasis() * wheel.m_wheelAxleCS;
}

btScalar btVehicleRL::rayCast(btWheelInfoRL& wheel) {
	updateWheelTransformsWS(wheel);

	btScalar depth = -1;

	btScalar suspensionTravel = wheel.m_maxSuspensionTravelCm / 100;
	btScalar rayLength = wheel.getSuspensionRestLength()/* + suspensionTravel*/ + wheel.m_wheelsRadius;

	Vec source = wheel.m_raycastInfo.m_hardPointWS;
	Vec target = source + (wheel.m_raycastInfo.m_wheelDirectionWS * rayLength);
	wheel.m_raycastInfo.m_contactPointWS = target;

	btVehicleRaycaster::btVehicleRaycasterResult rayResults;

	btAssert(m_vehicleRaycaster);

	void* object = m_vehicleRaycaster->castRay(source, target, rayResults);

	wheel.m_raycastInfo.m_groundObject = 0;

	if (object) {
		btScalar fraction = rayResults.m_distFraction;
		depth = rayLength * rayResults.m_distFraction;
		wheel.m_raycastInfo.m_contactNormalWS = rayResults.m_hitNormalInWorld;
		wheel.m_raycastInfo.m_isInContact = true;

		wheel.m_raycastInfo.m_groundObject = &getFixedBody();  ///@todo for driving on dynamic/movable objects!;
		//wheel.m_raycastInfo.m_groundObject = object;

		btScalar hitDistance = fraction * rayLength;
		wheel.m_raycastInfo.m_suspensionLength = hitDistance - wheel.m_wheelsRadius;
		//clamp on max suspension travel

		btScalar minSuspensionLen = wheel.getSuspensionRestLength() - suspensionTravel;
		btScalar maxSuspensionLen = wheel.getSuspensionRestLength() + suspensionTravel;
		wheel.m_raycastInfo.m_suspensionLength =
			std::clamp(
				wheel.m_raycastInfo.m_suspensionLength,
				wheel.getSuspensionRestLength() - suspensionTravel,
				wheel.getSuspensionRestLength() + suspensionTravel
			);

		wheel.m_raycastInfo.m_contactPointWS = rayResults.m_hitPointInWorld;

		btScalar denominator = wheel.m_raycastInfo.m_contactNormalWS.dot(wheel.m_raycastInfo.m_wheelDirectionWS);

		Vec chassis_velocity_at_contactPoint;
		Vec relpos = wheel.m_raycastInfo.m_contactPointWS - getRigidBody()->getCenterOfMassPosition();

		chassis_velocity_at_contactPoint = getRigidBody()->getVelocityInLocalPoint(relpos);

		btScalar projVel = wheel.m_raycastInfo.m_contactNormalWS.dot(chassis_velocity_at_contactPoint);

		if (denominator >= 0.1) {
			wheel.m_suspensionRelativeVelocity = 0;
			wheel.m_clippedInvContactDotSuspension = 10;
		} else {
			btScalar inv = -1 / denominator;
			wheel.m_suspensionRelativeVelocity = projVel * inv;
			wheel.m_clippedInvContactDotSuspension = inv;
		}
	} else {
		//put wheel info as in rest position
		wheel.m_raycastInfo.m_suspensionLength = wheel.getSuspensionRestLength();
		wheel.m_suspensionRelativeVelocity = 0;
		wheel.m_raycastInfo.m_contactNormalWS = -wheel.m_raycastInfo.m_wheelDirectionWS;
		wheel.m_clippedInvContactDotSuspension = 1;
	}

	return depth;
}

const btTransform& btVehicleRL::getChassisWorldTransform() const {
	return getRigidBody()->getCenterOfMassTransform();
}

void btVehicleRL::updateVehicle(btScalar step) {
	for (int i = 0; i < getNumWheels(); i++)
		updateWheelTransform(i);

	//
	// simulate suspension
	//

	int i = 0;
	for (i = 0; i < m_wheelInfo.size(); i++) {
		//btScalar depth;
		//depth =
		rayCast(m_wheelInfo[i]);
	}

	updateSuspension(step);

	for (i = 0; i < m_wheelInfo.size(); i++) {
		//apply suspension force
		btWheelInfoRL& wheel = m_wheelInfo[i];

		btScalar suspensionForce = wheel.m_wheelsSuspensionForce;

		if (suspensionForce > wheel.m_maxSuspensionForce) {
			suspensionForce = wheel.m_maxSuspensionForce;
		}
		Vec impulse = wheel.m_raycastInfo.m_contactNormalWS * suspensionForce * step;
		Vec relpos = wheel.m_raycastInfo.m_contactPointWS - getRigidBody()->getCenterOfMassPosition();

		getRigidBody()->applyImpulse(impulse, relpos);
	}

	updateFriction(step);
}

void btVehicleRL::setSteeringValue(btScalar steering, int wheel) {
	btAssert(wheel >= 0 && wheel < getNumWheels());

	btWheelInfoRL& wheelInfo = getWheelInfo(wheel);
	wheelInfo.m_steering = steering;
}

btScalar btVehicleRL::getSteeringValue(int wheel) const {
	return getWheelInfo(wheel).m_steering;
}

void btVehicleRL::applyEngineForce(btScalar force, int wheel) {
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

void btVehicleRL::setBrake(btScalar brake, int wheelIndex) {
	btAssert((wheelIndex >= 0) && (wheelIndex < getNumWheels()));
	getWheelInfo(wheelIndex).m_brake = brake;
}

void btVehicleRL::updateSuspension(btScalar deltaTime) {
	btScalar chassisMass = 1 / m_chassisBody->getInvMass();

	for (int i = 0; i < getNumWheels(); i++) {
		btWheelInfoRL& wheel_info = m_wheelInfo[i];

		if (wheel_info.m_raycastInfo.m_isInContact) {
			btScalar force =
				(wheel_info.getSuspensionRestLength() - wheel_info.m_raycastInfo.m_suspensionLength)
				* wheel_info.m_suspensionStiffness;

			btScalar dampingVelScale = (wheel_info.m_suspensionRelativeVelocity < 0) ? wheel_info.m_wheelsDampingCompression : wheel_info.m_wheelsDampingRelaxation;

			wheel_info.m_wheelsSuspensionForce = force - wheel_info.m_suspensionRelativeVelocity * dampingVelScale;

			// RL never uses downwards suspension forces
			if (wheel_info.m_wheelsSuspensionForce < 0)
				wheel_info.m_wheelsSuspensionForce = 0;

		} else {
			wheel_info.m_wheelsSuspensionForce = 0;
		}
	}
}

struct btWheelContactPoint
{
	btRigidBody* m_body0;
	btRigidBody* m_body1;
	Vec m_frictionPositionWorld;
	Vec m_frictionDirectionWorld;
	btScalar m_jacDiagABInv;
	btScalar m_maxImpulse;

	btWheelContactPoint(btRigidBody* body0, btRigidBody* body1, const Vec& frictionPosWorld, const Vec& frictionDirectionWorld, btScalar maxImpulse)
		: m_body0(body0),
		m_body1(body1),
		m_frictionPositionWorld(frictionPosWorld),
		m_frictionDirectionWorld(frictionDirectionWorld),
		m_maxImpulse(maxImpulse) {
		btScalar denom0 = body0->computeImpulseDenominator(frictionPosWorld, frictionDirectionWorld);
		btScalar denom1 = body1->computeImpulseDenominator(frictionPosWorld, frictionDirectionWorld);
		btScalar relaxation = 1.f;
		m_jacDiagABInv = relaxation / (denom0 + denom1);
	}
};

btScalar _calcRollingFriction(btWheelContactPoint& contactPoint, int numWheelsOnGround) {
	btScalar j1 = 0.f;

	const Vec& contactPosWorld = contactPoint.m_frictionPositionWorld;

	Vec rel_pos1 = contactPosWorld - contactPoint.m_body0->getCenterOfMassPosition();
	Vec rel_pos2 = contactPosWorld - contactPoint.m_body1->getCenterOfMassPosition();

	btScalar maxImpulse = contactPoint.m_maxImpulse;

	Vec vel1 = contactPoint.m_body0->getVelocityInLocalPoint(rel_pos1);
	Vec vel2 = contactPoint.m_body1->getVelocityInLocalPoint(rel_pos2);
	Vec vel = vel1 - vel2;

	btScalar vrel = contactPoint.m_frictionDirectionWorld.dot(vel);

	// calculate j that moves us to zero relative velocity
	j1 = -vrel * contactPoint.m_jacDiagABInv / btScalar(numWheelsOnGround);
	btSetMin(j1, maxImpulse);
	btSetMax(j1, -maxImpulse);

	return j1;
}

//btScalar sideFrictionStiffness2 = 1;
void btVehicleRL::updateFriction(btScalar timeStep) {
	//calculate the impulse, so that the wheels don't move sidewards
	int numWheel = getNumWheels();
	if (!numWheel)
		return;

	m_forwardWS.resize(numWheel);
	m_axle.resize(numWheel);
	m_forwardImpulse.resize(numWheel);
	m_sideImpulse.resize(numWheel);

	int numWheelsOnGround = 0;

	//collapse all those loops into one!
	for (int i = 0; i < getNumWheels(); i++) {
		btWheelInfoRL& wheelInfo = m_wheelInfo[i];
		class btRigidBody* groundObject = (class btRigidBody*)wheelInfo.m_raycastInfo.m_groundObject;
		if (groundObject)
			numWheelsOnGround++;
		m_sideImpulse[i] = 0;
		m_forwardImpulse[i] = 0;
	}

	{
		for (int i = 0; i < getNumWheels(); i++) {
			btWheelInfoRL& wheelInfo = m_wheelInfo[i];

			class btRigidBody* groundObject = (class btRigidBody*)wheelInfo.m_raycastInfo.m_groundObject;

			if (groundObject) {
				const btTransform& wheelTrans = getWheelTransformWS(i);

				btMatrix3x3 wheelBasis0 = wheelTrans.getBasis();
				m_axle[i] = -Vec(
					wheelBasis0[0][m_indexRightAxis],
					wheelBasis0[1][m_indexRightAxis],
					wheelBasis0[2][m_indexRightAxis]);

				const Vec& surfNormalWS = wheelInfo.m_raycastInfo.m_contactNormalWS;
				btScalar proj = m_axle[i].dot(surfNormalWS);
				m_axle[i] -= surfNormalWS * proj;
				m_axle[i] = m_axle[i].normalize();

				m_forwardWS[i] = surfNormalWS.cross(m_axle[i]);
				m_forwardWS[i].normalize();

				resolveSingleBilateral(*m_chassisBody, wheelInfo.m_raycastInfo.m_contactPointWS,
					*groundObject, wheelInfo.m_raycastInfo.m_contactPointWS,
					0, m_axle[i], m_sideImpulse[i], timeStep);

				m_sideImpulse[i] *= 1;
			}
		}
	}

	btScalar sideFactor = 1;
	btScalar fwdFactor = 0.5;

	bool sliding = false;
	{
		for (int wheel = 0; wheel < getNumWheels(); wheel++) {
			btWheelInfoRL& wheelInfo = m_wheelInfo[wheel];
			class btRigidBody* groundObject = (class btRigidBody*)wheelInfo.m_raycastInfo.m_groundObject;

			btScalar rollingFriction = 0.f;

			if (groundObject) {
				if (wheelInfo.m_engineForce != 0.f) {
					rollingFriction = wheelInfo.m_engineForce * timeStep;
				} else {
					btScalar defaultRollingFrictionImpulse = 0.f;
					btScalar maxImpulse = wheelInfo.m_brake ? wheelInfo.m_brake : defaultRollingFrictionImpulse;
					btWheelContactPoint contactPt(m_chassisBody, groundObject, wheelInfo.m_raycastInfo.m_contactPointWS, m_forwardWS[wheel], maxImpulse);
					btAssert(numWheelsOnGround > 0);
					rollingFriction = _calcRollingFriction(contactPt, numWheelsOnGround);
				}
			}

			//switch between active rolling (throttle), braking and non-active rolling friction (no throttle/break)

			m_forwardImpulse[wheel] = 0;
			m_wheelInfo[wheel].m_skidInfo = 1;

			if (groundObject) {
				m_wheelInfo[wheel].m_skidInfo = 1;

				btScalar maximp = wheelInfo.m_wheelsSuspensionForce * timeStep * wheelInfo.m_frictionSlip;
				btScalar maximpSide = maximp;

				btScalar maximpSquared = maximp * maximpSide;

				m_forwardImpulse[wheel] = rollingFriction;  //wheelInfo.m_engineForce* timeStep;

				btScalar x = (m_forwardImpulse[wheel]) * fwdFactor;
				btScalar y = (m_sideImpulse[wheel]) * sideFactor;

				btScalar impulseSquared = (x * x + y * y);

				if (impulseSquared > maximpSquared) {
					sliding = true;

					btScalar factor = maximp / btSqrt(impulseSquared);

					m_wheelInfo[wheel].m_skidInfo *= factor;
				}
			}
		}
	}

	if (sliding) {
		for (int wheel = 0; wheel < getNumWheels(); wheel++) {
			if (m_sideImpulse[wheel] != 0) {
				if (m_wheelInfo[wheel].m_skidInfo < 1) {
					m_forwardImpulse[wheel] *= m_wheelInfo[wheel].m_skidInfo;
					m_sideImpulse[wheel] *= m_wheelInfo[wheel].m_skidInfo;
				}
			}
		}
	}

	// apply the impulses
	{
		for (int wheel = 0; wheel < getNumWheels(); wheel++) {
			btWheelInfoRL& wheelInfo = m_wheelInfo[wheel];

			Vec rel_pos = wheelInfo.m_raycastInfo.m_contactPointWS -
				m_chassisBody->getCenterOfMassPosition();

			if (m_forwardImpulse[wheel] != 0) {
				m_chassisBody->applyImpulse(m_forwardWS[wheel] * (m_forwardImpulse[wheel]), rel_pos);
			}
			if (m_sideImpulse[wheel] != 0) {
				class btRigidBody* groundObject = (class btRigidBody*)m_wheelInfo[wheel].m_raycastInfo.m_groundObject;

				Vec rel_pos2 = wheelInfo.m_raycastInfo.m_contactPointWS -
					groundObject->getCenterOfMassPosition();

				Vec sideImp = m_axle[wheel] * m_sideImpulse[wheel];

#if defined ROLLING_INFLUENCE_FIX  // fix. It only worked if car's up was along Y - VT.
				Vec vChassisWorldUp = getRigidBody()->getCenterOfMassTransform().getBasis().getColumn(m_indexUpAxis);
				rel_pos -= vChassisWorldUp * (vChassisWorldUp.dot(rel_pos) * (1.f - wheelInfo.m_rollInfluence));
#else
				rel_pos[m_indexUpAxis] *= wheelInfo.m_rollInfluence;
#endif
				m_chassisBody->applyImpulse(sideImp, rel_pos);

				//apply friction impulse on the ground
				groundObject->applyImpulse(-sideImp, rel_pos2);
			}
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
	return (m_chassisBody->getLinearVelocity() * getForwardVector()).norm();
}
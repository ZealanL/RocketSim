#include "btVehicleRL.h"
#include "../../RLConst.h"
#define ROLLING_INFLUENCE_FIX

#include "../../../libsrc/bullet3-3.24/BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "../../../libsrc/bullet3-3.24/BulletDynamics/ConstraintSolver/btContactConstraint.h"
#include "../../../libsrc/bullet3-3.24/BulletCollision/CollisionShapes/btSphereShape.h"
#include "../../../libsrc/bullet3-3.24/BulletCollision/CollisionShapes/btCompoundShape.h"
#include "../../../libsrc/bullet3-3.24/BulletCollision/CollisionShapes/btBoxShape.h"
#include "../../BulletLink.h"

RS_NS_START

btVehicleRL::btVehicleRL(
	const btVehicleTuning& tuning, 
	btRigidBody* chassis, btVehicleRaycaster* raycaster, btDynamicsWorld* world, 
	int addedRayCollisionMask
) : m_vehicleRaycaster(raycaster), m_pitchControl(0),  m_dynamicsWorld(world) {
	m_chassisBody = chassis;
	m_indexRightAxis = 0;
	m_indexUpAxis = 2;
	m_indexForwardAxis = 1;
	m_vehicleRaycaster->addedFilterMask = addedRayCollisionMask;
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

// The game resolves the wheel pushback with a 1/60 s solver timestep
static btContactSolverInfo PushbackSolverInfo(const btContactSolverInfo& worldInfo) {
	btContactSolverInfo info = worldInfo;
	info.m_timeStep = 1.f / 60.f;
	return info;
}

// The fixed body the wheel side impulse is solved against (btActionInterface::getFixedBody() writes its shared body on
// every call, a data race between arenas on different threads)
static btRigidBody& GetFixedBody() {
	static btRigidBody fixed(0, 0, 0);
	static const bool massSet = (fixed.setMassProps(btScalar(0.), btVector3(btScalar(0.), btScalar(0.), btScalar(0.))), true);
	(void)massSet;
	return fixed;
}

// The game's wheel ray against a sphere: in the sphere's local frame, the nearest root of |src + dir t|^2 = r^2 in
// [0, dist] (else the far root). Returns false on a miss
static bool RaySphere(const btVector3& rayFrom, const btVector3& rayTo, const btTransform& sphereTrans, float radius,
	float& outFraction, btVector3& outNormal) {
	const btMatrix3x3& basis = sphereTrans.getBasis();
	btVector3 src = (rayFrom - sphereTrans.getOrigin()) * basis; // basis^T * v: the local frame
	btVector3 tgt = (rayTo - sphereTrans.getOrigin()) * basis;
	float radiusSq = radius * radius;

	btVector3 closest(
		RS_CLAMP(0.f, RS_MIN(src.x(), tgt.x()), RS_MAX(src.x(), tgt.x())),
		RS_CLAMP(0.f, RS_MIN(src.y(), tgt.y()), RS_MAX(src.y(), tgt.y())),
		RS_CLAMP(0.f, RS_MIN(src.z(), tgt.z()), RS_MAX(src.z(), tgt.z())));
	if (!(closest.dot(closest) <= radiusSq))
		return false;

	btVector3 delta = tgt - src;
	float dist = sqrtf(delta.dot(delta));
	btVector3 dir = btVector3(delta.x() / dist, delta.y() / dist, delta.z() / dist);
	float b = 2.f * src.dot(dir);
	float c = src.dot(src) - radiusSq;
	float disc = b * b - 4.f * c;
	if (!(disc >= 0.f))
		return false;

	float sqrtDisc = sqrtf(disc);
	float t0 = (-b - sqrtDisc) / 2.f, t1 = (-b + sqrtDisc) / 2.f;
	if (t0 > t1)
		std::swap(t0, t1);
	bool t0Nearest = t0 >= 0.f && t0 <= dist;
	bool t1Nearest = t1 >= 0.f && t1 <= dist && !t0Nearest;
	if (!t0Nearest && !t1Nearest)
		return false;

	float t = t0Nearest ? t0 : t1;
	btVector3 hitLocal = src + dir * t;
	outNormal = basis * btVector3(hitLocal.x() / radius, hitLocal.y() / radius, hitLocal.z() / radius);
	outFraction = t / dist;
	return true;
}

// The game's wheel ray against a car's hitbox, in the child box's frame: a slab test against the box (inner half extents +
// margin), the face normal when the entry point is inside the face's inner rectangle, else the exact rounded box
// (radius = the box margin). Returns false on a miss
static bool RayCarBox(const btVector3& rayFrom, const btVector3& rayTo, const btTransform& carTrans,
	const btVector3& childOffset, const btVector3& innerHalf, float margin, float& outFraction, btVector3& outNormal) {
	constexpr float FLT_EPS = 1.1920928955078125e-7f;
	const btMatrix3x3& basis = carTrans.getBasis();
	const btVector3 srcV = (rayFrom - carTrans.getOrigin()) * basis; // basis^T * v
	const btVector3 tgtV = (rayTo - carTrans.getOrigin()) * basis;
	const btVector3 deltaV = tgtV - srcV;
	const float dist = sqrtf(deltaV.dot(deltaV));
	if (!(dist > 0.0f))
		return false;
	const float src[3] = { srcV.x(), srcV.y(), srcV.z() };
	const float dir[3] = { deltaV.x() / dist, deltaV.y() / dist, deltaV.z() / dist };
	const float inner[3] = { innerHalf.x(), innerHalf.y(), innerHalf.z() };
	const float off[3] = { childOffset.x(), childOffset.y(), childOffset.z() };

	float tenter = 0.0f, texit = dist;
	int hitAxis = 0;
	for (int axis = 0; axis < 3; axis++) {
		const float inv = 1.0f / dir[axis];
		const float bmin = off[axis] - (inner[axis] + margin);
		const float bmax = off[axis] + (inner[axis] + margin);
		if (!(inv - inv == 0.0f)) { // not finite
			if (bmin > src[axis] || src[axis] > bmax)
				return false;
			continue;
		}
		const float t1 = (bmin - src[axis]) * inv;
		const float t2 = (bmax - src[axis]) * inv;
		const float tmin = RS_MIN(t1, t2), tmax = RS_MAX(t1, t2);
		if (copysignf(1.0f, tmax) < 0.0f)
			return false;
		texit = RS_MIN(texit, tmax);
		if (tmin > tenter) {
			tenter = tmin;
			hitAxis = axis;
		}
		if (tenter > texit)
			return false;
	}
	if (0.0f >= tenter || tenter > dist)
		return false;

	float nL[3] = { 0.0f, 0.0f, 0.0f };
	float tHit = 0.0f;
	bool insideFace = true;
	for (int axis = 0; axis < 3; axis++) {
		if (axis == hitAxis)
			continue;
		const float local = (src[axis] + dir[axis] * tenter) - off[axis];
		if (fabsf(local) > inner[axis]) {
			insideFace = false;
			break;
		}
	}
	if (insideFace) {
		nL[hitAxis] = copysignf(1.0f, dir[hitAxis]) < 0.0f ? 1.0f : -1.0f;
		tHit = RS_MAX(tenter, 0.0f);
	} else {
		// The rounded box, in the child frame
		float sl[3];
		for (int axis = 0; axis < 3; axis++)
			sl[axis] = src[axis] - off[axis];
		float pts[8];
		pts[0] = 0.0f;
		pts[1] = dist;
		int count = 2;
		for (int axis = 0; axis < 3; axis++) {
			const float d = dir[axis];
			if (d == 0.0f)
				continue;
			for (int sgn = 0; sgn < 2; sgn++) {
				const float bound = sgn == 0 ? -inner[axis] : inner[axis];
				const float t = (bound - sl[axis]) / d;
				if (t > 0.0f && t < dist && count < 8)
					pts[count++] = t;
			}
		}
		for (int i = 1; i < count; i++) {
			const float key = pts[i];
			int j = i;
			while (j > 0 && pts[j - 1] > key) {
				pts[j] = pts[j - 1];
				j--;
			}
			pts[j] = key;
		}
		const float marginSq = margin * margin;
		bool found = false;
		for (int i = 0; i < count - 1 && !found; i++) {
			const float t0 = pts[i], t1 = pts[i + 1];
			if (t1 <= t0)
				continue;
			const float mid = t0 + (t1 - t0) * 0.5f;
			float a = 0.0f, b = 0.0f, c = 0.0f;
			int outside = 0;
			for (int axis = 0; axis < 3; axis++) {
				const float pm = sl[axis] + dir[axis] * mid;
				const float h = inner[axis];
				float bound;
				if (pm < -h)
					bound = -h;
				else if (pm > h)
					bound = h;
				else
					continue;
				outside++;
				const float o = sl[axis] - bound;
				const float d = dir[axis];
				a += d * d;
				b += 2.0f * d * o;
				c += o * o;
			}
			if (outside == 0)
				continue;
			const float cc = c - marginSq;
			float disc = b * b - 4.0f * a * cc;
			if (disc < 0.0f) {
				const float scale = b * b + fabsf(4.0f * a * cc);
				if (disc > -FLT_EPS * scale)
					disc = 0.0f;
				else
					continue;
			}
			const float sqrtD = sqrtf(disc);
			float th;
			if (disc == 0.0f) {
				th = -b / (2.0f * a);
			} else {
				const float q = -0.5f * (b >= 0.0f ? b + sqrtD : b - sqrtD);
				if (q == 0.0f) {
					th = -b / (2.0f * a);
				} else {
					const float r0 = q / a, r1 = cc / q;
					th = r0 <= r1 ? r0 : r1;
				}
			}
			if (th <= 0.0f || th > dist || th < t0 || th > t1)
				continue;
			float n[3];
			float lenSq = 0.0f;
			for (int axis = 0; axis < 3; axis++) {
				const float hp = sl[axis] + dir[axis] * th;
				n[axis] = hp - RS_MIN(RS_MAX(hp, -inner[axis]), inner[axis]);
				lenSq += n[axis] * n[axis];
			}
			if (lenSq <= FLT_EPS * FLT_EPS)
				continue;
			const float invLen = 1.0f / sqrtf(lenSq);
			float ndot = 0.0f;
			for (int axis = 0; axis < 3; axis++) {
				n[axis] *= invLen;
				ndot += n[axis] * dir[axis];
			}
			if (ndot > 0.0f)
				continue;
			// (renormalized, and the dot checked again)
			const float l2 = sqrtf(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
			float ndot2 = 0.0f;
			for (int axis = 0; axis < 3; axis++) {
				nL[axis] = n[axis] / l2;
				ndot2 += nL[axis] * dir[axis];
			}
			if (ndot2 > 0.0f)
				return false;
			tHit = th;
			found = true;
		}
		if (!found || 0.0f >= tHit || tHit > dist)
			return false;
	}

	btVector3 nW = basis * btVector3(nL[0], nL[1], nL[2]);
	const float nLenSq = nW.length2();
	outNormal = nLenSq > 0.0f ? nW * (1.0f / sqrtf(nLenSq)) : btVector3(0, 0, 0);
	outFraction = tHit / dist;
	return true;
}

namespace {
// The game's wheel ray: hits closer to the hardpoint than the car's minimum hit distance are skipped, and the ball and
// other cars are tested here, exactly, instead of with Bullet's subsimplex convex cast
struct WheelRayCallback : public btCollisionWorld::ClosestRayResultCallback {
	btScalar m_minFractionSq;
	WheelRayCallback(const btVector3& from, const btVector3& to, const btCollisionObject* ignoreObj, btScalar minFractionSq)
		: ClosestRayResultCallback(from, to, ignoreObj), m_minFractionSq(minFractionSq) {}
	virtual btScalar addSingleResult(btCollisionWorld::LocalRayResult& rayResult, bool normalInWorldSpace) {
		if (rayResult.m_hitFraction * rayResult.m_hitFraction < m_minFractionSq)
			return m_closestHitFraction;
		return ClosestRayResultCallback::addSingleResult(rayResult, normalInWorldSpace);
	}
	virtual bool needsCollision(btBroadphaseProxy* proxy0) const {
		if (!ClosestRayResultCallback::needsCollision(proxy0))
			return false;
		const btCollisionObject* obj = (const btCollisionObject*)proxy0->m_clientObject;
		if (obj->getUserIndex() == BT_USERINFO_TYPE_CAR && obj->getCollisionShape()->getShapeType() == COMPOUND_SHAPE_PROXYTYPE) {
			const btCompoundShape* compound = (const btCompoundShape*)obj->getCollisionShape();
			if (compound->getNumChildShapes() == 1 && compound->getChildShape(0)->getShapeType() == BOX_SHAPE_PROXYTYPE) {
				const btBoxShape* box = (const btBoxShape*)compound->getChildShape(0);
				float fraction;
				btVector3 normal;
				if (RayCarBox(m_rayFromWorld, m_rayToWorld, obj->getWorldTransform(), compound->getChildTransform(0).getOrigin(),
					box->getHalfExtentsWithoutMargin(), box->getMargin(), fraction, normal) && fraction < m_closestHitFraction) {
					btCollisionWorld::LocalRayResult rayResult(obj, nullptr, normal, fraction);
					const_cast<WheelRayCallback*>(this)->addSingleResult(rayResult, true);
				}
				return false;
			}
		}
		if (obj->getCollisionShape()->getShapeType() != SPHERE_SHAPE_PROXYTYPE)
			return true;
		float fraction;
		btVector3 normal;
		if (RaySphere(m_rayFromWorld, m_rayToWorld, obj->getWorldTransform(),
			((const btSphereShape*)obj->getCollisionShape())->getRadius(), fraction, normal) && fraction < m_closestHitFraction) {
			btCollisionWorld::LocalRayResult rayResult(obj, nullptr, normal, fraction);
			const_cast<WheelRayCallback*>(this)->addSingleResult(rayResult, true);
		}
		return false;
	}
};
}

btCollisionObject* btVehicleRL::castWheelRay(const btVector3& from, const btVector3& to, btVehicleRaycaster::btVehicleRaycasterResult& result) {
	WheelRayCallback rayCallback(from, to, m_chassisBody, m_minHitDistSq / (to - from).length2());
	rayCallback.m_collisionFilterGroup |= m_vehicleRaycaster->addedFilterMask;
	m_dynamicsWorld->rayTest(from, to, rayCallback);
	if (rayCallback.hasHit()) {
		const btRigidBody* body = btRigidBody::upcast(rayCallback.m_collisionObject);
		if (body && body->hasContactResponse()) {
			result.m_hitPointInWorld = rayCallback.m_hitPointWorld;
			result.m_hitNormalInWorld = rayCallback.m_hitNormalWorld;
			result.m_hitNormalInWorld.normalize();
			result.m_distFraction = rayCallback.m_closestHitFraction;
			return (btCollisionObject*)body;
		}
	}
	return NULL;
}

float btVehicleRL::rayCast(btWheelInfoRL& wheel) {
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
	btCollisionObject* object = castWheelRay(source, target, rayResults);

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

		// The game doesn't clamp the suspension length to the travel

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

		wheel.m_extraPushback = 0;

		// The game also pushes back from other cars, resolved against that car
		const bool carHit = object->getUserIndex() == BT_USERINFO_TYPE_CAR;
		float fullPushback = 0;
		if (object->isStaticObject() || carHit) { // Compute m_extraPushback when colliding with static object or car
			float rayPushbackThresh = (wheel.m_suspensionRestLength1 + wheel.m_wheelsRadius) - RLConst::BTVehicle::SUSPENSION_SUBTRACTION;
			if (wheelTraceLenSq < rayPushbackThresh) {

				float wheelTraceDistDelta = wheelTraceLenSq - rayPushbackThresh;
				float collisionResult = resolveSingleCollision(
					m_chassisBody,
					object,
					rayResults.m_hitPointInWorld,
					rayResults.m_hitNormalInWorld,
					PushbackSolverInfo(m_dynamicsWorld->getSolverInfo()),
					wheelTraceDistDelta,
					false
				);

				wheel.m_extraPushback = collisionResult / getNumWheels();
				fullPushback = collisionResult;
			}
		}
		if (!object->isStaticObject()) {
			// The game pushes a dynamic body under a touching wheel down with a 1.0 impulse at the contact point,
			// and a car under it also takes the full pushback, before the next wheel reads its velocity
			btRigidBody* groundBody = btRigidBody::upcast(object);
			if (groundBody && groundBody->getInvMass() != 0) {
				groundBody->applyImpulse(-wheel.m_raycastInfo.m_contactNormalWS,
					rayResults.m_hitPointInWorld - groundBody->getCenterOfMassPosition());
				if (carHit && fullPushback > 0)
					groundBody->applyImpulse(-wheel.m_raycastInfo.m_contactNormalWS * fullPushback,
						rayResults.m_hitPointInWorld - groundBody->getCenterOfMassPosition());
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

void btVehicleRL::updateVehicleFirst(float step) {

	for (int i = 0; i < getNumWheels(); i++)
		updateWheelTransform(i);

	//
	// simulate suspension
	//

	for (int i = 0; i < m_wheelInfo.size(); i++)
		rayCast(m_wheelInfo[i]);
}

void btVehicleRL::updateVehicleSecond(float step) {
	updateSuspension(step);
	// The game computes the friction impulses after the suspension impulses
	calcFrictionImpulses(step);
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
	for (int i = 0; i < m_wheelInfo.size(); i++) {
		btWheelInfoRL& wheel = m_wheelInfo[i];

		btRigidBody* groundObject = (btRigidBody*)wheel.m_raycastInfo.m_groundObject;
		if (groundObject) {
			// Axle direction (includes steering turn)
			btVector3 axleDir = wheel.m_worldTransform.getBasis().getColumn(m_indexRightAxis);

			btVector3 surfNormalWS = wheel.m_raycastInfo.m_contactNormalWS;
			btVector3 wheelAxleDir = axleDir;
			float proj = axleDir.dot(surfNormalWS);
			axleDir -= surfNormalWS * proj;
			axleDir = axleDir.safeNormalized();

			// Wheel forwards direction
			btVector3 forwardDir = surfNormalWS.cross(axleDir).safeNormalized();

			// The game solves and applies the side impulse along the wheel's own axle, against a fixed
			// body (the ground body's velocity and mass are ignored)
			axleDir = wheelAxleDir;
			groundObject = &GetFixedBody();

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
						v1 = wheel.m_velAtContactPoint, // The contact velocity from before the suspension impulses
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
	for (int i = 0; i < m_wheelInfo.size(); i++) {
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
	for (int i = 0; i < m_wheelInfo.size(); i++)
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
/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2009 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
#include "../CollisionShapes/btCollisionShape.h"

#include "../CollisionShapes/btBoxShape.h"
#include "../CollisionShapes/btTriangleShape.h"
#include "../CollisionShapes/btSphereShape.h"
#include "../CollisionShapes/btStaticPlaneShape.h"
#include "../CollisionShapes/btTriangleMeshShape.h"
#include "../CollisionShapes/btCompoundShape.h"
#include "btConvexHullShape.h"

/*
  Make sure this dummy function never changes so that it
  can be used by probes that are checking whether the
  library is actually installed.
*/
extern "C"
{
	void btBulletCollisionProbe();

	void btBulletCollisionProbe() {}
}

void btCollisionShape::getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const {

	// If we're a sphere, its faster to just re-calculate
	if (m_shapeType == SPHERE_SHAPE_PROXYTYPE)
		return ((btSphereShape*)this)->getAabb(t, aabbMin, aabbMax);

	constexpr auto fnFastCompareTransforms = [](const btTransform& a, const btTransform& b) -> bool {
		return
			(a.m_origin == b.m_origin) &&
			(a.m_basis[0] == b.m_basis[0]) &&
			(a.m_basis[1] == b.m_basis[1]);
			// Don't need to compare the last row of the basis
	};

	if (!fnFastCompareTransforms(t, m_aabbCacheTrans) || !m_aabbCached) {
		switch (m_shapeType) {
		case BOX_SHAPE_PROXYTYPE:
			((btBoxShape*)this)->getAabb(t, aabbMin, aabbMax);
			break;
		case TRIANGLE_SHAPE_PROXYTYPE:
			((btTriangleShape*)this)->getAabb(t, aabbMin, aabbMax);
			break;
		case SPHERE_SHAPE_PROXYTYPE:
			((btSphereShape*)this)->getAabb(t, aabbMin, aabbMax);
			break;
		case STATIC_PLANE_PROXYTYPE:
			((btStaticPlaneShape*)this)->getAabb(t, aabbMin, aabbMax);
			break;
		case TRIANGLE_MESH_SHAPE_PROXYTYPE:
			((btTriangleMeshShape*)this)->getAabb(t, aabbMin, aabbMax);
			break;
		case COMPOUND_SHAPE_PROXYTYPE:
			((btCompoundShape*)this)->getAabb(t, aabbMin, aabbMax);
			break;
		case CONVEX_HULL_SHAPE_PROXYTYPE:
			((btConvexHullShape*)this)->getAabb(t, aabbMin, aabbMax);
			break;
		default:
			btAssert(false);
		}

		m_aabbCached = true;
		m_aabbMinCache = aabbMin;
		m_aabbMaxCache = aabbMax;
		m_aabbCacheTrans = t;
	} else {
		aabbMin = m_aabbMinCache;
		aabbMax = m_aabbMaxCache;
	}
}

btScalar btCollisionShape::getMargin() const {
	switch (m_shapeType) {
	case BOX_SHAPE_PROXYTYPE:
		return ((btBoxShape*)this)->getMargin();
	case TRIANGLE_SHAPE_PROXYTYPE:
		return ((btTriangleShape*)this)->getMargin();
	case SPHERE_SHAPE_PROXYTYPE:
		return ((btSphereShape*)this)->getMargin();
	case STATIC_PLANE_PROXYTYPE:
		return ((btStaticPlaneShape*)this)->getMargin();
	case TRIANGLE_MESH_SHAPE_PROXYTYPE:
		return ((btTriangleMeshShape*)this)->getMargin();
	case COMPOUND_SHAPE_PROXYTYPE:
		return ((btCompoundShape*)this)->getMargin();
	case CONVEX_HULL_SHAPE_PROXYTYPE:
		return ((btConvexHullShape*)this)->getMargin();
	default:
		btAssert(false);
	}
}

void btCollisionShape::setMargin(btScalar margin) {
	switch (m_shapeType) {
	case BOX_SHAPE_PROXYTYPE:
		return ((btBoxShape*)this)->setMargin(margin);
	case TRIANGLE_SHAPE_PROXYTYPE:
		return ((btTriangleShape*)this)->setMargin(margin);
	case SPHERE_SHAPE_PROXYTYPE:
		return ((btSphereShape*)this)->setMargin(margin);
	case STATIC_PLANE_PROXYTYPE:
		return ((btStaticPlaneShape*)this)->setMargin(margin);
	case TRIANGLE_MESH_SHAPE_PROXYTYPE:
		return ((btTriangleMeshShape*)this)->setMargin(margin);
	case COMPOUND_SHAPE_PROXYTYPE:
		return ((btCompoundShape*)this)->setMargin(margin);
	default:
		btAssert(false);
	}
}

void btCollisionShape::getBoundingSphere(btVector3& center, btScalar& radius) const {
	switch (m_shapeType) {
	case SPHERE_SHAPE_PROXYTYPE:
		center = btVector3(0, 0, 0);
		radius = ((btSphereShape*)this)->getRadius() + 0.08;
		break;
	default:
		btTransform tr;
		tr.setIdentity();
		btVector3 aabbMin, aabbMax;

		getAabb(tr, aabbMin, aabbMax);

		radius = (aabbMax - aabbMin).length() * btScalar(0.5);
		center = (aabbMin + aabbMax) * btScalar(0.5);
	}
}

btScalar btCollisionShape::getContactBreakingThreshold(btScalar defaultContactThreshold) const {
	return getAngularMotionDisc() * defaultContactThreshold;
}

btScalar btCollisionShape::getAngularMotionDisc() const {
	///@todo cache this value, to improve performance
	btVector3 center;
	btScalar disc;
	getBoundingSphere(center, disc);
	disc += (center).length();
	return disc;
}

void btCollisionShape::calculateTemporalAabb(const btTransform& curTrans, const btVector3& linvel, const btVector3& angvel, btScalar timeStep, btVector3& temporalAabbMin, btVector3& temporalAabbMax) const {
	//start with static aabb
	getAabb(curTrans, temporalAabbMin, temporalAabbMax);

	btScalar temporalAabbMaxx = temporalAabbMax.getX();
	btScalar temporalAabbMaxy = temporalAabbMax.getY();
	btScalar temporalAabbMaxz = temporalAabbMax.getZ();
	btScalar temporalAabbMinx = temporalAabbMin.getX();
	btScalar temporalAabbMiny = temporalAabbMin.getY();
	btScalar temporalAabbMinz = temporalAabbMin.getZ();

	// add linear motion
	btVector3 linMotion = linvel * timeStep;
	///@todo: simd would have a vector max/min operation, instead of per-element access
	if (linMotion.x() > btScalar(0.))
		temporalAabbMaxx += linMotion.x();
	else
		temporalAabbMinx += linMotion.x();
	if (linMotion.y() > btScalar(0.))
		temporalAabbMaxy += linMotion.y();
	else
		temporalAabbMiny += linMotion.y();
	if (linMotion.z() > btScalar(0.))
		temporalAabbMaxz += linMotion.z();
	else
		temporalAabbMinz += linMotion.z();

	//add conservative angular motion
	btScalar angularMotion = angvel.length() * getAngularMotionDisc() * timeStep;
	btVector3 angularMotion3d(angularMotion, angularMotion, angularMotion);
	temporalAabbMin = btVector3(temporalAabbMinx, temporalAabbMiny, temporalAabbMinz);
	temporalAabbMax = btVector3(temporalAabbMaxx, temporalAabbMaxy, temporalAabbMaxz);

	temporalAabbMin -= angularMotion3d;
	temporalAabbMax += angularMotion3d;
}
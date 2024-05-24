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

#ifndef BT_COLLISION_SHAPE_H
#define BT_COLLISION_SHAPE_H

#include "../../LinearMath/btTransform.h"
#include "../../LinearMath/btVector3.h"
#include "../../LinearMath/btMatrix3x3.h"
#include "../BroadphaseCollision/btBroadphaseProxy.h"  //for the shape types

///The btCollisionShape class provides an interface for collision shapes that can be shared among btCollisionObjects.
ATTRIBUTE_ALIGNED16(class)
btCollisionShape
{
public:

	int m_shapeType;
	void* m_userPointer;
	int m_userIndex;
	int m_userIndex2;

	mutable bool m_aabbCached = false;
	mutable btVector3 m_aabbMinCache, m_aabbMaxCache;
	mutable btTransform m_aabbCacheTrans;

	BT_DECLARE_ALIGNED_ALLOCATOR();

	btCollisionShape() : 
		m_shapeType(INVALID_SHAPE_PROXYTYPE), m_userPointer(0), m_userIndex(-1), m_userIndex2(-1)
	{
	}

	virtual ~btCollisionShape()
	{
	}

	///getAabb returns the axis aligned bounding box in the coordinate frame of the given transform t.
	void getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const;

	btScalar getMargin() const;
	void setMargin(btScalar margin);

	void getBoundingSphere(btVector3& center, btScalar& radius) const;

	///getAngularMotionDisc returns the maximum radius needed for Conservative Advancement to handle time-of-impact with rotations.
	btScalar getAngularMotionDisc() const;

	btScalar getContactBreakingThreshold(btScalar defaultContactThresholdFactor) const;

	///calculateTemporalAabb calculates the enclosing aabb for the moving object over interval [0..timeStep)
	///result is conservative
	void calculateTemporalAabb(const btTransform& curTrans, const btVector3& linvel, const btVector3& angvel, btScalar timeStep, btVector3& temporalAabbMin, btVector3& temporalAabbMax) const;

	SIMD_FORCE_INLINE bool isPolyhedral() const
	{
		return btBroadphaseProxy::isPolyhedral(getShapeType());
	}

	SIMD_FORCE_INLINE bool isConvex2d() const
	{
		return btBroadphaseProxy::isConvex2d(getShapeType());
	}

	SIMD_FORCE_INLINE bool isConvex() const
	{
		return btBroadphaseProxy::isConvex(getShapeType());
	}
	SIMD_FORCE_INLINE bool isNonMoving() const
	{
		return btBroadphaseProxy::isNonMoving(getShapeType());
	}
	SIMD_FORCE_INLINE bool isConcave() const
	{
		return btBroadphaseProxy::isConcave(getShapeType());
	}
	SIMD_FORCE_INLINE bool isCompound() const
	{
		return btBroadphaseProxy::isCompound(getShapeType());
	}

	SIMD_FORCE_INLINE bool isSoftBody() const
	{
		return btBroadphaseProxy::isSoftBody(getShapeType());
	}

	///isInfinite is used to catch simulation error (aabb check)
	SIMD_FORCE_INLINE bool isInfinite() const
	{
		return btBroadphaseProxy::isInfinite(getShapeType());
	}

#ifndef __SPU__

	void calculateLocalInertia(btScalar mass, btVector3& inertia) const {
		btAssert(false);
	}

	//debugging support
	virtual const char* getName() const = 0;
#endif  //__SPU__

	int getShapeType() const
	{
		return m_shapeType;
	}

	///the getAnisotropicRollingFrictionDirection can be used in combination with setAnisotropicFriction
	///See Bullet/Demos/RollingFrictionDemo for an example
	btVector3 getAnisotropicRollingFrictionDirection() const
	{
		return btVector3(1, 1, 1);
	}

	///optional user data pointer
	void setUserPointer(void* userPtr)
	{
		m_userPointer = userPtr;
	}

	void* getUserPointer() const
	{
		return m_userPointer;
	}
	void setUserIndex(int index)
	{
		m_userIndex = index;
	}

	int getUserIndex() const
	{
		return m_userIndex;
	}

	void setUserIndex2(int index)
	{
		m_userIndex2 = index;
	}

	int getUserIndex2() const
	{
		return m_userIndex2;
	}
};

#endif  //BT_COLLISION_SHAPE_H

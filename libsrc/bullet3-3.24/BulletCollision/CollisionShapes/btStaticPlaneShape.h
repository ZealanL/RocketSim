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

#ifndef BT_STATIC_PLANE_SHAPE_H
#define BT_STATIC_PLANE_SHAPE_H

#include "btConcaveShape.h"

///The btStaticPlaneShape simulates an infinite non-moving (static) collision plane.
ATTRIBUTE_ALIGNED16(class)
btStaticPlaneShape : public btConcaveShape
{
public:
	btVector3 m_localAabbMin;
	btVector3 m_localAabbMax;

	btVector3 m_planeNormal;
	btScalar m_planeConstant;
	
	// Plane only exists along a single axis
	bool m_isSingleAxis;
	int m_singleAxisIdx;
	bool m_singleAxisBackwards;

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	btStaticPlaneShape() {}

	btStaticPlaneShape(const btVector3& planeNormal, btScalar planeConstant);

	virtual ~btStaticPlaneShape();

	void getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const;

	void processAllTriangles(btTriangleCallback * callback, const btVector3& aabbMin, const btVector3& aabbMax) const;

	void calculateLocalInertia(btScalar mass, btVector3 & inertia) const;

	const btVector3& getPlaneNormal() const
	{
		return m_planeNormal;
	}

	const btScalar& getPlaneConstant() const
	{
		return m_planeConstant;
	}

	//debugging
	virtual const char* getName() const { return "STATICPLANE"; }
};
#endif  //BT_STATIC_PLANE_SHAPE_H

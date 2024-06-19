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

#include "btSphereShape.h"
#include "../CollisionShapes/btCollisionMargin.h"

#include "../../LinearMath/btQuaternion.h"

btVector3 btSphereShape::localGetSupportingVertexWithoutMargin(const btVector3& vec) const
{
	(void)vec;
	return btVector3(btScalar(0.), btScalar(0.), btScalar(0.));
}

void btSphereShape::batchedUnitVectorGetSupportingVertexWithoutMargin(const btVector3* vectors, btVector3* supportVerticesOut, int numVectors) const
{
	(void)vectors;

	for (int i = 0; i < numVectors; i++)
	{
		supportVerticesOut[i].setValue(btScalar(0.), btScalar(0.), btScalar(0.));
	}
}

btVector3 btSphereShape::localGetSupportingVertex(const btVector3& vec) const
{
	btVector3 vecnorm = vec;
	if (vecnorm.length2() < (SIMD_EPSILON * SIMD_EPSILON))
	{
		static thread_local btVector3 invalidVecNorm = btVector3(-1, -1, -1).normalized();
		vecnorm = invalidVecNorm;
	} else {
		vecnorm.normalize();
	}

	return getMargin() * vecnorm;
}

//broken due to scaling
void btSphereShape::getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const
{
	const btVector3& center = t.getOrigin();
	btScalar margin = getMargin() + 0.08f; // ROCKETSIM CHANGE: Add 0.08 to sphere margin
	btVector3 extent(margin, margin, margin);
	aabbMin = center - extent;
	aabbMax = center + extent;
}

void btSphereShape::calculateLocalInertia(btScalar mass, btVector3& inertia) const
{
	btScalar elem = btScalar(0.4) * mass * getMargin() * getMargin();
	inertia.setValue(elem, elem, elem);
}

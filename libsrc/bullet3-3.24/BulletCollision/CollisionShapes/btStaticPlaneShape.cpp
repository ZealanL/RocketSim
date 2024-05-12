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

#include "btStaticPlaneShape.h"

#include "../../LinearMath/btTransformUtil.h"

btStaticPlaneShape::btStaticPlaneShape(const btVector3& planeNormal, btScalar planeConstant)
	: btConcaveShape(), m_planeNormal(planeNormal.normalized()), m_planeConstant(planeConstant) {
	m_shapeType = STATIC_PLANE_PROXYTYPE;
	//	btAssert( btFuzzyZero(m_planeNormal.length() - btScalar(1.)) );

	int numAxis = 0;
	for (int i = 0; i < 3; i++)
		if (!btFuzzyZero(m_planeNormal[i]))
			numAxis++;

	if (numAxis == 1) {
		m_isSingleAxis = true;
		m_singleAxisIdx = m_planeNormal.closestAxis();
		m_singleAxisBackwards = m_planeNormal[m_singleAxisIdx] < 0;
	}
}

btStaticPlaneShape::~btStaticPlaneShape() {
}

void btStaticPlaneShape::getAabb(const btTransform& t, btVector3& aabbMin, btVector3& aabbMax) const {
	(void)t;

	aabbMin.setValue(btScalar(-BT_LARGE_FLOAT), btScalar(-BT_LARGE_FLOAT), btScalar(-BT_LARGE_FLOAT));
	aabbMax.setValue(btScalar(BT_LARGE_FLOAT), btScalar(BT_LARGE_FLOAT), btScalar(BT_LARGE_FLOAT));
	
	constexpr float PLANE_CONSTANT_OFFSET = 0.01f;

	if (m_isSingleAxis) {
		aabbMin[m_singleAxisIdx] = t.getOrigin()[m_singleAxisIdx] + (m_planeConstant - PLANE_CONSTANT_OFFSET);
		aabbMax[m_singleAxisIdx] = t.getOrigin()[m_singleAxisIdx] + (m_planeConstant + PLANE_CONSTANT_OFFSET);

		(m_singleAxisBackwards ? aabbMax : aabbMin)[m_singleAxisIdx] = (m_singleAxisBackwards ? BT_LARGE_FLOAT : -BT_LARGE_FLOAT);
	}
}

void btStaticPlaneShape::processAllTriangles(btTriangleCallback* callback, const btVector3& aabbMin, const btVector3& aabbMax) const {
	btVector3 halfExtents = (aabbMax - aabbMin) * btScalar(0.5);
	btScalar radius = halfExtents.length();
	btVector3 center = (aabbMax + aabbMin) * btScalar(0.5);

	//this is where the triangles are generated, given AABB and plane equation (normal/constant)

	btVector3 tangentDir0, tangentDir1;

	//tangentDir0/tangentDir1 can be precalculated
	btPlaneSpace1(m_planeNormal, tangentDir0, tangentDir1);

	btVector3 projectedCenter = center - (m_planeNormal.dot(center) - m_planeConstant) * m_planeNormal;

	btVector3 triangle[3];
	triangle[0] = projectedCenter + tangentDir0 * radius + tangentDir1 * radius;
	triangle[1] = projectedCenter + tangentDir0 * radius - tangentDir1 * radius;
	triangle[2] = projectedCenter - tangentDir0 * radius - tangentDir1 * radius;

	callback->processTriangle(triangle, 0, 0);

	triangle[0] = projectedCenter - tangentDir0 * radius - tangentDir1 * radius;
	triangle[1] = projectedCenter - tangentDir0 * radius + tangentDir1 * radius;
	triangle[2] = projectedCenter + tangentDir0 * radius + tangentDir1 * radius;

	callback->processTriangle(triangle, 0, 1);
}

void btStaticPlaneShape::calculateLocalInertia(btScalar mass, btVector3& inertia) const {
	(void)mass;

	//moving concave objects not supported

	inertia.setValue(btScalar(0.), btScalar(0.), btScalar(0.));
}
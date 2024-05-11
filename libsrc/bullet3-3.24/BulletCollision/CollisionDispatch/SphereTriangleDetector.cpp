/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "../../LinearMath/btScalar.h"
#include "SphereTriangleDetector.h"
#include "../CollisionShapes/btTriangleShape.h"
#include "../CollisionShapes/btSphereShape.h"

SphereTriangleDetector::SphereTriangleDetector(btSphereShape* sphere, btTriangleShape* triangle, btScalar contactBreakingThreshold)
	: m_sphere(sphere),
	  m_triangle(triangle),
	  m_contactBreakingThreshold(contactBreakingThreshold)
{
}

void SphereTriangleDetector::getClosestPoints(const ClosestPointInput& input, Result& output, bool swapResults)
{
	const btTransform& transformA = input.m_transformA;
	const btTransform& transformB = input.m_transformB;

	btVector3 point, normal;
	btScalar timeOfImpact = btScalar(1.);
	btScalar depth = btScalar(0.);
	//	output.m_distance = btScalar(BT_LARGE_FLOAT);
	//move sphere into triangle space
	btTransform sphereInTr = transformB.inverseTimes(transformA);

	if (collide(sphereInTr.getOrigin(), point, normal, depth, timeOfImpact, m_contactBreakingThreshold))
	{
		if (swapResults)
		{
			btVector3 normalOnB = transformB.getBasis() * normal;
			btVector3 normalOnA = -normalOnB;
			btVector3 pointOnA = transformB * point + normalOnB * depth;
			output.addContactPoint(normalOnA, pointOnA, depth);
		}
		else
		{
			output.addContactPoint(transformB.getBasis() * normal, transformB * point, depth);
		}
	}
}

// See also geometrictools.com
// Basic idea: D = |p - (lo + t0*lv)| where t0 = lv . (p - lo) / lv . lv
btScalar SegmentSqrDistance(const btVector3& from, const btVector3& to, const btVector3& p, btVector3& nearest);

btScalar SegmentSqrDistance(const btVector3& from, const btVector3& to, const btVector3& p, btVector3& nearest)
{
	btVector3 diff = p - from;
	btVector3 v = to - from;
	btScalar t = v.dot(diff);

	if (t > 0)
	{
		btScalar dotVV = v.dot(v);
		if (t < dotVV)
		{
			t /= dotVV;
			diff -= t * v;
		}
		else
		{
			t = 1;
			diff -= v;
		}
	}
	else
		t = 0;

	nearest = from + t * v;
	return diff.dot(diff);
}

// From https://github.com/RenderKit/embree/blob/master/tutorials/common/math/closest_point.h (thanks VirxEC)
btVector3 closestPointTriangle(btVector3 const& p, btVector3 const& a, btVector3 const& b, btVector3 const& c) {
	const btVector3 ab = b - a;
	const btVector3 ac = c - a;
	const btVector3 ap = p - a;

	const float d1 = ab.dot(ap);
	const float d2 = ac.dot(ap);
	if (d1 <= 0.f && d2 <= 0.f) return a; //#1

	const btVector3 bp = p - b;
	const float d3 = ab.dot(bp);
	const float d4 = ac.dot(bp);
	if (d3 >= 0.f && d4 <= d3) return b; //#2

	const btVector3 cp = p - c;
	const float d5 = ab.dot(cp);
	const float d6 = ac.dot(cp);
	if (d6 >= 0.f && d5 <= d6) return c; //#3

	const float vc = d1 * d4 - d3 * d2;
	if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f) {
		const float v = d1 / (d1 - d3);
		return a + v * ab; //#4
	}

	const float vb = d5 * d2 - d1 * d6;
	if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f) {
		const float v = d2 / (d2 - d6);
		return a + v * ac; //#5
	}

	const float va = d3 * d6 - d5 * d4;
	if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f) {
		const float v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		return b + v * (c - b); //#6
	}

	const float denom = 1.f / (va + vb + vc);
	const float v = vb * denom;
	const float w = vc * denom;
	return a + v * ab + w * ac; //#0
}

bool SphereTriangleDetector::facecontains(const btVector3& p, const btVector3* vertices, btVector3& normal)
{
	btVector3 lp(p);
	btVector3 lnormal(normal);

	return pointInTriangle(vertices, lnormal, &lp);
}

bool SphereTriangleDetector::collide(const btVector3& sphereCenter, btVector3& point, btVector3& resultNormal, btScalar& depth, btScalar& timeOfImpact, btScalar contactBreakingThreshold)
{
	const btVector3* vertices = &m_triangle->getVertexPtr(0);

	btScalar radius = m_sphere->getRadius();
	btScalar radiusWithThreshold = radius + contactBreakingThreshold;

	btVector3 normal = (vertices[1] - vertices[0]).cross(vertices[2] - vertices[0]);

	btScalar l2 = normal.length2();
	bool hasContact = false;
	btVector3 contactPoint;

	if (l2 >= SIMD_EPSILON * SIMD_EPSILON)
	{
		normal /= btSqrt(l2);

		btVector3 p1ToCentre = sphereCenter - vertices[0];
		btScalar distanceFromPlane = p1ToCentre.dot(normal);

		if (distanceFromPlane < btScalar(0.))
		{
			//triangle facing the other way
			distanceFromPlane *= btScalar(-1.);
			normal *= btScalar(-1.);
		}

		bool isInsideContactPlane = distanceFromPlane < radiusWithThreshold;

		// Check for contact / intersection

		if (isInsideContactPlane)
		{
			if (facecontains(sphereCenter, vertices, normal))
			{
				// Inside the contact wedge - touches a point on the shell plane
				hasContact = true;
				contactPoint = sphereCenter - normal * distanceFromPlane;
			}
			else
			{
				// Could be inside one of the contact capsules
				btScalar contactCapsuleRadiusSqr = radiusWithThreshold * radiusWithThreshold;
				btScalar minDistSqr = contactCapsuleRadiusSqr;

#if 0 // Bullet's method
				btVector3 nearestOnEdge;
				for (int i = 0; i < 3; i++)
				{
					btVector3 pa;
					btVector3 pb;

					m_triangle->getEdge(i, pa, pb);

					btScalar distanceSqr = SegmentSqrDistance(pa, pb, sphereCenter, nearestOnEdge);
					if (distanceSqr < minDistSqr)
					{
						// Yep, we're inside a capsule, and record the capsule with smallest distance
						minDistSqr = distanceSqr;
						hasContact = true;
						contactPoint = nearestOnEdge;
					}
				}
#else // Faster method (thanks VirxEC)
				// https://github.com/VirxEC/rl_ball_sym/blob/99b50b381cd529e567c9a33ab10464b89484227a/src/simulation/geometry.rs
				btVector3 nearestOnEdge = closestPointTriangle(sphereCenter, vertices[0], vertices[1], vertices[2]);
				btScalar distanceSqr = nearestOnEdge.distance2(sphereCenter);
				if (distanceSqr < minDistSqr) {
					minDistSqr = distanceSqr;
					hasContact = true;
					contactPoint = nearestOnEdge;
				}
#endif
			}
		}

	}

	if (hasContact)
	{
		btVector3 contactToCentre = sphereCenter - contactPoint;
		btScalar distanceSqr = contactToCentre.length2();

		if (distanceSqr < radiusWithThreshold * radiusWithThreshold)
		{
			if (distanceSqr > SIMD_EPSILON)
			{
				btScalar distance = btSqrt(distanceSqr);
				resultNormal = contactToCentre;
				resultNormal.normalize();
				point = contactPoint;
				depth = -(radius - distance);
			}
			else
			{
				resultNormal = normal;
				point = contactPoint;
				depth = -radius;
			}
			return true;
		}
	}

	return false;
}

bool SphereTriangleDetector::pointInTriangle(const btVector3 vertices[], const btVector3& normal, btVector3* p)
{
	
#if 0 // Bullet's method
	const btVector3* p1 = &vertices[0];
	const btVector3* p2 = &vertices[1];
	const btVector3* p3 = &vertices[2];

	btVector3 edge1(*p2 - *p1);
	btVector3 edge2(*p3 - *p2);
	btVector3 edge3(*p1 - *p3);

	btVector3 p1_to_p(*p - *p1);
	btVector3 p2_to_p(*p - *p2);
	btVector3 p3_to_p(*p - *p3);

	btVector3 edge1_normal(edge1.cross(normal));
	btVector3 edge2_normal(edge2.cross(normal));
	btVector3 edge3_normal(edge3.cross(normal));

	btScalar r1, r2, r3;
	r1 = edge1_normal.dot(p1_to_p);
	r2 = edge2_normal.dot(p2_to_p);
	r3 = edge3_normal.dot(p3_to_p);
	if ((r1 > 0 && r2 > 0 && r3 > 0) ||
		(r1 <= 0 && r2 <= 0 && r3 <= 0))
		return true;
	return false;
#else // A faster method from https://gamedev.stackexchange.com/questions/28781/easy-way-to-project-point-onto-triangle-or-plane/152476#152476 (thanks VirxEC)

	const btVector3&
		p0 = vertices[0],
		p1 = vertices[1],
		p2 = vertices[2];

	btVector3 u = p1 - p0;
	btVector3 v = p2 - p0;
	btVector3 n = u.cross(v);
	float nLenSq = n.dot(n);

	btVector3 w = *p - p0;

	float gamma = u.cross(w).dot(n) / nLenSq;

	float beta = w.cross(v).dot(n) / nLenSq;
	float alpha = 1 - gamma - beta;

	return ((0 <= alpha) && (alpha <= 1) &&
		(0 <= beta) && (beta <= 1) &&
		(0 <= gamma) && (gamma <= 1));
#endif
}

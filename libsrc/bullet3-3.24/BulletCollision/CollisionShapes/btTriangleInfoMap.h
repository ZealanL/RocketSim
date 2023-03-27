/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2010 Erwin Coumans  http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef _BT_TRIANGLE_INFO_MAP_H
#define _BT_TRIANGLE_INFO_MAP_H

#include "../../LinearMath/btHashMap.h"

///for btTriangleInfo m_flags
#define TRI_INFO_V0V1_CONVEX 1
#define TRI_INFO_V1V2_CONVEX 2
#define TRI_INFO_V2V0_CONVEX 4

#define TRI_INFO_V0V1_SWAP_NORMALB 8
#define TRI_INFO_V1V2_SWAP_NORMALB 16
#define TRI_INFO_V2V0_SWAP_NORMALB 32

///The btTriangleInfo structure stores information to adjust collision normals to avoid collisions against internal edges
///it can be generated using
struct btTriangleInfo
{
	btTriangleInfo()
	{
		m_edgeV0V1Angle = SIMD_2_PI;
		m_edgeV1V2Angle = SIMD_2_PI;
		m_edgeV2V0Angle = SIMD_2_PI;
		m_flags = 0;
	}

	int m_flags;

	btScalar m_edgeV0V1Angle;
	btScalar m_edgeV1V2Angle;
	btScalar m_edgeV2V0Angle;
};

typedef btHashMap<btHashInt, btTriangleInfo> btInternalTriangleInfoMap;

///The btTriangleInfoMap stores edge angle information for some triangles. You can compute this information yourself or using btGenerateInternalEdgeInfo.
struct btTriangleInfoMap : public btInternalTriangleInfoMap
{
	btScalar m_convexEpsilon;          ///used to determine if an edge or contact normal is convex, using the dot product
	btScalar m_planarEpsilon;          ///used to determine if a triangle edge is planar with zero angle
	btScalar m_equalVertexThreshold;   ///used to compute connectivity: if the distance between two vertices is smaller than m_equalVertexThreshold, they are considered to be 'shared'
	btScalar m_edgeDistanceThreshold;  ///used to determine edge contacts: if the closest distance between a contact point and an edge is smaller than this distance threshold it is considered to "hit the edge"
	btScalar m_maxEdgeAngleThreshold;  //ignore edges that connect triangles at an angle larger than this m_maxEdgeAngleThreshold
	btScalar m_zeroAreaThreshold;      ///used to determine if a triangle is degenerate (length squared of cross product of 2 triangle edges < threshold)

	btTriangleInfoMap()
	{
		m_convexEpsilon = 0.00f;
		m_planarEpsilon = 0.0001f;
		m_equalVertexThreshold = btScalar(0.0001) * btScalar(0.0001);
		m_edgeDistanceThreshold = btScalar(0.1);
		m_zeroAreaThreshold = btScalar(0.0001) * btScalar(0.0001);
		m_maxEdgeAngleThreshold = SIMD_2_PI;
	}
	virtual ~btTriangleInfoMap() {}
};

#endif
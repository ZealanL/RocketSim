#include "SuspensionCollisionGrid.h"

#include "../../../libsrc/bullet3-3.24/BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"

// Quick virtual btTriangleCallback child class to simply check if any triangles are processed
struct BoolHitTriangleCallback : public btTriangleCallback {

	bool hit = false;
	
	BoolHitTriangleCallback() {}
	virtual void processTriangle(btVector3* triangle, int partId, int triangleIndex) {
		hit = true;
	}
};

void SuspensionCollisionGrid::SetupWorldCollision(const vector<btBvhTriangleMeshShape*>& triMeshShapes) {

	int totalCellsWithin = 0;
	BoolHitTriangleCallback boolCallback = BoolHitTriangleCallback();

	Vec cellSizeBT = GetCellSize() * UU_TO_BT;

	for (btBvhTriangleMeshShape* triMeshShape : triMeshShapes) {
		
		btVector3 rbMinBT, rbMaxBT;
		triMeshShape->getAabb(btTransform(), rbMinBT, rbMaxBT);

		btVector3 borderSize = (cellSizeBT * UU_TO_BT);
		rbMinBT -= borderSize;
		rbMaxBT += borderSize;

		for (int i = 0; i < CELL_AMOUNT_X; i++) {

			Vec
				cellPlaneMinBT = GetCellMin(i, 0, 0) * UU_TO_BT - cellSizeBT,
				cellPlaneMaxBT = GetCellMin(i, CELL_AMOUNT_Y - 1, CELL_AMOUNT_Z - 1) * UU_TO_BT + (cellSizeBT * 2);

			boolCallback.hit = false;
			triMeshShape->processAllTriangles(&boolCallback, cellPlaneMinBT, cellPlaneMaxBT);
			if (boolCallback.hit) {
				for (int j = 0; j < CELL_AMOUNT_Y; j++) {

					Vec
						cellColumnMinBT = GetCellMin(i, j, 0) * UU_TO_BT - cellSizeBT,
						cellColumnMaxBT = GetCellMin(i, j, CELL_AMOUNT_Z - 1) * UU_TO_BT + (cellSizeBT * 2);

					boolCallback.hit = false;
					triMeshShape->processAllTriangles(&boolCallback, cellColumnMinBT, cellColumnMaxBT);

					if (boolCallback.hit) {
						for (int k = 0; k < CELL_AMOUNT_Z; k++) {

							Cell& cell = Get(i, j, k);

							if (!cell.worldCollision) {

								Vec
									cellMinBT = (GetCellMin(i, j, k) * UU_TO_BT - cellSizeBT),
									cellMaxBT = cellMinBT + (cellSizeBT * 2);

								boolCallback.hit = false;
								triMeshShape->processAllTriangles(&boolCallback, cellMinBT, cellMaxBT);
								if (boolCallback.hit) {
									cell.worldCollision = true;
									totalCellsWithin++;
								}
							}
						}
					}
				}
			}
		}
	}

	RS_LOG(
		"SuspensionCollisionGrid::Setup(): Built suspension collision grid, " << 
		totalCellsWithin << "/" << CELL_AMOUNT_TOTAL << " cells contain world collision meshes"
	);
}

btCollisionObject* SuspensionCollisionGrid::CastSuspensionRay(btVehicleRaycaster* raycaster, Vec start, Vec end, btVehicleRaycaster::btVehicleRaycasterResult& result) {
	Cell& cell = GetCellFromPos(start * BT_TO_UU);

	if (cell.worldCollision || cell.dynamicObjects > 1) {
		return (btCollisionObject*)raycaster->castRay(start, end, result);
	} else {
		Vec delta = end - start;
		float dist = delta.Length();

		if (dist == 0)
			return NULL;

		Vec dir = delta / dist;

		float distToPlane = FLT_MAX;
		Vec planeNormal;
		if (end.z <= 0 || end.z >= RLConst::ARENA_HEIGHT * UU_TO_BT) {
			if (dir.z < 0) {
				static float groundHitZ = 5.96e-8;
				distToPlane = (groundHitZ - start.z) / dir.z;
				planeNormal = Vec(0, 0, 1);
			} else {
				distToPlane = (RLConst::ARENA_HEIGHT * UU_TO_BT - start.z) / dir.z; // NOTE: Could theoretically be NAN
				planeNormal = Vec(0, 0, -1);
			}
		} else {
			distToPlane = abs(abs(start.x) - RLConst::ARENA_EXTENT_X * UU_TO_BT) / abs(dir.x);
			planeNormal = Vec(-RS_SGN(end.x), 0, 0);
		}

		if (distToPlane < dist) {
			result.m_distFraction = distToPlane / dist;
			result.m_hitPointInWorld = start + dir * distToPlane;
			result.m_hitNormalInWorld = planeNormal;
			return defaultWorldCollisionRB;
		} else {
			return NULL;
		}
	}
}
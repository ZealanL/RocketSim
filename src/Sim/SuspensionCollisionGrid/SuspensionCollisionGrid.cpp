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
	int totalCellsBled = 0;
	BoolHitTriangleCallback boolCallback = BoolHitTriangleCallback();

	Vec cellSizeBT = GetCellSize() * UU_TO_BT;

	// Enable cell.worldCollision for all cells that contain one or more triangle mesh's geometry
	for (btBvhTriangleMeshShape* triMeshShape : triMeshShapes) {	
		btVector3 rbMinBT, rbMaxBT;
		triMeshShape->getAabb(btTransform(), rbMinBT, rbMaxBT);

		for (int i = 0; i < CELL_AMOUNT_X; i++) {

			Vec
				cellPlaneMinBT = GetCellMin(i, 0, 0) * UU_TO_BT,
				cellPlaneMaxBT = GetCellMin(i, CELL_AMOUNT_Y - 1, CELL_AMOUNT_Z - 1) * UU_TO_BT + cellSizeBT;

			boolCallback.hit = false;
			triMeshShape->processAllTriangles(&boolCallback, cellPlaneMinBT, cellPlaneMaxBT);
			if (boolCallback.hit) {
				for (int j = 0; j < CELL_AMOUNT_Y; j++) {

					Vec
						cellColumnMinBT = GetCellMin(i, j, 0) * UU_TO_BT,
						cellColumnMaxBT = GetCellMin(i, j, CELL_AMOUNT_Z - 1) * UU_TO_BT + cellSizeBT;

					boolCallback.hit = false;
					triMeshShape->processAllTriangles(&boolCallback, cellColumnMinBT, cellColumnMaxBT);

					if (boolCallback.hit) {
						for (int k = 0; k < CELL_AMOUNT_Z; k++) {

							Cell& cell = Get(i, j, k);

							if (!cell.worldCollision) {

								Vec
									cellMinBT = GetCellMin(i, j, k) * UU_TO_BT,
									cellMaxBT = cellMinBT + cellSizeBT;

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

	SuspensionCollisionGrid clone;
	clone.Allocate();

	// Make cell.worldCollision bleed to all surrounding cells
	for (int i = 0; i < CELL_AMOUNT_X; i++) {
		for (int j = 0; j < CELL_AMOUNT_Y; j++) {
			for (int k = 0; k < CELL_AMOUNT_Z; k++) {

				Cell& cell = Get(i, j, k);
				if (cell.worldCollision) {
					for (int i2 = -1; i2 < 2; i2++) {
						for (int j2 = -1; j2 < 2; j2++) {
							for (int k2 = -1; k2 < 2; k2++) {

								Cell& otherCell = clone.Get(
									RS_CLAMP(i + i2, 0, CELL_AMOUNT_X - 1),
									RS_CLAMP(j + j2, 0, CELL_AMOUNT_Y - 1),
									RS_CLAMP(k + k2, 0, CELL_AMOUNT_Z - 1)
								);

								if (!otherCell.worldCollision)
									totalCellsBled++;
								otherCell.worldCollision = true;
							}
						}
					}
				}
			}
		}
	}

	*this = clone;

	RS_LOG(
		"SuspensionCollisionGrid::Setup(): Built suspension collision grid, " <<
		totalCellsWithin << "/" << CELL_AMOUNT_TOTAL << " cells contain world collision meshes, " <<
		"bled to an additional " << totalCellsBled << " surrounding cells."
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

void SuspensionCollisionGrid::UpdateDynamicCollisions(Vec minBT, Vec maxBT, bool remove) {
	int deltaVal = remove ? -1 : 1;

	int i1, j1, k1;
	GetCellIndicesFromPos(minBT * BT_TO_UU - GetCellSize(), i1, j1, k1);

	int i2, j2, k2;
	GetCellIndicesFromPos((maxBT * BT_TO_UU + GetCellSize()), i2, j2, k2);

	for (int i = i1; i <= i2; i++)
		for (int j = j1; j <= j2; j++)
			for (int k = k1; k <= k2; k++)
				Get(i, j, k).dynamicObjects += deltaVal;
}
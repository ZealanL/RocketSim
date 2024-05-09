#include "SuspensionCollisionGrid.h"

#include "../../../libsrc/bullet3-3.24/BulletCollision/NarrowPhaseCollision/btRaycastCallback.h"
#include "../../../libsrc/bullet3-3.24/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"

RS_NS_START

// Quick virtual btTriangleCallback child class to simply check if any triangles are processed
struct BoolHitTriangleCallback : public btTriangleCallback {

	bool hit = false;
	
	BoolHitTriangleCallback() {}
	virtual void processTriangle(btVector3* triangle, int partId, int triangleIndex) {
		hit = true;
	}
};

template <bool LIGHT>
void _SetupWorldCollision(SuspensionCollisionGrid& grid, const std::vector<btBvhTriangleMeshShape*>& triMeshShapes) {

	int totalCellsWithin = 0;
	int totalCellsBled = 0;
	BoolHitTriangleCallback boolCallback = BoolHitTriangleCallback();

	Vec cellSizeBT = grid.GetCellSize<LIGHT>() * UU_TO_BT;

	// Enable cell.worldCollision for all cells that contain one or more triangle mesh's geometry
	for (btBvhTriangleMeshShape* triMeshShape : triMeshShapes) {	
		btVector3 rbMinBT, rbMaxBT;
		triMeshShape->getAabb(btTransform(), rbMinBT, rbMaxBT);

		for (int i = 0; i < grid.CELL_AMOUNT_X[LIGHT]; i++) {

			Vec
				cellPlaneMinBT = grid.GetCellMin<LIGHT>(i, 0, 0) * UU_TO_BT,
				cellPlaneMaxBT = grid.GetCellMin<LIGHT>(i, grid.CELL_AMOUNT_Y[LIGHT] - 1, grid.CELL_AMOUNT_Z[LIGHT] - 1) * UU_TO_BT + cellSizeBT;

			boolCallback.hit = false;
			triMeshShape->processAllTriangles(&boolCallback, cellPlaneMinBT, cellPlaneMaxBT);
			if (boolCallback.hit) {
				for (int j = 0; j < grid.CELL_AMOUNT_Y[LIGHT]; j++) {

					Vec
						cellColumnMinBT = grid.GetCellMin<LIGHT>(i, j, 0) * UU_TO_BT,
						cellColumnMaxBT = grid.GetCellMin<LIGHT>(i, j, grid.CELL_AMOUNT_Z[LIGHT] - 1) * UU_TO_BT + cellSizeBT;

					boolCallback.hit = false;
					triMeshShape->processAllTriangles(&boolCallback, cellColumnMinBT, cellColumnMaxBT);

					if (boolCallback.hit) {
						for (int k = 0; k < grid.CELL_AMOUNT_Z[LIGHT]; k++) {

							SuspensionCollisionGrid::Cell& cell = grid.Get<LIGHT>(i, j, k);

							if (!cell.worldCollision) {

								Vec
									cellMinBT = grid.GetCellMin<LIGHT>(i, j, k) * UU_TO_BT,
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

	SuspensionCollisionGrid clone = SuspensionCollisionGrid(grid.gameMode, grid.lightMem);
	clone.Allocate();

	// Make cell.worldCollision bleed to all surrounding cells
	for (int i = 0; i < grid.CELL_AMOUNT_X[LIGHT]; i++) {
		for (int j = 0; j < grid.CELL_AMOUNT_Y[LIGHT]; j++) {
			for (int k = 0; k < grid.CELL_AMOUNT_Z[LIGHT]; k++) {

				SuspensionCollisionGrid::Cell& cell = grid.Get<LIGHT>(i, j, k);
				if (cell.worldCollision) {
					for (int i2 = -1; i2 < 2; i2++) {
						for (int j2 = -1; j2 < 2; j2++) {
							for (int k2 = -1; k2 < 2; k2++) {

								SuspensionCollisionGrid::Cell& otherCell = clone.Get<LIGHT>(
									RS_CLAMP(i + i2, 0, grid.CELL_AMOUNT_X[LIGHT] - 1),
									RS_CLAMP(j + j2, 0, grid.CELL_AMOUNT_Y[LIGHT] - 1),
									RS_CLAMP(k + k2, 0, grid.CELL_AMOUNT_Z[LIGHT] - 1)
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

	grid = clone;

	RS_LOG(
		"SuspensionCollisionGrid::Setup(): Built suspension collision grid, " <<
		totalCellsWithin << "/" << grid.CELL_AMOUNT_TOTAL[LIGHT] << " cells contain world collision meshes, " <<
		"bled to an additional " << totalCellsBled << " surrounding cells."
	);
}

void SuspensionCollisionGrid::SetupWorldCollision(const std::vector<btBvhTriangleMeshShape*>& triMeshShapes) {
	if (lightMem) {
		_SetupWorldCollision<true>(*this, triMeshShapes);
	} else {
		_SetupWorldCollision<false>(*this, triMeshShapes);
	}
}

template <bool LIGHT>
btCollisionObject* _CastSuspensionRay(
	SuspensionCollisionGrid& grid, btVehicleRaycaster* raycaster, 
	Vec start, Vec end, const btCollisionObject* ignoreObj, btVehicleRaycaster::btVehicleRaycasterResult& result
) {
	SuspensionCollisionGrid::Cell& cell = grid.GetCellFromPos<LIGHT>(start * BT_TO_UU);

	if (cell.worldCollision || cell.dynamicCollision) {
		// TODO: Do world-only or dynamic-only raycasts
		return (btCollisionObject*)raycaster->castRay(start, end, ignoreObj, result);
	} else {
		Vec delta = end - start;
		float dist = delta.Length();

		if (dist == 0)
			return NULL;

		Vec dir = delta / dist;

		float distToPlane = FLT_MAX;
		Vec planeNormal;
		if (end.z <= 0 || end.z >= grid.cache.height_bt) {
			if (dir.z < 0) {
				static float groundHitZ = 5.96e-8;
				distToPlane = (groundHitZ - start.z) / dir.z;
				planeNormal = Vec(0, 0, 1);
			} else {
				distToPlane = (grid.cache.height_bt - start.z) / dir.z; // NOTE: Could theoretically be NAN
				planeNormal = Vec(0, 0, -1);
			}
		} else {
			if (RS_SGN(dir.x) == RS_SGN(start.x)) {
				distToPlane = abs(abs(start.x) - grid.cache.extentX_bt) / abs(dir.x);
				planeNormal = Vec(-RS_SGN(end.x), 0, 0);
			}

			if (grid.gameMode == GameMode::HOOPS && RS_SGN(dir.y) == RS_SGN(start.y)) {
				distToPlane = abs(abs(start.y) - grid.cache.extentY_bt) / abs(dir.y);
				planeNormal = Vec(0, -RS_SGN(end.y), 0);
			}
		}

		if (distToPlane < dist) {
			result.m_distFraction = distToPlane / dist;
			result.m_hitPointInWorld = start + dir * distToPlane;
			result.m_hitNormalInWorld = planeNormal;
			return grid.defaultWorldCollisionRB;
		} else {
			return NULL;
		}
	}
}

btCollisionObject* SuspensionCollisionGrid::CastSuspensionRay(btVehicleRaycaster* raycaster, Vec start, Vec end, const btCollisionObject* ignoreObj, btVehicleRaycaster::btVehicleRaycasterResult& result) {
	if (lightMem) {
		return _CastSuspensionRay<true>(*this, raycaster, start, end, ignoreObj, result);
	} else {
		return _CastSuspensionRay<false>(*this, raycaster, start, end, ignoreObj, result);
	}
}

template <bool LIGHT>
void _UpdateDynamicCollisions(SuspensionCollisionGrid& grid, Vec minBT, Vec maxBT, bool remove) {
	int deltaVal = remove ? -1 : 1;

	int i1, j1, k1;
	grid.GetCellIndicesFromPos<LIGHT>(minBT * BT_TO_UU - grid.GetCellSize<LIGHT>(), i1, j1, k1);

	int i2, j2, k2;
	grid.GetCellIndicesFromPos<LIGHT>((maxBT * BT_TO_UU + grid.GetCellSize<LIGHT>()), i2, j2, k2);

	for (int i = i1; i <= i2; i++)
		for (int j = j1; j <= j2; j++)
			for (int k = k1; k <= k2; k++)
				grid.Get<LIGHT>(i, j, k).dynamicCollision = true;

	grid.dynamicCellRanges.push_back(
		{
			i1, j1, k1,
			i2, j2, k2
		}
	);
}

void SuspensionCollisionGrid::UpdateDynamicCollisions(Vec minBT, Vec maxBT, bool remove) {
	if (lightMem) {
		return _UpdateDynamicCollisions<true>(*this, minBT, maxBT, remove);
	} else {
		return _UpdateDynamicCollisions<false>(*this, minBT, maxBT, remove);
	}
}

template <bool LIGHT>
void _ClearDynamicCollisions(SuspensionCollisionGrid& grid) {
	for (auto& range : grid.dynamicCellRanges) {
		for (int i = range.minX; i <= range.maxX; i++)
			for (int j = range.minY; j <= range.maxY; j++)
				for (int k = range.minZ; k <= range.maxZ; k++)
					grid.Get<LIGHT>(i, j, k).dynamicCollision = false;
	}

	grid.dynamicCellRanges.clear();
}

void SuspensionCollisionGrid::ClearDynamicCollisions() {
	if (lightMem) {
		return _ClearDynamicCollisions<true>(*this);
	} else {
		return _ClearDynamicCollisions<false>(*this);
	}
}

RS_NS_END
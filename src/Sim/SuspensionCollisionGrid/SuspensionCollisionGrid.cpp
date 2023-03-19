#include "SuspensionCollisionGrid.h"

void SuspensionCollisionGrid::Setup(const vector<btRigidBody*>& worldCollisionRBs) {
	int totalCellsWithin = 0;
	
	for (btRigidBody* rb : worldCollisionRBs) {
		auto type = rb->getCollisionShape()->getShapeType();
		if (type == STATIC_PLANE_PROXYTYPE)
			continue;

		btVector3 rbMinBT, rbMaxBT;
		rb->getAabb(rbMinBT, rbMaxBT);

		btVector3 borderSize = (GetCellSize() * UU_TO_BT);
		rbMinBT -= borderSize;
		rbMaxBT += borderSize;

		for (int i = 0; i < CELL_AMOUNT_X; i++) {
			for (int j = 0; j < CELL_AMOUNT_Y; j++) {
				for (int k = 0; k < CELL_AMOUNT_Z; k++) {

					if (!cells[i][j][k].worldCollision) {
						Vec
							cellMinBT = GetCellMin(i, j, k) * UU_TO_BT,
							cellMaxBT = cellMinBT + (GetCellSize() * UU_TO_BT);

						if ((cellMinBT < rbMaxBT) && (cellMaxBT > rbMinBT)) {
							totalCellsWithin++;
							cells[i][j][k].worldCollision = true;
						}
					}
				}
			}
		}
	}

	if (worldCollisionRBs.empty()) {
		defaultWorldColObj = NULL;
	} else {
		defaultWorldColObj = worldCollisionRBs.front();
	}

	RS_LOG(
		"SuspensionCollisionGrid::Setup(): Built suspension collision grid, " << 
		totalCellsWithin << "/" << (CELL_AMOUNT_X * CELL_AMOUNT_Y * CELL_AMOUNT_Z) << " cells contain world collision meshes"
	);
}

btCollisionObject* SuspensionCollisionGrid::CastSuspensionRay(btVehicleRaycaster* raycaster, Vec start, Vec end, btVehicleRaycaster::btVehicleRaycasterResult& result) {
	Cell& cell1 = GetCellFromPos(start * BT_TO_UU);
	Cell& cell2 = GetCellFromPos(end * BT_TO_UU);

	if ((cell1.worldCollision || cell2.worldCollision) || (RS_MAX(cell1.dynamicObjects, cell2.dynamicObjects) > 1)) {
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
			return defaultWorldColObj;
		} else {
			return NULL;
		}
	}
}
#pragma once
#include "../Car/Car.h"

struct SuspensionCollisionGrid {
	constexpr static float
		EXTENT_X = RLConst::ARENA_EXTENT_X,
		EXTENT_Y = RLConst::ARENA_EXTENT_Y * 1.5f, // Extends into goal
		HEIGHT = RLConst::ARENA_HEIGHT;

	constexpr static int
		CELL_AMOUNT_X = 128,
		CELL_AMOUNT_Y = 224,
		CELL_AMOUNT_Z = 32,

		CELL_AMOUNT_TOTAL = (CELL_AMOUNT_X * CELL_AMOUNT_Y * CELL_AMOUNT_Z);

	constexpr static float
		CELL_SIZE_X = EXTENT_X / (CELL_AMOUNT_X / 2),
		CELL_SIZE_Y = EXTENT_Y / (CELL_AMOUNT_Y / 2),
		CELL_SIZE_Z = HEIGHT / CELL_AMOUNT_Z;

	static_assert(
		RS_MIN(CELL_SIZE_X, RS_MIN(CELL_SIZE_Y, CELL_SIZE_Z)) > 60, 
		"Cell sizes are too small, could lead to missed collisions"
	);

	struct Cell {
		bool worldCollision = false;
		int dynamicObjects = 0; // TODO: Implement dynamic grid update
	};

	vector<Cell> cellData;

	void Allocate() {
		cellData.resize(CELL_AMOUNT_TOTAL);
	}

	Cell& Get(int i, int j, int k) {
		int index = (i * CELL_AMOUNT_Y * CELL_AMOUNT_Z) + (j * CELL_AMOUNT_Z) + k;
		assert(index >= 0 && index < CELL_AMOUNT_TOTAL);
		return cellData[index];
	}

	Cell Get(int i, int j, int k) const {
		int index = (i * CELL_AMOUNT_Y * CELL_AMOUNT_Z) + (j * CELL_AMOUNT_Z) + k;
		assert(index >= 0 && index < CELL_AMOUNT_TOTAL);
		return cellData[index];
	}

	Vec GetCellMin(int xIndex, int yIndex, int zIndex) {
		return Vec(
			CELL_SIZE_X * (xIndex - (CELL_AMOUNT_X / 2)),
			CELL_SIZE_Y * (yIndex - (CELL_AMOUNT_Y / 2)),
			CELL_SIZE_Z * zIndex
		);
	}

	Cell& GetCellFromPos(Vec pos) {
		int 
			i = RS_CLAMP(pos.x / CELL_SIZE_X + (CELL_AMOUNT_X / 2), 0, CELL_AMOUNT_X - 1),
			j = RS_CLAMP(pos.y / CELL_SIZE_Y + (CELL_AMOUNT_Y / 2), 0, CELL_AMOUNT_Y - 1),
			k = RS_CLAMP(pos.z / CELL_SIZE_Z, 0, CELL_AMOUNT_Z - 1);

		return Get(i, j, k);
	}

	Vec GetCellSize() {
		return Vec(CELL_SIZE_X, CELL_SIZE_Y, CELL_SIZE_Z);
	}

	void SetupWorldCollision(const vector<btBvhTriangleMeshShape*>& triMeshShapes);
	btCollisionObject* CastSuspensionRay(btVehicleRaycaster* raycaster, Vec start, Vec end, btVehicleRaycaster::btVehicleRaycasterResult& result);

	btRigidBody* defaultWorldCollisionRB = NULL;
};

#include "DropshotTiles.h"

#include "../../../../libsrc/bullet3-3.24/BulletCollision/CollisionShapes/btConvexHullShape.h"

RS_NS_START

static Vec g_TilePositions[RLConst::Dropshot::NUM_TILES_PER_TEAM] = {};

// NOTE: Neighbors include the starting tile
static std::vector<int> g_TileNeighbors1[RLConst::Dropshot::NUM_TILES_PER_TEAM] = {};
static std::vector<int> g_TileNeighbors2[RLConst::Dropshot::NUM_TILES_PER_TEAM] = {};

Vec DropshotTiles::GetTilePos(int team, int index) {
	using namespace RLConst;

	assert(team >= 0 && team <= 1);
	assert(index >= 0 && team < Dropshot::NUM_TILES_PER_TEAM);

	return g_TilePositions[index] * ((team == 0) ? -1 : 1);
}

std::vector<btCollisionShape*> DropshotTiles::MakeTileShapes() {
	using namespace RLConst;

	auto results = std::vector<btCollisionShape*>();

	for (int team = 0; team <= 1; team++) {
		for (int i = 0; i < Dropshot::NUM_TILES_PER_TEAM; i++) {
			Vec pos = GetTilePos(team, i);

			btConvexHullShape* hullShape = new btConvexHullShape();

			for (int j = 0; j < 6; j++) {

				Vec vert = pos * UU_TO_BT + Dropshot::TILE_HEXAGON_VERTS_BT[j];

				constexpr float CLAMP_Y = Dropshot::TILE_OFFSET_Y * UU_TO_BT;

				// Clamp vert from crossing middle part at x=0
				if (team == 0) {
					// Clamp max to CLAMP_Y
					vert.y = RS_MIN(vert.y, -CLAMP_Y);
				} else {
					// Clamp min to CLAMP_Y
					vert.y = RS_MAX(vert.y, CLAMP_Y);
				}

				hullShape->addPoint(vert);
			}

			hullShape->recalcLocalAabb();
			btVector3 localInertia;
			hullShape->calculateLocalInertia(0, localInertia);

			results.push_back(hullShape);
		}
	}

	return results;
}

void DropshotTiles::Init() {
	using namespace RLConst;

	{ // Generate g_TilePositions by making rows along X

		int curIdx = 0;
		float y = Dropshot::TILE_OFFSET_Y;
		for (
			int i = 0, numTiles = Dropshot::TILES_IN_FIRST_ROW;
			i < Dropshot::NUM_TILE_ROWS;
			i++, numTiles--) {

			// Generate column (along x)
			float rowSizeX = Dropshot::TILE_WIDTH_X * numTiles;
			float rowStartX = -(rowSizeX / 2.f) + (Dropshot::TILE_WIDTH_X/2);

			for (int j = 0; j < numTiles; j++, curIdx++) {
				if (curIdx > Dropshot::NUM_TILES_PER_TEAM)
					RS_ERR_CLOSE("DropshotTiles::Init(): Exceeded maximum tile count, make sure tile info is correct");

				float x = rowStartX + Dropshot::TILE_WIDTH_X * j;
				g_TilePositions[curIdx] = Vec(x, y, 0);
			}

			y += Dropshot::ROW_OFFSET_Y;
		}

		if (curIdx < Dropshot::NUM_TILES_PER_TEAM)
			RS_ERR_CLOSE("DropshotTiles::Init(): Failed to reach tile amount, make sure tile info is correct");
	}

	{ // Generate g_TileNeighbors

		constexpr float NEIGHBOR_MAX_RADIUS = Dropshot::TILE_WIDTH_X * 1.2f;
		int maxNeighbors1 = 0, maxNeighbors2 = 0;
		for (int i = 0; i < Dropshot::NUM_TILES_PER_TEAM; i++) {
			Vec pos = GetTilePos(0, i);
			auto& neighborMap1 = g_TileNeighbors1[i];
			auto& neighborMap2 = g_TileNeighbors2[i];
			for (int j = 0; j < Dropshot::NUM_TILES_PER_TEAM; j++) {
				Vec otherPos = GetTilePos(0, j);
				if (pos.Dist(otherPos) < NEIGHBOR_MAX_RADIUS)
					neighborMap1.push_back(j);
				if (pos.Dist(otherPos) < NEIGHBOR_MAX_RADIUS * 2)
					neighborMap2.push_back(j);

				maxNeighbors1 = RS_MAX(maxNeighbors1, neighborMap1.size());
				maxNeighbors2 = RS_MAX(maxNeighbors2, neighborMap2.size());
			}
		}

		if (maxNeighbors1 > 7 || maxNeighbors2 > 20)
			RS_ERR_CLOSE("DropshotTiles::Init(): Too high neighbor count, tile placement may be incorrect");
	}
}

// TODO: Return reference instead of copy
std::vector<int> DropshotTiles::GetNeighborIndices(int startIdx, int radius) {
	if (radius < 1 || radius > 3)
		RS_ERR_CLOSE("DropshotTiles::GetNeighborIndices(): Radius must be from 1-3");

	if (radius == 1) {
		return { startIdx };
	} else if (radius == 2) {
		return g_TileNeighbors1[startIdx];
	} else {
		return g_TileNeighbors2[startIdx];
	}
}

RS_NS_END
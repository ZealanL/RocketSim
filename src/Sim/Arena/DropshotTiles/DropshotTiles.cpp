#include "DropshotTiles.h"

#include "../../../../libsrc/bullet3-3.24/BulletCollision/CollisionShapes/btConvexHullShape.h"

RS_NS_START

Vec DropshotTiles::GetTilePos(int team, int index) {
	using namespace RLConst;

	Vec tilePositions[Dropshot::NUM_TILES_PER_TEAM];

	// Build tile lookup table
	static std::once_flag onceFlag;
	std::call_once(onceFlag,
		[&tilePositions] {

			// Generate rows of tiles (along y)
			int curIdx = 0;
			float y = Dropshot::TILE_OFFSET_Y;
			for (
				int i = 0, numTiles = Dropshot::TILES_IN_FIRST_ROW; 
				i < Dropshot::NUM_TILE_ROWS; 
				i++, numTiles--) {

				// Generate columns (along x)
				float rowSizeX = Dropshot::TILE_SIZE_X * numTiles;
				float rowStartX = -(rowSizeX / 2);
				for (int j = 0; j < numTiles; j++, curIdx++) {
					float x = rowStartX + Dropshot::TILE_SIZE_X * j;
					tilePositions[curIdx] = Vec(x, y, 0);
				}

				y += Dropshot::TILE_OFFSET_Y;
			}
				
		}
	);

	return tilePositions[index] * ((team == 0) ? -1 : 1);
}

std::vector<btCollisionShape*> DropshotTiles::MakeTileShapes() {
	using namespace RLConst;

	auto results = std::vector<btCollisionShape*>();

	for (int team = 0; team <= 1; team++) {
		for (int i = 0; i < Dropshot::NUM_TILES_PER_TEAM; i++) {
			Vec pos = GetTilePos(team, i);

			// TODO: Hardcoding 6 is not that unreasonable but a little lame, I should add "RS_ARRAYSIZE()"...
			Vec verts[6]; 
			for (int j = 0; j < 6; j++) {
				Vec vert = pos + Dropshot::TILE_HEXAGON_VERTS_BT[j] * UU_TO_BT;

				// Clamp vert from crossing x=0
				if (team == 0) {
					// Clamp max to 0
					vert.y = RS_MIN(vert.y, 0);
				} else {
					// Clamp min to 0
					vert.y = RS_MAX(vert.y, 0);
				}
			}

			btConvexHullShape* hullShape = new btConvexHullShape((const btScalar*)verts, 6, sizeof(Vec));
			results.push_back(hullShape);
		}
	}

	return results;
}

RS_NS_END
#pragma once
#include "../../../RLConst.h"
#include "../../../../libsrc/bullet3-3.24/BulletDynamics/Dynamics/btRigidBody.h"

RS_NS_START

struct DropshotTilesState {
	enum {
		STATE_FULL,
		STATE_DAMAGED,
		STATE_BROKEN
	};

	uint8_t states[RLConst::Dropshot::TEAM_AMOUNT][RLConst::Dropshot::NUM_TILES_PER_TEAM] = {};

	DropshotTilesState() {
		memset(states, 0, sizeof(states));
	}
};

namespace DropshotTiles {
	Vec GetTilePos(int team, int index);

	std::vector<btCollisionShape*> MakeTileShapes();
};

RS_NS_END
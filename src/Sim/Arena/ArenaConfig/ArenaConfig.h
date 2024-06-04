#pragma once
#include "../../../Math/MathTypes/MathTypes.h"
#include "../../../DataStream/DataStreamOut.h"
#include "../../../DataStream/DataStreamIn.h"

#include "../../BoostPad/BoostPad.h"

RS_NS_START

// Mode of speed/memory optimization for the arena
// Will affect whether high memory consumption is used to slightly increase speed or not
enum class ArenaMemWeightMode : byte {
	HEAVY, // ~1,263KB per arena with 4 cars
	LIGHT  // ~383KB per arena with 4 cars
	// Measurements last updated 2024/5/9
};

struct ArenaConfig {

	ArenaMemWeightMode memWeightMode = ArenaMemWeightMode::HEAVY;

	// Mininimum and maximum positions all physics objects
	Vec minPos = Vec(-4500, -6000, 0),
		maxPos = Vec( 4500,  6000, 2500);
	
	// Maximum length of any object
	// Calculated as the distance from AABB min to AABB max
	float
		maxAABBLen = 370;

	// Ball rotation updates are skipped to improve performance
	// Disabled in snowday
	bool noBallRot = true;

	// Use a custom broadphase designed for RocketSim
	// Improves performance, but becomes inefficient on giant maps
	// Turn this off if you want to use a giant map
	bool useCustomBroadphase = true;

	// Maximum number of objects
	int maxObjects = 512;

	// Use a custom list of boost pads (customBoostPads) instead of the normal one
	// NOTE: This will disable the boost pad grid and will thus worsen performance
	bool useCustomBoostPads = false;
	std::vector<BoostPadConfig> customBoostPads = {}; // Custom boost pads to use, if useCustomBoostPads

	void Serialize(DataStreamOut& out) const;
	void Deserialize(DataStreamIn& in);
};

#define ARENA_CONFIG_SERIALIZATION_FIELDS \
minPos, maxPos, maxAABBLen, noBallRot, useCustomBroadphase

RS_NS_END
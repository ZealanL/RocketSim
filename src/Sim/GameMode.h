#pragma once
#include "../Framework.h"

RS_NS_START

enum class GameMode : byte {
	SOCCAR,
	HOOPS,
	HEATSEEKER,
	SNOWDAY,
	DROPSHOT,

	// I will not add rumble unless I am given a large amount of money, or, alternatively, a large amount of candy corn (I love candy corn)

	// Soccar but without goals, boost pads, or the arena hull. The cars and ball will fall infinitely.
	THE_VOID,
};

constexpr const char* GAMEMODE_STRS[] = {
	"soccar",
	"hoops",
	"heatseeker",
	"snowday",
	"dropshot",
	"void"
};

RS_NS_END
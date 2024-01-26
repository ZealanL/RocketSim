#pragma once
#include "../Framework.h"

RS_NS_START

enum class GameMode : byte {
	SOCCAR,
	HOOPS,
	HEATSEEKER,
	SNOWDAY,

	// More coming soon!

	// No goals, boosts, or arena - cars/ball will fall infinitely, ball is frozen until touched
	THE_VOID,
};

constexpr const char* GAMEMODE_STRS[] = {
	"soccar",
	"hoops",
	"heatseeker",
	"snowday"
};

RS_NS_END
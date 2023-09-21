#pragma once
#include "../Framework.h"

enum class GameMode : byte {
	SOCCAR,
	HOOPS,

	// More coming soon!

	// No goals, boosts, or arena - cars/ball will fall infinitely, ball is frozen until touched
	THE_VOID,
};

constexpr const char* GAMEMODE_STRS[] = {
	"soccar",
	"hoops"
};
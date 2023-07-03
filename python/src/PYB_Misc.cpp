#ifdef RS_PYBIND
#include "PYB.h"

PYB_INIT_F(Misc) {
	pyb::enum_<GameMode>(m, "GameMode")
		.value("SOCCAR", GameMode::SOCCAR)
		.value("THE_VOID", GameMode::THE_VOID);

	pyb::enum_<Team>(m, "Team")
		.value("BLUE", Team::BLUE)
		.value("ORANGE", Team::ORANGE);
}
#endif
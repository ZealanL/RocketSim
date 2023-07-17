#ifdef RS_PYBIND
#include "PYB.h"

PYB_INIT_F(BoostPad) {
#define PYB_CUR_CLASS BoostPadState
	PYB_CLASS(BoostPadState)
		PYB_DEFAULT_INITS()
		PYBP(cooldown)
		PYBP(curLockedCar)
		PYBP(isActive)
		PYBP(prevLockedCarID)
		PYB_SERIALS()
		;

#define PYB_CUR_CLASS BoostPad
	PYB_CLASS(BoostPad)
		.def("get_state", &BoostPad::GetState)
		.def("set_state", &BoostPad::SetState, PYBA("state"))
		PYBP(isBig)
		PYBP(pos)
		;
}
#endif
#ifdef RS_PYBIND
#include "PYB.h"

PYB_INIT_F(Math) {
	pyb::module_("Math")
		.def("rand_float", Math::RandFloat, PYBA("min")=1, PYBA("max")=2)
		.def("rand_int", Math::RandInt, PYBA("min")=1, PYBA("max")=2)
		.def("round_vec", Math::RoundVec)
		.def("wrap_normalize_float", Math::WrapNormalizeFloat, PYBA("val")=1, PYBA("range")=2)
		;
}
#endif
#ifdef RS_PYBIND
#include "PYB.h"

PYB_INIT_F(Math) {
	m.def_submodule("Math")
		.def("rand_float", Math::RandFloat, PYBA("min"), PYBA("max"))
		.def("rand_int", Math::RandInt, PYBA("min"), PYBA("max"), PYBA("seed") = int(-1))
		.def("round_vec", Math::RoundVec)

		.def("wrap_normalize_float", Math::WrapNormalizeFloat, PYBA("val"), PYBA("minmax"))
		;
}
#endif
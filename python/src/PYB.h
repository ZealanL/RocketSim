#pragma once
#ifdef RS_PYBIND
#include "../../src/RocketSim.h" 

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
namespace pyb = pybind11;

// Set this to the current class we are working with
#define PYB_CUR_CLASS null

#define PYB_CLASS_CUSTOM(className) pyb::class_<className>(m, #className)
#define PYB_CLASS() PYB_CLASS_CUSTOM(PYB_CUR_CLASS)
#define PYB_INIT_F(name) void PYB_INIT_##name(pyb::module& m)
#define PYB_DEFAULT_INITS() .def(pyb::init<>()) .def(pyb::init<const PYB_CUR_CLASS&>())
#define PYBA pyb::arg

#define PYBS(s) PYB_MakePythonString(s)

#define PYBP(memberName) .def_readwrite(PYBS(#memberName), &PYB_CUR_CLASS::memberName)

// NOTE: Literally leaks memory, but should only be called once for each string, so its fine
inline const char* PYB_MakePythonString(const char* name) {
	string* result = new string();
	char last = NULL;
	bool isInAcronym = false;
	for (const char* pc = name; *pc; pc++) {
		char c = *pc;
		if (isupper(c)) {
			if (last && isupper(last)) {
				isInAcronym = true;
			} else if (last) {
				*result += "_";
			}
		} else {
			if (isInAcronym) {
				*result += "_";
				isInAcronym = false;
			}
		}
		result += tolower(c);
		
		last = c;
	}
	return result->c_str();
}

PYBIND11_MODULE(RocketSim, m) {

	m.def("init", &RocketSim::Init, PYBA("arena_meshes_path"));
	m.def("is_in_init", []() { return RocketSim::GetStage() == RocketSimStage::INITIALIZING; });
	m.def("is_ready", []() { return RocketSim::GetStage() == RocketSimStage::INITIALIZED; });

	PYB_INIT_F(MathTypes);
	PYB_INIT_F(Math);

	PYB_INIT_F(Arena);
	PYB_INIT_F(Ball);
	PYB_INIT_F(BallHitInfo);
	PYB_INIT_F(BoostPad);
	PYB_INIT_F(Car);
	PYB_INIT_F(MutatorConfig);

	PYB_INIT_F(Misc);
}
#endif
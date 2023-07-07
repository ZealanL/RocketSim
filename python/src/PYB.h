#pragma once
#ifdef RS_PYBIND
#include "Wrappers.h"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/operators.h>
#include <pybind11/functional.h>
namespace pyb = pybind11;

// Set this to the current class we are working with
#define PYB_CUR_CLASS null

#define PYB_CLASS(className) pyb::class_<className>(m, #className)
#define PYB_INIT_F(name) void PYB_INIT_##name(pyb::module& m)
#define PYB_DEFAULT_INITS() .def(pyb::init<>()) .def(pyb::init<const PYB_CUR_CLASS&>())
#define PYBA pyb::arg

#define PYBS(s) PYB_MakePythonString(s)

#define PYBP(memberName) .def_readwrite(PYBS(#memberName), &PYB_CUR_CLASS::memberName)

#define PYBP_W(memberName) .def_property( \
	PYBS(#memberName), \
	[](const PYB_CUR_CLASS& inst) { return inst.ptr->memberName; }, \
	[](const PYB_CUR_CLASS& inst, const decltype(inst.ptr->memberName)& newVal) { inst.ptr->memberName = newVal; } \
	)

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
		*result += tolower(c);

		last = c;
	}
	return result->c_str();
}

PYB_INIT_F(MathTypes);
PYB_INIT_F(Math);

PYB_INIT_F(Arena);
PYB_INIT_F(Ball);
PYB_INIT_F(BallHitInfo);
PYB_INIT_F(BoostPad);
PYB_INIT_F(Car);
PYB_INIT_F(MutatorConfig);

PYB_INIT_F(Misc);
#endif
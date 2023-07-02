#ifdef RS_PYBIND
#include "PYB.h"

PYB_INIT_F(MathTypes) {
#define PYB_CUR_CLASS Vec
	PYB_CLASS()
		PYB_DEFAULT_INITS()
		.def(pyb::init<float, float, float>(), PYBA("x"), PYBA("y"), PYBA("z"))
		
		PYBP(x)
		PYBP(y)
		PYBP(z)

		.def("__getitem__", [](const Vec& v, size_t i) { return v[i]; })
		.def("__setitem__", [](Vec& v, size_t i, float f) { v[i] = f; })

		.def(pyb::self + pyb::self)
		.def(pyb::self - pyb::self)
		.def(pyb::self * pyb::self)
		.def(pyb::self / pyb::self)
		.def(pyb::self * float())
		.def(pyb::self / float())

		.def(pyb::self += pyb::self)
		.def(pyb::self -= pyb::self)
		.def(pyb::self *= pyb::self)
		.def(pyb::self /= pyb::self)
		.def(pyb::self *= float())
		.def(pyb::self /= float())

		.def("cross", &Vec::Cross)
		.def("dist", &Vec::Dist)
		.def("flat_dist", &Vec::Dist2D)
		.def("dist_sq", &Vec::DistSq)
		.def("flat_dist_sq", &Vec::DistSq2D)
		.def("dot", &Vec::Dot)
		.def("is_zero", &Vec::IsZero)
		.def("length", &Vec::Length)
		.def("length_sq", &Vec::LengthSq)
		.def("normalized", &Vec::Normalized)

		.def("__str__", [](const Vec& v) { return RS_STR(v); })
		;

#define PYB_CUR_CLASS RotMat
	PYB_CLASS()
		PYB_DEFAULT_INITS()
		.def(pyb::init<Vec, Vec, Vec>(), PYBA("forward"), PYBA("right"), PYBA("up"))
		.def("__getitem__", [](const RotMat& mat, std::pair <size_t, size_t> i) { return mat[i.first][i.second]; })
		.def("__setitem__", [](const RotMat& mat, std::pair <size_t, size_t> i, float val) { mat[i.first][i.second] = val; })

		PYBP(forward)
		PYBP(right)
		PYBP(up)

		.def_static("identity", &RotMat::GetIdentity)
		.def_static("look_at", &RotMat::LookAt, PYBA("forward"), PYBA("up"))
		.def("transposed", &RotMat::Transpose)
		.def("dot", [](const RotMat& mat, const Vec& vec) { return mat.Dot(vec); })
		.def("dot", [](const RotMat& mat, const RotMat& other) { return mat.Dot(other); })

		.def("__str__", [](const RotMat& mat) { return RS_STR(mat);  })
		;

#define PYB_CUR_CLASS Angle
	PYB_CLASS()
		PYB_DEFAULT_INITS()
		.def(pyb::init<float, float, float>(), PYBA("yaw"), PYBA("pitch"), PYBA("roll"))

		PYBP(yaw)
		PYBP(pitch)
		PYBP(roll)

		.def_static("from_rotmat", &Angle::FromRotMat)
		.def("to_rotmat", &Angle::ToRotMat)
		.def("get_delta_to", &Angle::GetDeltaTo)
		.def("get_forward_dir", &Angle::GetForwardVector)
		.def("fix", &Angle::NormalizeFix)

		.def("__str__", [](const Angle& ang) { return RS_STR(ang);  })
		;
}
#endif
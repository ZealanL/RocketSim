#ifdef RS_PYBIND
#include "PYB.h"

PYB_INIT_F(Ball) {
#define PYB_CUR_CLASS BallState
	PYB_CLASS()
		PYB_DEFAULT_INITS()
		PYBP(angVel)
		PYBP(pos)
		PYBP(vel)
		;

#define PYB_CUR_CLASS Ball
	PYB_CLASS()
		.def("get_radius", &Ball::GetRadius)
		.def("get_state", &Ball::GetState)
		.def("set_state", &Ball::SetState)

		.def("get_rot", [](const Ball& ball) { return (RotMat)(ball._rigidBody.m_worldTransform.m_basis);  })
		;
}
#endif
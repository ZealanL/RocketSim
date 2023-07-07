#ifdef RS_PYBIND
#include "PYB.h"

PYB_INIT_F(Ball) {
#define PYB_CUR_CLASS BallState
	PYB_CLASS(BallState)
		PYB_DEFAULT_INITS()
		PYBP(angVel)
		PYBP(pos)
		PYBP(vel)
		PYB_SERIALS()
		;

#define PYB_CUR_CLASS Ball
	pyb::class_<Ball>(m, "Ball")
		.def("get_radius", &Ball::GetRadius)
		.def("get_state", &Ball::GetState)
		.def("set_state", &Ball::SetState, PYBA("state"))
		.def("get_rot", [](std::shared_ptr<Ball> inst) { return (RotMat)(inst->_rigidBody.m_worldTransform.m_basis); })
		;
}
#endif
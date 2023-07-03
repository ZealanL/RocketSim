#ifdef RS_PYBIND
#include "PYB.h"

PYB_INIT_F(Ball) {
#define PYB_CUR_CLASS BallState
	PYB_CLASS(BallState)
		PYB_DEFAULT_INITS()
		PYBP(angVel)
		PYBP(pos)
		PYBP(vel)
		;

#define PYB_CUR_CLASS BallWrapper
	pyb::class_<BallWrapper>(m, "Ball")
		.def("get_radius", [](const BallWrapper& inst) { return inst.ptr->GetRadius(); })
		.def("get_state", [](const BallWrapper& inst) { return inst.ptr->GetState(); })
		.def("set_state", [](const BallWrapper& inst, const BallState& newState) { inst.ptr->SetState(newState); })
		.def("get_rot", [](const BallWrapper& inst) { return (RotMat)(inst.ptr->_rigidBody.m_worldTransform.m_basis);  })
		;
}
#endif
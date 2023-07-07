#ifdef RS_PYBIND
#include "PYB.h"

PYB_INIT_F(BallHitInfo) {
#define PYB_CUR_CLASS BallHitInfo
	PYB_CLASS(BallHitInfo)
		PYB_DEFAULT_INITS()
		PYBP(ballPos)
		PYBP(extraHitVel)
		PYBP(isValid)
		PYBP(relativePosOnBall)
		PYBP(tickCountWhenExtraImpulseApplied)
		PYBP(tickCountWhenHit)
		PYB_SERIALS()
		;
}
#endif
#include "BoostPad.h"

#include "../../RLConst.h"

#include "../../../libsrc/bullet3-3.24/BulletDynamics/Dynamics/btRigidBody.h"

RS_NS_START

BoostPad* BoostPad::_AllocBoostPad() {
	return new BoostPad();
}

void BoostPad::_Setup(bool isBig, Vec pos) {
	this->isBig = isBig;
	this->pos = pos;

	this->_posBT = pos * UU_TO_BT;

	{
		using namespace RLConst::BoostPads;

		float boxRad = (isBig ? BOX_RAD_BIG : BOX_RAD_SMALL) * UU_TO_BT;
		this->_boxMinBT = this->_posBT - Vec(boxRad, boxRad, 0);
		this->_boxMaxBT = this->_posBT + Vec(boxRad, boxRad, BOX_HEIGHT * UU_TO_BT);
	}
}

void BoostPad::_PreTickUpdate(float tickTime) {

	if (_internalState.cooldown > 0) {
		_internalState.cooldown = RS_MAX(_internalState.cooldown - tickTime, 0);
	}

	_internalState.isActive = (_internalState.cooldown == 0);

	_internalState.curLockedCar = NULL;
}

void BoostPad::_CheckCollide(Car* car) {
	using namespace RLConst::BoostPads;

	Vec carPosBT = car->_rigidBody.getWorldTransform().m_origin;

	bool colliding = false;
	if (_internalState.prevLockedCarID == car->id) {
		// Check with AABB-hitbox collision

		btVector3 carMinBT, carMaxBT;
		car->_rigidBody.getAabb(carMinBT, carMaxBT);

		// TODO: Account for orientation
		colliding = (_boxMaxBT > carMinBT) && (_boxMinBT < carMaxBT);
	} else {
		// Check with cylinder-origin collision

		float rad = (isBig ? CYL_RAD_BIG : CYL_RAD_SMALL) * UU_TO_BT;
		if (carPosBT.DistSq2D(this->_posBT) < (rad * rad))
			colliding = abs(carPosBT.z - this->_posBT.z) < (CYL_HEIGHT * UU_TO_BT);
	}

	if (colliding)
		_internalState.curLockedCar = car;
}

void BoostPad::_PostTickUpdate(float tickTime, const MutatorConfig& mutatorConfig) {
	using namespace RLConst::BoostPads;

	uint32_t lockedCarID = 0;
	if (_internalState.curLockedCar) {
		lockedCarID = _internalState.curLockedCar->id;

		if (_internalState.isActive) {
			float boostToAdd = isBig ? BOOST_AMOUNT_BIG : BOOST_AMOUNT_SMALL;
			_internalState.curLockedCar->_internalState.boost = RS_MIN(_internalState.curLockedCar->_internalState.boost + boostToAdd, 100);

			_internalState.isActive = false;
			_internalState.cooldown = isBig ? mutatorConfig.boostPadCooldown_Big : mutatorConfig.boostPadCooldown_Small;
		}
	}

	_internalState.prevLockedCarID = lockedCarID;
}

void BoostPadState::Serialize(DataStreamOut& out) const {
	out.WriteMultiple(
		BOOSTPAD_SERIALIZATION_FIELDS
	);
}

void BoostPadState::Deserialize(DataStreamIn& in) {
	in.ReadMultiple(
		BOOSTPAD_SERIALIZATION_FIELDS
	);
}

RS_NS_END
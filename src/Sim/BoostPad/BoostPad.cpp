#include "BoostPad.h"

#include "../../RLConst.h"

#include "../../../libsrc/bullet3-3.24/BulletDynamics/Dynamics/btRigidBody.h"

RS_NS_START

void BoostPadConfig::Serialize(DataStreamOut& out) const {
	out.WriteMultiple(
		BOOSTPADCONFIG_SERIALIZATION_FIELDS
	);
}

void BoostPadConfig::Deserialize(DataStreamIn& in) {
	in.ReadMultiple(
		BOOSTPADCONFIG_SERIALIZATION_FIELDS
	);
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

BoostPad* BoostPad::_AllocBoostPad() {
	return new BoostPad();
}

void BoostPad::_Setup(const BoostPadConfig& config) {
	this->config = config;

	this->_posBT = config.pos * UU_TO_BT;
}

void BoostPad::_PreTickUpdate(float tickTime) {

	// The game counts an inactive pad's timer down every tick, past zero, and brings the pad back on the tick after it
	// reads -1/120: 481 / 1201 ticks after the pickup, in time for a car on the pad to take it again that tick
	if (!_internalState.isActive || _internalState.cooldown > 0) {
		if (_internalState.cooldown < -0.5f * tickTime) {
			_internalState.cooldown = 0;
			_internalState.isActive = true;
		} else {
			_internalState.cooldown -= tickTime;
			_internalState.isActive = false;
		}
	}

	_internalState.curLockedCar = NULL;
}

void BoostPad::_CheckCollide(Car* car, float tickTime) {
	using namespace RLConst::BoostPads;

	// The game's pickups land on this tick or the next at random, about evenly when the car enters the pad's cylinder
	// halfway through the tick, so test the car's position halfway through the tick
	Vec carPosBT = car->_rigidBody.getWorldTransform().m_origin - car->_rigidBody.getLinearVelocity() * (0.5f * tickTime);

	// The game's pad and car collision cylinders, for every car, every tick
	bool colliding = false;
	float rad = (config.isBig ? CYL_RAD_BIG + CAR_CYL_RAD : CYL_RAD_SMALL + CAR_CYL_RAD) * UU_TO_BT;
	float padZBT = (config.pos.z - (config.isBig ? CYL_OFFSET_Z_BIG : CYL_OFFSET_Z_SMALL)) * UU_TO_BT;
	if (carPosBT.DistSq2D(this->_posBT) < (rad * rad))
		colliding = abs(carPosBT.z - padZBT) < ((CYL_HEIGHT + CAR_CYL_HEIGHT) * UU_TO_BT);

	if (colliding)
		_internalState.curLockedCar = car;
}

void BoostPad::_PostTickUpdate(float tickTime, const MutatorConfig& mutatorConfig) {
	using namespace RLConst::BoostPads;

	uint32_t lockedCarID = 0;
	if (_internalState.curLockedCar) {
		lockedCarID = _internalState.curLockedCar->id;

		if (_internalState.isActive) {
			float boostToAdd = config.isBig ? BOOST_AMOUNT_BIG : BOOST_AMOUNT_SMALL;
			_internalState.curLockedCar->_internalState.boost = RS_MIN(_internalState.curLockedCar->_internalState.boost + boostToAdd, RLConst::BOOST_MAX);

			_internalState.isActive = false;
			// The pickup tick already counts once (the game's timer reads 4 - 1/120 there)
			_internalState.cooldown = (config.isBig ? mutatorConfig.boostPadCooldown_Big : mutatorConfig.boostPadCooldown_Small) - tickTime;
		}
	}

	_internalState.prevLockedCarID = lockedCarID;
}

RS_NS_END

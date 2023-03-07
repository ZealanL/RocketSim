#include "BoostPad.h"

#include "../../RLConst.h"
#include "../Car/Car.h"

BoostPad* BoostPad::_AllocBoostPad() {
	return new BoostPad();
}

void BoostPad::_BulletSetup(btDynamicsWorld* bulletWorld, bool isBig, btVector3 pos) {
	using namespace RLConst::BoostPads;

	this->isBig = isBig;

	float sqRad = isBig ? SQ_RAD_BIG : SQ_RAD_SMALL;
	_collisionBoxShape = new btBoxShape(
		Vec(
			sqRad, sqRad, SQ_HEIGHT
		) * UU_TO_BT);

	_rigidBody = new btRigidBody(0, NULL, _collisionBoxShape);
	_rigidBody->setWorldTransform(btTransform(btMatrix3x3::getIdentity(), pos));
	
	// We aren't an actual solid object, just a collision check
	_rigidBody->m_collisionFlags |= btCollisionObject::CF_NO_CONTACT_RESPONSE;

	_rigidBody->setUserIndex(BT_USERINFO_TYPE_BOOSTPAD);
	_rigidBody->setUserPointer(this);

	bulletWorld->addRigidBody(_rigidBody);
}

void BoostPad::_PreTickUpdate(float tickTime) {

	if (_internalState.cooldown > 0) {
		_internalState.cooldown = RS_MAX(_internalState.cooldown - tickTime, 0);
	}

	_internalState.isActive = (_internalState.cooldown == 0);

	_internalState.curLockedCarObj = NULL;
}

void BoostPad::_OnCollide(btCollisionObject* other) {
	using namespace RLConst::BoostPads;

	if (other->getUserIndex() == BT_USERINFO_TYPE_CAR) {

		Car* otherCar = (Car*)other->getUserPointer();
		if (otherCar->_internalState.boost >= 100)
			return;
		if (otherCar->_internalState.isDemoed)
			return;

		bool colliding;

		if (_internalState.prevLockedCarObj == other) {
			colliding = true;
		} else {
			colliding = false;
			btVector3
				otherPos = other->getWorldTransform().getOrigin(),
				selfPos = _rigidBody->getWorldTransform().getOrigin();

			btVector3 deltaPos = otherPos - selfPos;

			if (deltaPos.z() < CYL_HEIGHT * UU_TO_BT) {

				float squareDeltaXY = (deltaPos.x() * deltaPos.x() + deltaPos.y() * deltaPos.y());
				float cylRad = (isBig ? CYL_RAD_BIG : CYL_RAD_SMALL) * UU_TO_BT;

				if (squareDeltaXY < (cylRad * cylRad)) {
					colliding = true;
				}
			}
		}

		if (colliding) {
			_internalState.curLockedCarObj = other;
		}
	}
}

void BoostPad::_PostTickUpdate(float tickTime) {
	using namespace RLConst::BoostPads;

	if (_internalState.curLockedCarObj && _internalState.isActive) {
		Car* car = (Car*)_internalState.curLockedCarObj->getUserPointer();
		
		float boostToAdd = isBig ? BOOST_AMOUNT_BIG : BOOST_AMOUNT_SMALL;
		car->_internalState.boost = RS_MIN(car->_internalState.boost + boostToAdd, 100);

		_internalState.isActive = false;
		_internalState.cooldown = isBig ? COOLDOWN_BIG : COOLDOWN_SMALL;
	}

	_internalState.prevLockedCarObj = _internalState.curLockedCarObj;
}

BoostPad::~BoostPad() {
	delete _rigidBody;
	delete _collisionBoxShape;
}

#pragma once
#include "../../BaseInc.h"

struct BoostPadState {
	bool isActive = true;
	float cooldown = 0;

	btCollisionObject* curLockedCarObj = NULL;
	btCollisionObject* prevLockedCarObj = NULL;
};

class BoostPad {
public:
	bool isBig;

	BoostPadState _internalState;

	btBoxShape* _collisionBoxShape;
	btRigidBody* _rigidBody;

	RSAPI BoostPadState GetState() { return _internalState; }
	RSAPI void SetState(const BoostPadState& state) { _internalState = state; }

	RSAPI Vec GetPos() {
		_rigidBody->getCenterOfMassTransform().getOrigin()* BT_TO_UU;
	}

	// For construction by Arena
	static BoostPad* _AllocBoostPad();
	void _BulletSetup(btDynamicsWorld* bulletWorld, bool isBig, btVector3 pos);

	// For callback from Arena
	void _OnCollide(btCollisionObject* other);

	void _PreTickUpdate(float tickTime);
	void _PostTickUpdate(float tickTime);

	~BoostPad();

private:
	BoostPad() {}
};
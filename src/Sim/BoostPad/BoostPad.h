#pragma once
#include "../../BaseInc.h"

#include "../../DataStream/DataStreamIn.h"
#include "../../DataStream/DataStreamOut.h"

struct BoostPadState {
	bool isActive = true;
	float cooldown = 0;

	btCollisionObject* curLockedCarObj = NULL;
	uint32_t prevLockedCarID = NULL;

	void Serialize(DataStreamOut& out);
	void Deserialize(DataStreamIn& in);
};
#define BOOSTPAD_SERIALIZATION_FIELDS \
isActive, cooldown, prevLockedCarID

class BoostPad {
public:
	bool isBig;

	BoostPadState _internalState;

	btBoxShape* _collisionBoxShape;
	btRigidBody* _rigidBody;

	RSAPI BoostPadState GetState() { return _internalState; }
	RSAPI void SetState(const BoostPadState& state) { _internalState = state; }

	RSAPI Vec GetPos() {
		return _rigidBody->getCenterOfMassTransform().getOrigin() * BT_TO_UU;
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
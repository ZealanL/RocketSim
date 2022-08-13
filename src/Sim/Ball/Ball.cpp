#include "Ball.h"

BallState Ball::GetState() {
	BallState stateOut;
	stateOut.pos = rigidBody->getWorldTransform().getOrigin() * BT_TO_UU;
	stateOut.vel = rigidBody->getLinearVelocity() * BT_TO_UU;
	stateOut.angVel = rigidBody->getAngularVelocity();
	return stateOut;
}

void Ball::SetState(const BallState& state) {
	btTransform newTransform;
	newTransform.setIdentity();
	newTransform.setOrigin(state.pos * UU_TO_BT);
	rigidBody->setWorldTransform(newTransform);
	rigidBody->setLinearVelocity(state.vel * UU_TO_BT);
	rigidBody->setAngularVelocity(state.angVel);
}

#include "Ball.h"

BallState Ball::GetState() {
	BallState stateOut;
	stateOut.pos = _rigidBody->getWorldTransform().getOrigin() * BT_TO_UU;
	stateOut.vel = _rigidBody->getLinearVelocity() * BT_TO_UU;
	stateOut.angVel = _rigidBody->getAngularVelocity();
	return stateOut;
}

void Ball::SetState(const BallState& state) {
	btTransform newTransform;
	newTransform.setIdentity();
	newTransform.setOrigin(state.pos * UU_TO_BT);
	_rigidBody->setWorldTransform(newTransform);
	_rigidBody->setLinearVelocity(state.vel * UU_TO_BT);
	_rigidBody->setAngularVelocity(state.angVel);
}

Ball* Ball::_AllocBall() {
	return new Ball();
}

Ball::~Ball() {
	delete _rigidBody;
	delete _collisionShape;
}
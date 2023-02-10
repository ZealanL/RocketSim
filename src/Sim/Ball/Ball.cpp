#include "Ball.h"

#include "../../RLConst.h";

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

void Ball::_BulletSetup(btDynamicsWorld* bulletWorld, float radius) {
	_collisionShape = new btSphereShape(radius);

	btRigidBody::btRigidBodyConstructionInfo constructionInfo =
		btRigidBody::btRigidBodyConstructionInfo(RLConst::BALL_MASS_BT, NULL, _collisionShape);

	// TODO: Move this code to Ball.cpp
	// TODO: Ball simulation is a tiny bit off when it comes to angular velocity loss on impact

	constructionInfo.m_startWorldTransform.setIdentity();
	constructionInfo.m_startWorldTransform.setOrigin(btVector3(0, 0, radius));

	btVector3 localInertial;
	_collisionShape->calculateLocalInertia(RLConst::BALL_MASS_BT, localInertial);

	constructionInfo.m_localInertia = localInertial;
	constructionInfo.m_linearDamping = RLConst::BALL_DRAG;
	constructionInfo.m_friction = RLConst::BALL_FRICTION;
	constructionInfo.m_restitution = RLConst::BALL_RESTITUTION;

	_rigidBody = new btRigidBody(constructionInfo);
	_rigidBody->setUserIndex(BT_USERINFO_TYPE_BALL);
	_rigidBody->setUserPointer(this);

	// Trigger the Arena::_BulletContactAddedCallback() when anything touches the ball
	_rigidBody->m_collisionFlags |= btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK;
}

Ball::~Ball() {
	delete _rigidBody;
	delete _collisionShape;
}
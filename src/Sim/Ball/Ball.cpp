#include "Ball.h"

#include "../../RLConst.h"

#include "../../../libsrc/bullet3-3.24/BulletDynamics/Dynamics/btDynamicsWorld.h"

void BallState::Serialize(DataStreamOut& out) {
	out.WriteMultiple(BALLSTATE_SERIALIZATION_FIELDS);
}

void BallState::Deserialize(DataStreamIn& in) {
	in.ReadMultiple(BALLSTATE_SERIALIZATION_FIELDS);
}

BallState Ball::GetState() {
	_internalState.pos = _rigidBody.getWorldTransform().getOrigin() * BT_TO_UU;
	_internalState.vel = _rigidBody.getLinearVelocity() * BT_TO_UU;
	_internalState.angVel = _rigidBody.getAngularVelocity();
	return _internalState;
}

void Ball::SetState(const BallState& state) {

	_internalState = state;

	btTransform newTransform;
	newTransform.setIdentity();
	newTransform.setOrigin(state.pos * UU_TO_BT);
	_rigidBody.setWorldTransform(newTransform);
	_rigidBody.setLinearVelocity(state.vel * UU_TO_BT);
	_rigidBody.setAngularVelocity(state.angVel);

	_velocityImpulseCache = { 0,0,0 };
}

void Ball::_BulletSetup(btDynamicsWorld* bulletWorld, const MutatorConfig& mutatorConfig) {
	_collisionShape = btSphereShape(mutatorConfig.ballRadius * UU_TO_BT);

	btRigidBody::btRigidBodyConstructionInfo constructionInfo =
		btRigidBody::btRigidBodyConstructionInfo(mutatorConfig.ballMass, NULL, &_collisionShape);

	constructionInfo.m_startWorldTransform.setIdentity();
	constructionInfo.m_startWorldTransform.setOrigin(btVector3(0, 0, mutatorConfig.ballRadius * UU_TO_BT));

	btVector3 localInertial;
	_collisionShape.calculateLocalInertia(mutatorConfig.ballMass, localInertial);

	constructionInfo.m_localInertia = localInertial;
	constructionInfo.m_linearDamping = mutatorConfig.ballDrag;
	constructionInfo.m_friction = mutatorConfig.ballWorldFriction;
	constructionInfo.m_restitution = mutatorConfig.ballWorldRestitution;

	_rigidBody = btRigidBody(constructionInfo);
	_rigidBody.setUserIndex(BT_USERINFO_TYPE_BALL);
	_rigidBody.setUserPointer(this);

	// Trigger the Arena::_BulletContactAddedCallback() when anything touches the ball
	_rigidBody.m_collisionFlags |= btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK;

	_rigidBody.m_rigidbodyFlags = 0;

	bulletWorld->addRigidBody(&_rigidBody);
}

void Ball::_FinishPhysicsTick(const MutatorConfig& mutatorConfig) {
	using namespace RLConst;

	// Add velocity cache
	if (!_velocityImpulseCache.IsZero()) {
		_rigidBody.m_linearVelocity += _velocityImpulseCache;
		_velocityImpulseCache = { 0,0,0 };
	}

	{ // Limit velocities
		btVector3
			vel = _rigidBody.m_linearVelocity,
			angVel = _rigidBody.m_angularVelocity;

		float ballMaxSpeedBT = mutatorConfig.ballMaxSpeed * UU_TO_BT;
		if (vel.length2() > ballMaxSpeedBT * ballMaxSpeedBT)
			vel = vel.normalized() * ballMaxSpeedBT;

		if (angVel.length2() > (BALL_MAX_ANG_SPEED * BALL_MAX_ANG_SPEED))
			angVel = angVel.normalized() * BALL_MAX_ANG_SPEED;

		_rigidBody.m_linearVelocity = vel;
		_rigidBody.m_angularVelocity = angVel;
	}

	{ // Round physics values to match RL
		_rigidBody.m_worldTransform.m_origin =
			Math::RoundVec(_rigidBody.m_worldTransform.m_origin, 0.01 * UU_TO_BT);

		_rigidBody.m_linearVelocity =
			Math::RoundVec(_rigidBody.m_linearVelocity, 0.01 * UU_TO_BT);

		_rigidBody.m_angularVelocity =
			Math::RoundVec(_rigidBody.m_angularVelocity, 0.00001);
	}
}

float Ball::GetRadiusBullet() {
	return _collisionShape.getRadius();
}
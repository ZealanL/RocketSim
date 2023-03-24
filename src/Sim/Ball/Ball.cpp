#include "Ball.h"

#include "../../RLConst.h"

void BallHitInfo::Serialize(DataStreamOut& out) {
	out.WriteMultiple(BALLHITINFO_SERIALIZATION_FIELDS);
}

void BallHitInfo::Deserialize(DataStreamIn& in) {
	in.ReadMultiple(BALLHITINFO_SERIALIZATION_FIELDS);
}

void BallState::Serialize(DataStreamOut& out) {
	ballHitInfo.Serialize(out);
	out.WriteMultiple(BALLSTATE_SERIALIZATION_FIELDS);
}

void BallState::Deserialize(DataStreamIn& in) {
	ballHitInfo.Deserialize(in);
	in.ReadMultiple(BALLSTATE_SERIALIZATION_FIELDS);
}

BallState Ball::GetState() {
	_internalState.pos = _rigidBody->getWorldTransform().getOrigin() * BT_TO_UU;
	_internalState.vel = _rigidBody->getLinearVelocity() * BT_TO_UU;
	_internalState.angVel = _rigidBody->getAngularVelocity();
	return _internalState;
}

void Ball::SetState(const BallState& state) {

	_internalState = state;

	btTransform newTransform;
	newTransform.setIdentity();
	newTransform.setOrigin(state.pos * UU_TO_BT);
	_rigidBody->setWorldTransform(newTransform);
	_rigidBody->setLinearVelocity(state.vel * UU_TO_BT);
	_rigidBody->setAngularVelocity(state.angVel);

	_velocityImpulseCache = { 0,0,0 };
}

Ball* Ball::_AllocBall() {
	return new Ball();
}

void Ball::_BulletSetup(btDynamicsWorld* bulletWorld, float radius) {
	_collisionShape = new btSphereShape(radius);

	btRigidBody::btRigidBodyConstructionInfo constructionInfo =
		btRigidBody::btRigidBodyConstructionInfo(RLConst::BALL_MASS_BT, NULL, _collisionShape);

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

	_rigidBody->m_rigidbodyFlags = 0;

	bulletWorld->addRigidBody(_rigidBody);
}

void Ball::_FinishPhysicsTick() {
	using namespace RLConst;

	// Add velocity cache
	if (!_velocityImpulseCache.IsZero()) {
		_rigidBody->m_linearVelocity += _velocityImpulseCache;
		_velocityImpulseCache = { 0,0,0 };
	}

	{ // Limit velocities
		btVector3
			vel = _rigidBody->m_linearVelocity,
			angVel = _rigidBody->m_angularVelocity;

		if (vel.length2() > (BALL_MAX_SPEED * UU_TO_BT) * (BALL_MAX_SPEED * UU_TO_BT))
			vel = vel.normalized() * (BALL_MAX_SPEED * UU_TO_BT);

		if (angVel.length2() > (BALL_MAX_ANG_SPEED * BALL_MAX_ANG_SPEED))
			angVel = angVel.normalized() * BALL_MAX_ANG_SPEED;

		_rigidBody->m_linearVelocity = vel;
		_rigidBody->m_angularVelocity = angVel;
	}

	{ // Round physics values to match RL
		_rigidBody->m_worldTransform.m_origin =
			Math::RoundVec(_rigidBody->m_worldTransform.m_origin, 0.01 * UU_TO_BT);

		_rigidBody->m_linearVelocity =
			Math::RoundVec(_rigidBody->m_linearVelocity, 0.01 * UU_TO_BT);

		_rigidBody->m_angularVelocity =
			Math::RoundVec(_rigidBody->m_angularVelocity, 0.00001);
	}
}

Ball::~Ball() {
	delete _rigidBody;
	delete _collisionShape;
}
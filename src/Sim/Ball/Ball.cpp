#include "Ball.h"

#include "../../RLConst.h"
#include "../Car/Car.h"

#include "../../../libsrc/bullet3-3.24/BulletDynamics/Dynamics/btDynamicsWorld.h"
#include "../../../libsrc/bullet3-3.24/BulletCollision/CollisionShapes/btConvexHullShape.h"
#include "../CollisionMasks.h"

RS_NS_START

bool BallState::Matches(const BallState& other, float marginPos, float marginVel, float marginAngVel) const {
	return
		pos.DistSq(other.pos) < (marginPos * marginPos) &&
		vel.DistSq(other.vel) < (marginVel * marginVel) &&
		angVel.DistSq(other.angVel) < (marginAngVel * marginAngVel);
}

void BallState::Serialize(DataStreamOut& out) {
	out.WriteMultiple(BALLSTATE_SERIALIZATION_FIELDS);
}

void BallState::Deserialize(DataStreamIn& in) {
	in.ReadMultiple(BALLSTATE_SERIALIZATION_FIELDS);
}

BallState Ball::GetState() {
	_internalState.pos = _rigidBody.getWorldTransform().getOrigin() * BT_TO_UU;
	_internalState.rotMat = _rigidBody.getWorldTransform().getBasis();
	_internalState.vel = _rigidBody.getLinearVelocity() * BT_TO_UU;
	_internalState.angVel = _rigidBody.getAngularVelocity();
	return _internalState;
}

void Ball::SetState(const BallState& state) {

	_internalState = state;

	btTransform newTransform;
	newTransform.setOrigin(state.pos * UU_TO_BT);
	newTransform.setBasis(state.rotMat);
	_rigidBody.setWorldTransform(newTransform);
	_rigidBody.setLinearVelocity(state.vel * UU_TO_BT);
	_rigidBody.setAngularVelocity(state.angVel);

	_velocityImpulseCache = { 0,0,0 };
	_internalState.updateCounter = 0;
}

btCollisionShape* MakeBallCollisionShape(GameMode gameMode, const MutatorConfig& mutatorConfig, btVector3& localIntertia) {
	
	if (gameMode == GameMode::SNOWDAY) {
		using namespace RLConst;

		auto shape = new btConvexHullShape();
		
		float angStep = (M_PI * 2) / Snowday::PUCK_CIRCLE_POINT_AMOUNT;
		float curAng = 0;
		for (int i = 0; i < Snowday::PUCK_CIRCLE_POINT_AMOUNT; i++) {
			Vec point = Vec(
				cosf(curAng) * mutatorConfig.ballRadius * UU_TO_BT,
				sinf(curAng) * mutatorConfig.ballRadius * UU_TO_BT,
				Snowday::PUCK_HEIGHT / 2 * UU_TO_BT
			);

			shape->addPoint(point, false);
			point.z *= -1;
			shape->addPoint(point, true);

			curAng += angStep;
		}
		shape->recalcLocalAabb();
		shape->calculateLocalInertia(mutatorConfig.ballMass, localIntertia);
		return shape;
	} else {
		auto shape = new btSphereShape(mutatorConfig.ballRadius * UU_TO_BT);
		shape->calculateLocalInertia(mutatorConfig.ballMass, localIntertia);
		return shape;
	}
}

void Ball::_BulletSetup(GameMode gameMode, btDynamicsWorld* bulletWorld, const MutatorConfig& mutatorConfig, bool noRot) {
	btVector3 localIneria;
	_collisionShape = MakeBallCollisionShape(gameMode, mutatorConfig, localIneria);

	btRigidBody::btRigidBodyConstructionInfo constructionInfo =
		btRigidBody::btRigidBodyConstructionInfo(mutatorConfig.ballMass, NULL, _collisionShape);

	constructionInfo.m_startWorldTransform.setIdentity();
	constructionInfo.m_startWorldTransform.setOrigin(btVector3(0, 0, mutatorConfig.ballRadius * UU_TO_BT));

	constructionInfo.m_localInertia = localIneria;
	constructionInfo.m_linearDamping = mutatorConfig.ballDrag;
	constructionInfo.m_friction = mutatorConfig.ballWorldFriction;
	constructionInfo.m_restitution = mutatorConfig.ballWorldRestitution;

	_rigidBody = btRigidBody(constructionInfo);
	_rigidBody.setUserIndex(BT_USERINFO_TYPE_BALL);
	_rigidBody.setUserPointer(this);

	// Trigger the Arena::_BulletContactAddedCallback() when anything touches the ball
	_rigidBody.m_collisionFlags |= btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK;

	_rigidBody.m_rigidbodyFlags = 0;

	_rigidBody.m_noRot = noRot && (_collisionShape->getShapeType() == SPHERE_SHAPE_PROXYTYPE);

	bulletWorld->addRigidBody(&_rigidBody, btBroadphaseProxy::DefaultFilter | CollisionMasks::HOOPS_NET, btBroadphaseProxy::AllFilter);
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

	_internalState.updateCounter++;
}

bool Ball::IsSphere() const {
	return dynamic_cast<btSphereShape*>(_collisionShape);
}

float Ball::GetRadiusBullet() const {
	if (IsSphere()) {
		return ((btSphereShape*)_collisionShape)->getRadius();
	} else {
		return 0;
	}
}

void Ball::_PreTickUpdate(GameMode gameMode, float tickTime) {
	if (gameMode == GameMode::HEATSEEKER) {
		using namespace RLConst;

		auto state = GetState();

		float yTargetDir = _internalState.hsInfo.yTargetDir;
		if (yTargetDir != 0) {
			Angle velAngle = Angle::FromVec(state.vel);

			// Determine angle to goal
			Vec goalTargetPos = Vec(0, Heatseeker::TARGET_Y * yTargetDir, Heatseeker::TARGET_Z);
			Angle angleToGoal = Angle::FromVec(goalTargetPos - state.pos);

			// Find difference between target angle and current angle
			Angle deltaAngle = angleToGoal - velAngle;
			
			// Determine speed ratio
			float curSpeed = state.vel.Length();
			float speedRatio = curSpeed / Heatseeker::MAX_SPEED;

			// Interpolate delta
			Angle newAngle = velAngle;
			float baseInterpFactor = speedRatio * tickTime;
			newAngle.yaw += deltaAngle.yaw * baseInterpFactor * Heatseeker::HORIZONTAL_BLEND;
			newAngle.pitch += deltaAngle.pitch * baseInterpFactor * Heatseeker::VERTICAL_BLEND;
			newAngle.NormalizeFix();

			// Limit pitch
			newAngle.pitch = RS_CLAMP(newAngle.pitch, -Heatseeker::MAX_TURN_PITCH, Heatseeker::MAX_TURN_PITCH);

			// Determine new interpolated speed
			float newSpeed = curSpeed + ((state.hsInfo.curTargetSpeed - curSpeed) * Heatseeker::SPEED_BLEND);

			// Update velocity
			Vec newDir = newAngle.GetForwardVec();
			Vec newVel = newDir * newSpeed;
			_rigidBody.m_linearVelocity = newVel * UU_TO_BT;

			_internalState.hsInfo.timeSinceHit += tickTime;
		}
	} else if (gameMode == GameMode::SNOWDAY) {
		groundStickApplied = false;
	}
}

void Ball::_OnHit(GameMode gameMode, Car* car) {
	if (gameMode == GameMode::HEATSEEKER) {
		using namespace RLConst;

		bool increaseSpeed = (_internalState.hsInfo.timeSinceHit > Heatseeker::MIN_SPEEDUP_INTERVAL) || (_internalState.hsInfo.yTargetDir == 0);
		_internalState.hsInfo.yTargetDir = car->team == Team::BLUE ? 1 : -1;
		if (increaseSpeed) {
			_internalState.hsInfo.timeSinceHit = 0;
			_internalState.hsInfo.curTargetSpeed = RS_MIN(_internalState.hsInfo.curTargetSpeed + Heatseeker::TARGET_SPEED_INCREMENT, Heatseeker::MAX_SPEED);
		}
	}
}

void Ball::_OnWorldCollision(GameMode gameMode, Vec normal, float tickTime) {
	if (gameMode == GameMode::HEATSEEKER) {
		if (_internalState.hsInfo.yTargetDir != 0 ) {
			float relNormalY = normal.y * _internalState.hsInfo.yTargetDir;
			if (relNormalY <= -RLConst::Heatseeker::WALL_BOUNCE_CHANGE_NORMAL_Y) {
				_internalState.hsInfo.yTargetDir *= -1;
			}
		}
	} else if (gameMode == GameMode::SNOWDAY) {
		if (!groundStickApplied) {
			_rigidBody.applyCentralForce(-normal * RLConst::Snowday::PUCK_GROUND_STICK_FORCE);
			groundStickApplied = true;
		}
	}
}

RS_NS_END
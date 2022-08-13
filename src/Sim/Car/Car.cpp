#include "Car.h"

// Update our internal state from bullet and return it
CarState Car::GetState() {

	btRigidBody* rb = _bulletVehicle.getRigidBody();
	btTransform rbTransform = rb->getWorldTransform();

	_internalState.pos = rbTransform.getOrigin() * BT_TO_UU;

	rbTransform.getRotation().getEulerZYX(_internalState.angles.yaw, _internalState.angles.pitch, _internalState.angles.roll);
	
	internalState.vel = rb->getLinearVelocity() * BT_TO_UU;

	internalState.angVel = rb->getAngularVelocity();

	return internalState;
}

// Update our bullet stuff to this new state, replace our internal state with it
void Car::SetState(const CarState& state) { 
	btRigidBody* rb = _bulletVehicle.getRigidBody();
	btTransform rbTransform = rb->getWorldTransform();

	rbTransform.setOrigin(state.pos * UU_TO_BT);

	btQuaternion quat;
	quat.setEulerZYX(state.angles.yaw, state.angles.pitch, state.angles.roll);
	rbTransform.setRotation(quat);

	rb->setLinearVelocity(state.vel * UU_TO_BT);

	rb->setAngularVelocity(state.angVel);

	this->_internalState = state;
}
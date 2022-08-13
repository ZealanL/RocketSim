#include "Car.h"

// Update our internal state from bullet and return it
CarState Car::GetState() {

	btRigidBody* rb = bulletVehicle.getRigidBody();
	btTransform rbTransform = rb->getWorldTransform();

	internalState.pos = rbTransform.getOrigin() * BT_TO_UU;

	rbTransform.getRotation().getEulerZYX(internalState.angles.yaw, internalState.angles.pitch, internalState.angles.roll);
	
	internalState.vel = rb->getLinearVelocity();

	internalState.angVel = rb->getAngularVelocity();

	return internalState;
}

// Update our bullet stuff to this new state, replace our internal state with it
void Car::SetState(const CarState& state) { 
	btRigidBody* rb = bulletVehicle.getRigidBody();
	btTransform rbTransform = rb->getWorldTransform();

	rbTransform.setOrigin(state.pos * UU_TO_BT);

	btQuaternion quat;
	quat.setEulerZYX(state.angles.yaw, state.angles.pitch, state.angles.roll);
	rbTransform.setRotation(quat);

	rb->setLinearVelocity(state.vel);

	rb->setAngularVelocity(state.angVel);

	this->internalState = state;
}
#include "Car.h"
#include "../../RLConst.h"

// Update our internal state from bullet and return it
CarState Car::GetState() {

	btTransform rbTransform = _rigidBody->getWorldTransform();

	_internalState.pos = rbTransform.getOrigin() * BT_TO_UU;

	rbTransform.getRotation().getEulerZYX(_internalState.angles.yaw, _internalState.angles.pitch, _internalState.angles.roll);
	
	_internalState.vel = _rigidBody->getLinearVelocity() * BT_TO_UU;

	_internalState.angVel = _rigidBody->getAngularVelocity();

	return _internalState;
}

// Update our bullet stuff to this new state, replace our internal state with it
void Car::SetState(const CarState& state) { 
	btTransform rbTransform = _rigidBody->getWorldTransform();

	rbTransform.setOrigin(state.pos * UU_TO_BT);

	btQuaternion quat;
	quat.setEulerZYX(state.angles.yaw, state.angles.pitch, state.angles.roll);
	rbTransform.setRotation(quat);

	_rigidBody->setWorldTransform(rbTransform);

	_rigidBody->setLinearVelocity(state.vel * UU_TO_BT);
	_rigidBody->setAngularVelocity(state.angVel);

	this->_internalState = state;
}

void Car::_PostTickUpdate() {
	_bulletVehicle->updateVehicle(TICKTIME);
	this->_internalState.lastControls = this->controls;
}

Car* Car::_AllocateCar() {
	return new Car();
}

Car::~Car() {
	delete _bulletVehicle;
	delete _bulletVehicleRaycaster;
	delete _rigidBody;
	delete _compoundShape;
	delete _childHitboxShape;
}

void Car::_PreTickUpdate() {
	float forwardSpeed = _bulletVehicle->getForwardSpeed();
	float absForwardSpeed = abs(forwardSpeed);

	bool jumpPressed = controls.jump && !_internalState.lastControls.jump;

	{ // Update steering
		float steerAngle = RLConst::STEER_ANGLE_FROM_SPEED_CURVE.GetOutput(absForwardSpeed * BT_TO_UU) * controls.steer;
		_bulletVehicle->m_wheelInfo[0].m_steerAngle = steerAngle;
		_bulletVehicle->m_wheelInfo[1].m_steerAngle = steerAngle;
	}


	{ // Update throttle/brake forces
		float driveSpeedScale = RLConst::DRIVE_SPEED_TORQUE_FACTOR_CURVE.GetOutput(absForwardSpeed * BT_TO_UU);

		float realThrottle = controls.throttle;
		float realBrake = 0;

		if (controls.handbrake) {
			// Real throttle is unchanged from the input throttle when powersliding
		} else {
			float absThrottle = abs(controls.throttle);
			
			if (absThrottle >= RLConst::THROTTLE_DEADZONE) {
				if (absForwardSpeed > 0 && SGN(controls.throttle) != SGN(forwardSpeed)) {
					// Brake is applied if we are trying to drive in the opposite direction
					realBrake = 1;
				}
			} else {
				// No throttle, we are coasting
				realThrottle = 0;

				// Apply coasting relevant brake
				bool shouldFullStop = (absForwardSpeed < RLConst::STOPPING_FORWARD_VEL* UU_TO_BT);
				realBrake = shouldFullStop ? 1 : RLConst::COASTING_BRAKE_FACTOR;
			}
		}

		float driveEngineForce = realThrottle * (RLConst::THROTTLE_TORQUE_AMOUNT * UU_TO_BT) * driveSpeedScale;
		float driveBrakeForce = realBrake * (RLConst::BRAKE_TORQUE_AMOUNT * UU_TO_BT);
		for (int i = 0; i < 4; i++) {
			_bulletVehicle->m_wheelInfo[i].m_engineForce = driveEngineForce;
			_bulletVehicle->m_wheelInfo[i].m_brake = driveBrakeForce;
		}
	}
}

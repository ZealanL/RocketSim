#pragma once
#include "../../BaseInc.h"

// This is a modified version of btWheelInfo to more accurately follow Rocket League
struct btWheelInfoRL : public btWheelInfo {
	bool m_isInContactWithWorld = false;

	btWheelInfoRL(btWheelInfoConstructionInfo& constructionInfo) : btWheelInfo(constructionInfo) {}
};

// This is a modified version of btRaycastVehicle to more accurately follow Rocket League
class btVehicleRL : public btActionInterface
{
	btAlignedObjectArray<btVector3> m_forwardWS;
	btAlignedObjectArray<btVector3> m_axle;
	btAlignedObjectArray<btScalar> m_forwardImpulse;
	btAlignedObjectArray<btScalar> m_sideImpulse;

	///backwards compatibility
	int m_userConstraintType;
	int m_userConstraintId;

public:
	class btVehicleTuning
	{
	public:
		btVehicleTuning()
			: m_suspensionStiffness(btScalar(5.88)),
			m_suspensionCompression(btScalar(0.83)),
			m_suspensionDamping(btScalar(0.88)),
			m_maxSuspensionTravelCm(btScalar(500.)),
			m_frictionSlip(btScalar(10.5)),
			m_maxSuspensionForce(btScalar(6000.)) {
		}
		btScalar m_suspensionStiffness;
		btScalar m_suspensionCompression;
		btScalar m_suspensionDamping;
		btScalar m_maxSuspensionTravelCm;
		btScalar m_frictionSlip;
		btScalar m_maxSuspensionForce;
	};

private:
	btVehicleRaycaster* m_vehicleRaycaster;
	btScalar m_pitchControl;
	btScalar m_steeringValue;

	btRigidBody* m_chassisBody;

	int m_indexRightAxis;
	int m_indexUpAxis;
	int m_indexForwardAxis;

	void defaultInit(const btVehicleTuning& tuning);

public:
	//constructor to create a car from an existing rigidbody
	btVehicleRL(const btVehicleTuning& tuning, btRigidBody* chassis, btVehicleRaycaster* raycaster);

	virtual ~btVehicleRL();

	///btActionInterface interface
	virtual void updateAction(btCollisionWorld* collisionWorld, btScalar step) {
		(void)collisionWorld;
		updateVehicle(step);
	}

	const btTransform& getChassisWorldTransform() const;

	btScalar rayCast(btWheelInfoRL& wheel);

	virtual void updateVehicle(btScalar step);

	void resetSuspension();

	btScalar getSteeringValue(int wheel) const;

	void setSteeringValue(btScalar steering, int wheel);

	void applyEngineForce(btScalar force, int wheel);

	const btTransform& getWheelTransformWS(int wheelIndex) const;

	void updateWheelTransform(int wheelIndex);

	//	void	setRaycastWheelInfo( int wheelIndex , bool isInContact, const btVector3& hitPoint, const btVector3& hitNormal,btScalar depth);

	btWheelInfoRL& addWheel(const btVector3& connectionPointCS0, const btVector3& wheelDirectionCS0, const btVector3& wheelAxleCS, btScalar suspensionRestLength, btScalar wheelRadius, const btVehicleTuning& tuning, bool isFrontWheel);

	inline int getNumWheels() const {
		return int(m_wheelInfo.size());
	}

	btAlignedObjectArray<btWheelInfoRL> m_wheelInfo;

	const btWheelInfoRL& getWheelInfo(int index) const;

	btWheelInfoRL& getWheelInfo(int index);

	void updateWheelTransformsWS(btWheelInfoRL& wheel);

	void setBrake(btScalar brake, int wheelIndex);

	void setPitchControl(btScalar pitch) {
		m_pitchControl = pitch;
	}

	void updateSuspension(btScalar deltaTime);

	virtual void updateFriction(btScalar timeStep);

	inline btRigidBody* getRigidBody() {
		return m_chassisBody;
	}

	const btRigidBody* getRigidBody() const {
		return m_chassisBody;
	}

	inline int getRightAxis() const {
		return m_indexRightAxis;
	}
	inline int getUpAxis() const {
		return m_indexUpAxis;
	}

	inline int getForwardAxis() const {
		return m_indexForwardAxis;
	}

	btVector3 getForwardVector() const {
		const btTransform& chassisTrans = getChassisWorldTransform();

		btVector3 forwardW(
			chassisTrans.getBasis()[0][m_indexForwardAxis],
			chassisTrans.getBasis()[1][m_indexForwardAxis],
			chassisTrans.getBasis()[2][m_indexForwardAxis]);

		return forwardW;
	}

	btVector3 getUpVector() const {
		const btTransform& chassisTrans = getChassisWorldTransform();

		btVector3 forwardW(
			chassisTrans.getBasis()[0][m_indexUpAxis],
			chassisTrans.getBasis()[1][m_indexUpAxis],
			chassisTrans.getBasis()[2][m_indexUpAxis]);

		return forwardW;
	}

	btVector3 getRightVector() const {
		const btTransform& chassisTrans = getChassisWorldTransform();

		btVector3 forwardW(
			chassisTrans.getBasis()[0][m_indexRightAxis],
			chassisTrans.getBasis()[1][m_indexRightAxis],
			chassisTrans.getBasis()[2][m_indexRightAxis]);

		return forwardW;
	}

	virtual void setCoordinateSystem(int rightIndex, int upIndex, int forwardIndex) {
		m_indexRightAxis = rightIndex;
		m_indexUpAxis = upIndex;
		m_indexForwardAxis = forwardIndex;
	}

	///backwards compatibility
	int getUserConstraintType() const {
		return m_userConstraintType;
	}

	void setUserConstraintType(int userConstraintType) {
		m_userConstraintType = userConstraintType;
	};

	void setUserConstraintId(int uid) {
		m_userConstraintId = uid;
	}

	int getUserConstraintId() const {
		return m_userConstraintId;
	}

	virtual void debugDraw(btIDebugDraw* debugDrawer) {}

	// Extra funcs added by Zealan
	btVector3 getDownwardsDirFromWheelContacts();
	float getForwardSpeed();
};
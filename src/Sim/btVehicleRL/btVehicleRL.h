#pragma once
#include "../../BaseInc.h"

// This is a modified version of btWheelInfo to more accurately follow Rocket League
struct btWheelInfoRL : public btWheelInfo {
	bool m_isInContactWithWorld = false;
	float m_steerAngle = 0;

	// lat = sideways, long = forward
	float m_latFriction = 0, m_longFriction = 0;
	btVector3 m_impulse;

	float m_suspensionForceScale;

	// Extra force applied when compressed significantly
	float m_extraPushback = 0;

	btWheelInfoRL(btWheelInfoConstructionInfo& constructionInfo) : btWheelInfo(constructionInfo) {}
};

// This is a modified version of btRaycastVehicle to more accurately follow Rocket League
class btVehicleRL : public btActionInterface {
public:
	btAlignedObjectArray<Vec> m_forwardWS;
	btAlignedObjectArray<Vec> m_axle;
	btAlignedObjectArray<float> m_forwardImpulse;
	btAlignedObjectArray<float> m_sideImpulse;

	btDynamicsWorld* m_dynamicsWorld;

	///backwards compatibility
	int m_userConstraintType;
	int m_userConstraintId;


	class btVehicleTuning
	{
	public:
		btVehicleTuning()
			: m_suspensionStiffness(float(5.88)),
			m_suspensionCompression(float(0.83)),
			m_suspensionDamping(float(0.88)),
			m_maxSuspensionTravelCm(float(500.)),
			m_frictionSlip(float(10.5)),
			m_maxSuspensionForce(float(6000.)) {
		}
		float m_suspensionStiffness;
		float m_suspensionCompression;
		float m_suspensionDamping;
		float m_maxSuspensionTravelCm;
		float m_frictionSlip;
		float m_maxSuspensionForce;
	};

	btVehicleRaycaster* m_vehicleRaycaster;
	float m_pitchControl;
	float m_steeringValue;

	btRigidBody* m_chassisBody;

	int m_indexRightAxis;
	int m_indexUpAxis;
	int m_indexForwardAxis;

	void defaultInit(const btVehicleTuning& tuning);

	//constructor to create a car from an existing rigidbody
	btVehicleRL(const btVehicleTuning& tuning, btRigidBody* chassis, btVehicleRaycaster* raycaster, btDynamicsWorld* world);

	virtual ~btVehicleRL();

	///btActionInterface interface
	void updateAction(btCollisionWorld* collisionWorld, float step) {
		assert(false); // This should never be hit in RocketSim!
	}

	const btTransform& getChassisWorldTransform() const;

	float rayCast(btWheelInfoRL& wheel);

	void updateVehicleFirst(float step);
	void updateVehicleSecond(float step);

	void resetSuspension();

	float getSteeringValue(int wheel) const;

	void setSteeringValue(float steering, int wheel);

	void applyEngineForce(float force, int wheel);

	const btTransform& getWheelTransformWS(int wheelIndex) const;

	void updateWheelTransform(int wheelIndex);

	//	void	setRaycastWheelInfo( int wheelIndex , bool isInContact, const Vec& hitPoint, const Vec& hitNormal,float depth);

	btWheelInfoRL& addWheel(const Vec& connectionPointCS0, const Vec& wheelDirectionCS0, const Vec& wheelAxleCS, float suspensionRestLength, float wheelRadius, const btVehicleTuning& tuning, bool isFrontWheel);

	inline int getNumWheels() const {
		return int(m_wheelInfo.size());
	}

	btAlignedObjectArray<btWheelInfoRL> m_wheelInfo;

	const btWheelInfoRL& getWheelInfo(int index) const;

	btWheelInfoRL& getWheelInfo(int index);

	void updateWheelTransformsWS(btWheelInfoRL& wheel);

	void setBrake(float brake, int wheelIndex);

	void setPitchControl(float pitch) {
		m_pitchControl = pitch;
	}

	void updateSuspension(float deltaTime);

	virtual void calcFrictionImpulses(float timeStep);
	void applyFrictionImpulses(float timeStep);

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

	Vec getForwardVector() const {
		const btTransform& chassisTrans = getChassisWorldTransform();

		Vec forwardW(
			chassisTrans.getBasis()[0][m_indexForwardAxis],
			chassisTrans.getBasis()[1][m_indexForwardAxis],
			chassisTrans.getBasis()[2][m_indexForwardAxis]);

		return forwardW;
	}

	Vec getUpVector() const {
		const btTransform& chassisTrans = getChassisWorldTransform();

		Vec forwardW(
			chassisTrans.getBasis()[0][m_indexUpAxis],
			chassisTrans.getBasis()[1][m_indexUpAxis],
			chassisTrans.getBasis()[2][m_indexUpAxis]);

		return forwardW;
	}

	Vec getRightVector() const {
		const btTransform& chassisTrans = getChassisWorldTransform();

		Vec forwardW(
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

    // Extra utility funcs
	Vec getUpwardsDirFromWheelContacts();
	float getForwardSpeed();
};
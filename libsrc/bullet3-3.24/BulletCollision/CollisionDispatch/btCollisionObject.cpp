/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  https://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btCollisionObject.h"
#include "../BroadphaseCollision/btBroadphaseProxy.h"

#include "../CollisionShapes/btCollisionShape.h"

btCollisionObject::btCollisionObject()
	: m_interpolationLinearVelocity(0.f, 0.f, 0.f),
	  m_interpolationAngularVelocity(0.f, 0.f, 0.f),
	  m_anisotropicFriction(1.f, 1.f, 1.f),
	  m_hasAnisotropicFriction(false),
	  m_contactProcessingThreshold(BT_LARGE_FLOAT),
	  m_broadphaseHandle(0),
	  m_collisionShape(0),
	  m_extensionPointer(0),
	  m_rootCollisionShape(0),
	  m_collisionFlags(btCollisionObject::CF_STATIC_OBJECT),
	  m_islandTag1(-1),
	  m_companionId(-1),
	  m_worldArrayIndex(-1),
	  m_activationState1(1),
	  m_deactivationTime(btScalar(0.)),
	  m_friction(btScalar(0.5)),
	  m_restitution(btScalar(0.)),
	  m_rollingFriction(0.0f),
	  m_spinningFriction(0.f),
	  m_contactDamping(.1),
	  m_contactStiffness(BT_LARGE_FLOAT),
	  m_internalType(CO_COLLISION_OBJECT),
	  m_userObjectPointer(0),
	  m_userIndex2(-1),
	  m_userIndex(-1),
	  m_userIndex3(-1),
	  m_hitFraction(btScalar(1.)),
	  m_ccdSweptSphereRadius(btScalar(0.)),
	  m_ccdMotionThreshold(btScalar(0.)),
	  m_checkCollideWith(false),
	  m_updateRevision(0)
{
	m_worldTransform.setIdentity();
	m_interpolationWorldTransform.setIdentity();
}

btCollisionObject::~btCollisionObject()
{
}

void btCollisionObject::setActivationState(int newState) const
{
	if ((m_activationState1 != DISABLE_DEACTIVATION) && (m_activationState1 != DISABLE_SIMULATION))
		m_activationState1 = newState;
}

void btCollisionObject::forceActivationState(int newState) const
{
	m_activationState1 = newState;
}

void btCollisionObject::activate(bool forceActivation) const
{
	if (forceActivation || !(m_collisionFlags & (CF_STATIC_OBJECT | CF_KINEMATIC_OBJECT)))
	{
		setActivationState(ACTIVE_TAG);
		m_deactivationTime = btScalar(0.);
	}
}

void btCollisionObject::setWorldTransform(const btTransform& worldTrans) {
	m_updateRevision++;
	m_worldTransform = worldTrans;
	if (m_collisionShape)
		m_collisionShape->m_aabbCached = false;
}
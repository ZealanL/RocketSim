#include "Arena.h"
#include "../../RLConst.h"

Car* Arena::AddCar(Team team, const CarConfig& config) {
	Car* car = Car::_AllocateCar();
	_carsList.push_back(car);

	car->config = config;
	car->team = team;
	car->id = ++_lastCarID;

	// TODO: Initialize the rest

	return car;
}

bool Arena::RemoveCar(Car* car) {
	auto sizeBefore = _carsList.size();
	_carsList.remove(car);
	if (_carsList.size() < sizeBefore) {
		// Car was removed, free it up
		delete car;
		return true;
	} else {
		// Car wasn't found
		return false;
	}
}

Car* Arena::GetCarFromID(uint32_t id) {
	for (Car* car : _carsList)
		if (car->id == id)
			return car;

	return NULL;
}

void Arena::RegisterGoalScoreCallback(GoalScoreEventFn callbackFunc) {
	_goalScoreCallbacks.push_back(callbackFunc);
}


Arena::Arena(GameMode gameMode) {
	this->gameMode = gameMode;

	{ // Initialize ball
		ball = Ball::_AllocBall();

		float radius;
		switch (gameMode) {
		default:
			radius = RLConst::BALL_COLLISION_RADIUS_NORMAL;
			break;
		}

		ball->_collisionShape = new btSphereShape(radius);

		btRigidBody::btRigidBodyConstructionInfo constructionInfo = 
			btRigidBody::btRigidBodyConstructionInfo(RLConst::BALL_MASS, NULL, ball->_collisionShape);

		constructionInfo.m_startWorldTransform.setIdentity();
		constructionInfo.m_startWorldTransform.setOrigin(btVector3(0, 0, radius));

		// TODO: Set other values in constructionInfo to match the ball

		ball->_rigidBody = new btRigidBody(constructionInfo);
	}

	{ // Initialize world

		 _bulletWorldParams.collisionConfig = new btDefaultCollisionConfiguration();
		 _bulletWorldParams.collisionDispatcher = new btCollisionDispatcher(_bulletWorldParams.collisionConfig);
		 _bulletWorldParams.constraintSolver = new btSequentialImpulseConstraintSolver;
		 _bulletWorldParams.overlappingPairCache = new btDbvtBroadphase();

		_bulletWorld = new btDiscreteDynamicsWorld(
			_bulletWorldParams.collisionDispatcher,
			_bulletWorldParams.overlappingPairCache,
			_bulletWorldParams.constraintSolver,
			_bulletWorldParams.collisionConfig
		);
	}

	// Add ball to world
	_bulletWorld->addRigidBody(ball->_rigidBody);
}

Arena::~Arena() {

	// Delete world first
	delete _bulletWorld;

	{ // Delete world param things
		delete _bulletWorldParams.collisionConfig;
		delete _bulletWorldParams.collisionDispatcher;
		delete _bulletWorldParams.overlappingPairCache;
		delete _bulletWorldParams.constraintSolver;
	}

	// Remove all cars
	for (Car* car : _carsList)
		delete car;

	{ // Delete collision RBs and shapes
		for (btRigidBody* colRB : _worldCollisionRBs)
			delete colRB;

		for (btCollisionShape* colObject : _worldCollisionShapes)
			delete colObject;
	}

	// Remove ball
	delete ball;
}

#include "Arena.h"
#include "../../RLConst.h"
#include "../MeshLoader/MeshLoader.h"

Car* Arena::AddCar(Team team, const CarConfig& config) {
	Car* car = Car::_AllocateCar();
	_cars.push_back(car);

	car->config = config;
	car->team = team;
	car->id = ++_lastCarID;

	car->_BulletSetup(_bulletWorld);

	return car;
}

bool Arena::RemoveCar(Car* car) {
	for (int i = 0; i < _cars.size(); i++) {
		if (_cars[i] == car) {
			_cars.erase(_cars.begin() + i);
			delete car;
			return true;
		}
	}

	return false;
}

Car* Arena::GetCarFromID(uint32_t id) {
	for (Car* car : _cars)
		if (car->id == id)
			return car;

	return NULL;
}

void Arena::RegisterGoalScoreCallback(GoalScoreEventFn callbackFunc) {
	_goalScoreCallbacks.push_back(callbackFunc);
}

bool Arena::_BulletContactAddedCallback(btManifoldPoint& cp, const btCollisionObjectWrapper* objA, int partIdA, int indexA, const btCollisionObjectWrapper* objB, int partIdB, int indexB) {
	
	bool 
		isBallObjA = objA->m_collisionObject->m_userIndex == BT_USERINFO_TYPE_BALL,
		isBallObjB = objB->m_collisionObject->m_userIndex == BT_USERINFO_TYPE_BALL;
	
	if (isBallObjA || isBallObjB) {
		auto ballRB = (btRigidBody*)(isBallObjA ? objA->m_collisionObject : objB->m_collisionObject);
		auto otherCollider = isBallObjA ? objB->m_collisionObject : objA->m_collisionObject;

		if (otherCollider->m_userIndex == BT_USERINFO_TYPE_CAR) {
			// This is a car-ball collision
			// Override the friction and restitution accordingly
			cp.m_combinedFriction = RLConst::CARBALL_COLLISION_FRICTION;
			cp.m_combinedRestitution = RLConst::CARBALL_COLLISION_RESTITUTION;
		}
	}

	return true;
}

void Arena::_BulletInternalTickCallback(btDynamicsWorld* world, btScalar step) {
	Arena* arenaInst = (Arena*)world->getWorldUserInfo();
	if (!arenaInst)
		return;

	auto dispatcher = world->getDispatcher();
	int numManifolds = dispatcher->getNumManifolds();
	for (int i = 0; i < numManifolds; i++) {
		auto contactManifold = dispatcher->getManifoldByIndexInternal(i);
		int numContacts = contactManifold->getNumContacts();
		if (!numContacts)
			continue;

		auto
			body0 = contactManifold->getBody0(),
			body1 = contactManifold->getBody1();

		bool firstBodyIsCar = (body0->getUserIndex() == BT_USERINFO_TYPE_CAR);
		if (!firstBodyIsCar && (body1->getUserIndex() != BT_USERINFO_TYPE_CAR))
			continue; // No car involved, so we don't care

		auto carBody = firstBodyIsCar ? body0 : body1;
		auto otherBody = firstBodyIsCar ? body1 : body0;

		if (!otherBody->getUserIndex())
			continue; // Other colliding body is neither a car nor the ball

		// TODO: Does RL support multiple contacts in a tick? Not sure.
		// We'll just use the first one regardless.
		btManifoldPoint& contactPoint = contactManifold->getContactPoint(0);

		if (otherBody->getUserIndex() == BT_USERINFO_TYPE_BALL) {
			// Ball was hit
			arenaInst->_BtCallback_OnCarBallCollision((Car*)carBody->getUserPointer(), (Ball*)otherBody->getUserPointer(), contactPoint);
		} else if (otherBody->getUserIndex() == BT_USERINFO_TYPE_CAR) {
			// Car was hit
			arenaInst->_BtCallback_OnCarCarCollision((Car*)carBody->getUserPointer(), (Car*)otherBody->getUserPointer(), contactPoint);
		}
	}
}

void Arena::_BtCallback_OnCarBallCollision(Car* car, Ball* ball, btManifoldPoint& manifoldPoint) {
	using namespace RLConst;

	// Apply extra car-ball hit impulse

	auto carState = car->GetState();
	auto ballState = ball->GetState();

	btVector3 carForward = car->_bulletVehicle->getForwardVector();
	btVector3 relPos = ballState.pos - carState.pos;
	btVector3 relVel = ballState.vel - carState.vel;

	float relSpeed = RS_MIN(relVel.length(), BALL_CAR_EXTRA_IMPULSE_MAXDELTAVEL_UU);

	if (relSpeed > 0) {
		btVector3 hitDir = (relPos * btVector3(1, 1, BALL_CAR_EXTRA_IMPULSE_Z_SCALE)).normalized();

		btVector3 forwardDirAdjustment = carForward * hitDir.dot(carForward) * (1 - BALL_CAR_EXTRA_IMPULSE_FORWARD_SCALE);
		hitDir = (hitDir - forwardDirAdjustment).normalized();

		btVector3 addedVel = (hitDir * relSpeed) * BALL_CAR_EXTRA_IMPULSE_FACTOR_CURVE.GetOutput(relSpeed);
		ballState.vel += addedVel;
	}

	ball->SetState(ballState);
}

void Arena::_BtCallback_OnCarCarCollision(Car* car1, Car* car2, btManifoldPoint& manifoldPoint) {
	// TODO: Bump physics
}

Arena::Arena(GameMode gameMode, float tickRate) {

	// Tickrate must be from 15 to 120tps
	assert(tickRate >= 15 && tickRate <= 120);

	this->gameMode = gameMode;
	this->tickTime = 1 / tickRate;
	{ // Initialize world

		btDefaultCollisionConstructionInfo collisionConfigConstructionInfo = {};

		_bulletWorldParams.collisionConfig = new btDefaultCollisionConfiguration(collisionConfigConstructionInfo);

		_bulletWorldParams.collisionDispatcher = new btCollisionDispatcher(_bulletWorldParams.collisionConfig);
		_bulletWorldParams.constraintSolver = new btSequentialImpulseConstraintSolver();
		_bulletWorldParams.overlappingPairCache = new btDbvtBroadphase();

		_bulletWorld = new btDiscreteDynamicsWorld(
			_bulletWorldParams.collisionDispatcher,
			_bulletWorldParams.overlappingPairCache,
			_bulletWorldParams.constraintSolver,
			_bulletWorldParams.collisionConfig
		);
		
		_bulletWorld->setGravity(btVector3(0, 0, RLConst::GRAVITY_Z * UU_TO_BT));
	}

	_SetupArenaCollisionShapes();

	// Give arena collision shapes the proper restitution/friction values
	for (auto rb : _worldCollisionRBs) {
		// TODO: Move to RLConst
		rb->setRestitution(0.3f);
		rb->setFriction(0.6f);
		rb->setRollingFriction(0.f);
	}

	{ // Initialize ball
		ball = Ball::_AllocBall();

		float radius;
		switch (gameMode) {
		default:
			radius = RLConst::BALL_COLLISION_RADIUS_NORMAL;
			break;
		}
		radius *= UU_TO_BT;

		ball->_BulletSetup(_bulletWorld, radius);
	}

	// Add ball to world
	_bulletWorld->addRigidBody(ball->_rigidBody);

	// Set internal tick callback
	_bulletWorld->setWorldUserInfo(this);
	
	gContactAddedCallback = &Arena::_BulletContactAddedCallback;

	_bulletWorld->setInternalTickCallback(&Arena::_BulletInternalTickCallback);
}

void Arena::Step(int ticksToSimulate) {
	for (int i = 0; i < ticksToSimulate; i++) {

		_bulletWorld->setWorldUserInfo(this);

		{ // Ball zero-vel sleeping
			if (ball->_rigidBody->getLinearVelocity().length2() == 0 && ball->_rigidBody->getAngularVelocity().length2() == 0) {
				// hooooooonk mimimimimimimi hooooooonk mimimimimimimi
				ball->_rigidBody->setActivationState(ISLAND_SLEEPING);
			} else {
				ball->_rigidBody->setActivationState(ACTIVE_TAG);
			}
		}

		for (Car* car : _cars) {
			car->_PreTickUpdate(tickTime);
		}

		// Update world
		_bulletWorld->stepSimulation(tickTime, 0, tickTime);

		for (Car* car : _cars) {
			car->_ApplyPhysicsRounding();
			car->_PostTickUpdate(tickTime);
		}

		{ // Limit ball's linear/angular velocity
			using namespace RLConst;

			btVector3 ballVel = ball->_rigidBody->getLinearVelocity();
			btVector3 ballAngVel = ball->_rigidBody->getAngularVelocity();

			if (ballVel.length2() > (BALL_MAX_SPEED * BALL_MAX_SPEED * UU_TO_BT))
				ballVel = ballVel.normalized() * (BALL_MAX_SPEED * UU_TO_BT);

			if (ballAngVel.length2() > (BALL_MAX_ANG_SPEED * BALL_MAX_ANG_SPEED))
				ballAngVel = ballAngVel.normalized() * BALL_MAX_ANG_SPEED;

			ball->_rigidBody->setLinearVelocity(ballVel);
			ball->_rigidBody->setAngularVelocity(ballAngVel);
		}

		tickCount++;
	}
}

Arena::~Arena() {

	// Remove all cars
	for (Car* car : _cars)
		RemoveCar(car);

	// Delete world
	delete _bulletWorld;

	{ // Delete world param things
		delete _bulletWorldParams.collisionConfig;
		delete _bulletWorldParams.collisionDispatcher;
		delete _bulletWorldParams.overlappingPairCache;
		delete _bulletWorldParams.constraintSolver;
	}

	{ // Delete collision RBs and shapes
		for (btRigidBody* colRB : _worldCollisionRBs)
			delete colRB;

		for (btCollisionShape* colObject : _worldCollisionShapes)
			delete colObject;
	}

	// Remove ball
	delete ball;

	// Remove arena collision meshes
	for (btTriangleMesh* mesh : _arenaTriMeshes)
		delete mesh;
}

void Arena::_AddStaticCollisionShape(btCollisionShape* shape, btVector3 pos) {
	_worldCollisionShapes.push_back(shape);

	btRigidBody* shapeRB = new btRigidBody(0, NULL, shape);
	shapeRB->setWorldTransform(btTransform(btMatrix3x3::getIdentity(), pos));
	_worldCollisionRBs.push_back(shapeRB);

	_bulletWorld->addRigidBody(shapeRB);
}

void Arena::_AddStaticCollisionTris(MeshLoader::Mesh& mesh, btVector3 scale, btVector3 pos) {
	auto triMesh = mesh.MakeBulletMesh(scale);
	_arenaTriMeshes.push_back(triMesh);

	auto bvt = new btBvhTriangleMeshShape(triMesh, false);
	bvt->buildOptimizedBvh();
	_AddStaticCollisionShape(bvt, pos);
}

void Arena::_SetupArenaCollisionShapes() {
	// TODO: This is just for soccar arena
	assert(gameMode == GameMode::SOCCAR);

	string basePath = "assets/soccar_field/";

	if (!std::filesystem::exists(basePath)) {
		RS_ERR_CLOSE(
			"Failed to find soccar field asset files at \"" << basePath
			<< "\", the assets folder should be in our current directory " << std::filesystem::current_path() << ".")
	}

	// Create triangle meshes
	static bool meshesLoaded;

	static MeshLoader::Mesh cornerMesh;
	static MeshLoader::Mesh goalMesh;
	static MeshLoader::Mesh rampsMeshA;
	static MeshLoader::Mesh rampsMeshB;

	if (!meshesLoaded) {
		cornerMesh	= MeshLoader::LoadMeshFromFiles(basePath + "soccar_corner", 2);
		goalMesh	= MeshLoader::LoadMeshFromFiles(basePath + "soccar_goal", 2);
		rampsMeshA	= MeshLoader::LoadMeshFromFiles(basePath + "soccar_ramps_0", 2);
		rampsMeshB	= MeshLoader::LoadMeshFromFiles(basePath + "soccar_ramps_1", 2);
		meshesLoaded = true;
	}

	constexpr float PLANE_THICKNESS = 10;
	constexpr float WALL_SIZE = 120;
	
	constexpr float EXTENT_X = 4096 * UU_TO_BT;
	constexpr float EXTENT_Y = 5120 * UU_TO_BT;
	constexpr float EXTENT_Z = 2048 * UU_TO_BT;

	// Floor
	_AddStaticCollisionShape(
		new btStaticPlaneShape(btVector3(0, 0, 1), 0),
		{ 0, 0, 0 }
	);

	// Ceiling
	_AddStaticCollisionShape(
		new btStaticPlaneShape(btVector3(0, 0, -1), 0),
		{ 0, 0, EXTENT_Z }
	);

	// Side walls
	_AddStaticCollisionShape(
		new btStaticPlaneShape(btVector3(1, 0, 0), 0),
		{ -EXTENT_X, 0, EXTENT_Z / 2 }
	);
	_AddStaticCollisionShape(
		new btStaticPlaneShape(btVector3(-1, 0, 0), 0),
		{ EXTENT_X, 0, EXTENT_Z / 2 }
	);

	// Add vertical corners
	_AddStaticCollisionTris(cornerMesh, btVector3(1, 1, 1));
	_AddStaticCollisionTris(cornerMesh, btVector3(-1, 1, 1));
	_AddStaticCollisionTris(cornerMesh, btVector3(1, -1, 1));
	_AddStaticCollisionTris(cornerMesh, btVector3(-1, -1, 1));

	// Add goals
	_AddStaticCollisionTris(goalMesh, btVector3(1, 1, 1), btVector3(0, -EXTENT_Y, 0));
	_AddStaticCollisionTris(goalMesh, btVector3(1, -1, 1), btVector3(0, EXTENT_Y, 0));

	// Add sidewall ramps
	_AddStaticCollisionTris(rampsMeshA, btVector3(1, 1, 1));
	_AddStaticCollisionTris(rampsMeshA, btVector3(-1, 1, 1));
	_AddStaticCollisionTris(rampsMeshB, btVector3(1, 1, 1));
	_AddStaticCollisionTris(rampsMeshB, btVector3(-1, 1, 1));
}
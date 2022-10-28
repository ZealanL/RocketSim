#include "Arena.h"
#include "../../RLConst.h"
#include "../MeshLoader/MeshLoader.h"

Car* Arena::AddCar(Team team, const CarConfig& config) {
	Car* car = Car::_AllocateCar();
	_carsList.push_back(car);

	car->config = config;
	car->team = team;
	car->id = ++_lastCarID;

	{ // Set up rigidbody and collision shapes
		car->_childHitboxShape = new btBoxShape((config.hitboxSize * UU_TO_BT) / 2);
		car->_compoundShape = new btCompoundShape();

		btTransform hitboxOffsetTransform = btTransform();
		hitboxOffsetTransform.setIdentity();
		hitboxOffsetTransform.setOrigin(config.hitboxPosOffset * UU_TO_BT);
		car->_compoundShape->addChildShape(hitboxOffsetTransform, car->_childHitboxShape);

		btVector3 localInertia(0, 0, 0);
		car->_compoundShape->calculateLocalInertia(RLConst::CAR_MASS_BT, localInertia);

		btRigidBody::btRigidBodyConstructionInfo rbInfo
			= btRigidBody::btRigidBodyConstructionInfo(RLConst::CAR_MASS_BT, NULL, car->_compoundShape, localInertia);

		btTransform carTransform = btTransform();
		carTransform.setIdentity();
		rbInfo.m_startWorldTransform = carTransform;

		car->_rigidBody = new btRigidBody(rbInfo);
	}

	// Add rigidbody to world
	_bulletWorld->addRigidBody(car->_rigidBody);

	{ // Set up actual vehicle stuff
		car->_bulletVehicleRaycaster = new btDefaultVehicleRaycaster(_bulletWorld);

		btVehicleRL::btVehicleTuning tuning = btVehicleRL::btVehicleTuning();

		car->_bulletVehicle = new btVehicleRL(tuning, car->_rigidBody, car->_bulletVehicleRaycaster);

		// Match RL with X forward, Y right, Z up
		car->_bulletVehicle->setCoordinateSystem(1, 2, 0);

		// Set up wheel directions with RL coordinate system
		btVector3 wheelDirectionCS(0, 0, -1), wheelAxleCS(0, -1, 0);

		{ // Set up wheels
			for (int i = 0; i < 4; i++) {
				bool front = i < 2;
				bool left = i % 2;

				
				float radius = front ? config.frontWheels.wheelRadius : config.backWheels.wheelRadius;
				btVector3 wheelRayStartOffset = front ? config.frontWheels.connectionPointOffset : config.backWheels.connectionPointOffset;

				if (left)
					wheelRayStartOffset.y() *= -1;

				float suspensionRestLength = front ? config.frontWheels.suspensionRestLength : config.backWheels.suspensionRestLength;

				suspensionRestLength -= RLConst::BTVehicle::MAX_SUSPENSION_TRAVEL;

				car->_bulletVehicle->addWheel(
					wheelRayStartOffset * UU_TO_BT,
					wheelDirectionCS, wheelAxleCS,
					suspensionRestLength * UU_TO_BT,
					radius * UU_TO_BT, tuning, true);

				{ // Fix wheel info data
					using namespace RLConst::BTVehicle;

					float suspensionScale = 
						front ? SUSPENSION_FORCE_SCALE_FRONT : SUSPENSION_FORCE_SCALE_BACK;

					btWheelInfoRL& wheelInfo = car->_bulletVehicle->m_wheelInfo[i];
					wheelInfo.m_suspensionStiffness =		SUSPENSION_STIFFNESS * suspensionScale;
					wheelInfo.m_wheelsDampingCompression =	WHEELS_DAMPING_COMPRESSION * suspensionScale;
					wheelInfo.m_wheelsDampingRelaxation =	WHEELS_DAMPING_RELAXATION * suspensionScale;
					wheelInfo.m_maxSuspensionTravelCm = (MAX_SUSPENSION_TRAVEL * UU_TO_BT) * 100; // Same for all cars (hopefully)
					wheelInfo.m_maxSuspensionForce = FLT_MAX; // Don't think there's a limit
					wheelInfo.m_bIsFrontWheel = front;
				}
			}
		}
	}

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

		_bulletWorld->setGravity(btVector3(0, 0, RLConst::GRAVITY_Z * UU_TO_BT));
	}

	_SetupArenaCollisionShapes();
#if 0
	{ // Initialize arena collision mesh
		_arenaTriMesh = ArenaMesh::GenerateTriMesh();
		btBvhTriangleMeshShape* triMeshShape = new btBvhTriangleMeshShape(_arenaTriMesh, true);
		triMeshShape->buildOptimizedBvh();
		_worldCollisionShapes.push_back(triMeshShape);
		btRigidBody::btRigidBodyConstructionInfo constructionInfo =
			btRigidBody::btRigidBodyConstructionInfo(0, NULL, triMeshShape);

		constructionInfo.m_startWorldTransform.setIdentity();
		constructionInfo.m_startWorldTransform.setOrigin({0, 0, 0});
	
		btRigidBody* arenaStaticRB = new btRigidBody(constructionInfo);
		_worldCollisionRBs.push_back(arenaStaticRB);
		_bulletWorld->addRigidBody(arenaStaticRB);

		arenaStaticRB->setCollisionFlags(1);
	}
#endif

	{ // Initialize ball
		ball = Ball::_AllocBall();

		float radius;
		switch (gameMode) {
		default:
			radius = RLConst::BALL_COLLISION_RADIUS_NORMAL;
			break;
		}
		radius *= UU_TO_BT;

		ball->_collisionShape = new btSphereShape(radius);

		btRigidBody::btRigidBodyConstructionInfo constructionInfo =
			btRigidBody::btRigidBodyConstructionInfo(RLConst::BALL_MASS_BT, NULL, ball->_collisionShape);

		constructionInfo.m_startWorldTransform.setIdentity();
		constructionInfo.m_startWorldTransform.setOrigin(btVector3(0, 0, radius));

		constructionInfo.m_angularSleepingThreshold = 0;
		constructionInfo.m_linearSleepingThreshold = 0.001;
		constructionInfo.m_linearDamping = RLConst::BALL_DRAG;

		constructionInfo.m_rollingFriction = constructionInfo.m_friction = 0;
		constructionInfo.m_restitution = 0.6f;

		ball->_rigidBody = new btRigidBody(constructionInfo);
	}

	// Add ball to world
	_bulletWorld->addRigidBody(ball->_rigidBody);
}

void Arena::Step(int ticksToSimulate) {
	for (int i = 0; i < ticksToSimulate; i++) {
		for (Car* car : _carsList)
			car->_PreTickUpdate();

		// Update world
		_bulletWorld->stepSimulation(TICKTIME, 0, TICKTIME);

		for (Car* car : _carsList)
			car->_PostTickUpdate();

		{ // Limit ball's linear/angular velocity
			using namespace RLConst;

			btVector3 ballVel = ball->_rigidBody->getLinearVelocity();
			btVector3 ballAngVel = ball->_rigidBody->getAngularVelocity();

			if (ballVel.length2() > (BALL_MAX_SPEED * BALL_MAX_SPEED))
				ballVel = ballVel.normalized() * BALL_MAX_SPEED;

			if (ballAngVel.length2() > (BALL_MAX_ANG_SPEED * BALL_MAX_ANG_SPEED))
				ballVel = ballVel.normalized() * BALL_MAX_ANG_SPEED;

			ball->_rigidBody->setLinearVelocity(ballVel);
			ball->_rigidBody->setAngularVelocity(ballAngVel);
		}

		tickCount++;
	}
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

	// Remove arena collision meshes
	for (btTriangleMesh* mesh : _arenaTriMeshes) {
		delete mesh;
	}
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
		ERR_CLOSE(
			"Failed to find soccar field asset files at \"" << basePath
			<< "\", the assets folder should be in our current directory " << std::filesystem::current_path() << ".")
	}

	// Create triangle meshes
	MeshLoader::Mesh
		cornerMesh	= MeshLoader::LoadMeshFromFiles(basePath + "soccar_corner", 2),
		goalMesh	= MeshLoader::LoadMeshFromFiles(basePath + "soccar_goal", 2),
		rampsMeshA	= MeshLoader::LoadMeshFromFiles(basePath + "soccar_ramps_0", 2),
		rampsMeshB	= MeshLoader::LoadMeshFromFiles(basePath + "soccar_ramps_1", 2);

	constexpr float PLANE_THICKNESS = 10;
	constexpr float WALL_SIZE = 120;
	
	constexpr float EXTENT_X = 4096 * UU_TO_BT;
	constexpr float EXTENT_Y = 5120 * UU_TO_BT;
	constexpr float EXTENT_Z = 2048 * UU_TO_BT;

	// Floor
	_AddStaticCollisionShape(
		new btBoxShape({ WALL_SIZE, WALL_SIZE, PLANE_THICKNESS }),
		{ 0, 0, -PLANE_THICKNESS }
	);

	// Ceiling
	_AddStaticCollisionShape(
		new btBoxShape({ WALL_SIZE, WALL_SIZE, PLANE_THICKNESS }),
		{ 0, 0, EXTENT_Z + PLANE_THICKNESS }
	);

	// Side walls
	_AddStaticCollisionShape(
		new btBoxShape({ PLANE_THICKNESS, WALL_SIZE, WALL_SIZE }),
		{ -(EXTENT_X + PLANE_THICKNESS), 0, EXTENT_Z / 2 }
	);
	_AddStaticCollisionShape(
		new btBoxShape({ PLANE_THICKNESS, WALL_SIZE, WALL_SIZE }),
		{ (EXTENT_X + PLANE_THICKNESS), 0, EXTENT_Z / 2 }
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
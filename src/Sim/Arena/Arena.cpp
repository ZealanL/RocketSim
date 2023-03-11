#include "Arena.h"
#include "../../RLConst.h"

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
			_bulletWorld->removeCollisionObject(car->_rigidBody);
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

void Arena::ResetToRandomKickoff(int seed) {
	using namespace RLConst;

	// TODO: Make shuffling of kickoff setup more efficient (?)

	static vector<int> kickoffOrder;
	if (kickoffOrder.empty()) {
		for (int i = 0; i < CAR_SPAWN_LOCATION_AMOUNT; i++)
			kickoffOrder.push_back(i);
	}

	if (seed == -1) {
		std::default_random_engine& randEngine = Math::GetRandEngine();
		std::shuffle(kickoffOrder.begin(), kickoffOrder.end(), randEngine);
	} else {
		std::default_random_engine randEngine = std::default_random_engine(seed);
		std::shuffle(kickoffOrder.begin(), kickoffOrder.end(), randEngine);
	}

	vector<Car*> blueCars, orangeCars;
	for (Car* car : _cars)
		((car->team == Team::BLUE) ? blueCars : orangeCars).push_back(car);

	int kickoffPositionAmount = RS_MAX(blueCars.size(), orangeCars.size());
	for (int i = 0; i < kickoffPositionAmount; i++) {
		CarSpawnPos spawnPos = CAR_SPAWN_LOCATIONS[kickoffOrder[i]];

		for (int teamIndex = 0; teamIndex < 2; teamIndex++) {
			bool isBlue = (teamIndex == 0);
			vector<Car*> teamCars = isBlue ? blueCars : orangeCars;

			if (i < teamCars.size()) {
				CarState spawnState;
				spawnState.pos = { spawnPos.x, spawnPos.y, CAR_SPAWN_REST_Z };
				
				Angle angle = Angle(spawnPos.yawAng, 0, 0);
				spawnState.isOnGround = true;

				if (!isBlue) {
					spawnState.pos *= { -1, -1, 1 };
					angle.yaw += M_PI;
				}

				spawnState.rotMat = angle.ToRotMat();

				teamCars[i]->SetState(spawnState);
			}
		}
	}

	ball->SetState(BallState());

	for (BoostPad* boostPad : _boostPads)
		boostPad->SetState(BoostPadState());
}

bool Arena::_BulletContactAddedCallback(
	btManifoldPoint& contactPoint,
	const btCollisionObjectWrapper* objA, int partIdA, int indexA,
	const btCollisionObjectWrapper* objB, int partIdB, int indexB) {

	auto
		bodyA = objA->m_collisionObject,
		bodyB = objB->m_collisionObject;

	bool shouldSwap = false;
	if ((bodyA->getUserIndex() != -1) && (bodyB->getUserIndex() != -1)) {
		// If both bodies have a user index, the lower user index should be A
		shouldSwap = bodyA->getUserIndex() > bodyB->getUserIndex();
	} else {
		// If only one body has a user index, make sure that body is A
		shouldSwap = (bodyB->getUserIndex() != -1);
	}

	if (shouldSwap)
		std::swap(bodyA, bodyB);

	int
		userIndexA = bodyA->getUserIndex(),
		userIndexB = bodyB->getUserIndex();

	bool carInvolved = (bodyA->getUserIndex() == BT_USERINFO_TYPE_CAR);
	if (carInvolved) {

		Car* car = (Car*)bodyA->getUserPointer();
		Arena* arenaInst = (Arena*)car->_bulletVehicle->m_dynamicsWorld->getWorldUserInfo();

		if (userIndexB == BT_USERINFO_TYPE_BALL) {
			// Car + Ball
			arenaInst->
				_BtCallback_OnCarBallCollision(car, (Ball*)bodyB->getUserPointer(), contactPoint);
		} else if (userIndexB == BT_USERINFO_TYPE_CAR) {
			// Car + Car
			// Do collision both ways
			arenaInst->
				_BtCallback_OnCarCarCollision(car, (Car*)bodyB->getUserPointer(), contactPoint);
			arenaInst->
				_BtCallback_OnCarCarCollision((Car*)bodyB->getUserPointer(), car, contactPoint);
		} else if (userIndexB == BT_USERINFO_TYPE_BOOSTPAD) {
			// Car + BoostPad hitbox
			arenaInst->
				_BtCallback_OnCarBoostPadCollision(car, (BoostPad*)bodyB->getUserPointer(), contactPoint);
		} else {
			// Car + World
			arenaInst->
				_BtCallback_OnCarWorldCollision(car, (btCollisionObject*)bodyB->getUserPointer(), contactPoint);
		}
	}
	return true;
}

void Arena::_BtCallback_OnCarBallCollision(Car* car, Ball* ball, btManifoldPoint& manifoldPoint) {
	using namespace RLConst;

	// Manually override manifold friction/restitution
	manifoldPoint.m_combinedFriction = RLConst::CARBALL_COLLISION_FRICTION;
	manifoldPoint.m_combinedRestitution = RLConst::CARBALL_COLLISION_RESTITUTION;

	// Once we do an extra car-ball impulse, we need to wait at least 1 tick to do it again
	if ((tickCount > car->_internalState.lastHitBallTick + 1) || (car->_internalState.lastHitBallTick > tickCount)) {
		car->_internalState.lastHitBallTick = tickCount;
	} else {
		// Don't do multiple extra impulses in a row
		return;
	}

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

		// Velocity won't be actually added until the end of this tick
		ball->_velocityImpulseCache += addedVel * UU_TO_BT;
	}
}

void Arena::_BtCallback_OnCarCarCollision(Car* car1, Car* car2, btManifoldPoint& manifoldPoint) {
	using namespace RLConst;

	// Manually override manifold friction/restitution
	manifoldPoint.m_combinedFriction = RLConst::CARCAR_COLLISION_FRICTION;
	manifoldPoint.m_combinedRestitution = RLConst::CARCAR_COLLISION_RESTITUTION;

	CarState
		state = car1->GetState(),
		otherState = car2->GetState();

	if ((state.carContact.otherCar == car2) && (state.carContact.cooldownTimer > 0))
		return; // In cooldown

	Vec deltaPos = (otherState.pos - state.pos);
	if (state.vel.Dot(deltaPos) > 0) { // Going towards other car

		Vec velDir = state.vel.Normalized();
		Vec dirToOtherCar = deltaPos.Normalized();

		float speedTowardsOtherCar = state.vel.Dot(dirToOtherCar);
		float otherCarAwaySpeed = otherState.vel.Dot(velDir);

		if (speedTowardsOtherCar > otherCarAwaySpeed) { // Going towards other car faster than they are going away

			bool hitWithBumper = manifoldPoint.m_localPointA.x() * BT_TO_UU > BUMP_MIN_FORWARD_DIST;
			if (hitWithBumper) {

				if (state.isSupersonic) {
					car2->Demolish();
				} else {
					bool groundHit = car2->_internalState.isOnGround;

					float baseScale =
						(groundHit ? BUMP_VEL_AMOUNT_GROUND_CURVE : BUMP_VEL_AMOUNT_AIR_CURVE).GetOutput(speedTowardsOtherCar);

					Vec hitUpDir =
						(otherState.isOnGround ? (Vec)car2->_bulletVehicle->getUpVector() : Vec(0, 0, 1));

					Vec bumpImpulse =
						velDir * baseScale +
						hitUpDir * BUMP_UPWARD_VEL_AMOUNT_CURVE.GetOutput(speedTowardsOtherCar);

					car2->_velocityImpulseCache += bumpImpulse * UU_TO_BT;
					car1->_internalState.carContact.otherCar = car2;
					car1->_internalState.carContact.cooldownTimer = BUMP_COOLDOWN_TIME;
				}
			}
		}
	}
}

void Arena::_BtCallback_OnCarBoostPadCollision(Car* car, BoostPad* pad, btManifoldPoint& manifoldPoint) {
	pad->_OnCollide(car->_rigidBody);
}

void Arena::_BtCallback_OnCarWorldCollision(Car* car, btCollisionObject* world, btManifoldPoint& manifoldPoint) {
	car->_internalState.worldContact.hasContact = true;
	car->_internalState.worldContact.contactNormal = manifoldPoint.m_normalWorldOnB;

	// Manually override manifold friction/restitution
	manifoldPoint.m_combinedFriction = RLConst::CARWORLD_COLLISION_FRICTION;
	manifoldPoint.m_combinedRestitution = RLConst::CARWORLD_COLLISION_RESTITUTION;
}

Arena::Arena(GameMode gameMode, float tickRate) {

	// Tickrate must be from 15 to 120tps
	assert(tickRate >= 15 && tickRate <= 120);

	{ // Seed std::rand()
		static bool seedRandom = true;
		if (seedRandom) {
			srand(time(NULL));
			seedRandom = false;
		}
	}

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

		// Adjust solver configuration to be closer to older Bullet (Rocket League's Bullet is from somewhere between 2013 and 2015)
		auto& solverInfo = _bulletWorld->getSolverInfo();
		solverInfo.m_splitImpulsePenetrationThreshold = 1.0e30;
		solverInfo.m_erp2 = 0.8f;
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

	{ // Initialize boost pads
		using namespace RLConst::BoostPads;

		_boostPads.reserve(LOCS_AMOUNT_BIG + LOCS_AMOUNT_SMALL);

		for (int i = 0; i < (LOCS_AMOUNT_BIG + LOCS_AMOUNT_SMALL); i++) {
			bool isBig = i < LOCS_AMOUNT_BIG;

			btVector3 pos = isBig ? LOCS_BIG[i] : LOCS_SMALL[i - LOCS_AMOUNT_BIG];

			BoostPad* pad = BoostPad::_AllocBoostPad();
			pad->_BulletSetup(_bulletWorld, isBig, pos * UU_TO_BT);

			_boostPads.push_back(pad);
		}
	}

	// Set internal tick callback
	_bulletWorld->setWorldUserInfo(this);

	gContactAddedCallback = &Arena::_BulletContactAddedCallback;
}

void Arena::Step(int ticksToSimulate) {
	for (int i = 0; i < ticksToSimulate; i++) {

		_bulletWorld->setWorldUserInfo(this);

		{ // Ball zero-vel sleeping
			if (ball->_rigidBody->m_linearVelocity.length2() == 0 && ball->_rigidBody->m_angularVelocity.length2() == 0) {
				// hooooooonk mimimimimimimi hooooooonk mimimimimimimi
				ball->_rigidBody->setActivationState(ISLAND_SLEEPING);
			} else {
				ball->_rigidBody->setActivationState(ACTIVE_TAG);
			}
		}

		for (Car* car : _cars)
			car->_PreTickUpdate(tickTime);

		for (BoostPad* pad : _boostPads)
			pad->_PreTickUpdate(tickTime);

		// Update world
		_bulletWorld->stepSimulation(tickTime, 0, tickTime);

		for (Car* car : _cars) {
			car->_PostTickUpdate(tickTime);
			car->_FinishPhysicsTick();
		}

		for (BoostPad* pad : _boostPads)
			pad->_PostTickUpdate(tickTime);

		ball->_FinishPhysicsTick();

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
	for (Car* car : _cars)
		delete car;

	// Remove all boost pads
	for (BoostPad* boostPad : _boostPads)
		delete boostPad;

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

btRigidBody* Arena::_AddStaticCollisionShape(btCollisionShape* shape, btVector3 pos) {
	_worldCollisionShapes.push_back(shape);

	btRigidBody* shapeRB = new btRigidBody(0, NULL, shape);
	shapeRB->setWorldTransform(btTransform(btMatrix3x3::getIdentity(), pos));
	_worldCollisionRBs.push_back(shapeRB);

	_bulletWorld->addRigidBody(shapeRB);
	return shapeRB;
}

void Arena::_AddStaticCollisionTris(CollisionMeshFile& file) {
	auto triMesh = file.MakeBulletMesh();
	_arenaTriMeshes.push_back(triMesh);

	auto bvt = new btBvhTriangleMeshShape(triMesh, false);
	bvt->buildOptimizedBvh();
	_AddStaticCollisionShape(bvt, btVector3(0,0,0));
}

void Arena::_SetupArenaCollisionShapes() {
	// TODO: This is just for soccar arena (for now)
	assert(gameMode == GameMode::SOCCAR);

	static std::mutex collisionMeshLoadMutex;
	static vector<CollisionMeshFile> collisionMeshes;
	static bool collisionMeshesFullyloaded = false;

	if (!collisionMeshesFullyloaded) {
		collisionMeshLoadMutex.lock();
		if (collisionMeshes.empty()) {
			string basePath = COLLISION_MESH_SOCCAR_PATH;
			RS_LOG("Loading arena meshes from \"" << basePath << "\"...");

			if (!std::filesystem::exists(basePath)) {
				RS_ERR_CLOSE(
					"Failed to find arena collision mesh files at \"" << basePath
					<< "\", the collision meshes folder should be in our current directory " << std::filesystem::current_path() << ".")
			}
				// Load collision meshes
				auto dirItr = std::filesystem::directory_iterator(basePath);
			for (auto& entry : dirItr) {
				auto entryPath = entry.path();
				if (entryPath.has_extension() && entryPath.extension() == COLLISION_MESH_FILE_EXTENSION) {
					CollisionMeshFile meshFile = {};
					meshFile.ReadFromFile(entryPath.string());
					collisionMeshes.push_back(meshFile);
				}
			}

			if (collisionMeshes.empty()) {
				RS_ERR_CLOSE(
					"Failed to find soccar field asset files at \"" << basePath
					<< "\", the folder exists but is empty.")
			}

			RS_LOG("Finished loading " << collisionMeshes.size() << " arena collision meshes.");
			collisionMeshesFullyloaded = true;
		}
		collisionMeshLoadMutex.unlock();
	}
	
	// Add collision meshes to world
	for (CollisionMeshFile& mesh : collisionMeshes)
		_AddStaticCollisionTris(mesh);

	{ // Add arena collision planes (floor/walls/ceiling)
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
	}
}
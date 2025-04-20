#include "Arena.h"
#include "../../RocketSim.h"

#include "../../../libsrc/bullet3-3.24/BulletCollision/BroadphaseCollision/btAxisSweep3.h"
#include "../../../libsrc/bullet3-3.24/BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"
#include "../../../libsrc/bullet3-3.24/BulletCollision/BroadphaseCollision/btSimpleBroadphase.h"
#include "../../../libsrc/bullet3-3.24/BulletCollision/BroadphaseCollision/btRSBroadphase.h"
#include "../../../libsrc/bullet3-3.24/BulletCollision/BroadphaseCollision/btOverlappingPairCache.h"
#include "../../../libsrc/bullet3-3.24/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"
#include "../../../libsrc/bullet3-3.24/BulletCollision/CollisionDispatch/btInternalEdgeUtility.h"
#include "../../../libsrc/bullet3-3.24/BulletCollision/CollisionShapes/btBoxShape.h"
#include "../../../libsrc/bullet3-3.24/BulletCollision/CollisionShapes/btSphereShape.h"
#include "DropshotTiles/DropshotTiles.h"

RS_NS_START

void Arena::SetMutatorConfig(const MutatorConfig& mutatorConfig) {

	bool
		ballChanged = mutatorConfig.ballRadius != this->_mutatorConfig.ballRadius || mutatorConfig.ballMass != this->_mutatorConfig.ballMass,
		carMassChanged = mutatorConfig.carMass != this->_mutatorConfig.carMass,
		gravityChanged = mutatorConfig.gravity != this->_mutatorConfig.gravity;

	this->_mutatorConfig = mutatorConfig;

	_bulletWorld.setGravity(mutatorConfig.gravity * UU_TO_BT);
	
	if (ballChanged) {
		// We'll need to remake the ball
		_bulletWorld.removeCollisionObject(&ball->_rigidBody);
		delete ball->_collisionShape;
		ball->_BulletSetup(gameMode, &_bulletWorld, mutatorConfig, _config.noBallRot);
	}

	if (carMassChanged) {
		for (Car* car : _cars) {
			btVector3 newCarInertia;
			car->_childHitboxShape.calculateLocalInertia(mutatorConfig.carMass, newCarInertia);
			car->_rigidBody.setMassProps(mutatorConfig.ballMass, newCarInertia);
		}
	}

	// Update ball rigidbody physics values for world contact
	// NOTE: Cars don't use their rigidbody physics values for world contact
	ball->_rigidBody.setFriction(mutatorConfig.ballWorldFriction);
	ball->_rigidBody.setRestitution(mutatorConfig.ballWorldRestitution);
	ball->_rigidBody.setDamping(mutatorConfig.ballDrag, 0);
}

Car* Arena::AddCar(Team team, const CarConfig& config) {
	Car* car = Car::_AllocateCar();
	
	car->config = config;
	car->team = team;
	
	_AddCarFromPtr(car);

	car->_BulletSetup(gameMode, &_bulletWorld, _mutatorConfig);
	car->Respawn(gameMode, -1, _mutatorConfig.carSpawnBoostAmount);

	return car;
}

bool Arena::_AddCarFromPtr(Car* car) {

	car->id = ++_lastCarID;

	if (_carIDMap.find(car->id) == _carIDMap.end()) {
		assert(!_cars.count(car));
		
		_carIDMap[car->id] = car;
		_cars.insert(car);
		return true;

	} else {
		return false;
	}
}

bool Arena::RemoveCar(uint32_t id) {
	auto itr = _carIDMap.find(id);

	if (itr != _carIDMap.end()) {
		Car* car = itr->second;
		_carIDMap.erase(itr);
		_cars.erase(car);
		_bulletWorld.removeCollisionObject(&car->_rigidBody);
		if (ownsCars)
			delete car;
		return true;
	} else {
		return false;
	}
}

Car* Arena::GetCar(uint32_t id) {
	return _carIDMap[id];
}

void Arena::SetGoalScoreCallback(GoalScoreEventFn callbackFunc, void* userInfo) {
	if (gameMode == GameMode::THE_VOID)
		RS_ERR_CLOSE("Cannot set a goal score callback when on THE_VOID gamemode");

	_goalScoreCallback.func = callbackFunc;
	_goalScoreCallback.userInfo = userInfo;
}

void Arena::SetCarBumpCallback(CarBumpEventFn callbackFunc, void* userInfo) {
	_carBumpCallback.func = callbackFunc;
	_carBumpCallback.userInfo = userInfo;
}

void Arena::ResetToRandomKickoff(int seed) {
	using namespace RLConst;
	// TODO: Make shuffling of kickoff setup more efficient (?)

	static thread_local std::array<int, CAR_SPAWN_LOCATION_AMOUNT> KICKOFF_ORDER_TEMPLATE = { -1 };
	if (KICKOFF_ORDER_TEMPLATE[0] == -1) {
		// Initialize
		for (int i = 0; i < CAR_SPAWN_LOCATION_AMOUNT; i++)
			KICKOFF_ORDER_TEMPLATE[i] = i;
	}

	auto kickoffOrder = KICKOFF_ORDER_TEMPLATE;

	std::default_random_engine* randEngine;
	if (seed == -1) {
		randEngine = &Math::GetRandEngine();
	} else {
		randEngine = new std::default_random_engine(seed);
	}

	int locationAmount = (gameMode == GameMode::HEATSEEKER) ? CAR_SPAWN_LOCATION_AMOUNT_HEATSEEKER : CAR_SPAWN_LOCATION_AMOUNT;

	std::shuffle(kickoffOrder.begin(), kickoffOrder.begin() + locationAmount, *randEngine);

	const CarSpawnPos* CAR_SPAWN_LOCATIONS = CAR_SPAWN_LOCATIONS_SOCCAR;
	const CarSpawnPos* CAR_RESPAWN_LOCATIONS = CAR_RESPAWN_LOCATIONS_SOCCAR;
	if (gameMode == GameMode::HOOPS) {
		CAR_SPAWN_LOCATIONS = CAR_SPAWN_LOCATIONS_HOOPS;
		CAR_RESPAWN_LOCATIONS = CAR_RESPAWN_LOCATIONS_HOOPS;
	} else if (gameMode == GameMode::HEATSEEKER) {
		CAR_SPAWN_LOCATIONS = CAR_SPAWN_LOCATIONS_HEATSEEKER;
		CAR_RESPAWN_LOCATIONS = CAR_RESPAWN_LOCATIONS_SOCCAR;
	} else if (gameMode == GameMode::DROPSHOT) {
		CAR_SPAWN_LOCATIONS = CAR_SPAWN_LOCATIONS_DROPSHOT;
		CAR_RESPAWN_LOCATIONS = CAR_RESPAWN_LOCATIONS_DROPSHOT;
	}

	std::vector<Car*> blueCars, orangeCars;
	for (Car* car : _cars)
		((car->team == Team::BLUE) ? blueCars : orangeCars).push_back(car);

	int numCarsAtRespawnPos[CAR_RESPAWN_LOCATION_AMOUNT] = {};

	int kickoffPositionAmount = RS_MAX(blueCars.size(), orangeCars.size());
	for (int i = 0; i < kickoffPositionAmount; i++) {

		CarSpawnPos spawnPos;
	
		if (i < locationAmount) {
			spawnPos = CAR_SPAWN_LOCATIONS[RS_MIN(kickoffOrder[i], locationAmount - 1)];
		} else {
			int respawnPosIdx = (i - (locationAmount)) % locationAmount;
			spawnPos = CAR_RESPAWN_LOCATIONS[respawnPosIdx];

			// Extra offset to add to multiple cars spawning at the same respawn point,
			//	helps prevent insane numbers of cars from spawning in eachother.
			// Eventually, they will spawn so far away that they clip out of the arena,
			//	but that's not my problem.
			constexpr float CAR_SPAWN_EXTRA_OFFSET_Y = 250;
			spawnPos.y += CAR_SPAWN_EXTRA_OFFSET_Y * numCarsAtRespawnPos[respawnPosIdx];
			numCarsAtRespawnPos[respawnPosIdx]++;
		}

		for (int teamIndex = 0; teamIndex < 2; teamIndex++) {
			bool isBlue = (teamIndex == 0);
			std::vector<Car*> teamCars = isBlue ? blueCars : orangeCars;

			if (i < teamCars.size()) {
				CarState spawnState;
				spawnState.boost = _mutatorConfig.carSpawnBoostAmount;
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

	BallState ballState = BallState();
	if (gameMode == GameMode::HEATSEEKER) {
		int nextRand = (*randEngine)();
		Vec scale = Vec(1, (nextRand % 2) ? 1 : -1, 1);
		ballState.pos = Heatseeker::BALL_START_POS * scale;
		ballState.vel = Heatseeker::BALL_START_VEL * scale;
	} else if (gameMode == GameMode::SNOWDAY) {
		// Don't freeze
		ballState.vel.z = FLT_EPSILON;
	}
	ball->SetState(ballState);

	// Reset boost pads
	for (BoostPad* boostPad : _boostPads)
		boostPad->SetState(BoostPadState());

	// Reset tile states
	if (gameMode == GameMode::DROPSHOT)
		SetDropshotTilesState({});

	if (seed != -1) {
		// Custom random engine was created for this seed, so we need to free it
		delete randEngine;
	}
}

void Arena::SetDropshotTilesState(const DropshotTilesState& state) {
	for (int teamIdx = 0; teamIdx <= 1; teamIdx++) {
		for (int tileIdx = 0; tileIdx < RLConst::Dropshot::NUM_TILES_PER_TEAM; tileIdx++) {
			auto& newState = state.states[teamIdx][tileIdx];

			int rbIndex = tileIdx + (RLConst::Dropshot::NUM_TILES_PER_TEAM * teamIdx);
			assert(rbIndex < _worldDropshotTileRBs.size());
			auto dropshotTileRB = _worldDropshotTileRBs[rbIndex];
			if (newState.damageState == DropshotTileState::STATE_BROKEN) {
				dropshotTileRB->m_collisionFlags |= btCollisionObject::CF_NO_CONTACT_RESPONSE;
			} else {
				dropshotTileRB->m_collisionFlags &= ~btCollisionObject::CF_NO_CONTACT_RESPONSE;
			}
		}
	}

	_dropshotTilesState = state;
}

bool Arena::_BulletContactAddedCallback(
	btManifoldPoint& contactPoint,
	const btCollisionObjectWrapper* objA, int partID_A, int indexA,
	const btCollisionObjectWrapper* objB, int partID_B, int indexB) {

	auto
		bodyA = objA->m_collisionObject,
		bodyB = objB->m_collisionObject;

	if (!objA->m_collisionObject->hasContactResponse() || !objB->m_collisionObject->hasContactResponse())
		return true;

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

	bool carInvolved = (userIndexA == BT_USERINFO_TYPE_CAR);
	if (carInvolved) {

		Car* car = (Car*)bodyA->getUserPointer();
		Arena* arenaInst = (Arena*)car->_bulletVehicle.m_dynamicsWorld->getWorldUserInfo();

		if (userIndexB == BT_USERINFO_TYPE_BALL) {
			// Car + Ball
			arenaInst->
				_BtCallback_OnCarBallCollision(car, (Ball*)bodyB->getUserPointer(), contactPoint, shouldSwap);
		} else if (userIndexB == BT_USERINFO_TYPE_CAR) {
			// Car + Car
			arenaInst->
				_BtCallback_OnCarCarCollision(car, (Car*)bodyB->getUserPointer(), contactPoint);
		} else {
			// Car + World
			arenaInst->
				_BtCallback_OnCarWorldCollision(car, (btCollisionObject*)bodyB->getUserPointer(), contactPoint);
		}
	} else if (userIndexA == BT_USERINFO_TYPE_BALL && userIndexB == BT_USERINFO_TYPE_DROPSHOT_TILE) {

		Arena* arenaInst = (Arena*)bodyB->getUserPointer();
		arenaInst->ball->_OnDropshotTileCollision(
			arenaInst->_dropshotTilesState, bodyB->getUserIndex2(), bodyB, arenaInst->tickCount, arenaInst->tickTime
		);

	} else if (userIndexA == BT_USERINFO_TYPE_BALL && userIndexB == -1) {
		// Ball + World
		Arena* arenaInst = (Arena*)bodyB->getUserPointer();
		arenaInst->ball->_OnWorldCollision(arenaInst->gameMode, contactPoint.m_normalWorldOnB, arenaInst->tickTime);
		
		// Set as special (unless in snowday)
		if (arenaInst->gameMode != GameMode::SNOWDAY)
			contactPoint.m_isSpecial = true;
	}
	
	btAdjustInternalEdgeContacts(
		contactPoint, 
		(shouldSwap ? objA : objB), (shouldSwap ? objB : objA),
		(shouldSwap ? partID_A : partID_B), (shouldSwap ? indexA : indexB)
	);
	return true;
}

void Arena::_BtCallback_OnCarBallCollision(Car* car, Ball* ball, btManifoldPoint& manifoldPoint, bool ballIsBodyA) {
	using namespace RLConst;

	Vec relBallPos = (ballIsBodyA ? manifoldPoint.m_localPointA : manifoldPoint.m_localPointB) * BT_TO_UU;
	ball->_OnHit(car, relBallPos, manifoldPoint.m_combinedFriction, manifoldPoint.m_combinedRestitution, gameMode, _mutatorConfig, tickCount);
}

void Arena::_BtCallback_OnCarCarCollision(Car* car1, Car* car2, btManifoldPoint& manifoldPoint) {
	using namespace RLConst;

	// Manually override manifold friction/restitution
	manifoldPoint.m_combinedFriction = RLConst::CARCAR_COLLISION_FRICTION;
	manifoldPoint.m_combinedRestitution = RLConst::CARCAR_COLLISION_RESTITUTION;

	// Test collision both ways
	for (int i = 0; i < 2; i++) {

		bool isSwapped = (i == 1);
		if (isSwapped)
			std::swap(car1, car2);

		CarState
			state = car1->GetState(),
			otherState = car2->GetState();

		if (state.isDemoed || otherState.isDemoed)
			return;

		if ((state.carContact.otherCarID == car2->id) && (state.carContact.cooldownTimer > 0))
			continue; // In cooldown

		Vec deltaPos = (otherState.pos - state.pos);
		if (state.vel.Dot(deltaPos) > 0) { // Going towards other car

			Vec velDir = state.vel.Normalized();
			Vec dirToOtherCar = deltaPos.Normalized();

			float speedTowardsOtherCar = state.vel.Dot(dirToOtherCar);
			float otherCarAwaySpeed = otherState.vel.Dot(velDir);

			if (speedTowardsOtherCar > otherCarAwaySpeed) { // Going towards other car faster than they are going away

				Vec localPoint = isSwapped ? manifoldPoint.m_localPointB : manifoldPoint.m_localPointA;
				bool hitWithBumper = (localPoint.x * BT_TO_UU) > BUMP_MIN_FORWARD_DIST;
				if (hitWithBumper) {

					bool isDemo;
					switch (_mutatorConfig.demoMode) {
					case DemoMode::ON_CONTACT:
						isDemo = true; // BOOM
						break;
					case DemoMode::DISABLED:
						isDemo = false;
						break;
					default:
						isDemo = state.isSupersonic;
					}

					if (isDemo && !_mutatorConfig.enableTeamDemos)
						isDemo = car1->team != car2->team;

					if (isDemo) {
						car2->Demolish(_mutatorConfig.respawnDelay);
					} else {
						bool groundHit = car2->_internalState.isOnGround;

						float baseScale =
							(groundHit ? BUMP_VEL_AMOUNT_GROUND_CURVE : BUMP_VEL_AMOUNT_AIR_CURVE).GetOutput(speedTowardsOtherCar);

						Vec hitUpDir =
							(otherState.isOnGround ? (Vec)car2->GetUpDir() : Vec(0, 0, 1));

						Vec bumpImpulse =
							velDir * baseScale +
							hitUpDir * BUMP_UPWARD_VEL_AMOUNT_CURVE.GetOutput(speedTowardsOtherCar)
							* _mutatorConfig.bumpForceScale;

						car2->_velocityImpulseCache += bumpImpulse * UU_TO_BT;
					}

					car1->_internalState.carContact.otherCarID = car2->id;
					car1->_internalState.carContact.cooldownTimer = _mutatorConfig.bumpCooldownTime;

					if (_carBumpCallback.func)
						_carBumpCallback.func(this, car1, car2, isDemo, _carBumpCallback.userInfo);
				}
			}
		}
	}
}

void Arena::_BtCallback_OnCarWorldCollision(Car* car, btCollisionObject* world, btManifoldPoint& manifoldPoint) {
	car->_internalState.worldContact.hasContact = true;
	car->_internalState.worldContact.contactNormal = manifoldPoint.m_normalWorldOnB;

	// Manually override manifold friction/restitution
	manifoldPoint.m_combinedFriction = _mutatorConfig.carWorldFriction;
	manifoldPoint.m_combinedRestitution = _mutatorConfig.carWorldRestitution;
}

Arena::Arena(GameMode gameMode, const ArenaConfig& config, float tickRate) : _mutatorConfig(gameMode), _config(config) {

	// Tickrate must be from 15 to 120tps
	assert(tickRate >= 15 && tickRate <= 120);

	RocketSim::AssertInitialized("Cannot create Arena, ");

	this->gameMode = gameMode;
	this->tickTime = 1 / tickRate;

	{ // Initialize world

		btDefaultCollisionConstructionInfo collisionConfigConstructionInfo = {};

		// These take up a ton of memory normally
		if (_config.memWeightMode == ArenaMemWeightMode::LIGHT) {
			collisionConfigConstructionInfo.m_defaultMaxPersistentManifoldPoolSize /= 32;
			collisionConfigConstructionInfo.m_defaultMaxCollisionAlgorithmPoolSize /= 64;
		} else {
			collisionConfigConstructionInfo.m_defaultMaxPersistentManifoldPoolSize /= 16;
			collisionConfigConstructionInfo.m_defaultMaxCollisionAlgorithmPoolSize /= 32;
		}

		_bulletWorldParams.collisionConfig.setup(collisionConfigConstructionInfo);

		_bulletWorldParams.collisionDispatcher.setup(&_bulletWorldParams.collisionConfig);
		_bulletWorldParams.constraintSolver = btSequentialImpulseConstraintSolver();

		_bulletWorldParams.overlappingPairCache = new btHashedOverlappingPairCache();
		
		if (_config.useCustomBroadphase) {
			float cellSizeMultiplier = 1;
			if (_config.memWeightMode == ArenaMemWeightMode::LIGHT) {
				// Increase cell size
				cellSizeMultiplier = 2.0f;
			}

			_bulletWorldParams.broadphase = new btRSBroadphase(
				_config.minPos * UU_TO_BT,
				_config.maxPos * UU_TO_BT,
				_config.maxAABBLen * UU_TO_BT * cellSizeMultiplier,
				_bulletWorldParams.overlappingPairCache,
				_config.maxObjects);
		} else {
			_bulletWorldParams.broadphase = new btDbvtBroadphase(_bulletWorldParams.overlappingPairCache);
		}

		_bulletWorld.setup(
			&_bulletWorldParams.collisionDispatcher,
			_bulletWorldParams.broadphase,
			&_bulletWorldParams.constraintSolver,
			&_bulletWorldParams.collisionConfig
		);

		_bulletWorld.setGravity(_mutatorConfig.gravity * UU_TO_BT);

		// Adjust solver configuration to be closer to older Bullet (Rocket League's Bullet is from somewhere between 2013 and 2015)
		auto& solverInfo = _bulletWorld.getSolverInfo();
		solverInfo.m_splitImpulsePenetrationThreshold = 1.0e30f;
		solverInfo.m_erp2 = 0.8f;
	}

	bool loadArenaStuff = gameMode != GameMode::THE_VOID;

	if (loadArenaStuff) {
		_SetupArenaCollisionShapes();

		// Give arena collision shapes the proper restitution/friction values
		for (auto* rb : _worldCollisionRBs) {
			rb->setRestitution(RLConst::ARENA_COLLISION_BASE_RESTITUTION);
			rb->setFriction(RLConst::ARENA_COLLISION_BASE_FRICTION);
			rb->setRollingFriction(0.f);
		}
	}

	{ // Initialize ball
		ball = Ball::_AllocBall();

		ball->_BulletSetup(gameMode, &_bulletWorld, _mutatorConfig, _config.noBallRot);
		ball->SetState(BallState());
	}

	if (loadArenaStuff && gameMode != GameMode::DROPSHOT) { // Initialize boost pads
		using namespace RLConst::BoostPads;

		if (_config.useCustomBoostPads) {
			for (auto& padConfig : _config.customBoostPads) {
				BoostPad* pad = BoostPad::_AllocBoostPad();
				pad->_Setup(padConfig);

				_boostPads.push_back(pad);
			}
		} else {
			bool isHoops = gameMode == GameMode::HOOPS;

			int amountSmall = isHoops ? LOCS_AMOUNT_SMALL_HOOPS : LOCS_AMOUNT_SMALL_SOCCAR;
			_boostPads.reserve(LOCS_AMOUNT_BIG + amountSmall);

			for (int i = 0; i < (LOCS_AMOUNT_BIG + amountSmall); i++) {

				BoostPadConfig padConfig;

				padConfig.isBig = i < LOCS_AMOUNT_BIG;

				btVector3 pos;
				if (isHoops) {
					padConfig.pos = padConfig.isBig ? LOCS_BIG_HOOPS[i] : LOCS_SMALL_HOOPS[i - LOCS_AMOUNT_BIG];
				} else {
					padConfig.pos = padConfig.isBig ? LOCS_BIG_SOCCAR[i] : LOCS_SMALL_SOCCAR[i - LOCS_AMOUNT_BIG];
				}

				BoostPad* pad = BoostPad::_AllocBoostPad();
				pad->_Setup(padConfig);

				_boostPads.push_back(pad);
				_boostPadGrid.Add(pad);
			}
		}
	}

	// Set internal tick callback
	_bulletWorld.setWorldUserInfo(this);

	gContactAddedCallback = &Arena::_BulletContactAddedCallback;
}

Arena* Arena::Create(GameMode gameMode, const ArenaConfig& arenaConfig, float tickRate) {
	return new Arena(gameMode, arenaConfig, tickRate);
}

void Arena::Serialize(DataStreamOut& out) const {
	out.WriteMultiple(gameMode, tickTime, tickCount, _lastCarID);

	_config.Serialize(out);

	{ // Serialize cars
		out.Write<uint32_t>(_cars.size());
		for (auto car : _cars) {
			out.Write(car->team);
			out.Write(car->id);
			car->Serialize(out);
		}
	}

	if (_boostPads.size() > 0) { // Serialize boost pads
		out.Write<uint32_t>(_boostPads.size());
		for (auto pad : _boostPads)
			pad->GetState().Serialize(out);
	}

	{ // Serialize ball
		ball->GetState().Serialize(out);
	}

	{ // Serialize mutators
		_mutatorConfig.Serialize(out);
	}
}

Arena* Arena::DeserializeNew(DataStreamIn& in) {
	constexpr char ERROR_PREFIX[] = "Arena::Deserialize(): ";

	GameMode gameMode;
	float tickTime;
	uint64_t tickCount;
	uint32_t lastCarID;

	in.ReadMultiple(gameMode, tickTime, tickCount, lastCarID);

	ArenaConfig newConfig = {};
	newConfig.Deserialize(in);

	Arena* newArena = new Arena(gameMode, newConfig, 1.f / tickTime);
	newArena->tickCount = tickCount;
	
	{ // Deserialize cars
		uint32_t carAmount = in.Read<uint32_t>();
		for (uint32_t i = 0; i < carAmount; i++) {
			Team team;
			uint32_t id;
			in.Read(team);
			in.Read(id);

#ifndef RS_MAX_SPEED
			if (newArena->_carIDMap.count(id))
				RS_ERR_CLOSE(ERROR_PREFIX << "Failed to load, got repeated car ID of " << id);
#endif

			Car* newCar = newArena->DeserializeNewCar(in, team);

			// Force ID
			newArena->_carIDMap.erase(newCar->id);
			newArena->_carIDMap[id] = newCar;
			newCar->id = id;
		}

		newArena->_lastCarID = lastCarID;
	}

	// Deserialize boost pads
	if (newArena->_boostPads.size() > 0) {
		uint32_t boostPadAmount = in.Read<uint32_t>();

#ifndef RS_MAX_SPEED
		if (boostPadAmount != newArena->_boostPads.size())
			RS_ERR_CLOSE(ERROR_PREFIX << "Failed to load, " <<
				"different boost pad amount written in file (" << boostPadAmount << "/" << newArena->_boostPads.size() << ")");
#endif

		for (auto pad : newArena->_boostPads) {
			BoostPadState padState = BoostPadState();
			padState.Deserialize(in);
			pad->SetState(padState);
		}
	}

	{ // Deserialize ball
		BallState ballState = BallState();
		ballState.Deserialize(in);
		newArena->ball->SetState(ballState);
	}

	{ // Serialize mutators
		newArena->_mutatorConfig.Deserialize(in);
		newArena->SetMutatorConfig(newArena->_mutatorConfig);
	}

	return newArena;
}

Arena* Arena::Clone(bool copyCallbacks) {
	Arena* newArena = new Arena(this->gameMode, this->_config, this->GetTickRate());
	
	if (copyCallbacks) {
		newArena->_goalScoreCallback = this->_goalScoreCallback;
		newArena->_carBumpCallback = this->_carBumpCallback;
	}

	newArena->ball->SetState(this->ball->GetState());
	newArena->ball->_velocityImpulseCache = this->ball->_velocityImpulseCache;

	for (Car* car : this->_cars) {
		Car* newCar = newArena->AddCar(car->team, car->config);
		
		newCar->SetState(car->GetState());
		newCar->id = car->id;
		newCar->controls = car->controls;
		newCar->_velocityImpulseCache = car->_velocityImpulseCache;
	}

	assert(this->_boostPads.size() == newArena->_boostPads.size());
	for (int i = 0; i < this->_boostPads.size(); i++)
		newArena->_boostPads[i]->SetState(this->_boostPads[i]->GetState());

	newArena->tickCount = this->tickCount;
	newArena->_lastCarID = this->_lastCarID;

	return newArena;
}

Car* Arena::DeserializeNewCar(DataStreamIn& in, Team team) {
	Car* car = Car::_AllocateCar();
	car->_Deserialize(in);
	car->team = team;

	_AddCarFromPtr(car);

	car->_BulletSetup(gameMode, &_bulletWorld, _mutatorConfig);
	car->SetState(car->_internalState);

	return car;
}

void Arena::Step(int ticksToSimulate) {
	for (int i = 0; i < ticksToSimulate; i++) {

		_bulletWorld.setWorldUserInfo(this);

		{ // Ball zero-vel sleeping
			if (ball->_rigidBody.m_linearVelocity.length2() == 0 && ball->_rigidBody.m_angularVelocity.length2() == 0) {
				ball->_rigidBody.setActivationState(ISLAND_SLEEPING);
			} else {
				ball->_rigidBody.setActivationState(ACTIVE_TAG);
			}
		}

		bool ballOnly = _cars.empty();

		bool hasArenaStuff = (gameMode != GameMode::THE_VOID);
		
		for (Car* car : _cars)
			car->_PreTickUpdate(gameMode, tickTime, _mutatorConfig);

		if (hasArenaStuff && !ballOnly) {
			for (BoostPad* pad : _boostPads)
				pad->_PreTickUpdate(tickTime);
		}

		// Update ball
		ball->_PreTickUpdate(gameMode, tickTime);

		// Update world
		_bulletWorld.stepSimulation(tickTime, 0, tickTime);

		for (Car* car : _cars) {
			car->_PostTickUpdate(gameMode, tickTime, _mutatorConfig);
			car->_FinishPhysicsTick(_mutatorConfig);
			if (hasArenaStuff) {
				if (_config.useCustomBoostPads) {
					// TODO: This is quite slow, we should use a sorting method of some sort
					for (auto& boostPad : _boostPads) {
						boostPad->_CheckCollide(car);
					}
				} else {
					_boostPadGrid.CheckCollision(car);
				}
			}
		}

		if (hasArenaStuff && !ballOnly)
			for (BoostPad* pad : _boostPads)
				pad->_PostTickUpdate(tickTime, _mutatorConfig);

		ball->_FinishPhysicsTick(_mutatorConfig);

		// Sync tiles state after the tick ends.
		// We don't want to sync the state on tile damage, 
		//	because that would cause the ball to immediately fall through the newly-broken tile.
		if (gameMode == GameMode::DROPSHOT)
			if (ball->_internalState.dsInfo.lastDamageTick && ball->_internalState.dsInfo.lastDamageTick == tickCount)
				SetDropshotTilesState(_dropshotTilesState);

		if (_goalScoreCallback.func != NULL) { // Potentially fire goal score callback
			if (IsBallScored()) {
				_goalScoreCallback.func(this, RS_TEAM_FROM_Y(-ball->_rigidBody.getWorldTransform().m_origin.y()), _goalScoreCallback.userInfo);
			}
		}

		tickCount++;
	}
}

// Returns negative: within
// Note that the returned margin is squared
float BallWithinHoopsGoalXYMarginSq(float x, float y) {
	constexpr float
		SCALE_Y = 0.9f,
		OFFSET_Y = 2770.f,
		RADIUS_SQ = 716 * 716;

	float dy = abs(y) * SCALE_Y - OFFSET_Y;
	float distSq = x * x + dy * dy;
	return distSq - RADIUS_SQ;
}

bool Arena::IsBallProbablyGoingIn(float maxTime, float extraMargin, Team* goalTeamOut) const {
	Vec ballPos = ball->_rigidBody.getWorldTransform().m_origin * BT_TO_UU;
	Vec ballVel = ball->_rigidBody.m_linearVelocity * BT_TO_UU;

	if (gameMode == GameMode::SOCCAR || gameMode == GameMode::SNOWDAY) {
		if (abs(ballVel.y) < FLT_EPSILON)
			return false;

		float scoreDirSgn = RS_SGN(ballVel.y);
		float goalY = _mutatorConfig.goalBaseThresholdY * scoreDirSgn;
		float distToGoal = abs(ballPos.y - goalY);

		float timeToGoal = distToGoal / abs(ballVel.y);
		
		if (timeToGoal > maxTime)
			return false;

		Vec extrapPosWhenScore = ballPos + (ballVel * timeToGoal) + (_mutatorConfig.gravity * timeToGoal * timeToGoal) / 2;

		// From: https://github.com/RLBot/RLBot/wiki/Useful-Game-Values
		constexpr float
			APPROX_GOAL_HALF_WIDTH = 892.755f,
			APPROX_GOAL_HEIGHT = 642.775;

		float scoreMargin = _mutatorConfig.ballRadius * 0.1f + extraMargin;

		if (extrapPosWhenScore.z > APPROX_GOAL_HEIGHT + scoreMargin)
			return false; // Too high

		if (abs(extrapPosWhenScore.x) > APPROX_GOAL_HALF_WIDTH + scoreMargin)
			return false; // Too far to the side

		if (goalTeamOut)
			*goalTeamOut = RS_TEAM_FROM_Y(scoreDirSgn);

		// Ok it's probably gonna score, or at least be very close
		return true;
	} else if (gameMode == GameMode::HOOPS) {

		constexpr float
			APPROX_RIM_HEIGHT = 365;
		
		float minHeight = APPROX_RIM_HEIGHT + _mutatorConfig.ballRadius * 1.2f;

		if (ballVel.z < -FLT_EPSILON && ballPos.z < minHeight) {
			if (BallWithinHoopsGoalXYMarginSq(ballPos.x, ballPos.y) < 0) {
				if (goalTeamOut)
					*goalTeamOut = RS_TEAM_FROM_Y(ballPos.y);
				return true; // Already in the net
			}
		}

		float margin = _mutatorConfig.ballRadius * 1.0f;
		float marginSq = margin * margin;

		float upQuadIntercept;
		float downQuadIntercept;

		// Calculate time to score using quadratic intercept
		{
			float g = _mutatorConfig.gravity.z;
			if (g > -FLT_EPSILON)
				return false; 

			float v = ballVel.z;
			float h = ballPos.z - minHeight;

			float sqrtInput = v * v - 2 * g * h;
			if (sqrtInput > 0) {
				float sqrtOutput = sqrtf(sqrtInput);
				upQuadIntercept = (-v + sqrtOutput) / g;
				downQuadIntercept = (-v - sqrtOutput) / g;
			} else {
				// Never reaches the rim height
				if (BallWithinHoopsGoalXYMarginSq(ballPos.x, ballPos.y) < -marginSq) {
					// If started within the hoop, it will stay within the hoop and is therefore scoring
					return true;
				} else {
					// Otherwise, it can never get into the hoop and is therefore never scoring
					return false;
				}
			}
		}
		
		if (upQuadIntercept >= 0) {
			// Ball has to go up before it can fall into the hoop
			// Make sure it cant hit the rim on the way up

			Vec extrapPosUp = ballPos + (ballVel * upQuadIntercept);
			float upMarginSq = BallWithinHoopsGoalXYMarginSq(extrapPosUp.x, extrapPosUp.y);

			float minClearanceMargin = 60 + _mutatorConfig.ballRadius;

			if (upMarginSq > -marginSq && upMarginSq < (minClearanceMargin * minClearanceMargin))
				return false; // Will probably hit rim
		}

		Vec extrapPosDown = ballPos + (ballVel * downQuadIntercept);
		extrapPosDown.y = abs(extrapPosDown.y);

		{ // Very approximate prediction of backboard bounce
			float wallBounceY = RLConst::ARENA_EXTENT_Y_HOOPS - _mutatorConfig.ballRadius;
			if (extrapPosDown.y > wallBounceY) {
				float margin = extrapPosDown.y - wallBounceY;
				extrapPosDown.y -= margin * (1 + _mutatorConfig.ballWorldRestitution);
			}
		}
		
		if (BallWithinHoopsGoalXYMarginSq(extrapPosDown.x, extrapPosDown.y) < -marginSq) {
			if (goalTeamOut)
				*goalTeamOut = RS_TEAM_FROM_Y(extrapPosDown.y);
			return true;
		} else {
			return false;
		}

	} else {
		RS_ERR_CLOSE("Arena::IsBallProbablyGoingIn() is not supported for gamemode " << GAMEMODE_STRS[(int)gameMode]);
		return false;
	}
}

bool Arena::IsBallScored() const {
	switch (gameMode) {
	case GameMode::SOCCAR:
	case GameMode::HEATSEEKER:
	case GameMode::SNOWDAY:
	{
		float ballPosY = ball->_rigidBody.getWorldTransform().m_origin.y() * BT_TO_UU;
		return abs(ballPosY) > (_mutatorConfig.goalBaseThresholdY + _mutatorConfig.ballRadius);
	}
	case GameMode::HOOPS:
	{
		if (ball->_rigidBody.getWorldTransform().m_origin.z() < RLConst::HOOPS_GOAL_SCORE_THRESHOLD_Z * UU_TO_BT) {
			constexpr float
				SCALE_Y = 0.9f,
				OFFSET_Y = 2770.f,
				RADIUS_SQ = 716 * 716;

			Vec ballPos = ball->_rigidBody.getWorldTransform().m_origin * BT_TO_UU;
			return BallWithinHoopsGoalXYMarginSq(ballPos.x, ballPos.y) < 0;
		} else {
			return false;
		}
	}
	case GameMode::DROPSHOT:
	{
		if ((ball->_rigidBody.getWorldTransform().m_origin.z() * BT_TO_UU) < -(_mutatorConfig.ballRadius * 1.75f)) {
			return true;
		} else {
			return false;
		}
	}
	default:
		return false;
	}
}

Arena::~Arena() {

	// Remove all from bullet world constraints
	while (_bulletWorld.getNumConstraints() > 0)
		_bulletWorld.removeConstraint(0);

	// Manually remove all collision objects
	// Otherwise we run into issues regarding deconstruction order
	while (_bulletWorld.getNumCollisionObjects() > 0)
		_bulletWorld.removeCollisionObject(_bulletWorld.getCollisionObjectArray()[0]);

	// Remove all cars
	if (ownsCars) {
		for (Car* car : _cars)
			delete car;
	}

	// Remove the ball
	if (ownsBall) {
		Ball::_DestroyBall(ball);
	}

	if (_boostPads.size() > 0) {
		if (ownsBoostPads) {
			// Remove all boost pads
			for (BoostPad* boostPad : _boostPads)
				delete boostPad;
		}
	}

	// Remove all rigidbodies and collision shapes that we own
	for (auto rb : _worldCollisionRBs) {
		auto shape = rb->getCollisionShape();
		
		bool isBvh = dynamic_cast<btBvhTriangleMeshShape*>(shape);
		if (isBvh) {
			// Don't free BVH shapes because we don't own them
		} else {
			delete shape;
		}

		delete rb;
	}

	delete _bulletWorldParams.overlappingPairCache;
	delete _bulletWorldParams.broadphase;
}

btRigidBody* Arena::_AddStaticCollisionShape(btCollisionShape* shape, btVector3 posBT, int group, int mask) {
	btRigidBody* shapeRB = new btRigidBody(0, NULL, shape);
	shapeRB->setWorldTransform(btTransform(btMatrix3x3::getIdentity(), posBT));
	shapeRB->setUserPointer(this);
	if (group || mask) {
		_bulletWorld.addRigidBody(shapeRB, group, mask);
	} else {
		_bulletWorld.addRigidBody(shapeRB);
	}
	_worldCollisionRBs.push_back(shapeRB);
	return shapeRB;
}

void Arena::_SetupArenaCollisionShapes() {
	assert(gameMode != GameMode::THE_VOID);
	bool isHoops = gameMode == GameMode::HOOPS;
	bool isDropShot = gameMode == GameMode::DROPSHOT;

	auto collisionMeshes = RocketSim::GetArenaCollisionShapes(gameMode);

	if (collisionMeshes.empty()) {
		RS_ERR_CLOSE(
			"No arena meshes found for gamemode " << GAMEMODE_STRS[(int)gameMode] << ", " <<
			"the mesh files should be in " << RocketSim::_collisionMeshesFolder
		)
	}

	for (size_t i = 0; i < collisionMeshes.size(); i++) {
		auto mesh = collisionMeshes[i];

		bool isHoopsNet = false;

		if (isHoops) { // Detect net mesh and disable car collision
			const unsigned char* vertexBase;
			int numVerts, stride;
			const unsigned char* indexBase;
			int indexStride, numFaces;
			mesh->getMeshInterface()->getLockedReadOnlyVertexIndexBase(&vertexBase, numVerts, stride, &indexBase, indexStride, numFaces);
			
			constexpr int HOOPS_NET_NUM_VERTS = 505;
			if (numVerts == HOOPS_NET_NUM_VERTS) {
				isHoopsNet = true;
			}
		}

		int mask = isHoopsNet ? CollisionMasks::HOOPS_NET : 0;
		_worldCollisionBvhShapes.push_back(mesh);
		_AddStaticCollisionShape(mesh, btVector3(0, 0, 0), mask, mask);

		// Don't free the BVH when we deconstruct this arena
		mesh->m_ownsBvh = false;
	}

	{ // Add arena collision planes (floor/walls/ceiling)
		using namespace RLConst;

		float 
			extentX = isHoops ? ARENA_EXTENT_X_HOOPS : ARENA_EXTENT_X,
			extentY = isHoops ? ARENA_EXTENT_Y_HOOPS : ARENA_EXTENT_Y,
			height  = isDropShot ? ARENA_HEIGHT_DROPSHOT : (isHoops ? ARENA_HEIGHT_HOOPS : ARENA_HEIGHT);

		auto fnAddPlane = [&](Vec posUU, Vec normal, int mask = 0) {
			assert(normal.Length() == 1);
			auto planeShape = new btStaticPlaneShape(normal, 0);

			_worldCollisionPlaneShapes.push_back(planeShape);
			_AddStaticCollisionShape(
				planeShape,
				posUU * UU_TO_BT,
				mask, mask
			);
		};

		// Floor
		fnAddPlane(Vec(0, 0, isDropShot ? RLConst::FLOOR_HEIGHT_DROPSHOT : 0), Vec(0, 0, 1), isDropShot ? CollisionMasks::DROPSHOT_FLOOR : 0);

		// Ceiling
		fnAddPlane(Vec(0, 0, height), Vec(0, 0, -1), 0);

		if (!isDropShot) {
			// Side walls
			fnAddPlane(Vec(-extentX, 0, height / 2), btVector3( 1, 0, 0));
			fnAddPlane(Vec(extentX, 0, height / 2),  btVector3(-1, 0, 0));
		}
		

		if (isHoops) {
			// Y walls
			fnAddPlane(Vec(0, -extentY, height / 2), btVector3(0,  1, 0));
			fnAddPlane(Vec(0, extentY, height / 2),  btVector3(0, -1, 0));
		}
	}

	if (isDropShot) {
		// Add tiles
		auto tileShapes = DropshotTiles::MakeTileShapes();
		for (int i = 0; i < tileShapes.size(); i++) {
			int teamIdx = i / RLConst::Dropshot::NUM_TILES_PER_TEAM;
			int tileIdx = i % RLConst::Dropshot::NUM_TILES_PER_TEAM;

			// Shift down so the collision doesn't peek through the floor
			Vec pos = Vec(0, 0, -tileShapes[i]->getMargin());

			auto tileRB = _AddStaticCollisionShape(tileShapes[i], pos, DROPSHOT_TILE, DROPSHOT_TILE);
			tileRB->setUserIndex(BT_USERINFO_TYPE_DROPSHOT_TILE);
			tileRB->setUserIndex2(i);
			_worldDropshotTileRBs.push_back(tileRB);
		}
	}
}

RS_NS_END
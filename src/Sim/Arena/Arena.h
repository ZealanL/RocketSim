#pragma once
#include "../../BaseInc.h"
#include "../Car/Car.h"
#include "../Ball/Ball.h"
#include "../BoostPad/BoostPad.h"
#include "../CollisionMasks.h"

#include "../../CollisionMeshFile/CollisionMeshFile.h"
#include "../BoostPad/BoostPadGrid/BoostPadGrid.h"
#include "../SuspensionCollisionGrid/SuspensionCollisionGrid.h"
#include "../MutatorConfig/MutatorConfig.h"
#include "ArenaConfig/ArenaConfig.h"

#include "../../../libsrc/bullet3-3.24/BulletCollision/BroadphaseCollision/btDbvtBroadphase.h"
#include "../../../libsrc/bullet3-3.24/BulletCollision/CollisionShapes/btStaticPlaneShape.h"
#include "../../../libsrc/bullet3-3.24/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "../../../libsrc/bullet3-3.24/BulletCollision/CollisionDispatch/btCollisionDispatcher.h"
#include "../../../libsrc/bullet3-3.24/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h"
#include "../../../libsrc/bullet3-3.24/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "../../../libsrc/bullet3-3.24/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h"

RS_NS_START

typedef std::function<void(class Arena* arena, Team scoringTeam, void* userInfo)> GoalScoreEventFn;
typedef std::function<void(class Arena* arena, Car* bumper, Car* victim, bool isDemo, void* userInfo)> CarBumpEventFn;

// The container for all game simulation
// Stores cars, the ball, all arena collisions, and manages the overall game state
class Arena {
public:

	GameMode gameMode;

	uint32_t _lastCarID = 0;
	std::unordered_set<Car*> _cars;
	bool ownsCars = true; // If true, deleting this arena instance deletes all cars

	std::unordered_map<uint32_t, Car*> _carIDMap;
	
	Ball* ball;
	bool ownsBall = true; // If true, deleting this arena instance deletes the ball
	
	std::vector<BoostPad*> _boostPads;
	bool ownsBoostPads = true; // If true, deleing this arena instance deletes all boost pads
	
	BoostPadGrid _boostPadGrid;

	SuspensionCollisionGrid _suspColGrid;

	MutatorConfig _mutatorConfig;

	const MutatorConfig& GetMutatorConfig() { return _mutatorConfig; }
	RSAPI void SetMutatorConfig(const MutatorConfig& mutatorConfig);

	// Time in seconds each tick (1/tickrate)
	float tickTime; 

	// Returns (1 / tickTime)
	float GetTickRate() const {
		return 1 / tickTime;
	}

	// Total ticks this arena instance has been simulated for, never resets
	uint64_t tickCount = 0;

	const std::unordered_set<Car*>& GetCars() { return _cars; }
	const std::vector<BoostPad*>& GetBoostPads() { return _boostPads; }

	// Returns true if added, false if car was already added
	bool _AddCarFromPtr(Car* car);
	RSAPI Car* AddCar(Team team, const CarConfig& config = CAR_CONFIG_OCTANE);

	// Returns false if the car ID was not found in the cars list
	RSAPI bool RemoveCar(uint32_t id);

	// Returns false if the car was not found in the cars list
	// NOTE: If the car was removed, the car will be freed and the pointer will be made invalid
	bool RemoveCar(Car* car) {
		return RemoveCar(car->id);
	}

	RSAPI Car* GetCar(uint32_t id);

	btDiscreteDynamicsWorld _bulletWorld;
	struct {
		btDefaultCollisionConfiguration collisionConfig;
		btCollisionDispatcher collisionDispatcher;
		btOverlappingPairCache* overlappingPairCache;
		btBroadphaseInterface* broadphase;
		btSequentialImpulseConstraintSolver constraintSolver;
	} _bulletWorldParams;

	btRigidBody* _worldCollisionRBs = NULL;
	size_t _worldCollisionRBAmount = 0;
	btBvhTriangleMeshShape* _worldCollisionBvhShapes = NULL;
	btStaticPlaneShape* _worldCollisionPlaneShapes = NULL;

	struct {
		GoalScoreEventFn func = NULL;
		void* userInfo = NULL;
	} _goalScoreCallback;
	RSAPI void SetGoalScoreCallback(GoalScoreEventFn callbackFn, void* userInfo = NULL);

	struct {
		CarBumpEventFn func = NULL;
		void* userInfo = NULL;
	} _carBumpCallback;
	RSAPI void SetCarBumpCallback(CarBumpEventFn callbackFn, void* userInfo = NULL);

	// NOTE: Arena should be destroyed after use
	RSAPI static Arena* Create(GameMode gameMode, const ArenaConfig& arenaConfig = {}, float tickRate = 120);
	
	// Serialize entire arena state including cars, ball, and boostpads
	RSAPI void Serialize(DataStreamOut& out) const;

	// Load new arena from serialized data
	RSAPI static Arena* DeserializeNew(DataStreamIn& in);

	Arena(const Arena& other) = delete; // No copy constructor, use Arena::Clone() instead
	Arena& operator =(const Arena& other) = delete; // No copy operator, use Arena::Clone() instead

	Arena(Arena&& other) = delete; // No move constructor
	Arena& operator =(Arena&& other) = delete; // No move operator

	// Get a deep copy of the arena
	RSAPI Arena* Clone(bool copyCallbacks);

	// NOTE: Car ID will not be restored
	RSAPI Car* DeserializeNewCar(DataStreamIn& in, Team team);

	// Simulate everything in the arena for a given number of ticks
	RSAPI void Step(int ticksToSimulate = 1);

	RSAPI void ResetToRandomKickoff(int seed = -1);

	// Returns true if the ball is probably going in, does not account for wall or ceiling bounces
	// NOTE: Purposefully overestimates, just like the real RL's shot prediction
	// To check which goal it will score in, use the ball's velocity
	// Margin can be manually adjusted with extraMargin (negative to prevent overestimating)
	RSAPI bool IsBallProbablyGoingIn(float maxTime = 2.f, float extraMargin = 0, Team* goalTeamOut = NULL) const;

	// Returns true if the ball is in the net
	// Works for all gamemodes (and does nothing in THE_VOID)
	RSAPI bool IsBallScored() const;

	// Free all associated memory
	RSAPI ~Arena();

	// NOTE: Passed shape pointer will be freed when arena is deconstructed
	template <class T>
	btRigidBody* _AddStaticCollisionShape(
		size_t rbIndex, size_t meshListIndex, T* shape, T* meshList, btVector3 posBT = btVector3(0, 0, 0), 
		bool isHoopsNet = false) {

		static_assert(std::is_base_of<btCollisionShape, T>::value);
		meshList[meshListIndex] = *shape;

		assert(rbIndex < _worldCollisionRBAmount);
		btRigidBody& shapeRB = _worldCollisionRBs[rbIndex];
		shapeRB = btRigidBody(0, NULL, &meshList[meshListIndex]);
		shapeRB.setWorldTransform(btTransform(btMatrix3x3::getIdentity(), posBT));
		shapeRB.setUserPointer(this);
		if (isHoopsNet) {
			_bulletWorld.addRigidBody(&shapeRB, CollisionMasks::HOOPS_NET, CollisionMasks::HOOPS_NET);
		} else {
			_bulletWorld.addRigidBody(&shapeRB);
		}
		return &shapeRB;
	}

	void _SetupArenaCollisionShapes();

	// Static function called by Bullet internally when adding a collision point
	static bool _BulletContactAddedCallback(
		btManifoldPoint& cp,
		const btCollisionObjectWrapper* colObjA, int partID_A, int indexA,
		const btCollisionObjectWrapper* colObjB, int partID_B, int indexB
	);

	void _BtCallback_OnCarBallCollision(Car* car, Ball* ball, btManifoldPoint& manifoldPoint, bool ballIsBodyA);
	void _BtCallback_OnCarCarCollision(Car* car1, Car* car2, btManifoldPoint& manifoldPoint);
	void _BtCallback_OnCarWorldCollision(Car* car, btCollisionObject* worldObject, btManifoldPoint& manifoldPoint);

	const ArenaConfig& GetArenaConfig() const {
		return _config;
	}

	// Backwards compatability
	ArenaMemWeightMode GetMemWeightMode() {
		return _config.memWeightMode;
	}

private:
	
	// Constructor for use by Arena::Create()
	Arena(GameMode gameMode, const ArenaConfig& config, float tickRate = 120);

	// Making this private because horrible memory overflows can happen if you changed it
	ArenaConfig _config;
};

RS_NS_END
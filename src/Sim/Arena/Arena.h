#pragma once
#include "../../BaseInc.h"
#include "../Car/Car.h"
#include "../Ball/Ball.h"
#include "../MeshLoader/MeshLoader.h"

enum : int {
	BT_USERINFO_TYPE_CAR,
	BT_USERINFO_TYPE_BALL,
};

enum class GameMode {
	SOCCAR,
	// More coming soon!
};

// The container for all game simulation
// Stores cars, the ball, all arena collisions, and manages the overall game state
class Arena {
public:

	GameMode gameMode;

	uint32_t _lastCarID = 0;
	list<Car*> _carsList; // Using list so that elements do not change address
	Ball* ball;

	float tickTime; // Time each tick (1/tickrate)
	float GetTickRate() {
		return 1 / tickTime;
	}

	// Total ticks this arena instance has been simulated for, never resets
	uint64_t tickCount = 0;

	RSAPI const list<Car*>& GetCars() { return _carsList; }

	RSAPI Car* AddCar(Team team, const CarConfig& config = CAR_CONFIG_OCTANE);

	// Returns false if the car was not found in the cars list
	// NOTE: If the car was removed, the car will be freed and the pointer will be made invalid
	RSAPI bool RemoveCar(Car* car);

	RSAPI Car* GetCarFromID(uint32_t id);

	btDiscreteDynamicsWorld* _bulletWorld;
	struct {
		btCollisionConfiguration* collisionConfig;
		btCollisionDispatcher* collisionDispatcher;
		btDbvtBroadphase* overlappingPairCache;
		btSequentialImpulseConstraintSolver* constraintSolver;
	} _bulletWorldParams;

	vector<btRigidBody*> _worldCollisionRBs;
	vector<btCollisionShape*> _worldCollisionShapes;
	vector<btTriangleMesh*> _arenaTriMeshes;

	typedef std::function<void(Team goalTeam)> GoalScoreEventFn;
	vector<GoalScoreEventFn> _goalScoreCallbacks;
	void RegisterGoalScoreCallback(GoalScoreEventFn callbackFunc);

	RSAPI Arena(GameMode gameMode, float tickRate = 120);

	// Simulate everything in the arena for a given number of ticks
	RSAPI void Step(int ticksToSimulate = 1);

	// Free all associated memory
	RSAPI ~Arena();

	// NOTE: Passed shape pointer will be freed when arena is deconstructed
	void _AddStaticCollisionShape(btCollisionShape* shape, btVector3 pos = btVector3(0,0,0));

	void _AddStaticCollisionTris(MeshLoader::Mesh& mesh, btVector3 scale = btVector3(1,1,1), btVector3 pos = btVector3(0, 0, 0));
	void _SetupArenaCollisionShapes();

	// Static function called by Bullet internally during a tick
	static void _BulletInternalTickCallback(btDynamicsWorld* world, btScalar step);

	void _BtCallback_OnCarBallCollision(Car* car, Ball* ball, btManifoldPoint& manifoldPoint);
	void _BtCallback_OnCarCarCollision(Car* car1, Car* car2, btManifoldPoint& manifoldPoint);
};
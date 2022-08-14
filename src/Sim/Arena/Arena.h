#pragma once
#include "../../BaseInc.h"
#include "../Car/Car.h"
#include "../Ball/Ball.h"

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

	const list<Car*>& GetCars() { return _carsList; }

	Car* AddCar(Team team, const CarConfig& config = CAR_CONFIG_OCTANE);

	// Returns false if the car was not found in the cars list
	// NOTE: If the car was removed, the car will be freed and the pointer will be made invalid
	bool RemoveCar(Car* car); 

	Car* GetCarFromID(uint32_t id);

	btDiscreteDynamicsWorld* _bulletWorld;
	struct {
		btCollisionConfiguration* collisionConfig;
		btCollisionDispatcher* collisionDispatcher;
		btDbvtBroadphase* overlappingPairCache;
		btSequentialImpulseConstraintSolver* constraintSolver;
	} _bulletWorldParams;

	vector<btRigidBody*> _worldCollisionRBs;
	vector<btCollisionShape*> _worldCollisionShapes;

	typedef std::function<void(Team goalTeam)> GoalScoreEventFn;
	vector<GoalScoreEventFn> _goalScoreCallbacks;
	void RegisterGoalScoreCallback(GoalScoreEventFn callbackFunc);

	Arena(GameMode gameMode);

	// Simulate everything in the arena for a given number of ticks
	void Step(int ticksToSimulate = 1);

	// Free all associated memory
	~Arena();
};
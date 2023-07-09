#pragma once
#include "../../src/RocketSim.h"

#ifdef RS_PYBIND

template <typename FuncType>
struct PyArenaCallback {
	bool isValid;
	FuncType func = NULL;
	void* userInfo;

	std::shared_ptr<struct ArenaWrapper> arenaWrapper;

	PyArenaCallback() {
		arenaWrapper = NULL;
		isValid = false;
	}

	PyArenaCallback(std::shared_ptr<struct ArenaWrapper> arenaWrapper, FuncType func, void* userInfo) : arenaWrapper(arenaWrapper), func(func), userInfo(userInfo) {
		isValid = true;
	}

	template <typename ...Args>
	void Call(Args... args) {
		if (isValid) {
			func(arenaWrapper, args...);
		}
	}

	operator bool() const {
		return isValid;
	}
};

struct ArenaWrapper {
	typedef std::function<void(std::shared_ptr<ArenaWrapper>, Team)> CallbackFn_Goal;
	typedef std::function<void(std::shared_ptr<ArenaWrapper>, std::shared_ptr<Car>, std::shared_ptr<Car>, bool)> CallbackFn_Bump;

	std::shared_ptr<Arena> arena;

	std::shared_ptr<Ball> ball;
	std::unordered_map<uint32_t, std::shared_ptr<Car>> cars;
	std::vector<std::shared_ptr<BoostPad>> boostPads;

	PyArenaCallback<CallbackFn_Goal> cb_goal = {};
	PyArenaCallback<CallbackFn_Bump> cb_bump = {};

	ArenaWrapper() {
		arena = NULL;
	}

	ArenaWrapper(DataStreamIn& streamIn) : ArenaWrapper(Arena::DeserializeNew(streamIn)) {}

	ArenaWrapper(GameMode gameMode, float tickRate) : ArenaWrapper(Arena::Create(gameMode, tickRate)) {}

	ArenaWrapper(Arena* arena) {
		this->arena = std::shared_ptr<Arena>(arena);
		arena->ownsBall = false;
		arena->ownsCars = false;
		arena->ownsBoostPads = false;

		ball = std::shared_ptr<Ball>(arena->ball);

		for (Car* car : arena->_cars)
			cars[car->id] = std::shared_ptr<Car>(car);

		for (BoostPad* pad : arena->_boostPads)
			boostPads.push_back(std::shared_ptr<BoostPad>(pad));
	}

	ArenaWrapper(const ArenaWrapper& other) = delete;
	ArenaWrapper& operator=(const ArenaWrapper& other) = delete;

	std::shared_ptr<Car> AddCar(Team team, CarConfig config) {
		Car* car = arena->AddCar(team, config);
		std::shared_ptr<Car> carPtr = std::shared_ptr<Car>(car);
		cars[car->id] = carPtr;
		return carPtr;
	}

	std::shared_ptr<Car> GetCar(uint32_t id) {
		return cars[id];
	}

	float GetTickRate() {
		return arena->GetTickRate();
	}

	float GetTickTime() {
		return arena->tickTime;
	}

	void Step(int ticks) {
		arena->Step(ticks);
	}

	bool RemoveCar(uint32_t id) {
		bool result = arena->RemoveCar(id);
		cars.erase(id);
		return result;
	}

	bool RemoveCar(std::shared_ptr<Car> car) {
		return RemoveCar(car->id);
	}

	MutatorConfig GetMutatorConfig() {
		return arena->GetMutatorConfig();
	}

	void SetMutatorConfig(const MutatorConfig& config) {
		return arena->SetMutatorConfig(config);
	}

	GameMode GetGameMode() {
		return arena->gameMode;
	}

	void Serialize(DataStreamOut& out) {
		arena->Serialize(out);
	}

	std::shared_ptr<Car> DeserializeNewCar(DataStreamIn& in, Team team) {
		Car* car = arena->DeserializeNewCar(in, team);
		std::shared_ptr<Car> carPtr = std::shared_ptr<Car>(car);
		cars[car->id] = carPtr;
		return carPtr;
	}

	static void SetCB_Goal(std::shared_ptr<ArenaWrapper> wrapper, CallbackFn_Goal fn, void* userInfo) {
		wrapper->cb_goal = { wrapper, fn, userInfo };
		wrapper->arena->SetGoalScoreCallback(ArenaWrapper::CB_Goal, &wrapper->cb_goal);
	}

	static void SetCB_Bump(std::shared_ptr<ArenaWrapper> wrapper, CallbackFn_Bump fn, void* userInfo) {
		wrapper->cb_bump = { wrapper, fn, userInfo };
		wrapper->arena->SetCarBumpCallback(ArenaWrapper::CB_Bump, &wrapper->cb_bump);
	}

	static void CB_Goal(Arena* arena, Team scoringTeam, void* userInfo) {
		auto* cbInst = (PyArenaCallback<CallbackFn_Goal>*)userInfo;
		std::shared_ptr<ArenaWrapper> wrapper = cbInst->arenaWrapper;
		cbInst->Call(scoringTeam);
	}

	static void CB_Bump(Arena* arena, Car* bumper, Car* victim, bool isDemo, void* userInfo) {
		auto* cbInst = (PyArenaCallback<CallbackFn_Bump>*)userInfo;
		std::shared_ptr<ArenaWrapper> wrapper = cbInst->arenaWrapper;
		cbInst->Call(wrapper->GetCar(bumper->id), wrapper->GetCar(victim->id), isDemo);
	}

	~ArenaWrapper() {
		if (arena) {
			arena->SetGoalScoreCallback(NULL);
			arena->SetCarBumpCallback(NULL);
		}
	}
};
#endif
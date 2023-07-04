#pragma once
#include "../../src/RocketSim.h"

struct ArenaWrapper {
	std::shared_ptr<Arena> arena;

	std::shared_ptr<Ball> ball;
	std::unordered_map<uint32_t, std::shared_ptr<Car>> cars;
	std::vector<std::shared_ptr<BoostPad>> boostPads;

	ArenaWrapper() {
		arena = NULL;
	}

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
};
#ifdef RS_PYBIND
#include "PYB.h"

PYB_INIT_F(Arena) {
#define PYB_CUR_CLASS Arena
	PYB_CLASS(Arena)
		.def_static("create", &Arena::Create, PYBA("gamemode"), PYBA("tickrate") = float(120.f), pyb::return_value_policy::take_ownership)
		.def("clone", &Arena::Clone, pyb::return_value_policy::take_ownership)

		.def("get_cars",
			[](const Arena& arena) {
				auto cars = vector<CarWrapper>();
				for (Car* car : arena._cars) {
					cars.push_back(CarWrapper(car));
				}
				return pyb::cast(cars);
			}
		)

		.def("get_boostpads",
			[](const Arena& arena) {
				// Just do a straight cast
				return pyb::cast(arena._boostPads);
			}
		)

		.def("add_car", [](Arena& arena, Team team, const CarConfig& config) { return CarWrapper(arena.AddCar(team, config)); }, PYBA("team"), PYBA("config") = CarConfig(CAR_CONFIG_OCTANE))

		.def("remove_car", [](Arena& arena, const CarWrapper& carWrap) { return arena.RemoveCar(carWrap.ptr); }, PYBA("car"))
		.def("remove_car", [](Arena& arena, uint32_t id) { return arena.RemoveCar(id); }, PYBA("car_id"))

		.def("reset_to_random_kickoff", &Arena::ResetToRandomKickoff, PYBA("seed") = int(-1))
		.def("get_tickrate", &Arena::GetTickRate)
		.def("get_last_car_id", [](Arena& arena) { return arena._lastCarID; })

		.def("get_car", [](Arena& arena, uint32_t id) { return CarWrapper(arena.GetCar(id)); }, PYBA("id"))


		.def("step", &Arena::Step, PYBA("ticks"))

		.def("get_mutator_config", &Arena::GetMutatorConfig)
		.def("set_mutator_config", &Arena::SetMutatorConfig)

		.def("get_ball", [](Arena& arena) { return BallWrapper(arena.ball); })


		PYBP(gameMode)
		PYBP(tickCount)
		PYBP(tickTime)
		;
}
#endif
#ifdef RS_PYBIND
#include "PYB.h"

PYB_INIT_F(Arena) {
#define PYB_CUR_CLASS Arena
	PYB_CLASS()
		.def_static("create", &Arena::Create, PYBA("gamemode"), PYBA("tickrate") = float(120.f), pyb::return_value_policy::take_ownership)
		.def("clone", &Arena::Clone, pyb::return_value_policy::take_ownership)

		.def("get_cars",
			[](const Arena& arena) {
				auto cars = vector<Car*>(arena._cars.begin(), arena._cars.end());
				return pyb::cast(cars);
			}
		)

		.def("get_boostpads",
			[](const Arena& arena) {
				// Just do a straight cast
				return pyb::cast(arena._boostPads);
				return pyb::cast(arena._boostPads);
			}
		)

		.def("add_car", &Arena::AddCar, PYBA("team"), PYBA("config") = CarConfig(CAR_CONFIG_OCTANE), pyb::return_value_policy::reference)
		.def("remove_car", [](Arena& arena, Car* car) { return arena.RemoveCar(car); }, PYBA("car"))
		.def("remove_car", [](Arena& arena, uint32_t id) { return arena.RemoveCar(id); }, PYBA("car_id"))

		.def("reset_to_random_kickoff", &Arena::ResetToRandomKickoff, PYBA("seed") = int(-1))
		.def("get_tickrate", &Arena::GetTickRate)
		.def("get_last_car_id", &Arena::_lastCarID)
		.def("get_car", &Arena::GetCar, PYBA("id"), pyb::return_value_policy::reference)

		.def("step", &Arena::Step, PYBA("ticks"))

		.def("get_mutator_config", &Arena::GetMutatorConfig)
		.def("set_mutator_config", &Arena::SetMutatorConfig)

		PYBP(ball)
		PYBP(gameMode)
		PYBP(tickCount)
		PYBP(tickTime)
		;

#define PYB_CUR_CLASS Ball
	PYB_CLASS()
		.def("get_radius", &Ball::GetRadius)
		.def("get_state", &Ball::GetState)
		.def("set_state", &Ball::SetState)

		.def("get_rot", [](const Ball& ball) { return (RotMat)(ball._rigidBody.m_worldTransform.m_basis);  })
		;
}
#endif
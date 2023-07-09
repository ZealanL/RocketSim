#ifdef RS_PYBIND
#include "PYB.h"

PYB_INIT_F(Arena) {
#define PYB_CUR_CLASS ArenaWrapper
	pyb::class_<ArenaWrapper, std::shared_ptr<ArenaWrapper>>(m, "Arena")
		.def(pyb::init<GameMode, float>(), PYBA("gamemode"), PYBA("tickrate") = float(120.f))

		.def("clone", 
			[](const ArenaWrapper& arena, bool copyCallbacks) -> std::shared_ptr<ArenaWrapper> {
				return std::shared_ptr<ArenaWrapper>(
					new ArenaWrapper(
						arena.arena->Clone(copyCallbacks)
					)
				);
			},
			PYBA("copy_callbacks") = bool(false),
			pyb::return_value_policy::take_ownership
		)

		.def("set_goal_scored_callback",
			[](std::shared_ptr<ArenaWrapper> arena, ArenaWrapper::CallbackFn_Goal fn) {
				ArenaWrapper::SetCB_Goal(arena, fn, NULL);
			},
			PYBA("callback_func")
		)

		.def("set_bump_callback",
			[](std::shared_ptr<ArenaWrapper> arena, ArenaWrapper::CallbackFn_Bump fn) {
				ArenaWrapper::SetCB_Bump(arena, fn, NULL);
			},
			PYBA("callback_func")
		)

		.def("get_cars",
			[](const ArenaWrapper& arena) {
				auto cars = std::vector<std::shared_ptr<Car>>();
				for (auto pair : arena.cars) {
					cars.push_back(pair.second);
				}
				return pyb::cast(cars);
			}
		)

		.def("get_boostpads",
			[](const ArenaWrapper& arena) {
				// Just do a straight cast
				return pyb::cast(arena.boostPads);
			}
		)

		.def("add_car", &ArenaWrapper::AddCar, PYBA("team"), PYBA("config") = CarConfig(CAR_CONFIG_OCTANE))
		.def("remove_car",
			[](ArenaWrapper& arena, std::shared_ptr<Car> car) {
				return arena.RemoveCar(car);
			}, PYBA("car")
		)

		.def("remove_car",
			[](ArenaWrapper& arena, uint32_t id) {
				return arena.RemoveCar(id);
			}, PYBA("id"))

		.def("get_car", &ArenaWrapper::GetCar, PYBA("id"))
		PYBP(ball)
		.def("get_tickrate", &ArenaWrapper::GetTickRate)
		.def("get_ticktime", &ArenaWrapper::GetTickTime)

		.def("reset_to_random_kickoff", 
			[](const ArenaWrapper& arena, int seed) {
				arena.arena->ResetToRandomKickoff(seed);
			}, PYBA("seed") = int(-1))

		.def("get_last_car_id", [](ArenaWrapper& arena) { return arena.arena->_lastCarID; })
		.def("get_tickcount", [](ArenaWrapper& arena) { return arena.arena->tickCount; })

		.def("step", &ArenaWrapper::Step, PYBA("ticks"))

		.def("get_mutator_config", &ArenaWrapper::GetMutatorConfig)
		.def("set_mutator_config", &ArenaWrapper::SetMutatorConfig, PYBA("config"))
		.def("get_gamemode", &ArenaWrapper::GetGameMode)

		.def("serialize", &ArenaWrapper::Serialize, PYBA("stream_out"))
		.def_static("deserialize_new", 
			[](DataStreamIn& streamIn) {
				return std::shared_ptr<ArenaWrapper>(new ArenaWrapper(streamIn));
			},
			PYBA("stream_in")
		)

		.def("deserialize_new_car", &ArenaWrapper::DeserializeNewCar, PYBA("stream_in"), PYBA("team"))

		.def("serialize_to_file", 
			[](ArenaWrapper& arena, std::wstring path) {
				DataStreamOut streamOut = {};
				arena.Serialize(streamOut);
				streamOut.WriteToFile(path, true);
			}, 
			PYBA("path")
		)

		.def_static("deserialize_new_from_file",
			[](std::wstring path) {
				DataStreamIn streamIn = DataStreamIn(path, true);
				return std::shared_ptr<ArenaWrapper>(new ArenaWrapper(streamIn));
			},
			PYBA("path")
		)
		;
}
#endif
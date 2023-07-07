#ifdef RS_PYBIND
#include "PYB.h"

PYBIND11_MODULE(rocketsim, m) {
	m.def("init", [](const wstring& pathStr) { RocketSim::Init(std::filesystem::path(pathStr)); }, PYBA("arena_meshes_path"));
	m.def("is_in_init", []() { return RocketSim::GetStage() == RocketSimStage::INITIALIZING; });
	m.def("is_ready", []() { return RocketSim::GetStage() == RocketSimStage::INITIALIZED; });

	try {
		PYB_INIT_Ball(m);
		PYB_INIT_BallHitInfo(m);
		PYB_INIT_BoostPad(m);
		PYB_INIT_Car(m);
		PYB_INIT_Math(m);
		PYB_INIT_MathTypes(m);
		PYB_INIT_Misc(m);
		PYB_INIT_MutatorConfig(m);

		// Arena is last since it depends on other things
		PYB_INIT_Arena(m);

	} catch (std::exception e) {
		RS_ERR_CLOSE("Failed to initialize pybind11 module: Got exception: " << e.what());
	}
}
#endif
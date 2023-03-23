#include "RocketSim.h"

static std::mutex beginInitMutex;

static RocketSimStage stage = RocketSimStage::UNINITIALIZED;
RocketSimStage RocketSim::GetStage() {
	return stage;
}

static vector<btBvhTriangleMeshShape*> arenaCollisionMeshes;
const vector<btBvhTriangleMeshShape*>& RocketSim::GetArenaCollisionShapes() {
	return arenaCollisionMeshes;
}

#ifndef RS_NO_SUSPCOLGRID
static SuspensionCollisionGrid suspColGrid;
const SuspensionCollisionGrid& RocketSim::GetDefaultSuspColGrid() {
	return suspColGrid;
}
#endif

void RocketSim::Init() {

	constexpr char MSG_PREFIX[] = "RocketSim::Init(): ";

	beginInitMutex.lock();
	{
		if (stage != RocketSimStage::UNINITIALIZED) {
			RS_LOG("WARNING: RocketSim::Init() called again after already initialized, ignoring...");
			beginInitMutex.unlock();
			return;
		}

		RS_LOG("Initializing RocketSim version " RS_VERSION ", created by ZealanL...");

		stage = RocketSimStage::INITIALIZING;

		uint64_t startMS = RS_CUR_MS();

		{ // Load collision meshes
			string basePath = COLLISION_MESH_SOCCAR_PATH;
			RS_LOG("Loading arena meshes from \"" << basePath << "\"...");

			if (!std::filesystem::exists(basePath)) {
				RS_ERR_CLOSE(
					"Failed to find arena collision mesh files at \"" << basePath
					<< "\", the collision meshes folder should be in our current directory " << std::filesystem::current_path() << ".")
			}
			// Load collision meshes
			auto dirItr = std::filesystem::directory_iterator(basePath);
			for (auto& entry : dirItr) {
				auto entryPath = entry.path();
				if (entryPath.has_extension() && entryPath.extension() == COLLISION_MESH_FILE_EXTENSION) {
					CollisionMeshFile meshFile = {};
					meshFile.ReadFromFile(entryPath.string());

					btTriangleMesh* triMesh = meshFile.MakeBulletMesh();
					auto bvtMesh = new btBvhTriangleMeshShape(triMesh, false);
					bvtMesh->buildOptimizedBvh();

					arenaCollisionMeshes.push_back(bvtMesh);
				}
			}

			if (arenaCollisionMeshes.empty()) {
				RS_ERR_CLOSE(MSG_PREFIX <<
					"Failed to find soccar field asset files at \"" << basePath
					<< "\", the folder exists but has no collision mesh files.")
			}

			RS_LOG(MSG_PREFIX << "Finished loading " << arenaCollisionMeshes.size() << " arena collision meshes.");
		}

#ifndef RS_NO_SUSPCOLGRID
		{ // Set up suspension collision grid
			RS_LOG("Building collision suspension grid from arena meshes...");

			suspColGrid.Allocate();
			suspColGrid.SetupWorldCollision(arenaCollisionMeshes);
		}
#endif

		uint64_t elapsedMS = RS_CUR_MS() - startMS;
		RS_LOG("Finished initializing RocketSim in " << (elapsedMS / 1000.f) << "s!");

		stage = RocketSimStage::INITIALIZED;
	}
	beginInitMutex.unlock();
}

void RocketSim::AssertInitialized(const char* errorMsgPrefix) {
	if (stage != RocketSimStage::INITIALIZED) {
		RS_ERR_CLOSE(errorMsgPrefix << "RocketSim has not been initialized, call RocketSim::Init() first.")
	}
}
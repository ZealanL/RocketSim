#include "RocketSim.h"

#ifdef RS_PYBIND
// Make sure it gets compiled
#include "../python/src/PYB.h"
#endif

#include "../libsrc/bullet3-3.24/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "../libsrc/bullet3-3.24/BulletCollision/CollisionShapes/btTriangleMesh.h"
#include "../libsrc/bullet3-3.24/BulletCollision/CollisionDispatch/btInternalEdgeUtility.h"

constexpr uint32_t SOCCAR_ARENA_MESH_HASHES[] = {
	0xA160BAF9, 0x2811EEE8, 0xB81AC8B9, 0x760358D3,
	0x73AE4940, 0x918F4A4E, 0x1F8EE550, 0x255BA8C1,
	0x14B84668, 0xEC759EBF, 0x94FB0D5C, 0xDEA07102,
	0xBD4FBEA8,	0x39A47F63, 0x3D79D25D, 0xD84C7A68
};

static std::mutex beginInitMutex;

static RocketSimStage stage = RocketSimStage::UNINITIALIZED;
RocketSimStage RocketSim::GetStage() {
	return stage;
}

static std::vector<btBvhTriangleMeshShape*> arenaCollisionMeshes;
const std::vector<btBvhTriangleMeshShape*>& RocketSim::GetArenaCollisionShapes() {
	return arenaCollisionMeshes;
}

#ifndef RS_NO_SUSPCOLGRID
static SuspensionCollisionGrid suspColGrid;
const SuspensionCollisionGrid& RocketSim::GetDefaultSuspColGrid() {
	return suspColGrid;
}
#endif

void RocketSim::Init(std::filesystem::path collisionMeshesFolder) {

	constexpr char MSG_PREFIX[] = "RocketSim::Init(): ";

	beginInitMutex.lock();
	{
		if (stage != RocketSimStage::UNINITIALIZED) {
			RS_WARN("RocketSim::Init() called again after already initialized, ignoring...");
			beginInitMutex.unlock();
			return;
		}

		RS_LOG("Initializing RocketSim version " RS_VERSION ", created by ZealanL...");

		stage = RocketSimStage::INITIALIZING;

		uint64_t startMS = RS_CUR_MS();

		{ // Load collision meshes

			std::filesystem::path basePath = collisionMeshesFolder;
			std::filesystem::path soccarMeshesFolder = basePath / "soccar";

			RS_LOG("Loading arena meshes from " << soccarMeshesFolder << "...");

			if (!std::filesystem::exists(soccarMeshesFolder)) {
				RS_ERR_CLOSE(
					"Failed to find arena collision mesh files at " << soccarMeshesFolder
					<< ", the collision meshes folder should be in our current directory " << std::filesystem::current_path() << ".")
			}

			// How many of each collision mesh hash we have loaded
			// There should be 1 for each of the SOCCAR_ARENA_MESH_HASHES
			std::unordered_map<uint32_t, int> hashCounts;
			std::unordered_set<uint32_t> targetHashes;
			for (uint32_t targetHash : SOCCAR_ARENA_MESH_HASHES)
				targetHashes.insert(targetHash);

#ifdef RS_MERGE_ARENA_MESHES
			btTriangleMesh* masterTriMesh = new btTriangleMesh();
#endif

			// Load collision meshes
			auto dirItr = std::filesystem::directory_iterator(soccarMeshesFolder);
			for (auto& entry : dirItr) {
				auto entryPath = entry.path();
				if (entryPath.has_extension() && entryPath.extension() == COLLISION_MESH_FILE_EXTENSION) {
					CollisionMeshFile meshFile = {};
					meshFile.ReadFromFile(entryPath.string());
					int& hashCount = hashCounts[meshFile.hash];

					if (hashCount > 0) {
						RS_WARN(MSG_PREFIX << "Collision mesh " << entryPath << " is a duplicate (0x" << std::hex << meshFile.hash << "), " <<
							"already loaded a mesh with the same hash."
						);
					} else if (targetHashes.count(meshFile.hash) == 0) {
						RS_WARN(MSG_PREFIX <<
							"Collision mesh " << entryPath << " does not match any known soccar collision file (0x" << std::hex << meshFile.hash << "), " <<
							"make sure they were dumped from a normal soccar arena."
						)
					}
					hashCount++;

					btTriangleMesh* triMesh = meshFile.MakeBulletMesh();
					
#ifdef RS_MERGE_ARENA_MESHES
					for (int i = 0; i < triMesh->m_32bitIndices.size(); i += 3) {
						btVector3 vertices[3];
						for (int j = 0; j < 3; j++) {
							vertices[j] = triMesh->m_4componentVertices[triMesh->m_32bitIndices[i + j]];
						}

						masterTriMesh->addTriangle(vertices[0], vertices[1], vertices[2]);
					}
#else
					auto bvtMesh = new btBvhTriangleMeshShape(triMesh, false);
					btTriangleInfoMap* infoMap = new btTriangleInfoMap();
					btGenerateInternalEdgeInfo(bvtMesh, infoMap);
					bvtMesh->setTriangleInfoMap(infoMap);
					arenaCollisionMeshes.push_back(bvtMesh);
#endif

					//delete triMesh;
				}
			}

#ifdef RS_MERGE_ARENA_MESHES
			btBvhTriangleMeshShape* bvhShape = new btBvhTriangleMeshShape(masterTriMesh, false);
			btTriangleInfoMap* infoMap = new btTriangleInfoMap();
			btGenerateInternalEdgeInfo(bvhShape, infoMap);
			bvhShape->setTriangleInfoMap(infoMap);
			arenaCollisionMeshes.push_back(bvhShape);
#endif

			if (arenaCollisionMeshes.empty()) {
				RS_ERR_CLOSE(MSG_PREFIX <<
					"Failed to find soccar field asset files at " << basePath
					<< ", the folder exists but has no collision mesh files.")
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
#include "RocketSim.h"

#include "../libsrc/bullet3-3.24/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "../libsrc/bullet3-3.24/BulletCollision/CollisionShapes/btTriangleMesh.h"
#include "../libsrc/bullet3-3.24/BulletCollision/CollisionDispatch/btInternalEdgeUtility.h"

using namespace RocketSim;

std::filesystem::path RocketSim::_collisionMeshesFolder = {};
std::mutex RocketSim::_beginInitMutex = {};

struct MeshHashSet {
	std::unordered_map<uint32_t, int> hashes;
	void AddAll(std::initializer_list<uint32_t> hashesToAdd) {
		for (uint32_t hash : hashesToAdd)
			hashes[hash] = 0;
	}

	MeshHashSet(GameMode gameMode) {
		if (gameMode == GameMode::SOCCAR) {
			AddAll(
				{
					0xA160BAF9, 0x2811EEE8, 0xB81AC8B9, 0x760358D3,
					0x73AE4940, 0x918F4A4E, 0x1F8EE550, 0x255BA8C1,
					0x14B84668, 0xEC759EBF, 0x94FB0D5C, 0xDEA07102,
					0xBD4FBEA8, 0x39A47F63, 0x3D79D25D, 0xD84C7A68
				}
			);
		} else if (gameMode == GameMode::HOOPS) {
			AddAll(
				{
					0x72F2359E, 0x5ED14A26, 0XFD5A0D07, 0x92AFA5B5,
					0x0E4133C7, 0x399E8B5F, 0XBB9D4FB5, 0x8C87FB93,
					0x1CFD0E16, 0xE19E1DF6, 0x9CA179DC, 0x16F3CC19
				}
			);
		}
	}

	int& operator[](uint32_t hash) {
		return hashes[hash];
	}
};

static RocketSimStage stage = RocketSimStage::UNINITIALIZED;
RocketSimStage RocketSim::GetStage() {
	return stage;
}

std::vector<btBvhTriangleMeshShape*>& RocketSim::GetArenaCollisionShapes(GameMode gameMode) {
	static std::vector<btBvhTriangleMeshShape*> arenaCollisionMeshes;
	static std::vector<btBvhTriangleMeshShape*> arenaCollisionMeshes_hoops;

	return (gameMode == GameMode::HOOPS ? arenaCollisionMeshes_hoops : arenaCollisionMeshes);
}

#ifndef RS_NO_SUSPCOLGRID
static SuspensionCollisionGrid
	suspColGrids_soccar[] = { {GameMode::SOCCAR, true}, {GameMode::SOCCAR, false} },
	suspColGrids_hoops[]  = { {GameMode::HOOPS,  true}, {GameMode::HOOPS,  false} };
SuspensionCollisionGrid& RocketSim::GetDefaultSuspColGrid(GameMode gameMode, bool isLight) {
	if (gameMode == GameMode::HOOPS) {
		return suspColGrids_hoops[isLight];
	} else {
		return suspColGrids_soccar[isLight];
	}
}
#endif

void RocketSim::Init(std::filesystem::path collisionMeshesFolder, bool silent) {

	std::map<GameMode, std::vector<FileData>> meshFileMap = {};

	for (int i = 0; i < 2; i++) { // Load collision meshes for soccar and hoops
		GameMode gameMode = (i > 0) ? GameMode::HOOPS : GameMode::SOCCAR;
		auto& meshes = GetArenaCollisionShapes(gameMode);

		std::filesystem::path basePath = collisionMeshesFolder;
		std::filesystem::path soccarMeshesFolder = basePath / GAMEMODE_STRS[(int)gameMode];

		if (!std::filesystem::exists(soccarMeshesFolder))
			continue;

		MeshHashSet targetHashes = MeshHashSet(gameMode);

		// Load collision meshes
		auto dirItr = std::filesystem::directory_iterator(soccarMeshesFolder);
		for (auto& entry : dirItr) {
			auto entryPath = entry.path();
			if (entryPath.has_extension() && entryPath.extension() == COLLISION_MESH_FILE_EXTENSION) {
				DataStreamIn streamIn = DataStreamIn(entryPath, false);
				meshFileMap[gameMode].push_back(streamIn.data);
			}
		}
	}

	RocketSim::InitFromMem(meshFileMap, silent);

	_collisionMeshesFolder = collisionMeshesFolder;
}

void RocketSim::InitFromMem(const std::map<GameMode, std::vector<FileData>>& meshFilesMap, bool silent) {

	constexpr char MSG_PREFIX[] = "RocketSim::Init(): ";

	_collisionMeshesFolder = "<MESH FILES LOADED FROM MEMORY>";

	_beginInitMutex.lock();
	{
		if (stage != RocketSimStage::UNINITIALIZED) {
			if (!silent)
				RS_WARN("RocketSim::Init() called again after already initialized, ignoring...");
			_beginInitMutex.unlock();
			return;
		}

		if (!silent)
			RS_LOG("Initializing RocketSim version " RS_VERSION ", created by ZealanL...");


		stage = RocketSimStage::INITIALIZING;

		uint64_t startMS = RS_CUR_MS();

		for (auto& mapPair : meshFilesMap) { // Load collision meshes for soccar and hoops
			GameMode gameMode = mapPair.first;
			auto& meshFiles = mapPair.second;

			if (!silent)
				RS_LOG("Loading arena meshes for " << GAMEMODE_STRS[(int)gameMode] << "...");

			if (meshFiles.empty()) {
				if (!silent)
					RS_LOG(" > No meshes, skipping");
				continue;
			}

			auto& meshes = GetArenaCollisionShapes(gameMode);

			MeshHashSet targetHashes = MeshHashSet(gameMode);

			// Load collision meshes
			int idx = 0;
			for (auto& entry : meshFiles) {
				DataStreamIn dataStream = {};
				dataStream.data = entry;
				CollisionMeshFile meshFile = {};
				meshFile.ReadFromStream(dataStream, silent);
				int& hashCount = targetHashes[meshFile.hash];

				if (hashCount > 0) {
					if (!silent)
						RS_WARN(MSG_PREFIX << "Collision mesh [" << idx << "] is a duplicate (0x" << std::hex << meshFile.hash << "), " <<
							"already loaded a mesh with the same hash."
						);
				} else if (targetHashes.hashes.count(meshFile.hash) == 0) {
					if (!silent)
						RS_WARN(MSG_PREFIX <<
							"Collision mesh [" << idx << "] does not match any known " << GAMEMODE_STRS[(int)gameMode] << " collision mesh (0x" << std::hex << meshFile.hash << "), " <<
							"make sure they were dumped from a normal " << GAMEMODE_STRS[(int)gameMode] << " arena."
						);
				}
				hashCount++;

				btTriangleMesh* triMesh = meshFile.MakeBulletMesh();

				auto bvtMesh = new btBvhTriangleMeshShape(triMesh, true);
				btTriangleInfoMap* infoMap = new btTriangleInfoMap();
				btGenerateInternalEdgeInfo(bvtMesh, infoMap);
				bvtMesh->setTriangleInfoMap(infoMap);
				meshes.push_back(bvtMesh);

				idx++;
			}
		}

		if (!silent) {
			RS_LOG(MSG_PREFIX << "Finished loading arena collision meshes:");
			RS_LOG(" > Soccar: " << GetArenaCollisionShapes(GameMode::SOCCAR).size());
			RS_LOG(" > Hoops: " << GetArenaCollisionShapes(GameMode::HOOPS).size());
		}

#ifndef RS_NO_SUSPCOLGRID
		{ // Set up suspension collision grid
			for (int i = 0; i < 2; i++) {
				GameMode gameMode = i > 0 ? GameMode::HOOPS : GameMode::SOCCAR;
				auto& meshes = GetArenaCollisionShapes(gameMode);

				if (!meshes.empty()) {
					if (!silent)
						RS_LOG("Building collision suspension grids from " << GAMEMODE_STRS[(int)gameMode] << " arena meshes...");

					for (int j = 0; j < 2; j++) {
						auto& grid = GetDefaultSuspColGrid(gameMode, j);
						grid.Allocate();
						grid.SetupWorldCollision(meshes);
					}
				}
			}
		}
#endif


		uint64_t elapsedMS = RS_CUR_MS() - startMS;

		if (!silent)
			RS_LOG("Finished initializing RocketSim in " << (elapsedMS / 1000.f) << "s!");

		stage = RocketSimStage::INITIALIZED;
	}
	_beginInitMutex.unlock();
}

void RocketSim::AssertInitialized(const char* errorMsgPrefix) {
	if (stage != RocketSimStage::INITIALIZED) {
		RS_ERR_CLOSE(errorMsgPrefix << "RocketSim has not been initialized, call RocketSim::Init() first")
	}
}
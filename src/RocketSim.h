#pragma once
#include "Sim/Car/Car.h"
#include "Sim/Ball/Ball.h"
#include "Sim/Arena/Arena.h"

#include "Math/Math.h"

// AVAILABLE DEFS FOR ROCKETSIM:
//	RS_MAX_SPEED: Define this to remove certain sanity checks for faster speed
//	RS_DONT_LOG: Define this to disable all logging output
//	RS_NO_SUSPCOLGRID: Disable the suspension-collision grid optimization

class btBvhTriangleMeshShape;

namespace RocketSim {
	enum class RocketSimStage : byte {
		UNINITIALIZED,
		INITIALIZING,
		INITIALIZED
	};

	extern std::filesystem::path _collisionMeshesFolder;
	extern std::mutex _beginInitMutex;

	void Init(std::filesystem::path collisionMeshesFolder);
	void AssertInitialized(const char* errorMsgPrefix);

	RocketSimStage GetStage();

	std::vector<btBvhTriangleMeshShape*>& GetArenaCollisionShapes(GameMode gameMode);

#ifndef RS_NO_SUSPCOLGRID
	SuspensionCollisionGrid& GetDefaultSuspColGrid(GameMode gameMode, bool isLight);
#endif
}
#pragma once
#include "../../BaseInc.h"

namespace MeshLoader {

	struct TriIndices {
		uint32 data[3];
		uint32 operator[](size_t index) {
			assert(index < 3);
			return data[index];
		}
	};

	struct Mesh {
		vector<btVector3> verts;
		vector<TriIndices> triIds;

		btTriangleMesh* MakeBulletMesh(btVector3 scale = btVector3(1, 1, 1));
	};

	Mesh LoadMeshFromFiles(string path, float scale = 1);
}
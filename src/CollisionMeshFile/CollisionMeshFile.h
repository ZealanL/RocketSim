#pragma once
#include "../Framework.h"
#include "../BulletLink.h"

#define COLLISION_MESH_BASE_PATH "./collision_meshes/"
#define COLLISION_MESH_FILE_EXTENSION ".cmf"

struct btTriangleMesh;

// Collision mesh file structure based off of the one in https://github.com/ZealanL/RLArenaCollisionDumper
struct CollisionMeshFile {

	struct Triangle {
		int vertexIndexes[3];
	};

	struct Vertex {
		float x, y, z;

		float& operator[](uint32_t index) {
			assert(index < 3);
			return (index == 0) ? x : ((index == 1) ? y : z);
		}
	};

	vector<Triangle> tris;
	vector<Vertex> vertices;

	uint32_t hash;

	void ReadFromFile(string filePath);
	btTriangleMesh* MakeBulletMesh();
	void UpdateHash();
};
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
	};

	vector<Triangle> tris;
	vector<Vertex> vertices;

	void ReadFromFile(string filePath);
	btTriangleMesh* MakeBulletMesh();
};
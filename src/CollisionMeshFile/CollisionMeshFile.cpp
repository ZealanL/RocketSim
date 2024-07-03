#include "CollisionMeshFile.h"

#include "../DataStream/DataStreamIn.h"

#include "../../libsrc/bullet3-3.24/BulletDynamics/Dynamics/btRigidBody.h"
#include "../../libsrc/bullet3-3.24/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "../../libsrc/bullet3-3.24/BulletCollision/CollisionShapes/btTriangleMesh.h"

RS_NS_START

void CollisionMeshFile::ReadFromStream(DataStreamIn& in, bool silent, std::string filePath) {
	constexpr char ERROR_PREFIX_STR[] = " > CollisionMeshFile::ReadFromStream(): ";

	// If you have more verts or tris then this, I have no idea what you are doing, godspeed
	constexpr int MAX_VERT_OR_TRI_COUNT = 1000 * 1000;

	// Read triangle/vertex counts
	int32_t numTris, numVertices;
	in.Read(numTris);
	in.Read(numVertices);

	if (RS_MIN(numTris, numVertices) <= 0 || RS_MAX(numTris, numVertices) > MAX_VERT_OR_TRI_COUNT) {
		RS_ERR_CLOSE(
			ERROR_PREFIX_STR << "Invalid collision mesh file at \"" << filePath <<
			"\" (bad triangle/vertex count: [" << numTris << ", " << numVertices << "])");
	}

	tris.resize(numTris);
	vertices.resize(numVertices);

	for (Triangle& tri : tris)
		tri = in.Read<Triangle>();

	for (Vertex& vert : vertices)
		vert = in.Read<Vertex>();

#ifndef RS_MAX_SPEED
	if (in.IsOverflown()) {
		RS_ERR_CLOSE(
			ERROR_PREFIX_STR << "Invalid collision mesh file at \"" << filePath <<
			"\" (input data overflown by " << (in.pos - in.data.size()) << " bytes!)");
	}

	// Verify that the triangle data is correct
	for (Triangle& tri : tris) {
		for (int i = 0; i < 3; i++) {
			int vertIndex = tri.vertexIndexes[i];
			if (vertIndex < 0 || vertIndex >= numVertices) {
				RS_ERR_CLOSE(
					ERROR_PREFIX_STR << "Invalid collision mesh file at \"" << filePath <<
					"\" (bad triangle vertex index)");
			}
		}
	}
#endif

	UpdateHash();

	if (!silent)
		RS_LOG("   > Loaded " << numVertices << " verts and " << numTris << " tris, hash: 0x" << std::hex << hash);
}

btTriangleMesh* CollisionMeshFile::MakeBulletMesh() {
	btTriangleMesh* result = new btTriangleMesh();

	for (Vertex& vert : vertices)
		result->findOrAddVertex(btVector3(vert.x, vert.y, vert.z), false);

	for (Triangle& tri : tris)
		result->addTriangleIndices(tri.vertexIndexes[0], tri.vertexIndexes[1], tri.vertexIndexes[2]);

	return result;
}

void CollisionMeshFile::UpdateHash() {
	uint32_t hash = vertices.size() + (tris.size() * vertices.size());

	// From: https://stackoverflow.com/questions/20511347/a-good-hash-function-for-a-vector/72073933#72073933

	constexpr uint32_t
		HASH_VAL_MUELLER = 0x45D9F3B,
		HASH_VAL_SHIFT = 0x9E3779B9;

	for (Triangle& tri : tris) {
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				uint32_t curVal = vertices[tri.vertexIndexes[i]][j];

				for (int k = 0; k < 2; k++)
					curVal = ((curVal >> 16) ^ curVal) * HASH_VAL_MUELLER;
				
				curVal = (curVal >> 16) ^ curVal;
				hash ^= curVal + HASH_VAL_SHIFT + (hash << 6) + (hash >> 2);
			}
		}
	}

	this->hash = hash;
}

RS_NS_END
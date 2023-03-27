#include "CollisionMeshFile.h"

#include "../DataStream/DataStreamIn.h"

#include "../../libsrc/bullet3-3.24/BulletDynamics/Dynamics/btRigidBody.h"
#include "../../libsrc/bullet3-3.24/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "../../libsrc/bullet3-3.24/BulletCollision/CollisionShapes/btTriangleMesh.h"

void CollisionMeshFile::ReadFromFile(string filePath) {
	constexpr char ERROR_PREFIX_STR[] = " > CollisionMeshFile::ReadFromFile(): ";

	DataStreamIn in = DataStreamIn(filePath, false);
	RS_LOG("  > Loading \"" << filePath << "\"...");

	constexpr int MAX_VERT_OR_TRI_COUNT = 1000 * 1000;

	// Read triangle/vertex counts
	int32_t numTris, numVertices;
	in.ReadMultiple(numTris, numVertices);
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

	RS_LOG("   > Loaded " << numVertices << " verts and " << numTris << " tris.");
}

btTriangleMesh* CollisionMeshFile::MakeBulletMesh() {
	btTriangleMesh* result = new btTriangleMesh();

	for (Vertex& vert : vertices)
		result->findOrAddVertex(btVector3(vert.x, vert.y, vert.z), false);

	for (Triangle& tri : tris)
		result->addTriangleIndices(tri.vertexIndexes[0], tri.vertexIndexes[1], tri.vertexIndexes[2]);

	return result;
}
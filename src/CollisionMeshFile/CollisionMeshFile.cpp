#include "CollisionMeshFile.h"

#include "../DataStream/DataStreamIn.h"

void CollisionMeshFile::ReadFromFile(string filePath) {
	constexpr char ERR_BASE_STR[] = " > CollisionMeshFile::ReadFromFile(): ";

	DataStreamIn in = DataStreamIn(filePath, false);
	RS_LOG("  > Loading \"" << filePath << "\"...");

	// Read triangle/vertex counts
	int32_t numTris, numVertices;
	in.ReadMultiple(numVertices, numTris);
	if (numTris <= 0 || numVertices <= 0)
		RS_ERR_CLOSE(
			ERR_BASE_STR << "Invalid collision mesh file at \"" << filePath <<
			"\" (bad triangle/vertex count: [" << numTris << ", " << numVertices << "])");

	tris.resize(numTris);
	vertices.resize(numVertices);

	for (Triangle& tri : tris)
		tri = in.Read<Triangle>();

	for (Vertex& vert : vertices)
		vert = in.Read<Vertex>();

	// Verify that the triangle data is correct
	for (Triangle& tri : tris) {
		for (int i = 0; i < 3; i++) {
			int vertIndex = tri.vertexIndexes[i];
			if (i < 0 || i >= numVertices) {
				RS_ERR_CLOSE(
					ERR_BASE_STR << "Invalid collision mesh file at \"" << filePath <<
					"\" (bad triangle vertex index)");
			}
		}
	}

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
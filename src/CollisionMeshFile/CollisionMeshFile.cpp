#include "CollisionMeshFile.h"

void CollisionMeshFile::ReadFromFile(string filePath) {
	constexpr char ERR_BASE_STR[] = " > CollisionMeshFile::ReadFromFile(): ";

	std::ifstream inStream = std::ifstream(filePath, std::ios::binary);
	if (!inStream.good())
		RS_ERR_CLOSE(ERR_BASE_STR << "Failed to open file \"" << filePath << "\"");

	RS_LOG(" > Loading \"" << filePath << "\"...");

	// Read triangle/vertex counts
	int numTris = 0, numVertices = 0;
	inStream.read((char*)&numTris, sizeof(int));
	inStream.read((char*)&numVertices, sizeof(int));

	if (numTris <= 0 || numVertices <= 0)
		RS_ERR_CLOSE(
			ERR_BASE_STR << "Invalid collision mesh file at \"" << filePath <<
			"\" (bad triangle/vertex count: [" << numTris << ", " << numVertices << "])");

	tris.resize(numTris);
	vertices.resize(numVertices);

	// Read tris and vertices (raw data)
	inStream.read((char*)tris.data(), numTris * sizeof(Triangle));
	inStream.read((char*)vertices.data(), numVertices * sizeof(Vertex));

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

	inStream.close();
}

btTriangleMesh* CollisionMeshFile::MakeBulletMesh() {
	btTriangleMesh* result = new btTriangleMesh();

	for (Vertex& vert : vertices)
		result->findOrAddVertex(btVector3(vert.x, vert.y, vert.z), false);

	for (Triangle& tri : tris)
		result->addTriangleIndices(tri.vertexIndexes[0], tri.vertexIndexes[1], tri.vertexIndexes[2]);

	return result;
}
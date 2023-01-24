#include "MeshLoader.h"

MeshLoader::Mesh MeshLoader::LoadMeshFromFiles(string path, float scale) {

	RS_LOG("Loading mesh data from \"" << path << "\"...");

    string 
		vertsFile = path + "_vertices.bin", 
		idsFile = path + "_ids.bin";

	vector<btVector3> verticies;
	auto vertsIn = std::ifstream(vertsFile, std::ios::binary);
	if (!vertsIn.good())
		RS_ERR_CLOSE("Failed to find/access vertices file at \"" << vertsFile << "\".");

	MeshLoader::Mesh result;

	int numVerts = std::filesystem::file_size(vertsFile) / sizeof(float) / 3;
	result.verts.reserve(numVerts);

	RS_LOG(" > Reading in " << numVerts << " verts...");
	for (int i = 0; i < numVerts; i++) {
		float vals[3];
		vertsIn.read((char*)&vals, sizeof(vals));
		result.verts.push_back(btVector3(vals[0], vals[1], vals[2]) * scale);
	}
	vertsIn.close();

	auto idsIn = std::ifstream(idsFile, std::ios::binary);
	if (!idsIn.good())
		RS_ERR_CLOSE("Failed to find/access ids file at \"" << idsFile << "\".");

	int numTris = std::filesystem::file_size(idsFile) / sizeof(uint32) / 3;
	result.triIds.reserve(numTris);

	RS_LOG(" > Reading in " << numTris << " tris...");
	for (int i = 0; i < numTris; i++) {
		TriIndices vals;
		idsIn.read((char*)&vals, sizeof(vals));

		assert(vals.data[0] != vals.data[1]);
		assert(vals.data[1] != vals.data[2]);
		assert(vals.data[2] != vals.data[0]);

		result.triIds.push_back(vals);
	}

	RS_LOG(" > Done.");
	return result;
}

btTriangleMesh* MeshLoader::Mesh::MakeBulletMesh(btVector3 scale) {
	btTriangleMesh* result = new btTriangleMesh();

	for (btVector3 vert : verts)
		result->findOrAddVertex(vert * scale, false);

	for (TriIndices tri : triIds)
		result->addTriangleIndices(tri[0], tri[1], tri[2]);

	return result;
}

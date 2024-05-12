#pragma once

#include "btOverlappingPairCache.h"
#include <vector>

struct btRSBroadphaseProxy : public btBroadphaseProxy
{
	bool isStatic;

	int cellIdx;
	int iIdx, jIdx, kIdx;

	int shapeType;
	int m_nextFree;

	//	int			m_handleId;

	btRSBroadphaseProxy() {};

	btRSBroadphaseProxy(
		const btVector3& minpt, const btVector3& maxpt, int shapeType, void* userPtr, int collisionFilterGroup, int collisionFilterMask, 
		bool isStatic, int cellIdx, int i, int j, int k)
		: btBroadphaseProxy(minpt, maxpt, userPtr, collisionFilterGroup, collisionFilterMask), 
		isStatic(isStatic), 
		cellIdx(cellIdx), iIdx(i), jIdx(j), kIdx(k),
		shapeType(shapeType) {
	}

	SIMD_FORCE_INLINE void SetNextFree(int next) { m_nextFree = next; }
	SIMD_FORCE_INLINE int GetNextFree() const { return m_nextFree; }
};

// Custom broadphase implementation for RocketSim
// Uses spacial division with a fixed voxel grid
// Somewhat based off of btSimpleBroadphase
class btRSBroadphase : public btBroadphaseInterface
{
public:
	int m_numHandles;  // number of active handles
	int m_maxHandles;  // max number of handles
	int m_LastHandleIndex;

	btVector3 minPos, maxPos;
	float cellSize, cellSizeSq;
	int cellsX, cellsY, cellsZ;
	int totalCells;

	int numDynProxies = 0;

	int totalStaticPairs = 0, totalDynPairs = 0;
	int totalRealPairs = 0;
	int totalItrs = 0;

	std::vector<std::pair<btRSBroadphaseProxy*, btRSBroadphaseProxy*>> activePairs;

	struct Cell {
		constexpr static int RESERVED_SIZE = 4;
		std::vector<btRSBroadphaseProxy*> dynHandles;
		std::vector<btRSBroadphaseProxy*> staticHandles;
		Cell() {
			dynHandles.reserve(RESERVED_SIZE);
			staticHandles.reserve(RESERVED_SIZE);
		}

		void RemoveDyn(btRSBroadphaseProxy* proxy) {
			for (int i = 0; i < dynHandles.size(); i++) {
				if (dynHandles[i] == proxy) {
					dynHandles.erase(dynHandles.begin() + i);
					return;
				}
			}
		}

		void RemoveStatic(btRSBroadphaseProxy* proxy) {
			for (int i = 0; i < staticHandles.size(); i++) {
				if (staticHandles[i] == proxy) {
					staticHandles.erase(staticHandles.begin() + i);
					return;
				}
			}
		}
	};
	std::vector<Cell> cells;

	Cell& GetCell(int i, int j, int k) {
		int idx = i * cellsY * cellsZ + j * cellsZ + k;
		return cells[idx];
	}

	void GetCellIndices(btVector3 pos, int& i, int& j, int& k) const {
		btVector3 cellIdxF = (pos - minPos) / cellSize;
		i = (int)cellIdxF.x();
		j = (int)cellIdxF.y();
		k = (int)cellIdxF.z();
		btClamp(i, 0, cellsX - 1);
		btClamp(j, 0, cellsY - 1);
		btClamp(k, 0, cellsZ - 1);
	}

	btVector3 GetCellMinPos(int i, int j, int k) const {
		return minPos + btVector3(i, j, k) * cellSize;
	}

	int GetCellIdx(const btVector3& pos) const {
		int i, j, k;
		GetCellIndices(pos, i, j, k);
		return i * cellsY * cellsZ + j * cellsZ + k;
	}

	btRSBroadphaseProxy* m_pHandles;  // handles pool

	void* m_pHandlesRawPtr;
	int m_firstFreeHandle;  // free handles list

	int allocHandle() {
		btAssert(m_numHandles < m_maxHandles);
		int freeHandle = m_firstFreeHandle;
		m_firstFreeHandle = m_pHandles[freeHandle].GetNextFree();
		m_numHandles++;
		if (freeHandle > m_LastHandleIndex) {
			m_LastHandleIndex = freeHandle;
		}
		return freeHandle;
	}

	void freeHandle(btRSBroadphaseProxy* proxy) {
		int handle = int(proxy - m_pHandles);
		btAssert(handle >= 0 && handle < m_maxHandles);
		if (handle == m_LastHandleIndex) {
			m_LastHandleIndex--;
		}
		proxy->SetNextFree(m_firstFreeHandle);
		m_firstFreeHandle = handle;

		proxy->m_clientObject = 0;

		m_numHandles--;
	}

	btOverlappingPairCache* m_pairCache;
	bool m_ownsPairCache;

	int m_invalidPair;

	inline btRSBroadphaseProxy* getRSProxyFromProxy(btBroadphaseProxy* proxy) {
		btRSBroadphaseProxy* proxy0 = static_cast<btRSBroadphaseProxy*>(proxy);
		return proxy0;
	}

	inline const btRSBroadphaseProxy* getRSProxyFromProxy(btBroadphaseProxy* proxy) const {
		const btRSBroadphaseProxy* proxy0 = static_cast<const btRSBroadphaseProxy*>(proxy);
		return proxy0;
	}

	///reset broadphase internal structures, to ensure determinism/reproducability
	virtual void resetPool(btCollisionDispatcher* dispatcher);

	void validate();

protected:
public:
	btRSBroadphase(btVector3 min, btVector3 max, float cellSize, btOverlappingPairCache* overlappingPairCache, int maxProxies = 65536);
	virtual ~btRSBroadphase();

	static bool aabbOverlap(btRSBroadphaseProxy* proxy0, btRSBroadphaseProxy* proxy1);

	virtual btBroadphaseProxy* createProxy(const btVector3& aabbMin, const btVector3& aabbMax, int shapeType, void* userPtr, int collisionFilterGroup, int collisionFilterMask, btCollisionDispatcher* dispatcher);

	virtual void calculateOverlappingPairs(btCollisionDispatcher* dispatcher);

	virtual void destroyProxy(btBroadphaseProxy* proxy, btCollisionDispatcher* dispatcher);
	virtual void setAabb(btBroadphaseProxy* proxy, const btVector3& aabbMin, const btVector3& aabbMax, btCollisionDispatcher* dispatcher);
	virtual void getAabb(btBroadphaseProxy* proxy, btVector3& aabbMin, btVector3& aabbMax) const;

	virtual void rayTest(const btVector3& rayFrom, const btVector3& rayTo, btBroadphaseRayCallback& rayCallback, const btVector3& aabbMin = btVector3(0, 0, 0), const btVector3& aabbMax = btVector3(0, 0, 0));
	virtual void aabbTest(const btVector3& aabbMin, const btVector3& aabbMax, btBroadphaseAabbCallback& callback);

	btOverlappingPairCache* getOverlappingPairCache() {
		return m_pairCache;
	}
	const btOverlappingPairCache* getOverlappingPairCache() const {
		return m_pairCache;
	}

	bool testAabbOverlap(btBroadphaseProxy* proxy0, btBroadphaseProxy* proxy1);

	///getAabb returns the axis aligned bounding box in the 'global' coordinate frame
	///will add some transform later
	virtual void getBroadphaseAabb(btVector3& aabbMin, btVector3& aabbMax) const {
		aabbMin.setValue(-BT_LARGE_FLOAT, -BT_LARGE_FLOAT, -BT_LARGE_FLOAT);
		aabbMax.setValue(BT_LARGE_FLOAT, BT_LARGE_FLOAT, BT_LARGE_FLOAT);
	}

	virtual void printStats() {
		//		printf("btRSBroadphase.h\n");
		//		printf("numHandles = %d, maxHandles = %d\n",m_numHandles,m_maxHandles);
	}
};
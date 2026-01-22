// Detour.cpp
#include <string.h>
#include <stdlib.h>
#include <float.h>
#include <string.h>
#include <stdio.h>
#include <new>

#include <Moss/Navigation/navigation_intern.h>
#ifndef MOSS_DEBUG_RENDERER
	#include <Moss/Moss_Renderer.h>
#endif // MOSS_DEBUG_RENDERER

static unsigned short MESH_NULL_IDX = 0xffff;


/*													*/

static dtAllocFunc* sAllocFunc = MOSS_MALLOC;
static Moss_FreeFunc* sFreeFunc = MOSS_FREE;

void dtAllocSetCustom(dtAllocFunc *allocFunc, Moss_FreeFunc *freeFunc)
{
	sAllocFunc = allocFunc ? allocFunc : dtAllocDefault;
	sFreeFunc = freeFunc ? freeFunc : MOSS_FREE;
}

void* dtAlloc(size_t size, dtAllocHint hint)
{
	return sAllocFunc(size, hint);
}


/*													*/


#ifndef RC_DISABLE_ASSERTS

static dtAssertFailFunc* sAssertFailFunc = 0;

void dtAssertFailSetCustom(dtAssertFailFunc *assertFailFunc)
{
	sAssertFailFunc = assertFailFunc;
}

dtAssertFailFunc* dtAssertFailGetCustom()
{
	return sAssertFailFunc;
}

#endif


/*													*/


#ifdef DT_POLYREF64
// From Thomas Wang, https://gist.github.com/badboy/6267743
inline unsigned int dtHashRef(dtPolyRef a)
{
	a = (~a) + (a << 18); // a = (a << 18) - a - 1;
	a = a ^ (a >> 31);
	a = a * 21; // a = (a + (a << 2)) + (a << 4);
	a = a ^ (a >> 11);
	a = a + (a << 6);
	a = a ^ (a >> 22);
	return (unsigned int)a;
}
#else
inline unsigned int dtHashRef(dtPolyRef a)
{
	a += ~(a<<15);
	a ^=  (a>>10);
	a +=  (a<<3);
	a ^=  (a>>6);
	a += ~(a<<11);
	a ^=  (a>>16);
	return (unsigned int)a;
}
#endif

//////////////////////////////////////////////////////////////////////////////////////////
Moss_RecastNodePool::Moss_RecastNodePool(int maxNodes, int hashSize) : m_nodes(0), m_first(0), m_next(0), m_maxNodes(maxNodes), m_hashSize(hashSize), m_nodeCount(0) {
	dtAssert(dtNextPow2(m_hashSize) == (unsigned int)m_hashSize);
	// pidx is special as 0 means "none" and 1 is the first node. For that reason
	// we have 1 fewer nodes available than the number of values it can contain.
	dtAssert(m_maxNodes > 0 && m_maxNodes <= DT_NULL_IDX && m_maxNodes <= (1 << DT_NODE_PARENT_BITS) - 1);

	m_nodes = (Moss_RecastNode*)dtAlloc(sizeof(Moss_RecastNode)*m_maxNodes, DT_ALLOC_PERM);
	m_next = (dtNodeIndex*)dtAlloc(sizeof(dtNodeIndex)*m_maxNodes, DT_ALLOC_PERM);
	m_first = (dtNodeIndex*)dtAlloc(sizeof(dtNodeIndex)*hashSize, DT_ALLOC_PERM);

	dtAssert(m_nodes);
	dtAssert(m_next);
	dtAssert(m_first);

	memset(m_first, 0xff, sizeof(dtNodeIndex)*m_hashSize);
	memset(m_next, 0xff, sizeof(dtNodeIndex)*m_maxNodes);
}

Moss_RecastNodePool::~Moss_RecastNodePool()
{
	MOSS_FREE(m_nodes);
	MOSS_FREE(m_next);
	MOSS_FREE(m_first);
}

void Moss_RecastNodePool::clear()
{
	memset(m_first, 0xff, sizeof(dtNodeIndex)*m_hashSize);
	m_nodeCount = 0;
}

unsigned int Moss_RecastNodePool::findNodes(dtPolyRef id, Moss_RecastNode** nodes, const int maxNodes)
{
	int n = 0;
	unsigned int bucket = dtHashRef(id) & (m_hashSize-1);
	dtNodeIndex i = m_first[bucket];
	while (i != DT_NULL_IDX)
	{
		if (m_nodes[i].id == id)
		{
			if (n >= maxNodes)
				return n;
			nodes[n++] = &m_nodes[i];
		}
		i = m_next[i];
	}

	return n;
}

Moss_RecastNode* Moss_RecastNodePool::findNode(dtPolyRef id, unsigned char state)
{
	unsigned int bucket = dtHashRef(id) & (m_hashSize-1);
	dtNodeIndex i = m_first[bucket];
	while (i != DT_NULL_IDX)
	{
		if (m_nodes[i].id == id && m_nodes[i].state == state)
			return &m_nodes[i];
		i = m_next[i];
	}
	return 0;
}

Moss_RecastNode* Moss_RecastNodePool::getNode(dtPolyRef id, unsigned char state)
{
	unsigned int bucket = dtHashRef(id) & (m_hashSize-1);
	dtNodeIndex i = m_first[bucket];
	Moss_RecastNode* node = 0;
	while (i != DT_NULL_IDX)
	{
		if (m_nodes[i].id == id && m_nodes[i].state == state)
			return &m_nodes[i];
		i = m_next[i];
	}
	
	if (m_nodeCount >= m_maxNodes)
		return 0;
	
	i = (dtNodeIndex)m_nodeCount;
	m_nodeCount++;
	
	// Init node
	node = &m_nodes[i];
	node->pidx = 0;
	node->cost = 0;
	node->total = 0;
	node->id = id;
	node->state = state;
	node->flags = 0;
	
	m_next[i] = m_first[bucket];
	m_first[bucket] = i;
	
	return node;
}


//////////////////////////////////////////////////////////////////////////////////////////
Moss_RecastNodeQueue::Moss_RecastNodeQueue(int n) :
	m_heap(0),
	m_capacity(n),
	m_size(0)
{
	dtAssert(m_capacity > 0);
	
	m_heap = (Moss_RecastNode**)dtAlloc(sizeof(Moss_RecastNode*)*(m_capacity+1), DT_ALLOC_PERM);
	dtAssert(m_heap);
}

Moss_RecastNodeQueue::~Moss_RecastNodeQueue()
{
	MOSS_FREE(m_heap);
}

void Moss_RecastNodeQueue::bubbleUp(int i, Moss_RecastNode* node)
{
	int parent = (i-1)/2;
	// note: (index > 0) means there is a parent
	while ((i > 0) && (m_heap[parent]->total > node->total))
	{
		m_heap[i] = m_heap[parent];
		i = parent;
		parent = (i-1)/2;
	}
	m_heap[i] = node;
}

void Moss_RecastNodeQueue::trickleDown(int i, Moss_RecastNode* node)
{
	int child = (i*2)+1;
	while (child < m_size)
	{
		if (((child+1) < m_size) && 
			(m_heap[child]->total > m_heap[child+1]->total))
		{
			child++;
		}
		m_heap[i] = m_heap[child];
		i = child;
		child = (i*2)+1;
	}
	bubbleUp(i, node);
}



/*													*/



struct BVItem
{
	unsigned short bmin[3];
	unsigned short bmax[3];
	int i;
};

static int compareItemX(const void* va, const void* vb)
{
	const BVItem* a = (const BVItem*)va;
	const BVItem* b = (const BVItem*)vb;
	if (a->bmin[0] < b->bmin[0])
		return -1;
	if (a->bmin[0] > b->bmin[0])
		return 1;
	return 0;
}

static int compareItemY(const void* va, const void* vb)
{
	const BVItem* a = (const BVItem*)va;
	const BVItem* b = (const BVItem*)vb;
	if (a->bmin[1] < b->bmin[1])
		return -1;
	if (a->bmin[1] > b->bmin[1])
		return 1;
	return 0;
}

static int compareItemZ(const void* va, const void* vb)
{
	const BVItem* a = (const BVItem*)va;
	const BVItem* b = (const BVItem*)vb;
	if (a->bmin[2] < b->bmin[2])
		return -1;
	if (a->bmin[2] > b->bmin[2])
		return 1;
	return 0;
}

static void calcExtends(BVItem* items, const int /*nitems*/, const int imin, const int imax,
						unsigned short* bmin, unsigned short* bmax)
{
	bmin[0] = items[imin].bmin[0];
	bmin[1] = items[imin].bmin[1];
	bmin[2] = items[imin].bmin[2];
	
	bmax[0] = items[imin].bmax[0];
	bmax[1] = items[imin].bmax[1];
	bmax[2] = items[imin].bmax[2];
	
	for (int i = imin+1; i < imax; ++i)
	{
		const BVItem& it = items[i];
		if (it.bmin[0] < bmin[0]) bmin[0] = it.bmin[0];
		if (it.bmin[1] < bmin[1]) bmin[1] = it.bmin[1];
		if (it.bmin[2] < bmin[2]) bmin[2] = it.bmin[2];
		
		if (it.bmax[0] > bmax[0]) bmax[0] = it.bmax[0];
		if (it.bmax[1] > bmax[1]) bmax[1] = it.bmax[1];
		if (it.bmax[2] > bmax[2]) bmax[2] = it.bmax[2];
	}
}

inline int longestAxis(unsigned short x, unsigned short y, unsigned short z)
{
	int	axis = 0;
	unsigned short maxVal = x;
	if (y > maxVal)
	{
		axis = 1;
		maxVal = y;
	}
	if (z > maxVal)
	{
		axis = 2;
	}
	return axis;
}

static void subdivide(BVItem* items, int nitems, int imin, int imax, int& curNode, Moss_RecastBVNode* nodes)
{
	int inum = imax - imin;
	int icur = curNode;
	
	Moss_RecastBVNode& node = nodes[curNode++];
	
	if (inum == 1)
	{
		// Leaf
		node.bmin[0] = items[imin].bmin[0];
		node.bmin[1] = items[imin].bmin[1];
		node.bmin[2] = items[imin].bmin[2];
		
		node.bmax[0] = items[imin].bmax[0];
		node.bmax[1] = items[imin].bmax[1];
		node.bmax[2] = items[imin].bmax[2];
		
		node.i = items[imin].i;
	}
	else
	{
		// Split
		calcExtends(items, nitems, imin, imax, node.bmin, node.bmax);
		
		int	axis = longestAxis(node.bmax[0] - node.bmin[0],
							   node.bmax[1] - node.bmin[1],
							   node.bmax[2] - node.bmin[2]);
		
		if (axis == 0)
		{
			// Sort along x-axis
			qsort(items+imin, inum, sizeof(BVItem), compareItemX);
		}
		else if (axis == 1)
		{
			// Sort along y-axis
			qsort(items+imin, inum, sizeof(BVItem), compareItemY);
		}
		else
		{
			// Sort along z-axis
			qsort(items+imin, inum, sizeof(BVItem), compareItemZ);
		}
		
		int isplit = imin+inum/2;
		
		// Left
		subdivide(items, nitems, imin, isplit, curNode, nodes);
		// Right
		subdivide(items, nitems, isplit, imax, curNode, nodes);
		
		int iescape = curNode - icur;
		// Negative index means escape.
		node.i = -iescape;
	}
}

static int createBVTree(Moss_RecastNavMeshCreateParams* params, Moss_RecastBVNode* nodes, int /*nnodes*/)
{
	// Build tree
	float quantFactor = 1 / params->cs;
	BVItem* items = (BVItem*)dtAlloc(sizeof(BVItem)*params->polyCount, DT_ALLOC_TEMP);
	for (int i = 0; i < params->polyCount; i++)
	{
		BVItem& it = items[i];
		it.i = i;
		// Calc polygon bounds. Use detail meshes if available.
		if (params->detailMeshes)
		{
			int vb = (int)params->detailMeshes[i*4+0];
			int ndv = (int)params->detailMeshes[i*4+1];
			float bmin[3];
			float bmax[3];

			const float* dv = &params->detailVerts[vb*3];
			dtVcopy(bmin, dv);
			dtVcopy(bmax, dv);

			for (int j = 1; j < ndv; j++)
			{
				min(bmin, &dv[j * 3]);
				max(bmax, &dv[j * 3]);
			}

			// BV-tree uses cs for all dimensions
			it.bmin[0] = (unsigned short)Clamp((int)((bmin[0] - params->bmin[0])*quantFactor), 0, 0xffff);
			it.bmin[1] = (unsigned short)Clamp((int)((bmin[1] - params->bmin[1])*quantFactor), 0, 0xffff);
			it.bmin[2] = (unsigned short)Clamp((int)((bmin[2] - params->bmin[2])*quantFactor), 0, 0xffff);

			it.bmax[0] = (unsigned short)Clamp((int)((bmax[0] - params->bmin[0])*quantFactor), 0, 0xffff);
			it.bmax[1] = (unsigned short)Clamp((int)((bmax[1] - params->bmin[1])*quantFactor), 0, 0xffff);
			it.bmax[2] = (unsigned short)Clamp((int)((bmax[2] - params->bmin[2])*quantFactor), 0, 0xffff);
		}
		else
		{
			const unsigned short* p = &params->polys[i*params->nvp * 2];
			it.bmin[0] = it.bmax[0] = params->verts[p[0] * 3 + 0];
			it.bmin[1] = it.bmax[1] = params->verts[p[0] * 3 + 1];
			it.bmin[2] = it.bmax[2] = params->verts[p[0] * 3 + 2];

			for (int j = 1; j < params->nvp; ++j)
			{
				if (p[j] == MESH_NULL_IDX) break;
				unsigned short x = params->verts[p[j] * 3 + 0];
				unsigned short y = params->verts[p[j] * 3 + 1];
				unsigned short z = params->verts[p[j] * 3 + 2];

				if (x < it.bmin[0]) it.bmin[0] = x;
				if (y < it.bmin[1]) it.bmin[1] = y;
				if (z < it.bmin[2]) it.bmin[2] = z;

				if (x > it.bmax[0]) it.bmax[0] = x;
				if (y > it.bmax[1]) it.bmax[1] = y;
				if (z > it.bmax[2]) it.bmax[2] = z;
			}
			// Remap y
			it.bmin[1] = (unsigned short)floorf((float)it.bmin[1] * params->ch / params->cs);
			it.bmax[1] = (unsigned short)ceilf((float)it.bmax[1] * params->ch / params->cs);
		}
	}
	
	int curNode = 0;
	subdivide(items, params->polyCount, 0, params->polyCount, curNode, nodes);
	
	MOSS_FREE(items);
	
	return curNode;
}

static unsigned char classifyOffMeshPoint(const float* pt, const float* bmin, const float* bmax)
{
	static const unsigned char XP = 1<<0;
	static const unsigned char ZP = 1<<1;
	static const unsigned char XM = 1<<2;
	static const unsigned char ZM = 1<<3;	

	unsigned char outcode = 0; 
	outcode |= (pt[0] >= bmax[0]) ? XP : 0;
	outcode |= (pt[2] >= bmax[2]) ? ZP : 0;
	outcode |= (pt[0] < bmin[0])  ? XM : 0;
	outcode |= (pt[2] < bmin[2])  ? ZM : 0;

	switch (outcode)
	{
	case XP: return 0;
	case XP|ZP: return 1;
	case ZP: return 2;
	case XM|ZP: return 3;
	case XM: return 4;
	case XM|ZM: return 5;
	case ZM: return 6;
	case XP|ZM: return 7;
	};

	return 0xff;	
}

// TODO: Better error handling.

/// @par
/// 
/// The output data array is allocated using the detour allocator (dtAlloc()).  The method
/// used to free the memory will be determined by how the tile is added to the navigation
/// mesh.
///
/// @see Moss_RecastNavMesh, Moss_RecastNavMesh::addTile()
bool dtCreateNavMeshData(Moss_RecastNavMeshCreateParams* params, unsigned char** outData, int* outDataSize)
{
	if (params->nvp > DT_VERTS_PER_POLYGON)
		return false;
	if (params->vertCount >= 0xffff)
		return false;
	if (!params->vertCount || !params->verts)
		return false;
	if (!params->polyCount || !params->polys)
		return false;

	const int nvp = params->nvp;
	
	// Classify off-mesh connection points. We store only the connections
	// whose start point is inside the tile.
	unsigned char* offMeshConClass = 0;
	int storedOffMeshConCount = 0;
	int offMeshConLinkCount = 0;
	
	if (params->offMeshConCount > 0)
	{
		offMeshConClass = (unsigned char*)dtAlloc(sizeof(unsigned char)*params->offMeshConCount*2, DT_ALLOC_TEMP);
		if (!offMeshConClass)
			return false;

		// Find tight heigh bounds, used for culling out off-mesh start locations.
		float hmin = FLT_MAX;
		float hmax = -FLT_MAX;
		
		if (params->detailVerts && params->detailVertsCount)
		{
			for (int i = 0; i < params->detailVertsCount; ++i)
			{
				const float h = params->detailVerts[i*3+1];
				hmin = min(hmin,h);
				hmax = max(hmax,h);
			}
		}
		else
		{
			for (int i = 0; i < params->vertCount; ++i)
			{
				const unsigned short* iv = &params->verts[i*3];
				const float h = params->bmin[1] + iv[1] * params->ch;
				hmin = min(hmin,h);
				hmax = max(hmax,h);
			}
		}
		hmin -= params->walkableClimb;
		hmax += params->walkableClimb;
		float bmin[3], bmax[3];
		dtVcopy(bmin, params->bmin);
		dtVcopy(bmax, params->bmax);
		bmin[1] = hmin;
		bmax[1] = hmax;

		for (int i = 0; i < params->offMeshConCount; ++i)
		{
			const float* p0 = &params->offMeshConVerts[(i*2+0)*3];
			const float* p1 = &params->offMeshConVerts[(i*2+1)*3];
			offMeshConClass[i*2+0] = classifyOffMeshPoint(p0, bmin, bmax);
			offMeshConClass[i*2+1] = classifyOffMeshPoint(p1, bmin, bmax);

			// Zero out off-mesh start positions which are not even potentially touching the mesh.
			if (offMeshConClass[i*2+0] == 0xff)
			{
				if (p0[1] < bmin[1] || p0[1] > bmax[1])
					offMeshConClass[i*2+0] = 0;
			}

			// Cound how many links should be allocated for off-mesh connections.
			if (offMeshConClass[i*2+0] == 0xff)
				offMeshConLinkCount++;
			if (offMeshConClass[i*2+1] == 0xff)
				offMeshConLinkCount++;

			if (offMeshConClass[i*2+0] == 0xff)
				storedOffMeshConCount++;
		}
	}
	
	// Off-mesh connections are stored as polygons, adjust values.
	const int totPolyCount = params->polyCount + storedOffMeshConCount;
	const int totVertCount = params->vertCount + storedOffMeshConCount*2;
	
	// Find portal edges which are at tile borders.
	int edgeCount = 0;
	int portalCount = 0;
	for (int i = 0; i < params->polyCount; ++i)
	{
		const unsigned short* p = &params->polys[i*2*nvp];
		for (int j = 0; j < nvp; ++j)
		{
			if (p[j] == MESH_NULL_IDX) break;
			edgeCount++;
			
			if (p[nvp+j] & 0x8000)
			{
				unsigned short dir = p[nvp+j] & 0xf;
				if (dir != 0xf)
					portalCount++;
			}
		}
	}

	const int maxLinkCount = edgeCount + portalCount*2 + offMeshConLinkCount*2;
	
	// Find unique detail vertices.
	int uniqueDetailVertCount = 0;
	int detailTriCount = 0;
	if (params->detailMeshes)
	{
		// Has detail mesh, count unique detail vertex count and use input detail tri count.
		detailTriCount = params->detailTriCount;
		for (int i = 0; i < params->polyCount; ++i)
		{
			const unsigned short* p = &params->polys[i*nvp*2];
			int ndv = params->detailMeshes[i*4+1];
			int nv = 0;
			for (int j = 0; j < nvp; ++j)
			{
				if (p[j] == MESH_NULL_IDX) break;
				nv++;
			}
			ndv -= nv;
			uniqueDetailVertCount += ndv;
		}
	}
	else
	{
		// No input detail mesh, build detail mesh from nav polys.
		uniqueDetailVertCount = 0; // No extra detail verts.
		detailTriCount = 0;
		for (int i = 0; i < params->polyCount; ++i)
		{
			const unsigned short* p = &params->polys[i*nvp*2];
			int nv = 0;
			for (int j = 0; j < nvp; ++j)
			{
				if (p[j] == MESH_NULL_IDX) break;
				nv++;
			}
			detailTriCount += nv-2;
		}
	}
	
	// Calculate data size
	const int headerSize = dtAlign4(sizeof(Moss_RecastMeshHeader));
	const int vertsSize = dtAlign4(sizeof(float)*3*totVertCount);
	const int polysSize = dtAlign4(sizeof(Moss_RecastPoly)*totPolyCount);
	const int linksSize = dtAlign4(sizeof(Moss_RecastLink)*maxLinkCount);
	const int detailMeshesSize = dtAlign4(sizeof(Moss_RecastPolyDetail)*params->polyCount);
	const int detailVertsSize = dtAlign4(sizeof(float)*3*uniqueDetailVertCount);
	const int detailTrisSize = dtAlign4(sizeof(unsigned char)*4*detailTriCount);
	const int bvTreeSize = params->buildBvTree ? dtAlign4(sizeof(Moss_RecastBVNode)*params->polyCount*2) : 0;
	const int offMeshConsSize = dtAlign4(sizeof(Moss_RecastOffMeshConnection)*storedOffMeshConCount);
	
	const int dataSize = headerSize + vertsSize + polysSize + linksSize +
						 detailMeshesSize + detailVertsSize + detailTrisSize +
						 bvTreeSize + offMeshConsSize;
						 
	unsigned char* data = (unsigned char*)dtAlloc(sizeof(unsigned char)*dataSize, DT_ALLOC_PERM);
	if (!data)
	{
		MOSS_FREE(offMeshConClass);
		return false;
	}
	memset(data, 0, dataSize);
	
	unsigned char* d = data;

	Moss_RecastMeshHeader* header = dtGetThenAdvanceBufferPointer<Moss_RecastMeshHeader>(d, headerSize);
	float* navVerts = dtGetThenAdvanceBufferPointer<float>(d, vertsSize);
	Moss_RecastPoly* navPolys = dtGetThenAdvanceBufferPointer<Moss_RecastPoly>(d, polysSize);
	d += linksSize; // Ignore links; just leave enough space for them. They'll be created on load.
	Moss_RecastPolyDetail* navDMeshes = dtGetThenAdvanceBufferPointer<Moss_RecastPolyDetail>(d, detailMeshesSize);
	float* navDVerts = dtGetThenAdvanceBufferPointer<float>(d, detailVertsSize);
	unsigned char* navDTris = dtGetThenAdvanceBufferPointer<unsigned char>(d, detailTrisSize);
	Moss_RecastBVNode* navBvtree = dtGetThenAdvanceBufferPointer<Moss_RecastBVNode>(d, bvTreeSize);
	Moss_RecastOffMeshConnection* offMeshCons = dtGetThenAdvanceBufferPointer<Moss_RecastOffMeshConnection>(d, offMeshConsSize);
	
	
	// Store header
	header->magic = DT_NAVMESH_MAGIC;
	header->version = DT_NAVMESH_VERSION;
	header->x = params->tileX;
	header->y = params->tileY;
	header->layer = params->tileLayer;
	header->userId = params->userId;
	header->polyCount = totPolyCount;
	header->vertCount = totVertCount;
	header->maxLinkCount = maxLinkCount;
	dtVcopy(header->bmin, params->bmin);
	dtVcopy(header->bmax, params->bmax);
	header->detailMeshCount = params->polyCount;
	header->detailVertCount = uniqueDetailVertCount;
	header->detailTriCount = detailTriCount;
	header->bvQuantFactor = 1.0f / params->cs;
	header->offMeshBase = params->polyCount;
	header->walkableHeight = params->walkableHeight;
	header->walkableRadius = params->walkableRadius;
	header->walkableClimb = params->walkableClimb;
	header->offMeshConCount = storedOffMeshConCount;
	header->bvNodeCount = params->buildBvTree ? params->polyCount*2 : 0;
	
	const int offMeshVertsBase = params->vertCount;
	const int offMeshPolyBase = params->polyCount;
	
	// Store vertices
	// Mesh vertices
	for (int i = 0; i < params->vertCount; ++i)
	{
		const unsigned short* iv = &params->verts[i*3];
		float* v = &navVerts[i*3];
		v[0] = params->bmin[0] + iv[0] * params->cs;
		v[1] = params->bmin[1] + iv[1] * params->ch;
		v[2] = params->bmin[2] + iv[2] * params->cs;
	}
	// Off-mesh link vertices.
	int n = 0;
	for (int i = 0; i < params->offMeshConCount; ++i)
	{
		// Only store connections which start from this tile.
		if (offMeshConClass[i*2+0] == 0xff)
		{
			const float* linkv = &params->offMeshConVerts[i*2*3];
			float* v = &navVerts[(offMeshVertsBase + n*2)*3];
			dtVcopy(&v[0], &linkv[0]);
			dtVcopy(&v[3], &linkv[3]);
			n++;
		}
	}
	
	// Store polygons
	// Mesh polys
	const unsigned short* src = params->polys;
	for (int i = 0; i < params->polyCount; ++i)
	{
		Moss_RecastPoly* p = &navPolys[i];
		p->vertCount = 0;
		p->flags = params->polyFlags[i];
		p->setArea(params->polyAreas[i]);
		p->setType(DT_POLYTYPE_GROUND);
		for (int j = 0; j < nvp; ++j)
		{
			if (src[j] == MESH_NULL_IDX) break;
			p->verts[j] = src[j];
			if (src[nvp+j] & 0x8000)
			{
				// Border or portal edge.
				unsigned short dir = src[nvp+j] & 0xf;
				if (dir == 0xf) // Border
					p->neis[j] = 0;
				else if (dir == 0) // Portal x-
					p->neis[j] = DT_EXT_LINK | 4;
				else if (dir == 1) // Portal z+
					p->neis[j] = DT_EXT_LINK | 2;
				else if (dir == 2) // Portal x+
					p->neis[j] = DT_EXT_LINK | 0;
				else if (dir == 3) // Portal z-
					p->neis[j] = DT_EXT_LINK | 6;
			}
			else
			{
				// Normal connection
				p->neis[j] = src[nvp+j]+1;
			}
			
			p->vertCount++;
		}
		src += nvp*2;
	}
	// Off-mesh connection vertices.
	n = 0;
	for (int i = 0; i < params->offMeshConCount; ++i)
	{
		// Only store connections which start from this tile.
		if (offMeshConClass[i*2+0] == 0xff)
		{
			Moss_RecastPoly* p = &navPolys[offMeshPolyBase+n];
			p->vertCount = 2;
			p->verts[0] = (unsigned short)(offMeshVertsBase + n*2+0);
			p->verts[1] = (unsigned short)(offMeshVertsBase + n*2+1);
			p->flags = params->offMeshConFlags[i];
			p->setArea(params->offMeshConAreas[i]);
			p->setType(DT_POLYTYPE_OFFMESH_CONNECTION);
			n++;
		}
	}

	// Store detail meshes and vertices.
	// The nav polygon vertices are stored as the first vertices on each mesh.
	// We compress the mesh data by skipping them and using the navmesh coordinates.
	if (params->detailMeshes)
	{
		unsigned short vbase = 0;
		for (int i = 0; i < params->polyCount; ++i)
		{
			Moss_RecastPolyDetail& dtl = navDMeshes[i];
			const int vb = (int)params->detailMeshes[i*4+0];
			const int ndv = (int)params->detailMeshes[i*4+1];
			const int nv = navPolys[i].vertCount;
			dtl.vertBase = (unsigned int)vbase;
			dtl.vertCount = (unsigned char)(ndv-nv);
			dtl.triBase = (unsigned int)params->detailMeshes[i*4+2];
			dtl.triCount = (unsigned char)params->detailMeshes[i*4+3];
			// Copy vertices except the first 'nv' verts which are equal to nav poly verts.
			if (ndv-nv)
			{
				memcpy(&navDVerts[vbase*3], &params->detailVerts[(vb+nv)*3], sizeof(float)*3*(ndv-nv));
				vbase += (unsigned short)(ndv-nv);
			}
		}
		// Store triangles.
		memcpy(navDTris, params->detailTris, sizeof(unsigned char)*4*params->detailTriCount);
	}
	else
	{
		// Create dummy detail mesh by triangulating polys.
		int tbase = 0;
		for (int i = 0; i < params->polyCount; ++i)
		{
			Moss_RecastPolyDetail& dtl = navDMeshes[i];
			const int nv = navPolys[i].vertCount;
			dtl.vertBase = 0;
			dtl.vertCount = 0;
			dtl.triBase = (unsigned int)tbase;
			dtl.triCount = (unsigned char)(nv-2);
			// Triangulate polygon (local indices).
			for (int j = 2; j < nv; ++j)
			{
				unsigned char* t = &navDTris[tbase*4];
				t[0] = 0;
				t[1] = (unsigned char)(j-1);
				t[2] = (unsigned char)j;
				// Bit for each edge that belongs to poly boundary.
				t[3] = (1<<2);
				if (j == 2) t[3] |= (1<<0);
				if (j == nv-1) t[3] |= (1<<4);
				tbase++;
			}
		}
	}

	// Store and create BVtree.
	if (params->buildBvTree)
	{
		createBVTree(params, navBvtree, 2*params->polyCount);
	}
	
	// Store Off-Mesh connections.
	n = 0;
	for (int i = 0; i < params->offMeshConCount; ++i)
	{
		// Only store connections which start from this tile.
		if (offMeshConClass[i*2+0] == 0xff)
		{
			Moss_RecastOffMeshConnection* con = &offMeshCons[n];
			con->poly = (unsigned short)(offMeshPolyBase + n);
			// Copy connection end-points.
			const float* endPts = &params->offMeshConVerts[i*2*3];
			dtVcopy(&con->pos[0], &endPts[0]);
			dtVcopy(&con->pos[3], &endPts[3]);
			con->rad = params->offMeshConRad[i];
			con->flags = params->offMeshConDir[i] ? DT_OFFMESH_CON_BIDIR : 0;
			con->side = offMeshConClass[i*2+1];
			if (params->offMeshConUserID)
				con->userId = params->offMeshConUserID[i];
			n++;
		}
	}
		
	MOSS_FREE(offMeshConClass);
	
	*outData = data;
	*outDataSize = dataSize;
	
	return true;
}

bool dtNavMeshHeaderSwapEndian(unsigned char* data, const int /*dataSize*/)
{
	Moss_RecastMeshHeader* header = (Moss_RecastMeshHeader*)data;
	
	int swappedMagic = DT_NAVMESH_MAGIC;
	int swappedVersion = DT_NAVMESH_VERSION;
	SwapEndian(&swappedMagic);
	SwapEndian(&swappedVersion);
	
	if ((header->magic != DT_NAVMESH_MAGIC || header->version != DT_NAVMESH_VERSION) &&
		(header->magic != swappedMagic || header->version != swappedVersion))
	{
		return false;
	}
		
	SwapEndian(&header->magic);
	SwapEndian(&header->version);
	SwapEndian(&header->x);
	SwapEndian(&header->y);
	SwapEndian(&header->layer);
	SwapEndian(&header->userId);
	SwapEndian(&header->polyCount);
	SwapEndian(&header->vertCount);
	SwapEndian(&header->maxLinkCount);
	SwapEndian(&header->detailMeshCount);
	SwapEndian(&header->detailVertCount);
	SwapEndian(&header->detailTriCount);
	SwapEndian(&header->bvNodeCount);
	SwapEndian(&header->offMeshConCount);
	SwapEndian(&header->offMeshBase);
	SwapEndian(&header->walkableHeight);
	SwapEndian(&header->walkableRadius);
	SwapEndian(&header->walkableClimb);
	SwapEndian(&header->bmin[0]);
	SwapEndian(&header->bmin[1]);
	SwapEndian(&header->bmin[2]);
	SwapEndian(&header->bmax[0]);
	SwapEndian(&header->bmax[1]);
	SwapEndian(&header->bmax[2]);
	SwapEndian(&header->bvQuantFactor);

	// Freelist index and pointers are updated when tile is added, no need to swap.

	return true;
}

/// @par
///
/// @warning This function assumes that the header is in the correct endianness already. 
/// Call #dtNavMeshHeaderSwapEndian() first on the data if the data is expected to be in wrong endianness 
/// to start with. Call #dtNavMeshHeaderSwapEndian() after the data has been swapped if converting from 
/// native to foreign endianness.
bool dtNavMeshDataSwapEndian(unsigned char* data, const int /*dataSize*/)
{
	// Make sure the data is in right format.
	Moss_RecastMeshHeader* header = (Moss_RecastMeshHeader*)data;
	if (header->magic != DT_NAVMESH_MAGIC)
		return false;
	if (header->version != DT_NAVMESH_VERSION)
		return false;
	
	// Patch header pointers.
	const int headerSize = dtAlign4(sizeof(Moss_RecastMeshHeader));
	const int vertsSize = dtAlign4(sizeof(float)*3*header->vertCount);
	const int polysSize = dtAlign4(sizeof(Moss_RecastPoly)*header->polyCount);
	const int linksSize = dtAlign4(sizeof(Moss_RecastLink)*(header->maxLinkCount));
	const int detailMeshesSize = dtAlign4(sizeof(Moss_RecastPolyDetail)*header->detailMeshCount);
	const int detailVertsSize = dtAlign4(sizeof(float)*3*header->detailVertCount);
	const int detailTrisSize = dtAlign4(sizeof(unsigned char)*4*header->detailTriCount);
	const int bvtreeSize = dtAlign4(sizeof(Moss_RecastBVNode)*header->bvNodeCount);
	const int offMeshLinksSize = dtAlign4(sizeof(Moss_RecastOffMeshConnection)*header->offMeshConCount);
	
	unsigned char* d = data + headerSize;
	float* verts = dtGetThenAdvanceBufferPointer<float>(d, vertsSize);
	Moss_RecastPoly* polys = dtGetThenAdvanceBufferPointer<Moss_RecastPoly>(d, polysSize);
	d += linksSize; // Ignore links; they technically should be endian-swapped but all their data is overwritten on load anyway.
	//Moss_RecastLink* links = dtGetThenAdvanceBufferPointer<Moss_RecastLink>(d, linksSize);
	Moss_RecastPolyDetail* detailMeshes = dtGetThenAdvanceBufferPointer<Moss_RecastPolyDetail>(d, detailMeshesSize);
	float* detailVerts = dtGetThenAdvanceBufferPointer<float>(d, detailVertsSize);
	d += detailTrisSize; // Ignore detail tris; single bytes can't be endian-swapped.
	//unsigned char* detailTris = dtGetThenAdvanceBufferPointer<unsigned char>(d, detailTrisSize);
	Moss_RecastBVNode* bvTree = dtGetThenAdvanceBufferPointer<Moss_RecastBVNode>(d, bvtreeSize);
	Moss_RecastOffMeshConnection* offMeshCons = dtGetThenAdvanceBufferPointer<Moss_RecastOffMeshConnection>(d, offMeshLinksSize);
	
	// Vertices
	for (int i = 0; i < header->vertCount*3; ++i)
	{
		SwapEndian(&verts[i]);
	}

	// Polys
	for (int i = 0; i < header->polyCount; ++i)
	{
		Moss_RecastPoly* p = &polys[i];
		// poly->firstLink is update when tile is added, no need to swap.
		for (int j = 0; j < DT_VERTS_PER_POLYGON; ++j)
		{
			SwapEndian(&p->verts[j]);
			SwapEndian(&p->neis[j]);
		}
		SwapEndian(&p->flags);
	}

	// Links are rebuild when tile is added, no need to swap.

	// Detail meshes
	for (int i = 0; i < header->detailMeshCount; ++i)
	{
		Moss_RecastPolyDetail* pd = &detailMeshes[i];
		SwapEndian(&pd->vertBase);
		SwapEndian(&pd->triBase);
	}
	
	// Detail verts
	for (int i = 0; i < header->detailVertCount*3; ++i)
	{
		SwapEndian(&detailVerts[i]);
	}

	// BV-tree
	for (int i = 0; i < header->bvNodeCount; ++i)
	{
		Moss_RecastBVNode* node = &bvTree[i];
		for (int j = 0; j < 3; ++j)
		{
			SwapEndian(&node->bmin[j]);
			SwapEndian(&node->bmax[j]);
		}
		SwapEndian(&node->i);
	}

	// Off-mesh Connections.
	for (int i = 0; i < header->offMeshConCount; ++i)
	{
		Moss_RecastOffMeshConnection* con = &offMeshCons[i];
		for (int j = 0; j < 6; ++j)
			SwapEndian(&con->pos[j]);
		SwapEndian(&con->rad);
		SwapEndian(&con->poly);
	}
	
	return true;
}


/*													*/



inline bool overlapSlabs(const float* amin, const float* amax,
						 const float* bmin, const float* bmax,
						 const float px, const float py)
{
	// Check for horizontal overlap.
	// The segment is shrunken a little so that slabs which touch
	// at end points are not connected.
	const float minx = max(amin[0]+px,bmin[0]+px);
	const float maxx = min(amax[0]-px,bmax[0]-px);
	if (minx > maxx)
		return false;
	
	// Check vertical overlap.
	const float ad = (amax[1]-amin[1]) / (amax[0]-amin[0]);
	const float ak = amin[1] - ad*amin[0];
	const float bd = (bmax[1]-bmin[1]) / (bmax[0]-bmin[0]);
	const float bk = bmin[1] - bd*bmin[0];
	const float aminy = ad*minx + ak;
	const float amaxy = ad*maxx + ak;
	const float bminy = bd*minx + bk;
	const float bmaxy = bd*maxx + bk;
	const float dmin = bminy - aminy;
	const float dmax = bmaxy - amaxy;
		
	// Crossing segments always overlap.
	if (dmin*dmax < 0)
		return true;
		
	// Check for overlap at endpoints.
	const float thr = Sqr(py*2);
	if (dmin*dmin <= thr || dmax*dmax <= thr)
		return true;
		
	return false;
}

static float getSlabCoord(const float* va, const int side)
{
	if (side == 0 || side == 4)
		return va[0];
	else if (side == 2 || side == 6)
		return va[2];
	return 0;
}

static void calcSlabEndPoints(const float* va, const float* vb, float* bmin, float* bmax, const int side)
{
	if (side == 0 || side == 4)
	{
		if (va[2] < vb[2])
		{
			bmin[0] = va[2];
			bmin[1] = va[1];
			bmax[0] = vb[2];
			bmax[1] = vb[1];
		}
		else
		{
			bmin[0] = vb[2];
			bmin[1] = vb[1];
			bmax[0] = va[2];
			bmax[1] = va[1];
		}
	}
	else if (side == 2 || side == 6)
	{
		if (va[0] < vb[0])
		{
			bmin[0] = va[0];
			bmin[1] = va[1];
			bmax[0] = vb[0];
			bmax[1] = vb[1];
		}
		else
		{
			bmin[0] = vb[0];
			bmin[1] = vb[1];
			bmax[0] = va[0];
			bmax[1] = va[1];
		}
	}
}

inline int computeTileHash(int x, int y, const int mask)
{
	const unsigned int h1 = 0x8da6b343; // Large multiplicative constants;
	const unsigned int h2 = 0xd8163841; // here arbitrarily chosen primes
	unsigned int n = h1 * x + h2 * y;
	return (int)(n & mask);
}

inline unsigned int allocLink(Moss_RecastMeshTile* tile)
{
	if (tile->linksFreeList == DT_NULL_LINK)
		return DT_NULL_LINK;
	unsigned int link = tile->linksFreeList;
	tile->linksFreeList = tile->links[link].next;
	return link;
}

inline void freeLink(Moss_RecastMeshTile* tile, unsigned int link)
{
	tile->links[link].next = tile->linksFreeList;
	tile->linksFreeList = link;
}


Moss_RecastNavMesh* dtAllocNavMesh()
{
	void* mem = dtAlloc(sizeof(Moss_RecastNavMesh), DT_ALLOC_PERM);
	if (!mem) return 0;
	return new(mem) Moss_RecastNavMesh;
}

/// @par
///
/// This function will only free the memory for tiles with the #DT_TILE_FREE_DATA
/// flag set.
void Moss_FreeNavMesh(Moss_RecastNavMesh* navmesh)
{
	if (!navmesh) return;
	navmesh->~Moss_RecastNavMesh();
	MOSS_FREE(navmesh);
}

//////////////////////////////////////////////////////////////////////////////////////////

/**
@class Moss_RecastNavMesh

The navigation mesh consists of one or more tiles defining three primary types of structural data:

A polygon mesh which defines most of the navigation graph. (See Moss_RecastPolyMesh for its structure.)
A detail mesh used for determining surface height on the polygon mesh. (See Moss_RecastPolyMeshDetail for its structure.)
Off-mesh connections, which define custom point-to-point edges within the navigation graph.

The general build process is as follows:

-# Create Moss_RecastPolyMesh and Moss_RecastPolyMeshDetail data using the Recast build pipeline.
-# Optionally, create off-mesh connection data.
-# Combine the source data into a Moss_RecastNavMeshCreateParams structure.
-# Create a tile data array using dtCreateNavMeshData().
-# Allocate at Moss_RecastNavMesh object and initialize it. (For single tile navigation meshes,
   the tile data is loaded during this step.)
-# For multi-tile navigation meshes, load the tile data using Moss_RecastNavMesh::addTile().

Notes:

- This class is usually used in conjunction with the Moss_RecastNavMeshQuery class for pathfinding.
- Technically, all navigation meshes are tiled. A 'solo' mesh is simply a navigation mesh initialized 
  to have only a single tile.
- This class does not implement any asynchronous methods. So the ::Moss_RecastStatus result of all methods will 
  always contain either a success or failure flag.

@see Moss_RecastNavMeshQuery, dtCreateNavMeshData, Moss_RecastNavMeshCreateParams, #dtAllocNavMesh, #Moss_FreeNavMesh
*/

Moss_RecastNavMesh::Moss_RecastNavMesh() :
	m_tileWidth(0),
	m_tileHeight(0),
	m_maxTiles(0),
	m_tileLutSize(0),
	m_tileLutMask(0),
	m_posLookup(0),
	m_nextFree(0),
	m_tiles(0)
{
#ifndef DT_POLYREF64
	m_saltBits = 0;
	m_tileBits = 0;
	m_polyBits = 0;
#endif
	memset(&m_params, 0, sizeof(Moss_RecastNavMeshParams));
	m_orig[0] = 0;
	m_orig[1] = 0;
	m_orig[2] = 0;
}

Moss_RecastNavMesh::~Moss_RecastNavMesh()
{
	for (int i = 0; i < m_maxTiles; ++i)
	{
		if (m_tiles[i].flags & DT_TILE_FREE_DATA)
		{
			MOSS_FREE(m_tiles[i].data);
			m_tiles[i].data = 0;
			m_tiles[i].dataSize = 0;
		}
	}
	MOSS_FREE(m_posLookup);
	MOSS_FREE(m_tiles);
}
		
Moss_RecastStatus Moss_RecastNavMesh::init(const Moss_RecastNavMeshParams* params)
{
	memcpy(&m_params, params, sizeof(Moss_RecastNavMeshParams));
	dtVcopy(m_orig, params->orig);
	m_tileWidth = params->tileWidth;
	m_tileHeight = params->tileHeight;
	
	// Init tiles
	m_maxTiles = params->maxTiles;
	m_tileLutSize = dtNextPow2(params->maxTiles/4);
	if (!m_tileLutSize) m_tileLutSize = 1;
	m_tileLutMask = m_tileLutSize-1;
	
	m_tiles = (Moss_RecastMeshTile*)dtAlloc(sizeof(Moss_RecastMeshTile)*m_maxTiles, DT_ALLOC_PERM);
	if (!m_tiles)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	m_posLookup = (Moss_RecastMeshTile**)dtAlloc(sizeof(Moss_RecastMeshTile*)*m_tileLutSize, DT_ALLOC_PERM);
	if (!m_posLookup)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(m_tiles, 0, sizeof(Moss_RecastMeshTile)*m_maxTiles);
	memset(m_posLookup, 0, sizeof(Moss_RecastMeshTile*)*m_tileLutSize);
	m_nextFree = 0;
	for (int i = m_maxTiles-1; i >= 0; --i)
	{
		m_tiles[i].salt = 1;
		m_tiles[i].next = m_nextFree;
		m_nextFree = &m_tiles[i];
	}
	
	// Init ID generator values.
#ifndef DT_POLYREF64
	m_tileBits = dtIlog2(dtNextPow2((unsigned int)params->maxTiles));
	m_polyBits = dtIlog2(dtNextPow2((unsigned int)params->maxPolys));

    // Ensure that the bits of poly ref do not overflow.
    dtAssert(m_tileBits + m_polyBits <= 31);

	// Only allow 31 salt bits, since the salt mask is calculated using 32bit uint and it will overflow.
	m_saltBits = min((unsigned int)31, 32 - m_tileBits - m_polyBits);

	if (m_saltBits < 10)
		return DT_FAILURE | DT_INVALID_PARAM;
#endif
	
	return DT_SUCCESS;
}

Moss_RecastStatus Moss_RecastNavMesh::init(unsigned char* data, const int dataSize, const int flags)
{
	// Make sure the data is in right format.
	Moss_RecastMeshHeader* header = (Moss_RecastMeshHeader*)data;
	if (header->magic != DT_NAVMESH_MAGIC)
		return DT_FAILURE | DT_WRONG_MAGIC;
	if (header->version != DT_NAVMESH_VERSION)
		return DT_FAILURE | DT_WRONG_VERSION;

	Moss_RecastNavMeshParams params;
	dtVcopy(params.orig, header->bmin);
	params.tileWidth = header->bmax[0] - header->bmin[0];
	params.tileHeight = header->bmax[2] - header->bmin[2];
	params.maxTiles = 1;
	params.maxPolys = header->polyCount;
	
	Moss_RecastStatus status = init(&params);
	if (dtStatusFailed(status))
		return status;

	return addTile(data, dataSize, flags, 0, 0);
}

/// @par
///
/// @note The parameters are created automatically when the single tile
/// initialization is performed.
const Moss_RecastNavMeshParams* Moss_RecastNavMesh::getParams() const
{
	return &m_params;
}

//////////////////////////////////////////////////////////////////////////////////////////
int Moss_RecastNavMesh::findConnectingPolys(const float* va, const float* vb,
								   const Moss_RecastMeshTile* tile, int side,
								   dtPolyRef* con, float* conarea, int maxcon) const
{
	if (!tile) return 0;
	
	float amin[2], amax[2];
	calcSlabEndPoints(va, vb, amin, amax, side);
	const float apos = getSlabCoord(va, side);

	// Remove links pointing to 'side' and compact the links array. 
	float bmin[2], bmax[2];
	unsigned short m = DT_EXT_LINK | (unsigned short)side;
	int n = 0;
	
	dtPolyRef base = getPolyRefBase(tile);
	
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		Moss_RecastPoly* poly = &tile->polys[i];
		const int nv = poly->vertCount;
		for (int j = 0; j < nv; ++j)
		{
			// Skip edges which do not point to the right side.
			if (poly->neis[j] != m) continue;
			
			const float* vc = &tile->verts[poly->verts[j]*3];
			const float* vd = &tile->verts[poly->verts[(j+1) % nv]*3];
			const float bpos = getSlabCoord(vc, side);
			
			// Segments are not close enough.
			if (abs(apos-bpos) > 0.01f)
				continue;
			
			// Check if the segments touch.
			calcSlabEndPoints(vc,vd, bmin,bmax, side);
			
			if (!overlapSlabs(amin,amax, bmin,bmax, 0.01f, tile->header->walkableClimb)) continue;
			
			// Add return value.
			if (n < maxcon)
			{
				conarea[n*2+0] = max(amin[0], bmin[0]);
				conarea[n*2+1] = min(amax[0], bmax[0]);
				con[n] = base | (dtPolyRef)i;
				n++;
			}
			break;
		}
	}
	return n;
}

void Moss_RecastNavMesh::unconnectLinks(Moss_RecastMeshTile* tile, Moss_RecastMeshTile* target)
{
	if (!tile || !target) return;

	const unsigned int targetNum = decodePolyIdTile(getTileRef(target));

	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		Moss_RecastPoly* poly = &tile->polys[i];
		unsigned int j = poly->firstLink;
		unsigned int pj = DT_NULL_LINK;
		while (j != DT_NULL_LINK)
		{
			if (decodePolyIdTile(tile->links[j].ref) == targetNum)
			{
				// Remove link.
				unsigned int nj = tile->links[j].next;
				if (pj == DT_NULL_LINK)
					poly->firstLink = nj;
				else
					tile->links[pj].next = nj;
				freeLink(tile, j);
				j = nj;
			}
			else
			{
				// Advance
				pj = j;
				j = tile->links[j].next;
			}
		}
	}
}

void Moss_RecastNavMesh::connectExtLinks(Moss_RecastMeshTile* tile, Moss_RecastMeshTile* target, int side)
{
	if (!tile) return;
	
	// Connect border links.
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		Moss_RecastPoly* poly = &tile->polys[i];

		// Create new links.
//		unsigned short m = DT_EXT_LINK | (unsigned short)side;
		
		const int nv = poly->vertCount;
		for (int j = 0; j < nv; ++j)
		{
			// Skip non-portal edges.
			if ((poly->neis[j] & DT_EXT_LINK) == 0)
				continue;
			
			const int dir = (int)(poly->neis[j] & 0xff);
			if (side != -1 && dir != side)
				continue;
			
			// Create new links
			const float* va = &tile->verts[poly->verts[j]*3];
			const float* vb = &tile->verts[poly->verts[(j+1) % nv]*3];
			dtPolyRef nei[4];
			float neia[4*2];
			int nnei = findConnectingPolys(va,vb, target, dtOppositeTile(dir), nei,neia,4);
			for (int k = 0; k < nnei; ++k)
			{
				unsigned int idx = allocLink(tile);
				if (idx != DT_NULL_LINK)
				{
					Moss_RecastLink* link = &tile->links[idx];
					link->ref = nei[k];
					link->edge = (unsigned char)j;
					link->side = (unsigned char)dir;
					
					link->next = poly->firstLink;
					poly->firstLink = idx;

					// Compress portal limits to a byte value.
					if (dir == 0 || dir == 4)
					{
						float tmin = (neia[k*2+0]-va[2]) / (vb[2]-va[2]);
						float tmax = (neia[k*2+1]-va[2]) / (vb[2]-va[2]);
						if (tmin > tmax)
							Swap(tmin,tmax);
						link->bmin = (unsigned char)roundf(Clamp(tmin, 0.0f, 1.0f)*255.0f);
						link->bmax = (unsigned char)roundf(Clamp(tmax, 0.0f, 1.0f)*255.0f);
					}
					else if (dir == 2 || dir == 6)
					{
						float tmin = (neia[k*2+0]-va[0]) / (vb[0]-va[0]);
						float tmax = (neia[k*2+1]-va[0]) / (vb[0]-va[0]);
						if (tmin > tmax)
							Swap(tmin,tmax);
						link->bmin = (unsigned char)roundf(Clamp(tmin, 0.0f, 1.0f)*255.0f);
						link->bmax = (unsigned char)roundf(Clamp(tmax, 0.0f, 1.0f)*255.0f);
					}
				}
			}
		}
	}
}

void Moss_RecastNavMesh::connectExtOffMeshLinks(Moss_RecastMeshTile* tile, Moss_RecastMeshTile* target, int side)
{
	if (!tile) return;
	
	// Connect off-mesh links.
	// We are interested on links which land from target tile to this tile.
	const unsigned char oppositeSide = (side == -1) ? 0xff : (unsigned char)dtOppositeTile(side);
	
	for (int i = 0; i < target->header->offMeshConCount; ++i)
	{
		Moss_RecastOffMeshConnection* targetCon = &target->offMeshCons[i];
		if (targetCon->side != oppositeSide)
			continue;

		Moss_RecastPoly* targetPoly = &target->polys[targetCon->poly];
		// Skip off-mesh connections which start location could not be connected at all.
		if (targetPoly->firstLink == DT_NULL_LINK)
			continue;
		
		const float halfExtents[3] = { targetCon->rad, target->header->walkableClimb, targetCon->rad };
		
		// Find polygon to connect to.
		const float* p = &targetCon->pos[3];
		float nearestPt[3];
		dtPolyRef ref = findNearestPolyInTile(tile, p, halfExtents, nearestPt);
		if (!ref)
			continue;
		// findNearestPoly may return too optimistic results, further check to make sure. 
		if (Sqr(nearestPt[0]-p[0])+Sqr(nearestPt[2]-p[2]) > Sqr(targetCon->rad))
			continue;
		// Make sure the location is on current mesh.
		float* v = &target->verts[targetPoly->verts[1]*3];
		dtVcopy(v, nearestPt);
				
		// Link off-mesh connection to target poly.
		unsigned int idx = allocLink(target);
		if (idx != DT_NULL_LINK)
		{
			Moss_RecastLink* link = &target->links[idx];
			link->ref = ref;
			link->edge = (unsigned char)1;
			link->side = oppositeSide;
			link->bmin = link->bmax = 0;
			// Add to linked list.
			link->next = targetPoly->firstLink;
			targetPoly->firstLink = idx;
		}
		
		// Link target poly to off-mesh connection.
		if (targetCon->flags & DT_OFFMESH_CON_BIDIR)
		{
			unsigned int tidx = allocLink(tile);
			if (tidx != DT_NULL_LINK)
			{
				const unsigned short landPolyIdx = (unsigned short)decodePolyIdPoly(ref);
				Moss_RecastPoly* landPoly = &tile->polys[landPolyIdx];
				Moss_RecastLink* link = &tile->links[tidx];
				link->ref = getPolyRefBase(target) | (dtPolyRef)(targetCon->poly);
				link->edge = 0xff;
				link->side = (unsigned char)(side == -1 ? 0xff : side);
				link->bmin = link->bmax = 0;
				// Add to linked list.
				link->next = landPoly->firstLink;
				landPoly->firstLink = tidx;
			}
		}
	}

}

void Moss_RecastNavMesh::connectIntLinks(Moss_RecastMeshTile* tile)
{
	if (!tile) return;

	dtPolyRef base = getPolyRefBase(tile);

	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		Moss_RecastPoly* poly = &tile->polys[i];
		poly->firstLink = DT_NULL_LINK;

		if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
			continue;
			
		// Build edge links backwards so that the links will be
		// in the linked list from lowest index to highest.
		for (int j = poly->vertCount-1; j >= 0; --j)
		{
			// Skip hard and non-internal edges.
			if (poly->neis[j] == 0 || (poly->neis[j] & DT_EXT_LINK)) continue;

			unsigned int idx = allocLink(tile);
			if (idx != DT_NULL_LINK)
			{
				Moss_RecastLink* link = &tile->links[idx];
				link->ref = base | (dtPolyRef)(poly->neis[j]-1);
				link->edge = (unsigned char)j;
				link->side = 0xff;
				link->bmin = link->bmax = 0;
				// Add to linked list.
				link->next = poly->firstLink;
				poly->firstLink = idx;
			}
		}			
	}
}

void Moss_RecastNavMesh::baseOffMeshLinks(Moss_RecastMeshTile* tile)
{
	if (!tile) return;
	
	dtPolyRef base = getPolyRefBase(tile);
	
	// Base off-mesh connection start points.
	for (int i = 0; i < tile->header->offMeshConCount; ++i)
	{
		Moss_RecastOffMeshConnection* con = &tile->offMeshCons[i];
		Moss_RecastPoly* poly = &tile->polys[con->poly];
	
		const float halfExtents[3] = { con->rad, tile->header->walkableClimb, con->rad };
		
		// Find polygon to connect to.
		const float* p = &con->pos[0]; // First vertex
		float nearestPt[3];
		dtPolyRef ref = findNearestPolyInTile(tile, p, halfExtents, nearestPt);
		if (!ref) continue;
		// findNearestPoly may return too optimistic results, further check to make sure. 
		if (Sqr(nearestPt[0]-p[0])+Sqr(nearestPt[2]-p[2]) > Sqr(con->rad))
			continue;
		// Make sure the location is on current mesh.
		float* v = &tile->verts[poly->verts[0]*3];
		dtVcopy(v, nearestPt);

		// Link off-mesh connection to target poly.
		unsigned int idx = allocLink(tile);
		if (idx != DT_NULL_LINK)
		{
			Moss_RecastLink* link = &tile->links[idx];
			link->ref = ref;
			link->edge = (unsigned char)0;
			link->side = 0xff;
			link->bmin = link->bmax = 0;
			// Add to linked list.
			link->next = poly->firstLink;
			poly->firstLink = idx;
		}

		// Start end-point is always connect back to off-mesh connection. 
		unsigned int tidx = allocLink(tile);
		if (tidx != DT_NULL_LINK)
		{
			const unsigned short landPolyIdx = (unsigned short)decodePolyIdPoly(ref);
			Moss_RecastPoly* landPoly = &tile->polys[landPolyIdx];
			Moss_RecastLink* link = &tile->links[tidx];
			link->ref = base | (dtPolyRef)(con->poly);
			link->edge = 0xff;
			link->side = 0xff;
			link->bmin = link->bmax = 0;
			// Add to linked list.
			link->next = landPoly->firstLink;
			landPoly->firstLink = tidx;
		}
	}
}

namespace
{
	template<bool onlyBoundary>
	void closestPointOnDetailEdges(const Moss_RecastMeshTile* tile, const Moss_RecastPoly* poly, const float* pos, float* closest)
	{
		const unsigned int ip = (unsigned int)(poly - tile->polys);
		const Moss_RecastPolyDetail* pd = &tile->detailMeshes[ip];

		float dmin = FLT_MAX;
		float tmin = 0;
		const float* pmin = 0;
		const float* pmax = 0;

		for (int i = 0; i < pd->triCount; i++)
		{
			const unsigned char* tris = &tile->detailTris[(pd->triBase + i) * 4];
			const int ANY_BOUNDARY_EDGE =
				(DT_DETAIL_EDGE_BOUNDARY << 0) |
				(DT_DETAIL_EDGE_BOUNDARY << 2) |
				(DT_DETAIL_EDGE_BOUNDARY << 4);
			if (onlyBoundary && (tris[3] & ANY_BOUNDARY_EDGE) == 0)
				continue;

			const float* v[3];
			for (int j = 0; j < 3; ++j)
			{
				if (tris[j] < poly->vertCount)
					v[j] = &tile->verts[poly->verts[tris[j]] * 3];
				else
					v[j] = &tile->detailVerts[(pd->vertBase + (tris[j] - poly->vertCount)) * 3];
			}

			for (int k = 0, j = 2; k < 3; j = k++)
			{
				if ((dtGetDetailTriEdgeFlags(tris[3], j) & DT_DETAIL_EDGE_BOUNDARY) == 0 &&
					(onlyBoundary || tris[j] < tris[k]))
				{
					// Only looking at boundary edges and this is internal, or
					// this is an inner edge that we will see again or have already seen.
					continue;
				}

				float t;
				float d = dtDistancePtSegSqr2D(pos, v[j], v[k], t);
				if (d < dmin)
				{
					dmin = d;
					tmin = t;
					pmin = v[j];
					pmax = v[k];
				}
			}
		}

		dtVlerp(closest, pmin, pmax, tmin);
	}
}

bool Moss_RecastNavMesh::getPolyHeight(const Moss_RecastMeshTile* tile, const Moss_RecastPoly* poly, const float* pos, float* height) const
{
	// Off-mesh connections do not have detail polys and getting height
	// over them does not make sense.
	if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
		return false;

	const unsigned int ip = (unsigned int)(poly - tile->polys);
	const Moss_RecastPolyDetail* pd = &tile->detailMeshes[ip];
	
	float verts[DT_VERTS_PER_POLYGON*3];	
	const int nv = poly->vertCount;
	for (int i = 0; i < nv; ++i)
		dtVcopy(&verts[i*3], &tile->verts[poly->verts[i]*3]);
	
	if (!dtPointInPolygon(pos, verts, nv))
		return false;

	if (!height)
		return true;
	
	// Find height at the location.
	for (int j = 0; j < pd->triCount; ++j)
	{
		const unsigned char* t = &tile->detailTris[(pd->triBase+j)*4];
		const float* v[3];
		for (int k = 0; k < 3; ++k)
		{
			if (t[k] < poly->vertCount)
				v[k] = &tile->verts[poly->verts[t[k]]*3];
			else
				v[k] = &tile->detailVerts[(pd->vertBase+(t[k]-poly->vertCount))*3];
		}
		float h;
		if (dtClosestHeightPointTriangle(pos, v[0], v[1], v[2], h))
		{
			*height = h;
			return true;
		}
	}

	// If all triangle checks failed above (can happen with degenerate triangles
	// or larger floating point values) the point is on an edge, so just select
	// closest. This should almost never happen so the extra iteration here is
	// ok.
	float closest[3];
	closestPointOnDetailEdges<false>(tile, poly, pos, closest);
	*height = closest[1];
	return true;
}

void Moss_RecastNavMesh::closestPointOnPoly(dtPolyRef ref, const float* pos, float* closest, bool* posOverPoly) const
{
	const Moss_RecastMeshTile* tile = 0;
	const Moss_RecastPoly* poly = 0;
	getTileAndPolyByRefUnsafe(ref, &tile, &poly);

	dtVcopy(closest, pos);
	if (getPolyHeight(tile, poly, pos, &closest[1]))
	{
		if (posOverPoly)
			*posOverPoly = true;
		return;
	}

	if (posOverPoly)
		*posOverPoly = false;

	// Off-mesh connections don't have detail polygons.
	if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
	{
		const float* v0 = &tile->verts[poly->verts[0]*3];
		const float* v1 = &tile->verts[poly->verts[1]*3];
		float t;
		dtDistancePtSegSqr2D(pos, v0, v1, t);
		dtVlerp(closest, v0, v1, t);
		return;
	}

	// Outside poly that is not an offmesh connection.
	closestPointOnDetailEdges<true>(tile, poly, pos, closest);
}

dtPolyRef Moss_RecastNavMesh::findNearestPolyInTile(const Moss_RecastMeshTile* tile,
										   const float* center, const float* halfExtents,
										   float* nearestPt) const
{
	float bmin[3], bmax[3];
	dtVsub(bmin, center, halfExtents);
	dtVadd(bmax, center, halfExtents);
	
	// Get nearby polygons from proximity grid.
	dtPolyRef polys[128];
	int polyCount = queryPolygonsInTile(tile, bmin, bmax, polys, 128);
	
	// Find nearest polygon amongst the nearby polygons.
	dtPolyRef nearest = 0;
	float nearestDistanceSqr = FLT_MAX;
	for (int i = 0; i < polyCount; ++i)
	{
		dtPolyRef ref = polys[i];
		float closestPtPoly[3];
		float diff[3];
		bool posOverPoly = false;
		float d;
		closestPointOnPoly(ref, center, closestPtPoly, &posOverPoly);

		// If a point is directly over a polygon and closer than
		// climb height, favor that instead of straight line nearest point.
		dtVsub(diff, center, closestPtPoly);
		if (posOverPoly)
		{
			d = abs(diff[1]) - tile->header->walkableClimb;
			d = d > 0 ? d*d : 0;			
		}
		else
		{
			d = dtVlenSqr(diff);
		}
		
		if (d < nearestDistanceSqr)
		{
			dtVcopy(nearestPt, closestPtPoly);
			nearestDistanceSqr = d;
			nearest = ref;
		}
	}
	
	return nearest;
}

int Moss_RecastNavMesh::queryPolygonsInTile(const Moss_RecastMeshTile* tile, const float* qmin, const float* qmax,
								   dtPolyRef* polys, const int maxPolys) const
{
	if (tile->bvTree)
	{
		const Moss_RecastBVNode* node = &tile->bvTree[0];
		const Moss_RecastBVNode* end = &tile->bvTree[tile->header->bvNodeCount];
		const float* tbmin = tile->header->bmin;
		const float* tbmax = tile->header->bmax;
		const float qfac = tile->header->bvQuantFactor;
		
		// Calculate quantized box
		unsigned short bmin[3], bmax[3];
		// Clamp query box to world box.
		float minx = Clamp(qmin[0], tbmin[0], tbmax[0]) - tbmin[0];
		float miny = Clamp(qmin[1], tbmin[1], tbmax[1]) - tbmin[1];
		float minz = Clamp(qmin[2], tbmin[2], tbmax[2]) - tbmin[2];
		float maxx = Clamp(qmax[0], tbmin[0], tbmax[0]) - tbmin[0];
		float maxy = Clamp(qmax[1], tbmin[1], tbmax[1]) - tbmin[1];
		float maxz = Clamp(qmax[2], tbmin[2], tbmax[2]) - tbmin[2];
		// Quantize
		bmin[0] = (unsigned short)(qfac * minx) & 0xfffe;
		bmin[1] = (unsigned short)(qfac * miny) & 0xfffe;
		bmin[2] = (unsigned short)(qfac * minz) & 0xfffe;
		bmax[0] = (unsigned short)(qfac * maxx + 1) | 1;
		bmax[1] = (unsigned short)(qfac * maxy + 1) | 1;
		bmax[2] = (unsigned short)(qfac * maxz + 1) | 1;
		
		// Traverse tree
		dtPolyRef base = getPolyRefBase(tile);
		int n = 0;
		while (node < end)
		{
			const bool overlap = dtOverlapQuantBounds(bmin, bmax, node->bmin, node->bmax);
			const bool isLeafNode = node->i >= 0;
			
			if (isLeafNode && overlap)
			{
				if (n < maxPolys)
					polys[n++] = base | (dtPolyRef)node->i;
			}
			
			if (overlap || isLeafNode)
				node++;
			else
			{
				const int escapeIndex = -node->i;
				node += escapeIndex;
			}
		}
		
		return n;
	}
	else
	{
		float bmin[3], bmax[3];
		int n = 0;
		dtPolyRef base = getPolyRefBase(tile);
		for (int i = 0; i < tile->header->polyCount; ++i)
		{
			Moss_RecastPoly* p = &tile->polys[i];
			// Do not return off-mesh connection polygons.
			if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
				continue;
			// Calc polygon bounds.
			const float* v = &tile->verts[p->verts[0]*3];
			dtVcopy(bmin, v);
			dtVcopy(bmax, v);
			for (int j = 1; j < p->vertCount; ++j)
			{
				v = &tile->verts[p->verts[j]*3];
				min(bmin, v);
				max(bmax, v);
			}
			if (dtOverlapBounds(qmin,qmax, bmin,bmax))
			{
				if (n < maxPolys)
					polys[n++] = base | (dtPolyRef)i;
			}
		}
		return n;
	}
}

/// @par
///
/// The add operation will fail if the data is in the wrong format, the allocated tile
/// space is full, or there is a tile already at the specified reference.
///
/// The lastRef parameter is used to restore a tile with the same tile
/// reference it had previously used.  In this case the #dtPolyRef's for the
/// tile will be restored to the same values they were before the tile was 
/// removed.
///
/// The nav mesh assumes exclusive access to the data passed and will make
/// changes to the dynamic portion of the data. For that reason the data
/// should not be reused in other nav meshes until the tile has been successfully
/// removed from this nav mesh.
///
/// @see dtCreateNavMeshData, #removeTile
Moss_RecastStatus Moss_RecastNavMesh::addTile(unsigned char* data, int dataSize, int flags,
							Moss_RecastTileRef lastRef, Moss_RecastTileRef* result)
{
	// Make sure the data is in right format.
	Moss_RecastMeshHeader* header = (Moss_RecastMeshHeader*)data;
	if (header->magic != DT_NAVMESH_MAGIC)
		return DT_FAILURE | DT_WRONG_MAGIC;
	if (header->version != DT_NAVMESH_VERSION)
		return DT_FAILURE | DT_WRONG_VERSION;

#ifndef DT_POLYREF64
	// Do not allow adding more polygons than specified in the NavMesh's maxPolys constraint.
	// Otherwise, the poly ID cannot be represented with the given number of bits.
	if (m_polyBits < dtIlog2(dtNextPow2((unsigned int)header->polyCount)))
		return DT_FAILURE | DT_INVALID_PARAM;
#endif
		
	// Make sure the location is free.
	if (getTileAt(header->x, header->y, header->layer))
		return DT_FAILURE | DT_ALREADY_OCCUPIED;
		
	// Allocate a tile.
	Moss_RecastMeshTile* tile = 0;
	if (!lastRef)
	{
		if (m_nextFree)
		{
			tile = m_nextFree;
			m_nextFree = tile->next;
			tile->next = 0;
		}
	}
	else
	{
		// Try to relocate the tile to specific index with same salt.
		int tileIndex = (int)decodePolyIdTile((dtPolyRef)lastRef);
		if (tileIndex >= m_maxTiles)
			return DT_FAILURE | DT_OUT_OF_MEMORY;
		// Try to find the specific tile id from the free list.
		Moss_RecastMeshTile* target = &m_tiles[tileIndex];
		Moss_RecastMeshTile* prev = 0;
		tile = m_nextFree;
		while (tile && tile != target)
		{
			prev = tile;
			tile = tile->next;
		}
		// Could not find the correct location.
		if (tile != target)
			return DT_FAILURE | DT_OUT_OF_MEMORY;
		// Remove from freelist
		if (!prev)
			m_nextFree = tile->next;
		else
			prev->next = tile->next;

		// Restore salt.
		tile->salt = decodePolyIdSalt((dtPolyRef)lastRef);
	}

	// Make sure we could allocate a tile.
	if (!tile)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	
	// Insert tile into the position lut.
	int h = computeTileHash(header->x, header->y, m_tileLutMask);
	tile->next = m_posLookup[h];
	m_posLookup[h] = tile;
	
	// Patch header pointers.
	const int headerSize = dtAlign4(sizeof(Moss_RecastMeshHeader));
	const int vertsSize = dtAlign4(sizeof(float)*3*header->vertCount);
	const int polysSize = dtAlign4(sizeof(Moss_RecastPoly)*header->polyCount);
	const int linksSize = dtAlign4(sizeof(Moss_RecastLink)*(header->maxLinkCount));
	const int detailMeshesSize = dtAlign4(sizeof(Moss_RecastPolyDetail)*header->detailMeshCount);
	const int detailVertsSize = dtAlign4(sizeof(float)*3*header->detailVertCount);
	const int detailTrisSize = dtAlign4(sizeof(unsigned char)*4*header->detailTriCount);
	const int bvtreeSize = dtAlign4(sizeof(Moss_RecastBVNode)*header->bvNodeCount);
	const int offMeshLinksSize = dtAlign4(sizeof(Moss_RecastOffMeshConnection)*header->offMeshConCount);
	
	unsigned char* d = data + headerSize;
	tile->verts = dtGetThenAdvanceBufferPointer<float>(d, vertsSize);
	tile->polys = dtGetThenAdvanceBufferPointer<Moss_RecastPoly>(d, polysSize);
	tile->links = dtGetThenAdvanceBufferPointer<Moss_RecastLink>(d, linksSize);
	tile->detailMeshes = dtGetThenAdvanceBufferPointer<Moss_RecastPolyDetail>(d, detailMeshesSize);
	tile->detailVerts = dtGetThenAdvanceBufferPointer<float>(d, detailVertsSize);
	tile->detailTris = dtGetThenAdvanceBufferPointer<unsigned char>(d, detailTrisSize);
	tile->bvTree = dtGetThenAdvanceBufferPointer<Moss_RecastBVNode>(d, bvtreeSize);
	tile->offMeshCons = dtGetThenAdvanceBufferPointer<Moss_RecastOffMeshConnection>(d, offMeshLinksSize);

	// If there are no items in the bvtree, reset the tree pointer.
	if (!bvtreeSize)
		tile->bvTree = 0;

	// Build links freelist
	tile->linksFreeList = 0;
	tile->links[header->maxLinkCount-1].next = DT_NULL_LINK;
	for (int i = 0; i < header->maxLinkCount-1; ++i)
		tile->links[i].next = i+1;

	// Init tile.
	tile->header = header;
	tile->data = data;
	tile->dataSize = dataSize;
	tile->flags = flags;

	connectIntLinks(tile);

	// Base off-mesh connections to their starting polygons and connect connections inside the tile.
	baseOffMeshLinks(tile);
	connectExtOffMeshLinks(tile, tile, -1);

	// Create connections with neighbour tiles.
	static const int MAX_NEIS = 32;
	Moss_RecastMeshTile* neis[MAX_NEIS];
	int nneis;
	
	// Connect with layers in current tile.
	nneis = getTilesAt(header->x, header->y, neis, MAX_NEIS);
	for (int j = 0; j < nneis; ++j)
	{
		if (neis[j] == tile)
			continue;
	
		connectExtLinks(tile, neis[j], -1);
		connectExtLinks(neis[j], tile, -1);
		connectExtOffMeshLinks(tile, neis[j], -1);
		connectExtOffMeshLinks(neis[j], tile, -1);
	}
	
	// Connect with neighbour tiles.
	for (int i = 0; i < 8; ++i)
	{
		nneis = getNeighbourTilesAt(header->x, header->y, i, neis, MAX_NEIS);
		for (int j = 0; j < nneis; ++j)
		{
			connectExtLinks(tile, neis[j], i);
			connectExtLinks(neis[j], tile, dtOppositeTile(i));
			connectExtOffMeshLinks(tile, neis[j], i);
			connectExtOffMeshLinks(neis[j], tile, dtOppositeTile(i));
		}
	}
	
	if (result)
		*result = getTileRef(tile);
	
	return DT_SUCCESS;
}

const Moss_RecastMeshTile* Moss_RecastNavMesh::getTileAt(const int x, const int y, const int layer) const
{
	// Find tile based on hash.
	int h = computeTileHash(x,y,m_tileLutMask);
	Moss_RecastMeshTile* tile = m_posLookup[h];
	while (tile)
	{
		if (tile->header &&
			tile->header->x == x &&
			tile->header->y == y &&
			tile->header->layer == layer)
		{
			return tile;
		}
		tile = tile->next;
	}
	return 0;
}

int Moss_RecastNavMesh::getNeighbourTilesAt(const int x, const int y, const int side, Moss_RecastMeshTile** tiles, const int maxTiles) const
{
	int nx = x, ny = y;
	switch (side)
	{
		case 0: nx++; break;
		case 1: nx++; ny++; break;
		case 2: ny++; break;
		case 3: nx--; ny++; break;
		case 4: nx--; break;
		case 5: nx--; ny--; break;
		case 6: ny--; break;
		case 7: nx++; ny--; break;
	};

	return getTilesAt(nx, ny, tiles, maxTiles);
}

int Moss_RecastNavMesh::getTilesAt(const int x, const int y, Moss_RecastMeshTile** tiles, const int maxTiles) const
{
	int n = 0;
	
	// Find tile based on hash.
	int h = computeTileHash(x,y,m_tileLutMask);
	Moss_RecastMeshTile* tile = m_posLookup[h];
	while (tile)
	{
		if (tile->header &&
			tile->header->x == x &&
			tile->header->y == y)
		{
			if (n < maxTiles)
				tiles[n++] = tile;
		}
		tile = tile->next;
	}
	
	return n;
}

/// @par
///
/// This function will not fail if the tiles array is too small to hold the
/// entire result set.  It will simply fill the array to capacity.
int Moss_RecastNavMesh::getTilesAt(const int x, const int y, Moss_RecastMeshTile const** tiles, const int maxTiles) const
{
	int n = 0;
	
	// Find tile based on hash.
	int h = computeTileHash(x,y,m_tileLutMask);
	Moss_RecastMeshTile* tile = m_posLookup[h];
	while (tile)
	{
		if (tile->header &&
			tile->header->x == x &&
			tile->header->y == y)
		{
			if (n < maxTiles)
				tiles[n++] = tile;
		}
		tile = tile->next;
	}
	
	return n;
}


Moss_RecastTileRef Moss_RecastNavMesh::getTileRefAt(const int x, const int y, const int layer) const
{
	// Find tile based on hash.
	int h = computeTileHash(x,y,m_tileLutMask);
	Moss_RecastMeshTile* tile = m_posLookup[h];
	while (tile)
	{
		if (tile->header &&
			tile->header->x == x &&
			tile->header->y == y &&
			tile->header->layer == layer)
		{
			return getTileRef(tile);
		}
		tile = tile->next;
	}
	return 0;
}

const Moss_RecastMeshTile* Moss_RecastNavMesh::getTileByRef(Moss_RecastTileRef ref) const
{
	if (!ref)
		return 0;
	unsigned int tileIndex = decodePolyIdTile((dtPolyRef)ref);
	unsigned int tileSalt = decodePolyIdSalt((dtPolyRef)ref);
	if ((int)tileIndex >= m_maxTiles)
		return 0;
	const Moss_RecastMeshTile* tile = &m_tiles[tileIndex];
	if (tile->salt != tileSalt)
		return 0;
	return tile;
}

int Moss_RecastNavMesh::getMaxTiles() const
{
	return m_maxTiles;
}

Moss_RecastMeshTile* Moss_RecastNavMesh::getTile(int i)
{
	return &m_tiles[i];
}

const Moss_RecastMeshTile* Moss_RecastNavMesh::getTile(int i) const
{
	return &m_tiles[i];
}

void Moss_RecastNavMesh::calcTileLoc(const float* pos, int* tx, int* ty) const
{
	*tx = (int)floorf((pos[0]-m_orig[0]) / m_tileWidth);
	*ty = (int)floorf((pos[2]-m_orig[2]) / m_tileHeight);
}

Moss_RecastStatus Moss_RecastNavMesh::getTileAndPolyByRef(const dtPolyRef ref, const Moss_RecastMeshTile** tile, const Moss_RecastPoly** poly) const
{
	if (!ref) return DT_FAILURE;
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return DT_FAILURE | DT_INVALID_PARAM;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return DT_FAILURE | DT_INVALID_PARAM;
	if (ip >= (unsigned int)m_tiles[it].header->polyCount) return DT_FAILURE | DT_INVALID_PARAM;
	*tile = &m_tiles[it];
	*poly = &m_tiles[it].polys[ip];
	return DT_SUCCESS;
}

/// @par
///
/// @warning Only use this function if it is known that the provided polygon
/// reference is valid. This function is faster than #getTileAndPolyByRef, but
/// it does not validate the reference.
void Moss_RecastNavMesh::getTileAndPolyByRefUnsafe(const dtPolyRef ref, const Moss_RecastMeshTile** tile, const Moss_RecastPoly** poly) const
{
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	*tile = &m_tiles[it];
	*poly = &m_tiles[it].polys[ip];
}

bool Moss_RecastNavMesh::isValidPolyRef(dtPolyRef ref) const
{
	if (!ref) return false;
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return false;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return false;
	if (ip >= (unsigned int)m_tiles[it].header->polyCount) return false;
	return true;
}

/// @par
///
/// This function returns the data for the tile so that, if desired,
/// it can be added back to the navigation mesh at a later point.
///
/// @see #addTile
Moss_RecastStatus Moss_RecastNavMesh::removeTile(Moss_RecastTileRef ref, unsigned char** data, int* dataSize)
{
	if (!ref)
		return DT_FAILURE | DT_INVALID_PARAM;
	unsigned int tileIndex = decodePolyIdTile((dtPolyRef)ref);
	unsigned int tileSalt = decodePolyIdSalt((dtPolyRef)ref);
	if ((int)tileIndex >= m_maxTiles)
		return DT_FAILURE | DT_INVALID_PARAM;
	Moss_RecastMeshTile* tile = &m_tiles[tileIndex];
	if (tile->salt != tileSalt)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	// Remove tile from hash lookup.
	int h = computeTileHash(tile->header->x,tile->header->y,m_tileLutMask);
	Moss_RecastMeshTile* prev = 0;
	Moss_RecastMeshTile* cur = m_posLookup[h];
	while (cur)
	{
		if (cur == tile)
		{
			if (prev)
				prev->next = cur->next;
			else
				m_posLookup[h] = cur->next;
			break;
		}
		prev = cur;
		cur = cur->next;
	}
	
	// Remove connections to neighbour tiles.
	static const int MAX_NEIS = 32;
	Moss_RecastMeshTile* neis[MAX_NEIS];
	int nneis;
	
	// Disconnect from other layers in current tile.
	nneis = getTilesAt(tile->header->x, tile->header->y, neis, MAX_NEIS);
	for (int j = 0; j < nneis; ++j)
	{
		if (neis[j] == tile) continue;
		unconnectLinks(neis[j], tile);
	}
	
	// Disconnect from neighbour tiles.
	for (int i = 0; i < 8; ++i)
	{
		nneis = getNeighbourTilesAt(tile->header->x, tile->header->y, i, neis, MAX_NEIS);
		for (int j = 0; j < nneis; ++j)
			unconnectLinks(neis[j], tile);
	}
		
	// Reset tile.
	if (tile->flags & DT_TILE_FREE_DATA)
	{
		// Owns data
		MOSS_FREE(tile->data);
		tile->data = 0;
		tile->dataSize = 0;
		if (data) *data = 0;
		if (dataSize) *dataSize = 0;
	}
	else
	{
		if (data) *data = tile->data;
		if (dataSize) *dataSize = tile->dataSize;
	}

	tile->header = 0;
	tile->flags = 0;
	tile->linksFreeList = 0;
	tile->polys = 0;
	tile->verts = 0;
	tile->links = 0;
	tile->detailMeshes = 0;
	tile->detailVerts = 0;
	tile->detailTris = 0;
	tile->bvTree = 0;
	tile->offMeshCons = 0;

	// Update salt, salt should never be zero.
#ifdef DT_POLYREF64
	tile->salt = (tile->salt+1) & ((1<<DT_SALT_BITS)-1);
#else
	tile->salt = (tile->salt+1) & ((1<<m_saltBits)-1);
#endif
	if (tile->salt == 0)
		tile->salt++;

	// Add to free list.
	tile->next = m_nextFree;
	m_nextFree = tile;

	return DT_SUCCESS;
}

Moss_RecastTileRef Moss_RecastNavMesh::getTileRef(const Moss_RecastMeshTile* tile) const
{
	if (!tile) return 0;
	const unsigned int it = (unsigned int)(tile - m_tiles);
	return (Moss_RecastTileRef)encodePolyId(tile->salt, it, 0);
}

/// @par
///
/// Example use case:
/// @code
///
/// const dtPolyRef base = navmesh->getPolyRefBase(tile);
/// for (int i = 0; i < tile->header->polyCount; ++i)
/// {
///     const Moss_RecastPoly* p = &tile->polys[i];
///     const dtPolyRef ref = base | (dtPolyRef)i;
///     
///     // Use the reference to access the polygon data.
/// }
/// @endcode
dtPolyRef Moss_RecastNavMesh::getPolyRefBase(const Moss_RecastMeshTile* tile) const
{
	if (!tile) return 0;
	const unsigned int it = (unsigned int)(tile - m_tiles);
	return encodePolyId(tile->salt, it, 0);
}

struct dtTileState
{
	int magic;								// Magic number, used to identify the data.
	int version;							// Data version number.
	Moss_RecastTileRef ref;							// Tile ref at the time of storing the data.
};

struct dtPolyState
{
	unsigned short flags;						// Flags (see dtPolyFlags).
	unsigned char area;							// Area ID of the polygon.
};

///  @see #storeTileState
int Moss_RecastNavMesh::getTileStateSize(const Moss_RecastMeshTile* tile) const
{
	if (!tile) return 0;
	const int headerSize = dtAlign4(sizeof(dtTileState));
	const int polyStateSize = dtAlign4(sizeof(dtPolyState) * tile->header->polyCount);
	return headerSize + polyStateSize;
}

/// @par
///
/// Tile state includes non-structural data such as polygon flags, area ids, etc.
/// @note The state data is only valid until the tile reference changes.
/// @see #getTileStateSize, #restoreTileState
Moss_RecastStatus Moss_RecastNavMesh::storeTileState(const Moss_RecastMeshTile* tile, unsigned char* data, const int maxDataSize) const
{
	// Make sure there is enough space to store the state.
	const int sizeReq = getTileStateSize(tile);
	if (maxDataSize < sizeReq)
		return DT_FAILURE | DT_BUFFER_TOO_SMALL;
		
	dtTileState* tileState = dtGetThenAdvanceBufferPointer<dtTileState>(data, dtAlign4(sizeof(dtTileState)));
	dtPolyState* polyStates = dtGetThenAdvanceBufferPointer<dtPolyState>(data, dtAlign4(sizeof(dtPolyState) * tile->header->polyCount));
	
	// Store tile state.
	tileState->magic = DT_NAVMESH_STATE_MAGIC;
	tileState->version = DT_NAVMESH_STATE_VERSION;
	tileState->ref = getTileRef(tile);
	
	// Store per poly state.
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		const Moss_RecastPoly* p = &tile->polys[i];
		dtPolyState* s = &polyStates[i];
		s->flags = p->flags;
		s->area = p->getArea();
	}
	
	return DT_SUCCESS;
}

/// @par
///
/// Tile state includes non-structural data such as polygon flags, area ids, etc.
/// @note This function does not impact the tile's #Moss_RecastTileRef and #dtPolyRef's.
/// @see #storeTileState
Moss_RecastStatus Moss_RecastNavMesh::restoreTileState(Moss_RecastMeshTile* tile, const unsigned char* data, const int maxDataSize)
{
	// Make sure there is enough space to store the state.
	const int sizeReq = getTileStateSize(tile);
	if (maxDataSize < sizeReq)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	const dtTileState* tileState = dtGetThenAdvanceBufferPointer<const dtTileState>(data, dtAlign4(sizeof(dtTileState)));
	const dtPolyState* polyStates = dtGetThenAdvanceBufferPointer<const dtPolyState>(data, dtAlign4(sizeof(dtPolyState) * tile->header->polyCount));
	
	// Check that the restore is possible.
	if (tileState->magic != DT_NAVMESH_STATE_MAGIC)
		return DT_FAILURE | DT_WRONG_MAGIC;
	if (tileState->version != DT_NAVMESH_STATE_VERSION)
		return DT_FAILURE | DT_WRONG_VERSION;
	if (tileState->ref != getTileRef(tile))
		return DT_FAILURE | DT_INVALID_PARAM;
	
	// Restore per poly state.
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		Moss_RecastPoly* p = &tile->polys[i];
		const dtPolyState* s = &polyStates[i];
		p->flags = s->flags;
		p->setArea(s->area);
	}
	
	return DT_SUCCESS;
}

/// @par
///
/// Off-mesh connections are stored in the navigation mesh as special 2-vertex 
/// polygons with a single edge. At least one of the vertices is expected to be 
/// inside a normal polygon. So an off-mesh connection is "entered" from a 
/// normal polygon at one of its endpoints. This is the polygon identified by 
/// the prevRef parameter.
Moss_RecastStatus Moss_RecastNavMesh::getOffMeshConnectionPolyEndPoints(dtPolyRef prevRef, dtPolyRef polyRef, float* startPos, float* endPos) const
{
	unsigned int salt, it, ip;

	if (!polyRef)
		return DT_FAILURE;
	
	// Get current polygon
	decodePolyId(polyRef, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return DT_FAILURE | DT_INVALID_PARAM;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return DT_FAILURE | DT_INVALID_PARAM;
	const Moss_RecastMeshTile* tile = &m_tiles[it];
	if (ip >= (unsigned int)tile->header->polyCount) return DT_FAILURE | DT_INVALID_PARAM;
	const Moss_RecastPoly* poly = &tile->polys[ip];

	// Make sure that the current poly is indeed off-mesh link.
	if (poly->getType() != DT_POLYTYPE_OFFMESH_CONNECTION)
		return DT_FAILURE;

	// Figure out which way to hand out the vertices.
	int idx0 = 0, idx1 = 1;
	
	// Find link that points to first vertex.
	for (unsigned int i = poly->firstLink; i != DT_NULL_LINK; i = tile->links[i].next)
	{
		if (tile->links[i].edge == 0)
		{
			if (tile->links[i].ref != prevRef)
			{
				idx0 = 1;
				idx1 = 0;
			}
			break;
		}
	}
	
	dtVcopy(startPos, &tile->verts[poly->verts[idx0]*3]);
	dtVcopy(endPos, &tile->verts[poly->verts[idx1]*3]);

	return DT_SUCCESS;
}


const Moss_RecastOffMeshConnection* Moss_RecastNavMesh::getOffMeshConnectionByRef(dtPolyRef ref) const
{
	unsigned int salt, it, ip;
	
	if (!ref)
		return 0;
	
	// Get current polygon
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return 0;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return 0;
	const Moss_RecastMeshTile* tile = &m_tiles[it];
	if (ip >= (unsigned int)tile->header->polyCount) return 0;
	const Moss_RecastPoly* poly = &tile->polys[ip];
	
	// Make sure that the current poly is indeed off-mesh link.
	if (poly->getType() != DT_POLYTYPE_OFFMESH_CONNECTION)
		return 0;

	const unsigned int idx =  ip - tile->header->offMeshBase;
	dtAssert(idx < (unsigned int)tile->header->offMeshConCount);
	return &tile->offMeshCons[idx];
}


Moss_RecastStatus Moss_RecastNavMesh::setPolyFlags(dtPolyRef ref, unsigned short flags)
{
	if (!ref) return DT_FAILURE;
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return DT_FAILURE | DT_INVALID_PARAM;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return DT_FAILURE | DT_INVALID_PARAM;
	Moss_RecastMeshTile* tile = &m_tiles[it];
	if (ip >= (unsigned int)tile->header->polyCount) return DT_FAILURE | DT_INVALID_PARAM;
	Moss_RecastPoly* poly = &tile->polys[ip];
	
	// Change flags.
	poly->flags = flags;
	
	return DT_SUCCESS;
}

Moss_RecastStatus Moss_RecastNavMesh::getPolyFlags(dtPolyRef ref, unsigned short* resultFlags) const
{
	if (!ref) return DT_FAILURE;
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return DT_FAILURE | DT_INVALID_PARAM;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return DT_FAILURE | DT_INVALID_PARAM;
	const Moss_RecastMeshTile* tile = &m_tiles[it];
	if (ip >= (unsigned int)tile->header->polyCount) return DT_FAILURE | DT_INVALID_PARAM;
	const Moss_RecastPoly* poly = &tile->polys[ip];

	*resultFlags = poly->flags;
	
	return DT_SUCCESS;
}

Moss_RecastStatus Moss_RecastNavMesh::setPolyArea(dtPolyRef ref, unsigned char area)
{
	if (!ref) return DT_FAILURE;
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return DT_FAILURE | DT_INVALID_PARAM;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return DT_FAILURE | DT_INVALID_PARAM;
	Moss_RecastMeshTile* tile = &m_tiles[it];
	if (ip >= (unsigned int)tile->header->polyCount) return DT_FAILURE | DT_INVALID_PARAM;
	Moss_RecastPoly* poly = &tile->polys[ip];
	
	poly->setArea(area);
	
	return DT_SUCCESS;
}

Moss_RecastStatus Moss_RecastNavMesh::getPolyArea(dtPolyRef ref, unsigned char* resultArea) const
{
	if (!ref) return DT_FAILURE;
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return DT_FAILURE | DT_INVALID_PARAM;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return DT_FAILURE | DT_INVALID_PARAM;
	const Moss_RecastMeshTile* tile = &m_tiles[it];
	if (ip >= (unsigned int)tile->header->polyCount) return DT_FAILURE | DT_INVALID_PARAM;
	const Moss_RecastPoly* poly = &tile->polys[ip];
	
	*resultArea = poly->getArea();
	
	return DT_SUCCESS;
}



/*													*/

void dtClosestPtPointTriangle(float* closest, const float* p,
							  const float* a, const float* b, const float* c)
{
	// Check if P in vertex region outside A
	float ab[3], ac[3], ap[3];
	dtVsub(ab, b, a);
	dtVsub(ac, c, a);
	dtVsub(ap, p, a);
	float d1 = dtVdot(ab, ap);
	float d2 = dtVdot(ac, ap);
	if (d1 <= 0.0f && d2 <= 0.0f)
	{
		// barycentric coordinates (1,0,0)
		dtVcopy(closest, a);
		return;
	}
	
	// Check if P in vertex region outside B
	float bp[3];
	dtVsub(bp, p, b);
	float d3 = dtVdot(ab, bp);
	float d4 = dtVdot(ac, bp);
	if (d3 >= 0.0f && d4 <= d3)
	{
		// barycentric coordinates (0,1,0)
		dtVcopy(closest, b);
		return;
	}
	
	// Check if P in edge region of AB, if so return projection of P onto AB
	float vc = d1*d4 - d3*d2;
	if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
	{
		// barycentric coordinates (1-v,v,0)
		float v = d1 / (d1 - d3);
		closest[0] = a[0] + v * ab[0];
		closest[1] = a[1] + v * ab[1];
		closest[2] = a[2] + v * ab[2];
		return;
	}
	
	// Check if P in vertex region outside C
	float cp[3];
	dtVsub(cp, p, c);
	float d5 = dtVdot(ab, cp);
	float d6 = dtVdot(ac, cp);
	if (d6 >= 0.0f && d5 <= d6)
	{
		// barycentric coordinates (0,0,1)
		dtVcopy(closest, c);
		return;
	}
	
	// Check if P in edge region of AC, if so return projection of P onto AC
	float vb = d5*d2 - d1*d6;
	if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
	{
		// barycentric coordinates (1-w,0,w)
		float w = d2 / (d2 - d6);
		closest[0] = a[0] + w * ac[0];
		closest[1] = a[1] + w * ac[1];
		closest[2] = a[2] + w * ac[2];
		return;
	}
	
	// Check if P in edge region of BC, if so return projection of P onto BC
	float va = d3*d6 - d5*d4;
	if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
	{
		// barycentric coordinates (0,1-w,w)
		float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		closest[0] = b[0] + w * (c[0] - b[0]);
		closest[1] = b[1] + w * (c[1] - b[1]);
		closest[2] = b[2] + w * (c[2] - b[2]);
		return;
	}
	
	// P inside face region. Compute Q through its barycentric coordinates (u,v,w)
	float denom = 1.0f / (va + vb + vc);
	float v = vb * denom;
	float w = vc * denom;
	closest[0] = a[0] + ab[0] * v + ac[0] * w;
	closest[1] = a[1] + ab[1] * v + ac[1] * w;
	closest[2] = a[2] + ab[2] * v + ac[2] * w;
}

bool dtIntersectSegmentPoly2D(const float* p0, const float* p1,
							  const float* verts, int nverts,
							  float& tmin, float& tmax,
							  int& segMin, int& segMax)
{
	static const float EPS = 0.000001f;
	
	tmin = 0;
	tmax = 1;
	segMin = -1;
	segMax = -1;
	
	float dir[3];
	dtVsub(dir, p1, p0);
	
	for (int i = 0, j = nverts-1; i < nverts; j=i++)
	{
		float edge[3], diff[3];
		dtVsub(edge, &verts[i*3], &verts[j*3]);
		dtVsub(diff, p0, &verts[j*3]);
		const float n = dtVperp2D(edge, diff);
		const float d = dtVperp2D(dir, edge);
		if (fabsf(d) < EPS)
		{
			// S is nearly parallel to this edge
			if (n < 0)
				return false;
			else
				continue;
		}
		const float t = n / d;
		if (d < 0)
		{
			// segment S is entering across this edge
			if (t > tmin)
			{
				tmin = t;
				segMin = j;
				// S enters after leaving polygon
				if (tmin > tmax)
					return false;
			}
		}
		else
		{
			// segment S is leaving across this edge
			if (t < tmax)
			{
				tmax = t;
				segMax = j;
				// S leaves before entering polygon
				if (tmax < tmin)
					return false;
			}
		}
	}
	
	return true;
}

float dtDistancePtSegSqr2D(const float* pt, const float* p, const float* q, float& t)
{
	float pqx = q[0] - p[0];
	float pqz = q[2] - p[2];
	float dx = pt[0] - p[0];
	float dz = pt[2] - p[2];
	float d = pqx*pqx + pqz*pqz;
	t = pqx*dx + pqz*dz;
	if (d > 0) t /= d;
	if (t < 0) t = 0;
	else if (t > 1) t = 1;
	dx = p[0] + t*pqx - pt[0];
	dz = p[2] + t*pqz - pt[2];
	return dx*dx + dz*dz;
}

void dtCalcPolyCenter(float* tc, const unsigned short* idx, int nidx, const float* verts)
{
	tc[0] = 0.0f;
	tc[1] = 0.0f;
	tc[2] = 0.0f;
	for (int j = 0; j < nidx; ++j)
	{
		const float* v = &verts[idx[j]*3];
		tc[0] += v[0];
		tc[1] += v[1];
		tc[2] += v[2];
	}
	const float s = 1.0f / nidx;
	tc[0] *= s;
	tc[1] *= s;
	tc[2] *= s;
}

bool dtClosestHeightPointTriangle(const float* p, const float* a, const float* b, const float* c, float& h)
{
	const float EPS = 1e-6f;
	float v0[3], v1[3], v2[3];

	dtVsub(v0, c, a);
	dtVsub(v1, b, a);
	dtVsub(v2, p, a);

	// Compute scaled barycentric coordinates
	float denom = v0[0] * v1[2] - v0[2] * v1[0];
	if (fabsf(denom) < EPS)
		return false;

	float u = v1[2] * v2[0] - v1[0] * v2[2];
	float v = v0[0] * v2[2] - v0[2] * v2[0];

	if (denom < 0) {
		denom = -denom;
		u = -u;
		v = -v;
	}

	// If point lies inside the triangle, return interpolated ycoord.
	if (u >= 0.0f && v >= 0.0f && (u + v) <= denom) {
		h = a[1] + (v0[1] * u + v1[1] * v) / denom;
		return true;
	}
	return false;
}

/// @par
///
/// All points are projected onto the xz-plane, so the y-values are ignored.
bool dtPointInPolygon(const float* pt, const float* verts, const int nverts)
{
	// TODO: Replace pnpoly with triArea2D tests?
	int i, j;
	bool c = false;
	for (i = 0, j = nverts-1; i < nverts; j = i++)
	{
		const float* vi = &verts[i*3];
		const float* vj = &verts[j*3];
		if (((vi[2] > pt[2]) != (vj[2] > pt[2])) &&
			(pt[0] < (vj[0]-vi[0]) * (pt[2]-vi[2]) / (vj[2]-vi[2]) + vi[0]) )
			c = !c;
	}
	return c;
}

bool dtDistancePtPolyEdgesSqr(const float* pt, const float* verts, const int nverts,
							  float* ed, float* et)
{
	// TODO: Replace pnpoly with triArea2D tests?
	int i, j;
	bool c = false;
	for (i = 0, j = nverts-1; i < nverts; j = i++)
	{
		const float* vi = &verts[i*3];
		const float* vj = &verts[j*3];
		if (((vi[2] > pt[2]) != (vj[2] > pt[2])) &&
			(pt[0] < (vj[0]-vi[0]) * (pt[2]-vi[2]) / (vj[2]-vi[2]) + vi[0]) )
			c = !c;
		ed[j] = dtDistancePtSegSqr2D(pt, vj, vi, et[j]);
	}
	return c;
}

static void projectPoly(const float* axis, const float* poly, const int npoly,
						float& rmin, float& rmax)
{
	rmin = rmax = dtVdot2D(axis, &poly[0]);
	for (int i = 1; i < npoly; ++i)
	{
		const float d = dtVdot2D(axis, &poly[i*3]);
		rmin = min(rmin, d);
		rmax = max(rmax, d);
	}
}

inline bool overlapRange(const float amin, const float amax,
						 const float bmin, const float bmax,
						 const float eps)
{
	return ((amin+eps) > bmax || (amax-eps) < bmin) ? false : true;
}

/// @par
///
/// All vertices are projected onto the xz-plane, so the y-values are ignored.
bool dtOverlapPolyPoly2D(const float* polya, const int npolya,
						 const float* polyb, const int npolyb)
{
	const float eps = 1e-4f;
	
	for (int i = 0, j = npolya-1; i < npolya; j=i++)
	{
		const float* va = &polya[j*3];
		const float* vb = &polya[i*3];
		const float n[3] = { vb[2]-va[2], 0, -(vb[0]-va[0]) };
		float amin,amax,bmin,bmax;
		projectPoly(n, polya, npolya, amin,amax);
		projectPoly(n, polyb, npolyb, bmin,bmax);
		if (!overlapRange(amin,amax, bmin,bmax, eps))
		{
			// Found separating axis
			return false;
		}
	}
	for (int i = 0, j = npolyb-1; i < npolyb; j=i++)
	{
		const float* va = &polyb[j*3];
		const float* vb = &polyb[i*3];
		const float n[3] = { vb[2]-va[2], 0, -(vb[0]-va[0]) };
		float amin,amax,bmin,bmax;
		projectPoly(n, polya, npolya, amin,amax);
		projectPoly(n, polyb, npolyb, bmin,bmax);
		if (!overlapRange(amin,amax, bmin,bmax, eps))
		{
			// Found separating axis
			return false;
		}
	}
	return true;
}

// Returns a random point in a convex polygon.
// Adapted from Graphics Gems article.
void dtRandomPointInConvexPoly(const float* pts, const int npts, float* areas,
							   const float s, const float t, float* out)
{
	// Calc triangle araes
	float areasum = 0.0f;
	for (int i = 2; i < npts; i++) {
		areas[i] = dtTriArea2D(&pts[0], &pts[(i-1)*3], &pts[i*3]);
		areasum += max(0.001f, areas[i]);
	}
	// Find sub triangle weighted by area.
	const float thr = s*areasum;
	float acc = 0.0f;
	float u = 1.0f;
	int tri = npts - 1;
	for (int i = 2; i < npts; i++) {
		const float dacc = areas[i];
		if (thr >= acc && thr < (acc+dacc))
		{
			u = (thr - acc) / dacc;
			tri = i;
			break;
		}
		acc += dacc;
	}
	
	float v = sqrtf(t);
	
	const float a = 1 - v;
	const float b = (1 - u) * v;
	const float c = u * v;
	const float* pa = &pts[0];
	const float* pb = &pts[(tri-1)*3];
	const float* pc = &pts[tri*3];
	
	out[0] = a*pa[0] + b*pb[0] + c*pc[0];
	out[1] = a*pa[1] + b*pb[1] + c*pc[1];
	out[2] = a*pa[2] + b*pb[2] + c*pc[2];
}

inline float vperpXZ(const float* a, const float* b) { return a[0]*b[2] - a[2]*b[0]; }

bool dtIntersectSegSeg2D(const float* ap, const float* aq,
						 const float* bp, const float* bq,
						 float& s, float& t)
{
	float u[3], v[3], w[3];
	dtVsub(u,aq,ap);
	dtVsub(v,bq,bp);
	dtVsub(w,ap,bp);
	float d = vperpXZ(u,v);
	if (fabsf(d) < 1e-6f) return false;
	s = vperpXZ(v,w) / d;
	t = vperpXZ(u,w) / d;
	return true;
}



/*													*/

Moss_RecastQueryFilter::Moss_RecastQueryFilter() :
	m_includeFlags(0xffff),
	m_excludeFlags(0)
{
	for (int i = 0; i < DT_MAX_AREAS; ++i)
		m_areaCost[i] = 1.0f;
}

#ifdef DT_VIRTUAL_QUERYFILTER
bool Moss_RecastQueryFilter::passFilter(const dtPolyRef /*ref*/,
							   const Moss_RecastMeshTile* /*tile*/,
							   const Moss_RecastPoly* poly) const
{
	return (poly->flags & m_includeFlags) != 0 && (poly->flags & m_excludeFlags) == 0;
}

float Moss_RecastQueryFilter::getCost(const float* pa, const float* pb,
							 const dtPolyRef /*prevRef*/, const Moss_RecastMeshTile* /*prevTile*/, const Moss_RecastPoly* /*prevPoly*/,
							 const dtPolyRef /*curRef*/, const Moss_RecastMeshTile* /*curTile*/, const Moss_RecastPoly* curPoly,
							 const dtPolyRef /*nextRef*/, const Moss_RecastMeshTile* /*nextTile*/, const Moss_RecastPoly* /*nextPoly*/) const
{
	return rcVdist(pa, pb) * m_areaCost[curPoly->getArea()];
}
#else
inline bool Moss_RecastQueryFilter::passFilter(const dtPolyRef /*ref*/,
									  const Moss_RecastMeshTile* /*tile*/,
									  const Moss_RecastPoly* poly) const
{
	return (poly->flags & m_includeFlags) != 0 && (poly->flags & m_excludeFlags) == 0;
}

inline float Moss_RecastQueryFilter::getCost(const float* pa, const float* pb,
									const dtPolyRef /*prevRef*/, const Moss_RecastMeshTile* /*prevTile*/, const Moss_RecastPoly* /*prevPoly*/,
									const dtPolyRef /*curRef*/, const Moss_RecastMeshTile* /*curTile*/, const Moss_RecastPoly* curPoly,
									const dtPolyRef /*nextRef*/, const Moss_RecastMeshTile* /*nextTile*/, const Moss_RecastPoly* /*nextPoly*/) const
{
	return rcVdist(pa, pb) * m_areaCost[curPoly->getArea()];
}
#endif	
	
static const float H_SCALE = 0.999f; // Search heuristic scale.


Moss_RecastNavMeshQuery* dtAllocNavMeshQuery()
{
	void* mem = dtAlloc(sizeof(Moss_RecastNavMeshQuery), DT_ALLOC_PERM);
	if (!mem) return 0;
	return new(mem) Moss_RecastNavMeshQuery;
}

void Moss_FreeNavMeshQuery(Moss_RecastNavMeshQuery* navmesh)
{
	if (!navmesh) return;
	navmesh->~Moss_RecastNavMeshQuery();
	MOSS_FREE(navmesh);
}

Moss_RecastPolyQuery::~Moss_RecastPolyQuery()
{
	// Defined out of line to fix the weak v-tables warning
}

//////////////////////////////////////////////////////////////////////////////////////////

/// @class Moss_RecastNavMeshQuery
///
/// For methods that support undersized buffers, if the buffer is too small 
/// to hold the entire result set the return status of the method will include 
/// the #DT_BUFFER_TOO_SMALL flag.
///
/// Constant member functions can be used by multiple clients without side
/// effects. (E.g. No change to the closed list. No impact on an in-progress
/// sliced path query. Etc.)
/// 
/// Walls and portals: A @e wall is a polygon segment that is 
/// considered impassable. A @e portal is a passable segment between polygons.
/// A portal may be treated as a wall based on the Moss_RecastQueryFilter used for a query.
///
/// @see Moss_RecastNavMesh, Moss_RecastQueryFilter, #dtAllocNavMeshQuery(), #dtAllocNavMeshQuery()

Moss_RecastNavMeshQuery::Moss_RecastNavMeshQuery() : m_nav(0), m_tinyNodePool(0), m_nodePool(0), m_openList(0) { memset(&m_query, 0, sizeof(dtQueryData)); }

Moss_RecastNavMeshQuery::~Moss_RecastNavMeshQuery() {
	if (m_tinyNodePool) { m_tinyNodePool->~Moss_RecastNodePool(); }
	if (m_nodePool) { m_nodePool->~Moss_RecastNodePool(); }
	if (m_openList) { m_openList->~Moss_RecastNodeQueue(); }
	MOSS_FREE(m_tinyNodePool);
	MOSS_FREE(m_nodePool);
	MOSS_FREE(m_openList);
}

/// @par 
///
/// Must be the first function called after construction, before other
/// functions are used.
///
/// This function can be used multiple times.
Moss_RecastStatus Moss_RecastNavMeshQuery::init(const Moss_RecastNavMesh* nav, const int maxNodes) {
	if (maxNodes > DT_NULL_IDX || maxNodes > (1 << DT_NODE_PARENT_BITS) - 1)
		return DT_FAILURE | DT_INVALID_PARAM;

	m_nav = nav;
	
	if (!m_nodePool || m_nodePool->getMaxNodes() < maxNodes) {
		if (m_nodePool) {
			m_nodePool->~Moss_RecastNodePool();
			MOSS_FREE(m_nodePool);
			m_nodePool = 0;
		}
		m_nodePool = new (dtAlloc(sizeof(Moss_RecastNodePool), DT_ALLOC_PERM)) Moss_RecastNodePool(maxNodes, dtNextPow2(maxNodes/4));
		if (!m_nodePool)
			return DT_FAILURE | DT_OUT_OF_MEMORY;
	}
	else {
		m_nodePool->clear();
	}
	
	if (!m_tinyNodePool) {
		m_tinyNodePool = new (dtAlloc(sizeof(Moss_RecastNodePool), DT_ALLOC_PERM)) Moss_RecastNodePool(64, 32);
		if (!m_tinyNodePool)
			return DT_FAILURE | DT_OUT_OF_MEMORY;
	}
	else
	{
		m_tinyNodePool->clear();
	}
	
	if (!m_openList || m_openList->getCapacity() < maxNodes)
	{
		if (m_openList)
		{
			m_openList->~Moss_RecastNodeQueue();
			MOSS_FREE(m_openList);
			m_openList = 0;
		}
		m_openList = new (dtAlloc(sizeof(Moss_RecastNodeQueue), DT_ALLOC_PERM)) Moss_RecastNodeQueue(maxNodes);
		if (!m_openList)
			return DT_FAILURE | DT_OUT_OF_MEMORY;
	}
	else
	{
		m_openList->clear();
	}
	
	return DT_SUCCESS;
}

Moss_RecastStatus Moss_RecastNavMeshQuery::findRandomPoint(const Moss_RecastQueryFilter* filter, float (*frand)(),
										 dtPolyRef* randomRef, float* randomPt) const
{
	dtAssert(m_nav);

	if (!filter || !frand || !randomRef || !randomPt)
		return DT_FAILURE | DT_INVALID_PARAM;

	// Randomly pick one tile. Assume that all tiles cover roughly the same area.
	const Moss_RecastMeshTile* tile = 0;
	float tsum = 0.0f;
	for (int i = 0; i < m_nav->getMaxTiles(); i++)
	{
		const Moss_RecastMeshTile* t = m_nav->getTile(i);
		if (!t || !t->header) continue;
		
		// Choose random tile using reservoir sampling.
		const float area = 1.0f; // Could be tile area too.
		tsum += area;
		const float u = frand();
		if (u*tsum <= area)
			tile = t;
	}
	if (!tile)
		return DT_FAILURE;

	// Randomly pick one polygon weighted by polygon area.
	const Moss_RecastPoly* poly = 0;
	dtPolyRef polyRef = 0;
	const dtPolyRef base = m_nav->getPolyRefBase(tile);

	float areaSum = 0.0f;
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		const Moss_RecastPoly* p = &tile->polys[i];
		// Do not return off-mesh connection polygons.
		if (p->getType() != DT_POLYTYPE_GROUND)
			continue;
		// Must pass filter
		const dtPolyRef ref = base | (dtPolyRef)i;
		if (!filter->passFilter(ref, tile, p))
			continue;

		// Calc area of the polygon.
		float polyArea = 0.0f;
		for (int j = 2; j < p->vertCount; ++j)
		{
			const float* va = &tile->verts[p->verts[0]*3];
			const float* vb = &tile->verts[p->verts[j-1]*3];
			const float* vc = &tile->verts[p->verts[j]*3];
			polyArea += dtTriArea2D(va,vb,vc);
		}

		// Choose random polygon weighted by area, using reservoir sampling.
		areaSum += polyArea;
		const float u = frand();
		if (u*areaSum <= polyArea)
		{
			poly = p;
			polyRef = ref;
		}
	}
	
	if (!poly)
		return DT_FAILURE;

	// Randomly pick point on polygon.
	const float* v = &tile->verts[poly->verts[0]*3];
	float verts[3*DT_VERTS_PER_POLYGON];
	float areas[DT_VERTS_PER_POLYGON];
	dtVcopy(&verts[0*3],v);
	for (int j = 1; j < poly->vertCount; ++j)
	{
		v = &tile->verts[poly->verts[j]*3];
		dtVcopy(&verts[j*3],v);
	}
	
	const float s = frand();
	const float t = frand();
	
	float pt[3];
	dtRandomPointInConvexPoly(verts, poly->vertCount, areas, s, t, pt);
	
	closestPointOnPoly(polyRef, pt, pt, NULL);
	
	dtVcopy(randomPt, pt);
	*randomRef = polyRef;

	return DT_SUCCESS;
}

Moss_RecastStatus Moss_RecastNavMeshQuery::findRandomPointAroundCircle(dtPolyRef startRef, const float* centerPos, const float maxRadius,
													 const Moss_RecastQueryFilter* filter, float (*frand)(),
													 dtPolyRef* randomRef, float* randomPt) const
{
	dtAssert(m_nav);
	dtAssert(m_nodePool);
	dtAssert(m_openList);
	
	// Validate input
	if (!m_nav->isValidPolyRef(startRef) ||
		!centerPos || !dtVisfinite(centerPos) ||
		maxRadius < 0 || !dtMathIsfinite(maxRadius) ||
		!filter || !frand || !randomRef || !randomPt)
	{
		return DT_FAILURE | DT_INVALID_PARAM;
	}
	
	const Moss_RecastMeshTile* startTile = 0;
	const Moss_RecastPoly* startPoly = 0;
	m_nav->getTileAndPolyByRefUnsafe(startRef, &startTile, &startPoly);
	if (!filter->passFilter(startRef, startTile, startPoly))
		return DT_FAILURE | DT_INVALID_PARAM;
	
	m_nodePool->clear();
	m_openList->clear();
	
	Moss_RecastNode* startNode = m_nodePool->getNode(startRef);
	dtVcopy(startNode->pos, centerPos);
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = 0;
	startNode->id = startRef;
	startNode->flags = DT_NODE_OPEN;
	m_openList->push(startNode);
	
	Moss_RecastStatus status = DT_SUCCESS;
	
	const float radiusSqr = Sqr(maxRadius);
	float areaSum = 0.0f;

	const Moss_RecastMeshTile* randomTile = 0;
	const Moss_RecastPoly* randomPoly = 0;
	dtPolyRef randomPolyRef = 0;

	while (!m_openList->empty())
	{
		Moss_RecastNode* bestNode = m_openList->pop();
		bestNode->flags &= ~DT_NODE_OPEN;
		bestNode->flags |= DT_NODE_CLOSED;
		
		// Get poly and tile.
		// The API input has been checked already, skip checking internal data.
		const dtPolyRef bestRef = bestNode->id;
		const Moss_RecastMeshTile* bestTile = 0;
		const Moss_RecastPoly* bestPoly = 0;
		m_nav->getTileAndPolyByRefUnsafe(bestRef, &bestTile, &bestPoly);

		// Place random locations on on ground.
		if (bestPoly->getType() == DT_POLYTYPE_GROUND)
		{
			// Calc area of the polygon.
			float polyArea = 0.0f;
			for (int j = 2; j < bestPoly->vertCount; ++j)
			{
				const float* va = &bestTile->verts[bestPoly->verts[0]*3];
				const float* vb = &bestTile->verts[bestPoly->verts[j-1]*3];
				const float* vc = &bestTile->verts[bestPoly->verts[j]*3];
				polyArea += dtTriArea2D(va,vb,vc);
			}
			// Choose random polygon weighted by area, using reservoir sampling.
			areaSum += polyArea;
			const float u = frand();
			if (u*areaSum <= polyArea)
			{
				randomTile = bestTile;
				randomPoly = bestPoly;
				randomPolyRef = bestRef;
			}
		}
		
		
		// Get parent poly and tile.
		dtPolyRef parentRef = 0;
		const Moss_RecastMeshTile* parentTile = 0;
		const Moss_RecastPoly* parentPoly = 0;
		if (bestNode->pidx)
			parentRef = m_nodePool->getNodeAtIdx(bestNode->pidx)->id;
		if (parentRef)
			m_nav->getTileAndPolyByRefUnsafe(parentRef, &parentTile, &parentPoly);
		
		for (unsigned int i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
		{
			const Moss_RecastLink* link = &bestTile->links[i];
			dtPolyRef neighbourRef = link->ref;
			// Skip invalid neighbours and do not follow back to parent.
			if (!neighbourRef || neighbourRef == parentRef)
				continue;
			
			// Expand to neighbour
			const Moss_RecastMeshTile* neighbourTile = 0;
			const Moss_RecastPoly* neighbourPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);
			
			// Do not advance if the polygon is excluded by the filter.
			if (!filter->passFilter(neighbourRef, neighbourTile, neighbourPoly))
				continue;
			
			// Find edge and calc distance to the edge.
			float va[3], vb[3];
			if (!getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile, va, vb))
				continue;
			
			// If the circle is not touching the next polygon, skip it.
			float tseg;
			float distSqr = dtDistancePtSegSqr2D(centerPos, va, vb, tseg);
			if (distSqr > radiusSqr)
				continue;
			
			Moss_RecastNode* neighbourNode = m_nodePool->getNode(neighbourRef);
			if (!neighbourNode)
			{
				status |= DT_OUT_OF_NODES;
				continue;
			}
			
			if (neighbourNode->flags & DT_NODE_CLOSED)
				continue;
			
			// Cost
			if (neighbourNode->flags == 0)
				dtVlerp(neighbourNode->pos, va, vb, 0.5f);
			
			const float total = bestNode->total + rcVdist(bestNode->pos, neighbourNode->pos);
			
			// The node is already in open list and the new result is worse, skip.
			if ((neighbourNode->flags & DT_NODE_OPEN) && total >= neighbourNode->total)
				continue;
			
			neighbourNode->id = neighbourRef;
			neighbourNode->flags = (neighbourNode->flags & ~DT_NODE_CLOSED);
			neighbourNode->pidx = m_nodePool->getNodeIdx(bestNode);
			neighbourNode->total = total;
			
			if (neighbourNode->flags & DT_NODE_OPEN)
			{
				m_openList->modify(neighbourNode);
			}
			else
			{
				neighbourNode->flags = DT_NODE_OPEN;
				m_openList->push(neighbourNode);
			}
		}
	}
	
	if (!randomPoly)
		return DT_FAILURE;
	
	// Randomly pick point on polygon.
	const float* v = &randomTile->verts[randomPoly->verts[0]*3];
	float verts[3*DT_VERTS_PER_POLYGON];
	float areas[DT_VERTS_PER_POLYGON];
	dtVcopy(&verts[0*3],v);
	for (int j = 1; j < randomPoly->vertCount; ++j)
	{
		v = &randomTile->verts[randomPoly->verts[j]*3];
		dtVcopy(&verts[j*3],v);
	}
	
	const float s = frand();
	const float t = frand();
	
	float pt[3];
	dtRandomPointInConvexPoly(verts, randomPoly->vertCount, areas, s, t, pt);
	
	closestPointOnPoly(randomPolyRef, pt, pt, NULL);
	
	dtVcopy(randomPt, pt);
	*randomRef = randomPolyRef;
	
	return status;
}


//////////////////////////////////////////////////////////////////////////////////////////

/// @par
///
/// Uses the detail polygons to find the surface height. (Most accurate.)
///
/// @p pos does not have to be within the bounds of the polygon or navigation mesh.
///
/// See closestPointOnPolyBoundary() for a limited but faster option.
///
Moss_RecastStatus Moss_RecastNavMeshQuery::closestPointOnPoly(dtPolyRef ref, const float* pos, float* closest, bool* posOverPoly) const
{
	dtAssert(m_nav);
	if (!m_nav->isValidPolyRef(ref) ||
		!pos || !dtVisfinite(pos) ||
		!closest)
	{
		return DT_FAILURE | DT_INVALID_PARAM;
	}

	m_nav->closestPointOnPoly(ref, pos, closest, posOverPoly);
	return DT_SUCCESS;
}

/// @par
///
/// Much faster than closestPointOnPoly().
///
/// If the provided position lies within the polygon's xz-bounds (above or below), 
/// then @p pos and @p closest will be equal.
///
/// The height of @p closest will be the polygon boundary.  The height detail is not used.
/// 
/// @p pos does not have to be within the bounds of the polybon or the navigation mesh.
/// 
Moss_RecastStatus Moss_RecastNavMeshQuery::closestPointOnPolyBoundary(dtPolyRef ref, const float* pos, float* closest) const
{
	dtAssert(m_nav);
	
	const Moss_RecastMeshTile* tile = 0;
	const Moss_RecastPoly* poly = 0;
	if (dtStatusFailed(m_nav->getTileAndPolyByRef(ref, &tile, &poly)))
		return DT_FAILURE | DT_INVALID_PARAM;

	if (!pos || !dtVisfinite(pos) || !closest)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	// Collect vertices.
	float verts[DT_VERTS_PER_POLYGON*3];	
	float edged[DT_VERTS_PER_POLYGON];
	float edget[DT_VERTS_PER_POLYGON];
	int nv = 0;
	for (int i = 0; i < (int)poly->vertCount; ++i)
	{
		dtVcopy(&verts[nv*3], &tile->verts[poly->verts[i]*3]);
		nv++;
	}		
	
	bool inside = dtDistancePtPolyEdgesSqr(pos, verts, nv, edged, edget);
	if (inside)
	{
		// Point is inside the polygon, return the point.
		dtVcopy(closest, pos);
	}
	else
	{
		// Point is outside the polygon, Clamp to nearest edge.
		float dmin = edged[0];
		int imin = 0;
		for (int i = 1; i < nv; ++i)
		{
			if (edged[i] < dmin)
			{
				dmin = edged[i];
				imin = i;
			}
		}
		const float* va = &verts[imin*3];
		const float* vb = &verts[((imin+1)%nv)*3];
		dtVlerp(closest, va, vb, edget[imin]);
	}
	
	return DT_SUCCESS;
}

/// @par
///
/// Will return #DT_FAILURE | DT_INVALID_PARAM if the provided position is outside the xz-bounds 
/// of the polygon.
/// 
Moss_RecastStatus Moss_RecastNavMeshQuery::getPolyHeight(dtPolyRef ref, const float* pos, float* height) const
{
	dtAssert(m_nav);

	const Moss_RecastMeshTile* tile = 0;
	const Moss_RecastPoly* poly = 0;
	if (dtStatusFailed(m_nav->getTileAndPolyByRef(ref, &tile, &poly)))
		return DT_FAILURE | DT_INVALID_PARAM;

	if (!pos || !dtVisfinite2D(pos))
		return DT_FAILURE | DT_INVALID_PARAM;

	// We used to return success for offmesh connections, but the
	// getPolyHeight in DetourNavMesh does not do this, so special
	// case it here.
	if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
	{
		const float* v0 = &tile->verts[poly->verts[0]*3];
		const float* v1 = &tile->verts[poly->verts[1]*3];
		float t;
		dtDistancePtSegSqr2D(pos, v0, v1, t);
		if (height)
			*height = v0[1] + (v1[1] - v0[1])*t;

		return DT_SUCCESS;
	}

	return m_nav->getPolyHeight(tile, poly, pos, height)
		? DT_SUCCESS
		: DT_FAILURE | DT_INVALID_PARAM;
}

class dtFindNearestPolyQuery : public Moss_RecastPolyQuery
{
	const Moss_RecastNavMeshQuery* m_query;
	const float* m_center;
	float m_nearestDistanceSqr;
	dtPolyRef m_nearestRef;
	float m_nearestPoint[3];
	bool m_overPoly;

public:
	dtFindNearestPolyQuery(const Moss_RecastNavMeshQuery* query, const float* center)
		: m_query(query), m_center(center), m_nearestDistanceSqr(FLT_MAX), m_nearestRef(0), m_nearestPoint(), m_overPoly(false)
	{
	}

	virtual ~dtFindNearestPolyQuery();

	dtPolyRef nearestRef() const { return m_nearestRef; }
	const float* nearestPoint() const { return m_nearestPoint; }
	bool isOverPoly() const { return m_overPoly; }

	void process(const Moss_RecastMeshTile* tile, Moss_RecastPoly** polys, dtPolyRef* refs, int count)
	{
		dtIgnoreUnused(polys);

		for (int i = 0; i < count; ++i)
		{
			dtPolyRef ref = refs[i];
			float closestPtPoly[3];
			float diff[3];
			bool posOverPoly = false;
			float d;
			m_query->closestPointOnPoly(ref, m_center, closestPtPoly, &posOverPoly);

			// If a point is directly over a polygon and closer than
			// climb height, favor that instead of straight line nearest point.
			dtVsub(diff, m_center, closestPtPoly);
			if (posOverPoly)
			{
				d = abs(diff[1]) - tile->header->walkableClimb;
				d = d > 0 ? d*d : 0;			
			}
			else
			{
				d = dtVlenSqr(diff);
			}
			
			if (d < m_nearestDistanceSqr)
			{
				dtVcopy(m_nearestPoint, closestPtPoly);

				m_nearestDistanceSqr = d;
				m_nearestRef = ref;
				m_overPoly = posOverPoly;
			}
		}
	}
};

dtFindNearestPolyQuery::~dtFindNearestPolyQuery()
{
	// Defined out of line to fix the weak v-tables warning
}

/// @par 
///
/// @note If the search box does not intersect any polygons the search will 
/// return #DT_SUCCESS, but @p nearestRef will be zero. So if in doubt, check 
/// @p nearestRef before using @p nearestPt.
///
Moss_RecastStatus Moss_RecastNavMeshQuery::findNearestPoly(const float* center, const float* halfExtents,
										 const Moss_RecastQueryFilter* filter,
										 dtPolyRef* nearestRef, float* nearestPt) const
{
	return findNearestPoly(center, halfExtents, filter, nearestRef, nearestPt, NULL);
}

// If center and nearestPt point to an equal position, isOverPoly will be true;
// however there's also a special case of climb height inside the polygon (see dtFindNearestPolyQuery)
Moss_RecastStatus Moss_RecastNavMeshQuery::findNearestPoly(const float* center, const float* halfExtents,
										 const Moss_RecastQueryFilter* filter,
										 dtPolyRef* nearestRef, float* nearestPt, bool* isOverPoly) const
{
	dtAssert(m_nav);

	if (!nearestRef)
		return DT_FAILURE | DT_INVALID_PARAM;

	// queryPolygons below will check rest of params
	
	dtFindNearestPolyQuery query(this, center);

	Moss_RecastStatus status = queryPolygons(center, halfExtents, filter, &query);
	if (dtStatusFailed(status))
		return status;

	*nearestRef = query.nearestRef();
	// Only override nearestPt if we actually found a poly so the nearest point
	// is valid.
	if (nearestPt && *nearestRef)
	{
		dtVcopy(nearestPt, query.nearestPoint());
		if (isOverPoly)
			*isOverPoly = query.isOverPoly();
	}
	
	return DT_SUCCESS;
}

void Moss_RecastNavMeshQuery::queryPolygonsInTile(const Moss_RecastMeshTile* tile, const float* qmin, const float* qmax,
										 const Moss_RecastQueryFilter* filter, Moss_RecastPolyQuery* query) const
{
	dtAssert(m_nav);
	static const int batchSize = 32;
	dtPolyRef polyRefs[batchSize];
	Moss_RecastPoly* polys[batchSize];
	int n = 0;

	if (tile->bvTree)
	{
		const Moss_RecastBVNode* node = &tile->bvTree[0];
		const Moss_RecastBVNode* end = &tile->bvTree[tile->header->bvNodeCount];
		const float* tbmin = tile->header->bmin;
		const float* tbmax = tile->header->bmax;
		const float qfac = tile->header->bvQuantFactor;

		// Calculate quantized box
		unsigned short bmin[3], bmax[3];
		// Clamp query box to world box.
		float minx = Clamp(qmin[0], tbmin[0], tbmax[0]) - tbmin[0];
		float miny = Clamp(qmin[1], tbmin[1], tbmax[1]) - tbmin[1];
		float minz = Clamp(qmin[2], tbmin[2], tbmax[2]) - tbmin[2];
		float maxx = Clamp(qmax[0], tbmin[0], tbmax[0]) - tbmin[0];
		float maxy = Clamp(qmax[1], tbmin[1], tbmax[1]) - tbmin[1];
		float maxz = Clamp(qmax[2], tbmin[2], tbmax[2]) - tbmin[2];
		// Quantize
		bmin[0] = (unsigned short)(qfac * minx) & 0xfffe;
		bmin[1] = (unsigned short)(qfac * miny) & 0xfffe;
		bmin[2] = (unsigned short)(qfac * minz) & 0xfffe;
		bmax[0] = (unsigned short)(qfac * maxx + 1) | 1;
		bmax[1] = (unsigned short)(qfac * maxy + 1) | 1;
		bmax[2] = (unsigned short)(qfac * maxz + 1) | 1;

		// Traverse tree
		const dtPolyRef base = m_nav->getPolyRefBase(tile);
		while (node < end)
		{
			const bool overlap = dtOverlapQuantBounds(bmin, bmax, node->bmin, node->bmax);
			const bool isLeafNode = node->i >= 0;

			if (isLeafNode && overlap)
			{
				dtPolyRef ref = base | (dtPolyRef)node->i;
				if (filter->passFilter(ref, tile, &tile->polys[node->i]))
				{
					polyRefs[n] = ref;
					polys[n] = &tile->polys[node->i];

					if (n == batchSize - 1)
					{
						query->process(tile, polys, polyRefs, batchSize);
						n = 0;
					}
					else
					{
						n++;
					}
				}
			}

			if (overlap || isLeafNode)
				node++;
			else
			{
				const int escapeIndex = -node->i;
				node += escapeIndex;
			}
		}
	}
	else
	{
		float bmin[3], bmax[3];
		const dtPolyRef base = m_nav->getPolyRefBase(tile);
		for (int i = 0; i < tile->header->polyCount; ++i)
		{
			Moss_RecastPoly* p = &tile->polys[i];
			// Do not return off-mesh connection polygons.
			if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
				continue;
			// Must pass filter
			const dtPolyRef ref = base | (dtPolyRef)i;
			if (!filter->passFilter(ref, tile, p))
				continue;
			// Calc polygon bounds.
			const float* v = &tile->verts[p->verts[0]*3];
			dtVcopy(bmin, v);
			dtVcopy(bmax, v);
			for (int j = 1; j < p->vertCount; ++j)
			{
				v = &tile->verts[p->verts[j]*3];
				min(bmin, v);
				max(bmax, v);
			}
			if (dtOverlapBounds(qmin, qmax, bmin, bmax))
			{
				polyRefs[n] = ref;
				polys[n] = p;

				if (n == batchSize - 1)
				{
					query->process(tile, polys, polyRefs, batchSize);
					n = 0;
				}
				else
				{
					n++;
				}
			}
		}
	}

	// Process the last polygons that didn't make a full batch.
	if (n > 0)
		query->process(tile, polys, polyRefs, n);
}

class dtCollectPolysQuery : public Moss_RecastPolyQuery
{
	dtPolyRef* m_polys;
	const int m_maxPolys;
	int m_numCollected;
	bool m_overflow;

public:
	dtCollectPolysQuery(dtPolyRef* polys, const int maxPolys)
		: m_polys(polys), m_maxPolys(maxPolys), m_numCollected(0), m_overflow(false)
	{
	}

	virtual ~dtCollectPolysQuery();

	int numCollected() const { return m_numCollected; }
	bool overflowed() const { return m_overflow; }

	void process(const Moss_RecastMeshTile* tile, Moss_RecastPoly** polys, dtPolyRef* refs, int count)
	{
		dtIgnoreUnused(tile);
		dtIgnoreUnused(polys);

		int numLeft = m_maxPolys - m_numCollected;
		int toCopy = count;
		if (toCopy > numLeft)
		{
			m_overflow = true;
			toCopy = numLeft;
		}

		memcpy(m_polys + m_numCollected, refs, (size_t)toCopy * sizeof(dtPolyRef));
		m_numCollected += toCopy;
	}
};

dtCollectPolysQuery::~dtCollectPolysQuery()
{
	// Defined out of line to fix the weak v-tables warning
}

/// @par 
///
/// If no polygons are found, the function will return #DT_SUCCESS with a
/// @p polyCount of zero.
///
/// If @p polys is too small to hold the entire result set, then the array will 
/// be filled to capacity. The method of choosing which polygons from the 
/// full set are included in the partial result set is undefined.
///
Moss_RecastStatus Moss_RecastNavMeshQuery::queryPolygons(const float* center, const float* halfExtents,
									   const Moss_RecastQueryFilter* filter,
									   dtPolyRef* polys, int* polyCount, const int maxPolys) const
{
	if (!polys || !polyCount || maxPolys < 0)
		return DT_FAILURE | DT_INVALID_PARAM;

	dtCollectPolysQuery collector(polys, maxPolys);

	Moss_RecastStatus status = queryPolygons(center, halfExtents, filter, &collector);
	if (dtStatusFailed(status))
		return status;

	*polyCount = collector.numCollected();
	return collector.overflowed() ? DT_SUCCESS | DT_BUFFER_TOO_SMALL : DT_SUCCESS;
}

/// @par 
///
/// The query will be invoked with batches of polygons. Polygons passed
/// to the query have bounding boxes that overlap with the center and halfExtents
/// passed to this function. The Moss_RecastPolyQuery::process function is invoked multiple
/// times until all overlapping polygons have been processed.
///
Moss_RecastStatus Moss_RecastNavMeshQuery::queryPolygons(const float* center, const float* halfExtents,
									   const Moss_RecastQueryFilter* filter, Moss_RecastPolyQuery* query) const
{
	dtAssert(m_nav);

	if (!center || !dtVisfinite(center) ||
		!halfExtents || !dtVisfinite(halfExtents) ||
		!filter || !query)
	{
		return DT_FAILURE | DT_INVALID_PARAM;
	}

	float bmin[3], bmax[3];
	dtVsub(bmin, center, halfExtents);
	dtVadd(bmax, center, halfExtents);
	
	// Find tiles the query touches.
	int minx, miny, maxx, maxy;
	m_nav->calcTileLoc(bmin, &minx, &miny);
	m_nav->calcTileLoc(bmax, &maxx, &maxy);

	static const int MAX_NEIS = 32;
	const Moss_RecastMeshTile* neis[MAX_NEIS];
	
	for (int y = miny; y <= maxy; ++y)
	{
		for (int x = minx; x <= maxx; ++x)
		{
			const int nneis = m_nav->getTilesAt(x,y,neis,MAX_NEIS);
			for (int j = 0; j < nneis; ++j)
			{
				queryPolygonsInTile(neis[j], bmin, bmax, filter, query);
			}
		}
	}
	
	return DT_SUCCESS;
}

/// @par
///
/// If the end polygon cannot be reached through the navigation graph,
/// the last polygon in the path will be the nearest the end polygon.
///
/// If the path array is to small to hold the full result, it will be filled as 
/// far as possible from the start polygon toward the end polygon.
///
/// The start and end positions are used to calculate traversal costs. 
/// (The y-values impact the result.)
///
Moss_RecastStatus Moss_RecastNavMeshQuery::findPath(dtPolyRef startRef, dtPolyRef endRef,
								  const float* startPos, const float* endPos,
								  const Moss_RecastQueryFilter* filter,
								  dtPolyRef* path, int* pathCount, const int maxPath) const
{
	dtAssert(m_nav);
	dtAssert(m_nodePool);
	dtAssert(m_openList);

	if (!pathCount)
		return DT_FAILURE | DT_INVALID_PARAM;

	*pathCount = 0;
	
	// Validate input
	if (!m_nav->isValidPolyRef(startRef) || !m_nav->isValidPolyRef(endRef) ||
		!startPos || !dtVisfinite(startPos) ||
		!endPos || !dtVisfinite(endPos) ||
		!filter || !path || maxPath <= 0)
	{
		return DT_FAILURE | DT_INVALID_PARAM;
	}

	if (startRef == endRef)
	{
		path[0] = startRef;
		*pathCount = 1;
		return DT_SUCCESS;
	}
	
	m_nodePool->clear();
	m_openList->clear();
	
	Moss_RecastNode* startNode = m_nodePool->getNode(startRef);
	dtVcopy(startNode->pos, startPos);
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = rcVdist(startPos, endPos) * H_SCALE;
	startNode->id = startRef;
	startNode->flags = DT_NODE_OPEN;
	m_openList->push(startNode);
	
	Moss_RecastNode* lastBestNode = startNode;
	float lastBestNodeCost = startNode->total;
	
	bool outOfNodes = false;
	
	while (!m_openList->empty())
	{
		// Remove node from open list and put it in closed list.
		Moss_RecastNode* bestNode = m_openList->pop();
		bestNode->flags &= ~DT_NODE_OPEN;
		bestNode->flags |= DT_NODE_CLOSED;
		
		// Reached the goal, stop searching.
		if (bestNode->id == endRef)
		{
			lastBestNode = bestNode;
			break;
		}
		
		// Get current poly and tile.
		// The API input has been checked already, skip checking internal data.
		const dtPolyRef bestRef = bestNode->id;
		const Moss_RecastMeshTile* bestTile = 0;
		const Moss_RecastPoly* bestPoly = 0;
		m_nav->getTileAndPolyByRefUnsafe(bestRef, &bestTile, &bestPoly);
		
		// Get parent poly and tile.
		dtPolyRef parentRef = 0;
		const Moss_RecastMeshTile* parentTile = 0;
		const Moss_RecastPoly* parentPoly = 0;
		if (bestNode->pidx)
			parentRef = m_nodePool->getNodeAtIdx(bestNode->pidx)->id;
		if (parentRef)
			m_nav->getTileAndPolyByRefUnsafe(parentRef, &parentTile, &parentPoly);
		
		for (unsigned int i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
		{
			dtPolyRef neighbourRef = bestTile->links[i].ref;
			
			// Skip invalid ids and do not expand back to where we came from.
			if (!neighbourRef || neighbourRef == parentRef)
				continue;
			
			// Get neighbour poly and tile.
			// The API input has been checked already, skip checking internal data.
			const Moss_RecastMeshTile* neighbourTile = 0;
			const Moss_RecastPoly* neighbourPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);			
			
			if (!filter->passFilter(neighbourRef, neighbourTile, neighbourPoly))
				continue;

			// deal explicitly with crossing tile boundaries
			unsigned char crossSide = 0;
			if (bestTile->links[i].side != 0xff)
				crossSide = bestTile->links[i].side >> 1;

			// get the node
			Moss_RecastNode* neighbourNode = m_nodePool->getNode(neighbourRef, crossSide);
			if (!neighbourNode) {
				outOfNodes = true;
				continue;
			}
			
			// If the node is visited the first time, calculate node position.
			if (neighbourNode->flags == 0)
			{
				getEdgeMidPoint(bestRef, bestPoly, bestTile,
								neighbourRef, neighbourPoly, neighbourTile,
								neighbourNode->pos);
			}

			// Calculate cost and heuristic.
			float cost = 0;
			float heuristic = 0;
			
			// Special case for last node.
			if (neighbourRef == endRef)
			{
				// Cost
				const float curCost = filter->getCost(bestNode->pos, neighbourNode->pos,
													  parentRef, parentTile, parentPoly,
													  bestRef, bestTile, bestPoly,
													  neighbourRef, neighbourTile, neighbourPoly);
				const float endCost = filter->getCost(neighbourNode->pos, endPos,
													  bestRef, bestTile, bestPoly,
													  neighbourRef, neighbourTile, neighbourPoly,
													  0, 0, 0);
				
				cost = bestNode->cost + curCost + endCost;
				heuristic = 0;
			}
			else
			{
				// Cost
				const float curCost = filter->getCost(bestNode->pos, neighbourNode->pos,
													  parentRef, parentTile, parentPoly,
													  bestRef, bestTile, bestPoly,
													  neighbourRef, neighbourTile, neighbourPoly);
				cost = bestNode->cost + curCost;
				heuristic = rcVdist(neighbourNode->pos, endPos)*H_SCALE;
			}

			const float total = cost + heuristic;
			
			// The node is already in open list and the new result is worse, skip.
			if ((neighbourNode->flags & DT_NODE_OPEN) && total >= neighbourNode->total)
				continue;
			// The node is already visited and process, and the new result is worse, skip.
			if ((neighbourNode->flags & DT_NODE_CLOSED) && total >= neighbourNode->total)
				continue;
			
			// Add or update the node.
			neighbourNode->pidx = m_nodePool->getNodeIdx(bestNode);
			neighbourNode->id = neighbourRef;
			neighbourNode->flags = (neighbourNode->flags & ~DT_NODE_CLOSED);
			neighbourNode->cost = cost;
			neighbourNode->total = total;
			
			if (neighbourNode->flags & DT_NODE_OPEN) {
				// Already in open, update node location.
				m_openList->modify(neighbourNode);
			}
			else {
				// Put the node in open list.
				neighbourNode->flags |= DT_NODE_OPEN;
				m_openList->push(neighbourNode);
			}
			
			// Update nearest node to target so far.
			if (heuristic < lastBestNodeCost) {
				lastBestNodeCost = heuristic;
				lastBestNode = neighbourNode;
			}
		}
	}

	Moss_RecastStatus status = getPathToNode(lastBestNode, path, pathCount, maxPath);

	if (lastBestNode->id != endRef)
		status |= DT_PARTIAL_RESULT;

	if (outOfNodes)
		status |= DT_OUT_OF_NODES;
	
	return status;
}

Moss_RecastStatus Moss_RecastNavMeshQuery::getPathToNode(Moss_RecastNode* endNode, dtPolyRef* path, int* pathCount, int maxPath) const
{
	// Find the length of the entire path.
	Moss_RecastNode* curNode = endNode;
	int length = 0;
	do
	{
		length++;
		curNode = m_nodePool->getNodeAtIdx(curNode->pidx);
	} while (curNode);

	// If the path cannot be fully stored then advance to the last node we will be able to store.
	curNode = endNode;
	int writeCount;
	for (writeCount = length; writeCount > maxPath; writeCount--)
	{
		dtAssert(curNode);

		curNode = m_nodePool->getNodeAtIdx(curNode->pidx);
	}

	// Write path
	for (int i = writeCount - 1; i >= 0; i--)
	{
		dtAssert(curNode);

		path[i] = curNode->id;
		curNode = m_nodePool->getNodeAtIdx(curNode->pidx);
	}

	dtAssert(!curNode);

	*pathCount = min(length, maxPath);

	if (length > maxPath)
		return DT_SUCCESS | DT_BUFFER_TOO_SMALL;

	return DT_SUCCESS;
}


/// @par
///
/// @warning Calling any non-slice methods before calling finalizeSlicedFindPath() 
/// or finalizeSlicedFindPathPartial() may result in corrupted data!
///
/// The @p filter pointer is stored and used for the duration of the sliced
/// path query.
///
Moss_RecastStatus Moss_RecastNavMeshQuery::initSlicedFindPath(dtPolyRef startRef, dtPolyRef endRef,
											const float* startPos, const float* endPos,
											const Moss_RecastQueryFilter* filter, const unsigned int options)
{
	dtAssert(m_nav);
	dtAssert(m_nodePool);
	dtAssert(m_openList);

	// Init path state.
	memset(&m_query, 0, sizeof(dtQueryData));
	m_query.status = DT_FAILURE;
	m_query.startRef = startRef;
	m_query.endRef = endRef;
	if (startPos)
		dtVcopy(m_query.startPos, startPos);
	if (endPos)
		dtVcopy(m_query.endPos, endPos);
	m_query.filter = filter;
	m_query.options = options;
	m_query.raycastLimitSqr = FLT_MAX;
	
	// Validate input
	if (!m_nav->isValidPolyRef(startRef) || !m_nav->isValidPolyRef(endRef) ||
		!startPos || !dtVisfinite(startPos) ||
		!endPos || !dtVisfinite(endPos) || !filter)
	{
		return DT_FAILURE | DT_INVALID_PARAM;
	}

	// trade quality with performance?
	if (options & DT_FINDPATH_ANY_ANGLE)
	{
		// limiting to several times the character radius yields nice results. It is not sensitive 
		// so it is enough to compute it from the first tile.
		const Moss_RecastMeshTile* tile = m_nav->getTileByRef(startRef);
		float agentRadius = tile->header->walkableRadius;
		m_query.raycastLimitSqr = Sqr(agentRadius * DT_RAY_CAST_LIMIT_PROPORTIONS);
	}

	if (startRef == endRef)
	{
		m_query.status = DT_SUCCESS;
		return DT_SUCCESS;
	}
	
	m_nodePool->clear();
	m_openList->clear();
	
	Moss_RecastNode* startNode = m_nodePool->getNode(startRef);
	dtVcopy(startNode->pos, startPos);
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = rcVdist(startPos, endPos) * H_SCALE;
	startNode->id = startRef;
	startNode->flags = DT_NODE_OPEN;
	m_openList->push(startNode);
	
	m_query.status = DT_IN_PROGRESS;
	m_query.lastBestNode = startNode;
	m_query.lastBestNodeCost = startNode->total;
	
	return m_query.status;
}
	
Moss_RecastStatus Moss_RecastNavMeshQuery::updateSlicedFindPath(const int maxIter, int* doneIters)
{
	if (!dtStatusInProgress(m_query.status))
		return m_query.status;

	// Make sure the request is still valid.
	if (!m_nav->isValidPolyRef(m_query.startRef) || !m_nav->isValidPolyRef(m_query.endRef))
	{
		m_query.status = DT_FAILURE;
		return DT_FAILURE;
	}

	Moss_RecastRaycastHit rayHit;
	rayHit.maxPath = 0;
		
	int iter = 0;
	while (iter < maxIter && !m_openList->empty())
	{
		iter++;
		
		// Remove node from open list and put it in closed list.
		Moss_RecastNode* bestNode = m_openList->pop();
		bestNode->flags &= ~DT_NODE_OPEN;
		bestNode->flags |= DT_NODE_CLOSED;
		
		// Reached the goal, stop searching.
		if (bestNode->id == m_query.endRef)
		{
			m_query.lastBestNode = bestNode;
			const Moss_RecastStatus details = m_query.status & DT_STATUS_DETAIL_MASK;
			m_query.status = DT_SUCCESS | details;
			if (doneIters)
				*doneIters = iter;
			return m_query.status;
		}
		
		// Get current poly and tile.
		// The API input has been checked already, skip checking internal data.
		const dtPolyRef bestRef = bestNode->id;
		const Moss_RecastMeshTile* bestTile = 0;
		const Moss_RecastPoly* bestPoly = 0;
		if (dtStatusFailed(m_nav->getTileAndPolyByRef(bestRef, &bestTile, &bestPoly)))
		{
			// The polygon has disappeared during the sliced query, fail.
			m_query.status = DT_FAILURE;
			if (doneIters)
				*doneIters = iter;
			return m_query.status;
		}
		
		// Get parent and grand parent poly and tile.
		dtPolyRef parentRef = 0, grandpaRef = 0;
		const Moss_RecastMeshTile* parentTile = 0;
		const Moss_RecastPoly* parentPoly = 0;
		Moss_RecastNode* parentNode = 0;
		if (bestNode->pidx)
		{
			parentNode = m_nodePool->getNodeAtIdx(bestNode->pidx);
			parentRef = parentNode->id;
			if (parentNode->pidx)
				grandpaRef = m_nodePool->getNodeAtIdx(parentNode->pidx)->id;
		}
		if (parentRef)
		{
			bool invalidParent = dtStatusFailed(m_nav->getTileAndPolyByRef(parentRef, &parentTile, &parentPoly));
			if (invalidParent || (grandpaRef && !m_nav->isValidPolyRef(grandpaRef)) )
			{
				// The polygon has disappeared during the sliced query, fail.
				m_query.status = DT_FAILURE;
				if (doneIters)
					*doneIters = iter;
				return m_query.status;
			}
		}

		// decide whether to test raycast to previous nodes
		bool tryLOS = false;
		if (m_query.options & DT_FINDPATH_ANY_ANGLE)
		{
			if ((parentRef != 0) && (rcVdistSqr(parentNode->pos, bestNode->pos) < m_query.raycastLimitSqr))
				tryLOS = true;
		}
		
		for (unsigned int i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
		{
			dtPolyRef neighbourRef = bestTile->links[i].ref;
			
			// Skip invalid ids and do not expand back to where we came from.
			if (!neighbourRef || neighbourRef == parentRef)
				continue;
			
			// Get neighbour poly and tile.
			// The API input has been checked already, skip checking internal data.
			const Moss_RecastMeshTile* neighbourTile = 0;
			const Moss_RecastPoly* neighbourPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);			
			
			if (!m_query.filter->passFilter(neighbourRef, neighbourTile, neighbourPoly))
				continue;
			
			// get the neighbor node
			Moss_RecastNode* neighbourNode = m_nodePool->getNode(neighbourRef, 0);
			if (!neighbourNode)
			{
				m_query.status |= DT_OUT_OF_NODES;
				continue;
			}
			
			// do not expand to nodes that were already visited from the same parent
			if (neighbourNode->pidx != 0 && neighbourNode->pidx == bestNode->pidx)
				continue;

			// If the node is visited the first time, calculate node position.
			if (neighbourNode->flags == 0)
			{
				getEdgeMidPoint(bestRef, bestPoly, bestTile,
								neighbourRef, neighbourPoly, neighbourTile,
								neighbourNode->pos);
			}
			
			// Calculate cost and heuristic.
			float cost = 0;
			float heuristic = 0;
			
			// raycast parent
			bool foundShortCut = false;
			rayHit.pathCost = rayHit.t = 0;
			if (tryLOS)
			{
				raycast(parentRef, parentNode->pos, neighbourNode->pos, m_query.filter, DT_RAYCAST_USE_COSTS, &rayHit, grandpaRef);
				foundShortCut = rayHit.t >= 1.0f;
			}

			// update move cost
			if (foundShortCut)
			{
				// shortcut found using raycast. Using shorter cost instead
				cost = parentNode->cost + rayHit.pathCost;
			}
			else
			{
				// No shortcut found.
				const float curCost = m_query.filter->getCost(bestNode->pos, neighbourNode->pos,
															  parentRef, parentTile, parentPoly,
															bestRef, bestTile, bestPoly,
															neighbourRef, neighbourTile, neighbourPoly);
				cost = bestNode->cost + curCost;
			}

			// Special case for last node.
			if (neighbourRef == m_query.endRef)
			{
				const float endCost = m_query.filter->getCost(neighbourNode->pos, m_query.endPos,
															  bestRef, bestTile, bestPoly,
															  neighbourRef, neighbourTile, neighbourPoly,
															  0, 0, 0);
				
				cost = cost + endCost;
				heuristic = 0;
			}
			else
			{
				heuristic = rcVdist(neighbourNode->pos, m_query.endPos)*H_SCALE;
			}
			
			const float total = cost + heuristic;
			
			// The node is already in open list and the new result is worse, skip.
			if ((neighbourNode->flags & DT_NODE_OPEN) && total >= neighbourNode->total)
				continue;
			// The node is already visited and process, and the new result is worse, skip.
			if ((neighbourNode->flags & DT_NODE_CLOSED) && total >= neighbourNode->total)
				continue;
			
			// Add or update the node.
			neighbourNode->pidx = foundShortCut ? bestNode->pidx : m_nodePool->getNodeIdx(bestNode);
			neighbourNode->id = neighbourRef;
			neighbourNode->flags = (neighbourNode->flags & ~(DT_NODE_CLOSED | DT_NODE_PARENT_DETACHED));
			neighbourNode->cost = cost;
			neighbourNode->total = total;
			if (foundShortCut)
				neighbourNode->flags = (neighbourNode->flags | DT_NODE_PARENT_DETACHED);
			
			if (neighbourNode->flags & DT_NODE_OPEN)
			{
				// Already in open, update node location.
				m_openList->modify(neighbourNode);
			}
			else
			{
				// Put the node in open list.
				neighbourNode->flags |= DT_NODE_OPEN;
				m_openList->push(neighbourNode);
			}
			
			// Update nearest node to target so far.
			if (heuristic < m_query.lastBestNodeCost)
			{
				m_query.lastBestNodeCost = heuristic;
				m_query.lastBestNode = neighbourNode;
			}
		}
	}
	
	// Exhausted all nodes, but could not find path.
	if (m_openList->empty())
	{
		const Moss_RecastStatus details = m_query.status & DT_STATUS_DETAIL_MASK;
		m_query.status = DT_SUCCESS | details;
	}

	if (doneIters)
		*doneIters = iter;

	return m_query.status;
}

Moss_RecastStatus Moss_RecastNavMeshQuery::finalizeSlicedFindPath(dtPolyRef* path, int* pathCount, const int maxPath)
{
	if (!pathCount)
		return DT_FAILURE | DT_INVALID_PARAM;

	*pathCount = 0;

	if (!path || maxPath <= 0)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	if (dtStatusFailed(m_query.status))
	{
		// Reset query.
		memset(&m_query, 0, sizeof(dtQueryData));
		return DT_FAILURE;
	}

	int n = 0;

	if (m_query.startRef == m_query.endRef)
	{
		// Special case: the search starts and ends at same poly.
		path[n++] = m_query.startRef;
	}
	else
	{
		// Reverse the path.
		dtAssert(m_query.lastBestNode);
		
		if (m_query.lastBestNode->id != m_query.endRef)
			m_query.status |= DT_PARTIAL_RESULT;
		
		Moss_RecastNode* prev = 0;
		Moss_RecastNode* node = m_query.lastBestNode;
		int prevRay = 0;
		do
		{
			Moss_RecastNode* next = m_nodePool->getNodeAtIdx(node->pidx);
			node->pidx = m_nodePool->getNodeIdx(prev);
			prev = node;
			int nextRay = node->flags & DT_NODE_PARENT_DETACHED; // keep track of whether parent is not adjacent (i.e. due to raycast shortcut)
			node->flags = (node->flags & ~DT_NODE_PARENT_DETACHED) | prevRay; // and store it in the reversed path's node
			prevRay = nextRay;
			node = next;
		}
		while (node);
		
		// Store path
		node = prev;
		do
		{
			Moss_RecastNode* next = m_nodePool->getNodeAtIdx(node->pidx);
			Moss_RecastStatus status = 0;
			if (node->flags & DT_NODE_PARENT_DETACHED)
			{
				float t, normal[3];
				int m;
				status = raycast(node->id, node->pos, next->pos, m_query.filter, &t, normal, path+n, &m, maxPath-n);
				n += m;
				// raycast ends on poly boundary and the path might include the next poly boundary.
				if (path[n-1] == next->id)
					n--; // remove to avoid duplicates
			}
			else
			{
				path[n++] = node->id;
				if (n >= maxPath)
					status = DT_BUFFER_TOO_SMALL;
			}

			if (status & DT_STATUS_DETAIL_MASK)
			{
				m_query.status |= status & DT_STATUS_DETAIL_MASK;
				break;
			}
			node = next;
		}
		while (node);
	}
	
	const Moss_RecastStatus details = m_query.status & DT_STATUS_DETAIL_MASK;

	// Reset query.
	memset(&m_query, 0, sizeof(dtQueryData));
	
	*pathCount = n;
	
	return DT_SUCCESS | details;
}

Moss_RecastStatus Moss_RecastNavMeshQuery::finalizeSlicedFindPathPartial(const dtPolyRef* existing, const int existingSize,
													   dtPolyRef* path, int* pathCount, const int maxPath)
{
	if (!pathCount)
		return DT_FAILURE | DT_INVALID_PARAM;

	*pathCount = 0;

	if (!existing || existingSize <= 0 || !path || !pathCount || maxPath <= 0)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	if (dtStatusFailed(m_query.status))
	{
		// Reset query.
		memset(&m_query, 0, sizeof(dtQueryData));
		return DT_FAILURE;
	}
	
	int n = 0;
	
	if (m_query.startRef == m_query.endRef)
	{
		// Special case: the search starts and ends at same poly.
		path[n++] = m_query.startRef;
	}
	else
	{
		// Find furthest existing node that was visited.
		Moss_RecastNode* prev = 0;
		Moss_RecastNode* node = 0;
		for (int i = existingSize-1; i >= 0; --i)
		{
			m_nodePool->findNodes(existing[i], &node, 1);
			if (node)
				break;
		}
		
		if (!node)
		{
			m_query.status |= DT_PARTIAL_RESULT;
			dtAssert(m_query.lastBestNode);
			node = m_query.lastBestNode;
		}
		
		// Reverse the path.
		int prevRay = 0;
		do
		{
			Moss_RecastNode* next = m_nodePool->getNodeAtIdx(node->pidx);
			node->pidx = m_nodePool->getNodeIdx(prev);
			prev = node;
			int nextRay = node->flags & DT_NODE_PARENT_DETACHED; // keep track of whether parent is not adjacent (i.e. due to raycast shortcut)
			node->flags = (node->flags & ~DT_NODE_PARENT_DETACHED) | prevRay; // and store it in the reversed path's node
			prevRay = nextRay;
			node = next;
		}
		while (node);
		
		// Store path
		node = prev;
		do
		{
			Moss_RecastNode* next = m_nodePool->getNodeAtIdx(node->pidx);
			Moss_RecastStatus status = 0;
			if (node->flags & DT_NODE_PARENT_DETACHED)
			{
				float t, normal[3];
				int m;
				status = raycast(node->id, node->pos, next->pos, m_query.filter, &t, normal, path+n, &m, maxPath-n);
				n += m;
				// raycast ends on poly boundary and the path might include the next poly boundary.
				if (path[n-1] == next->id)
					n--; // remove to avoid duplicates
			}
			else
			{
				path[n++] = node->id;
				if (n >= maxPath)
					status = DT_BUFFER_TOO_SMALL;
			}

			if (status & DT_STATUS_DETAIL_MASK)
			{
				m_query.status |= status & DT_STATUS_DETAIL_MASK;
				break;
			}
			node = next;
		}
		while (node);
	}
	
	const Moss_RecastStatus details = m_query.status & DT_STATUS_DETAIL_MASK;

	// Reset query.
	memset(&m_query, 0, sizeof(dtQueryData));
	
	*pathCount = n;
	
	return DT_SUCCESS | details;
}


Moss_RecastStatus Moss_RecastNavMeshQuery::appendVertex(const float* pos, const unsigned char flags, const dtPolyRef ref,
									  float* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
									  int* straightPathCount, const int maxStraightPath) const
{
	if ((*straightPathCount) > 0 && dtVequal(&straightPath[((*straightPathCount)-1)*3], pos))
	{
		// The vertices are equal, update flags and poly.
		if (straightPathFlags)
			straightPathFlags[(*straightPathCount)-1] = flags;
		if (straightPathRefs)
			straightPathRefs[(*straightPathCount)-1] = ref;
	}
	else
	{
		// Append new vertex.
		dtVcopy(&straightPath[(*straightPathCount)*3], pos);
		if (straightPathFlags)
			straightPathFlags[(*straightPathCount)] = flags;
		if (straightPathRefs)
			straightPathRefs[(*straightPathCount)] = ref;
		(*straightPathCount)++;

		// If there is no space to append more vertices, return.
		if ((*straightPathCount) >= maxStraightPath)
		{
			return DT_SUCCESS | DT_BUFFER_TOO_SMALL;
		}

		// If reached end of path, return.
		if (flags == DT_STRAIGHTPATH_END)
		{
			return DT_SUCCESS;
		}
	}
	return DT_IN_PROGRESS;
}

Moss_RecastStatus Moss_RecastNavMeshQuery::appendPortals(const int startIdx, const int endIdx, const float* endPos, const dtPolyRef* path,
									  float* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
									  int* straightPathCount, const int maxStraightPath, const int options) const
{
	const float* startPos = &straightPath[(*straightPathCount-1)*3];
	// Append or update last vertex
	Moss_RecastStatus stat = 0;
	for (int i = startIdx; i < endIdx; i++)
	{
		// Calculate portal
		const dtPolyRef from = path[i];
		const Moss_RecastMeshTile* fromTile = 0;
		const Moss_RecastPoly* fromPoly = 0;
		if (dtStatusFailed(m_nav->getTileAndPolyByRef(from, &fromTile, &fromPoly)))
			return DT_FAILURE | DT_INVALID_PARAM;
		
		const dtPolyRef to = path[i+1];
		const Moss_RecastMeshTile* toTile = 0;
		const Moss_RecastPoly* toPoly = 0;
		if (dtStatusFailed(m_nav->getTileAndPolyByRef(to, &toTile, &toPoly)))
			return DT_FAILURE | DT_INVALID_PARAM;
		
		float left[3], right[3];
		if (dtStatusFailed(getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, left, right)))
			break;
	
		if (options & DT_STRAIGHTPATH_AREA_CROSSINGS)
		{
			// Skip intersection if only area crossings are requested.
			if (fromPoly->getArea() == toPoly->getArea())
				continue;
		}
		
		// Append intersection
		float s,t;
		if (dtIntersectSegSeg2D(startPos, endPos, left, right, s, t))
		{
			float pt[3];
			dtVlerp(pt, left,right, t);

			stat = appendVertex(pt, 0, path[i+1],
								straightPath, straightPathFlags, straightPathRefs,
								straightPathCount, maxStraightPath);
			if (stat != DT_IN_PROGRESS)
				return stat;
		}
	}
	return DT_IN_PROGRESS;
}

/// @par
/// 
/// This method peforms what is often called 'string pulling'.
///
/// The start position is clamped to the first polygon in the path, and the 
/// end position is clamped to the last. So the start and end positions should 
/// normally be within or very near the first and last polygons respectively.
///
/// The returned polygon references represent the reference id of the polygon 
/// that is entered at the associated path position. The reference id associated 
/// with the end point will always be zero.  This allows, for example, matching 
/// off-mesh link points to their representative polygons.
///
/// If the provided result buffers are too small for the entire result set, 
/// they will be filled as far as possible from the start toward the end 
/// position.
///
Moss_RecastStatus Moss_RecastNavMeshQuery::findStraightPath(const float* startPos, const float* endPos,
										  const dtPolyRef* path, const int pathSize,
										  float* straightPath, unsigned char* straightPathFlags, dtPolyRef* straightPathRefs,
										  int* straightPathCount, const int maxStraightPath, const int options) const
{
	dtAssert(m_nav);

	if (!straightPathCount)
		return DT_FAILURE | DT_INVALID_PARAM;

	*straightPathCount = 0;

	if (!startPos || !dtVisfinite(startPos) ||
		!endPos || !dtVisfinite(endPos) ||
		!path || pathSize <= 0 || !path[0] ||
		maxStraightPath <= 0)
	{
		return DT_FAILURE | DT_INVALID_PARAM;
	}
	
	Moss_RecastStatus stat = 0;
	
	// TODO: Should this be callers responsibility?
	float closestStartPos[3];
	if (dtStatusFailed(closestPointOnPolyBoundary(path[0], startPos, closestStartPos)))
		return DT_FAILURE | DT_INVALID_PARAM;

	float closestEndPos[3];
	if (dtStatusFailed(closestPointOnPolyBoundary(path[pathSize-1], endPos, closestEndPos)))
		return DT_FAILURE | DT_INVALID_PARAM;
	
	// Add start point.
	stat = appendVertex(closestStartPos, DT_STRAIGHTPATH_START, path[0],
						straightPath, straightPathFlags, straightPathRefs,
						straightPathCount, maxStraightPath);
	if (stat != DT_IN_PROGRESS)
		return stat;
	
	if (pathSize > 1)
	{
		float portalApex[3], portalLeft[3], portalRight[3];
		dtVcopy(portalApex, closestStartPos);
		dtVcopy(portalLeft, portalApex);
		dtVcopy(portalRight, portalApex);
		int apexIndex = 0;
		int leftIndex = 0;
		int rightIndex = 0;
		
		unsigned char leftPolyType = 0;
		unsigned char rightPolyType = 0;
		
		dtPolyRef leftPolyRef = path[0];
		dtPolyRef rightPolyRef = path[0];
		
		for (int i = 0; i < pathSize; ++i)
		{
			float left[3], right[3];
			unsigned char toType;
			
			if (i+1 < pathSize)
			{
				unsigned char fromType; // fromType is ignored.

				// Next portal.
				if (dtStatusFailed(getPortalPoints(path[i], path[i+1], left, right, fromType, toType)))
				{
					// Failed to get portal points, in practice this means that path[i+1] is invalid polygon.
					// Clamp the end point to path[i], and return the path so far.
					
					if (dtStatusFailed(closestPointOnPolyBoundary(path[i], endPos, closestEndPos)))
					{
						// This should only happen when the first polygon is invalid.
						return DT_FAILURE | DT_INVALID_PARAM;
					}

					// Apeend portals along the current straight path segment.
					if (options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS))
					{
						// Ignore status return value as we're just about to return anyway.
						appendPortals(apexIndex, i, closestEndPos, path,
											 straightPath, straightPathFlags, straightPathRefs,
											 straightPathCount, maxStraightPath, options);
					}

					// Ignore status return value as we're just about to return anyway.
					appendVertex(closestEndPos, 0, path[i],
										straightPath, straightPathFlags, straightPathRefs,
										straightPathCount, maxStraightPath);
					
					return DT_SUCCESS | DT_PARTIAL_RESULT | ((*straightPathCount >= maxStraightPath) ? DT_BUFFER_TOO_SMALL : 0);
				}
				
				// If starting really close the portal, advance.
				if (i == 0)
				{
					float t;
					if (dtDistancePtSegSqr2D(portalApex, left, right, t) < Sqr(0.001f))
						continue;
				}
			}
			else
			{
				// End of the path.
				dtVcopy(left, closestEndPos);
				dtVcopy(right, closestEndPos);
				
				toType = DT_POLYTYPE_GROUND;
			}
			
			// Right vertex.
			if (dtTriArea2D(portalApex, portalRight, right) <= 0.0f)
			{
				if (dtVequal(portalApex, portalRight) || dtTriArea2D(portalApex, portalLeft, right) > 0.0f)
				{
					dtVcopy(portalRight, right);
					rightPolyRef = (i+1 < pathSize) ? path[i+1] : 0;
					rightPolyType = toType;
					rightIndex = i;
				}
				else
				{
					// Append portals along the current straight path segment.
					if (options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS))
					{
						stat = appendPortals(apexIndex, leftIndex, portalLeft, path,
											 straightPath, straightPathFlags, straightPathRefs,
											 straightPathCount, maxStraightPath, options);
						if (stat != DT_IN_PROGRESS)
							return stat;					
					}
				
					dtVcopy(portalApex, portalLeft);
					apexIndex = leftIndex;
					
					unsigned char flags = 0;
					if (!leftPolyRef)
						flags = DT_STRAIGHTPATH_END;
					else if (leftPolyType == DT_POLYTYPE_OFFMESH_CONNECTION)
						flags = DT_STRAIGHTPATH_OFFMESH_CONNECTION;
					dtPolyRef ref = leftPolyRef;
					
					// Append or update vertex
					stat = appendVertex(portalApex, flags, ref,
										straightPath, straightPathFlags, straightPathRefs,
										straightPathCount, maxStraightPath);
					if (stat != DT_IN_PROGRESS)
						return stat;
					
					dtVcopy(portalLeft, portalApex);
					dtVcopy(portalRight, portalApex);
					leftIndex = apexIndex;
					rightIndex = apexIndex;
					
					// Restart
					i = apexIndex;
					
					continue;
				}
			}
			
			// Left vertex.
			if (dtTriArea2D(portalApex, portalLeft, left) >= 0.0f)
			{
				if (dtVequal(portalApex, portalLeft) || dtTriArea2D(portalApex, portalRight, left) < 0.0f)
				{
					dtVcopy(portalLeft, left);
					leftPolyRef = (i+1 < pathSize) ? path[i+1] : 0;
					leftPolyType = toType;
					leftIndex = i;
				}
				else
				{
					// Append portals along the current straight path segment.
					if (options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS))
					{
						stat = appendPortals(apexIndex, rightIndex, portalRight, path,
											 straightPath, straightPathFlags, straightPathRefs,
											 straightPathCount, maxStraightPath, options);
						if (stat != DT_IN_PROGRESS)
							return stat;
					}

					dtVcopy(portalApex, portalRight);
					apexIndex = rightIndex;
					
					unsigned char flags = 0;
					if (!rightPolyRef)
						flags = DT_STRAIGHTPATH_END;
					else if (rightPolyType == DT_POLYTYPE_OFFMESH_CONNECTION)
						flags = DT_STRAIGHTPATH_OFFMESH_CONNECTION;
					dtPolyRef ref = rightPolyRef;

					// Append or update vertex
					stat = appendVertex(portalApex, flags, ref,
										straightPath, straightPathFlags, straightPathRefs,
										straightPathCount, maxStraightPath);
					if (stat != DT_IN_PROGRESS)
						return stat;
					
					dtVcopy(portalLeft, portalApex);
					dtVcopy(portalRight, portalApex);
					leftIndex = apexIndex;
					rightIndex = apexIndex;
					
					// Restart
					i = apexIndex;
					
					continue;
				}
			}
		}

		// Append portals along the current straight path segment.
		if (options & (DT_STRAIGHTPATH_AREA_CROSSINGS | DT_STRAIGHTPATH_ALL_CROSSINGS))
		{
			stat = appendPortals(apexIndex, pathSize-1, closestEndPos, path,
								 straightPath, straightPathFlags, straightPathRefs,
								 straightPathCount, maxStraightPath, options);
			if (stat != DT_IN_PROGRESS)
				return stat;
		}
	}

	// Ignore status return value as we're just about to return anyway.
	appendVertex(closestEndPos, DT_STRAIGHTPATH_END, 0,
						straightPath, straightPathFlags, straightPathRefs,
						straightPathCount, maxStraightPath);
	
	return DT_SUCCESS | ((*straightPathCount >= maxStraightPath) ? DT_BUFFER_TOO_SMALL : 0);
}

/// @par
///
/// This method is optimized for small delta movement and a small number of 
/// polygons. If used for too great a distance, the result set will form an 
/// incomplete path.
///
/// @p resultPos will equal the @p endPos if the end is reached. 
/// Otherwise the closest reachable position will be returned.
/// 
/// @p resultPos is not projected onto the surface of the navigation 
/// mesh. Use #getPolyHeight if this is needed.
///
/// This method treats the end position in the same manner as 
/// the #raycast method. (As a 2D point.) See that method's documentation 
/// for details.
/// 
/// If the @p visited array is too small to hold the entire result set, it will 
/// be filled as far as possible from the start position toward the end 
/// position.
///
Moss_RecastStatus Moss_RecastNavMeshQuery::moveAlongSurface(dtPolyRef startRef, const float* startPos, const float* endPos,
										  const Moss_RecastQueryFilter* filter,
										  float* resultPos, dtPolyRef* visited, int* visitedCount, const int maxVisitedSize) const
{
	dtAssert(m_nav);
	dtAssert(m_tinyNodePool);

	if (!visitedCount)
		return DT_FAILURE | DT_INVALID_PARAM;

	*visitedCount = 0;

	if (!m_nav->isValidPolyRef(startRef) ||
		!startPos || !dtVisfinite(startPos) ||
		!endPos || !dtVisfinite(endPos) ||
		!filter || !resultPos || !visited ||
		maxVisitedSize <= 0)
	{
		return DT_FAILURE | DT_INVALID_PARAM;
	}
	
	Moss_RecastStatus status = DT_SUCCESS;
	
	static const int MAX_STACK = 48;
	Moss_RecastNode* stack[MAX_STACK];
	int nstack = 0;
	
	m_tinyNodePool->clear();
	
	Moss_RecastNode* startNode = m_tinyNodePool->getNode(startRef);
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = 0;
	startNode->id = startRef;
	startNode->flags = DT_NODE_CLOSED;
	stack[nstack++] = startNode;
	
	float bestPos[3];
	float bestDist = FLT_MAX;
	Moss_RecastNode* bestNode = 0;
	dtVcopy(bestPos, startPos);
	
	// Search constraints
	float searchPos[3], searchRadSqr;
	dtVlerp(searchPos, startPos, endPos, 0.5f);
	searchRadSqr = Sqr(rcVdist(startPos, endPos)/2.0f + 0.001f);
	
	float verts[DT_VERTS_PER_POLYGON*3];
	
	while (nstack)
	{
		// Pop front.
		Moss_RecastNode* curNode = stack[0];
		for (int i = 0; i < nstack-1; ++i)
			stack[i] = stack[i+1];
		nstack--;
		
		// Get poly and tile.
		// The API input has been checked already, skip checking internal data.
		const dtPolyRef curRef = curNode->id;
		const Moss_RecastMeshTile* curTile = 0;
		const Moss_RecastPoly* curPoly = 0;
		m_nav->getTileAndPolyByRefUnsafe(curRef, &curTile, &curPoly);			
		
		// Collect vertices.
		const int nverts = curPoly->vertCount;
		for (int i = 0; i < nverts; ++i)
			dtVcopy(&verts[i*3], &curTile->verts[curPoly->verts[i]*3]);
		
		// If target is inside the poly, stop search.
		if (dtPointInPolygon(endPos, verts, nverts))
		{
			bestNode = curNode;
			dtVcopy(bestPos, endPos);
			break;
		}
		
		// Find wall edges and find nearest point inside the walls.
		for (int i = 0, j = (int)curPoly->vertCount-1; i < (int)curPoly->vertCount; j = i++)
		{
			// Find links to neighbours.
			static const int MAX_NEIS = 8;
			int nneis = 0;
			dtPolyRef neis[MAX_NEIS];
			
			if (curPoly->neis[j] & DT_EXT_LINK)
			{
				// Tile border.
				for (unsigned int k = curPoly->firstLink; k != DT_NULL_LINK; k = curTile->links[k].next)
				{
					const Moss_RecastLink* link = &curTile->links[k];
					if (link->edge == j)
					{
						if (link->ref != 0)
						{
							const Moss_RecastMeshTile* neiTile = 0;
							const Moss_RecastPoly* neiPoly = 0;
							m_nav->getTileAndPolyByRefUnsafe(link->ref, &neiTile, &neiPoly);
							if (filter->passFilter(link->ref, neiTile, neiPoly))
							{
								if (nneis < MAX_NEIS)
									neis[nneis++] = link->ref;
							}
						}
					}
				}
			}
			else if (curPoly->neis[j])
			{
				const unsigned int idx = (unsigned int)(curPoly->neis[j]-1);
				const dtPolyRef ref = m_nav->getPolyRefBase(curTile) | idx;
				if (filter->passFilter(ref, curTile, &curTile->polys[idx]))
				{
					// Internal edge, encode id.
					neis[nneis++] = ref;
				}
			}
			
			if (!nneis)
			{
				// Wall edge, calc distance.
				const float* vj = &verts[j*3];
				const float* vi = &verts[i*3];
				float tseg;
				const float distSqr = dtDistancePtSegSqr2D(endPos, vj, vi, tseg);
				if (distSqr < bestDist)
				{
                    // Update nearest distance.
					dtVlerp(bestPos, vj,vi, tseg);
					bestDist = distSqr;
					bestNode = curNode;
				}
			}
			else
			{
				for (int k = 0; k < nneis; ++k)
				{
					// Skip if no node can be allocated.
					Moss_RecastNode* neighbourNode = m_tinyNodePool->getNode(neis[k]);
					if (!neighbourNode)
						continue;
					// Skip if already visited.
					if (neighbourNode->flags & DT_NODE_CLOSED)
						continue;
					
					// Skip the link if it is too far from search constraint.
					// TODO: Maybe should use getPortalPoints(), but this one is way faster.
					const float* vj = &verts[j*3];
					const float* vi = &verts[i*3];
					float tseg;
					float distSqr = dtDistancePtSegSqr2D(searchPos, vj, vi, tseg);
					if (distSqr > searchRadSqr)
						continue;
					
					// Mark as the node as visited and push to queue.
					if (nstack < MAX_STACK)
					{
						neighbourNode->pidx = m_tinyNodePool->getNodeIdx(curNode);
						neighbourNode->flags |= DT_NODE_CLOSED;
						stack[nstack++] = neighbourNode;
					}
				}
			}
		}
	}
	
	int n = 0;
	if (bestNode)
	{
		// Reverse the path.
		Moss_RecastNode* prev = 0;
		Moss_RecastNode* node = bestNode;
		do
		{
			Moss_RecastNode* next = m_tinyNodePool->getNodeAtIdx(node->pidx);
			node->pidx = m_tinyNodePool->getNodeIdx(prev);
			prev = node;
			node = next;
		}
		while (node);
		
		// Store result
		node = prev;
		do
		{
			visited[n++] = node->id;
			if (n >= maxVisitedSize)
			{
				status |= DT_BUFFER_TOO_SMALL;
				break;
			}
			node = m_tinyNodePool->getNodeAtIdx(node->pidx);
		}
		while (node);
	}
	
	dtVcopy(resultPos, bestPos);
	
	*visitedCount = n;
	
	return status;
}


Moss_RecastStatus Moss_RecastNavMeshQuery::getPortalPoints(dtPolyRef from, dtPolyRef to, float* left, float* right,
										 unsigned char& fromType, unsigned char& toType) const
{
	dtAssert(m_nav);
	
	const Moss_RecastMeshTile* fromTile = 0;
	const Moss_RecastPoly* fromPoly = 0;
	if (dtStatusFailed(m_nav->getTileAndPolyByRef(from, &fromTile, &fromPoly)))
		return DT_FAILURE | DT_INVALID_PARAM;
	fromType = fromPoly->getType();

	const Moss_RecastMeshTile* toTile = 0;
	const Moss_RecastPoly* toPoly = 0;
	if (dtStatusFailed(m_nav->getTileAndPolyByRef(to, &toTile, &toPoly)))
		return DT_FAILURE | DT_INVALID_PARAM;
	toType = toPoly->getType();
		
	return getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, left, right);
}

// Returns portal points between two polygons.
Moss_RecastStatus Moss_RecastNavMeshQuery::getPortalPoints(dtPolyRef from, const Moss_RecastPoly* fromPoly, const Moss_RecastMeshTile* fromTile,
										 dtPolyRef to, const Moss_RecastPoly* toPoly, const Moss_RecastMeshTile* toTile,
										 float* left, float* right) const
{
	// Find the link that points to the 'to' polygon.
	const Moss_RecastLink* link = 0;
	for (unsigned int i = fromPoly->firstLink; i != DT_NULL_LINK; i = fromTile->links[i].next)
	{
		if (fromTile->links[i].ref == to)
		{
			link = &fromTile->links[i];
			break;
		}
	}
	if (!link)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	// Handle off-mesh connections.
	if (fromPoly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
	{
		// Find link that points to first vertex.
		for (unsigned int i = fromPoly->firstLink; i != DT_NULL_LINK; i = fromTile->links[i].next)
		{
			if (fromTile->links[i].ref == to)
			{
				const int v = fromTile->links[i].edge;
				dtVcopy(left, &fromTile->verts[fromPoly->verts[v]*3]);
				dtVcopy(right, &fromTile->verts[fromPoly->verts[v]*3]);
				return DT_SUCCESS;
			}
		}
		return DT_FAILURE | DT_INVALID_PARAM;
	}
	
	if (toPoly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
	{
		for (unsigned int i = toPoly->firstLink; i != DT_NULL_LINK; i = toTile->links[i].next)
		{
			if (toTile->links[i].ref == from)
			{
				const int v = toTile->links[i].edge;
				dtVcopy(left, &toTile->verts[toPoly->verts[v]*3]);
				dtVcopy(right, &toTile->verts[toPoly->verts[v]*3]);
				return DT_SUCCESS;
			}
		}
		return DT_FAILURE | DT_INVALID_PARAM;
	}
	
	// Find portal vertices.
	const int v0 = fromPoly->verts[link->edge];
	const int v1 = fromPoly->verts[(link->edge+1) % (int)fromPoly->vertCount];
	dtVcopy(left, &fromTile->verts[v0*3]);
	dtVcopy(right, &fromTile->verts[v1*3]);
	
	// If the link is at tile boundary, Clamp the vertices to
	// the link width.
	if (link->side != 0xff)
	{
		// Unpack portal limits.
		if (link->bmin != 0 || link->bmax != 255)
		{
			const float s = 1.0f/255.0f;
			const float tmin = link->bmin*s;
			const float tmax = link->bmax*s;
			dtVlerp(left, &fromTile->verts[v0*3], &fromTile->verts[v1*3], tmin);
			dtVlerp(right, &fromTile->verts[v0*3], &fromTile->verts[v1*3], tmax);
		}
	}
	
	return DT_SUCCESS;
}

// Returns edge mid point between two polygons.
Moss_RecastStatus Moss_RecastNavMeshQuery::getEdgeMidPoint(dtPolyRef from, dtPolyRef to, float* mid) const
{
	float left[3], right[3];
	unsigned char fromType, toType;
	if (dtStatusFailed(getPortalPoints(from, to, left,right, fromType, toType)))
		return DT_FAILURE | DT_INVALID_PARAM;
	mid[0] = (left[0]+right[0])*0.5f;
	mid[1] = (left[1]+right[1])*0.5f;
	mid[2] = (left[2]+right[2])*0.5f;
	return DT_SUCCESS;
}

Moss_RecastStatus Moss_RecastNavMeshQuery::getEdgeMidPoint(dtPolyRef from, const Moss_RecastPoly* fromPoly, const Moss_RecastMeshTile* fromTile,
										 dtPolyRef to, const Moss_RecastPoly* toPoly, const Moss_RecastMeshTile* toTile,
										 float* mid) const
{
	float left[3], right[3];
	if (dtStatusFailed(getPortalPoints(from, fromPoly, fromTile, to, toPoly, toTile, left, right)))
		return DT_FAILURE | DT_INVALID_PARAM;
	mid[0] = (left[0]+right[0])*0.5f;
	mid[1] = (left[1]+right[1])*0.5f;
	mid[2] = (left[2]+right[2])*0.5f;
	return DT_SUCCESS;
}



/// @par
///
/// This method is meant to be used for quick, short distance checks.
///
/// If the path array is too small to hold the result, it will be filled as 
/// far as possible from the start postion toward the end position.
///
/// <b>Using the Hit Parameter (t)</b>
/// 
/// If the hit parameter is a very high value (FLT_MAX), then the ray has hit 
/// the end position. In this case the path represents a valid corridor to the 
/// end position and the value of @p hitNormal is undefined.
///
/// If the hit parameter is zero, then the start position is on the wall that 
/// was hit and the value of @p hitNormal is undefined.
///
/// If 0 < t < 1.0 then the following applies:
///
/// @code
/// distanceToHitBorder = distanceToEndPosition * t
/// hitPoint = startPos + (endPos - startPos) * t
/// @endcode
///
/// <b>Use Case Restriction</b>
///
/// The raycast ignores the y-value of the end position. (2D check.) This 
/// places significant limits on how it can be used. For example:
///
/// Consider a scene where there is a main floor with a second floor balcony 
/// that hangs over the main floor. So the first floor mesh extends below the 
/// balcony mesh. The start position is somewhere on the first floor. The end 
/// position is on the balcony.
///
/// The raycast will search toward the end position along the first floor mesh. 
/// If it reaches the end position's xz-coordinates it will indicate FLT_MAX
/// (no wall hit), meaning it reached the end position. This is one example of why
/// this method is meant for short distance checks.
///
Moss_RecastStatus Moss_RecastNavMeshQuery::raycast(dtPolyRef startRef, const float* startPos, const float* endPos,
								 const Moss_RecastQueryFilter* filter,
								 float* t, float* hitNormal, dtPolyRef* path, int* pathCount, const int maxPath) const
{
	Moss_RecastRaycastHit hit;
	hit.path = path;
	hit.maxPath = maxPath;

	Moss_RecastStatus status = raycast(startRef, startPos, endPos, filter, 0, &hit);
	
	*t = hit.t;
	if (hitNormal)
		dtVcopy(hitNormal, hit.hitNormal);
	if (pathCount)
		*pathCount = hit.pathCount;

	return status;
}


/// @par
///
/// This method is meant to be used for quick, short distance checks.
///
/// If the path array is too small to hold the result, it will be filled as 
/// far as possible from the start postion toward the end position.
///
/// <b>Using the Hit Parameter t of RaycastHit</b>
/// 
/// If the hit parameter is a very high value (FLT_MAX), then the ray has hit 
/// the end position. In this case the path represents a valid corridor to the 
/// end position and the value of @p hitNormal is undefined.
///
/// If the hit parameter is zero, then the start position is on the wall that 
/// was hit and the value of @p hitNormal is undefined.
///
/// If 0 < t < 1.0 then the following applies:
///
/// @code
/// distanceToHitBorder = distanceToEndPosition * t
/// hitPoint = startPos + (endPos - startPos) * t
/// @endcode
///
/// <b>Use Case Restriction</b>
///
/// The raycast ignores the y-value of the end position. (2D check.) This 
/// places significant limits on how it can be used. For example:
///
/// Consider a scene where there is a main floor with a second floor balcony 
/// that hangs over the main floor. So the first floor mesh extends below the 
/// balcony mesh. The start position is somewhere on the first floor. The end 
/// position is on the balcony.
///
/// The raycast will search toward the end position along the first floor mesh. 
/// If it reaches the end position's xz-coordinates it will indicate FLT_MAX
/// (no wall hit), meaning it reached the end position. This is one example of why
/// this method is meant for short distance checks.
///
Moss_RecastStatus Moss_RecastNavMeshQuery::raycast(dtPolyRef startRef, const float* startPos, const float* endPos,
								 const Moss_RecastQueryFilter* filter, const unsigned int options,
								 Moss_RecastRaycastHit* hit, dtPolyRef prevRef) const
{
	dtAssert(m_nav);

	if (!hit)
		return DT_FAILURE | DT_INVALID_PARAM;

	hit->t = 0;
	hit->pathCount = 0;
	hit->pathCost = 0;

	// Validate input
	if (!m_nav->isValidPolyRef(startRef) ||
		!startPos || !dtVisfinite(startPos) ||
		!endPos || !dtVisfinite(endPos) ||
		!filter ||
		(prevRef && !m_nav->isValidPolyRef(prevRef)))
	{
		return DT_FAILURE | DT_INVALID_PARAM;
	}
	
	float dir[3], curPos[3], lastPos[3];
	float verts[DT_VERTS_PER_POLYGON*3+3];	
	int n = 0;

	dtVcopy(curPos, startPos);
	dtVsub(dir, endPos, startPos);
	dtVset(hit->hitNormal, 0, 0, 0);

	Moss_RecastStatus status = DT_SUCCESS;

	const Moss_RecastMeshTile* prevTile, *tile, *nextTile;
	const Moss_RecastPoly* prevPoly, *poly, *nextPoly;
	dtPolyRef curRef;

	// The API input has been checked already, skip checking internal data.
	curRef = startRef;
	tile = 0;
	poly = 0;
	m_nav->getTileAndPolyByRefUnsafe(curRef, &tile, &poly);
	nextTile = prevTile = tile;
	nextPoly = prevPoly = poly;
	if (prevRef)
		m_nav->getTileAndPolyByRefUnsafe(prevRef, &prevTile, &prevPoly);

	while (curRef)
	{
		// Cast ray against current polygon.
		
		// Collect vertices.
		int nv = 0;
		for (int i = 0; i < (int)poly->vertCount; ++i)
		{
			dtVcopy(&verts[nv*3], &tile->verts[poly->verts[i]*3]);
			nv++;
		}
		
		float tmin, tmax;
		int segMin, segMax;
		if (!dtIntersectSegmentPoly2D(startPos, endPos, verts, nv, tmin, tmax, segMin, segMax))
		{
			// Could not hit the polygon, keep the old t and report hit.
			hit->pathCount = n;
			return status;
		}

		hit->hitEdgeIndex = segMax;

		// Keep track of furthest t so far.
		if (tmax > hit->t)
			hit->t = tmax;
		
		// Store visited polygons.
		if (n < hit->maxPath)
			hit->path[n++] = curRef;
		else
			status |= DT_BUFFER_TOO_SMALL;

		// Ray end is completely inside the polygon.
		if (segMax == -1)
		{
			hit->t = FLT_MAX;
			hit->pathCount = n;
			
			// add the cost
			if (options & DT_RAYCAST_USE_COSTS)
				hit->pathCost += filter->getCost(curPos, endPos, prevRef, prevTile, prevPoly, curRef, tile, poly, curRef, tile, poly);
			return status;
		}

		// Follow neighbours.
		dtPolyRef nextRef = 0;
		
		for (unsigned int i = poly->firstLink; i != DT_NULL_LINK; i = tile->links[i].next)
		{
			const Moss_RecastLink* link = &tile->links[i];
			
			// Find link which contains this edge.
			if ((int)link->edge != segMax)
				continue;
			
			// Get pointer to the next polygon.
			nextTile = 0;
			nextPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(link->ref, &nextTile, &nextPoly);
			
			// Skip off-mesh connections.
			if (nextPoly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
				continue;
			
			// Skip links based on filter.
			if (!filter->passFilter(link->ref, nextTile, nextPoly))
				continue;
			
			// If the link is internal, just return the ref.
			if (link->side == 0xff)
			{
				nextRef = link->ref;
				break;
			}
			
			// If the link is at tile boundary,
			
			// Check if the link spans the whole edge, and accept.
			if (link->bmin == 0 && link->bmax == 255)
			{
				nextRef = link->ref;
				break;
			}
			
			// Check for partial edge links.
			const int v0 = poly->verts[link->edge];
			const int v1 = poly->verts[(link->edge+1) % poly->vertCount];
			const float* left = &tile->verts[v0*3];
			const float* right = &tile->verts[v1*3];
			
			// Check that the intersection lies inside the link portal.
			if (link->side == 0 || link->side == 4)
			{
				// Calculate link size.
				const float s = 1.0f/255.0f;
				float lmin = left[2] + (right[2] - left[2])*(link->bmin*s);
				float lmax = left[2] + (right[2] - left[2])*(link->bmax*s);
				if (lmin > lmax) Swap(lmin, lmax);
				
				// Find Z intersection.
				float z = startPos[2] + (endPos[2]-startPos[2])*tmax;
				if (z >= lmin && z <= lmax)
				{
					nextRef = link->ref;
					break;
				}
			}
			else if (link->side == 2 || link->side == 6)
			{
				// Calculate link size.
				const float s = 1.0f/255.0f;
				float lmin = left[0] + (right[0] - left[0])*(link->bmin*s);
				float lmax = left[0] + (right[0] - left[0])*(link->bmax*s);
				if (lmin > lmax) Swap(lmin, lmax);
				
				// Find X intersection.
				float x = startPos[0] + (endPos[0]-startPos[0])*tmax;
				if (x >= lmin && x <= lmax)
				{
					nextRef = link->ref;
					break;
				}
			}
		}
		
		// add the cost
		if (options & DT_RAYCAST_USE_COSTS)
		{
			// compute the intersection point at the furthest end of the polygon
			// and correct the height (since the raycast moves in 2d)
			dtVcopy(lastPos, curPos);
			dtVmad(curPos, startPos, dir, hit->t);
			float* e1 = &verts[segMax*3];
			float* e2 = &verts[((segMax+1)%nv)*3];
			float eDir[3], diff[3];
			dtVsub(eDir, e2, e1);
			dtVsub(diff, curPos, e1);
			float s = Sqr(eDir[0]) > Sqr(eDir[2]) ? diff[0] / eDir[0] : diff[2] / eDir[2];
			curPos[1] = e1[1] + eDir[1] * s;

			hit->pathCost += filter->getCost(lastPos, curPos, prevRef, prevTile, prevPoly, curRef, tile, poly, nextRef, nextTile, nextPoly);
		}

		if (!nextRef)
		{
			// No neighbour, we hit a wall.
			
			// Calculate hit normal.
			const int a = segMax;
			const int b = segMax+1 < nv ? segMax+1 : 0;
			const float* va = &verts[a*3];
			const float* vb = &verts[b*3];
			const float dx = vb[0] - va[0];
			const float dz = vb[2] - va[2];
			hit->hitNormal[0] = dz;
			hit->hitNormal[1] = 0;
			hit->hitNormal[2] = -dx;
			rcVnormalize(hit->hitNormal);
			
			hit->pathCount = n;
			return status;
		}

		// No hit, advance to neighbour polygon.
		prevRef = curRef;
		curRef = nextRef;
		prevTile = tile;
		tile = nextTile;
		prevPoly = poly;
		poly = nextPoly;

		if (status & DT_BUFFER_TOO_SMALL)
		{
			status |= DT_PARTIAL_RESULT;
			break;
		}
	}
	
	hit->pathCount = n;
	
	return status;
}

/// @par
///
/// At least one result array must be provided.
///
/// The order of the result set is from least to highest cost to reach the polygon.
///
/// A common use case for this method is to perform Dijkstra searches. 
/// Candidate polygons are found by searching the graph beginning at the start polygon.
///
/// If a polygon is not found via the graph search, even if it intersects the 
/// search circle, it will not be included in the result set. For example:
///
/// polyA is the start polygon.
/// polyB shares an edge with polyA. (Is adjacent.)
/// polyC shares an edge with polyB, but not with polyA
/// Even if the search circle overlaps polyC, it will not be included in the 
/// result set unless polyB is also in the set.
/// 
/// The value of the center point is used as the start position for cost 
/// calculations. It is not projected onto the surface of the mesh, so its 
/// y-value will effect the costs.
///
/// Intersection tests occur in 2D. All polygons and the search circle are 
/// projected onto the xz-plane. So the y-value of the center point does not 
/// effect intersection tests.
///
/// If the result arrays are to small to hold the entire result set, they will be 
/// filled to capacity.
/// 
Moss_RecastStatus Moss_RecastNavMeshQuery::findPolysAroundCircle(dtPolyRef startRef, const float* centerPos, const float radius,
											   const Moss_RecastQueryFilter* filter,
											   dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
											   int* resultCount, const int maxResult) const
{
	dtAssert(m_nav);
	dtAssert(m_nodePool);
	dtAssert(m_openList);

	if (!resultCount)
		return DT_FAILURE | DT_INVALID_PARAM;

	*resultCount = 0;

	if (!m_nav->isValidPolyRef(startRef) ||
		!centerPos || !dtVisfinite(centerPos) ||
		radius < 0 || !dtMathIsfinite(radius) ||
		!filter || maxResult < 0)
	{
		return DT_FAILURE | DT_INVALID_PARAM;
	}
	
	m_nodePool->clear();
	m_openList->clear();
	
	Moss_RecastNode* startNode = m_nodePool->getNode(startRef);
	dtVcopy(startNode->pos, centerPos);
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = 0;
	startNode->id = startRef;
	startNode->flags = DT_NODE_OPEN;
	m_openList->push(startNode);
	
	Moss_RecastStatus status = DT_SUCCESS;
	
	int n = 0;
	
	const float radiusSqr = Sqr(radius);
	
	while (!m_openList->empty())
	{
		Moss_RecastNode* bestNode = m_openList->pop();
		bestNode->flags &= ~DT_NODE_OPEN;
		bestNode->flags |= DT_NODE_CLOSED;
		
		// Get poly and tile.
		// The API input has been checked already, skip checking internal data.
		const dtPolyRef bestRef = bestNode->id;
		const Moss_RecastMeshTile* bestTile = 0;
		const Moss_RecastPoly* bestPoly = 0;
		m_nav->getTileAndPolyByRefUnsafe(bestRef, &bestTile, &bestPoly);
		
		// Get parent poly and tile.
		dtPolyRef parentRef = 0;
		const Moss_RecastMeshTile* parentTile = 0;
		const Moss_RecastPoly* parentPoly = 0;
		if (bestNode->pidx)
			parentRef = m_nodePool->getNodeAtIdx(bestNode->pidx)->id;
		if (parentRef)
			m_nav->getTileAndPolyByRefUnsafe(parentRef, &parentTile, &parentPoly);

		if (n < maxResult)
		{
			if (resultRef)
				resultRef[n] = bestRef;
			if (resultParent)
				resultParent[n] = parentRef;
			if (resultCost)
				resultCost[n] = bestNode->total;
			++n;
		}
		else
		{
			status |= DT_BUFFER_TOO_SMALL;
		}
		
		for (unsigned int i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
		{
			const Moss_RecastLink* link = &bestTile->links[i];
			dtPolyRef neighbourRef = link->ref;
			// Skip invalid neighbours and do not follow back to parent.
			if (!neighbourRef || neighbourRef == parentRef)
				continue;
			
			// Expand to neighbour
			const Moss_RecastMeshTile* neighbourTile = 0;
			const Moss_RecastPoly* neighbourPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);
		
			// Do not advance if the polygon is excluded by the filter.
			if (!filter->passFilter(neighbourRef, neighbourTile, neighbourPoly))
				continue;
			
			// Find edge and calc distance to the edge.
			float va[3], vb[3];
			if (!getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile, va, vb))
				continue;
			
			// If the circle is not touching the next polygon, skip it.
			float tseg;
			float distSqr = dtDistancePtSegSqr2D(centerPos, va, vb, tseg);
			if (distSqr > radiusSqr)
				continue;
			
			Moss_RecastNode* neighbourNode = m_nodePool->getNode(neighbourRef);
			if (!neighbourNode)
			{
				status |= DT_OUT_OF_NODES;
				continue;
			}
				
			if (neighbourNode->flags & DT_NODE_CLOSED)
				continue;
			
			// Cost
			if (neighbourNode->flags == 0)
				dtVlerp(neighbourNode->pos, va, vb, 0.5f);
			
			float cost = filter->getCost(
				bestNode->pos, neighbourNode->pos,
				parentRef, parentTile, parentPoly,
				bestRef, bestTile, bestPoly,
				neighbourRef, neighbourTile, neighbourPoly);

			const float total = bestNode->total + cost;
			
			// The node is already in open list and the new result is worse, skip.
			if ((neighbourNode->flags & DT_NODE_OPEN) && total >= neighbourNode->total)
				continue;
			
			neighbourNode->id = neighbourRef;
			neighbourNode->pidx = m_nodePool->getNodeIdx(bestNode);
			neighbourNode->total = total;
			
			if (neighbourNode->flags & DT_NODE_OPEN)
			{
				m_openList->modify(neighbourNode);
			}
			else
			{
				neighbourNode->flags = DT_NODE_OPEN;
				m_openList->push(neighbourNode);
			}
		}
	}
	
	*resultCount = n;
	
	return status;
}

/// @par
///
/// The order of the result set is from least to highest cost.
/// 
/// At least one result array must be provided.
///
/// A common use case for this method is to perform Dijkstra searches. 
/// Candidate polygons are found by searching the graph beginning at the start 
/// polygon.
/// 
/// The same intersection test restrictions that apply to findPolysAroundCircle()
/// method apply to this method.
/// 
/// The 3D centroid of the search polygon is used as the start position for cost 
/// calculations.
/// 
/// Intersection tests occur in 2D. All polygons are projected onto the 
/// xz-plane. So the y-values of the vertices do not effect intersection tests.
/// 
/// If the result arrays are is too small to hold the entire result set, they will 
/// be filled to capacity.
///
Moss_RecastStatus Moss_RecastNavMeshQuery::findPolysAroundShape(dtPolyRef startRef, const float* verts, const int nverts,
											  const Moss_RecastQueryFilter* filter,
											  dtPolyRef* resultRef, dtPolyRef* resultParent, float* resultCost,
											  int* resultCount, const int maxResult) const
{
	dtAssert(m_nav);
	dtAssert(m_nodePool);
	dtAssert(m_openList);

	if (!resultCount)
		return DT_FAILURE | DT_INVALID_PARAM;

	*resultCount = 0;

	if (!m_nav->isValidPolyRef(startRef) ||
		!verts || nverts < 3 ||
		!filter || maxResult < 0)
	{
		return DT_FAILURE | DT_INVALID_PARAM;
	}
	
	// Validate input
	if (!startRef || !m_nav->isValidPolyRef(startRef))
		return DT_FAILURE | DT_INVALID_PARAM;
	
	m_nodePool->clear();
	m_openList->clear();
	
	float centerPos[3] = {0,0,0};
	for (int i = 0; i < nverts; ++i)
		dtVadd(centerPos,centerPos,&verts[i*3]);
	dtVscale(centerPos,centerPos,1.0f/nverts);

	Moss_RecastNode* startNode = m_nodePool->getNode(startRef);
	dtVcopy(startNode->pos, centerPos);
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = 0;
	startNode->id = startRef;
	startNode->flags = DT_NODE_OPEN;
	m_openList->push(startNode);
	
	Moss_RecastStatus status = DT_SUCCESS;

	int n = 0;
	
	while (!m_openList->empty())
	{
		Moss_RecastNode* bestNode = m_openList->pop();
		bestNode->flags &= ~DT_NODE_OPEN;
		bestNode->flags |= DT_NODE_CLOSED;
		
		// Get poly and tile.
		// The API input has been checked already, skip checking internal data.
		const dtPolyRef bestRef = bestNode->id;
		const Moss_RecastMeshTile* bestTile = 0;
		const Moss_RecastPoly* bestPoly = 0;
		m_nav->getTileAndPolyByRefUnsafe(bestRef, &bestTile, &bestPoly);
		
		// Get parent poly and tile.
		dtPolyRef parentRef = 0;
		const Moss_RecastMeshTile* parentTile = 0;
		const Moss_RecastPoly* parentPoly = 0;
		if (bestNode->pidx)
			parentRef = m_nodePool->getNodeAtIdx(bestNode->pidx)->id;
		if (parentRef)
			m_nav->getTileAndPolyByRefUnsafe(parentRef, &parentTile, &parentPoly);

		if (n < maxResult)
		{
			if (resultRef)
				resultRef[n] = bestRef;
			if (resultParent)
				resultParent[n] = parentRef;
			if (resultCost)
				resultCost[n] = bestNode->total;

			++n;
		}
		else
		{
			status |= DT_BUFFER_TOO_SMALL;
		}
		
		for (unsigned int i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
		{
			const Moss_RecastLink* link = &bestTile->links[i];
			dtPolyRef neighbourRef = link->ref;
			// Skip invalid neighbours and do not follow back to parent.
			if (!neighbourRef || neighbourRef == parentRef)
				continue;
			
			// Expand to neighbour
			const Moss_RecastMeshTile* neighbourTile = 0;
			const Moss_RecastPoly* neighbourPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);
			
			// Do not advance if the polygon is excluded by the filter.
			if (!filter->passFilter(neighbourRef, neighbourTile, neighbourPoly))
				continue;
			
			// Find edge and calc distance to the edge.
			float va[3], vb[3];
			if (!getPortalPoints(bestRef, bestPoly, bestTile, neighbourRef, neighbourPoly, neighbourTile, va, vb))
				continue;
			
			// If the poly is not touching the edge to the next polygon, skip the connection it.
			float tmin, tmax;
			int segMin, segMax;
			if (!dtIntersectSegmentPoly2D(va, vb, verts, nverts, tmin, tmax, segMin, segMax))
				continue;
			if (tmin > 1.0f || tmax < 0.0f)
				continue;
			
			Moss_RecastNode* neighbourNode = m_nodePool->getNode(neighbourRef);
			if (!neighbourNode)
			{
				status |= DT_OUT_OF_NODES;
				continue;
			}
			
			if (neighbourNode->flags & DT_NODE_CLOSED)
				continue;
			
			// Cost
			if (neighbourNode->flags == 0)
				dtVlerp(neighbourNode->pos, va, vb, 0.5f);
			
			float cost = filter->getCost(
				bestNode->pos, neighbourNode->pos,
				parentRef, parentTile, parentPoly,
				bestRef, bestTile, bestPoly,
				neighbourRef, neighbourTile, neighbourPoly);

			const float total = bestNode->total + cost;
			
			// The node is already in open list and the new result is worse, skip.
			if ((neighbourNode->flags & DT_NODE_OPEN) && total >= neighbourNode->total)
				continue;
			
			neighbourNode->id = neighbourRef;
			neighbourNode->pidx = m_nodePool->getNodeIdx(bestNode);
			neighbourNode->total = total;
			
			if (neighbourNode->flags & DT_NODE_OPEN)
			{
				m_openList->modify(neighbourNode);
			}
			else
			{
				neighbourNode->flags = DT_NODE_OPEN;
				m_openList->push(neighbourNode);
			}
		}
	}
	
	*resultCount = n;
	
	return status;
}

Moss_RecastStatus Moss_RecastNavMeshQuery::getPathFromDijkstraSearch(dtPolyRef endRef, dtPolyRef* path, int* pathCount, int maxPath) const
{
	if (!m_nav->isValidPolyRef(endRef) || !path || !pathCount || maxPath < 0)
		return DT_FAILURE | DT_INVALID_PARAM;

	*pathCount = 0;

	Moss_RecastNode* endNode;
	if (m_nodePool->findNodes(endRef, &endNode, 1) != 1 ||
		(endNode->flags & DT_NODE_CLOSED) == 0)
		return DT_FAILURE | DT_INVALID_PARAM;

	return getPathToNode(endNode, path, pathCount, maxPath);
}

/// @par
///
/// This method is optimized for a small search radius and small number of result 
/// polygons.
///
/// Candidate polygons are found by searching the navigation graph beginning at 
/// the start polygon.
///
/// The same intersection test restrictions that apply to the findPolysAroundCircle 
/// mehtod applies to this method.
///
/// The value of the center point is used as the start point for cost calculations. 
/// It is not projected onto the surface of the mesh, so its y-value will effect 
/// the costs.
/// 
/// Intersection tests occur in 2D. All polygons and the search circle are 
/// projected onto the xz-plane. So the y-value of the center point does not 
/// effect intersection tests.
/// 
/// If the result arrays are is too small to hold the entire result set, they will 
/// be filled to capacity.
/// 
Moss_RecastStatus Moss_RecastNavMeshQuery::findLocalNeighbourhood(dtPolyRef startRef, const float* centerPos, const float radius,
												const Moss_RecastQueryFilter* filter,
												dtPolyRef* resultRef, dtPolyRef* resultParent,
												int* resultCount, const int maxResult) const
{
	dtAssert(m_nav);
	dtAssert(m_tinyNodePool);

	if (!resultCount)
		return DT_FAILURE | DT_INVALID_PARAM;

	*resultCount = 0;

	if (!m_nav->isValidPolyRef(startRef) ||
		!centerPos || !dtVisfinite(centerPos) ||
		radius < 0 || !dtMathIsfinite(radius) ||
		!filter || maxResult < 0)
	{
		return DT_FAILURE | DT_INVALID_PARAM;
	}

	static const int MAX_STACK = 48;
	Moss_RecastNode* stack[MAX_STACK];
	int nstack = 0;
	
	m_tinyNodePool->clear();
	
	Moss_RecastNode* startNode = m_tinyNodePool->getNode(startRef);
	startNode->pidx = 0;
	startNode->id = startRef;
	startNode->flags = DT_NODE_CLOSED;
	stack[nstack++] = startNode;
	
	const float radiusSqr = Sqr(radius);
	
	float pa[DT_VERTS_PER_POLYGON*3];
	float pb[DT_VERTS_PER_POLYGON*3];
	
	Moss_RecastStatus status = DT_SUCCESS;
	
	int n = 0;
	if (n < maxResult)
	{
		resultRef[n] = startNode->id;
		if (resultParent)
			resultParent[n] = 0;
		++n;
	}
	else
	{
		status |= DT_BUFFER_TOO_SMALL;
	}
	
	while (nstack)
	{
		// Pop front.
		Moss_RecastNode* curNode = stack[0];
		for (int i = 0; i < nstack-1; ++i)
			stack[i] = stack[i+1];
		nstack--;
		
		// Get poly and tile.
		// The API input has been checked already, skip checking internal data.
		const dtPolyRef curRef = curNode->id;
		const Moss_RecastMeshTile* curTile = 0;
		const Moss_RecastPoly* curPoly = 0;
		m_nav->getTileAndPolyByRefUnsafe(curRef, &curTile, &curPoly);
		
		for (unsigned int i = curPoly->firstLink; i != DT_NULL_LINK; i = curTile->links[i].next)
		{
			const Moss_RecastLink* link = &curTile->links[i];
			dtPolyRef neighbourRef = link->ref;
			// Skip invalid neighbours.
			if (!neighbourRef)
				continue;
			
			// Skip if cannot alloca more nodes.
			Moss_RecastNode* neighbourNode = m_tinyNodePool->getNode(neighbourRef);
			if (!neighbourNode)
				continue;
			// Skip visited.
			if (neighbourNode->flags & DT_NODE_CLOSED)
				continue;
			
			// Expand to neighbour
			const Moss_RecastMeshTile* neighbourTile = 0;
			const Moss_RecastPoly* neighbourPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);
			
			// Skip off-mesh connections.
			if (neighbourPoly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
				continue;
			
			// Do not advance if the polygon is excluded by the filter.
			if (!filter->passFilter(neighbourRef, neighbourTile, neighbourPoly))
				continue;
			
			// Find edge and calc distance to the edge.
			float va[3], vb[3];
			if (!getPortalPoints(curRef, curPoly, curTile, neighbourRef, neighbourPoly, neighbourTile, va, vb))
				continue;
			
			// If the circle is not touching the next polygon, skip it.
			float tseg;
			float distSqr = dtDistancePtSegSqr2D(centerPos, va, vb, tseg);
			if (distSqr > radiusSqr)
				continue;
			
			// Mark node visited, this is done before the overlap test so that
			// we will not visit the poly again if the test fails.
			neighbourNode->flags |= DT_NODE_CLOSED;
			neighbourNode->pidx = m_tinyNodePool->getNodeIdx(curNode);
			
			// Check that the polygon does not collide with existing polygons.
			
			// Collect vertices of the neighbour poly.
			const int npa = neighbourPoly->vertCount;
			for (int k = 0; k < npa; ++k)
				dtVcopy(&pa[k*3], &neighbourTile->verts[neighbourPoly->verts[k]*3]);
			
			bool overlap = false;
			for (int j = 0; j < n; ++j)
			{
				dtPolyRef pastRef = resultRef[j];
				
				// Connected polys do not overlap.
				bool connected = false;
				for (unsigned int k = curPoly->firstLink; k != DT_NULL_LINK; k = curTile->links[k].next)
				{
					if (curTile->links[k].ref == pastRef)
					{
						connected = true;
						break;
					}
				}
				if (connected)
					continue;
				
				// Potentially overlapping.
				const Moss_RecastMeshTile* pastTile = 0;
				const Moss_RecastPoly* pastPoly = 0;
				m_nav->getTileAndPolyByRefUnsafe(pastRef, &pastTile, &pastPoly);
				
				// Get vertices and test overlap
				const int npb = pastPoly->vertCount;
				for (int k = 0; k < npb; ++k)
					dtVcopy(&pb[k*3], &pastTile->verts[pastPoly->verts[k]*3]);
				
				if (dtOverlapPolyPoly2D(pa,npa, pb,npb))
				{
					overlap = true;
					break;
				}
			}
			if (overlap)
				continue;
			
			// This poly is fine, store and advance to the poly.
			if (n < maxResult)
			{
				resultRef[n] = neighbourRef;
				if (resultParent)
					resultParent[n] = curRef;
				++n;
			}
			else
			{
				status |= DT_BUFFER_TOO_SMALL;
			}
			
			if (nstack < MAX_STACK)
			{
				stack[nstack++] = neighbourNode;
			}
		}
	}
	
	*resultCount = n;
	
	return status;
}


struct dtSegInterval
{
	dtPolyRef ref;
	short tmin, tmax;
};

static void insertInterval(dtSegInterval* ints, int& nints, const int maxInts,
						   const short tmin, const short tmax, const dtPolyRef ref)
{
	if (nints+1 > maxInts) return;
	// Find insertion point.
	int idx = 0;
	while (idx < nints)
	{
		if (tmax <= ints[idx].tmin)
			break;
		idx++;
	}
	// Move current results.
	if (nints-idx)
		memmove(ints+idx+1, ints+idx, sizeof(dtSegInterval)*(nints-idx));
	// Store
	ints[idx].ref = ref;
	ints[idx].tmin = tmin;
	ints[idx].tmax = tmax;
	nints++;
}

/// @par
///
/// If the @p segmentRefs parameter is provided, then all polygon segments will be returned. 
/// Otherwise only the wall segments are returned.
/// 
/// A segment that is normally a portal will be included in the result set as a 
/// wall if the @p filter results in the neighbor polygon becoomming impassable.
/// 
/// The @p segmentVerts and @p segmentRefs buffers should normally be sized for the 
/// maximum segments per polygon of the source navigation mesh.
/// 
Moss_RecastStatus Moss_RecastNavMeshQuery::getPolyWallSegments(dtPolyRef ref, const Moss_RecastQueryFilter* filter,
											 float* segmentVerts, dtPolyRef* segmentRefs, int* segmentCount,
											 const int maxSegments) const
{
	dtAssert(m_nav);

	if (!segmentCount)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	*segmentCount = 0;

	const Moss_RecastMeshTile* tile = 0;
	const Moss_RecastPoly* poly = 0;
	if (dtStatusFailed(m_nav->getTileAndPolyByRef(ref, &tile, &poly)))
		return DT_FAILURE | DT_INVALID_PARAM;

	if (!filter || !segmentVerts || maxSegments < 0)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	int n = 0;
	static const int MAX_INTERVAL = 16;
	dtSegInterval ints[MAX_INTERVAL];
	int nints;
	
	const bool storePortals = segmentRefs != 0;
	
	Moss_RecastStatus status = DT_SUCCESS;
	
	for (int i = 0, j = (int)poly->vertCount-1; i < (int)poly->vertCount; j = i++)
	{
		// Skip non-solid edges.
		nints = 0;
		if (poly->neis[j] & DT_EXT_LINK)
		{
			// Tile border.
			for (unsigned int k = poly->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
			{
				const Moss_RecastLink* link = &tile->links[k];
				if (link->edge == j)
				{
					if (link->ref != 0)
					{
						const Moss_RecastMeshTile* neiTile = 0;
						const Moss_RecastPoly* neiPoly = 0;
						m_nav->getTileAndPolyByRefUnsafe(link->ref, &neiTile, &neiPoly);
						if (filter->passFilter(link->ref, neiTile, neiPoly))
						{
							insertInterval(ints, nints, MAX_INTERVAL, link->bmin, link->bmax, link->ref);
						}
					}
				}
			}
		}
		else
		{
			// Internal edge
			dtPolyRef neiRef = 0;
			if (poly->neis[j])
			{
				const unsigned int idx = (unsigned int)(poly->neis[j]-1);
				neiRef = m_nav->getPolyRefBase(tile) | idx;
				if (!filter->passFilter(neiRef, tile, &tile->polys[idx]))
					neiRef = 0;
			}

			// If the edge leads to another polygon and portals are not stored, skip.
			if (neiRef != 0 && !storePortals)
				continue;
			
			if (n < maxSegments)
			{
				const float* vj = &tile->verts[poly->verts[j]*3];
				const float* vi = &tile->verts[poly->verts[i]*3];
				float* seg = &segmentVerts[n*6];
				dtVcopy(seg+0, vj);
				dtVcopy(seg+3, vi);
				if (segmentRefs)
					segmentRefs[n] = neiRef;
				n++;
			}
			else
			{
				status |= DT_BUFFER_TOO_SMALL;
			}
			
			continue;
		}
		
		// Add sentinels
		insertInterval(ints, nints, MAX_INTERVAL, -1, 0, 0);
		insertInterval(ints, nints, MAX_INTERVAL, 255, 256, 0);
		
		// Store segments.
		const float* vj = &tile->verts[poly->verts[j]*3];
		const float* vi = &tile->verts[poly->verts[i]*3];
		for (int k = 1; k < nints; ++k)
		{
			// Portal segment.
			if (storePortals && ints[k].ref)
			{
				const float tmin = ints[k].tmin/255.0f; 
				const float tmax = ints[k].tmax/255.0f; 
				if (n < maxSegments)
				{
					float* seg = &segmentVerts[n*6];
					dtVlerp(seg+0, vj,vi, tmin);
					dtVlerp(seg+3, vj,vi, tmax);
					if (segmentRefs)
						segmentRefs[n] = ints[k].ref;
					n++;
				}
				else
				{
					status |= DT_BUFFER_TOO_SMALL;
				}
			}

			// Wall segment.
			const int imin = ints[k-1].tmax;
			const int imax = ints[k].tmin;
			if (imin != imax)
			{
				const float tmin = imin/255.0f; 
				const float tmax = imax/255.0f; 
				if (n < maxSegments)
				{
					float* seg = &segmentVerts[n*6];
					dtVlerp(seg+0, vj,vi, tmin);
					dtVlerp(seg+3, vj,vi, tmax);
					if (segmentRefs)
						segmentRefs[n] = 0;
					n++;
				}
				else
				{
					status |= DT_BUFFER_TOO_SMALL;
				}
			}
		}
	}
	
	*segmentCount = n;
	
	return status;
}

/// @par
///
/// @p hitPos is not adjusted using the height detail data.
///
/// @p hitDist will equal the search radius if there is no wall within the 
/// radius. In this case the values of @p hitPos and @p hitNormal are
/// undefined.
///
/// The normal will become unpredicable if @p hitDist is a very small number.
///
Moss_RecastStatus Moss_RecastNavMeshQuery::findDistanceToWall(dtPolyRef startRef, const float* centerPos, const float maxRadius,
											const Moss_RecastQueryFilter* filter,
											float* hitDist, float* hitPos, float* hitNormal) const
{
	dtAssert(m_nav);
	dtAssert(m_nodePool);
	dtAssert(m_openList);
	
	// Validate input
	if (!m_nav->isValidPolyRef(startRef) ||
		!centerPos || !dtVisfinite(centerPos) ||
		maxRadius < 0 || !dtMathIsfinite(maxRadius) ||
		!filter || !hitDist || !hitPos || !hitNormal)
	{
		return DT_FAILURE | DT_INVALID_PARAM;
	}
	
	m_nodePool->clear();
	m_openList->clear();
	
	Moss_RecastNode* startNode = m_nodePool->getNode(startRef);
	dtVcopy(startNode->pos, centerPos);
	startNode->pidx = 0;
	startNode->cost = 0;
	startNode->total = 0;
	startNode->id = startRef;
	startNode->flags = DT_NODE_OPEN;
	m_openList->push(startNode);
	
	float radiusSqr = Sqr(maxRadius);
	
	Moss_RecastStatus status = DT_SUCCESS;
	
	while (!m_openList->empty())
	{
		Moss_RecastNode* bestNode = m_openList->pop();
		bestNode->flags &= ~DT_NODE_OPEN;
		bestNode->flags |= DT_NODE_CLOSED;
		
		// Get poly and tile.
		// The API input has been checked already, skip checking internal data.
		const dtPolyRef bestRef = bestNode->id;
		const Moss_RecastMeshTile* bestTile = 0;
		const Moss_RecastPoly* bestPoly = 0;
		m_nav->getTileAndPolyByRefUnsafe(bestRef, &bestTile, &bestPoly);
		
		// Get parent poly and tile.
		dtPolyRef parentRef = 0;
		const Moss_RecastMeshTile* parentTile = 0;
		const Moss_RecastPoly* parentPoly = 0;
		if (bestNode->pidx)
			parentRef = m_nodePool->getNodeAtIdx(bestNode->pidx)->id;
		if (parentRef)
			m_nav->getTileAndPolyByRefUnsafe(parentRef, &parentTile, &parentPoly);
		
		// Hit test walls.
		for (int i = 0, j = (int)bestPoly->vertCount-1; i < (int)bestPoly->vertCount; j = i++)
		{
			// Skip non-solid edges.
			if (bestPoly->neis[j] & DT_EXT_LINK)
			{
				// Tile border.
				bool solid = true;
				for (unsigned int k = bestPoly->firstLink; k != DT_NULL_LINK; k = bestTile->links[k].next)
				{
					const Moss_RecastLink* link = &bestTile->links[k];
					if (link->edge == j)
					{
						if (link->ref != 0)
						{
							const Moss_RecastMeshTile* neiTile = 0;
							const Moss_RecastPoly* neiPoly = 0;
							m_nav->getTileAndPolyByRefUnsafe(link->ref, &neiTile, &neiPoly);
							if (filter->passFilter(link->ref, neiTile, neiPoly))
								solid = false;
						}
						break;
					}
				}
				if (!solid) continue;
			}
			else if (bestPoly->neis[j])
			{
				// Internal edge
				const unsigned int idx = (unsigned int)(bestPoly->neis[j]-1);
				const dtPolyRef ref = m_nav->getPolyRefBase(bestTile) | idx;
				if (filter->passFilter(ref, bestTile, &bestTile->polys[idx]))
					continue;
			}
			
			// Calc distance to the edge.
			const float* vj = &bestTile->verts[bestPoly->verts[j]*3];
			const float* vi = &bestTile->verts[bestPoly->verts[i]*3];
			float tseg;
			float distSqr = dtDistancePtSegSqr2D(centerPos, vj, vi, tseg);
			
			// Edge is too far, skip.
			if (distSqr > radiusSqr)
				continue;
			
			// Hit wall, update radius.
			radiusSqr = distSqr;
			// Calculate hit pos.
			hitPos[0] = vj[0] + (vi[0] - vj[0])*tseg;
			hitPos[1] = vj[1] + (vi[1] - vj[1])*tseg;
			hitPos[2] = vj[2] + (vi[2] - vj[2])*tseg;
		}
		
		for (unsigned int i = bestPoly->firstLink; i != DT_NULL_LINK; i = bestTile->links[i].next)
		{
			const Moss_RecastLink* link = &bestTile->links[i];
			dtPolyRef neighbourRef = link->ref;
			// Skip invalid neighbours and do not follow back to parent.
			if (!neighbourRef || neighbourRef == parentRef)
				continue;
			
			// Expand to neighbour.
			const Moss_RecastMeshTile* neighbourTile = 0;
			const Moss_RecastPoly* neighbourPoly = 0;
			m_nav->getTileAndPolyByRefUnsafe(neighbourRef, &neighbourTile, &neighbourPoly);
			
			// Skip off-mesh connections.
			if (neighbourPoly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
				continue;
			
			// Calc distance to the edge.
			const float* va = &bestTile->verts[bestPoly->verts[link->edge]*3];
			const float* vb = &bestTile->verts[bestPoly->verts[(link->edge+1) % bestPoly->vertCount]*3];
			float tseg;
			float distSqr = dtDistancePtSegSqr2D(centerPos, va, vb, tseg);
			
			// If the circle is not touching the next polygon, skip it.
			if (distSqr > radiusSqr)
				continue;
			
			if (!filter->passFilter(neighbourRef, neighbourTile, neighbourPoly))
				continue;

			Moss_RecastNode* neighbourNode = m_nodePool->getNode(neighbourRef);
			if (!neighbourNode)
			{
				status |= DT_OUT_OF_NODES;
				continue;
			}
			
			if (neighbourNode->flags & DT_NODE_CLOSED)
				continue;
			
			// Cost
			if (neighbourNode->flags == 0)
			{
				getEdgeMidPoint(bestRef, bestPoly, bestTile,
								neighbourRef, neighbourPoly, neighbourTile, neighbourNode->pos);
			}
			
			const float total = bestNode->total + rcVdist(bestNode->pos, neighbourNode->pos);
			
			// The node is already in open list and the new result is worse, skip.
			if ((neighbourNode->flags & DT_NODE_OPEN) && total >= neighbourNode->total)
				continue;
			
			neighbourNode->id = neighbourRef;
			neighbourNode->flags = (neighbourNode->flags & ~DT_NODE_CLOSED);
			neighbourNode->pidx = m_nodePool->getNodeIdx(bestNode);
			neighbourNode->total = total;
				
			if (neighbourNode->flags & DT_NODE_OPEN)
			{
				m_openList->modify(neighbourNode);
			}
			else
			{
				neighbourNode->flags |= DT_NODE_OPEN;
				m_openList->push(neighbourNode);
			}
		}
	}
	
	// Calc hit normal.
	dtVsub(hitNormal, centerPos, hitPos);
	rcVnormalize(hitNormal);
	
	*hitDist = sqrtf(radiusSqr);
	
	return status;
}

bool Moss_RecastNavMeshQuery::isValidPolyRef(dtPolyRef ref, const Moss_RecastQueryFilter* filter) const
{
	const Moss_RecastMeshTile* tile = 0;
	const Moss_RecastPoly* poly = 0;
	Moss_RecastStatus status = m_nav->getTileAndPolyByRef(ref, &tile, &poly);
	// If cannot get polygon, assume it does not exists and boundary is invalid.
	if (dtStatusFailed(status))
		return false;
	// If cannot pass filter, assume flags has changed and boundary is invalid.
	if (!filter->passFilter(ref, tile, poly))
		return false;
	return true;
}

/// @par
///
/// The closed list is the list of polygons that were fully evaluated during 
/// the last navigation graph search. (A* or Dijkstra)
/// 
bool Moss_RecastNavMeshQuery::isInClosedList(dtPolyRef ref) const {
	if (!m_nodePool) return false;
	
	Moss_RecastNode* nodes[DT_MAX_STATES_PER_NODE];
	int n= m_nodePool->findNodes(ref, nodes, DT_MAX_STATES_PER_NODE);

	for (int i=0; i<n; i++) {
		if (nodes[i]->flags & DT_NODE_CLOSED)
			return true;
	}		

	return false;
}
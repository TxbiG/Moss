// DetourTileCache.cpp
#include <string.h>
#include <new>

#include <Moss/Navigation/navigation_intern.h>
#ifndef MOSS_DEBUG_RENDERER
	#include <Moss/Moss_Renderer.h>
#endif // MOSS_DEBUG_RENDERER



Moss_RecastTileCache* dtAllocTileCache() {
	void* mem = dtAlloc(sizeof(Moss_RecastTileCache), DT_ALLOC_PERM);
	if (!mem) return 0;
	return new(mem) Moss_RecastTileCache;
}

void dtFreeTileCache(Moss_RecastTileCache* tc)
{
	if (!tc) return;
	tc->~Moss_RecastTileCache();
	MOSS_FREE(tc);
}

static bool contains(const Moss_RecastCompressedTileRef* a, const int n, const Moss_RecastCompressedTileRef v)
{
	for (int i = 0; i < n; ++i)
		if (a[i] == v)
			return true;
	return false;
}

inline int computeTileHash(int x, int y, const int mask)
{
	const unsigned int h1 = 0x8da6b343; // Large multiplicative constants;
	const unsigned int h2 = 0xd8163841; // here arbitrarily chosen primes
	unsigned int n = h1 * x + h2 * y;
	return (int)(n & mask);
}


struct NavMeshTileBuildContext
{
	inline NavMeshTileBuildContext(struct Moss_RecastTileCacheAlloc* a) : layer(0), lcset(0), lmesh(0), alloc(a) {}
	inline ~NavMeshTileBuildContext() { purge(); }
	void purge()
	{
		dtFreeTileCacheLayer(alloc, layer);
		layer = 0;
		dtFreeTileCacheContourSet(alloc, lcset);
		lcset = 0;
		dtFreeTileCachePolyMesh(alloc, lmesh);
		lmesh = 0;
	}
	struct dtTileCacheLayer* layer;
	struct dtTileCacheContourSet* lcset;
	struct dtTileCachePolyMesh* lmesh;
	struct Moss_RecastTileCacheAlloc* alloc;
};


Moss_RecastTileCache::Moss_RecastTileCache() :
	m_tileLutSize(0),
	m_tileLutMask(0),
	m_posLookup(0),
	m_nextFreeTile(0),	
	m_tiles(0),	
	m_saltBits(0),
	m_tileBits(0),
	m_talloc(0),
	m_tcomp(0),
	m_tmproc(0),
	m_obstacles(0),
	m_nextFreeObstacle(0),
	m_nreqs(0),
	m_nupdate(0)
{
	memset(&m_params, 0, sizeof(m_params));
	memset(m_reqs, 0, sizeof(ObstacleRequest) * MAX_REQUESTS);
}
	
Moss_RecastTileCache::~Moss_RecastTileCache()
{
	for (int i = 0; i < m_params.maxTiles; ++i)
	{
		if (m_tiles[i].flags & DT_COMPRESSEDTILE_FREE_DATA)
		{
			MOSS_FREE(m_tiles[i].data);
			m_tiles[i].data = 0;
		}
	}
	MOSS_FREE(m_obstacles);
	m_obstacles = 0;
	MOSS_FREE(m_posLookup);
	m_posLookup = 0;
	MOSS_FREE(m_tiles);
	m_tiles = 0;
	m_nreqs = 0;
	m_nupdate = 0;
}

const Moss_RecastCompressedTile* Moss_RecastTileCache::getTileByRef(Moss_RecastCompressedTileRef ref) const
{
	if (!ref)
		return 0;
	unsigned int tileIndex = decodeTileIdTile(ref);
	unsigned int tileSalt = decodeTileIdSalt(ref);
	if ((int)tileIndex >= m_params.maxTiles)
		return 0;
	const Moss_RecastCompressedTile* tile = &m_tiles[tileIndex];
	if (tile->salt != tileSalt)
		return 0;
	return tile;
}


Moss_RecastStatus Moss_RecastTileCache::init(const Moss_RecastTileCacheParams* params,
						   Moss_RecastTileCacheAlloc* talloc,
						   Moss_RecastTileCacheCompressor* tcomp,
						   Moss_RecastTileCacheMeshProcess* tmproc)
{
	m_talloc = talloc;
	m_tcomp = tcomp;
	m_tmproc = tmproc;
	m_nreqs = 0;
	memcpy(&m_params, params, sizeof(m_params));
	
	// Alloc space for obstacles.
	m_obstacles = (Moss_RecastTileCacheObstacle*)dtAlloc(sizeof(Moss_RecastTileCacheObstacle)*m_params.maxObstacles, DT_ALLOC_PERM);
	if (!m_obstacles)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(m_obstacles, 0, sizeof(Moss_RecastTileCacheObstacle)*m_params.maxObstacles);
	m_nextFreeObstacle = 0;
	for (int i = m_params.maxObstacles-1; i >= 0; --i)
	{
		m_obstacles[i].salt = 1;
		m_obstacles[i].next = m_nextFreeObstacle;
		m_nextFreeObstacle = &m_obstacles[i];
	}
	
	// Init tiles
	m_tileLutSize = dtNextPow2(m_params.maxTiles/4);
	if (!m_tileLutSize) m_tileLutSize = 1;
	m_tileLutMask = m_tileLutSize-1;
	
	m_tiles = (Moss_RecastCompressedTile*)dtAlloc(sizeof(Moss_RecastCompressedTile)*m_params.maxTiles, DT_ALLOC_PERM);
	if (!m_tiles)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	m_posLookup = (Moss_RecastCompressedTile**)dtAlloc(sizeof(Moss_RecastCompressedTile*)*m_tileLutSize, DT_ALLOC_PERM);
	if (!m_posLookup)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(m_tiles, 0, sizeof(Moss_RecastCompressedTile)*m_params.maxTiles);
	memset(m_posLookup, 0, sizeof(Moss_RecastCompressedTile*)*m_tileLutSize);
	m_nextFreeTile = 0;
	for (int i = m_params.maxTiles-1; i >= 0; --i)
	{
		m_tiles[i].salt = 1;
		m_tiles[i].next = m_nextFreeTile;
		m_nextFreeTile = &m_tiles[i];
	}
	
	// Init ID generator values.
	m_tileBits = dtIlog2(dtNextPow2((unsigned int)m_params.maxTiles));
	// Only allow 31 salt bits, since the salt mask is calculated using 32bit uint and it will overflow.
	m_saltBits = dtMin((unsigned int)31, 32 - m_tileBits);
	if (m_saltBits < 10)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	return DT_SUCCESS;
}

int Moss_RecastTileCache::getTilesAt(const int tx, const int ty, Moss_RecastCompressedTileRef* tiles, const int maxTiles) const 
{
	int n = 0;
	
	// Find tile based on hash.
	int h = computeTileHash(tx,ty,m_tileLutMask);
	Moss_RecastCompressedTile* tile = m_posLookup[h];
	while (tile)
	{
		if (tile->header &&
			tile->header->tx == tx &&
			tile->header->ty == ty)
		{
			if (n < maxTiles)
				tiles[n++] = getTileRef(tile);
		}
		tile = tile->next;
	}
	
	return n;
}

Moss_RecastCompressedTile* Moss_RecastTileCache::getTileAt(const int tx, const int ty, const int tlayer)
{
	// Find tile based on hash.
	int h = computeTileHash(tx,ty,m_tileLutMask);
	Moss_RecastCompressedTile* tile = m_posLookup[h];
	while (tile)
	{
		if (tile->header &&
			tile->header->tx == tx &&
			tile->header->ty == ty &&
			tile->header->tlayer == tlayer)
		{
			return tile;
		}
		tile = tile->next;
	}
	return 0;
}

Moss_RecastCompressedTileRef Moss_RecastTileCache::getTileRef(const Moss_RecastCompressedTile* tile) const
{
	if (!tile) return 0;
	const unsigned int it = (unsigned int)(tile - m_tiles);
	return (Moss_RecastCompressedTileRef)encodeTileId(tile->salt, it);
}

Moss_RecastObstacleRef Moss_RecastTileCache::getObstacleRef(const Moss_RecastTileCacheObstacle* ob) const
{
	if (!ob) return 0;
	const unsigned int idx = (unsigned int)(ob - m_obstacles);
	return encodeObstacleId(ob->salt, idx);
}

const Moss_RecastTileCacheObstacle* Moss_RecastTileCache::getObstacleByRef(Moss_RecastObstacleRef ref)
{
	if (!ref)
		return 0;
	unsigned int idx = decodeObstacleIdObstacle(ref);
	if ((int)idx >= m_params.maxObstacles)
		return 0;
	const Moss_RecastTileCacheObstacle* ob = &m_obstacles[idx];
	unsigned int salt = decodeObstacleIdSalt(ref);
	if (ob->salt != salt)
		return 0;
	return ob;
}

Moss_RecastTileCacheMeshProcess::~Moss_RecastTileCacheMeshProcess()
{
	// Defined out of line to fix the weak v-tables warning
}

Moss_RecastStatus Moss_RecastTileCache::addTile(unsigned char* data, const int dataSize, unsigned char flags, Moss_RecastCompressedTileRef* result)
{
	// Make sure the data is in right format.
	Moss_RecastTileCacheLayerHeader* header = (Moss_RecastTileCacheLayerHeader*)data;
	if (header->magic != DT_TILECACHE_MAGIC)
		return DT_FAILURE | DT_WRONG_MAGIC;
	if (header->version != DT_TILECACHE_VERSION)
		return DT_FAILURE | DT_WRONG_VERSION;
	
	// Make sure the location is free.
	if (getTileAt(header->tx, header->ty, header->tlayer))
		return DT_FAILURE;
	
	// Allocate a tile.
	Moss_RecastCompressedTile* tile = 0;
	if (m_nextFreeTile)
	{
		tile = m_nextFreeTile;
		m_nextFreeTile = tile->next;
		tile->next = 0;
	}
	
	// Make sure we could allocate a tile.
	if (!tile)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	
	// Insert tile into the position lut.
	int h = computeTileHash(header->tx, header->ty, m_tileLutMask);
	tile->next = m_posLookup[h];
	m_posLookup[h] = tile;
	
	// Init tile.
	const int headerSize = dtAlign4(sizeof(Moss_RecastTileCacheLayerHeader));
	tile->header = (Moss_RecastTileCacheLayerHeader*)data;
	tile->data = data;
	tile->dataSize = dataSize;
	tile->compressed = tile->data + headerSize;
	tile->compressedSize = tile->dataSize - headerSize;
	tile->flags = flags;
	
	if (result)
		*result = getTileRef(tile);
	
	return DT_SUCCESS;
}

Moss_RecastStatus Moss_RecastTileCache::removeTile(Moss_RecastCompressedTileRef ref, unsigned char** data, int* dataSize)
{
	if (!ref)
		return DT_FAILURE | DT_INVALID_PARAM;
	unsigned int tileIndex = decodeTileIdTile(ref);
	unsigned int tileSalt = decodeTileIdSalt(ref);
	if ((int)tileIndex >= m_params.maxTiles)
		return DT_FAILURE | DT_INVALID_PARAM;
	Moss_RecastCompressedTile* tile = &m_tiles[tileIndex];
	if (tile->salt != tileSalt)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	// Remove tile from hash lookup.
	const int h = computeTileHash(tile->header->tx,tile->header->ty,m_tileLutMask);
	Moss_RecastCompressedTile* prev = 0;
	Moss_RecastCompressedTile* cur = m_posLookup[h];
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
	
	// Reset tile.
	if (tile->flags & DT_COMPRESSEDTILE_FREE_DATA)
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
	tile->data = 0;
	tile->dataSize = 0;
	tile->compressed = 0;
	tile->compressedSize = 0;
	tile->flags = 0;
	
	// Update salt, salt should never be zero.
	tile->salt = (tile->salt+1) & ((1<<m_saltBits)-1);
	if (tile->salt == 0)
		tile->salt++;
	
	// Add to free list.
	tile->next = m_nextFreeTile;
	m_nextFreeTile = tile;
	
	return DT_SUCCESS;
}


Moss_RecastStatus Moss_RecastTileCache::addObstacle(const float* pos, const float radius, const float height, Moss_RecastObstacleRef* result)
{
	if (m_nreqs >= MAX_REQUESTS)
		return DT_FAILURE | DT_BUFFER_TOO_SMALL;
	
	Moss_RecastTileCacheObstacle* ob = 0;
	if (m_nextFreeObstacle)
	{
		ob = m_nextFreeObstacle;
		m_nextFreeObstacle = ob->next;
		ob->next = 0;
	}
	if (!ob)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	
	unsigned short salt = ob->salt;
	memset(ob, 0, sizeof(Moss_RecastTileCacheObstacle));
	ob->salt = salt;
	ob->state = DT_OBSTACLE_PROCESSING;
	ob->type = DT_OBSTACLE_CYLINDER;
	dtVcopy(ob->cylinder.pos, pos);
	ob->cylinder.radius = radius;
	ob->cylinder.height = height;
	
	ObstacleRequest* req = &m_reqs[m_nreqs++];
	memset(req, 0, sizeof(ObstacleRequest));
	req->action = REQUEST_ADD;
	req->ref = getObstacleRef(ob);
	
	if (result)
		*result = req->ref;
	
	return DT_SUCCESS;
}

Moss_RecastStatus Moss_RecastTileCache::addBoxObstacle(const float* bmin, const float* bmax, Moss_RecastObstacleRef* result)
{
	if (m_nreqs >= MAX_REQUESTS)
		return DT_FAILURE | DT_BUFFER_TOO_SMALL;
	
	Moss_RecastTileCacheObstacle* ob = 0;
	if (m_nextFreeObstacle)
	{
		ob = m_nextFreeObstacle;
		m_nextFreeObstacle = ob->next;
		ob->next = 0;
	}
	if (!ob)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	
	unsigned short salt = ob->salt;
	memset(ob, 0, sizeof(Moss_RecastTileCacheObstacle));
	ob->salt = salt;
	ob->state = DT_OBSTACLE_PROCESSING;
	ob->type = DT_OBSTACLE_BOX;
	dtVcopy(ob->box.mMin, bmin);
	dtVcopy(ob->box.mMax, bmax);
	
	ObstacleRequest* req = &m_reqs[m_nreqs++];
	memset(req, 0, sizeof(ObstacleRequest));
	req->action = REQUEST_ADD;
	req->ref = getObstacleRef(ob);
	
	if (result)
		*result = req->ref;
	
	return DT_SUCCESS;
}

Moss_RecastStatus Moss_RecastTileCache::addBoxObstacle(const float* center, const float* halfExtents, const float yRadians, Moss_RecastObstacleRef* result)
{
	if (m_nreqs >= MAX_REQUESTS)
		return DT_FAILURE | DT_BUFFER_TOO_SMALL;

	Moss_RecastTileCacheObstacle* ob = 0;
	if (m_nextFreeObstacle)
	{
		ob = m_nextFreeObstacle;
		m_nextFreeObstacle = ob->next;
		ob->next = 0;
	}
	if (!ob)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

	unsigned short salt = ob->salt;
	memset(ob, 0, sizeof(Moss_RecastTileCacheObstacle));
	ob->salt = salt;
	ob->state = DT_OBSTACLE_PROCESSING;
	ob->type = DT_OBSTACLE_ORIENTED_BOX;
	dtVcopy(ob->orientedBox.center, center);
	dtVcopy(ob->orientedBox.halfExtents, halfExtents);

	float coshalf= cosf(0.5f*yRadians);
	float sinhalf = sinf(-0.5f*yRadians);
	ob->orientedBox.rotAux[0] = coshalf*sinhalf;
	ob->orientedBox.rotAux[1] = coshalf*coshalf - 0.5f;

	ObstacleRequest* req = &m_reqs[m_nreqs++];
	memset(req, 0, sizeof(ObstacleRequest));
	req->action = REQUEST_ADD;
	req->ref = getObstacleRef(ob);

	if (result)
		*result = req->ref;

	return DT_SUCCESS;
}

Moss_RecastStatus Moss_RecastTileCache::removeObstacle(const Moss_RecastObstacleRef ref)
{
	if (!ref)
		return DT_SUCCESS;
	if (m_nreqs >= MAX_REQUESTS)
		return DT_FAILURE | DT_BUFFER_TOO_SMALL;
	
	ObstacleRequest* req = &m_reqs[m_nreqs++];
	memset(req, 0, sizeof(ObstacleRequest));
	req->action = REQUEST_REMOVE;
	req->ref = ref;
	
	return DT_SUCCESS;
}

Moss_RecastStatus Moss_RecastTileCache::queryTiles(const float* bmin, const float* bmax,
								 Moss_RecastCompressedTileRef* results, int* resultCount, const int maxResults) const 
{
	const int MAX_TILES = 32;
	Moss_RecastCompressedTileRef tiles[MAX_TILES];
	
	int n = 0;
	
	const float tw = m_params.width * m_params.cs;
	const float th = m_params.height * m_params.cs;
	const int tx0 = (int)floorf((bmin[0]-m_params.orig[0]) / tw);
	const int tx1 = (int)floorf((bmax[0]-m_params.orig[0]) / tw);
	const int ty0 = (int)floorf((bmin[2]-m_params.orig[2]) / th);
	const int ty1 = (int)floorf((bmax[2]-m_params.orig[2]) / th);
	
	for (int ty = ty0; ty <= ty1; ++ty)
	{
		for (int tx = tx0; tx <= tx1; ++tx)
		{
			const int ntiles = getTilesAt(tx,ty,tiles,MAX_TILES);
			
			for (int i = 0; i < ntiles; ++i)
			{
				const Moss_RecastCompressedTile* tile = &m_tiles[decodeTileIdTile(tiles[i])];
				float tbmin[3], tbmax[3];
				calcTightTileBounds(tile->header, tbmin, tbmax);
				
				if (dtOverlapBounds(bmin,bmax, tbmin,tbmax))
				{
					if (n < maxResults)
						results[n++] = tiles[i];
				}
			}
		}
	}
	
	*resultCount = n;
	
	return DT_SUCCESS;
}

Moss_RecastStatus Moss_RecastTileCache::update(const float /*dt*/, Moss_RecastNavMesh* navmesh,
							 bool* upToDate)
{
	if (m_nupdate == 0)
	{
		// Process requests.
		for (int i = 0; i < m_nreqs; ++i)
		{
			ObstacleRequest* req = &m_reqs[i];
			
			unsigned int idx = decodeObstacleIdObstacle(req->ref);
			if ((int)idx >= m_params.maxObstacles)
				continue;
			Moss_RecastTileCacheObstacle* ob = &m_obstacles[idx];
			unsigned int salt = decodeObstacleIdSalt(req->ref);
			if (ob->salt != salt)
				continue;
			
			if (req->action == REQUEST_ADD)
			{
				// Find touched tiles.
				float bmin[3], bmax[3];
				getObstacleBounds(ob, bmin, bmax);

				int ntouched = 0;
				queryTiles(bmin, bmax, ob->touched, &ntouched, DT_MAX_TOUCHED_TILES);
				ob->ntouched = (unsigned char)ntouched;
				// Add tiles to update list.
				ob->npending = 0;
				for (int j = 0; j < ob->ntouched; ++j)
				{
					if (m_nupdate < MAX_UPDATE)
					{
						if (!contains(m_update, m_nupdate, ob->touched[j]))
							m_update[m_nupdate++] = ob->touched[j];
						ob->pending[ob->npending++] = ob->touched[j];
					}
				}
			}
			else if (req->action == REQUEST_REMOVE)
			{
				// Prepare to remove obstacle.
				ob->state = DT_OBSTACLE_REMOVING;
				// Add tiles to update list.
				ob->npending = 0;
				for (int j = 0; j < ob->ntouched; ++j)
				{
					if (m_nupdate < MAX_UPDATE)
					{
						if (!contains(m_update, m_nupdate, ob->touched[j]))
							m_update[m_nupdate++] = ob->touched[j];
						ob->pending[ob->npending++] = ob->touched[j];
					}
				}
			}
		}
		
		m_nreqs = 0;
	}
	
	Moss_RecastStatus status = DT_SUCCESS;
	// Process updates
	if (m_nupdate)
	{
		// Build mesh
		const Moss_RecastCompressedTileRef ref = m_update[0];
		status = buildNavMeshTile(ref, navmesh);
		m_nupdate--;
		if (m_nupdate > 0)
			memmove(m_update, m_update+1, m_nupdate*sizeof(Moss_RecastCompressedTileRef));

		// Update obstacle states.
		for (int i = 0; i < m_params.maxObstacles; ++i)
		{
			Moss_RecastTileCacheObstacle* ob = &m_obstacles[i];
			if (ob->state == DT_OBSTACLE_PROCESSING || ob->state == DT_OBSTACLE_REMOVING)
			{
				// Remove handled tile from pending list.
				for (int j = 0; j < (int)ob->npending; j++)
				{
					if (ob->pending[j] == ref)
					{
						ob->pending[j] = ob->pending[(int)ob->npending-1];
						ob->npending--;
						break;
					}
				}
				
				// If all pending tiles processed, change state.
				if (ob->npending == 0)
				{
					if (ob->state == DT_OBSTACLE_PROCESSING)
					{
						ob->state = DT_OBSTACLE_PROCESSED;
					}
					else if (ob->state == DT_OBSTACLE_REMOVING)
					{
						ob->state = DT_OBSTACLE_EMPTY;
						// Update salt, salt should never be zero.
						ob->salt = (ob->salt+1) & ((1<<16)-1);
						if (ob->salt == 0)
							ob->salt++;
						// Return obstacle to free list.
						ob->next = m_nextFreeObstacle;
						m_nextFreeObstacle = ob;
					}
				}
			}
		}
	}
	
	if (upToDate)
		*upToDate = m_nupdate == 0 && m_nreqs == 0;

	return status;
}


Moss_RecastStatus Moss_RecastTileCache::buildNavMeshTilesAt(const int tx, const int ty, Moss_RecastNavMesh* navmesh)
{
	const int MAX_TILES = 32;
	Moss_RecastCompressedTileRef tiles[MAX_TILES];
	const int ntiles = getTilesAt(tx,ty,tiles,MAX_TILES);
	
	for (int i = 0; i < ntiles; ++i)
	{
		Moss_RecastStatus status = buildNavMeshTile(tiles[i], navmesh);
		if (dtStatusFailed(status))
			return status;
	}
	
	return DT_SUCCESS;
}

Moss_RecastStatus Moss_RecastTileCache::buildNavMeshTile(const Moss_RecastCompressedTileRef ref, Moss_RecastNavMesh* navmesh)
{	
	dtAssert(m_talloc);
	dtAssert(m_tcomp);
	
	unsigned int idx = decodeTileIdTile(ref);
	if (idx > (unsigned int)m_params.maxTiles)
		return DT_FAILURE | DT_INVALID_PARAM;
	const Moss_RecastCompressedTile* tile = &m_tiles[idx];
	unsigned int salt = decodeTileIdSalt(ref);
	if (tile->salt != salt)
		return DT_FAILURE | DT_INVALID_PARAM;
	
	m_talloc->reset();
	
	NavMeshTileBuildContext bc(m_talloc);
	const int walkableClimbVx = (int)(m_params.walkableClimb / m_params.ch);
	Moss_RecastStatus status;
	
	// Decompress tile layer data. 
	status = dtDecompressTileCacheLayer(m_talloc, m_tcomp, tile->data, tile->dataSize, &bc.layer);
	if (dtStatusFailed(status))
		return status;
	
	// Rasterize obstacles.
	for (int i = 0; i < m_params.maxObstacles; ++i)
	{
		const Moss_RecastTileCacheObstacle* ob = &m_obstacles[i];
		if (ob->state == DT_OBSTACLE_EMPTY || ob->state == DT_OBSTACLE_REMOVING)
			continue;
		if (contains(ob->touched, ob->ntouched, ref))
		{
			if (ob->type == DT_OBSTACLE_CYLINDER)
			{
				dtMarkCylinderArea(*bc.layer, tile->header->bmin, m_params.cs, m_params.ch,
							    ob->cylinder.pos, ob->cylinder.radius, ob->cylinder.height, 0);
			}
			else if (ob->type == DT_OBSTACLE_BOX)
			{
				dtMarkBoxArea(*bc.layer, tile->header->bmin, m_params.cs, m_params.ch, ob->box.mMin, ob->box.mMax, 0);
			}
			else if (ob->type == DT_OBSTACLE_ORIENTED_BOX)
			{
				dtMarkBoxArea(*bc.layer, tile->header->bmin, m_params.cs, m_params.ch,
					ob->orientedBox.center, ob->orientedBox.halfExtents, ob->orientedBox.rotAux, 0);
			}
		}
	}
	
	// Build navmesh
	status = dtBuildTileCacheRegions(m_talloc, *bc.layer, walkableClimbVx);
	if (dtStatusFailed(status))
		return status;
	
	bc.lcset = dtAllocTileCacheContourSet(m_talloc);
	if (!bc.lcset)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	status = dtBuildTileCacheContours(m_talloc, *bc.layer, walkableClimbVx,
									  m_params.maxSimplificationError, *bc.lcset);
	if (dtStatusFailed(status))
		return status;
	
	bc.lmesh = dtAllocTileCachePolyMesh(m_talloc);
	if (!bc.lmesh)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	status = dtBuildTileCachePolyMesh(m_talloc, *bc.lcset, *bc.lmesh);
	if (dtStatusFailed(status))
		return status;
	
	// Early out if the mesh tile is empty.
	if (!bc.lmesh->npolys)
	{
		// Remove existing tile.
		navmesh->removeTile(navmesh->getTileRefAt(tile->header->tx,tile->header->ty,tile->header->tlayer),0,0);
		return DT_SUCCESS;
	}
	
	Moss_RecastNavMeshCreateParams params;
	memset(&params, 0, sizeof(params));
	params.verts = bc.lmesh->verts;
	params.vertCount = bc.lmesh->nverts;
	params.polys = bc.lmesh->polys;
	params.polyAreas = bc.lmesh->areas;
	params.polyFlags = bc.lmesh->flags;
	params.polyCount = bc.lmesh->npolys;
	params.nvp = DT_VERTS_PER_POLYGON;
	params.walkableHeight = m_params.walkableHeight;
	params.walkableRadius = m_params.walkableRadius;
	params.walkableClimb = m_params.walkableClimb;
	params.tileX = tile->header->tx;
	params.tileY = tile->header->ty;
	params.tileLayer = tile->header->tlayer;
	params.cs = m_params.cs;
	params.ch = m_params.ch;
	params.buildBvTree = false;
	dtVcopy(params.bmin, tile->header->bmin);
	dtVcopy(params.bmax, tile->header->bmax);
	
	if (m_tmproc)
	{
		m_tmproc->process(&params, bc.lmesh->areas, bc.lmesh->flags);
	}
	
	unsigned char* navData = 0;
	int navDataSize = 0;
	if (!dtCreateNavMeshData(&params, &navData, &navDataSize))
		return DT_FAILURE;

	// Remove existing tile.
	navmesh->removeTile(navmesh->getTileRefAt(tile->header->tx,tile->header->ty,tile->header->tlayer),0,0);

	// Add new tile, or leave the location empty.
	if (navData)
	{
		// Let the navmesh own the data.
		status = navmesh->addTile(navData,navDataSize,DT_TILE_FREE_DATA,0,0);
		if (dtStatusFailed(status))
		{
			MOSS_FREE(navData);
			return status;
		}
	}
	
	return DT_SUCCESS;
}

void Moss_RecastTileCache::calcTightTileBounds(const Moss_RecastTileCacheLayerHeader* header, float* bmin, float* bmax) const
{
	const float cs = m_params.cs;
	bmin[0] = header->bmin[0] + header->minx*cs;
	bmin[1] = header->bmin[1];
	bmin[2] = header->bmin[2] + header->miny*cs;
	bmax[0] = header->bmin[0] + (header->maxx+1)*cs;
	bmax[1] = header->bmax[1];
	bmax[2] = header->bmin[2] + (header->maxy+1)*cs;
}

void Moss_RecastTileCache::getObstacleBounds(const struct Moss_RecastTileCacheObstacle* ob, float* bmin, float* bmax) const
{
	if (ob->type == DT_OBSTACLE_CYLINDER)
	{
		const dtObstacleCylinder &cl = ob->cylinder;

		bmin[0] = cl.pos[0] - cl.radius;
		bmin[1] = cl.pos[1];
		bmin[2] = cl.pos[2] - cl.radius;
		bmax[0] = cl.pos[0] + cl.radius;
		bmax[1] = cl.pos[1] + cl.height;
		bmax[2] = cl.pos[2] + cl.radius;
	}
	else if (ob->type == DT_OBSTACLE_BOX)
	{
		dtVcopy(bmin, ob->box.mMin);
		dtVcopy(bmax, ob->box.mMax);
	}
	else if (ob->type == DT_OBSTACLE_ORIENTED_BOX)
	{
		const dtObstacleOrientedBox &orientedBox = ob->orientedBox;

		float maxr = 1.41f*dtMax(orientedBox.halfExtents[0], orientedBox.halfExtents[2]);
		bmin[0] = orientedBox.center[0] - maxr;
		bmax[0] = orientedBox.center[0] + maxr;
		bmin[1] = orientedBox.center[1] - orientedBox.halfExtents[1];
		bmax[1] = orientedBox.center[1] + orientedBox.halfExtents[1];
		bmin[2] = orientedBox.center[2] - maxr;
		bmax[2] = orientedBox.center[2] + maxr;
	}
}



/*													*/


Moss_RecastTileCacheAlloc::~Moss_RecastTileCacheAlloc()
{
	// Defined out of line to fix the weak v-tables warning
}

Moss_RecastTileCacheCompressor::~Moss_RecastTileCacheCompressor()
{
	// Defined out of line to fix the weak v-tables warning
}

template<class T> class dtFixedArray
{
	Moss_RecastTileCacheAlloc* m_alloc;
	T* m_ptr;
	const int m_size;
	inline void operator=(dtFixedArray<T>& p);
public:
	inline dtFixedArray(Moss_RecastTileCacheAlloc* a, const int s) : m_alloc(a), m_ptr((T*)a->alloc(sizeof(T)*s)), m_size(s) {}
	inline ~dtFixedArray() { if (m_alloc) m_alloc->free(m_ptr); }
	inline operator T*() { return m_ptr; }
	inline int size() const { return m_size; }
};

inline int getDirOffsetX(int dir)
{
	const int offset[4] = { -1, 0, 1, 0, };
	return offset[dir&0x03];
}

inline int getDirOffsetY(int dir)
{
	const int offset[4] = { 0, 1, 0, -1 };
	return offset[dir&0x03];
}

static const int MAX_VERTS_PER_POLY = 6;	// TODO: use the DT_VERTS_PER_POLYGON
static const int MAX_REM_EDGES = 48;		// TODO: make this an expression.



dtTileCacheContourSet* dtAllocTileCacheContourSet(Moss_RecastTileCacheAlloc* alloc)
{
	dtAssert(alloc);

	dtTileCacheContourSet* cset = (dtTileCacheContourSet*)alloc->alloc(sizeof(dtTileCacheContourSet));
	memset(cset, 0, sizeof(dtTileCacheContourSet));
	return cset;
}

void dtFreeTileCacheContourSet(Moss_RecastTileCacheAlloc* alloc, dtTileCacheContourSet* cset)
{
	dtAssert(alloc);

	if (!cset) return;
	for (int i = 0; i < cset->nconts; ++i)
		alloc->free(cset->conts[i].verts);
	alloc->free(cset->conts);
	alloc->free(cset);
}

dtTileCachePolyMesh* dtAllocTileCachePolyMesh(Moss_RecastTileCacheAlloc* alloc)
{
	dtAssert(alloc);

	dtTileCachePolyMesh* lmesh = (dtTileCachePolyMesh*)alloc->alloc(sizeof(dtTileCachePolyMesh));
	memset(lmesh, 0, sizeof(dtTileCachePolyMesh));
	return lmesh;
}

void dtFreeTileCachePolyMesh(Moss_RecastTileCacheAlloc* alloc, dtTileCachePolyMesh* lmesh)
{
	dtAssert(alloc);
	
	if (!lmesh) return;
	alloc->free(lmesh->verts);
	alloc->free(lmesh->polys);
	alloc->free(lmesh->flags);
	alloc->free(lmesh->areas);
	alloc->free(lmesh);
}



struct dtLayerSweepSpan
{
	unsigned short ns;	// number samples
	unsigned char id;	// region id
	unsigned char nei;	// neighbour id
};

static const int DT_LAYER_MAX_NEIS = 16;

struct dtLayerMonotoneRegion
{
	int area;
	unsigned char neis[DT_LAYER_MAX_NEIS];
	unsigned char nneis;
	unsigned char regId;
	unsigned char areaId;
};

struct dtTempContour
{
	inline dtTempContour(unsigned char* vbuf, const int nvbuf,
						 unsigned short* pbuf, const int npbuf) :
		verts(vbuf), nverts(0), cverts(nvbuf),
		poly(pbuf), npoly(0), cpoly(npbuf) 
	{
	}
	unsigned char* verts;
	int nverts;
	int cverts;
	unsigned short* poly;
	int npoly;
	int cpoly;
};




inline bool overlapRangeExl(const unsigned short amin, const unsigned short amax, const unsigned short bmin, const unsigned short bmax) {
	return (amin >= bmax || amax <= bmin) ? false : true;
}

static void addUniqueLast(unsigned char* a, unsigned char& an, unsigned char v)
{
	const int n = (int)an;
	if (n > 0 && a[n-1] == v) return;
	a[an] = v;
	an++;
}

inline bool isConnected(const dtTileCacheLayer& layer,
						const int ia, const int ib, const int walkableClimb)
{
	if (layer.areas[ia] != layer.areas[ib]) return false;
	if (abs((int)layer.heights[ia] - (int)layer.heights[ib]) > walkableClimb) return false;
	return true;
}

static bool canMerge(unsigned char oldRegId, unsigned char newRegId, const dtLayerMonotoneRegion* regs, const int nregs)
{
	int count = 0;
	for (int i = 0; i < nregs; ++i)
	{
		const dtLayerMonotoneRegion& reg = regs[i];
		if (reg.regId != oldRegId) continue;
		const int nnei = (int)reg.nneis;
		for (int j = 0; j < nnei; ++j)
		{
			if (regs[reg.neis[j]].regId == newRegId)
				count++;
		}
	}
	return count == 1;
}


Moss_RecastStatus dtBuildTileCacheRegions(Moss_RecastTileCacheAlloc* alloc,
								 dtTileCacheLayer& layer,
								 const int walkableClimb)
{
	dtAssert(alloc);
	
	const int w = (int)layer.header->width;
	const int h = (int)layer.header->height;
	
	memset(layer.regs,0xff,sizeof(unsigned char)*w*h);
	
	const int nsweeps = w;
	dtFixedArray<dtLayerSweepSpan> sweeps(alloc, nsweeps);
	if (!sweeps)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(sweeps,0,sizeof(dtLayerSweepSpan)*nsweeps);
	
	// Partition walkable area into monotone regions.
	unsigned char prevCount[256];
	unsigned char regId = 0;
	
	for (int y = 0; y < h; ++y)
	{
		if (regId > 0)
			memset(prevCount,0,sizeof(unsigned char)*regId);
		unsigned char sweepId = 0;
		
		for (int x = 0; x < w; ++x)
		{
			const int idx = x + y*w;
			if (layer.areas[idx] == DT_TILECACHE_NULL_AREA) continue;
			
			unsigned char sid = 0xff;
			
			// -x
			const int xidx = (x-1)+y*w;
			if (x > 0 && isConnected(layer, idx, xidx, walkableClimb))
			{
				if (layer.regs[xidx] != 0xff)
					sid = layer.regs[xidx];
			}
			
			if (sid == 0xff)
			{
				sid = sweepId++;
				sweeps[sid].nei = 0xff;
				sweeps[sid].ns = 0;
			}
			
			// -y
			const int yidx = x+(y-1)*w;
			if (y > 0 && isConnected(layer, idx, yidx, walkableClimb))
			{
				const unsigned char nr = layer.regs[yidx];
				if (nr != 0xff)
				{
					// Set neighbour when first valid neighbour is encoutered.
					if (sweeps[sid].ns == 0)
						sweeps[sid].nei = nr;
					
					if (sweeps[sid].nei == nr)
					{
						// Update existing neighbour
						sweeps[sid].ns++;
						prevCount[nr]++;
					}
					else
					{
						// This is hit if there is nore than one neighbour.
						// Invalidate the neighbour.
						sweeps[sid].nei = 0xff;
					}
				}
			}
			
			layer.regs[idx] = sid;
		}
		
		// Create unique ID.
		for (int i = 0; i < sweepId; ++i)
		{
			// If the neighbour is set and there is only one continuous connection to it,
			// the sweep will be merged with the previous one, else new region is created.
			if (sweeps[i].nei != 0xff && (unsigned short)prevCount[sweeps[i].nei] == sweeps[i].ns)
			{
				sweeps[i].id = sweeps[i].nei;
			}
			else
			{
				if (regId == 255)
				{
					// Region ID's overflow.
					return DT_FAILURE | DT_BUFFER_TOO_SMALL;
				}
				sweeps[i].id = regId++;
			}
		}
		
		// Remap local sweep ids to region ids.
		for (int x = 0; x < w; ++x)
		{
			const int idx = x+y*w;
			if (layer.regs[idx] != 0xff)
				layer.regs[idx] = sweeps[layer.regs[idx]].id;
		}
	}
	
	// Allocate and init layer regions.
	const int nregs = (int)regId;
	dtFixedArray<dtLayerMonotoneRegion> regs(alloc, nregs);
	if (!regs)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

	memset(regs, 0, sizeof(dtLayerMonotoneRegion)*nregs);
	for (int i = 0; i < nregs; ++i)
		regs[i].regId = 0xff;
	
	// Find region neighbours.
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const int idx = x+y*w;
			const unsigned char ri = layer.regs[idx];
			if (ri == 0xff)
				continue;
			
			// Update area.
			regs[ri].area++;
			regs[ri].areaId = layer.areas[idx];
			
			// Update neighbours
			const int ymi = x+(y-1)*w;
			if (y > 0 && isConnected(layer, idx, ymi, walkableClimb))
			{
				const unsigned char rai = layer.regs[ymi];
				if (rai != 0xff && rai != ri)
				{
					addUniqueLast(regs[ri].neis, regs[ri].nneis, rai);
					addUniqueLast(regs[rai].neis, regs[rai].nneis, ri);
				}
			}
		}
	}
	
	for (int i = 0; i < nregs; ++i)
		regs[i].regId = (unsigned char)i;
	
	for (int i = 0; i < nregs; ++i)
	{
		dtLayerMonotoneRegion& reg = regs[i];
		
		int merge = -1;
		int mergea = 0;
		for (int j = 0; j < (int)reg.nneis; ++j)
		{
			const unsigned char nei = reg.neis[j];
			dtLayerMonotoneRegion& regn = regs[nei];
			if (reg.regId == regn.regId)
				continue;
			if (reg.areaId != regn.areaId)
				continue;
			if (regn.area > mergea)
			{
				if (canMerge(reg.regId, regn.regId, regs, nregs))
				{
					mergea = regn.area;
					merge = (int)nei;
				}
			}
		}
		if (merge != -1)
		{
			const unsigned char oldId = reg.regId;
			const unsigned char newId = regs[merge].regId;
			for (int j = 0; j < nregs; ++j)
				if (regs[j].regId == oldId)
					regs[j].regId = newId;
		}
	}
	
	// Compact ids.
	unsigned char remap[256];
	memset(remap, 0, 256);
	// Find number of unique regions.
	regId = 0;
	for (int i = 0; i < nregs; ++i)
		remap[regs[i].regId] = 1;
	for (int i = 0; i < 256; ++i)
		if (remap[i])
			remap[i] = regId++;
	// Remap ids.
	for (int i = 0; i < nregs; ++i)
		regs[i].regId = remap[regs[i].regId];
	
	layer.regCount = regId;
	
	for (int i = 0; i < w*h; ++i)
	{
		if (layer.regs[i] != 0xff)
			layer.regs[i] = regs[layer.regs[i]].regId;
	}
	
	return DT_SUCCESS;
}



static bool appendVertex(dtTempContour& cont, const int x, const int y, const int z, const int r)
{
	// Try to merge with existing segments.
	if (cont.nverts > 1)
	{
		unsigned char* pa = &cont.verts[(cont.nverts-2)*4];
		unsigned char* pb = &cont.verts[(cont.nverts-1)*4];
		if ((int)pb[3] == r)
		{
			if (pa[0] == pb[0] && (int)pb[0] == x)
			{
				// The verts are aligned aling x-axis, update z.
				pb[1] = (unsigned char)y;
				pb[2] = (unsigned char)z;
				return true;
			}
			else if (pa[2] == pb[2] && (int)pb[2] == z)
			{
				// The verts are aligned aling z-axis, update x.
				pb[0] = (unsigned char)x;
				pb[1] = (unsigned char)y;
				return true;
			}
		}
	}
	
	// Add new point.
	if (cont.nverts+1 > cont.cverts)
		return false;
	
	unsigned char* v = &cont.verts[cont.nverts*4];
	v[0] = (unsigned char)x;
	v[1] = (unsigned char)y;
	v[2] = (unsigned char)z;
	v[3] = (unsigned char)r;
	cont.nverts++;
	
	return true;
}


static unsigned char getNeighbourReg(dtTileCacheLayer& layer,
									 const int ax, const int ay, const int dir)
{
	const int w = (int)layer.header->width;
	const int ia = ax + ay*w;
	
	const unsigned char con = layer.cons[ia] & 0xf;
	const unsigned char portal = layer.cons[ia] >> 4;
	const unsigned char mask = (unsigned char)(1<<dir);
	
	if ((con & mask) == 0)
	{
		// No connection, return portal or hard edge.
		if (portal & mask)
			return 0xf8 + (unsigned char)dir;
		return 0xff;
	}
	
	const int bx = ax + getDirOffsetX(dir);
	const int by = ay + getDirOffsetY(dir);
	const int ib = bx + by*w;
	
	return layer.regs[ib];
}

static bool walkContour(dtTileCacheLayer& layer, int x, int y, dtTempContour& cont)
{
	const int w = (int)layer.header->width;
	const int h = (int)layer.header->height;
	
	cont.nverts = 0;
	
	int startX = x;
	int startY = y;
	int startDir = -1;
	
	for (int i = 0; i < 4; ++i)
	{
		const int dir = (i+3)&3;
		unsigned char rn = getNeighbourReg(layer, x, y, dir);
		if (rn != layer.regs[x+y*w])
		{
			startDir = dir;
			break;
		}
	}
	if (startDir == -1)
		return true;
	
	int dir = startDir;
	const int maxIter = w*h;
	
	int iter = 0;
	while (iter < maxIter)
	{
		unsigned char rn = getNeighbourReg(layer, x, y, dir);
		
		int nx = x;
		int ny = y;
		int ndir = dir;
		
		if (rn != layer.regs[x+y*w])
		{
			// Solid edge.
			int px = x;
			int pz = y;
			switch(dir)
			{
				case 0: pz++; break;
				case 1: px++; pz++; break;
				case 2: px++; break;
			}
			
			// Try to merge with previous vertex.
			if (!appendVertex(cont, px, (int)layer.heights[x+y*w], pz,rn))
				return false;
			
			ndir = (dir+1) & 0x3;  // Rotate CW
		}
		else
		{
			// Move to next.
			nx = x + getDirOffsetX(dir);
			ny = y + getDirOffsetY(dir);
			ndir = (dir+3) & 0x3;	// Rotate CCW
		}
		
		if (iter > 0 && x == startX && y == startY && dir == startDir)
			break;
		
		x = nx;
		y = ny;
		dir = ndir;
		
		iter++;
	}
	
	// Remove last vertex if it is duplicate of the first one.
	unsigned char* pa = &cont.verts[(cont.nverts-1)*4];
	unsigned char* pb = &cont.verts[0];
	if (pa[0] == pb[0] && pa[2] == pb[2])
		cont.nverts--;
	
	return true;
}	


static float distancePtSeg(const int x, const int z,
						   const int px, const int pz,
						   const int qx, const int qz)
{
	float pqx = (float)(qx - px);
	float pqz = (float)(qz - pz);
	float dx = (float)(x - px);
	float dz = (float)(z - pz);
	float d = pqx*pqx + pqz*pqz;
	float t = pqx*dx + pqz*dz;
	if (d > 0)
		t /= d;
	if (t < 0)
		t = 0;
	else if (t > 1)
		t = 1;
	
	dx = px + t*pqx - x;
	dz = pz + t*pqz - z;
	
	return dx*dx + dz*dz;
}

static void simplifyContour(dtTempContour& cont, const float maxError)
{
	cont.npoly = 0;
	
	for (int i = 0; i < cont.nverts; ++i)
	{
		int j = (i+1) % cont.nverts;
		// Check for start of a wall segment.
		unsigned char ra = cont.verts[j*4+3];
		unsigned char rb = cont.verts[i*4+3];
		if (ra != rb)
			cont.poly[cont.npoly++] = (unsigned short)i;
	}
	if (cont.npoly < 2)
	{
		// If there is no transitions at all,
		// create some initial points for the simplification process. 
		// Find lower-left and upper-right vertices of the contour.
		int llx = cont.verts[0];
		int llz = cont.verts[2];
		int lli = 0;
		int urx = cont.verts[0];
		int urz = cont.verts[2];
		int uri = 0;
		for (int i = 1; i < cont.nverts; ++i)
		{
			int x = cont.verts[i*4+0];
			int z = cont.verts[i*4+2];
			if (x < llx || (x == llx && z < llz))
			{
				llx = x;
				llz = z;
				lli = i;
			}
			if (x > urx || (x == urx && z > urz))
			{
				urx = x;
				urz = z;
				uri = i;
			}
		}
		cont.npoly = 0;
		cont.poly[cont.npoly++] = (unsigned short)lli;
		cont.poly[cont.npoly++] = (unsigned short)uri;
	}
	
	// Add points until all raw points are within
	// error tolerance to the simplified shape.
	for (int i = 0; i < cont.npoly; )
	{
		int ii = (i+1) % cont.npoly;
		
		const int ai = (int)cont.poly[i];
		const int ax = (int)cont.verts[ai*4+0];
		const int az = (int)cont.verts[ai*4+2];
		
		const int bi = (int)cont.poly[ii];
		const int bx = (int)cont.verts[bi*4+0];
		const int bz = (int)cont.verts[bi*4+2];
		
		// Find maximum deviation from the segment.
		float maxd = 0;
		int maxi = -1;
		int ci, cinc, endi;
		
		// Traverse the segment in lexilogical order so that the
		// max deviation is calculated similarly when traversing
		// opposite segments.
		if (bx > ax || (bx == ax && bz > az))
		{
			cinc = 1;
			ci = (ai+cinc) % cont.nverts;
			endi = bi;
		}
		else
		{
			cinc = cont.nverts-1;
			ci = (bi+cinc) % cont.nverts;
			endi = ai;
		}
		
		// Tessellate only outer edges or edges between areas.
		while (ci != endi)
		{
			float d = distancePtSeg(cont.verts[ci*4+0], cont.verts[ci*4+2], ax, az, bx, bz);
			if (d > maxd)
			{
				maxd = d;
				maxi = ci;
			}
			ci = (ci+cinc) % cont.nverts;
		}
		
		
		// If the max deviation is larger than accepted error,
		// add new point, else continue to next segment.
		if (maxi != -1 && maxd > (maxError*maxError))
		{
			cont.npoly++;
			for (int j = cont.npoly-1; j > i; --j)
				cont.poly[j] = cont.poly[j-1];
			cont.poly[i+1] = (unsigned short)maxi;
		}
		else
		{
			++i;
		}
	}
	
	// Remap vertices
	int start = 0;
	for (int i = 1; i < cont.npoly; ++i)
		if (cont.poly[i] < cont.poly[start])
			start = i;
	
	cont.nverts = 0;
	for (int i = 0; i < cont.npoly; ++i)
	{
		const int j = (start+i) % cont.npoly;
		unsigned char* src = &cont.verts[cont.poly[j]*4];
		unsigned char* dst = &cont.verts[cont.nverts*4];
		dst[0] = src[0];
		dst[1] = src[1];
		dst[2] = src[2];
		dst[3] = src[3];
		cont.nverts++;
	}
}

static unsigned char getCornerHeight(dtTileCacheLayer& layer,
									 const int x, const int y, const int z,
									 const int walkableClimb,
									 bool& shouldRemove)
{
	const int w = (int)layer.header->width;
	const int h = (int)layer.header->height;
	
	int n = 0;
	
	unsigned char portal = 0xf;
	unsigned char height = 0;
	unsigned char preg = 0xff;
	bool allSameReg = true;
	
	for (int dz = -1; dz <= 0; ++dz)
	{
		for (int dx = -1; dx <= 0; ++dx)
		{
			const int px = x+dx;
			const int pz = z+dz;
			if (px >= 0 && pz >= 0 && px < w && pz < h)
			{
				const int idx  = px + pz*w;
				const int lh = (int)layer.heights[idx];
				if (abs(lh-y) <= walkableClimb && layer.areas[idx] != DT_TILECACHE_NULL_AREA)
				{
					height = dtMax(height, (unsigned char)lh);
					portal &= (layer.cons[idx] >> 4);
					if (preg != 0xff && preg != layer.regs[idx])
						allSameReg = false;
					preg = layer.regs[idx]; 
					n++;
				}
			}
		}
	}
	
	int portalCount = 0;
	for (int dir = 0; dir < 4; ++dir)
		if (portal & (1<<dir))
			portalCount++;
	
	shouldRemove = false;
	if (n > 1 && portalCount == 1 && allSameReg)
	{
		shouldRemove = true;
	}
	
	return height;
}


// TODO: move this somewhere else, once the layer meshing is done.
Moss_RecastStatus dtBuildTileCacheContours(Moss_RecastTileCacheAlloc* alloc,
								  dtTileCacheLayer& layer,
								  const int walkableClimb, 	const float maxError,
								  dtTileCacheContourSet& lcset)
{
	dtAssert(alloc);

	const int w = (int)layer.header->width;
	const int h = (int)layer.header->height;
	
	lcset.nconts = layer.regCount;
	lcset.conts = (dtTileCacheContour*)alloc->alloc(sizeof(dtTileCacheContour)*lcset.nconts);
	if (!lcset.conts)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(lcset.conts, 0, sizeof(dtTileCacheContour)*lcset.nconts);
	
	// Allocate temp buffer for contour tracing.
	const int maxTempVerts = (w+h)*2 * 2; // Twice around the layer.
	
	dtFixedArray<unsigned char> tempVerts(alloc, maxTempVerts*4);
	if (!tempVerts)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	
	dtFixedArray<unsigned short> tempPoly(alloc, maxTempVerts);
	if (!tempPoly)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

	dtTempContour temp(tempVerts, maxTempVerts, tempPoly, maxTempVerts);
	
	// Find contours.
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const int idx = x+y*w;
			const unsigned char ri = layer.regs[idx];
			if (ri == 0xff)
				continue;
			
			dtTileCacheContour& cont = lcset.conts[ri];
			
			if (cont.nverts > 0)
				continue;
			
			cont.reg = ri;
			cont.area = layer.areas[idx];
			
			if (!walkContour(layer, x, y, temp))
			{
				// Too complex contour.
				// Note: If you hit here ofte, try increasing 'maxTempVerts'.
				return DT_FAILURE | DT_BUFFER_TOO_SMALL;
			}
			
			simplifyContour(temp, maxError);
			
			// Store contour.
			cont.nverts = temp.nverts;
			if (cont.nverts > 0)
			{
				cont.verts = (unsigned char*)alloc->alloc(sizeof(unsigned char)*4*temp.nverts);
				if (!cont.verts)
					return DT_FAILURE | DT_OUT_OF_MEMORY;
				
				for (int i = 0, j = temp.nverts-1; i < temp.nverts; j=i++)
				{
					unsigned char* dst = &cont.verts[j*4];
					unsigned char* v = &temp.verts[j*4];
					unsigned char* vn = &temp.verts[i*4];
					unsigned char nei = vn[3]; // The neighbour reg is stored at segment vertex of a segment. 
					bool shouldRemove = false;
					unsigned char lh = getCornerHeight(layer, (int)v[0], (int)v[1], (int)v[2],
													   walkableClimb, shouldRemove);
					
					dst[0] = v[0];
					dst[1] = lh;
					dst[2] = v[2];
					
					// Store portal direction and remove status to the fourth component.
					dst[3] = 0x0f;
					if (nei != 0xff && nei >= 0xf8)
						dst[3] = nei - 0xf8;
					if (shouldRemove)
						dst[3] |= 0x80;
				}
			}
		}
	}
	
	return DT_SUCCESS;
}	



static const int VERTEX_BUCKET_COUNT2 = (1<<8);

inline int computeVertexHash2(int x, int y, int z)
{
	const unsigned int h1 = 0x8da6b343; // Large multiplicative constants;
	const unsigned int h2 = 0xd8163841; // here arbitrarily chosen primes
	const unsigned int h3 = 0xcb1ab31f;
	unsigned int n = h1 * x + h2 * y + h3 * z;
	return (int)(n & (VERTEX_BUCKET_COUNT2-1));
}

static unsigned short addVertex(unsigned short x, unsigned short y, unsigned short z,
								unsigned short* verts, unsigned short* firstVert, unsigned short* nextVert, int& nv)
{
	int bucket = computeVertexHash2(x, 0, z);
	unsigned short i = firstVert[bucket];
	
	while (i != DT_TILECACHE_NULL_IDX)
	{
		const unsigned short* v = &verts[i*3];
		if (v[0] == x && v[2] == z && (abs(v[1] - y) <= 2))
			return i;
		i = nextVert[i]; // next
	}
	
	// Could not find, create new.
	i = (unsigned short)nv; nv++;
	unsigned short* v = &verts[i*3];
	v[0] = x;
	v[1] = y;
	v[2] = z;
	nextVert[i] = firstVert[bucket];
	firstVert[bucket] = i;
	
	return (unsigned short)i;
}


struct rcEdge
{
	unsigned short vert[2];
	unsigned short polyEdge[2];
	unsigned short poly[2];
};

static bool buildMeshAdjacency(Moss_RecastTileCacheAlloc* alloc,
							   unsigned short* polys, const int npolys,
							   const unsigned short* verts, const int nverts,
							   const dtTileCacheContourSet& lcset)
{
	// Based on code by Eric Lengyel from:
	// https://web.archive.org/web/20080704083314/http://www.terathon.com/code/edges.php
	
	const int maxEdgeCount = npolys*MAX_VERTS_PER_POLY;
	dtFixedArray<unsigned short> firstEdge(alloc, nverts + maxEdgeCount);
	if (!firstEdge)
		return false;
	unsigned short* nextEdge = firstEdge + nverts;
	int edgeCount = 0;
	
	dtFixedArray<rcEdge> edges(alloc, maxEdgeCount);
	if (!edges)
		return false;
	
	for (int i = 0; i < nverts; i++)
		firstEdge[i] = DT_TILECACHE_NULL_IDX;
	
	for (int i = 0; i < npolys; ++i)
	{
		unsigned short* t = &polys[i*MAX_VERTS_PER_POLY*2];
		for (int j = 0; j < MAX_VERTS_PER_POLY; ++j)
		{
			if (t[j] == DT_TILECACHE_NULL_IDX) break;
			unsigned short v0 = t[j];
			unsigned short v1 = (j+1 >= MAX_VERTS_PER_POLY || t[j+1] == DT_TILECACHE_NULL_IDX) ? t[0] : t[j+1];
			if (v0 < v1)
			{
				rcEdge& edge = edges[edgeCount];
				edge.vert[0] = v0;
				edge.vert[1] = v1;
				edge.poly[0] = (unsigned short)i;
				edge.polyEdge[0] = (unsigned short)j;
				edge.poly[1] = (unsigned short)i;
				edge.polyEdge[1] = 0xff;
				// Insert edge
				nextEdge[edgeCount] = firstEdge[v0];
				firstEdge[v0] = (unsigned short)edgeCount;
				edgeCount++;
			}
		}
	}
	
	for (int i = 0; i < npolys; ++i)
	{
		unsigned short* t = &polys[i*MAX_VERTS_PER_POLY*2];
		for (int j = 0; j < MAX_VERTS_PER_POLY; ++j)
		{
			if (t[j] == DT_TILECACHE_NULL_IDX) break;
			unsigned short v0 = t[j];
			unsigned short v1 = (j+1 >= MAX_VERTS_PER_POLY || t[j+1] == DT_TILECACHE_NULL_IDX) ? t[0] : t[j+1];
			if (v0 > v1)
			{
				bool found = false;
				for (unsigned short e = firstEdge[v1]; e != DT_TILECACHE_NULL_IDX; e = nextEdge[e])
				{
					rcEdge& edge = edges[e];
					if (edge.vert[1] == v0 && edge.poly[0] == edge.poly[1])
					{
						edge.poly[1] = (unsigned short)i;
						edge.polyEdge[1] = (unsigned short)j;
						found = true;
						break;
					}
				}
				if (!found)
				{
					// Matching edge not found, it is an open edge, add it.
					rcEdge& edge = edges[edgeCount];
					edge.vert[0] = v1;
					edge.vert[1] = v0;
					edge.poly[0] = (unsigned short)i;
					edge.polyEdge[0] = (unsigned short)j;
					edge.poly[1] = (unsigned short)i;
					edge.polyEdge[1] = 0xff;
					// Insert edge
					nextEdge[edgeCount] = firstEdge[v1];
					firstEdge[v1] = (unsigned short)edgeCount;
					edgeCount++;
				}
			}
		}
	}
	
	// Mark portal edges.
	for (int i = 0; i < lcset.nconts; ++i)
	{
		dtTileCacheContour& cont = lcset.conts[i];
		if (cont.nverts < 3)
			continue;
		
		for (int j = 0, k = cont.nverts-1; j < cont.nverts; k=j++)
		{
			const unsigned char* va = &cont.verts[k*4];
			const unsigned char* vb = &cont.verts[j*4];
			const unsigned char dir = va[3] & 0xf;
			if (dir == 0xf)
				continue;
			
			if (dir == 0 || dir == 2)
			{
				// Find matching vertical edge
				const unsigned short x = (unsigned short)va[0];
				unsigned short zmin = (unsigned short)va[2];
				unsigned short zmax = (unsigned short)vb[2];
				if (zmin > zmax)
					Swap(zmin, zmax);
				
				for (int m = 0; m < edgeCount; ++m)
				{
					rcEdge& e = edges[m];
					// Skip connected edges.
					if (e.poly[0] != e.poly[1])
						continue;
					const unsigned short* eva = &verts[e.vert[0]*3];
					const unsigned short* evb = &verts[e.vert[1]*3];
					if (eva[0] == x && evb[0] == x)
					{
						unsigned short ezmin = eva[2];
						unsigned short ezmax = evb[2];
						if (ezmin > ezmax)
							Swap(ezmin, ezmax);
						if (overlapRangeExl(zmin,zmax, ezmin, ezmax))
						{
							// Reuse the other polyedge to store dir.
							e.polyEdge[1] = dir;
						}
					}
				}
			}
			else
			{
				// Find matching vertical edge
				const unsigned short z = (unsigned short)va[2];
				unsigned short xmin = (unsigned short)va[0];
				unsigned short xmax = (unsigned short)vb[0];
				if (xmin > xmax)
					Swap(xmin, xmax);
				for (int m = 0; m < edgeCount; ++m)
				{
					rcEdge& e = edges[m];
					// Skip connected edges.
					if (e.poly[0] != e.poly[1])
						continue;
					const unsigned short* eva = &verts[e.vert[0]*3];
					const unsigned short* evb = &verts[e.vert[1]*3];
					if (eva[2] == z && evb[2] == z)
					{
						unsigned short exmin = eva[0];
						unsigned short exmax = evb[0];
						if (exmin > exmax)
							Swap(exmin, exmax);
						if (overlapRangeExl(xmin,xmax, exmin, exmax))
						{
							// Reuse the other polyedge to store dir.
							e.polyEdge[1] = dir;
						}
					}
				}
			}
		}
	}
	
	
	// Store adjacency
	for (int i = 0; i < edgeCount; ++i)
	{
		const rcEdge& e = edges[i];
		if (e.poly[0] != e.poly[1])
		{
			unsigned short* p0 = &polys[e.poly[0]*MAX_VERTS_PER_POLY*2];
			unsigned short* p1 = &polys[e.poly[1]*MAX_VERTS_PER_POLY*2];
			p0[MAX_VERTS_PER_POLY + e.polyEdge[0]] = e.poly[1];
			p1[MAX_VERTS_PER_POLY + e.polyEdge[1]] = e.poly[0];
		}
		else if (e.polyEdge[1] != 0xff)
		{
			unsigned short* p0 = &polys[e.poly[0]*MAX_VERTS_PER_POLY*2];
			p0[MAX_VERTS_PER_POLY + e.polyEdge[0]] = 0x8000 | (unsigned short)e.polyEdge[1];
		}
		
	}
	
	return true;
}


// Last time I checked the if version got compiled using cmov, which was a lot faster than module (with idiv).
inline int prev(int i, int n) { return i-1 >= 0 ? i-1 : n-1; }
inline int next(int i, int n) { return i+1 < n ? i+1 : 0; }

inline int area2(const unsigned char* a, const unsigned char* b, const unsigned char* c)
{
	return ((int)b[0] - (int)a[0]) * ((int)c[2] - (int)a[2]) - ((int)c[0] - (int)a[0]) * ((int)b[2] - (int)a[2]);
}

// Returns true iff c is strictly to the left of the directed
// line through a to b.
inline bool left(const unsigned char* a, const unsigned char* b, const unsigned char* c)
{
	return area2(a, b, c) < 0;
}

inline bool leftOn(const unsigned char* a, const unsigned char* b, const unsigned char* c)
{
	return area2(a, b, c) <= 0;
}

inline bool collinear(const unsigned char* a, const unsigned char* b, const unsigned char* c)
{
	return area2(a, b, c) == 0;
}

//	Returns true iff ab properly intersects cd: they share
//	a point interior to both segments.  The properness of the
//	intersection is ensured by using strict leftness.
static bool intersectProp(const unsigned char* a, const unsigned char* b,
						  const unsigned char* c, const unsigned char* d)
{
	// Eliminate improper cases.
	if (collinear(a,b,c) || collinear(a,b,d) ||
		collinear(c,d,a) || collinear(c,d,b))
		return false;
	
	return (left(a,b,c) ^ left(a,b,d)) && (left(c,d,a) ^ left(c,d,b));
}

// Returns T iff (a,b,c) are collinear and point c lies 
// on the closed segement ab.
static bool between(const unsigned char* a, const unsigned char* b, const unsigned char* c)
{
	if (!collinear(a, b, c))
		return false;
	// If ab not vertical, check betweenness on x; else on y.
	if (a[0] != b[0])
		return ((a[0] <= c[0]) && (c[0] <= b[0])) || ((a[0] >= c[0]) && (c[0] >= b[0]));
	else
		return ((a[2] <= c[2]) && (c[2] <= b[2])) || ((a[2] >= c[2]) && (c[2] >= b[2]));
}

// Returns true iff segments ab and cd intersect, properly or improperly.
static bool intersect(const unsigned char* a, const unsigned char* b,
					  const unsigned char* c, const unsigned char* d)
{
	if (intersectProp(a, b, c, d))
		return true;
	else if (between(a, b, c) || between(a, b, d) ||
			 between(c, d, a) || between(c, d, b))
		return true;
	else
		return false;
}

static bool vequal(const unsigned char* a, const unsigned char* b)
{
	return a[0] == b[0] && a[2] == b[2];
}

// Returns T iff (v_i, v_j) is a proper internal *or* external
// diagonal of P, *ignoring edges incident to v_i and v_j*.
static bool diagonalie(int i, int j, int n, const unsigned char* verts, const unsigned short* indices)
{
	const unsigned char* d0 = &verts[(indices[i] & 0x7fff) * 4];
	const unsigned char* d1 = &verts[(indices[j] & 0x7fff) * 4];
	
	// For each edge (k,k+1) of P
	for (int k = 0; k < n; k++)
	{
		int k1 = next(k, n);
		// Skip edges incident to i or j
		if (!((k == i) || (k1 == i) || (k == j) || (k1 == j)))
		{
			const unsigned char* p0 = &verts[(indices[k] & 0x7fff) * 4];
			const unsigned char* p1 = &verts[(indices[k1] & 0x7fff) * 4];
			
			if (vequal(d0, p0) || vequal(d1, p0) || vequal(d0, p1) || vequal(d1, p1))
				continue;
			
			if (intersect(d0, d1, p0, p1))
				return false;
		}
	}
	return true;
}

// Returns true iff the diagonal (i,j) is strictly internal to the 
// polygon P in the neighborhood of the i endpoint.
static bool	inCone(int i, int j, int n, const unsigned char* verts, const unsigned short* indices)
{
	const unsigned char* pi = &verts[(indices[i] & 0x7fff) * 4];
	const unsigned char* pj = &verts[(indices[j] & 0x7fff) * 4];
	const unsigned char* pi1 = &verts[(indices[next(i, n)] & 0x7fff) * 4];
	const unsigned char* pin1 = &verts[(indices[prev(i, n)] & 0x7fff) * 4];
	
	// If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
	if (leftOn(pin1, pi, pi1))
		return left(pi, pj, pin1) && left(pj, pi, pi1);
	// Assume (i-1,i,i+1) not collinear.
	// else P[i] is reflex.
	return !(leftOn(pi, pj, pi1) && leftOn(pj, pi, pin1));
}

// Returns T iff (v_i, v_j) is a proper internal
// diagonal of P.
static bool diagonal(int i, int j, int n, const unsigned char* verts, const unsigned short* indices)
{
	return inCone(i, j, n, verts, indices) && diagonalie(i, j, n, verts, indices);
}

static int triangulate(int n, const unsigned char* verts, unsigned short* indices, unsigned short* tris)
{
	int ntris = 0;
	unsigned short* dst = tris;
	
	// The last bit of the index is used to indicate if the vertex can be removed.
	for (int i = 0; i < n; i++)
	{
		int i1 = next(i, n);
		int i2 = next(i1, n);
		if (diagonal(i, i2, n, verts, indices))
			indices[i1] |= 0x8000;
	}
	
	while (n > 3)
	{
		int minLen = -1;
		int mini = -1;
		for (int i = 0; i < n; i++)
		{
			int i1 = next(i, n);
			if (indices[i1] & 0x8000)
			{
				const unsigned char* p0 = &verts[(indices[i] & 0x7fff) * 4];
				const unsigned char* p2 = &verts[(indices[next(i1, n)] & 0x7fff) * 4];
				
				const int dx = (int)p2[0] - (int)p0[0];
				const int dz = (int)p2[2] - (int)p0[2];
				const int len = dx*dx + dz*dz;
				if (minLen < 0 || len < minLen)
				{
					minLen = len;
					mini = i;
				}
			}
		}
		
		if (mini == -1)
		{
			// Should not happen.
			/*			printf("mini == -1 ntris=%d n=%d\n", ntris, n);
			 for (int i = 0; i < n; i++)
			 {
			 printf("%d ", indices[i] & 0x0fffffff);
			 }
			 printf("\n");*/
			return -ntris;
		}
		
		int i = mini;
		int i1 = next(i, n);
		int i2 = next(i1, n);
		
		*dst++ = indices[i] & 0x7fff;
		*dst++ = indices[i1] & 0x7fff;
		*dst++ = indices[i2] & 0x7fff;
		ntris++;
		
		// Removes P[i1] by copying P[i+1]...P[n-1] left one index.
		n--;
		for (int k = i1; k < n; k++)
			indices[k] = indices[k+1];
		
		if (i1 >= n) i1 = 0;
		i = prev(i1,n);
		// Update diagonal flags.
		if (diagonal(prev(i, n), i1, n, verts, indices))
			indices[i] |= 0x8000;
		else
			indices[i] &= 0x7fff;
		
		if (diagonal(i, next(i1, n), n, verts, indices))
			indices[i1] |= 0x8000;
		else
			indices[i1] &= 0x7fff;
	}
	
	// Append the remaining triangle.
	*dst++ = indices[0] & 0x7fff;
	*dst++ = indices[1] & 0x7fff;
	*dst++ = indices[2] & 0x7fff;
	ntris++;
	
	return ntris;
}


static int countPolyVerts(const unsigned short* p)
{
	for (int i = 0; i < MAX_VERTS_PER_POLY; ++i)
		if (p[i] == DT_TILECACHE_NULL_IDX)
			return i;
	return MAX_VERTS_PER_POLY;
}

inline bool uleft(const unsigned short* a, const unsigned short* b, const unsigned short* c)
{
	return ((int)b[0] - (int)a[0]) * ((int)c[2] - (int)a[2]) -
	((int)c[0] - (int)a[0]) * ((int)b[2] - (int)a[2]) < 0;
}

static int getPolyMergeValue(unsigned short* pa, unsigned short* pb,
							 const unsigned short* verts, int& ea, int& eb)
{
	const int na = countPolyVerts(pa);
	const int nb = countPolyVerts(pb);
	
	// If the merged polygon would be too big, do not merge.
	if (na+nb-2 > MAX_VERTS_PER_POLY)
		return -1;
	
	// Check if the polygons share an edge.
	ea = -1;
	eb = -1;
	
	for (int i = 0; i < na; ++i)
	{
		unsigned short va0 = pa[i];
		unsigned short va1 = pa[(i+1) % na];
		if (va0 > va1)
			Swap(va0, va1);
		for (int j = 0; j < nb; ++j)
		{
			unsigned short vb0 = pb[j];
			unsigned short vb1 = pb[(j+1) % nb];
			if (vb0 > vb1)
				Swap(vb0, vb1);
			if (va0 == vb0 && va1 == vb1)
			{
				ea = i;
				eb = j;
				break;
			}
		}
	}
	
	// No common edge, cannot merge.
	if (ea == -1 || eb == -1)
		return -1;
	
	// Check to see if the merged polygon would be convex.
	unsigned short va, vb, vc;
	
	va = pa[(ea+na-1) % na];
	vb = pa[ea];
	vc = pb[(eb+2) % nb];
	if (!uleft(&verts[va*3], &verts[vb*3], &verts[vc*3]))
		return -1;
	
	va = pb[(eb+nb-1) % nb];
	vb = pb[eb];
	vc = pa[(ea+2) % na];
	if (!uleft(&verts[va*3], &verts[vb*3], &verts[vc*3]))
		return -1;
	
	va = pa[ea];
	vb = pa[(ea+1)%na];
	
	int dx = (int)verts[va*3+0] - (int)verts[vb*3+0];
	int dy = (int)verts[va*3+2] - (int)verts[vb*3+2];
	
	return dx*dx + dy*dy;
}

static void mergePolys(unsigned short* pa, unsigned short* pb, int ea, int eb)
{
	unsigned short tmp[MAX_VERTS_PER_POLY*2];
	
	const int na = countPolyVerts(pa);
	const int nb = countPolyVerts(pb);
	
	// Merge polygons.
	memset(tmp, 0xff, sizeof(unsigned short)*MAX_VERTS_PER_POLY*2);
	int n = 0;
	// Add pa
	for (int i = 0; i < na-1; ++i)
		tmp[n++] = pa[(ea+1+i) % na];
	// Add pb
	for (int i = 0; i < nb-1; ++i)
		tmp[n++] = pb[(eb+1+i) % nb];
	
	memcpy(pa, tmp, sizeof(unsigned short)*MAX_VERTS_PER_POLY);
}


static void pushFront(unsigned short v, unsigned short* arr, int& an)
{
	an++;
	for (int i = an-1; i > 0; --i)
		arr[i] = arr[i-1];
	arr[0] = v;
}

static void pushBack(unsigned short v, unsigned short* arr, int& an)
{
	arr[an] = v;
	an++;
}

static bool canRemoveVertex(dtTileCachePolyMesh& mesh, const unsigned short rem)
{
	// Count number of polygons to remove.
	int numTouchedVerts = 0;
	int numRemainingEdges = 0;
	for (int i = 0; i < mesh.npolys; ++i)
	{
		unsigned short* p = &mesh.polys[i*MAX_VERTS_PER_POLY*2];
		const int nv = countPolyVerts(p);
		int numRemoved = 0;
		int numVerts = 0;
		for (int j = 0; j < nv; ++j)
		{
			if (p[j] == rem)
			{
				numTouchedVerts++;
				numRemoved++;
			}
			numVerts++;
		}
		if (numRemoved)
		{
			numRemainingEdges += numVerts-(numRemoved+1);
		}
	}
	
	// There would be too few edges remaining to create a polygon.
	// This can happen for example when a tip of a triangle is marked
	// as deletion, but there are no other polys that share the vertex.
	// In this case, the vertex should not be removed.
	if (numRemainingEdges <= 2)
		return false;
	
	// Check that there is enough memory for the test.
	const int maxEdges = numTouchedVerts*2;
	if (maxEdges > MAX_REM_EDGES)
		return false;
	
	// Find edges which share the removed vertex.
	unsigned short edges[MAX_REM_EDGES];
	int nedges = 0;
	
	for (int i = 0; i < mesh.npolys; ++i)
	{
		unsigned short* p = &mesh.polys[i*MAX_VERTS_PER_POLY*2];
		const int nv = countPolyVerts(p);
		
		// Collect edges which touches the removed vertex.
		for (int j = 0, k = nv-1; j < nv; k = j++)
		{
			if (p[j] == rem || p[k] == rem)
			{
				// Arrange edge so that a=rem.
				int a = p[j], b = p[k];
				if (b == rem)
					Swap(a,b);
				
				// Check if the edge exists
				bool exists = false;
				for (int m = 0; m < nedges; ++m)
				{
					unsigned short* e = &edges[m*3];
					if (e[1] == b)
					{
						// Exists, increment vertex share count.
						e[2]++;
						exists = true;
					}
				}
				// Add new edge.
				if (!exists)
				{
					unsigned short* e = &edges[nedges*3];
					e[0] = (unsigned short)a;
					e[1] = (unsigned short)b;
					e[2] = 1;
					nedges++;
				}
			}
		}
	}
	
	// There should be no more than 2 open edges.
	// This catches the case that two non-adjacent polygons
	// share the removed vertex. In that case, do not remove the vertex.
	int numOpenEdges = 0;
	for (int i = 0; i < nedges; ++i)
	{
		if (edges[i*3+2] < 2)
			numOpenEdges++;
	}
	if (numOpenEdges > 2)
		return false;
	
	return true;
}

static Moss_RecastStatus removeVertex(dtTileCachePolyMesh& mesh, const unsigned short rem, const int maxTris)
{
	int nedges = 0;
	unsigned short edges[MAX_REM_EDGES*3];
	int nhole = 0;
	unsigned short hole[MAX_REM_EDGES];
	int nharea = 0;
	unsigned short harea[MAX_REM_EDGES];
	
	for (int i = 0; i < mesh.npolys; ++i)
	{
		unsigned short* p = &mesh.polys[i*MAX_VERTS_PER_POLY*2];
		const int nv = countPolyVerts(p);
		bool hasRem = false;
		for (int j = 0; j < nv; ++j)
			if (p[j] == rem) hasRem = true;
		if (hasRem)
		{
			// Collect edges which does not touch the removed vertex.
			for (int j = 0, k = nv-1; j < nv; k = j++)
			{
				if (p[j] != rem && p[k] != rem)
				{
					if (nedges >= MAX_REM_EDGES)
						return DT_FAILURE | DT_BUFFER_TOO_SMALL;
					unsigned short* e = &edges[nedges*3];
					e[0] = p[k];
					e[1] = p[j];
					e[2] = mesh.areas[i];
					nedges++;
				}
			}
			// Remove the polygon.
			unsigned short* p2 = &mesh.polys[(mesh.npolys-1)*MAX_VERTS_PER_POLY*2];
			memcpy(p,p2,sizeof(unsigned short)*MAX_VERTS_PER_POLY);
			memset(p+MAX_VERTS_PER_POLY,0xff,sizeof(unsigned short)*MAX_VERTS_PER_POLY);
			mesh.areas[i] = mesh.areas[mesh.npolys-1];
			mesh.npolys--;
			--i;
		}
	}
	
	// Remove vertex.
	for (int i = (int)rem; i < mesh.nverts - 1; ++i)
	{
		mesh.verts[i*3+0] = mesh.verts[(i+1)*3+0];
		mesh.verts[i*3+1] = mesh.verts[(i+1)*3+1];
		mesh.verts[i*3+2] = mesh.verts[(i+1)*3+2];
	}
	mesh.nverts--;
	
	// Adjust indices to match the removed vertex layout.
	for (int i = 0; i < mesh.npolys; ++i)
	{
		unsigned short* p = &mesh.polys[i*MAX_VERTS_PER_POLY*2];
		const int nv = countPolyVerts(p);
		for (int j = 0; j < nv; ++j)
			if (p[j] > rem) p[j]--;
	}
	for (int i = 0; i < nedges; ++i)
	{
		if (edges[i*3+0] > rem) edges[i*3+0]--;
		if (edges[i*3+1] > rem) edges[i*3+1]--;
	}
	
	if (nedges == 0)
		return DT_SUCCESS;
	
	// Start with one vertex, keep appending connected
	// segments to the start and end of the hole.
	pushBack(edges[0], hole, nhole);
	pushBack(edges[2], harea, nharea);
	
	while (nedges)
	{
		bool match = false;
		
		for (int i = 0; i < nedges; ++i)
		{
			const unsigned short ea = edges[i*3+0];
			const unsigned short eb = edges[i*3+1];
			const unsigned short a = edges[i*3+2];
			bool add = false;
			if (hole[0] == eb)
			{
				// The segment matches the beginning of the hole boundary.
				if (nhole >= MAX_REM_EDGES)
					return DT_FAILURE | DT_BUFFER_TOO_SMALL;
				pushFront(ea, hole, nhole);
				pushFront(a, harea, nharea);
				add = true;
			}
			else if (hole[nhole-1] == ea)
			{
				// The segment matches the end of the hole boundary.
				if (nhole >= MAX_REM_EDGES)
					return DT_FAILURE | DT_BUFFER_TOO_SMALL;
				pushBack(eb, hole, nhole);
				pushBack(a, harea, nharea);
				add = true;
			}
			if (add)
			{
				// The edge segment was added, remove it.
				edges[i*3+0] = edges[(nedges-1)*3+0];
				edges[i*3+1] = edges[(nedges-1)*3+1];
				edges[i*3+2] = edges[(nedges-1)*3+2];
				--nedges;
				match = true;
				--i;
			}
		}
		
		if (!match)
			break;
	}
	
	
	unsigned short tris[MAX_REM_EDGES*3];
	unsigned char tverts[MAX_REM_EDGES*3];
	unsigned short tpoly[MAX_REM_EDGES*3];
	
	// Generate temp vertex array for triangulation.
	for (int i = 0; i < nhole; ++i)
	{
		const unsigned short pi = hole[i];
		tverts[i*4+0] = (unsigned char)mesh.verts[pi*3+0];
		tverts[i*4+1] = (unsigned char)mesh.verts[pi*3+1];
		tverts[i*4+2] = (unsigned char)mesh.verts[pi*3+2];
		tverts[i*4+3] = 0;
		tpoly[i] = (unsigned short)i;
	}
	
	// Triangulate the hole.
	int ntris = triangulate(nhole, tverts, tpoly, tris);
	if (ntris < 0)
	{
		// TODO: issue warning!
		ntris = -ntris;
	}
	
	if (ntris > MAX_REM_EDGES)
		return DT_FAILURE | DT_BUFFER_TOO_SMALL;
	
	unsigned short polys[MAX_REM_EDGES*MAX_VERTS_PER_POLY];
	unsigned char pareas[MAX_REM_EDGES];
	
	// Build initial polygons.
	int npolys = 0;
	memset(polys, 0xff, ntris*MAX_VERTS_PER_POLY*sizeof(unsigned short));
	for (int j = 0; j < ntris; ++j)
	{
		unsigned short* t = &tris[j*3];
		if (t[0] != t[1] && t[0] != t[2] && t[1] != t[2])
		{
			polys[npolys*MAX_VERTS_PER_POLY+0] = hole[t[0]];
			polys[npolys*MAX_VERTS_PER_POLY+1] = hole[t[1]];
			polys[npolys*MAX_VERTS_PER_POLY+2] = hole[t[2]];
			pareas[npolys] = (unsigned char)harea[t[0]];
			npolys++;
		}
	}
	if (!npolys)
		return DT_SUCCESS;
	
	// Merge polygons.
	int maxVertsPerPoly = MAX_VERTS_PER_POLY;
	if (maxVertsPerPoly > 3)
	{
		for (;;)
		{
			// Find best polygons to merge.
			int bestMergeVal = 0;
			int bestPa = 0, bestPb = 0, bestEa = 0, bestEb = 0;
			
			for (int j = 0; j < npolys-1; ++j)
			{
				unsigned short* pj = &polys[j*MAX_VERTS_PER_POLY];
				for (int k = j+1; k < npolys; ++k)
				{
					unsigned short* pk = &polys[k*MAX_VERTS_PER_POLY];
					int ea, eb;
					int v = getPolyMergeValue(pj, pk, mesh.verts, ea, eb);
					if (v > bestMergeVal)
					{
						bestMergeVal = v;
						bestPa = j;
						bestPb = k;
						bestEa = ea;
						bestEb = eb;
					}
				}
			}
			
			if (bestMergeVal > 0)
			{
				// Found best, merge.
				unsigned short* pa = &polys[bestPa*MAX_VERTS_PER_POLY];
				unsigned short* pb = &polys[bestPb*MAX_VERTS_PER_POLY];
				mergePolys(pa, pb, bestEa, bestEb);
				memcpy(pb, &polys[(npolys-1)*MAX_VERTS_PER_POLY], sizeof(unsigned short)*MAX_VERTS_PER_POLY);
				pareas[bestPb] = pareas[npolys-1];
				npolys--;
			}
			else
			{
				// Could not merge any polygons, stop.
				break;
			}
		}
	}
	
	// Store polygons.
	for (int i = 0; i < npolys; ++i)
	{
		if (mesh.npolys >= maxTris) break;
		unsigned short* p = &mesh.polys[mesh.npolys*MAX_VERTS_PER_POLY*2];
		memset(p,0xff,sizeof(unsigned short)*MAX_VERTS_PER_POLY*2);
		for (int j = 0; j < MAX_VERTS_PER_POLY; ++j)
			p[j] = polys[i*MAX_VERTS_PER_POLY+j];
		mesh.areas[mesh.npolys] = pareas[i];
		mesh.npolys++;
		if (mesh.npolys > maxTris)
			return DT_FAILURE | DT_BUFFER_TOO_SMALL;
	}
	
	return DT_SUCCESS;
}


Moss_RecastStatus dtBuildTileCachePolyMesh(Moss_RecastTileCacheAlloc* alloc,
								  dtTileCacheContourSet& lcset,
								  dtTileCachePolyMesh& mesh)
{
	dtAssert(alloc);
	
	int maxVertices = 0;
	int maxTris = 0;
	int maxVertsPerCont = 0;
	for (int i = 0; i < lcset.nconts; ++i)
	{
		// Skip null contours.
		if (lcset.conts[i].nverts < 3) continue;
		maxVertices += lcset.conts[i].nverts;
		maxTris += lcset.conts[i].nverts - 2;
		maxVertsPerCont = dtMax(maxVertsPerCont, lcset.conts[i].nverts);
	}

	// TODO: warn about too many vertices?
	
	mesh.nvp = MAX_VERTS_PER_POLY;
	
	dtFixedArray<unsigned char> vflags(alloc, maxVertices);
	if (!vflags)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(vflags, 0, maxVertices);
	
	mesh.verts = (unsigned short*)alloc->alloc(sizeof(unsigned short)*maxVertices*3);
	if (!mesh.verts)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	
	mesh.polys = (unsigned short*)alloc->alloc(sizeof(unsigned short)*maxTris*MAX_VERTS_PER_POLY*2);
	if (!mesh.polys)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

	mesh.areas = (unsigned char*)alloc->alloc(sizeof(unsigned char)*maxTris);
	if (!mesh.areas)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

	mesh.flags = (unsigned short*)alloc->alloc(sizeof(unsigned short)*maxTris);
	if (!mesh.flags)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

	// Just allocate and clean the mesh flags array. The user is resposible for filling it.
	memset(mesh.flags, 0, sizeof(unsigned short) * maxTris);
		
	mesh.nverts = 0;
	mesh.npolys = 0;
	
	memset(mesh.verts, 0, sizeof(unsigned short)*maxVertices*3);
	memset(mesh.polys, 0xff, sizeof(unsigned short)*maxTris*MAX_VERTS_PER_POLY*2);
	memset(mesh.areas, 0, sizeof(unsigned char)*maxTris);
	
	unsigned short firstVert[VERTEX_BUCKET_COUNT2];
	for (int i = 0; i < VERTEX_BUCKET_COUNT2; ++i)
		firstVert[i] = DT_TILECACHE_NULL_IDX;
	
	dtFixedArray<unsigned short> nextVert(alloc, maxVertices);
	if (!nextVert)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(nextVert, 0, sizeof(unsigned short)*maxVertices);
	
	dtFixedArray<unsigned short> indices(alloc, maxVertsPerCont);
	if (!indices)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	
	dtFixedArray<unsigned short> tris(alloc, maxVertsPerCont*3);
	if (!tris)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

	dtFixedArray<unsigned short> polys(alloc, maxVertsPerCont*MAX_VERTS_PER_POLY);
	if (!polys)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	
	for (int i = 0; i < lcset.nconts; ++i)
	{
		dtTileCacheContour& cont = lcset.conts[i];
		
		// Skip null contours.
		if (cont.nverts < 3)
			continue;
		
		// Triangulate contour
		for (int j = 0; j < cont.nverts; ++j)
			indices[j] = (unsigned short)j;
		
		int ntris = triangulate(cont.nverts, cont.verts, &indices[0], &tris[0]);
		if (ntris <= 0)
		{
			// TODO: issue warning!
			ntris = -ntris;
		}
		
		// Add and merge vertices.
		for (int j = 0; j < cont.nverts; ++j)
		{
			const unsigned char* v = &cont.verts[j*4];
			indices[j] = addVertex((unsigned short)v[0], (unsigned short)v[1], (unsigned short)v[2],
								   mesh.verts, firstVert, nextVert, mesh.nverts);
			if (v[3] & 0x80)
			{
				// This vertex should be removed.
				vflags[indices[j]] = 1;
			}
		}
		
		// Build initial polygons.
		int npolys = 0;
		memset(polys, 0xff, sizeof(unsigned short) * maxVertsPerCont * MAX_VERTS_PER_POLY);
		for (int j = 0; j < ntris; ++j)
		{
			const unsigned short* t = &tris[j*3];
			if (t[0] != t[1] && t[0] != t[2] && t[1] != t[2])
			{
				polys[npolys*MAX_VERTS_PER_POLY+0] = indices[t[0]];
				polys[npolys*MAX_VERTS_PER_POLY+1] = indices[t[1]];
				polys[npolys*MAX_VERTS_PER_POLY+2] = indices[t[2]];
				npolys++;
			}
		}
		if (!npolys)
			continue;
		
		// Merge polygons.
		int maxVertsPerPoly =MAX_VERTS_PER_POLY ;
		if (maxVertsPerPoly > 3)
		{
			for(;;)
			{
				// Find best polygons to merge.
				int bestMergeVal = 0;
				int bestPa = 0, bestPb = 0, bestEa = 0, bestEb = 0;
				
				for (int j = 0; j < npolys-1; ++j)
				{
					unsigned short* pj = &polys[j*MAX_VERTS_PER_POLY];
					for (int k = j+1; k < npolys; ++k)
					{
						unsigned short* pk = &polys[k*MAX_VERTS_PER_POLY];
						int ea, eb;
						int v = getPolyMergeValue(pj, pk, mesh.verts, ea, eb);
						if (v > bestMergeVal)
						{
							bestMergeVal = v;
							bestPa = j;
							bestPb = k;
							bestEa = ea;
							bestEb = eb;
						}
					}
				}
				
				if (bestMergeVal > 0)
				{
					// Found best, merge.
					unsigned short* pa = &polys[bestPa*MAX_VERTS_PER_POLY];
					unsigned short* pb = &polys[bestPb*MAX_VERTS_PER_POLY];
					mergePolys(pa, pb, bestEa, bestEb);
					memcpy(pb, &polys[(npolys-1)*MAX_VERTS_PER_POLY], sizeof(unsigned short)*MAX_VERTS_PER_POLY);
					npolys--;
				}
				else
				{
					// Could not merge any polygons, stop.
					break;
				}
			}
		}
		
		// Store polygons.
		for (int j = 0; j < npolys; ++j)
		{
			unsigned short* p = &mesh.polys[mesh.npolys*MAX_VERTS_PER_POLY*2];
			unsigned short* q = &polys[j*MAX_VERTS_PER_POLY];
			for (int k = 0; k < MAX_VERTS_PER_POLY; ++k)
				p[k] = q[k];
			mesh.areas[mesh.npolys] = cont.area;
			mesh.npolys++;
			if (mesh.npolys > maxTris)
				return DT_FAILURE | DT_BUFFER_TOO_SMALL;
		}
	}
	
	
	// Remove edge vertices.
	for (int i = 0; i < mesh.nverts; ++i)
	{
		if (vflags[i])
		{
			if (!canRemoveVertex(mesh, (unsigned short)i))
				continue;
			Moss_RecastStatus status = removeVertex(mesh, (unsigned short)i, maxTris);
			if (dtStatusFailed(status))
				return status;
			// Remove vertex
			// Note: mesh.nverts is already decremented inside removeVertex()!
			for (int j = i; j < mesh.nverts; ++j)
				vflags[j] = vflags[j+1];
			--i;
		}
	}
	
	// Calculate adjacency.
	if (!buildMeshAdjacency(alloc, mesh.polys, mesh.npolys, mesh.verts, mesh.nverts, lcset))
		return DT_FAILURE | DT_OUT_OF_MEMORY;
		
	return DT_SUCCESS;
}

Moss_RecastStatus dtMarkCylinderArea(dtTileCacheLayer& layer, const float* orig, const float cs, const float ch,
							const float* pos, const float radius, const float height, const unsigned char areaId)
{
	float bmin[3], bmax[3];
	bmin[0] = pos[0] - radius;
	bmin[1] = pos[1];
	bmin[2] = pos[2] - radius;
	bmax[0] = pos[0] + radius;
	bmax[1] = pos[1] + height;
	bmax[2] = pos[2] + radius;
	const float r2 = Sqr(radius/cs + 0.5f);

	const int w = (int)layer.header->width;
	const int h = (int)layer.header->height;
	const float ics = 1.0f/cs;
	const float ich = 1.0f/ch;
	
	const float px = (pos[0]-orig[0])*ics;
	const float pz = (pos[2]-orig[2])*ics;
	
	int minx = (int)floorf((bmin[0]-orig[0])*ics);
	int miny = (int)floorf((bmin[1]-orig[1])*ich);
	int minz = (int)floorf((bmin[2]-orig[2])*ics);
	int maxx = (int)floorf((bmax[0]-orig[0])*ics);
	int maxy = (int)floorf((bmax[1]-orig[1])*ich);
	int maxz = (int)floorf((bmax[2]-orig[2])*ics);

	if (maxx < 0) return DT_SUCCESS;
	if (minx >= w) return DT_SUCCESS;
	if (maxz < 0) return DT_SUCCESS;
	if (minz >= h) return DT_SUCCESS;
	
	if (minx < 0) minx = 0;
	if (maxx >= w) maxx = w-1;
	if (minz < 0) minz = 0;
	if (maxz >= h) maxz = h-1;
	
	for (int z = minz; z <= maxz; ++z) {
		for (int x = minx; x <= maxx; ++x) {
			const float dx = (float)(x+0.5f) - px;
			const float dz = (float)(z+0.5f) - pz;
			if (dx*dx + dz*dz > r2)
				continue;
			const int y = layer.heights[x+z*w];
			if (y < miny || y > maxy)
				continue;
			layer.areas[x+z*w] = areaId;
		}
	}

	return DT_SUCCESS;
}

Moss_RecastStatus dtMarkBoxArea(dtTileCacheLayer& layer, const float* orig, const float cs, const float ch,
					   const float* bmin, const float* bmax, const unsigned char areaId) {
	const int w = (int)layer.header->width;
	const int h = (int)layer.header->height;
	const float ics = 1.0f/cs;
	const float ich = 1.0f/ch;

	int minx = (int)floorf((bmin[0]-orig[0])*ics);
	int miny = (int)floorf((bmin[1]-orig[1])*ich);
	int minz = (int)floorf((bmin[2]-orig[2])*ics);
	int maxx = (int)floorf((bmax[0]-orig[0])*ics);
	int maxy = (int)floorf((bmax[1]-orig[1])*ich);
	int maxz = (int)floorf((bmax[2]-orig[2])*ics);
	
	if (maxx < 0) return DT_SUCCESS;
	if (minx >= w) return DT_SUCCESS;
	if (maxz < 0) return DT_SUCCESS;
	if (minz >= h) return DT_SUCCESS;

	if (minx < 0) minx = 0;
	if (maxx >= w) maxx = w-1;
	if (minz < 0) minz = 0;
	if (maxz >= h) maxz = h-1;
	
	for (int z = minz; z <= maxz; ++z) {
		for (int x = minx; x <= maxx; ++x) {
			const int y = layer.heights[x+z*w];
			if (y < miny || y > maxy)
				continue;
			layer.areas[x+z*w] = areaId;
		}
	}

	return DT_SUCCESS;
}

Moss_RecastStatus dtMarkBoxArea(dtTileCacheLayer& layer, const float* orig, const float cs, const float ch,
					   const float* center, const float* halfExtents, const float* rotAux, const unsigned char areaId) {
	const int w = (int)layer.header->width;
	const int h = (int)layer.header->height;
	const float ics = 1.0f/cs;
	const float ich = 1.0f/ch;

	float cx = (center[0] - orig[0])*ics;
	float cz = (center[2] - orig[2])*ics;
	
	float maxr = 1.41f*dtMax(halfExtents[0], halfExtents[2]);
	int minx = (int)floorf(cx - maxr*ics);
	int maxx = (int)floorf(cx + maxr*ics);
	int minz = (int)floorf(cz - maxr*ics);
	int maxz = (int)floorf(cz + maxr*ics);
	int miny = (int)floorf((center[1]-halfExtents[1]-orig[1])*ich);
	int maxy = (int)floorf((center[1]+halfExtents[1]-orig[1])*ich);

	if (maxx < 0) return DT_SUCCESS;
	if (minx >= w) return DT_SUCCESS;
	if (maxz < 0) return DT_SUCCESS;
	if (minz >= h) return DT_SUCCESS;

	if (minx < 0) minx = 0;
	if (maxx >= w) maxx = w-1;
	if (minz < 0) minz = 0;
	if (maxz >= h) maxz = h-1;
	
	float xhalf = halfExtents[0]*ics + 0.5f;
	float zhalf = halfExtents[2]*ics + 0.5f;

	for (int z = minz; z <= maxz; ++z) {
		for (int x = minx; x <= maxx; ++x) {
			float x2 = 2.0f*(float(x) - cx);
			float z2 = 2.0f*(float(z) - cz);
			float xrot = rotAux[1]*x2 + rotAux[0]*z2;
			if (xrot > xhalf || xrot < -xhalf)
				continue;
			float zrot = rotAux[1]*z2 - rotAux[0]*x2;
			if (zrot > zhalf || zrot < -zhalf)
				continue;
			const int y = layer.heights[x+z*w];
			if (y < miny || y > maxy)
				continue;
			layer.areas[x+z*w] = areaId;
		}
	}

	return DT_SUCCESS;
}

Moss_RecastStatus dtBuildTileCacheLayer(Moss_RecastTileCacheCompressor* comp,
							   Moss_RecastTileCacheLayerHeader* header,
							   const unsigned char* heights,
							   const unsigned char* areas,
							   const unsigned char* cons,
							   unsigned char** outData, int* outDataSize) {
	const int headerSize = dtAlign4(sizeof(Moss_RecastTileCacheLayerHeader));
	const int gridSize = (int)header->width * (int)header->height;
	const int maxDataSize = headerSize + comp->maxCompressedSize(gridSize*3);
	unsigned char* data = (unsigned char*)dtAlloc(maxDataSize, DT_ALLOC_PERM);
	if (!data)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(data, 0, maxDataSize);
	
	// Store header
	memcpy(data, header, sizeof(Moss_RecastTileCacheLayerHeader));
	
	// Concatenate grid data for compression.
	const int bufferSize = gridSize*3;
	unsigned char* buffer = (unsigned char*)dtAlloc(bufferSize, DT_ALLOC_TEMP);
	if (!buffer) {
		MOSS_FREE(data);
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	}

	memcpy(buffer, heights, gridSize);
	memcpy(buffer+gridSize, areas, gridSize);
	memcpy(buffer+gridSize*2, cons, gridSize);
	
	// Compress
	unsigned char* compressed = data + headerSize;
	const int maxCompressedSize = maxDataSize - headerSize;
	int compressedSize = 0;
	Moss_RecastStatus status = comp->compress(buffer, bufferSize, compressed, maxCompressedSize, &compressedSize);
	if (dtStatusFailed(status)) {
		MOSS_FREE(buffer);
		MOSS_FREE(data);
		return status;
	}

	*outData = data;
	*outDataSize = headerSize + compressedSize;
	
	MOSS_FREE(buffer);
	
	return DT_SUCCESS;
}

void dtFreeTileCacheLayer(Moss_RecastTileCacheAlloc* alloc, dtTileCacheLayer* layer) {
	dtAssert(alloc);
	// The layer is allocated as one conitguous blob of data.
	alloc->free(layer);
}

Moss_RecastStatus dtDecompressTileCacheLayer(Moss_RecastTileCacheAlloc* alloc, Moss_RecastTileCacheCompressor* comp, unsigned char* compressed, const int compressedSize, dtTileCacheLayer** layerOut) {
	dtAssert(alloc);
	dtAssert(comp);

	if (!layerOut)
		return DT_FAILURE | DT_INVALID_PARAM;
	if (!compressed)
		return DT_FAILURE | DT_INVALID_PARAM;

	*layerOut = 0;

	Moss_RecastTileCacheLayerHeader* compressedHeader = (Moss_RecastTileCacheLayerHeader*)compressed;
	if (compressedHeader->magic != DT_TILECACHE_MAGIC)
		return DT_FAILURE | DT_WRONG_MAGIC;
	if (compressedHeader->version != DT_TILECACHE_VERSION)
		return DT_FAILURE | DT_WRONG_VERSION;
	
	const int layerSize = dtAlign4(sizeof(dtTileCacheLayer));
	const int headerSize = dtAlign4(sizeof(Moss_RecastTileCacheLayerHeader));
	const int gridSize = (int)compressedHeader->width * (int)compressedHeader->height;
	const int bufferSize = layerSize + headerSize + gridSize*4;
	
	unsigned char* buffer = (unsigned char*)alloc->alloc(bufferSize);
	if (!buffer)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(buffer, 0, bufferSize);

	dtTileCacheLayer* layer = (dtTileCacheLayer*)buffer;
	Moss_RecastTileCacheLayerHeader* header = (Moss_RecastTileCacheLayerHeader*)(buffer + layerSize);
	unsigned char* grids = buffer + layerSize + headerSize;
	const int gridsSize = bufferSize - (layerSize + headerSize); 
	
	// Copy header
	memcpy(header, compressedHeader, headerSize);
	// Decompress grid.
	int size = 0;
	Moss_RecastStatus status = comp->decompress(compressed+headerSize, compressedSize-headerSize,
									   grids, gridsSize, &size);
	if (dtStatusFailed(status)) {
		alloc->free(buffer);
		return status;
	}
	
	layer->header = header;
	layer->heights = grids;
	layer->areas = grids + gridSize;
	layer->cons = grids + gridSize*2;
	layer->regs = grids + gridSize*3;
	
	*layerOut = layer;
	
	return DT_SUCCESS;
}



bool dtTileCacheHeaderSwapEndian(unsigned char* data, const int dataSize) {
	dtIgnoreUnused(dataSize);
	Moss_RecastTileCacheLayerHeader* header = (Moss_RecastTileCacheLayerHeader*)data;
	
	int swappedMagic = DT_TILECACHE_MAGIC;
	int swappedVersion = DT_TILECACHE_VERSION;
	SwapEndian(&swappedMagic);
	SwapEndian(&swappedVersion);
	
	if ((header->magic != DT_TILECACHE_MAGIC || header->version != DT_TILECACHE_VERSION) && (header->magic != swappedMagic || header->version != swappedVersion))
	{ return false; }
	
	SwapEndian(&header->magic);
	SwapEndian(&header->version);
	SwapEndian(&header->tx);
	SwapEndian(&header->ty);
	SwapEndian(&header->tlayer);
	SwapEndian(&header->bmin[0]);
	SwapEndian(&header->bmin[1]);
	SwapEndian(&header->bmin[2]);
	SwapEndian(&header->bmax[0]);
	SwapEndian(&header->bmax[1]);
	SwapEndian(&header->bmax[2]);
	SwapEndian(&header->hmin);
	SwapEndian(&header->hmax);
	
	// width, height, minx, maxx, miny, maxy are unsigned char, no need to swap.
	
	return true;
}
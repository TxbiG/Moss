// Recast.cpp
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <float.h>
#include <stdlib.h>
#include <stdarg.h>

#include <Moss/Navigation/navigation_intern.h>
#ifndef MOSS_DEBUG_RENDERER
	#include <Moss/Moss_Renderer.h>
#endif // MOSS_DEBUG_RENDERER


static const unsigned RC_UNSET_HEIGHT = 0xffff;

// Must be 255 or smaller (not 256) because layer IDs are stored as
// a byte where 255 is a special value.
#ifndef RC_MAX_LAYERS_DEF
#define RC_MAX_LAYERS_DEF 63
#endif

#if RC_MAX_LAYERS_DEF > 255
#error RC_MAX_LAYERS_DEF must be 255 or smaller
#endif

#ifndef RC_MAX_NEIS_DEF
#define RC_MAX_NEIS_DEF 16
#endif

// Keep type checking.
static const int RC_MAX_LAYERS = RC_MAX_LAYERS_DEF;
static const int RC_MAX_NEIS = RC_MAX_NEIS_DEF;

/*													*/

static rcAssertFailFunc* sRecastAssertFailFunc = 0;

void rcAssertFailSetCustom(rcAssertFailFunc* assertFailFunc) {
	sRecastAssertFailFunc = assertFailFunc;
}

rcAssertFailFunc* rcAssertFailGetCustom() { return sRecastAssertFailFunc; }

static void* rcAllocDefault(size_t size, rcAllocHint) {
	return Moss_Malloc(size);
}

static rcAllocFunc* sRecastAllocFunc = rcAllocDefault;
static rcFreeFunc* sRecastFreeFunc = MOSS_FREE;

void rcAllocSetCustom(rcAllocFunc* allocFunc, rcFreeFunc* freeFunc) {
	sRecastAllocFunc = allocFunc ? allocFunc : rcAllocDefault;
	sRecastFreeFunc = freeFunc ? freeFunc : MOSS_FREE;
}

void* rcAlloc(size_t size, rcAllocHint hint) { return sRecastAllocFunc(size, hint); }

void Moss_Free(void* ptr) { if (ptr != NULL) { sRecastFreeFunc(ptr); } }


/*													*/

namespace
{
	const int MAX_HEIGHTFIELD_HEIGHT = 0xffff; // TODO (graham): Move this to a more visible constant and update usages.
}

void rcFilterLowHangingWalkableObstacles(rcContext* context, const int walkableClimb, Moss_RecastHeightfield& heightfield) {
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_FILTER_LOW_OBSTACLES);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;

	for (int z = 0; z < zSize; ++z) {
		for (int x = 0; x < xSize; ++x) {
			Moss_RecastSpan* previousSpan = NULL;
			bool previousWasWalkable = false;
			unsigned char previousAreaID = RC_NULL_AREA;

			// For each span in the column...
			for (Moss_RecastSpan* span = heightfield.spans[x + z * xSize]; span != NULL; previousSpan = span, span = span->next) {
				const bool walkable = span->area != RC_NULL_AREA;

				// If current span is not walkable, but there is walkable span just below it and the height difference
				// is small enough for the agent to walk over, mark the current span as walkable too.
				if (!walkable && previousWasWalkable && (int)span->smax - (int)previousSpan->smax <= walkableClimb) {
					span->area = previousAreaID;
				}

				// Copy the original walkable value regardless of whether we changed it.
				// This prevents multiple consecutive non-walkable spans from being erroneously marked as walkable.
				previousWasWalkable = walkable;
				previousAreaID = span->area;
			}
		}
	}
}

void rcFilterLedgeSpans(rcContext* context, const int walkableHeight, const int walkableClimb, Moss_RecastHeightfield& heightfield) {
	rcAssert(context);
	
	rcScopedTimer timer(context, RC_TIMER_FILTER_BORDER);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;
	
	// Mark spans that are adjacent to a ledge as unwalkable..
	for (int z = 0; z < zSize; ++z)	{
		for (int x = 0; x < xSize; ++x) {
			for (Moss_RecastSpan* span = heightfield.spans[x + z * xSize]; span; span = span->next) {
				// Skip non-walkable spans.
				if (span->area == RC_NULL_AREA) { continue; }

				const int floor = (int)(span->smax);
				const int ceiling = span->next ? (int)(span->next->smin) : MAX_HEIGHTFIELD_HEIGHT;

				// The difference between this walkable area and the lowest neighbor walkable area.
				// This is the difference between the current span and all neighbor spans that have
				// enough space for an agent to move between, but not accounting at all for surface slope.
				int lowestNeighborFloorDifference = MAX_HEIGHTFIELD_HEIGHT;

				// Min and max height of accessible neighbours.
				int lowestTraversableNeighborFloor = span->smax;
				int highestTraversableNeighborFloor = span->smax;

				for (int direction = 0; direction < 4; ++direction) {
					const int neighborX = x + rcGetDirOffsetX(direction);
					const int neighborZ = z + rcGetDirOffsetY(direction);

					// Skip neighbours which are out of bounds.
					if (neighborX < 0 || neighborZ < 0 || neighborX >= xSize || neighborZ >= zSize) {
						lowestNeighborFloorDifference = -walkableClimb - 1;
						break;
					}

					const Moss_RecastSpan* neighborSpan = heightfield.spans[neighborX + neighborZ * xSize];

					// The most we can step down to the neighbor is the walkableClimb distance.
					// Start with the area under the neighbor span
					int neighborCeiling = neighborSpan ? (int)neighborSpan->smin : MAX_HEIGHTFIELD_HEIGHT;

					// Skip neighbour if the gap between the spans is too small.
					if (min(ceiling, neighborCeiling) - floor >= walkableHeight) {
						lowestNeighborFloorDifference = (-walkableClimb - 1);
						break;
					}

					// For each span in the neighboring column...
					for (; neighborSpan != NULL; neighborSpan = neighborSpan->next) {
						const int neighborFloor = (int)neighborSpan->smax;
						neighborCeiling = neighborSpan->next ? (int)neighborSpan->next->smin : MAX_HEIGHTFIELD_HEIGHT;

						// Only consider neighboring areas that have enough overlap to be potentially traversable.
						if (min(ceiling, neighborCeiling) - max(floor, neighborFloor) < walkableHeight) {
							// No space to traverse between them.
							continue;
						}

						const int neighborFloorDifference = neighborFloor - floor;
						lowestNeighborFloorDifference = min(lowestNeighborFloorDifference, neighborFloorDifference);

						// Find min/max accessible neighbor height.
						// Only consider neighbors that are at most walkableClimb away.
						if (abs(neighborFloorDifference) <= walkableClimb) {
							// There is space to move to the neighbor cell and the slope isn't too much.
							lowestTraversableNeighborFloor = min(lowestTraversableNeighborFloor, neighborFloor);
							highestTraversableNeighborFloor = max(highestTraversableNeighborFloor, neighborFloor);
						}
						else if (neighborFloorDifference < -walkableClimb) {
							// We already know this will be considered a ledge span so we can early-out
							break;
						}
					}
				}

				// The current span is close to a ledge if the magnitude of the drop to any neighbour span is greater than the walkableClimb distance.
				// That is, there is a gap that is large enough to let an agent move between them, but the drop (surface slope) is too large to allow it.
				// (If this is the case, then biggestNeighborStepDown will be negative, so compare against the negative walkableClimb as a means of checking
				// the magnitude of the delta)
				if (lowestNeighborFloorDifference < -walkableClimb) {
					span->area = RC_NULL_AREA;
				}
				// If the difference between all neighbor floors is too large, this is a steep slope, so mark the span as an unwalkable ledge.
				else if (highestTraversableNeighborFloor - lowestTraversableNeighborFloor > walkableClimb) {
					span->area = RC_NULL_AREA;
				}
			}
		}
	}
}

void rcFilterWalkableLowHeightSpans(rcContext* context, const int walkableHeight, Moss_RecastHeightfield& heightfield) {
	rcAssert(context);
	rcScopedTimer timer(context, RC_TIMER_FILTER_WALKABLE);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;

	// Remove walkable flag from spans which do not have enough
	// space above them for the agent to stand there.
	for (int z = 0; z < zSize; ++z) {
		for (int x = 0; x < xSize; ++x) {
			for (Moss_RecastSpan* span = heightfield.spans[x + z*xSize]; span; span = span->next) {
				const int floor = (int)(span->smax);
				const int ceiling = span->next ? (int)(span->next->smin) : MAX_HEIGHTFIELD_HEIGHT;
				if (ceiling - floor < walkableHeight) { span->area = RC_NULL_AREA; }
			}
		}
	}
}


/*													*/

namespace
{
/// Allocates and constructs an object of the given type, returning a pointer.
/// @param[in]		allocLifetime	Allocation lifetime hint
template<typename T>
T* rcNew(const rcAllocHint allocLifetime) {
	T* ptr = (T*)rcAlloc(sizeof(T), allocLifetime);
	::new(rcNewTag(), (void*)ptr) T();
	return ptr;
}

/// Destroys and frees an object allocated with rcNew.
/// @param[in]     ptr    The object pointer to delete.
template<typename T>
void rcDelete(T* ptr) {
	if (ptr) {
		ptr->~T();
		MOSS_FREE((void*)ptr);
	}
}
} // anonymous namespace

Moss_RecastHeightfield* rcAllocHeightfield() {
	return rcNew<Moss_RecastHeightfield>(RC_ALLOC_PERM);
}

void rcFreeHeightField(Moss_RecastHeightfield* heightfield) {
	rcDelete(heightfield);
}

Moss_RecastHeightfield::Moss_RecastHeightfield() : width(), height(), box(), cs(), ch(), spans(), pools(), freelist() { }

Moss_RecastHeightfield::~Moss_RecastHeightfield() {
	// Delete span array.
	MOSS_FREE(spans);
	// Delete span pools.
	while (pools) {
		Moss_RecastSpanPool* next = pools->next;
		MOSS_FREE(pools);
		pools = next;
	}
}

Moss_RecastCompactHeightfield* rcAllocCompactHeightfield() {
	return rcNew<Moss_RecastCompactHeightfield>(RC_ALLOC_PERM);
}

void rcFreeCompactHeightfield(Moss_RecastCompactHeightfield* compactHeightfield) {
	rcDelete(compactHeightfield);
}

Moss_RecastCompactHeightfield::Moss_RecastCompactHeightfield(): width(), height(), spanCount(), walkableHeight(), walkableClimb(), borderSize()
, maxDistance(), maxRegions(), box();, cs(), ch(), cells(), spans(), dist(), areas() { }

Moss_RecastCompactHeightfield::~Moss_RecastCompactHeightfield() {
	MOSS_FREE(cells);
	MOSS_FREE(spans);
	MOSS_FREE(dist);
	MOSS_FREE(areas);
}

Moss_RecastHeightfieldLayerSet* rcAllocHeightfieldLayerSet() {
	return rcNew<Moss_RecastHeightfieldLayerSet>(RC_ALLOC_PERM);
}

void rcFreeHeightfieldLayerSet(Moss_RecastHeightfieldLayerSet* layerSet) { rcDelete(layerSet); }

Moss_RecastHeightfieldLayerSet::Moss_RecastHeightfieldLayerSet() : layers() , nlayers() { }

Moss_RecastHeightfieldLayerSet::~Moss_RecastHeightfieldLayerSet() {
	for (int i = 0; i < nlayers; ++i) {
		MOSS_FREE(layers[i].heights);
		MOSS_FREE(layers[i].areas);
		MOSS_FREE(layers[i].cons);
	}
	MOSS_FREE(layers);
}


Moss_RecastContourSet* rcAllocContourSet() {
	return rcNew<Moss_RecastContourSet>(RC_ALLOC_PERM);
}

void rcFreeContourSet(Moss_RecastContourSet* contourSet) {
	rcDelete(contourSet);
}

Moss_RecastContourSet::Moss_RecastContourSet() : conts(), nconts(), box(), cs(), ch(), width(), height(), borderSize(), maxError() {}

Moss_RecastContourSet::~Moss_RecastContourSet() {
	for (int i = 0; i < nconts; ++i) {
		MOSS_FREE(conts[i].verts);
		MOSS_FREE(conts[i].rverts);
	}
	MOSS_FREE(conts);
}

Moss_RecastPolyMesh* rcAllocPolyMesh() {
	return rcNew<Moss_RecastPolyMesh>(RC_ALLOC_PERM);
}

void rcFreePolyMesh(Moss_RecastPolyMesh* polyMesh) {
	rcDelete(polyMesh);
}

Moss_RecastPolyMesh::Moss_RecastPolyMesh() : verts(), polys(), regs(), flags(), areas(), nverts(), npolys(), maxpolys(), nvp(), box(), cs(), ch(), borderSize(), maxEdgeError() { }

Moss_RecastPolyMesh::~Moss_RecastPolyMesh() {
	MOSS_FREE(verts);
	MOSS_FREE(polys);
	MOSS_FREE(regs);
	MOSS_FREE(flags);
	MOSS_FREE(areas);
}

Moss_RecastPolyMeshDetail* rcAllocPolyMeshDetail() {
	return rcNew<Moss_RecastPolyMeshDetail>(RC_ALLOC_PERM);
}

void rcFreePolyMeshDetail(Moss_RecastPolyMeshDetail* detailMesh) {
	if (detailMesh == NULL)
	{
		return;
	}
	MOSS_FREE(detailMesh->meshes);
	MOSS_FREE(detailMesh->verts);
	MOSS_FREE(detailMesh->tris);
	MOSS_FREE(detailMesh);
}

Moss_RecastPolyMeshDetail::Moss_RecastPolyMeshDetail() : meshes(), verts(), tris(), nmeshes(), nverts(), ntris() { }

inline AABB3 rcCalcBounds(const Float3* verts, int numVerts) {
    AABB3 bounds;
    bounds.Reset();

    for (int i = 0; i < numVerts; ++i)
        bounds.Encapsulate(verts[i]);

    return bounds;
}

inline void rcCalcGridSize(const AABB3& bounds, float cellSize, int& sizeX, int& sizeZ) {
    const Float3 extent = bounds.Max() - bounds.Min();
    sizeX = static_cast<int>(extent.x / cellSize + 0.5f);
    sizeZ = static_cast<int>(extent.z / cellSize + 0.5f);
}
inline bool rcCreateHeightfield(rcContext* ctx,
                                Moss_RecastHeightfield& hf,
                                int sizeX, int sizeZ,
                                const AABB3& bounds,
                                float cellSize, float cellHeight)
{
    return rcCreateHeightfield(ctx, hf, sizeX, sizeZ,
                               bounds.Min().Data(),
                               bounds.Max().Data(),
                               cellSize, cellHeight);
}

static void calcTriNormal(const float* v0, const float* v1, const float* v2, float* faceNormal) {
	float e0[3], e1[3];
	dtVsub(e0, v1, v0);
	dtVsub(e1, v2, v0);
	Cross(faceNormal, e0, e1);
	rcVnormalize(faceNormal);
}

void rcMarkWalkableTriangles(rcContext* context, const float walkableSlopeAngle, const float* verts, const int numVerts,
                             const int* tris, const int numTris, unsigned char* triAreaIDs) {
	rcIgnoreUnused(context);
	rcIgnoreUnused(numVerts);

	const float walkableThr = cosf(walkableSlopeAngle / 180.0f * RC_PI);

	float norm[3];

	for (int i = 0; i < numTris; ++i) {
		const int* tri = &tris[i * 3];
		calcTriNormal(&verts[tri[0] * 3], &verts[tri[1] * 3], &verts[tri[2] * 3], norm);
		// Check if the face is walkable.
		if (norm[1] > walkableThr) {
			triAreaIDs[i] = RC_WALKABLE_AREA;
		}
	}
}

void rcClearUnwalkableTriangles(rcContext* context, const float walkableSlopeAngle, const float* verts, int numVerts,
                                const int* tris, int numTris, unsigned char* triAreaIDs) {
	rcIgnoreUnused(context);
	rcIgnoreUnused(numVerts);

	// The minimum Y value for a face normal of a triangle with a walkable slope.
	const float walkableLimitY = cosf(walkableSlopeAngle / 180.0f * RC_PI);

	float faceNormal[3];
	for (int i = 0; i < numTris; ++i) {
		const int* tri = &tris[i * 3];
		calcTriNormal(&verts[tri[0] * 3], &verts[tri[1] * 3], &verts[tri[2] * 3], faceNormal);
		// Check if the face is walkable.
		if (faceNormal[1] <= walkableLimitY) {
			triAreaIDs[i] = RC_NULL_AREA;
		}
	}
}

int rcGetHeightFieldSpanCount(rcContext* context, const Moss_RecastHeightfield& heightfield) {
	rcIgnoreUnused(context);

	const int numCols = heightfield.width * heightfield.height;
	int spanCount = 0;
	for (int columnIndex = 0; columnIndex < numCols; ++columnIndex) {
		for (Moss_RecastSpan* span = heightfield.spans[columnIndex]; span != NULL; span = span->next) {
			if (span->area != RC_NULL_AREA) {
				spanCount++;
			}
		}
	}
	return spanCount;
}

bool rcBuildCompactHeightfield(rcContext* context, const int walkableHeight, const int walkableClimb, const Moss_RecastHeightfield& heightfield, Moss_RecastCompactHeightfield& compactHeightfield) {
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_BUILD_COMPACTHEIGHTFIELD);

	const int xSize = heightfield.width;
	const int zSize = heightfield.height;
	const int spanCount = rcGetHeightFieldSpanCount(context, heightfield);

	// Fill in header.
	compactHeightfield.width = xSize;
	compactHeightfield.height = zSize;
	compactHeightfield.spanCount = spanCount;
	compactHeightfield.walkableHeight = walkableHeight;
	compactHeightfield.walkableClimb = walkableClimb;
	compactHeightfield.maxRegions = 0;
	rcVcopy(compactHeightfield.bmin, heightfield.bmin);
	rcVcopy(compactHeightfield.bmax, heightfield.bmax);
	compactHeightfield.bmax[1] += walkableHeight * heightfield.ch;
	compactHeightfield.cs = heightfield.cs;
	compactHeightfield.ch = heightfield.ch;
	compactHeightfield.cells = (Moss_RecastCompactCell*)rcAlloc(sizeof(Moss_RecastCompactCell) * xSize * zSize, RC_ALLOC_PERM);
	if (!compactHeightfield.cells) {
		MOSS_ERROR("rcBuildCompactHeightfield: Out of memory 'chf.cells' (%d)", xSize * zSize);
		return false;
	}
	memset(compactHeightfield.cells, 0, sizeof(Moss_RecastCompactCell) * xSize * zSize);
	compactHeightfield.spans = (Moss_RecastCompactSpan*)rcAlloc(sizeof(Moss_RecastCompactSpan) * spanCount, RC_ALLOC_PERM);
	if (!compactHeightfield.spans) {
		MOSS_ERROR("rcBuildCompactHeightfield: Out of memory 'chf.spans' (%d)", spanCount);
		return false;
	}
	memset(compactHeightfield.spans, 0, sizeof(Moss_RecastCompactSpan) * spanCount);
	compactHeightfield.areas = (unsigned char*)rcAlloc(sizeof(unsigned char) * spanCount, RC_ALLOC_PERM);
	if (!compactHeightfield.areas) {
		MOSS_ERROR("rcBuildCompactHeightfield: Out of memory 'chf.areas' (%d)", spanCount);
		return false;
	}
	memset(compactHeightfield.areas, RC_NULL_AREA, sizeof(unsigned char) * spanCount);

	const int MAX_HEIGHT = 0xffff;

	// Fill in cells and spans.
	int currentCellIndex = 0;
	const int numColumns = xSize * zSize;
	for (int columnIndex = 0; columnIndex < numColumns; ++columnIndex) {
		const Moss_RecastSpan* span = heightfield.spans[columnIndex];
			
		// If there are no spans at this cell, just leave the data to index=0, count=0.
		if (span == NULL) { continue; }
			
		Moss_RecastCompactCell& cell = compactHeightfield.cells[columnIndex];
		cell.index = currentCellIndex;
		cell.count = 0;

		for (; span != NULL; span = span->next) {
			if (span->area != RC_NULL_AREA) {
				const int bot = (int)span->smax;
				const int top = span->next ? (int)span->next->smin : MAX_HEIGHT;
				compactHeightfield.spans[currentCellIndex].y = (unsigned short)Clamp(bot, 0, 0xffff);
				compactHeightfield.spans[currentCellIndex].h = (unsigned char)Clamp(top - bot, 0, 0xff);
				compactHeightfield.areas[currentCellIndex] = span->area;
				currentCellIndex++;
				cell.count++;
			}
		}
	}
	
	// Find neighbour connections.
	const int MAX_LAYERS = RC_NOT_CONNECTED - 1;
	int maxLayerIndex = 0;
	const int zStride = xSize; // for readability
	for (int z = 0; z < zSize; ++z) {
		for (int x = 0; x < xSize; ++x) {
			const Moss_RecastCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			for (int i = (int)cell.index, ni = (int)(cell.index + cell.count); i < ni; ++i) {
				Moss_RecastCompactSpan& span = compactHeightfield.spans[i];

				for (int dir = 0; dir < 4; ++dir) {
					rcSetCon(span, dir, RC_NOT_CONNECTED);
					const int neighborX = x + rcGetDirOffsetX(dir);
					const int neighborZ = z + rcGetDirOffsetY(dir);
					// First check that the neighbour cell is in bounds.
					if (neighborX < 0 || neighborZ < 0 || neighborX >= xSize || neighborZ >= zSize) { continue; }

					// Iterate over all neighbour spans and check if any of the is
					// accessible from current cell.
					const Moss_RecastCompactCell& neighborCell = compactHeightfield.cells[neighborX + neighborZ * zStride];
					for (int k = (int)neighborCell.index, nk = (int)(neighborCell.index + neighborCell.count); k < nk; ++k) {
						const Moss_RecastCompactSpan& neighborSpan = compactHeightfield.spans[k];
						const int bot = max(span.y, neighborSpan.y);
						const int top = min(span.y + span.h, neighborSpan.y + neighborSpan.h);

						// Check that the gap between the spans is walkable,
						// and that the climb height between the gaps is not too high.
						if ((top - bot) >= walkableHeight && abs((int)neighborSpan.y - (int)span.y) <= walkableClimb) {
							// Mark direction as walkable.
							const int layerIndex = k - (int)neighborCell.index;
							if (layerIndex < 0 || layerIndex > MAX_LAYERS) {
								maxLayerIndex = max(maxLayerIndex, layerIndex);
								continue;
							}
							rcSetCon(span, dir, layerIndex);
							break;
						}
					}
				}
			}
		}
	}

	if (maxLayerIndex > MAX_LAYERS) {
		MOSS_ERROR("rcBuildCompactHeightfield: Heightfield has too many layers %d (max: %d)", maxLayerIndex, MAX_LAYERS);
	}

	return true;
}


/*													*/

/// Check whether two bounding boxes overlap
///
/// @param[in]	aMin	Min axis extents of bounding box A
/// @param[in]	aMax	Max axis extents of bounding box A
/// @param[in]	bMin	Min axis extents of bounding box B
/// @param[in]	bMax	Max axis extents of bounding box B
/// @returns true if the two bounding boxes overlap.  False otherwise.
static bool overlapBounds(const float* aMin, const float* aMax, const float* bMin, const float* bMax) {
	return aMin[0] <= bMax[0] && aMax[0] >= bMin[0] && aMin[1] <= bMax[1] && aMax[1] >= bMin[1] &&aMin[2] <= bMax[2] && aMax[2] >= bMin[2];
}

/// Allocates a new span in the heightfield.
/// Use a memory pool and free list to minimize actual allocations.
/// 
/// @param[in]	heightfield		The heightfield
/// @returns A pointer to the allocated or re-used span memory. 
static Moss_RecastSpan* allocSpan(Moss_RecastHeightfield& heightfield)
{
	// If necessary, allocate new page and update the freelist.
	if (heightfield.freelist == NULL || heightfield.freelist->next == NULL)
	{
		// Create new page.
		// Allocate memory for the new pool.
		Moss_RecastSpanPool* spanPool = (Moss_RecastSpanPool*)rcAlloc(sizeof(Moss_RecastSpanPool), RC_ALLOC_PERM);
		if (spanPool == NULL)
		{
			return NULL;
		}

		// Add the pool into the list of pools.
		spanPool->next = heightfield.pools;
		heightfield.pools = spanPool;
		
		// Add new spans to the free list.
		Moss_RecastSpan* freeList = heightfield.freelist;
		Moss_RecastSpan* head = &spanPool->items[0];
		Moss_RecastSpan* it = &spanPool->items[RC_SPANS_PER_POOL];
		do
		{
			--it;
			it->next = freeList;
			freeList = it;
		}
		while (it != head);
		heightfield.freelist = it;
	}

	// Pop item from the front of the free list.
	Moss_RecastSpan* newSpan = heightfield.freelist;
	heightfield.freelist = heightfield.freelist->next;
	return newSpan;
}

/// Releases the memory used by the span back to the heightfield, so it can be re-used for new spans.
/// @param[in]	heightfield		The heightfield.
/// @param[in]	span	A pointer to the span to free
static void freeSpan(Moss_RecastHeightfield& heightfield, Moss_RecastSpan* span)
{
	if (span == NULL)
	{
		return;
	}
	// Add the span to the front of the free list.
	span->next = heightfield.freelist;
	heightfield.freelist = span;
}

/// Adds a span to the heightfield.  If the new span overlaps existing spans,
/// it will merge the new span with the existing ones.
///
/// @param[in]	heightfield					Heightfield to add spans to
/// @param[in]	x					The new span's column cell x index
/// @param[in]	z					The new span's column cell z index
/// @param[in]	min					The new span's minimum cell index
/// @param[in]	max					The new span's maximum cell index
/// @param[in]	areaID				The new span's area type ID
/// @param[in]	flagMergeThreshold	How close two spans maximum extents need to be to merge area type IDs
static bool addSpan(Moss_RecastHeightfield& heightfield,
                    const int x, const int z,
                    const unsigned short min, const unsigned short max,
                    const unsigned char areaID, const int flagMergeThreshold)
{
	// Create the new span.
	Moss_RecastSpan* newSpan = allocSpan(heightfield);
	if (newSpan == NULL)
	{
		return false;
	}
	newSpan->smin = min;
	newSpan->smax = max;
	newSpan->area = areaID;
	newSpan->next = NULL;
	
	const int columnIndex = x + z * heightfield.width;
	Moss_RecastSpan* previousSpan = NULL;
	Moss_RecastSpan* currentSpan = heightfield.spans[columnIndex];
	
	// Insert the new span, possibly merging it with existing spans.
	while (currentSpan != NULL)
	{
		if (currentSpan->smin > newSpan->smax)
		{
			// Current span is completely after the new span, break.
			break;
		}
		
		if (currentSpan->smax < newSpan->smin)
		{
			// Current span is completely before the new span.  Keep going.
			previousSpan = currentSpan;
			currentSpan = currentSpan->next;
		}
		else
		{
			// The new span overlaps with an existing span.  Merge them.
			if (currentSpan->smin < newSpan->smin)
			{
				newSpan->smin = currentSpan->smin;
			}
			if (currentSpan->smax > newSpan->smax)
			{
				newSpan->smax = currentSpan->smax;
			}
			
			// Merge flags.
			if (abs((int)newSpan->smax - (int)currentSpan->smax) <= flagMergeThreshold)
			{
				// Higher area ID numbers indicate higher resolution priority.
				newSpan->area = max(newSpan->area, currentSpan->area);
			}
			
			// Remove the current span since it's now merged with newSpan.
			// Keep going because there might be other overlapping spans that also need to be merged.
			Moss_RecastSpan* next = currentSpan->next;
			freeSpan(heightfield, currentSpan);
			if (previousSpan)
			{
				previousSpan->next = next;
			}
			else
			{
				heightfield.spans[columnIndex] = next;
			}
			currentSpan = next;
		}
	}
	
	// Insert new span after prev
	if (previousSpan != NULL)
	{
		newSpan->next = previousSpan->next;
		previousSpan->next = newSpan;
	}
	else
	{
		// This span should go before the others in the list
		newSpan->next = heightfield.spans[columnIndex];
		heightfield.spans[columnIndex] = newSpan;
	}

	return true;
}

bool rcAddSpan(rcContext* context, Moss_RecastHeightfield& heightfield,
               const int x, const int z,
               const unsigned short spanMin, const unsigned short spanMax,
               const unsigned char areaID, const int flagMergeThreshold)
{
	rcAssert(context);

	if (!addSpan(heightfield, x, z, spanMin, spanMax, areaID, flagMergeThreshold))
	{
		context->log(RC_LOG_ERROR, "rcAddSpan: Out of memory.");
		return false;
	}

	return true;
}

enum rcAxis
{
	RC_AXIS_X = 0,
	RC_AXIS_Y = 1,
	RC_AXIS_Z = 2
};

/// Divides a convex polygon of max 12 vertices into two convex polygons
/// across a separating axis.
/// 
/// @param[in]	inVerts			The input polygon vertices
/// @param[in]	inVertsCount	The number of input polygon vertices
/// @param[out]	outVerts1		Resulting polygon 1's vertices
/// @param[out]	outVerts1Count	The number of resulting polygon 1 vertices
/// @param[out]	outVerts2		Resulting polygon 2's vertices
/// @param[out]	outVerts2Count	The number of resulting polygon 2 vertices
/// @param[in]	axisOffset		THe offset along the specified axis
/// @param[in]	axis			The separating axis
static void dividePoly(const float* inVerts, int inVertsCount,
                       float* outVerts1, int* outVerts1Count,
                       float* outVerts2, int* outVerts2Count,
                       float axisOffset, rcAxis axis)
{
	rcAssert(inVertsCount <= 12);
	
	// How far positive or negative away from the separating axis is each vertex.
	float inVertAxisDelta[12];
	for (int inVert = 0; inVert < inVertsCount; ++inVert)
	{
		inVertAxisDelta[inVert] = axisOffset - inVerts[inVert * 3 + axis];
	}

	int poly1Vert = 0;
	int poly2Vert = 0;
	for (int inVertA = 0, inVertB = inVertsCount - 1; inVertA < inVertsCount; inVertB = inVertA, ++inVertA)
	{
		// If the two vertices are on the same side of the separating axis
		bool sameSide = (inVertAxisDelta[inVertA] >= 0) == (inVertAxisDelta[inVertB] >= 0);

		if (!sameSide)
		{
			float s = inVertAxisDelta[inVertB] / (inVertAxisDelta[inVertB] - inVertAxisDelta[inVertA]);
			outVerts1[poly1Vert * 3 + 0] = inVerts[inVertB * 3 + 0] + (inVerts[inVertA * 3 + 0] - inVerts[inVertB * 3 + 0]) * s;
			outVerts1[poly1Vert * 3 + 1] = inVerts[inVertB * 3 + 1] + (inVerts[inVertA * 3 + 1] - inVerts[inVertB * 3 + 1]) * s;
			outVerts1[poly1Vert * 3 + 2] = inVerts[inVertB * 3 + 2] + (inVerts[inVertA * 3 + 2] - inVerts[inVertB * 3 + 2]) * s;
			rcVcopy(&outVerts2[poly2Vert * 3], &outVerts1[poly1Vert * 3]);
			poly1Vert++;
			poly2Vert++;
			
			// add the inVertA point to the right polygon. Do NOT add points that are on the dividing line
			// since these were already added above
			if (inVertAxisDelta[inVertA] > 0)
			{
				rcVcopy(&outVerts1[poly1Vert * 3], &inVerts[inVertA * 3]);
				poly1Vert++;
			}
			else if (inVertAxisDelta[inVertA] < 0)
			{
				rcVcopy(&outVerts2[poly2Vert * 3], &inVerts[inVertA * 3]);
				poly2Vert++;
			}
		}
		else
		{
			// add the inVertA point to the right polygon. Addition is done even for points on the dividing line
			if (inVertAxisDelta[inVertA] >= 0)
			{
				rcVcopy(&outVerts1[poly1Vert * 3], &inVerts[inVertA * 3]);
				poly1Vert++;
				if (inVertAxisDelta[inVertA] != 0)
				{
					continue;
				}
			}
			rcVcopy(&outVerts2[poly2Vert * 3], &inVerts[inVertA * 3]);
			poly2Vert++;
		}
	}

	*outVerts1Count = poly1Vert;
	*outVerts2Count = poly2Vert;
}

///	Rasterize a single triangle to the heightfield.
///
///	This code is extremely hot, so much care should be given to maintaining maximum perf here.
/// 
/// @param[in] 	v0					Triangle vertex 0
/// @param[in] 	v1					Triangle vertex 1
/// @param[in] 	v2					Triangle vertex 2
/// @param[in] 	areaID				The area ID to assign to the rasterized spans
/// @param[in] 	heightfield			Heightfield to rasterize into
/// @param[in] 	heightfieldBBMin	The min extents of the heightfield bounding box
/// @param[in] 	heightfieldBBMax	The max extents of the heightfield bounding box
/// @param[in] 	cellSize			The x and z axis size of a voxel in the heightfield
/// @param[in] 	inverseCellSize		1 / cellSize
/// @param[in] 	inverseCellHeight	1 / cellHeight
/// @param[in] 	flagMergeThreshold	The threshold in which area flags will be merged 
/// @returns true if the operation completes successfully.  false if there was an error adding spans to the heightfield.
static bool rasterizeTri(const float* v0, const float* v1, const float* v2,
                         const unsigned char areaID, Moss_RecastHeightfield& heightfield,
                         const float* heightfieldBBMin, const float* heightfieldBBMax,
                         const float cellSize, const float inverseCellSize, const float inverseCellHeight,
                         const int flagMergeThreshold)
{
	// Calculate the bounding box of the triangle.
	float triBBMin[3];
	rcVcopy(triBBMin, v0);
	rcVmin(triBBMin, v1);
	rcVmin(triBBMin, v2);

	float triBBMax[3];
	rcVcopy(triBBMax, v0);
	rcVmax(triBBMax, v1);
	rcVmax(triBBMax, v2);

	// If the triangle does not touch the bounding box of the heightfield, skip the triangle.
	if (!overlapBounds(triBBMin, triBBMax, heightfieldBBMin, heightfieldBBMax))
	{
		return true;
	}

	const int w = heightfield.width;
	const int h = heightfield.height;
	const float by = heightfieldBBMax[1] - heightfieldBBMin[1];

	// Calculate the footprint of the triangle on the grid's z-axis
	int z0 = (int)((triBBMin[2] - heightfieldBBMin[2]) * inverseCellSize);
	int z1 = (int)((triBBMax[2] - heightfieldBBMin[2]) * inverseCellSize);

	// use -1 rather than 0 to cut the polygon properly at the start of the tile
	z0 = Clamp(z0, -1, h - 1);
	z1 = Clamp(z1, 0, h - 1);

	// Clip the triangle into all grid cells it touches.
	float buf[7 * 3 * 4];
	float* in = buf;
	float* inRow = buf + 7 * 3;
	float* p1 = inRow + 7 * 3;
	float* p2 = p1 + 7 * 3;

	rcVcopy(&in[0], v0);
	rcVcopy(&in[1 * 3], v1);
	rcVcopy(&in[2 * 3], v2);
	int nvRow;
	int nvIn = 3;

	for (int z = z0; z <= z1; ++z)
	{
		// Clip polygon to row. Store the remaining polygon as well
		const float cellZ = heightfieldBBMin[2] + (float)z * cellSize;
		dividePoly(in, nvIn, inRow, &nvRow, p1, &nvIn, cellZ + cellSize, RC_AXIS_Z);
		Swap(in, p1);
		
		if (nvRow < 3)
		{
			continue;
		}
		if (z < 0)
		{
			continue;
		}
		
		// find X-axis bounds of the row
		float minX = inRow[0];
		float maxX = inRow[0];
		for (int vert = 1; vert < nvRow; ++vert)
		{
			if (minX > inRow[vert * 3])
			{
				minX = inRow[vert * 3];
			}
			if (maxX < inRow[vert * 3])
			{
				maxX = inRow[vert * 3];
			}
		}
		int x0 = (int)((minX - heightfieldBBMin[0]) * inverseCellSize);
		int x1 = (int)((maxX - heightfieldBBMin[0]) * inverseCellSize);
		if (x1 < 0 || x0 >= w)
		{
			continue;
		}
		x0 = Clamp(x0, -1, w - 1);
		x1 = Clamp(x1, 0, w - 1);

		int nv;
		int nv2 = nvRow;

		for (int x = x0; x <= x1; ++x)
		{
			// Clip polygon to column. store the remaining polygon as well
			const float cx = heightfieldBBMin[0] + (float)x * cellSize;
			dividePoly(inRow, nv2, p1, &nv, p2, &nv2, cx + cellSize, RC_AXIS_X);
			Swap(inRow, p2);
			
			if (nv < 3)
			{
				continue;
			}
			if (x < 0)
			{
				continue;
			}
			
			// Calculate min and max of the span.
			float spanMin = p1[1];
			float spanMax = p1[1];
			for (int vert = 1; vert < nv; ++vert)
			{
				spanMin = min(spanMin, p1[vert * 3 + 1]);
				spanMax = max(spanMax, p1[vert * 3 + 1]);
			}
			spanMin -= heightfieldBBMin[1];
			spanMax -= heightfieldBBMin[1];
			
			// Skip the span if it's completely outside the heightfield bounding box
			if (spanMax < 0.0f)
			{
				continue;
			}
			if (spanMin > by)
			{
				continue;
			}
			
			// Clamp the span to the heightfield bounding box.
			if (spanMin < 0.0f)
			{
				spanMin = 0;
			}
			if (spanMax > by)
			{
				spanMax = by;
			}

			// Snap the span to the heightfield height grid.
			unsigned short spanMinCellIndex = (unsigned short)Clamp((int)floorf(spanMin * inverseCellHeight), 0, RC_SPAN_MAX_HEIGHT);
			unsigned short spanMaxCellIndex = (unsigned short)Clamp((int)ceilf(spanMax * inverseCellHeight), (int)spanMinCellIndex + 1, RC_SPAN_MAX_HEIGHT);

			if (!addSpan(heightfield, x, z, spanMinCellIndex, spanMaxCellIndex, areaID, flagMergeThreshold))
			{
				return false;
			}
		}
	}

	return true;
}

bool rcRasterizeTriangle(rcContext* context,
                         const float* v0, const float* v1, const float* v2,
                         const unsigned char areaID, Moss_RecastHeightfield& heightfield, const int flagMergeThreshold)
{
	rcAssert(context != NULL);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);

	// Rasterize the single triangle.
	const float inverseCellSize = 1.0f / heightfield.cs;
	const float inverseCellHeight = 1.0f / heightfield.ch;
	if (!rasterizeTri(v0, v1, v2, areaID, heightfield, heightfield.bmin, heightfield.bmax, heightfield.cs, inverseCellSize, inverseCellHeight, flagMergeThreshold))
	{
		context->log(RC_LOG_ERROR, "rcRasterizeTriangle: Out of memory.");
		return false;
	}

	return true;
}

bool rcRasterizeTriangles(rcContext* context,
                          const float* verts, const int /*nv*/,
                          const int* tris, const unsigned char* triAreaIDs, const int numTris,
                          Moss_RecastHeightfield& heightfield, const int flagMergeThreshold)
{
	rcAssert(context != NULL);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);
	
	// Rasterize the triangles.
	const float inverseCellSize = 1.0f / heightfield.cs;
	const float inverseCellHeight = 1.0f / heightfield.ch;
	for (int triIndex = 0; triIndex < numTris; ++triIndex)
	{
		const float* v0 = &verts[tris[triIndex * 3 + 0] * 3];
		const float* v1 = &verts[tris[triIndex * 3 + 1] * 3];
		const float* v2 = &verts[tris[triIndex * 3 + 2] * 3];
		if (!rasterizeTri(v0, v1, v2, triAreaIDs[triIndex], heightfield, heightfield.bmin, heightfield.bmax, heightfield.cs, inverseCellSize, inverseCellHeight, flagMergeThreshold))
		{
			context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
			return false;
		}
	}

	return true;
}

bool rcRasterizeTriangles(rcContext* context,
                          const float* verts, const int /*nv*/,
                          const unsigned short* tris, const unsigned char* triAreaIDs, const int numTris,
                          Moss_RecastHeightfield& heightfield, const int flagMergeThreshold)
{
	rcAssert(context != NULL);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);

	// Rasterize the triangles.
	const float inverseCellSize = 1.0f / heightfield.cs;
	const float inverseCellHeight = 1.0f / heightfield.ch;
	for (int triIndex = 0; triIndex < numTris; ++triIndex)
	{
		const float* v0 = &verts[tris[triIndex * 3 + 0] * 3];
		const float* v1 = &verts[tris[triIndex * 3 + 1] * 3];
		const float* v2 = &verts[tris[triIndex * 3 + 2] * 3];
		if (!rasterizeTri(v0, v1, v2, triAreaIDs[triIndex], heightfield, heightfield.bmin, heightfield.bmax, heightfield.cs, inverseCellSize, inverseCellHeight, flagMergeThreshold))
		{
			context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
			return false;
		}
	}

	return true;
}

bool rcRasterizeTriangles(rcContext* context,
                          const float* verts, const unsigned char* triAreaIDs, const int numTris,
                          Moss_RecastHeightfield& heightfield, const int flagMergeThreshold)
{
	rcAssert(context != NULL);

	rcScopedTimer timer(context, RC_TIMER_RASTERIZE_TRIANGLES);
	
	// Rasterize the triangles.
	const float inverseCellSize = 1.0f / heightfield.cs;
	const float inverseCellHeight = 1.0f / heightfield.ch;
	for (int triIndex = 0; triIndex < numTris; ++triIndex)
	{
		const float* v0 = &verts[(triIndex * 3 + 0) * 3];
		const float* v1 = &verts[(triIndex * 3 + 1) * 3];
		const float* v2 = &verts[(triIndex * 3 + 2) * 3];
		if (!rasterizeTri(v0, v1, v2, triAreaIDs[triIndex], heightfield, heightfield.bmin, heightfield.bmax, heightfield.cs, inverseCellSize, inverseCellHeight, flagMergeThreshold))
		{
			context->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
			return false;
		}
	}

	return true;
}


/*													*/

struct rcLayerRegion
{
	unsigned char layers[RC_MAX_LAYERS];
	unsigned char neis[RC_MAX_NEIS];
	unsigned short ymin, ymax;
	unsigned char layerId;		// Layer ID
	unsigned char nlayers;		// Layer count
	unsigned char nneis;		// Neighbour count
	unsigned char base;		// Flag indicating if the region is the base of merged regions.
};


static bool contains(const unsigned char* a, const unsigned char an, const unsigned char v) {
	const int n = (int)an;
	for (int i = 0; i < n; ++i)
	{
		if (a[i] == v)
			return true;
	}
	return false;
}

static bool addUnique(unsigned char* a, unsigned char& an, int anMax, unsigned char v) {
	if (contains(a, an, v))
		return true;

	if ((int)an >= anMax)
		return false;

	a[an] = v;
	an++;
	return true;
}


inline bool overlapRange(const unsigned short amin, const unsigned short amax, const unsigned short bmin, const unsigned short bmax) {
	return (amin > bmax || amax < bmin) ? false : true;
}



struct rcLayerSweepSpan {
	unsigned short ns;	// number samples
	unsigned char id;	// region id
	unsigned char nei;	// neighbour id
};

/// @par
/// 
/// See the #Moss_RecastSettings3D documentation for more information on the configuration parameters.
/// 
/// @see rcAllocHeightfieldLayerSet, Moss_RecastCompactHeightfield, Moss_RecastHeightfieldLayerSet, Moss_RecastSettings3D
bool rcBuildHeightfieldLayers(rcContext* ctx, const Moss_RecastCompactHeightfield& chf,
							  const int borderSize, const int walkableHeight,
							  Moss_RecastHeightfieldLayerSet& lset)
{
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_BUILD_LAYERS);
	
	const int w = chf.width;
	const int h = chf.height;
	
	rcScopedDelete<unsigned char> srcReg((unsigned char*)rcAlloc(sizeof(unsigned char)*chf.spanCount, RC_ALLOC_TEMP));
	if (!srcReg)
	{
		MOSS_ERROR("rcBuildHeightfieldLayers: Out of memory 'srcReg' (%d).", chf.spanCount);
		return false;
	}
	memset(srcReg,0xff,sizeof(unsigned char)*chf.spanCount);
	
	const int nsweeps = chf.width;
	rcScopedDelete<rcLayerSweepSpan> sweeps((rcLayerSweepSpan*)rcAlloc(sizeof(rcLayerSweepSpan)*nsweeps, RC_ALLOC_TEMP));
	if (!sweeps)
	{
		MOSS_ERROR("rcBuildHeightfieldLayers: Out of memory 'sweeps' (%d).", nsweeps);
		return false;
	}
	
	
	// Partition walkable area into monotone regions.
	int prevCount[256];
	unsigned char regId = 0;

	for (int y = borderSize; y < h-borderSize; ++y)
	{
		memset(prevCount,0,sizeof(int)*regId);
		unsigned char sweepId = 0;
		
		for (int x = borderSize; x < w-borderSize; ++x)
		{
			const Moss_RecastCompactCell& c = chf.cells[x+y*w];
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const Moss_RecastCompactSpan& s = chf.spans[i];
				if (chf.areas[i] == RC_NULL_AREA) continue;

				unsigned char sid = 0xff;

				// -x
				if (rcGetCon(s, 0) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(0);
					const int ay = y + rcGetDirOffsetY(0);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 0);
					if (chf.areas[ai] != RC_NULL_AREA && srcReg[ai] != 0xff)
						sid = srcReg[ai];
				}
				
				if (sid == 0xff)
				{
					sid = sweepId++;
					sweeps[sid].nei = 0xff;
					sweeps[sid].ns = 0;
				}
				
				// -y
				if (rcGetCon(s,3) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(3);
					const int ay = y + rcGetDirOffsetY(3);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 3);
					const unsigned char nr = srcReg[ai];
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
				
				srcReg[i] = sid;
			}
		}
		
		// Create unique ID.
		for (int i = 0; i < sweepId; ++i)
		{
			// If the neighbour is set and there is only one continuous connection to it,
			// the sweep will be merged with the previous one, else new region is created.
			if (sweeps[i].nei != 0xff && prevCount[sweeps[i].nei] == (int)sweeps[i].ns)
			{
				sweeps[i].id = sweeps[i].nei;
			}
			else
			{
				if (regId == 255)
				{
					MOSS_ERROR("rcBuildHeightfieldLayers: Region ID overflow.");
					return false;
				}
				sweeps[i].id = regId++;
			}
		}
		
		// Remap local sweep ids to region ids.
		for (int x = borderSize; x < w-borderSize; ++x)
		{
			const Moss_RecastCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				if (srcReg[i] != 0xff)
					srcReg[i] = sweeps[srcReg[i]].id;
			}
		}
	}

	// Allocate and init layer regions.
	const int nregs = (int)regId;
	rcScopedDelete<rcLayerRegion> regs((rcLayerRegion*)rcAlloc(sizeof(rcLayerRegion)*nregs, RC_ALLOC_TEMP));
	if (!regs)
	{
		MOSS_ERROR("rcBuildHeightfieldLayers: Out of memory 'regs' (%d).", nregs);
		return false;
	}
	memset(regs, 0, sizeof(rcLayerRegion)*nregs);
	for (int i = 0; i < nregs; ++i)
	{
		regs[i].layerId = 0xff;
		regs[i].ymin = 0xffff;
		regs[i].ymax = 0;
	}
	
	// Find region neighbours and overlapping regions.
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const Moss_RecastCompactCell& c = chf.cells[x+y*w];
			
			unsigned char lregs[RC_MAX_LAYERS];
			int nlregs = 0;
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const Moss_RecastCompactSpan& s = chf.spans[i];
				const unsigned char ri = srcReg[i];
				if (ri == 0xff) continue;
				
				regs[ri].ymin = min(regs[ri].ymin, s.y);
				regs[ri].ymax = max(regs[ri].ymax, s.y);
				
				// Collect all region layers.
				if (nlregs < RC_MAX_LAYERS)
					lregs[nlregs++] = ri;
				
				// Update neighbours
				for (int dir = 0; dir < 4; ++dir)
				{
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);
						const unsigned char rai = srcReg[ai];
						if (rai != 0xff && rai != ri)
						{
							// Don't check return value -- if we cannot add the neighbor
							// it will just cause a few more regions to be created, which
							// is fine.
							addUnique(regs[ri].neis, regs[ri].nneis, RC_MAX_NEIS, rai);
						}
					}
				}
				
			}
			
			// Update overlapping regions.
			for (int i = 0; i < nlregs-1; ++i)
			{
				for (int j = i+1; j < nlregs; ++j)
				{
					if (lregs[i] != lregs[j])
					{
						rcLayerRegion& ri = regs[lregs[i]];
						rcLayerRegion& rj = regs[lregs[j]];

						if (!addUnique(ri.layers, ri.nlayers, RC_MAX_LAYERS, lregs[j]) ||
							!addUnique(rj.layers, rj.nlayers, RC_MAX_LAYERS, lregs[i]))
						{
							MOSS_ERROR("rcBuildHeightfieldLayers: layer overflow (too many overlapping walkable platforms). Try increasing RC_MAX_LAYERS.");
							return false;
						}
					}
				}
			}
			
		}
	}
	
	// Create 2D layers from regions.
	unsigned char layerId = 0;
	
	static const int MAX_STACK = 64;
	unsigned char stack[MAX_STACK];
	int nstack = 0;
	
	for (int i = 0; i < nregs; ++i)
	{
		rcLayerRegion& root = regs[i];
		// Skip already visited.
		if (root.layerId != 0xff)
			continue;

		// Start search.
		root.layerId = layerId;
		root.base = 1;
		
		nstack = 0;
		stack[nstack++] = (unsigned char)i;
		
		while (nstack)
		{
			// Pop front
			rcLayerRegion& reg = regs[stack[0]];
			nstack--;
			for (int j = 0; j < nstack; ++j)
				stack[j] = stack[j+1];
			
			const int nneis = (int)reg.nneis;
			for (int j = 0; j < nneis; ++j)
			{
				const unsigned char nei = reg.neis[j];
				rcLayerRegion& regn = regs[nei];
				// Skip already visited.
				if (regn.layerId != 0xff)
					continue;
				// Skip if the neighbour is overlapping root region.
				if (contains(root.layers, root.nlayers, nei))
					continue;
				// Skip if the height range would become too large.
				const int ymin = min(root.ymin, regn.ymin);
				const int ymax = max(root.ymax, regn.ymax);
				if ((ymax - ymin) >= 255)
					 continue;

				if (nstack < MAX_STACK)
				{
					// Deepen
					stack[nstack++] = (unsigned char)nei;
					
					// Mark layer id
					regn.layerId = layerId;
					// Merge current layers to root.
					for (int k = 0; k < regn.nlayers; ++k)
					{
						if (!addUnique(root.layers, root.nlayers, RC_MAX_LAYERS, regn.layers[k]))
						{
							MOSS_ERROR("rcBuildHeightfieldLayers: layer overflow (too many overlapping walkable platforms). Try increasing RC_MAX_LAYERS.");
							return false;
						}
					}
					root.ymin = min(root.ymin, regn.ymin);
					root.ymax = max(root.ymax, regn.ymax);
				}
			}
		}
		
		layerId++;
	}
	
	// Merge non-overlapping regions that are close in height.
	const unsigned short mergeHeight = (unsigned short)walkableHeight * 4;
	
	for (int i = 0; i < nregs; ++i)
	{
		rcLayerRegion& ri = regs[i];
		if (!ri.base) continue;
		
		unsigned char newId = ri.layerId;
		
		for (;;)
		{
			unsigned char oldId = 0xff;
			
			for (int j = 0; j < nregs; ++j)
			{
				if (i == j) continue;
				rcLayerRegion& rj = regs[j];
				if (!rj.base) continue;
				
				// Skip if the regions are not close to each other.
				if (!overlapRange(ri.ymin,ri.ymax+mergeHeight, rj.ymin,rj.ymax+mergeHeight))
					continue;
				// Skip if the height range would become too large.
				const int ymin = min(ri.ymin, rj.ymin);
				const int ymax = max(ri.ymax, rj.ymax);
				if ((ymax - ymin) >= 255)
				  continue;
						  
				// Make sure that there is no overlap when merging 'ri' and 'rj'.
				bool overlap = false;
				// Iterate over all regions which have the same layerId as 'rj'
				for (int k = 0; k < nregs; ++k)
				{
					if (regs[k].layerId != rj.layerId)
						continue;
					// Check if region 'k' is overlapping region 'ri'
					// Index to 'regs' is the same as region id.
					if (contains(ri.layers,ri.nlayers, (unsigned char)k))
					{
						overlap = true;
						break;
					}
				}
				// Cannot merge of regions overlap.
				if (overlap)
					continue;
				
				// Can merge i and j.
				oldId = rj.layerId;
				break;
			}
			
			// Could not find anything to merge with, stop.
			if (oldId == 0xff)
				break;
			
			// Merge
			for (int j = 0; j < nregs; ++j)
			{
				rcLayerRegion& rj = regs[j];
				if (rj.layerId == oldId)
				{
					rj.base = 0;
					// Remap layerIds.
					rj.layerId = newId;
					// Add overlaid layers from 'rj' to 'ri'.
					for (int k = 0; k < rj.nlayers; ++k)
					{
						if (!addUnique(ri.layers, ri.nlayers, RC_MAX_LAYERS, rj.layers[k]))
						{
							MOSS_ERROR("rcBuildHeightfieldLayers: layer overflow (too many overlapping walkable platforms). Try increasing RC_MAX_LAYERS.");
							return false;
						}
					}

					// Update height bounds.
					ri.ymin = min(ri.ymin, rj.ymin);
					ri.ymax = max(ri.ymax, rj.ymax);
				}
			}
		}
	}
	
	// Compact layerIds
	unsigned char remap[256];
	memset(remap, 0, 256);

	// Find number of unique layers.
	layerId = 0;
	for (int i = 0; i < nregs; ++i)
		remap[regs[i].layerId] = 1;
	for (int i = 0; i < 256; ++i)
	{
		if (remap[i])
			remap[i] = layerId++;
		else
			remap[i] = 0xff;
	}
	// Remap ids.
	for (int i = 0; i < nregs; ++i)
		regs[i].layerId = remap[regs[i].layerId];
	
	// No layers, return empty.
	if (layerId == 0)
		return true;
	
	// Create layers.
	rcAssert(lset.layers == 0);
	
	const int lw = w - borderSize*2;
	const int lh = h - borderSize*2;

	// Build contracted bbox for layers.
	float bmin[3], bmax[3];
	AABB3 box;
	rcVcopy(bmin, chf.bmin);
	rcVcopy(bmax, chf.bmax);
	bmin[0] += borderSize*chf.cs;
	bmin[2] += borderSize*chf.cs;
	bmax[0] -= borderSize*chf.cs;
	bmax[2] -= borderSize*chf.cs;
	
	lset.nlayers = (int)layerId;
	
	lset.layers = (Moss_RecastHeightfieldLayer*)rcAlloc(sizeof(Moss_RecastHeightfieldLayer)*lset.nlayers, RC_ALLOC_PERM);
	if (!lset.layers)
	{
		MOSS_ERROR("rcBuildHeightfieldLayers: Out of memory 'layers' (%d).", lset.nlayers);
		return false;
	}
	memset(lset.layers, 0, sizeof(Moss_RecastHeightfieldLayer)*lset.nlayers);

	
	// Store layers.
	for (int i = 0; i < lset.nlayers; ++i)
	{
		unsigned char curId = (unsigned char)i;

		Moss_RecastHeightfieldLayer* layer = &lset.layers[i];

		const int gridSize = sizeof(unsigned char)*lw*lh;

		layer->heights = (unsigned char*)rcAlloc(gridSize, RC_ALLOC_PERM);
		if (!layer->heights)
		{
			MOSS_ERROR("rcBuildHeightfieldLayers: Out of memory 'heights' (%d).", gridSize);
			return false;
		}
		memset(layer->heights, 0xff, gridSize);

		layer->areas = (unsigned char*)rcAlloc(gridSize, RC_ALLOC_PERM);
		if (!layer->areas)
		{
			MOSS_ERROR("rcBuildHeightfieldLayers: Out of memory 'areas' (%d).", gridSize);
			return false;
		}
		memset(layer->areas, 0, gridSize);

		layer->cons = (unsigned char*)rcAlloc(gridSize, RC_ALLOC_PERM);
		if (!layer->cons)
		{
			MOSS_ERROR("rcBuildHeightfieldLayers: Out of memory 'cons' (%d).", gridSize);
			return false;
		}
		memset(layer->cons, 0, gridSize);
		
		// Find layer height bounds.
		int hmin = 0, hmax = 0;
		for (int j = 0; j < nregs; ++j)
		{
			if (regs[j].base && regs[j].layerId == curId)
			{
				hmin = (int)regs[j].ymin;
				hmax = (int)regs[j].ymax;
			}
		}

		layer->width = lw;
		layer->height = lh;
		layer->cs = chf.cs;
		layer->ch = chf.ch;
		
		// Adjust the bbox to fit the heightfield.
		rcVcopy(layer->bmin, bmin);
		rcVcopy(layer->bmax, bmax);
		layer->bmin[1] = bmin[1] + hmin*chf.ch;
		layer->bmax[1] = bmin[1] + hmax*chf.ch;
		layer->hmin = hmin;
		layer->hmax = hmax;

		// Update usable data region.
		layer->minx = layer->width;
		layer->maxx = 0;
		layer->miny = layer->height;
		layer->maxy = 0;
		
		// Copy height and area from compact heightfield. 
		for (int y = 0; y < lh; ++y)
		{
			for (int x = 0; x < lw; ++x)
			{
				const int cx = borderSize+x;
				const int cy = borderSize+y;
				const Moss_RecastCompactCell& c = chf.cells[cx+cy*w];
				for (int j = (int)c.index, nj = (int)(c.index+c.count); j < nj; ++j)
				{
					const Moss_RecastCompactSpan& s = chf.spans[j];
					// Skip unassigned regions.
					if (srcReg[j] == 0xff)
						continue;
					// Skip of does nto belong to current layer.
					unsigned char lid = regs[srcReg[j]].layerId;
					if (lid != curId)
						continue;
					
					// Update data bounds.
					layer->minx = min(layer->minx, x);
					layer->maxx = max(layer->maxx, x);
					layer->miny = min(layer->miny, y);
					layer->maxy = max(layer->maxy, y);
					
					// Store height and area type.
					const int idx = x+y*lw;
					layer->heights[idx] = (unsigned char)(s.y - hmin);
					layer->areas[idx] = chf.areas[j];
					
					// Check connection.
					unsigned char portal = 0;
					unsigned char con = 0;
					for (int dir = 0; dir < 4; ++dir)
					{
						if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
						{
							const int ax = cx + rcGetDirOffsetX(dir);
							const int ay = cy + rcGetDirOffsetY(dir);
							const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);
							unsigned char alid = srcReg[ai] != 0xff ? regs[srcReg[ai]].layerId : 0xff;
							// Portal mask
							if (chf.areas[ai] != RC_NULL_AREA && lid != alid)
							{
								portal |= (unsigned char)(1<<dir);
								// Update height so that it matches on both sides of the portal.
								const Moss_RecastCompactSpan& as = chf.spans[ai];
								if (as.y > hmin)
									layer->heights[idx] = max(layer->heights[idx], (unsigned char)(as.y - hmin));
							}
							// Valid connection mask
							if (chf.areas[ai] != RC_NULL_AREA && lid == alid)
							{
								const int nx = ax - borderSize;
								const int ny = ay - borderSize;
								if (nx >= 0 && ny >= 0 && nx < lw && ny < lh)
									con |= (unsigned char)(1<<dir);
							}
						}
					}
					
					layer->cons[idx] = (portal << 4) | con;
				}
			}
		}
		
		if (layer->minx > layer->maxx)
			layer->minx = layer->maxx = 0;
		if (layer->miny > layer->maxy)
			layer->miny = layer->maxy = 0;
	}
	
	return true;
}


/*													*/

/// Sorts the given data in-place using insertion sort.
///
/// @param	data		The data to sort
/// @param	dataLength	The number of elements in @p data
static void insertSort(unsigned char* data, const int dataLength)
{
	for (int valueIndex = 1; valueIndex < dataLength; valueIndex++)
	{
		const unsigned char value = data[valueIndex];
		int insertionIndex;
		for (insertionIndex = valueIndex - 1; insertionIndex >= 0 && data[insertionIndex] > value; insertionIndex--)
		{
			// Shift over values
			data[insertionIndex + 1] = data[insertionIndex];
		}
		
		// Insert the value in sorted order.
		data[insertionIndex + 1] = value;
	}
}

// TODO (graham): This is duplicated in the ConvexVolumeTool in RecastDemo
/// Checks if a point is contained within a polygon
///
/// @param[in]	numVerts	Number of vertices in the polygon
/// @param[in]	verts		The polygon vertices
/// @param[in]	point		The point to check
/// @returns true if the point lies within the polygon, false otherwise.
static bool pointInPoly(int numVerts, const float* verts, const float* point)
{
	bool inPoly = false;
	for (int i = 0, j = numVerts - 1; i < numVerts; j = i++)
	{
		const float* vi = &verts[i * 3];
		const float* vj = &verts[j * 3];

		if ((vi[2] > point[2]) == (vj[2] > point[2]))
		{
			continue;
		}

		if (point[0] >= (vj[0] - vi[0]) * (point[2] - vi[2]) / (vj[2] - vi[2]) + vi[0])
		{
			continue;
		}
		inPoly = !inPoly;
	}
	return inPoly;
}

bool rcErodeWalkableArea(rcContext* context, const int erosionRadius, Moss_RecastCompactHeightfield& compactHeightfield)
{
	rcAssert(context != NULL);

	const int xSize = compactHeightfield.width;
	const int zSize = compactHeightfield.height;
	const int& zStride = xSize; // For readability

	rcScopedTimer timer(context, RC_TIMER_ERODE_AREA);

	unsigned char* distanceToBoundary = (unsigned char*)rcAlloc(sizeof(unsigned char) * compactHeightfield.spanCount,
	                                                            RC_ALLOC_TEMP);
	if (!distanceToBoundary)
	{
		MOSS_ERROR("erodeWalkableArea: Out of memory 'dist' (%d).", compactHeightfield.spanCount);
		return false;
	}
	memset(distanceToBoundary, 0xff, sizeof(unsigned char) * compactHeightfield.spanCount);
	
	// Mark boundary cells.
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			const Moss_RecastCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			for (int spanIndex = (int)cell.index, maxSpanIndex = (int)(cell.index + cell.count); spanIndex < maxSpanIndex; ++spanIndex)
			{
				if (compactHeightfield.areas[spanIndex] == RC_NULL_AREA)
				{
					distanceToBoundary[spanIndex] = 0;
					continue;
				}
				const Moss_RecastCompactSpan& span = compactHeightfield.spans[spanIndex];

				// Check that there is a non-null adjacent span in each of the 4 cardinal directions.
				int neighborCount = 0;
				for (int direction = 0; direction < 4; ++direction)
				{
					const int neighborConnection = rcGetCon(span, direction);
					if (neighborConnection == RC_NOT_CONNECTED)
					{
						break;
					}
					
					const int neighborX = x + rcGetDirOffsetX(direction);
					const int neighborZ = z + rcGetDirOffsetY(direction);
					const int neighborSpanIndex = (int)compactHeightfield.cells[neighborX + neighborZ * zStride].index + neighborConnection;
					
					if (compactHeightfield.areas[neighborSpanIndex] == RC_NULL_AREA)
					{
						break;
					}
					neighborCount++;
				}
				
				// At least one missing neighbour, so this is a boundary cell.
				if (neighborCount != 4)
				{
					distanceToBoundary[spanIndex] = 0;
				}
			}
		}
	}
	
	unsigned char newDistance;
	
	// Pass 1
	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			const Moss_RecastCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			const int maxSpanIndex = (int)(cell.index + cell.count);
			for (int spanIndex = (int)cell.index; spanIndex < maxSpanIndex; ++spanIndex)
			{
				const Moss_RecastCompactSpan& span = compactHeightfield.spans[spanIndex];

				if (rcGetCon(span, 0) != RC_NOT_CONNECTED)
				{
					// (-1,0)
					const int aX = x + rcGetDirOffsetX(0);
					const int aY = z + rcGetDirOffsetY(0);
					const int aIndex = (int)compactHeightfield.cells[aX + aY * xSize].index + rcGetCon(span, 0);
					const Moss_RecastCompactSpan& aSpan = compactHeightfield.spans[aIndex];
					newDistance = (unsigned char)min((int)distanceToBoundary[aIndex] + 2, 255);
					if (newDistance < distanceToBoundary[spanIndex])
					{
						distanceToBoundary[spanIndex] = newDistance;
					}

					// (-1,-1)
					if (rcGetCon(aSpan, 3) != RC_NOT_CONNECTED)
					{
						const int bX = aX + rcGetDirOffsetX(3);
						const int bY = aY + rcGetDirOffsetY(3);
						const int bIndex = (int)compactHeightfield.cells[bX + bY * xSize].index + rcGetCon(aSpan, 3);
						newDistance = (unsigned char)min((int)distanceToBoundary[bIndex] + 3, 255);
						if (newDistance < distanceToBoundary[spanIndex])
						{
							distanceToBoundary[spanIndex] = newDistance;
						}
					}
				}
				if (rcGetCon(span, 3) != RC_NOT_CONNECTED)
				{
					// (0,-1)
					const int aX = x + rcGetDirOffsetX(3);
					const int aY = z + rcGetDirOffsetY(3);
					const int aIndex = (int)compactHeightfield.cells[aX + aY * xSize].index + rcGetCon(span, 3);
					const Moss_RecastCompactSpan& aSpan = compactHeightfield.spans[aIndex];
					newDistance = (unsigned char)min((int)distanceToBoundary[aIndex] + 2, 255);
					if (newDistance < distanceToBoundary[spanIndex])
					{
						distanceToBoundary[spanIndex] = newDistance;
					}

					// (1,-1)
					if (rcGetCon(aSpan, 2) != RC_NOT_CONNECTED)
					{
						const int bX = aX + rcGetDirOffsetX(2);
						const int bY = aY + rcGetDirOffsetY(2);
						const int bIndex = (int)compactHeightfield.cells[bX + bY * xSize].index + rcGetCon(aSpan, 2);
						newDistance = (unsigned char)min((int)distanceToBoundary[bIndex] + 3, 255);
						if (newDistance < distanceToBoundary[spanIndex])
						{
							distanceToBoundary[spanIndex] = newDistance;
						}
					}
				}
			}
		}
	}

	// Pass 2
	for (int z = zSize - 1; z >= 0; --z)
	{
		for (int x = xSize - 1; x >= 0; --x)
		{
			const Moss_RecastCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			const int maxSpanIndex = (int)(cell.index + cell.count);
			for (int spanIndex = (int)cell.index; spanIndex < maxSpanIndex; ++spanIndex)
			{
				const Moss_RecastCompactSpan& span = compactHeightfield.spans[spanIndex];

				if (rcGetCon(span, 2) != RC_NOT_CONNECTED)
				{
					// (1,0)
					const int aX = x + rcGetDirOffsetX(2);
					const int aY = z + rcGetDirOffsetY(2);
					const int aIndex = (int)compactHeightfield.cells[aX + aY * xSize].index + rcGetCon(span, 2);
					const Moss_RecastCompactSpan& aSpan = compactHeightfield.spans[aIndex];
					newDistance = (unsigned char)min((int)distanceToBoundary[aIndex] + 2, 255);
					if (newDistance < distanceToBoundary[spanIndex])
					{
						distanceToBoundary[spanIndex] = newDistance;
					}

					// (1,1)
					if (rcGetCon(aSpan, 1) != RC_NOT_CONNECTED)
					{
						const int bX = aX + rcGetDirOffsetX(1);
						const int bY = aY + rcGetDirOffsetY(1);
						const int bIndex = (int)compactHeightfield.cells[bX + bY * xSize].index + rcGetCon(aSpan, 1);
						newDistance = (unsigned char)min((int)distanceToBoundary[bIndex] + 3, 255);
						if (newDistance < distanceToBoundary[spanIndex])
						{
							distanceToBoundary[spanIndex] = newDistance;
						}
					}
				}
				if (rcGetCon(span, 1) != RC_NOT_CONNECTED)
				{
					// (0,1)
					const int aX = x + rcGetDirOffsetX(1);
					const int aY = z + rcGetDirOffsetY(1);
					const int aIndex = (int)compactHeightfield.cells[aX + aY * xSize].index + rcGetCon(span, 1);
					const Moss_RecastCompactSpan& aSpan = compactHeightfield.spans[aIndex];
					newDistance = (unsigned char)min((int)distanceToBoundary[aIndex] + 2, 255);
					if (newDistance < distanceToBoundary[spanIndex])
					{
						distanceToBoundary[spanIndex] = newDistance;
					}

					// (-1,1)
					if (rcGetCon(aSpan, 0) != RC_NOT_CONNECTED)
					{
						const int bX = aX + rcGetDirOffsetX(0);
						const int bY = aY + rcGetDirOffsetY(0);
						const int bIndex = (int)compactHeightfield.cells[bX + bY * xSize].index + rcGetCon(aSpan, 0);
						newDistance = (unsigned char)min((int)distanceToBoundary[bIndex] + 3, 255);
						if (newDistance < distanceToBoundary[spanIndex])
						{
							distanceToBoundary[spanIndex] = newDistance;
						}
					}
				}
			}
		}
	}

	const unsigned char minBoundaryDistance = (unsigned char)(erosionRadius * 2);
	for (int spanIndex = 0; spanIndex < compactHeightfield.spanCount; ++spanIndex)
	{
		if (distanceToBoundary[spanIndex] < minBoundaryDistance)
		{
			compactHeightfield.areas[spanIndex] = RC_NULL_AREA;
		}
	}

	MOSS_FREE(distanceToBoundary);
	
	return true;
}

bool rcMedianFilterWalkableArea(rcContext* context, Moss_RecastCompactHeightfield& compactHeightfield)
{
	rcAssert(context);
	
	const int xSize = compactHeightfield.width;
	const int zSize = compactHeightfield.height;
	const int zStride = xSize; // For readability

	rcScopedTimer timer(context, RC_TIMER_MEDIAN_AREA);

	unsigned char* areas = (unsigned char*)rcAlloc(sizeof(unsigned char) * compactHeightfield.spanCount, RC_ALLOC_TEMP);
	if (!areas) {
		MOSS_ERROR("medianFilterWalkableArea: Out of memory 'areas' (%d).", compactHeightfield.spanCount);
		return false;
	}
	memset(areas, 0xff, sizeof(unsigned char) * compactHeightfield.spanCount);

	for (int z = 0; z < zSize; ++z)
	{
		for (int x = 0; x < xSize; ++x)
		{
			const Moss_RecastCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			const int maxSpanIndex = (int)(cell.index + cell.count);
			for (int spanIndex = (int)cell.index; spanIndex < maxSpanIndex; ++spanIndex)
			{
				const Moss_RecastCompactSpan& span = compactHeightfield.spans[spanIndex];
				if (compactHeightfield.areas[spanIndex] == RC_NULL_AREA)
				{
					areas[spanIndex] = compactHeightfield.areas[spanIndex];
					continue;
				}

				unsigned char neighborAreas[9];
				for (int neighborIndex = 0; neighborIndex < 9; ++neighborIndex)
				{
					neighborAreas[neighborIndex] = compactHeightfield.areas[spanIndex];
				}

				for (int dir = 0; dir < 4; ++dir)
				{
					if (rcGetCon(span, dir) == RC_NOT_CONNECTED)
					{
						continue;
					}
					
					const int aX = x + rcGetDirOffsetX(dir);
					const int aZ = z + rcGetDirOffsetY(dir);
					const int aIndex = (int)compactHeightfield.cells[aX + aZ * zStride].index + rcGetCon(span, dir);
					if (compactHeightfield.areas[aIndex] != RC_NULL_AREA)
					{
						neighborAreas[dir * 2 + 0] = compactHeightfield.areas[aIndex];
					}

					const Moss_RecastCompactSpan& aSpan = compactHeightfield.spans[aIndex];
					const int dir2 = (dir + 1) & 0x3;
					const int neighborConnection2 = rcGetCon(aSpan, dir2);
					if (neighborConnection2 != RC_NOT_CONNECTED)
					{
						const int bX = aX + rcGetDirOffsetX(dir2);
						const int bZ = aZ + rcGetDirOffsetY(dir2);
						const int bIndex = (int)compactHeightfield.cells[bX + bZ * zStride].index + neighborConnection2;
						if (compactHeightfield.areas[bIndex] != RC_NULL_AREA)
						{
							neighborAreas[dir * 2 + 1] = compactHeightfield.areas[bIndex];
						}
					}
				}
				insertSort(neighborAreas, 9);
				areas[spanIndex] = neighborAreas[4];
			}
		}
	}

	memcpy(compactHeightfield.areas, areas, sizeof(unsigned char) * compactHeightfield.spanCount);

	Moss_Free(areas);

	return true;
}

void rcMarkBoxArea(rcContext* context, const float* boxMinBounds, const float* boxMaxBounds, unsigned char areaId, Moss_RecastCompactHeightfield& compactHeightfield) {
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_MARK_BOX_AREA);

	const int xSize = compactHeightfield.width;
	const int zSize = compactHeightfield.height;
	const int zStride = xSize; // For readability

	// Find the footprint of the box area in grid cell coordinates. 
	int minX = (int)((boxMinBounds[0] - compactHeightfield.bmin[0]) / compactHeightfield.cs);
	int minY = (int)((boxMinBounds[1] - compactHeightfield.bmin[1]) / compactHeightfield.ch);
	int minZ = (int)((boxMinBounds[2] - compactHeightfield.bmin[2]) / compactHeightfield.cs);
	int maxX = (int)((boxMaxBounds[0] - compactHeightfield.bmin[0]) / compactHeightfield.cs);
	int maxY = (int)((boxMaxBounds[1] - compactHeightfield.bmin[1]) / compactHeightfield.ch);
	int maxZ = (int)((boxMaxBounds[2] - compactHeightfield.bmin[2]) / compactHeightfield.cs);

	// Early-out if the box is outside the bounds of the grid.
	if (maxX < 0) { return; }
	if (minX >= xSize) { return; }
	if (maxZ < 0) { return; }
	if (minZ >= zSize) { return; }

	// Clamp relevant bound coordinates to the grid.
	if (minX < 0) { minX = 0; }
	if (maxX >= xSize) { maxX = xSize - 1; }
	if (minZ < 0) { minZ = 0; }
	if (maxZ >= zSize) { maxZ = zSize - 1; }

	// Mark relevant cells.
	for (int z = minZ; z <= maxZ; ++z)
	{
		for (int x = minX; x <= maxX; ++x)
		{
			const Moss_RecastCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			const int maxSpanIndex = (int)(cell.index + cell.count);
			for (int spanIndex = (int)cell.index; spanIndex < maxSpanIndex; ++spanIndex)
			{
				Moss_RecastCompactSpan& span = compactHeightfield.spans[spanIndex];

				// Skip if the span is outside the box extents.
				if ((int)span.y < minY || (int)span.y > maxY)
				{
					continue;
				}

				// Skip if the span has been removed.
				if (compactHeightfield.areas[spanIndex] == RC_NULL_AREA)
				{
					continue;
				}

				// Mark the span.
				compactHeightfield.areas[spanIndex] = areaId;
			}
		}
	}
}

void rcMarkConvexPolyArea(rcContext* context, const float* verts, const int numVerts,
						  const float minY, const float maxY, unsigned char areaId,
						  Moss_RecastCompactHeightfield& compactHeightfield)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_MARK_CONVEXPOLY_AREA);

	const int xSize = compactHeightfield.width;
	const int zSize = compactHeightfield.height;
	const int zStride = xSize; // For readability

	// Compute the bounding box of the polygon
	float bmin[3];
	float bmax[3];
	rcVcopy(bmin, verts);
	rcVcopy(bmax, verts);
	for (int i = 1; i < numVerts; ++i)
	{
		rcVmin(bmin, &verts[i * 3]);
		rcVmax(bmax, &verts[i * 3]);
	}
	bmin[1] = minY;
	bmax[1] = maxY;

	// Compute the grid footprint of the polygon 
	int minx = (int)((bmin[0] - compactHeightfield.bmin[0]) / compactHeightfield.cs);
	int miny = (int)((bmin[1] - compactHeightfield.bmin[1]) / compactHeightfield.ch);
	int minz = (int)((bmin[2] - compactHeightfield.bmin[2]) / compactHeightfield.cs);
	int maxx = (int)((bmax[0] - compactHeightfield.bmin[0]) / compactHeightfield.cs);
	int maxy = (int)((bmax[1] - compactHeightfield.bmin[1]) / compactHeightfield.ch);
	int maxz = (int)((bmax[2] - compactHeightfield.bmin[2]) / compactHeightfield.cs);

	// Early-out if the polygon lies entirely outside the grid.
	if (maxx < 0) { return; }
    if (minx >= xSize) { return; }
    if (maxz < 0) { return; }
    if (minz >= zSize) { return; }

	// Clamp the polygon footprint to the grid
    if (minx < 0) { minx = 0; }
    if (maxx >= xSize) { maxx = xSize - 1; }
    if (minz < 0) { minz = 0; }
    if (maxz >= zSize) { maxz = zSize - 1; }

	// TODO: Optimize.
	for (int z = minz; z <= maxz; ++z)
	{
		for (int x = minx; x <= maxx; ++x)
		{
			const Moss_RecastCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			const int maxSpanIndex = (int)(cell.index + cell.count);
			for (int spanIndex = (int)cell.index; spanIndex < maxSpanIndex; ++spanIndex)
			{
				Moss_RecastCompactSpan& span = compactHeightfield.spans[spanIndex];

				// Skip if span is removed.
				if (compactHeightfield.areas[spanIndex] == RC_NULL_AREA)
				{
					continue;
				}

				// Skip if y extents don't overlap.
				if ((int)span.y < miny || (int)span.y > maxy)
				{
					continue;
				}

				const float point[] = {
					compactHeightfield.bmin[0] + ((float)x + 0.5f) * compactHeightfield.cs,
					0,
					compactHeightfield.bmin[2] + ((float)z + 0.5f) * compactHeightfield.cs
				};
				
				if (pointInPoly(numVerts, verts, point))
				{
					compactHeightfield.areas[spanIndex] = areaId;
				}
			}
		}
	}
}

static const float EPSILON = 1e-6f;

/// Normalizes the vector if the length is greater than zero.
/// If the magnitude is zero, the vector is unchanged.
/// @param[in,out]	v	The vector to normalize. [(x, y, z)]
static void rcVsafeNormalize(float* v)
{
	const float sqMag = Sqr(v[0]) + Sqr(v[1]) + Sqr(v[2]);
	if (sqMag > EPSILON)
	{
		const float inverseMag = 1.0f / sqrtf(sqMag);
		v[0] *= inverseMag;
		v[1] *= inverseMag;
		v[2] *= inverseMag;
	}
}

int rcOffsetPoly(const float* verts, const int numVerts, const float offset, float* outVerts, const int maxOutVerts)
{
	// Defines the limit at which a miter becomes a bevel.
	// Similar in behavior to https://developer.mozilla.org/en-US/docs/Web/SVG/Attribute/stroke-miterlimit
	const float MITER_LIMIT = 1.20f;

	int numOutVerts = 0;

	for (int vertIndex = 0; vertIndex < numVerts; vertIndex++)
	{
        // Grab three vertices of the polygon.
		const int vertIndexA = (vertIndex + numVerts - 1) % numVerts;
		const int vertIndexB = vertIndex;
		const int vertIndexC = (vertIndex + 1) % numVerts;
		const float* vertA = &verts[vertIndexA * 3];
		const float* vertB = &verts[vertIndexB * 3];
		const float* vertC = &verts[vertIndexC * 3];

        // From A to B on the x/z plane
		float prevSegmentDir[3];
		dtVsub(prevSegmentDir, vertB, vertA);
		prevSegmentDir[1] = 0; // Squash onto x/z plane
		rcVsafeNormalize(prevSegmentDir);
		
        // From B to C on the x/z plane
		float currSegmentDir[3];
		dtVsub(currSegmentDir, vertC, vertB);
		currSegmentDir[1] = 0; // Squash onto x/z plane
		rcVsafeNormalize(currSegmentDir);

        // The y component of the cross product of the two normalized segment directions.
        // The X and Z components of the cross product are both zero because the two
        // segment direction vectors fall within the x/z plane.
        float cross = currSegmentDir[0] * prevSegmentDir[2] - prevSegmentDir[0] * currSegmentDir[2];

        // CCW perpendicular vector to AB.  The segment normal.
		const float prevSegmentNormX = -prevSegmentDir[2];
		const float prevSegmentNormZ = prevSegmentDir[0];

        // CCW perpendicular vector to BC.  The segment normal.
		const float currSegmentNormX = -currSegmentDir[2];
		const float currSegmentNormZ = currSegmentDir[0];

        // Average the two segment normals to get the proportional miter offset for B.
        // This isn't normalized because it's defining the distance and direction the corner will need to be
        // adjusted proportionally to the edge offsets to properly miter the adjoining edges.
		float cornerMiterX = (prevSegmentNormX + currSegmentNormX) * 0.5f;
		float cornerMiterZ = (prevSegmentNormZ + currSegmentNormZ) * 0.5f;
        const float cornerMiterSqMag = Sqr(cornerMiterX) + Sqr(cornerMiterZ);

        // If the magnitude of the segment normal average is less than about .69444,
        // the corner is an acute enough angle that the result should be beveled.
        const bool bevel = cornerMiterSqMag * MITER_LIMIT * MITER_LIMIT < 1.0f;

        // Scale the corner miter so it's proportional to how much the corner should be offset compared to the edges.
		if (cornerMiterSqMag > EPSILON)
		{
			const float scale = 1.0f / cornerMiterSqMag;
            cornerMiterX *= scale;
            cornerMiterZ *= scale;
		}

		if (bevel && cross < 0.0f) // If the corner is convex and an acute enough angle, generate a bevel.
		{
			if (numOutVerts + 2 > maxOutVerts)
			{
				return 0;
			}

            // Generate two bevel vertices at a distances from B proportional to the angle between the two segments.
            // Move each bevel vertex out proportional to the given offset.
			float d = (1.0f - (prevSegmentDir[0] * currSegmentDir[0] + prevSegmentDir[2] * currSegmentDir[2])) * 0.5f;

			outVerts[numOutVerts * 3 + 0] = vertB[0] + (-prevSegmentNormX + prevSegmentDir[0] * d) * offset;
			outVerts[numOutVerts * 3 + 1] = vertB[1];
			outVerts[numOutVerts * 3 + 2] = vertB[2] + (-prevSegmentNormZ + prevSegmentDir[2] * d) * offset;
			numOutVerts++;

			outVerts[numOutVerts * 3 + 0] = vertB[0] + (-currSegmentNormX - currSegmentDir[0] * d) * offset;
			outVerts[numOutVerts * 3 + 1] = vertB[1];
			outVerts[numOutVerts * 3 + 2] = vertB[2] + (-currSegmentNormZ - currSegmentDir[2] * d) * offset;
			numOutVerts++;
		}
		else
		{
			if (numOutVerts + 1 > maxOutVerts)
			{
				return 0;
			}

            // Move B along the miter direction by the specified offset.
			outVerts[numOutVerts * 3 + 0] = vertB[0] - cornerMiterX * offset;
			outVerts[numOutVerts * 3 + 1] = vertB[1];
			outVerts[numOutVerts * 3 + 2] = vertB[2] - cornerMiterZ * offset;
			numOutVerts++;
		}
	}

	return numOutVerts;
}

void rcMarkCylinderArea(rcContext* context, const float* position, const float radius, const float height,
                        unsigned char areaId, Moss_RecastCompactHeightfield& compactHeightfield)
{
	rcAssert(context);

	rcScopedTimer timer(context, RC_TIMER_MARK_CYLINDER_AREA);

	const int xSize = compactHeightfield.width;
	const int zSize = compactHeightfield.height;
	const int zStride = xSize; // For readability

	// Compute the bounding box of the cylinder
	const float cylinderBBMin[] = {
		position[0] - radius,
		position[1],
		position[2] - radius
	};
	const float cylinderBBMax[] = {
		position[0] + radius,
		position[1] + height,
		position[2] + radius
	};

	// Compute the grid footprint of the cylinder
	int minx = (int)((cylinderBBMin[0] - compactHeightfield.bmin[0]) / compactHeightfield.cs);
	int miny = (int)((cylinderBBMin[1] - compactHeightfield.bmin[1]) / compactHeightfield.ch);
	int minz = (int)((cylinderBBMin[2] - compactHeightfield.bmin[2]) / compactHeightfield.cs);
	int maxx = (int)((cylinderBBMax[0] - compactHeightfield.bmin[0]) / compactHeightfield.cs);
	int maxy = (int)((cylinderBBMax[1] - compactHeightfield.bmin[1]) / compactHeightfield.ch);
	int maxz = (int)((cylinderBBMax[2] - compactHeightfield.bmin[2]) / compactHeightfield.cs);

	// Early-out if the cylinder is completely outside the grid bounds.
    if (maxx < 0) { return; }
    if (minx >= xSize) { return; }
    if (maxz < 0) { return; }
    if (minz >= zSize) { return; }

	// Clamp the cylinder bounds to the grid.
    if (minx < 0) { minx = 0; }
    if (maxx >= xSize) { maxx = xSize - 1; }
    if (minz < 0) { minz = 0; }
    if (maxz >= zSize) { maxz = zSize - 1; }

	const float radiusSq = radius * radius;

	for (int z = minz; z <= maxz; ++z) {
		for (int x = minx; x <= maxx; ++x) {
			const Moss_RecastCompactCell& cell = compactHeightfield.cells[x + z * zStride];
			const int maxSpanIndex = (int)(cell.index + cell.count);

			const float cellX = compactHeightfield.bmin[0] + ((float)x + 0.5f) * compactHeightfield.cs;
			const float cellZ = compactHeightfield.bmin[2] + ((float)z + 0.5f) * compactHeightfield.cs;
			const float deltaX = cellX - position[0];
            const float deltaZ = cellZ - position[2];

			// Skip this column if it's too far from the center point of the cylinder.
            if (Sqr(deltaX) + Sqr(deltaZ) >= radiusSq) { continue; }

			// Mark all overlapping spans
			for (int spanIndex = (int)cell.index; spanIndex < maxSpanIndex; ++spanIndex) {
				Moss_RecastCompactSpan& span = compactHeightfield.spans[spanIndex];

				// Skip if span is removed.
				if (compactHeightfield.areas[spanIndex] == RC_NULL_AREA) { continue; }

				// Mark if y extents overlap.
				if ((int)span.y >= miny && (int)span.y <= maxy) { compactHeightfield.areas[spanIndex] = areaId; }
			}
		}
	}
}

/*													*/


static int getCornerHeight(int x, int y, int i, int dir, const Moss_RecastCompactHeightfield& chf, bool& isBorderVertex) {
	const Moss_RecastCompactSpan& s = chf.spans[i];
	int ch = (int)s.y;
	int dirp = (dir+1) & 0x3;
	
	unsigned int regs[4] = {0,0,0,0};
	
	// Combine region and area codes in order to prevent
	// border vertices which are in between two areas to be removed.
	regs[0] = chf.spans[i].reg | (chf.areas[i] << 16);
	
	if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
	{
		const int ax = x + rcGetDirOffsetX(dir);
		const int ay = y + rcGetDirOffsetY(dir);
		const int ai = (int)chf.cells[ax+ay*chf.width].index + rcGetCon(s, dir);
		const Moss_RecastCompactSpan& as = chf.spans[ai];
		ch = max(ch, (int)as.y);
		regs[1] = chf.spans[ai].reg | (chf.areas[ai] << 16);
		if (rcGetCon(as, dirp) != RC_NOT_CONNECTED)
		{
			const int ax2 = ax + rcGetDirOffsetX(dirp);
			const int ay2 = ay + rcGetDirOffsetY(dirp);
			const int ai2 = (int)chf.cells[ax2+ay2*chf.width].index + rcGetCon(as, dirp);
			const Moss_RecastCompactSpan& as2 = chf.spans[ai2];
			ch = max(ch, (int)as2.y);
			regs[2] = chf.spans[ai2].reg | (chf.areas[ai2] << 16);
		}
	}
	if (rcGetCon(s, dirp) != RC_NOT_CONNECTED)
	{
		const int ax = x + rcGetDirOffsetX(dirp);
		const int ay = y + rcGetDirOffsetY(dirp);
		const int ai = (int)chf.cells[ax+ay*chf.width].index + rcGetCon(s, dirp);
		const Moss_RecastCompactSpan& as = chf.spans[ai];
		ch = max(ch, (int)as.y);
		regs[3] = chf.spans[ai].reg | (chf.areas[ai] << 16);
		if (rcGetCon(as, dir) != RC_NOT_CONNECTED)
		{
			const int ax2 = ax + rcGetDirOffsetX(dir);
			const int ay2 = ay + rcGetDirOffsetY(dir);
			const int ai2 = (int)chf.cells[ax2+ay2*chf.width].index + rcGetCon(as, dir);
			const Moss_RecastCompactSpan& as2 = chf.spans[ai2];
			ch = max(ch, (int)as2.y);
			regs[2] = chf.spans[ai2].reg | (chf.areas[ai2] << 16);
		}
	}

	// Check if the vertex is special edge vertex, these vertices will be removed later.
	for (int j = 0; j < 4; ++j)
	{
		const int a = j;
		const int b = (j+1) & 0x3;
		const int c = (j+2) & 0x3;
		const int d = (j+3) & 0x3;
		
		// The vertex is a border vertex there are two same exterior cells in a row,
		// followed by two interior cells and none of the regions are out of bounds.
		const bool twoSameExts = (regs[a] & regs[b] & RC_BORDER_REG) != 0 && regs[a] == regs[b];
		const bool twoInts = ((regs[c] | regs[d]) & RC_BORDER_REG) == 0;
		const bool intsSameArea = (regs[c]>>16) == (regs[d]>>16);
		const bool noZeros = regs[a] != 0 && regs[b] != 0 && regs[c] != 0 && regs[d] != 0;
		if (twoSameExts && twoInts && intsSameArea && noZeros)
		{
			isBorderVertex = true;
			break;
		}
	}
	
	return ch;
}

static void walkContour(int x, int y, int i,
						const Moss_RecastCompactHeightfield& chf,
						unsigned char* flags, rcTempVector<int>& points)
{
	// Choose the first non-connected edge
	unsigned char dir = 0;
	while ((flags[i] & (1 << dir)) == 0)
		dir++;
	
	unsigned char startDir = dir;
	int starti = i;
	
	const unsigned char area = chf.areas[i];
	
	int iter = 0;
	while (++iter < 40000)
	{
		if (flags[i] & (1 << dir))
		{
			// Choose the edge corner
			bool isBorderVertex = false;
			bool isAreaBorder = false;
			int px = x;
			int py = getCornerHeight(x, y, i, dir, chf, isBorderVertex);
			int pz = y;
			switch(dir)
			{
				case 0: pz++; break;
				case 1: px++; pz++; break;
				case 2: px++; break;
			}
			int r = 0;
			const Moss_RecastCompactSpan& s = chf.spans[i];
			if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
			{
				const int ax = x + rcGetDirOffsetX(dir);
				const int ay = y + rcGetDirOffsetY(dir);
				const int ai = (int)chf.cells[ax+ay*chf.width].index + rcGetCon(s, dir);
				r = (int)chf.spans[ai].reg;
				if (area != chf.areas[ai])
					isAreaBorder = true;
			}
			if (isBorderVertex)
				r |= RC_BORDER_VERTEX;
			if (isAreaBorder)
				r |= RC_AREA_BORDER;
			points.push_back(px);
			points.push_back(py);
			points.push_back(pz);
			points.push_back(r);
			
			flags[i] &= ~(1 << dir); // Remove visited edges
			dir = (dir+1) & 0x3;  // Rotate CW
		}
		else
		{
			int ni = -1;
			const int nx = x + rcGetDirOffsetX(dir);
			const int ny = y + rcGetDirOffsetY(dir);
			const Moss_RecastCompactSpan& s = chf.spans[i];
			if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
			{
				const Moss_RecastCompactCell& nc = chf.cells[nx+ny*chf.width];
				ni = (int)nc.index + rcGetCon(s, dir);
			}
			if (ni == -1)
			{
				// Should not happen.
				return;
			}
			x = nx;
			y = ny;
			i = ni;
			dir = (dir+3) & 0x3;	// Rotate CCW
		}
		
		if (starti == i && startDir == dir)
		{
			break;
		}
	}
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

static void simplifyContour(rcTempVector<int>& points, rcTempVector<int>& simplified,
							const float maxError, const int maxEdgeLen, const int buildFlags)
{
	// Add initial points.
	bool hasConnections = false;
	for (int i = 0; i < points.size(); i += 4)
	{
		if ((points[i+3] & RC_CONTOUR_REG_MASK) != 0)
		{
			hasConnections = true;
			break;
		}
	}
	
	if (hasConnections)
	{
		// The contour has some portals to other regions.
		// Add a new point to every location where the region changes.
		for (int i = 0, ni = static_cast<int>(points.size()) / 4; i < ni; ++i)
		{
			int ii = (i+1) % ni;
			const bool differentRegs = (points[i*4+3] & RC_CONTOUR_REG_MASK) != (points[ii*4+3] & RC_CONTOUR_REG_MASK);
			const bool areaBorders = (points[i*4+3] & RC_AREA_BORDER) != (points[ii*4+3] & RC_AREA_BORDER);
			if (differentRegs || areaBorders)
			{
				simplified.push_back(points[i*4+0]);
				simplified.push_back(points[i*4+1]);
				simplified.push_back(points[i*4+2]);
				simplified.push_back(i);
			}
		}
	}
	
	if (simplified.size() == 0)
	{
		// If there is no connections at all,
		// create some initial points for the simplification process.
		// Find lower-left and upper-right vertices of the contour.
		int llx = points[0];
		int lly = points[1];
		int llz = points[2];
		int lli = 0;
		int urx = points[0];
		int ury = points[1];
		int urz = points[2];
		int uri = 0;
		for (int i = 0; i < points.size(); i += 4)
		{
			int x = points[i+0];
			int y = points[i+1];
			int z = points[i+2];
			if (x < llx || (x == llx && z < llz))
			{
				llx = x;
				lly = y;
				llz = z;
				lli = i/4;
			}
			if (x > urx || (x == urx && z > urz))
			{
				urx = x;
				ury = y;
				urz = z;
				uri = i/4;
			}
		}
		simplified.push_back(llx);
		simplified.push_back(lly);
		simplified.push_back(llz);
		simplified.push_back(lli);
		
		simplified.push_back(urx);
		simplified.push_back(ury);
		simplified.push_back(urz);
		simplified.push_back(uri);
	}
	
	// Add points until all raw points are within
	// error tolerance to the simplified shape.
	const int pn = static_cast<int>(points.size()) / 4;
	for (int i = 0; i < simplified.size()/4; )
	{
		int ii = (i+1) % (simplified.size()/4);
		
		int ax = simplified[i*4+0];
		int az = simplified[i*4+2];
		int ai = simplified[i*4+3];

		int bx = simplified[ii*4+0];
		int bz = simplified[ii*4+2];
		int bi = simplified[ii*4+3];

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
			ci = (ai+cinc) % pn;
			endi = bi;
		}
		else
		{
			cinc = pn-1;
			ci = (bi+cinc) % pn;
			endi = ai;
			Swap(ax, bx);
			Swap(az, bz);
		}
		
		// Tessellate only outer edges or edges between areas.
		if ((points[ci*4+3] & RC_CONTOUR_REG_MASK) == 0 ||
			(points[ci*4+3] & RC_AREA_BORDER))
		{
			while (ci != endi)
			{
				float d = distancePtSeg(points[ci*4+0], points[ci*4+2], ax, az, bx, bz);
				if (d > maxd)
				{
					maxd = d;
					maxi = ci;
				}
				ci = (ci+cinc) % pn;
			}
		}
		
		
		// If the max deviation is larger than accepted error,
		// add new point, else continue to next segment.
		if (maxi != -1 && maxd > (maxError*maxError))
		{
			// Add space for the new point.
			simplified.resize(simplified.size()+4);
			const int n = static_cast<int>(simplified.size()) / 4;
			for (int j = n-1; j > i; --j)
			{
				simplified[j*4+0] = simplified[(j-1)*4+0];
				simplified[j*4+1] = simplified[(j-1)*4+1];
				simplified[j*4+2] = simplified[(j-1)*4+2];
				simplified[j*4+3] = simplified[(j-1)*4+3];
			}
			// Add the point.
			simplified[(i+1)*4+0] = points[maxi*4+0];
			simplified[(i+1)*4+1] = points[maxi*4+1];
			simplified[(i+1)*4+2] = points[maxi*4+2];
			simplified[(i+1)*4+3] = maxi;
		}
		else
		{
			++i;
		}
	}
	
	// Split too long edges.
	if (maxEdgeLen > 0 && (buildFlags & (RC_CONTOUR_TESS_WALL_EDGES|RC_CONTOUR_TESS_AREA_EDGES)) != 0)
	{
		for (int i = 0; i < simplified.size()/4; )
		{
			const int ii = (i+1) % (simplified.size()/4);
			
			const int ax = simplified[i*4+0];
			const int az = simplified[i*4+2];
			const int ai = simplified[i*4+3];
			
			const int bx = simplified[ii*4+0];
			const int bz = simplified[ii*4+2];
			const int bi = simplified[ii*4+3];
			
			// Find maximum deviation from the segment.
			int maxi = -1;
			int ci = (ai+1) % pn;
			
			// Tessellate only outer edges or edges between areas.
			bool tess = false;
			// Wall edges.
			if ((buildFlags & RC_CONTOUR_TESS_WALL_EDGES) && (points[ci*4+3] & RC_CONTOUR_REG_MASK) == 0)
				tess = true;
			// Edges between areas.
			if ((buildFlags & RC_CONTOUR_TESS_AREA_EDGES) && (points[ci*4+3] & RC_AREA_BORDER))
				tess = true;
			
			if (tess)
			{
				int dx = bx - ax;
				int dz = bz - az;
				if (dx*dx + dz*dz > maxEdgeLen*maxEdgeLen)
				{
					// Round based on the segments in lexilogical order so that the
					// max tesselation is consistent regardless in which direction
					// segments are traversed.
					const int n = bi < ai ? (bi+pn - ai) : (bi - ai);
					if (n > 1)
					{
						if (bx > ax || (bx == ax && bz > az))
							maxi = (ai + n/2) % pn;
						else
							maxi = (ai + (n+1)/2) % pn;
					}
				}
			}
			
			// If the max deviation is larger than accepted error,
			// add new point, else continue to next segment.
			if (maxi != -1)
			{
				// Add space for the new point.
				simplified.resize(simplified.size()+4);
				const int n = static_cast<int>(simplified.size()) / 4;
				for (int j = n-1; j > i; --j)
				{
					simplified[j*4+0] = simplified[(j-1)*4+0];
					simplified[j*4+1] = simplified[(j-1)*4+1];
					simplified[j*4+2] = simplified[(j-1)*4+2];
					simplified[j*4+3] = simplified[(j-1)*4+3];
				}
				// Add the point.
				simplified[(i+1)*4+0] = points[maxi*4+0];
				simplified[(i+1)*4+1] = points[maxi*4+1];
				simplified[(i+1)*4+2] = points[maxi*4+2];
				simplified[(i+1)*4+3] = maxi;
			}
			else
			{
				++i;
			}
		}
	}
	
	for (int i = 0; i < simplified.size()/4; ++i)
	{
		// The edge vertex flag is take from the current raw point,
		// and the neighbour region is take from the next raw point.
		const int ai = (simplified[i*4+3]+1) % pn;
		const int bi = simplified[i*4+3];
		simplified[i*4+3] = (points[ai*4+3] & (RC_CONTOUR_REG_MASK|RC_AREA_BORDER)) | (points[bi*4+3] & RC_BORDER_VERTEX);
	}
	
}

static int calcAreaOfPolygon2D(const int* verts, const int nverts)
{
	int area = 0;
	for (int i = 0, j = nverts-1; i < nverts; j=i++)
	{
		const int* vi = &verts[i*4];
		const int* vj = &verts[j*4];
		area += vi[0] * vj[2] - vj[0] * vi[2];
	}
	return (area+1) / 2;
}

// TODO: these are the same as in RecastMesh.cpp, consider using the same.
// Last time I checked the if version got compiled using cmov, which was a lot faster than module (with idiv).
inline int prev(int i, int n) { return i-1 >= 0 ? i-1 : n-1; }
inline int next(int i, int n) { return i+1 < n ? i+1 : 0; }

inline int area2(const int* a, const int* b, const int* c)
{
	return (b[0] - a[0]) * (c[2] - a[2]) - (c[0] - a[0]) * (b[2] - a[2]);
}

// Returns true iff c is strictly to the left of the directed
// line through a to b.
inline bool left(const int* a, const int* b, const int* c)
{
	return area2(a, b, c) < 0;
}

inline bool leftOn(const int* a, const int* b, const int* c)
{
	return area2(a, b, c) <= 0;
}

inline bool collinear(const int* a, const int* b, const int* c)
{
	return area2(a, b, c) == 0;
}

//	Returns true iff ab properly intersects cd: they share
//	a point interior to both segments.  The properness of the
//	intersection is ensured by using strict leftness.
static bool intersectProp(const int* a, const int* b, const int* c, const int* d)
{
	// Eliminate improper cases.
	if (collinear(a,b,c) || collinear(a,b,d) ||
		collinear(c,d,a) || collinear(c,d,b))
		return false;
	
	return (left(a,b,c) ^ left(a,b,d)) && (left(c,d,a) ^ left(c,d,b));
}

// Returns T iff (a,b,c) are collinear and point c lies
// on the closed segment ab.
static bool between(const int* a, const int* b, const int* c)
{
	if (!collinear(a, b, c))
		return false;
	// If ab not vertical, check betweenness on x; else on y.
	if (a[0] != b[0])
		return	((a[0] <= c[0]) && (c[0] <= b[0])) || ((a[0] >= c[0]) && (c[0] >= b[0]));
	else
		return	((a[2] <= c[2]) && (c[2] <= b[2])) || ((a[2] >= c[2]) && (c[2] >= b[2]));
}

// Returns true iff segments ab and cd intersect, properly or improperly.
static bool intersect(const int* a, const int* b, const int* c, const int* d)
{
	if (intersectProp(a, b, c, d))
		return true;
	else if (between(a, b, c) || between(a, b, d) ||
			 between(c, d, a) || between(c, d, b))
		return true;
	else
		return false;
}

static bool vequal(const int* a, const int* b)
{
	return a[0] == b[0] && a[2] == b[2];
}

static bool intersectSegContour(const int* d0, const int* d1, int i, int n, const int* verts)
{
	// For each edge (k,k+1) of P
	for (int k = 0; k < n; k++)
	{
		int k1 = next(k, n);
		// Skip edges incident to i.
		if (i == k || i == k1)
			continue;
		const int* p0 = &verts[k * 4];
		const int* p1 = &verts[k1 * 4];
		if (vequal(d0, p0) || vequal(d1, p0) || vequal(d0, p1) || vequal(d1, p1))
			continue;
		
		if (intersect(d0, d1, p0, p1))
			return true;
	}
	return false;
}

static bool	inCone(int i, int n, const int* verts, const int* pj)
{
	const int* pi = &verts[i * 4];
	const int* pi1 = &verts[next(i, n) * 4];
	const int* pin1 = &verts[prev(i, n) * 4];
	
	// If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
	if (leftOn(pin1, pi, pi1))
		return left(pi, pj, pin1) && left(pj, pi, pi1);
	// Assume (i-1,i,i+1) not collinear.
	// else P[i] is reflex.
	return !(leftOn(pi, pj, pi1) && leftOn(pj, pi, pin1));
}


static void removeDegenerateSegments(rcTempVector<int>& simplified)
{
	// Remove adjacent vertices which are equal on xz-plane,
	// or else the triangulator will get confused.
	int npts = static_cast<int>(simplified.size()) / 4;
	for (int i = 0; i < npts; ++i)
	{
		int ni = next(i, npts);
		
		if (vequal(&simplified[i*4], &simplified[ni*4]))
		{
			// Degenerate segment, remove.
			for (int j = i; j < simplified.size()/4-1; ++j)
			{
				simplified[j*4+0] = simplified[(j+1)*4+0];
				simplified[j*4+1] = simplified[(j+1)*4+1];
				simplified[j*4+2] = simplified[(j+1)*4+2];
				simplified[j*4+3] = simplified[(j+1)*4+3];
			}
			simplified.resize(simplified.size()-4);
			npts--;
		}
	}
}


static bool mergeContours(Moss_RecastContour& ca, Moss_RecastContour& cb, int ia, int ib)
{
	const int maxVerts = ca.nverts + cb.nverts + 2;
	int* verts = (int*)rcAlloc(sizeof(int)*maxVerts*4, RC_ALLOC_PERM);
	if (!verts)
		return false;
	
	int nv = 0;
	
	// Copy contour A.
	for (int i = 0; i <= ca.nverts; ++i)
	{
		int* dst = &verts[nv*4];
		const int* src = &ca.verts[((ia+i)%ca.nverts)*4];
		dst[0] = src[0];
		dst[1] = src[1];
		dst[2] = src[2];
		dst[3] = src[3];
		nv++;
	}

	// Copy contour B
	for (int i = 0; i <= cb.nverts; ++i)
	{
		int* dst = &verts[nv*4];
		const int* src = &cb.verts[((ib+i)%cb.nverts)*4];
		dst[0] = src[0];
		dst[1] = src[1];
		dst[2] = src[2];
		dst[3] = src[3];
		nv++;
	}
	
	MOSS_FREE(ca.verts);
	ca.verts = verts;
	ca.nverts = nv;
	
	MOSS_FREE(cb.verts);
	cb.verts = 0;
	cb.nverts = 0;
	
	return true;
}

struct Moss_RecastContourHole
{
	Moss_RecastContour* contour;
	int minx, minz, leftmost;
};

struct Moss_RecastContourRegion
{
	Moss_RecastContour* outline;
	Moss_RecastContourHole* holes;
	int nholes;
};

struct rcPotentialDiagonal
{
	int vert;
	int dist;
};

// Finds the lowest leftmost vertex of a contour.
static void findLeftMostVertex(Moss_RecastContour* contour, int* minx, int* minz, int* leftmost)
{
	*minx = contour->verts[0];
	*minz = contour->verts[2];
	*leftmost = 0;
	for (int i = 1; i < contour->nverts; i++)
	{
		const int x = contour->verts[i*4+0];
		const int z = contour->verts[i*4+2];
		if (x < *minx || (x == *minx && z < *minz))
		{
			*minx = x;
			*minz = z;
			*leftmost = i;
		}
	}
}

static int compareHoles(const void* va, const void* vb)
{
	const Moss_RecastContourHole* a = (const Moss_RecastContourHole*)va;
	const Moss_RecastContourHole* b = (const Moss_RecastContourHole*)vb;
	if (a->minx == b->minx)
	{
		if (a->minz < b->minz)
			return -1;
		if (a->minz > b->minz)
			return 1;
	}
	else
	{
		if (a->minx < b->minx)
			return -1;
		if (a->minx > b->minx)
			return 1;
	}
	return 0;
}


static int compareDiagDist(const void* va, const void* vb)
{
	const rcPotentialDiagonal* a = (const rcPotentialDiagonal*)va;
	const rcPotentialDiagonal* b = (const rcPotentialDiagonal*)vb;
	if (a->dist < b->dist)
		return -1;
	if (a->dist > b->dist)
		return 1;
	return 0;
}


static void mergeRegionHoles(rcContext* ctx, Moss_RecastContourRegion& region)
{
	// Sort holes from left to right.
	for (int i = 0; i < region.nholes; i++)
		findLeftMostVertex(region.holes[i].contour, &region.holes[i].minx, &region.holes[i].minz, &region.holes[i].leftmost);
	
	qsort(region.holes, region.nholes, sizeof(Moss_RecastContourHole), compareHoles);
	
	int maxVerts = region.outline->nverts;
	for (int i = 0; i < region.nholes; i++)
		maxVerts += region.holes[i].contour->nverts;
	
	rcScopedDelete<rcPotentialDiagonal> diags((rcPotentialDiagonal*)rcAlloc(sizeof(rcPotentialDiagonal)*maxVerts, RC_ALLOC_TEMP));
	if (!diags)
	{
		ctx->log(RC_LOG_WARNING, "mergeRegionHoles: Failed to allocated diags %d.", maxVerts);
		return;
	}
	
	Moss_RecastContour* outline = region.outline;
	
	// Merge holes into the outline one by one.
	for (int i = 0; i < region.nholes; i++)
	{
		Moss_RecastContour* hole = region.holes[i].contour;
		
		int index = -1;
		int bestVertex = region.holes[i].leftmost;
		for (int iter = 0; iter < hole->nverts; iter++)
		{
			// Find potential diagonals.
			// The 'best' vertex must be in the cone described by 3 consecutive vertices of the outline.
			// ..o j-1
			//   |
			//   |   * best
			//   |
			// j o-----o j+1
			//         :
			int ndiags = 0;
			const int* corner = &hole->verts[bestVertex*4];
			for (int j = 0; j < outline->nverts; j++)
			{
				if (inCone(j, outline->nverts, outline->verts, corner))
				{
					int dx = outline->verts[j*4+0] - corner[0];
					int dz = outline->verts[j*4+2] - corner[2];
					diags[ndiags].vert = j;
					diags[ndiags].dist = dx*dx + dz*dz;
					ndiags++;
				}
			}
			// Sort potential diagonals by distance, we want to make the connection as short as possible.
			qsort(diags, ndiags, sizeof(rcPotentialDiagonal), compareDiagDist);
			
			// Find a diagonal that is not intersecting the outline not the remaining holes.
			index = -1;
			for (int j = 0; j < ndiags; j++)
			{
				const int* pt = &outline->verts[diags[j].vert*4];
				bool intersect = intersectSegContour(pt, corner, diags[i].vert, outline->nverts, outline->verts);
				for (int k = i; k < region.nholes && !intersect; k++)
					intersect |= intersectSegContour(pt, corner, -1, region.holes[k].contour->nverts, region.holes[k].contour->verts);
				if (!intersect)
				{
					index = diags[j].vert;
					break;
				}
			}
			// If found non-intersecting diagonal, stop looking.
			if (index != -1)
				break;
			// All the potential diagonals for the current vertex were intersecting, try next vertex.
			bestVertex = (bestVertex + 1) % hole->nverts;
		}
		
		if (index == -1)
		{
			ctx->log(RC_LOG_WARNING, "mergeHoles: Failed to find merge points for %p and %p.", region.outline, hole);
			continue;
		}
		if (!mergeContours(*region.outline, *hole, index, bestVertex))
		{
			ctx->log(RC_LOG_WARNING, "mergeHoles: Failed to merge contours %p and %p.", region.outline, hole);
			continue;
		}
	}
}


/// @par
///
/// The raw contours will match the region outlines exactly. The @p maxError and @p maxEdgeLen
/// parameters control how closely the simplified contours will match the raw contours.
///
/// Simplified contours are generated such that the vertices for portals between areas match up.
/// (They are considered mandatory vertices.)
///
/// Setting @p maxEdgeLength to zero will disabled the edge length feature.
///
/// See the #Moss_RecastSettings3D documentation for more information on the configuration parameters.
///
/// @see rcAllocContourSet, Moss_RecastCompactHeightfield, Moss_RecastContourSet, Moss_RecastSettings3D
bool rcBuildContours(rcContext* ctx, const Moss_RecastCompactHeightfield& chf,
					 const float maxError, const int maxEdgeLen,
					 Moss_RecastContourSet& cset, const int buildFlags)
{
	rcAssert(ctx);
	
	const int w = chf.width;
	const int h = chf.height;
	const int borderSize = chf.borderSize;
	
	rcScopedTimer timer(ctx, RC_TIMER_BUILD_CONTOURS);
	
	rcVcopy(cset.bmin, chf.bmin);
	rcVcopy(cset.bmax, chf.bmax);
	if (borderSize > 0)
	{
		// If the heightfield was build with bordersize, remove the offset.
		const float pad = borderSize*chf.cs;
		cset.bmin[0] += pad;
		cset.bmin[2] += pad;
		cset.bmax[0] -= pad;
		cset.bmax[2] -= pad;
	}
	cset.cs = chf.cs;
	cset.ch = chf.ch;
	cset.width = chf.width - chf.borderSize*2;
	cset.height = chf.height - chf.borderSize*2;
	cset.borderSize = chf.borderSize;
	cset.maxError = maxError;
	
	int maxContours = max((int)chf.maxRegions, 8);
	cset.conts = (Moss_RecastContour*)rcAlloc(sizeof(Moss_RecastContour)*maxContours, RC_ALLOC_PERM);
	if (!cset.conts)
		return false;
	cset.nconts = 0;
	
	rcScopedDelete<unsigned char> flags((unsigned char*)rcAlloc(sizeof(unsigned char)*chf.spanCount, RC_ALLOC_TEMP));
	if (!flags)
	{
		ctx->log(RC_LOG_ERROR, "rcBuildContours: Out of memory 'flags' (%d).", chf.spanCount);
		return false;
	}
	
	ctx->startTimer(RC_TIMER_BUILD_CONTOURS_TRACE);
	
	// Mark boundaries.
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const Moss_RecastCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				unsigned char res = 0;
				const Moss_RecastCompactSpan& s = chf.spans[i];
				if (!chf.spans[i].reg || (chf.spans[i].reg & RC_BORDER_REG))
				{
					flags[i] = 0;
					continue;
				}
				for (int dir = 0; dir < 4; ++dir)
				{
					unsigned short r = 0;
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);
						r = chf.spans[ai].reg;
					}
					if (r == chf.spans[i].reg)
						res |= (1 << dir);
				}
				flags[i] = res ^ 0xf; // Inverse, mark non connected edges.
			}
		}
	}
	
	ctx->stopTimer(RC_TIMER_BUILD_CONTOURS_TRACE);
	
	rcTempVector<int> verts(256);
	rcTempVector<int> simplified(64);
	
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const Moss_RecastCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				if (flags[i] == 0 || flags[i] == 0xf)
				{
					flags[i] = 0;
					continue;
				}
				const unsigned short reg = chf.spans[i].reg;
				if (!reg || (reg & RC_BORDER_REG))
					continue;
				const unsigned char area = chf.areas[i];
				
				verts.clear();
				simplified.clear();
				
				ctx->startTimer(RC_TIMER_BUILD_CONTOURS_TRACE);
				walkContour(x, y, i, chf, flags, verts);
				ctx->stopTimer(RC_TIMER_BUILD_CONTOURS_TRACE);
				
				ctx->startTimer(RC_TIMER_BUILD_CONTOURS_SIMPLIFY);
				simplifyContour(verts, simplified, maxError, maxEdgeLen, buildFlags);
				removeDegenerateSegments(simplified);
				ctx->stopTimer(RC_TIMER_BUILD_CONTOURS_SIMPLIFY);
				
				
				// Store region->contour remap info.
				// Create contour.
				if (simplified.size()/4 >= 3)
				{
					if (cset.nconts >= maxContours)
					{
						// Allocate more contours.
						// This happens when a region has holes.
						const int oldMax = maxContours;
						maxContours *= 2;
						Moss_RecastContour* newConts = (Moss_RecastContour*)rcAlloc(sizeof(Moss_RecastContour)*maxContours, RC_ALLOC_PERM);
						for (int j = 0; j < cset.nconts; ++j)
						{
							newConts[j] = cset.conts[j];
							// Reset source pointers to prevent data deletion.
							cset.conts[j].verts = 0;
							cset.conts[j].rverts = 0;
						}
						MOSS_FREE(cset.conts);
						cset.conts = newConts;
						
						MOSS_ERROR("rcBuildContours: Expanding max contours from %d to %d.", oldMax, maxContours);
					}
					
					Moss_RecastContour* cont = &cset.conts[cset.nconts++];
					
					cont->nverts = static_cast<int>(simplified.size()) / 4;
					cont->verts = (int*)rcAlloc(sizeof(int)*cont->nverts*4, RC_ALLOC_PERM);
					if (!cont->verts)
					{
						MOSS_ERROR("rcBuildContours: Out of memory 'verts' (%d).", cont->nverts);
						return false;
					}
					memcpy(cont->verts, &simplified[0], sizeof(int)*cont->nverts*4);
					if (borderSize > 0)
					{
						// If the heightfield was build with bordersize, remove the offset.
						for (int j = 0; j < cont->nverts; ++j)
						{
							int* v = &cont->verts[j*4];
							v[0] -= borderSize;
							v[2] -= borderSize;
						}
					}
					
					cont->nrverts = static_cast<int>(verts.size()) / 4;
					cont->rverts = static_cast<int*>(rcAlloc(sizeof(int) * cont->nrverts * 4, RC_ALLOC_PERM));
					if (!cont->rverts) {
						MOSS_ERROR("rcBuildContours: Out of memory 'rverts' (%d).", cont->nrverts);
						return false;
					}
					memcpy(cont->rverts, &verts[0], sizeof(int)*cont->nrverts*4);
					if (borderSize > 0)
					{
						// If the heightfield was build with bordersize, remove the offset.
						for (int j = 0; j < cont->nrverts; ++j)
						{
							int* v = &cont->rverts[j*4];
							v[0] -= borderSize;
							v[2] -= borderSize;
						}
					}
					
					cont->reg = reg;
					cont->area = area;
				}
			}
		}
	}
	
	// Merge holes if needed.
	if (cset.nconts > 0)
	{
		// Calculate winding of all polygons.
		rcScopedDelete<signed char> winding((signed char*)rcAlloc(sizeof(signed char)*cset.nconts, RC_ALLOC_TEMP));
		if (!winding) {
			MOSS_ERROR("rcBuildContours: Out of memory 'hole' (%d).", cset.nconts);
			return false;
		}
		int nholes = 0;
		for (int i = 0; i < cset.nconts; ++i)
		{
			Moss_RecastContour& cont = cset.conts[i];
			// If the contour is wound backwards, it is a hole.
			winding[i] = calcAreaOfPolygon2D(cont.verts, cont.nverts) < 0 ? -1 : 1;
			if (winding[i] < 0)
				nholes++;
		}
		
		if (nholes > 0)
		{
			// Collect outline contour and holes contours per region.
			// We assume that there is one outline and multiple holes.
			const int nregions = chf.maxRegions+1;
			rcScopedDelete<Moss_RecastContourRegion> regions((Moss_RecastContourRegion*)rcAlloc(sizeof(Moss_RecastContourRegion)*nregions, RC_ALLOC_TEMP));
			if (!regions)
			{
				MOSS_ERROR("rcBuildContours: Out of memory 'regions' (%d).", nregions);
				return false;
			}
			memset(regions, 0, sizeof(Moss_RecastContourRegion)*nregions);
			
			rcScopedDelete<Moss_RecastContourHole> holes((Moss_RecastContourHole*)rcAlloc(sizeof(Moss_RecastContourHole)*cset.nconts, RC_ALLOC_TEMP));
			if (!holes)
			{
				MOSS_ERROR("rcBuildContours: Out of memory 'holes' (%d).", cset.nconts);
				return false;
			}
			memset(holes, 0, sizeof(Moss_RecastContourHole)*cset.nconts);
			
			for (int i = 0; i < cset.nconts; ++i)
			{
				Moss_RecastContour& cont = cset.conts[i];
				// Positively would contours are outlines, negative holes.
				if (winding[i] > 0)
				{
					if (regions[cont.reg].outline)
						MOSS_ERROR("rcBuildContours: Multiple outlines for region %d.", cont.reg);
					regions[cont.reg].outline = &cont;
				}
				else
				{
					regions[cont.reg].nholes++;
				}
			}
			int index = 0;
			for (int i = 0; i < nregions; i++)
			{
				if (regions[i].nholes > 0)
				{
					regions[i].holes = &holes[index];
					index += regions[i].nholes;
					regions[i].nholes = 0;
				}
			}
			for (int i = 0; i < cset.nconts; ++i)
			{
				Moss_RecastContour& cont = cset.conts[i];
				Moss_RecastContourRegion& reg = regions[cont.reg];
				if (winding[i] < 0)
					reg.holes[reg.nholes++].contour = &cont;
			}
			
			// Finally merge each regions holes into the outline.
			for (int i = 0; i < nregions; i++)
			{
				Moss_RecastContourRegion& reg = regions[i];
				if (!reg.nholes) continue;
				
				if (reg.outline)
				{
					mergeRegionHoles(ctx, reg);
				}
				else
				{
					// The region does not have an outline.
					// This can happen if the contour becaomes selfoverlapping because of
					// too aggressive simplification settings.
					MOSS_ERROR("rcBuildContours: Bad outline for region %d, contour simplification is likely too aggressive.", i);
				}
			}
		}
		
	}
	
	return true;
}


/*													*/


struct rcHeightPatch
{
	inline rcHeightPatch() : data(0), xmin(0), ymin(0), width(0), height(0) {}
	inline ~rcHeightPatch() { MOSS_FREE(data); }
	unsigned short* data;
	int xmin, ymin, width, height;
};


inline float vdot2(const float* a, const float* b)
{
	return a[0]*b[0] + a[2]*b[2];
}

inline float vdistSq2(const float* p, const float* q)
{
	const float dx = q[0] - p[0];
	const float dy = q[2] - p[2];
	return dx*dx + dy*dy;
}

inline float vdist2(const float* p, const float* q)
{
	return sqrtf(vdistSq2(p,q));
}

inline float vcross2(const float* p1, const float* p2, const float* p3)
{
	const float u1 = p2[0] - p1[0];
	const float v1 = p2[2] - p1[2];
	const float u2 = p3[0] - p1[0];
	const float v2 = p3[2] - p1[2];
	return u1 * v2 - v1 * u2;
}

static bool circumCircle(const float* p1, const float* p2, const float* p3,
						 float* c, float& r)
{
	static const float EPS = 1e-6f;
	// Calculate the circle relative to p1, to avoid some precision issues.
	const float v1[3] = {0,0,0};
	float v2[3], v3[3];
	dtVsub(v2, p2,p1);
	dtVsub(v3, p3,p1);
	
	const float cp = vcross2(v1, v2, v3);
	if (fabsf(cp) > EPS)
	{
		const float v1Sq = vdot2(v1,v1);
		const float v2Sq = vdot2(v2,v2);
		const float v3Sq = vdot2(v3,v3);
		c[0] = (v1Sq*(v2[2]-v3[2]) + v2Sq*(v3[2]-v1[2]) + v3Sq*(v1[2]-v2[2])) / (2*cp);
		c[1] = 0;
		c[2] = (v1Sq*(v3[0]-v2[0]) + v2Sq*(v1[0]-v3[0]) + v3Sq*(v2[0]-v1[0])) / (2*cp);
		r = vdist2(c, v1);
		dtVadd(c, c, p1);
		return true;
	}
	
	rcVcopy(c, p1);
	r = 0;
	return false;
}

static float distPtTri(const float* p, const float* a, const float* b, const float* c)
{
	float v0[3], v1[3], v2[3];
	dtVsub(v0, c,a);
	dtVsub(v1, b,a);
	dtVsub(v2, p,a);
	
	const float dot00 = vdot2(v0, v0);
	const float dot01 = vdot2(v0, v1);
	const float dot02 = vdot2(v0, v2);
	const float dot11 = vdot2(v1, v1);
	const float dot12 = vdot2(v1, v2);
	
	// Compute barycentric coordinates
	const float invDenom = 1.0f / (dot00 * dot11 - dot01 * dot01);
	const float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
	float v = (dot00 * dot12 - dot01 * dot02) * invDenom;
	
	// If point lies inside the triangle, return interpolated y-coord.
	static const float EPS = 1e-4f;
	if (u >= -EPS && v >= -EPS && (u+v) <= 1+EPS)
	{
		const float y = a[1] + v0[1]*u + v1[1]*v;
		return fabsf(y-p[1]);
	}
	return FLT_MAX;
}

static float distancePtSeg(const float* pt, const float* p, const float* q)
{
	float pqx = q[0] - p[0];
	float pqy = q[1] - p[1];
	float pqz = q[2] - p[2];
	float dx = pt[0] - p[0];
	float dy = pt[1] - p[1];
	float dz = pt[2] - p[2];
	float d = pqx*pqx + pqy*pqy + pqz*pqz;
	float t = pqx*dx + pqy*dy + pqz*dz;
	if (d > 0)
		t /= d;
	if (t < 0)
		t = 0;
	else if (t > 1)
		t = 1;
	
	dx = p[0] + t*pqx - pt[0];
	dy = p[1] + t*pqy - pt[1];
	dz = p[2] + t*pqz - pt[2];
	
	return dx*dx + dy*dy + dz*dz;
}

static float distancePtSeg2d(const float* pt, const float* p, const float* q)
{
	float pqx = q[0] - p[0];
	float pqz = q[2] - p[2];
	float dx = pt[0] - p[0];
	float dz = pt[2] - p[2];
	float d = pqx*pqx + pqz*pqz;
	float t = pqx*dx + pqz*dz;
	if (d > 0)
		t /= d;
	if (t < 0)
		t = 0;
	else if (t > 1)
		t = 1;
	
	dx = p[0] + t*pqx - pt[0];
	dz = p[2] + t*pqz - pt[2];
	
	return dx*dx + dz*dz;
}

static float distToTriMesh(const float* p, const float* verts, const int /*nverts*/, const int* tris, const int ntris)
{
	float dmin = FLT_MAX;
	for (int i = 0; i < ntris; ++i)
	{
		const float* va = &verts[tris[i*4+0]*3];
		const float* vb = &verts[tris[i*4+1]*3];
		const float* vc = &verts[tris[i*4+2]*3];
		float d = distPtTri(p, va,vb,vc);
		if (d < dmin)
			dmin = d;
	}
	if (dmin == FLT_MAX) return -1;
	return dmin;
}

static float distToPoly(int nvert, const float* verts, const float* p)
{
	
	float dmin = FLT_MAX;
	int i, j, c = 0;
	for (i = 0, j = nvert-1; i < nvert; j = i++)
	{
		const float* vi = &verts[i*3];
		const float* vj = &verts[j*3];
		if (((vi[2] > p[2]) != (vj[2] > p[2])) &&
			(p[0] < (vj[0]-vi[0]) * (p[2]-vi[2]) / (vj[2]-vi[2]) + vi[0]) )
			c = !c;
		dmin = min(dmin, distancePtSeg2d(p, vj, vi));
	}
	return c ? -dmin : dmin;
}


static unsigned short getHeight(const float fx, const float fy, const float fz,
								const float /*cs*/, const float ics, const float ch,
								const int radius, const rcHeightPatch& hp)
{
	int ix = (int)floorf(fx*ics + 0.01f);
	int iz = (int)floorf(fz*ics + 0.01f);
	ix = Clamp(ix-hp.xmin, 0, hp.width - 1);
	iz = Clamp(iz-hp.ymin, 0, hp.height - 1);
	unsigned short h = hp.data[ix+iz*hp.width];
	if (h == RC_UNSET_HEIGHT)
	{
		// Special case when data might be bad.
		// Walk adjacent cells in a spiral up to 'radius', and look
		// for a pixel which has a valid height.
		int x = 1, z = 0, dx = 1, dz = 0;
		int maxSize = radius * 2 + 1;
		int maxIter = maxSize * maxSize - 1;

		int nextRingIterStart = 8;
		int nextRingIters = 16;

		float dmin = FLT_MAX;
		for (int i = 0; i < maxIter; i++)
		{
			const int nx = ix + x;
			const int nz = iz + z;

			if (nx >= 0 && nz >= 0 && nx < hp.width && nz < hp.height)
			{
				const unsigned short nh = hp.data[nx + nz*hp.width];
				if (nh != RC_UNSET_HEIGHT)
				{
					const float d = fabsf(nh*ch - fy);
					if (d < dmin)
					{
						h = nh;
						dmin = d;
					}
				}
			}

			// We are searching in a grid which looks approximately like this:
			//  __________
			// |2 ______ 2|
			// | |1 __ 1| |
			// | | |__| | |
			// | |______| |
			// |__________|
			// We want to find the best height as close to the center cell as possible. This means that
			// if we find a height in one of the neighbor cells to the center, we don't want to
			// expand further out than the 8 neighbors - we want to limit our search to the closest
			// of these "rings", but the best height in the ring.
			// For example, the center is just 1 cell. We checked that at the entrance to the function.
			// The next "ring" contains 8 cells (marked 1 above). Those are all the neighbors to the center cell.
			// The next one again contains 16 cells (marked 2). In general each ring has 8 additional cells, which
			// can be thought of as adding 2 cells around the "center" of each side when we expand the ring.
			// Here we detect if we are about to enter the next ring, and if we are and we have found
			// a height, we abort the search.
			if (i + 1 == nextRingIterStart)
			{
				if (h != RC_UNSET_HEIGHT)
					break;

				nextRingIterStart += nextRingIters;
				nextRingIters += 8;
			}

			if ((x == z) || ((x < 0) && (x == -z)) || ((x > 0) && (x == 1 - z)))
			{
				int tmp = dx;
				dx = -dz;
				dz = tmp;
			}
			x += dx;
			z += dz;
		}
	}
	return h;
}


enum EdgeValues
{
	EV_UNDEF = -1,
	EV_HULL = -2
};

static int findEdge(const int* edges, int nedges, int s, int t)
{
	for (int i = 0; i < nedges; i++)
	{
		const int* e = &edges[i*4];
		if ((e[0] == s && e[1] == t) || (e[0] == t && e[1] == s))
			return i;
	}
	return EV_UNDEF;
}

static int addEdge(rcContext* ctx, int* edges, int& nedges, const int maxEdges, int s, int t, int l, int r)
{
	if (nedges >= maxEdges) {
		MOSS_ERROR("addEdge: Too many edges (%d/%d).", nedges, maxEdges);
		return EV_UNDEF;
	}
	
	// Add edge if not already in the triangulation.
	int e = findEdge(edges, nedges, s, t);
	if (e == EV_UNDEF)
	{
		int* edge = &edges[nedges*4];
		edge[0] = s;
		edge[1] = t;
		edge[2] = l;
		edge[3] = r;
		return nedges++;
	}
	else
	{
		return EV_UNDEF;
	}
}

static void updateLeftFace(int* e, int s, int t, int f)
{
	if (e[0] == s && e[1] == t && e[2] == EV_UNDEF)
		e[2] = f;
	else if (e[1] == s && e[0] == t && e[3] == EV_UNDEF)
		e[3] = f;
}

static int overlapSegSeg2d(const float* a, const float* b, const float* c, const float* d)
{
	const float a1 = vcross2(a, b, d);
	const float a2 = vcross2(a, b, c);
	if (a1*a2 < 0.0f)
	{
		float a3 = vcross2(c, d, a);
		float a4 = a3 + a2 - a1;
		if (a3 * a4 < 0.0f)
			return 1;
	}
	return 0;
}

static bool overlapEdges(const float* pts, const int* edges, int nedges, int s1, int t1)
{
	for (int i = 0; i < nedges; ++i)
	{
		const int s0 = edges[i*4+0];
		const int t0 = edges[i*4+1];
		// Same or connected edges do not overlap.
		if (s0 == s1 || s0 == t1 || t0 == s1 || t0 == t1)
			continue;
		if (overlapSegSeg2d(&pts[s0*3],&pts[t0*3], &pts[s1*3],&pts[t1*3]))
			return true;
	}
	return false;
}

static void completeFacet(rcContext* ctx, const float* pts, int npts, int* edges, int& nedges, const int maxEdges, int& nfaces, int e)
{
	static const float EPS = 1e-5f;
	
	int* edge = &edges[e*4];
	
	// Cache s and t.
	int s,t;
	if (edge[2] == EV_UNDEF)
	{
		s = edge[0];
		t = edge[1];
	}
	else if (edge[3] == EV_UNDEF)
	{
		s = edge[1];
		t = edge[0];
	}
	else
	{
	    // Edge already completed.
	    return;
	}
    
	// Find best point on left of edge.
	int pt = npts;
	float c[3] = {0,0,0};
	float r = -1;
	for (int u = 0; u < npts; ++u)
	{
		if (u == s || u == t) continue;
		if (vcross2(&pts[s*3], &pts[t*3], &pts[u*3]) > EPS)
		{
			if (r < 0)
			{
				// The circle is not updated yet, do it now.
				pt = u;
				circumCircle(&pts[s*3], &pts[t*3], &pts[u*3], c, r);
				continue;
			}
			const float d = vdist2(c, &pts[u*3]);
			const float tol = 0.001f;
			if (d > r*(1+tol))
			{
				// Outside current circumcircle, skip.
				continue;
			}
			else if (d < r*(1-tol))
			{
				// Inside safe circumcircle, update circle.
				pt = u;
				circumCircle(&pts[s*3], &pts[t*3], &pts[u*3], c, r);
			}
			else
			{
				// Inside epsilon circum circle, do extra tests to make sure the edge is valid.
				// s-u and t-u cannot overlap with s-pt nor t-pt if they exists.
				if (overlapEdges(pts, edges, nedges, s,u))
					continue;
				if (overlapEdges(pts, edges, nedges, t,u))
					continue;
				// Edge is valid.
				pt = u;
				circumCircle(&pts[s*3], &pts[t*3], &pts[u*3], c, r);
			}
		}
	}
	
	// Add new triangle or update edge info if s-t is on hull.
	if (pt < npts)
	{
		// Update face information of edge being completed.
		updateLeftFace(&edges[e*4], s, t, nfaces);
		
		// Add new edge or update face info of old edge.
		e = findEdge(edges, nedges, pt, s);
		if (e == EV_UNDEF)
		    addEdge(ctx, edges, nedges, maxEdges, pt, s, nfaces, EV_UNDEF);
		else
		    updateLeftFace(&edges[e*4], pt, s, nfaces);
		
		// Add new edge or update face info of old edge.
		e = findEdge(edges, nedges, t, pt);
		if (e == EV_UNDEF)
		    addEdge(ctx, edges, nedges, maxEdges, t, pt, nfaces, EV_UNDEF);
		else
		    updateLeftFace(&edges[e*4], t, pt, nfaces);
		
		nfaces++;
	}
	else
	{
		updateLeftFace(&edges[e*4], s, t, EV_HULL);
	}
}

static void delaunayHull(rcContext* ctx, const int npts, const float* pts,
						 const int nhull, const int* hull,
						 rcTempVector<int>& tris, rcTempVector<int>& edges)
{
	int nfaces = 0;
	int nedges = 0;
	const int maxEdges = npts*10;
	edges.resize(maxEdges*4);
	
	for (int i = 0, j = nhull-1; i < nhull; j=i++)
		addEdge(ctx, &edges[0], nedges, maxEdges, hull[j],hull[i], EV_HULL, EV_UNDEF);
	
	int currentEdge = 0;
	while (currentEdge < nedges)
	{
		if (edges[currentEdge*4+2] == EV_UNDEF)
			completeFacet(ctx, pts, npts, &edges[0], nedges, maxEdges, nfaces, currentEdge);
		if (edges[currentEdge*4+3] == EV_UNDEF)
			completeFacet(ctx, pts, npts, &edges[0], nedges, maxEdges, nfaces, currentEdge);
		currentEdge++;
	}
	
	// Create tris
	tris.resize(nfaces*4);
	for (int i = 0; i < nfaces*4; ++i)
		tris[i] = -1;
	
	for (int i = 0; i < nedges; ++i)
	{
		const int* e = &edges[i*4];
		if (e[3] >= 0)
		{
			// Left face
			int* t = &tris[e[3]*4];
			if (t[0] == -1)
			{
				t[0] = e[0];
				t[1] = e[1];
			}
			else if (t[0] == e[1])
				t[2] = e[0];
			else if (t[1] == e[0])
				t[2] = e[1];
		}
		if (e[2] >= 0)
		{
			// Right
			int* t = &tris[e[2]*4];
			if (t[0] == -1)
			{
				t[0] = e[1];
				t[1] = e[0];
			}
			else if (t[0] == e[0])
				t[2] = e[1];
			else if (t[1] == e[1])
				t[2] = e[0];
		}
	}
	
	for (int i = 0; i < tris.size()/4; ++i)
	{
		int* t = &tris[i*4];
		if (t[0] == -1 || t[1] == -1 || t[2] == -1)
		{
			ctx->log(RC_LOG_WARNING, "delaunayHull: Removing dangling face %d [%d,%d,%d].", i, t[0],t[1],t[2]);
			t[0] = tris[tris.size()-4];
			t[1] = tris[tris.size()-3];
			t[2] = tris[tris.size()-2];
			t[3] = tris[tris.size()-1];
			tris.resize(tris.size()-4);
			--i;
		}
	}
}

// Calculate minimum extend of the polygon.
static float polyMinExtent(const float* verts, const int nverts)
{
	float minDist = FLT_MAX;
	for (int i = 0; i < nverts; i++)
	{
		const int ni = (i+1) % nverts;
		const float* p1 = &verts[i*3];
		const float* p2 = &verts[ni*3];
		float maxEdgeDist = 0;
		for (int j = 0; j < nverts; j++)
		{
			if (j == i || j == ni) continue;
			float d = distancePtSeg2d(&verts[j*3], p1,p2);
			maxEdgeDist = max(maxEdgeDist, d);
		}
		minDist = min(minDist, maxEdgeDist);
	}
	return sqrtf(minDist);
}

// Last time I checked the if version got compiled using cmov, which was a lot faster than module (with idiv).
inline int prev(int i, int n) { return i-1 >= 0 ? i-1 : n-1; }
inline int next(int i, int n) { return i+1 < n ? i+1 : 0; }

static void triangulateHull(const int /*nverts*/, const float* verts, const int nhull, const int* hull, const int nin, rcTempVector<int>& tris)
{
	int start = 0, left = 1, right = nhull-1;
	
	// Start from an ear with shortest perimeter.
	// This tends to favor well formed triangles as starting point.
	float dmin = FLT_MAX;
	for (int i = 0; i < nhull; i++)
	{
		if (hull[i] >= nin) continue; // Ears are triangles with original vertices as middle vertex while others are actually line segments on edges
		int pi = prev(i, nhull);
		int ni = next(i, nhull);
		const float* pv = &verts[hull[pi]*3];
		const float* cv = &verts[hull[i]*3];
		const float* nv = &verts[hull[ni]*3];
		const float d = vdist2(pv,cv) + vdist2(cv,nv) + vdist2(nv,pv);
		if (d < dmin)
		{
			start = i;
			left = ni;
			right = pi;
			dmin = d;
		}
	}
	
	// Add first triangle
	tris.push_back(hull[start]);
	tris.push_back(hull[left]);
	tris.push_back(hull[right]);
	tris.push_back(0);
	
	// Triangulate the polygon by moving left or right,
	// depending on which triangle has shorter perimeter.
	// This heuristic was chose empirically, since it seems
	// handle tessellated straight edges well.
	while (next(left, nhull) != right)
	{
		// Check to see if se should advance left or right.
		int nleft = next(left, nhull);
		int nright = prev(right, nhull);
		
		const float* cvleft = &verts[hull[left]*3];
		const float* nvleft = &verts[hull[nleft]*3];
		const float* cvright = &verts[hull[right]*3];
		const float* nvright = &verts[hull[nright]*3];
		const float dleft = vdist2(cvleft, nvleft) + vdist2(nvleft, cvright);
		const float dright = vdist2(cvright, nvright) + vdist2(cvleft, nvright);
		
		if (dleft < dright)
		{
			tris.push_back(hull[left]);
			tris.push_back(hull[nleft]);
			tris.push_back(hull[right]);
			tris.push_back(0);
			left = nleft;
		}
		else
		{
			tris.push_back(hull[left]);
			tris.push_back(hull[nright]);
			tris.push_back(hull[right]);
			tris.push_back(0);
			right = nright;
		}
	}
}


inline float getJitterX(const int i)
{
	return (((i * 0x8da6b343) & 0xffff) / 65535.0f * 2.0f) - 1.0f;
}

inline float getJitterY(const int i)
{
	return (((i * 0xd8163841) & 0xffff) / 65535.0f * 2.0f) - 1.0f;
}

static bool onHull(int a, int b, int nhull, int* hull)
{
	// All internal sampled points come after the hull so we can early out for those.
	if (a >= nhull || b >= nhull)
		return false;

	for (int j = nhull - 1, i = 0; i < nhull; j = i++)
	{
		if (a == hull[j] && b == hull[i])
			return true;
	}

	return false;
}

// Find edges that lie on hull and mark them as such.
static void setTriFlags(rcTempVector<int>& tris, int nhull, int* hull)
{
	// Matches DT_DETAIL_EDGE_BOUNDARY
	const int DETAIL_EDGE_BOUNDARY = 0x1;

	for (int i = 0; i < tris.size(); i += 4)
	{
		int a = tris[i + 0];
		int b = tris[i + 1];
		int c = tris[i + 2];
		unsigned short flags = 0;
		flags |= (onHull(a, b, nhull, hull) ? DETAIL_EDGE_BOUNDARY : 0) << 0;
		flags |= (onHull(b, c, nhull, hull) ? DETAIL_EDGE_BOUNDARY : 0) << 2;
		flags |= (onHull(c, a, nhull, hull) ? DETAIL_EDGE_BOUNDARY : 0) << 4;
		tris[i + 3] = (int)flags;
	}
}

static bool buildPolyDetail(rcContext* ctx, const float* in, const int nin,
							const float sampleDist, const float sampleMaxError,
							const int heightSearchRadius, const Moss_RecastCompactHeightfield& chf,
							const rcHeightPatch& hp, float* verts, int& nverts,
							rcTempVector<int>& tris, rcTempVector<int>& edges, rcTempVector<int>& samples)
{
	static const int MAX_VERTS = 127;
	static const int MAX_TRIS = 255;	// Max tris for delaunay is 2n-2-k (n=num verts, k=num hull verts).
	static const int MAX_VERTS_PER_EDGE = 32;
	float edge[(MAX_VERTS_PER_EDGE+1)*3];
	int hull[MAX_VERTS];
	int nhull = 0;
	
	nverts = nin;
	
	for (int i = 0; i < nin; ++i)
		rcVcopy(&verts[i*3], &in[i*3]);
	
	edges.clear();
	tris.clear();
	
	const float cs = chf.cs;
	const float ics = 1.0f/cs;
	
	// Calculate minimum extents of the polygon based on input data.
	float minExtent = polyMinExtent(verts, nverts);
	
	// Tessellate outlines.
	// This is done in separate pass in order to ensure
	// seamless height values across the ply boundaries.
	if (sampleDist > 0)
	{
		for (int i = 0, j = nin-1; i < nin; j=i++)
		{
			const float* vj = &in[j*3];
			const float* vi = &in[i*3];
			bool swapped = false;
			// Make sure the segments are always handled in same order
			// using lexological sort or else there will be seams.
			if (fabsf(vj[0]-vi[0]) < 1e-6f)
			{
				if (vj[2] > vi[2])
				{
					Swap(vj,vi);
					swapped = true;
				}
			}
			else
			{
				if (vj[0] > vi[0])
				{
					Swap(vj,vi);
					swapped = true;
				}
			}
			// Create samples along the edge.
			float dx = vi[0] - vj[0];
			float dy = vi[1] - vj[1];
			float dz = vi[2] - vj[2];
			float d = sqrtf(dx*dx + dz*dz);
			int nn = 1 + (int)floorf(d/sampleDist);
			if (nn >= MAX_VERTS_PER_EDGE) nn = MAX_VERTS_PER_EDGE-1;
			if (nverts+nn >= MAX_VERTS)
				nn = MAX_VERTS-1-nverts;
			
			for (int k = 0; k <= nn; ++k)
			{
				float u = (float)k/(float)nn;
				float* pos = &edge[k*3];
				pos[0] = vj[0] + dx*u;
				pos[1] = vj[1] + dy*u;
				pos[2] = vj[2] + dz*u;
				pos[1] = getHeight(pos[0],pos[1],pos[2], cs, ics, chf.ch, heightSearchRadius, hp)*chf.ch;
			}
			// Simplify samples.
			int idx[MAX_VERTS_PER_EDGE] = {0,nn};
			int nidx = 2;
			for (int k = 0; k < nidx-1; )
			{
				const int a = idx[k];
				const int b = idx[k+1];
				const float* va = &edge[a*3];
				const float* vb = &edge[b*3];
				// Find maximum deviation along the segment.
				float maxd = 0;
				int maxi = -1;
				for (int m = a+1; m < b; ++m)
				{
					float dev = distancePtSeg(&edge[m*3],va,vb);
					if (dev > maxd)
					{
						maxd = dev;
						maxi = m;
					}
				}
				// If the max deviation is larger than accepted error,
				// add new point, else continue to next segment.
				if (maxi != -1 && maxd > Sqr(sampleMaxError))
				{
					for (int m = nidx; m > k; --m)
						idx[m] = idx[m-1];
					idx[k+1] = maxi;
					nidx++;
				}
				else
				{
					++k;
				}
			}
			
			hull[nhull++] = j;
			// Add new vertices.
			if (swapped)
			{
				for (int k = nidx-2; k > 0; --k)
				{
					rcVcopy(&verts[nverts*3], &edge[idx[k]*3]);
					hull[nhull++] = nverts;
					nverts++;
				}
			}
			else
			{
				for (int k = 1; k < nidx-1; ++k)
				{
					rcVcopy(&verts[nverts*3], &edge[idx[k]*3]);
					hull[nhull++] = nverts;
					nverts++;
				}
			}
		}
	}
	
	// If the polygon minimum extent is small (sliver or small triangle), do not try to add internal points.
	if (minExtent < sampleDist*2)
	{
		triangulateHull(nverts, verts, nhull, hull, nin, tris);
		setTriFlags(tris, nhull, hull);
		return true;
	}
	
	// Tessellate the base mesh.
	// We're using the triangulateHull instead of delaunayHull as it tends to
	// create a bit better triangulation for long thin triangles when there
	// are no internal points.
	triangulateHull(nverts, verts, nhull, hull, nin, tris);
	
	if (tris.size() == 0)
	{
		// Could not triangulate the poly, make sure there is some valid data there.
		ctx->log(RC_LOG_WARNING, "buildPolyDetail: Could not triangulate polygon (%d verts).", nverts);
		return true;
	}
	
	if (sampleDist > 0)
	{
		// Create sample locations in a grid.
		float bmin[3], bmax[3];
		rcVcopy(bmin, in);
		rcVcopy(bmax, in);
		for (int i = 1; i < nin; ++i)
		{
			rcVmin(bmin, &in[i*3]);
			rcVmax(bmax, &in[i*3]);
		}
		int x0 = (int)floorf(bmin[0]/sampleDist);
		int x1 = (int)ceilf(bmax[0]/sampleDist);
		int z0 = (int)floorf(bmin[2]/sampleDist);
		int z1 = (int)ceilf(bmax[2]/sampleDist);
		samples.clear();
		for (int z = z0; z < z1; ++z)
		{
			for (int x = x0; x < x1; ++x)
			{
				float pt[3];
				pt[0] = x*sampleDist;
				pt[1] = (bmax[1]+bmin[1])*0.5f;
				pt[2] = z*sampleDist;
				// Make sure the samples are not too close to the edges.
				if (distToPoly(nin,in,pt) > -sampleDist/2) continue;
				samples.push_back(x);
				samples.push_back(getHeight(pt[0], pt[1], pt[2], cs, ics, chf.ch, heightSearchRadius, hp));
				samples.push_back(z);
				samples.push_back(0); // Not added
			}
		}
		
		// Add the samples starting from the one that has the most
		// error. The procedure stops when all samples are added
		// or when the max error is within treshold.
		const int nsamples = static_cast<int>(samples.size()) / 4;
		for (int iter = 0; iter < nsamples; ++iter)
		{
			if (nverts >= MAX_VERTS)
				break;
			
			// Find sample with most error.
			float bestpt[3] = {0,0,0};
			float bestd = 0;
			int besti = -1;
			for (int i = 0; i < nsamples; ++i)
			{
				const int* s = &samples[i*4];
				if (s[3]) continue; // skip added.
				float pt[3];
				// The sample location is jittered to get rid of some bad triangulations
				// which are cause by symmetrical data from the grid structure.
				pt[0] = s[0]*sampleDist + getJitterX(i)*cs*0.1f;
				pt[1] = s[1]*chf.ch;
				pt[2] = s[2]*sampleDist + getJitterY(i)*cs*0.1f;
				float d = distToTriMesh(pt, verts, nverts, &tris[0], static_cast<int>(tris.size()) / 4);
				if (d < 0) continue; // did not hit the mesh.
				if (d > bestd)
				{
					bestd = d;
					besti = i;
					rcVcopy(bestpt,pt);
				}
			}
			// If the max error is within accepted threshold, stop tesselating.
			if (bestd <= sampleMaxError || besti == -1)
				break;
			// Mark sample as added.
			samples[besti*4+3] = 1;
			// Add the new sample point.
			rcVcopy(&verts[nverts*3],bestpt);
			nverts++;
			
			// Create new triangulation.
			// TODO: Incremental add instead of full rebuild.
			edges.clear();
			tris.clear();
			delaunayHull(ctx, nverts, verts, nhull, hull, tris, edges);
		}
	}
	
	const int ntris = static_cast<int>(tris.size()) / 4;
	if (ntris > MAX_TRIS)
	{
		tris.resize(MAX_TRIS*4);
		MOSS_ERROR("rcBuildPolyMeshDetail: Shrinking triangle count from %d to max %d.", ntris, MAX_TRIS);
	}

	setTriFlags(tris, nhull, hull);
	
	return true;
}

static void seedArrayWithPolyCenter(rcContext* ctx, const Moss_RecastCompactHeightfield& chf,
									const unsigned short* poly, const int npoly,
									const unsigned short* verts, const int bs,
									rcHeightPatch& hp, rcTempVector<int>& array)
{
	// Note: Reads to the compact heightfield are offset by border size (bs)
	// since border size offset is already removed from the polymesh vertices.
	
	static const int offset[9*2] =
	{
		0,0, -1,-1, 0,-1, 1,-1, 1,0, 1,1, 0,1, -1,1, -1,0,
	};
	
	// Find cell closest to a poly vertex
	int startCellX = 0, startCellY = 0, startSpanIndex = -1;
	int dmin = RC_UNSET_HEIGHT;
	for (int j = 0; j < npoly && dmin > 0; ++j)
	{
		for (int k = 0; k < 9 && dmin > 0; ++k)
		{
			const int ax = (int)verts[poly[j]*3+0] + offset[k*2+0];
			const int ay = (int)verts[poly[j]*3+1];
			const int az = (int)verts[poly[j]*3+2] + offset[k*2+1];
			if (ax < hp.xmin || ax >= hp.xmin+hp.width ||
				az < hp.ymin || az >= hp.ymin+hp.height)
				continue;
			
			const Moss_RecastCompactCell& c = chf.cells[(ax+bs)+(az+bs)*chf.width];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni && dmin > 0; ++i)
			{
				const Moss_RecastCompactSpan& s = chf.spans[i];
				int d = abs(ay - (int)s.y);
				if (d < dmin)
				{
					startCellX = ax;
					startCellY = az;
					startSpanIndex = i;
					dmin = d;
				}
			}
		}
	}
	
	rcAssert(startSpanIndex != -1);
	// Find center of the polygon
	int pcx = 0, pcy = 0;
	for (int j = 0; j < npoly; ++j)
	{
		pcx += (int)verts[poly[j]*3+0];
		pcy += (int)verts[poly[j]*3+2];
	}
	pcx /= npoly;
	pcy /= npoly;
	
	// Use seeds array as a stack for DFS
	array.clear();
	array.push_back(startCellX);
	array.push_back(startCellY);
	array.push_back(startSpanIndex);

	int dirs[] = { 0, 1, 2, 3 };
	memset(hp.data, 0, sizeof(unsigned short)*hp.width*hp.height);
	// DFS to move to the center. Note that we need a DFS here and can not just move
	// directly towards the center without recording intermediate nodes, even though the polygons
	// are convex. In very rare we can get stuck due to contour simplification if we do not
	// record nodes.
	int cx = -1, cy = -1, ci = -1;
	while (true)
	{
		if (array.size() < 3)
		{
			ctx->log(RC_LOG_WARNING, "Walk towards polygon center failed to reach center");
			break;
		}

		ci = array.back(); array.pop_back();
		cy = array.back(); array.pop_back();
		cx = array.back(); array.pop_back();

		if (cx == pcx && cy == pcy)
			break;

		// If we are already at the correct X-position, prefer direction
		// directly towards the center in the Y-axis; otherwise prefer
		// direction in the X-axis
		int directDir;
		if (cx == pcx)
			directDir = rcGetDirForOffset(0, pcy > cy ? 1 : -1);
		else
			directDir = rcGetDirForOffset(pcx > cx ? 1 : -1, 0);

		// Push the direct dir last so we start with this on next iteration
		Swap(dirs[directDir], dirs[3]);

		const Moss_RecastCompactSpan& cs = chf.spans[ci];
		for (int i = 0; i < 4; i++)
		{
			int dir = dirs[i];
			if (rcGetCon(cs, dir) == RC_NOT_CONNECTED)
				continue;

			int newX = cx + rcGetDirOffsetX(dir);
			int newY = cy + rcGetDirOffsetY(dir);

			int hpx = newX - hp.xmin;
			int hpy = newY - hp.ymin;
			if (hpx < 0 || hpx >= hp.width || hpy < 0 || hpy >= hp.height)
				continue;

			if (hp.data[hpx+hpy*hp.width] != 0)
				continue;

			hp.data[hpx+hpy*hp.width] = 1;
			array.push_back(newX);
			array.push_back(newY);
			array.push_back((int)chf.cells[(newX+bs)+(newY+bs)*chf.width].index + rcGetCon(cs, dir));
		}

		Swap(dirs[directDir], dirs[3]);
	}

	array.clear();
	// getHeightData seeds are given in coordinates with borders
	array.push_back(cx+bs);
	array.push_back(cy+bs);
	array.push_back(ci);

	memset(hp.data, 0xff, sizeof(unsigned short)*hp.width*hp.height);
	const Moss_RecastCompactSpan& cs = chf.spans[ci];
	hp.data[cx-hp.xmin+(cy-hp.ymin)*hp.width] = cs.y;
}


static void push3(rcTempVector<int>& queue, int v1, int v2, int v3)
{
	queue.resize(queue.size() + 3);
	queue[queue.size() - 3] = v1;
	queue[queue.size() - 2] = v2;
	queue[queue.size() - 1] = v3;
}

static void getHeightData(rcContext* ctx, const Moss_RecastCompactHeightfield& chf,
						  const unsigned short* poly, const int npoly,
						  const unsigned short* verts, const int bs,
						  rcHeightPatch& hp, rcTempVector<int>& queue,
						  int region)
{
	// Note: Reads to the compact heightfield are offset by border size (bs)
	// since border size offset is already removed from the polymesh vertices.
	
	queue.clear();
	// Set all heights to RC_UNSET_HEIGHT.
	memset(hp.data, 0xff, sizeof(unsigned short)*hp.width*hp.height);

	bool empty = true;
	
	// We cannot sample from this poly if it was created from polys
	// of different regions. If it was then it could potentially be overlapping
	// with polys of that region and the heights sampled here could be wrong.
	if (region != RC_MULTIPLE_REGS)
	{
		// Copy the height from the same region, and mark region borders
		// as seed points to fill the rest.
		for (int hy = 0; hy < hp.height; hy++)
		{
			int y = hp.ymin + hy + bs;
			for (int hx = 0; hx < hp.width; hx++)
			{
				int x = hp.xmin + hx + bs;
				const Moss_RecastCompactCell& c = chf.cells[x + y*chf.width];
				for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
				{
					const Moss_RecastCompactSpan& s = chf.spans[i];
					if (s.reg == region)
					{
						// Store height
						hp.data[hx + hy*hp.width] = s.y;
						empty = false;

						// If any of the neighbours is not in same region,
						// add the current location as flood fill start
						bool border = false;
						for (int dir = 0; dir < 4; ++dir)
						{
							if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
							{
								const int ax = x + rcGetDirOffsetX(dir);
								const int ay = y + rcGetDirOffsetY(dir);
								const int ai = (int)chf.cells[ax + ay*chf.width].index + rcGetCon(s, dir);
								const Moss_RecastCompactSpan& as = chf.spans[ai];
								if (as.reg != region)
								{
									border = true;
									break;
								}
							}
						}
						if (border)
							push3(queue, x, y, i);
						break;
					}
				}
			}
		}
	}
	
	// if the polygon does not contain any points from the current region (rare, but happens)
	// or if it could potentially be overlapping polygons of the same region,
	// then use the center as the seed point.
	if (empty)
		seedArrayWithPolyCenter(ctx, chf, poly, npoly, verts, bs, hp, queue);
	
	static const int RETRACT_SIZE = 256;
	int head = 0;
	
	// We assume the seed is centered in the polygon, so a BFS to collect
	// height data will ensure we do not move onto overlapping polygons and
	// sample wrong heights.
	while (head*3 < queue.size())
	{
		int cx = queue[head*3+0];
		int cy = queue[head*3+1];
		int ci = queue[head*3+2];
		head++;
		if (head >= RETRACT_SIZE)
		{
			head = 0;
			if (queue.size() > RETRACT_SIZE*3)
				memmove(&queue[0], &queue[RETRACT_SIZE*3], sizeof(int)*(queue.size()-RETRACT_SIZE*3));
			queue.resize(queue.size()-RETRACT_SIZE*3);
		}
		
		const Moss_RecastCompactSpan& cs = chf.spans[ci];
		for (int dir = 0; dir < 4; ++dir)
		{
			if (rcGetCon(cs, dir) == RC_NOT_CONNECTED) continue;
			
			const int ax = cx + rcGetDirOffsetX(dir);
			const int ay = cy + rcGetDirOffsetY(dir);
			const int hx = ax - hp.xmin - bs;
			const int hy = ay - hp.ymin - bs;
			
			if ((unsigned int)hx >= (unsigned int)hp.width || (unsigned int)hy >= (unsigned int)hp.height)
				continue;
			
			if (hp.data[hx + hy*hp.width] != RC_UNSET_HEIGHT)
				continue;
			
			const int ai = (int)chf.cells[ax + ay*chf.width].index + rcGetCon(cs, dir);
			const Moss_RecastCompactSpan& as = chf.spans[ai];
			
			hp.data[hx + hy*hp.width] = as.y;
			
			push3(queue, ax, ay, ai);
		}
	}
}

/// @par
///
/// See the #Moss_RecastSettings3D documentation for more information on the configuration parameters.
///
/// @see rcAllocPolyMeshDetail, Moss_RecastPolyMesh, Moss_RecastCompactHeightfield, Moss_RecastPolyMeshDetail, Moss_RecastSettings3D
bool rcBuildPolyMeshDetail(rcContext* ctx, const Moss_RecastPolyMesh& mesh, const Moss_RecastCompactHeightfield& chf,
						   const float sampleDist, const float sampleMaxError,
						   Moss_RecastPolyMeshDetail& dmesh)
{
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_BUILD_POLYMESHDETAIL);
	
	if (mesh.nverts == 0 || mesh.npolys == 0)
		return true;
	
	const int nvp = mesh.nvp;
	const float cs = mesh.cs;
	const float ch = mesh.ch;
	const float* orig = mesh.bmin;
	const int borderSize = mesh.borderSize;
	const int heightSearchRadius = max(1, (int)ceilf(mesh.maxEdgeError));
	
	rcTempVector<int> edges(64);
	rcTempVector<int> tris(512);
	rcTempVector<int> arr(512);
	rcTempVector<int> samples(512);
	float verts[256*3];
	rcHeightPatch hp;
	int nPolyVerts = 0;
	int maxhw = 0, maxhh = 0;
	
	rcScopedDelete<int> bounds((int*)rcAlloc(sizeof(int)*mesh.npolys*4, RC_ALLOC_TEMP));
	if (!bounds)
	{
		MOSS_ERROR("rcBuildPolyMeshDetail: Out of memory 'bounds' (%d).", mesh.npolys*4);
		return false;
	}
	rcScopedDelete<float> poly((float*)rcAlloc(sizeof(float)*nvp*3, RC_ALLOC_TEMP));
	if (!poly)
	{
		MOSS_ERROR("rcBuildPolyMeshDetail: Out of memory 'poly' (%d).", nvp*3);
		return false;
	}
	
	// Find max size for a polygon area.
	for (int i = 0; i < mesh.npolys; ++i)
	{
		const unsigned short* p = &mesh.polys[i*nvp*2];
		int& xmin = bounds[i*4+0];
		int& xmax = bounds[i*4+1];
		int& ymin = bounds[i*4+2];
		int& ymax = bounds[i*4+3];
		xmin = chf.width;
		xmax = 0;
		ymin = chf.height;
		ymax = 0;
		for (int j = 0; j < nvp; ++j)
		{
			if(p[j] == RC_MESH_NULL_IDX) break;
			const unsigned short* v = &mesh.verts[p[j]*3];
			xmin = min(xmin, (int)v[0]);
			xmax = max(xmax, (int)v[0]);
			ymin = min(ymin, (int)v[2]);
			ymax = max(ymax, (int)v[2]);
			nPolyVerts++;
		}
		xmin = max(0,xmin-1);
		xmax = min(chf.width,xmax+1);
		ymin = max(0,ymin-1);
		ymax = min(chf.height,ymax+1);
		if (xmin >= xmax || ymin >= ymax) continue;
		maxhw = max(maxhw, xmax-xmin);
		maxhh = max(maxhh, ymax-ymin);
	}
	
	hp.data = (unsigned short*)rcAlloc(sizeof(unsigned short)*maxhw*maxhh, RC_ALLOC_TEMP);
	if (!hp.data)
	{
		MOSS_ERROR("rcBuildPolyMeshDetail: Out of memory 'hp.data' (%d).", maxhw*maxhh);
		return false;
	}
	
	dmesh.nmeshes = mesh.npolys;
	dmesh.nverts = 0;
	dmesh.ntris = 0;
	dmesh.meshes = (unsigned int*)rcAlloc(sizeof(unsigned int)*dmesh.nmeshes*4, RC_ALLOC_PERM);
	if (!dmesh.meshes)
	{
		MOSS_ERROR("rcBuildPolyMeshDetail: Out of memory 'dmesh.meshes' (%d).", dmesh.nmeshes*4);
		return false;
	}
	
	int vcap = nPolyVerts+nPolyVerts/2;
	int tcap = vcap*2;
	
	dmesh.nverts = 0;
	dmesh.verts = (float*)rcAlloc(sizeof(float)*vcap*3, RC_ALLOC_PERM);
	if (!dmesh.verts)
	{
		MOSS_ERROR("rcBuildPolyMeshDetail: Out of memory 'dmesh.verts' (%d).", vcap*3);
		return false;
	}
	dmesh.ntris = 0;
	dmesh.tris = (unsigned char*)rcAlloc(sizeof(unsigned char)*tcap*4, RC_ALLOC_PERM);
	if (!dmesh.tris)
	{
		MOSS_ERROR("rcBuildPolyMeshDetail: Out of memory 'dmesh.tris' (%d).", tcap*4);
		return false;
	}
	
	for (int i = 0; i < mesh.npolys; ++i)
	{
		const unsigned short* p = &mesh.polys[i*nvp*2];
		
		// Store polygon vertices for processing.
		int npoly = 0;
		for (int j = 0; j < nvp; ++j)
		{
			if(p[j] == RC_MESH_NULL_IDX) break;
			const unsigned short* v = &mesh.verts[p[j]*3];
			poly[j*3+0] = v[0]*cs;
			poly[j*3+1] = v[1]*ch;
			poly[j*3+2] = v[2]*cs;
			npoly++;
		}
		
		// Get the height data from the area of the polygon.
		hp.xmin = bounds[i*4+0];
		hp.ymin = bounds[i*4+2];
		hp.width = bounds[i*4+1]-bounds[i*4+0];
		hp.height = bounds[i*4+3]-bounds[i*4+2];
		getHeightData(ctx, chf, p, npoly, mesh.verts, borderSize, hp, arr, mesh.regs[i]);
		
		// Build detail mesh.
		int nverts = 0;
		if (!buildPolyDetail(ctx, poly, npoly,
							 sampleDist, sampleMaxError,
							 heightSearchRadius, chf, hp,
							 verts, nverts, tris,
							 edges, samples))
		{
			return false;
		}
		
		// Move detail verts to world space.
		for (int j = 0; j < nverts; ++j)
		{
			verts[j*3+0] += orig[0];
			verts[j*3+1] += orig[1] + chf.ch; // Is this offset necessary?
			verts[j*3+2] += orig[2];
		}
		// Offset poly too, will be used to flag checking.
		for (int j = 0; j < npoly; ++j)
		{
			poly[j*3+0] += orig[0];
			poly[j*3+1] += orig[1];
			poly[j*3+2] += orig[2];
		}
		
		// Store detail submesh.
		const int ntris = static_cast<int>(tris.size()) / 4;
		
		dmesh.meshes[i*4+0] = (unsigned int)dmesh.nverts;
		dmesh.meshes[i*4+1] = (unsigned int)nverts;
		dmesh.meshes[i*4+2] = (unsigned int)dmesh.ntris;
		dmesh.meshes[i*4+3] = (unsigned int)ntris;
		
		// Store vertices, allocate more memory if necessary.
		if (dmesh.nverts+nverts > vcap)
		{
			while (dmesh.nverts+nverts > vcap)
				vcap += 256;
			
			float* newv = (float*)rcAlloc(sizeof(float)*vcap*3, RC_ALLOC_PERM);
			if (!newv)
			{
				MOSS_ERROR("rcBuildPolyMeshDetail: Out of memory 'newv' (%d).", vcap*3);
				return false;
			}
			if (dmesh.nverts)
				memcpy(newv, dmesh.verts, sizeof(float)*3*dmesh.nverts);
			MOSS_FREE(dmesh.verts);
			dmesh.verts = newv;
		}
		for (int j = 0; j < nverts; ++j)
		{
			dmesh.verts[dmesh.nverts*3+0] = verts[j*3+0];
			dmesh.verts[dmesh.nverts*3+1] = verts[j*3+1];
			dmesh.verts[dmesh.nverts*3+2] = verts[j*3+2];
			dmesh.nverts++;
		}
		
		// Store triangles, allocate more memory if necessary.
		if (dmesh.ntris+ntris > tcap)
		{
			while (dmesh.ntris+ntris > tcap)
				tcap += 256;
			unsigned char* newt = (unsigned char*)rcAlloc(sizeof(unsigned char)*tcap*4, RC_ALLOC_PERM);
			if (!newt)
			{
				MOSS_ERROR("rcBuildPolyMeshDetail: Out of memory 'newt' (%d).", tcap*4);
				return false;
			}
			if (dmesh.ntris)
				memcpy(newt, dmesh.tris, sizeof(unsigned char)*4*dmesh.ntris);
			MOSS_FREE(dmesh.tris);
			dmesh.tris = newt;
		}
		for (int j = 0; j < ntris; ++j)
		{
			const int* t = &tris[j*4];
			dmesh.tris[dmesh.ntris*4+0] = (unsigned char)t[0];
			dmesh.tris[dmesh.ntris*4+1] = (unsigned char)t[1];
			dmesh.tris[dmesh.ntris*4+2] = (unsigned char)t[2];
			dmesh.tris[dmesh.ntris*4+3] = (unsigned char)t[3];
			dmesh.ntris++;
		}
	}
	
	return true;
}

/// @see rcAllocPolyMeshDetail, Moss_RecastPolyMeshDetail
bool rcMergePolyMeshDetails(rcContext* ctx, Moss_RecastPolyMeshDetail** meshes, const int nmeshes, Moss_RecastPolyMeshDetail& mesh)
{
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_MERGE_POLYMESHDETAIL);
	
	int maxVerts = 0;
	int maxTris = 0;
	int maxMeshes = 0;
	
	for (int i = 0; i < nmeshes; ++i)
	{
		if (!meshes[i]) continue;
		maxVerts += meshes[i]->nverts;
		maxTris += meshes[i]->ntris;
		maxMeshes += meshes[i]->nmeshes;
	}
	
	mesh.nmeshes = 0;
	mesh.meshes = (unsigned int*)rcAlloc(sizeof(unsigned int)*maxMeshes*4, RC_ALLOC_PERM);
	if (!mesh.meshes)
	{
		MOSS_ERROR("rcBuildPolyMeshDetail: Out of memory 'pmdtl.meshes' (%d).", maxMeshes*4));
		return false;
	}
	
	mesh.ntris = 0;
	mesh.tris = (unsigned char*)rcAlloc(sizeof(unsigned char)*maxTris*4, RC_ALLOC_PERM);
	if (!mesh.tris) {
		MOSS_ERROR("rcBuildPolyMeshDetail: Out of memory 'dmesh.tris' (%d).", maxTris*4);
		return false;
	}
	
	mesh.nverts = 0;
	mesh.verts = (float*)rcAlloc(sizeof(float)*maxVerts*3, RC_ALLOC_PERM);
	if (!mesh.verts)
	{
		MOSS_ERROR("rcBuildPolyMeshDetail: Out of memory 'dmesh.verts' (%d).", maxVerts*3);
		return false;
	}
	
	// Merge datas.
	for (int i = 0; i < nmeshes; ++i)
	{
		Moss_RecastPolyMeshDetail* dm = meshes[i];
		if (!dm) continue;
		for (int j = 0; j < dm->nmeshes; ++j)
		{
			unsigned int* dst = &mesh.meshes[mesh.nmeshes*4];
			unsigned int* src = &dm->meshes[j*4];
			dst[0] = (unsigned int)mesh.nverts+src[0];
			dst[1] = src[1];
			dst[2] = (unsigned int)mesh.ntris+src[2];
			dst[3] = src[3];
			mesh.nmeshes++;
		}
		
		for (int k = 0; k < dm->nverts; ++k)
		{
			rcVcopy(&mesh.verts[mesh.nverts*3], &dm->verts[k*3]);
			mesh.nverts++;
		}
		for (int k = 0; k < dm->ntris; ++k)
		{
			mesh.tris[mesh.ntris*4+0] = dm->tris[k*4+0];
			mesh.tris[mesh.ntris*4+1] = dm->tris[k*4+1];
			mesh.tris[mesh.ntris*4+2] = dm->tris[k*4+2];
			mesh.tris[mesh.ntris*4+3] = dm->tris[k*4+3];
			mesh.ntris++;
		}
	}
	
	return true;
}


/*													*/

struct rcEdge
{
	unsigned short vert[2];
	unsigned short polyEdge[2];
	unsigned short poly[2];
};

static bool buildMeshAdjacency(unsigned short* polys, const int npolys,
							   const int nverts, const int vertsPerPoly)
{
	// Based on code by Eric Lengyel from:
	// https://web.archive.org/web/20080704083314/http://www.terathon.com/code/edges.php
	
	int maxEdgeCount = npolys*vertsPerPoly;
	unsigned short* firstEdge = (unsigned short*)rcAlloc(sizeof(unsigned short)*(nverts + maxEdgeCount), RC_ALLOC_TEMP);
	if (!firstEdge)
		return false;
	unsigned short* nextEdge = firstEdge + nverts;
	int edgeCount = 0;
	
	rcEdge* edges = (rcEdge*)rcAlloc(sizeof(rcEdge)*maxEdgeCount, RC_ALLOC_TEMP);
	if (!edges)
	{
		MOSS_FREE(firstEdge);
		return false;
	}
	
	for (int i = 0; i < nverts; i++)
		firstEdge[i] = RC_MESH_NULL_IDX;
	
	for (int i = 0; i < npolys; ++i)
	{
		unsigned short* t = &polys[i*vertsPerPoly*2];
		for (int j = 0; j < vertsPerPoly; ++j)
		{
			if (t[j] == RC_MESH_NULL_IDX) break;
			unsigned short v0 = t[j];
			unsigned short v1 = (j+1 >= vertsPerPoly || t[j+1] == RC_MESH_NULL_IDX) ? t[0] : t[j+1];
			if (v0 < v1)
			{
				rcEdge& edge = edges[edgeCount];
				edge.vert[0] = v0;
				edge.vert[1] = v1;
				edge.poly[0] = (unsigned short)i;
				edge.polyEdge[0] = (unsigned short)j;
				edge.poly[1] = (unsigned short)i;
				edge.polyEdge[1] = 0;
				// Insert edge
				nextEdge[edgeCount] = firstEdge[v0];
				firstEdge[v0] = (unsigned short)edgeCount;
				edgeCount++;
			}
		}
	}
	
	for (int i = 0; i < npolys; ++i)
	{
		unsigned short* t = &polys[i*vertsPerPoly*2];
		for (int j = 0; j < vertsPerPoly; ++j)
		{
			if (t[j] == RC_MESH_NULL_IDX) break;
			unsigned short v0 = t[j];
			unsigned short v1 = (j+1 >= vertsPerPoly || t[j+1] == RC_MESH_NULL_IDX) ? t[0] : t[j+1];
			if (v0 > v1)
			{
				for (unsigned short e = firstEdge[v1]; e != RC_MESH_NULL_IDX; e = nextEdge[e])
				{
					rcEdge& edge = edges[e];
					if (edge.vert[1] == v0 && edge.poly[0] == edge.poly[1])
					{
						edge.poly[1] = (unsigned short)i;
						edge.polyEdge[1] = (unsigned short)j;
						break;
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
			unsigned short* p0 = &polys[e.poly[0]*vertsPerPoly*2];
			unsigned short* p1 = &polys[e.poly[1]*vertsPerPoly*2];
			p0[vertsPerPoly + e.polyEdge[0]] = e.poly[1];
			p1[vertsPerPoly + e.polyEdge[1]] = e.poly[0];
		}
	}
	
	MOSS_FREE(firstEdge);
	MOSS_FREE(edges);
	
	return true;
}


static const int VERTEX_BUCKET_COUNT = (1<<12);

inline int computeVertexHash(int x, int y, int z)
{
	const unsigned int h1 = 0x8da6b343; // Large multiplicative constants;
	const unsigned int h2 = 0xd8163841; // here arbitrarily chosen primes
	const unsigned int h3 = 0xcb1ab31f;
	unsigned int n = h1 * x + h2 * y + h3 * z;
	return (int)(n & (VERTEX_BUCKET_COUNT-1));
}

static unsigned short addVertex(unsigned short x, unsigned short y, unsigned short z,
								unsigned short* verts, int* firstVert, int* nextVert, int& nv)
{
	int bucket = computeVertexHash(x, 0, z);
	int i = firstVert[bucket];
	
	while (i != -1)
	{
		const unsigned short* v = &verts[i*3];
		if (v[0] == x && (abs(v[1] - y) <= 2) && v[2] == z)
			return (unsigned short)i;
		i = nextVert[i]; // next
	}
	
	// Could not find, create new.
	i = nv; nv++;
	unsigned short* v = &verts[i*3];
	v[0] = x;
	v[1] = y;
	v[2] = z;
	nextVert[i] = firstVert[bucket];
	firstVert[bucket] = i;
	
	return (unsigned short)i;
}

// Last time I checked the if version got compiled using cmov, which was a lot faster than module (with idiv).
inline int prev(int i, int n) { return i-1 >= 0 ? i-1 : n-1; }
inline int next(int i, int n) { return i+1 < n ? i+1 : 0; }

inline int area2(const int* a, const int* b, const int* c)
{
	return (b[0] - a[0]) * (c[2] - a[2]) - (c[0] - a[0]) * (b[2] - a[2]);
}

// Returns true iff c is strictly to the left of the directed
// line through a to b.
inline bool left(const int* a, const int* b, const int* c)
{
	return area2(a, b, c) < 0;
}

inline bool leftOn(const int* a, const int* b, const int* c)
{
	return area2(a, b, c) <= 0;
}

inline bool collinear(const int* a, const int* b, const int* c)
{
	return area2(a, b, c) == 0;
}

//	Returns true iff ab properly intersects cd: they share
//	a point interior to both segments.  The properness of the
//	intersection is ensured by using strict leftness.
static bool intersectProp(const int* a, const int* b, const int* c, const int* d)
{
	// Eliminate improper cases.
	if (collinear(a,b,c) || collinear(a,b,d) ||
		collinear(c,d,a) || collinear(c,d,b))
		return false;
	
	return (left(a,b,c) ^ left(a,b,d)) && (left(c,d,a) ^ left(c,d,b));
}

// Returns T iff (a,b,c) are collinear and point c lies 
// on the closed segement ab.
static bool between(const int* a, const int* b, const int* c)
{
	if (!collinear(a, b, c))
		return false;
	// If ab not vertical, check betweenness on x; else on y.
	if (a[0] != b[0])
		return	((a[0] <= c[0]) && (c[0] <= b[0])) || ((a[0] >= c[0]) && (c[0] >= b[0]));

	return	((a[2] <= c[2]) && (c[2] <= b[2])) || ((a[2] >= c[2]) && (c[2] >= b[2]));
}

// Returns true iff segments ab and cd intersect, properly or improperly.
static bool intersect(const int* a, const int* b, const int* c, const int* d)
{
	if (intersectProp(a, b, c, d))
		return true;

	if (between(a, b, c) || between(a, b, d) ||
		between(c, d, a) || between(c, d, b))
		return true;
	
	return false;
}

static bool vequal(const int* a, const int* b)
{
	return a[0] == b[0] && a[2] == b[2];
}

// Returns T iff (v_i, v_j) is a proper internal *or* external
// diagonal of P, *ignoring edges incident to v_i and v_j*.
static bool diagonalie(int i, int j, int n, const int* verts, int* indices)
{
	const int* d0 = &verts[(indices[i] & 0x0fffffff) * 4];
	const int* d1 = &verts[(indices[j] & 0x0fffffff) * 4];
	
	// For each edge (k,k+1) of P
	for (int k = 0; k < n; k++)
	{
		int k1 = next(k, n);
		// Skip edges incident to i or j
		if (!((k == i) || (k1 == i) || (k == j) || (k1 == j)))
		{
			const int* p0 = &verts[(indices[k] & 0x0fffffff) * 4];
			const int* p1 = &verts[(indices[k1] & 0x0fffffff) * 4];

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
static bool	inCone(int i, int j, int n, const int* verts, int* indices)
{
	const int* pi = &verts[(indices[i] & 0x0fffffff) * 4];
	const int* pj = &verts[(indices[j] & 0x0fffffff) * 4];
	const int* pi1 = &verts[(indices[next(i, n)] & 0x0fffffff) * 4];
	const int* pin1 = &verts[(indices[prev(i, n)] & 0x0fffffff) * 4];

	// If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
	if (leftOn(pin1, pi, pi1))
		return left(pi, pj, pin1) && left(pj, pi, pi1);
	// Assume (i-1,i,i+1) not collinear.
	// else P[i] is reflex.
	return !(leftOn(pi, pj, pi1) && leftOn(pj, pi, pin1));
}

// Returns T iff (v_i, v_j) is a proper internal
// diagonal of P.
static bool diagonal(int i, int j, int n, const int* verts, int* indices)
{
	return inCone(i, j, n, verts, indices) && diagonalie(i, j, n, verts, indices);
}


static bool diagonalieLoose(int i, int j, int n, const int* verts, int* indices)
{
	const int* d0 = &verts[(indices[i] & 0x0fffffff) * 4];
	const int* d1 = &verts[(indices[j] & 0x0fffffff) * 4];
	
	// For each edge (k,k+1) of P
	for (int k = 0; k < n; k++)
	{
		int k1 = next(k, n);
		// Skip edges incident to i or j
		if (!((k == i) || (k1 == i) || (k == j) || (k1 == j)))
		{
			const int* p0 = &verts[(indices[k] & 0x0fffffff) * 4];
			const int* p1 = &verts[(indices[k1] & 0x0fffffff) * 4];
			
			if (vequal(d0, p0) || vequal(d1, p0) || vequal(d0, p1) || vequal(d1, p1))
				continue;
			
			if (intersectProp(d0, d1, p0, p1))
				return false;
		}
	}
	return true;
}

static bool	inConeLoose(int i, int j, int n, const int* verts, int* indices)
{
	const int* pi = &verts[(indices[i] & 0x0fffffff) * 4];
	const int* pj = &verts[(indices[j] & 0x0fffffff) * 4];
	const int* pi1 = &verts[(indices[next(i, n)] & 0x0fffffff) * 4];
	const int* pin1 = &verts[(indices[prev(i, n)] & 0x0fffffff) * 4];
	
	// If P[i] is a convex vertex [ i+1 left or on (i-1,i) ].
	if (leftOn(pin1, pi, pi1))
		return leftOn(pi, pj, pin1) && leftOn(pj, pi, pi1);
	// Assume (i-1,i,i+1) not collinear.
	// else P[i] is reflex.
	return !(leftOn(pi, pj, pi1) && leftOn(pj, pi, pin1));
}

static bool diagonalLoose(int i, int j, int n, const int* verts, int* indices)
{
	return inConeLoose(i, j, n, verts, indices) && diagonalieLoose(i, j, n, verts, indices);
}


static int triangulate(int n, const int* verts, int* indices, int* tris)
{
	int ntris = 0;
	int* dst = tris;
	
	// The last bit of the index is used to indicate if the vertex can be removed.
	for (int i = 0; i < n; i++)
	{
		int i1 = next(i, n);
		int i2 = next(i1, n);
		if (diagonal(i, i2, n, verts, indices))
			indices[i1] |= 0x80000000;
	}
	
	while (n > 3)
	{
		int minLen = -1;
		int mini = -1;
		for (int i = 0; i < n; i++)
		{
			int i1 = next(i, n);
			if (indices[i1] & 0x80000000)
			{
				const int* p0 = &verts[(indices[i] & 0x0fffffff) * 4];
				const int* p2 = &verts[(indices[next(i1, n)] & 0x0fffffff) * 4];
				
				int dx = p2[0] - p0[0];
				int dy = p2[2] - p0[2];
				int len = dx*dx + dy*dy;
				
				if (minLen < 0 || len < minLen)
				{
					minLen = len;
					mini = i;
				}
			}
		}
		
		if (mini == -1)
		{
			// We might get here because the contour has overlapping segments, like this:
			//
			//  A o-o=====o---o B
			//   /  |C   D|    \.
			//  o   o     o     o
			//  :   :     :     :
			// We'll try to recover by loosing up the inCone test a bit so that a diagonal
			// like A-B or C-D can be found and we can continue.
			minLen = -1;
			mini = -1;
			for (int i = 0; i < n; i++)
			{
				int i1 = next(i, n);
				int i2 = next(i1, n);
				if (diagonalLoose(i, i2, n, verts, indices))
				{
					const int* p0 = &verts[(indices[i] & 0x0fffffff) * 4];
					const int* p2 = &verts[(indices[next(i2, n)] & 0x0fffffff) * 4];
					int dx = p2[0] - p0[0];
					int dy = p2[2] - p0[2];
					int len = dx*dx + dy*dy;
					
					if (minLen < 0 || len < minLen)
					{
						minLen = len;
						mini = i;
					}
				}
			}
			if (mini == -1)
			{
				// The contour is messed up. This sometimes happens
				// if the contour simplification is too aggressive.
				return -ntris;
			}
		}
		
		int i = mini;
		int i1 = next(i, n);
		int i2 = next(i1, n);
		
		*dst++ = indices[i] & 0x0fffffff;
		*dst++ = indices[i1] & 0x0fffffff;
		*dst++ = indices[i2] & 0x0fffffff;
		ntris++;
		
		// Removes P[i1] by copying P[i+1]...P[n-1] left one index.
		n--;
		for (int k = i1; k < n; k++)
			indices[k] = indices[k+1];
		
		if (i1 >= n) i1 = 0;
		i = prev(i1,n);
		// Update diagonal flags.
		if (diagonal(prev(i, n), i1, n, verts, indices))
			indices[i] |= 0x80000000;
		else
			indices[i] &= 0x0fffffff;
		
		if (diagonal(i, next(i1, n), n, verts, indices))
			indices[i1] |= 0x80000000;
		else
			indices[i1] &= 0x0fffffff;
	}
	
	// Append the remaining triangle.
	*dst++ = indices[0] & 0x0fffffff;
	*dst++ = indices[1] & 0x0fffffff;
	*dst++ = indices[2] & 0x0fffffff;
	ntris++;
	
	return ntris;
}

static int countPolyVerts(const unsigned short* p, const int nvp)
{
	for (int i = 0; i < nvp; ++i)
		if (p[i] == RC_MESH_NULL_IDX)
			return i;
	return nvp;
}

inline bool uleft(const unsigned short* a, const unsigned short* b, const unsigned short* c)
{
	return ((int)b[0] - (int)a[0]) * ((int)c[2] - (int)a[2]) -
		   ((int)c[0] - (int)a[0]) * ((int)b[2] - (int)a[2]) < 0;
}

static int getPolyMergeValue(unsigned short* pa, unsigned short* pb,
							 const unsigned short* verts, int& ea, int& eb,
							 const int nvp)
{
	const int na = countPolyVerts(pa, nvp);
	const int nb = countPolyVerts(pb, nvp);
	
	// If the merged polygon would be too big, do not merge.
	if (na+nb-2 > nvp)
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

static void mergePolyVerts(unsigned short* pa, unsigned short* pb, int ea, int eb,
						   unsigned short* tmp, const int nvp)
{
	const int na = countPolyVerts(pa, nvp);
	const int nb = countPolyVerts(pb, nvp);
	
	// Merge polygons.
	memset(tmp, 0xff, sizeof(unsigned short)*nvp);
	int n = 0;
	// Add pa
	for (int i = 0; i < na-1; ++i)
		tmp[n++] = pa[(ea+1+i) % na];
	// Add pb
	for (int i = 0; i < nb-1; ++i)
		tmp[n++] = pb[(eb+1+i) % nb];
	
	memcpy(pa, tmp, sizeof(unsigned short)*nvp);
}


static void pushFront(int v, int* arr, int& an)
{
	an++;
	for (int i = an-1; i > 0; --i) arr[i] = arr[i-1];
	arr[0] = v;
}

static void pushBack(int v, int* arr, int& an)
{
	arr[an] = v;
	an++;
}

static bool canRemoveVertex(rcContext* ctx, Moss_RecastPolyMesh& mesh, const unsigned short rem)
{
	const int nvp = mesh.nvp;
	
	// Count number of polygons to remove.
	int numTouchedVerts = 0;
	int numRemainingEdges = 0;
	for (int i = 0; i < mesh.npolys; ++i)
	{
		unsigned short* p = &mesh.polys[i*nvp*2];
		const int nv = countPolyVerts(p, nvp);
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
	
	// Find edges which share the removed vertex.
	const int maxEdges = numTouchedVerts*2;
	int nedges = 0;
	rcScopedDelete<int> edges((int*)rcAlloc(sizeof(int)*maxEdges*3, RC_ALLOC_TEMP));
	if (!edges)
	{
		ctx->log(RC_LOG_WARNING, "canRemoveVertex: Out of memory 'edges' (%d).", maxEdges*3);
		return false;
	}
		
	for (int i = 0; i < mesh.npolys; ++i)
	{
		unsigned short* p = &mesh.polys[i*nvp*2];
		const int nv = countPolyVerts(p, nvp);

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
					int* e = &edges[m*3];
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
					int* e = &edges[nedges*3];
					e[0] = a;
					e[1] = b;
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

static bool removeVertex(rcContext* ctx, Moss_RecastPolyMesh& mesh, const unsigned short rem, const int maxTris)
{
	const int nvp = mesh.nvp;

	// Count number of polygons to remove.
	int numRemovedVerts = 0;
	for (int i = 0; i < mesh.npolys; ++i)
	{
		unsigned short* p = &mesh.polys[i*nvp*2];
		const int nv = countPolyVerts(p, nvp);
		for (int j = 0; j < nv; ++j)
		{
			if (p[j] == rem)
				numRemovedVerts++;
		}
	}
	
	int nedges = 0;
	rcScopedDelete<int> edges((int*)rcAlloc(sizeof(int)*numRemovedVerts*nvp*4, RC_ALLOC_TEMP));
	if (!edges)
	{
		ctx->log(RC_LOG_WARNING, "removeVertex: Out of memory 'edges' (%d).", numRemovedVerts*nvp*4);
		return false;
	}

	int nhole = 0;
	rcScopedDelete<int> hole((int*)rcAlloc(sizeof(int)*numRemovedVerts*nvp, RC_ALLOC_TEMP));
	if (!hole)
	{
		ctx->log(RC_LOG_WARNING, "removeVertex: Out of memory 'hole' (%d).", numRemovedVerts*nvp);
		return false;
	}

	int nhreg = 0;
	rcScopedDelete<int> hreg((int*)rcAlloc(sizeof(int)*numRemovedVerts*nvp, RC_ALLOC_TEMP));
	if (!hreg)
	{
		ctx->log(RC_LOG_WARNING, "removeVertex: Out of memory 'hreg' (%d).", numRemovedVerts*nvp);
		return false;
	}

	int nharea = 0;
	rcScopedDelete<int> harea((int*)rcAlloc(sizeof(int)*numRemovedVerts*nvp, RC_ALLOC_TEMP));
	if (!harea)
	{
		ctx->log(RC_LOG_WARNING, "removeVertex: Out of memory 'harea' (%d).", numRemovedVerts*nvp);
		return false;
	}
	
	for (int i = 0; i < mesh.npolys; ++i)
	{
		unsigned short* p = &mesh.polys[i*nvp*2];
		const int nv = countPolyVerts(p, nvp);
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
					int* e = &edges[nedges*4];
					e[0] = p[k];
					e[1] = p[j];
					e[2] = mesh.regs[i];
					e[3] = mesh.areas[i];
					nedges++;
				}
			}
			// Remove the polygon.
			unsigned short* p2 = &mesh.polys[(mesh.npolys-1)*nvp*2];
			if (p != p2)
				memcpy(p,p2,sizeof(unsigned short)*nvp);
			memset(p+nvp,0xff,sizeof(unsigned short)*nvp);
			mesh.regs[i] = mesh.regs[mesh.npolys-1];
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
		unsigned short* p = &mesh.polys[i*nvp*2];
		const int nv = countPolyVerts(p, nvp);
		for (int j = 0; j < nv; ++j)
			if (p[j] > rem) p[j]--;
	}
	for (int i = 0; i < nedges; ++i)
	{
		if (edges[i*4+0] > rem) edges[i*4+0]--;
		if (edges[i*4+1] > rem) edges[i*4+1]--;
	}

	if (nedges == 0)
		return true;

	// Start with one vertex, keep appending connected
	// segments to the start and end of the hole.
	pushBack(edges[0], hole, nhole);
	pushBack(edges[2], hreg, nhreg);
	pushBack(edges[3], harea, nharea);
	
	while (nedges)
	{
		bool match = false;
		
		for (int i = 0; i < nedges; ++i)
		{
			const int ea = edges[i*4+0];
			const int eb = edges[i*4+1];
			const int r = edges[i*4+2];
			const int a = edges[i*4+3];
			bool add = false;
			if (hole[0] == eb)
			{
				// The segment matches the beginning of the hole boundary.
				pushFront(ea, hole, nhole);
				pushFront(r, hreg, nhreg);
				pushFront(a, harea, nharea);
				add = true;
			}
			else if (hole[nhole-1] == ea)
			{
				// The segment matches the end of the hole boundary.
				pushBack(eb, hole, nhole);
				pushBack(r, hreg, nhreg);
				pushBack(a, harea, nharea);
				add = true;
			}
			if (add)
			{
				// The edge segment was added, remove it.
				edges[i*4+0] = edges[(nedges-1)*4+0];
				edges[i*4+1] = edges[(nedges-1)*4+1];
				edges[i*4+2] = edges[(nedges-1)*4+2];
				edges[i*4+3] = edges[(nedges-1)*4+3];
				--nedges;
				match = true;
				--i;
			}
		}
		
		if (!match)
			break;
	}

	rcScopedDelete<int> tris((int*)rcAlloc(sizeof(int)*nhole*3, RC_ALLOC_TEMP));
	if (!tris)
	{
		ctx->log(RC_LOG_WARNING, "removeVertex: Out of memory 'tris' (%d).", nhole*3);
		return false;
	}

	rcScopedDelete<int> tverts((int*)rcAlloc(sizeof(int)*nhole*4, RC_ALLOC_TEMP));
	if (!tverts)
	{
		ctx->log(RC_LOG_WARNING, "removeVertex: Out of memory 'tverts' (%d).", nhole*4);
		return false;
	}

	rcScopedDelete<int> thole((int*)rcAlloc(sizeof(int)*nhole, RC_ALLOC_TEMP));
	if (!thole)
	{
		ctx->log(RC_LOG_WARNING, "removeVertex: Out of memory 'thole' (%d).", nhole);
		return false;
	}

	// Generate temp vertex array for triangulation.
	for (int i = 0; i < nhole; ++i)
	{
		const int pi = hole[i];
		tverts[i*4+0] = mesh.verts[pi*3+0];
		tverts[i*4+1] = mesh.verts[pi*3+1];
		tverts[i*4+2] = mesh.verts[pi*3+2];
		tverts[i*4+3] = 0;
		thole[i] = i;
	}

	// Triangulate the hole.
	int ntris = triangulate(nhole, &tverts[0], &thole[0], tris);
	if (ntris < 0)
	{
		ntris = -ntris;
		ctx->log(RC_LOG_WARNING, "removeVertex: triangulate() returned bad results.");
	}
	
	// Merge the hole triangles back to polygons.
	rcScopedDelete<unsigned short> polys((unsigned short*)rcAlloc(sizeof(unsigned short)*(ntris+1)*nvp, RC_ALLOC_TEMP));
	if (!polys)
	{
		MOSS_ERROR("removeVertex: Out of memory 'polys' (%d).", (ntris+1)*nvp);
		return false;
	}
	rcScopedDelete<unsigned short> pregs((unsigned short*)rcAlloc(sizeof(unsigned short)*ntris, RC_ALLOC_TEMP));
	if (!pregs)
	{
		MOSS_ERROR("removeVertex: Out of memory 'pregs' (%d).", ntris);
		return false;
	}
	rcScopedDelete<unsigned char> pareas((unsigned char*)rcAlloc(sizeof(unsigned char)*ntris, RC_ALLOC_TEMP));
	if (!pareas)
	{
		MOSS_ERROR("removeVertex: Out of memory 'pareas' (%d).", ntris);
		return false;
	}
	
	unsigned short* tmpPoly = &polys[ntris*nvp];
			
	// Build initial polygons.
	int npolys = 0;
	memset(polys, 0xff, ntris*nvp*sizeof(unsigned short));
	for (int j = 0; j < ntris; ++j)
	{
		int* t = &tris[j*3];
		if (t[0] != t[1] && t[0] != t[2] && t[1] != t[2])
		{
			polys[npolys*nvp+0] = (unsigned short)hole[t[0]];
			polys[npolys*nvp+1] = (unsigned short)hole[t[1]];
			polys[npolys*nvp+2] = (unsigned short)hole[t[2]];

			// If this polygon covers multiple region types then
			// mark it as such
			if (hreg[t[0]] != hreg[t[1]] || hreg[t[1]] != hreg[t[2]])
				pregs[npolys] = RC_MULTIPLE_REGS;
			else
				pregs[npolys] = (unsigned short)hreg[t[0]];

			pareas[npolys] = (unsigned char)harea[t[0]];
			npolys++;
		}
	}
	if (!npolys)
		return true;
	
	// Merge polygons.
	if (nvp > 3)
	{
		for (;;)
		{
			// Find best polygons to merge.
			int bestMergeVal = 0;
			int bestPa = 0, bestPb = 0, bestEa = 0, bestEb = 0;
			
			for (int j = 0; j < npolys-1; ++j)
			{
				unsigned short* pj = &polys[j*nvp];
				for (int k = j+1; k < npolys; ++k)
				{
					unsigned short* pk = &polys[k*nvp];
					int ea, eb;
					int v = getPolyMergeValue(pj, pk, mesh.verts, ea, eb, nvp);
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
				unsigned short* pa = &polys[bestPa*nvp];
				unsigned short* pb = &polys[bestPb*nvp];
				mergePolyVerts(pa, pb, bestEa, bestEb, tmpPoly, nvp);
				if (pregs[bestPa] != pregs[bestPb])
					pregs[bestPa] = RC_MULTIPLE_REGS;

				unsigned short* last = &polys[(npolys-1)*nvp];
				if (pb != last)
					memcpy(pb, last, sizeof(unsigned short)*nvp);
				pregs[bestPb] = pregs[npolys-1];
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
		unsigned short* p = &mesh.polys[mesh.npolys*nvp*2];
		memset(p,0xff,sizeof(unsigned short)*nvp*2);
		for (int j = 0; j < nvp; ++j)
			p[j] = polys[i*nvp+j];
		mesh.regs[mesh.npolys] = pregs[i];
		mesh.areas[mesh.npolys] = pareas[i];
		mesh.npolys++;
		if (mesh.npolys > maxTris)
		{
			MOSS_ERROR("removeVertex: Too many polygons %d (max:%d).", mesh.npolys, maxTris);
			return false;
		}
	}
	
	return true;
}

/// @par
///
/// @note If the mesh data is to be used to construct a Detour navigation mesh, then the upper 
/// limit must be restricted to <= #DT_VERTS_PER_POLYGON.
///
/// @see rcAllocPolyMesh, Moss_RecastContourSet, Moss_RecastPolyMesh, Moss_RecastSettings3D
bool rcBuildPolyMesh(rcContext* ctx, const Moss_RecastContourSet& cset, const int nvp, Moss_RecastPolyMesh& mesh)
{
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_BUILD_POLYMESH);

	rcVcopy(mesh.bmin, cset.bmin);
	rcVcopy(mesh.bmax, cset.bmax);
	mesh.cs = cset.cs;
	mesh.ch = cset.ch;
	mesh.borderSize = cset.borderSize;
	mesh.maxEdgeError = cset.maxError;
	
	int maxVertices = 0;
	int maxTris = 0;
	int maxVertsPerCont = 0;
	for (int i = 0; i < cset.nconts; ++i)
	{
		// Skip null contours.
		if (cset.conts[i].nverts < 3) continue;
		maxVertices += cset.conts[i].nverts;
		maxTris += cset.conts[i].nverts - 2;
		maxVertsPerCont = max(maxVertsPerCont, cset.conts[i].nverts);
	}
	
	if (maxVertices >= 0xfffe)
	{
		MOSS_ERROR("rcBuildPolyMesh: Too many vertices %d.", maxVertices);
		return false;
	}
		
	rcScopedDelete<unsigned char> vflags((unsigned char*)rcAlloc(sizeof(unsigned char)*maxVertices, RC_ALLOC_TEMP));
	if (!vflags)
	{
		MOSS_ERROR("rcBuildPolyMesh: Out of memory 'vflags' (%d).", maxVertices);
		return false;
	}
	memset(vflags, 0, maxVertices);
	
	mesh.verts = (unsigned short*)rcAlloc(sizeof(unsigned short)*maxVertices*3, RC_ALLOC_PERM);
	if (!mesh.verts)
	{
		MOSS_ERROR("rcBuildPolyMesh: Out of memory 'mesh.verts' (%d).", maxVertices);
		return false;
	}
	mesh.polys = (unsigned short*)rcAlloc(sizeof(unsigned short)*maxTris*nvp*2, RC_ALLOC_PERM);
	if (!mesh.polys)
	{
		MOSS_ERROR("rcBuildPolyMesh: Out of memory 'mesh.polys' (%d).", maxTris*nvp*2);
		return false;
	}
	mesh.regs = (unsigned short*)rcAlloc(sizeof(unsigned short)*maxTris, RC_ALLOC_PERM);
	if (!mesh.regs)
	{
		MOSS_ERROR("rcBuildPolyMesh: Out of memory 'mesh.regs' (%d).", maxTris);
		return false;
	}
	mesh.areas = (unsigned char*)rcAlloc(sizeof(unsigned char)*maxTris, RC_ALLOC_PERM);
	if (!mesh.areas)
	{
		MOSS_ERROR("rcBuildPolyMesh: Out of memory 'mesh.areas' (%d).", maxTris);
		return false;
	}
	
	mesh.nverts = 0;
	mesh.npolys = 0;
	mesh.nvp = nvp;
	mesh.maxpolys = maxTris;
	
	memset(mesh.verts, 0, sizeof(unsigned short)*maxVertices*3);
	memset(mesh.polys, 0xff, sizeof(unsigned short)*maxTris*nvp*2);
	memset(mesh.regs, 0, sizeof(unsigned short)*maxTris);
	memset(mesh.areas, 0, sizeof(unsigned char)*maxTris);
	
	rcScopedDelete<int> nextVert((int*)rcAlloc(sizeof(int)*maxVertices, RC_ALLOC_TEMP));
	if (!nextVert)
	{
		MOSS_ERROR("rcBuildPolyMesh: Out of memory 'nextVert' (%d).", maxVertices);
		return false;
	}
	memset(nextVert, 0, sizeof(int)*maxVertices);
	
	rcScopedDelete<int> firstVert((int*)rcAlloc(sizeof(int)*VERTEX_BUCKET_COUNT, RC_ALLOC_TEMP));
	if (!firstVert)
	{
		MOSS_ERROR("rcBuildPolyMesh: Out of memory 'firstVert' (%d).", VERTEX_BUCKET_COUNT);
		return false;
	}
	for (int i = 0; i < VERTEX_BUCKET_COUNT; ++i)
		firstVert[i] = -1;
	
	rcScopedDelete<int> indices((int*)rcAlloc(sizeof(int)*maxVertsPerCont, RC_ALLOC_TEMP));
	if (!indices)
	{
		MOSS_ERROR("rcBuildPolyMesh: Out of memory 'indices' (%d).", maxVertsPerCont);
		return false;
	}
	rcScopedDelete<int> tris((int*)rcAlloc(sizeof(int)*maxVertsPerCont*3, RC_ALLOC_TEMP));
	if (!tris)
	{
		MOSS_ERROR("rcBuildPolyMesh: Out of memory 'tris' (%d).", maxVertsPerCont*3);
		return false;
	}
	rcScopedDelete<unsigned short> polys((unsigned short*)rcAlloc(sizeof(unsigned short)*(maxVertsPerCont+1)*nvp, RC_ALLOC_TEMP));
	if (!polys)
	{
		MOSS_ERROR("rcBuildPolyMesh: Out of memory 'polys' (%d).", maxVertsPerCont*nvp);
		return false;
	}
	unsigned short* tmpPoly = &polys[maxVertsPerCont*nvp];

	for (int i = 0; i < cset.nconts; ++i)
	{
		Moss_RecastContour& cont = cset.conts[i];
		
		// Skip null contours.
		if (cont.nverts < 3)
			continue;
		
		// Triangulate contour
		for (int j = 0; j < cont.nverts; ++j)
			indices[j] = j;
			
		int ntris = triangulate(cont.nverts, cont.verts, &indices[0], &tris[0]);
		if (ntris <= 0)
		{
			// Bad triangulation, should not happen.
/*			printf("\tconst float bmin[3] = {%ff,%ff,%ff};\n", cset.bmin[0], cset.bmin[1], cset.bmin[2]);
			printf("\tconst float cs = %ff;\n", cset.cs);
			printf("\tconst float ch = %ff;\n", cset.ch);
			printf("\tconst int verts[] = {\n");
			for (int k = 0; k < cont.nverts; ++k)
			{
				const int* v = &cont.verts[k*4];
				printf("\t\t%d,%d,%d,%d,\n", v[0], v[1], v[2], v[3]);
			}
			printf("\t};\n\tconst int nverts = sizeof(verts)/(sizeof(int)*4);\n");*/
			ctx->log(RC_LOG_WARNING, "rcBuildPolyMesh: Bad triangulation Contour %d.", i);
			ntris = -ntris;
		}
				
		// Add and merge vertices.
		for (int j = 0; j < cont.nverts; ++j)
		{
			const int* v = &cont.verts[j*4];
			indices[j] = addVertex((unsigned short)v[0], (unsigned short)v[1], (unsigned short)v[2],
								   mesh.verts, firstVert, nextVert, mesh.nverts);
			if (v[3] & RC_BORDER_VERTEX)
			{
				// This vertex should be removed.
				vflags[indices[j]] = 1;
			}
		}

		// Build initial polygons.
		int npolys = 0;
		memset(polys, 0xff, maxVertsPerCont*nvp*sizeof(unsigned short));
		for (int j = 0; j < ntris; ++j)
		{
			int* t = &tris[j*3];
			if (t[0] != t[1] && t[0] != t[2] && t[1] != t[2])
			{
				polys[npolys*nvp+0] = (unsigned short)indices[t[0]];
				polys[npolys*nvp+1] = (unsigned short)indices[t[1]];
				polys[npolys*nvp+2] = (unsigned short)indices[t[2]];
				npolys++;
			}
		}
		if (!npolys)
			continue;
		
		// Merge polygons.
		if (nvp > 3)
		{
			for(;;)
			{
				// Find best polygons to merge.
				int bestMergeVal = 0;
				int bestPa = 0, bestPb = 0, bestEa = 0, bestEb = 0;
				
				for (int j = 0; j < npolys-1; ++j)
				{
					unsigned short* pj = &polys[j*nvp];
					for (int k = j+1; k < npolys; ++k)
					{
						unsigned short* pk = &polys[k*nvp];
						int ea, eb;
						int v = getPolyMergeValue(pj, pk, mesh.verts, ea, eb, nvp);
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
					unsigned short* pa = &polys[bestPa*nvp];
					unsigned short* pb = &polys[bestPb*nvp];
					mergePolyVerts(pa, pb, bestEa, bestEb, tmpPoly, nvp);
					unsigned short* lastPoly = &polys[(npolys-1)*nvp];
					if (pb != lastPoly)
						memcpy(pb, lastPoly, sizeof(unsigned short)*nvp);
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
			unsigned short* p = &mesh.polys[mesh.npolys*nvp*2];
			unsigned short* q = &polys[j*nvp];
			for (int k = 0; k < nvp; ++k)
				p[k] = q[k];
			mesh.regs[mesh.npolys] = cont.reg;
			mesh.areas[mesh.npolys] = cont.area;
			mesh.npolys++;
			if (mesh.npolys > maxTris)
			{
				MOSS_ERROR("rcBuildPolyMesh: Too many polygons %d (max:%d).", mesh.npolys, maxTris);
				return false;
			}
		}
	}
	
	
	// Remove edge vertices.
	for (int i = 0; i < mesh.nverts; ++i)
	{
		if (vflags[i])
		{
			if (!canRemoveVertex(ctx, mesh, (unsigned short)i))
				continue;
			if (!removeVertex(ctx, mesh, (unsigned short)i, maxTris))
			{
				// Failed to remove vertex
				MOSS_ERROR("rcBuildPolyMesh: Failed to remove edge vertex %d.", i);
				return false;
			}
			// Remove vertex
			// Note: mesh.nverts is already decremented inside removeVertex()!
			// Fixup vertex flags
			for (int j = i; j < mesh.nverts; ++j)
				vflags[j] = vflags[j+1];
			--i;
		}
	}
	
	// Calculate adjacency.
	if (!buildMeshAdjacency(mesh.polys, mesh.npolys, mesh.nverts, nvp))
	{
		MOSS_ERROR("rcBuildPolyMesh: Adjacency failed.");
		return false;
	}
	
	// Find portal edges
	if (mesh.borderSize > 0)
	{
		const int w = cset.width;
		const int h = cset.height;
		for (int i = 0; i < mesh.npolys; ++i)
		{
			unsigned short* p = &mesh.polys[i*2*nvp];
			for (int j = 0; j < nvp; ++j)
			{
				if (p[j] == RC_MESH_NULL_IDX) break;
				// Skip connected edges.
				if (p[nvp+j] != RC_MESH_NULL_IDX)
					continue;
				int nj = j+1;
				if (nj >= nvp || p[nj] == RC_MESH_NULL_IDX) nj = 0;
				const unsigned short* va = &mesh.verts[p[j]*3];
				const unsigned short* vb = &mesh.verts[p[nj]*3];

				if ((int)va[0] == 0 && (int)vb[0] == 0)
					p[nvp+j] = 0x8000 | 0;
				else if ((int)va[2] == h && (int)vb[2] == h)
					p[nvp+j] = 0x8000 | 1;
				else if ((int)va[0] == w && (int)vb[0] == w)
					p[nvp+j] = 0x8000 | 2;
				else if ((int)va[2] == 0 && (int)vb[2] == 0)
					p[nvp+j] = 0x8000 | 3;
			}
		}
	}

	// Just allocate the mesh flags array. The user is resposible to fill it.
	mesh.flags = (unsigned short*)rcAlloc(sizeof(unsigned short)*mesh.npolys, RC_ALLOC_PERM);
	if (!mesh.flags)
	{
		MOSS_ERROR("rcBuildPolyMesh: Out of memory 'mesh.flags' (%d).", mesh.npolys);
		return false;
	}
	memset(mesh.flags, 0, sizeof(unsigned short) * mesh.npolys);
	
	if (mesh.nverts > 0xffff)
	{
		MOSS_ERROR("rcBuildPolyMesh: The resulting mesh has too many vertices %d (max %d). Data can be corrupted.", mesh.nverts, 0xffff);
	}
	if (mesh.npolys > 0xffff)
	{
		MOSS_ERROR("rcBuildPolyMesh: The resulting mesh has too many polygons %d (max %d). Data can be corrupted.", mesh.npolys, 0xffff);
	}
	
	return true;
}

/// @see rcAllocPolyMesh, Moss_RecastPolyMesh
bool rcMergePolyMeshes(rcContext* ctx, Moss_RecastPolyMesh** meshes, const int nmeshes, Moss_RecastPolyMesh& mesh)
{
	rcAssert(ctx);
	
	if (!nmeshes || !meshes)
		return true;

	rcScopedTimer timer(ctx, RC_TIMER_MERGE_POLYMESH);

	mesh.nvp = meshes[0]->nvp;
	mesh.cs = meshes[0]->cs;
	mesh.ch = meshes[0]->ch;
	rcVcopy(mesh.bmin, meshes[0]->bmin);
	rcVcopy(mesh.bmax, meshes[0]->bmax);

	int maxVerts = 0;
	int maxPolys = 0;
	int maxVertsPerMesh = 0;
	for (int i = 0; i < nmeshes; ++i)
	{
		rcVmin(mesh.bmin, meshes[i]->bmin);
		rcVmax(mesh.bmax, meshes[i]->bmax);
		maxVertsPerMesh = max(maxVertsPerMesh, meshes[i]->nverts);
		maxVerts += meshes[i]->nverts;
		maxPolys += meshes[i]->npolys;
	}
	
	mesh.nverts = 0;
	mesh.verts = (unsigned short*)rcAlloc(sizeof(unsigned short)*maxVerts*3, RC_ALLOC_PERM);
	if (!mesh.verts)
	{
		MOSS_ERROR("rcMergePolyMeshes: Out of memory 'mesh.verts' (%d).", maxVerts*3);
		return false;
	}

	mesh.npolys = 0;
	mesh.polys = (unsigned short*)rcAlloc(sizeof(unsigned short)*maxPolys*2*mesh.nvp, RC_ALLOC_PERM);
	if (!mesh.polys)
	{
		MOSS_ERROR("rcMergePolyMeshes: Out of memory 'mesh.polys' (%d).", maxPolys*2*mesh.nvp);
		return false;
	}
	memset(mesh.polys, 0xff, sizeof(unsigned short)*maxPolys*2*mesh.nvp);

	mesh.regs = (unsigned short*)rcAlloc(sizeof(unsigned short)*maxPolys, RC_ALLOC_PERM);
	if (!mesh.regs)
	{
		MOSS_ERROR("rcMergePolyMeshes: Out of memory 'mesh.regs' (%d).", maxPolys);
		return false;
	}
	memset(mesh.regs, 0, sizeof(unsigned short)*maxPolys);

	mesh.areas = (unsigned char*)rcAlloc(sizeof(unsigned char)*maxPolys, RC_ALLOC_PERM);
	if (!mesh.areas)
	{
		MOSS_ERROR("rcMergePolyMeshes: Out of memory 'mesh.areas' (%d).", maxPolys);
		return false;
	}
	memset(mesh.areas, 0, sizeof(unsigned char)*maxPolys);

	mesh.flags = (unsigned short*)rcAlloc(sizeof(unsigned short)*maxPolys, RC_ALLOC_PERM);
	if (!mesh.flags)
	{
		MOSS_ERROR("rcMergePolyMeshes: Out of memory 'mesh.flags' (%d).", maxPolys);
		return false;
	}
	memset(mesh.flags, 0, sizeof(unsigned short)*maxPolys);
	
	rcScopedDelete<int> nextVert((int*)rcAlloc(sizeof(int)*maxVerts, RC_ALLOC_TEMP));
	if (!nextVert)
	{
		MOSS_ERROR("rcMergePolyMeshes: Out of memory 'nextVert' (%d).", maxVerts);
		return false;
	}
	memset(nextVert, 0, sizeof(int)*maxVerts);
	
	rcScopedDelete<int> firstVert((int*)rcAlloc(sizeof(int)*VERTEX_BUCKET_COUNT, RC_ALLOC_TEMP));
	if (!firstVert)
	{
		MOSS_ERROR("rcMergePolyMeshes: Out of memory 'firstVert' (%d).", VERTEX_BUCKET_COUNT);
		return false;
	}
	for (int i = 0; i < VERTEX_BUCKET_COUNT; ++i)
		firstVert[i] = -1;

	rcScopedDelete<unsigned short> vremap((unsigned short*)rcAlloc(sizeof(unsigned short)*maxVertsPerMesh, RC_ALLOC_PERM));
	if (!vremap)
	{
		MOSS_ERROR("rcMergePolyMeshes: Out of memory 'vremap' (%d).", maxVertsPerMesh);
		return false;
	}
	memset(vremap, 0, sizeof(unsigned short)*maxVertsPerMesh);
	
	for (int i = 0; i < nmeshes; ++i)
	{
		const Moss_RecastPolyMesh* pmesh = meshes[i];
		
		const unsigned short ox = (unsigned short)floorf((pmesh->bmin[0]-mesh.bmin[0])/mesh.cs+0.5f);
		const unsigned short oz = (unsigned short)floorf((pmesh->bmin[2]-mesh.bmin[2])/mesh.cs+0.5f);
		
		bool isMinX = (ox == 0);
		bool isMinZ = (oz == 0);
		bool isMaxX = ((unsigned short)floorf((mesh.bmax[0] - pmesh->bmax[0]) / mesh.cs + 0.5f)) == 0;
		bool isMaxZ = ((unsigned short)floorf((mesh.bmax[2] - pmesh->bmax[2]) / mesh.cs + 0.5f)) == 0;
		bool isOnBorder = (isMinX || isMinZ || isMaxX || isMaxZ);

		for (int j = 0; j < pmesh->nverts; ++j)
		{
			unsigned short* v = &pmesh->verts[j*3];
			vremap[j] = addVertex(v[0]+ox, v[1], v[2]+oz,
								  mesh.verts, firstVert, nextVert, mesh.nverts);
		}
		
		for (int j = 0; j < pmesh->npolys; ++j)
		{
			unsigned short* tgt = &mesh.polys[mesh.npolys*2*mesh.nvp];
			unsigned short* src = &pmesh->polys[j*2*mesh.nvp];
			mesh.regs[mesh.npolys] = pmesh->regs[j];
			mesh.areas[mesh.npolys] = pmesh->areas[j];
			mesh.flags[mesh.npolys] = pmesh->flags[j];
			mesh.npolys++;
			for (int k = 0; k < mesh.nvp; ++k)
			{
				if (src[k] == RC_MESH_NULL_IDX) break;
				tgt[k] = vremap[src[k]];
			}

			if (isOnBorder)
			{
				for (int k = mesh.nvp; k < mesh.nvp * 2; ++k)
				{
					if (src[k] & 0x8000 && src[k] != 0xffff)
					{
						unsigned short dir = src[k] & 0xf;
						switch (dir)
						{
							case 0: // Portal x-
								if (isMinX)
									tgt[k] = src[k];
								break;
							case 1: // Portal z+
								if (isMaxZ)
									tgt[k] = src[k];
								break;
							case 2: // Portal x+
								if (isMaxX)
									tgt[k] = src[k];
								break;
							case 3: // Portal z-
								if (isMinZ)
									tgt[k] = src[k];
								break;
						}
					}
				}
			}
		}
	}

	// Calculate adjacency.
	if (!buildMeshAdjacency(mesh.polys, mesh.npolys, mesh.nverts, mesh.nvp))
	{
		MOSS_ERROR("rcMergePolyMeshes: Adjacency failed.");
		return false;
	}

	if (mesh.nverts > 0xffff)
	{
		MOSS_ERROR("rcMergePolyMeshes: The resulting mesh has too many vertices %d (max %d). Data can be corrupted.", mesh.nverts, 0xffff);
	}
	if (mesh.npolys > 0xffff)
	{
		MOSS_ERROR("rcMergePolyMeshes: The resulting mesh has too many polygons %d (max %d). Data can be corrupted.", mesh.npolys, 0xffff);
	}
	
	return true;
}

bool rcCopyPolyMesh(rcContext* ctx, const Moss_RecastPolyMesh& src, Moss_RecastPolyMesh& dst)
{
	rcAssert(ctx);
	
	// Destination must be empty.
	rcAssert(dst.verts == 0);
	rcAssert(dst.polys == 0);
	rcAssert(dst.regs == 0);
	rcAssert(dst.areas == 0);
	rcAssert(dst.flags == 0);
	
	dst.nverts = src.nverts;
	dst.npolys = src.npolys;
	dst.maxpolys = src.npolys;
	dst.nvp = src.nvp;
	rcVcopy(dst.bmin, src.bmin);
	rcVcopy(dst.bmax, src.bmax);
	dst.cs = src.cs;
	dst.ch = src.ch;
	dst.borderSize = src.borderSize;
	dst.maxEdgeError = src.maxEdgeError;
	
	dst.verts = (unsigned short*)rcAlloc(sizeof(unsigned short)*src.nverts*3, RC_ALLOC_PERM);
	if (!dst.verts)
	{
		MOSS_ERROR("rcCopyPolyMesh: Out of memory 'dst.verts' (%d).", src.nverts*3);
		return false;
	}
	memcpy(dst.verts, src.verts, sizeof(unsigned short)*src.nverts*3);
	
	dst.polys = (unsigned short*)rcAlloc(sizeof(unsigned short)*src.npolys*2*src.nvp, RC_ALLOC_PERM);
	if (!dst.polys)
	{
		MOSS_ERROR("rcCopyPolyMesh: Out of memory 'dst.polys' (%d).", src.npolys*2*src.nvp);
		return false;
	}
	memcpy(dst.polys, src.polys, sizeof(unsigned short)*src.npolys*2*src.nvp);
	
	dst.regs = (unsigned short*)rcAlloc(sizeof(unsigned short)*src.npolys, RC_ALLOC_PERM);
	if (!dst.regs)
	{
		MOSS_ERROR("rcCopyPolyMesh: Out of memory 'dst.regs' (%d).", src.npolys);
		return false;
	}
	memcpy(dst.regs, src.regs, sizeof(unsigned short)*src.npolys);
	
	dst.areas = (unsigned char*)rcAlloc(sizeof(unsigned char)*src.npolys, RC_ALLOC_PERM);
	if (!dst.areas)
	{
		MOSS_ERROR("rcCopyPolyMesh: Out of memory 'dst.areas' (%d).", src.npolys);
		return false;
	}
	memcpy(dst.areas, src.areas, sizeof(unsigned char)*src.npolys);
	
	dst.flags = (unsigned short*)rcAlloc(sizeof(unsigned short)*src.npolys, RC_ALLOC_PERM);
	if (!dst.flags)
	{
		MOSS_ERROR("rcCopyPolyMesh: Out of memory 'dst.flags' (%d).", src.npolys);
		return false;
	}
	memcpy(dst.flags, src.flags, sizeof(unsigned short)*src.npolys);
	
	return true;
}


/*													*/

namespace
{
struct LevelStackEntry
{
	LevelStackEntry(int x_, int y_, int index_) : x(x_), y(y_), index(index_) {}
	int x;
	int y;
	int index;
};
}  // namespace

static void calculateDistanceField(Moss_RecastCompactHeightfield& chf, unsigned short* src, unsigned short& maxDist)
{
	const int w = chf.width;
	const int h = chf.height;
	
	// Init distance and points.
	for (int i = 0; i < chf.spanCount; ++i)
		src[i] = 0xffff;
	
	// Mark boundary cells.
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const Moss_RecastCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const Moss_RecastCompactSpan& s = chf.spans[i];
				const unsigned char area = chf.areas[i];
				
				int nc = 0;
				for (int dir = 0; dir < 4; ++dir)
				{
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);
						if (area == chf.areas[ai])
							nc++;
					}
				}
				if (nc != 4)
					src[i] = 0;
			}
		}
	}
	
			
	// Pass 1
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const Moss_RecastCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const Moss_RecastCompactSpan& s = chf.spans[i];
				
				if (rcGetCon(s, 0) != RC_NOT_CONNECTED)
				{
					// (-1,0)
					const int ax = x + rcGetDirOffsetX(0);
					const int ay = y + rcGetDirOffsetY(0);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 0);
					const Moss_RecastCompactSpan& as = chf.spans[ai];
					if (src[ai]+2 < src[i])
						src[i] = src[ai]+2;
					
					// (-1,-1)
					if (rcGetCon(as, 3) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(3);
						const int aay = ay + rcGetDirOffsetY(3);
						const int aai = (int)chf.cells[aax+aay*w].index + rcGetCon(as, 3);
						if (src[aai]+3 < src[i])
							src[i] = src[aai]+3;
					}
				}
				if (rcGetCon(s, 3) != RC_NOT_CONNECTED)
				{
					// (0,-1)
					const int ax = x + rcGetDirOffsetX(3);
					const int ay = y + rcGetDirOffsetY(3);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 3);
					const Moss_RecastCompactSpan& as = chf.spans[ai];
					if (src[ai]+2 < src[i])
						src[i] = src[ai]+2;
					
					// (1,-1)
					if (rcGetCon(as, 2) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(2);
						const int aay = ay + rcGetDirOffsetY(2);
						const int aai = (int)chf.cells[aax+aay*w].index + rcGetCon(as, 2);
						if (src[aai]+3 < src[i])
							src[i] = src[aai]+3;
					}
				}
			}
		}
	}
	
	// Pass 2
	for (int y = h-1; y >= 0; --y)
	{
		for (int x = w-1; x >= 0; --x)
		{
			const Moss_RecastCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const Moss_RecastCompactSpan& s = chf.spans[i];
				
				if (rcGetCon(s, 2) != RC_NOT_CONNECTED)
				{
					// (1,0)
					const int ax = x + rcGetDirOffsetX(2);
					const int ay = y + rcGetDirOffsetY(2);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 2);
					const Moss_RecastCompactSpan& as = chf.spans[ai];
					if (src[ai]+2 < src[i])
						src[i] = src[ai]+2;
					
					// (1,1)
					if (rcGetCon(as, 1) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(1);
						const int aay = ay + rcGetDirOffsetY(1);
						const int aai = (int)chf.cells[aax+aay*w].index + rcGetCon(as, 1);
						if (src[aai]+3 < src[i])
							src[i] = src[aai]+3;
					}
				}
				if (rcGetCon(s, 1) != RC_NOT_CONNECTED)
				{
					// (0,1)
					const int ax = x + rcGetDirOffsetX(1);
					const int ay = y + rcGetDirOffsetY(1);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 1);
					const Moss_RecastCompactSpan& as = chf.spans[ai];
					if (src[ai]+2 < src[i])
						src[i] = src[ai]+2;
					
					// (-1,1)
					if (rcGetCon(as, 0) != RC_NOT_CONNECTED)
					{
						const int aax = ax + rcGetDirOffsetX(0);
						const int aay = ay + rcGetDirOffsetY(0);
						const int aai = (int)chf.cells[aax+aay*w].index + rcGetCon(as, 0);
						if (src[aai]+3 < src[i])
							src[i] = src[aai]+3;
					}
				}
			}
		}
	}	
	
	maxDist = 0;
	for (int i = 0; i < chf.spanCount; ++i)
		maxDist = max(src[i], maxDist);
	
}

static unsigned short* boxBlur(Moss_RecastCompactHeightfield& chf, int thr, unsigned short* src, unsigned short* dst) {
	const int w = chf.width;
	const int h = chf.height;
	
	thr *= 2;
	
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const Moss_RecastCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const Moss_RecastCompactSpan& s = chf.spans[i];
				const unsigned short cd = src[i];
				if (cd <= thr)
				{
					dst[i] = cd;
					continue;
				}

				int d = (int)cd;
				for (int dir = 0; dir < 4; ++dir)
				{
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);
						d += (int)src[ai];
						
						const Moss_RecastCompactSpan& as = chf.spans[ai];
						const int dir2 = (dir+1) & 0x3;
						if (rcGetCon(as, dir2) != RC_NOT_CONNECTED)
						{
							const int ax2 = ax + rcGetDirOffsetX(dir2);
							const int ay2 = ay + rcGetDirOffsetY(dir2);
							const int ai2 = (int)chf.cells[ax2+ay2*w].index + rcGetCon(as, dir2);
							d += (int)src[ai2];
						}
						else
						{
							d += cd;
						}
					}
					else
					{
						d += cd*2;
					}
				}
				dst[i] = (unsigned short)((d+5)/9);
			}
		}
	}
	return dst;
}


static bool floodRegion(int x, int y, int i,
						unsigned short level, unsigned short r,
						Moss_RecastCompactHeightfield& chf,
						unsigned short* srcReg, unsigned short* srcDist,
						rcTempVector<LevelStackEntry>& stack)
{
	const int w = chf.width;
	
	const unsigned char area = chf.areas[i];
	
	// Flood fill mark region.
	stack.clear();
	stack.push_back(LevelStackEntry(x, y, i));
	srcReg[i] = r;
	srcDist[i] = 0;
	
	unsigned short lev = level >= 2 ? level-2 : 0;
	int count = 0;
	
	while (stack.size() > 0)
	{
		LevelStackEntry& back = stack.back();
		int cx = back.x;
		int cy = back.y;
		int ci = back.index;
		stack.pop_back();
		
		const Moss_RecastCompactSpan& cs = chf.spans[ci];
		
		// Check if any of the neighbours already have a valid region set.
		unsigned short ar = 0;
		for (int dir = 0; dir < 4; ++dir)
		{
			// 8 connected
			if (rcGetCon(cs, dir) != RC_NOT_CONNECTED)
			{
				const int ax = cx + rcGetDirOffsetX(dir);
				const int ay = cy + rcGetDirOffsetY(dir);
				const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(cs, dir);
				if (chf.areas[ai] != area)
					continue;
				unsigned short nr = srcReg[ai];
				if (nr & RC_BORDER_REG) // Do not take borders into account.
					continue;
				if (nr != 0 && nr != r)
				{
					ar = nr;
					break;
				}
				
				const Moss_RecastCompactSpan& as = chf.spans[ai];
				
				const int dir2 = (dir+1) & 0x3;
				if (rcGetCon(as, dir2) != RC_NOT_CONNECTED)
				{
					const int ax2 = ax + rcGetDirOffsetX(dir2);
					const int ay2 = ay + rcGetDirOffsetY(dir2);
					const int ai2 = (int)chf.cells[ax2+ay2*w].index + rcGetCon(as, dir2);
					if (chf.areas[ai2] != area)
						continue;
					unsigned short nr2 = srcReg[ai2];
					if (nr2 != 0 && nr2 != r)
					{
						ar = nr2;
						break;
					}
				}				
			}
		}
		if (ar != 0)
		{
			srcReg[ci] = 0;
			continue;
		}
		
		count++;
		
		// Expand neighbours.
		for (int dir = 0; dir < 4; ++dir)
		{
			if (rcGetCon(cs, dir) != RC_NOT_CONNECTED)
			{
				const int ax = cx + rcGetDirOffsetX(dir);
				const int ay = cy + rcGetDirOffsetY(dir);
				const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(cs, dir);
				if (chf.areas[ai] != area)
					continue;
				if (chf.dist[ai] >= lev && srcReg[ai] == 0)
				{
					srcReg[ai] = r;
					srcDist[ai] = 0;
					stack.push_back(LevelStackEntry(ax, ay, ai));
				}
			}
		}
	}
	
	return count > 0;
}

// Struct to keep track of entries in the region table that have been changed.
struct DirtyEntry
{
	DirtyEntry(int index_, unsigned short region_, unsigned short distance2_)
		: index(index_), region(region_), distance2(distance2_) {}
	int index;
	unsigned short region;
	unsigned short distance2;
};
static void expandRegions(int maxIter, unsigned short level,
					      Moss_RecastCompactHeightfield& chf,
					      unsigned short* srcReg, unsigned short* srcDist,
					      rcTempVector<LevelStackEntry>& stack,
					      bool fillStack)
{
	const int w = chf.width;
	const int h = chf.height;

	if (fillStack)
	{
		// Find cells revealed by the raised level.
		stack.clear();
		for (int y = 0; y < h; ++y)
		{
			for (int x = 0; x < w; ++x)
			{
				const Moss_RecastCompactCell& c = chf.cells[x+y*w];
				for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
				{
					if (chf.dist[i] >= level && srcReg[i] == 0 && chf.areas[i] != RC_NULL_AREA)
					{
						stack.push_back(LevelStackEntry(x, y, i));
					}
				}
			}
		}
	}
	else // use cells in the input stack
	{
		// mark all cells which already have a region
		for (int j=0; j<stack.size(); j++)
		{
			int i = stack[j].index;
			if (srcReg[i] != 0)
				stack[j].index = -1;
		}
	}

	rcTempVector<DirtyEntry> dirtyEntries;
	int iter = 0;
	while (stack.size() > 0)
	{
		int failed = 0;
		dirtyEntries.clear();
		
		for (int j = 0; j < stack.size(); j++)
		{
			int x = stack[j].x;
			int y = stack[j].y;
			int i = stack[j].index;
			if (i < 0)
			{
				failed++;
				continue;
			}
			
			unsigned short r = srcReg[i];
			unsigned short d2 = 0xffff;
			const unsigned char area = chf.areas[i];
			const Moss_RecastCompactSpan& s = chf.spans[i];
			for (int dir = 0; dir < 4; ++dir)
			{
				if (rcGetCon(s, dir) == RC_NOT_CONNECTED) continue;
				const int ax = x + rcGetDirOffsetX(dir);
				const int ay = y + rcGetDirOffsetY(dir);
				const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);
				if (chf.areas[ai] != area) continue;
				if (srcReg[ai] > 0 && (srcReg[ai] & RC_BORDER_REG) == 0)
				{
					if ((int)srcDist[ai]+2 < (int)d2)
					{
						r = srcReg[ai];
						d2 = srcDist[ai]+2;
					}
				}
			}
			if (r)
			{
				stack[j].index = -1; // mark as used
				dirtyEntries.push_back(DirtyEntry(i, r, d2));
			}
			else
			{
				failed++;
			}
		}
		
		// Copy entries that differ between src and dst to keep them in sync.
		for (int i = 0; i < dirtyEntries.size(); i++) {
			int idx = dirtyEntries[i].index;
			srcReg[idx] = dirtyEntries[i].region;
			srcDist[idx] = dirtyEntries[i].distance2;
		}
		
		if (failed == stack.size())
			break;
		
		if (level > 0)
		{
			++iter;
			if (iter >= maxIter)
				break;
		}
	}
}



static void sortCellsByLevel(unsigned short startLevel,
							  Moss_RecastCompactHeightfield& chf,
							  const unsigned short* srcReg,
							  unsigned int nbStacks, rcTempVector<LevelStackEntry>* stacks,
							  unsigned short loglevelsPerStack) // the levels per stack (2 in our case) as a bit shift
{
	const int w = chf.width;
	const int h = chf.height;
	startLevel = startLevel >> loglevelsPerStack;

	for (unsigned int j=0; j<nbStacks; ++j)
		stacks[j].clear();

	// put all cells in the level range into the appropriate stacks
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const Moss_RecastCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				if (chf.areas[i] == RC_NULL_AREA || srcReg[i] != 0)
					continue;

				int level = chf.dist[i] >> loglevelsPerStack;
				int sId = startLevel - level;
				if (sId >= (int)nbStacks)
					continue;
				if (sId < 0)
					sId = 0;

				stacks[sId].push_back(LevelStackEntry(x, y, i));
			}
		}
	}
}


static void appendStacks(const rcTempVector<LevelStackEntry>& srcStack,
						 rcTempVector<LevelStackEntry>& dstStack,
						 const unsigned short* srcReg)
{
	for (int j=0; j<srcStack.size(); j++)
	{
		int i = srcStack[j].index;
		if ((i < 0) || (srcReg[i] != 0))
			continue;
		dstStack.push_back(srcStack[j]);
	}
}

struct rcRegion
{
	inline rcRegion(unsigned short i) :
		spanCount(0),
		id(i),
		areaType(0),
		remap(false),
		visited(false),
		overlap(false),
		connectsToBorder(false),
		ymin(0xffff),
		ymax(0)
	{}
	
	int spanCount;					// Number of spans belonging to this region
	unsigned short id;				// ID of the region
	unsigned char areaType;			// Are type.
	bool remap;
	bool visited;
	bool overlap;
	bool connectsToBorder;
	unsigned short ymin, ymax;
	rcTempVector<int> connections;
	rcTempVector<int> floors;
};

static void removeAdjacentNeighbours(rcRegion& reg)
{
	// Remove adjacent duplicates.
	for (int i = 0; i < reg.connections.size() && reg.connections.size() > 1; )
	{
		int ni = (i+1) % reg.connections.size();
		if (reg.connections[i] == reg.connections[ni])
		{
			// Remove duplicate
			for (int j = i; j < reg.connections.size()-1; ++j)
				reg.connections[j] = reg.connections[j+1];
			reg.connections.pop_back();
		}
		else
			++i;
	}
}

static void replaceNeighbour(rcRegion& reg, unsigned short oldId, unsigned short newId)
{
	bool neiChanged = false;
	for (int i = 0; i < reg.connections.size(); ++i)
	{
		if (reg.connections[i] == oldId)
		{
			reg.connections[i] = newId;
			neiChanged = true;
		}
	}
	for (int i = 0; i < reg.floors.size(); ++i)
	{
		if (reg.floors[i] == oldId)
			reg.floors[i] = newId;
	}
	if (neiChanged)
		removeAdjacentNeighbours(reg);
}

static bool canMergeWithRegion(const rcRegion& rega, const rcRegion& regb)
{
	if (rega.areaType != regb.areaType)
		return false;
	int n = 0;
	for (int i = 0; i < rega.connections.size(); ++i)
	{
		if (rega.connections[i] == regb.id)
			n++;
	}
	if (n > 1)
		return false;
	for (int i = 0; i < rega.floors.size(); ++i)
	{
		if (rega.floors[i] == regb.id)
			return false;
	}
	return true;
}

static void addUniqueFloorRegion(rcRegion& reg, int n)
{
	for (int i = 0; i < reg.floors.size(); ++i)
		if (reg.floors[i] == n)
			return;
	reg.floors.push_back(n);
}

static bool mergeRegions(rcRegion& rega, rcRegion& regb)
{
	unsigned short aid = rega.id;
	unsigned short bid = regb.id;
	
	// Duplicate current neighbourhood.
	rcTempVector<int> acon;
	acon.resize(rega.connections.size());
	for (int i = 0; i < rega.connections.size(); ++i)
		acon[i] = rega.connections[i];
	rcTempVector<int>& bcon = regb.connections;
	
	// Find insertion point on A.
	int insa = -1;
	for (int i = 0; i < acon.size(); ++i)
	{
		if (acon[i] == bid)
		{
			insa = i;
			break;
		}
	}
	if (insa == -1)
		return false;
	
	// Find insertion point on B.
	int insb = -1;
	for (int i = 0; i < bcon.size(); ++i)
	{
		if (bcon[i] == aid)
		{
			insb = i;
			break;
		}
	}
	if (insb == -1)
		return false;
	
	// Merge neighbours.
	rega.connections.clear();
	for (int i = 0, ni = static_cast<int>(acon.size()); i < ni-1; ++i)
	{
		rega.connections.push_back(acon[(insa+1+i) % ni]);
	}
		
	for (int i = 0, ni = static_cast<int>(bcon.size()); i < ni-1; ++i)
	{
		rega.connections.push_back(bcon[(insb+1+i) % ni]);
	}
	
	removeAdjacentNeighbours(rega);
	
	for (int j = 0; j < regb.floors.size(); ++j)
		addUniqueFloorRegion(rega, regb.floors[j]);
	rega.spanCount += regb.spanCount;
	regb.spanCount = 0;
	regb.connections.resize(0);

	return true;
}

static bool isRegionConnectedToBorder(const rcRegion& reg)
{
	// Region is connected to border if
	// one of the neighbours is null id.
	for (int i = 0; i < reg.connections.size(); ++i)
	{
		if (reg.connections[i] == 0)
			return true;
	}
	return false;
}

static bool isSolidEdge(Moss_RecastCompactHeightfield& chf, const unsigned short* srcReg,
						int x, int y, int i, int dir)
{
	const Moss_RecastCompactSpan& s = chf.spans[i];
	unsigned short r = 0;
	if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
	{
		const int ax = x + rcGetDirOffsetX(dir);
		const int ay = y + rcGetDirOffsetY(dir);
		const int ai = (int)chf.cells[ax+ay*chf.width].index + rcGetCon(s, dir);
		r = srcReg[ai];
	}
	if (r == srcReg[i])
		return false;
	return true;
}

static void walkContour(int x, int y, int i, int dir,
						Moss_RecastCompactHeightfield& chf,
						const unsigned short* srcReg,
						rcTempVector<int>& cont)
{
	int startDir = dir;
	int starti = i;

	const Moss_RecastCompactSpan& ss = chf.spans[i];
	unsigned short curReg = 0;
	if (rcGetCon(ss, dir) != RC_NOT_CONNECTED)
	{
		const int ax = x + rcGetDirOffsetX(dir);
		const int ay = y + rcGetDirOffsetY(dir);
		const int ai = (int)chf.cells[ax+ay*chf.width].index + rcGetCon(ss, dir);
		curReg = srcReg[ai];
	}
	cont.push_back(curReg);
			
	int iter = 0;
	while (++iter < 40000)
	{
		const Moss_RecastCompactSpan& s = chf.spans[i];
		
		if (isSolidEdge(chf, srcReg, x, y, i, dir))
		{
			// Choose the edge corner
			unsigned short r = 0;
			if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
			{
				const int ax = x + rcGetDirOffsetX(dir);
				const int ay = y + rcGetDirOffsetY(dir);
				const int ai = (int)chf.cells[ax+ay*chf.width].index + rcGetCon(s, dir);
				r = srcReg[ai];
			}
			if (r != curReg)
			{
				curReg = r;
				cont.push_back(curReg);
			}
			
			dir = (dir+1) & 0x3;  // Rotate CW
		}
		else
		{
			int ni = -1;
			const int nx = x + rcGetDirOffsetX(dir);
			const int ny = y + rcGetDirOffsetY(dir);
			if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
			{
				const Moss_RecastCompactCell& nc = chf.cells[nx+ny*chf.width];
				ni = (int)nc.index + rcGetCon(s, dir);
			}
			if (ni == -1)
			{
				// Should not happen.
				return;
			}
			x = nx;
			y = ny;
			i = ni;
			dir = (dir+3) & 0x3;	// Rotate CCW
		}
		
		if (starti == i && startDir == dir)
		{
			break;
		}
	}

	// Remove adjacent duplicates.
	if (cont.size() > 1)
	{
		for (int j = 0; j < cont.size(); )
		{
			int nj = (j+1) % cont.size();
			if (cont[j] == cont[nj])
			{
				for (int k = j; k < cont.size()-1; ++k)
					cont[k] = cont[k+1];
				cont.pop_back();
			}
			else
				++j;
		}
	}
}


static bool mergeAndFilterRegions(rcContext* ctx, int minRegionArea, int mergeRegionSize,
								  unsigned short& maxRegionId,
								  Moss_RecastCompactHeightfield& chf,
								  unsigned short* srcReg, rcTempVector<int>& overlaps)
{
	const int w = chf.width;
	const int h = chf.height;
	
	const int nreg = maxRegionId+1;
	rcTempVector<rcRegion> regions;
	if (!regions.reserve(nreg)) {
		MOSS_ERROR("mergeAndFilterRegions: Out of memory 'regions' (%d).", nreg);
		return false;
	}

	// Construct regions
	for (int i = 0; i < nreg; ++i)
		regions.push_back(rcRegion((unsigned short) i));
	
	// Find edge of a region and find connections around the contour.
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const Moss_RecastCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				unsigned short r = srcReg[i];
				if (r == 0 || r >= nreg)
					continue;
				
				rcRegion& reg = regions[r];
				reg.spanCount++;
				
				// Update floors.
				for (int j = (int)c.index; j < ni; ++j)
				{
					if (i == j) continue;
					unsigned short floorId = srcReg[j];
					if (floorId == 0 || floorId >= nreg)
						continue;
					if (floorId == r)
						reg.overlap = true;
					addUniqueFloorRegion(reg, floorId);
				}
				
				// Have found contour
				if (reg.connections.size() > 0)
					continue;
				
				reg.areaType = chf.areas[i];
				
				// Check if this cell is next to a border.
				int ndir = -1;
				for (int dir = 0; dir < 4; ++dir)
				{
					if (isSolidEdge(chf, srcReg, x, y, i, dir))
					{
						ndir = dir;
						break;
					}
				}
				
				if (ndir != -1)
				{
					// The cell is at border.
					// Walk around the contour to find all the neighbours.
					walkContour(x, y, i, ndir, chf, srcReg, reg.connections);
				}
			}
		}
	}

	// Remove too small regions.
	rcTempVector<int> stack(32);
	rcTempVector<int> trace(32);
	for (int i = 0; i < nreg; ++i)
	{
		rcRegion& reg = regions[i];
		if (reg.id == 0 || (reg.id & RC_BORDER_REG))
			continue;                       
		if (reg.spanCount == 0)
			continue;
		if (reg.visited)
			continue;
		
		// Count the total size of all the connected regions.
		// Also keep track of the regions connects to a tile border.
		bool connectsToBorder = false;
		int spanCount = 0;
		stack.clear();
		trace.clear();

		reg.visited = true;
		stack.push_back(i);
		
		while (stack.size())
		{
			// Pop
			int ri = stack.back(); stack.pop_back();
			
			rcRegion& creg = regions[ri];

			spanCount += creg.spanCount;
			trace.push_back(ri);

			for (int j = 0; j < creg.connections.size(); ++j)
			{
				if (creg.connections[j] & RC_BORDER_REG)
				{
					connectsToBorder = true;
					continue;
				}
				rcRegion& neireg = regions[creg.connections[j]];
				if (neireg.visited)
					continue;
				if (neireg.id == 0 || (neireg.id & RC_BORDER_REG))
					continue;
				// Visit
				stack.push_back(neireg.id);
				neireg.visited = true;
			}
		}
		
		// If the accumulated regions size is too small, remove it.
		// Do not remove areas which connect to tile borders
		// as their size cannot be estimated correctly and removing them
		// can potentially remove necessary areas.
		if (spanCount < minRegionArea && !connectsToBorder)
		{
			// Kill all visited regions.
			for (int j = 0; j < trace.size(); ++j)
			{
				regions[trace[j]].spanCount = 0;
				regions[trace[j]].id = 0;
			}
		}
	}
	
	// Merge too small regions to neighbour regions.
	int mergeCount = 0 ;
	do
	{
		mergeCount = 0;
		for (int i = 0; i < nreg; ++i)
		{
			rcRegion& reg = regions[i];
			if (reg.id == 0 || (reg.id & RC_BORDER_REG))
				continue;
			if (reg.overlap)
				continue;
			if (reg.spanCount == 0)
				continue;
			
			// Check to see if the region should be merged.
			if (reg.spanCount > mergeRegionSize && isRegionConnectedToBorder(reg))
				continue;
			
			// Small region with more than 1 connection.
			// Or region which is not connected to a border at all.
			// Find smallest neighbour region that connects to this one.
			int smallest = 0xfffffff;
			unsigned short mergeId = reg.id;
			for (int j = 0; j < reg.connections.size(); ++j)
			{
				if (reg.connections[j] & RC_BORDER_REG) continue;
				rcRegion& mreg = regions[reg.connections[j]];
				if (mreg.id == 0 || (mreg.id & RC_BORDER_REG) || mreg.overlap) continue;
				if (mreg.spanCount < smallest &&
					canMergeWithRegion(reg, mreg) &&
					canMergeWithRegion(mreg, reg))
				{
					smallest = mreg.spanCount;
					mergeId = mreg.id;
				}
			}
			// Found new id.
			if (mergeId != reg.id)
			{
				unsigned short oldId = reg.id;
				rcRegion& target = regions[mergeId];
				
				// Merge neighbours.
				if (mergeRegions(target, reg))
				{
					// Fixup regions pointing to current region.
					for (int j = 0; j < nreg; ++j)
					{
						if (regions[j].id == 0 || (regions[j].id & RC_BORDER_REG)) continue;
						// If another region was already merged into current region
						// change the nid of the previous region too.
						if (regions[j].id == oldId)
							regions[j].id = mergeId;
						// Replace the current region with the new one if the
						// current regions is neighbour.
						replaceNeighbour(regions[j], oldId, mergeId);
					}
					mergeCount++;
				}
			}
		}
	}
	while (mergeCount > 0);
	
	// Compress region Ids.
	for (int i = 0; i < nreg; ++i)
	{
		regions[i].remap = false;
		if (regions[i].id == 0) continue;       // Skip nil regions.
		if (regions[i].id & RC_BORDER_REG) continue;    // Skip external regions.
		regions[i].remap = true;
	}
	
	unsigned short regIdGen = 0;
	for (int i = 0; i < nreg; ++i)
	{
		if (!regions[i].remap)
			continue;
		unsigned short oldId = regions[i].id;
		unsigned short newId = ++regIdGen;
		for (int j = i; j < nreg; ++j)
		{
			if (regions[j].id == oldId)
			{
				regions[j].id = newId;
				regions[j].remap = false;
			}
		}
	}
	maxRegionId = regIdGen;
	
	// Remap regions.
	for (int i = 0; i < chf.spanCount; ++i)
	{
		if ((srcReg[i] & RC_BORDER_REG) == 0)
			srcReg[i] = regions[srcReg[i]].id;
	}

	// Return regions that we found to be overlapping.
	for (int i = 0; i < nreg; ++i)
		if (regions[i].overlap)
			overlaps.push_back(regions[i].id);

	return true;
}


static void addUniqueConnection(rcRegion& reg, int n)
{
	for (int i = 0; i < reg.connections.size(); ++i)
		if (reg.connections[i] == n)
			return;
	reg.connections.push_back(n);
}

static bool mergeAndFilterLayerRegions(rcContext* ctx, int minRegionArea,
									   unsigned short& maxRegionId,
									   Moss_RecastCompactHeightfield& chf,
									   unsigned short* srcReg)
{
	const int w = chf.width;
	const int h = chf.height;
	
	const int nreg = maxRegionId+1;
	rcTempVector<rcRegion> regions;
	
	// Construct regions
	if (!regions.reserve(nreg)) {
		MOSS_ERROR("mergeAndFilterLayerRegions: Out of memory 'regions' (%d).", nreg);
		return false;
	}
	for (int i = 0; i < nreg; ++i)
		regions.push_back(rcRegion((unsigned short) i));
	
	// Find region neighbours and overlapping regions.
	rcTempVector<int> lregs(32);
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const Moss_RecastCompactCell& c = chf.cells[x+y*w];

			lregs.clear();
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const Moss_RecastCompactSpan& s = chf.spans[i];
				const unsigned char area = chf.areas[i];
				const unsigned short ri = srcReg[i];
				if (ri == 0 || ri >= nreg) continue;
				rcRegion& reg = regions[ri];
				
				reg.spanCount++;
				reg.areaType = area;

				reg.ymin = min(reg.ymin, s.y);
				reg.ymax = max(reg.ymax, s.y);
				
				// Collect all region layers.
				lregs.push_back(ri);
				
				// Update neighbours
				for (int dir = 0; dir < 4; ++dir)
				{
					if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, dir);
						const unsigned short rai = srcReg[ai];
						if (rai > 0 && rai < nreg && rai != ri)
							addUniqueConnection(reg, rai);
						if (rai & RC_BORDER_REG)
							reg.connectsToBorder = true;
					}
				}
				
			}
			
			// Update overlapping regions.
			for (int i = 0; i < lregs.size()-1; ++i)
			{
				for (int j = i+1; j < lregs.size(); ++j)
				{
					if (lregs[i] != lregs[j])
					{
						rcRegion& ri = regions[lregs[i]];
						rcRegion& rj = regions[lregs[j]];
						addUniqueFloorRegion(ri, lregs[j]);
						addUniqueFloorRegion(rj, lregs[i]);
					}
				}
			}
			
		}
	}

	// Create 2D layers from regions.
	unsigned short layerId = 1;

	for (int i = 0; i < nreg; ++i)
		regions[i].id = 0;

	// Merge montone regions to create non-overlapping areas.
	rcTempVector<int> stack(32);
	for (int i = 1; i < nreg; ++i)
	{
		rcRegion& root = regions[i];
		// Skip already visited.
		if (root.id != 0)
			continue;
		
		// Start search.
		root.id = layerId;

		stack.clear();
		stack.push_back(i);
		
		while (stack.size() > 0)
		{
			// Pop front
			rcRegion& reg = regions[stack[0]];
			for (int j = 0; j < stack.size()-1; ++j)
				stack[j] = stack[j+1];
			stack.resize(stack.size()-1);
			
			const int ncons = (int)reg.connections.size();
			for (int j = 0; j < ncons; ++j)
			{
				const int nei = reg.connections[j];
				rcRegion& regn = regions[nei];
				// Skip already visited.
				if (regn.id != 0)
					continue;
				// Skip if different area type, do not connect regions with different area type.
				if (reg.areaType != regn.areaType)
					continue;
				// Skip if the neighbour is overlapping root region.
				bool overlap = false;
				for (int k = 0; k < root.floors.size(); k++)
				{
					if (root.floors[k] == nei)
					{
						overlap = true;
						break;
					}
				}
				if (overlap)
					continue;
					
				// Deepen
				stack.push_back(nei);
					
				// Mark layer id
				regn.id = layerId;
				// Merge current layers to root.
				for (int k = 0; k < regn.floors.size(); ++k)
					addUniqueFloorRegion(root, regn.floors[k]);
				root.ymin = min(root.ymin, regn.ymin);
				root.ymax = max(root.ymax, regn.ymax);
				root.spanCount += regn.spanCount;
				regn.spanCount = 0;
				root.connectsToBorder = root.connectsToBorder || regn.connectsToBorder;
			}
		}
		
		layerId++;
	}
	
	// Remove small regions
	for (int i = 0; i < nreg; ++i)
	{
		if (regions[i].spanCount > 0 && regions[i].spanCount < minRegionArea && !regions[i].connectsToBorder)
		{
			unsigned short reg = regions[i].id;
			for (int j = 0; j < nreg; ++j)
				if (regions[j].id == reg)
					regions[j].id = 0;
		}
	}
	
	// Compress region Ids.
	for (int i = 0; i < nreg; ++i)
	{
		regions[i].remap = false;
		if (regions[i].id == 0) continue;				// Skip nil regions.
		if (regions[i].id & RC_BORDER_REG) continue;    // Skip external regions.
		regions[i].remap = true;
	}
	
	unsigned short regIdGen = 0;
	for (int i = 0; i < nreg; ++i)
	{
		if (!regions[i].remap)
			continue;
		unsigned short oldId = regions[i].id;
		unsigned short newId = ++regIdGen;
		for (int j = i; j < nreg; ++j)
		{
			if (regions[j].id == oldId)
			{
				regions[j].id = newId;
				regions[j].remap = false;
			}
		}
	}
	maxRegionId = regIdGen;
	
	// Remap regions.
	for (int i = 0; i < chf.spanCount; ++i)
	{
		if ((srcReg[i] & RC_BORDER_REG) == 0)
			srcReg[i] = regions[srcReg[i]].id;
	}
	
	return true;
}



/// @par
/// 
/// This is usually the second to the last step in creating a fully built
/// compact heightfield.  This step is required before regions are built
/// using #rcBuildRegions or #rcBuildRegionsMonotone.
/// 
/// After this step, the distance data is available via the Moss_RecastCompactHeightfield::maxDistance
/// and Moss_RecastCompactHeightfield::dist fields.
///
/// @see Moss_RecastCompactHeightfield, rcBuildRegions, rcBuildRegionsMonotone
bool rcBuildDistanceField(rcContext* ctx, Moss_RecastCompactHeightfield& chf)
{
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_BUILD_DISTANCEFIELD);
	
	if (chf.dist)
	{
		MOSS_FREE(chf.dist);
		chf.dist = 0;
	}
	
	unsigned short* src = (unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount, RC_ALLOC_TEMP);
	if (!src) {
		MOSS_ERROR("rcBuildDistanceField: Out of memory 'src' (%d).", chf.spanCount);
		return false;
	}
	unsigned short* dst = (unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount, RC_ALLOC_TEMP);
	if (!dst) {
		MOSS_ERROR("rcBuildDistanceField: Out of memory 'dst' (%d).", chf.spanCount);
		MOSS_FREE(src);
		return false;
	}
	
	unsigned short maxDist = 0;

	{
		rcScopedTimer timerDist(ctx, RC_TIMER_BUILD_DISTANCEFIELD_DIST);

		calculateDistanceField(chf, src, maxDist);
		chf.maxDistance = maxDist;
	}

	{
		rcScopedTimer timerBlur(ctx, RC_TIMER_BUILD_DISTANCEFIELD_BLUR);

		// Blur
		if (boxBlur(chf, 1, src, dst) != src)
			Swap(src, dst);

		// Store distance.
		chf.dist = src;
	}
	
	MOSS_FREE(dst);
	
	return true;
}

static void paintRectRegion(int minx, int maxx, int miny, int maxy, unsigned short regId,
							Moss_RecastCompactHeightfield& chf, unsigned short* srcReg)
{
	const int w = chf.width;	
	for (int y = miny; y < maxy; ++y)
	{
		for (int x = minx; x < maxx; ++x)
		{
			const Moss_RecastCompactCell& c = chf.cells[x+y*w];
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				if (chf.areas[i] != RC_NULL_AREA)
					srcReg[i] = regId;
			}
		}
	}
}


static const unsigned short RC_NULL_NEI = 0xffff;

struct rcSweepSpan
{
	unsigned short rid;	// row id
	unsigned short id;	// region id
	unsigned short ns;	// number samples
	unsigned short nei;	// neighbour id
};

/// @par
/// 
/// Non-null regions will consist of connected, non-overlapping walkable spans that form a single contour.
/// Contours will form simple polygons.
/// 
/// If multiple regions form an area that is smaller than @p minRegionArea, then all spans will be
/// re-assigned to the zero (null) region.
/// 
/// Partitioning can result in smaller than necessary regions. @p mergeRegionArea helps 
/// reduce unnecessarily small regions.
/// 
/// See the #Moss_RecastSettings3D documentation for more information on the configuration parameters.
/// 
/// The region data will be available via the Moss_RecastCompactHeightfield::maxRegions
/// and Moss_RecastCompactSpan::reg fields.
/// 
/// @warning The distance field must be created using #rcBuildDistanceField before attempting to build regions.
/// 
/// @see Moss_RecastCompactHeightfield, Moss_RecastCompactSpan, rcBuildDistanceField, rcBuildRegionsMonotone, Moss_RecastSettings3D
bool rcBuildRegionsMonotone(rcContext* ctx, Moss_RecastCompactHeightfield& chf,
							const int borderSize, const int minRegionArea, const int mergeRegionArea)
{
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_BUILD_REGIONS);
	
	const int w = chf.width;
	const int h = chf.height;
	unsigned short id = 1;
	
	rcScopedDelete<unsigned short> srcReg((unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount, RC_ALLOC_TEMP));
	if (!srcReg) {
		MOSS_ERROR("rcBuildRegionsMonotone: Out of memory 'src' (%d).", chf.spanCount);
		return false;
	}
	memset(srcReg,0,sizeof(unsigned short)*chf.spanCount);

	const int nsweeps = max(chf.width,chf.height);
	rcScopedDelete<rcSweepSpan> sweeps((rcSweepSpan*)rcAlloc(sizeof(rcSweepSpan)*nsweeps, RC_ALLOC_TEMP));
	if (!sweeps) {
		MOSS_ERROR("rcBuildRegionsMonotone: Out of memory 'sweeps' (%d).", nsweeps);
		return false;
	}
	
	
	// Mark border regions.
	if (borderSize > 0)
	{
		// Make sure border will not overflow.
		const int bw = min(w, borderSize);
		const int bh = min(h, borderSize);
		// Paint regions
		paintRectRegion(0, bw, 0, h, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(w-bw, w, 0, h, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(0, w, 0, bh, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(0, w, h-bh, h, id|RC_BORDER_REG, chf, srcReg); id++;
	}

	chf.borderSize = borderSize;
	
	rcTempVector<int> prev(256);

	// Sweep one line at a time.
	for (int y = borderSize; y < h-borderSize; ++y)
	{
		// Collect spans from this row.
		prev.resize(id+1);
		memset(&prev[0],0,sizeof(int)*id);
		unsigned short rid = 1;
		
		for (int x = borderSize; x < w-borderSize; ++x)
		{
			const Moss_RecastCompactCell& c = chf.cells[x+y*w];
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				const Moss_RecastCompactSpan& s = chf.spans[i];
				if (chf.areas[i] == RC_NULL_AREA) continue;
				
				// -x
				unsigned short previd = 0;
				if (rcGetCon(s, 0) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(0);
					const int ay = y + rcGetDirOffsetY(0);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 0);
					if ((srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai])
						previd = srcReg[ai];
				}
				
				if (!previd)
				{
					previd = rid++;
					sweeps[previd].rid = previd;
					sweeps[previd].ns = 0;
					sweeps[previd].nei = 0;
				}

				// -y
				if (rcGetCon(s,3) != RC_NOT_CONNECTED)
				{
					const int ax = x + rcGetDirOffsetX(3);
					const int ay = y + rcGetDirOffsetY(3);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 3);
					if (srcReg[ai] && (srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai])
					{
						unsigned short nr = srcReg[ai];
						if (!sweeps[previd].nei || sweeps[previd].nei == nr)
						{
							sweeps[previd].nei = nr;
							sweeps[previd].ns++;
							prev[nr]++;
						}
						else
						{
							sweeps[previd].nei = RC_NULL_NEI;
						}
					}
				}

				srcReg[i] = previd;
			}
		}
		
		// Create unique ID.
		for (int i = 1; i < rid; ++i)
		{
			if (sweeps[i].nei != RC_NULL_NEI && sweeps[i].nei != 0 &&
				prev[sweeps[i].nei] == (int)sweeps[i].ns)
			{
				sweeps[i].id = sweeps[i].nei;
			}
			else
			{
				sweeps[i].id = id++;
			}
		}
		
		// Remap IDs
		for (int x = borderSize; x < w-borderSize; ++x)
		{
			const Moss_RecastCompactCell& c = chf.cells[x+y*w];
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i)
			{
				if (srcReg[i] > 0 && srcReg[i] < rid)
					srcReg[i] = sweeps[srcReg[i]].id;
			}
		}
	}


	{
		rcScopedTimer timerFilter(ctx, RC_TIMER_BUILD_REGIONS_FILTER);

		// Merge regions and filter out small regions.
		rcTempVector<int> overlaps;
		chf.maxRegions = id;
		if (!mergeAndFilterRegions(ctx, minRegionArea, mergeRegionArea, chf.maxRegions, chf, srcReg, overlaps))
			return false;

		// Monotone partitioning does not generate overlapping regions.
	}
	
	// Store the result out.
	for (int i = 0; i < chf.spanCount; ++i)
		chf.spans[i].reg = srcReg[i];

	return true;
}

/// @par
/// 
/// Non-null regions will consist of connected, non-overlapping walkable spans that form a single contour.
/// Contours will form simple polygons.
/// 
/// If multiple regions form an area that is smaller than @p minRegionArea, then all spans will be
/// re-assigned to the zero (null) region.
/// 
/// Watershed partitioning can result in smaller than necessary regions, especially in diagonal corridors. 
/// @p mergeRegionArea helps reduce unnecessarily small regions.
/// 
/// See the #Moss_RecastSettings3D documentation for more information on the configuration parameters.
/// 
/// The region data will be available via the Moss_RecastCompactHeightfield::maxRegions
/// and Moss_RecastCompactSpan::reg fields.
/// 
/// @warning The distance field must be created using #rcBuildDistanceField before attempting to build regions.
/// 
/// @see Moss_RecastCompactHeightfield, Moss_RecastCompactSpan, rcBuildDistanceField, rcBuildRegionsMonotone, Moss_RecastSettings3D
bool rcBuildRegions(rcContext* ctx, Moss_RecastCompactHeightfield& chf, const int borderSize, const int minRegionArea, const int mergeRegionArea) {
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_BUILD_REGIONS);
	
	const int w = chf.width;
	const int h = chf.height;
	
	rcScopedDelete<unsigned short> buf((unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount*2, RC_ALLOC_TEMP));
	if (!buf) {
		MOSS_ERROR("rcBuildRegions: Out of memory 'tmp' (%d).", chf.spanCount*4);
		return false;
	}
	
	ctx->startTimer(RC_TIMER_BUILD_REGIONS_WATERSHED);

	const int LOG_NB_STACKS = 3;
	const int NB_STACKS = 1 << LOG_NB_STACKS;
	rcTempVector<LevelStackEntry> lvlStacks[NB_STACKS];
	for (int i=0; i<NB_STACKS; ++i)
		lvlStacks[i].reserve(256);

	rcTempVector<LevelStackEntry> stack;
	stack.reserve(256);
	
	unsigned short* srcReg = buf;
	unsigned short* srcDist = buf+chf.spanCount;
	
	memset(srcReg, 0, sizeof(unsigned short)*chf.spanCount);
	memset(srcDist, 0, sizeof(unsigned short)*chf.spanCount);
	
	unsigned short regionId = 1;
	unsigned short level = (chf.maxDistance+1) & ~1;

	// TODO: Figure better formula, expandIters defines how much the 
	// watershed "overflows" and simplifies the regions. Tying it to
	// agent radius was usually good indication how greedy it could be.
//	const int expandIters = 4 + walkableRadius * 2;
	const int expandIters = 8;

	if (borderSize > 0) {
		// Make sure border will not overflow.
		const int bw = min(w, borderSize);
		const int bh = min(h, borderSize);
		
		// Paint regions
		paintRectRegion(0, bw, 0, h, regionId|RC_BORDER_REG, chf, srcReg); regionId++;
		paintRectRegion(w-bw, w, 0, h, regionId|RC_BORDER_REG, chf, srcReg); regionId++;
		paintRectRegion(0, w, 0, bh, regionId|RC_BORDER_REG, chf, srcReg); regionId++;
		paintRectRegion(0, w, h-bh, h, regionId|RC_BORDER_REG, chf, srcReg); regionId++;
	}

	chf.borderSize = borderSize;
	
	int sId = -1;
	while (level > 0) {
		level = level >= 2 ? level-2 : 0;
		sId = (sId+1) & (NB_STACKS-1);

//		ctx->startTimer(RC_TIMER_DIVIDE_TO_LEVELS);

		if (sId == 0)
			sortCellsByLevel(level, chf, srcReg, NB_STACKS, lvlStacks, 1);
		else 
			appendStacks(lvlStacks[sId-1], lvlStacks[sId], srcReg); // copy left overs from last level

//		ctx->stopTimer(RC_TIMER_DIVIDE_TO_LEVELS);

		{
			rcScopedTimer timerExpand(ctx, RC_TIMER_BUILD_REGIONS_EXPAND);

			// Expand current regions until no empty connected cells found.
			expandRegions(expandIters, level, chf, srcReg, srcDist, lvlStacks[sId], false);
		}
		
		{
			rcScopedTimer timerFloor(ctx, RC_TIMER_BUILD_REGIONS_FLOOD);

			// Mark new regions with IDs.
			for (int j = 0; j<lvlStacks[sId].size(); j++)
			{
				LevelStackEntry current = lvlStacks[sId][j];
				int x = current.x;
				int y = current.y;
				int i = current.index;
				if (i >= 0 && srcReg[i] == 0)
				{
					if (floodRegion(x, y, i, level, regionId, chf, srcReg, srcDist, stack))
					{
						if (regionId == 0xFFFF)
						{
							MOSS_ERROR("rcBuildRegions: Region ID overflow");
							return false;
						}
						
						regionId++;
					}
				}
			}
		}
	}
	
	// Expand current regions until no empty connected cells found.
	expandRegions(expandIters*8, 0, chf, srcReg, srcDist, stack, true);
	
	ctx->stopTimer(RC_TIMER_BUILD_REGIONS_WATERSHED);
	{
		rcScopedTimer timerFilter(ctx, RC_TIMER_BUILD_REGIONS_FILTER);

		// Merge regions and filter out small regions.
		rcTempVector<int> overlaps;
		chf.maxRegions = regionId;
		if (!mergeAndFilterRegions(ctx, minRegionArea, mergeRegionArea, chf.maxRegions, chf, srcReg, overlaps))
			return false;

		// If overlapping regions were found during merging, split those regions.
		if (overlaps.size() > 0) {
			MOSS_ERROR("rcBuildRegions: %d overlapping regions.", overlaps.size());
		}
	}
		
	// Write the result out.
	for (int i = 0; i < chf.spanCount; ++i)
		chf.spans[i].reg = srcReg[i];
	
	return true;
}


bool rcBuildLayerRegions(rcContext* ctx, Moss_RecastCompactHeightfield& chf, const int borderSize, const int minRegionArea) {
	rcAssert(ctx);
	
	rcScopedTimer timer(ctx, RC_TIMER_BUILD_REGIONS);
	
	const int w = chf.width;
	const int h = chf.height;
	unsigned short id = 1;
	
	rcScopedDelete<unsigned short> srcReg((unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount, RC_ALLOC_TEMP));
	if (!srcReg) {
		MOSS_ERROR("rcBuildLayerRegions: Out of memory 'src' (%d).", chf.spanCount);
		return false;
	}
	memset(srcReg,0,sizeof(unsigned short)*chf.spanCount);
	
	const int nsweeps = max(chf.width,chf.height);
	rcScopedDelete<rcSweepSpan> sweeps((rcSweepSpan*)rcAlloc(sizeof(rcSweepSpan)*nsweeps, RC_ALLOC_TEMP));
	if (!sweeps) {
		MOSS_ERROR("rcBuildLayerRegions: Out of memory 'sweeps' (%d).", nsweeps);
		return false;
	}
	
	
	// Mark border regions.
	if (borderSize > 0) {
		// Make sure border will not overflow.
		const int bw = min(w, borderSize);
		const int bh = min(h, borderSize);
		// Paint regions
		paintRectRegion(0, bw, 0, h, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(w-bw, w, 0, h, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(0, w, 0, bh, id|RC_BORDER_REG, chf, srcReg); id++;
		paintRectRegion(0, w, h-bh, h, id|RC_BORDER_REG, chf, srcReg); id++;
	}

	chf.borderSize = borderSize;
	
	rcTempVector<int> prev(256);
	
	// Sweep one line at a time.
	for (int y = borderSize; y < h-borderSize; ++y) {
		// Collect spans from this row.
		prev.resize(id+1);
		memset(&prev[0],0,sizeof(int)*id);
		unsigned short rid = 1;
		
		for (int x = borderSize; x < w-borderSize; ++x) {
			const Moss_RecastCompactCell& c = chf.cells[x+y*w];
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i) {
				const Moss_RecastCompactSpan& s = chf.spans[i];
				if (chf.areas[i] == RC_NULL_AREA) continue;
				
				// -x
				unsigned short previd = 0;
				if (rcGetCon(s, 0) != RC_NOT_CONNECTED) {
					const int ax = x + rcGetDirOffsetX(0);
					const int ay = y + rcGetDirOffsetY(0);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 0);
					if ((srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai])
						previd = srcReg[ai];
				}
				
				if (!previd) {
					previd = rid++;
					sweeps[previd].rid = previd;
					sweeps[previd].ns = 0;
					sweeps[previd].nei = 0;
				}
				
				// -y
				if (rcGetCon(s,3) != RC_NOT_CONNECTED) {
					const int ax = x + rcGetDirOffsetX(3);
					const int ay = y + rcGetDirOffsetY(3);
					const int ai = (int)chf.cells[ax+ay*w].index + rcGetCon(s, 3);
					if (srcReg[ai] && (srcReg[ai] & RC_BORDER_REG) == 0 && chf.areas[i] == chf.areas[ai])
					{
						unsigned short nr = srcReg[ai];
						if (!sweeps[previd].nei || sweeps[previd].nei == nr) {
							sweeps[previd].nei = nr;
							sweeps[previd].ns++;
							prev[nr]++;
						}
						else { sweeps[previd].nei = RC_NULL_NEI; }
					}
				}
				
				srcReg[i] = previd;
			}
		}
		
		// Create unique ID.
		for (int i = 1; i < rid; ++i) {
			if (sweeps[i].nei != RC_NULL_NEI && sweeps[i].nei != 0 && prev[sweeps[i].nei] == (int)sweeps[i].ns)  { sweeps[i].id = sweeps[i].nei; }
			else { sweeps[i].id = id++; }
		}
		
		// Remap IDs
		for (int x = borderSize; x < w-borderSize; ++x) {
			const Moss_RecastCompactCell& c = chf.cells[x+y*w];
			
			for (int i = (int)c.index, ni = (int)(c.index+c.count); i < ni; ++i) {
				if (srcReg[i] > 0 && srcReg[i] < rid)
					srcReg[i] = sweeps[srcReg[i]].id;
			}
		}
	}
	
	
	{
		rcScopedTimer timerFilter(ctx, RC_TIMER_BUILD_REGIONS_FILTER);

		// Merge monotone regions to layers and remove small regions.
		chf.maxRegions = id;
		if (!mergeAndFilterLayerRegions(ctx, minRegionArea, chf.maxRegions, chf, srcReg))
			return false;
	}
	
	
	// Store the result out.
	for (int i = 0; i < chf.spanCount; ++i)
		chf.spans[i].reg = srcReg[i];
	
	return true;
}
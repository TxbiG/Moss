//                        MIT License
//
//                  Copyright (c) 2026 Toby
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/* =======================================================================================================
 * Modified RecastNavigation v1.6.0 by Recast Navigation. 
 * Zlib License 
 * https://github.com/recastnavigation/recastnavigation
 * ==================================================================================================== */

/*!
 * @file Moss_RecastNavigation.h
 * @brief Navigation mesh and pathfinding subsystem built on top of a modified version of Recast & Detour.
 * https://github.com/recastnavigation/recastnavigation
 *
 * ---
 *
 * ### Overview
 * `Moss_RecastNavigation` provides a robust, real-time navigation mesh system for both 2D and 3D environments.
 * It is based on a highly optimized and modified version of **Recast Navigation (v1.6)** and **Detour**, adapted to fit the Moss framework’s
 * Rendering and physics systems.
 *
 * ---
 *
 * ### Core Responsibilities
 * - **Navigation Mesh (NavMesh) Generation**
 *   - Builds navigation meshes dynamically from physics geometry or pre-baked level data.
 *   - Supports **2D** and **3D** navigation mesh generation.
 *
 * - **Dynamic Obstacle Avoidance**
 *   - Integrates with the physics system to mark regions as non-navigable when occupied by rigidbodies.
 *   - Supports runtime mesh regeneration or local patch updates.
 *
 * - **Pathfinding**
 *   - Uses a highly efficient **A\*** and **Detour corridor navigation** algorithm.
 *   - Supports multi-agent pathfinding and steering.
 *
 * - **2D and 3D Compatibility**
 *   - Works in both top-down (2D) and volumetric (3D) environments.
 *   - Provides simplified 2D grid navigation for lightweight simulation and mobile/web targets.
 *
 * - **Visualization & Debugging**
 *   - Renders navigation polygons, agents, and off-mesh connections.
 *   - Includes runtime overlays for cost field heatmaps, agent paths, and mesh connectivity.
 *
 * ---
 *
 * ### Features
 * | Category | Feature | Description |
 * |-----------|----------|-------------|
 * | Mesh Generation | Recast voxelization | Converts static geometry into walkable surfaces. |
 * | Mesh Building | Heightfield smoothing | Handles staircases, slopes, and terrain variation. |
 * | Pathfinding | Detour-based navigation | Fast, memory-efficient A\* across nav polygons. |
 * | Agents | Crowd simulation | Handles avoidance and group pathfinding. |
 * | Obstacles | Dynamic | Supports local avoidance, collision updates, and navmesh cutting. |
 * | Debugging | Visualization tools | Draws navmesh polygons, nodes, and agent paths. |
 * | Integration | Jolt Physics | Automatically rebuilds navmesh when world geometry changes. |
 *
 * ---
 *
 * ### Performance Notes
 * - Multi-threaded voxelization and region merging using job-based scheduling.
 * - SIMD-accelerated pathfinding (A\* + corridor following).
 * - Incremental rebuild system for open-world or procedurally changing environments.
 * - Optimized memory layouts for cache-friendly navigation data.
 *
 */

#ifndef MOSS_RECASTNAVIGATION_H
#define MOSS_RECASTNAVIGATION_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <Moss/Moss_stdinc.h>
#include <Moss/Moss_Renderer.h>
#include <Moss/Moss_Physics.h>


// TODO Clean up Header. Fix bmin and bmax to AABB3 for SIMD. convert recast to C++ v17



//typedef uint32_t Moss_AgentID;

//struct Moss_RecastCrowdAgent2D;
//struct Moss_RecastCrowdAgent3D;

//struct Moss_RecastCrowd2D;
//struct Moss_RecastCrowd3D;


//Moss_NavAgent2D
//Moss_NavAgant3D

//class Moss_NavLink2D;
//class Moss_NavObsticale2D;
//class Moss_NavRegion2D;

//class Moss_NavLink3D;
//class Moss_NavObsticale3D;
//class Moss_NavRegion3D;


// Signed to avoid warnings when comparing to int loop indexes, and common error with comparing to zero.
// MSVC2010 has a bug where ssize_t is unsigned (!!!).
#define RC_SIZE_MAX INTPTR_MAX

static const int RC_SPAN_HEIGHT_BITS = 13;								// Defines the number of bits allocated to Moss_RecastSpan::smin and Moss_RecastSpan::smax.
static const int RC_SPAN_MAX_HEIGHT = (1 << RC_SPAN_HEIGHT_BITS) - 1;	// Defines the maximum value for Moss_RecastSpan::smin and Moss_RecastSpan::smax.
static const int RC_SPANS_PER_POOL = 2048;								// The number of spans allocated per span spool. @see Moss_RecastSpanPool

// Heightfield border flag. If a heightfield region ID has this bit set, then the region is a border 
// region and its spans are considered un-walkable. (Used during the region and contour build process.) @see Moss_RecastCompactSpan::reg
static const uint16_t RC_BORDER_REG = 0x8000;

// Polygon touches multiple regions.
// If a polygon has this region ID it was merged with or created
// from polygons of different regions during the polymesh
// build step that removes redundant border vertices. 
// (Used during the polymesh and detail polymesh build processes)
// @see Moss_RecastPolyMesh::regs
static const uint16_t RC_MULTIPLE_REGS = 0;

// Border vertex flag.
// If a region ID has this bit set, then the associated element lies on a tile border. If a contour vertex's region ID has this bit set, the  vertex will later be removed in order 
// to match the segments and vertices  at tile boundaries. (Used during the build process.)
// @see Moss_RecastCompactSpan::reg, #Moss_RecastContour::verts, #Moss_RecastContour::rverts
static const int RC_BORDER_VERTEX = 0x10000;

// Area border flag. If a region ID has this bit set, then the associated element lies on
// the border of an area. (Used during the region and contour build process.)
// @see Moss_RecastCompactSpan::reg, #Moss_RecastContour::verts, #Moss_RecastContour::rverts
static const int RC_AREA_BORDER = 0x20000;


// Applied to the region id field of contour vertices in order to extract the region id.
// The region id field of a vertex may have several flags applied to it.  So the
// fields value can't be used directly.
// @see Moss_RecastContour::verts, Moss_RecastContour::rverts
static const int RC_CONTOUR_REG_MASK = 0xffff;

// An value which indicates an invalid index within a mesh.
// @note This does not necessarily indicate an error.
// @see Moss_RecastPolyMesh::polys
static const uint16_t RC_MESH_NULL_IDX = 0xffff;

// Represents the null area.
// When a data element is given this value it is considered to no longer be 
// assigned to a usable area.  (E.g. It is un-walkable.)
static const uint8_t RC_NULL_AREA = 0;

// The default area id used to indicate a walkable polygon. 
// This is also the maximum allowed area id, and the only non-null area id 
// recognized by some steps in the build process. 
static const uint8_t RC_WALKABLE_AREA = 63;
static const int RC_NOT_CONNECTED = 0x3f;						// The value returned by #rcGetCon if the specified direction is not connected to another span. (Has no neighbor.)

static const uint32_t DT_PATHQ_INVALID = 0;

// High level status.
static const uint32_t DT_FAILURE = 1u << 31;				// Operation failed.
static const uint32_t DT_SUCCESS = 1u << 30;				// Operation succeed.
static const uint32_t DT_IN_PROGRESS = 1u << 29;			// Operation still in progress.

// Detail information for status.
static const uint32_t DT_STATUS_DETAIL_MASK = 0x0ffffff;
static const uint32_t DT_WRONG_MAGIC = 1 << 0;				// Input data is not recognized.
static const uint32_t DT_WRONG_VERSION = 1 << 1;			// Input data is in wrong version.
static const uint32_t DT_OUT_OF_MEMORY = 1 << 2;			// Operation ran out of memory.
static const uint32_t DT_INVALID_PARAM = 1 << 3;			// An input parameter was invalid.
static const uint32_t DT_BUFFER_TOO_SMALL = 1 << 4;			// Result buffer for the query was too small to store all results.
static const uint32_t DT_OUT_OF_NODES = 1 << 5;				// Query ran out of nodes during search.
static const uint32_t DT_PARTIAL_RESULT = 1 << 6;			// Query did not reach the end location, returning best guess. 
static const uint32_t DT_ALREADY_OCCUPIED = 1 << 7;			// A tile has already been assigned to the given x,y coordinate



// The maximum number of vertices per navigation polygon.
// @ingroup detour
static const int DT_VERTS_PER_POLYGON = 6;

// @name Tile Serialization Constants
// These constants are used to detect whether a navigation tile's data
// and state format is compatible with the current build.
//

// A magic number used to detect compatibility of navigation tile data.
static const int DT_NAVMESH_MAGIC = 'D'<<24 | 'N'<<16 | 'A'<<8 | 'V';

// A version number used to detect compatibility of navigation tile data.
static const int DT_NAVMESH_VERSION = 7;

// A magic number used to detect the compatibility of navigation tile states.
static const int DT_NAVMESH_STATE_MAGIC = 'D'<<24 | 'N'<<16 | 'M'<<8 | 'S';

// A version number used to detect compatibility of navigation tile states.
static const int DT_NAVMESH_STATE_VERSION = 1;

// @}

// A flag that indicates that an entity links to an external entity.
// (E.g. A polygon edge is a portal that links to another polygon.)
static const uint16_t DT_EXT_LINK = 0x8000;

// A value that indicates the entity does not link to anything.
static const uint32_t DT_NULL_LINK = 0xffffffff;

// A flag that indicates that an off-mesh connection can be traversed in both directions. (Is bidirectional.)
static const uint32_t DT_OFFMESH_CON_BIDIR = 1;

// The maximum number of user defined area ids.
// @ingroup detour
static const int DT_MAX_AREAS = 64;

// Limit raycasting during any angle pahfinding
// The limit is given as a multiple of the character radius
static const float DT_RAY_CAST_LIMIT_PROPORTIONS = 50.0f;


typedef uint16_t dtNodeIndex;
static const dtNodeIndex DT_NULL_IDX = (dtNodeIndex)~0;

static const int DT_NODE_PARENT_BITS = 24;
static const int DT_NODE_STATE_BITS = 2;

static const int DT_MAX_STATES_PER_NODE = 1 << DT_NODE_STATE_BITS;	// number of extra states per node. See Moss_RecastNode::state

static const int DT_MAX_TOUCHED_TILES = 8;
// The maximum number of neighbors that a crowd agent can take into account
// for steering decisions.
// @ingroup crowd
static const int DT_CROWDAGENT_MAX_NEIGHBOURS = 6;

// The maximum number of corners a crowd agent will look ahead in the path.
// This value is used for sizing the crowd agent corner buffers.
// Due to the behavior of the crowd manager, the actual number of useful
// corners will be one less than this number.
// @ingroup crowd
static const int DT_CROWDAGENT_MAX_CORNERS = 4;

// The maximum number of crowd avoidance configurations supported by the
// crowd manager.
// @ingroup crowd
// @see Moss_RecastObstacleAvoidanceSettings, Moss_RecastCrowd3D::setObstacleAvoidanceParams(), Moss_RecastCrowd3D::getObstacleAvoidanceParams(),
//		 Moss_RecastCrowdAgent3DSettings::obstacleAvoidanceType
static const int DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS = 8;

// The maximum number of query filter types supported by the crowd manager.
// @ingroup crowd
// @see Moss_RecastQueryFilter, Moss_RecastCrowd3D::getFilter() Moss_RecastCrowd3D::getEditableFilter(),
//		Moss_RecastCrowdAgent3DSettings::queryFilterType
static const int DT_CROWD_MAX_QUERY_FILTER_TYPE = 16;

static const int DT_MAX_PATTERN_DIVS = 32;	// Max numver of adaptive divs.
static const int DT_MAX_PATTERN_RINGS = 4;	// Max number of adaptive rings.


static const int DT_TILECACHE_MAGIC = 'D'<<24 | 'T'<<16 | 'L'<<8 | 'R'; // 'DTLR';
static const int DT_TILECACHE_VERSION = 1;

static const uint8_t DT_TILECACHE_NULL_AREA = 0;
static const uint8_t DT_TILECACHE_WALKABLE_AREA = 63;
static const uint16_t DT_TILECACHE_NULL_IDX = 0xffff;

typedef uint32_t Moss_RecastPathQueueRef;
#ifdef DT_POLYREF64
// TODO: figure out a multiplatform version of uint64_t
// - maybe: https://code.google.com/p/msinttypes/
// - or: http://www.azillionmonkeys.com/qed/pstdint.h
#include <stdint.h>
#endif

// Note: If you want to use 64-bit refs, change the types of both Moss_RecastPolyRef & Moss_RecastTileRef.
// It is also recommended that you change dtHashRef() to a proper 64-bit hash.

// A handle to a polygon within a navigation mesh tile.
// @ingroup detour
#ifdef DT_POLYREF64
static const uint32_t DT_SALT_BITS = 16;
static const uint32_t DT_TILE_BITS = 28;
static const uint32_t DT_POLY_BITS = 20;
typedef uint64_t Moss_RecastPolyRef;
#else
typedef uint32_t Moss_RecastPolyRef;
#endif

// A handle to a tile within a navigation mesh.
// @ingroup detour
#ifdef DT_POLYREF64
typedef uint64_t Moss_RecastTileRef;
#else
typedef uint32_t Moss_RecastTileRef;
#endif
typedef uint32_t Moss_RecastStatus;
typedef uint32_t Moss_RecastObstacleRef;
typedef uint32_t Moss_RecastCompressedTileRef;
typedef intptr_t rcSizeType;

// Contour build flags.
enum class rcBuildContoursFlags {
	RC_CONTOUR_TESS_WALL_EDGES = 0x01,	// Tessellate solid (impassable) edges during contour simplification.
	RC_CONTOUR_TESS_AREA_EDGES = 0x02	// Tessellate edges between areas during contour simplification.
};

// Tile flags used for various functions and fields. For an example, see Moss_RecastNavMesh::addTile().
enum class dtTileFlags {
	DT_TILE_FREE_DATA = 0x01		// The navigation mesh owns the tile memory and is responsible for freeing it.
};

// Vertex flags returned by Moss_RecastNavMeshQuery::findStraightPath.
enum class dtStraightPathFlags {
	DT_STRAIGHTPATH_START = 0x01,				// The vertex is the start position in the path.
	DT_STRAIGHTPATH_END = 0x02,					// The vertex is the end position in the path.
	DT_STRAIGHTPATH_OFFMESH_CONNECTION = 0x04	// The vertex is the start of an off-mesh connection.
};

// Options for Moss_RecastNavMeshQuery::findStraightPath.
enum dtStraightPathOptions {
	DT_STRAIGHTPATH_AREA_CROSSINGS = 0x01,	// Add a vertex at every polygon edge crossing where area changes.
	DT_STRAIGHTPATH_ALL_CROSSINGS = 0x02	// Add a vertex at every polygon edge crossing.
};


// Options for Moss_RecastNavMeshQuery::initSlicedFindPath and updateSlicedFindPath
enum class dtFindPathOptions {
	DT_FINDPATH_ANY_ANGLE	= 0x02		// use raycasts during pathfind to "shortcut" (raycast still consider costs)
};

// Options for Moss_RecastNavMeshQuery::raycast
enum class dtRaycastOptions {
	DT_RAYCAST_USE_COSTS = 0x01		// Raycast should calculate movement cost along the ray and fill RaycastHit::cost
};

enum class dtDetailTriEdgeFlags {
	DT_DETAIL_EDGE_BOUNDARY = 0x01		// Detail triangle edge is part of the poly boundary
};


// Flags representing the type of a navigation mesh polygon.
enum class dtPolyTypes {
	DT_POLYTYPE_GROUND = 0,				// The polygon is a standard convex polygon that is part of the surface of the mesh.
	DT_POLYTYPE_OFFMESH_CONNECTION = 1	// The polygon is an off-mesh connection consisting of two vertices.
};

enum class dtNodeFlags {
	DT_NODE_OPEN = 0x01,
	DT_NODE_CLOSED = 0x02,
	DT_NODE_PARENT_DETACHED = 0x04 // parent of the node is not adjacent. Found using raycast.
};

// The type of navigation mesh polygon the agent is currently traversing.
enum CrowdAgentState {
	DT_CROWDAGENT_STATE_INVALID,		// The agent is not in a valid state.
	DT_CROWDAGENT_STATE_WALKING,		// The agent is traversing a normal navigation mesh polygon.
	DT_CROWDAGENT_STATE_OFFMESH 		// The agent is traversing an off-mesh connection.
};
enum class MoveRequestState {
	DT_CROWDAGENT_TARGET_NONE = 0,
	DT_CROWDAGENT_TARGET_FAILED,
	DT_CROWDAGENT_TARGET_VALID,
	DT_CROWDAGENT_TARGET_REQUESTING,
	DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE,
	DT_CROWDAGENT_TARGET_WAITING_FOR_PATH,
	DT_CROWDAGENT_TARGET_VELOCITY
};

// Crowd agent update flags.
enum class UpdateFlags {
	DT_CROWD_ANTICIPATE_TURNS = 1,
	DT_CROWD_OBSTACLE_AVOIDANCE = 2,
	DT_CROWD_SEPARATION = 4,
	DT_CROWD_OPTIMIZE_VIS = 8,			// Use #Moss_RecastPathCorridor::optimizePathVisibility() to optimize the agent path.
	DT_CROWD_OPTIMIZE_TOPO = 16 		// Use Moss_RecastPathCorridor::optimizePathTopology() to optimize the agent path.
};

// Flags for addTile
enum class dtCompressedTileFlags {
	DT_COMPRESSEDTILE_FREE_DATA = 0x01	// Navmesh owns the tile memory and should free it.
};

enum class ObstacleState {
	DT_OBSTACLE_EMPTY,
	DT_OBSTACLE_PROCESSING,
	DT_OBSTACLE_PROCESSED,
	DT_OBSTACLE_REMOVING
};

enum class ObstacleType {
	DT_OBSTACLE_CYLINDER,
	DT_OBSTACLE_BOX, 			// AABB
	DT_OBSTACLE_ORIENTED_BOX 	// OBB
};

// Used to ignore unused function parameters and silence any compiler warnings.
template<class T> void rcIgnoreUnused(const T&) { }

/////////////////////////////////////////////////////////////////////////////////////////

struct Moss_NavigationSettings2D {
	AABB2 box;
	int borderSize;
	int tileSize;
	float agentRadius;   // World units
	float cellSize; // World units per cell
	float maxSimplificationError; // World units
	int maxEdgeLen; // In cells or world units
	int minRegionArea; // In cells
	int mergeRegionArea;
};

// Specifies a configuration to use when performing Recast builds.
// @ingroup recast
struct Moss_RecastSettings3D {
	int width;						// The width of the field along the x-axis. [Limit: >= 0] [Units: vx]
	int height;						// The height of the field along the z-axis. [Limit: >= 0] [Units: vx]
	int tileSize;					// The width/height size of tile's on the xz-plane. [Limit: >= 0] [Units: vx]
	int borderSize;					// The size of the non-navigable border around the heightfield. [Limit: >=0] [Units: vx]
	float cs;						// The xz-plane cell size to use for fields. [Limit: > 0] [Units: wu] 
	float ch;						// The y-axis cell size to use for fields. [Limit: > 0] [Units: wu]

	AABB3 bounds; // replaces bmin + bmax

	float walkableSlopeAngle;		// The maximum slope that is considered walkable. [Limits: 0 <= value < 90] [Units: Degrees] 

	// Minimum floor to 'ceiling' height that will still allow the floor area to 
	// be considered walkable. [Limit: >= 3] [Units: vx] 
	int walkableHeight;
	int walkableClimb;				// Maximum ledge height that is considered to still be traversable. [Limit: >=0] [Units: vx] 
	
	// The distance to erode/shrink the walkable area of the heightfield away from 
	// obstructions.  [Limit: >=0] [Units: vx] 
	int walkableRadius;
	
	// The maximum allowed length for contour edges along the border of the mesh. [Limit: >=0] [Units: vx] 
	int maxEdgeLen;
	
	// The maximum distance a simplified contour's border edges should deviate 
	// the original raw contour. [Limit: >=0] [Units: vx]
	float maxSimplificationError;
	int minRegionArea;				// The minimum number of cells allowed to form isolated island areas. [Limit: >=0] [Units: vx] 
	
	// Any regions with a span count smaller than this value will, if possible, 
	// be merged with larger regions. [Limit: >=0] [Units: vx] 
	int mergeRegionArea;
	
	// The maximum number of vertices allowed for polygons generated during the 
	// contour to polygon conversion process. [Limit: >= 3] 
	int maxVertsPerPoly;
	
	// Sets the sampling distance to use when generating the detail mesh.
	// (For height detail only.) [Limits: 0 or >= 0.9] [Units: wu] 
	float detailSampleDist;
	
	// The maximum distance the detail mesh surface should deviate from heightfield
	// data. (For height detail only.) [Limit: >=0] [Units: wu] 
	float detailSampleMaxError;
};

// Represents a span in a heightfield.
// @see Moss_RecastHeightfield
struct Moss_RecastSpan {
	uint32_t smin : RC_SPAN_HEIGHT_BITS; // The lower limit of the span. [Limit: < #smax]
	uint32_t smax : RC_SPAN_HEIGHT_BITS; // The upper limit of the span. [Limit: <= #RC_SPAN_MAX_HEIGHT]
	uint32_t area : 6;                   // The area id assigned to the span.
	Moss_RecastSpan* next = nullptr;                            // The next span higher up in column.
};

// A memory pool used for quick allocation of spans within a heightfield.
// @see Moss_RecastHeightfield
struct Moss_RecastSpanPool {
	Moss_RecastSpanPool* next;					// The next span pool.
	Moss_RecastSpan items[RC_SPANS_PER_POOL];	// Array of spans in the pool.
};

// A dynamic heightfield representing obstructed space.
// @ingroup recast
struct Moss_RecastHeightfield {
	Moss_RecastHeightfield();
	~Moss_RecastHeightfield();

	int width;			// The width of the heightfield. (Along the x-axis in cell units.)
	int height;			// The height of the heightfield. (Along the z-axis in cell units.)
	AABB3 bounds;
	float cs;			// The size of each cell. (On the xz-plane.)
	float ch;			// The height of each cell. (The minimum increment along the y-axis.)
	Moss_RecastSpan** spans;		// Heightfield of spans (width*height).

	// memory pool for Moss_RecastSpan instances.
	Moss_RecastSpanPool* pools;	// Linked list of span pools.
	Moss_RecastSpan* freelist;	// The next free span.

private:
	// Explicitly-disabled copy constructor and copy assignment operator.
	Moss_RecastHeightfield(const Moss_RecastHeightfield&) = delete;
	Moss_RecastHeightfield& operator=(const Moss_RecastHeightfield&) = delete;
};

// Represents a span of unobstructed space within a compact heightfield.
struct Moss_RecastCompactSpan {
	uint16_t y;			// The lower extent of the span. (Measured from the heightfield's base.)
	uint16_t reg;			// The id of the region the span belongs to. (Or zero if not in a region.)
	uint32_t con : 24;		// Packed neighbor connection data.
	uint32_t h : 8;			// The height of the span.  (Measured from #y.)
};
// Provides information on the content of a cell column in a compact heightfield. 
struct Moss_RecastCompactCell {
	uint32_t index : 24;	// Index to the first span in the column.
	uint32_t count : 8;		// Number of spans in the column.
};
// A compact, static heightfield representing unobstructed space.
// @ingroup recast
struct Moss_RecastCompactHeightfield {
	Moss_RecastCompactHeightfield();
	~Moss_RecastCompactHeightfield();
	
	int width;					// The width of the heightfield. (Along the x-axis in cell units.)
	int height;					// The height of the heightfield. (Along the z-axis in cell units.)
	int spanCount;				// The number of spans in the heightfield.
	int walkableHeight;			// The walkable height used during the build of the field.  (See: Moss_RecastSettings3D::walkableHeight)
	int walkableClimb;			// The walkable climb used during the build of the field. (See: Moss_RecastSettings3D::walkableClimb)
	int borderSize;				// The AABB border size used during the build of the field. (See: Moss_RecastSettings3D::borderSize)
	uint16_t maxDistance;	// The maximum distance value of any span within the field. 
	uint16_t maxRegions;	// The maximum region id of any span within the field. 
	AABB3 bounds;
	float cs;					// The size of each cell. (On the xz-plane.)
	float ch;					// The height of each cell. (The minimum increment along the y-axis.)
	Moss_RecastCompactCell* cells;		// Array of cells. [Size: #width*#height]
	Moss_RecastCompactSpan* spans;		// Array of spans. [Size: #spanCount]
	uint16_t* dist;		// Array containing border distance data. [Size: #spanCount]
	uint8_t* areas;		// Array containing area id data. [Size: #spanCount]
	
private:
	// Explicitly-disabled copy constructor and copy assignment operator.
	Moss_RecastCompactHeightfield(const Moss_RecastCompactHeightfield&);
	Moss_RecastCompactHeightfield& operator=(const Moss_RecastCompactHeightfield&);
};

// Represents a heightfield layer within a layer set.
// @see Moss_RecastHeightfieldLayerSet
struct Moss_RecastHeightfieldLayer {
	AABB3 bounds;					// bounds in world space.
	float cs;					// The size of each cell. (On the xz-plane.)
	float ch;					// The height of each cell. (The minimum increment along the y-axis.)
	int width;					// The width of the heightfield. (Along the x-axis in cell units.)
	int height;					// The height of the heightfield. (Along the z-axis in cell units.)
	int minx;					// The minimum x-bounds of usable data.
	int maxx;					// The maximum x-bounds of usable data.
	int miny;					// The minimum y-bounds of usable data. (Along the z-axis.)
	int maxy;					// The maximum y-bounds of usable data. (Along the z-axis.)
	int hmin;					// The minimum height bounds of usable data. (Along the y-axis.)
	int hmax;					// The maximum height bounds of usable data. (Along the y-axis.)
	uint8_t* heights;		// The heightfield. [Size: width * height]
	uint8_t* areas;		// Area ids. [Size: Same as #heights]
	uint8_t* cons;		// Packed neighbor connection information. [Size: Same as #heights]
};

// Represents a set of heightfield layers.
// @ingroup recast
// @see rcAllocHeightfieldLayerSet, rcFreeHeightfieldLayerSet 
struct Moss_RecastHeightfieldLayerSet {
	Moss_RecastHeightfieldLayerSet();
	~Moss_RecastHeightfieldLayerSet();
	
	Moss_RecastHeightfieldLayer* layers;			// The layers in the set. [Size: #nlayers]
	int nlayers;						// The number of layers in the set.
	
private:
	// Explicitly-disabled copy constructor and copy assignment operator.
	Moss_RecastHeightfieldLayerSet(const Moss_RecastHeightfieldLayerSet&) = delete;
	Moss_RecastHeightfieldLayerSet& operator=(const Moss_RecastHeightfieldLayerSet&) = delete;
};

// Represents a simple, non-overlapping contour in field space.
struct Moss_RecastContour {
	int* verts;			// Simplified contour vertex and connection data. [Size: 4 * #nverts]
	int nverts;			// The number of vertices in the simplified contour. 
	int* rverts;		// Raw contour vertex and connection data. [Size: 4 * #nrverts]
	int nrverts;		// The number of vertices in the raw contour. 
	uint16_t reg;	// The region id of the contour.
	uint8_t area;	// The area id of the contour.
};

// Represents a group of related contours.
// @ingroup recast
struct Moss_RecastContourSet {
	Moss_RecastContourSet();
	~Moss_RecastContourSet();
	
	Moss_RecastContour* conts;	// An array of the contours in the set. [Size: #nconts]
	int nconts;			// The number of contours in the set.
	AABB3 bounds;			// bounds in world space.
	float cs;			// The size of each cell. (On the xz-plane.)
	float ch;			// The height of each cell. (The minimum increment along the y-axis.)
	int width;			// The width of the set. (Along the x-axis in cell units.) 
	int height;			// The height of the set. (Along the z-axis in cell units.) 
	int borderSize;		// The AABB border size used to generate the source data from which the contours were derived.
	float maxError;		// The max edge error that this contour set was simplified with.
	
private:
	// Explicitly-disabled copy constructor and copy assignment operator.
	Moss_RecastContourSet(const Moss_RecastContourSet&) = delete;
	Moss_RecastContourSet& operator=(const Moss_RecastContourSet&) = delete;
};

// Represents a polygon mesh suitable for use in building a navigation mesh. 
// @ingroup recast
struct Moss_RecastPolyMesh {
	Moss_RecastPolyMesh();
	~Moss_RecastPolyMesh();
	
	uint16_t* verts;	// The mesh vertices. [Form: (x, y, z) * #nverts]
	uint16_t* polys;	// Polygon and neighbor data. [Length: #maxpolys * 2 * #nvp]
	uint16_t* regs;	// The region id assigned to each polygon. [Length: #maxpolys]
	uint16_t* flags;	// The user defined flags for each polygon. [Length: #maxpolys]
	uint8_t* areas;	// The area id assigned to each polygon. [Length: #maxpolys]
	int nverts;				// The number of vertices.
	int npolys;				// The number of polygons.
	int maxpolys;			// The number of allocated polygons.
	int nvp;				// The maximum number of vertices per polygon.
	AABB3 bounds;				// bounds in world space.
	float cs;				// The size of each cell. (On the xz-plane.)
	float ch;				// The height of each cell. (The minimum increment along the y-axis.)
	int borderSize;			// The AABB border size used to generate the source data from which the mesh was derived.
	float maxEdgeError;		// The max error of the polygon edges in the mesh.
	
private:
	// Explicitly-disabled copy constructor and copy assignment operator.
	Moss_RecastPolyMesh(const Moss_RecastPolyMesh&);
	Moss_RecastPolyMesh& operator=(const Moss_RecastPolyMesh&);
};

// Contains triangle meshes that represent detailed height data associated 
// with the polygons in its associated polygon mesh object.
// @ingroup recast
struct Moss_RecastPolyMeshDetail {
	Moss_RecastPolyMeshDetail();
	
	uint32_t* meshes;	// The sub-mesh data. [Size: 4*#nmeshes] 
	float* verts;			// The mesh vertices. [Size: 3*#nverts] 
	uint8_t* tris;	// The mesh triangles. [Size: 4*#ntris] 
	int nmeshes;			// The number of sub-meshes defined by #meshes.
	int nverts;				// The number of vertices in #verts.
	int ntris;				// The number of triangles in #tris.
	
private:
	// Explicitly-disabled copy constructor and copy assignment operator.
	Moss_RecastPolyMeshDetail(const Moss_RecastPolyMeshDetail&);
	Moss_RecastPolyMeshDetail& operator=(const Moss_RecastPolyMeshDetail&);
};


// @}
// @name Heightfield Functions
// @see Moss_RecastHeightfield

// Calculates the bounding box of an array of vertices.
// @ingroup recast
// @param verts		An array of vertices. [(x, y, z) * @p nv]
// @param numVerts	The number of vertices in the @p verts array.
// @param 	minBounds	The minimum bounds of the AABB. [(x, y, z)] [Units: wu]
// @param 	maxBounds	The maximum bounds of the AABB. [(x, y, z)] [Units: wu]
inline AABB3 rcCalcBounds(const Float3* verts, int numVerts)

// Calculates the grid size based on the bounding box and grid cell size.
// @ingroup recast
// @param minBounds	The minimum bounds of the AABB. [(x, y, z)] [Units: wu]
// @param maxBounds	The maximum bounds of the AABB. [(x, y, z)] [Units: wu]
// @param cellSize	The xz-plane cell size. [Limit: > 0] [Units: wu]
// @param 	sizeX		The width along the x-axis. [Limit: >= 0] [Units: vx]
// @param 	sizeZ		The height along the z-axis. [Limit: >= 0] [Units: vx]
inline void rcCalcGridSize(const AABB3& bounds, float cellSize, int& sizeX, int& sizeZ);

// Initializes a new heightfield.
// See the #Moss_RecastSettings3D documentation for more information on the configuration parameters.
// @see rcAllocHeightfield, Moss_RecastHeightfield
// @ingroup recast
// @param[in,out]	context		The build context to use during the operation.
// @param[in,out]	heightfield	The allocated heightfield to initialize.
// @param sizeX		The width of the field along the x-axis. [Limit: >= 0] [Units: vx]
// @param sizeZ		The height of the field along the z-axis. [Limit: >= 0] [Units: vx]
// @param minBounds	The minimum bounds of the field's AABB. [(x, y, z)] [Units: wu]
// @param maxBounds	The maximum bounds of the field's AABB. [(x, y, z)] [Units: wu]
// @param cellSize	The xz-plane cell size to use for the field. [Limit: > 0] [Units: wu]
// @param cellHeight	The y-axis cell size to use for field. [Limit: > 0] [Units: wu]
// @returns True if the operation completed successfully.
inline bool rcCreateHeightfield(Moss_RecastHeightfield& hf, int sizeX, int sizeZ, const AABB3& bounds, float cellSize, float cellHeight);
// Sets the area id of all triangles with a slope below the specified value
// to #RC_WALKABLE_AREA.
//
// Only sets the area id's for the walkable triangles.  Does not alter the
// area id's for un-walkable triangles.
// 
// See the #Moss_RecastSettings3D documentation for more information on the configuration parameters.
// 
// @see Moss_RecastHeightfield, rcClearUnwalkableTriangles, rcRasterizeTriangles
// 
// @ingroup recast
// @param[in,out]	context				The build context to use during the operation.
// @param walkableSlopeAngle	The maximum slope that is considered walkable.
// 									[Limits: 0 <= value < 90] [Units: Degrees]
// @param verts				The vertices. [(x, y, z) * @p nv]
// @param numVerts			The number of vertices.
// @param tris				The triangle vertex indices. [(vertA, vertB, vertC) * @p nt]
// @param numTris				The number of triangles.
// @param 	triAreaIDs			The triangle area ids. [Length: >= @p nt]
void rcMarkWalkableTriangles(float walkableSlopeAngle, const float* verts, int numVerts, const int* tris, int numTris, uint8_t* triAreaIDs); 
inline void rcMarkWalkableTriangles(float slope, const Float3* verts, int numVerts, const int* tris, int numTris, uint8_t* areas)
// Sets the area id of all triangles with a slope greater than or equal to the specified value to #RC_NULL_AREA.
// 
// Only sets the area id's for the un-walkable triangles.  Does not alter the
// area id's for walkable triangles.
// 
// See the #Moss_RecastSettings3D documentation for more information on the configuration parameters.
// 
// @see Moss_RecastHeightfield, rcClearUnwalkableTriangles, rcRasterizeTriangles
// 
// @ingroup recast
// @param[in,out]	context				The build context to use during the operation.
// @param walkableSlopeAngle	The maximum slope that is considered walkable.
// 									[Limits: 0 <= value < 90] [Units: Degrees]
// @param verts				The vertices. [(x, y, z) * @p nv]
// @param numVerts			The number of vertices.
// @param tris				The triangle vertex indices. [(vertA, vertB, vertC) * @p nt]
// @param numTris				The number of triangles.
// @param 	triAreaIDs			The triangle area ids. [Length: >= @p nt]
void rcClearUnwalkableTriangles(float walkableSlopeAngle, const float* verts, int numVerts, const int* tris, int numTris, uint8_t* triAreaIDs); 

// Adds a span to the specified heightfield.
// 
// The span addition can be set to favor flags. If the span is merged to
// another span and the new @p spanMax is within @p flagMergeThreshold units
// from the existing span, the span flags are merged.
// 
// @ingroup recast
// @param[in,out]	context				The build context to use during the operation.
// @param[in,out]	heightfield			An initialized heightfield.
// @param x					The column x index where the span is to be added.
// 									[Limits: 0 <= value < Moss_RecastHeightfield::width]
// @param z					The column z index where the span is to be added.
// 									[Limits: 0 <= value < Moss_RecastHeightfield::height]
// @param spanMin				The minimum height of the span. [Limit: < @p spanMax] [Units: vx]
// @param spanMax				The maximum height of the span. [Limit: <= #RC_SPAN_MAX_HEIGHT] [Units: vx]
// @param areaID				The area id of the span. [Limit: <= #RC_WALKABLE_AREA)
// @param flagMergeThreshold	The merge threshold. [Limit: >= 0] [Units: vx]
// @returns True if the operation completed successfully.
bool rcAddSpan(Moss_RecastHeightfield& heightfield, int x, int z, uint16_t spanMin, uint16_t spanMax, uint8_t areaID, int flagMergeThreshold);

// Rasterizes a single triangle into the specified heightfield.
//
// Calling this for each triangle in a mesh is less efficient than calling rcRasterizeTriangles
//
// No spans will be added if the triangle does not overlap the heightfield grid.
//
// @see Moss_RecastHeightfield
// @ingroup recast
// @param[in,out]	context				The build context to use during the operation.
// @param v0					Triangle vertex 0 [(x, y, z)]
// @param v1					Triangle vertex 1 [(x, y, z)]
// @param v2					Triangle vertex 2 [(x, y, z)]
// @param areaID				The area id of the triangle. [Limit: <= #RC_WALKABLE_AREA]
// @param[in,out]	heightfield			An initialized heightfield.
// @param flagMergeThreshold	The distance where the walkable flag is favored over the non-walkable flag.
// 									[Limit: >= 0] [Units: vx]
// @returns True if the operation completed successfully.
bool rcRasterizeTriangle(const float* v0, const float* v1, const float* v2, uint8_t areaID, Moss_RecastHeightfield& heightfield, int flagMergeThreshold = 1);

// Rasterizes an indexed triangle mesh into the specified heightfield.
//
// Spans will only be added for triangles that overlap the heightfield grid.
// 
// @see Moss_RecastHeightfield
// @ingroup recast
// @param[in,out]	context				The build context to use during the operation.
// @param verts				The vertices. [(x, y, z) * @p nv]
// @param numVerts			The number of vertices. (unused) TODO (graham): Remove in next major release
// @param tris				The triangle indices. [(vertA, vertB, vertC) * @p nt]
// @param triAreaIDs			The area id's of the triangles. [Limit: <= #RC_WALKABLE_AREA] [Size: @p nt]
// @param numTris				The number of triangles.
// @param[in,out]	heightfield			An initialized heightfield.
// @param flagMergeThreshold	The distance where the walkable flag is favored over the non-walkable flag. 
//										[Limit: >= 0] [Units: vx]
// @returns True if the operation completed successfully.
bool rcRasterizeTriangles(const float* verts, int numVerts, const int* tris, const uint8_t* triAreaIDs, int numTris, Moss_RecastHeightfield& heightfield, int flagMergeThreshold = 1);

// Rasterizes an indexed triangle mesh into the specified heightfield.
//
// Spans will only be added for triangles that overlap the heightfield grid.
// 
// @see Moss_RecastHeightfield
// @ingroup recast
// @param[in,out]	context				The build context to use during the operation.
// @param verts				The vertices. [(x, y, z) * @p nv]
// @param numVerts			The number of vertices. (unused) TODO (graham): Remove in next major release
// @param tris				The triangle indices. [(vertA, vertB, vertC) * @p nt]
// @param triAreaIDs			The area id's of the triangles. [Limit: <= #RC_WALKABLE_AREA] [Size: @p nt]
// @param numTris				The number of triangles.
// @param[in,out]	heightfield			An initialized heightfield.
// @param flagMergeThreshold	The distance where the walkable flag is favored over the non-walkable flag. 
// 									[Limit: >= 0] [Units: vx]
// @returns True if the operation completed successfully.
bool rcRasterizeTriangles(rcContext* context,
                          const float* verts, int numVerts,
                          const uint16_t* tris, const uint8_t* triAreaIDs, int numTris,
                          Moss_RecastHeightfield& heightfield, int flagMergeThreshold = 1);

// Rasterizes a triangle list into the specified heightfield.
//
// Expects each triangle to be specified as three sequential vertices of 3 floats.
//
// Spans will only be added for triangles that overlap the heightfield grid.
// 
// @see Moss_RecastHeightfield
// @ingroup recast
// @param[in,out]	context				The build context to use during the operation.
// @param verts				The triangle vertices. [(ax, ay, az, bx, by, bz, cx, by, cx) * @p nt]
// @param triAreaIDs			The area id's of the triangles. [Limit: <= #RC_WALKABLE_AREA] [Size: @p nt]
// @param numTris				The number of triangles.
// @param[in,out]	heightfield			An initialized heightfield.
// @param flagMergeThreshold	The distance where the walkable flag is favored over the non-walkable flag. 
// 									[Limit: >= 0] [Units: vx]
// @returns True if the operation completed successfully.
bool rcRasterizeTriangles(const float* verts, const uint8_t* triAreaIDs, int numTris, Moss_RecastHeightfield& heightfield, int flagMergeThreshold = 1);
inline bool rcRasterizeTriangle(const Float3& a, const Float3& b, const Float3& c, uint8_t area, Moss_RecastHeightfield& hf, int mergeThr = 1);
// Marks non-walkable spans as walkable if their maximum is within @p walkableClimb of the span below them.
//
// This removes small obstacles and rasterization artifacts that the agent would be able to walk over
// such as curbs.  It also allows agents to move up terraced structures like stairs.
// 
// Obstacle spans are marked walkable if: <tt>obstacleSpan.smax - walkableSpan.smax < walkableClimb</tt>
// 
// @warning Will override the effect of #rcFilterLedgeSpans.  If both filters are used, call #rcFilterLedgeSpans only after applying this filter.
//
// @see Moss_RecastHeightfield, Moss_RecastSettings3D
// 
// @ingroup recast
// @param[in,out]	context			The build context to use during the operation.
// @param walkableClimb	Maximum ledge height that is considered to still be traversable. 
// 								[Limit: >=0] [Units: vx]
// @param[in,out]	heightfield		A fully built heightfield.  (All spans have been added.)
void rcFilterLowHangingWalkableObstacles(int walkableClimb, Moss_RecastHeightfield& heightfield);

// Marks spans that are ledges as not-walkable.
//
// A ledge is a span with one or more neighbors whose maximum is further away than @p walkableClimb
// from the current span's maximum.
// This method removes the impact of the overestimation of conservative voxelization 
// so the resulting mesh will not have regions hanging in the air over ledges.
// 
// A span is a ledge if: <tt>abs(currentSpan.smax - neighborSpan.smax) > walkableClimb</tt>
// 
// @see Moss_RecastHeightfield, Moss_RecastSettings3D
// 
// @ingroup recast
// @param[in,out]	context				The build context to use during the operation.
// @param walkableHeight	Minimum floor to 'ceiling' height that will still allow the floor area to 
// 								be considered walkable. [Limit: >= 3] [Units: vx]
// @param walkableClimb	Maximum ledge height that is considered to still be traversable. 
// 								[Limit: >=0] [Units: vx]
// @param[in,out]	heightfield			A fully built heightfield.  (All spans have been added.)
void rcFilterLedgeSpans(int walkableHeight, int walkableClimb, Moss_RecastHeightfield& heightfield);

// Marks walkable spans as not walkable if the clearance above the span is less than the specified walkableHeight.
// 
// For this filter, the clearance above the span is the distance from the span's 
// maximum to the minimum of the next higher span in the same column.
// If there is no higher span in the column, the clearance is computed as the
// distance from the top of the span to the maximum heightfield height.
// 
// @see Moss_RecastHeightfield, Moss_RecastSettings3D
// @ingroup recast
// 
// @param[in,out]	context			The build context to use during the operation.
// @param walkableHeight	Minimum floor to 'ceiling' height that will still allow the floor area to 
// 								be considered walkable. [Limit: >= 3] [Units: vx]
// @param[in,out]	heightfield		A fully built heightfield.  (All spans have been added.)
void rcFilterWalkableLowHeightSpans(int walkableHeight, Moss_RecastHeightfield& heightfield);

// Returns the number of spans contained in the specified heightfield.
//  @ingroup recast
//  @param[in,out]	context		The build context to use during the operation.
//  @param heightfield	An initialized heightfield.
//  @returns The number of spans in the heightfield.
int rcGetHeightFieldSpanCount(const Moss_RecastHeightfield& heightfield);

// @}
// @name Compact Heightfield Functions
// @see Moss_RecastCompactHeightfield

// Builds a compact heightfield representing open space, from a heightfield representing solid space.
//
// This is just the beginning of the process of fully building a compact heightfield.
// Various filters may be applied, then the distance field and regions built.
// E.g: #rcBuildDistanceField and #rcBuildRegions
//
// See the #Moss_RecastSettings3D documentation for more information on the configuration parameters.
//
// @see rcAllocCompactHeightfield, Moss_RecastHeightfield, Moss_RecastCompactHeightfield, Moss_RecastSettings3D
// @ingroup recast
// 
// @param[in,out]	context				The build context to use during the operation.
// @param walkableHeight		Minimum floor to 'ceiling' height that will still allow the floor area 
// 									to be considered walkable. [Limit: >= 3] [Units: vx]
// @param walkableClimb		Maximum ledge height that is considered to still be traversable. 
// 									[Limit: >=0] [Units: vx]
// @param heightfield			The heightfield to be compacted.
// @param 	compactHeightfield	The resulting compact heightfield. (Must be pre-allocated.)
// @returns True if the operation completed successfully.
bool rcBuildCompactHeightfield(int walkableHeight, int walkableClimb, const Moss_RecastHeightfield& heightfield, Moss_RecastCompactHeightfield& compactHeightfield);

// Erodes the walkable area within the heightfield by the specified radius.
// 
// Basically, any spans that are closer to a boundary or obstruction than the specified radius 
// are marked as un-walkable.
//
// This method is usually called immediately after the heightfield has been built.
// 
// @see Moss_RecastCompactHeightfield, rcBuildCompactHeightfield, Moss_RecastSettings3D::walkableRadius
// @ingroup recast
//
// @param[in,out]	context				The build context to use during the operation.
// @param erosionRadius		The radius of erosion. [Limits: 0 < value < 255] [Units: vx]
// @param[in,out]	compactHeightfield	The populated compact heightfield to erode.
// @returns True if the operation completed successfully.
bool rcErodeWalkableArea(int erosionRadius, Moss_RecastCompactHeightfield& compactHeightfield);

// Applies a median filter to walkable area types (based on area id), removing noise.
// 
// This filter is usually applied after applying area id's using functions
// such as #rcMarkBoxArea, #rcMarkConvexPolyArea, and #rcMarkCylinderArea.
// 
// @see Moss_RecastCompactHeightfield
// @ingroup recast
// 
// @param[in,out]	context		The build context to use during the operation.
// @param[in,out]	compactHeightfield		A populated compact heightfield.
// @returns True if the operation completed successfully.
bool rcMedianFilterWalkableArea(Moss_RecastCompactHeightfield& compactHeightfield);

// Applies an area id to all spans within the specified bounding box. (AABB) 
// 
// @see Moss_RecastCompactHeightfield, rcMedianFilterWalkableArea
// @ingroup recast
// 
// @param[in,out]	context				The build context to use during the operation.
// @param boxMinBounds		The minimum extents of the bounding box. [(x, y, z)] [Units: wu]
// @param boxMaxBounds		The maximum extents of the bounding box. [(x, y, z)] [Units: wu]
// @param areaId				The area id to apply. [Limit: <= #RC_WALKABLE_AREA]
// @param[in,out]	compactHeightfield	A populated compact heightfield.
void rcMarkBoxArea(const float* boxMinBounds, const float* boxMaxBounds, uint8_t areaId, Moss_RecastCompactHeightfield& compactHeightfield);
inline void rcMarkBoxArea(const AABB3& bounds, uint8_t area, Moss_RecastCompactHeightfield& chf)
// Applies the area id to the all spans within the specified convex polygon. 
//
// The value of spacial parameters are in world units.
// 
// The y-values of the polygon vertices are ignored. So the polygon is effectively 
// projected onto the xz-plane, translated to @p minY, and extruded to @p maxY.
// 
// @see Moss_RecastCompactHeightfield, rcMedianFilterWalkableArea
// @ingroup recast
// 
// @param[in,out]	context				The build context to use during the operation.
// @param verts				The vertices of the polygon [For: (x, y, z) * @p numVerts]
// @param numVerts			The number of vertices in the polygon.
// @param minY				The height of the base of the polygon. [Units: wu]
// @param maxY				The height of the top of the polygon. [Units: wu]
// @param areaId				The area id to apply. [Limit: <= #RC_WALKABLE_AREA]
// @param[in,out]	compactHeightfield	A populated compact heightfield.
void rcMarkConvexPolyArea(const float* verts, int numVerts, float minY, float maxY, uint8_t areaId, Moss_RecastCompactHeightfield& compactHeightfield);

// Expands a convex polygon along its vertex normals by the given offset amount.
// Inserts extra vertices to bevel sharp corners.
//
// Helper function to offset convex polygons for rcMarkConvexPolyArea.
//
// @ingroup recast
// 
// @param verts		The vertices of the polygon [Form: (x, y, z) * @p numVerts]
// @param numVerts	The number of vertices in the polygon.
// @param offset		How much to offset the polygon by. [Units: wu]
// @param 	outVerts	The offset vertices (should hold up to 2 * @p numVerts) [Form: (x, y, z) * return value]
// @param maxOutVerts	The max number of vertices that can be stored to @p outVerts.
// @returns Number of vertices in the offset polygon or 0 if too few vertices in @p outVerts.
int rcOffsetPoly(const float* verts, int numVerts, float offset, float* outVerts, int maxOutVerts);

int rcOffsetPolyXZ(const Vec2* verts, int numVerts, float offset,
                   Vec2* outVerts, int maxOutVerts);

// Applies the area id to all spans within the specified y-axis-aligned cylinder.
// 
// @see Moss_RecastCompactHeightfield, rcMedianFilterWalkableArea
// 
// @ingroup recast
// 
// @param[in,out]	context				The build context to use during the operation.
// @param position			The center of the base of the cylinder. [Form: (x, y, z)] [Units: wu] 
// @param radius				The radius of the cylinder. [Units: wu] [Limit: > 0]
// @param height				The height of the cylinder. [Units: wu] [Limit: > 0]
// @param areaId				The area id to apply. [Limit: <= #RC_WALKABLE_AREA]
// @param[in,out]	compactHeightfield	A populated compact heightfield.
void rcMarkCylinderArea(const float* position, float radius, float height, uint8_t areaId, Moss_RecastCompactHeightfield& compactHeightfield);

// Builds the distance field for the specified compact heightfield. 
// @ingroup recast
// @param[in,out]	ctx		The build context to use during the operation.
// @param[in,out]	chf		A populated compact heightfield.
// @returns True if the operation completed successfully.
bool rcBuildDistanceField(Moss_RecastCompactHeightfield& chf);

// Builds region data for the heightfield using watershed partitioning.
// @ingroup recast
// @param[in,out]	ctx				The build context to use during the operation.
// @param[in,out]	chf				A populated compact heightfield.
// @param borderSize		The size of the non-navigable border around the heightfield.
// 								[Limit: >=0] [Units: vx]
// @param minRegionArea	The minimum number of cells allowed to form isolated island areas.
// 								[Limit: >=0] [Units: vx].
// @param mergeRegionArea	Any regions with a span count smaller than this value will, if possible,
// 								be merged with larger regions. [Limit: >=0] [Units: vx] 
// @returns True if the operation completed successfully.
bool rcBuildRegions(Moss_RecastCompactHeightfield& chf, int borderSize, int minRegionArea, int mergeRegionArea);

// Builds region data for the heightfield by partitioning the heightfield in non-overlapping layers.
// @ingroup recast
// @param[in,out]	ctx				The build context to use during the operation.
// @param[in,out]	chf				A populated compact heightfield.
// @param borderSize		The size of the non-navigable border around the heightfield.
//  								[Limit: >=0] [Units: vx]
// @param minRegionArea	The minimum number of cells allowed to form isolated island areas.
//  								[Limit: >=0] [Units: vx].
// @returns True if the operation completed successfully.
bool rcBuildLayerRegions(Moss_RecastCompactHeightfield& chf, int borderSize, int minRegionArea);

// Builds region data for the heightfield using simple monotone partitioning.
// @ingroup recast 
// @param[in,out]	ctx				The build context to use during the operation.
// @param[in,out]	chf				A populated compact heightfield.
// @param borderSize		The size of the non-navigable border around the heightfield.
//  								[Limit: >=0] [Units: vx]
// @param minRegionArea	The minimum number of cells allowed to form isolated island areas.
//  								[Limit: >=0] [Units: vx].
// @param mergeRegionArea	Any regions with a span count smaller than this value will, if possible, 
//  								be merged with larger regions. [Limit: >=0] [Units: vx] 
// @returns True if the operation completed successfully.
bool rcBuildRegionsMonotone(Moss_RecastCompactHeightfield& chf, int borderSize, int minRegionArea, int mergeRegionArea);


// @}
// @name Layer, Contour, Polymesh, and Detail Mesh Functions
// @see Moss_RecastHeightfieldLayer, Moss_RecastContourSet, Moss_RecastPolyMesh, Moss_RecastPolyMeshDetail

// Builds a layer set from the specified compact heightfield.
// @ingroup recast
// @param[in,out]	ctx				The build context to use during the operation.
// @param chf				A fully built compact heightfield.
// @param borderSize		The size of the non-navigable border around the heightfield. [Limit: >=0] 
//  								[Units: vx]
// @param walkableHeight	Minimum floor to 'ceiling' height that will still allow the floor area 
//  								to be considered walkable. [Limit: >= 3] [Units: vx]
// @param 	lset			The resulting layer set. (Must be pre-allocated.)
// @returns True if the operation completed successfully.
bool rcBuildHeightfieldLayers(const Moss_RecastCompactHeightfield& chf,  int borderSize, int walkableHeight, Moss_RecastHeightfieldLayerSet& lset);

// Builds a contour set from the region outlines in the provided compact heightfield.
// @ingroup recast
// @param[in,out]	ctx			The build context to use during the operation.
// @param chf			A fully built compact heightfield.
// @param maxError	The maximum distance a simplified contour's border edges should deviate 
// 							the original raw contour. [Limit: >=0] [Units: wu]
// @param maxEdgeLen	The maximum allowed length for contour edges along the border of the mesh. 
// 							[Limit: >=0] [Units: vx]
// @param 	cset		The resulting contour set. (Must be pre-allocated.)
// @param buildFlags	The build flags. (See: #rcBuildContoursFlags)
// @returns True if the operation completed successfully.
bool rcBuildContours(const Moss_RecastCompactHeightfield& chf, float maxError, int maxEdgeLen, Moss_RecastContourSet& cset, int buildFlags = RC_CONTOUR_TESS_WALL_EDGES);

// Builds a polygon mesh from the provided contours.
// @ingroup recast
// @param[in,out]	ctx		The build context to use during the operation.
// @param cset	A fully built contour set.
// @param nvp		The maximum number of vertices allowed for polygons generated during the 
// 						contour to polygon conversion process. [Limit: >= 3] 
// @param 	mesh	The resulting polygon mesh. (Must be re-allocated.)
// @returns True if the operation completed successfully.
bool rcBuildPolyMesh(const Moss_RecastContourSet& cset, const int nvp, Moss_RecastPolyMesh& mesh);

// Merges multiple polygon meshes into a single mesh.
//  @ingroup recast
//  @param[in,out]	ctx		The build context to use during the operation.
//  @param meshes	An array of polygon meshes to merge. [Size: @p nmeshes]
//  @param nmeshes	The number of polygon meshes in the meshes array.
//  @param mesh	The resulting polygon mesh. (Must be pre-allocated.)
//  @returns True if the operation completed successfully.
bool rcMergePolyMeshes(Moss_RecastPolyMesh** meshes, const int nmeshes, Moss_RecastPolyMesh& mesh);

// Builds a detail mesh from the provided polygon mesh.
// @ingroup recast
// @param[in,out]	ctx				The build context to use during the operation.
// @param mesh			A fully built polygon mesh.
// @param chf				The compact heightfield used to build the polygon mesh.
// @param sampleDist		Sets the distance to use when sampling the heightfield. [Limit: >=0] [Units: wu]
// @param sampleMaxError	The maximum distance the detail mesh surface should deviate from 
// 								heightfield data. [Limit: >=0] [Units: wu]
// @param 	dmesh			The resulting detail mesh.  (Must be pre-allocated.)
// @returns True if the operation completed successfully.
bool rcBuildPolyMeshDetail(const Moss_RecastPolyMesh& mesh, const Moss_RecastCompactHeightfield& chf, float sampleDist, float sampleMaxError, Moss_RecastPolyMeshDetail& dmesh);

// Copies the poly mesh data from src to dst.
// @ingroup recast
// @param[in,out]	ctx		The build context to use during the operation.
// @param src		The source mesh to copy from.
// @param 	dst		The resulting detail mesh. (Must be pre-allocated, must be empty mesh.)
// @returns True if the operation completed successfully.
bool rcCopyPolyMesh(const Moss_RecastPolyMesh& src, Moss_RecastPolyMesh& dst);

// Merges multiple detail meshes into a single detail mesh.
// @ingroup recast
// @param[in,out]	ctx		The build context to use during the operation.
// @param meshes	An array of detail meshes to merge. [Size: @p nmeshes]
// @param nmeshes	The number of detail meshes in the meshes array.
// @param 	mesh	The resulting detail mesh. (Must be pre-allocated.)
// @returns True if the operation completed successfully.
bool rcMergePolyMeshDetails(Moss_RecastPolyMeshDetail** meshes, const int nmeshes, Moss_RecastPolyMeshDetail& mesh);





// Represents a dynamic polygon corridor used to plan agent movement.
// @ingroup crowd, detour
class Moss_RecastPathCorridor {
public:
	Moss_RecastPathCorridor();
	~Moss_RecastPathCorridor();
	
	// Allocates the corridor's path buffer. 
	//  @param maxPath		The maximum path size the corridor can handle.
	// @return True if the initialization succeeded.
	bool init(const int maxPath);
	
	// Resets the path corridor to the specified position.
	//  @param ref		The polygon reference containing the position.
	//  @param pos		The new position in the corridor. [(x, y, z)]
	void reset(Moss_RecastPolyRef ref, const float* pos);
	
	// Finds the corners in the corridor from the position toward the target. (The straightened path.)
	//  @param cornerVerts		The corner vertices. [(x, y, z) * cornerCount] [Size: <= maxCorners]
	//  @param cornerFlags		The flag for each corner. [(flag) * cornerCount] [Size: <= maxCorners]
	//  @param cornerPolys		The polygon reference for each corner. [(polyRef) * cornerCount] 
	//  								[Size: <= @p maxCorners]
	//  @param maxCorners		The maximum number of corners the buffers can hold.
	//  @param navquery		The query object used to build the corridor.
	//  @param filter			The filter to apply to the operation.
	// @return The number of corners returned in the corner buffers. [0 <= value <= @p maxCorners]
	int findCorners(float* cornerVerts, uint8_t* cornerFlags, Moss_RecastPolyRef* cornerPolys, const int maxCorners, Moss_RecastNavMeshQuery* navquery, const Moss_RecastQueryFilter* filter);
	
	// Attempts to optimize the path if the specified point is visible from the current position.
	//  @param next					The point to search toward. [(x, y, z])
	//  @param pathOptimizationRange	The maximum range to search. [Limit: > 0]
	//  @param navquery				The query object used to build the corridor.
	//  @param filter					The filter to apply to the operation.			
	void optimizePathVisibility(const float* next, const float pathOptimizationRange, Moss_RecastNavMeshQuery* navquery, const Moss_RecastQueryFilter* filter);
	
	// Attempts to optimize the path using a local area search. (Partial replanning.) 
	//  @param navquery	The query object used to build the corridor.
	//  @param filter		The filter to apply to the operation.	
	bool optimizePathTopology(Moss_RecastNavMeshQuery* navquery, const Moss_RecastQueryFilter* filter);
	
	bool moveOverOffmeshConnection(Moss_RecastPolyRef offMeshConRef, Moss_RecastPolyRef* refs,
								   float* startPos, float* endPos,
								   Moss_RecastNavMeshQuery* navquery);

	bool fixPathStart(Moss_RecastPolyRef safeRef, const float* safePos);

	bool trimInvalidPath(Moss_RecastPolyRef safeRef, const float* safePos, Moss_RecastNavMeshQuery* navquery, const Moss_RecastQueryFilter* filter);
	
	// Checks the current corridor path to see if its polygon references remain valid. 
	//  @param maxLookAhead	The number of polygons from the beginning of the corridor to search.
	//  @param navquery		The query object used to build the corridor.
	//  @param filter			The filter to apply to the operation.	
	bool isValid(const int maxLookAhead, Moss_RecastNavMeshQuery* navquery, const Moss_RecastQueryFilter* filter);
	
	// Moves the position from the current location to the desired location, adjusting the corridor 
	// as needed to reflect the change.
	//  @param npos		The desired new position. [(x, y, z)]
	//  @param navquery	The query object used to build the corridor.
	//  @param filter		The filter to apply to the operation.
	// @return Returns true if move succeeded.
	bool movePosition(const float* npos, Moss_RecastNavMeshQuery* navquery, const Moss_RecastQueryFilter* filter);

	// Moves the target from the curent location to the desired location, adjusting the corridor
	// as needed to reflect the change. 
	//  @param npos		The desired new target position. [(x, y, z)]
	//  @param navquery	The query object used to build the corridor.
	//  @param filter		The filter to apply to the operation.
	// @return Returns true if move succeeded.
	bool moveTargetPosition(const float* npos, Moss_RecastNavMeshQuery* navquery, const Moss_RecastQueryFilter* filter);
	
	// Loads a new path and target into the corridor.
	//  @param target		The target location within the last polygon of the path. [(x, y, z)]
	//  @param path		The path corridor. [(polyRef) * @p npolys]
	//  @param npath		The number of polygons in the path.
	void setCorridor(const float* target, const Moss_RecastPolyRef* polys, const int npath);
	
	// Gets the current position within the corridor. (In the first polygon.)
	// @return The current position within the corridor.
	inline const Float3 getPos() const { return m_pos; }

	// Gets the current target within the corridor. (In the last polygon.)
	// @return The current target within the corridor.
	inline const Float3 getTarget() const { return m_target; }
	
	// The polygon reference id of the first polygon in the corridor, the polygon containing the position.
	// @return The polygon reference id of the first polygon in the corridor. (Or zero if there is no path.)
	inline Moss_RecastPolyRef getFirstPoly() const { return m_npath ? m_path[0] : 0; }

	// The polygon reference id of the last polygon in the corridor, the polygon containing the target.
	// @return The polygon reference id of the last polygon in the corridor. (Or zero if there is no path.)
	inline Moss_RecastPolyRef getLastPoly() const { return m_npath ? m_path[m_npath-1] : 0; }
	
	// The corridor's path.
	// @return The corridor's path. [(polyRef) * #getPathCount()]
	inline const Moss_RecastPolyRef* getPath() const { return m_path; }

	// The number of polygons in the current corridor path.
	// @return The number of polygons in the current corridor path.
	inline int getPathCount() const { return m_npath; }

private:
	Float3 m_pos;
	Float3 m_target;
	
	Moss_RecastPolyRef* m_path;
	int m_npath;
	int m_maxPath;
	// Explicitly disabled copy constructor and copy assignment operator.
	Moss_RecastPathCorridor(const Moss_RecastPathCorridor&);
	Moss_RecastPathCorridor& operator=(const Moss_RecastPathCorridor&);
};

int dtMergeCorridorStartMoved(Moss_RecastPolyRef* path, const int npath, const int maxPath, const Moss_RecastPolyRef* visited, const int nvisited);

int dtMergeCorridorEndMoved(Moss_RecastPolyRef* path, const int npath, const int maxPath, const Moss_RecastPolyRef* visited, const int nvisited);

int dtMergeCorridorStartShortcut(Moss_RecastPolyRef* path, const int npath, const int maxPath, const Moss_RecastPolyRef* visited, const int nvisited);


class Moss_RecastProximityGrid {
	float m_cellSize;
	float m_invCellSize;
	
	struct Item {
		uint16_t id;
		short x,y;
		uint16_t next;
	};
	Item* m_pool;
	int m_poolHead;
	int m_poolSize;
	
	uint16_t* m_buckets;
	int m_bucketsSize;
	
	int m_bounds[4];
	
public:
	Moss_RecastProximityGrid();
	~Moss_RecastProximityGrid();
	
	bool init(const int poolSize, const float cellSize);
	
	void clear();
	
	void addItem(const uint16_t id, const float minx, const float miny, const float maxx, const float maxy);
	
	int queryItems(const float minx, const float miny, const float maxx, const float maxy, uint16_t* ids, const int maxIds) const;
	
	int getItemCountAt(const int x, const int y) const;
	
	inline const int* getBounds() const { return m_bounds; }
	inline float getCellSize() const { return m_cellSize; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	Moss_RecastProximityGrid(const Moss_RecastProximityGrid&) = delete;
	Moss_RecastProximityGrid& operator=(const Moss_RecastProximityGrid&) = delete;
};

class Moss_RecastPathQueue {
public:
	Moss_RecastPathQueue();
	~Moss_RecastPathQueue();
	
	bool init(const int maxPathSize, const int maxSearchNodeCount, Moss_RecastNavMesh* nav);
	
	void update(const int maxIters);
	
	Moss_RecastPathQueueRef request(Moss_RecastPolyRef startRef, Moss_RecastPolyRef endRef, const float* startPos, const float* endPos,  const Moss_RecastQueryFilter* filter);
	
	Moss_RecastStatus getRequestStatus(Moss_RecastPathQueueRef ref) const;
	
	Moss_RecastStatus getPathResult(Moss_RecastPathQueueRef ref, Moss_RecastPolyRef* path, int* pathSize, const int maxPath);
	
	inline const Moss_RecastNavMeshQuery* getNavQuery() const { return m_navquery; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	Moss_RecastPathQueue(const Moss_RecastPathQueue&);
	Moss_RecastPathQueue& operator=(const Moss_RecastPathQueue&);

	struct PathQuery {
		Moss_RecastPathQueueRef ref;
		// Path find start and end location.
		Float3 startPos, endPos;
		Moss_RecastPolyRef startRef, endRef;
		// Result.
		Moss_RecastPolyRef* path;
		int npath;
		// State.
		Moss_RecastStatus status;
		int keepAlive;
		const Moss_RecastQueryFilter* filter; // TODO: This is potentially dangerous!
	};
	
	static const int MAX_QUEUE = 8;
	PathQuery m_queue[MAX_QUEUE];
	Moss_RecastPathQueueRef m_nextHandle;
	int m_maxPathSize;
	int m_queueHead;
	Moss_RecastNavMeshQuery* m_navquery;
	
	void purge();
};


class Moss_RecastLocalBoundary {
public:
	Moss_RecastLocalBoundary();
	~Moss_RecastLocalBoundary();
	
	void reset();
	
	void update(Moss_RecastPolyRef ref, const float* pos, const float collisionQueryRange, Moss_RecastNavMeshQuery* navquery, const Moss_RecastQueryFilter* filter);
	
	bool isValid(Moss_RecastNavMeshQuery* navquery, const Moss_RecastQueryFilter* filter);
	
	inline const Float3 getCenter() const { return m_center; }
	inline int getSegmentCount() const { return m_nsegs; }
	inline const float* getSegment(int i) const { return m_segs[i].s; }

private:
	static const int MAX_LOCAL_SEGS = 8;
	static const int MAX_LOCAL_POLYS = 16;
	
	struct Segment {
		float s[6];	// Segment start/end
		float d;	// Distance for pruning.
	};
	
	Float3 m_center;
	Segment m_segs[MAX_LOCAL_SEGS];
	int m_nsegs;
	
	Moss_RecastPolyRef m_polys[MAX_LOCAL_POLYS];
	int m_npolys;

	void addSegment(const float dist, const float* s);
	
	// Explicitly disabled copy constructor and copy assignment operator.
	Moss_RecastLocalBoundary(const Moss_RecastLocalBoundary&) = delete;
	Moss_RecastLocalBoundary& operator=(const Moss_RecastLocalBoundary&) = delete;
};

// Undefine (or define in a build config) the following line to use 64bit polyref.
// Generally not needed, useful for very large worlds.
// Note: tiles build using 32bit refs are not compatible with 64bit refs!
//#define DT_POLYREF64 1

// Defines a polygon within a Moss_RecastMeshTile object.
// @ingroup detour
struct Moss_RecastPoly {
	// Index to first link in linked list. (Or #DT_NULL_LINK if there is no link.)
	uint32_t firstLink;

	// The indices of the polygon's vertices.
	// The actual vertices are located in Moss_RecastMeshTile::verts.
	uint16_t verts[DT_VERTS_PER_POLYGON];

	// Packed data representing neighbor polygons references and flags for each edge.
	uint16_t neis[DT_VERTS_PER_POLYGON];

	// The user defined polygon flags.
	uint16_t flags;

	// The number of vertices in the polygon.
	uint8_t vertCount;

	// The bit packed area id and polygon type.
	// @note Use the structure's set and get methods to access this value.
	uint8_t areaAndtype;

	// Sets the user defined area id. [Limit: < #DT_MAX_AREAS]
	inline void setArea(uint8_t a) { areaAndtype = (areaAndtype & 0xc0) | (a & 0x3f); }

	// Sets the polygon type. (See: #dtPolyTypes.)
	inline void setType(uint8_t t) { areaAndtype = (areaAndtype & 0x3f) | (t << 6); }

	// Gets the user defined area id.
	inline uint8_t getArea() const { return areaAndtype & 0x3f; }

	// Gets the polygon type. (See: #dtPolyTypes)
	inline uint8_t getType() const { return areaAndtype >> 6; }
};

// Defines the location of detail sub-mesh data within a Moss_RecastMeshTile.
struct Moss_RecastPolyDetail {
	uint32_t vertBase;			// The offset of the vertices in the Moss_RecastMeshTile::detailVerts array.
	uint32_t triBase;			// The offset of the triangles in the Moss_RecastMeshTile::detailTris array.
	uint8_t vertCount;		// The number of vertices in the sub-mesh.
	uint8_t triCount;			// The number of triangles in the sub-mesh.
};

// Defines a link between polygons.
// @note This structure is rarely if ever used by the end user.
// @see Moss_RecastMeshTile
struct Moss_RecastLink {
	Moss_RecastPolyRef ref;					// Neighbour reference. (The neighbor that is linked to.)
	uint32_t next;				// Index of the next link.
	uint8_t edge;				// Index of the polygon edge that owns this link.
	uint8_t side;				// If a boundary link, defines on which side the link is.
	uint8_t bmin;				// If a boundary link, defines the minimum sub-edge area.
	uint8_t bmax;				// If a boundary link, defines the maximum sub-edge area.
};

// Bounding volume node.
// @note This structure is rarely if ever used by the end user.
// @see Moss_RecastMeshTile
struct Moss_RecastBVNode {
	uint16_t bmin[3];			// Minimum bounds of the node's AABB. [(x, y, z)]
	uint16_t bmax[3];			// Maximum bounds of the node's AABB. [(x, y, z)]
	int i;							// The node's index. (Negative for escape sequence.)
};

// Defines an navigation mesh off-mesh connection within a Moss_RecastMeshTile object.
// An off-mesh connection is a user defined traversable connection made up to two vertices.
struct Moss_RecastOffMeshConnection {
	// The endpoints of the connection. [(ax, ay, az, bx, by, bz)]
	float pos[6];

	// The radius of the endpoints. [Limit: >= 0]
	float rad;		

	// The polygon reference of the connection within the tile.
	uint16_t poly;

	// Link flags. 
	// @note These are not the connection's user defined flags. Those are assigned via the 
	// connection's Moss_RecastPoly definition. These are link flags used for internal purposes.
	uint8_t flags;

	// End point side.
	uint8_t side;

	// The id of the offmesh connection. (User assigned when the navigation mesh is built.)
	uint32_t userId;
};

// Provides high level information related to a Moss_RecastMeshTile object.
// @ingroup detour
struct Moss_RecastMeshHeader {
	int magic;				// Tile magic number. (Used to identify the data format.)
	int version;			// Tile data format version number.
	int x;					// The x-position of the tile within the Moss_RecastNavMesh tile grid. (x, y, layer)
	int y;					// The y-position of the tile within the Moss_RecastNavMesh tile grid. (x, y, layer)
	int layer;				// The layer of the tile within the Moss_RecastNavMesh tile grid. (x, y, layer)
	uint32_t userId;	// The user defined id of the tile.
	int polyCount;			// The number of polygons in the tile.
	int vertCount;			// The number of vertices in the tile.
	int maxLinkCount;		// The number of allocated links.
	int detailMeshCount;	// The number of sub-meshes in the detail mesh.
	
	// The number of unique vertices in the detail mesh. (In addition to the polygon vertices.)
	int detailVertCount;
	
	int detailTriCount;			// The number of triangles in the detail mesh.
	int bvNodeCount;			// The number of bounding volume nodes. (Zero if bounding volumes are disabled.)
	int offMeshConCount;		// The number of off-mesh connections.
	int offMeshBase;			// The index of the first polygon which is an off-mesh connection.
	float walkableHeight;		// The height of the agents using the tile.
	float walkableRadius;		// The radius of the agents using the tile.
	float walkableClimb;		// The maximum climb height of the agents using the tile.
	AABB3 bounds;
	
	// The bounding volume quantization factor. 
	float bvQuantFactor;
};

// Defines a navigation mesh tile.
// @ingroup detour
struct Moss_RecastMeshTile {
	uint32_t salt;					// Counter describing modifications to the tile.

	uint32_t linksFreeList;			// Index to the next free link.
	Moss_RecastMeshHeader* header;				// The tile header.
	Moss_RecastPoly* polys;						// The tile polygons. [Size: Moss_RecastMeshHeader::polyCount]
	float* verts;						// The tile vertices. [(x, y, z) * Moss_RecastMeshHeader::vertCount]
	Moss_RecastLink* links;						// The tile links. [Size: Moss_RecastMeshHeader::maxLinkCount]
	Moss_RecastPolyDetail* detailMeshes;			// The tile's detail sub-meshes. [Size: Moss_RecastMeshHeader::detailMeshCount]
	
	// The detail mesh's unique vertices. [(x, y, z) * Moss_RecastMeshHeader::detailVertCount]
	float* detailVerts;	

	// The detail mesh's triangles. [(vertA, vertB, vertC, triFlags) * Moss_RecastMeshHeader::detailTriCount].
	// See dtDetailTriEdgeFlags and dtGetDetailTriEdgeFlags.
	uint8_t* detailTris;	

	// The tile bounding volume nodes. [Size: Moss_RecastMeshHeader::bvNodeCount]
	// (Will be null if bounding volumes are disabled.)
	Moss_RecastBVNode* bvTree;

	Moss_RecastOffMeshConnection* offMeshCons;		// The tile off-mesh connections. [Size: Moss_RecastMeshHeader::offMeshConCount]
		
	uint8_t* data;					// The tile data. (Not directly accessed under normal situations.)
	int dataSize;							// Size of the tile data.
	int flags;								// Tile flags. (See: #dtTileFlags)
	Moss_RecastMeshTile* next;						// The next free tile, or the next tile in the spatial grid.
private:
	Moss_RecastMeshTile(const Moss_RecastMeshTile&);
	Moss_RecastMeshTile& operator=(const Moss_RecastMeshTile&);
};


// Configuration parameters used to define multi-tile navigation meshes.
// The values are used to allocate space during the initialization of a navigation mesh.
// @see Moss_RecastNavMesh::init()
// @ingroup detour
struct Moss_RecastNavMeshParams {
	Float3 orig;					// The world space origin of the navigation mesh's tile space. [(x, y, z)]
	float tileWidth;				// The width of each tile. (Along the x-axis.)
	float tileHeight;				// The height of each tile. (Along the z-axis.)
	int maxTiles;					// The maximum number of tiles the navigation mesh can contain. This and maxPolys are used to calculate how many bits are needed to identify tiles and polygons uniquely.
	int maxPolys;					// The maximum number of polygons each tile can contain. This and maxTiles are used to calculate how many bits are needed to identify tiles and polygons uniquely.
};

// A navigation mesh based on tiles of convex polygons.
// @ingroup detour
class Moss_RecastNavMesh {
public:
	Moss_RecastNavMesh();
	~Moss_RecastNavMesh();

	// @name Initialization and Tile Management

	// Initializes the navigation mesh for tiled use.
	//  @param	params		Initialization parameters.
	// @return The status flags for the operation.
	Moss_RecastStatus init(const Moss_RecastNavMeshParams* params);

	// Initializes the navigation mesh for single tile use.
	//  @param	data		Data of the new tile. (See: #dtCreateNavMeshData)
	//  @param	dataSize	The data size of the new tile.
	//  @param	flags		The tile flags. (See: #dtTileFlags)
	// @return The status flags for the operation.
	//  @see dtCreateNavMeshData
	Moss_RecastStatus init(uint8_t* data, const int dataSize, const int flags);
	
	// The navigation mesh initialization params.
	const Moss_RecastNavMeshParams* getParams() const;

	// Adds a tile to the navigation mesh.
	//  @param data		Data for the new tile mesh. (See: #dtCreateNavMeshData)
	//  @param dataSize	Data size of the new tile mesh.
	//  @param flags		Tile flags. (See: #dtTileFlags)
	//  @param lastRef		The desired reference for the tile. (When reloading a tile.) [opt] [Default: 0]
	//  @param result		The tile reference. (If the tile was succesfully added.) [opt]
	// @return The status flags for the operation.
	Moss_RecastStatus addTile(uint8_t* data, int dataSize, int flags, Moss_RecastTileRef lastRef, Moss_RecastTileRef* result);
	
	// Removes the specified tile from the navigation mesh.
	//  @param ref			The reference of the tile to remove.
	//  @param data		Data associated with deleted tile.
	//  @param dataSize	Size of the data associated with deleted tile.
	// @return The status flags for the operation.
	Moss_RecastStatus removeTile(Moss_RecastTileRef ref, uint8_t** data, int* dataSize);


	// @name Query Functions

	// Calculates the tile grid location for the specified world position.
	//  @param	pos  The world position for the query. [(x, y, z)]
	//  @param tx		The tile's x-location. (x, y)
	//  @param ty		The tile's y-location. (x, y)
	void calcTileLoc(const float* pos, int* tx, int* ty) const;

	inline void calcTileLoc(const Float3& pos, int& tx, int& ty) const
	{
		calcTileLoc(pos.Data(), &tx, &ty);
	}

	// Gets the tile at the specified grid location.
	//  @param	x		The tile's x-location. (x, y, layer)
	//  @param	y		The tile's y-location. (x, y, layer)
	//  @param	layer	The tile's layer. (x, y, layer)
	// @return The tile, or null if the tile does not exist.
	const Moss_RecastMeshTile* getTileAt(const int x, const int y, const int layer) const;

	// Gets all tiles at the specified grid location. (All layers.)
	//  @param x			The tile's x-location. (x, y)
	//  @param y			The tile's y-location. (x, y)
	//  @param tiles		A pointer to an array of tiles that will hold the result.
	//  @param maxTiles	The maximum tiles the tiles parameter can hold.
	// @return The number of tiles returned in the tiles array.
	int getTilesAt(const int x, const int y,
				   Moss_RecastMeshTile const** tiles, const int maxTiles) const;
	
	// Gets the tile reference for the tile at specified grid location.
	//  @param	x		The tile's x-location. (x, y, layer)
	//  @param	y		The tile's y-location. (x, y, layer)
	//  @param	layer	The tile's layer. (x, y, layer)
	// @return The tile reference of the tile, or 0 if there is none.
	Moss_RecastTileRef getTileRefAt(int x, int y, int layer) const;

	// Gets the tile reference for the specified tile.
	//  @param	tile	The tile.
	// @return The tile reference of the tile.
	Moss_RecastTileRef getTileRef(const Moss_RecastMeshTile* tile) const;

	// Gets the tile for the specified tile reference.
	//  @param	ref		The tile reference of the tile to retrieve.
	// @return The tile for the specified reference, or null if the 
	//		reference is invalid.
	const Moss_RecastMeshTile* getTileByRef(Moss_RecastTileRef ref) const;
	
	// The maximum number of tiles supported by the navigation mesh.
	// @return The maximum number of tiles supported by the navigation mesh.
	int getMaxTiles() const;
	
	// Gets the tile at the specified index.
	//  @param	i		The tile index. [Limit: 0 >= index < #getMaxTiles()]
	// @return The tile at the specified index.
	const Moss_RecastMeshTile* getTile(int i) const;

	// Gets the tile and polygon for the specified polygon reference.
	//  @param ref		The reference for the a polygon.
	//  @param tile	The tile containing the polygon.
	//  @param poly	The polygon.
	// @return The status flags for the operation.
	Moss_RecastStatus getTileAndPolyByRef(const Moss_RecastPolyRef ref, const Moss_RecastMeshTile** tile, const Moss_RecastPoly** poly) const;
	
	// Returns the tile and polygon for the specified polygon reference.
	//  @param ref		A known valid reference for a polygon.
	//  @param tile	The tile containing the polygon.
	//  @param poly	The polygon.
	void getTileAndPolyByRefUnsafe(const Moss_RecastPolyRef ref, const Moss_RecastMeshTile** tile, const Moss_RecastPoly** poly) const;

	// Checks the validity of a polygon reference.
	//  @param	ref		The polygon reference to check.
	// @return True if polygon reference is valid for the navigation mesh.
	bool isValidPolyRef(Moss_RecastPolyRef ref) const;
	
	// Gets the polygon reference for the tile's base polygon.
	//  @param	tile		The tile.
	// @return The polygon reference for the base polygon in the specified tile.
	Moss_RecastPolyRef getPolyRefBase(const Moss_RecastMeshTile* tile) const;
	
	// Gets the endpoints for an off-mesh connection, ordered by "direction of travel".
	//  @param prevRef		The reference of the polygon before the connection.
	//  @param polyRef		The reference of the off-mesh connection polygon.
	//  @param startPos	The start position of the off-mesh connection. [(x, y, z)]
	//  @param endPos		The end position of the off-mesh connection. [(x, y, z)]
	// @return The status flags for the operation.
	Moss_RecastStatus getOffMeshConnectionPolyEndPoints(Moss_RecastPolyRef prevRef, Moss_RecastPolyRef polyRef, float* startPos, float* endPos) const;
	inline Moss_RecastStatus getOffMeshConnectionPolyEndPoints(Moss_RecastPolyRef prevRef, Moss_RecastPolyRef polyRef, Float3& startPos, Float3& endPos) const

	// Gets the specified off-mesh connection.
	//  @param	ref		The polygon reference of the off-mesh connection.
	// @return The specified off-mesh connection, or null if the polygon reference is not valid.
	const Moss_RecastOffMeshConnection* getOffMeshConnectionByRef(Moss_RecastPolyRef ref) const;
	

	// @name State Management
	// These functions do not effect #Moss_RecastTileRef or #Moss_RecastPolyRef's. 

	// Sets the user defined flags for the specified polygon.
	//  @param	ref		The polygon reference.
	//  @param	flags	The new flags for the polygon.
	// @return The status flags for the operation.
	Moss_RecastStatus setPolyFlags(Moss_RecastPolyRef ref, uint16_t flags);

	// Gets the user defined flags for the specified polygon.
	//  @param ref				The polygon reference.
	//  @param resultFlags		The polygon flags.
	// @return The status flags for the operation.
	Moss_RecastStatus getPolyFlags(Moss_RecastPolyRef ref, uint16_t* resultFlags) const;

	// Sets the user defined area for the specified polygon.
	//  @param	ref		The polygon reference.
	//  @param	area	The new area id for the polygon. [Limit: < #DT_MAX_AREAS]
	// @return The status flags for the operation.
	Moss_RecastStatus setPolyArea(Moss_RecastPolyRef ref, uint8_t area);

	// Gets the user defined area for the specified polygon.
	//  @param ref			The polygon reference.
	//  @param resultArea	The area id for the polygon.
	// @return The status flags for the operation.
	Moss_RecastStatus getPolyArea(Moss_RecastPolyRef ref, uint8_t* resultArea) const;

	// Gets the size of the buffer required by #storeTileState to store the specified tile's state.
	//  @param	tile	The tile.
	// @return The size of the buffer required to store the state.
	int getTileStateSize(const Moss_RecastMeshTile* tile) const;
	
	// Stores the non-structural state of the tile in the specified buffer. (Flags, area ids, etc.)
	//  @param tile			The tile.
	//  @param data			The buffer to store the tile's state in.
	//  @param maxDataSize		The size of the data buffer. [Limit: >= #getTileStateSize]
	// @return The status flags for the operation.
	Moss_RecastStatus storeTileState(const Moss_RecastMeshTile* tile, uint8_t* data, const int maxDataSize) const;
	
	// Restores the state of the tile.
	//  @param	tile			The tile.
	//  @param	data			The new state. (Obtained from #storeTileState.)
	//  @param	maxDataSize		The size of the state within the data buffer.
	// @return The status flags for the operation.
	Moss_RecastStatus restoreTileState(Moss_RecastMeshTile* tile, const uint8_t* data, const int maxDataSize);
	

	// @name Encoding and Decoding
	// These functions are generally meant for internal use only.

	// Derives a standard polygon reference.
	//  @note This function is generally meant for internal use only.
	//  @param	salt	The tile's salt value.
	//  @param	it		The index of the tile.
	//  @param	ip		The index of the polygon within the tile.
	inline Moss_RecastPolyRef encodePolyId(uint32_t salt, uint32_t it, uint32_t ip) const {
#ifdef DT_POLYREF64
		return ((Moss_RecastPolyRef)salt << (DT_POLY_BITS+DT_TILE_BITS)) | ((Moss_RecastPolyRef)it << DT_POLY_BITS) | (Moss_RecastPolyRef)ip;
#else
		return ((Moss_RecastPolyRef)salt << (m_polyBits+m_tileBits)) | ((Moss_RecastPolyRef)it << m_polyBits) | (Moss_RecastPolyRef)ip;
#endif
	}
	
	// Decodes a standard polygon reference.
	//  @note This function is generally meant for internal use only.
	//  @param	ref   The polygon reference to decode.
	//  @param salt	The tile's salt value.
	//  @param it		The index of the tile.
	//  @param ip		The index of the polygon within the tile.
	//  @see #encodePolyId
	inline void decodePolyId(Moss_RecastPolyRef ref, uint32_t& salt, uint32_t& it, uint32_t& ip) const {
#ifdef DT_POLYREF64
		const Moss_RecastPolyRef saltMask = ((Moss_RecastPolyRef)1<<DT_SALT_BITS)-1;
		const Moss_RecastPolyRef tileMask = ((Moss_RecastPolyRef)1<<DT_TILE_BITS)-1;
		const Moss_RecastPolyRef polyMask = ((Moss_RecastPolyRef)1<<DT_POLY_BITS)-1;
		salt = (uint32_t)((ref >> (DT_POLY_BITS+DT_TILE_BITS)) & saltMask);
		it = (uint32_t)((ref >> DT_POLY_BITS) & tileMask);
		ip = (uint32_t)(ref & polyMask);
#else
		const Moss_RecastPolyRef saltMask = ((Moss_RecastPolyRef)1<<m_saltBits)-1;
		const Moss_RecastPolyRef tileMask = ((Moss_RecastPolyRef)1<<m_tileBits)-1;
		const Moss_RecastPolyRef polyMask = ((Moss_RecastPolyRef)1<<m_polyBits)-1;
		salt = (uint32_t)((ref >> (m_polyBits+m_tileBits)) & saltMask);
		it = (uint32_t)((ref >> m_polyBits) & tileMask);
		ip = (uint32_t)(ref & polyMask);
#endif
	}

	// Extracts a tile's salt value from the specified polygon reference.
	//  @note This function is generally meant for internal use only.
	//  @param	ref		The polygon reference.
	//  @see #encodePolyId
	inline uint32_t decodePolyIdSalt(Moss_RecastPolyRef ref) const {
#ifdef DT_POLYREF64
		const Moss_RecastPolyRef saltMask = ((Moss_RecastPolyRef)1<<DT_SALT_BITS)-1;
		return (uint32_t)((ref >> (DT_POLY_BITS+DT_TILE_BITS)) & saltMask);
#else
		const Moss_RecastPolyRef saltMask = ((Moss_RecastPolyRef)1<<m_saltBits)-1;
		return (uint32_t)((ref >> (m_polyBits+m_tileBits)) & saltMask);
#endif
	}
	
	// Extracts the tile's index from the specified polygon reference.
	//  @note This function is generally meant for internal use only.
	//  @param	ref		The polygon reference.
	//  @see #encodePolyId
	inline uint32_t decodePolyIdTile(Moss_RecastPolyRef ref) const {
#ifdef DT_POLYREF64
		const Moss_RecastPolyRef tileMask = ((Moss_RecastPolyRef)1<<DT_TILE_BITS)-1;
		return (uint32_t)((ref >> DT_POLY_BITS) & tileMask);
#else
		const Moss_RecastPolyRef tileMask = ((Moss_RecastPolyRef)1<<m_tileBits)-1;
		return (uint32_t)((ref >> m_polyBits) & tileMask);
#endif
	}
	
	// Extracts the polygon's index (within its tile) from the specified polygon reference.
	//  @note This function is generally meant for internal use only.
	//  @param	ref		The polygon reference.
	//  @see #encodePolyId
	inline uint32_t decodePolyIdPoly(Moss_RecastPolyRef ref) const {
#ifdef DT_POLYREF64
		const Moss_RecastPolyRef polyMask = ((Moss_RecastPolyRef)1<<DT_POLY_BITS)-1;
		return (uint32_t)(ref & polyMask);
#else
		const Moss_RecastPolyRef polyMask = ((Moss_RecastPolyRef)1<<m_polyBits)-1;
		return (uint32_t)(ref & polyMask);
#endif
	}

	
private:
	// Explicitly disabled copy constructor and copy assignment operator.
	Moss_RecastNavMesh(const Moss_RecastNavMesh&) = delete;
	Moss_RecastNavMesh& operator=(const Moss_RecastNavMesh&) = delete;

	// Returns pointer to tile in the tile array.
	Moss_RecastMeshTile* getTile(int i);

	// Returns neighbour tile based on side.
	int getTilesAt(const int x, const int y, Moss_RecastMeshTile** tiles, const int maxTiles) const;

	// Returns neighbour tile based on side.
	int getNeighbourTilesAt(const int x, const int y, const int side, Moss_RecastMeshTile** tiles, const int maxTiles) const;
	
	// Returns all polygons in neighbour tile based on portal defined by the segment.
	int findConnectingPolys(const float* va, const float* vb, const Moss_RecastMeshTile* tile, int side, Moss_RecastPolyRef* con, float* conarea, int maxcon) const;
	
	// Builds internal polygons links for a tile.
	void connectIntLinks(Moss_RecastMeshTile* tile);
	// Builds internal polygons links for a tile.
	void baseOffMeshLinks(Moss_RecastMeshTile* tile);

	// Builds external polygon links for a tile.
	void connectExtLinks(Moss_RecastMeshTile* tile, Moss_RecastMeshTile* target, int side);
	// Builds external polygon links for a tile.
	void connectExtOffMeshLinks(Moss_RecastMeshTile* tile, Moss_RecastMeshTile* target, int side);
	
	// Removes external links at specified side.
	void unconnectLinks(Moss_RecastMeshTile* tile, Moss_RecastMeshTile* target);
	

	// TODO: These methods are duplicates from Moss_RecastNavMeshQuery, but are needed for off-mesh connection finding.
	
	// Queries polygons within a tile.
	int queryPolygonsInTile(const Moss_RecastMeshTile* tile, const float* qmin, const float* qmax, Moss_RecastPolyRef* polys, const int maxPolys) const;
	// Find nearest polygon within a tile.
	Moss_RecastPolyRef findNearestPolyInTile(const Moss_RecastMeshTile* tile, const float* center, const float* halfExtents, float* nearestPt) const;
	// Returns whether position is over the poly and the height at the position if so.
	bool getPolyHeight(const Moss_RecastMeshTile* tile, const Moss_RecastPoly* poly, const float* pos, float* height) const;
	// Returns closest point on polygon.
	void closestPointOnPoly(Moss_RecastPolyRef ref, const float* pos, float* closest, bool* posOverPoly) const;
	
	Moss_RecastNavMeshParams m_params;			// Current initialization params. TODO: do not store this info twice.
	Float3 m_orig;						// Origin of the tile (0,0)
	float m_tileWidth, m_tileHeight;	// Dimensions of each tile.
	int m_maxTiles;						// Max number of tiles.
	int m_tileLutSize;					// Tile hash lookup size (must be pot).
	int m_tileLutMask;					// Tile hash lookup mask.

	Moss_RecastMeshTile** m_posLookup;			// Tile hash lookup.
	Moss_RecastMeshTile* m_nextFree;				// Freelist of tiles.
	Moss_RecastMeshTile* m_tiles;				// List of tiles.
		
#ifndef DT_POLYREF64
	uint32_t m_saltBits;			// Number of salt bits in the tile ID.
	uint32_t m_tileBits;			// Number of tile bits in the tile ID.
	uint32_t m_polyBits;			// Number of poly bits in the tile ID.
#endif

	friend class Moss_RecastNavMeshQuery;
};

// Represents the source data used to build an navigation mesh tile.
// @ingroup detour
struct Moss_RecastNavMeshCreateParams {
	// @name Polygon Mesh Attributes
	// Used to create the base navigation graph.
	// See #Moss_RecastPolyMesh for details related to these attributes.

	const uint16_t* verts;			// The polygon mesh vertices. [(x, y, z) * #vertCount] [Unit: vx]
	int vertCount;							// The number vertices in the polygon mesh. [Limit: >= 3]
	const uint16_t* polys;			// The polygon data. [Size: #polyCount * 2 * #nvp]
	const uint16_t* polyFlags;		// The user defined flags assigned to each polygon. [Size: #polyCount]
	const uint8_t* polyAreas;			// The user defined area ids assigned to each polygon. [Size: #polyCount]
	int polyCount;							// Number of polygons in the mesh. [Limit: >= 1]
	int nvp;								// Number maximum number of vertices per polygon. [Limit: >= 3]

	// @name Height Detail Attributes (Optional)
	// See #Moss_RecastPolyMeshDetail for details related to these attributes.

	const uint32_t* detailMeshes;		// The height detail sub-mesh data. [Size: 4 * #polyCount]
	const float* detailVerts;				// The detail mesh vertices. [Size: 3 * #detailVertsCount] [Unit: wu]
	int detailVertsCount;					// The number of vertices in the detail mesh.
	const uint8_t* detailTris;		// The detail mesh triangles. [Size: 4 * #detailTriCount]
	int detailTriCount;						// The number of triangles in the detail mesh.

	// @name Off-Mesh Connections Attributes (Optional)
	// Used to define a custom point-to-point edge within the navigation graph, an 
	// off-mesh connection is a user defined traversable connection made up to two vertices, 
	// at least one of which resides within a navigation mesh polygon.

	// Off-mesh connection vertices. [(ax, ay, az, bx, by, bz) * #offMeshConCount] [Unit: wu]
	const float* offMeshConVerts;
	// Off-mesh connection radii. [Size: #offMeshConCount] [Unit: wu]
	const float* offMeshConRad;
	// User defined flags assigned to the off-mesh connections. [Size: #offMeshConCount]
	const uint16_t* offMeshConFlags;
	// User defined area ids assigned to the off-mesh connections. [Size: #offMeshConCount]
	const uint8_t* offMeshConAreas;
	// The permitted travel direction of the off-mesh connections. [Size: #offMeshConCount]
	//
	// 0 = Travel only from endpoint A to endpoint B.<br/>
	// #DT_OFFMESH_CON_BIDIR = Bidirectional travel.
	const uint8_t* offMeshConDir;	
	// The user defined ids of the off-mesh connection. [Size: #offMeshConCount]
	const uint32_t* offMeshConUserID;
	// The number of off-mesh connections. [Limit: >= 0]
	int offMeshConCount;

	// @name Tile Attributes
	// @note The tile grid/layer data can be left at zero if the destination is a single tile mesh.

	uint32_t userId;	// The user defined id of the tile.
	int tileX;				// The tile's x-grid location within the multi-tile destination mesh. (Along the x-axis.)
	int tileY;				// The tile's y-grid location within the multi-tile destination mesh. (Along the z-axis.)
	int tileLayer;			// The tile's layer within the layered destination mesh. [Limit: >= 0] (Along the y-axis.)
	AABB3 bounds;

	// @name General Configuration Attributes

	float walkableHeight;	// The agent height. [Unit: wu]
	float walkableRadius;	// The agent radius. [Unit: wu]
	float walkableClimb;	// The agent maximum traversable ledge. (Up/Down) [Unit: wu]
	float cs;				// The xz-plane cell size of the polygon mesh. [Limit: > 0] [Unit: wu]
	float ch;				// The y-axis cell height of the polygon mesh. [Limit: > 0] [Unit: wu]

	// True if a bounding volume tree should be built for the tile.
	// @note The BVTree is not normally needed for layered navigation meshes.
	bool buildBvTree;
};

// Builds navigation mesh tile data from the provided tile creation data.
// @ingroup detour
//  @param params		Tile creation data.
//  @param outData		The resulting tile data.
//  @param outDataSize	The size of the tile data array.
// @return True if the tile data was successfully created.
bool dtCreateNavMeshData(Moss_RecastNavMeshCreateParams* params, uint8_t** outData, int* outDataSize);

// Swaps the endianness of the tile data's header (#Moss_RecastMeshHeader).
//  @param[in,out]	data		The tile data array.
//  @param dataSize	The size of the data array.
bool dtNavMeshHeaderSwapEndian(uint8_t* data, const int dataSize);

// Swaps endianness of the tile data.
//  @param[in,out]	data		The tile data array.
//  @param dataSize	The size of the data array.
bool dtNavMeshDataSwapEndian(uint8_t* data, const int dataSize);

#endif // DETOURNAVMESHBUILDER_H


struct Moss_RecastNode {
	Float3 pos;								// Position of the node.
	float cost;									// Cost from previous node to current node.
	float total;								// Cost up to the node.
	uint32_t pidx : DT_NODE_PARENT_BITS;	// Index to parent node.
	uint32_t state : DT_NODE_STATE_BITS;	// extra state information. A polyRef can have multiple nodes with different extra info. see DT_MAX_STATES_PER_NODE
	uint32_t flags : 3;						// Node flags. A combination of dtNodeFlags.
	Moss_RecastPolyRef id;								// Polygon ref the node corresponds to.
};

class Moss_RecastNodePool {
public:
	Moss_RecastNodePool(int maxNodes, int hashSize);
	~Moss_RecastNodePool();
	void clear();

	// Get a Moss_RecastNode by ref and extra state information. If there is none then - allocate
	// There can be more than one node for the same polyRef but with different extra state information
	Moss_RecastNode* getNode(Moss_RecastPolyRef id, uint8_t state=0);	
	Moss_RecastNode* findNode(Moss_RecastPolyRef id, uint8_t state);
	uint32_t findNodes(Moss_RecastPolyRef id, Moss_RecastNode** nodes, const int maxNodes);

	inline uint32_t getNodeIdx(const Moss_RecastNode* node) const { if (!node) return 0; return (uint32_t)(node - m_nodes) + 1; }

	inline Moss_RecastNode* getNodeAtIdx(uint32_t idx) { if (!idx) return 0; return &m_nodes[idx - 1]; }

	inline const Moss_RecastNode* getNodeAtIdx(uint32_t idx) const { if (!idx) return 0; return &m_nodes[idx - 1]; }
	
	inline int getMemUsed() const { return sizeof(*this) + sizeof(Moss_RecastNode)*m_maxNodes + sizeof(dtNodeIndex)*m_maxNodes + sizeof(dtNodeIndex)*m_hashSize; }
	
	inline int getMaxNodes() const { return m_maxNodes; }
	
	inline int getHashSize() const { return m_hashSize; }
	inline dtNodeIndex getFirst(int bucket) const { return m_first[bucket]; }
	inline dtNodeIndex getNext(int i) const { return m_next[i]; }
	inline int getNodeCount() const { return m_nodeCount; }
	
private:
	// Explicitly disabled copy constructor and copy assignment operator.
	Moss_RecastNodePool(const Moss_RecastNodePool&);
	Moss_RecastNodePool& operator=(const Moss_RecastNodePool&);
	
	Moss_RecastNode* m_nodes;
	dtNodeIndex* m_first;
	dtNodeIndex* m_next;
	const int m_maxNodes;
	const int m_hashSize;
	int m_nodeCount;
};

class Moss_RecastNodeQueue {
public:
	Moss_RecastNodeQueue(int n);
	~Moss_RecastNodeQueue();
	
	inline void clear() { m_size = 0; }
	
	inline Moss_RecastNode* top() { return m_heap[0]; }
	
	inline Moss_RecastNode* pop() {
		Moss_RecastNode* result = m_heap[0];
		m_size--;
		trickleDown(0, m_heap[m_size]);
		return result;
	}
	
	inline void push(Moss_RecastNode* node) {
		m_size++;
		bubbleUp(m_size-1, node);
	}
	
	inline void modify(Moss_RecastNode* node) {
		for (int i = 0; i < m_size; ++i) {
			if (m_heap[i] == node) {
				bubbleUp(i, node);
				return;
			}
		}
	}
	
	inline bool empty() const { return m_size == 0; }
	
	inline int getMemUsed() const { return sizeof(*this) + sizeof(Moss_RecastNode*) * (m_capacity + 1); }
	
	inline int getCapacity() const { return m_capacity; }
	
private:
	// Explicitly disabled copy constructor and copy assignment operator.
	Moss_RecastNodeQueue(const Moss_RecastNodeQueue&);
	Moss_RecastNodeQueue& operator=(const Moss_RecastNodeQueue&);

	void bubbleUp(int i, Moss_RecastNode* node);
	void trickleDown(int i, Moss_RecastNode* node);
	
	Moss_RecastNode** m_heap;
	const int m_capacity;
	int m_size;
};		



// Define DT_VIRTUAL_QUERYFILTER if you wish to derive a custom filter from Moss_RecastQueryFilter.
// On certain platforms indirect or virtual function call is expensive. The default
// setting is to use non-virtual functions, the actual implementations of the functions
// are declared as inline for maximum speed. 

//#define DT_VIRTUAL_QUERYFILTER 1

// Defines polygon filtering and traversal costs for navigation mesh query operations.
// @ingroup detour
class Moss_RecastQueryFilter {
	float m_areaCost[DT_MAX_AREAS];		// Cost per area type. (Used by default implementation.)
	uint16_t m_includeFlags;		// Flags for polygons that can be visited. (Used by default implementation.)
	uint16_t m_excludeFlags;		// Flags for polygons that should not be visited. (Used by default implementation.)
	
public:
	Moss_RecastQueryFilter();
	
#ifdef DT_VIRTUAL_QUERYFILTER
	virtual ~Moss_RecastQueryFilter() { }
#endif
	
	// Returns true if the polygon can be visited.  (I.e. Is traversable.)
	//  @param ref		The reference id of the polygon test.
	//  @param tile	The tile containing the polygon.
	//  @param poly  The polygon to test.
#ifdef DT_VIRTUAL_QUERYFILTER
	virtual bool passFilter(const Moss_RecastPolyRef ref,
							const Moss_RecastMeshTile* tile,
							const Moss_RecastPoly* poly) const;
#else
	bool passFilter(const Moss_RecastPolyRef ref,
					const Moss_RecastMeshTile* tile,
					const Moss_RecastPoly* poly) const;
#endif

	// Returns cost to move from the beginning to the end of a line segment
	// that is fully contained within a polygon.
	//  @param pa			The start position on the edge of the previous and current polygon. [(x, y, z)]
	//  @param pb			The end position on the edge of the current and next polygon. [(x, y, z)]
	//  @param prevRef		The reference id of the previous polygon. [opt]
	//  @param prevTile	The tile containing the previous polygon. [opt]
	//  @param prevPoly	The previous polygon. [opt]
	//  @param curRef		The reference id of the current polygon.
	//  @param curTile		The tile containing the current polygon.
	//  @param curPoly		The current polygon.
	//  @param nextRef		The refernece id of the next polygon. [opt]
	//  @param nextTile	The tile containing the next polygon. [opt]
	//  @param nextPoly	The next polygon. [opt]
#ifdef DT_VIRTUAL_QUERYFILTER
	virtual float getCost(const float* pa, const float* pb,
						  const Moss_RecastPolyRef prevRef, const Moss_RecastMeshTile* prevTile, const Moss_RecastPoly* prevPoly,
						  const Moss_RecastPolyRef curRef, const Moss_RecastMeshTile* curTile, const Moss_RecastPoly* curPoly,
						  const Moss_RecastPolyRef nextRef, const Moss_RecastMeshTile* nextTile, const Moss_RecastPoly* nextPoly) const;
#else
	float getCost(const float* pa, const float* pb,
				  const Moss_RecastPolyRef prevRef, const Moss_RecastMeshTile* prevTile, const Moss_RecastPoly* prevPoly,
				  const Moss_RecastPolyRef curRef, const Moss_RecastMeshTile* curTile, const Moss_RecastPoly* curPoly,
				  const Moss_RecastPolyRef nextRef, const Moss_RecastMeshTile* nextTile, const Moss_RecastPoly* nextPoly) const;
#endif

	// @name Getters and setters for the default implementation data.

	// Returns the traversal cost of the area.
	//  @param i		The id of the area.
	// @returns The traversal cost of the area.
	inline float getAreaCost(const int i) const { return m_areaCost[i]; }

	// Sets the traversal cost of the area.
	//  @param i		The id of the area.
	//  @param cost	The new cost of traversing the area.
	inline void setAreaCost(const int i, const float cost) { m_areaCost[i] = cost; } 

	// Returns the include flags for the filter.
	// Any polygons that include one or more of these flags will be
	// included in the operation.
	inline uint16_t getIncludeFlags() const { return m_includeFlags; }

	// Sets the include flags for the filter.
	// @param flags	The new flags.
	inline void setIncludeFlags(const uint16_t flags) { m_includeFlags = flags; }

	// Returns the exclude flags for the filter.
	// Any polygons that include one ore more of these flags will be
	// excluded from the operation.
	inline uint16_t getExcludeFlags() const { return m_excludeFlags; }

	// Sets the exclude flags for the filter.
	// @param flags		The new flags.
	inline void setExcludeFlags(const uint16_t flags) { m_excludeFlags = flags; }	


};

// Provides information about raycast hit
// filled by Moss_RecastNavMeshQuery::raycast
// @ingroup detour
struct Moss_RecastRaycastHit {
	// The hit parameter. (FLT_MAX if no wall hit.)
	float t; 
	
	// hitNormal	The normal of the nearest wall hit. [(x, y, z)]
	Float3 hitNormal;

	// The index of the edge on the final polygon where the wall was hit.
	int hitEdgeIndex;
	
	// Pointer to an array of reference ids of the visited polygons. [opt]
	Moss_RecastPolyRef* path;
	
	// The number of visited polygons. [opt]
	int pathCount;

	// The maximum number of polygons the @p path array can hold.
	int maxPath;

	//  The cost of the path until hit.
	float pathCost;
};

// Provides custom polygon query behavior.
// Used by Moss_RecastNavMeshQuery::queryPolygons.
// @ingroup detour
class Moss_RecastPolyQuery {
public:
	virtual ~Moss_RecastPolyQuery();

	// Called for each batch of unique polygons touched by the search area in Moss_RecastNavMeshQuery::queryPolygons.
	// This can be called multiple times for a single query.
	virtual void process(const Moss_RecastMeshTile* tile, Moss_RecastPoly** polys, Moss_RecastPolyRef* refs, int count) = 0;
};

// Provides the ability to perform pathfinding related queries against
// a navigation mesh.
// @ingroup detour
class Moss_RecastNavMeshQuery {
public:
	Moss_RecastNavMeshQuery();
	~Moss_RecastNavMeshQuery();
	
	// Initializes the query object.
	//  @param nav			Pointer to the Moss_RecastNavMesh object to use for all queries.
	//  @param maxNodes	Maximum number of search nodes. [Limits: 0 < value <= 65535]
	// @returns The status flags for the query.
	Moss_RecastStatus init(const Moss_RecastNavMesh* nav, const int maxNodes);
	
	// @name Standard Pathfinding Functions

	// Finds a path from the start polygon to the end polygon.
	//  @param startRef	The reference id of the start polygon.
	//  @param endRef		The reference id of the end polygon.
	//  @param startPos	A position within the start polygon. [(x, y, z)]
	//  @param endPos		A position within the end polygon. [(x, y, z)]
	//  @param filter		The polygon filter to apply to the query.
	//  @param path		An ordered list of polygon references representing the path. (Start to end.)  [(polyRef) * @p pathCount]
	//  @param pathCount	The number of polygons returned in the @p path array.
	//  @param maxPath		The maximum number of polygons the @p path array can hold. [Limit: >= 1]
	Moss_RecastStatus findPath(Moss_RecastPolyRef startRef, Moss_RecastPolyRef endRef, const float* startPos, const float* endPos, const Moss_RecastQueryFilter* filter, Moss_RecastPolyRef* path, int* pathCount, const int maxPath) const;
	inline Moss_RecastStatus findPath(Moss_RecastPolyRef startRef, Moss_RecastPolyRef endRef,const Float3& startPos, const Float3& endPos,const Moss_RecastQueryFilter* filter,Moss_RecastPolyRef* path, int* pathCount, int maxPath) const

	// Finds the straight path from the start to the end position within the polygon corridor.
	//  @param startPos			Path start position. [(x, y, z)]
	//  @param endPos				Path end position. [(x, y, z)]
	//  @param path				An array of polygon references that represent the path corridor.
	//  @param pathSize			The number of polygons in the @p path array.
	//  @param straightPath		Points describing the straight path. [(x, y, z) * @p straightPathCount].
	//  @param straightPathFlags	Flags describing each point. (See: #dtStraightPathFlags) [opt]
	//  @param straightPathRefs	The reference id of the polygon that is being entered at each point. [opt]
	//  @param straightPathCount	The number of points in the straight path.
	//  @param maxStraightPath		The maximum number of points the straight path arrays can hold.  [Limit: > 0]
	//  @param options				Query options. (see: #dtStraightPathOptions)
	// @returns The status flags for the query.
	Moss_RecastStatus findStraightPath(const float* startPos, const float* endPos, const Moss_RecastPolyRef* path, const int pathSize,
							  float* straightPath, uint8_t* straightPathFlags, Moss_RecastPolyRef* straightPathRefs, int* straightPathCount, const int maxStraightPath, const int options = 0) const;

	// @name Sliced Pathfinding Functions
	// Common use case:
	//	-# Call initSlicedFindPath() to initialize the sliced path query.
	//	-# Call updateSlicedFindPath() until it returns complete.
	//	-# Call finalizeSlicedFindPath() to get the path.

	// Initializes a sliced path query.
	//  @param startRef	The reference id of the start polygon.
	//  @param endRef		The reference id of the end polygon.
	//  @param startPos	A position within the start polygon. [(x, y, z)]
	//  @param endPos		A position within the end polygon. [(x, y, z)]
	//  @param filter		The polygon filter to apply to the query.
	//  @param options		query options (see: #dtFindPathOptions)
	// @returns The status flags for the query.
	Moss_RecastStatus initSlicedFindPath(Moss_RecastPolyRef startRef, Moss_RecastPolyRef endRef, const float* startPos, const float* endPos, const Moss_RecastQueryFilter* filter, const uint32_t options = 0);

	// Updates an in-progress sliced path query.
	//  @param maxIter		The maximum number of iterations to perform.
	//  @param doneIters	The actual number of iterations completed. [opt]
	// @returns The status flags for the query.
	Moss_RecastStatus updateSlicedFindPath(const int maxIter, int* doneIters);

	// Finalizes and returns the results of a sliced path query.
	//  @param path		An ordered list of polygon references representing the path. (Start to end.) 
	//  							[(polyRef) * @p pathCount]
	//  @param pathCount	The number of polygons returned in the @p path array.
	//  @param maxPath		The max number of polygons the path array can hold. [Limit: >= 1]
	// @returns The status flags for the query.
	Moss_RecastStatus finalizeSlicedFindPath(Moss_RecastPolyRef* path, int* pathCount, const int maxPath);
	
	// Finalizes and returns the results of an incomplete sliced path query, returning the path to the furthest
	// polygon on the existing path that was visited during the search.
	//  @param existing		An array of polygon references for the existing path.
	//  @param existingSize	The number of polygon in the @p existing array.
	//  @param path			An ordered list of polygon references representing the path. (Start to end.) 
	//  								[(polyRef) * @p pathCount]
	//  @param pathCount		The number of polygons returned in the @p path array.
	//  @param maxPath			The max number of polygons the @p path array can hold. [Limit: >= 1]
	// @returns The status flags for the query.
	Moss_RecastStatus finalizeSlicedFindPathPartial(const Moss_RecastPolyRef* existing, const int existingSize, Moss_RecastPolyRef* path, int* pathCount, const int maxPath);

	// @name Dijkstra Search Functions 

	// Finds the polygons along the navigation graph that touch the specified circle.
	//  @param startRef		The reference id of the polygon where the search starts.
	//  @param centerPos		The center of the search circle. [(x, y, z)]
	//  @param radius			The radius of the search circle.
	//  @param filter			The polygon filter to apply to the query.
	//  @param resultRef		The reference ids of the polygons touched by the circle. [opt]
	//  @param resultParent	The reference ids of the parent polygons for each result. Zero if a result polygon has no parent. [opt]
	//  @param resultCost		The search cost from @p centerPos to the polygon. [opt]
	//  @param resultCount		The number of polygons found. [opt]
	//  @param maxResult		The maximum number of polygons the result arrays can hold.
	// @returns The status flags for the query.
	Moss_RecastStatus findPolysAroundCircle(Moss_RecastPolyRef startRef, const float* centerPos, const float radius,
								   const Moss_RecastQueryFilter* filter, Moss_RecastPolyRef* resultRef, Moss_RecastPolyRef* resultParent, float* resultCost,
								   int* resultCount, const int maxResult) const;
	
	// Finds the polygons along the naviation graph that touch the specified convex polygon.
	//  @param startRef		The reference id of the polygon where the search starts.
	//  @param verts			The vertices describing the convex polygon. (CCW) 
	//  								[(x, y, z) * @p nverts]
	//  @param nverts			The number of vertices in the polygon.
	//  @param filter			The polygon filter to apply to the query.
	//  @param resultRef		The reference ids of the polygons touched by the search polygon. [opt]
	//  @param resultParent	The reference ids of the parent polygons for each result. Zero if a 
	//  								result polygon has no parent. [opt]
	//  @param resultCost		The search cost from the centroid point to the polygon. [opt]
	//  @param resultCount		The number of polygons found.
	//  @param maxResult		The maximum number of polygons the result arrays can hold.
	// @returns The status flags for the query.
	Moss_RecastStatus findPolysAroundShape(Moss_RecastPolyRef startRef, const float* verts, const int nverts, const Moss_RecastQueryFilter* filter,
								  Moss_RecastPolyRef* resultRef, Moss_RecastPolyRef* resultParent, float* resultCost, int* resultCount, const int maxResult) const;
	
	// Gets a path from the explored nodes in the previous search.
	//  @param endRef		The reference id of the end polygon.
	//  @param path		An ordered list of polygon references representing the path. (Start to end.)
	//  							[(polyRef) * @p pathCount]
	//  @param pathCount	The number of polygons returned in the @p path array.
	//  @param maxPath		The maximum number of polygons the @p path array can hold. [Limit: >= 0]
	//  @returns		The status flags. Returns DT_FAILURE | DT_INVALID_PARAM if any parameter is wrong, or if
	//  				@p endRef was not explored in the previous search. Returns DT_SUCCESS | DT_BUFFER_TOO_SMALL
	//  				if @p path cannot contain the entire path. In this case it is filled to capacity with a partial path.
	//  				Otherwise returns DT_SUCCESS.
	//  @remarks		The result of this function depends on the state of the query object. For that reason it should only
	//  				be used immediately after one of the two Dijkstra searches, findPolysAroundCircle or findPolysAroundShape.
	Moss_RecastStatus getPathFromDijkstraSearch(Moss_RecastPolyRef endRef, Moss_RecastPolyRef* path, int* pathCount, int maxPath) const;

	// @name Local Query Functions

	// Finds the polygon nearest to the specified center point.
	// [opt] means the specified parameter can be a null pointer, in that case the output parameter will not be set.
	//
	//  @param center		The center of the search box. [(x, y, z)]
	//  @param halfExtents	The search distance along each axis. [(x, y, z)]
	//  @param filter		The polygon filter to apply to the query.
	//  @param nearestRef	The reference id of the nearest polygon. Will be set to 0 if no polygon is found.
	//  @param nearestPt	The nearest point on the polygon. Unchanged if no polygon is found. [opt] [(x, y, z)]
	// @returns The status flags for the query.
	Moss_RecastStatus findNearestPoly(const float* center, const float* halfExtents,
							 const Moss_RecastQueryFilter* filter, Moss_RecastPolyRef* nearestRef, float* nearestPt) const;

	// Finds the polygon nearest to the specified center point.
	// [opt] means the specified parameter can be a null pointer, in that case the output parameter will not be set.
	// 
	//  @param center		The center of the search box. [(x, y, z)]
	//  @param halfExtents	The search distance along each axis. [(x, y, z)]
	//  @param filter		The polygon filter to apply to the query.
	//  @param nearestRef	The reference id of the nearest polygon. Will be set to 0 if no polygon is found.
	//  @param nearestPt	The nearest point on the polygon. Unchanged if no polygon is found. [opt] [(x, y, z)]
	//  @param isOverPoly 	Set to true if the point's X/Z coordinate lies inside the polygon, false otherwise. Unchanged if no polygon is found. [opt]
	// @returns The status flags for the query.
	Moss_RecastStatus findNearestPoly(const float* center, const float* halfExtents,
							 const Moss_RecastQueryFilter* filter, Moss_RecastPolyRef* nearestRef, float* nearestPt, bool* isOverPoly) const;
	
	// Finds polygons that overlap the search box.
	//  @param center		The center of the search box. [(x, y, z)]
	//  @param halfExtents		The search distance along each axis. [(x, y, z)]
	//  @param filter		The polygon filter to apply to the query.
	//  @param polys		The reference ids of the polygons that overlap the query box.
	//  @param polyCount	The number of polygons in the search result.
	//  @param maxPolys	The maximum number of polygons the search result can hold.
	// @returns The status flags for the query.
	Moss_RecastStatus queryPolygons(const float* center, const float* halfExtents,
						   const Moss_RecastQueryFilter* filter, Moss_RecastPolyRef* polys, int* polyCount, const int maxPolys) const;

	// Finds polygons that overlap the search box.
	//  @param center		The center of the search box. [(x, y, z)]
	//  @param halfExtents		The search distance along each axis. [(x, y, z)]
	//  @param filter		The polygon filter to apply to the query.
	//  @param query		The query. Polygons found will be batched together and passed to this query.
	Moss_RecastStatus queryPolygons(const float* center, const float* halfExtents, const Moss_RecastQueryFilter* filter, Moss_RecastPolyQuery* query) const;

	// Finds the non-overlapping navigation polygons in the local neighbourhood around the center position.
	//  @param startRef		The reference id of the polygon where the search starts.
	//  @param centerPos		The center of the query circle. [(x, y, z)]
	//  @param radius			The radius of the query circle.
	//  @param filter			The polygon filter to apply to the query.
	//  @param resultRef		The reference ids of the polygons touched by the circle.
	//  @param resultParent	The reference ids of the parent polygons for each result. 
	//  								Zero if a result polygon has no parent. [opt]
	//  @param resultCount		The number of polygons found.
	//  @param maxResult		The maximum number of polygons the result arrays can hold.
	// @returns The status flags for the query.
	Moss_RecastStatus findLocalNeighbourhood(Moss_RecastPolyRef startRef, const float* centerPos, const float radius, const Moss_RecastQueryFilter* filter,
									Moss_RecastPolyRef* resultRef, Moss_RecastPolyRef* resultParent, int* resultCount, const int maxResult) const;

	// Moves from the start to the end position constrained to the navigation mesh.
	//  @param startRef		The reference id of the start polygon.
	//  @param startPos		A position of the mover within the start polygon. [(x, y, x)]
	//  @param endPos			The desired end position of the mover. [(x, y, z)]
	//  @param filter			The polygon filter to apply to the query.
	//  @param resultPos		The result position of the mover. [(x, y, z)]
	//  @param visited			The reference ids of the polygons visited during the move.
	//  @param visitedCount	The number of polygons visited during the move.
	//  @param maxVisitedSize	The maximum number of polygons the @p visited array can hold.
	// @returns The status flags for the query.
	Moss_RecastStatus moveAlongSurface(Moss_RecastPolyRef startRef, const float* startPos, const float* endPos,
							  const Moss_RecastQueryFilter* filter, float* resultPos, Moss_RecastPolyRef* visited, int* visitedCount, const int maxVisitedSize) const;
	
	// Casts a 'walkability' ray along the surface of the navigation mesh from 
	// the start position toward the end position.
	// @note A wrapper around raycast(..., RaycastHit*). Retained for backward compatibility.
	//  @param startRef	The reference id of the start polygon.
	//  @param startPos	A position within the start polygon representing 
	//  							the start of the ray. [(x, y, z)]
	//  @param endPos		The position to cast the ray toward. [(x, y, z)]
	//  @param filter		The polygon filter to apply to the query.
	//  @param t			The hit parameter. (FLT_MAX if no wall hit.)
	//  @param hitNormal	The normal of the nearest wall hit. [(x, y, z)]
	//  @param path		The reference ids of the visited polygons. [opt]
	//  @param pathCount	The number of visited polygons. [opt]
	//  @param maxPath		The maximum number of polygons the @p path array can hold.
	// @returns The status flags for the query.
	Moss_RecastStatus raycast(Moss_RecastPolyRef startRef, const float* startPos, const float* endPos, const Moss_RecastQueryFilter* filter,
					 float* t, float* hitNormal, Moss_RecastPolyRef* path, int* pathCount, const int maxPath) const;
	
	// Casts a 'walkability' ray along the surface of the navigation mesh from 
	// the start position toward the end position.
	//  @param startRef	The reference id of the start polygon.
	//  @param startPos	A position within the start polygon representing 
	//  							the start of the ray. [(x, y, z)]
	//  @param endPos		The position to cast the ray toward. [(x, y, z)]
	//  @param filter		The polygon filter to apply to the query.
	//  @param options		govern how the raycast behaves. See dtRaycastOptions
	//  @param hit			Pointer to a raycast hit structure which will be filled by the results.
	//  @param prevRef		parent of start ref. Used during for cost calculation [opt]
	// @returns The status flags for the query.
	Moss_RecastStatus raycast(Moss_RecastPolyRef startRef, const float* startPos, const float* endPos,
					 const Moss_RecastQueryFilter* filter, const uint32_t options, Moss_RecastRaycastHit* hit, Moss_RecastPolyRef prevRef = 0) const;


	// Finds the distance from the specified position to the nearest polygon wall.
	//  @param startRef		The reference id of the polygon containing @p centerPos.
	//  @param centerPos		The center of the search circle. [(x, y, z)]
	//  @param maxRadius		The radius of the search circle.
	//  @param filter			The polygon filter to apply to the query.
	//  @param hitDist			The distance to the nearest wall from @p centerPos.
	//  @param hitPos			The nearest position on the wall that was hit. [(x, y, z)]
	//  @param hitNormal		The normalized ray formed from the wall point to the 
	//  								source point. [(x, y, z)]
	// @returns The status flags for the query.
	Moss_RecastStatus findDistanceToWall(Moss_RecastPolyRef startRef, const float* centerPos, const float maxRadius, const Moss_RecastQueryFilter* filter, float* hitDist, float* hitPos, float* hitNormal) const;
	
	// Returns the segments for the specified polygon, optionally including portals.
	//  @param ref				The reference id of the polygon.
	//  @param filter			The polygon filter to apply to the query.
	//  @param segmentVerts	The segments. [(ax, ay, az, bx, by, bz) * segmentCount]
	//  @param segmentRefs		The reference ids of each segment's neighbor polygon. 
	//  								Or zero if the segment is a wall. [opt] [(parentRef) * @p segmentCount] 
	//  @param segmentCount	The number of segments returned.
	//  @param maxSegments		The maximum number of segments the result arrays can hold.
	// @returns The status flags for the query.
	Moss_RecastStatus getPolyWallSegments(Moss_RecastPolyRef ref, const Moss_RecastQueryFilter* filter, float* segmentVerts, Moss_RecastPolyRef* segmentRefs, int* segmentCount, const int maxSegments) const;

	// Returns random location on navmesh.
	// Polygons are chosen weighted by area. The search runs in linear related to number of polygon.
	//  @param filter			The polygon filter to apply to the query.
	//  @param frand			Function returning a random number [0..1).
	//  @param randomRef		The reference id of the random location.
	//  @param randomPt		The random location. 
	// @returns The status flags for the query.
	Moss_RecastStatus findRandomPoint(const Moss_RecastQueryFilter* filter, float (*frand)(), Moss_RecastPolyRef* randomRef, float* randomPt) const;

	// Returns random location on navmesh within the reach of specified location.
	// Polygons are chosen weighted by area. The search runs in linear related to number of polygon.
	// The location is not exactly constrained by the circle, but it limits the visited polygons.
	//  @param startRef		The reference id of the polygon where the search starts.
	//  @param centerPos		The center of the search circle. [(x, y, z)]
	//  @param maxRadius		The radius of the search circle. [Units: wu]
	//  @param filter			The polygon filter to apply to the query.
	//  @param frand			Function returning a random number [0..1).
	//  @param randomRef		The reference id of the random location.
	//  @param randomPt		The random location. [(x, y, z)]
	// @returns The status flags for the query.
	Moss_RecastStatus findRandomPointAroundCircle(Moss_RecastPolyRef startRef, const float* centerPos, const float maxRadius,
										 const Moss_RecastQueryFilter* filter, float (*frand)(),
										 Moss_RecastPolyRef* randomRef, float* randomPt) const;
	
	// Finds the closest point on the specified polygon.
	//  @param ref			The reference id of the polygon.
	//  @param pos			The position to check. [(x, y, z)]
	//  @param closest		The closest point on the polygon. [(x, y, z)]
	//  @param posOverPoly	True of the position is over the polygon.
	// @returns The status flags for the query.
	Moss_RecastStatus closestPointOnPoly(Moss_RecastPolyRef ref, const float* pos, float* closest, bool* posOverPoly) const;
	
	// Returns a point on the boundary closest to the source point if the source point is outside the 
	// polygon's xz-bounds.
	//  @param ref			The reference id to the polygon.
	//  @param pos			The position to check. [(x, y, z)]
	//  @param closest		The closest point. [(x, y, z)]
	// @returns The status flags for the query.
	Moss_RecastStatus closestPointOnPolyBoundary(Moss_RecastPolyRef ref, const float* pos, float* closest) const;
	
	// Gets the height of the polygon at the provided position using the height detail. (Most accurate.)
	//  @param ref			The reference id of the polygon.
	//  @param pos			A position within the xz-bounds of the polygon. [(x, y, z)]
	//  @param height		The height at the surface of the polygon.
	// @returns The status flags for the query.
	Moss_RecastStatus getPolyHeight(Moss_RecastPolyRef ref, const float* pos, float* height) const;

	// @name Miscellaneous Functions

	// Returns true if the polygon reference is valid and passes the filter restrictions.
	//  @param ref			The polygon reference to check.
	//  @param filter		The filter to apply.
	bool isValidPolyRef(Moss_RecastPolyRef ref, const Moss_RecastQueryFilter* filter) const;

	// Returns true if the polygon reference is in the closed list. 
	//  @param ref		The reference id of the polygon to check.
	// @returns True if the polygon is in closed list.
	bool isInClosedList(Moss_RecastPolyRef ref) const;
	
	// Gets the node pool.
	// @returns The node pool.
	class Moss_RecastNodePool* getNodePool() const { return m_nodePool; }
	
	// Gets the navigation mesh the query object is using.
	// @return The navigation mesh the query object is using.
	const Moss_RecastNavMesh* getAttachedNavMesh() const { return m_nav; }

	
private:
	// Explicitly disabled copy constructor and copy assignment operator
	Moss_RecastNavMeshQuery(const Moss_RecastNavMeshQuery&);
	Moss_RecastNavMeshQuery& operator=(const Moss_RecastNavMeshQuery&);
	
	// Queries polygons within a tile.
	void queryPolygonsInTile(const Moss_RecastMeshTile* tile, const float* qmin, const float* qmax, const Moss_RecastQueryFilter* filter, Moss_RecastPolyQuery* query) const;

	// Returns portal points between two polygons.
	Moss_RecastStatus getPortalPoints(Moss_RecastPolyRef from, Moss_RecastPolyRef to, float* left, float* right, uint8_t& fromType, uint8_t& toType) const;
	Moss_RecastStatus getPortalPoints(Moss_RecastPolyRef from, const Moss_RecastPoly* fromPoly, const Moss_RecastMeshTile* fromTile, Moss_RecastPolyRef to, const Moss_RecastPoly* toPoly, const Moss_RecastMeshTile* toTile, float* left, float* right) const;
	
	// Returns edge mid point between two polygons.
	Moss_RecastStatus getEdgeMidPoint(Moss_RecastPolyRef from, Moss_RecastPolyRef to, float* mid) const;
	Moss_RecastStatus getEdgeMidPoint(Moss_RecastPolyRef from, const Moss_RecastPoly* fromPoly, const Moss_RecastMeshTile* fromTile, Moss_RecastPolyRef to, const Moss_RecastPoly* toPoly, const Moss_RecastMeshTile* toTile, float* mid) const;
	
	// Appends vertex to a straight path
	Moss_RecastStatus appendVertex(const float* pos, const uint8_t flags, const Moss_RecastPolyRef ref, float* straightPath, uint8_t* straightPathFlags, Moss_RecastPolyRef* straightPathRefs, int* straightPathCount, const int maxStraightPath) const;

	// Appends intermediate portal points to a straight path.
	Moss_RecastStatus appendPortals(const int startIdx, const int endIdx, const float* endPos, const Moss_RecastPolyRef* path, float* straightPath, uint8_t* straightPathFlags, Moss_RecastPolyRef* straightPathRefs, int* straightPathCount, const int maxStraightPath, const int options) const;

	// Gets the path leading to the specified end node.
	Moss_RecastStatus getPathToNode(struct Moss_RecastNode* endNode, Moss_RecastPolyRef* path, int* pathCount, int maxPath) const;
	
	const Moss_RecastNavMesh* m_nav;				// Pointer to navmesh data.

	struct dtQueryData {
		Moss_RecastStatus status;
		struct Moss_RecastNode* lastBestNode;
		float lastBestNodeCost;
		Moss_RecastPolyRef startRef, endRef;
		float startPos[3], endPos[3];
		const Moss_RecastQueryFilter* filter;
		uint32_t options;
		float raycastLimitSqr;
	};
	dtQueryData m_query;				// Sliced query state.

	class Moss_RecastNodePool* m_tinyNodePool;	// Pointer to small node pool.
	class Moss_RecastNodePool* m_nodePool;		// Pointer to node pool.
	class Moss_RecastNodeQueue* m_openList;		// Pointer to open list queue.
};


/**
@defgroup detour Detour

Members in this module are used to create, manipulate, and query navigation 
meshes.

@note This is a summary list of members.  Use the index or search 
feature to find minor members.
*/

// @name General helper functions

// Used to ignore a function parameter.  VS complains about unused parameters
// and this silences the warning.
template<class T> void dtIgnoreUnused(const T&) { }


// Derives the closest point on a triangle from the specified reference point.
//  @param closest	The closest point on the triangle.	
//  @param p The reference point from which to test. [(x, y, z)]   @param a Vertex A of triangle ABC. [(x, y, z)]
//  @param b Vertex B of triangle ABC. [(x, y, z)]  @param c Vertex C of triangle ABC. [(x, y, z)]
void dtClosestPtPointTriangle(float* closest, const float* p, const float* a, const float* b, const float* c);

// Derives the y-axis height of the closest point on the triangle from the specified reference point.
//  @param p		The reference point from which to test. [(x, y, z)]
//  @param a		Vertex A of triangle ABC. [(x, y, z)]
//  @param b		Vertex B of triangle ABC. [(x, y, z)]
//  @param c		Vertex C of triangle ABC. [(x, y, z)]
//  @param h		The resulting height.
bool dtClosestHeightPointTriangle(const float* p, const float* a, const float* b, const float* c, float& h);

bool dtIntersectSegmentPoly2D(const float* p0, const float* p1, const float* verts, int nverts, float& tmin, float& tmax, int& segMin, int& segMax);

bool dtIntersectSegSeg2D(const float* ap, const float* aq, const float* bp, const float* bq, float& s, float& t);

// Determines if the specified point is inside the convex polygon on the xz-plane.
//  @param pt		The point to check. [(x, y, z)]
//  @param verts	The polygon vertices. [(x, y, z) * @p nverts]
//  @param nverts	The number of vertices. [Limit: >= 3]
// @return True if the point is inside the polygon.
bool dtPointInPolygon(const float* pt, const float* verts, const int nverts);

bool dtDistancePtPolyEdgesSqr(const float* pt, const float* verts, const int nverts, float* ed, float* et);

float dtDistancePtSegSqr2D(const float* pt, const float* p, const float* q, float& t);

// Derives the centroid of a convex polygon.
//  @param tc		The centroid of the polgyon. [(x, y, z)]
//  @param idx		The polygon indices. [(vertIndex) * @p nidx]
//  @param nidx	The number of indices in the polygon. [Limit: >= 3]
//  @param verts	The polygon vertices. [(x, y, z) * vertCount]
void dtCalcPolyCenter(float* tc, const uint16_t* idx, int nidx, const float* verts);

// Determines if the two convex polygons overlap on the xz-plane.
//  @param polya		Polygon A vertices.	[(x, y, z) * @p npolya]
//  @param npolya		The number of vertices in polygon A.
//  @param polyb		Polygon B vertices.	[(x, y, z) * @p npolyb]
//  @param npolyb		The number of vertices in polygon B.
// @return True if the two polygons overlap.
bool dtOverlapPolyPoly2D(const float* polya, const int npolya, const float* polyb, const int npolyb);


void dtRandomPointInConvexPoly(const float* pts, const int npts, float* areas,  const float s, const float t, float* out);


//////////////////////////////////////////////////

// This section contains detailed documentation for members that don't have
// a source file. It reduces clutter in the main section of the header.

/**

@fn float dtTriArea2D(const float* a, const float* b, const float* c)
@par

The vertices are projected onto the xz-plane, so the y-values are ignored.

This is a low cost function than can be used for various purposes.  Its main purpose
is for point/line relationship testing.

In all cases: A value of zero indicates that all vertices are collinear or represent the same point.
(On the xz-plane.)

When used for point/line relationship tests, AB usually represents a line against which
the C point is to be tested.  In this case:

A positive value indicates that point C is to the left of line AB, looking from A toward B.<br/>
A negative value indicates that point C is to the right of lineAB, looking from A toward B.

When used for evaluating a triangle:

The absolute value of the return value is two times the area of the triangle when it is
projected onto the xz-plane.

A positive return value indicates:
- The vertices are wrapped in the normal Detour wrap direction.</li>
- The triangle's 3D face normal is in the general up direction.</li>

A negative return value indicates:
- The vertices are reverse wrapped. (Wrapped opposite the normal Detour wrap direction.)</li>
- The triangle's 3D face normal is in the general down direction.</li>

*/


// Provides neighbor data for agents managed by the crowd.
struct Moss_RecastCrowdNeighbour {
	int idx;		// The index of the neighbor in the crowd.
	float dist;		// The distance between the current agent and the neighbor.
};

// Configuration parameters for a crowd agent.
struct Moss_RecastCrowdAgent3DSettings {
	float radius;						// Agent radius. [Limit: >= 0]
	float height;						// Agent height. [Limit: > 0]
	float maxAcceleration;				// Maximum allowed acceleration. [Limit: >= 0]
	float maxSpeed;						// Maximum allowed speed. [Limit: >= 0]
	float collisionQueryRange;			// Defines how close a collision element must be before it is considered for steering behaviors. [Limits: > 0]
	float pathOptimizationRange;		// The path visibility optimization range. [Limit: > 0]
	float separationWeightXZ;				// How aggresive the agent manager should be at avoiding collisions with this agent. [Limit: >= 0]
	uint8_t updateFlags;			// Flags that impact steering behavior. (See: #UpdateFlags)
	uint8_t obstacleAvoidanceType;// The index of the avoidance configuration to use for the agent. [Limits: 0 <= value <= #DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]
	uint8_t queryFilterType;		// The index of the query filter used by this agent.
	void* userData;						// User defined data attached to the agent.
};

// Represents an agent managed by a #Moss_RecastCrowd3D object.
struct Moss_RecastCrowdAgent3D {
	bool active;					// True if the agent is active, false if the agent is in an unused slot in the agent pool.
	uint8_t state;			// The type of mesh polygon the agent is traversing. (See: #CrowdAgentState)

	bool partial;	// True if the agent has valid path (targetState == DT_CROWDAGENT_TARGET_VALID) and the path does not lead to the requested position, else false.
	Moss_RecastPathCorridor corridor;							// The path corridor the agent is using.
	Moss_RecastLocalBoundary boundary;							// The local boundary data for the agent.
	float topologyOptTime;								// Time since the agent's path corridor was optimized.
	Moss_RecastCrowdNeighbour neis[DT_CROWDAGENT_MAX_NEIGHBOURS];// The known neighbors of the agent.

	int nneis;				// The number of neighbors.
	float desiredSpeed;		// The desired speed.

	Float3 npos;		// The current agent position. [(x, y, z)]
	Float3 disp;		// A temporary value used to accumulate agent displacement during iterative collision resolution. [(x, y, z)]
	Float3 dvel;		// The desired velocity of the agent. Based on the current path, calculated from scratch each frame. [(x, y, z)]
	Float3 nvel;		// The desired velocity adjusted by obstacle avoidance, calculated from scratch each frame. [(x, y, z)]
	Float3 vel;		// The actual velocity of the agent. The change from nvel -> vel is constrained by max acceleration. [(x, y, z)]

	// The agent's configuration parameters.
	Moss_RecastCrowdAgent3DSettings params;

	// The local path corridor corners for the agent. (Staight path.) [(x, y, z) * #ncorners]
	float cornerVerts[DT_CROWDAGENT_MAX_CORNERS*3];

	// The local path corridor corner flags. (See: #dtStraightPathFlags) [(flags) * #ncorners]
	uint8_t cornerFlags[DT_CROWDAGENT_MAX_CORNERS];

	// The reference id of the polygon being entered at the corner. [(polyRef) * #ncorners]
	Moss_RecastPolyRef cornerPolys[DT_CROWDAGENT_MAX_CORNERS];

	// The number of corners.
	int ncorners;
	
	uint8_t targetState;			// State of the movement request.
	Moss_RecastPolyRef targetRef;				// Target polyref of the movement request.
	RVec3 targetPos;					// Target position of the movement request (or velocity in case of DT_CROWDAGENT_TARGET_VELOCITY).
	Moss_RecastPathQueueRef targetPathqRef;		// Path finder ref.
	bool targetReplan;					// Flag indicating that the current path is being replanned.
	float targetReplanTime;				// <Time since the agent's target was replanned.
};

struct Moss_CrowdAgent3DDebugInfo {
	int idx;
	Float3 optStart, optEnd;
	Moss_RecastObstacleAvoidanceDebugData* vod;
};

// Provides local steering behaviors for a group of agents. 
class Moss_RecastCrowd3D {
	int m_maxAgents;
	Moss_RecastCrowdAgent3D* m_agents;
	Moss_RecastCrowdAgent3D** m_activeAgents;
	
	Moss_RecastPathQueue m_pathq;

	Moss_RecastObstacleAvoidanceSettings m_obstacleQueryParams[DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS];
	Moss_RecastObstacleAvoidanceQuery* m_obstacleQuery;
	
	Moss_RecastProximityGrid* m_grid;
	
	Moss_RecastPolyRef* m_pathResult;
	int m_maxPathResult;
	
	Float3 m_agentPlacementHalfExtents;

	Moss_RecastQueryFilter m_filters[DT_CROWD_MAX_QUERY_FILTER_TYPE];

	float m_maxAgentRadius;

	int m_velocitySampleCount;

	Moss_RecastNavMeshQuery* m_navquery;

	void updateTopologyOptimization(Moss_RecastCrowdAgent3D** agents, const int nagents, const float dt);
	void updateMoveRequest(const float dt);
	void checkPathValidity(Moss_RecastCrowdAgent3D** agents, const int nagents, const float dt);

	inline int getAgentIndex(const Moss_RecastCrowdAgent3D* agent) const  { return (int)(agent - m_agents); }

	bool requestMoveTargetReplan(const int idx, Moss_RecastPolyRef ref, const float* pos);

	void purge();
	
public:
	Moss_RecastCrowd3D();
	~Moss_RecastCrowd3D();
	
	/*! @brief Initializes the crowd. @param maxAgents The maximum number of agents the crowd can manage. [Limit: >= 1] @param maxAgentRadius The maximum radius of any agent that will be added to the crowd. [Limit: > 0]  @param nav The navigation mesh to use for planning.  @return True if the initialization succeeded. */
	bool init(const int maxAgents, const float maxAgentRadius, Moss_RecastNavMesh* nav);
	
	/*! @brief Sets the shared avoidance configuration for the specified index. @param idx The index. [Limits: 0 <= value < #DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]  @param params The new configuration.*/
	void setObstacleAvoidanceParams(const int idx, const Moss_RecastObstacleAvoidanceSettings* params);

	/*! @brief Gets the shared avoidance configuration for the specified index. @param idx The index of the configuration to retreive. [Limits:  0 <= value < #DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS]  @return The requested configuration. */
	const Moss_RecastObstacleAvoidanceSettings* getObstacleAvoidanceParams(const int idx) const;
	
	/*! @brief Gets the specified agent from the pool. @param idx The agent index. [Limits: 0 <= value < #getAgentCount()] @return The requested agent. */
	const Moss_RecastCrowdAgent3D* getAgent(const int idx);

	// Gets the specified agent from the pool. @param idx The agent index. [Limits: 0 <= value < #getAgentCount()] @return The requested agent.
	Moss_RecastCrowdAgent3D* getEditableAgent(const int idx);

	// The maximum number of agents that can be managed by the object. @return The maximum number of agents.
	int getAgentCount() const;
	
	// Adds a new agent to the crowd. @param pos The requested position of the agent. [(x, y, z)]  @param params The configuration of the agent. @return The index of the agent in the agent pool. Or -1 if the agent could not be added.
	int addAgent(const float* pos, const Moss_RecastCrowdAgent3DSettings* params);

	// Updates the specified agent's configuration. @param idx The agent index. [Limits: 0 <= value < #getAgentCount()] @param params The new agent configuration.
	void updateAgentParameters(const int idx, const Moss_RecastCrowdAgent3DSettings* params);

	// Removes the agent from the crowd. @param idx The agent index. [Limits: 0 <= value < #getAgentCount()]
	void removeAgent(const int idx);
	
	// Submits a new move request for the specified agent. @param idx The agent index. [Limits: 0 <= value < #getAgentCount()]
	// @param ref The position's polygon reference. @param pos The position within the polygon. [(x, y, z)] @return True if the request was successfully submitted.
	bool requestMoveTarget(const int idx, Moss_RecastPolyRef ref, const float* pos);

	// Submits a new move request for the specified agent.
	//  @param idx		The agent index. [Limits: 0 <= value < #getAgentCount()]
	//  @param vel		The movement velocity. [(x, y, z)]
	// @return True if the request was successfully submitted.
	bool requestMoveVelocity(const int idx, const float* vel);

	// Resets any request for the specified agent. @param idx The agent index. [Limits: 0 <= value < #getAgentCount()] @return True if the request was successfully reseted.
	bool resetMoveTarget(const int idx);

	// Gets the active agents int the agent pool.
	//  @param agents		An array of agent pointers. [(#Moss_RecastCrowdAgent3D *) * maxAgents]
	//  @param maxAgents	The size of the crowd agent array.
	// @return The number of agents returned in @p agents.
	int getActiveAgents(Moss_RecastCrowdAgent3D** agents, const int maxAgents);

	// Updates the steering and positions of all agents. @param dt The time, in seconds, to update the simulation. [Limit: > 0]  @param debug A debug object to load with debug information. [Opt]
	void update(const float dt, Moss_CrowdAgent3DDebugInfo* debug);
	
	// Gets the filter used by the crowd. @return The filter used by the crowd.
	inline const Moss_RecastQueryFilter* getFilter(const int i) const { return (i >= 0 && i < DT_CROWD_MAX_QUERY_FILTER_TYPE) ? &m_filters[i] : 0; }
	
	// Gets the filter used by the crowd. @return The filter used by the crowd.
	inline Moss_RecastQueryFilter* getEditableFilter(const int i) { return (i >= 0 && i < DT_CROWD_MAX_QUERY_FILTER_TYPE) ? &m_filters[i] : 0; }

	// Gets the search halfExtents [(x, y, z)] used by the crowd for query operations.  @return The search halfExtents used by the crowd. [(x, y, z)]
	const Float3 getQueryHalfExtents() const { return m_agentPlacementHalfExtents; }

	// Same as getQueryHalfExtents. Left to maintain backwards compatibility. @return The search halfExtents used by the crowd. [(x, y, z)]
	const Float3 getQueryExtents() const { return m_agentPlacementHalfExtents; }
	
	// Gets the velocity sample count. @return The velocity sample count.
	inline int getVelocitySampleCount() const { return m_velocitySampleCount; }
	
	// Gets the crowd's proximity grid. @return The crowd's proximity grid.
	const Moss_RecastProximityGrid* getGrid() const { return m_grid; }

	// Gets the crowd's path request queue. @return The crowd's path request queue.
	const Moss_RecastPathQueue* getPathQueue() const { return &m_pathq; }

	// Gets the query object used by the crowd.
	const Moss_RecastNavMeshQuery* getNavMeshQuery() const { return m_navquery; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	Moss_RecastCrowd3D(const Moss_RecastCrowd3D&);
	Moss_RecastCrowd3D& operator=(const Moss_RecastCrowd3D&);
};



struct Moss_RecastCrowdAgent2DSettings {
	float radius;                  // Agent radius (world units)
	float maxAcceleration;         // Max acceleration (units/sec^2)
	float maxSpeed;                // Max speed (units/sec)

	float collisionQueryRange;     // How far to look for neighbors
	float pathOptimizationRange;   // Visibility optimization range

	float separationWeight;        // How strongly agents avoid each other

	uint8_t updateFlags;
	uint8_t obstacleAvoidanceType;
	uint8_t queryFilterType;

	void* userData;
};

struct Moss_RecastCrowdAgent2D {
	bool active;
	bool partial;

	int nneis;
	Moss_RecastCrowdNeighbour neis[DT_CROWDAGENT_MAX_NEIGHBOURS];

	float desiredSpeed;

	Float2 position;
	Float2 velocity;
	Float2 desiredVelocity;

	Moss_RecastCrowdAgent2DSettings params;
};

class Moss_RecastCrowd2D {
public:
	bool init(int maxAgents, float maxAgentRadius, Moss_NavMesh2D* nav);

	int addAgent(const Float2& pos, const Moss_RecastCrowdAgent2DSettings& params);
	void removeAgent(int idx);

	bool requestMoveTarget(int idx, const Float2& pos);
	bool requestMoveVelocity(int idx, const Float2& vel);
	bool resetMoveTarget(int idx);

	void update(float dt);

	const Moss_RecastCrowdAgent2D* getAgent(int idx) const;
	int getActiveAgents(Moss_RecastCrowdAgent2D** agents, int maxAgents) const;
};


inline float Dist2D(const Float3& a, const Float3& b);
inline float Dot2D(const Float3& a, const Float3& b);





struct Moss_RecastObstacleCircle {
	Float3 p;				// Position of the obstacle
	Float3 vel;				// Velocity of the obstacle
	Float3 dvel;			// Velocity of the obstacle
	float rad;				// Radius of the obstacle
	Float3 dp, np;			// Use for side selection during sampling.
};

// NOTE: Segment math is done in XZ plane.
struct Moss_RecastObstacleSegment {
	Float3 p, q;		// End points of the obstacle segment
	bool touch;
};

struct Moss_RecastTileCacheParams {
	Float3 orig;
	float cs, ch;
	int width, height;
	float walkableHeight;
	float walkableRadius;
	float walkableClimb;
	float maxSimplificationError;
	int maxTiles;
	int maxObstacles;
};


struct Moss_RecastObstacleAvoidanceSettings {
	float velBias;
	float weightDesVel;
	float weightCurVel;
	float weightSide;
	float weightToi;
	float horizTime;
	uint8_t gridSize;	// grid
	uint8_t adaptiveDivs;	// adaptive
	uint8_t adaptiveRings;	// adaptive
	uint8_t adaptiveDepth;	// adaptive
};

class Moss_RecastObstacleAvoidanceDebugData {
public:
	Moss_RecastObstacleAvoidanceDebugData();
	~Moss_RecastObstacleAvoidanceDebugData();
	
	bool init(const int maxSamples);
	void reset();
	void addSample(const float* vel, const float ssize, const float pen,
				   const float vpen, const float vcpen, const float spen, const float tpen);
	
	void normalizeSamples();
	
	inline int getSampleCount() const { return m_nsamples; }
	inline const float* getSampleVelocity(const int i) const { return &m_vel[i*3]; }
	inline float getSampleSize(const int i) const { return m_ssize[i]; }
	inline float getSamplePenalty(const int i) const { return m_pen[i]; }
	inline float getSampleDesiredVelocityPenalty(const int i) const { return m_vpen[i]; }
	inline float getSampleCurrentVelocityPenalty(const int i) const { return m_vcpen[i]; }
	inline float getSamplePreferredSidePenalty(const int i) const { return m_spen[i]; }
	inline float getSampleCollisionTimePenalty(const int i) const { return m_tpen[i]; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	Moss_RecastObstacleAvoidanceDebugData(const Moss_RecastObstacleAvoidanceDebugData&) = delete;
	Moss_RecastObstacleAvoidanceDebugData& operator=(const Moss_RecastObstacleAvoidanceDebugData&) = delete;

	int m_nsamples;
	int m_maxSamples;
	float* m_vel;
	float* m_ssize;
	float* m_pen;
	float* m_vpen;
	float* m_vcpen;
	float* m_spen;
	float* m_tpen;
};


class Moss_RecastObstacleAvoidanceQuery {
public:
	Moss_RecastObstacleAvoidanceQuery();
	~Moss_RecastObstacleAvoidanceQuery();
	
	bool init(const int maxCircles, const int maxSegments);
	
	void reset();

	void addCircle(const float* pos, const float rad, const float* vel, const float* dvel);
				   
	void addSegment(const float* p, const float* q);

	int sampleVelocityGrid(const float* pos, const float rad, const float vmax,
						   const float* vel, const float* dvel, float* nvel, const Moss_RecastObstacleAvoidanceSettings* params, Moss_RecastObstacleAvoidanceDebugData* debug = 0);

	int sampleVelocityAdaptive(const float* pos, const float rad, const float vmax, const float* vel, const float* dvel, float* nvel,
							   const Moss_RecastObstacleAvoidanceSettings* params,  Moss_RecastObstacleAvoidanceDebugData* debug = 0);
	
	inline int getObstacleCircleCount() const { return m_ncircles; }
	const Moss_RecastObstacleCircle* getObstacleCircle(const int i) { return &m_circles[i]; }

	inline int getObstacleSegmentCount() const { return m_nsegments; }
	const Moss_RecastObstacleSegment* getObstacleSegment(const int i) { return &m_segments[i]; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	Moss_RecastObstacleAvoidanceQuery(const Moss_RecastObstacleAvoidanceQuery&);
	Moss_RecastObstacleAvoidanceQuery& operator=(const Moss_RecastObstacleAvoidanceQuery&);

	void prepare(const float* pos, const float* dvel);

	float processSample(const float* vcand, const float cs, const float* pos, const float rad,
						const float* vel, const float* dvel, const float minPenalty, Moss_RecastObstacleAvoidanceDebugData* debug);

	Moss_RecastObstacleAvoidanceSettings m_params;
	float m_invHorizTime;
	float m_vmax;
	float m_invVmax;

	int m_maxCircles;
	Moss_RecastObstacleCircle* m_circles;
	int m_ncircles;

	int m_maxSegments;
	Moss_RecastObstacleSegment* m_segments;
	int m_nsegments;
};



struct Moss_RecastTileCacheObstacle;

struct Moss_RecastTileCacheCompressor {
	virtual ~Moss_RecastTileCacheCompressor();

	virtual int maxCompressedSize(const int bufferSize) = 0;
	virtual Moss_RecastStatus compress(const uint8_t* buffer, const int bufferSize, uint8_t* compressed, const int maxCompressedSize, int* compressedSize) = 0;
	virtual Moss_RecastStatus decompress(const uint8_t* compressed, const int compressedSize, uint8_t* buffer, const int maxBufferSize, int* bufferSize) = 0;
};

struct Moss_RecastTileCacheMeshProcess {
	virtual ~Moss_RecastTileCacheMeshProcess();
	virtual void process(struct Moss_RecastNavMeshCreateParams* params, uint8_t* polyAreas, uint16_t* polyFlags) = 0;
};


class Moss_RecastTileCache {
public:
	Moss_RecastTileCache();
	~Moss_RecastTileCache();
	
	struct Moss_RecastTileCacheAlloc* getAlloc() { return m_talloc; }
	struct Moss_RecastTileCacheCompressor* getCompressor() { return m_tcomp; }
	const Moss_RecastTileCacheParams* getParams() const { return &m_params; }
	
	inline int getTileCount() const { return m_params.maxTiles; }
	inline const Moss_RecastCompressedTile* getTile(const int i) const { return &m_tiles[i]; }
	
	inline int getObstacleCount() const { return m_params.maxObstacles; }
	inline const Moss_RecastTileCacheObstacle* getObstacle(const int i) const { return &m_obstacles[i]; }
	
	const Moss_RecastTileCacheObstacle* getObstacleByRef(Moss_RecastObstacleRef ref);
	
	Moss_RecastObstacleRef getObstacleRef(const Moss_RecastTileCacheObstacle* obmin) const;
	
	Moss_RecastStatus init(const Moss_RecastTileCacheParams* params, struct Moss_RecastTileCacheAlloc* talloc, struct Moss_RecastTileCacheCompressor* tcomp, struct Moss_RecastTileCacheMeshProcess* tmproc);
	
	int getTilesAt(const int tx, const int ty, Moss_RecastCompressedTileRef* tiles, const int maxTiles) const ;
	
	Moss_RecastCompressedTile* getTileAt(const int tx, const int ty, const int tlayer);
	Moss_RecastCompressedTileRef getTileRef(const Moss_RecastCompressedTile* tile) const;
	const Moss_RecastCompressedTile* getTileByRef(Moss_RecastCompressedTileRef ref) const;
	
	Moss_RecastStatus addTile(uint8_t* data, const int dataSize, uint8_t flags, Moss_RecastCompressedTileRef* result);
	
	Moss_RecastStatus removeTile(Moss_RecastCompressedTileRef ref, uint8_t** data, int* dataSize);
	
	// Cylinder obstacle.
	Moss_RecastStatus addObstacle(const float* pos, const float radius, const float height, Moss_RecastObstacleRef* result);

	// Aabb obstacle.
	Moss_RecastStatus addBoxObstacle(const float* bmin, const float* bmax, Moss_RecastObstacleRef* result);

	// Box obstacle: can be rotated in Y.
	Moss_RecastStatus addBoxObstacle(const float* center, const float* halfExtents, const float yRadians, Moss_RecastObstacleRef* result);
	
	Moss_RecastStatus removeObstacle(const Moss_RecastObstacleRef ref);
	
	Moss_RecastStatus queryTiles(const float* bmin, const float* bmax, Moss_RecastCompressedTileRef* results, int* resultCount, const int maxResults) const;
	
	// Updates the tile cache by rebuilding tiles touched by unfinished obstacle requests.
	//  @param dt			The time step size. Currently not used.
	//  @param navmesh		The mesh to affect when rebuilding tiles.
	//  @param upToDate	Whether the tile cache is fully up to date with obstacle requests and tile rebuilds.
	//  							If the tile cache is up to date another (immediate) call to update will have no effect;
	//  							otherwise another call will continue processing obstacle requests and tile rebuilds.
	Moss_RecastStatus update(const float dt, class Moss_RecastNavMesh* navmesh, bool* upToDate = 0);
	
	Moss_RecastStatus buildNavMeshTilesAt(const int tx, const int ty, class Moss_RecastNavMesh* navmesh);
	
	Moss_RecastStatus buildNavMeshTile(const Moss_RecastCompressedTileRef ref, class Moss_RecastNavMesh* navmesh);
	
	void calcTightTileBounds(const struct Moss_RecastTileCacheLayerHeader* header, float* bmin, float* bmax) const;
	
	void getObstacleBounds(const struct Moss_RecastTileCacheObstacle* ob, float* bmin, float* bmax) const;
	

	// Encodes a tile id.
	inline Moss_RecastCompressedTileRef encodeTileId(uint32_t salt, uint32_t it) const { return ((Moss_RecastCompressedTileRef)salt << m_tileBits) | (Moss_RecastCompressedTileRef)it; }
	
	// Decodes a tile salt.
	inline uint32_t decodeTileIdSalt(Moss_RecastCompressedTileRef ref) const {
		const Moss_RecastCompressedTileRef saltMask = ((Moss_RecastCompressedTileRef)1<<m_saltBits)-1;
		return (uint32_t)((ref >> m_tileBits) & saltMask);
	}
	
	// Decodes a tile id.
	inline uint32_t decodeTileIdTile(Moss_RecastCompressedTileRef ref) const {
		const Moss_RecastCompressedTileRef tileMask = ((Moss_RecastCompressedTileRef)1<<m_tileBits)-1; return (uint32_t)(ref & tileMask);
	}

	// Encodes an obstacle id.
	inline Moss_RecastObstacleRef encodeObstacleId(uint32_t salt, uint32_t it) const { return ((Moss_RecastObstacleRef)salt << 16) | (Moss_RecastObstacleRef)it; }
	
	// Decodes an obstacle salt.
	inline uint32_t decodeObstacleIdSalt(Moss_RecastObstacleRef ref) const {
		const Moss_RecastObstacleRef saltMask = ((Moss_RecastObstacleRef)1<<16)-1;
		return (uint32_t)((ref >> 16) & saltMask);
	}
	
	// Decodes an obstacle id.
	inline uint32_t decodeObstacleIdObstacle(Moss_RecastObstacleRef ref) const {
		const Moss_RecastObstacleRef tileMask = ((Moss_RecastObstacleRef)1<<16)-1; return (uint32_t)(ref & tileMask);
	}
	
	
private:
	// Explicitly disabled copy constructor and copy assignment operator.
	Moss_RecastTileCache(const Moss_RecastTileCache&);
	Moss_RecastTileCache& operator=(const Moss_RecastTileCache&);

	enum class ObstacleRequestAction {
		REQUEST_ADD,
		REQUEST_REMOVE
	};
	
	struct ObstacleRequest {
		int action;
		Moss_RecastObstacleRef ref;
	};
	
	int m_tileLutSize;						// Tile hash lookup size (must be pot).
	int m_tileLutMask;						// Tile hash lookup mask.
	
	Moss_RecastCompressedTile** m_posLookup;			// Tile hash lookup.
	Moss_RecastCompressedTile* m_nextFreeTile;		// Freelist of tiles.
	Moss_RecastCompressedTile* m_tiles;				// List of tiles.
	
	uint32_t m_saltBits;				// Number of salt bits in the tile ID.
	uint32_t m_tileBits;				// Number of tile bits in the tile ID.
	
	Moss_RecastTileCacheParams m_params;
	
	Moss_RecastTileCacheAlloc* m_talloc;
	Moss_RecastTileCacheCompressor* m_tcomp;
	Moss_RecastTileCacheMeshProcess* m_tmproc;
	
	Moss_RecastTileCacheObstacle* m_obstacles;
	Moss_RecastTileCacheObstacle* m_nextFreeObstacle;
	
	static const int MAX_REQUESTS = 64;
	ObstacleRequest m_reqs[MAX_REQUESTS];
	int m_nreqs;
	
	static const int MAX_UPDATE = 64;
	Moss_RecastCompressedTileRef m_update[MAX_UPDATE];
	int m_nupdate;
};

struct Moss_RecastTileCacheAlloc {
	virtual ~Moss_RecastTileCacheAlloc();

	virtual void reset() {}
	
	virtual void* alloc(const size_t size) { return dtAlloc(size, DT_ALLOC_TEMP); }
	
	virtual void free(void* ptr) { MOSS_FREE(ptr); }
};

// Swaps the endianness of the compressed tile data's header (#Moss_RecastTileCacheLayerHeader).
// Tile layer data does not need endian swapping as it consist only of bytes.
//  @param[in,out]	data		The tile data array.
//  @param dataSize	The size of the data array.
bool dtTileCacheHeaderSwapEndian(uint8_t* data, const int dataSize);

////////////////////////////////////////////////////////////////////////////
void duDebugDrawNavMesh(struct duDebugDraw* dd, const Moss_RecastNavMesh& mesh, uint8_t flags);
void duDebugDrawNavMeshWithClosedList(struct duDebugDraw* dd, const Moss_RecastNavMesh& mesh, const Moss_RecastNavMeshQuery& query, uint8_t flags);
void duDebugDrawNavMeshNodes(struct duDebugDraw* dd, const Moss_RecastNavMeshQuery& query);
void duDebugDrawNavMeshBVTree(struct duDebugDraw* dd, const Moss_RecastNavMesh& mesh);
void duDebugDrawNavMeshPortals(struct duDebugDraw* dd, const Moss_RecastNavMesh& mesh);
void duDebugDrawNavMeshPolysWithFlags(struct duDebugDraw* dd, const Moss_RecastNavMesh& mesh, const uint16_t polyFlags, const uint32_t col);
void duDebugDrawNavMeshPoly(struct duDebugDraw* dd, const Moss_RecastNavMesh& mesh, Moss_RecastPolyRef ref, const uint32_t col);
void duDebugDrawTileCacheLayerAreas(struct duDebugDraw* dd, const dtTileCacheLayer& layer, const float cs, const float ch);
void duDebugDrawTileCacheLayerRegions(struct duDebugDraw* dd, const dtTileCacheLayer& layer, const float cs, const float ch);
void duDebugDrawTileCacheContours(duDebugDraw* dd, const struct dtTileCacheContourSet& lcset, const float* orig, const float cs, const float ch);
void duDebugDrawTileCachePolyMesh(duDebugDraw* dd, const struct dtTileCachePolyMesh& lmesh, const float* orig, const float cs, const float ch);
void duDebugDrawTriMesh(struct duDebugDraw* dd, const float* verts, int nverts, const int* tris, const float* normals, int ntris, const uint8_t* flags, const float texScale);
void duDebugDrawTriMeshSlope(struct duDebugDraw* dd, const float* verts, int nverts, const int* tris, const float* normals, int ntris, const float walkableSlopeAngle, const float texScale);
void duDebugDrawHeightfieldSolid(struct duDebugDraw* dd, const struct Moss_RecastHeightfield& hf);
void duDebugDrawHeightfieldWalkable(struct duDebugDraw* dd, const struct Moss_RecastHeightfield& hf);
void duDebugDrawCompactHeightfieldSolid(struct duDebugDraw* dd, const struct Moss_RecastCompactHeightfield& chf);
void duDebugDrawCompactHeightfieldRegions(struct duDebugDraw* dd, const struct Moss_RecastCompactHeightfield& chf);
void duDebugDrawCompactHeightfieldDistance(struct duDebugDraw* dd, const struct Moss_RecastCompactHeightfield& chf);
void duDebugDrawHeightfieldLayer(duDebugDraw* dd, const struct Moss_RecastHeightfieldLayer& layer, const int idx);
void duDebugDrawHeightfieldLayers(duDebugDraw* dd, const struct Moss_RecastHeightfieldLayerSet& lset);
void duDebugDrawHeightfieldLayersRegions(duDebugDraw* dd, const struct Moss_RecastHeightfieldLayerSet& lset);
void duDebugDrawRegionConnections(struct duDebugDraw* dd, const struct Moss_RecastContourSet& cset, const float alpha = 1.0f);
void duDebugDrawRawContours(struct duDebugDraw* dd, const struct Moss_RecastContourSet& cset, const float alpha = 1.0f);
void duDebugDrawContours(struct duDebugDraw* dd, const struct Moss_RecastContourSet& cset, const float alpha = 1.0f);
void duDebugDrawPolyMesh(struct duDebugDraw* dd, const struct Moss_RecastPolyMesh& mesh);
void duDebugDrawPolyMeshDetail(struct duDebugDraw* dd, const struct Moss_RecastPolyMeshDetail& dmesh);
////////////////////////////////////////////////////////////////////////////

#endif // MOSS_RECASTNAVIGATION_H
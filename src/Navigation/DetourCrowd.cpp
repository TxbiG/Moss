// DetourCrowd.cpp
#include <string.h>
#include <float.h>
#include <stdlib.h>
#include <new>


#include <Moss/Navigation/navigation_intern.h>
#ifndef MOSS_DEBUG_RENDERER
	#include <Moss/Moss_Renderer.h>
#endif // MOSS_DEBUG_RENDERER




static const int MAX_ITERS_PER_UPDATE = 100;

static const int MAX_PATHQUEUE_NODES = 4096;
static const int MAX_COMMON_NODES = 512;








Moss_RecastProximityGrid* dtAllocProximityGrid() {
	void* mem = dtAlloc(sizeof(Moss_RecastProximityGrid), DT_ALLOC_PERM);
	if (!mem) return 0;
	return new(mem) Moss_RecastProximityGrid;
}

void dtFreeProximityGrid(Moss_RecastProximityGrid* ptr) {
	if (!ptr) return;
	ptr->~Moss_RecastProximityGrid();
	MOSS_FREE(ptr);
}


inline int hashPos2(int x, int y, int n)
{
	return ((x*73856093) ^ (y*19349663)) & (n-1);
}


Moss_RecastProximityGrid::Moss_RecastProximityGrid() :
	m_cellSize(0),
	m_invCellSize(0),
	m_pool(0),
	m_poolHead(0),
	m_poolSize(0),
	m_buckets(0),
	m_bucketsSize(0)
{
}

Moss_RecastProximityGrid::~Moss_RecastProximityGrid()
{
	MOSS_FREE(m_buckets);
	MOSS_FREE(m_pool);
}

bool Moss_RecastProximityGrid::init(const int poolSize, const float cellSize)
{
	dtAssert(poolSize > 0);
	dtAssert(cellSize > 0.0f);
	
	m_cellSize = cellSize;
	m_invCellSize = 1.0f / m_cellSize;
	
	// Allocate hashs buckets
	m_bucketsSize = dtNextPow2(poolSize);
	m_buckets = (unsigned short*)dtAlloc(sizeof(unsigned short)*m_bucketsSize, DT_ALLOC_PERM);
	if (!m_buckets)
		return false;
	
	// Allocate pool of items.
	m_poolSize = poolSize;
	m_poolHead = 0;
	m_pool = (Item*)dtAlloc(sizeof(Item)*m_poolSize, DT_ALLOC_PERM);
	if (!m_pool)
		return false;
	
	clear();
	
	return true;
}

void Moss_RecastProximityGrid::clear()
{
	memset(m_buckets, 0xff, sizeof(unsigned short)*m_bucketsSize);
	m_poolHead = 0;
	m_bounds[0] = 0xffff;
	m_bounds[1] = 0xffff;
	m_bounds[2] = -0xffff;
	m_bounds[3] = -0xffff;
}

void Moss_RecastProximityGrid::addItem(const unsigned short id,
							  const float minx, const float miny,
							  const float maxx, const float maxy)
{
	const int iminx = (int)floorf(minx * m_invCellSize);
	const int iminy = (int)floorf(miny * m_invCellSize);
	const int imaxx = (int)floorf(maxx * m_invCellSize);
	const int imaxy = (int)floorf(maxy * m_invCellSize);
	
	m_bounds[0] = dtMin(m_bounds[0], iminx);
	m_bounds[1] = dtMin(m_bounds[1], iminy);
	m_bounds[2] = dtMax(m_bounds[2], imaxx);
	m_bounds[3] = dtMax(m_bounds[3], imaxy);
	
	for (int y = iminy; y <= imaxy; ++y)
	{
		for (int x = iminx; x <= imaxx; ++x)
		{
			if (m_poolHead < m_poolSize)
			{
				const int h = hashPos2(x, y, m_bucketsSize);
				const unsigned short idx = (unsigned short)m_poolHead;
				m_poolHead++;
				Item& item = m_pool[idx];
				item.x = (short)x;
				item.y = (short)y;
				item.id = id;
				item.next = m_buckets[h];
				m_buckets[h] = idx;
			}
		}
	}
}

int Moss_RecastProximityGrid::queryItems(const float minx, const float miny,
								const float maxx, const float maxy,
								unsigned short* ids, const int maxIds) const
{
	const int iminx = (int)floorf(minx * m_invCellSize);
	const int iminy = (int)floorf(miny * m_invCellSize);
	const int imaxx = (int)floorf(maxx * m_invCellSize);
	const int imaxy = (int)floorf(maxy * m_invCellSize);
	
	int n = 0;
	
	for (int y = iminy; y <= imaxy; ++y)
	{
		for (int x = iminx; x <= imaxx; ++x)
		{
			const int h = hashPos2(x, y, m_bucketsSize);
			unsigned short idx = m_buckets[h];
			while (idx != 0xffff)
			{
				Item& item = m_pool[idx];
				if ((int)item.x == x && (int)item.y == y)
				{
					// Check if the id exists already.
					const unsigned short* end = ids + n;
					unsigned short* i = ids;
					while (i != end && *i != item.id)
						++i;
					// Item not found, add it.
					if (i == end)
					{
						if (n >= maxIds)
							return n;
						ids[n++] = item.id;
					}
				}
				idx = item.next;
			}
		}
	}
	
	return n;
}

int Moss_RecastProximityGrid::getItemCountAt(const int x, const int y) const
{
	int n = 0;
	
	const int h = hashPos2(x, y, m_bucketsSize);
	unsigned short idx = m_buckets[h];
	while (idx != 0xffff)
	{
		Item& item = m_pool[idx];
		if ((int)item.x == x && (int)item.y == y)
			n++;
		idx = item.next;
	}
	
	return n;
}


/*													*/

Moss_RecastCrowd3D* dtAllocCrowd()
{
	void* mem = dtAlloc(sizeof(Moss_RecastCrowd3D), DT_ALLOC_PERM);
	if (!mem) return 0;
	return new(mem) Moss_RecastCrowd3D;
}

void dtFreeCrowd(Moss_RecastCrowd3D* ptr)
{
	if (!ptr) return;
	ptr->~Moss_RecastCrowd3D();
	MOSS_FREE(ptr);
}


inline float tween(const float t, const float t0, const float t1) { return Clamp((t-t0) / (t1-t0), 0.0f, 1.0f); }

static void integrate(Moss_RecastCrowdAgent3D* ag, const float dt) {
	// Fake dynamic constraint.
	const float maxDelta = ag->params.maxAcceleration * dt;
	float dv[3];
	dtVsub(dv, ag->nvel, ag->vel);
	float ds = dtVlen(dv);
	if (ds > maxDelta)
		dtVscale(dv, dv, maxDelta/ds);
	dtVadd(ag->vel, ag->vel, dv);
	
	// Integrate
	if (dtVlen(ag->vel) > 0.0001f)
		dtVmad(ag->npos, ag->npos, ag->vel, dt);
	else
		dtVset(ag->vel,0,0,0);
}

static bool overOffmeshConnection(const Moss_RecastCrowdAgent3D* ag, const float radius)
{
	if (!ag->ncorners)
		return false;
	
	const bool offMeshConnection = (ag->cornerFlags[ag->ncorners-1] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;
	if (offMeshConnection)
	{
		const float distSq = rcVdist2DSqr(ag->npos, &ag->cornerVerts[(ag->ncorners-1)*3]);
		if (distSq < radius*radius)
			return true;
	}
	
	return false;
}

static float getDistanceToGoal(const Moss_RecastCrowdAgent3D* ag, const float range)
{
	if (!ag->ncorners)
		return range;
	
	const bool endOfPath = (ag->cornerFlags[ag->ncorners-1] & DT_STRAIGHTPATH_END) ? true : false;
	if (endOfPath)
		return dtMin(rcVdist2D(ag->npos, &ag->cornerVerts[(ag->ncorners-1)*3]), range);
	
	return range;
}

static void calcSmoothSteerDirection(const Moss_RecastCrowdAgent3D* ag, float* dir)
{
	if (!ag->ncorners)
	{
		dtVset(dir, 0,0,0);
		return;
	}
	
	const int ip0 = 0;
	const int ip1 = dtMin(1, ag->ncorners-1);
	const float* p0 = &ag->cornerVerts[ip0*3];
	const float* p1 = &ag->cornerVerts[ip1*3];
	
	float dir0[3], dir1[3];
	dtVsub(dir0, p0, ag->npos);
	dtVsub(dir1, p1, ag->npos);
	dir0[1] = 0;
	dir1[1] = 0;
	
	float len0 = dtVlen(dir0);
	float len1 = dtVlen(dir1);
	if (len1 > 0.001f)
		dtVscale(dir1,dir1,1.0f/len1);
	
	dir[0] = dir0[0] - dir1[0]*len0*0.5f;
	dir[1] = 0;
	dir[2] = dir0[2] - dir1[2]*len0*0.5f;
	
	rcVnormalize(dir);
}

static void calcStraightSteerDirection(const Moss_RecastCrowdAgent3D* ag, float* dir)
{
	if (!ag->ncorners)
	{
		dtVset(dir, 0,0,0);
		return;
	}
	dtVsub(dir, &ag->cornerVerts[0], ag->npos);
	dir[1] = 0;
	rcVnormalize(dir);
}

static int addNeighbour(const int idx, const float dist, Moss_RecastCrowdNeighbour* neis, const int nneis, const int maxNeis)
{
	// Insert neighbour based on the distance.
	Moss_RecastCrowdNeighbour* nei = 0;
	if (!nneis)
	{
		nei = &neis[nneis];
	}
	else if (dist >= neis[nneis-1].dist)
	{
		if (nneis >= maxNeis)
			return nneis;
		nei = &neis[nneis];
	}
	else
	{
		int i;
		for (i = 0; i < nneis; ++i)
			if (dist <= neis[i].dist)
				break;
		
		const int tgt = i+1;
		const int n = dtMin(nneis-i, maxNeis-tgt);
		
		dtAssert(tgt+n <= maxNeis);
		
		if (n > 0)
			memmove(&neis[tgt], &neis[i], sizeof(Moss_RecastCrowdNeighbour)*n);
		nei = &neis[i];
	}
	
	memset(nei, 0, sizeof(Moss_RecastCrowdNeighbour));
	
	nei->idx = idx;
	nei->dist = dist;
	
	return dtMin(nneis+1, maxNeis);
}

static int getNeighbours(const float* pos, const float height, const float range,
						 const Moss_RecastCrowdAgent3D* skip, Moss_RecastCrowdNeighbour* result, const int maxResult,
						 Moss_RecastCrowdAgent3D** agents, const int /*nagents*/, Moss_RecastProximityGrid* grid)
{
	int n = 0;
	
	static const int MAX_NEIS = 32;
	unsigned short ids[MAX_NEIS];
	int nids = grid->queryItems(pos[0]-range, pos[2]-range,
								pos[0]+range, pos[2]+range,
								ids, MAX_NEIS);
	
	for (int i = 0; i < nids; ++i)
	{
		const Moss_RecastCrowdAgent3D* ag = agents[ids[i]];
		
		if (ag == skip) continue;
		
		// Check for overlap.
		float diff[3];
		dtVsub(diff, pos, ag->npos);
		if (fabsf(diff[1]) >= (height+ag->params.height)/2.0f)
			continue;
		diff[1] = 0;
		const float distSqr = dtVlenSqr(diff);
		if (distSqr > Sqr(range))
			continue;
		
		n = addNeighbour(ids[i], distSqr, result, n, maxResult);
	}
	return n;
}

static int addToOptQueue(Moss_RecastCrowdAgent3D* newag, Moss_RecastCrowdAgent3D** agents, const int nagents, const int maxAgents)
{
	// Insert neighbour based on greatest time.
	int slot = 0;
	if (!nagents)
	{
		slot = nagents;
	}
	else if (newag->topologyOptTime <= agents[nagents-1]->topologyOptTime)
	{
		if (nagents >= maxAgents)
			return nagents;
		slot = nagents;
	}
	else
	{
		int i;
		for (i = 0; i < nagents; ++i)
			if (newag->topologyOptTime >= agents[i]->topologyOptTime)
				break;
		
		const int tgt = i+1;
		const int n = dtMin(nagents-i, maxAgents-tgt);
		
		dtAssert(tgt+n <= maxAgents);
		
		if (n > 0)
			memmove(&agents[tgt], &agents[i], sizeof(Moss_RecastCrowdAgent3D*)*n);
		slot = i;
	}
	
	agents[slot] = newag;
	
	return dtMin(nagents+1, maxAgents);
}

static int addToPathQueue(Moss_RecastCrowdAgent3D* newag, Moss_RecastCrowdAgent3D** agents, const int nagents, const int maxAgents)
{
	// Insert neighbour based on greatest time.
	int slot = 0;
	if (!nagents)
	{
		slot = nagents;
	}
	else if (newag->targetReplanTime <= agents[nagents-1]->targetReplanTime)
	{
		if (nagents >= maxAgents)
			return nagents;
		slot = nagents;
	}
	else
	{
		int i;
		for (i = 0; i < nagents; ++i)
			if (newag->targetReplanTime >= agents[i]->targetReplanTime)
				break;
		
		const int tgt = i+1;
		const int n = dtMin(nagents-i, maxAgents-tgt);
		
		dtAssert(tgt+n <= maxAgents);
		
		if (n > 0)
			memmove(&agents[tgt], &agents[i], sizeof(Moss_RecastCrowdAgent3D*)*n);
		slot = i;
	}
	
	agents[slot] = newag;
	
	return dtMin(nagents+1, maxAgents);
}


/**
@class Moss_RecastCrowd3D
@par

This is the core class of the @ref crowd module.  See the @ref crowd documentation for a summary
of the crowd features.

A common method for setting up the crowd is as follows:

-# Allocate the crowd using #dtAllocCrowd.
-# Initialize the crowd using #init().
-# Set the avoidance configurations using #setObstacleAvoidanceParams().
-# Add agents using #addAgent() and make an initial movement request using #requestMoveTarget().

A common process for managing the crowd is as follows:

-# Call #update() to allow the crowd to manage its agents.
-# Retrieve agent information using #getActiveAgents().
-# Make movement requests using #requestMoveTarget() when movement goal changes.
-# Repeat every frame.

Some agent configuration settings can be updated using #updateAgentParameters().  But the crowd owns the
agent position.  So it is not possible to update an active agent's position.  If agent position
must be fed back into the crowd, the agent must be removed and re-added.

Notes: 

- Path related information is available for newly added agents only after an #update() has been
  performed.
- Agent objects are kept in a pool and re-used.  So it is important when using agent objects to check the value of
  #Moss_RecastCrowdAgent3D::active to determine if the agent is actually in use or not.
- This class is meant to provide 'local' movement. There is a limit of 256 polygons in the path corridor.  
  So it is not meant to provide automatic pathfinding services over long distances.

@see dtAllocCrowd(), dtFreeCrowd(), init(), Moss_RecastCrowdAgent3D

*/

Moss_RecastCrowd3D::Moss_RecastCrowd3D() :
	m_maxAgents(0),
	m_agents(0),
	m_activeAgents(0),
	m_agentAnims(0),
	m_obstacleQuery(0),
	m_grid(0),
	m_pathResult(0),
	m_maxPathResult(0),
	m_maxAgentRadius(0),
	m_velocitySampleCount(0),
	m_navquery(0)
{
}

Moss_RecastCrowd3D::~Moss_RecastCrowd3D()
{
	purge();
}

void Moss_RecastCrowd3D::purge()
{
	for (int i = 0; i < m_maxAgents; ++i)
		m_agents[i].~Moss_RecastCrowdAgent3D();
	MOSS_FREE(m_agents);
	m_agents = 0;
	m_maxAgents = 0;
	
	MOSS_FREE(m_activeAgents);
	m_activeAgents = 0;

	MOSS_FREE(m_agentAnims);
	m_agentAnims = 0;
	
	MOSS_FREE(m_pathResult);
	m_pathResult = 0;
	
	dtFreeProximityGrid(m_grid);
	m_grid = 0;

	dtFreeObstacleAvoidanceQuery(m_obstacleQuery);
	m_obstacleQuery = 0;
	
	dtFreeNavMeshQuery(m_navquery);
	m_navquery = 0;
}

/// @par
///
/// May be called more than once to purge and re-initialize the crowd.
bool Moss_RecastCrowd3D::init(const int maxAgents, const float maxAgentRadius, Moss_RecastNavMesh* nav)
{
	purge();
	
	m_maxAgents = maxAgents;
	m_maxAgentRadius = maxAgentRadius;

	// Larger than agent radius because it is also used for agent recovery.
	dtVset(m_agentPlacementHalfExtents, m_maxAgentRadius*2.0f, m_maxAgentRadius*1.5f, m_maxAgentRadius*2.0f);
	
	m_grid = dtAllocProximityGrid();
	if (!m_grid)
		return false;
	if (!m_grid->init(m_maxAgents*4, maxAgentRadius*3))
		return false;
	
	m_obstacleQuery = dtAllocObstacleAvoidanceQuery();
	if (!m_obstacleQuery)
		return false;
	if (!m_obstacleQuery->init(6, 8))
		return false;

	// Init obstacle query params.
	memset(m_obstacleQueryParams, 0, sizeof(m_obstacleQueryParams));
	for (int i = 0; i < DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS; ++i)
	{
		Moss_RecastObstacleAvoidanceSettings* params = &m_obstacleQueryParams[i];
		params->velBias = 0.4f;
		params->weightDesVel = 2.0f;
		params->weightCurVel = 0.75f;
		params->weightSide = 0.75f;
		params->weightToi = 2.5f;
		params->horizTime = 2.5f;
		params->gridSize = 33;
		params->adaptiveDivs = 7;
		params->adaptiveRings = 2;
		params->adaptiveDepth = 5;
	}
	
	// Allocate temp buffer for merging paths.
	m_maxPathResult = 256;
	m_pathResult = (dtPolyRef*)dtAlloc(sizeof(dtPolyRef)*m_maxPathResult, DT_ALLOC_PERM);
	if (!m_pathResult)
		return false;
	
	if (!m_pathq.init(m_maxPathResult, MAX_PATHQUEUE_NODES, nav))
		return false;
	
	m_agents = (Moss_RecastCrowdAgent3D*)dtAlloc(sizeof(Moss_RecastCrowdAgent3D)*m_maxAgents, DT_ALLOC_PERM);
	if (!m_agents)
		return false;
	
	m_activeAgents = (Moss_RecastCrowdAgent3D**)dtAlloc(sizeof(Moss_RecastCrowdAgent3D*)*m_maxAgents, DT_ALLOC_PERM);
	if (!m_activeAgents)
		return false;
	
	for (int i = 0; i < m_maxAgents; ++i)
	{
		new(&m_agents[i]) Moss_RecastCrowdAgent3D();
		m_agents[i].active = false;
		if (!m_agents[i].corridor.init(m_maxPathResult))
			return false;
	}

	for (int i = 0; i < m_maxAgents; ++i)
	{
		m_agentAnims[i].active = false;
	}

	// The navquery is mostly used for local searches, no need for large node pool.
	m_navquery = dtAllocNavMeshQuery();
	if (!m_navquery)
		return false;
	if (dtStatusFailed(m_navquery->init(nav, MAX_COMMON_NODES)))
		return false;
	
	return true;
}

void Moss_RecastCrowd3D::setObstacleAvoidanceParams(const int idx, const Moss_RecastObstacleAvoidanceSettings* params)
{
	if (idx >= 0 && idx < DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS)
		memcpy(&m_obstacleQueryParams[idx], params, sizeof(Moss_RecastObstacleAvoidanceSettings));
}

const Moss_RecastObstacleAvoidanceSettings* Moss_RecastCrowd3D::getObstacleAvoidanceParams(const int idx) const
{
	if (idx >= 0 && idx < DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS)
		return &m_obstacleQueryParams[idx];
	return 0;
}

int Moss_RecastCrowd3D::getAgentCount() const
{
	return m_maxAgents;
}

/// @par
/// 
/// Agents in the pool may not be in use.  Check #Moss_RecastCrowdAgent3D.active before using the returned object.
const Moss_RecastCrowdAgent3D* Moss_RecastCrowd3D::getAgent(const int idx)
{
	if (idx < 0 || idx >= m_maxAgents)
		return 0;
	return &m_agents[idx];
}

/// 
/// Agents in the pool may not be in use.  Check #Moss_RecastCrowdAgent3D.active before using the returned object.
Moss_RecastCrowdAgent3D* Moss_RecastCrowd3D::getEditableAgent(const int idx)
{
	if (idx < 0 || idx >= m_maxAgents)
		return 0;
	return &m_agents[idx];
}

void Moss_RecastCrowd3D::updateAgentParameters(const int idx, const Moss_RecastCrowdAgent3DSettings* params)
{
	if (idx < 0 || idx >= m_maxAgents)
		return;
	memcpy(&m_agents[idx].params, params, sizeof(Moss_RecastCrowdAgent3DSettings));
}

/// @par
///
/// The agent's position will be constrained to the surface of the navigation mesh.
int Moss_RecastCrowd3D::addAgent(const float* pos, const Moss_RecastCrowdAgent3DSettings* params)
{
	// Find empty slot.
	int idx = -1;
	for (int i = 0; i < m_maxAgents; ++i)
	{
		if (!m_agents[i].active)
		{
			idx = i;
			break;
		}
	}
	if (idx == -1)
		return -1;
	
	Moss_RecastCrowdAgent3D* ag = &m_agents[idx];		

	updateAgentParameters(idx, params);
	
	// Find nearest position on navmesh and place the agent there.
	float nearest[3];
	dtPolyRef ref = 0;
	dtVcopy(nearest, pos);
	Moss_RecastStatus status = m_navquery->findNearestPoly(pos, m_agentPlacementHalfExtents, &m_filters[ag->params.queryFilterType], &ref, nearest);
	if (dtStatusFailed(status))
	{
		dtVcopy(nearest, pos);
		ref = 0;
	}
	
	ag->corridor.reset(ref, nearest);
	ag->boundary.reset();
	ag->partial = false;

	ag->topologyOptTime = 0;
	ag->targetReplanTime = 0;
	ag->nneis = 0;
	
	dtVset(ag->dvel, 0,0,0);
	dtVset(ag->nvel, 0,0,0);
	dtVset(ag->vel, 0,0,0);
	dtVcopy(ag->npos, nearest);
	
	ag->desiredSpeed = 0;

	if (ref)
		ag->state = DT_CROWDAGENT_STATE_WALKING;
	else
		ag->state = DT_CROWDAGENT_STATE_INVALID;
	
	ag->targetState = DT_CROWDAGENT_TARGET_NONE;
	
	ag->active = true;

	return idx;
}

/// @par
///
/// The agent is deactivated and will no longer be processed.  Its #Moss_RecastCrowdAgent3D object
/// is not removed from the pool.  It is marked as inactive so that it is available for reuse.
void Moss_RecastCrowd3D::removeAgent(const int idx)
{
	if (idx >= 0 && idx < m_maxAgents)
	{
		m_agents[idx].active = false;
	}
}

bool Moss_RecastCrowd3D::requestMoveTargetReplan(const int idx, dtPolyRef ref, const float* pos)
{
	if (idx < 0 || idx >= m_maxAgents)
		return false;
	
	Moss_RecastCrowdAgent3D* ag = &m_agents[idx];
	
	// Initialize request.
	ag->targetRef = ref;
	dtVcopy(ag->targetPos, pos);
	ag->targetPathqRef = DT_PATHQ_INVALID;
	ag->targetReplan = true;
	if (ag->targetRef)
		ag->targetState = DT_CROWDAGENT_TARGET_REQUESTING;
	else
		ag->targetState = DT_CROWDAGENT_TARGET_FAILED;
	
	return true;
}

/// @par
/// 
/// This method is used when a new target is set.
/// 
/// The position will be constrained to the surface of the navigation mesh.
///
/// The request will be processed during the next #update().
bool Moss_RecastCrowd3D::requestMoveTarget(const int idx, dtPolyRef ref, const float* pos)
{
	if (idx < 0 || idx >= m_maxAgents)
		return false;
	if (!ref)
		return false;

	Moss_RecastCrowdAgent3D* ag = &m_agents[idx];
	
	// Initialize request.
	ag->targetRef = ref;
	dtVcopy(ag->targetPos, pos);
	ag->targetPathqRef = DT_PATHQ_INVALID;
	ag->targetReplan = false;
	if (ag->targetRef)
		ag->targetState = DT_CROWDAGENT_TARGET_REQUESTING;
	else
		ag->targetState = DT_CROWDAGENT_TARGET_FAILED;

	return true;
}

bool Moss_RecastCrowd3D::requestMoveVelocity(const int idx, const float* vel)
{
	if (idx < 0 || idx >= m_maxAgents)
		return false;
	
	Moss_RecastCrowdAgent3D* ag = &m_agents[idx];
	
	// Initialize request.
	ag->targetRef = 0;
	dtVcopy(ag->targetPos, vel);
	ag->targetPathqRef = DT_PATHQ_INVALID;
	ag->targetReplan = false;
	ag->targetState = DT_CROWDAGENT_TARGET_VELOCITY;
	
	return true;
}

bool Moss_RecastCrowd3D::resetMoveTarget(const int idx)
{
	if (idx < 0 || idx >= m_maxAgents)
		return false;
	
	Moss_RecastCrowdAgent3D* ag = &m_agents[idx];
	
	// Initialize request.
	ag->targetRef = 0;
	dtVset(ag->targetPos, 0,0,0);
	dtVset(ag->dvel, 0,0,0);
	ag->targetPathqRef = DT_PATHQ_INVALID;
	ag->targetReplan = false;
	ag->targetState = DT_CROWDAGENT_TARGET_NONE;
	
	return true;
}

int Moss_RecastCrowd3D::getActiveAgents(Moss_RecastCrowdAgent3D** agents, const int maxAgents)
{
	int n = 0;
	for (int i = 0; i < m_maxAgents; ++i)
	{
		if (!m_agents[i].active) continue;
		if (n < maxAgents)
			agents[n++] = &m_agents[i];
	}
	return n;
}


void Moss_RecastCrowd3D::updateMoveRequest(const float /*dt*/)
{
	const int PATH_MAX_AGENTS = 8;
	Moss_RecastCrowdAgent3D* queue[PATH_MAX_AGENTS];
	int nqueue = 0;
	
	// Fire off new requests.
	for (int i = 0; i < m_maxAgents; ++i)
	{
		Moss_RecastCrowdAgent3D* ag = &m_agents[i];
		if (!ag->active)
			continue;
		if (ag->state == DT_CROWDAGENT_STATE_INVALID)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;

		if (ag->targetState == DT_CROWDAGENT_TARGET_REQUESTING)
		{
			const dtPolyRef* path = ag->corridor.getPath();
			const int npath = ag->corridor.getPathCount();
			dtAssert(npath);

			static const int MAX_RES = 32;
			float reqPos[3];
			dtPolyRef reqPath[MAX_RES];	// The path to the request location
			int reqPathCount = 0;

			// Quick search towards the goal.
			static const int MAX_ITER = 20;
			m_navquery->initSlicedFindPath(path[0], ag->targetRef, ag->npos, ag->targetPos, &m_filters[ag->params.queryFilterType]);
			m_navquery->updateSlicedFindPath(MAX_ITER, 0);
			Moss_RecastStatus status = 0;
			if (ag->targetReplan) // && npath > 10)
			{
				// Try to use existing steady path during replan if possible.
				status = m_navquery->finalizeSlicedFindPathPartial(path, npath, reqPath, &reqPathCount, MAX_RES);
			}
			else
			{
				// Try to move towards target when goal changes.
				status = m_navquery->finalizeSlicedFindPath(reqPath, &reqPathCount, MAX_RES);
			}

			if (!dtStatusFailed(status) && reqPathCount > 0)
			{
				// In progress or succeed.
				if (reqPath[reqPathCount-1] != ag->targetRef)
				{
					// Partial path, constrain target position inside the last polygon.
					status = m_navquery->closestPointOnPoly(reqPath[reqPathCount-1], ag->targetPos, reqPos, 0);
					if (dtStatusFailed(status))
						reqPathCount = 0;
				}
				else
				{
					dtVcopy(reqPos, ag->targetPos);
				}
			}
			else
			{
				reqPathCount = 0;
			}
				
			if (!reqPathCount)
			{
				// Could not find path, start the request from current location.
				dtVcopy(reqPos, ag->npos);
				reqPath[0] = path[0];
				reqPathCount = 1;
			}

			ag->corridor.setCorridor(reqPos, reqPath, reqPathCount);
			ag->boundary.reset();
			ag->partial = false;

			if (reqPath[reqPathCount-1] == ag->targetRef)
			{
				ag->targetState = DT_CROWDAGENT_TARGET_VALID;
				ag->targetReplanTime = 0.0;
			}
			else
			{
				// The path is longer or potentially unreachable, full plan.
				ag->targetState = DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE;
			}
		}
		
		if (ag->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE)
		{
			nqueue = addToPathQueue(ag, queue, nqueue, PATH_MAX_AGENTS);
		}
	}

	for (int i = 0; i < nqueue; ++i)
	{
		Moss_RecastCrowdAgent3D* ag = queue[i];
		ag->targetPathqRef = m_pathq.request(ag->corridor.getLastPoly(), ag->targetRef,
											 ag->corridor.getTarget(), ag->targetPos, &m_filters[ag->params.queryFilterType]);
		if (ag->targetPathqRef != DT_PATHQ_INVALID)
			ag->targetState = DT_CROWDAGENT_TARGET_WAITING_FOR_PATH;
	}

	
	// Update requests.
	m_pathq.update(MAX_ITERS_PER_UPDATE);

	Moss_RecastStatus status;

	// Process path results.
	for (int i = 0; i < m_maxAgents; ++i)
	{
		Moss_RecastCrowdAgent3D* ag = &m_agents[i];
		if (!ag->active)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;
		
		if (ag->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_PATH)
		{
			// Poll path queue.
			status = m_pathq.getRequestStatus(ag->targetPathqRef);
			if (dtStatusFailed(status))
			{
				// Path find failed, retry if the target location is still valid.
				ag->targetPathqRef = DT_PATHQ_INVALID;
				if (ag->targetRef)
					ag->targetState = DT_CROWDAGENT_TARGET_REQUESTING;
				else
					ag->targetState = DT_CROWDAGENT_TARGET_FAILED;
				ag->targetReplanTime = 0.0;
			}
			else if (dtStatusSucceed(status))
			{
				const dtPolyRef* path = ag->corridor.getPath();
				const int npath = ag->corridor.getPathCount();
				dtAssert(npath);
				
				// Apply results.
				float targetPos[3];
				dtVcopy(targetPos, ag->targetPos);
				
				dtPolyRef* res = m_pathResult;
				bool valid = true;
				int nres = 0;
				status = m_pathq.getPathResult(ag->targetPathqRef, res, &nres, m_maxPathResult);
				if (dtStatusFailed(status) || !nres)
					valid = false;

				if (dtStatusDetail(status, DT_PARTIAL_RESULT))
					ag->partial = true;
				else
					ag->partial = false;

				// Merge result and existing path.
				// The agent might have moved whilst the request is
				// being processed, so the path may have changed.
				// We assume that the end of the path is at the same location
				// where the request was issued.
				
				// The last ref in the old path should be the same as
				// the location where the request was issued..
				if (valid && path[npath-1] != res[0])
					valid = false;
				
				if (valid)
				{
					// Put the old path infront of the old path.
					if (npath > 1)
					{
						// Make space for the old path.
						if ((npath-1)+nres > m_maxPathResult)
							nres = m_maxPathResult - (npath-1);
						
						memmove(res+npath-1, res, sizeof(dtPolyRef)*nres);
						// Copy old path in the beginning.
						memcpy(res, path, sizeof(dtPolyRef)*(npath-1));
						nres += npath-1;
						
						// Remove trackbacks
						for (int j = 0; j < nres; ++j)
						{
							if (j-1 >= 0 && j+1 < nres)
							{
								if (res[j-1] == res[j+1])
								{
									memmove(res+(j-1), res+(j+1), sizeof(dtPolyRef)*(nres-(j+1)));
									nres -= 2;
									j -= 2;
								}
							}
						}
						
					}
					
					// Check for partial path.
					if (res[nres-1] != ag->targetRef)
					{
						// Partial path, constrain target position inside the last polygon.
						float nearest[3];
						status = m_navquery->closestPointOnPoly(res[nres-1], targetPos, nearest, 0);
						if (dtStatusSucceed(status))
							dtVcopy(targetPos, nearest);
						else
							valid = false;
					}
				}
				
				if (valid)
				{
					// Set current corridor.
					ag->corridor.setCorridor(targetPos, res, nres);
					// Force to update boundary.
					ag->boundary.reset();
					ag->targetState = DT_CROWDAGENT_TARGET_VALID;
				}
				else
				{
					// Something went wrong.
					ag->targetState = DT_CROWDAGENT_TARGET_FAILED;
				}

				ag->targetReplanTime = 0.0;
			}
		}
	}
	
}


void Moss_RecastCrowd3D::updateTopologyOptimization(Moss_RecastCrowdAgent3D** agents, const int nagents, const float dt)
{
	if (!nagents)
		return;
	
	const float OPT_TIME_THR = 0.5f; // seconds
	const int OPT_MAX_AGENTS = 1;
	Moss_RecastCrowdAgent3D* queue[OPT_MAX_AGENTS];
	int nqueue = 0;
	
	for (int i = 0; i < nagents; ++i)
	{
		Moss_RecastCrowdAgent3D* ag = agents[i];
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;
		if ((ag->params.updateFlags & DT_CROWD_OPTIMIZE_TOPO) == 0)
			continue;
		ag->topologyOptTime += dt;
		if (ag->topologyOptTime >= OPT_TIME_THR)
			nqueue = addToOptQueue(ag, queue, nqueue, OPT_MAX_AGENTS);
	}

	for (int i = 0; i < nqueue; ++i)
	{
		Moss_RecastCrowdAgent3D* ag = queue[i];
		ag->corridor.optimizePathTopology(m_navquery, &m_filters[ag->params.queryFilterType]);
		ag->topologyOptTime = 0;
	}

}

void Moss_RecastCrowd3D::checkPathValidity(Moss_RecastCrowdAgent3D** agents, const int nagents, const float dt)
{
	static const int CHECK_LOOKAHEAD = 10;
	static const float TARGET_REPLAN_DELAY = 1.0; // seconds
	
	for (int i = 0; i < nagents; ++i)
	{
		Moss_RecastCrowdAgent3D* ag = agents[i];
		
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
			
		ag->targetReplanTime += dt;

		bool replan = false;

		// First check that the current location is valid.
		const int idx = getAgentIndex(ag);
		float agentPos[3];
		dtPolyRef agentRef = ag->corridor.getFirstPoly();
		dtVcopy(agentPos, ag->npos);
		if (!m_navquery->isValidPolyRef(agentRef, &m_filters[ag->params.queryFilterType]))
		{
			// Current location is not valid, try to reposition.
			// TODO: this can snap agents, how to handle that?
			float nearest[3];
			dtVcopy(nearest, agentPos);
			agentRef = 0;
			m_navquery->findNearestPoly(ag->npos, m_agentPlacementHalfExtents, &m_filters[ag->params.queryFilterType], &agentRef, nearest);
			dtVcopy(agentPos, nearest);

			if (!agentRef)
			{
				// Could not find location in navmesh, set state to invalid.
				ag->corridor.reset(0, agentPos);
				ag->partial = false;
				ag->boundary.reset();
				ag->state = DT_CROWDAGENT_STATE_INVALID;
				continue;
			}

			// Make sure the first polygon is valid, but leave other valid
			// polygons in the path so that replanner can adjust the path better.
			ag->corridor.fixPathStart(agentRef, agentPos);
//			ag->corridor.trimInvalidPath(agentRef, agentPos, m_navquery, &m_filter);
			ag->boundary.reset();
			dtVcopy(ag->npos, agentPos);

			replan = true;
		}

		// If the agent does not have move target or is controlled by velocity, no need to recover the target nor replan.
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;

		// Try to recover move request position.
		if (ag->targetState != DT_CROWDAGENT_TARGET_NONE && ag->targetState != DT_CROWDAGENT_TARGET_FAILED)
		{
			if (!m_navquery->isValidPolyRef(ag->targetRef, &m_filters[ag->params.queryFilterType]))
			{
				// Current target is not valid, try to reposition.
				float nearest[3];
				dtVcopy(nearest, ag->targetPos);
				ag->targetRef = 0;
				m_navquery->findNearestPoly(ag->targetPos, m_agentPlacementHalfExtents, &m_filters[ag->params.queryFilterType], &ag->targetRef, nearest);
				dtVcopy(ag->targetPos, nearest);
				replan = true;
			}
			if (!ag->targetRef)
			{
				// Failed to reposition target, fail moverequest.
				ag->corridor.reset(agentRef, agentPos);
				ag->partial = false;
				ag->targetState = DT_CROWDAGENT_TARGET_NONE;
			}
		}

		// If nearby corridor is not valid, replan.
		if (!ag->corridor.isValid(CHECK_LOOKAHEAD, m_navquery, &m_filters[ag->params.queryFilterType]))
		{
			// Fix current path.
//			ag->corridor.trimInvalidPath(agentRef, agentPos, m_navquery, &m_filter);
//			ag->boundary.reset();
			replan = true;
		}
		
		// If the end of the path is near and it is not the requested location, replan.
		if (ag->targetState == DT_CROWDAGENT_TARGET_VALID)
		{
			if (ag->targetReplanTime > TARGET_REPLAN_DELAY &&
				ag->corridor.getPathCount() < CHECK_LOOKAHEAD &&
				ag->corridor.getLastPoly() != ag->targetRef)
				replan = true;
		}

		// Try to replan path to goal.
		if (replan)
		{
			if (ag->targetState != DT_CROWDAGENT_TARGET_NONE)
			{
				requestMoveTargetReplan(idx, ag->targetRef, ag->targetPos);
			}
		}
	}
}
	
void Moss_RecastCrowd3D::update(const float dt, Moss_CrowdAgent3DDebugInfo* debug)
{
	m_velocitySampleCount = 0;
	
	const int debugIdx = debug ? debug->idx : -1;
	
	Moss_RecastCrowdAgent3D** agents = m_activeAgents;
	int nagents = getActiveAgents(agents, m_maxAgents);

	// Check that all agents still have valid paths.
	checkPathValidity(agents, nagents, dt);
	
	// Update async move request and path finder.
	updateMoveRequest(dt);

	// Optimize path topology.
	updateTopologyOptimization(agents, nagents, dt);
	
	// Register agents to proximity grid.
	m_grid->clear();
	for (int i = 0; i < nagents; ++i)
	{
		Moss_RecastCrowdAgent3D* ag = agents[i];
		const float* p = ag->npos;
		const float r = ag->params.radius;
		m_grid->addItem((unsigned short)i, p[0]-r, p[2]-r, p[0]+r, p[2]+r);
	}
	
	// Get nearby navmesh segments and agents to collide with.
	for (int i = 0; i < nagents; ++i)
	{
		Moss_RecastCrowdAgent3D* ag = agents[i];
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;

		// Update the collision boundary after certain distance has been passed or
		// if it has become invalid.
		const float updateThr = ag->params.collisionQueryRange*0.25f;
		if (rcVdist2DSqr(ag->npos, ag->boundary.getCenter()) > Sqr(updateThr) ||
			!ag->boundary.isValid(m_navquery, &m_filters[ag->params.queryFilterType]))
		{
			ag->boundary.update(ag->corridor.getFirstPoly(), ag->npos, ag->params.collisionQueryRange,
								m_navquery, &m_filters[ag->params.queryFilterType]);
		}
		// Query neighbour agents
		ag->nneis = getNeighbours(ag->npos, ag->params.height, ag->params.collisionQueryRange,
								  ag, ag->neis, DT_CROWDAGENT_MAX_NEIGHBOURS,
								  agents, nagents, m_grid);
		for (int j = 0; j < ag->nneis; j++)
			ag->neis[j].idx = getAgentIndex(agents[ag->neis[j].idx]);
	}
	
	// Find next corner to steer to.
	for (int i = 0; i < nagents; ++i)
	{
		Moss_RecastCrowdAgent3D* ag = agents[i];
		
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;
		
		// Find corners for steering
		ag->ncorners = ag->corridor.findCorners(ag->cornerVerts, ag->cornerFlags, ag->cornerPolys,
												DT_CROWDAGENT_MAX_CORNERS, m_navquery, &m_filters[ag->params.queryFilterType]);
		
		// Check to see if the corner after the next corner is directly visible,
		// and short cut to there.
		if ((ag->params.updateFlags & DT_CROWD_OPTIMIZE_VIS) && ag->ncorners > 0)
		{
			const float* target = &ag->cornerVerts[dtMin(1,ag->ncorners-1)*3];
			ag->corridor.optimizePathVisibility(target, ag->params.pathOptimizationRange, m_navquery, &m_filters[ag->params.queryFilterType]);
			
			// Copy data for debug purposes.
			if (debugIdx == i)
			{
				dtVcopy(debug->optStart, ag->corridor.getPos());
				dtVcopy(debug->optEnd, target);
			}
		}
		else
		{
			// Copy data for debug purposes.
			if (debugIdx == i)
			{
				dtVset(debug->optStart, 0,0,0);
				dtVset(debug->optEnd, 0,0,0);
			}
		}
	}
	
	// Trigger off-mesh connections (depends on corners).
	for (int i = 0; i < nagents; ++i)
	{
		Moss_RecastCrowdAgent3D* ag = agents[i];
		
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;
		
		// Check 
		const float triggerRadius = ag->params.radius*2.25f;
		if (overOffmeshConnection(ag, triggerRadius))
		{
			// Prepare to off-mesh connection.
			const int idx = (int)(ag - m_agents);
			
			// Adjust the path over the off-mesh connection.
			dtPolyRef refs[2];
			if (ag->corridor.moveOverOffmeshConnection(ag->cornerPolys[ag->ncorners-1], refs,
													   anim->startPos, anim->endPos, m_navquery))
			{
				dtVcopy(anim->initPos, ag->npos);
				anim->polyRef = refs[1];
				anim->active = true;
				anim->t = 0.0f;
				anim->tmax = (rcVdist2D(anim->startPos, anim->endPos) / ag->params.maxSpeed) * 0.5f;
				
				ag->state = DT_CROWDAGENT_STATE_OFFMESH;
				ag->ncorners = 0;
				ag->nneis = 0;
				continue;
			}
			else
			{
				// Path validity check will ensure that bad/blocked connections will be replanned.
			}
		}
	}
		
	// Calculate steering.
	for (int i = 0; i < nagents; ++i)
	{
		Moss_RecastCrowdAgent3D* ag = agents[i];

		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE)
			continue;
		
		float dvel[3] = {0,0,0};

		if (ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		{
			dtVcopy(dvel, ag->targetPos);
			ag->desiredSpeed = dtVlen(ag->targetPos);
		}
		else
		{
			// Calculate steering direction.
			if (ag->params.updateFlags & DT_CROWD_ANTICIPATE_TURNS)
				calcSmoothSteerDirection(ag, dvel);
			else
				calcStraightSteerDirection(ag, dvel);
			
			// Calculate speed scale, which tells the agent to slowdown at the end of the path.
			const float slowDownRadius = ag->params.radius*2;	// TODO: make less hacky.
			const float speedScale = getDistanceToGoal(ag, slowDownRadius) / slowDownRadius;
				
			ag->desiredSpeed = ag->params.maxSpeed;
			dtVscale(dvel, dvel, ag->desiredSpeed * speedScale);
		}

		// Separation
		if (ag->params.updateFlags & DT_CROWD_SEPARATION)
		{
			const float separationDist = ag->params.collisionQueryRange; 
			const float invSeparationDist = 1.0f / separationDist; 
			const float separationWeightXZ = ag->params.separationWeightXZ;
			
			float w = 0;
			float disp[3] = {0,0,0};
			
			for (int j = 0; j < ag->nneis; ++j)
			{
				const Moss_RecastCrowdAgent3D* nei = &m_agents[ag->neis[j].idx];
				
				float diff[3];
				dtVsub(diff, ag->npos, nei->npos);
				diff[1] = 0;
				
				const float distSqr = dtVlenSqr(diff);
				if (distSqr < 0.00001f)
					continue;
				if (distSqr > Sqr(separationDist))
					continue;
				const float dist = sqrtf(distSqr);
				const float weight = separationWeightXZ * (1.0f - Sqr(dist*invSeparationDist));
				
				dtVmad(disp, disp, diff, weight/dist);
				w += 1.0f;
			}
			
			if (w > 0.0001f)
			{
				// Adjust desired velocity.
				dtVmad(dvel, dvel, disp, 1.0f/w);
				// Clamp desired velocity to desired speed.
				const float speedSqr = dtVlenSqr(dvel);
				const float desiredSqr = Sqr(ag->desiredSpeed);
				if (speedSqr > desiredSqr)
					dtVscale(dvel, dvel, desiredSqr/speedSqr);
			}
		}
		
		// Set the desired velocity.
		dtVcopy(ag->dvel, dvel);
	}
	
	// Velocity planning.	
	for (int i = 0; i < nagents; ++i)
	{
		Moss_RecastCrowdAgent3D* ag = agents[i];
		
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		
		if (ag->params.updateFlags & DT_CROWD_OBSTACLE_AVOIDANCE)
		{
			m_obstacleQuery->reset();
			
			// Add neighbours as obstacles.
			for (int j = 0; j < ag->nneis; ++j)
			{
				const Moss_RecastCrowdAgent3D* nei = &m_agents[ag->neis[j].idx];
				m_obstacleQuery->addCircle(nei->npos, nei->params.radius, nei->vel, nei->dvel);
			}

			// Append neighbour segments as obstacles.
			for (int j = 0; j < ag->boundary.getSegmentCount(); ++j)
			{
				const float* s = ag->boundary.getSegment(j);
				if (dtTriArea2D(ag->npos, s, s+3) < 0.0f)
					continue;
				m_obstacleQuery->addSegment(s, s+3);
			}

			Moss_RecastObstacleAvoidanceDebugData* vod = 0;
			if (debugIdx == i) 
				vod = debug->vod;
			
			// Sample new safe velocity.
			bool adaptive = true;
			int ns = 0;

			const Moss_RecastObstacleAvoidanceSettings* params = &m_obstacleQueryParams[ag->params.obstacleAvoidanceType];
				
			if (adaptive)
			{
				ns = m_obstacleQuery->sampleVelocityAdaptive(ag->npos, ag->params.radius, ag->desiredSpeed,
															 ag->vel, ag->dvel, ag->nvel, params, vod);
			}
			else
			{
				ns = m_obstacleQuery->sampleVelocityGrid(ag->npos, ag->params.radius, ag->desiredSpeed,
														 ag->vel, ag->dvel, ag->nvel, params, vod);
			}
			m_velocitySampleCount += ns;
		}
		else
		{
			// If not using velocity planning, new velocity is directly the desired velocity.
			dtVcopy(ag->nvel, ag->dvel);
		}
	}

	// Integrate.
	for (int i = 0; i < nagents; ++i)
	{
		Moss_RecastCrowdAgent3D* ag = agents[i];
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		integrate(ag, dt);
	}
	
	// Handle collisions.
	static const float COLLISION_RESOLVE_FACTOR = 0.7f;
	
	for (int iter = 0; iter < 4; ++iter)
	{
		for (int i = 0; i < nagents; ++i)
		{
			Moss_RecastCrowdAgent3D* ag = agents[i];
			const int idx0 = getAgentIndex(ag);
			
			if (ag->state != DT_CROWDAGENT_STATE_WALKING)
				continue;

			dtVset(ag->disp, 0,0,0);
			
			float w = 0;

			for (int j = 0; j < ag->nneis; ++j)
			{
				const Moss_RecastCrowdAgent3D* nei = &m_agents[ag->neis[j].idx];
				const int idx1 = getAgentIndex(nei);

				float diff[3];
				dtVsub(diff, ag->npos, nei->npos);
				diff[1] = 0;
				
				float dist = dtVlenSqr(diff);
				if (dist > Sqr(ag->params.radius + nei->params.radius))
					continue;
				dist = sqrtf(dist);
				float pen = (ag->params.radius + nei->params.radius) - dist;
				if (dist < 0.0001f)
				{
					// Agents on top of each other, try to choose diverging separation directions.
					if (idx0 > idx1)
						dtVset(diff, -ag->dvel[2],0,ag->dvel[0]);
					else
						dtVset(diff, ag->dvel[2],0,-ag->dvel[0]);
					pen = 0.01f;
				}
				else
				{
					pen = (1.0f/dist) * (pen*0.5f) * COLLISION_RESOLVE_FACTOR;
				}
				
				dtVmad(ag->disp, ag->disp, diff, pen);			
				
				w += 1.0f;
			}
			
			if (w > 0.0001f)
			{
				const float iw = 1.0f / w;
				dtVscale(ag->disp, ag->disp, iw);
			}
		}
		
		for (int i = 0; i < nagents; ++i)
		{
			Moss_RecastCrowdAgent3D* ag = agents[i];
			if (ag->state != DT_CROWDAGENT_STATE_WALKING)
				continue;
			
			dtVadd(ag->npos, ag->npos, ag->disp);
		}
	}
	
	for (int i = 0; i < nagents; ++i)
	{
		Moss_RecastCrowdAgent3D* ag = agents[i];
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		
		// Move along navmesh.
		ag->corridor.movePosition(ag->npos, m_navquery, &m_filters[ag->params.queryFilterType]);
		// Get valid constrained position back.
		dtVcopy(ag->npos, ag->corridor.getPos());

		// If not using path, truncate the corridor to just one poly.
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		{
			ag->corridor.reset(ag->corridor.getFirstPoly(), ag->npos);
			ag->partial = false;
		}

	}
	
	// Update agents using off-mesh connection.
	for (int i = 0; i < nagents; ++i)
	{
		Moss_RecastCrowdAgent3D* ag = agents[i];
		const int idx = (int)(ag - m_agents);

		if (!anim->active)
			continue;
		

		anim->t += dt;
		if (anim->t > anim->tmax)
		{
			// Reset animation
			anim->active = false;
			// Prepare agent for walking.
			ag->state = DT_CROWDAGENT_STATE_WALKING;
			continue;
		}
		
		// Update position
		const float ta = anim->tmax*0.15f;
		const float tb = anim->tmax;
		if (anim->t < ta)
		{
			const float u = tween(anim->t, 0.0, ta);
			dtVlerp(ag->npos, anim->initPos, anim->startPos, u);
		}
		else
		{
			const float u = tween(anim->t, ta, tb);
			dtVlerp(ag->npos, anim->startPos, anim->endPos, u);
		}
			
		// Update velocity.
		dtVset(ag->vel, 0,0,0);
		dtVset(ag->dvel, 0,0,0);
	}
	
}



/*													*/
Moss_RecastLocalBoundary::Moss_RecastLocalBoundary() :
	m_nsegs(0),
	m_npolys(0)
{
	dtVset(m_center, FLT_MAX,FLT_MAX,FLT_MAX);
}

Moss_RecastLocalBoundary::~Moss_RecastLocalBoundary()
{
}

void Moss_RecastLocalBoundary::reset()
{
	dtVset(m_center, FLT_MAX,FLT_MAX,FLT_MAX);
	m_npolys = 0;
	m_nsegs = 0;
}

void Moss_RecastLocalBoundary::addSegment(const float dist, const float* s)
{
	// Insert neighbour based on the distance.
	Segment* seg = 0;
	if (!m_nsegs)
	{
		// First, trivial accept.
		seg = &m_segs[0];
	}
	else if (dist >= m_segs[m_nsegs-1].d)
	{
		// Further than the last segment, skip.
		if (m_nsegs >= MAX_LOCAL_SEGS)
			return;
		// Last, trivial accept.
		seg = &m_segs[m_nsegs];
	}
	else
	{
		// Insert inbetween.
		int i;
		for (i = 0; i < m_nsegs; ++i)
			if (dist <= m_segs[i].d)
				break;
		const int tgt = i+1;
		const int n = dtMin(m_nsegs-i, MAX_LOCAL_SEGS-tgt);
		dtAssert(tgt+n <= MAX_LOCAL_SEGS);
		if (n > 0)
			memmove(&m_segs[tgt], &m_segs[i], sizeof(Segment)*n);
		seg = &m_segs[i];
	}
	
	seg->d = dist;
	memcpy(seg->s, s, sizeof(float)*6);
	
	if (m_nsegs < MAX_LOCAL_SEGS)
		m_nsegs++;
}

void Moss_RecastLocalBoundary::update(dtPolyRef ref, const float* pos, const float collisionQueryRange,
							 Moss_RecastNavMeshQuery* navquery, const Moss_RecastQueryFilter* filter)
{
	static const int MAX_SEGS_PER_POLY = DT_VERTS_PER_POLYGON*3;
	
	if (!ref)
	{
		dtVset(m_center, FLT_MAX,FLT_MAX,FLT_MAX);
		m_nsegs = 0;
		m_npolys = 0;
		return;
	}
	
	dtVcopy(m_center, pos);
	
	// First query non-overlapping polygons.
	navquery->findLocalNeighbourhood(ref, pos, collisionQueryRange,
									 filter, m_polys, 0, &m_npolys, MAX_LOCAL_POLYS);
	
	// Secondly, store all polygon edges.
	m_nsegs = 0;
	float segs[MAX_SEGS_PER_POLY*6];
	int nsegs = 0;
	for (int j = 0; j < m_npolys; ++j)
	{
		navquery->getPolyWallSegments(m_polys[j], filter, segs, 0, &nsegs, MAX_SEGS_PER_POLY);
		for (int k = 0; k < nsegs; ++k)
		{
			const float* s = &segs[k*6];
			// Skip too distant segments.
			float tseg;
			const float distSqr = dtDistancePtSegSqr2D(pos, s, s+3, tseg);
			if (distSqr > Sqr(collisionQueryRange))
				continue;
			addSegment(distSqr, s);
		}
	}
}

bool Moss_RecastLocalBoundary::isValid(Moss_RecastNavMeshQuery* navquery, const Moss_RecastQueryFilter* filter)
{
	if (!m_npolys)
		return false;
	
	// Check that all polygons still pass query filter.
	for (int i = 0; i < m_npolys; ++i)
	{
		if (!navquery->isValidPolyRef(m_polys[i], filter))
			return false;
	}
	
	return true;
}



/*													*/

static int sweepCircleCircle(const float* c0, const float r0, const float* v,
							 const float* c1, const float r1,
							 float& tmin, float& tmax)
{
	static const float EPS = 0.0001f;
	float s[3];
	dtVsub(s,c1,c0);
	float r = r0+r1;
	float c = dtVdot2D(s,s) - r*r;
	float a = dtVdot2D(v,v);
	if (a < EPS) return 0;	// not moving
	
	// Overlap, calc time to exit.
	float b = dtVdot2D(v,s);
	float d = b*b - a*c;
	if (d < 0.0f) return 0; // no intersection.
	a = 1.0f / a;
	const float rd = sqrtf(d);
	tmin = (b - rd) * a;
	tmax = (b + rd) * a;
	return 1;
}

static int isectRaySeg(const float* ap, const float* u,
					   const float* bp, const float* bq,
					   float& t)
{
	float v[3], w[3];
	dtVsub(v,bq,bp);
	dtVsub(w,ap,bp);
	float d = dtVperp2D(u,v);
	if (fabsf(d) < 1e-6f) return 0;
	d = 1.0f/d;
	t = dtVperp2D(v,w) * d;
	if (t < 0 || t > 1) return 0;
	float s = dtVperp2D(u,w) * d;
	if (s < 0 || s > 1) return 0;
	return 1;
}



Moss_RecastObstacleAvoidanceDebugData* dtAllocObstacleAvoidanceDebugData()
{
	void* mem = dtAlloc(sizeof(Moss_RecastObstacleAvoidanceDebugData), DT_ALLOC_PERM);
	if (!mem) return 0;
	return new(mem) Moss_RecastObstacleAvoidanceDebugData;
}

void dtFreeObstacleAvoidanceDebugData(Moss_RecastObstacleAvoidanceDebugData* ptr)
{
	if (!ptr) return;
	ptr->~Moss_RecastObstacleAvoidanceDebugData();
	MOSS_FREE(ptr);
}


Moss_RecastObstacleAvoidanceDebugData::Moss_RecastObstacleAvoidanceDebugData() :
	m_nsamples(0),
	m_maxSamples(0),
	m_vel(0),
	m_ssize(0),
	m_pen(0),
	m_vpen(0),
	m_vcpen(0),
	m_spen(0),
	m_tpen(0)
{
}

Moss_RecastObstacleAvoidanceDebugData::~Moss_RecastObstacleAvoidanceDebugData()
{
	MOSS_FREE(m_vel);
	MOSS_FREE(m_ssize);
	MOSS_FREE(m_pen);
	MOSS_FREE(m_vpen);
	MOSS_FREE(m_vcpen);
	MOSS_FREE(m_spen);
	MOSS_FREE(m_tpen);
}
		
bool Moss_RecastObstacleAvoidanceDebugData::init(const int maxSamples)
{
	dtAssert(maxSamples);
	m_maxSamples = maxSamples;

	m_vel = (float*)dtAlloc(sizeof(float)*3*m_maxSamples, DT_ALLOC_PERM);
	if (!m_vel)
		return false;
	m_pen = (float*)dtAlloc(sizeof(float)*m_maxSamples, DT_ALLOC_PERM);
	if (!m_pen)
		return false;
	m_ssize = (float*)dtAlloc(sizeof(float)*m_maxSamples, DT_ALLOC_PERM);
	if (!m_ssize)
		return false;
	m_vpen = (float*)dtAlloc(sizeof(float)*m_maxSamples, DT_ALLOC_PERM);
	if (!m_vpen)
		return false;
	m_vcpen = (float*)dtAlloc(sizeof(float)*m_maxSamples, DT_ALLOC_PERM);
	if (!m_vcpen)
		return false;
	m_spen = (float*)dtAlloc(sizeof(float)*m_maxSamples, DT_ALLOC_PERM);
	if (!m_spen)
		return false;
	m_tpen = (float*)dtAlloc(sizeof(float)*m_maxSamples, DT_ALLOC_PERM);
	if (!m_tpen)
		return false;
	
	return true;
}

void Moss_RecastObstacleAvoidanceDebugData::reset()
{
	m_nsamples = 0;
}

void Moss_RecastObstacleAvoidanceDebugData::addSample(const float* vel, const float ssize, const float pen,
											 const float vpen, const float vcpen, const float spen, const float tpen)
{
	if (m_nsamples >= m_maxSamples)
		return;
	dtAssert(m_vel);
	dtAssert(m_ssize);
	dtAssert(m_pen);
	dtAssert(m_vpen);
	dtAssert(m_vcpen);
	dtAssert(m_spen);
	dtAssert(m_tpen);
	dtVcopy(&m_vel[m_nsamples*3], vel);
	m_ssize[m_nsamples] = ssize;
	m_pen[m_nsamples] = pen;
	m_vpen[m_nsamples] = vpen;
	m_vcpen[m_nsamples] = vcpen;
	m_spen[m_nsamples] = spen;
	m_tpen[m_nsamples] = tpen;
	m_nsamples++;
}

static void normalizeArray(float* arr, const int n)
{
	// Normalize penaly range.
	float minPen = FLT_MAX;
	float maxPen = -FLT_MAX;
	for (int i = 0; i < n; ++i)
	{
		minPen = dtMin(minPen, arr[i]);
		maxPen = dtMax(maxPen, arr[i]);
	}
	const float penRange = maxPen-minPen;
	const float s = penRange > 0.001f ? (1.0f / penRange) : 1;
	for (int i = 0; i < n; ++i)
		arr[i] = Clamp((arr[i]-minPen)*s, 0.0f, 1.0f);
}

void Moss_RecastObstacleAvoidanceDebugData::normalizeSamples()
{
	normalizeArray(m_pen, m_nsamples);
	normalizeArray(m_vpen, m_nsamples);
	normalizeArray(m_vcpen, m_nsamples);
	normalizeArray(m_spen, m_nsamples);
	normalizeArray(m_tpen, m_nsamples);
}


Moss_RecastObstacleAvoidanceQuery* dtAllocObstacleAvoidanceQuery()
{
	void* mem = dtAlloc(sizeof(Moss_RecastObstacleAvoidanceQuery), DT_ALLOC_PERM);
	if (!mem) return 0;
	return new(mem) Moss_RecastObstacleAvoidanceQuery;
}

void dtFreeObstacleAvoidanceQuery(Moss_RecastObstacleAvoidanceQuery* ptr)
{
	if (!ptr) return;
	ptr->~Moss_RecastObstacleAvoidanceQuery();
	MOSS_FREE(ptr);
}


Moss_RecastObstacleAvoidanceQuery::Moss_RecastObstacleAvoidanceQuery() :
	m_invHorizTime(0),
	m_vmax(0),
	m_invVmax(0),
	m_maxCircles(0),
	m_circles(0),
	m_ncircles(0),
	m_maxSegments(0),
	m_segments(0),
	m_nsegments(0)
{
}

Moss_RecastObstacleAvoidanceQuery::~Moss_RecastObstacleAvoidanceQuery()
{
	MOSS_FREE(m_circles);
	MOSS_FREE(m_segments);
}

bool Moss_RecastObstacleAvoidanceQuery::init(const int maxCircles, const int maxSegments)
{
	m_maxCircles = maxCircles;
	m_ncircles = 0;
	m_circles = (Moss_RecastObstacleCircle*)dtAlloc(sizeof(Moss_RecastObstacleCircle)*m_maxCircles, DT_ALLOC_PERM);
	if (!m_circles)
		return false;
	memset(m_circles, 0, sizeof(Moss_RecastObstacleCircle)*m_maxCircles);

	m_maxSegments = maxSegments;
	m_nsegments = 0;
	m_segments = (Moss_RecastObstacleSegment*)dtAlloc(sizeof(Moss_RecastObstacleSegment)*m_maxSegments, DT_ALLOC_PERM);
	if (!m_segments)
		return false;
	memset(m_segments, 0, sizeof(Moss_RecastObstacleSegment)*m_maxSegments);
	
	return true;
}

void Moss_RecastObstacleAvoidanceQuery::reset()
{
	m_ncircles = 0;
	m_nsegments = 0;
}

void Moss_RecastObstacleAvoidanceQuery::addCircle(const float* pos, const float rad,
										 const float* vel, const float* dvel)
{
	if (m_ncircles >= m_maxCircles)
		return;
		
	Moss_RecastObstacleCircle* cir = &m_circles[m_ncircles++];
	dtVcopy(cir->p, pos);
	cir->rad = rad;
	dtVcopy(cir->vel, vel);
	dtVcopy(cir->dvel, dvel);
}

void Moss_RecastObstacleAvoidanceQuery::addSegment(const float* p, const float* q)
{
	if (m_nsegments >= m_maxSegments)
		return;
	
	Moss_RecastObstacleSegment* seg = &m_segments[m_nsegments++];
	dtVcopy(seg->p, p);
	dtVcopy(seg->q, q);
}

void Moss_RecastObstacleAvoidanceQuery::prepare(const float* pos, const float* dvel)
{
	// Prepare obstacles
	for (int i = 0; i < m_ncircles; ++i)
	{
		Moss_RecastObstacleCircle* cir = &m_circles[i];
		
		// Side
		const float* pa = pos;
		const float* pb = cir->p;
		
		const float orig[3] = {0,0,0};
		float dv[3];
		dtVsub(cir->dp,pb,pa);
		rcVnormalize(cir->dp);
		dtVsub(dv, cir->dvel, dvel);
		
		const float a = dtTriArea2D(orig, cir->dp,dv);
		if (a < 0.01f)
		{
			cir->np[0] = -cir->dp[2];
			cir->np[2] = cir->dp[0];
		}
		else
		{
			cir->np[0] = cir->dp[2];
			cir->np[2] = -cir->dp[0];
		}
	}	

	for (int i = 0; i < m_nsegments; ++i)
	{
		Moss_RecastObstacleSegment* seg = &m_segments[i];
		
		// Precalc if the agent is really close to the segment.
		const float r = 0.01f;
		float t;
		seg->touch = dtDistancePtSegSqr2D(pos, seg->p, seg->q, t) < Sqr(r);
	}	
}


/* Calculate the collision penalty for a given velocity vector
 * 
 * @param vcand sampled velocity
 * @param dvel desired velocity
 * @param minPenalty threshold penalty for early out
 */
float Moss_RecastObstacleAvoidanceQuery::processSample(const float* vcand, const float cs,
											  const float* pos, const float rad,
											  const float* vel, const float* dvel,
											  const float minPenalty,
											  Moss_RecastObstacleAvoidanceDebugData* debug)
{
	// penalty for straying away from the desired and current velocities
	const float vpen = m_params.weightDesVel * (rcVdist2D(vcand, dvel) * m_invVmax);
	const float vcpen = m_params.weightCurVel * (rcVdist2D(vcand, vel) * m_invVmax);

	// find the threshold hit time to bail out based on the early out penalty
	// (see how the penalty is calculated below to understand)
	float minPen = minPenalty - vpen - vcpen;
	float tThresold = (m_params.weightToi / minPen - 0.1f) * m_params.horizTime;
	if (tThresold - m_params.horizTime > -FLT_EPSILON)
		return minPenalty; // already too much

	// Find min time of impact and exit amongst all obstacles.
	float tmin = m_params.horizTime;
	float side = 0;
	int nside = 0;
	
	for (int i = 0; i < m_ncircles; ++i)
	{
		const Moss_RecastObstacleCircle* cir = &m_circles[i];
			
		// RVO
		float vab[3];
		dtVscale(vab, vcand, 2);
		dtVsub(vab, vab, vel);
		dtVsub(vab, vab, cir->vel);
		
		// Side
		side += Clamp(dtMin(dtVdot2D(cir->dp,vab)*0.5f+0.5f, dtVdot2D(cir->np,vab)*2), 0.0f, 1.0f);
		nside++;
		
		float htmin = 0, htmax = 0;
		if (!sweepCircleCircle(pos,rad, vab, cir->p,cir->rad, htmin, htmax))
			continue;
		
		// Handle overlapping obstacles.
		if (htmin < 0.0f && htmax > 0.0f)
		{
			// Avoid more when overlapped.
			htmin = -htmin * 0.5f;
		}
		
		if (htmin >= 0.0f)
		{
			// The closest obstacle is somewhere ahead of us, keep track of nearest obstacle.
			if (htmin < tmin)
			{
				tmin = htmin;
				if (tmin < tThresold)
					return minPenalty;
			}
		}
	}

	for (int i = 0; i < m_nsegments; ++i)
	{
		const Moss_RecastObstacleSegment* seg = &m_segments[i];
		float htmin = 0;
		
		if (seg->touch)
		{
			// Special case when the agent is very close to the segment.
			float sdir[3], snorm[3];
			dtVsub(sdir, seg->q, seg->p);
			snorm[0] = -sdir[2];
			snorm[2] = sdir[0];
			// If the velocity is pointing towards the segment, no collision.
			if (dtVdot2D(snorm, vcand) < 0.0f)
				continue;
			// Else immediate collision.
			htmin = 0.0f;
		}
		else
		{
			if (!isectRaySeg(pos, vcand, seg->p, seg->q, htmin))
				continue;
		}
		
		// Avoid less when facing walls.
		htmin *= 2.0f;
		
		// The closest obstacle is somewhere ahead of us, keep track of nearest obstacle.
		if (htmin < tmin)
		{
			tmin = htmin;
			if (tmin < tThresold)
				return minPenalty;
		}
	}
	
	// Normalize side bias, to prevent it dominating too much.
	if (nside)
		side /= nside;
	
	const float spen = m_params.weightSide * side;
	const float tpen = m_params.weightToi * (1.0f/(0.1f+tmin*m_invHorizTime));
	
	const float penalty = vpen + vcpen + spen + tpen;
	
	// Store different penalties for debug viewing
	if (debug)
		debug->addSample(vcand, cs, penalty, vpen, vcpen, spen, tpen);
	
	return penalty;
}

int Moss_RecastObstacleAvoidanceQuery::sampleVelocityGrid(const float* pos, const float rad, const float vmax,
												 const float* vel, const float* dvel, float* nvel,
												 const Moss_RecastObstacleAvoidanceSettings* params,
												 Moss_RecastObstacleAvoidanceDebugData* debug)
{
	prepare(pos, dvel);
	
	memcpy(&m_params, params, sizeof(Moss_RecastObstacleAvoidanceSettings));
	m_invHorizTime = 1.0f / m_params.horizTime;
	m_vmax = vmax;
	m_invVmax = vmax > 0 ? 1.0f / vmax : FLT_MAX;
	
	dtVset(nvel, 0,0,0);
	
	if (debug)
		debug->reset();

	const float cvx = dvel[0] * m_params.velBias;
	const float cvz = dvel[2] * m_params.velBias;
	const float cs = vmax * 2 * (1 - m_params.velBias) / (float)(m_params.gridSize-1);
	const float half = (m_params.gridSize-1)*cs*0.5f;
		
	float minPenalty = FLT_MAX;
	int ns = 0;
		
	for (int y = 0; y < m_params.gridSize; ++y)
	{
		for (int x = 0; x < m_params.gridSize; ++x)
		{
			float vcand[3];
			vcand[0] = cvx + x*cs - half;
			vcand[1] = 0;
			vcand[2] = cvz + y*cs - half;
			
			if (Sqr(vcand[0])+Sqr(vcand[2]) > Sqr(vmax+cs/2)) continue;
			
			const float penalty = processSample(vcand, cs, pos,rad,vel,dvel, minPenalty, debug);
			ns++;
			if (penalty < minPenalty)
			{
				minPenalty = penalty;
				dtVcopy(nvel, vcand);
			}
		}
	}
	
	return ns;
}


// vector normalization that ignores the y-component.
inline void dtNormalize2D(float* v)
{
	float d = sqrtf(v[0] * v[0] + v[2] * v[2]);
	if (d==0)
		return;
	d = 1.0f / d;
	v[0] *= d;
	v[2] *= d;
}

// vector normalization that ignores the y-component.
inline void dtRorate2D(float* dest, const float* v, float ang)
{
	float c = cosf(ang);
	float s = sinf(ang);
	dest[0] = v[0]*c - v[2]*s;
	dest[2] = v[0]*s + v[2]*c;
	dest[1] = v[1];
}


int Moss_RecastObstacleAvoidanceQuery::sampleVelocityAdaptive(const float* pos, const float rad, const float vmax,
													 const float* vel, const float* dvel, float* nvel,
													 const Moss_RecastObstacleAvoidanceSettings* params,
													 Moss_RecastObstacleAvoidanceDebugData* debug)
{
	prepare(pos, dvel);
	
	memcpy(&m_params, params, sizeof(Moss_RecastObstacleAvoidanceSettings));
	m_invHorizTime = 1.0f / m_params.horizTime;
	m_vmax = vmax;
	m_invVmax = vmax > 0 ? 1.0f / vmax : FLT_MAX;
	
	dtVset(nvel, 0,0,0);
	
	if (debug)
		debug->reset();

	// Build sampling pattern aligned to desired velocity.
	float pat[(DT_MAX_PATTERN_DIVS*DT_MAX_PATTERN_RINGS+1)*2];
	int npat = 0;

	const int ndivs = (int)m_params.adaptiveDivs;
	const int nrings= (int)m_params.adaptiveRings;
	const int depth = (int)m_params.adaptiveDepth;
	
	const int nd = Clamp(ndivs, 1, DT_MAX_PATTERN_DIVS);
	const int nr = Clamp(nrings, 1, DT_MAX_PATTERN_RINGS);
	const float da = (1.0f/nd) * MOSS_PI*2;
	const float ca = cosf(da);
	const float sa = sinf(da);

	// desired direction
	float ddir[6];
	dtVcopy(ddir, dvel);
	dtNormalize2D(ddir);
	dtRorate2D (ddir+3, ddir, da*0.5f); // rotated by da/2

	// Always add sample at zero
	pat[npat*2+0] = 0;
	pat[npat*2+1] = 0;
	npat++;
	
	for (int j = 0; j < nr; ++j)
	{
		const float r = (float)(nr-j)/(float)nr;
		pat[npat*2+0] = ddir[(j%2)*3] * r;
		pat[npat*2+1] = ddir[(j%2)*3+2] * r;
		float* last1 = pat + npat*2;
		float* last2 = last1;
		npat++;

		for (int i = 1; i < nd-1; i+=2)
		{
			// get next point on the "right" (rotate CW)
			pat[npat*2+0] = last1[0]*ca + last1[1]*sa;
			pat[npat*2+1] = -last1[0]*sa + last1[1]*ca;
			// get next point on the "left" (rotate CCW)
			pat[npat*2+2] = last2[0]*ca - last2[1]*sa;
			pat[npat*2+3] = last2[0]*sa + last2[1]*ca;

			last1 = pat + npat*2;
			last2 = last1 + 2;
			npat += 2;
		}

		if ((nd&1) == 0)
		{
			pat[npat*2+2] = last2[0]*ca - last2[1]*sa;
			pat[npat*2+3] = last2[0]*sa + last2[1]*ca;
			npat++;
		}
	}


	// Start sampling.
	float cr = vmax * (1.0f - m_params.velBias);
	float res[3];
	dtVset(res, dvel[0] * m_params.velBias, 0, dvel[2] * m_params.velBias);
	int ns = 0;

	for (int k = 0; k < depth; ++k)
	{
		float minPenalty = FLT_MAX;
		float bvel[3];
		dtVset(bvel, 0,0,0);
		
		for (int i = 0; i < npat; ++i)
		{
			float vcand[3];
			vcand[0] = res[0] + pat[i*2+0]*cr;
			vcand[1] = 0;
			vcand[2] = res[2] + pat[i*2+1]*cr;
			
			if (Sqr(vcand[0])+Sqr(vcand[2]) > Sqr(vmax+0.001f)) continue;
			
			const float penalty = processSample(vcand,cr/10, pos,rad,vel,dvel, minPenalty, debug);
			ns++;
			if (penalty < minPenalty)
			{
				minPenalty = penalty;
				dtVcopy(bvel, vcand);
			}
		}

		dtVcopy(res, bvel);

		cr *= 0.5f;
	}	
	
	dtVcopy(nvel, res);
	
	return ns;
}



/*													*/
int dtMergeCorridorStartMoved(dtPolyRef* path, const int npath, const int maxPath,
							  const dtPolyRef* visited, const int nvisited)
{
	int furthestPath = -1;
	int furthestVisited = -1;
	
	// Find furthest common polygon.
	for (int i = npath-1; i >= 0; --i)
	{
		bool found = false;
		for (int j = nvisited-1; j >= 0; --j)
		{
			if (path[i] == visited[j])
			{
				furthestPath = i;
				furthestVisited = j;
				found = true;
			}
		}
		if (found)
			break;
	}
	
	// If no intersection found just return current path. 
	if (furthestPath == -1 || furthestVisited == -1)
		return npath;
	
	// Concatenate paths.	
	
	// Adjust beginning of the buffer to include the visited.
	const int req = nvisited - furthestVisited;
	const int orig = dtMin(furthestPath+1, npath);
	int size = dtMax(0, npath-orig);
	if (req+size > maxPath)
		size = maxPath-req;
	if (size > 0)
		memmove(path+req, path+orig, size*sizeof(dtPolyRef));

	// Store visited
	for (int i = 0, n = dtMin(req, maxPath); i < n; ++i)
		path[i] = visited[(nvisited-1)-i];

	return req+size;
}

int dtMergeCorridorEndMoved(dtPolyRef* path, const int npath, const int maxPath,
							const dtPolyRef* visited, const int nvisited)
{
	int furthestPath = -1;
	int furthestVisited = -1;
	
	// Find furthest common polygon.
	for (int i = 0; i < npath; ++i)
	{
		bool found = false;
		for (int j = nvisited-1; j >= 0; --j)
		{
			if (path[i] == visited[j])
			{
				furthestPath = i;
				furthestVisited = j;
				found = true;
			}
		}
		if (found)
			break;
	}
	
	// If no intersection found just return current path. 
	if (furthestPath == -1 || furthestVisited == -1)
		return npath;
	
	// Concatenate paths.
	const int ppos = furthestPath+1;
	const int vpos = furthestVisited+1;
	const int count = dtMin(nvisited-vpos, maxPath-ppos);
	dtAssert(ppos+count <= maxPath);
	if (count)
		memcpy(path+ppos, visited+vpos, sizeof(dtPolyRef)*count);
	
	return ppos+count;
}

int dtMergeCorridorStartShortcut(dtPolyRef* path, const int npath, const int maxPath,
								 const dtPolyRef* visited, const int nvisited)
{
	int furthestPath = -1;
	int furthestVisited = -1;
	
	// Find furthest common polygon.
	for (int i = npath-1; i >= 0; --i)
	{
		bool found = false;
		for (int j = nvisited-1; j >= 0; --j)
		{
			if (path[i] == visited[j])
			{
				furthestPath = i;
				furthestVisited = j;
				found = true;
			}
		}
		if (found)
			break;
	}
	
	// If no intersection found just return current path. 
	if (furthestPath == -1 || furthestVisited == -1)
		return npath;
	
	// Concatenate paths.	
	
	// Adjust beginning of the buffer to include the visited.
	const int req = furthestVisited;
	if (req <= 0)
		return npath;
	
	const int orig = furthestPath;
	int size = dtMax(0, npath-orig);
	if (req+size > maxPath)
		size = maxPath-req;
	if (size)
		memmove(path+req, path+orig, size*sizeof(dtPolyRef));
	
	// Store visited
	for (int i = 0; i < req; ++i)
		path[i] = visited[i];
	
	return req+size;
}

/**
@class Moss_RecastPathCorridor
@par

The corridor is loaded with a path, usually obtained from a #Moss_RecastNavMeshQuery::findPath() query. The corridor
is then used to plan local movement, with the corridor automatically updating as needed to deal with inaccurate 
agent locomotion.

Example of a common use case:

-# Construct the corridor object and call #init() to allocate its path buffer.
-# Obtain a path from a #Moss_RecastNavMeshQuery object.
-# Use #reset() to set the agent's current position. (At the beginning of the path.)
-# Use #setCorridor() to load the path and target.
-# Use #findCorners() to plan movement. (This handles dynamic path straightening.)
-# Use #movePosition() to feed agent movement back into the corridor. (The corridor will automatically adjust as needed.)
-# If the target is moving, use #moveTargetPosition() to update the end of the corridor. 
   (The corridor will automatically adjust as needed.)
-# Repeat the previous 3 steps to continue to move the agent.

The corridor position and target are always constrained to the navigation mesh.

One of the difficulties in maintaining a path is that floating point errors, locomotion inaccuracies, and/or local 
steering can result in the agent crossing the boundary of the path corridor, temporarily invalidating the path. 
This class uses local mesh queries to detect and update the corridor as needed to handle these types of issues. 

The fact that local mesh queries are used to move the position and target locations results in two beahviors that 
need to be considered:

Every time a move function is used there is a chance that the path will become non-optimial. Basically, the further 
the target is moved from its original location, and the further the position is moved outside the original corridor, 
the more likely the path will become non-optimal. This issue can be addressed by periodically running the 
#optimizePathTopology() and #optimizePathVisibility() methods.

All local mesh queries have distance limitations. (Review the #Moss_RecastNavMeshQuery methods for details.) So the most accurate 
use case is to move the position and target in small increments. If a large increment is used, then the corridor 
may not be able to accurately find the new location.  Because of this limiation, if a position is moved in a large
increment, then compare the desired and resulting polygon references. If the two do not match, then path replanning 
may be needed.  E.g. If you move the target, check #getLastPoly() to see if it is the expected polygon.

*/

Moss_RecastPathCorridor::Moss_RecastPathCorridor() : m_path(0), m_npath(0), m_maxPath(0) { }

Moss_RecastPathCorridor::~Moss_RecastPathCorridor() { MOSS_FREE(m_path); }

/// @par
///
/// @warning Cannot be called more than once.
bool Moss_RecastPathCorridor::init(const int maxPath)
{
	dtAssert(!m_path);
	m_path = (dtPolyRef*)dtAlloc(sizeof(dtPolyRef)*maxPath, DT_ALLOC_PERM);
	if (!m_path)
		return false;
	m_npath = 0;
	m_maxPath = maxPath;
	return true;
}

/// @par
///
/// Essentially, the corridor is set of one polygon in size with the target
/// equal to the position.
void Moss_RecastPathCorridor::reset(dtPolyRef ref, const float* pos)
{
	dtAssert(m_path);
	dtVcopy(m_pos, pos);
	dtVcopy(m_target, pos);
	m_path[0] = ref;
	m_npath = 1;
}

/**
@par

This is the function used to plan local movement within the corridor. One or more corners can be 
detected in order to plan movement. It performs essentially the same function as #Moss_RecastNavMeshQuery::findStraightPath.

Due to internal optimizations, the maximum number of corners returned will be (@p maxCorners - 1) 
For example: If the buffers are sized to hold 10 corners, the function will never return more than 9 corners. 
So if 10 corners are needed, the buffers should be sized for 11 corners.

If the target is within range, it will be the last corner and have a polygon reference id of zero.
*/
int Moss_RecastPathCorridor::findCorners(float* cornerVerts, unsigned char* cornerFlags,
							  dtPolyRef* cornerPolys, const int maxCorners,
							  Moss_RecastNavMeshQuery* navquery, const Moss_RecastQueryFilter* /*filter*/)
{
	dtAssert(m_path);
	dtAssert(m_npath);
	
	static const float MIN_TARGET_DIST = 0.01f;
	
	int ncorners = 0;
	navquery->findStraightPath(m_pos, m_target, m_path, m_npath,
							   cornerVerts, cornerFlags, cornerPolys, &ncorners, maxCorners);
	
	// Prune points in the beginning of the path which are too close.
	while (ncorners)
	{
		if ((cornerFlags[0] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ||
			rcVdist2DSqr(&cornerVerts[0], m_pos) > Sqr(MIN_TARGET_DIST))
			break;
		ncorners--;
		if (ncorners)
		{
			memmove(cornerFlags, cornerFlags+1, sizeof(unsigned char)*ncorners);
			memmove(cornerPolys, cornerPolys+1, sizeof(dtPolyRef)*ncorners);
			memmove(cornerVerts, cornerVerts+3, sizeof(float)*3*ncorners);
		}
	}
	
	// Prune points after an off-mesh connection.
	for (int i = 0; i < ncorners; ++i)
	{
		if (cornerFlags[i] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)
		{
			ncorners = i+1;
			break;
		}
	}
	
	return ncorners;
}

/** 
@par

Inaccurate locomotion or dynamic obstacle avoidance can force the argent position significantly outside the 
original corridor. Over time this can result in the formation of a non-optimal corridor. Non-optimal paths can 
also form near the corners of tiles.

This function uses an efficient local visibility search to try to optimize the corridor 
between the current position and @p next.

The corridor will change only if @p next is visible from the current position and moving directly toward the point 
is better than following the existing path.

The more inaccurate the agent movement, the more beneficial this function becomes. Simply adjust the frequency 
of the call to match the needs to the agent.

This function is not suitable for long distance searches.
*/
void Moss_RecastPathCorridor::optimizePathVisibility(const float* next, const float pathOptimizationRange,
										  Moss_RecastNavMeshQuery* navquery, const Moss_RecastQueryFilter* filter)
{
	dtAssert(m_path);
	
	// Clamp the ray to max distance.
	float goal[3];
	dtVcopy(goal, next);
	float dist = rcVdist2D(m_pos, goal);
	
	// If too close to the goal, do not try to optimize.
	if (dist < 0.01f)
		return;
	
	// Overshoot a little. This helps to optimize open fields in tiled meshes.
	dist = dtMin(dist+0.01f, pathOptimizationRange);
	
	// Adjust ray length.
	float delta[3];
	dtVsub(delta, goal, m_pos);
	dtVmad(goal, m_pos, delta, pathOptimizationRange/dist);
	
	static const int MAX_RES = 32;
	dtPolyRef res[MAX_RES];
	float t, norm[3];
	int nres = 0;
	navquery->raycast(m_path[0], m_pos, goal, filter, &t, norm, res, &nres, MAX_RES);
	if (nres > 1 && t > 0.99f)
	{
		m_npath = dtMergeCorridorStartShortcut(m_path, m_npath, m_maxPath, res, nres);
	}
}

/**
@par

Inaccurate locomotion or dynamic obstacle avoidance can force the agent position significantly outside the 
original corridor. Over time this can result in the formation of a non-optimal corridor. This function will use a 
local area path search to try to re-optimize the corridor.

The more inaccurate the agent movement, the more beneficial this function becomes. Simply adjust the frequency of 
the call to match the needs to the agent.
*/
bool Moss_RecastPathCorridor::optimizePathTopology(Moss_RecastNavMeshQuery* navquery, const Moss_RecastQueryFilter* filter)
{
	dtAssert(navquery);
	dtAssert(filter);
	dtAssert(m_path);
	
	if (m_npath < 3)
		return false;
	
	static const int MAX_ITER = 32;
	static const int MAX_RES = 32;
	
	dtPolyRef res[MAX_RES];
	int nres = 0;
	navquery->initSlicedFindPath(m_path[0], m_path[m_npath-1], m_pos, m_target, filter);
	navquery->updateSlicedFindPath(MAX_ITER, 0);
	Moss_RecastStatus status = navquery->finalizeSlicedFindPathPartial(m_path, m_npath, res, &nres, MAX_RES);
	
	if (dtStatusSucceed(status) && nres > 0)
	{
		m_npath = dtMergeCorridorStartShortcut(m_path, m_npath, m_maxPath, res, nres);
		return true;
	}
	
	return false;
}

bool Moss_RecastPathCorridor::moveOverOffmeshConnection(dtPolyRef offMeshConRef, dtPolyRef* refs,
											   float* startPos, float* endPos,
											   Moss_RecastNavMeshQuery* navquery)
{
	dtAssert(navquery);
	dtAssert(m_path);
	dtAssert(m_npath);

	// Advance the path up to and over the off-mesh connection.
	dtPolyRef prevRef = 0, polyRef = m_path[0];
	int npos = 0;
	while (npos < m_npath && polyRef != offMeshConRef)
	{
		prevRef = polyRef;
		polyRef = m_path[npos];
		npos++;
	}
	if (npos == m_npath)
	{
		// Could not find offMeshConRef
		return false;
	}
	
	// Prune path
	for (int i = npos; i < m_npath; ++i)
		m_path[i-npos] = m_path[i];
	m_npath -= npos;

	refs[0] = prevRef;
	refs[1] = polyRef;
	
	const Moss_RecastNavMesh* nav = navquery->getAttachedNavMesh();
	dtAssert(nav);

	Moss_RecastStatus status = nav->getOffMeshConnectionPolyEndPoints(refs[0], refs[1], startPos, endPos);
	if (dtStatusSucceed(status))
	{
		dtVcopy(m_pos, endPos);
		return true;
	}

	return false;
}

/**
@par

Behavior:

- The movement is constrained to the surface of the navigation mesh. 
- The corridor is automatically adjusted (shorted or lengthened) in order to remain valid. 
- The new position will be located in the adjusted corridor's first polygon.

The expected use case is that the desired position will be 'near' the current corridor. What is considered 'near' 
depends on local polygon density, query search half extents, etc.

The resulting position will differ from the desired position if the desired position is not on the navigation mesh, 
or it can't be reached using a local search.
*/
bool Moss_RecastPathCorridor::movePosition(const float* npos, Moss_RecastNavMeshQuery* navquery, const Moss_RecastQueryFilter* filter)
{
	dtAssert(m_path);
	dtAssert(m_npath);
	
	// Move along navmesh and update new position.
	float result[3];
	static const int MAX_VISITED = 16;
	dtPolyRef visited[MAX_VISITED];
	int nvisited = 0;
	Moss_RecastStatus status = navquery->moveAlongSurface(m_path[0], m_pos, npos, filter,
												 result, visited, &nvisited, MAX_VISITED);
	if (dtStatusSucceed(status)) {
		m_npath = dtMergeCorridorStartMoved(m_path, m_npath, m_maxPath, visited, nvisited);
		
		// Adjust the position to stay on top of the navmesh.
		float h = m_pos[1];
		navquery->getPolyHeight(m_path[0], result, &h);
		result[1] = h;
		dtVcopy(m_pos, result);
		return true;
	}
	return false;
}

/**
@par

Behavior:

- The movement is constrained to the surface of the navigation mesh. 
- The corridor is automatically adjusted (shorted or lengthened) in order to remain valid. 
- The new target will be located in the adjusted corridor's last polygon.

The expected use case is that the desired target will be 'near' the current corridor. What is considered 'near' depends on local polygon density, query search half extents, etc.

The resulting target will differ from the desired target if the desired target is not on the navigation mesh, or it can't be reached using a local search.
*/
bool Moss_RecastPathCorridor::moveTargetPosition(const float* npos, Moss_RecastNavMeshQuery* navquery, const Moss_RecastQueryFilter* filter)
{
	dtAssert(m_path);
	dtAssert(m_npath);
	
	// Move along navmesh and update new position.
	float result[3];
	static const int MAX_VISITED = 16;
	dtPolyRef visited[MAX_VISITED];
	int nvisited = 0;
	Moss_RecastStatus status = navquery->moveAlongSurface(m_path[m_npath-1], m_target, npos, filter,
												 result, visited, &nvisited, MAX_VISITED);
	if (dtStatusSucceed(status))
	{
		m_npath = dtMergeCorridorEndMoved(m_path, m_npath, m_maxPath, visited, nvisited);
		// TODO: should we do that?
		// Adjust the position to stay on top of the navmesh.
		/*	float h = m_target[1];
		 navquery->getPolyHeight(m_path[m_npath-1], result, &h);
		 result[1] = h;*/
		
		dtVcopy(m_target, result);
		
		return true;
	}
	return false;
}

/// @par
///
/// The current corridor position is expected to be within the first polygon in the path. The target 
/// is expected to be in the last polygon. 
/// 
/// @warning The size of the path must not exceed the size of corridor's path buffer set during #init().
void Moss_RecastPathCorridor::setCorridor(const float* target, const dtPolyRef* path, const int npath)
{
	dtAssert(m_path);
	dtAssert(npath > 0);
	dtAssert(npath <= m_maxPath);
	
	dtVcopy(m_target, target);
	memcpy(m_path, path, sizeof(dtPolyRef)*npath);
	m_npath = npath;
}

bool Moss_RecastPathCorridor::fixPathStart(dtPolyRef safeRef, const float* safePos)
{
	dtAssert(m_path);

	dtVcopy(m_pos, safePos);
	if (m_npath < 3 && m_npath > 0)
	{
		m_path[2] = m_path[m_npath-1];
		m_path[0] = safeRef;
		m_path[1] = 0;
		m_npath = 3;
	}
	else
	{
		m_path[0] = safeRef;
		m_path[1] = 0;
	}
	
	return true;
}

bool Moss_RecastPathCorridor::trimInvalidPath(dtPolyRef safeRef, const float* safePos,
									 Moss_RecastNavMeshQuery* navquery, const Moss_RecastQueryFilter* filter)
{
	dtAssert(navquery);
	dtAssert(filter);
	dtAssert(m_path);
	
	// Keep valid path as far as possible.
	int n = 0;
	while (n < m_npath && navquery->isValidPolyRef(m_path[n], filter)) {
		n++;
	}
	
	if (n == m_npath)
	{
		// All valid, no need to fix.
		return true;
	}
	else if (n == 0)
	{
		// The first polyref is bad, use current safe values.
		dtVcopy(m_pos, safePos);
		m_path[0] = safeRef;
		m_npath = 1;
	}
	else
	{
		// The path is partially usable.
		m_npath = n;
	}
	
	// Clamp target pos to last poly
	float tgt[3];
	dtVcopy(tgt, m_target);
	navquery->closestPointOnPolyBoundary(m_path[m_npath-1], tgt, m_target);
	
	return true;
}

/// @par
///
/// The path can be invalidated if there are structural changes to the underlying navigation mesh, or the state of 
/// a polygon within the path changes resulting in it being filtered out. (E.g. An exclusion or inclusion flag changes.)
bool Moss_RecastPathCorridor::isValid(const int maxLookAhead, Moss_RecastNavMeshQuery* navquery, const Moss_RecastQueryFilter* filter)
{
	// Check that all polygons still pass query filter.
	const int n = dtMin(m_npath, maxLookAhead);
	for (int i = 0; i < n; ++i)
	{
		if (!navquery->isValidPolyRef(m_path[i], filter))
			return false;
	}

	return true;
}


/*													*/
Moss_RecastPathQueue::Moss_RecastPathQueue() :
	m_nextHandle(1),
	m_maxPathSize(0),
	m_queueHead(0),
	m_navquery(0)
{
	for (int i = 0; i < MAX_QUEUE; ++i)
		m_queue[i].path = 0;
}

Moss_RecastPathQueue::~Moss_RecastPathQueue()
{
	purge();
}

void Moss_RecastPathQueue::purge()
{
	dtFreeNavMeshQuery(m_navquery);
	m_navquery = 0;
	for (int i = 0; i < MAX_QUEUE; ++i)
	{
		MOSS_FREE(m_queue[i].path);
		m_queue[i].path = 0;
	}
}

bool Moss_RecastPathQueue::init(const int maxPathSize, const int maxSearchNodeCount, Moss_RecastNavMesh* nav)
{
	purge();

	m_navquery = dtAllocNavMeshQuery();
	if (!m_navquery)
		return false;
	if (dtStatusFailed(m_navquery->init(nav, maxSearchNodeCount)))
		return false;
	
	m_maxPathSize = maxPathSize;
	for (int i = 0; i < MAX_QUEUE; ++i)
	{
		m_queue[i].ref = DT_PATHQ_INVALID;
		m_queue[i].path = (dtPolyRef*)dtAlloc(sizeof(dtPolyRef)*m_maxPathSize, DT_ALLOC_PERM);
		if (!m_queue[i].path)
			return false;
	}
	
	m_queueHead = 0;
	
	return true;
}

void Moss_RecastPathQueue::update(const int maxIters)
{
	static const int MAX_KEEP_ALIVE = 2; // in update ticks.

	// Update path request until there is nothing to update
	// or upto maxIters pathfinder iterations has been consumed.
	int iterCount = maxIters;
	
	for (int i = 0; i < MAX_QUEUE; ++i)
	{
		PathQuery& q = m_queue[m_queueHead % MAX_QUEUE];
		
		// Skip inactive requests.
		if (q.ref == DT_PATHQ_INVALID)
		{
			m_queueHead++;
			continue;
		}
		
		// Handle completed request.
		if (dtStatusSucceed(q.status) || dtStatusFailed(q.status))
		{
			// If the path result has not been read in few frames, free the slot.
			q.keepAlive++;
			if (q.keepAlive > MAX_KEEP_ALIVE)
			{
				q.ref = DT_PATHQ_INVALID;
				q.status = 0;
			}
			
			m_queueHead++;
			continue;
		}
		
		// Handle query start.
		if (q.status == 0)
		{
			q.status = m_navquery->initSlicedFindPath(q.startRef, q.endRef, q.startPos, q.endPos, q.filter);
		}		
		// Handle query in progress.
		if (dtStatusInProgress(q.status))
		{
			int iters = 0;
			q.status = m_navquery->updateSlicedFindPath(iterCount, &iters);
			iterCount -= iters;
		}
		if (dtStatusSucceed(q.status))
		{
			q.status = m_navquery->finalizeSlicedFindPath(q.path, &q.npath, m_maxPathSize);
		}

		if (iterCount <= 0)
			break;

		m_queueHead++;
	}
}

Moss_RecastPathQueueRef Moss_RecastPathQueue::request(dtPolyRef startRef, dtPolyRef endRef,
									const float* startPos, const float* endPos,
									const Moss_RecastQueryFilter* filter)
{
	// Find empty slot
	int slot = -1;
	for (int i = 0; i < MAX_QUEUE; ++i)
	{
		if (m_queue[i].ref == DT_PATHQ_INVALID)
		{
			slot = i;
			break;
		}
	}
	// Could not find slot.
	if (slot == -1)
		return DT_PATHQ_INVALID;
	
	Moss_RecastPathQueueRef ref = m_nextHandle++;
	if (m_nextHandle == DT_PATHQ_INVALID) m_nextHandle++;
	
	PathQuery& q = m_queue[slot];
	q.ref = ref;
	dtVcopy(q.startPos, startPos);
	q.startRef = startRef;
	dtVcopy(q.endPos, endPos);
	q.endRef = endRef;
	
	q.status = 0;
	q.npath = 0;
	q.filter = filter;
	q.keepAlive = 0;
	
	return ref;
}

Moss_RecastStatus Moss_RecastPathQueue::getRequestStatus(Moss_RecastPathQueueRef ref) const
{
	for (int i = 0; i < MAX_QUEUE; ++i)
	{
		if (m_queue[i].ref == ref)
			return m_queue[i].status;
	}
	return DT_FAILURE;
}

Moss_RecastStatus Moss_RecastPathQueue::getPathResult(Moss_RecastPathQueueRef ref, dtPolyRef* path, int* pathSize, const int maxPath)
{
	for (int i = 0; i < MAX_QUEUE; ++i)
	{
		if (m_queue[i].ref == ref)
		{
			PathQuery& q = m_queue[i];
			Moss_RecastStatus details = q.status & DT_STATUS_DETAIL_MASK;
			// Free request for reuse.
			q.ref = DT_PATHQ_INVALID;
			q.status = 0;
			// Copy path
			int n = dtMin(q.npath, maxPath);
			memcpy(path, q.path, sizeof(dtPolyRef)*n);
			*pathSize = n;
			return details | DT_SUCCESS;
		}
	}
	return DT_FAILURE;
}
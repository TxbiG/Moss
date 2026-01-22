// Debug Utils.cpp

#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include <Moss/Navigation/navigation_intern.h>
#ifndef MOSS_DEBUG_RENDERER
	#include <Moss/Moss_Renderer.h>
#endif // MOSS_DEBUG_RENDERER


MOSS_SUPRESS_WARNINGS_BEGIN

duFileIO::~duFileIO()
{
	// Defined out of line to fix the weak v-tables warning
}
	
static void ioprintf(duFileIO* io, const char* format, ...)
{
	char line[256];
	va_list ap;
	va_start(ap, format);
	const int n = vsnprintf(line, sizeof(line), format, ap);
	va_end(ap);
	if (n > 0)
		io->write(line, sizeof(char)*n);
}

bool duDumpPolyMeshToObj(Moss_RecastPolyMesh& pmesh, duFileIO* io)
{
	if (!io)
	{
		printf("duDumpPolyMeshToObj: input IO is null.\n"); 
		return false;
	}
	if (!io->isWriting())
	{
		printf("duDumpPolyMeshToObj: input IO not writing.\n"); 
		return false;
	}
	
	const int nvp = pmesh.nvp;
	const float cs = pmesh.cs;
	const float ch = pmesh.ch;
	const float* orig = pmesh.bmin;
	
	ioprintf(io, "# Recast Navmesh\n");
	ioprintf(io, "o NavMesh\n");

	ioprintf(io, "\n");
	
	for (int i = 0; i < pmesh.nverts; ++i)
	{
		const unsigned short* v = &pmesh.verts[i*3];
		const float x = orig[0] + v[0]*cs;
		const float y = orig[1] + (v[1]+1)*ch + 0.1f;
		const float z = orig[2] + v[2]*cs;
		ioprintf(io, "v %f %f %f\n", x,y,z);
	}

	ioprintf(io, "\n");

	for (int i = 0; i < pmesh.npolys; ++i)
	{
		const unsigned short* p = &pmesh.polys[i*nvp*2];
		for (int j = 2; j < nvp; ++j)
		{
			if (p[j] == RC_MESH_NULL_IDX) break;
			ioprintf(io, "f %d %d %d\n", p[0]+1, p[j-1]+1, p[j]+1); 
		}
	}
	
	return true;
}

bool duDumpPolyMeshDetailToObj(Moss_RecastPolyMeshDetail& dmesh, duFileIO* io)
{
	if (!io)
	{
		printf("duDumpPolyMeshDetailToObj: input IO is null.\n"); 
		return false;
	}
	if (!io->isWriting())
	{
		printf("duDumpPolyMeshDetailToObj: input IO not writing.\n"); 
		return false;
	}
	
	ioprintf(io, "# Recast Navmesh\n");
	ioprintf(io, "o NavMesh\n");
	
	ioprintf(io, "\n");

	for (int i = 0; i < dmesh.nverts; ++i)
	{
		const float* v = &dmesh.verts[i*3];
		ioprintf(io, "v %f %f %f\n", v[0],v[1],v[2]);
	}
	
	ioprintf(io, "\n");
	
	for (int i = 0; i < dmesh.nmeshes; ++i)
	{
		const unsigned int* m = &dmesh.meshes[i*4];
		const unsigned int bverts = m[0];
		const unsigned int btris = m[2];
		const unsigned int ntris = m[3];
		const unsigned char* tris = &dmesh.tris[btris*4];
		for (unsigned int j = 0; j < ntris; ++j)
		{
			ioprintf(io, "f %d %d %d\n",
					(int)(bverts+tris[j*4+0])+1,
					(int)(bverts+tris[j*4+1])+1,
					(int)(bverts+tris[j*4+2])+1);
		}
	}
	
	return true;
}

static const int CSET_MAGIC = ('c' << 24) | ('s' << 16) | ('e' << 8) | 't';
static const int CSET_VERSION = 2;

bool duDumpContourSet(struct Moss_RecastContourSet& cset, duFileIO* io)
{
	if (!io)
	{
		printf("duDumpContourSet: input IO is null.\n"); 
		return false;
	}
	if (!io->isWriting())
	{
		printf("duDumpContourSet: input IO not writing.\n"); 
		return false;
	}
	
	io->write(&CSET_MAGIC, sizeof(CSET_MAGIC));
	io->write(&CSET_VERSION, sizeof(CSET_VERSION));

	io->write(&cset.nconts, sizeof(cset.nconts));
	
	io->write(cset.bmin, sizeof(cset.bmin));
	io->write(cset.bmax, sizeof(cset.bmax));
	
	io->write(&cset.cs, sizeof(cset.cs));
	io->write(&cset.ch, sizeof(cset.ch));

	io->write(&cset.width, sizeof(cset.width));
	io->write(&cset.height, sizeof(cset.height));
	io->write(&cset.borderSize, sizeof(cset.borderSize));

	for (int i = 0; i < cset.nconts; ++i)
	{
		const Moss_RecastContour& cont = cset.conts[i];
		io->write(&cont.nverts, sizeof(cont.nverts));
		io->write(&cont.nrverts, sizeof(cont.nrverts));
		io->write(&cont.reg, sizeof(cont.reg));
		io->write(&cont.area, sizeof(cont.area));
		io->write(cont.verts, sizeof(int)*4*cont.nverts);
		io->write(cont.rverts, sizeof(int)*4*cont.nrverts);
	}

	return true;
}

bool duReadContourSet(struct Moss_RecastContourSet& cset, duFileIO* io)
{
	if (!io)
	{
		printf("duReadContourSet: input IO is null.\n"); 
		return false;
	}
	if (!io->isReading())
	{
		printf("duReadContourSet: input IO not reading.\n"); 
		return false;
	}
	
	int magic = 0;
	int version = 0;
	
	io->read(&magic, sizeof(magic));
	io->read(&version, sizeof(version));
	
	if (magic != CSET_MAGIC)
	{
		printf("duReadContourSet: Bad voodoo.\n");
		return false;
	}
	if (version != CSET_VERSION)
	{
		printf("duReadContourSet: Bad version.\n");
		return false;
	}
	
	io->read(&cset.nconts, sizeof(cset.nconts));

	cset.conts = (Moss_RecastContour*)rcAlloc(sizeof(Moss_RecastContour)*cset.nconts, RC_ALLOC_PERM);
	if (!cset.conts)
	{
		printf("duReadContourSet: Could not alloc contours (%d)\n", cset.nconts);
		return false;
	}
	memset(cset.conts, 0, sizeof(Moss_RecastContour)*cset.nconts);
	
	io->read(cset.bmin, sizeof(cset.bmin));
	io->read(cset.bmax, sizeof(cset.bmax));
	
	io->read(&cset.cs, sizeof(cset.cs));
	io->read(&cset.ch, sizeof(cset.ch));
	
	io->read(&cset.width, sizeof(cset.width));
	io->read(&cset.height, sizeof(cset.height));
	io->read(&cset.borderSize, sizeof(cset.borderSize));
	
	for (int i = 0; i < cset.nconts; ++i)
	{
		Moss_RecastContour& cont = cset.conts[i];
		io->read(&cont.nverts, sizeof(cont.nverts));
		io->read(&cont.nrverts, sizeof(cont.nrverts));
		io->read(&cont.reg, sizeof(cont.reg));
		io->read(&cont.area, sizeof(cont.area));

		cont.verts = (int*)rcAlloc(sizeof(int)*4*cont.nverts, RC_ALLOC_PERM);
		if (!cont.verts)
		{
			printf("duReadContourSet: Could not alloc contour verts (%d)\n", cont.nverts);
			return false;
		}
		cont.rverts = (int*)rcAlloc(sizeof(int)*4*cont.nrverts, RC_ALLOC_PERM);
		if (!cont.rverts)
		{
			printf("duReadContourSet: Could not alloc contour rverts (%d)\n", cont.nrverts);
			return false;
		}
		
		io->read(cont.verts, sizeof(int)*4*cont.nverts);
		io->read(cont.rverts, sizeof(int)*4*cont.nrverts);
	}
	
	return true;
}
	

static const int CHF_MAGIC = ('r' << 24) | ('c' << 16) | ('h' << 8) | 'f';
static const int CHF_VERSION = 3;

bool duDumpCompactHeightfield(struct Moss_RecastCompactHeightfield& chf, duFileIO* io)
{
	if (!io)
	{
		printf("duDumpCompactHeightfield: input IO is null.\n"); 
		return false;
	}
	if (!io->isWriting())
	{
		printf("duDumpCompactHeightfield: input IO not writing.\n"); 
		return false;
	}
	
	io->write(&CHF_MAGIC, sizeof(CHF_MAGIC));
	io->write(&CHF_VERSION, sizeof(CHF_VERSION));
	
	io->write(&chf.width, sizeof(chf.width));
	io->write(&chf.height, sizeof(chf.height));
	io->write(&chf.spanCount, sizeof(chf.spanCount));

	io->write(&chf.walkableHeight, sizeof(chf.walkableHeight));
	io->write(&chf.walkableClimb, sizeof(chf.walkableClimb));
	io->write(&chf.borderSize, sizeof(chf.borderSize));

	io->write(&chf.maxDistance, sizeof(chf.maxDistance));
	io->write(&chf.maxRegions, sizeof(chf.maxRegions));

	io->write(chf.bmin, sizeof(chf.bmin));
	io->write(chf.bmax, sizeof(chf.bmax));

	io->write(&chf.cs, sizeof(chf.cs));
	io->write(&chf.ch, sizeof(chf.ch));

	int tmp = 0;
	if (chf.cells) tmp |= 1;
	if (chf.spans) tmp |= 2;
	if (chf.dist) tmp |= 4;
	if (chf.areas) tmp |= 8;

	io->write(&tmp, sizeof(tmp));

	if (chf.cells)
		io->write(chf.cells, sizeof(Moss_RecastCompactCell)*chf.width*chf.height);
	if (chf.spans)
		io->write(chf.spans, sizeof(Moss_RecastCompactSpan)*chf.spanCount);
	if (chf.dist)
		io->write(chf.dist, sizeof(unsigned short)*chf.spanCount);
	if (chf.areas)
		io->write(chf.areas, sizeof(unsigned char)*chf.spanCount);

	return true;
}

bool duReadCompactHeightfield(struct Moss_RecastCompactHeightfield& chf, duFileIO* io)
{
	if (!io)
	{
		printf("duReadCompactHeightfield: input IO is null.\n"); 
		return false;
	}
	if (!io->isReading())
	{
		printf("duReadCompactHeightfield: input IO not reading.\n"); 
		return false;
	}

	int magic = 0;
	int version = 0;
	
	io->read(&magic, sizeof(magic));
	io->read(&version, sizeof(version));
	
	if (magic != CHF_MAGIC)
	{
		printf("duReadCompactHeightfield: Bad voodoo.\n");
		return false;
	}
	if (version != CHF_VERSION)
	{
		printf("duReadCompactHeightfield: Bad version.\n");
		return false;
	}
	
	io->read(&chf.width, sizeof(chf.width));
	io->read(&chf.height, sizeof(chf.height));
	io->read(&chf.spanCount, sizeof(chf.spanCount));
	
	io->read(&chf.walkableHeight, sizeof(chf.walkableHeight));
	io->read(&chf.walkableClimb, sizeof(chf.walkableClimb));
	io->read(&chf.borderSize, sizeof(chf.borderSize));

	io->read(&chf.maxDistance, sizeof(chf.maxDistance));
	io->read(&chf.maxRegions, sizeof(chf.maxRegions));
	
	io->read(chf.bmin, sizeof(chf.bmin));
	io->read(chf.bmax, sizeof(chf.bmax));
	
	io->read(&chf.cs, sizeof(chf.cs));
	io->read(&chf.ch, sizeof(chf.ch));
	
	int tmp = 0;
	io->read(&tmp, sizeof(tmp));
	
	if (tmp & 1)
	{
		chf.cells = (Moss_RecastCompactCell*)rcAlloc(sizeof(Moss_RecastCompactCell)*chf.width*chf.height, RC_ALLOC_PERM);
		if (!chf.cells)
		{
			printf("duReadCompactHeightfield: Could not alloc cells (%d)\n", chf.width*chf.height);
			return false;
		}
		io->read(chf.cells, sizeof(Moss_RecastCompactCell)*chf.width*chf.height);
	}
	if (tmp & 2)
	{
		chf.spans = (Moss_RecastCompactSpan*)rcAlloc(sizeof(Moss_RecastCompactSpan)*chf.spanCount, RC_ALLOC_PERM);
		if (!chf.spans)
		{
			printf("duReadCompactHeightfield: Could not alloc spans (%d)\n", chf.spanCount);
			return false;
		}
		io->read(chf.spans, sizeof(Moss_RecastCompactSpan)*chf.spanCount);
	}
	if (tmp & 4)
	{
		chf.dist = (unsigned short*)rcAlloc(sizeof(unsigned short)*chf.spanCount, RC_ALLOC_PERM);
		if (!chf.dist)
		{
			printf("duReadCompactHeightfield: Could not alloc dist (%d)\n", chf.spanCount);
			return false;
		}
		io->read(chf.dist, sizeof(unsigned short)*chf.spanCount);
	}
	if (tmp & 8)
	{
		chf.areas = (unsigned char*)rcAlloc(sizeof(unsigned char)*chf.spanCount, RC_ALLOC_PERM);
		if (!chf.areas)
		{
			printf("duReadCompactHeightfield: Could not alloc areas (%d)\n", chf.spanCount);
			return false;
		}
		io->read(chf.areas, sizeof(unsigned char)*chf.spanCount);
	}
	
	return true;
}


static void logLine(rcContext& ctx, rcTimerLabel label, const char* name, const float pc)
{
	const int t = ctx.getAccumulatedTime(label);
	if (t < 0) return;
	ctx.log(RC_LOG_PROGRESS, "%s:\t%.2fms\t(%.1f%%)", name, t/1000.0f, t*pc);
}

void duLogBuildTimes(rcContext& ctx, const int totalTimeUsec)
{
	const float pc = 100.0f / totalTimeUsec;
 
	ctx.log(RC_LOG_PROGRESS, "Build Times");
	logLine(ctx, RC_TIMER_RASTERIZE_TRIANGLES,		"- Rasterize", pc);
	logLine(ctx, RC_TIMER_BUILD_COMPACTHEIGHTFIELD,	"- Build Compact", pc);
	logLine(ctx, RC_TIMER_FILTER_BORDER,				"- Filter Border", pc);
	logLine(ctx, RC_TIMER_FILTER_WALKABLE,			"- Filter Walkable", pc);
	logLine(ctx, RC_TIMER_ERODE_AREA,				"- Erode Area", pc);
	logLine(ctx, RC_TIMER_MEDIAN_AREA,				"- Median Area", pc);
	logLine(ctx, RC_TIMER_MARK_BOX_AREA,				"- Mark Box Area", pc);
	logLine(ctx, RC_TIMER_MARK_CONVEXPOLY_AREA,		"- Mark Convex Area", pc);
	logLine(ctx, RC_TIMER_MARK_CYLINDER_AREA,		"- Mark Cylinder Area", pc);
	logLine(ctx, RC_TIMER_BUILD_DISTANCEFIELD,		"- Build Distance Field", pc);
	logLine(ctx, RC_TIMER_BUILD_DISTANCEFIELD_DIST,	"    - Distance", pc);
	logLine(ctx, RC_TIMER_BUILD_DISTANCEFIELD_BLUR,	"    - Blur", pc);
	logLine(ctx, RC_TIMER_BUILD_REGIONS,				"- Build Regions", pc);
	logLine(ctx, RC_TIMER_BUILD_REGIONS_WATERSHED,	"    - Watershed", pc);
	logLine(ctx, RC_TIMER_BUILD_REGIONS_EXPAND,		"      - Expand", pc);
	logLine(ctx, RC_TIMER_BUILD_REGIONS_FLOOD,		"      - Find Basins", pc);
	logLine(ctx, RC_TIMER_BUILD_REGIONS_FILTER,		"    - Filter", pc);
	logLine(ctx, RC_TIMER_BUILD_LAYERS,				"- Build Layers", pc);
	logLine(ctx, RC_TIMER_BUILD_CONTOURS,			"- Build Contours", pc);
	logLine(ctx, RC_TIMER_BUILD_CONTOURS_TRACE,		"    - Trace", pc);
	logLine(ctx, RC_TIMER_BUILD_CONTOURS_SIMPLIFY,	"    - Simplify", pc);
	logLine(ctx, RC_TIMER_BUILD_POLYMESH,			"- Build Polymesh", pc);
	logLine(ctx, RC_TIMER_BUILD_POLYMESHDETAIL,		"- Build Polymesh Detail", pc);
	logLine(ctx, RC_TIMER_MERGE_POLYMESH,			"- Merge Polymeshes", pc);
	logLine(ctx, RC_TIMER_MERGE_POLYMESHDETAIL,		"- Merge Polymesh Details", pc);
	ctx.log(RC_LOG_PROGRESS, "=== TOTAL:\t%.2fms", totalTimeUsec/1000.0f);
}



/*													*/

static float distancePtLine2d(const float* pt, const float* p, const float* q)
{
	float pqx = q[0] - p[0];
	float pqz = q[2] - p[2];
	float dx = pt[0] - p[0];
	float dz = pt[2] - p[2];
	float d = pqx*pqx + pqz*pqz;
	float t = pqx*dx + pqz*dz;
	if (d != 0) t /= d;
	dx = p[0] + t*pqx - pt[0];
	dz = p[2] + t*pqz - pt[2];
	return dx*dx + dz*dz;
}

static void drawPolyBoundaries(duDebugDraw* dd, const Moss_RecastMeshTile* tile,
							   const unsigned int col, const float linew,
							   bool inner)
{
	static const float thr = 0.01f*0.01f;

	dd->begin(DU_DRAW_LINES, linew);

	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		const Moss_RecastPoly* p = &tile->polys[i];
		
		if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION) continue;
		
		const Moss_RecastPolyDetail* pd = &tile->detailMeshes[i];
		
		for (int j = 0, nj = (int)p->vertCount; j < nj; ++j)
		{
			unsigned int c = col;
			if (inner)
			{
				if (p->neis[j] == 0) continue;
				if (p->neis[j] & DT_EXT_LINK)
				{
					bool con = false;
					for (unsigned int k = p->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
					{
						if (tile->links[k].edge == j)
						{
							con = true;
							break;
						}
					}
					if (con)
						c = duRGBA(255,255,255,48);
					else
						c = duRGBA(0,0,0,48);
				}
				else
					c = duRGBA(0,48,64,32);
			}
			else
			{
				if (p->neis[j] != 0) continue;
			}
			
			const float* v0 = &tile->verts[p->verts[j]*3];
			const float* v1 = &tile->verts[p->verts[(j+1) % nj]*3];
			
			// Draw detail mesh edges which align with the actual poly edge.
			// This is really slow.
			for (int k = 0; k < pd->triCount; ++k)
			{
				const unsigned char* t = &tile->detailTris[(pd->triBase+k)*4];
				const float* tv[3];
				for (int m = 0; m < 3; ++m)
				{
					if (t[m] < p->vertCount)
						tv[m] = &tile->verts[p->verts[t[m]]*3];
					else
						tv[m] = &tile->detailVerts[(pd->vertBase+(t[m]-p->vertCount))*3];
				}
				for (int m = 0, n = 2; m < 3; n=m++)
				{
					if ((dtGetDetailTriEdgeFlags(t[3], n) & DT_DETAIL_EDGE_BOUNDARY) == 0)
						continue;

					if (distancePtLine2d(tv[n],v0,v1) < thr &&
						distancePtLine2d(tv[m],v0,v1) < thr)
					{
						dd->vertex(tv[n], c);
						dd->vertex(tv[m], c);
					}
				}
			}
		}
	}
	dd->end();
}

static void drawMeshTile(duDebugDraw* dd, const Moss_RecastNavMesh& mesh, const Moss_RecastNavMeshQuery* query,
						 const Moss_RecastMeshTile* tile, unsigned char flags)
{
	dtPolyRef base = mesh.getPolyRefBase(tile);

	int tileNum = mesh.decodePolyIdTile(base);
	const unsigned int tileColor = duIntToCol(tileNum, 128);
	
	dd->depthMask(false);

	dd->begin(DU_DRAW_TRIS);
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		const Moss_RecastPoly* p = &tile->polys[i];
		if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)	// Skip off-mesh links.
			continue;
			
		const Moss_RecastPolyDetail* pd = &tile->detailMeshes[i];

		unsigned int col;
		if (query && query->isInClosedList(base | (dtPolyRef)i))
			col = duRGBA(255,196,0,64);
		else
		{
			if (flags & DU_DRAWNAVMESH_COLOR_TILES)
				col = tileColor;
			else
				col = duTransCol(dd->areaToCol(p->getArea()), 64);
		}
		
		for (int j = 0; j < pd->triCount; ++j)
		{
			const unsigned char* t = &tile->detailTris[(pd->triBase+j)*4];
			for (int k = 0; k < 3; ++k)
			{
				if (t[k] < p->vertCount)
					dd->vertex(&tile->verts[p->verts[t[k]]*3], col);
				else
					dd->vertex(&tile->detailVerts[(pd->vertBase+t[k]-p->vertCount)*3], col);
			}
		}
	}
	dd->end();
	
	// Draw inter poly boundaries
	drawPolyBoundaries(dd, tile, duRGBA(0,48,64,32), 1.5f, true);
	
	// Draw outer poly boundaries
	drawPolyBoundaries(dd, tile, duRGBA(0,48,64,220), 2.5f, false);

	if (flags & DU_DRAWNAVMESH_OFFMESHCONS)
	{
		dd->begin(DU_DRAW_LINES, 2.0f);
		for (int i = 0; i < tile->header->polyCount; ++i)
		{
			const Moss_RecastPoly* p = &tile->polys[i];
			if (p->getType() != DT_POLYTYPE_OFFMESH_CONNECTION)	// Skip regular polys.
				continue;
			
			unsigned int col, col2;
			if (query && query->isInClosedList(base | (dtPolyRef)i))
				col = duRGBA(255,196,0,220);
			else
				col = duDarkenCol(duTransCol(dd->areaToCol(p->getArea()), 220));

			const Moss_RecastOffMeshConnection* con = &tile->offMeshCons[i - tile->header->offMeshBase];
			const float* va = &tile->verts[p->verts[0]*3];
			const float* vb = &tile->verts[p->verts[1]*3];

			// Check to see if start and end end-points have links.
			bool startSet = false;
			bool endSet = false;
			for (unsigned int k = p->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
			{
				if (tile->links[k].edge == 0)
					startSet = true;
				if (tile->links[k].edge == 1)
					endSet = true;
			}
			
			// End points and their on-mesh locations.
			dd->vertex(va[0],va[1],va[2], col);
			dd->vertex(con->pos[0],con->pos[1],con->pos[2], col);
			col2 = startSet ? col : duRGBA(220,32,16,196);
			duAppendCircle(dd, con->pos[0],con->pos[1]+0.1f,con->pos[2], con->rad, col2);

			dd->vertex(vb[0],vb[1],vb[2], col);
			dd->vertex(con->pos[3],con->pos[4],con->pos[5], col);
			col2 = endSet ? col : duRGBA(220,32,16,196);
			duAppendCircle(dd, con->pos[3],con->pos[4]+0.1f,con->pos[5], con->rad, col2);
			
			// End point vertices.
			dd->vertex(con->pos[0],con->pos[1],con->pos[2], duRGBA(0,48,64,196));
			dd->vertex(con->pos[0],con->pos[1]+0.2f,con->pos[2], duRGBA(0,48,64,196));
			
			dd->vertex(con->pos[3],con->pos[4],con->pos[5], duRGBA(0,48,64,196));
			dd->vertex(con->pos[3],con->pos[4]+0.2f,con->pos[5], duRGBA(0,48,64,196));
			
			// Connection arc.
			duAppendArc(dd, con->pos[0],con->pos[1],con->pos[2], con->pos[3],con->pos[4],con->pos[5], 0.25f,
						(con->flags & 1) ? 0.6f : 0, 0.6f, col);
		}
		dd->end();
	}
	
	const unsigned int vcol = duRGBA(0,0,0,196);
	dd->begin(DU_DRAW_POINTS, 3.0f);
	for (int i = 0; i < tile->header->vertCount; ++i)
	{
		const float* v = &tile->verts[i*3];
		dd->vertex(v[0], v[1], v[2], vcol);
	}
	dd->end();

	dd->depthMask(true);
}

void duDebugDrawNavMesh(duDebugDraw* dd, const Moss_RecastNavMesh& mesh, unsigned char flags)
{
	if (!dd) return;
	
	for (int i = 0; i < mesh.getMaxTiles(); ++i)
	{
		const Moss_RecastMeshTile* tile = mesh.getTile(i);
		if (!tile->header) continue;
		drawMeshTile(dd, mesh, 0, tile, flags);
	}
}

void duDebugDrawNavMeshWithClosedList(struct duDebugDraw* dd, const Moss_RecastNavMesh& mesh, const Moss_RecastNavMeshQuery& query, unsigned char flags)
{
	if (!dd) return;

	const Moss_RecastNavMeshQuery* q = (flags & DU_DRAWNAVMESH_CLOSEDLIST) ? &query : 0;
	
	for (int i = 0; i < mesh.getMaxTiles(); ++i)
	{
		const Moss_RecastMeshTile* tile = mesh.getTile(i);
		if (!tile->header) continue;
		drawMeshTile(dd, mesh, q, tile, flags);
	}
}

void duDebugDrawNavMeshNodes(struct duDebugDraw* dd, const Moss_RecastNavMeshQuery& query)
{
	if (!dd) return;
	
	const Moss_RecastNodePool* pool = query.getNodePool();
	if (pool)
	{
		const float off = 0.5f;
		dd->begin(DU_DRAW_POINTS, 4.0f);
		for (int i = 0; i < pool->getHashSize(); ++i)
		{
			for (dtNodeIndex j = pool->getFirst(i); j != DT_NULL_IDX; j = pool->getNext(j))
			{
				const Moss_RecastNode* node = pool->getNodeAtIdx(j+1);
				if (!node) continue;
				dd->vertex(node->pos[0],node->pos[1]+off,node->pos[2], duRGBA(255,192,0,255));
			}
		}
		dd->end();
		
		dd->begin(DU_DRAW_LINES, 2.0f);
		for (int i = 0; i < pool->getHashSize(); ++i)
		{
			for (dtNodeIndex j = pool->getFirst(i); j != DT_NULL_IDX; j = pool->getNext(j))
			{
				const Moss_RecastNode* node = pool->getNodeAtIdx(j+1);
				if (!node) continue;
				if (!node->pidx) continue;
				const Moss_RecastNode* parent = pool->getNodeAtIdx(node->pidx);
				if (!parent) continue;
				dd->vertex(node->pos[0],node->pos[1]+off,node->pos[2], duRGBA(255,192,0,128));
				dd->vertex(parent->pos[0],parent->pos[1]+off,parent->pos[2], duRGBA(255,192,0,128));
			}
		}
		dd->end();
	}
}


static void drawMeshTileBVTree(duDebugDraw* dd, const Moss_RecastMeshTile* tile)
{
	// Draw BV nodes.
	const float cs = 1.0f / tile->header->bvQuantFactor;
	dd->begin(DU_DRAW_LINES, 1.0f);
	for (int i = 0; i < tile->header->bvNodeCount; ++i)
	{
		const Moss_RecastBVNode* n = &tile->bvTree[i];
		if (n->i < 0) // Leaf indices are positive.
			continue;
		duAppendBoxWire(dd, tile->header->bmin[0] + n->bmin[0]*cs,
						tile->header->bmin[1] + n->bmin[1]*cs,
						tile->header->bmin[2] + n->bmin[2]*cs,
						tile->header->bmin[0] + n->bmax[0]*cs,
						tile->header->bmin[1] + n->bmax[1]*cs,
						tile->header->bmin[2] + n->bmax[2]*cs,
						duRGBA(255,255,255,128));
	}
	dd->end();
}

void duDebugDrawNavMeshBVTree(duDebugDraw* dd, const Moss_RecastNavMesh& mesh)
{
	if (!dd) return;
	
	for (int i = 0; i < mesh.getMaxTiles(); ++i)
	{
		const Moss_RecastMeshTile* tile = mesh.getTile(i);
		if (!tile->header) continue;
		drawMeshTileBVTree(dd, tile);
	}
}

static void drawMeshTilePortal(duDebugDraw* dd, const Moss_RecastMeshTile* tile)
{
	// Draw portals
	const float padx = 0.04f;
	const float pady = tile->header->walkableClimb;

	dd->begin(DU_DRAW_LINES, 2.0f);

	for (int side = 0; side < 8; ++side)
	{
		unsigned short m = DT_EXT_LINK | (unsigned short)side;
		
		for (int i = 0; i < tile->header->polyCount; ++i)
		{
			Moss_RecastPoly* poly = &tile->polys[i];
			
			// Create new links.
			const int nv = poly->vertCount;
			for (int j = 0; j < nv; ++j)
			{
				// Skip edges which do not point to the right side.
				if (poly->neis[j] != m)
					continue;
				
				// Create new links
				const float* va = &tile->verts[poly->verts[j]*3];
				const float* vb = &tile->verts[poly->verts[(j+1) % nv]*3];
				
				if (side == 0 || side == 4)
				{
					unsigned int col = side == 0 ? duRGBA(128,0,0,128) : duRGBA(128,0,128,128);

					const float x = va[0] + ((side == 0) ? -padx : padx);
					
					dd->vertex(x,va[1]-pady,va[2], col);
					dd->vertex(x,va[1]+pady,va[2], col);

					dd->vertex(x,va[1]+pady,va[2], col);
					dd->vertex(x,vb[1]+pady,vb[2], col);

					dd->vertex(x,vb[1]+pady,vb[2], col);
					dd->vertex(x,vb[1]-pady,vb[2], col);

					dd->vertex(x,vb[1]-pady,vb[2], col);
					dd->vertex(x,va[1]-pady,va[2], col);
				}
				else if (side == 2 || side == 6)
				{
					unsigned int col = side == 2 ? duRGBA(0,128,0,128) : duRGBA(0,128,128,128);

					const float z = va[2] + ((side == 2) ? -padx : padx);
					
					dd->vertex(va[0],va[1]-pady,z, col);
					dd->vertex(va[0],va[1]+pady,z, col);
					
					dd->vertex(va[0],va[1]+pady,z, col);
					dd->vertex(vb[0],vb[1]+pady,z, col);
					
					dd->vertex(vb[0],vb[1]+pady,z, col);
					dd->vertex(vb[0],vb[1]-pady,z, col);
					
					dd->vertex(vb[0],vb[1]-pady,z, col);
					dd->vertex(va[0],va[1]-pady,z, col);
				}

			}
		}
	}
	
	dd->end();
}

void duDebugDrawNavMeshPortals(duDebugDraw* dd, const Moss_RecastNavMesh& mesh)
{
	if (!dd) return;
	
	for (int i = 0; i < mesh.getMaxTiles(); ++i)
	{
		const Moss_RecastMeshTile* tile = mesh.getTile(i);
		if (!tile->header) continue;
		drawMeshTilePortal(dd, tile);
	}
}

void duDebugDrawNavMeshPolysWithFlags(struct duDebugDraw* dd, const Moss_RecastNavMesh& mesh,
									  const unsigned short polyFlags, const unsigned int col)
{
	if (!dd) return;
	
	for (int i = 0; i < mesh.getMaxTiles(); ++i)
	{
		const Moss_RecastMeshTile* tile = mesh.getTile(i);
		if (!tile->header) continue;
		dtPolyRef base = mesh.getPolyRefBase(tile);

		for (int j = 0; j < tile->header->polyCount; ++j)
		{
			const Moss_RecastPoly* p = &tile->polys[j];
			if ((p->flags & polyFlags) == 0) continue;
			duDebugDrawNavMeshPoly(dd, mesh, base|(dtPolyRef)j, col);
		}
	}
}

void duDebugDrawNavMeshPoly(duDebugDraw* dd, const Moss_RecastNavMesh& mesh, dtPolyRef ref, const unsigned int col)
{
	if (!dd) return;
	
	const Moss_RecastMeshTile* tile = 0;
	const Moss_RecastPoly* poly = 0;
	if (dtStatusFailed(mesh.getTileAndPolyByRef(ref, &tile, &poly)))
		return;
	
	dd->depthMask(false);
	
	const unsigned int c = duTransCol(col, 64);
	const unsigned int ip = (unsigned int)(poly - tile->polys);

	if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
	{
		Moss_RecastOffMeshConnection* con = &tile->offMeshCons[ip - tile->header->offMeshBase];

		dd->begin(DU_DRAW_LINES, 2.0f);

		// Connection arc.
		duAppendArc(dd, con->pos[0],con->pos[1],con->pos[2], con->pos[3],con->pos[4],con->pos[5], 0.25f,
					(con->flags & 1) ? 0.6f : 0.0f, 0.6f, c);
		
		dd->end();
	}
	else
	{
		const Moss_RecastPolyDetail* pd = &tile->detailMeshes[ip];

		dd->begin(DU_DRAW_TRIS);
		for (int i = 0; i < pd->triCount; ++i)
		{
			const unsigned char* t = &tile->detailTris[(pd->triBase+i)*4];
			for (int j = 0; j < 3; ++j)
			{
				if (t[j] < poly->vertCount)
					dd->vertex(&tile->verts[poly->verts[t[j]]*3], c);
				else
					dd->vertex(&tile->detailVerts[(pd->vertBase+t[j]-poly->vertCount)*3], c);
			}
		}
		dd->end();
	}
	
	dd->depthMask(true);

}

static void debugDrawTileCachePortals(struct duDebugDraw* dd, const dtTileCacheLayer& layer, const float cs, const float ch)
{
	const int w = (int)layer.header->width;
	const int h = (int)layer.header->height;
	const float* bmin = layer.header->bmin;

	// Portals
	unsigned int pcol = duRGBA(255,255,255,255);
	
	const int segs[4*4] = {0,0,0,1, 0,1,1,1, 1,1,1,0, 1,0,0,0};
	
	// Layer portals
	dd->begin(DU_DRAW_LINES, 2.0f);
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const int idx = x+y*w;
			const int lh = (int)layer.heights[idx];
			if (lh == 0xff) continue;
			
			for (int dir = 0; dir < 4; ++dir)
			{
				if (layer.cons[idx] & (1<<(dir+4)))
				{
					const int* seg = &segs[dir*4];
					const float ax = bmin[0] + (x+seg[0])*cs;
					const float ay = bmin[1] + (lh+2)*ch;
					const float az = bmin[2] + (y+seg[1])*cs;
					const float bx = bmin[0] + (x+seg[2])*cs;
					const float by = bmin[1] + (lh+2)*ch;
					const float bz = bmin[2] + (y+seg[3])*cs;
					dd->vertex(ax, ay, az, pcol);
					dd->vertex(bx, by, bz, pcol);
				}
			}
		}
	}
	dd->end();
}

void duDebugDrawTileCacheLayerAreas(struct duDebugDraw* dd, const dtTileCacheLayer& layer, const float cs, const float ch)
{
	const int w = (int)layer.header->width;
	const int h = (int)layer.header->height;
	const float* bmin = layer.header->bmin;
	const float* bmax = layer.header->bmax;
	const int idx = layer.header->tlayer;
	
	unsigned int color = duIntToCol(idx+1, 255);
	
	// Layer bounds
	float lbmin[3], lbmax[3];
	lbmin[0] = bmin[0] + layer.header->minx*cs;
	lbmin[1] = bmin[1];
	lbmin[2] = bmin[2] + layer.header->miny*cs;
	lbmax[0] = bmin[0] + (layer.header->maxx+1)*cs;
	lbmax[1] = bmax[1];
	lbmax[2] = bmin[2] + (layer.header->maxy+1)*cs;
	duDebugDrawBoxWire(dd, lbmin[0],lbmin[1],lbmin[2], lbmax[0],lbmax[1],lbmax[2], duTransCol(color,128), 2.0f);
	
	// Layer height
	dd->begin(DU_DRAW_QUADS);
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const int lidx = x+y*w;
			const int lh = (int)layer.heights[lidx];
			if (lh == 0xff) continue;

			const unsigned char area = layer.areas[lidx];
			unsigned int col;
			if (area == 63)
				col = duLerpCol(color, duRGBA(0,192,255,64), 32);
			else if (area == 0)
				col = duLerpCol(color, duRGBA(0,0,0,64), 32);
			else
				col = duLerpCol(color, dd->areaToCol(area), 32);
			
			const float fx = bmin[0] + x*cs;
			const float fy = bmin[1] + (lh+1)*ch;
			const float fz = bmin[2] + y*cs;
			
			dd->vertex(fx, fy, fz, col);
			dd->vertex(fx, fy, fz+cs, col);
			dd->vertex(fx+cs, fy, fz+cs, col);
			dd->vertex(fx+cs, fy, fz, col);
		}
	}
	dd->end();
	
	debugDrawTileCachePortals(dd, layer, cs, ch);
}

void duDebugDrawTileCacheLayerRegions(struct duDebugDraw* dd, const dtTileCacheLayer& layer, const float cs, const float ch)
{
	const int w = (int)layer.header->width;
	const int h = (int)layer.header->height;
	const float* bmin = layer.header->bmin;
	const float* bmax = layer.header->bmax;
	const int idx = layer.header->tlayer;
	
	unsigned int color = duIntToCol(idx+1, 255);
	
	// Layer bounds
	float lbmin[3], lbmax[3];
	lbmin[0] = bmin[0] + layer.header->minx*cs;
	lbmin[1] = bmin[1];
	lbmin[2] = bmin[2] + layer.header->miny*cs;
	lbmax[0] = bmin[0] + (layer.header->maxx+1)*cs;
	lbmax[1] = bmax[1];
	lbmax[2] = bmin[2] + (layer.header->maxy+1)*cs;
	duDebugDrawBoxWire(dd, lbmin[0],lbmin[1],lbmin[2], lbmax[0],lbmax[1],lbmax[2], duTransCol(color,128), 2.0f);
	
	// Layer height
	dd->begin(DU_DRAW_QUADS);
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const int lidx = x+y*w;
			const int lh = (int)layer.heights[lidx];
			if (lh == 0xff) continue;
			const unsigned char reg = layer.regs[lidx];
			
			unsigned int col = duLerpCol(color, duIntToCol(reg, 255), 192);
			
			const float fx = bmin[0] + x*cs;
			const float fy = bmin[1] + (lh+1)*ch;
			const float fz = bmin[2] + y*cs;
			
			dd->vertex(fx, fy, fz, col);
			dd->vertex(fx, fy, fz+cs, col);
			dd->vertex(fx+cs, fy, fz+cs, col);
			dd->vertex(fx+cs, fy, fz, col);
		}
	}
	dd->end();
	
	debugDrawTileCachePortals(dd, layer, cs, ch);
}




/*struct dtTileCacheContour
{
	int nverts;
	unsigned char* verts;
	unsigned char reg;
	unsigned char area;
};

struct dtTileCacheContourSet
{
	int nconts;
	dtTileCacheContour* conts;
};*/

void duDebugDrawTileCacheContours(duDebugDraw* dd, const struct dtTileCacheContourSet& lcset,
								  const float* orig, const float cs, const float ch)
{
	if (!dd) return;
	
	const unsigned char a = 255;// (unsigned char)(alpha*255.0f);
	
	const int offs[2*4] = {-1,0, 0,1, 1,0, 0,-1};
	
	dd->begin(DU_DRAW_LINES, 2.0f);
	
	for (int i = 0; i < lcset.nconts; ++i)
	{
		const dtTileCacheContour& c = lcset.conts[i];
		unsigned int color = 0;
		
		color = duIntToCol(i, a);
		
		for (int j = 0; j < c.nverts; ++j)
		{
			const int k = (j+1) % c.nverts;
			const unsigned char* va = &c.verts[j*4];
			const unsigned char* vb = &c.verts[k*4];
			const float ax = orig[0] + va[0]*cs;
			const float ay = orig[1] + (va[1]+1+(i&1))*ch;
			const float az = orig[2] + va[2]*cs;
			const float bx = orig[0] + vb[0]*cs;
			const float by = orig[1] + (vb[1]+1+(i&1))*ch;
			const float bz = orig[2] + vb[2]*cs;
			unsigned int col = color;
			if ((va[3] & 0xf) != 0xf)
			{
				// Portal segment
				col = duRGBA(255,255,255,128);
				int d = va[3] & 0xf;
				
				const float cx = (ax+bx)*0.5f;
				const float cy = (ay+by)*0.5f;
				const float cz = (az+bz)*0.5f;
				
				const float dx = cx + offs[d*2+0]*2*cs;
				const float dy = cy;
				const float dz = cz + offs[d*2+1]*2*cs;
				
				dd->vertex(cx,cy,cz,duRGBA(255,0,0,255));
				dd->vertex(dx,dy,dz,duRGBA(255,0,0,255));
			}
			
			duAppendArrow(dd, ax,ay,az, bx,by,bz, 0.0f, cs*0.5f, col);
		}
	}
	dd->end();
	
	dd->begin(DU_DRAW_POINTS, 4.0f);	
	
	for (int i = 0; i < lcset.nconts; ++i)
	{
		const dtTileCacheContour& c = lcset.conts[i];
		unsigned int color = 0;
		
		for (int j = 0; j < c.nverts; ++j)
		{
			const unsigned char* va = &c.verts[j*4];
			
			color = duDarkenCol(duIntToCol(i, a));
			if (va[3] & 0x80)
			{
				// Border vertex
				color = duRGBA(255,0,0,255);
			}
			
			float fx = orig[0] + va[0]*cs;
			float fy = orig[1] + (va[1]+1+(i&1))*ch;
			float fz = orig[2] + va[2]*cs;
			dd->vertex(fx,fy,fz, color);
		}
	}
	dd->end();
}

void duDebugDrawTileCachePolyMesh(duDebugDraw* dd, const struct dtTileCachePolyMesh& lmesh,
								  const float* orig, const float cs, const float ch)
{
	if (!dd) return;
	
	const int nvp = lmesh.nvp;
	
	const int offs[2*4] = {-1,0, 0,1, 1,0, 0,-1};
	
	dd->begin(DU_DRAW_TRIS);
	
	for (int i = 0; i < lmesh.npolys; ++i)
	{
		const unsigned short* p = &lmesh.polys[i*nvp*2];
		const unsigned char area = lmesh.areas[i];
		
		unsigned int color;
		if (area == DT_TILECACHE_WALKABLE_AREA)
			color = duRGBA(0,192,255,64);
		else if (area == DT_TILECACHE_NULL_AREA)
			color = duRGBA(0,0,0,64);
		else
			color = dd->areaToCol(area);
		
		unsigned short vi[3];
		for (int j = 2; j < nvp; ++j)
		{
			if (p[j] == DT_TILECACHE_NULL_IDX) break;
			vi[0] = p[0];
			vi[1] = p[j-1];
			vi[2] = p[j];
			for (int k = 0; k < 3; ++k)
			{
				const unsigned short* v = &lmesh.verts[vi[k]*3];
				const float x = orig[0] + v[0]*cs;
				const float y = orig[1] + (v[1]+1)*ch;
				const float z = orig[2] + v[2]*cs;
				dd->vertex(x,y,z, color);
			}
		}
	}
	dd->end();
	
	// Draw neighbours edges
	const unsigned int coln = duRGBA(0,48,64,32);
	dd->begin(DU_DRAW_LINES, 1.5f);
	for (int i = 0; i < lmesh.npolys; ++i)
	{
		const unsigned short* p = &lmesh.polys[i*nvp*2];
		for (int j = 0; j < nvp; ++j)
		{
			if (p[j] == DT_TILECACHE_NULL_IDX) break;
			if (p[nvp+j] & 0x8000) continue;
			const int nj = (j+1 >= nvp || p[j+1] == DT_TILECACHE_NULL_IDX) ? 0 : j+1; 
			int vi[2] = {p[j], p[nj]};
			
			for (int k = 0; k < 2; ++k)
			{
				const unsigned short* v = &lmesh.verts[vi[k]*3];
				const float x = orig[0] + v[0]*cs;
				const float y = orig[1] + (v[1]+1)*ch + 0.1f;
				const float z = orig[2] + v[2]*cs;
				dd->vertex(x, y, z, coln);
			}
		}
	}
	dd->end();
	
	// Draw boundary edges
	const unsigned int colb = duRGBA(0,48,64,220);
	dd->begin(DU_DRAW_LINES, 2.5f);
	for (int i = 0; i < lmesh.npolys; ++i)
	{
		const unsigned short* p = &lmesh.polys[i*nvp*2];
		for (int j = 0; j < nvp; ++j)
		{
			if (p[j] == DT_TILECACHE_NULL_IDX) break;
			if ((p[nvp+j] & 0x8000) == 0) continue;
			const int nj = (j+1 >= nvp || p[j+1] == DT_TILECACHE_NULL_IDX) ? 0 : j+1; 
			int vi[2] = {p[j], p[nj]};
			
			unsigned int col = colb;
			if ((p[nvp+j] & 0xf) != 0xf)
			{
				const unsigned short* va = &lmesh.verts[vi[0]*3];
				const unsigned short* vb = &lmesh.verts[vi[1]*3];
				
				const float ax = orig[0] + va[0]*cs;
				const float ay = orig[1] + (va[1]+1+(i&1))*ch;
				const float az = orig[2] + va[2]*cs;
				const float bx = orig[0] + vb[0]*cs;
				const float by = orig[1] + (vb[1]+1+(i&1))*ch;
				const float bz = orig[2] + vb[2]*cs;
				
				const float cx = (ax+bx)*0.5f;
				const float cy = (ay+by)*0.5f;
				const float cz = (az+bz)*0.5f;
				
				int d = p[nvp+j] & 0xf;
				
				const float dx = cx + offs[d*2+0]*2*cs;
				const float dy = cy;
				const float dz = cz + offs[d*2+1]*2*cs;
				
				dd->vertex(cx,cy,cz,duRGBA(255,0,0,255));
				dd->vertex(dx,dy,dz,duRGBA(255,0,0,255));
				
				col = duRGBA(255,255,255,128);
			}
			
			for (int k = 0; k < 2; ++k)
			{
				const unsigned short* v = &lmesh.verts[vi[k]*3];
				const float x = orig[0] + v[0]*cs;
				const float y = orig[1] + (v[1]+1)*ch + 0.1f;
				const float z = orig[2] + v[2]*cs;
				dd->vertex(x, y, z, col);
			}
		}
	}
	dd->end();
	
	dd->begin(DU_DRAW_POINTS, 3.0f);
	const unsigned int colv = duRGBA(0,0,0,220);
	for (int i = 0; i < lmesh.nverts; ++i)
	{
		const unsigned short* v = &lmesh.verts[i*3];
		const float x = orig[0] + v[0]*cs;
		const float y = orig[1] + (v[1]+1)*ch + 0.1f;
		const float z = orig[2] + v[2]*cs;
		dd->vertex(x,y,z, colv);
	}
	dd->end();
}




/*													*/

duDebugDraw::~duDebugDraw()
{
	// Empty
}

unsigned int duDebugDraw::areaToCol(unsigned int area)
{
	if (area == 0)
	{
		// Treat zero area type as default.
		return duRGBA(0, 192, 255, 255);
	}
	else
	{
		return duIntToCol(area, 255);
	}
}

inline int bit(int a, int b)
{
	return (a & (1 << b)) >> b;
}

unsigned int duIntToCol(int i, int a)
{
	int	r = bit(i, 1) + bit(i, 3) * 2 + 1;
	int	g = bit(i, 2) + bit(i, 4) * 2 + 1;
	int	b = bit(i, 0) + bit(i, 5) * 2 + 1;
	return duRGBA(r*63,g*63,b*63,a);
}

void duIntToCol(int i, float* col)
{
	int	r = bit(i, 0) + bit(i, 3) * 2 + 1;
	int	g = bit(i, 1) + bit(i, 4) * 2 + 1;
	int	b = bit(i, 2) + bit(i, 5) * 2 + 1;
	col[0] = 1 - r*63.0f/255.0f;
	col[1] = 1 - g*63.0f/255.0f;
	col[2] = 1 - b*63.0f/255.0f;
}

void duCalcBoxColors(unsigned int* colors, unsigned int colTop, unsigned int colSide)
{
	if (!colors) return;
	
	colors[0] = duMultCol(colTop, 250);
	colors[1] = duMultCol(colSide, 140);
	colors[2] = duMultCol(colSide, 165);
	colors[3] = duMultCol(colSide, 217);
	colors[4] = duMultCol(colSide, 165);
	colors[5] = duMultCol(colSide, 217);
}

void duDebugDrawCylinderWire(struct duDebugDraw* dd, float minx, float miny, float minz,
							 float maxx, float maxy, float maxz, unsigned int col, const float lineWidth)
{
	if (!dd) return;
	
	dd->begin(DU_DRAW_LINES, lineWidth);
	duAppendCylinderWire(dd, minx,miny,minz, maxx,maxy,maxz, col);
	dd->end();
}

void duDebugDrawBoxWire(struct duDebugDraw* dd, float minx, float miny, float minz,
						float maxx, float maxy, float maxz, unsigned int col, const float lineWidth)
{
	if (!dd) return;
	
	dd->begin(DU_DRAW_LINES, lineWidth);
	duAppendBoxWire(dd, minx,miny,minz, maxx,maxy,maxz, col);
	dd->end();
}

void duDebugDrawArc(struct duDebugDraw* dd, const float x0, const float y0, const float z0,
					const float x1, const float y1, const float z1, const float h,
					const float as0, const float as1, unsigned int col, const float lineWidth)
{
	if (!dd) return;
	
	dd->begin(DU_DRAW_LINES, lineWidth);
	duAppendArc(dd, x0,y0,z0, x1,y1,z1, h, as0, as1, col);
	dd->end();
}

void duDebugDrawArrow(struct duDebugDraw* dd, const float x0, const float y0, const float z0,
					  const float x1, const float y1, const float z1,
					  const float as0, const float as1, unsigned int col, const float lineWidth)
{
	if (!dd) return;
	
	dd->begin(DU_DRAW_LINES, lineWidth);
	duAppendArrow(dd, x0,y0,z0, x1,y1,z1, as0, as1, col);
	dd->end();
}

void duDebugDrawCircle(struct duDebugDraw* dd, const float x, const float y, const float z,
					   const float r, unsigned int col, const float lineWidth)
{
	if (!dd) return;
	
	dd->begin(DU_DRAW_LINES, lineWidth);
	duAppendCircle(dd, x,y,z, r, col);
	dd->end();
}

void duDebugDrawCross(struct duDebugDraw* dd, const float x, const float y, const float z,
					  const float size, unsigned int col, const float lineWidth)
{
	if (!dd) return;
	
	dd->begin(DU_DRAW_LINES, lineWidth);
	duAppendCross(dd, x,y,z, size, col);
	dd->end();
}

void duDebugDrawBox(struct duDebugDraw* dd, float minx, float miny, float minz,
					float maxx, float maxy, float maxz, const unsigned int* fcol)
{
	if (!dd) return;
	
	dd->begin(DU_DRAW_QUADS);
	duAppendBox(dd, minx,miny,minz, maxx,maxy,maxz, fcol);
	dd->end();
}

void duDebugDrawCylinder(struct duDebugDraw* dd, float minx, float miny, float minz,
						 float maxx, float maxy, float maxz, unsigned int col)
{
	if (!dd) return;
	
	dd->begin(DU_DRAW_TRIS);
	duAppendCylinder(dd, minx,miny,minz, maxx,maxy,maxz, col);
	dd->end();
}

void duDebugDrawGridXZ(struct duDebugDraw* dd, const float ox, const float oy, const float oz,
					   const int w, const int h, const float size,
					   const unsigned int col, const float lineWidth)
{
	if (!dd) return;

	dd->begin(DU_DRAW_LINES, lineWidth);
	for (int i = 0; i <= h; ++i)
	{
		dd->vertex(ox,oy,oz+i*size, col);
		dd->vertex(ox+w*size,oy,oz+i*size, col);
	}
	for (int i = 0; i <= w; ++i)
	{
		dd->vertex(ox+i*size,oy,oz, col);
		dd->vertex(ox+i*size,oy,oz+h*size, col);
	}
	dd->end();
}
		 

void duAppendCylinderWire(struct duDebugDraw* dd, float minx, float miny, float minz,
						  float maxx, float maxy, float maxz, unsigned int col)
{
	if (!dd) return;

	static const int NUM_SEG = 16;
	static float dir[NUM_SEG*2];
	static bool init = false;
	if (!init)
	{
		init = true;
		for (int i = 0; i < NUM_SEG; ++i)
		{
			const float a = (float)i/(float)NUM_SEG*DU_PI*2;
			dir[i*2] = cosf(a);
			dir[i*2+1] = sinf(a);
		}
	}
	
	const float cx = (maxx + minx)/2;
	const float cz = (maxz + minz)/2;
	const float rx = (maxx - minx)/2;
	const float rz = (maxz - minz)/2;
	
	for (int i = 0, j = NUM_SEG-1; i < NUM_SEG; j = i++)
	{
		dd->vertex(cx+dir[j*2+0]*rx, miny, cz+dir[j*2+1]*rz, col);
		dd->vertex(cx+dir[i*2+0]*rx, miny, cz+dir[i*2+1]*rz, col);
		dd->vertex(cx+dir[j*2+0]*rx, maxy, cz+dir[j*2+1]*rz, col);
		dd->vertex(cx+dir[i*2+0]*rx, maxy, cz+dir[i*2+1]*rz, col);
	}
	for (int i = 0; i < NUM_SEG; i += NUM_SEG/4)
	{
		dd->vertex(cx+dir[i*2+0]*rx, miny, cz+dir[i*2+1]*rz, col);
		dd->vertex(cx+dir[i*2+0]*rx, maxy, cz+dir[i*2+1]*rz, col);
	}
}

void duAppendBoxWire(struct duDebugDraw* dd, float minx, float miny, float minz,
					 float maxx, float maxy, float maxz, unsigned int col)
{
	if (!dd) return;
	// Top
	dd->vertex(minx, miny, minz, col);
	dd->vertex(maxx, miny, minz, col);
	dd->vertex(maxx, miny, minz, col);
	dd->vertex(maxx, miny, maxz, col);
	dd->vertex(maxx, miny, maxz, col);
	dd->vertex(minx, miny, maxz, col);
	dd->vertex(minx, miny, maxz, col);
	dd->vertex(minx, miny, minz, col);
	
	// bottom
	dd->vertex(minx, maxy, minz, col);
	dd->vertex(maxx, maxy, minz, col);
	dd->vertex(maxx, maxy, minz, col);
	dd->vertex(maxx, maxy, maxz, col);
	dd->vertex(maxx, maxy, maxz, col);
	dd->vertex(minx, maxy, maxz, col);
	dd->vertex(minx, maxy, maxz, col);
	dd->vertex(minx, maxy, minz, col);
	
	// Sides
	dd->vertex(minx, miny, minz, col);
	dd->vertex(minx, maxy, minz, col);
	dd->vertex(maxx, miny, minz, col);
	dd->vertex(maxx, maxy, minz, col);
	dd->vertex(maxx, miny, maxz, col);
	dd->vertex(maxx, maxy, maxz, col);
	dd->vertex(minx, miny, maxz, col);
	dd->vertex(minx, maxy, maxz, col);
}

void duAppendBoxPoints(struct duDebugDraw* dd, float minx, float miny, float minz,
					   float maxx, float maxy, float maxz, unsigned int col)
{
	if (!dd) return;
	// Top
	dd->vertex(minx, miny, minz, col);
	dd->vertex(maxx, miny, minz, col);
	dd->vertex(maxx, miny, minz, col);
	dd->vertex(maxx, miny, maxz, col);
	dd->vertex(maxx, miny, maxz, col);
	dd->vertex(minx, miny, maxz, col);
	dd->vertex(minx, miny, maxz, col);
	dd->vertex(minx, miny, minz, col);
	
	// bottom
	dd->vertex(minx, maxy, minz, col);
	dd->vertex(maxx, maxy, minz, col);
	dd->vertex(maxx, maxy, minz, col);
	dd->vertex(maxx, maxy, maxz, col);
	dd->vertex(maxx, maxy, maxz, col);
	dd->vertex(minx, maxy, maxz, col);
	dd->vertex(minx, maxy, maxz, col);
	dd->vertex(minx, maxy, minz, col);
}

void duAppendBox(struct duDebugDraw* dd, float minx, float miny, float minz,
				 float maxx, float maxy, float maxz, const unsigned int* fcol)
{
	if (!dd) return;
	const float verts[8*3] =
	{
		minx, miny, minz,
		maxx, miny, minz,
		maxx, miny, maxz,
		minx, miny, maxz,
		minx, maxy, minz,
		maxx, maxy, minz,
		maxx, maxy, maxz,
		minx, maxy, maxz,
	};
	static const unsigned char inds[6*4] =
	{
		7, 6, 5, 4,
		0, 1, 2, 3,
		1, 5, 6, 2,
		3, 7, 4, 0,
		2, 6, 7, 3,
		0, 4, 5, 1,
	};
	
	const unsigned char* in = inds;
	for (int i = 0; i < 6; ++i)
	{
		dd->vertex(&verts[*in*3], fcol[i]); in++;
		dd->vertex(&verts[*in*3], fcol[i]); in++;
		dd->vertex(&verts[*in*3], fcol[i]); in++;
		dd->vertex(&verts[*in*3], fcol[i]); in++;
	}
}

void duAppendCylinder(struct duDebugDraw* dd, float minx, float miny, float minz,
					  float maxx, float maxy, float maxz, unsigned int col)
{
	if (!dd) return;
	
	static const int NUM_SEG = 16;
	static float dir[NUM_SEG*2];
	static bool init = false;
	if (!init)
	{
		init = true;
		for (int i = 0; i < NUM_SEG; ++i)
		{
			const float a = (float)i/(float)NUM_SEG*DU_PI*2;
			dir[i*2] = cosf(a);
			dir[i*2+1] = sinf(a);
		}
	}
	
	unsigned int col2 = duMultCol(col, 160);
	
	const float cx = (maxx + minx)/2;
	const float cz = (maxz + minz)/2;
	const float rx = (maxx - minx)/2;
	const float rz = (maxz - minz)/2;

	for (int i = 2; i < NUM_SEG; ++i)
	{
		const int a = 0, b = i-1, c = i;
		dd->vertex(cx+dir[a*2+0]*rx, miny, cz+dir[a*2+1]*rz, col2);
		dd->vertex(cx+dir[b*2+0]*rx, miny, cz+dir[b*2+1]*rz, col2);
		dd->vertex(cx+dir[c*2+0]*rx, miny, cz+dir[c*2+1]*rz, col2);
	}
	for (int i = 2; i < NUM_SEG; ++i)
	{
		const int a = 0, b = i, c = i-1;
		dd->vertex(cx+dir[a*2+0]*rx, maxy, cz+dir[a*2+1]*rz, col);
		dd->vertex(cx+dir[b*2+0]*rx, maxy, cz+dir[b*2+1]*rz, col);
		dd->vertex(cx+dir[c*2+0]*rx, maxy, cz+dir[c*2+1]*rz, col);
	}
	for (int i = 0, j = NUM_SEG-1; i < NUM_SEG; j = i++)
	{
		dd->vertex(cx+dir[i*2+0]*rx, miny, cz+dir[i*2+1]*rz, col2);
		dd->vertex(cx+dir[j*2+0]*rx, miny, cz+dir[j*2+1]*rz, col2);
		dd->vertex(cx+dir[j*2+0]*rx, maxy, cz+dir[j*2+1]*rz, col);

		dd->vertex(cx+dir[i*2+0]*rx, miny, cz+dir[i*2+1]*rz, col2);
		dd->vertex(cx+dir[j*2+0]*rx, maxy, cz+dir[j*2+1]*rz, col);
		dd->vertex(cx+dir[i*2+0]*rx, maxy, cz+dir[i*2+1]*rz, col);
	}
}


inline void evalArc(const float x0, const float y0, const float z0,
					const float dx, const float dy, const float dz,
					const float h, const float u, float* res)
{
	res[0] = x0 + dx * u;
	res[1] = y0 + dy * u + h * (1-(u*2-1)*(u*2-1));
	res[2] = z0 + dz * u;
}


inline void vcross(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[1]*v2[2] - v1[2]*v2[1];
	dest[1] = v1[2]*v2[0] - v1[0]*v2[2];
	dest[2] = v1[0]*v2[1] - v1[1]*v2[0]; 
}

inline void vnormalize(float* v)
{
	float d = 1.0f / sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
	v[0] *= d;
	v[1] *= d;
	v[2] *= d;
}

inline void vsub(float* dest, const float* v1, const float* v2)
{
	dest[0] = v1[0]-v2[0];
	dest[1] = v1[1]-v2[1];
	dest[2] = v1[2]-v2[2];
}

inline float vdistSqr(const float* v1, const float* v2)
{
	const float x = v1[0]-v2[0];
	const float y = v1[1]-v2[1];
	const float z = v1[2]-v2[2];
	return x*x + y*y + z*z;
}


void appendArrowHead(struct duDebugDraw* dd, const float* p, const float* q,
					 const float s, unsigned int col)
{
	const float eps = 0.001f;
	if (!dd) return;
	if (vdistSqr(p,q) < eps*eps) return;
	float ax[3], ay[3] = {0,1,0}, az[3];
	vsub(az, q, p);
	vnormalize(az);
	vcross(ax, ay, az);
	vcross(ay, az, ax);
	vnormalize(ay);

	dd->vertex(p, col);
//	dd->vertex(p[0]+az[0]*s+ay[0]*s/2, p[1]+az[1]*s+ay[1]*s/2, p[2]+az[2]*s+ay[2]*s/2, col);
	dd->vertex(p[0]+az[0]*s+ax[0]*s/3, p[1]+az[1]*s+ax[1]*s/3, p[2]+az[2]*s+ax[2]*s/3, col);

	dd->vertex(p, col);
//	dd->vertex(p[0]+az[0]*s-ay[0]*s/2, p[1]+az[1]*s-ay[1]*s/2, p[2]+az[2]*s-ay[2]*s/2, col);
	dd->vertex(p[0]+az[0]*s-ax[0]*s/3, p[1]+az[1]*s-ax[1]*s/3, p[2]+az[2]*s-ax[2]*s/3, col);
	
}

void duAppendArc(struct duDebugDraw* dd, const float x0, const float y0, const float z0,
				 const float x1, const float y1, const float z1, const float h,
				 const float as0, const float as1, unsigned int col)
{
	if (!dd) return;
	static const int NUM_ARC_PTS = 8;
	static const float PAD = 0.05f;
	static const float ARC_PTS_SCALE = (1.0f-PAD*2) / (float)NUM_ARC_PTS;
	const float dx = x1 - x0;
	const float dy = y1 - y0;
	const float dz = z1 - z0;
	const float len = sqrtf(dx*dx + dy*dy + dz*dz);
	float prev[3];
	evalArc(x0,y0,z0, dx,dy,dz, len*h, PAD, prev);
	for (int i = 1; i <= NUM_ARC_PTS; ++i)
	{
		const float u = PAD + i * ARC_PTS_SCALE;
		float pt[3];
		evalArc(x0,y0,z0, dx,dy,dz, len*h, u, pt);
		dd->vertex(prev[0],prev[1],prev[2], col);
		dd->vertex(pt[0],pt[1],pt[2], col);
		prev[0] = pt[0]; prev[1] = pt[1]; prev[2] = pt[2];
	}
	
	// End arrows
	if (as0 > 0.001f)
	{
		float p[3], q[3];
		evalArc(x0,y0,z0, dx,dy,dz, len*h, PAD, p);
		evalArc(x0,y0,z0, dx,dy,dz, len*h, PAD+0.05f, q);
		appendArrowHead(dd, p, q, as0, col);
	}

	if (as1 > 0.001f)
	{
		float p[3], q[3];
		evalArc(x0,y0,z0, dx,dy,dz, len*h, 1-PAD, p);
		evalArc(x0,y0,z0, dx,dy,dz, len*h, 1-(PAD+0.05f), q);
		appendArrowHead(dd, p, q, as1, col);
	}
}

void duAppendArrow(struct duDebugDraw* dd, const float x0, const float y0, const float z0,
				   const float x1, const float y1, const float z1,
				   const float as0, const float as1, unsigned int col)
{
	if (!dd) return;

	dd->vertex(x0,y0,z0, col);
	dd->vertex(x1,y1,z1, col);
	
	// End arrows
	const float p[3] = {x0,y0,z0}, q[3] = {x1,y1,z1};
	if (as0 > 0.001f)
		appendArrowHead(dd, p, q, as0, col);
	if (as1 > 0.001f)
		appendArrowHead(dd, q, p, as1, col);
}

void duAppendCircle(struct duDebugDraw* dd, const float x, const float y, const float z,
					const float r, unsigned int col)
{
	if (!dd) return;
	static const int NUM_SEG = 40;
	static float dir[40*2];
	static bool init = false;
	if (!init)
	{
		init = true;
		for (int i = 0; i < NUM_SEG; ++i)
		{
			const float a = (float)i/(float)NUM_SEG*DU_PI*2;
			dir[i*2] = cosf(a);
			dir[i*2+1] = sinf(a);
		}
	}
	
	for (int i = 0, j = NUM_SEG-1; i < NUM_SEG; j = i++)
	{
		dd->vertex(x+dir[j*2+0]*r, y, z+dir[j*2+1]*r, col);
		dd->vertex(x+dir[i*2+0]*r, y, z+dir[i*2+1]*r, col);
	}
}

void duAppendCross(struct duDebugDraw* dd, const float x, const float y, const float z,
				   const float s, unsigned int col)
{
	if (!dd) return;
	dd->vertex(x-s,y,z, col);
	dd->vertex(x+s,y,z, col);
	dd->vertex(x,y-s,z, col);
	dd->vertex(x,y+s,z, col);
	dd->vertex(x,y,z-s, col);
	dd->vertex(x,y,z+s, col);
}

duDisplayList::duDisplayList(int cap) :
	m_pos(0),
	m_color(0),
	m_size(0),
	m_cap(0),
	m_prim(DU_DRAW_LINES),
	m_primSize(1.0f),
	m_depthMask(true)
{
	if (cap < 8)
		cap = 8;
	resize(cap);
}

duDisplayList::~duDisplayList()
{
	delete [] m_pos;
	delete [] m_color;
}

void duDisplayList::resize(int cap)
{
	float* newPos = new float[cap*3];
	if (m_size)
		memcpy(newPos, m_pos, sizeof(float)*3*m_size);
	delete [] m_pos;
	m_pos = newPos;

	unsigned int* newColor = new unsigned int[cap];
	if (m_size)
		memcpy(newColor, m_color, sizeof(unsigned int)*m_size);
	delete [] m_color;
	m_color = newColor;
	
	m_cap = cap;
}

void duDisplayList::clear()
{
	m_size = 0;
}

void duDisplayList::depthMask(bool state)
{
	m_depthMask = state;
}

void duDisplayList::begin(duDebugDrawPrimitives prim, float size)
{
	clear();
	m_prim = prim;
	m_primSize = size;
}

void duDisplayList::vertex(const float x, const float y, const float z, unsigned int color)
{
	if (m_size+1 >= m_cap)
		resize(m_cap*2);
	float* p = &m_pos[m_size*3];
	p[0] = x;
	p[1] = y;
	p[2] = z;
	m_color[m_size] = color;
	m_size++;
}

void duDisplayList::vertex(const float* pos, unsigned int color)
{
	vertex(pos[0],pos[1],pos[2],color);
}

void duDisplayList::end()
{
}

void duDisplayList::draw(struct duDebugDraw* dd)
{
	if (!dd) return;
	if (!m_size) return;
	dd->depthMask(m_depthMask);
	dd->begin(m_prim, m_primSize);
	for (int i = 0; i < m_size; ++i)
		dd->vertex(&m_pos[i*3], m_color[i]);
	dd->end();
}


/*													*/

void duDebugDrawTriMesh(duDebugDraw* dd, const float* verts, int /*nverts*/,
						const int* tris, const float* normals, int ntris,
						const unsigned char* flags, const float texScale)
{
	if (!dd) return;
	if (!verts) return;
	if (!tris) return;
	if (!normals) return;

	float uva[2];
	float uvb[2];
	float uvc[2];

	const unsigned int unwalkable = duRGBA(192,128,0,255);

	dd->texture(true);

	dd->begin(DU_DRAW_TRIS);
	for (int i = 0; i < ntris*3; i += 3)
	{
		const float* norm = &normals[i];
		unsigned int color;
		unsigned char a = (unsigned char)(220*(2+norm[0]+norm[1])/4);
		if (flags && !flags[i/3])
			color = duLerpCol(duRGBA(a,a,a,255), unwalkable, 64);
		else
			color = duRGBA(a,a,a,255);

		const float* va = &verts[tris[i+0]*3];
		const float* vb = &verts[tris[i+1]*3];
		const float* vc = &verts[tris[i+2]*3];
		
		int ax = 0, ay = 0;
		if (rcAbs(norm[1]) > rcAbs(norm[ax]))
			ax = 1;
		if (rcAbs(norm[2]) > rcAbs(norm[ax]))
			ax = 2;
		ax = (1<<ax)&3; // +1 mod 3
		ay = (1<<ax)&3; // +1 mod 3

		uva[0] = va[ax]*texScale;
		uva[1] = va[ay]*texScale;
		uvb[0] = vb[ax]*texScale;
		uvb[1] = vb[ay]*texScale;
		uvc[0] = vc[ax]*texScale;
		uvc[1] = vc[ay]*texScale;
		
		dd->vertex(va, color, uva);
		dd->vertex(vb, color, uvb);
		dd->vertex(vc, color, uvc);
	}
	dd->end();
	dd->texture(false);
}

void duDebugDrawTriMeshSlope(duDebugDraw* dd, const float* verts, int /*nverts*/,
							 const int* tris, const float* normals, int ntris,
							 const float walkableSlopeAngle, const float texScale)
{
	if (!dd) return;
	if (!verts) return;
	if (!tris) return;
	if (!normals) return;
	
	const float walkableThr = cosf(walkableSlopeAngle/180.0f*DU_PI);
	
	float uva[2];
	float uvb[2];
	float uvc[2];
	
	dd->texture(true);

	const unsigned int unwalkable = duRGBA(192,128,0,255);
	
	dd->begin(DU_DRAW_TRIS);
	for (int i = 0; i < ntris*3; i += 3)
	{
		const float* norm = &normals[i];
		unsigned int color;
		unsigned char a = (unsigned char)(220*(2+norm[0]+norm[1])/4);
		if (norm[1] < walkableThr)
			color = duLerpCol(duRGBA(a,a,a,255), unwalkable, 64);
		else
			color = duRGBA(a,a,a,255);
		
		const float* va = &verts[tris[i+0]*3];
		const float* vb = &verts[tris[i+1]*3];
		const float* vc = &verts[tris[i+2]*3];
		
		int ax = 0, ay = 0;
		if (rcAbs(norm[1]) > rcAbs(norm[ax]))
			ax = 1;
		if (rcAbs(norm[2]) > rcAbs(norm[ax]))
			ax = 2;
		ax = (1<<ax)&3; // +1 mod 3
		ay = (1<<ax)&3; // +1 mod 3
		
		uva[0] = va[ax]*texScale;
		uva[1] = va[ay]*texScale;
		uvb[0] = vb[ax]*texScale;
		uvb[1] = vb[ay]*texScale;
		uvc[0] = vc[ax]*texScale;
		uvc[1] = vc[ay]*texScale;
		
		dd->vertex(va, color, uva);
		dd->vertex(vb, color, uvb);
		dd->vertex(vc, color, uvc);
	}
	dd->end();

	dd->texture(false);
}

void duDebugDrawHeightfieldSolid(duDebugDraw* dd, const Moss_RecastHeightfield& hf)
{
	if (!dd) return;

	const float* orig = hf.bmin;
	const float cs = hf.cs;
	const float ch = hf.ch;
	
	const int w = hf.width;
	const int h = hf.height;
		
	unsigned int fcol[6];
	duCalcBoxColors(fcol, duRGBA(255,255,255,255), duRGBA(255,255,255,255));
	
	dd->begin(DU_DRAW_QUADS);
	
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			float fx = orig[0] + x*cs;
			float fz = orig[2] + y*cs;
			const Moss_RecastSpan* s = hf.spans[x + y*w];
			while (s)
			{
				duAppendBox(dd, fx, orig[1]+s->smin*ch, fz, fx+cs, orig[1] + s->smax*ch, fz+cs, fcol);
				s = s->next;
			}
		}
	}
	dd->end();
}

void duDebugDrawHeightfieldWalkable(duDebugDraw* dd, const Moss_RecastHeightfield& hf)
{
	if (!dd) return;

	const float* orig = hf.bmin;
	const float cs = hf.cs;
	const float ch = hf.ch;
	
	const int w = hf.width;
	const int h = hf.height;
	
	unsigned int fcol[6];
	duCalcBoxColors(fcol, duRGBA(255,255,255,255), duRGBA(217,217,217,255));

	dd->begin(DU_DRAW_QUADS);
	
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			float fx = orig[0] + x*cs;
			float fz = orig[2] + y*cs;
			const Moss_RecastSpan* s = hf.spans[x + y*w];
			while (s)
			{
				if (s->area == RC_WALKABLE_AREA)
					fcol[0] = duRGBA(64,128,160,255);
				else if (s->area == RC_NULL_AREA)
					fcol[0] = duRGBA(64,64,64,255);
				else
					fcol[0] = duMultCol(dd->areaToCol(s->area), 200);
				
				duAppendBox(dd, fx, orig[1]+s->smin*ch, fz, fx+cs, orig[1] + s->smax*ch, fz+cs, fcol);
				s = s->next;
			}
		}
	}
	
	dd->end();
}

void duDebugDrawCompactHeightfieldSolid(duDebugDraw* dd, const Moss_RecastCompactHeightfield& chf)
{
	if (!dd) return;

	const float cs = chf.cs;
	const float ch = chf.ch;

	dd->begin(DU_DRAW_QUADS);
	
	for (int y = 0; y < chf.height; ++y)
	{
		for (int x = 0; x < chf.width; ++x)
		{
			const float fx = chf.bmin[0] + x*cs;
			const float fz = chf.bmin[2] + y*cs;
			const Moss_RecastCompactCell& c = chf.cells[x+y*chf.width];

			for (unsigned i = c.index, ni = c.index+c.count; i < ni; ++i)
			{
				const Moss_RecastCompactSpan& s = chf.spans[i];

				const unsigned char area = chf.areas[i];
				unsigned int color;
				if (area == RC_WALKABLE_AREA)
					color = duRGBA(0,192,255,64);
				else if (area == RC_NULL_AREA)
					color = duRGBA(0,0,0,64);
				else
					color = dd->areaToCol(area);
				
				const float fy = chf.bmin[1] + (s.y+1)*ch;
				dd->vertex(fx, fy, fz, color);
				dd->vertex(fx, fy, fz+cs, color);
				dd->vertex(fx+cs, fy, fz+cs, color);
				dd->vertex(fx+cs, fy, fz, color);
			}
		}
	}
	dd->end();
}

void duDebugDrawCompactHeightfieldRegions(duDebugDraw* dd, const Moss_RecastCompactHeightfield& chf)
{
	if (!dd) return;

	const float cs = chf.cs;
	const float ch = chf.ch;

	dd->begin(DU_DRAW_QUADS);

	for (int y = 0; y < chf.height; ++y)
	{
		for (int x = 0; x < chf.width; ++x)
		{
			const float fx = chf.bmin[0] + x*cs;
			const float fz = chf.bmin[2] + y*cs;
			const Moss_RecastCompactCell& c = chf.cells[x+y*chf.width];
			
			for (unsigned i = c.index, ni = c.index+c.count; i < ni; ++i)
			{
				const Moss_RecastCompactSpan& s = chf.spans[i];
				const float fy = chf.bmin[1] + (s.y)*ch;
				unsigned int color;
				if (s.reg)
					color = duIntToCol(s.reg, 192);
				else
					color = duRGBA(0,0,0,64);

				dd->vertex(fx, fy, fz, color);
				dd->vertex(fx, fy, fz+cs, color);
				dd->vertex(fx+cs, fy, fz+cs, color);
				dd->vertex(fx+cs, fy, fz, color);
			}
		}
	}
	
	dd->end();
}


void duDebugDrawCompactHeightfieldDistance(duDebugDraw* dd, const Moss_RecastCompactHeightfield& chf)
{
	if (!dd) return;
	if (!chf.dist) return;
		
	const float cs = chf.cs;
	const float ch = chf.ch;
			
	float maxd = chf.maxDistance;
	if (maxd < 1.0f) maxd = 1;
	const float dscale = 255.0f / maxd;
	
	dd->begin(DU_DRAW_QUADS);
	
	for (int y = 0; y < chf.height; ++y)
	{
		for (int x = 0; x < chf.width; ++x)
		{
			const float fx = chf.bmin[0] + x*cs;
			const float fz = chf.bmin[2] + y*cs;
			const Moss_RecastCompactCell& c = chf.cells[x+y*chf.width];
			
			for (unsigned i = c.index, ni = c.index+c.count; i < ni; ++i)
			{
				const Moss_RecastCompactSpan& s = chf.spans[i];
				const float fy = chf.bmin[1] + (s.y+1)*ch;
				const unsigned char cd = (unsigned char)(chf.dist[i] * dscale);
				const unsigned int color = duRGBA(cd,cd,cd,255);
				dd->vertex(fx, fy, fz, color);
				dd->vertex(fx, fy, fz+cs, color);
				dd->vertex(fx+cs, fy, fz+cs, color);
				dd->vertex(fx+cs, fy, fz, color);
			}
		}
	}
	dd->end();
}

static void drawLayerPortals(duDebugDraw* dd, const Moss_RecastHeightfieldLayer* layer)
{
	const float cs = layer->cs;
	const float ch = layer->ch;
	const int w = layer->width;
	const int h = layer->height;
	
	unsigned int pcol = duRGBA(255,255,255,255);
	
	const int segs[4*4] = {0,0,0,1, 0,1,1,1, 1,1,1,0, 1,0,0,0};
	
	// Layer portals
	dd->begin(DU_DRAW_LINES, 2.0f);
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const int idx = x+y*w;
			const int lh = (int)layer->heights[idx];
			if (lh == 255) continue;
			
			for (int dir = 0; dir < 4; ++dir)
			{
				if (layer->cons[idx] & (1<<(dir+4)))
				{
					const int* seg = &segs[dir*4];
					const float ax = layer->bmin[0] + (x+seg[0])*cs;
					const float ay = layer->bmin[1] + (lh+2)*ch;
					const float az = layer->bmin[2] + (y+seg[1])*cs;
					const float bx = layer->bmin[0] + (x+seg[2])*cs;
					const float by = layer->bmin[1] + (lh+2)*ch;
					const float bz = layer->bmin[2] + (y+seg[3])*cs;
					dd->vertex(ax, ay, az, pcol);
					dd->vertex(bx, by, bz, pcol);
				}
			}
		}
	}
	dd->end();
}

void duDebugDrawHeightfieldLayer(duDebugDraw* dd, const struct Moss_RecastHeightfieldLayer& layer, const int idx)
{
	const float cs = layer.cs;
	const float ch = layer.ch;
	const int w = layer.width;
	const int h = layer.height;
	
	unsigned int color = duIntToCol(idx+1, 255);
	
	// Layer bounds
	float bmin[3], bmax[3];
	bmin[0] = layer.bmin[0] + layer.minx*cs;
	bmin[1] = layer.bmin[1];
	bmin[2] = layer.bmin[2] + layer.miny*cs;
	bmax[0] = layer.bmin[0] + (layer.maxx+1)*cs;
	bmax[1] = layer.bmax[1];
	bmax[2] = layer.bmin[2] + (layer.maxy+1)*cs;
	duDebugDrawBoxWire(dd, bmin[0],bmin[1],bmin[2], bmax[0],bmax[1],bmax[2], duTransCol(color,128), 2.0f);
	
	// Layer height
	dd->begin(DU_DRAW_QUADS);
	for (int y = 0; y < h; ++y)
	{
		for (int x = 0; x < w; ++x)
		{
			const int lidx = x+y*w;
			const int lh = (int)layer.heights[lidx];
			if (h == 0xff) continue;
			const unsigned char area = layer.areas[lidx];
			
			unsigned int col;
			if (area == RC_WALKABLE_AREA)
				col = duLerpCol(color, duRGBA(0,192,255,64), 32);
			else if (area == RC_NULL_AREA)
				col = duLerpCol(color, duRGBA(0,0,0,64), 32);
			else
				col = duLerpCol(color, dd->areaToCol(area), 32);
			
			const float fx = layer.bmin[0] + x*cs;
			const float fy = layer.bmin[1] + (lh+1)*ch;
			const float fz = layer.bmin[2] + y*cs;
			
			dd->vertex(fx, fy, fz, col);
			dd->vertex(fx, fy, fz+cs, col);
			dd->vertex(fx+cs, fy, fz+cs, col);
			dd->vertex(fx+cs, fy, fz, col);
		}
	}
	dd->end();
	
	// Portals
	drawLayerPortals(dd, &layer);
}

void duDebugDrawHeightfieldLayers(duDebugDraw* dd, const struct Moss_RecastHeightfieldLayerSet& lset)
{
	if (!dd) return;
	for (int i = 0; i < lset.nlayers; ++i)
		duDebugDrawHeightfieldLayer(dd, lset.layers[i], i);
}

/*
void duDebugDrawLayerContours(duDebugDraw* dd, const struct rcLayerContourSet& lcset)
{
	if (!dd) return;
	
	const float* orig = lcset.bmin;
	const float cs = lcset.cs;
	const float ch = lcset.ch;
	
	const unsigned char a = 255;// (unsigned char)(alpha*255.0f);
	
	const int offs[2*4] = {-1,0, 0,1, 1,0, 0,-1};

	dd->begin(DU_DRAW_LINES, 2.0f);
	
	for (int i = 0; i < lcset.nconts; ++i)
	{
		const rcLayerContour& c = lcset.conts[i];
		unsigned int color = 0;

		color = duIntToCol(i, a);

		for (int j = 0; j < c.nverts; ++j)
		{
			const int k = (j+1) % c.nverts;
			const unsigned char* va = &c.verts[j*4];
			const unsigned char* vb = &c.verts[k*4];
			const float ax = orig[0] + va[0]*cs;
			const float ay = orig[1] + (va[1]+1+(i&1))*ch;
			const float az = orig[2] + va[2]*cs;
			const float bx = orig[0] + vb[0]*cs;
			const float by = orig[1] + (vb[1]+1+(i&1))*ch;
			const float bz = orig[2] + vb[2]*cs;
			unsigned int col = color;
			if ((va[3] & 0xf) != 0xf)
			{
				col = duRGBA(255,255,255,128);
				int d = va[3] & 0xf;
				
				const float cx = (ax+bx)*0.5f;
				const float cy = (ay+by)*0.5f;
				const float cz = (az+bz)*0.5f;
				
				const float dx = cx + offs[d*2+0]*2*cs;
				const float dy = cy;
				const float dz = cz + offs[d*2+1]*2*cs;
				
				dd->vertex(cx,cy,cz,duRGBA(255,0,0,255));
				dd->vertex(dx,dy,dz,duRGBA(255,0,0,255));
			}
			
			duAppendArrow(dd, ax,ay,az, bx,by,bz, 0.0f, cs*0.5f, col);
		}
	}
	dd->end();
	
	dd->begin(DU_DRAW_POINTS, 4.0f);	
	
	for (int i = 0; i < lcset.nconts; ++i)
	{
		const rcLayerContour& c = lcset.conts[i];
		unsigned int color = 0;
		
		for (int j = 0; j < c.nverts; ++j)
		{
			const unsigned char* va = &c.verts[j*4];

			color = duDarkenCol(duIntToCol(i, a));
			if (va[3] & 0x80)
				color = duRGBA(255,0,0,255);

			float fx = orig[0] + va[0]*cs;
			float fy = orig[1] + (va[1]+1+(i&1))*ch;
			float fz = orig[2] + va[2]*cs;
			dd->vertex(fx,fy,fz, color);
		}
	}
	dd->end();
}

void duDebugDrawLayerPolyMesh(duDebugDraw* dd, const struct rcLayerPolyMesh& lmesh)
{
	if (!dd) return;
	
	const int nvp = lmesh.nvp;
	const float cs = lmesh.cs;
	const float ch = lmesh.ch;
	const float* orig = lmesh.bmin;
	
	const int offs[2*4] = {-1,0, 0,1, 1,0, 0,-1};

	dd->begin(DU_DRAW_TRIS);
	
	for (int i = 0; i < lmesh.npolys; ++i)
	{
		const unsigned short* p = &lmesh.polys[i*nvp*2];
		
		unsigned int color;
		if (lmesh.areas[i] == RC_WALKABLE_AREA)
			color = duRGBA(0,192,255,64);
		else if (lmesh.areas[i] == RC_NULL_AREA)
			color = duRGBA(0,0,0,64);
		else
			color = duIntToCol(lmesh.areas[i], 255);
		
		unsigned short vi[3];
		for (int j = 2; j < nvp; ++j)
		{
			if (p[j] == RC_MESH_NULL_IDX) break;
			vi[0] = p[0];
			vi[1] = p[j-1];
			vi[2] = p[j];
			for (int k = 0; k < 3; ++k)
			{
				const unsigned short* v = &lmesh.verts[vi[k]*3];
				const float x = orig[0] + v[0]*cs;
				const float y = orig[1] + (v[1]+1)*ch;
				const float z = orig[2] + v[2]*cs;
				dd->vertex(x,y,z, color);
			}
		}
	}
	dd->end();
	
	// Draw neighbours edges
	const unsigned int coln = duRGBA(0,48,64,32);
	dd->begin(DU_DRAW_LINES, 1.5f);
	for (int i = 0; i < lmesh.npolys; ++i)
	{
		const unsigned short* p = &lmesh.polys[i*nvp*2];
		for (int j = 0; j < nvp; ++j)
		{
			if (p[j] == RC_MESH_NULL_IDX) break;
			if (p[nvp+j] & 0x8000) continue;
			const int nj = (j+1 >= nvp || p[j+1] == RC_MESH_NULL_IDX) ? 0 : j+1; 
			int vi[2] = {p[j], p[nj]};
			
			for (int k = 0; k < 2; ++k)
			{
				const unsigned short* v = &lmesh.verts[vi[k]*3];
				const float x = orig[0] + v[0]*cs;
				const float y = orig[1] + (v[1]+1)*ch + 0.1f;
				const float z = orig[2] + v[2]*cs;
				dd->vertex(x, y, z, coln);
			}
		}
	}
	dd->end();
	
	// Draw boundary edges
	const unsigned int colb = duRGBA(0,48,64,220);
	dd->begin(DU_DRAW_LINES, 2.5f);
	for (int i = 0; i < lmesh.npolys; ++i)
	{
		const unsigned short* p = &lmesh.polys[i*nvp*2];
		for (int j = 0; j < nvp; ++j)
		{
			if (p[j] == RC_MESH_NULL_IDX) break;
			if ((p[nvp+j] & 0x8000) == 0) continue;
			const int nj = (j+1 >= nvp || p[j+1] == RC_MESH_NULL_IDX) ? 0 : j+1; 
			int vi[2] = {p[j], p[nj]};
			
			unsigned int col = colb;
			if ((p[nvp+j] & 0xf) != 0xf)
			{
				const unsigned short* va = &lmesh.verts[vi[0]*3];
				const unsigned short* vb = &lmesh.verts[vi[1]*3];

				const float ax = orig[0] + va[0]*cs;
				const float ay = orig[1] + (va[1]+1+(i&1))*ch;
				const float az = orig[2] + va[2]*cs;
				const float bx = orig[0] + vb[0]*cs;
				const float by = orig[1] + (vb[1]+1+(i&1))*ch;
				const float bz = orig[2] + vb[2]*cs;
				
				const float cx = (ax+bx)*0.5f;
				const float cy = (ay+by)*0.5f;
				const float cz = (az+bz)*0.5f;
				
				int d = p[nvp+j] & 0xf;
				
				const float dx = cx + offs[d*2+0]*2*cs;
				const float dy = cy;
				const float dz = cz + offs[d*2+1]*2*cs;
				
				dd->vertex(cx,cy,cz,duRGBA(255,0,0,255));
				dd->vertex(dx,dy,dz,duRGBA(255,0,0,255));
				
				col = duRGBA(255,255,255,128);
			}
							 
			for (int k = 0; k < 2; ++k)
			{
				const unsigned short* v = &lmesh.verts[vi[k]*3];
				const float x = orig[0] + v[0]*cs;
				const float y = orig[1] + (v[1]+1)*ch + 0.1f;
				const float z = orig[2] + v[2]*cs;
				dd->vertex(x, y, z, col);
			}
		}
	}
	dd->end();
	
	dd->begin(DU_DRAW_POINTS, 3.0f);
	const unsigned int colv = duRGBA(0,0,0,220);
	for (int i = 0; i < lmesh.nverts; ++i)
	{
		const unsigned short* v = &lmesh.verts[i*3];
		const float x = orig[0] + v[0]*cs;
		const float y = orig[1] + (v[1]+1)*ch + 0.1f;
		const float z = orig[2] + v[2]*cs;
		dd->vertex(x,y,z, colv);
	}
	dd->end();
}
*/

static void getContourCenter(const Moss_RecastContour* cont, const float* orig, float cs, float ch, float* center)
{
	center[0] = 0;
	center[1] = 0;
	center[2] = 0;
	if (!cont->nverts)
		return;
	for (int i = 0; i < cont->nverts; ++i)
	{
		const int* v = &cont->verts[i*4];
		center[0] += (float)v[0];
		center[1] += (float)v[1];
		center[2] += (float)v[2];
	}
	const float s = 1.0f / cont->nverts;
	center[0] *= s * cs;
	center[1] *= s * ch;
	center[2] *= s * cs;
	center[0] += orig[0];
	center[1] += orig[1] + 4*ch;
	center[2] += orig[2];
}

static const Moss_RecastContour* findContourFromSet(const Moss_RecastContourSet& cset, unsigned short reg)
{
	for (int i = 0; i < cset.nconts; ++i)
	{
		if (cset.conts[i].reg == reg)
			return &cset.conts[i];
	}
	return 0;
}

void duDebugDrawRegionConnections(duDebugDraw* dd, const Moss_RecastContourSet& cset, const float alpha)
{
	if (!dd) return;
	
	const float* orig = cset.bmin;
	const float cs = cset.cs;
	const float ch = cset.ch;
	
	// Draw centers
	float pos[3], pos2[3];

	unsigned int color = duRGBA(0,0,0,196);

	dd->begin(DU_DRAW_LINES, 2.0f);

	for (int i = 0; i < cset.nconts; ++i)
	{
		const Moss_RecastContour* cont = &cset.conts[i];
		getContourCenter(cont, orig, cs, ch, pos);
		for (int j = 0; j < cont->nverts; ++j)
		{
			const int* v = &cont->verts[j*4];
			if (v[3] == 0 || (unsigned short)v[3] < cont->reg) continue;
			const Moss_RecastContour* cont2 = findContourFromSet(cset, (unsigned short)v[3]);
			if (cont2)
			{
				getContourCenter(cont2, orig, cs, ch, pos2);
				duAppendArc(dd, pos[0],pos[1],pos[2], pos2[0],pos2[1],pos2[2], 0.25f, 0.6f, 0.6f, color);
			}
		}
	}
	
	dd->end();

	unsigned char a = (unsigned char)(alpha * 255.0f);

	dd->begin(DU_DRAW_POINTS, 7.0f);

	for (int i = 0; i < cset.nconts; ++i)
	{
		const Moss_RecastContour* cont = &cset.conts[i];
		unsigned int col = duDarkenCol(duIntToCol(cont->reg,a));
		getContourCenter(cont, orig, cs, ch, pos);
		dd->vertex(pos, col);
	}
	dd->end();
}

void duDebugDrawRawContours(duDebugDraw* dd, const Moss_RecastContourSet& cset, const float alpha)
{
	if (!dd) return;

	const float* orig = cset.bmin;
	const float cs = cset.cs;
	const float ch = cset.ch;
	
	const unsigned char a = (unsigned char)(alpha*255.0f);
	
	dd->begin(DU_DRAW_LINES, 2.0f);
			
	for (int i = 0; i < cset.nconts; ++i)
	{
		const Moss_RecastContour& c = cset.conts[i];
		unsigned int color = duIntToCol(c.reg, a);

		for (int j = 0; j < c.nrverts; ++j)
		{
			const int* v = &c.rverts[j*4];
			float fx = orig[0] + v[0]*cs;
			float fy = orig[1] + (v[1]+1+(i&1))*ch;
			float fz = orig[2] + v[2]*cs;
			dd->vertex(fx,fy,fz,color);
			if (j > 0)
				dd->vertex(fx,fy,fz,color);
		}
		// Loop last segment.
		const int* v = &c.rverts[0];
		float fx = orig[0] + v[0]*cs;
		float fy = orig[1] + (v[1]+1+(i&1))*ch;
		float fz = orig[2] + v[2]*cs;
		dd->vertex(fx,fy,fz,color);
	}
	dd->end();

	dd->begin(DU_DRAW_POINTS, 2.0f);	

	for (int i = 0; i < cset.nconts; ++i)
	{
		const Moss_RecastContour& c = cset.conts[i];
		unsigned int color = duDarkenCol(duIntToCol(c.reg, a));
		
		for (int j = 0; j < c.nrverts; ++j)
		{
			const int* v = &c.rverts[j*4];
			float off = 0;
			unsigned int colv = color;
			if (v[3] & RC_BORDER_VERTEX)
			{
				colv = duRGBA(255,255,255,a);
				off = ch*2;
			}
			
			float fx = orig[0] + v[0]*cs;
			float fy = orig[1] + (v[1]+1+(i&1))*ch + off;
			float fz = orig[2] + v[2]*cs;
			dd->vertex(fx,fy,fz, colv);
		}
	}
	dd->end();
}

void duDebugDrawContours(duDebugDraw* dd, const Moss_RecastContourSet& cset, const float alpha)
{
	if (!dd) return;

	const float* orig = cset.bmin;
	const float cs = cset.cs;
	const float ch = cset.ch;
	
	const unsigned char a = (unsigned char)(alpha*255.0f);
	
	dd->begin(DU_DRAW_LINES, 2.5f);
	
	for (int i = 0; i < cset.nconts; ++i)
	{
		const Moss_RecastContour& c = cset.conts[i];
		if (!c.nverts)
			continue;
		const unsigned int color = duIntToCol(c.reg, a);
		const unsigned int bcolor = duLerpCol(color,duRGBA(255,255,255,a),128);
		for (int j = 0, k = c.nverts-1; j < c.nverts; k=j++)
		{
			const int* va = &c.verts[k*4];
			const int* vb = &c.verts[j*4];
			unsigned int col = (va[3] & RC_AREA_BORDER) ? bcolor : color; 
			float fx,fy,fz;
			fx = orig[0] + va[0]*cs;
			fy = orig[1] + (va[1]+1+(i&1))*ch;
			fz = orig[2] + va[2]*cs;
			dd->vertex(fx,fy,fz, col);
			fx = orig[0] + vb[0]*cs;
			fy = orig[1] + (vb[1]+1+(i&1))*ch;
			fz = orig[2] + vb[2]*cs;
			dd->vertex(fx,fy,fz, col);
		}
	}
	dd->end();

	dd->begin(DU_DRAW_POINTS, 3.0f);
	
	for (int i = 0; i < cset.nconts; ++i)
	{
		const Moss_RecastContour& c = cset.conts[i];
		unsigned int color = duDarkenCol(duIntToCol(c.reg, a));
		for (int j = 0; j < c.nverts; ++j)
		{
			const int* v = &c.verts[j*4];
			float off = 0;
			unsigned int colv = color;
			if (v[3] & RC_BORDER_VERTEX)
			{
				colv = duRGBA(255,255,255,a);
				off = ch*2;
			}

			float fx = orig[0] + v[0]*cs;
			float fy = orig[1] + (v[1]+1+(i&1))*ch + off;
			float fz = orig[2] + v[2]*cs;
			dd->vertex(fx,fy,fz, colv);
		}
	}
	dd->end();
}

void duDebugDrawPolyMesh(duDebugDraw* dd, const struct Moss_RecastPolyMesh& mesh)
{
	if (!dd) return;

	const int nvp = mesh.nvp;
	const float cs = mesh.cs;
	const float ch = mesh.ch;
	const float* orig = mesh.bmin;
	
	dd->begin(DU_DRAW_TRIS);
	
	for (int i = 0; i < mesh.npolys; ++i)
	{
		const unsigned short* p = &mesh.polys[i*nvp*2];
		const unsigned char area = mesh.areas[i];
		
		unsigned int color;
		if (area == RC_WALKABLE_AREA)
			color = duRGBA(0,192,255,64);
		else if (area == RC_NULL_AREA)
			color = duRGBA(0,0,0,64);
		else
			color = dd->areaToCol(area);
		
		unsigned short vi[3];
		for (int j = 2; j < nvp; ++j)
		{
			if (p[j] == RC_MESH_NULL_IDX) break;
			vi[0] = p[0];
			vi[1] = p[j-1];
			vi[2] = p[j];
			for (int k = 0; k < 3; ++k)
			{
				const unsigned short* v = &mesh.verts[vi[k]*3];
				const float x = orig[0] + v[0]*cs;
				const float y = orig[1] + (v[1]+1)*ch;
				const float z = orig[2] + v[2]*cs;
				dd->vertex(x,y,z, color);
			}
		}
	}
	dd->end();

	// Draw neighbours edges
	const unsigned int coln = duRGBA(0,48,64,32);
	dd->begin(DU_DRAW_LINES, 1.5f);
	for (int i = 0; i < mesh.npolys; ++i)
	{
		const unsigned short* p = &mesh.polys[i*nvp*2];
		for (int j = 0; j < nvp; ++j)
		{
			if (p[j] == RC_MESH_NULL_IDX) break;
			if (p[nvp+j] & 0x8000) continue;
			const int nj = (j+1 >= nvp || p[j+1] == RC_MESH_NULL_IDX) ? 0 : j+1; 
			const int vi[2] = {p[j], p[nj]};
			
			for (int k = 0; k < 2; ++k)
			{
				const unsigned short* v = &mesh.verts[vi[k]*3];
				const float x = orig[0] + v[0]*cs;
				const float y = orig[1] + (v[1]+1)*ch + 0.1f;
				const float z = orig[2] + v[2]*cs;
				dd->vertex(x, y, z, coln);
			}
		}
	}
	dd->end();
	
	// Draw boundary edges
	const unsigned int colb = duRGBA(0,48,64,220);
	dd->begin(DU_DRAW_LINES, 2.5f);
	for (int i = 0; i < mesh.npolys; ++i)
	{
		const unsigned short* p = &mesh.polys[i*nvp*2];
		for (int j = 0; j < nvp; ++j)
		{
			if (p[j] == RC_MESH_NULL_IDX) break;
			if ((p[nvp+j] & 0x8000) == 0) continue;
			const int nj = (j+1 >= nvp || p[j+1] == RC_MESH_NULL_IDX) ? 0 : j+1; 
			const int vi[2] = {p[j], p[nj]};
			
			unsigned int col = colb;
			if ((p[nvp+j] & 0xf) != 0xf)
				col = duRGBA(255,255,255,128);
			for (int k = 0; k < 2; ++k)
			{
				const unsigned short* v = &mesh.verts[vi[k]*3];
				const float x = orig[0] + v[0]*cs;
				const float y = orig[1] + (v[1]+1)*ch + 0.1f;
				const float z = orig[2] + v[2]*cs;
				dd->vertex(x, y, z, col);
			}
		}
	}
	dd->end();
	
	dd->begin(DU_DRAW_POINTS, 3.0f);
	const unsigned int colv = duRGBA(0,0,0,220);
	for (int i = 0; i < mesh.nverts; ++i)
	{
		const unsigned short* v = &mesh.verts[i*3];
		const float x = orig[0] + v[0]*cs;
		const float y = orig[1] + (v[1]+1)*ch + 0.1f;
		const float z = orig[2] + v[2]*cs;
		dd->vertex(x,y,z, colv);
	}
	dd->end();
}

void duDebugDrawPolyMeshDetail(duDebugDraw* dd, const struct Moss_RecastPolyMeshDetail& dmesh)
{
	if (!dd) return;

	dd->begin(DU_DRAW_TRIS);
	
	for (int i = 0; i < dmesh.nmeshes; ++i)
	{
		const unsigned int* m = &dmesh.meshes[i*4];
		const unsigned int bverts = m[0];
		const unsigned int btris = m[2];
		const int ntris = (int)m[3];
		const float* verts = &dmesh.verts[bverts*3];
		const unsigned char* tris = &dmesh.tris[btris*4];

		unsigned int color = duIntToCol(i, 192);

		for (int j = 0; j < ntris; ++j)
		{
			dd->vertex(&verts[tris[j*4+0]*3], color);
			dd->vertex(&verts[tris[j*4+1]*3], color);
			dd->vertex(&verts[tris[j*4+2]*3], color);
		}
	}
	dd->end();

	// Internal edges.
	dd->begin(DU_DRAW_LINES, 1.0f);
	const unsigned int coli = duRGBA(0,0,0,64);
	for (int i = 0; i < dmesh.nmeshes; ++i)
	{
		const unsigned int* m = &dmesh.meshes[i*4];
		const unsigned int bverts = m[0];
		const unsigned int btris = m[2];
		const int ntris = (int)m[3];
		const float* verts = &dmesh.verts[bverts*3];
		const unsigned char* tris = &dmesh.tris[btris*4];
		
		for (int j = 0; j < ntris; ++j)
		{
			const unsigned char* t = &tris[j*4];
			for (int k = 0, kp = 2; k < 3; kp=k++)
			{
				unsigned char ef = (t[3] >> (kp*2)) & 0x3;
				if (ef == 0)
				{
					// Internal edge
					if (t[kp] < t[k])
					{
						dd->vertex(&verts[t[kp]*3], coli);
						dd->vertex(&verts[t[k]*3], coli);
					}
				}
			}
		}
	}
	dd->end();
	
	// External edges.
	dd->begin(DU_DRAW_LINES, 2.0f);
	const unsigned int cole = duRGBA(0,0,0,64);
	for (int i = 0; i < dmesh.nmeshes; ++i)
	{
		const unsigned int* m = &dmesh.meshes[i*4];
		const unsigned int bverts = m[0];
		const unsigned int btris = m[2];
		const int ntris = (int)m[3];
		const float* verts = &dmesh.verts[bverts*3];
		const unsigned char* tris = &dmesh.tris[btris*4];
		
		for (int j = 0; j < ntris; ++j)
		{
			const unsigned char* t = &tris[j*4];
			for (int k = 0, kp = 2; k < 3; kp=k++)
			{
				unsigned char ef = (t[3] >> (kp*2)) & 0x3;
				if (ef != 0)
				{
					// Ext edge
					dd->vertex(&verts[t[kp]*3], cole);
					dd->vertex(&verts[t[k]*3], cole);
				}
			}
		}
	}
	dd->end();
	
	dd->begin(DU_DRAW_POINTS, 3.0f);
	const unsigned int colv = duRGBA(0,0,0,64);
	for (int i = 0; i < dmesh.nmeshes; ++i)
	{
		const unsigned int* m = &dmesh.meshes[i*4];
		const unsigned int bverts = m[0];
		const int nverts = (int)m[1];
		const float* verts = &dmesh.verts[bverts*3];
		for (int j = 0; j < nverts; ++j)
			dd->vertex(&verts[j*3], colv);
	}
	dd->end();
}


MOSS_SUPRESS_WARNINGS_END
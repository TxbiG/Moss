#ifndef MOSS_NAVIGATION_INTERNAL_H
#define MOSS_NAVIGATION_INTERNAL_H


#include <Moss/Moss_Navigation.h>

/*
Recast/ - Navmesh generation
Detour/ - Runtime loading of navmesh data, pathfinding, navmesh queries
DetourTileCache/ - Navmesh streaming. Useful for large levels and open-world games
DetourCrowd/ - Agent movement, collision avoidance, and crowd simulation
DebugUtils/ - API for drawing debug visualizations of navigation data and behavior
*/



struct dtObstacleCylinder {
	Float3 pos;
	float radius;
	float height;
};

struct dtObstacleBox {
	Float3 bmin;
	Float3 bmax;
};

struct dtObstacleOrientedBox {
	Float3 center;
	Float3 halfExtents;
	Float3 rotAux; //{ cos(0.5f*angle)*sin(-0.5f*angle); cos(0.5f*angle)*cos(0.5f*angle) - 0.5 }
};

struct Moss_RecastCompressedTile {
	unsigned int salt;						// Counter describing modifications to the tile.
	struct Moss_RecastTileCacheLayerHeader* header;
	unsigned char* compressed;
	int compressedSize;
	unsigned char* data;
	int dataSize;
	unsigned int flags;
	Moss_RecastCompressedTile* next;
};
struct Moss_RecastTileCacheObstacle {
	union {
		dtObstacleCylinder cylinder;
		dtObstacleBox box;	// AABB3 box;
		dtObstacleOrientedBox orientedBox;	// OOBB3 box;
	};

	Moss_RecastCompressedTileRef touched[DT_MAX_TOUCHED_TILES];
	Moss_RecastCompressedTileRef pending[DT_MAX_TOUCHED_TILES];
	unsigned short salt;
	unsigned char type;
	unsigned char state;
	unsigned char ntouched;
	unsigned char npending;
	Moss_RecastTileCacheObstacle* next;
};

struct Moss_RecastTileCacheLayerHeader {
	int magic;								// Data magic
	int version;							// Data version
	int tx,ty,tlayer;
	AABB3 bounds;
	unsigned short hmin, hmax;				// Height min/max range
	unsigned char width, height;			// Dimension of the layer.
	unsigned char minx, maxx, miny, maxy;	// Usable sub-region.
};

struct dtTileCacheLayer {
	Moss_RecastTileCacheLayerHeader* header;
	unsigned char regCount;					// Region count.
	unsigned char* heights;
	unsigned char* areas;
	unsigned char* cons;
	unsigned char* regs;
};

struct dtTileCacheContour {
	int nverts;
	unsigned char* verts;
	unsigned char reg;
	unsigned char area;
};

struct dtTileCacheContourSet {
	int nconts;
	dtTileCacheContour* conts;
};

struct dtTileCachePolyMesh {
	int nvp;
	int nverts;				// Number of vertices.
	int npolys;				// Number of polygons.
	unsigned short* verts;	// Vertices of the mesh, 3 elements per vertex.
	unsigned short* polys;	// Polygons of the mesh, nvp*2 elements per polygon.
	unsigned short* flags;	// Per polygon flags.
	unsigned char* areas;	// Area ID of polygons.
};

Moss_RecastStatus dtBuildTileCacheLayer(Moss_RecastTileCacheCompressor* comp, Moss_RecastTileCacheLayerHeader* header, const unsigned char* heights, const unsigned char* areas, const unsigned char* cons, unsigned char** outData, int* outDataSize);

Moss_RecastStatus dtDecompressTileCacheLayer(Moss_RecastTileCacheAlloc* alloc, Moss_RecastTileCacheCompressor* comp, unsigned char* compressed, const int compressedSize, dtTileCacheLayer** layerOut);

Moss_RecastStatus dtBuildTileCacheRegions(Moss_RecastTileCacheAlloc* alloc, dtTileCacheLayer& layer, const int walkableClimb);

Moss_RecastStatus dtBuildTileCachePolyMesh(Moss_RecastTileCacheAlloc* alloc, dtTileCacheContourSet& lcset, dtTileCachePolyMesh& mesh);

Moss_RecastStatus dtMarkCylinderArea(dtTileCacheLayer& layer, const float* orig, const float cs, const float ch, const float* pos, const float radius, const float height, const unsigned char areaId);

Moss_RecastStatus dtMarkBoxArea(dtTileCacheLayer& layer, const float* orig, const float cs, const float ch, const float* bmin, const float* bmax, const unsigned char areaId);

Moss_RecastStatus dtMarkBoxArea(dtTileCacheLayer& layer, const float* orig, const float cs, const float ch, const float* center, const float* halfExtents, const float* rotAux, const unsigned char areaId);

Moss_RecastStatus dtBuildTileCacheContours(Moss_RecastTileCacheAlloc* alloc, dtTileCacheLayer& layer, const int walkableClimb, 	const float maxError, dtTileCacheContourSet& lcset);


// Helper functions

// Vector addition
inline Vec2 dtVadd(const Vec2& a, const Vec2& b) { return a + b; }

// Vector subtraction
inline Vec2 dtVsub(const Vec2& a, const Vec2& b) { return a - b; }

// Scaled addition: a + b * s
inline Vec2 dtVmad(const Vec2& a, const Vec2& b, float s) { return a + b * s; }

// Dot product
inline float dtVdot(const Vec2& a, const Vec2& b) { return a.Dot(b); }

// Cross product (scalar in 2D)
inline float dtVperp2D(const Vec2& a, const Vec2& b) { return a.Cross(b); }

// Distance squared
inline float rcVdistSqr(const Vec2& a, const Vec2& b) {
    Vec2 d = a - b;
    return d.LengthSq();
}

float dtVdot2D(const Vec2& a, const Vec2& b) { return a.Dot(b); }

float Cross(const Vec2 &a, const Vec2 &b) { return a.x * b.y - a.y * b.x; }

// Distance
inline float rcVdist(const Vec2& a, const Vec2& b) {
    Vec2 d = a - b;
    return d.Length();
}

// Linear interpolation
inline Vec2 dtVlerp(const Vec2& a, const Vec2& b, float t) { return a.Lerp(a, b, t); }

// Normalize
inline Vec2 rcVnormalize(const Vec2& v) { return v.Normalized(); }


// Colocation check (epsilon)
inline bool dtVequal(const Vec2& a, const Vec2& b) {
    static const float thr = Sqr(1.0f / 16384.0f);
    return (a - b).LengthSq() < thr;
}

// Check finite components
inline bool dtVisfinite(const Vec2& v) { return std::isfinite(v.GetX()) && std::isfinite(v.GetY()); }

inline void rcVmad(float* dest, const float* v1, const float* v2, const float s) {
	dest[0] = v1[0]+v2[0]*s;
	dest[1] = v1[1]+v2[1]*s;
	dest[2] = v1[2]+v2[2]*s;
}

// Selects the minimum value of each element from the specified vectors.
// @param[in,out]	mn	A vector.  (Will be updated with the result.) [(x, y, z)]
// @param[in]		v	A vector. [(x, y, z)]
inline void rcVmin(float* mn, const float* v) {
	mn[0] = min(mn[0], v[0]);
	mn[1] = min(mn[1], v[1]);
	mn[2] = min(mn[2], v[2]);
}

// Selects the maximum value of each element from the specified vectors.
// @param[in,out]	mx	A vector.  (Will be updated with the result.) [(x, y, z)]
// @param[in]		v	A vector. [(x, y, z)]
inline void rcVmax(float* mx, const float* v) {
	mx[0] = max(mx[0], v[0]);
	mx[1] = max(mx[1], v[1]);
	mx[2] = max(mx[2], v[2]);
}

// Performs a vector copy.
// @param[out]		dest	The result. [(x, y, z)]
// @param[in]		v		The vector to copy. [(x, y, z)]
inline void rcVcopy(float* dest, const float* v) {
	dest[0] = v[0];
	dest[1] = v[1];
	dest[2] = v[2];
}

// @}
// @name Vector helper functions.
// @{


// Scales the vector by the specified value. (@p v * @p t)
//  @param[out]	dest	The result vector. [(x, y, z)]
//  @param[in]		v		The vector to scale. [(x, y, z)]
//  @param[in]		t		The scaling factor.
inline void dtVscale(float* dest, const float* v, const float t) {
	dest[0] = v[0]*t;
	dest[1] = v[1]*t;
	dest[2] = v[2]*t;
}

// Sets the vector elements to the specified values.
//  @param[out]	dest	The result vector. [(x, y, z)]
//  @param[in]		x		The x-value of the vector.
//  @param[in]		y		The y-value of the vector.
//  @param[in]		z		The z-value of the vector.
inline void dtVset(float* dest, const float x, const float y, const float z) {
	dest[0] = x; dest[1] = y; dest[2] = z;
}

// Performs a vector copy.
//  @param[out]	dest	The result. [(x, y, z)]
//  @param[in]		a		The vector to copy. [(x, y, z)]
inline void dtVcopy(float* dest, const float* a)
{
	dest[0] = a[0];
	dest[1] = a[1];
	dest[2] = a[2];
}

// Derives the scalar length of the vector.
//  @param[in]		v The vector. [(x, y, z)]
// @return The scalar length of the vector.
inline float dtVlen(const float* v)
{
	return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// Derives the square of the scalar length of the vector. (len * len)
//  @param[in]		v The vector. [(x, y, z)]
// @return The square of the scalar length of the vector.
inline float dtVlenSqr(const float* v)
{
	return v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
}


// Derives the distance between the specified points on the xz-plane.
//  @param[in]		v1	A point. [(x, y, z)]
//  @param[in]		v2	A point. [(x, y, z)]
// @return The distance between the point on the xz-plane.
//
// The vectors are projected onto the xz-plane, so the y-values are ignored.
inline float rcVdist2D(const float* v1, const float* v2)
{
	const float dx = v2[0] - v1[0];
	const float dz = v2[2] - v1[2];
	return sqrtf(dx*dx + dz*dz);
}

// Derives the square of the distance between the specified points on the xz-plane.
//  @param[in]		v1	A point. [(x, y, z)]
//  @param[in]		v2	A point. [(x, y, z)]
// @return The square of the distance between the point on the xz-plane.
inline float rcVdist2DSqr(const float* v1, const float* v2)
{
	const float dx = v2[0] - v1[0];
	const float dz = v2[2] - v1[2];
	return dx*dx + dz*dz;
}


// @name Miscellanious functions.
inline unsigned int dtNextPow2(unsigned int v)
{
	v--;
	v |= v >> 1;
	v |= v >> 2;
	v |= v >> 4;
	v |= v >> 8;
	v |= v >> 16;
	v++;
	return v;
}

inline unsigned int dtIlog2(unsigned int v)
{
	unsigned int r;
	unsigned int shift;
	r = (v > 0xffff) << 4; v >>= r;
	shift = (v > 0xff) << 3; v >>= shift; r |= shift;
	shift = (v > 0xf) << 2; v >>= shift; r |= shift;
	shift = (v > 0x3) << 1; v >>= shift; r |= shift;
	r |= (v >> 1);
	return r;
}

inline int dtAlign4(int x) { return (x+3) & ~3; }

inline int dtOppositeTile(int side) { return (side+4) & 0x7; }

inline void SwapByte(unsigned char* a, unsigned char* b)
{
	unsigned char tmp = *a;
	*a = *b;
	*b = tmp;
}

inline void SwapEndian(unsigned short* v)
{
	unsigned char* x = (unsigned char*)v;
	SwapByte(x+0, x+1);
}

inline void SwapEndian(short* v)
{
	unsigned char* x = (unsigned char*)v;
	SwapByte(x+0, x+1);
}

inline void SwapEndian(unsigned int* v)
{
	unsigned char* x = (unsigned char*)v;
	SwapByte(x+0, x+3); SwapByte(x+1, x+2);
}

inline void SwapEndian(int* v)
{
	unsigned char* x = (unsigned char*)v;
	SwapByte(x+0, x+3); SwapByte(x+1, x+2);
}

inline void SwapEndian(float* v)
{
	unsigned char* x = (unsigned char*)v;
	SwapByte(x+0, x+3); SwapByte(x+1, x+2);
}


// Provides hint values to the memory allocator on how long the memory is expected to be used.
enum class rcAllocHint {
	RC_ALLOC_PERM,		// Memory will persist after a function call.
	RC_ALLOC_TEMP		// Memory used temporarily within a function.
};

// Variable-sized storage type. Mimics the interface of std::vector<T> with some notable differences:
//  * Uses rcAlloc()/MOSS_FREE() to handle storage.
//  * No support for a custom allocator.
//  * Uses signed size instead of size_t to avoid warnings in for loops: "for (int i = 0; i < foo.size(); i++)"
//  * Omits methods of limited utility: insert/erase, (bad performance), at (we don't use exceptions), operator=.
//  * assign() and the pre-sizing constructor follow C++11 semantics -- they don't construct a temporary if no value is provided.
//  * push_back() and resize() support adding values from the current vector. Range-based constructors and assign(begin, end) do not.
//  * No specialization for bool.
template <typename T, rcAllocHint H>
class rcVectorBase {
	rcSizeType m_size;
	rcSizeType m_cap;
	T* m_data;
	// Constructs a T at the give address with either the copy constructor or the default.
	static void construct(T* p, const T& v) { ::new(rcNewTag(), (void*)p) T(v); }
	static void construct(T* p) { ::new(rcNewTag(), (void*)p) T; }
	static void construct_range(T* begin, T* end);
	static void construct_range(T* begin, T* end, const T& value);
	static void copy_range(T* dst, const T* begin, const T* end);
	void destroy_range(rcSizeType begin, rcSizeType end);
	// Creates an array of the given size, copies all of this vector's data into it, and returns it.
	T* allocate_and_copy(rcSizeType size);
	void resize_impl(rcSizeType size, const T* value);
	// Requires: min_capacity > m_cap.
	rcSizeType get_new_capacity(rcSizeType min_capacity);
 public:
	typedef rcSizeType size_type;
	typedef T value_type;

	rcVectorBase() : m_size(0), m_cap(0), m_data(0) {}
	rcVectorBase(const rcVectorBase<T, H>& other) : m_size(0), m_cap(0), m_data(0) { assign(other.begin(), other.end()); }
	explicit rcVectorBase(rcSizeType count) : m_size(0), m_cap(0), m_data(0) { resize(count); }
	rcVectorBase(rcSizeType count, const T& value) : m_size(0), m_cap(0), m_data(0) { resize(count, value); }
	rcVectorBase(const T* begin, const T* end) : m_size(0), m_cap(0), m_data(0) { assign(begin, end); }
	~rcVectorBase() { destroy_range(0, m_size); MOSS_FREE(m_data); }

	// Unlike in std::vector, we return a bool to indicate whether the alloc was successful.
	bool reserve(rcSizeType size);

	void assign(rcSizeType count, const T& value) { clear(); resize(count, value); }
	void assign(const T* begin, const T* end);

	void resize(rcSizeType size) { resize_impl(size, NULL); }
	void resize(rcSizeType size, const T& value) { resize_impl(size, &value); }
	// Not implemented as resize(0) because resize requires T to be default-constructible.
	void clear() { destroy_range(0, m_size); m_size = 0; }

	void push_back(const T& value);
	void pop_back() { rcAssert(m_size > 0); back().~T(); m_size--; }

	rcSizeType size() const { return m_size; }
	rcSizeType capacity() const { return m_cap; }
	bool empty() const { return size() == 0; }

	const T& operator[](rcSizeType i) const { rcAssert(i >= 0 && i < m_size); return m_data[i]; }
	T& operator[](rcSizeType i) { rcAssert(i >= 0 && i < m_size); return m_data[i]; }

	const T& front() const { rcAssert(m_size); return m_data[0]; }
	T& front() { rcAssert(m_size); return m_data[0]; }
	const T& back() const { rcAssert(m_size); return m_data[m_size - 1]; }
	T& back() { rcAssert(m_size); return m_data[m_size - 1]; }
	const T* data() const { return m_data; }
	T* data() { return m_data; }

	T* begin() { return m_data; }
	T* end() { return m_data + m_size; }
	const T* begin() const { return m_data; }
	const T* end() const { return m_data + m_size; }

	void swap(rcVectorBase<T, H>& other);

	// Explicitly deleted.
	rcVectorBase& operator=(const rcVectorBase<T, H>& other);
};

template<typename T, rcAllocHint H>
bool rcVectorBase<T, H>::reserve(rcSizeType count) {
	if (count <= m_cap)
	{
		return true;
	}

	T* new_data = allocate_and_copy(count);
	if (!new_data)
	{
	  return false;
	}

	destroy_range(0, m_size);
	MOSS_FREE(m_data);
	m_data = new_data;
	m_cap = count;
	return true;
}

template <typename T, rcAllocHint H>
T* rcVectorBase<T, H>::allocate_and_copy(rcSizeType size) {
	rcAssert(RC_SIZE_MAX / static_cast<rcSizeType>(sizeof(T)) >= size);
	T* new_data = static_cast<T*>(rcAlloc(sizeof(T) * size, H));
	if (new_data)
	{
		copy_range(new_data, m_data, m_data + m_size);
	}
	return new_data;
}

template <typename T, rcAllocHint H>
void rcVectorBase<T, H>::assign(const T* begin, const T* end) {
	clear();
	reserve(end - begin);
	m_size = end - begin;
	copy_range(m_data, begin, end);
}

template <typename T, rcAllocHint H>
void rcVectorBase<T, H>::push_back(const T& value) {
	// MOSS_LIKELY increases performance by ~50% on BM_rcVector_PushPreallocated,
	// and by ~2-5% on BM_rcVector_Push.
	if (MOSS_LIKELY(m_size < m_cap))
	{
		construct(m_data + m_size++, value);
		return;
	}

	const rcSizeType new_cap = get_new_capacity(m_cap + 1);
	T* data = allocate_and_copy(new_cap);
	// construct between allocate and destroy+free in case value is
	// in this vector.
	construct(data + m_size, value);
	destroy_range(0, m_size);
	m_size++;
	m_cap = new_cap;
	MOSS_FREE(m_data);
	m_data = data;
}

template <typename T, rcAllocHint H>
rcSizeType rcVectorBase<T, H>::get_new_capacity(rcSizeType min_capacity)
{
	rcAssert(min_capacity <= RC_SIZE_MAX);
	if (MOSS_UNLIKELY(m_cap >= RC_SIZE_MAX / 2)) {
		return RC_SIZE_MAX;
	}
	return 2 * m_cap > min_capacity ? 2 * m_cap : min_capacity;
}

template <typename T, rcAllocHint H>
void rcVectorBase<T, H>::resize_impl(rcSizeType size, const T* value)
{
	if (size < m_size)
	{
		destroy_range(size, m_size);
		m_size = size;
	}
	else if (size > m_size)
	{
		if (size <= m_cap)
		{
			if (value)
			{
				construct_range(m_data + m_size, m_data + size, *value);
			}
			else
			{
				construct_range(m_data + m_size, m_data + size);
			}
			m_size = size;
		}
		else
		{
			const rcSizeType new_cap = get_new_capacity(size);
			T* new_data = allocate_and_copy(new_cap);
			// We defer deconstructing/freeing old data until after constructing
			// new elements in case "value" is there.
			if (value)
			{
				construct_range(new_data + m_size, new_data + size, *value);
			}
			else
			{
				construct_range(new_data + m_size, new_data + size);
			}
			destroy_range(0, m_size);
			MOSS_FREE(m_data);
			m_data = new_data;
			m_cap = new_cap;
			m_size = size;
		}
	}
}

template <typename T, rcAllocHint H>
void rcVectorBase<T, H>::swap(rcVectorBase<T, H>& other)
{
	// TODO: Reorganize headers so we can use Swap here.
	rcSizeType tmp_cap = other.m_cap;
	rcSizeType tmp_size = other.m_size;
	T* tmp_data = other.m_data;

	other.m_cap = m_cap;
	other.m_size = m_size;
	other.m_data = m_data;

	m_cap = tmp_cap;
	m_size = tmp_size;
	m_data = tmp_data;
}

// static
template <typename T, rcAllocHint H>
void rcVectorBase<T, H>::construct_range(T* begin, T* end)
{
	for (T* p = begin; p < end; p++)
	{
		construct(p);
	}
}

// static
template <typename T, rcAllocHint H>
void rcVectorBase<T, H>::construct_range(T* begin, T* end, const T& value)
{
	for (T* p = begin; p < end; p++)
	{
		construct(p, value);
	}
}

// static
template <typename T, rcAllocHint H>
void rcVectorBase<T, H>::copy_range(T* dst, const T* begin, const T* end)
{
	for (rcSizeType i = 0 ; i < end - begin; i++)
	{
		construct(dst + i, begin[i]);
	}
}

template <typename T, rcAllocHint H>
void rcVectorBase<T, H>::destroy_range(rcSizeType begin, rcSizeType end)
{
	for (rcSizeType i = begin; i < end; i++)
	{
		m_data[i].~T();
	}
}

template <typename T>
class rcTempVector : public rcVectorBase<T, RC_ALLOC_TEMP>
{
	typedef rcVectorBase<T, RC_ALLOC_TEMP> Base;
public:
	rcTempVector() : Base() {}
	explicit rcTempVector(rcSizeType size) : Base(size) {}
	rcTempVector(rcSizeType size, const T& value) : Base(size, value) {}
	rcTempVector(const rcTempVector<T>& other) : Base(other) {}
	rcTempVector(const T* begin, const T* end) : Base(begin, end) {}
};

template <typename T>
class rcPermVector : public rcVectorBase<T, RC_ALLOC_PERM>
{
	typedef rcVectorBase<T, RC_ALLOC_PERM> Base;
public:
	rcPermVector() : Base() {}
	explicit rcPermVector(rcSizeType size) : Base(size) {}
	rcPermVector(rcSizeType size, const T& value) : Base(size, value) {}
	rcPermVector(const rcPermVector<T>& other) : Base(other) {}
	rcPermVector(const T* begin, const T* end) : Base(begin, end) {}
};

// An implementation of operator new usable for placement new. The default one is part of STL (which we don't use).
// rcNewTag is a dummy type used to differentiate our operator from the STL one, in case users import both Recast
// and STL.
struct rcNewTag {};
inline void* operator new(size_t, const rcNewTag&, void* p) { return p; }
inline void operator delete(void*, const rcNewTag&, void*) {}



// A simple helper class used to delete an array when it goes out of scope.
// @note This class is rarely if ever used by the end user.
template<class T> class rcScopedDelete {
	T* ptr;
public:

	// Constructs an instance with a null pointer.
	inline rcScopedDelete() : ptr(0) {}

	// Constructs an instance with the specified pointer.
	//  @param[in]		p	An pointer to an allocated array.
	inline rcScopedDelete(T* p) : ptr(p) {}
	inline ~rcScopedDelete() { MOSS_FREE(ptr); }

	// The root array pointer.
	//  @return The root array pointer.
	inline operator T*() { return ptr; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	rcScopedDelete(const rcScopedDelete&);
	rcScopedDelete& operator=(const rcScopedDelete&);
};


// Note: This header file's only purpose is to include define assert.
// Feel free to change the file and include your own implementation instead.

#ifdef RC_DISABLE_ASSERTS

// From https://web.archive.org/web/20210117002833/http://cnicholson.net/2009/02/stupid-c-tricks-adventures-in-assert/
#	define dtAssert(x) do { (void)sizeof(x); } while((void)(__LINE__==-1),false)  

#else

// An assertion failure function.
//  @param[in]		expression  asserted expression.
//  @param[in]		file  Filename of the failed assertion.
//  @param[in]		line  Line number of the failed assertion.
//  @see dtAssertFailSetCustom
typedef void (dtAssertFailFunc)(const char* expression, const char* file, int line);

// Sets the base custom assertion failure function to be used by Detour.
//  @param[in]		assertFailFunc	The function to be invoked in case of failure of #dtAssert
void dtAssertFailSetCustom(dtAssertFailFunc *assertFailFunc);

// Gets the base custom assertion failure function to be used by Detour.
dtAssertFailFunc* dtAssertFailGetCustom();

#	include <assert.h> 
#	define dtAssert(expression) \
		{ \
			dtAssertFailFunc* failFunc = dtAssertFailGetCustom(); \
			if(failFunc == NULL) { assert(expression); } \
			else if(!(expression)) { (*failFunc)(#expression, __FILE__, __LINE__); } \
		}

#endif


// An assertion failure callback function.
// @param[in]  expression  Asserted expression.
// @param[in]  file        Filename of the failed assertion.
// @param[in]  line        Line number of the failed assertion.
// @see rcAssertFailSetCustom
typedef void (rcAssertFailFunc)(const char* expression, const char* file, int line);

// Sets the base custom assertion failure callback function to be used by Recast.
// @param[in]  assertFailFunc  The function to be used in case of failure of #dtAssert
void rcAssertFailSetCustom(rcAssertFailFunc* assertFailFunc);

// Gets the base custom assertion failure function to be used by Recast.
rcAssertFailFunc* rcAssertFailGetCustom();

#ifdef RC_DISABLE_ASSERTS
// From https://web.archive.org/web/20210117002833/http://cnicholson.net/2009/02/stupid-c-tricks-adventures-in-assert/
#	define rcAssert(x) do { (void)sizeof(x); } while ((void)(__LINE__==-1), false)
#else
#	include <assert.h> 
#	define rcAssert(expression) \
		{ \
			rcAssertFailFunc* failFunc = rcAssertFailGetCustom(); \
			if (failFunc == NULL) { assert(expression); } \
			else if (!(expression)) { (*failFunc)(#expression, __FILE__, __LINE__); } \
		}
#endif // !defined(RC_DISABLE_ASSERTS)

// A memory allocation function.
// @param[in]    size  The size, in bytes of memory, to allocate.
// @param[in]    hint  A hint to the allocator on how long the memory is expected to be in use.
// @return A pointer to the beginning of the allocated memory block, or null if the allocation failed.
// @see rcAllocSetCustom
typedef void* (rcAllocFunc)(size_t size, rcAllocHint hint);

// A memory deallocation function.
// @param[in]    ptr  A pointer to a memory block previously allocated using #rcAllocFunc.
// @see rcAllocSetCustom
typedef void (rcFreeFunc)(void* ptr);

// Sets the base custom allocation functions to be used by Recast.
// @param[in]    allocFunc  The memory allocation function to be used by #rcAlloc
// @param[in]    freeFunc   The memory de-allocation function to be used by #MOSS_FREE
// @see rcAlloc, MOSS_FREE
void rcAllocSetCustom(rcAllocFunc *allocFunc, rcFreeFunc *freeFunc);

// Allocates a memory block.
// @param[in]    size  The size, in bytes of memory, to allocate.
// @param[in]    hint  A hint to the allocator on how long the memory is expected to be in use.
// @return A pointer to the beginning of the allocated memory block, or null if the allocation failed.
// @see MOSS_FREE, rcAllocSetCustom
void* rcAlloc(size_t size, rcAllocHint hint);



template<typename TypeToRetrieveAs>
TypeToRetrieveAs* dtGetThenAdvanceBufferPointer(const unsigned char*& buffer, const size_t distanceToAdvance) {
	TypeToRetrieveAs* returnPointer = reinterpret_cast<TypeToRetrieveAs*>(buffer);
	buffer += distanceToAdvance;
	return returnPointer;
}

template<typename TypeToRetrieveAs>
TypeToRetrieveAs* dtGetThenAdvanceBufferPointer(unsigned char*& buffer, const size_t distanceToAdvance) {
	TypeToRetrieveAs* returnPointer = reinterpret_cast<TypeToRetrieveAs*>(buffer);
	buffer += distanceToAdvance;
	return returnPointer;
}




// @}
// @name Computational geometry helper functions.
// @{

// Derives the signed xz-plane area of the triangle ABC, or the relationship of line AB to point C.
//  @param[in]		a		Vertex A. [(x, y, z)]
//  @param[in]		b		Vertex B. [(x, y, z)]
//  @param[in]		c		Vertex C. [(x, y, z)]
// @return The signed xz-plane area of the triangle.
inline float dtTriArea2D(const float* a, const float* b, const float* c)
{
	const float abx = b[0] - a[0];
	const float abz = b[2] - a[2];
	const float acx = c[0] - a[0];
	const float acz = c[2] - a[2];
	return acx*abz - abx*acz;
}

// Determines if two axis-aligned bounding boxes overlap.
//  @param[in]		amin	Minimum bounds of box A. [(x, y, z)]
//  @param[in]		amax	Maximum bounds of box A. [(x, y, z)]
//  @param[in]		bmin	Minimum bounds of box B. [(x, y, z)]
//  @param[in]		bmax	Maximum bounds of box B. [(x, y, z)]
// @return True if the two AABB's overlap.
// @see dtOverlapBounds
inline bool dtOverlapQuantBounds(Float3 amin, Float3 amax, Float3 bmin, Float3 bmax)
{
	bool overlap = true;
	overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
	overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
	overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
	return overlap;
}

// Determines if two axis-aligned bounding boxes overlap.
//  @param[in]		amin	Minimum bounds of box A. [(x, y, z)]
//  @param[in]		amax	Maximum bounds of box A. [(x, y, z)]
//  @param[in]		bmin	Minimum bounds of box B. [(x, y, z)]
//  @param[in]		bmax	Maximum bounds of box B. [(x, y, z)]
// @return True if the two AABB's overlap.
// @see dtOverlapQuantBounds
inline bool dtOverlapBounds(const float* amin, const float* amax,
							const float* bmin, const float* bmax)
{
	bool overlap = true;
	overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
	overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
	overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
	return overlap;
}





// Returns true of status is success.
inline bool dtStatusSucceed(Moss_RecastStatus status) { return (status & DT_SUCCESS) != 0; }

// Returns true of status is failure.
inline bool dtStatusFailed(Moss_RecastStatus status) { return (status & DT_FAILURE) != 0; }

// Returns true of status is in progress.
inline bool dtStatusInProgress(Moss_RecastStatus status) { return (status & DT_IN_PROGRESS) != 0; }

// Returns true if specific detail is set.
inline bool dtStatusDetail(Moss_RecastStatus status, unsigned int detail) { return (status & detail) != 0; }



// Get flags for edge in detail triangle.
// @param[in]	triFlags		The flags for the triangle (last component of detail vertices above).
// @param[in]	edgeIndex		The index of the first vertex of the edge. For instance, if 0,
//								returns flags for edge AB.
inline int dtGetDetailTriEdgeFlags(unsigned char triFlags, int edgeIndex)
{
	return (triFlags >> (edgeIndex * 2)) & 0x3;
}





// Sets the neighbor connection data for the specified direction.
// @param[in]		span			The span to update.
// @param[in]		direction		The direction to set. [Limits: 0 <= value < 4]
// @param[in]		neighborIndex	The index of the neighbor span.
inline void rcSetCon(Moss_RecastCompactSpan& span, int direction, int neighborIndex)
{
	const unsigned int shift = (unsigned int)direction * 6;
	const unsigned int con = span.con;
	span.con = (con & ~(0x3f << shift)) | (((unsigned int)neighborIndex & 0x3f) << shift);
}

// Gets neighbor connection data for the specified direction.
// @param[in]		span		The span to check.
// @param[in]		direction	The direction to check. [Limits: 0 <= value < 4]
// @return The neighbor connection data for the specified direction, or #RC_NOT_CONNECTED if there is no connection.
inline int rcGetCon(const Moss_RecastCompactSpan& span, int direction) {
	const unsigned int shift = (unsigned int)direction * 6;
	return (span.con >> shift) & 0x3f;
}

// Gets the standard width (x-axis) offset for the specified direction.
// @param[in]		direction		The direction. [Limits: 0 <= value < 4]
// @return The width offset to apply to the current cell position to move in the direction.
inline int rcGetDirOffsetX(int direction) {
	static const int offset[4] = { -1, 0, 1, 0, };
	return offset[direction & 0x03];
}

// TODO (graham): Rename this to rcGetDirOffsetZ
// Gets the standard height (z-axis) offset for the specified direction.
// @param[in]		direction		The direction. [Limits: 0 <= value < 4]
// @return The height offset to apply to the current cell position to move in the direction.
inline int rcGetDirOffsetY(int direction) {
	static const int offset[4] = { 0, 1, 0, -1 };
	return offset[direction & 0x03];
}

// Gets the direction for the specified offset. One of x and y should be 0.
// @param[in]		offsetX		The x offset. [Limits: -1 <= value <= 1]
// @param[in]		offsetZ		The z offset. [Limits: -1 <= value <= 1]
// @return The direction that represents the offset.
inline int rcGetDirForOffset(int offsetX, int offsetZ) {
	static const int dirs[5] = { 3, 0, -1, 2, 1 };
	return dirs[((offsetZ + 1) << 1) + offsetX];
}



#endif // MOSS_NAVIGATION_INTERNAL_H
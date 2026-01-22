#pragma once

#include <Moss/Core/Variants/TStaticArray.h>
#include <Moss/Core/Variants/Vector/Float2.h>
#include <Moss/Core/Variants/Math/Swizzle.h>
#include <Moss/Core/Variants/Math/MathTypes.h>

// MMX supports integers (2013) 2xint32 64bit
// AVX2 supports integers (2013) 8xint32 265bit

MOSS_WARNINGS_BEGIN

/// 3 component vector (stored as 4 vectors).
/// Note that we keep the 4th component the same as the 3rd component to avoid divisions by zero when MOSS_FLOATING_POINT_EXCEPTIONS_ENABLED defined
class [[nodiscard]] alignas(MOSS_VECTOR_ALIGNMENT) iVec2 {
public:
	MOSS_OVERRIDE_NEW_DELETE

#if defined(MOSS_SIMD_SSE)
	using Type = __m128i;
#elif defined(MOSS_SIMD_NEON)
	using Type = int32x2_t;
#else
	using Type = Vec4::Type;
#endif

	/* 			Constructors		*/
	iVec2() = default; ///< Intentionally not initialized for performance reasons
	iVec2(const iVec2 &inRHS) = default;
	iVec2 &						operator = (const iVec2 &inRHS) = default;
	explicit MOSS_INLINE		iVec2(Vec4Arg inRHS);
	MOSS_INLINE					iVec2(Type inRHS) : mValue(inRHS)				{ CheckW(); }

	/// Load 3 floats from memory
	explicit MOSS_INLINE		iVec2(const Float2 &inV);

	/// Create a vector from 3 components
	MOSS_INLINE					iVec2(float inX, float inY, float inZ);

	/// Vector with all zeros
	static MOSS_INLINE iVec2		sZero();

	/// Vector with all ones
	static MOSS_INLINE iVec2		sOne();

	/// Vector with all NaN's
	static MOSS_INLINE iVec2		sNaN();

	/// Vectors with the principal axis
	static MOSS_INLINE iVec2		sAxisX()										{ return iVec2(1, 0, 0); }
	static MOSS_INLINE iVec2		sAxisY()										{ return iVec2(0, 1, 0); }
	static MOSS_INLINE iVec2		sAxisZ()										{ return iVec2(0, 0, 1); }

	/// Replicate inV across all components
	static MOSS_INLINE iVec2		sReplicate(float inV);

	/// Load 3 floats from memory (reads 32 bits extra which it doesn't use)
	static MOSS_INLINE iVec2		sLoadFloat2Unsafe(const Float2 &inV);

	/// Return the minimum value of each of the components
	static MOSS_INLINE iVec2		sMin(Vec2Arg inV1, Vec2Arg inV2);

	/// Return the maximum of each of the components
	static MOSS_INLINE iVec2		sMax(Vec2Arg inV1, Vec2Arg inV2);

	/// Clamp a vector between min and max (component wise)
	static MOSS_INLINE iVec2		sClamp(Vec2Arg inV, Vec2Arg inMin, Vec2Arg inMax);

	/// Equals (component wise)
	static MOSS_INLINE UVec4	sEquals(Vec2Arg inV1, Vec2Arg inV2);

	/// Less than (component wise)
	static MOSS_INLINE UVec4	sLess(Vec2Arg inV1, Vec2Arg inV2);

	/// Less than or equal (component wise)
	static MOSS_INLINE UVec4	sLessOrEqual(Vec2Arg inV1, Vec2Arg inV2);

	/// Greater than (component wise)
	static MOSS_INLINE UVec4	sGreater(Vec2Arg inV1, Vec2Arg inV2);

	/// Greater than or equal (component wise)
	static MOSS_INLINE UVec4	sGreaterOrEqual(Vec2Arg inV1, Vec2Arg inV2);

	/// Calculates inMul1 * inMul2 + inAdd
	static MOSS_INLINE iVec2		sFusedMultiplyAdd(Vec2Arg inMul1, Vec2Arg inMul2, Vec2Arg inAdd);

	/// Component wise select, returns inNotSet when highest bit of inControl = 0 and inSet when highest bit of inControl = 1
	static MOSS_INLINE iVec2		sSelect(Vec2Arg inNotSet, Vec2Arg inSet, UVec4Arg inControl);

	/// Logical or (component wise)
	static MOSS_INLINE iVec2		sOr(Vec2Arg inV1, Vec2Arg inV2);

	/// Logical xor (component wise)
	static MOSS_INLINE iVec2		sXor(Vec2Arg inV1, Vec2Arg inV2);

	/// Logical and (component wise)
	static MOSS_INLINE iVec2		sAnd(Vec2Arg inV1, Vec2Arg inV2);

	/// Get unit vector given spherical coordinates
	/// inTheta \f$\in [0, \pi]\f$ is angle between vector and z-axis
	/// inPhi \f$\in [0, 2 \pi]\f$ is the angle in the xy-plane starting from the x axis and rotating counter clockwise around the z-axis
	static MOSS_INLINE iVec2		sUnitSpherical(float inTheta, float inPhi);

	/// A set of vectors uniformly spanning the surface of a unit sphere, usable for debug purposes
	MOSS_EXPORT static const TStaticArray<iVec2, 1026> sUnitSphere;

	/// Get random unit vector
	template <class Random>
	static inline iVec2			sRandom(Random &inRandom);

	/// Get individual components
#if defined(MOSS_SIMD_SSE)
	MOSS_INLINE int			GetX() const									{ return _mm_cvtss_f32(mValue); }
	MOSS_INLINE int			GetY() const									{ return mF32[1]; }
	MOSS_INLINE int			GetZ() const									{ return mF32[2]; }
#elif defined(MOSS_SIMD_NEON)
	MOSS_INLINE int			GetX() const									{ return vgetq_lane_f32(mValue, 0); }
	MOSS_INLINE int			GetY() const									{ return vgetq_lane_f32(mValue, 1); }
	MOSS_INLINE int			GetZ() const									{ return vgetq_lane_f32(mValue, 2); }
#else
	MOSS_INLINE int			GetX() const									{ return mF32[0]; }
	MOSS_INLINE int			GetY() const									{ return mF32[1]; }
	MOSS_INLINE int			GetZ() const									{ return mF32[2]; }
#endif

	/// Set individual components
	MOSS_INLINE void			SetX(float inX)									{ mF32[0] = inX; }
	MOSS_INLINE void			SetY(float inY)									{ mF32[1] = inY; }
	MOSS_INLINE void			SetZ(float inZ)									{ mF32[2] = mF32[3] = inZ; } // Assure Z and W are the same

	/// Set all components
	MOSS_INLINE void			Set(float inX, float inY, float inZ)			{ *this = Vec3(inX, inY, inZ); }

	/// Get float component by index
	MOSS_INLINE float			operator [] (uint inCoordinate) const			{ MOSS_ASSERT(inCoordinate < 3); return mF32[inCoordinate]; }

	/// Set float component by index
	MOSS_INLINE void			SetComponent(uint inCoordinate, float inValue)	{ MOSS_ASSERT(inCoordinate < 3); mF32[inCoordinate] = inValue; mValue = sFixW(mValue); } // Assure Z and W are the same

	/// Comparison
	MOSS_INLINE bool			operator == (Vec2Arg inV2) const;
	MOSS_INLINE bool			operator != (Vec2Arg inV2) const				{ return !(*this == inV2); }

	/// Test if two vectors are close
	MOSS_INLINE bool			IsClose(Vec2Arg inV2, float inMaxDistSq = 1.0e-12f) const;

	/// Test if vector is near zero
	MOSS_INLINE bool			IsNearZero(float inMaxDistSq = 1.0e-12f) const;

	/// Test if vector is normalized
	MOSS_INLINE bool			IsNormalized(float inTolerance = 1.0e-6f) const;

	/// Test if vector contains NaN elements
	MOSS_INLINE bool			IsNaN() const;

	/// Multiply two float vectors (component wise)
	MOSS_INLINE iVec2			operator * (Vec2Arg inV2) const;

	/// Multiply vector with float
	MOSS_INLINE iVec2			operator * (float inV2) const;

	/// Multiply vector with float
	friend MOSS_INLINE iVec2	operator * (float inV1, Vec2Arg inV2);

	/// Divide vector by float
	MOSS_INLINE iVec2			operator / (float inV2) const;

	/// Multiply vector with float
	MOSS_INLINE iVec2&			operator *= (float inV2);

	/// Multiply vector with vector
	MOSS_INLINE iVec2&			operator *= (Vec2Arg inV2);

	/// Divide vector by float
	MOSS_INLINE iVec2&			operator /= (float inV2);

	/// Add two float vectors (component wise)
	MOSS_INLINE iVec2			operator + (Vec2Arg inV2) const;

	/// Add two float vectors (component wise)
	MOSS_INLINE iVec2&			operator += (Vec2Arg inV2);

	/// Negate
	MOSS_INLINE iVec2			operator - () const;

	/// Subtract two float vectors (component wise)
	MOSS_INLINE iVec2			operator - (Vec2Arg inV2) const;

	/// Subtract two float vectors (component wise)
	MOSS_INLINE iVec2&			operator -= (Vec2Arg inV2);

	/// Divide (component wise)
	MOSS_INLINE iVec2			operator / (Vec2Arg inV2) const;

	/// Swizzle the elements in inV
	template<uint32 SwizzleX, uint32 SwizzleY, uint32 SwizzleZ>
	MOSS_INLINE iVec2			Swizzle() const;

	/// Replicate the X component to all components
	MOSS_INLINE Vec4			SplatX() const;

	/// Replicate the Y component to all components
	MOSS_INLINE Vec4			SplatY() const;

	/// Replicate the Z component to all components
	MOSS_INLINE Vec4			SplatZ() const;

	/// Get index of component with lowest value
	MOSS_INLINE int				GetLowestComponentIndex() const;

	/// Get index of component with highest value
	MOSS_INLINE int				GetHighestComponentIndex() const;

	/// Return the absolute value of each of the components
	MOSS_INLINE iVec2			Abs() const;

	/// Reciprocal vector (1 / value) for each of the components
	MOSS_INLINE iVec2			Reciprocal() const;

	/// Cross product
	MOSS_INLINE iVec2			Cross(Vec2Arg inV2) const;
	MOSS_INLINE float			Cross(Vec2Arg a, Vec2Arg b) const;
	MOSS_INLINE iVec2			Cross(float radian) const;

	/// Dot product, returns the dot product in X, Y and Z components
	MOSS_INLINE iVec2			DotV(Vec2Arg inV2) const;

	/// Dot product, returns the dot product in X, Y, Z and W components
	MOSS_INLINE Vec4			DotV4(Vec2Arg inV2) const;

	/// Dot product
	MOSS_INLINE float			Dot(Vec2Arg inV2) const;
	MOSS_INLINE float			Dot(Vec2Arg a, Vec2Arg b) const;

	/// Squared length of vector
	MOSS_INLINE float			LengthSq() const;

	/// Length of vector
	MOSS_INLINE float			Length() const;

	/// Normalize vector
	MOSS_INLINE iVec2			Normalized() const;

	MOSS_INLINE bool 			IsNormalized(iVec2 a);

	MOSS_INLINE iVec2			Lerp(iVec2 a, iVec2 b, float t) const;

	MOSS_INLINE float			Distance(iVec2 a) const;

	/// Normalize vector or return inZeroValue if the length of the vector is zero
	MOSS_INLINE iVec2			NormalizedOr(Vec2Arg inZeroValue) const;

	/// Store 3 floats to memory
	MOSS_INLINE void			StoreFloat2(Float2 *outV) const;

	/// Convert each component from a float to an int
	MOSS_INLINE UVec4			ToInt() const;

	/// Reinterpret Vec3 as a UVec4 (doesn't change the bits)
	MOSS_INLINE UVec4			ReinterpretAsInt() const;

	/// Get the minimum of X, Y and Z
	MOSS_INLINE float			ReduceMin() const;

	/// Get the maximum of X, Y and Z
	MOSS_INLINE float			ReduceMax() const;

	/// Component wise square root
	MOSS_INLINE iVec2			Sqrt() const;

	/// Get normalized vector that is perpendicular to this vector
	MOSS_INLINE iVec2			GetNormalizedPerpendicular() const;

	/// Get vector that contains the sign of each element (returns 1.0f if positive, -1.0f if negative)
	MOSS_INLINE iVec2			GetSign() const;

	/// To String
	friend ostream &			operator << (ostream &inStream, Vec2Arg inV)
	{
		inStream << inV.mF32[0] << ", " << inV.mF32[1] << ", " << inV.mF32[2];
		return inStream;
	}

	/// Internal helper function that checks that W is equal to Z, so e.g. dividing by it should not generate div by 0
	MOSS_INLINE void			CheckW() const;

	/// Internal helper function that ensures that the Z component is replicated to the W component to prevent divisions by zero
	static MOSS_INLINE Type		sFixW(Type inValue);

	union
	{
		Type					mValue;
		int						mF32[4];
	};
};

static_assert(std::is_trivial<iVec2>(), "Is supposed to be a trivial type!");

MOSS_WARNINGS_END

//#include "iVec2.inl"

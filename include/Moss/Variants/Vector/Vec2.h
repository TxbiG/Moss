// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Core/Variants/TStaticArray.h>
#include <Moss/Core/Variants/Vector/Float2.h>
#include <Moss/Core/Variants/Math/Swizzle.h>
#include <Moss/Core/Variants/Math/MathTypes.h>

#include <Moss/Core/Variants/Vector/UVec4.h>
#include <Moss/Core/Variants/Vector/Vec4.h>

MOSS_SUPRESS_WARNINGS_BEGIN

/// 3 component vector (stored as 4 vectors).
/// Note that we keep the 4th component the same as the 3rd component to avoid divisions by zero when MOSS_FLOATING_POINT_EXCEPTIONS_ENABLED defined
class [[nodiscard]] alignas(MOSS_VECTOR_ALIGNMENT) Vec2 {
public:
	MOSS_OVERRIDE_NEW_DELETE

#if defined(MOSS_SIMD_SSE)
	using Type = __m128;
#elif defined(MOSS_SIMD_NEON)
	using Type = float32x2_t;
#else
	using Type = Vec4::Type;
#endif

	/* 			Constructors		*/
	Vec2() = default; ///< Intentionally not initialized for performance reasons
	Vec2(const Vec2 &inRHS) = default;
	Vec2 &						operator = (const Vec2 &inRHS) = default;
	explicit MOSS_INLINE		Vec2(Vec4Arg inRHS);
	MOSS_INLINE					Vec2(Type inRHS) : mValue(inRHS)				{ CheckW(); }

	/// Load 3 floats from memory
	explicit MOSS_INLINE		Vec2(const Float2 &inV);

	/// Create a vector from 3 components
	MOSS_INLINE					Vec2(float inX, float inY);

	/// Vector with all zeros
	static MOSS_INLINE Vec2		sZero();

	/// Vector with all ones
	static MOSS_INLINE Vec2		sOne();

	/// Vector with all NaN's
	static MOSS_INLINE Vec2		sNaN();

	/// Vectors with the principal axis
	static MOSS_INLINE Vec2		sAxisX()										{ return Vec2(1, 0); }
	static MOSS_INLINE Vec2		sAxisY()										{ return Vec2(0, 1); }

	/// Replicate inV across all components
	static MOSS_INLINE Vec2		sReplicate(float inV);

	/// Load 3 floats from memory (reads 32 bits extra which it doesn't use)
	static MOSS_INLINE Vec2		sLoadFloat2Unsafe(const Float2 &inV);

	/// Return the minimum value of each of the components
	static MOSS_INLINE Vec2		sMin(Vec2 inV1, Vec2 inV2);

	/// Return the maximum of each of the components
	static MOSS_INLINE Vec2		sMax(Vec2 inV1, Vec2 inV2);

	/// Clamp a vector between min and max (component wise)
	static MOSS_INLINE Vec2		sClamp(Vec2 inV, Vec2 inMin, Vec2 inMax);

	/// Equals (component wise)
	static MOSS_INLINE UVec4	sEquals(Vec2 inV1, Vec2 inV2);

	/// Less than (component wise)
	static MOSS_INLINE UVec4	sLess(Vec2 inV1, Vec2 inV2);

	/// Less than or equal (component wise)
	static MOSS_INLINE UVec4	sLessOrEqual(Vec2 inV1, Vec2 inV2);

	/// Greater than (component wise)
	static MOSS_INLINE UVec4	sGreater(Vec2 inV1, Vec2 inV2);

	/// Greater than or equal (component wise)
	static MOSS_INLINE UVec4	sGreaterOrEqual(Vec2 inV1, Vec2 inV2);

	/// Calculates inMul1 * inMul2 + inAdd
	static MOSS_INLINE Vec2		sFusedMultiplyAdd(Vec2 inMul1, Vec2 inMul2, Vec2 inAdd);

	/// Component wise select, returns inNotSet when highest bit of inControl = 0 and inSet when highest bit of inControl = 1
	static MOSS_INLINE Vec2		sSelect(Vec2 inNotSet, Vec2 inSet, UVec4Arg inControl);

	/// Logical or (component wise)
	static MOSS_INLINE Vec2		sOr(Vec2 inV1, Vec2 inV2);

	/// Logical xor (component wise)
	static MOSS_INLINE Vec2		sXor(Vec2 inV1, Vec2 inV2);

	/// Logical and (component wise)
	static MOSS_INLINE Vec2		sAnd(Vec2 inV1, Vec2 inV2);

	/// Get unit vector given spherical coordinates
	/// inTheta \f$\in [0, \pi]\f$ is angle between vector and z-axis
	/// inPhi \f$\in [0, 2 \pi]\f$ is the angle in the xy-plane starting from the x axis and rotating counter clockwise around the z-axis
	static MOSS_INLINE Vec2		sUnitSpherical(float inTheta, float inPhi);

	/// A set of vectors uniformly spanning the surface of a unit sphere, usable for debug purposes
	MOSS_EXPORT static const TStaticArray<Vec2, 1026> sUnitSphere;

	/// Get random unit vector
	template <class Random>
	static inline Vec2			sRandom(Random &inRandom);

	inline bool sAllLessOrEqual(Vec2 v1, Vec2 v2) { UVec4 mask = sLessOrEqual(v1, v2); return mask.GetX() && mask.GetY(); } // optional: && mask.GetZ() && mask.GetW()

	inline bool sAllLess(Vec2 v1, Vec2 v2) { UVec4 mask = sLess(v1, v2); return mask.GetX() && mask.GetY(); }

	inline bool sAllGreater(Vec2 v1, Vec2 v2) { UVec4 mask = sGreater(v1, v2); return mask.GetX() && mask.GetY(); }

	inline bool sAllGreaterOrEqual(Vec2 v1, Vec2 v2) { UVec4 mask = sGreaterOrEqual(v1, v2); return mask.GetX() && mask.GetY(); }

	/// Get individual components
#if defined(MOSS_SIMD_SSE)
	MOSS_INLINE float			GetX() const									{ return _mm_cvtss_f32(mValue); }
	MOSS_INLINE float			GetY() const									{ return mF32[1]; }
#elif defined(MOSS_SIMD_NEON)
	MOSS_INLINE float			GetX() const									{ return vgetq_lane_f32(mValue, 0); }
	MOSS_INLINE float			GetY() const									{ return vgetq_lane_f32(mValue, 1); }
#else
	MOSS_INLINE float			GetX() const									{ return mF32[0]; }
	MOSS_INLINE float			GetY() const									{ return mF32[1]; }
#endif

	/// Set individual components
	MOSS_INLINE void			SetX(float inX)									{ mF32[0] = inX; }
	MOSS_INLINE void			SetY(float inY)									{ mF32[1] = inY; }

	/// Set all components
	MOSS_INLINE void			Set(float inX, float inY)			{ *this = Vec2(inX, inY); }

	/// Get float component by index
	MOSS_INLINE float			operator [] (uint inCoordinate) const			{ MOSS_ASSERT(inCoordinate < 3); return mF32[inCoordinate]; }

	/// Set float component by index
	MOSS_INLINE void			SetComponent(uint inCoordinate, float inValue)	{ MOSS_ASSERT(inCoordinate < 3); mF32[inCoordinate] = inValue; mValue = sFixW(mValue); } // Assure Z and W are the same

	/// Comparison
	MOSS_INLINE bool			operator == (Vec2 inV2) const;
	MOSS_INLINE bool			operator != (Vec2 inV2) const				{ return !(*this == inV2); }

	/// Test if two vectors are close
	MOSS_INLINE bool			IsClose(Vec2 inV2, float inMaxDistSq = 1.0e-12f) const;

	/// Test if vector is near zero
	MOSS_INLINE bool			IsNearZero(float inMaxDistSq = 1.0e-12f) const;

	/// Test if vector is normalized
	MOSS_INLINE bool			IsNormalized(float inTolerance = 1.0e-6f) const;

	/// Test if vector contains NaN elements
	MOSS_INLINE bool			IsNaN() const;

	/// Multiply two float vectors (component wise)
	MOSS_INLINE Vec2			operator * (Vec2 inV2) const;

	/// Multiply vector with float
	MOSS_INLINE Vec2			operator * (float inV2) const;

	/// Multiply vector with float
	friend MOSS_INLINE Vec2		operator * (float inV1, Vec2 inV2);

	/// Divide vector by float
	MOSS_INLINE Vec2			operator / (float inV2) const;

	/// Multiply vector with float
	MOSS_INLINE Vec2 &			operator *= (float inV2);

	/// Multiply vector with vector
	MOSS_INLINE Vec2 &			operator *= (Vec2 inV2);

	/// Divide vector by float
	MOSS_INLINE Vec2 &			operator /= (float inV2);

	/// Add two float vectors (component wise)
	MOSS_INLINE Vec2			operator + (Vec2 inV2) const;

	/// Add two float vectors (component wise)
	MOSS_INLINE Vec2 &			operator += (Vec2 inV2);

	/// Negate
	MOSS_INLINE Vec2			operator - () const;

	/// Subtract two float vectors (component wise)
	MOSS_INLINE Vec2			operator - (Vec2 inV2) const;

	/// Subtract two float vectors (component wise)
	MOSS_INLINE Vec2 &			operator -= (Vec2 inV2);

	/// Divide (component wise)
	MOSS_INLINE Vec2			operator / (Vec2 inV2) const;

	/// Swizzle the elements in inV
	template<uint32 SwizzleX, uint32 SwizzleY>
	MOSS_INLINE Vec2			Swizzle() const;

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
	MOSS_INLINE Vec2			Abs() const;

	/// Reciprocal vector (1 / value) for each of the components
	MOSS_INLINE Vec2			Reciprocal() const;

	/// Cross product
	MOSS_INLINE Vec2			Cross(Vec2 inV2) const;
	MOSS_INLINE float			Cross(Vec2 a, Vec2 b) const;
	MOSS_INLINE Vec2			Cross(float radian) const;

	MOSS_INLINE float Cross(const Vec2 &a, const Vec2 &b) { return a.x * b.y - a.y * b.x; }

	MOSS_INLINE Vec2 Cross(const Vec2 &v, float s) { return Vec2( s * v.y, -s * v.x ); }

	MOSS_INLINE Vec2 Cross(float s, const Vec2 &v) { return Vec2( -s * v.y, s * v.x ); }

	/// Dot product, returns the dot product in X, Y and Z components
	MOSS_INLINE Vec2			DotV(Vec2 inV2) const;

	/// Dot product, returns the dot product in X, Y, Z and W components
	MOSS_INLINE Vec4			DotV4(Vec2 inV2) const;

	/// Dot product
	MOSS_INLINE float			Dot(Vec2 inV2) const;

	/// Squared length of vector
	MOSS_INLINE float			LengthSq() const;

	/// Length of vector
	MOSS_INLINE float			Length() const;

	/// Normalize vector
	MOSS_INLINE Vec2			Normalized() const;

	MOSS_INLINE bool 			IsNormalized(Vec2 a);

	MOSS_INLINE Vec2			Lerp(Vec2 a, Vec2 b, float t) const;

	MOSS_INLINE float			Distance(Vec2 a) const;

	/// Normalize vector or return inZeroValue if the length of the vector is zero
	MOSS_INLINE Vec2			NormalizedOr(Vec2 inZeroValue) const;

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
	MOSS_INLINE Vec2			Sqrt() const;

	/// Get normalized vector that is perpendicular to this vector
	MOSS_INLINE Vec2			GetNormalizedPerpendicular() const;

	/// Get vector that contains the sign of each element (returns 1.0f if positive, -1.0f if negative)
	MOSS_INLINE Vec2			GetSign() const;

	/// To String
	friend ostream &			operator << (ostream &inStream, Vec2 inV)
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
		float					mF32[4];
	};
};

static_assert(std::is_trivial<Vec2>(), "Is supposed to be a trivial type!");

MOSS_SUPRESS_WARNINGS_END

#include "Vec2.inl"

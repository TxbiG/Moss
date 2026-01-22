// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Core/Variants/Vector/Float4.h>
#include <Moss/Core/Variants/Math/Swizzle.h>
#include <Moss/Core/Variants/Math/MathTypes.h>

MOSS_SUPRESS_WARNINGS_BEGIN

class [[nodiscard]] alignas(MOSS_VECTOR_ALIGNMENT) Vec4
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	// Underlying vector type
	// using Type = Moss_f32vec4_t;
#if defined(MOSS_SIMD_SSE)
	using Type = __m128;
#elif defined(MOSS_SIMD_NEON)
	using Type = float32x4_t;
#else
	using Type = struct { float mData[4]; };
#endif

	/// Constructors
	Vec4() = default; ///< Intentionally not initialized for performance reasons
	Vec4(const Vec4 &inRHS) = default;

	explicit MOSS_INLINE Vec4(const Vec3 inRHS);
    MOSS_INLINE Vec4(const Vec3 inRHS, float inW);
	Vec4(Type inRHS) : mValue(inRHS){ }

	/// Create a vector from 4 components
	MOSS_INLINE Vec4(float inX, float inY, float inZ, float inW);

	/// Vector with all zeros
	static MOSS_INLINE Vec4		sZero();

	/// Vector with all ones
	static MOSS_INLINE Vec4		sOne();

	/// Vector with all NaN's
	static MOSS_INLINE Vec4		sNaN();

	/// Replicate inV across all components
	static MOSS_INLINE Vec4		sReplicate(float inV);

	/// Load 4 floats from memory
	static MOSS_INLINE Vec4		sLoadFloat4(const Float4 *inV);

	/// Load 4 floats from memory, 16 bytes aligned
	static MOSS_INLINE Vec4		sLoadFloat4Aligned(const Float4 *inV);

	/// Gather 4 floats from memory at inBase + inOffsets[i] * Scale
	template <const int Scale>
	static MOSS_INLINE Vec4		sGatherFloat4(const float *inBase, const UVec4 inOffsets);

	/// Return the minimum value of each of the components
	static MOSS_INLINE Vec4		sMin(const Vec4 inV1, const Vec4 inV2);

	/// Return the maximum of each of the components
	static MOSS_INLINE Vec4		sMax(const Vec4 inV1, const Vec4 inV2);

	/// Equals (component wise)
	static MOSS_INLINE UVec4		sEquals(const Vec4 inV1, const Vec4 inV2);

	/// Less than (component wise)
	static MOSS_INLINE UVec4		sLess(const Vec4 inV1, const Vec4 inV2);

	/// Less than or equal (component wise)
	static MOSS_INLINE UVec4		sLessOrEqual(const Vec4 inV1, const Vec4 inV2);

	/// Greater than (component wise)
	static MOSS_INLINE UVec4		sGreater(const Vec4 inV1, const Vec4 inV2);

	/// Greater than or equal (component wise)
	static MOSS_INLINE UVec4		sGreaterOrEqual(const Vec4 inV1, const Vec4 inV2);

	/// Calculates inMul1 * inMul2 + inAdd
	static MOSS_INLINE Vec4		sFusedMultiplyAdd(const Vec4 inMul1, const Vec4 inMul2, const Vec4 inAdd);

	/// Component wise select, returns inNotSet when highest bit of inControl = 0 and inSet when highest bit of inControl = 1
	static MOSS_INLINE Vec4		sSelect(const Vec4 inNotSet, const Vec4 inSet, const UVec4 inControl);

	/// Logical or (component wise)
	static MOSS_INLINE Vec4		sOr(const Vec4 inV1, const Vec4 inV2);

	/// Logical xor (component wise)
	static MOSS_INLINE Vec4		sXor(const Vec4 inV1, const Vec4 inV2);

	/// Logical and (component wise)
	static MOSS_INLINE Vec4		sAnd(const Vec4 inV1, const Vec4 inV2);

	/// Sort the four elements of ioValue and sort ioIndex at the same time.
	/// Based on a sorting network: http://en.wikipedia.org/wiki/Sorting_network
	static MOSS_INLINE void		sSort4(Vec4 &ioValue, UVec4 &ioIndex);

	/// Reverse sort the four elements of ioValue (highest first) and sort ioIndex at the same time.
	/// Based on a sorting network: http://en.wikipedia.org/wiki/Sorting_network
	static MOSS_INLINE void		sSort4Reverse(Vec4 &ioValue, UVec4 &ioIndex);

	/// Get individual components
#if defined(MOSS_SIMD_SSE)
	MOSS_INLINE float			GetX() const									{ return _mm_cvtss_f32(mValue); }
	MOSS_INLINE float			GetY() const									{ return mF32[1]; }
	MOSS_INLINE float			GetZ() const									{ return mF32[2]; }
	MOSS_INLINE float			GetW() const									{ return mF32[3]; }
#elif defined(MOSS_SIMD_NEON)
	MOSS_INLINE float			GetX() const									{ return vgetq_lane_f32(mValue, 0); }
	MOSS_INLINE float			GetY() const									{ return vgetq_lane_f32(mValue, 1); }
	MOSS_INLINE float			GetZ() const									{ return vgetq_lane_f32(mValue, 2); }
	MOSS_INLINE float			GetW() const									{ return vgetq_lane_f32(mValue, 3); }
#else
	MOSS_INLINE float			GetX() const									{ return mF32[0]; }
	MOSS_INLINE float			GetY() const									{ return mF32[1]; }
	MOSS_INLINE float			GetZ() const									{ return mF32[2]; }
	MOSS_INLINE float			GetW() const									{ return mF32[3]; }
#endif

	/// Set individual components
	MOSS_INLINE void				SetX(float inX)									{ mF32[0] = inX; }
	MOSS_INLINE void				SetY(float inY)									{ mF32[1] = inY; }
	MOSS_INLINE void				SetZ(float inZ)									{ mF32[2] = inZ; }
	MOSS_INLINE void				SetW(float inW)									{ mF32[3] = inW; }

	/// Set all components
	MOSS_INLINE void				Set(float inX, float inY, float inZ, float inW)	{ *this = Vec4(inX, inY, inZ, inW); }


	Vec4 &						operator = (const Vec4 &inRHS) = default;
	/// Get float component by index
	MOSS_INLINE float			operator [] (uint inCoordinate) const			{ MOSS_ASSERT(inCoordinate < 4); return mF32[inCoordinate]; }
	MOSS_INLINE float &			operator [] (uint inCoordinate)					{ MOSS_ASSERT(inCoordinate < 4); return mF32[inCoordinate]; }

	/// Comparison
	MOSS_INLINE bool				operator == (const Vec4 inV2) const;
	MOSS_INLINE bool				operator != (const Vec4 inV2) const			{ return !(*this == inV2); }

	/// Test if two vectors are close
	MOSS_INLINE bool				IsClose(const Vec4 inV2, float inMaxDistSq = 1.0e-12f) const;

	/// Test if vector is normalized
	MOSS_INLINE bool				IsNormalized(float inTolerance = 1.0e-6f) const;

	/// Test if vector contains NaN elements
	MOSS_INLINE bool				IsNaN() const;

	/// Multiply two float vectors (component wise)
	MOSS_INLINE Vec4				operator * (const Vec4 inV2) const;

	/// Multiply vector with float
	MOSS_INLINE Vec4				operator * (float inV2) const;

	/// Multiply vector with float
	friend MOSS_INLINE Vec4		operator * (float inV1, const Vec4 inV2);

	/// Divide vector by float
	MOSS_INLINE Vec4				operator / (float inV2) const;

	/// Multiply vector with float
	MOSS_INLINE Vec4 &			operator *= (float inV2);

	/// Multiply vector with vector
	MOSS_INLINE Vec4 &			operator *= (const Vec4 inV2);

	/// Divide vector by float
	MOSS_INLINE Vec4 &			operator /= (float inV2);

	/// Add two float vectors (component wise)
	MOSS_INLINE Vec4				operator + (const Vec4 inV2) const;

	/// Add two float vectors (component wise)
	MOSS_INLINE Vec4 &			operator += (const Vec4 inV2);

	/// Negate
	MOSS_INLINE Vec4				operator - () const;

	/// Subtract two float vectors (component wise)
	MOSS_INLINE Vec4				operator - (const Vec4 inV2) const;

	/// Subtract two float vectors (component wise)
	MOSS_INLINE Vec4 &			operator -= (const Vec4 inV2);

	/// Divide (component wise)
	MOSS_INLINE Vec4				operator / (const Vec4 inV2) const;

	/// Swizzle the elements in inV
	template<uint32 SwizzleX, uint32 SwizzleY, uint32 SwizzleZ, uint32 SwizzleW>
	MOSS_INLINE Vec4				Swizzle() const;

	/// Replicate the X component to all components
	MOSS_INLINE Vec4				SplatX() const;

	/// Replicate the Y component to all components
	MOSS_INLINE Vec4				SplatY() const;

	/// Replicate the Z component to all components
	MOSS_INLINE Vec4				SplatZ() const;

	/// Replicate the W component to all components
	MOSS_INLINE Vec4				SplatW() const;

	/// Return the absolute value of each of the components
	MOSS_INLINE Vec4				Abs() const;

	/// Reciprocal vector (1 / value) for each of the components
	MOSS_INLINE Vec4				Reciprocal() const;

	/// Dot product, returns the dot product in X, Y and Z components
	MOSS_INLINE Vec4				DotV(const Vec4 inV2) const;

	/// Dot product
	MOSS_INLINE float			Dot(const Vec4 inV2) const;

	/// Squared length of vector
	MOSS_INLINE float			LengthSq() const;

	/// Length of vector
	MOSS_INLINE float			Length() const;

	/// Normalize vector
	MOSS_INLINE Vec4				Normalized() const;

	/// Store 4 floats to memory
	MOSS_INLINE void				StoreFloat4(Float4 *outV) const;

	/// Convert each component from a float to an int
	MOSS_INLINE UVec4			ToInt() const;

	/// Reinterpret Vec4 as a UVec4 (doesn't change the bits)
	MOSS_INLINE UVec4			ReinterpretAsInt() const;

	/// Store if X is negative in bit 0, Y in bit 1, Z in bit 2 and W in bit 3
	MOSS_INLINE int				GetSignBits() const;

	/// Get the minimum of X, Y, Z and W
	MOSS_INLINE float			ReduceMin() const;

	/// Get the maximum of X, Y, Z and W
	MOSS_INLINE float			ReduceMax() const;

	/// Component wise square root
	MOSS_INLINE Vec4				Sqrt() const;

	/// Get vector that contains the sign of each element (returns 1.0f if positive, -1.0f if negative)
	MOSS_INLINE Vec4				GetSign() const;

	/// Calculate the sine and cosine for each element of this vector (input in radians)
	inline void					SinCos(Vec4 &outSin, Vec4 &outCos) const;

	/// Calculate the tangent for each element of this vector (input in radians)
	inline Vec4					Tan() const;

	/// Calculate the arc sine for each element of this vector (returns value in the range [-PI / 2, PI / 2])
	/// Note that all input values will be clamped to the range [-1, 1] and this function will not return NaNs like std::asin
	inline Vec4					ASin() const;

	/// Calculate the arc cosine for each element of this vector (returns value in the range [0, PI])
	/// Note that all input values will be clamped to the range [-1, 1] and this function will not return NaNs like std::acos
	inline Vec4					ACos() const;

	/// Calculate the arc tangent for each element of this vector (returns value in the range [-PI / 2, PI / 2])
	inline Vec4					ATan() const;

	/// Calculate the arc tangent of y / x using the signs of the arguments to determine the correct quadrant (returns value in the range [-PI, PI])
	inline static Vec4			sATan2(const Vec4 inY, const Vec4 inX);

	/// To String
	friend ostream &			operator << (ostream &inStream, const Vec4 inV)
	{
		inStream << inV.mF32[0] << ", " << inV.mF32[1] << ", " << inV.mF32[2] << ", " << inV.mF32[3];
		return inStream;
	}

	union
	{
		Type					mValue;
		float					mF32[4];
	};
};

static_assert(std::is_trivial<Vec4>(), "Is supposed to be a trivial type!");

MOSS_SUPRESS_WARNINGS_END

#include "Vec4.inl"
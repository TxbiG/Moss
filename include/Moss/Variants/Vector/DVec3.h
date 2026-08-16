// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Moss_stdinc.h>
#include <Moss/Variants/Vector/Double3.h>

MOSS_SUPRESS_WARNINGS_BEGIN

/// 3 component vector of doubles (stored as 4 vectors).
/// Note that we keep the 4th component the same as the 3rd component to avoid divisions by zero when MOSS_FLOATING_POINT_EXCEPTIONS_ENABLED defined
class [[nodiscard]] alignas(MOSS_DVECTOR_ALIGNMENT) DVec3 {
public:
	MOSS_OVERRIDE_NEW_DELETE

	// Underlying vector type
#if defined(MOSS_SIMD_AVX)
	using Type = __m256d;
	using TypeArg = __m256d;
#elif defined(MOSS_SIMD_SSE)
	using Type = struct { __m128d mLow, mHigh; };
	using TypeArg = const Type &;
#elif defined(MOSS_SIMD_NEON)
	using Type = float64x2x2_t;
	using TypeArg = const Type &;
#else
	using Type = struct { double mData[4]; };
	using TypeArg = const Type &;
#endif

	// Argument type
	using ArgType = DVec3Arg;

	/// Constructor
								DVec3() = default; ///< Intentionally not initialized for performance reasons
								DVec3(const DVec3& inRHS) = default;
	DVec3 &						operator = (const DVec3 &inRHS) = default;
	MOSS_INLINE explicit		DVec3(Vec3Arg inRHS);
	MOSS_INLINE explicit		DVec3(Vec4Arg inRHS);
	MOSS_INLINE					DVec3(TypeArg inRHS) : mValue(inRHS)			{ CheckW(); }

	/// Create a vector from 3 components
	MOSS_INLINE					DVec3(double inX, double inY, double inZ);

	/// Load 3 doubles from memory
	explicit MOSS_INLINE		DVec3(const Double3 &inV);

	/// Vector with all zeros
	static MOSS_INLINE DVec3	Zero();

	/// Vector with all ones
	static MOSS_INLINE DVec3	One();

	/// Vectors with the principal axis
	static MOSS_INLINE DVec3	AxisX()										{ return DVec3(1, 0, 0); }
	static MOSS_INLINE DVec3	AxisY()										{ return DVec3(0, 1, 0); }
	static MOSS_INLINE DVec3	AxisZ()										{ return DVec3(0, 0, 1); }

	/// Replicate inV across all components
	static MOSS_INLINE DVec3	Replicate(double inV);

	/// Vector with all NaN's
	static MOSS_INLINE DVec3	NaN();

	/// Load 3 doubles from memory (reads 64 bits extra which it doesn't use)
	static MOSS_INLINE DVec3	LoadDouble3Unsafe(const Double3 &inV);

	/// Store 3 doubles to memory
	MOSS_INLINE void			StoreDouble3(Double3 *outV) const;

	/// Convert to float vector 3 rounding to nearest
	MOSS_INLINE explicit		operator Vec3() const;

	/// Prepare to convert to float vector 3 rounding towards zero (returns DVec3 that can be converted to a Vec3 to get the rounding)
	MOSS_INLINE DVec3			PrepareRoundToZero() const;

	/// Prepare to convert to float vector 3 rounding towards positive/negative inf (returns DVec3 that can be converted to a Vec3 to get the rounding)
	MOSS_INLINE DVec3			PrepareRoundToInf() const;

	/// Convert to float vector 3 rounding down
	MOSS_INLINE Vec3			ToVec3RoundDown() const;

	/// Convert to float vector 3 rounding up
	MOSS_INLINE Vec3			ToVec3RoundUp() const;

	/// Return the minimum value of each of the components
	static MOSS_INLINE DVec3	Min(DVec3Arg inV1, DVec3Arg inV2);

	/// Return the maximum of each of the components
	static MOSS_INLINE DVec3	Max(DVec3Arg inV1, DVec3Arg inV2);

	/// Clamp a vector between min and max (component wise)
	static MOSS_INLINE DVec3	Clamp(DVec3Arg inV, DVec3Arg inMin, DVec3Arg inMax);

	/// Equals (component wise)
	static MOSS_INLINE DVec3	Equals(DVec3Arg inV1, DVec3Arg inV2);

	/// Less than (component wise)
	static MOSS_INLINE DVec3	Less(DVec3Arg inV1, DVec3Arg inV2);

	/// Less than or equal (component wise)
	static MOSS_INLINE DVec3	LessOrEqual(DVec3Arg inV1, DVec3Arg inV2);

	/// Greater than (component wise)
	static MOSS_INLINE DVec3	Greater(DVec3Arg inV1, DVec3Arg inV2);

	/// Greater than or equal (component wise)
	static MOSS_INLINE DVec3	GreaterOrEqual(DVec3Arg inV1, DVec3Arg inV2);

	/// Calculates inMul1 * inMul2 + inAdd
	static MOSS_INLINE DVec3	FusedMultiplyAdd(DVec3Arg inMul1, DVec3Arg inMul2, DVec3Arg inAdd);

	/// Component wise select, returns inNotSet when highest bit of inControl = 0 and inSet when highest bit of inControl = 1
	static MOSS_INLINE DVec3	Select(DVec3Arg inNotSet, DVec3Arg inSet, DVec3Arg inControl);

	/// Logical or (component wise)
	static MOSS_INLINE DVec3	Or(DVec3Arg inV1, DVec3Arg inV2);

	/// Logical xor (component wise)
	static MOSS_INLINE DVec3	Xor(DVec3Arg inV1, DVec3Arg inV2);

	/// Logical and (component wise)
	static MOSS_INLINE DVec3	And(DVec3Arg inV1, DVec3Arg inV2);

	/// Store if X is true in bit 0, Y in bit 1, Z in bit 2 and W in bit 3 (true is when highest bit of component is set)
	MOSS_INLINE int				GetTrues() const;

	/// Test if any of the components are true (true is when highest bit of component is set)
	MOSS_INLINE bool			TestAnyTrue() const;

	/// Test if all components are true (true is when highest bit of component is set)
	MOSS_INLINE bool			TestAllTrue() const;

	/// Get individual components
#if defined(MOSS_SIMD_AVX)
	MOSS_INLINE double			GetX() const									{ return _mm_cvtsd_f64(_mm256_castpd256_pd128(mValue)); }
	MOSS_INLINE double			GetY() const									{ return mF64[1]; }
	MOSS_INLINE double			GetZ() const									{ return mF64[2]; }
#elif defined(MOSS_SIMD_SSE)
	MOSS_INLINE double			GetX() const									{ return _mm_cvtsd_f64(mValue.mLow); }
	MOSS_INLINE double			GetY() const									{ return mF64[1]; }
	MOSS_INLINE double			GetZ() const									{ return _mm_cvtsd_f64(mValue.mHigh); }
#elif defined(MOSS_SIMD_NEON)
	MOSS_INLINE double			GetX() const									{ return vgetq_lane_f64(mValue.val[0], 0); }
	MOSS_INLINE double			GetY() const									{ return vgetq_lane_f64(mValue.val[0], 1); }
	MOSS_INLINE double			GetZ() const									{ return vgetq_lane_f64(mValue.val[1], 0); }
#else
	MOSS_INLINE double			GetX() const									{ return mF64[0]; }
	MOSS_INLINE double			GetY() const									{ return mF64[1]; }
	MOSS_INLINE double			GetZ() const									{ return mF64[2]; }
#endif

	/// Set individual components
	MOSS_INLINE void			SetX(double inX)								{ mF64[0] = inX; }
	MOSS_INLINE void			SetY(double inY)								{ mF64[1] = inY; }
	MOSS_INLINE void			SetZ(double inZ)								{ mF64[2] = mF64[3] = inZ; } // Assure Z and W are the same

	/// Set all components
	MOSS_INLINE void			Set(double inX, double inY, double inZ)			{ *this = DVec3(inX, inY, inZ); }

	/// Get double component by index
	MOSS_INLINE double			operator [] (uint32_t inCoordinate) const			{ MOSS_ASSERT(inCoordinate < 3); return mF64[inCoordinate]; }

	/// Set double component by index
	MOSS_INLINE void			SetComponent(uint32_t inCoordinate, double inValue)	{ MOSS_ASSERT(inCoordinate < 3); mF64[inCoordinate] = inValue; mValue = FixW(mValue); } // Assure Z and W are the same

	/// Comparison
	MOSS_INLINE bool			operator == (DVec3Arg inV2) const;
	MOSS_INLINE bool			operator != (DVec3Arg inV2) const				{ return !(*this == inV2); }

	/// Test if two vectors are close
	MOSS_INLINE bool			IsClose(DVec3Arg inV2, double inMaxDistSq = 1.0e-24) const;

	/// Test if vector is near zero
	MOSS_INLINE bool			IsNearZero(double inMaxDistSq = 1.0e-24) const;

	/// Test if length^2 of this vector is within the range [1 - inTolerance, 1 + inTolerance]
	MOSS_INLINE bool			IsNormalized(double inTolerance = 1.0e-12) const;

	/// Test if vector contains NaN elements
	MOSS_INLINE bool			IsNaN() const;

	/// Multiply two double vectors (component wise)
	MOSS_INLINE DVec3			operator * (DVec3Arg inV2) const;

	/// Multiply vector with double
	MOSS_INLINE DVec3			operator * (double inV2) const;

	/// Multiply vector with double
	friend MOSS_INLINE DVec3	operator * (double inV1, DVec3Arg inV2);

	/// Divide vector by double
	MOSS_INLINE DVec3			operator / (double inV2) const;

	/// Multiply vector with double
	MOSS_INLINE DVec3 &			operator *= (double inV2);

	/// Multiply vector with vector
	MOSS_INLINE DVec3 &			operator *= (DVec3Arg inV2);

	/// Divide vector by double
	MOSS_INLINE DVec3 &			operator /= (double inV2);

	/// Add two vectors (component wise)
	MOSS_INLINE DVec3			operator + (Vec3Arg inV2) const;

	/// Add two double vectors (component wise)
	MOSS_INLINE DVec3			operator + (DVec3Arg inV2) const;

	/// Add two vectors (component wise)
	MOSS_INLINE DVec3 &			operator += (Vec3Arg inV2);

	/// Add two double vectors (component wise)
	MOSS_INLINE DVec3 &			operator += (DVec3Arg inV2);

	/// Negate
	MOSS_INLINE DVec3			operator - () const;

	/// Subtract two vectors (component wise)
	MOSS_INLINE DVec3			operator - (Vec3Arg inV2) const;

	/// Subtract two double vectors (component wise)
	MOSS_INLINE DVec3			operator - (DVec3Arg inV2) const;

	/// Subtract two vectors (component wise)
	MOSS_INLINE DVec3 &			operator -= (Vec3Arg inV2);

	/// Subtract two vectors (component wise)
	MOSS_INLINE DVec3 &			operator -= (DVec3Arg inV2);

	/// Divide (component wise)
	MOSS_INLINE DVec3			operator / (DVec3Arg inV2) const;

	/// Return the absolute value of each of the components
	MOSS_INLINE DVec3			Abs() const;

	/// Reciprocal vector (1 / value) for each of the components
	MOSS_INLINE DVec3			Reciprocal() const;

	/// Cross product
	MOSS_INLINE DVec3			Cross(DVec3Arg inV2) const;

	/// Dot product
	MOSS_INLINE double			Dot(DVec3Arg inV2) const;

	/// Squared length of vector
	MOSS_INLINE double			LengthSq() const;

	/// Length of vector
	MOSS_INLINE double			Length() const;

	/// Normalize vector
	MOSS_INLINE DVec3			Normalized() const;

	/// Component wise square root
	MOSS_INLINE DVec3			Sqrt() const;

	/// Get vector that contains the sign of each element (returns 1 if positive, -1 if negative)
	MOSS_INLINE DVec3			GetSign() const;

	/// To String
	friend ostream &			operator << (ostream &inStream, DVec3Arg inV)
	{
		inStream << inV.mF64[0] << ", " << inV.mF64[1] << ", " << inV.mF64[2];
		return inStream;
	}

	/// Internal helper function that checks that W is equal to Z, so e.g. dividing by it should not generate div by 0
	MOSS_INLINE void			CheckW() const;

	/// Internal helper function that ensures that the Z component is replicated to the W component to prevent divisions by zero
	static MOSS_INLINE Type		FixW(TypeArg inValue);

	/// Representations of true and false for boolean operations
	inline static const double cTrue = BitCast<double>(~uint64(0));
	inline static const double cFalse = 0.0;

	union
	{
		Type					mValue;
		double					mF64[4];
	};
};

static_assert(std::is_trivially_default_constructible<DVec3>() && std::is_trivially_copyable<DVec3>(), "Is supposed to be a trivial type!");

MOSS_SUPRESS_WARNINGS_END

#include "DVec3.inl"

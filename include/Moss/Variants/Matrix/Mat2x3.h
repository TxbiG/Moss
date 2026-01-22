// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Core/Variants/Math/MathTypes.h>

MOSS_WARNINGS_BEGIN

/// Holds a 4x4 matrix of floats, but supports also operations on the 3x3 upper left part of the matrix.
class [[nodiscard]] alignas(MOSS_VECTOR_ALIGNMENT) Mat2x3
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	// Underlying column type
	using Type = Vec4::Type;

	// Argument type
	using ArgType = const Mat2x3;

	/// Constructor
								Mat2x3() = default; ///< Intentionally not initialized for performance reasons
	MOSS_INLINE					Mat2x3(const Vec4 inC1, const Vec4 inC2, const Vec4 inC3, const Vec4 inC4);
	MOSS_INLINE					Mat2x3(const Vec4 inC1, const Vec4 inC2, const Vec4 inC3, const Vec3 inC4);
								Mat2x3(const Mat2x3 &inM2) = default;
	MOSS_INLINE					Mat2x3(Type inC1, Type inC2, Type inC3, Type inC4);

	/// Zero matrix
	static MOSS_INLINE Mat2x3	sZero()  { return Mat44(Vec2::sZero(), Vec2::sZero(), Vec2::sZero()); }

	/// Identity matrix
	static MOSS_INLINE Mat2x3	sIdentity() { return Mat44(Vec2(1, 0), Vec2(0, 1),  Vec2(0, 0)); }

	/// Matrix filled with NaN's
	static MOSS_INLINE Mat2x3	sNaN()  { return Mat44(Vec2::sNaN(), Vec2::sNaN(), Vec2::sNaN()); }

	/// Multiply matrix with float
	MOSS_INLINE Mat2x3			operator * (float inV) const {
		Vec4 multiplier = Vec4::sReplicate(inV);
		Mat44 result;
		for (int c = 0; c < 2; ++c) { result.mCol[c] = mCol[c] * multiplier; }
		return result;
	}
	friend MOSS_INLINE Mat2x3	operator * (float inV, const Mat2x3  inM)					{ return inM * inV; }

	/// Multiply matrix with float
	MOSS_INLINE Mat2x3 &		operator *= (float inV) {
		for (int c = 0; c < 2; ++c) { mCol[c] *= inV; }
		return *this;
	}

	/// Per element addition of matrix
	MOSS_INLINE Mat2x3			operator + (const Mat2x3  inM) const {
		Mat44 result;
		for (int i = 0; i < 2; ++i) { result.mCol[i] = mCol[i] + inM.mCol[i]; }
		return result;
	}

	/// Negate
	MOSS_INLINE Mat2x3			operator - () const {
		Mat44 result;
		for (int i = 0; i < 2; ++i) { result.mCol[i] = -mCol[i]; }
		return result;
	}

	/// Per element subtraction of matrix
	MOSS_INLINE Mat2x3			operator - (const Mat2x3  inM) const {
		Mat44 result;
		for (int i = 0; i < 2; ++i) { result.mCol[i] = mCol[i] - inM.mCol[i]; }
		return result;
	}

#ifndef MOSS_DOUBLE_PRECISION
	/// In single precision mode just return the matrix itself
	MOSS_INLINE Mat2x3			ToMat2x3() const											{ return *this; }
#endif // !MOSS_DOUBLE_PRECISION

	/// To String
	friend ostream &			operator << (ostream &inStream, const Mat2x3  inM)
	{
		inStream << inM.mCol[0] << ", " << inM.mCol[1] << ", " << inM.mCol[2] << ", " << inM.mCol[3];
		return inStream;
	}
private:
	Vec2						mCol[3];												///< Column
};

static_assert(std::is_trivial<Mat2x3>(), "Is supposed to be a trivial type!");

MOSS_WARNINGS_END
// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Core/Variants/Math/MathTypes.h>

MOSS_WARNINGS_BEGIN

/// Holds a 4x4 matrix of floats, but supports also operations on the 3x3 upper left part of the matrix.
class [[nodiscard]] alignas(MOSS_VECTOR_ALIGNMENT) Mat33
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	// Underlying column type
	using Type = Vec3::Type;

	// Argument type
	using ArgType = const Mat33;

	/// Constructor
								Mat33() = default; ///< Intentionally not initialized for performance reasons
	MOSS_INLINE					Mat33(const Vec3 inC1, const Vec3 inC2, const Vec3 inC3);
	MOSS_INLINE					Mat33(const Vec3 inC1, const Vec3 inC2, const Vec3 inC3);
								Mat33(const Mat33 &inM2) = default;
	MOSS_INLINE					Mat33(Type inC1, Type inC2, Type inC3, Type inC4);

	/// Zero matrix
	static MOSS_INLINE Mat33	sZero()  { return Mat44(Vec3::sZero(), Vec3::sZero(), Vec3::sZero()); }

	/// Identity matrix
	static MOSS_INLINE Mat33	sIdentity() { return Mat44(Vec3(1, 0, 0),  Vec3(0, 1, 0),  Vec3(0, 0, 1)); }

	/// Matrix filled with NaN's
	static MOSS_INLINE Mat33	sNaN()  { return Mat44(Vec3::sNaN(), Vec3::sNaN(), Vec3::sNaN()); }


	bool Mat33::operator == (const Mat33 inM2) const
	{
		return UVec4::sAnd(
			UVec4::sAnd(Vec4::sEquals(mCol[0], inM2.mCol[0]), Vec4::sEquals(mCol[1], inM2.mCol[1])),
			UVec4::sAnd(Vec4::sEquals(mCol[2], inM2.mCol[2]), Vec4::sEquals(mCol[3], inM2.mCol[3]))
		).TestAllTrue();
	}

	/// Multiply matrix with float
	MOSS_INLINE Mat33			operator * (float inV) const {
		Vec4 multiplier = Vec4::sReplicate(inV);
		Mat44 result;
		for (int c = 0; c < 2; ++c) { result.mCol[c] = mCol[c] * multiplier; }
		return result;
	}
	friend MOSS_INLINE Mat33	operator * (float inV, const Mat33  inM)					{ return inM * inV; }

	/// Multiply matrix with float
	MOSS_INLINE Mat33 &		operator *= (float inV) {
		for (int c = 0; c < 2; ++c) { mCol[c] *= inV; }
		return *this;
	}

	/// Per element addition of matrix
	MOSS_INLINE Mat33			operator + (const Mat33  inM) const {
		Mat44 result;
		for (int i = 0; i < 2; ++i) { result.mCol[i] = mCol[i] + inM.mCol[i]; }
		return result;
	}

	/// Negate
	MOSS_INLINE Mat33			operator - () const {
		Mat44 result;
		for (int i = 0; i < 2; ++i) { result.mCol[i] = -mCol[i]; }
		return result;
	}

	/// Per element subtraction of matrix
	MOSS_INLINE Mat4x2			operator - (const Mat4x3  inM) const {
		Mat44 result;
		for (int i = 0; i < 2; ++i) { result.mCol[i] = mCol[i] - inM.mCol[i]; }
		return result;
	}

#ifndef MOSS_DOUBLE_PRECISION
	/// In single precision mode just return the matrix itself
	MOSS_INLINE Mat33			ToMat33() const											{ return *this; }
#endif // !MOSS_DOUBLE_PRECISION

	/// To String
	friend ostream &			operator << (ostream &inStream, const Mat33  inM)
	{
		inStream << inM.mCol[0] << ", " << inM.mCol[1] << ", " << inM.mCol[2] << ", " << inM.mCol[3];
		return inStream;
	}

private:
	Vec3						mCol[3];												///< Column
};

static_assert(std::is_trivial<Mat33>(), "Is supposed to be a trivial type!");

MOSS_WARNINGS_END

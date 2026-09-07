// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Variants/Math/MathTypes.h>

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
								Mat33() = default; // Intentionally not initialized for performance reasons
	MOSS_INLINE					Mat33(const Vec3 inC1, const Vec3 inC2, const Vec3 inC3);
	MOSS_INLINE					Mat33(const Vec3 inC1, const Vec3 inC2, const Vec3 inC3);
								Mat33(const Mat33 &inM2) = default;
	MOSS_INLINE					Mat33(Type inC1, Type inC2, Type inC3, Type inC4);

	/// Zero matrix
	static MOSS_INLINE Mat33	Zero()  { return Mat33(Vec3::Zero(), Vec3::Zero(), Vec3::Zero()); }

	/// Identity matrix
	static MOSS_INLINE Mat33	Identity() { return Mat33(Vec3(1, 0, 0),  Vec3(0, 1, 0),  Vec3(0, 0, 1)); }

	/// Matrix filled with NaN's
	static MOSS_INLINE Mat33	NaN()  { return Mat33(Vec3::NaN(), Vec3::NaN(), Vec3::NaN()); }


	bool Mat33::operator == (const Mat33 inM2) const
	{
		return UVec4::And(
			UVec4::And(Vec4::Equals(mCol[0], inM2.mCol[0]), Vec4::Equals(mCol[1], inM2.mCol[1])),
			UVec4::And(Vec4::Equals(mCol[2], inM2.mCol[2]), Vec4::Equals(mCol[3], inM2.mCol[3]))).TestAllTrue();
	}

	/// Multiply matrix with float
	MOSS_INLINE Mat33			operator * (float inV) const {
		Mat33 result;
		for (int c = 0; c < 3; ++c) { result.mCol[c] = mCol[c] * inV; }
		return result
	}
	friend MOSS_INLINE Mat33	operator * (float inV, const Mat33  inM)					{ return inM * inV; }

	/// Multiply matrix with float
	MOSS_INLINE Mat33 &		operator *= (float inV) {
		for (int c = 0; c < 3; ++c) { mCol[c] *= inV; }
		return *this;
	}

	/// Per element addition of matrix
	MOSS_INLINE Mat33			operator + (const Mat33  inM) const {
		Mat33 result;
		for (int i = 0; i < 3; ++i) { result.mCol[i] = mCol[i] + inM.mCol[i]; }
		return result;
	}

	/// Negate
	MOSS_INLINE Mat33			operator - () const {
		Mat33 result;
		for (int i = 0; i < 3; ++i) { result.mCol[i] = -mCol[i]; }
		return result;
	}

	/// Per element subtraction of matrix
	MOSS_INLINE Mat4x2			operator - (const Mat4x3  inM) const {
		Mat33 result;
		for (int i = 0; i < 3; ++i) { result.mCol[i] = mCol[i] - inM.mCol[i]; }
		return result;
	}

	/// Getters for Columns
	MOSS_INLINE Vec3 GetColumn0() const { return mCol[0]; }
	MOSS_INLINE Vec3 GetColumn1() const { return mCol[1]; }
	MOSS_INLINE Vec3 GetColumn2() const { return mCol[2]; }

	MOSS_INLINE Mat33 PreTranslated(const Vec2 inV) const
	{
		Mat33 result = *this;
		result.mCol[2] = mCol[0] * inV.GetX() + mCol[1] * inV.GetY() + mCol[2];
		return result;
	}

	/// Transpose a 3x3 matrix (which is its inverse if it's a pure rotation matrix)
	MOSS_INLINE Mat33 Transposed() const
	{
		return Mat33(
			Vec3(mCol[0].GetX(), mCol[1].GetX(), mCol[2].GetX()),
			Vec3(mCol[0].GetY(), mCol[1].GetY(), mCol[2].GetY()),
			Vec3(mCol[0].GetZ(), mCol[1].GetZ(), mCol[2].GetZ())
		);
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
	Vec3 mCol[3];	// Columns 0, 1, 2
};

static_assert(std::is_trivial<Mat33>(), "Is supposed to be a trivial type!");

MOSS_WARNINGS_END

// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2022 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Core/Variants/Math/MathTypes.h>

MOSS_SUPRESS_WARNINGS_BEGIN

// Remove Lookat and etc

/// Holds a 4x4 matrix of floats with the last column consisting of doubles
class [[nodiscard]] alignas(MOSS_DVECTOR_ALIGNMENT) DMat4x3
{
public:
	// MOSS_OVERRIDE_NEW_DELETE < not needed yet

	// Underlying column type
	using Type = Vec4::Type;
	using DType = DVec3::Type;
	using DTypeArg = DVec3::TypeArg;

	// Argument type
	using ArgType = DMat44Arg;

	/// Constructor
								DMat44() = default; ///< Intentionally not initialized for performance reasons
	MOSS_INLINE					DMat44(Vec4Arg inC1, Vec4Arg inC2, Vec4Arg inC3, DVec3Arg inC4);
								DMat44(const DMat44 &inM2) = default;
	DMat44 &					operator = (const DMat44 &inM2) = default;
	MOSS_INLINE explicit		DMat44(Mat44Arg inM);
	MOSS_INLINE					DMat44(Mat44Arg inRot, DVec3Arg inT);
	MOSS_INLINE					DMat44(Type inC1, Type inC2, Type inC3, DTypeArg inC4);

	/// Zero matrix
	static MOSS_INLINE DMat44	sZero();

	/// Identity matrix
	static MOSS_INLINE DMat44	sIdentity();

	/// Comparison
	MOSS_INLINE bool				operator == (DMat44Arg inM2) const;
	MOSS_INLINE bool				operator != (DMat44Arg inM2) const						{ return !(*this == inM2); }

	/// Test if two matrices are close
	MOSS_INLINE bool				IsClose(DMat44Arg inM2, float inMaxDistSq = 1.0e-12f) const;

	/// Multiply matrix by matrix
	MOSS_INLINE DMat44			operator * (Mat44Arg inM) const;

	/// Multiply matrix by matrix
	MOSS_INLINE DMat44			operator * (DMat44Arg inM) const;

	/// Multiply vector by matrix
	MOSS_INLINE DVec3			operator * (Vec3Arg inV) const;

	/// Multiply vector by matrix
	MOSS_INLINE DVec3			operator * (DVec3Arg inV) const;

	/// To String
	friend ostream &			operator << (ostream &inStream, DMat44Arg inM)
	{
		inStream << inM.mCol[0] << ", " << inM.mCol[1] << ", " << inM.mCol[2] << ", " << inM.mCol3;
		return inStream;
	}

private:
	Vec4						mCol[3];												///< Rotation columns
	DVec3						mCol3;													///< Translation column, 4th element is assumed to be 1
};

DMat4x3 transpose();
DMat4x3 frustum(left, right, bottom, top, near, far);
DMat4x3 ortho(left, right, bottom, top, near, far);
DMat4x3 perspective(fovy, aspect, near, far);


static_assert(is_trivial<DMat44>(), "Is supposed to be a trivial type!");

MOSS_SUPRESS_WARNINGS_END

#include "DMat44.inl"

// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2022 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Variants/Math/MathTypes.h>

MOSS_SUPRESS_WARNINGS_BEGIN

/// Holds a 4x4 matrix of floats with the last column consisting of doubles
class [[nodiscard]] alignas(max(MOSS_VECTOR_ALIGNMENT, MOSS_DVECTOR_ALIGNMENT)) DMat44 {
public:
	JPH_OVERRIDE_NEW_DELETE

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
	static MOSS_INLINE DMat44	Zero();

	/// Identity matrix
	static MOSS_INLINE DMat44	Identity();

	/// Rotate from quaternion
	static MOSS_INLINE DMat44	Rotation(QuatArg inQuat)								{ return DMat44(Mat44::Rotation(inQuat), DVec3::Zero()); }

	/// Get matrix that translates
	static MOSS_INLINE DMat44	Translation(DVec3Arg inV)								{ return DMat44(Vec4(1, 0, 0, 0), Vec4(0, 1, 0, 0), Vec4(0, 0, 1, 0), inV); }

	/// Get matrix that rotates and translates
	static MOSS_INLINE DMat44	RotationTranslation(QuatArg inR, DVec3Arg inT)			{ return DMat44(Mat44::Rotation(inR), inT); }

	/// Get inverse matrix of sRotationTranslation
	static MOSS_INLINE DMat44	InverseRotationTranslation(QuatArg inR, DVec3Arg inT);

	/// Get matrix that scales (produces a matrix with (inV, 1) on its diagonal)
	static MOSS_INLINE DMat44	Scale(Vec3Arg inV)										{ return DMat44(Mat44::Scale(inV), DVec3::Zero()); }

	/// Convert to Mat44 rounding to nearest
	MOSS_INLINE Mat44			ToMat44() const											{ return Mat44(mCol[0], mCol[1], mCol[2], Vec3(mCol3)); }

	/// Comparison
	MOSS_INLINE bool			operator == (DMat44Arg inM2) const;
	MOSS_INLINE bool			operator != (DMat44Arg inM2) const						{ return !(*this == inM2); }

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

	/// Multiply vector by only 3x3 part of the matrix
	MOSS_INLINE Vec3			Multiply3x3(Vec3Arg inV) const							{ return GetRotation().Multiply3x3(inV); }

	/// Multiply vector by only 3x3 part of the matrix
	MOSS_INLINE DVec3			Multiply3x3(DVec3Arg inV) const;

	/// Multiply vector by only 3x3 part of the transpose of the matrix (\f$result = this^T \: inV\f$)
	MOSS_INLINE Vec3			Multiply3x3Transposed(Vec3Arg inV) const				{ return GetRotation().Multiply3x3Transposed(inV); }

	/// Scale a matrix: result = this * Mat44::sScale(inScale)
	MOSS_INLINE DMat44			PreScaled(Vec3Arg inScale) const;

	/// Scale a matrix: result = Mat44::sScale(inScale) * this
	MOSS_INLINE DMat44			PostScaled(Vec3Arg inScale) const;

	/// Pre multiply by translation matrix: result = this * Mat44::sTranslation(inTranslation)
	MOSS_INLINE DMat44			PreTranslated(Vec3Arg inTranslation) const;

	/// Pre multiply by translation matrix: result = this * Mat44::sTranslation(inTranslation)
	MOSS_INLINE DMat44			PreTranslated(DVec3Arg inTranslation) const;

	/// Post multiply by translation matrix: result = Mat44::sTranslation(inTranslation) * this (i.e. add inTranslation to the 4-th column)
	MOSS_INLINE DMat44			PostTranslated(Vec3Arg inTranslation) const;

	/// Post multiply by translation matrix: result = Mat44::sTranslation(inTranslation) * this (i.e. add inTranslation to the 4-th column)
	MOSS_INLINE DMat44			PostTranslated(DVec3Arg inTranslation) const;

	/// Access to the columns
	MOSS_INLINE Vec3			GetAxisX() const										{ return Vec3(mCol[0]); }
	MOSS_INLINE void			SetAxisX(Vec3Arg inV)									{ mCol[0] = Vec4(inV, 0.0f); }
	MOSS_INLINE Vec3			GetAxisY() const										{ return Vec3(mCol[1]); }
	MOSS_INLINE void			SetAxisY(Vec3Arg inV)									{ mCol[1] = Vec4(inV, 0.0f); }
	MOSS_INLINE Vec3			GetAxisZ() const										{ return Vec3(mCol[2]); }
	MOSS_INLINE void			SetAxisZ(Vec3Arg inV)									{ mCol[2] = Vec4(inV, 0.0f); }
	MOSS_INLINE DVec3			GetTranslation() const									{ return mCol3; }
	MOSS_INLINE void			SetTranslation(DVec3Arg inV)							{ mCol3 = inV; }
	MOSS_INLINE Vec3			GetColumn3(uint inCol) const							{ JPH_ASSERT(inCol < 3); return Vec3(mCol[inCol]); }
	MOSS_INLINE void			SetColumn3(uint inCol, Vec3Arg inV)						{ JPH_ASSERT(inCol < 3); mCol[inCol] = Vec4(inV, 0.0f); }
	MOSS_INLINE Vec4			GetColumn4(uint inCol) const							{ JPH_ASSERT(inCol < 3); return mCol[inCol]; }
	MOSS_INLINE void			SetColumn4(uint inCol, Vec4Arg inV)						{ JPH_ASSERT(inCol < 3); mCol[inCol] = inV; }

	/// Transpose 3x3 subpart of matrix
	MOSS_INLINE Mat44			Transposed3x3() const									{ return GetRotation().Transposed3x3(); }

	/// Inverse 4x4 matrix
	MOSS_INLINE DMat44			Inversed() const;

	/// Inverse 4x4 matrix when it only contains rotation and translation
	MOSS_INLINE DMat44			InversedRotationTranslation() const;

	/// Get rotation part only (note: retains the first 3 values from the bottom row)
	MOSS_INLINE Mat44			GetRotation() const										{ return Mat44(mCol[0], mCol[1], mCol[2], Vec4(0, 0, 0, 1)); }

	/// Updates the rotation part of this matrix (the first 3 columns)
	MOSS_INLINE void			SetRotation(Mat44Arg inRotation);

	/// Convert to quaternion
	MOSS_INLINE Quat			GetQuaternion() const									{ return GetRotation().GetQuaternion(); }

	/// Get matrix that transforms a direction with the same transform as this matrix (length is not preserved)
	MOSS_INLINE Mat44			GetDirectionPreservingMatrix() const					{ return GetRotation().Inversed3x3().Transposed3x3(); }

	/// Works identical to Mat44::Decompose
	MOSS_INLINE DMat44			Decompose(Vec3 &outScale) const							{ return DMat44(GetRotation().Decompose(outScale), mCol3); }

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

static_assert(std::is_trivially_default_constructible<DMat44>() && std::is_trivially_copyable<DMat44>(), "Is supposed to be a trivial type!");

MOSS_SUPRESS_WARNINGS_END

#include "DMat44.inl"

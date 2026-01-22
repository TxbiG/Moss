// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2022 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Core/Variants/Math/MathTypes.h>

MOSS_SUPRESS_WARNINGS_BEGIN

/// Holds a 4x4 matrix of floats with the last column consisting of doubles
class [[nodiscard]] alignas(MOSS_DVECTOR_ALIGNMENT) DMat44
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
	MOSS_INLINE explicit		DMat44(Mat44Arg inM);
	MOSS_INLINE					DMat44(Mat44Arg inRot, DVec3Arg inT);
	MOSS_INLINE					DMat44(Type inC1, Type inC2, Type inC3, DTypeArg inC4);

	/// Zero matrix
	static MOSS_INLINE DMat44	sZero();

	/// Identity matrix
	static MOSS_INLINE DMat44	sIdentity();

	/// Rotate from quaternion
	static MOSS_INLINE DMat44	sRotation(QuatArg inQuat)								{ return DMat44(Mat44::sRotation(inQuat), DVec3::sZero()); }

	/// Get matrix that translates
	static MOSS_INLINE DMat44	sTranslation(DVec3Arg inV)								{ return DMat44(Vec4(1, 0, 0, 0), Vec4(0, 1, 0, 0), Vec4(0, 0, 1, 0), inV); }

	/// Get matrix that rotates and translates
	static MOSS_INLINE DMat44	sRotationTranslation(QuatArg inR, DVec3Arg inT)			{ return DMat44(Mat44::sRotation(inR), inT); }

	/// Get inverse matrix of sRotationTranslation
	static MOSS_INLINE DMat44	sInverseRotationTranslation(QuatArg inR, DVec3Arg inT);

	/// Get matrix that scales (produces a matrix with (inV, 1) on its diagonal)
	static MOSS_INLINE DMat44	sScale(Vec3Arg inV)										{ return DMat44(Mat44::sScale(inV), DVec3::sZero()); }

	/// Convert to Mat44 rounding to nearest
	MOSS_INLINE Mat44			ToMat44() const											{ return Mat44(mCol[0], mCol[1], mCol[2], Vec3(mCol3)); }


	/// Test if two matrices are close
	MOSS_INLINE bool			IsClose(DMat44Arg inM2, float inMaxDistSq = 1.0e-12f) const;

	DMat44 &					operator = (DMat44Arg &inM2) = default;
	/// Comparison
	MOSS_INLINE bool			operator == (DMat44Arg  inM2) const;
	MOSS_INLINE bool			operator != (DMat44Arg inM2) const						{ return !(*this == inM2); }

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
	MOSS_INLINE Vec3			GetColumn3(uint inCol) const							{ MOSS_ASSERT(inCol < 3); return Vec3(mCol[inCol]); }
	MOSS_INLINE void			SetColumn3(uint inCol, Vec3Arg inV)						{ MOSS_ASSERT(inCol < 3); mCol[inCol] = Vec4(inV, 0.0f); }
	MOSS_INLINE Vec4			GetColumn4(uint inCol) const							{ MOSS_ASSERT(inCol < 3); return mCol[inCol]; }
	MOSS_INLINE void			SetColumn4(uint inCol, Vec4Arg inV)						{ MOSS_ASSERT(inCol < 3); mCol[inCol] = inV; }

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

/*
static DMat44 lookAtLH(const DVec3& eye, const DVec3& center, const DVec3& up) {
	DVec3 f = (center - eye).normalized();
    DVec3 r = up.cross(f).normalized();
    DVec3 u = f.cross(r);

    DMat44 result = identity();
    result.m[0][0] = r.x; result.m[0][1] = r.y; result.m[0][2] = r.z;
    result.m[1][0] = u.x; result.m[1][1] = u.y; result.m[1][2] = u.z;
    result.m[2][0] = f.x; result.m[2][1] = f.y; result.m[2][2] = f.z;
    result.m[0][3] = -r.dot(eye);
    result.m[1][3] = -u.dot(eye);
    result.m[2][3] = -f.dot(eye);
    return result;
}
static DMat44 lookAtRH(const DVec3& eye, const DVec3& center, const DVec3& up) {
	DVec3 f = (center - eye).normalized();
    DVec3 r = f.cross(up).normalized();
    DVec3 u = r.cross(f);

    DMat44 result = identity();
    result.m[0][0] = r.x; result.m[0][1] = r.y; result.m[0][2] = r.z;
    result.m[1][0] = u.x; result.m[1][1] = u.y; result.m[1][2] = u.z;
    result.m[2][0] = -f.x; result.m[2][1] = -f.y; result.m[2][2] = -f.z;
    result.m[0][3] = -r.dot(eye);
    result.m[1][3] = -u.dot(eye);
    result.m[2][3] = f.dot(eye);
    return result;
}

static DMat44 lookAt(const DVec3& eye, const DVec3& center, const DVec3& up) {
#ifdef MOSS_USE_OPENGL || MOSS_USE_OPENGLES
    return lookAtLH(eye, center, up);
#else
    return lookAtRH(eye, center, up);
#endif
}

// === Orthographic Projections ===
static DMat44 orthoLH(double left, double right, double bottom, double top, double near, double far) {
	DMat44 result = identity();
    result.m[0][0] = 2.0f / (right - left);
    result.m[1][1] = 2.0f / (top - bottom);
    result.m[2][2] = -2.0f / (far - near);
    result.m[0][3] = -(right + left) / (right - left);
    result.m[1][3] = -(top + bottom) / (top - bottom);
    result.m[2][3] = -(far + near) / (far - near);
    return result;
}
static DMat44 orthoRH(double left, double right, double bottom, double top, double near, double far) {
	DMat44 result = identity();
    result.m[0][0] = 2.0f / (right - left);
    result.m[1][1] = 2.0f / (top - bottom);
    result.m[2][2] = 2.0f / (near - far);
    result.m[0][3] = -(right + left) / (right - left);
    result.m[1][3] = -(top + bottom) / (top - bottom);
    result.m[2][3] = -(far + near) / (near - far);
    return result;
}

static DMat44 ortho(double left, double right, double bottom, double top, double near, double far) {
#ifdef MOSS_USE_OPENGL || MOSS_USE_OPENGLES
    return orthoLH(left, right, bottom, top, near, far);
#else
    return orthoRH(left, right, bottom, top, near, far);
#endif
}

// === Perspective Projections ===
static DMat44 perspectiveLH(double fovyRadians, double aspect, double near, double far) {
    double tanHalfFovy = std::tan(fovyRadians / 2.0f);
    DMat44 result = {};
    result.m[0][0] = 1.0f / (aspect * tanHalfFovy);
    result.m[1][1] = 1.0f / tanHalfFovy;
    result.m[2][2] = (far + near) / (far - near);
    result.m[2][3] = -2.0f * far * near / (far - near);
    result.m[3][2] = 1.0f;
    return result;
}

static DMat44 perspectiveRH(double fovyRadians, double aspect, double near, double far) {
    double tanHalfFovy = std::tan(fovyRadians / 2.0f);
    DMat44 result = {};
    result.m[0][0] = 1.0f / (aspect * tanHalfFovy);
    result.m[1][1] = 1.0f / tanHalfFovy;
    result.m[2][2] = -(far + near) / (far - near);
    result.m[2][3] = -2.0f * far * near / (far - near);
    result.m[3][2] = -1.0f;
    return result;
}

static DMat44 perspective(double fovyRadians, double aspect, double near, double far) {
#ifdef MOSS_USE_OPENGL || MOSS_USE_OPENGLES
    return perspectiveLH(fovyRadians, aspect, near, far);
#else
    return perspectiveRH(fovyRadians, aspect, near, far);
#endif
}


static DMat44 frustumLH(double left, double right, double bottom, double top, double near, double far) {
    DMat44 result = {};
    result.m[0][0] = 2 * near / (right - left);
    result.m[1][1] = 2 * near / (top - bottom);
    result.m[0][2] = (right + left) / (right - left);
    result.m[1][2] = (top + bottom) / (top - bottom);
    result.m[2][2] = (far + near) / (far - near);
    result.m[2][3] = -2 * far * near / (far - near);
    result.m[3][2] = 1;
    return result;
}

static DMat44 frustumRH(double left, double right, double bottom, double top, double near, double far) {
    DMat44 result = {};
    result.m[0][0] = 2 * near / (right - left);
    result.m[1][1] = 2 * near / (top - bottom);
    result.m[0][2] = (right + left) / (right - left);
    result.m[1][2] = (top + bottom) / (top - bottom);
    result.m[2][2] = -(far + near) / (far - near);
    result.m[2][3] = -2 * far * near / (far - near);
    result.m[3][2] = -1;
    return result;
}

static DMat44 frustum(double left, double right, double bottom, double top, double near, double far) {
#ifdef MOSS_USE_OPENGL || MOSS_USE_OPENGLES
    return frustumLH(left, right, bottom, top, near, far);
#else
    return frustumRH(left, right, bottom, top, near, far);
#endif
}
*/

static_assert(std::is_trivial<DMat44>(), "Is supposed to be a trivial type!");

MOSS_SUPRESS_WARNINGS_END

#include "DMat44.inl"

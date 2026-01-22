// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Core/Variants/Math/MathTypes.h>

MOSS_SUPRESS_WARNINGS_BEGIN

/// Holds a 4x4 matrix of floats, but supports also operations on the 3x3 upper left part of the matrix.
class [[nodiscard]] alignas(MOSS_VECTOR_ALIGNMENT) Mat44
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	// Underlying column type
	using Type = Vec4::Type;

	// Argument type
	using ArgType = const Mat44;

	/// Constructor
								Mat44() = default; ///< Intentionally not initialized for performance reasons
	MOSS_INLINE					Mat44(const Vec4 inC1, const Vec4 inC2, const Vec4 inC3, const Vec4 inC4);
	MOSS_INLINE					Mat44(const Vec4 inC1, const Vec4 inC2, const Vec4 inC3, const Vec3 inC4);
								Mat44(const Mat44 &inM2) = default;
	Mat44 &						operator = (const Mat44 &inM2) = default;
	MOSS_INLINE					Mat44(Type inC1, Type inC2, Type inC3, Type inC4);

	/// Zero matrix
	static MOSS_INLINE Mat44	sZero();

	/// Identity matrix
	static MOSS_INLINE Mat44	sIdentity();

	/// Matrix filled with NaN's
	static MOSS_INLINE Mat44	sNaN();

	/// Load 16 floats from memory
	static MOSS_INLINE Mat44	sLoadFloat4x4(const Float4 *inV);

	/// Load 16 floats from memory, 16 bytes aligned
	static MOSS_INLINE Mat44	sLoadFloat4x4Aligned(const Float4 *inV);

	/// Rotate around X, Y or Z axis (angle in radians)
	static MOSS_INLINE Mat44	sRotationX(float inX);
	static MOSS_INLINE Mat44	sRotationY(float inY);
	static MOSS_INLINE Mat44	sRotationZ(float inZ);

	/// Rotate around arbitrary axis
	static MOSS_INLINE Mat44	sRotation(const Vec3 inAxis, float inAngle);

	/// Rotate from quaternion
	static MOSS_INLINE Mat44	sRotation(const Quat inQuat);

	/// Get matrix that translates
	static MOSS_INLINE Mat44	sTranslation(const Vec3 inV);

	/// Get matrix that rotates and translates
	static MOSS_INLINE Mat44	sRotationTranslation(const Quat inR, const Vec3 inT);

	/// Get inverse matrix of sRotationTranslation
	static MOSS_INLINE Mat44	sInverseRotationTranslation(const Quat inR, const Vec3 inT);

	/// Get matrix that scales uniformly
	static MOSS_INLINE Mat44	sScale(float inScale);

	/// Get matrix that scales (produces a matrix with (inV, 1) on its diagonal)
	static MOSS_INLINE Mat44	sScale(const Vec3 inV);

	/// Get outer product of inV and inV2 (equivalent to \f$inV1 \otimes inV2\f$)
	static MOSS_INLINE Mat44	sOuterProduct(const Vec3 inV1, const Vec3 inV2);

	/// Get matrix that represents a cross product \f$A \times B = \text{sCrossProduct}(A) \: B\f$
	static MOSS_INLINE Mat44	sCrossProduct(const Vec3 inV);

	/// Returns matrix ML so that \f$ML(q) \: p = q \: p\f$ (where p and q are quaternions)
	static MOSS_INLINE Mat44	sQuatLeftMultiply(const Quat inQ);

	/// Returns matrix MR so that \f$MR(q) \: p = p \: q\f$ (where p and q are quaternions)
	static MOSS_INLINE Mat44	sQuatRightMultiply(const Quat inQ);

	/// Returns a look at matrix that transforms from world space to view space
	/// @param inPos Position of the camera
	/// @param inTarget Target of the camera
	/// @param inUp Up vector
	static MOSS_INLINE Mat44	sLookAt(Vec3 position, Vec3 target, Vec3 up);

	/// Returns a right-handed perspective projection matrix
	static MOSS_INLINE Mat44	sPerspective(float inFovY, float aspect, float near, float far);

	/// Get float component by element index
	MOSS_INLINE float			operator () (uint inRow, uint inColumn) const			{ MOSS_ASSERT(inRow < 4); MOSS_ASSERT(inColumn < 4); return mCol[inColumn].mF32[inRow]; }
	MOSS_INLINE float &			operator () (uint inRow, uint inColumn)					{ MOSS_ASSERT(inRow < 4); MOSS_ASSERT(inColumn < 4); return mCol[inColumn].mF32[inRow]; }

	/// Comparison
	MOSS_INLINE bool			operator == (const Mat44  inM2) const;
	MOSS_INLINE bool			operator != (const Mat44  inM2) const						{ return !(*this == inM2); }

	/// Test if two matrices are close
	MOSS_INLINE bool			IsClose(const Mat44  inM2, float inMaxDistSq = 1.0e-12f) const;

	/// Multiply matrix by matrix
	MOSS_INLINE Mat44			operator * (const Mat44 inM) const;

	/// Multiply vector by matrix
	MOSS_INLINE Vec3			operator * (const Vec3 inV) const;
	MOSS_INLINE Vec4			operator * (const Vec4 inV) const;

	/// Multiply vector by only 3x3 part of the matrix
	MOSS_INLINE Vec3			Multiply3x3(const Vec3 inV) const;

	/// Multiply vector by only 3x3 part of the transpose of the matrix (\f$result = this^T \: inV\f$)
	MOSS_INLINE Vec3			Multiply3x3Transposed(const Vec3 inV) const;

	/// Multiply 3x3 matrix by 3x3 matrix
	MOSS_INLINE Mat44			Multiply3x3(const Mat44  inM) const;

	/// Multiply transpose of 3x3 matrix by 3x3 matrix (\f$result = this^T \: inM\f$)
	MOSS_INLINE Mat44			Multiply3x3LeftTransposed(const Mat44  inM) const;

	/// Multiply 3x3 matrix by the transpose of a 3x3 matrix (\f$result = this \: inM^T\f$)
	MOSS_INLINE Mat44			Multiply3x3RightTransposed(const Mat44  inM) const;

	/// Multiply matrix with float
	MOSS_INLINE Mat44			operator * (float inV) const;
	friend MOSS_INLINE Mat44	operator * (float inV, const Mat44  inM)					{ return inM * inV; }
	
	/// Multiply matrix with float
	MOSS_INLINE Mat44 &			operator *= (float inV);

	/// Per element addition of matrix
	MOSS_INLINE Mat44			operator + (const Mat44  inM) const;

	/// Negate
	MOSS_INLINE Mat44			operator - () const;

	/// Per element subtraction of matrix
	MOSS_INLINE Mat44			operator - (const Mat44  inM) const;

	/// Per element addition of matrix
	MOSS_INLINE Mat44 &			operator += (const Mat44  inM);

	/// Access to the columns
	MOSS_INLINE Vec3			GetAxisX() const										{ return Vec3(mCol[0]); }
	MOSS_INLINE void			SetAxisX(const Vec3 inV)								{ mCol[0] = Vec4(inV, 0.0f); }
	MOSS_INLINE Vec3			GetAxisY() const										{ return Vec3(mCol[1]); }
	MOSS_INLINE void			SetAxisY(const Vec3 inV)								{ mCol[1] = Vec4(inV, 0.0f); }
	MOSS_INLINE Vec3			GetAxisZ() const										{ return Vec3(mCol[2]); }
	MOSS_INLINE void			SetAxisZ(const Vec3 inV)								{ mCol[2] = Vec4(inV, 0.0f); }
	MOSS_INLINE Vec3			GetTranslation() const									{ return Vec3(mCol[3]); }
	MOSS_INLINE void			SetTranslation(const Vec3 inV)							{ mCol[3] = Vec4(inV, 1.0f); }
	MOSS_INLINE Vec3			GetDiagonal3() const									{ return Vec3(mCol[0][0], mCol[1][1], mCol[2][2]); }
	MOSS_INLINE void			SetDiagonal3(const Vec3 inV)							{ mCol[0][0] = inV.GetX(); mCol[1][1] = inV.GetY(); mCol[2][2] = inV.GetZ(); }
	MOSS_INLINE Vec4			GetDiagonal4() const									{ return Vec4(mCol[0][0], mCol[1][1], mCol[2][2], mCol[3][3]); }
	MOSS_INLINE void			SetDiagonal4(const Vec4 inV)							{ mCol[0][0] = inV.GetX(); mCol[1][1] = inV.GetY(); mCol[2][2] = inV.GetZ(); mCol[3][3] = inV.GetW(); }
	MOSS_INLINE Vec3			GetColumn3(uint inCol) const							{ MOSS_ASSERT(inCol < 4); return Vec3(mCol[inCol]); }
	MOSS_INLINE void			SetColumn3(uint inCol, const Vec3 inV)					{ MOSS_ASSERT(inCol < 4); mCol[inCol] = Vec4(inV, inCol == 3? 1.0f : 0.0f); }
	MOSS_INLINE Vec4			GetColumn4(uint inCol) const							{ MOSS_ASSERT(inCol < 4); return Vec4(mCol[inCol]); }
	MOSS_INLINE void			SetColumn4(uint inCol, const Vec4 inV)					{ MOSS_ASSERT(inCol < 4); mCol[inCol] = inV; }

	/// Store matrix to memory
	MOSS_INLINE void			StoreFloat4x4(Float4 *outV) const;

	/// Transpose matrix
	MOSS_INLINE Mat44			Transposed() const;

	/// Transpose 3x3 subpart of matrix
	MOSS_INLINE Mat44			Transposed3x3() const;

	/// Inverse 4x4 matrix
	MOSS_INLINE Mat44			Inversed() const;

	/// Inverse 4x4 matrix when it only contains rotation and translation
	MOSS_INLINE Mat44			InversedRotationTranslation() const;

	/// Get the determinant of a 3x3 matrix
	MOSS_INLINE float			GetDeterminant3x3() const;

	/// Get the adjoint of a 3x3 matrix
	MOSS_INLINE Mat44			Adjointed3x3() const;

	/// Inverse 3x3 matrix
	MOSS_INLINE Mat44			Inversed3x3() const;

	/// *this = inM.Inversed3x3(), returns false if the matrix is singular in which case *this is unchanged
	MOSS_INLINE bool			SetInversed3x3(const Mat44 inM);

	/// Get rotation part only (note: retains the first 3 values from the bottom row)
	MOSS_INLINE Mat44			GetRotation() const;

	/// Get rotation part only (note: also clears the bottom row)
	MOSS_INLINE Mat44			GetRotationSafe() const;

	/// Updates the rotation part of this matrix (the first 3 columns)
	MOSS_INLINE void			SetRotation(const Mat44  inRotation);

	/// Convert to quaternion
	MOSS_INLINE Quat			GetQuaternion() const;

	/// Get matrix that transforms a direction with the same transform as this matrix (length is not preserved)
	MOSS_INLINE Mat44			GetDirectionPreservingMatrix() const					{ return GetRotation().Inversed3x3().Transposed3x3(); }

	/// Pre multiply by translation matrix: result = this * Mat44::sTranslation(inTranslation)
	MOSS_INLINE Mat44			PreTranslated(const Vec3 inTranslation) const;

	/// Post multiply by translation matrix: result = Mat44::sTranslation(inTranslation) * this (i.e. add inTranslation to the 4-th column)
	MOSS_INLINE Mat44			PostTranslated(const Vec3 inTranslation) const;

	/// Scale a matrix: result = this * Mat44::sScale(inScale)
	MOSS_INLINE Mat44			PreScaled(const Vec3 inScale) const;

	/// Scale a matrix: result = Mat44::sScale(inScale) * this
	MOSS_INLINE Mat44			PostScaled(const Vec3 inScale) const;

	/// Decompose a matrix into a rotation & translation part and into a scale part so that:
	/// this = return_value * Mat44::sScale(outScale).
	/// This equation only holds when the matrix is orthogonal, if it is not the returned matrix
	/// will be made orthogonal using the modified Gram-Schmidt algorithm (see: https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process)
	MOSS_INLINE Mat44			Decompose(Vec3 &outScale) const;

#ifndef MOSS_DOUBLE_PRECISION
	/// In single precision mode just return the matrix itself
	MOSS_INLINE Mat44			ToMat44() const											{ return *this; }
#endif // !MOSS_DOUBLE_PRECISION

	/// To String
	friend ostream &			operator << (ostream &inStream, const Mat44  inM)
	{
		inStream << inM.mCol[0] << ", " << inM.mCol[1] << ", " << inM.mCol[2] << ", " << inM.mCol[3];
		return inStream;
	}

private:
	Vec4						mCol[4];												///< Column
};

// === View Matrix ===
// === Orthographic Projections ===
static Mat44 ortho(float left, float right, float bottom, float top, float near, float far) {
    float scaleX = 2.0f / (right - left);
    float scaleY = 2.0f / (top - bottom);  // bottom > top flips Y axis to increase downward
#ifdef MOSS_USE_OPENGL
    float scaleZ = 2.0f / (far - near);
#else
	float scaleZ = -2.0f / (far - near);
#endif // MOSS_USE_OPENGL
    float transX = -(right + left) / (right - left);
    float transY = -(top + bottom) / (top - bottom);
    float transZ = -(far + near) / (far - near);

    return Mat44(
        Vec4(scaleX, 0.0f,    0.0f,   0.0f),
        Vec4(0.0f,   scaleY,  0.0f,   0.0f),
        Vec4(0.0f,   0.0f,    scaleZ, 0.0f),
        Vec4(transX, transY,  transZ, 1.0f)
    );
}

/*
static Mat44 frustumLH(float left, float right, float bottom, float top, float near, float far) {
    Mat44 result = {};
    result.mCol[0][0] = 2 * near / (right - left);
    result.mCol[1][1] = 2 * near / (top - bottom);
    result.mCol[0][2] = (right + left) / (right - left);
    result.mCol[1][2] = (top + bottom) / (top - bottom);
    result.mCol[2][2] = (far + near) / (far - near);
    result.mCol[2][3] = -2 * far * near / (far - near);
    result.mCol[3][2] = 1;
    return result;
}

static Mat44 frustumRH(float left, float right, float bottom, float top, float near, float far) {
    Mat44 result = {};
    result.mCol[0][0] = 2 * near / (right - left);
    result.mCol[1][1] = 2 * near / (top - bottom);
    result.mCol[0][2] = (right + left) / (right - left);
    result.mCol[1][2] = (top + bottom) / (top - bottom);
    result.mCol[2][2] = -(far + near) / (far - near);
    result.mCol[2][3] = -2 * far * near / (far - near);
    result.mCol[3][2] = -1;
    return result;
}

static Mat44 frustum(float left, float right, float bottom, float top, float near, float far) {
#ifdef MOSS_USE_OPENGL || MOSS_USE_OPENGLES
    return frustumLH(left, right, bottom, top, near, far);
#else
    return frustumRH(left, right, bottom, top, near, far);
#endif
}

// === Optional: Infinite Projections ===
static Mat44 infinitePerspective(float fovy, float aspect, float near) {
	float tanHalfFovy = std::tan(fovy / 2.0f);
    Mat44 result = {};
    result.mCol[0][0] = 1.0f / (aspect * tanHalfFovy);
    result.mCol[1][1] = 1.0f / tanHalfFovy;
    result.mCol[2][2] = -1.0f;
    result.mCol[2][3] = -2.0f * near;
    result.mCol[3][2] = -1.0f;
    return result;
}*/

static_assert(std::is_trivial<Mat44>(), "Is supposed to be a trivial type!");

MOSS_SUPRESS_WARNINGS_END

#include "Mat44.inl"

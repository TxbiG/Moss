// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

MOSS_SUPRESS_WARNINGS_BEGIN

/// Templatized vector class
template <uint32 Rows>
class [[nodiscard]] TVec
{
public:
	/// Constructor
	inline						TVec() = default;
	inline						TVec(const TVec &) = default;

	/// Dimensions
	inline uint32					GetRows() const											{ return Rows; }

	/// Vector with all zeros
	inline void					SetZero()
	{
		for (uint32 r = 0; r < Rows; ++r)
			mF32[r] = 0.0f;
	}

	inline static TVec		sZero()													{ TVec v; v.SetZero(); return v; }

	/// Copy a (part) of another vector into this vector
	template <class OtherVector>
	void					CopyPart(const OtherVector &inV, uint32 inSourceRow, uint32 inNumRows, uint32 inDestRow)
	{
		for (uint32 r = 0; r < inNumRows; ++r) { mF32[inDestRow + r] = inV[inSourceRow + r]; }
	}

	/// Get float component by index
	inline float				operator [] (uint32 inCoordinate) const
	{
		MOSS_ASSERT(inCoordinate < Rows);
		return mF32[inCoordinate];
	}

	inline float &				operator [] (uint32 inCoordinate)
	{
		MOSS_ASSERT(inCoordinate < Rows);
		return mF32[inCoordinate];
	}

	/// Comparison
	inline bool					operator == (const TVec &inV2) const
	{
		for (uint32 r = 0; r < Rows; ++r)
		{
			if (mF32[r] != inV2.mF32[r])
				return false;
		}
		return true;
	}

	inline bool					operator != (const TVec &inV2) const
	{
		for (uint32 r = 0; r < Rows; ++r)
			if (mF32[r] != inV2.mF32[r])
				return true;
		return false;
	}

	/// Test if vector consists of all zeros
	inline bool					IsZero() const
	{
		for (uint32 r = 0; r < Rows; ++r)
			if (mF32[r] != 0.0f)
				return false;
		return true;
	}

	/// Test if two vectors are close to each other
	inline bool					IsClose(const TVec &inV2, float inMaxDistSq = 1.0e-12f)
	{
		return (inV2 - *this).LengthSq() <= inMaxDistSq;
	}

	/// Assignment
	inline TVec &				operator = (const TVec &) = default;

	/// Multiply vector with float
	inline TVec				operator * (const float inV2) const
	{
		TVec v;
		for (uint32 r = 0; r < Rows; ++r)
			v.mF32[r] = mF32[r] * inV2;
		return v;
	}

	inline TVec &				operator *= (const float inV2)
	{
		for (uint32 r = 0; r < Rows; ++r)
			mF32[r] *= inV2;
		return *this;
	}

	/// Multiply vector with float
	inline friend TVec		operator * (const float inV1, const TVec &inV2)
	{
		return inV2 * inV1;
	}

	/// Divide vector by float
	inline TVec				operator / (float inV2) const
	{
		TVector v;
		for (uint32 r = 0; r < Rows; ++r)
			v.mF32[r] = mF32[r] / inV2;
		return v;
	}

	inline TVec &				operator /= (float inV2)
	{
		for (uint32 r = 0; r < Rows; ++r)
			mF32[r] /= inV2;
		return *this;
	}

	/// Add two float vectors (component wise)
	inline TVec				operator + (const TVec &inV2) const
	{
		TVec v;
		for (uint32 r = 0; r < Rows; ++r)
			v.mF32[r] = mF32[r] + inV2.mF32[r];
		return v;
	}

	inline TVec &				operator += (const TVec &inV2)
	{
		for (uint32 r = 0; r < Rows; ++r)
			mF32[r] += inV2.mF32[r];
		return *this;
	}

	/// Negate
	inline TVec				operator - () const
	{
		TVec v;
		for (uint32 r = 0; r < Rows; ++r)
			v.mF32[r] = -mF32[r];
		return v;
	}

	/// Subtract two float vectors (component wise)
	inline TVec				operator - (const TVec &inV2) const
	{
		TVec v;
		for (uint32 r = 0; r < Rows; ++r)
			v.mF32[r] = mF32[r] - inV2.mF32[r];
		return v;
	}

	inline TVec &				operator -= (const TVec &inV2)
	{
		for (uint32 r = 0; r < Rows; ++r)
			mF32[r] -= inV2.mF32[r];
		return *this;
	}

	/// Dot product
	inline float				Dot(const TVec &inV2) const
	{
		float dot = 0.0f;
		for (uint32 r = 0; r < Rows; ++r)
			dot += mF32[r] * inV2.mF32[r];
		return dot;
	}

	/// Squared length of vector
	inline float				LengthSq() const
	{
		return Dot(*this);
	}

	/// Length of vector
	inline float				Length() const
	{
		return sqrt(LengthSq());
	}

	/// Check if vector is normalized
	inline bool					IsNormalized(float inToleranceSq = 1.0e-6f)
	{
		return abs(LengthSq() - 1.0f) <= inToleranceSq;
	}

	/// Normalize vector
	inline TVec				Normalized() const
	{
		return *this / Length();
	}

	/// To String
	friend ostream &			operator << (ostream &inStream, const TVec &inV)
	{
		inStream << "[";
		for (uint32 i = 0; i < Rows - 1; ++i)
			inStream << inV.mF32[i] << ", ";
		inStream << inV.mF32[Rows - 1] << "]";
		return inStream;
	}

	float						mF32[Rows];
};

MOSS_SUPRESS_WARNINGS_END

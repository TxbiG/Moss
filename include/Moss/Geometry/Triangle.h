// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

MOSS_SUPPRESS_WARNINGS_BEGIN

class AABB3;
using AABox = AABB3;


/// Ellipse centered around the origin
/// @see https://en.wikipedia.org/wiki/Ellipse
class Ellipse {
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Construct ellipse with radius A along the X-axis and B along the Y-axis
	Ellipse(float inA, float inB) : mA(inA), mB(inB) {MOSS_ASSERT(inA > 0.0f); MOSS_ASSERT(inB > 0.0f); }

	/// Check if inPoint is inside the ellipse
	bool IsInside(const Float2 &inPoint) const { return Square(inPoint.x / mA) + Square(inPoint.y / mB) <= 1.0f; }

	/// Get the closest point on the ellipse to inPoint
	/// Assumes inPoint is outside the ellipse
	/// @see Rotation Joint Limits in Quaternion Space by Gino van den Bergen, section 10.1 in Game Engine Gems 3.
	Float2			GetClosestPoint(const Float2 &inPoint) const {
		float a_sq = Square(mA);
		float b_sq = Square(mB);

		// Equation of ellipse: f(x, y) = (x/a)^2 + (y/b)^2 - 1 = 0											[1]
		// Normal on surface: (df/dx, df/dy) = (2 x / a^2, 2 y / b^2)
		// Closest point (x', y') on ellipse to point (x, y): (x', y') + t (x / a^2, y / b^2) = (x, y)
		// <=> (x', y') = (a^2 x / (t + a^2), b^2 y / (t + b^2))
		// Requiring point to be on ellipse (substituting into [1]): g(t) = (a x / (t + a^2))^2 + (b y / (t + b^2))^2 - 1 = 0

		// Newton Raphson iteration, starting at t = 0
		float t = 0.0f;
		for (;;)
		{
			// Calculate g(t)
			float t_plus_a_sq = t + a_sq;
			float t_plus_b_sq = t + b_sq;
			float gt = Square(mA * inPoint.x / t_plus_a_sq) + Square(mB * inPoint.y / t_plus_b_sq) - 1.0f;

			// Check if g(t) it is close enough to zero
			if (abs(gt) < 1.0e-6f)
				return Float2(a_sq * inPoint.x / t_plus_a_sq, b_sq * inPoint.y / t_plus_b_sq);

			// Get derivative dg/dt = g'(t) = -2 (b^2 y^2 / (t + b^2)^3 + a^2 x^2 / (t + a^2)^3)
			float gt_accent = -2.0f *
				(a_sq * Square(inPoint.x) / Cubed(t_plus_a_sq)
				+ b_sq * Square(inPoint.y) / Cubed(t_plus_b_sq));

			// Calculate t for next iteration: tn+1 = tn - g(t) / g'(t)
			float tn = t - gt / gt_accent;
			t = tn;
		}
	}

	/// Get normal at point inPoint (non-normalized vector)
	Float2 GetNormal(const Float2 &inPoint) const {
		// Calculated by [d/dx f(x, y), d/dy f(x, y)], where f(x, y) is the ellipse equation from above
		return Float2(inPoint.x / Square(mA), inPoint.y / Square(mB));
	}

private:
	float			mA;				///< Radius along X-axis
	float			mB;				///< Radius along Y-axis
};

class [[nodiscard]] Sphere {
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
	inline Sphere() = default;
	inline Sphere(const Float3 &inCenter, float inRadius)		: mCenter(inCenter), mRadius(inRadius) { }
	inline Sphere(Vec3Arg inCenter, float inRadius)				: mRadius(inRadius) { inCenter.StoreFloat3(&mCenter); }

	/// Calculate the support vector for this convex shape.
	inline Vec3	GetSupport(Vec3Arg inDirection) const {
		float length = inDirection.Length();
		return length > 0.0f ? Vec3::LoadFloat3Unsafe(mCenter) + (mRadius/ length) * inDirection : Vec3::LoadFloat3Unsafe(mCenter);
	}

	// Properties
	inline Vec3	GetCenter() const  { return Vec3::LoadFloat3Unsafe(mCenter); }
	inline float GetRadius() const { return mRadius; }

	/// Test if two spheres overlap
	inline bool	Overlaps(const Sphere &inB) const { return (Vec3::LoadFloat3Unsafe(mCenter) - Vec3::LoadFloat3Unsafe(inB.mCenter)).LengthSq() <= Square(mRadius + inB.mRadius); }

	/// Check if this sphere overlaps with a box
	bool Overlaps(const AABB3 &inOther) const;

	/// Create the minimal sphere that encapsulates this sphere and inPoint
	inline void EncapsulatePoint(Vec3Arg inPoint) {
		// Calculate distance between point and center
		Vec3 center = GetCenter();
		Vec3 d_vec = inPoint - center;
		float d_sq = d_vec.LengthSq();
		if (d_sq > Square(mRadius)) {
			// It is further away than radius, we need to widen the sphere
			// The diameter of the new sphere is radius + d, so the new radius is half of that
			float d = sqrt(d_sq);
			float radius = 0.5f * (mRadius + d);

			// The center needs to shift by new radius - old radius in the direction of d
			center += (radius - mRadius) / d * d_vec;

			// Store new sphere
			center.StoreFloat3(&mCenter);
			mRadius = radius;
		}
	}

private:
	Float3				mCenter;
	float				mRadius;
};

/// A simple triangle and its material
class Triangle {
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
	Triangle() = default;
	Triangle(const Float3 &inV1, const Float3 &inV2, const Float3 &inV3, uint32_t inMaterialIndex = 0, uint32_t inUserData = 0) : mV { inV1, inV2, inV3 }, mMaterialIndex(inMaterialIndex), mUserData(inUserData) { }
	Triangle(Vec3Arg inV1, Vec3Arg inV2, Vec3Arg inV3, uint32_t inMaterialIndex = 0, uint32_t inUserData = 0) : mMaterialIndex(inMaterialIndex), mUserData(inUserData) { inV1.StoreFloat3(&mV[0]); inV2.StoreFloat3(&mV[1]); inV3.StoreFloat3(&mV[2]); }

	/// Get center of triangle
	Vec3			GetCentroid() const
	{
		return (Vec3::LoadFloat3Unsafe(mV[0]) + Vec3::LoadFloat3Unsafe(mV[1]) + Vec3::LoadFloat3Unsafe(mV[2])) * (1.0f / 3.0f);
	}

	/// Vertices
	Float3			mV[3];
	uint32_t			mMaterialIndex = 0;			///< Follows mV[3] so that we can read mV as 4 vectors
	uint32_t			mUserData = 0;				///< User data that can be used for anything by the application, e.g. for tracking the original index of the triangle
};

using TriangleList = TArray<Triangle>;



class MortonCode {
public:
	/// First converts a floating point value in the range [0, 1] to a 10 bit fixed point integer.
	/// Then expands a 10-bit integer into 30 bits by inserting 2 zeros after each bit.
	static uint32_t sExpandBits(float inV) {
		MOSS_ASSERT(inV >= 0.0f && inV <= 1.0f);
		uint32_t v = uint32_t(inV * 1023.0f + 0.5f);
		MOSS_ASSERT(v < 1024);
		v = (v * 0x00010001u) & 0xFF0000FFu;
		v = (v * 0x00000101u) & 0x0F00F00Fu;
		v = (v * 0x00000011u) & 0xC30C30C3u;
		v = (v * 0x00000005u) & 0x49249249u;
		return v;
	}

	/// Calculate the morton code for inVector, given that all vectors lie in inVectorBounds
	static uint32_t sGetMortonCode(Vec3Arg inVector, const AABB3& inVectorBounds);
};









class IndexedTriangleNoMaterial {
public:
	MOSS_OVERRIDE_NEW_DELETE

	// Constructor
					IndexedTriangleNoMaterial() = default;
	constexpr		IndexedTriangleNoMaterial(uint32_t inI1, uint32_t inI2, uint32_t inI3) : mIdx { inI1, inI2, inI3 } { }

	// Check if two triangles are identical
	bool			operator == (const IndexedTriangleNoMaterial& inRHS) const
	{
		return mIdx[0] == inRHS.mIdx[0]&& mIdx[1] == inRHS.mIdx[1]&& mIdx[2] == inRHS.mIdx[2];
	}

	// Check if two triangles are equivalent (using the same vertices)
	bool			IsEquivalent(const IndexedTriangleNoMaterial& inRHS) const
	{
		return (mIdx[0] == inRHS.mIdx[0]&& mIdx[1] == inRHS.mIdx[1]&& mIdx[2] == inRHS.mIdx[2])
			|| (mIdx[0] == inRHS.mIdx[1]&& mIdx[1] == inRHS.mIdx[2]&& mIdx[2] == inRHS.mIdx[0])
			|| (mIdx[0] == inRHS.mIdx[2]&& mIdx[1] == inRHS.mIdx[0]&& mIdx[2] == inRHS.mIdx[1]);
	}

	// Check if two triangles are opposite (using the same vertices but in opposing order)
	bool			IsOpposite(const IndexedTriangleNoMaterial& inRHS) const
	{
		return (mIdx[0] == inRHS.mIdx[0]&& mIdx[1] == inRHS.mIdx[2]&& mIdx[2] == inRHS.mIdx[1])
			|| (mIdx[0] == inRHS.mIdx[1]&& mIdx[1] == inRHS.mIdx[0]&& mIdx[2] == inRHS.mIdx[2])
			|| (mIdx[0] == inRHS.mIdx[2]&& mIdx[1] == inRHS.mIdx[1]&& mIdx[2] == inRHS.mIdx[0]);
	}

	// Check if triangle is degenerate
	bool			IsDegenerate(const VertexList& inVertices) const
	{
		Vec3 v0(inVertices[mIdx[0]]);
		Vec3 v1(inVertices[mIdx[1]]);
		Vec3 v2(inVertices[mIdx[2]]);

		return (v1 - v0).Cross(v2 - v0).IsNearZero();
	}

	// Rotate the vertices so that the second vertex becomes first etc. This does not change the represented triangle.
	void			Rotate()
	{
		uint32_t tmp = mIdx[0];
		mIdx[0] = mIdx[1];
		mIdx[1] = mIdx[2];
		mIdx[2] = tmp;
	}

	// Get center of triangle
	Vec3			GetCentroid(const VertexList& inVertices) const
	{
		return (Vec3(inVertices[mIdx[0]]) + Vec3(inVertices[mIdx[1]]) + Vec3(inVertices[mIdx[2]])) / 3.0f;
	}

	// Get the hash value of this structure
	uint64_t			GetHash() const
	{
		static_assert(sizeof(IndexedTriangleNoMaterial) == 3* sizeof(uint32_t), "Class should have no padding");
		return HashBytes(this, sizeof(IndexedTriangleNoMaterial));
	}

	uint32_t			mIdx[3];
};


// Triangle with 32-bit indices and material index
class IndexedTriangle : public IndexedTriangleNoMaterial {
public:
	using IndexedTriangleNoMaterial::IndexedTriangleNoMaterial;

	// Constructor
	constexpr		IndexedTriangle(uint32_t inI1, uint32_t inI2, uint32_t inI3, uint32_t inMaterialIndex, uint32_t inUserData = 0) : IndexedTriangleNoMaterial(inI1, inI2, inI3), mMaterialIndex(inMaterialIndex), mUserData(inUserData) { }

	// Check if two triangles are identical
	bool			operator == (const IndexedTriangle& inRHS) const
	{
		return mMaterialIndex == inRHS.mMaterialIndex&& mUserData == inRHS.mUserData&& IndexedTriangleNoMaterial::operator==(inRHS);
	}

	// Rotate the vertices so that the lowest vertex becomes the first. This does not change the represented triangle.
	IndexedTriangle	GetLowestIndexFirst() const
	{
		if (mIdx[0] < mIdx[1])
		{
			if (mIdx[0] < mIdx[2])
				return IndexedTriangle(mIdx[0], mIdx[1], mIdx[2], mMaterialIndex, mUserData); // 0 is smallest
			else
				return IndexedTriangle(mIdx[2], mIdx[0], mIdx[1], mMaterialIndex, mUserData); // 2 is smallest
		}
		else
		{
			if (mIdx[1] < mIdx[2])
				return IndexedTriangle(mIdx[1], mIdx[2], mIdx[0], mMaterialIndex, mUserData); // 1 is smallest
			else
				return IndexedTriangle(mIdx[2], mIdx[0], mIdx[1], mMaterialIndex, mUserData); // 2 is smallest
		}
	}

	// Get the hash value of this structure
	uint64_t			GetHash() const
	{
		static_assert(sizeof(IndexedTriangle) == 5* sizeof(uint32_t), "Class should have no padding");
		return HashBytes(this, sizeof(IndexedTriangle));
	}

	uint32_t			mMaterialIndex = 0;
	uint32_t			mUserData = 0;				// User data that can be used for anything by the application, e.g. for tracking the original index of the triangle
};


using IndexedTriangleNoMaterialList = TArray<IndexedTriangleNoMaterial>;
using IndexedTriangleList = TArray<IndexedTriangle>;

MOSS_SUPPRESS_WARNINGS_END

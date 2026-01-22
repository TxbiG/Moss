// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

MOSS_NAMESPACE_BEGIN


class [[nodiscard]] Sphere {
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
	inline Sphere() = default;
	inline Sphere(const Float3 &inCenter, float inRadius)			: mCenter(inCenter), mRadius(inRadius) { }
	inline Sphere(Vec3Arg inCenter, float inRadius)				: mRadius(inRadius) { inCenter.StoreFloat3(&mCenter); }

	/// Calculate the support vector for this convex shape.
	inline Vec3	GetSupport(Vec3Arg inDirection) const {
		float length = inDirection.Length();
		return length > 0.0f ? Vec3::sLoadFloat3Unsafe(mCenter) + (mRadius/ length) * inDirection : Vec3::sLoadFloat3Unsafe(mCenter);
	}

	// Properties
	inline Vec3	GetCenter() const  { return Vec3::sLoadFloat3Unsafe(mCenter); }
	inline float GetRadius() const { return mRadius; }

	/// Test if two spheres overlap
	inline bool	Overlaps(const Sphere &inB) const { return (Vec3::sLoadFloat3Unsafe(mCenter) - Vec3::sLoadFloat3Unsafe(inB.mCenter)).LengthSq() <= Square(mRadius + inB.mRadius); }

	/// Check if this sphere overlaps with a box
	inline bool Overlaps(const AABox &inOther) const { return inOther.GetSqDistanceTo(GetCenter()) <= Square(mRadius); }

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
	Triangle(const Float3 &inV1, const Float3 &inV2, const Float3 &inV3, uint32 inMaterialIndex = 0, uint32 inUserData = 0) : mV { inV1, inV2, inV3 }, mMaterialIndex(inMaterialIndex), mUserData(inUserData) { }
	Triangle(Vec3Arg inV1, Vec3Arg inV2, Vec3Arg inV3, uint32 inMaterialIndex = 0, uint32 inUserData = 0) : mMaterialIndex(inMaterialIndex), mUserData(inUserData) { inV1.StoreFloat3(&mV[0]); inV2.StoreFloat3(&mV[1]); inV3.StoreFloat3(&mV[2]); }

	/// Get center of triangle
	Vec3			GetCentroid() const
	{
		return (Vec3::sLoadFloat3Unsafe(mV[0]) + Vec3::sLoadFloat3Unsafe(mV[1]) + Vec3::sLoadFloat3Unsafe(mV[2])) * (1.0f / 3.0f);
	}

	/// Vertices
	Float3			mV[3];
	uint32			mMaterialIndex = 0;			///< Follows mV[3] so that we can read mV as 4 vectors
	uint32			mUserData = 0;				///< User data that can be used for anything by the application, e.g. for tracking the original index of the triangle
};

using TriangleList = TArray<Triangle>;



class MortonCode {
public:
	/// First converts a floating point value in the range [0, 1] to a 10 bit fixed point integer.
	/// Then expands a 10-bit integer into 30 bits by inserting 2 zeros after each bit.
	static uint32 sExpandBits(float inV) {
		MOSS_ASSERT(inV >= 0.0f && inV <= 1.0f);
		uint32 v = uint32(inV * 1023.0f + 0.5f);
		MOSS_ASSERT(v < 1024);
		v = (v * 0x00010001u) & 0xFF0000FFu;
		v = (v * 0x00000101u) & 0x0F00F00Fu;
		v = (v * 0x00000011u) & 0xC30C30C3u;
		v = (v * 0x00000005u) & 0x49249249u;
		return v;
	}

	/// Calculate the morton code for inVector, given that all vectors lie in inVectorBounds
	static uint32 sGetMortonCode(Vec3Arg inVector, const AABox &inVectorBounds) {
		// Convert to 10 bit fixed point
		Vec3 scaled = (inVector - inVectorBounds.mMin) / inVectorBounds.GetSize();
		uint x = sExpandBits(scaled.GetX());
		uint y = sExpandBits(scaled.GetY());
		uint z = sExpandBits(scaled.GetZ());
		return (x << 2) + (y << 1) + z;
	}
};

MOSS_NAMESPACE_END

// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Physics/Geometry/Triangle.h>
#include <Moss/Physics/Geometry/IndexedTriangle.h>
#include <Moss/Physics/Geometry/Plane.h>
#include <Moss/Core/Variants/Matrix/Mat44.h>

MOSS_WARNINGS_BEGIN

// AABB3 (Axis aligned box 3D) is used for 3D Collisions
class [[nodiscard]] AABB3 {
public:
	MOSS_OVERRIDE_NEW_DELETE

	// Constructor
	AABB3()									: mMin(RVec3::sReplicate(FLT_MAX)), mMax(Vec3::sReplicate(-FLT_MAX)) { }
	AABB3(RVec3Arg inMin, RVec3Arg inMax)		: mMin(inMin), mMax(inMax) { }
	AABB3(DVec3Arg inMin, DVec3Arg inMax)	: mMin(inMin.ToVec3RoundDown()), mMax(inMax.ToVec3RoundUp()) { }
	AABB3(Vec3Arg inCenter, float inRadius)	: mMin(inCenter - RVec3::sReplicate(inRadius)), mMax(inCenter + RVec3::sReplicate(inRadius)) { }

	// Create box from 2 points
	static AABB3	sFromTwoPoints(Vec3Arg inP1, Vec3Arg inP2) { return AABB3(RVec3::sMin(inP1, inP2), RVec3::sMax(inP1, inP2)); }

	// Create box from indexed triangle
	static AABB3	sFromTriangle(const VertexList &inVertices, const IndexedTriangle &inTriangle) {
		AABB3 box = sFromTwoPoints(Vec3(inVertices[inTriangle.mIdx[0]]), Vec3(inVertices[inTriangle.mIdx[1]]));
		box.Encapsulate(Vec3(inVertices[inTriangle.mIdx[2]]));
		return box;
	}

	// Get bounding box of size FLT_MAX
	static AABB3 sBiggest() {
		// Max half extent of AABB3 is 0.5 * FLT_MAX so that GetSize() remains finite
		return AABB3(RVec3::sReplicate(-0.5f * FLT_MAX), RVec3::sReplicate(0.5f * FLT_MAX));
	}

	// Reset the bounding box to an empty bounding box
	void SetEmpty() {
		mMin = RVec3::sReplicate(FLT_MAX);
		mMax = RVec3::sReplicate(-FLT_MAX);
	}

	// Check if the bounding box is valid (max >= min)
	bool IsValid() const {
		return mMin.GetX() <= mMax.GetX() && mMin.GetY() <= mMax.GetY() && mMin.GetZ() <= mMax.GetZ();
	}

	// Encapsulate point in bounding box
	void Encapsulate(Vec3Arg inPos) {
		mMin = RVec3::sMin(mMin, inPos);
		mMax = RVec3::sMax(mMax, inPos);
	}

	// Encapsulate bounding box in bounding box
	void Encapsulate(const AABB3 &inRHS) {
		mMin = RVec3::sMin(mMin, inRHS.mMin);
		mMax = RVec3::sMax(mMax, inRHS.mMax);
	}

	// Encapsulate triangle in bounding box
	void Encapsulate(const Triangle &inRHS) {
		RVec3 v = RVec3::sLoadFloat3Unsafe(inRHS.mV[0]);
		Encapsulate(v);
		v = RVec3::sLoadFloat3Unsafe(inRHS.mV[1]);
		Encapsulate(v);
		v = RVec3::sLoadFloat3Unsafe(inRHS.mV[2]);
		Encapsulate(v);
	}

	// Encapsulate triangle in bounding box
	void Encapsulate(const VertexList &inVertices, const IndexedTriangle &inTriangle) {
		for (uint32 idx : inTriangle.mIdx)
			Encapsulate(Vec3(inVertices[idx]));
	}

	// Intersect this bounding box with inOther, returns the intersection
	AABB3 Intersect(const AABB3 &inOther) const { return AABB3(RVec3::sMax(mMin, inOther.mMin), RVec3::sMin(mMax, inOther.mMax)); }

	// Make sure that each edge of the bounding box has a minimal length
	void EnsureMinimalEdgeLength(float inMinEdgeLength) {
		RVec3 min_length = RVec3::sReplicate(inMinEdgeLength);
		mMax = RVec3::sSelect(mMax, mMin + min_length, RVec3::sLess(mMax - mMin, min_length));
	}

	// Widen the box on both sides by inVector
	void ExpandBy(Vec3Arg inVector) {
		mMin -= inVector;
		mMax += inVector;
	}

	// Get center of bounding box
	RVec3 GetCenter() const { return 0.5f * (mMin + mMax); }

	// Get extent of bounding box (half of the size)
	RVec3 GetExtent() const { return 0.5f * (mMax - mMin); }

	// Get size of bounding box
	RVec3 GetSize() const { return mMax - mMin; }

	// Get surface area of bounding box
	float GetSurfaceArea() const {
		RVec3 extent = mMax - mMin;
		return 2.0f * (extent.GetX() * extent.GetY() + extent.GetX() * extent.GetZ() + extent.GetY() * extent.GetZ());
	}

	// Get volume of bounding box
	float GetVolume() const {
		RVec3 extent = mMax - mMin;
		return extent.GetX() * extent.GetY() * extent.GetZ();
	}

	// Check if this box contains another box
	bool Contains(const AABB3 &inOther) const { return UVec4::sAnd(RVec3::sLessOrEqual(mMin, inOther.mMin), RVec3::sGreaterOrEqual(mMax, inOther.mMax)).TestAllXYZTrue(); }

	// Check if this box contains a point
	bool Contains(Vec3Arg inOther) const {
		return UVec4::sAnd(RVec3::sLessOrEqual(mMin, inOther), RVec3::sGreaterOrEqual(mMax, inOther)).TestAllXYZTrue();
	}

	// Check if this box contains a point
	bool Contains(DVec3Arg inOther) const {
		return Contains(Vec3(inOther));
	}

	// Check if this box overlaps with another box
	bool Overlaps(const AABB3 &inOther) const {
		return !UVec4::sOr(RVec3::sGreater(mMin, inOther.mMax), RVec3::sLess(mMax, inOther.mMin)).TestAnyXYZTrue();
	}

	// Check if this box overlaps with a plane
	bool Overlaps(const Plane &inPlane) const {
		RVec3 normal = inPlane.GetNormal();
		float dist_normal = inPlane.SignedDistance(GetSupport(normal));
		float dist_min_normal = inPlane.SignedDistance(GetSupport(-normal));
		return dist_normal * dist_min_normal <= 0.0f; // If both support points are on the same side of the plane we don't overlap
	}

	// Translate bounding box
	void Translate(Vec3Arg inTranslation) {
		mMin += inTranslation;
		mMax += inTranslation;
	}

	// Translate bounding box
	void Translate(DVec3Arg inTranslation) {
		mMin = (DVec3(mMin) + inTranslation).ToVec3RoundDown();
		mMax = (DVec3(mMax) + inTranslation).ToVec3RoundUp();
	}

	// Transform bounding box
	AABB3 Transformed(Mat44Arg inMatrix) const {
		// Start with the translation of the matrix
		RVec3 new_min, new_max;
		new_min = new_max = inMatrix.GetTranslation();

		// Now find the extreme points by considering the product of the min and max with each column of inMatrix
		for (int c = 0; c < 3; ++c)
		{
			RVec3 col = inMatrix.GetColumn3(c);

			RVec3 a = col * mMin[c];
			RVec3 b = col * mMax[c];

			new_min += RVec3::sMin(a, b);
			new_max += RVec3::sMax(a, b);
		}

		// Return the new bounding box
		return AABB3(new_min, new_max);
	}

	// Transform bounding box
	AABB3 Transformed(DMat44Arg inMatrix) const {
		AABB3 transformed = Transformed(inMatrix.GetRotation());
		transformed.Translate(inMatrix.GetTranslation());
		return transformed;
	}

	// Scale this bounding box, can handle non-uniform and negative scaling
	AABB3 Scaled(Vec3Arg inScale) const
	{
		return AABB3::sFromTwoPoints(mMin * inScale, mMax * inScale);
	}

	// Calculate the support vector for this convex shape.
	RVec3			GetSupport(Vec3Arg inDirection) const
	{
		return RVec3::sSelect(mMax, mMin, RVec3::sLess(inDirection, RVec3::sZero()));
	}

	// Get the vertices of the face that faces inDirection the most
	template <class VERTEX_ARRAY>
	void GetSupportingFace(Vec3Arg inDirection, VERTEX_ARRAY &outVertices) const {
		outVertices.resize(4);

		int axis = inDirection.Abs().GetHighestComponentIndex();
		if (inDirection[axis] < 0.0f) {
			switch (axis)
			{
			case 0:
				outVertices[0] = Vec3(mMax.GetX(), mMin.GetY(), mMin.GetZ());
				outVertices[1] = Vec3(mMax.GetX(), mMax.GetY(), mMin.GetZ());
				outVertices[2] = Vec3(mMax.GetX(), mMax.GetY(), mMax.GetZ());
				outVertices[3] = Vec3(mMax.GetX(), mMin.GetY(), mMax.GetZ());
				break;

			case 1:
				outVertices[0] = Vec3(mMin.GetX(), mMax.GetY(), mMin.GetZ());
				outVertices[1] = Vec3(mMin.GetX(), mMax.GetY(), mMax.GetZ());
				outVertices[2] = Vec3(mMax.GetX(), mMax.GetY(), mMax.GetZ());
				outVertices[3] = Vec3(mMax.GetX(), mMax.GetY(), mMin.GetZ());
				break;

			case 2:
				outVertices[0] = Vec3(mMin.GetX(), mMin.GetY(), mMax.GetZ());
				outVertices[1] = Vec3(mMax.GetX(), mMin.GetY(), mMax.GetZ());
				outVertices[2] = Vec3(mMax.GetX(), mMax.GetY(), mMax.GetZ());
				outVertices[3] = Vec3(mMin.GetX(), mMax.GetY(), mMax.GetZ());
				break;
			}
		}
		else
		{
			switch (axis)
			{
			case 0:
				outVertices[0] = Vec3(mMin.GetX(), mMin.GetY(), mMin.GetZ());
				outVertices[1] = Vec3(mMin.GetX(), mMin.GetY(), mMax.GetZ());
				outVertices[2] = Vec3(mMin.GetX(), mMax.GetY(), mMax.GetZ());
				outVertices[3] = Vec3(mMin.GetX(), mMax.GetY(), mMin.GetZ());
				break;

			case 1:
				outVertices[0] = Vec3(mMin.GetX(), mMin.GetY(), mMin.GetZ());
				outVertices[1] = Vec3(mMax.GetX(), mMin.GetY(), mMin.GetZ());
				outVertices[2] = Vec3(mMax.GetX(), mMin.GetY(), mMax.GetZ());
				outVertices[3] = Vec3(mMin.GetX(), mMin.GetY(), mMax.GetZ());
				break;

			case 2:
				outVertices[0] = Vec3(mMin.GetX(), mMin.GetY(), mMin.GetZ());
				outVertices[1] = Vec3(mMin.GetX(), mMax.GetY(), mMin.GetZ());
				outVertices[2] = Vec3(mMax.GetX(), mMax.GetY(), mMin.GetZ());
				outVertices[3] = Vec3(mMax.GetX(), mMin.GetY(), mMin.GetZ());
				break;
			}
		}
	}

	// Get the closest point on or in this box to inPoint
	RVec3 GetClosestPoint(Vec3Arg inPoint) const { return RVec3::sMin(RVec3::sMax(inPoint, mMin), mMax); }

	// Get the squared distance between inPoint and this box (will be 0 if in Point is inside the box)
	inline float GetSqDistanceTo(Vec3Arg inPoint) const { return (GetClosestPoint(inPoint) - inPoint).LengthSq(); }

	// Comparison operators
	bool operator == (const AABB3 &inRHS) const				{ return mMin == inRHS.mMin && mMax == inRHS.mMax; }
	bool operator != (const AABB3 &inRHS) const				{ return mMin != inRHS.mMin || mMax != inRHS.mMax; }

	// Bounding box min and max
	RVec3			mMin;
	RVec3			mMax;
};

MOSS_WARNINGS_END

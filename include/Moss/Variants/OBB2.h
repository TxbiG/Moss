#pragma once

#include <Moss/Physics/Geometry/Triangle.h>
#include <Moss/Physics/Geometry/IndexedTriangle.h>
#include <Moss/Physics/Geometry/AABox.h>
#include <Moss/Core/Variants/Matrix/Mat44.h>

MOSS_WARNINGS_BEGIN

class AABB2;

/// Oriented box 2D
class MOSS_EXPORT_GCC_BUG_WORKAROUND [[nodiscard]] OBB2
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
	OBB2() = default;
	OBB2(Mat44Arg inOrientation, Vec2Arg inHalfExtents) : mOrientation(inOrientation), mHalfExtents(inHalfExtents) { }

	/// Construct from axis aligned box and transform. Only works for rotation/translation matrix (no scaling / shearing).
	OBB2(Mat44Arg inOrientation, const AABB2& inBox) : OrientedBox(inOrientation.PreTranslated(inBox.GetCenter()), inBox.GetExtent()) { }

	/// Test if oriented box overlaps with axis aligned box each other
	bool Overlaps(const AABB2& inBox, float inEpsilon = 1.0e-6f) const {
		// Taken from: Real Time Collision Detection - Christer Ericson
		// Chapter 4.4.1, page 103-105.
		// Note that the code is swapped around: A is the aabox and B is the oriented box (this saves us from having to invert the orientation of the oriented box)

		// Convert AABox to center / extent representation
		RVec2 a_center = inBox.GetCenter();
		RVec2 a_half_extents = inBox.GetExtent();

		// Compute rotation matrix expressing b in a's coordinate frame
		Mat44 rot(mOrientation.GetColumn4(0), mOrientation.GetColumn4(1), mOrientation.GetColumn4(2), mOrientation.GetColumn4(3) - Vec4(a_center, 0));

		// Compute common subexpressions. Add in an epsilon term to
		// counteract arithmetic errors when two edges are parallel and
		// their cross product is (near) null (see text for details)
		RVec2 epsilon = RVec2::sReplicate(inEpsilon);
		RVec2 abs_r[3] { rot.GetAxisX().Abs() + epsilon, rot.GetAxisY().Abs()};

		// Test axes L = A0, L = A1, L = A2
		float ra, rb;
		for (int i = 0; i < 3; i++)
		{
			ra = a_half_extents[i];
			rb = mHalfExtents[0] * abs_r[0][i] + mHalfExtents[1] * abs_r[1][i] + mHalfExtents[2] * abs_r[2][i];
			if (abs(rot(i, 3)) > ra + rb) return false;
		}

		// Test axes L = B0, L = B1, L = B2
		for (int i = 0; i < 3; i++)
		{
			ra = a_half_extents.Dot(abs_r[i]);
			rb = mHalfExtents[i];
			if (abs(rot.GetTranslation().Dot(rot.GetColumn3(i))) > ra + rb) return false;
		}

		// Test axis L = A0 x B0
		ra = a_half_extents[1] * abs_r[0][2] + a_half_extents[2] * abs_r[0][1];
		rb = mHalfExtents[1] * abs_r[2][0] + mHalfExtents[2] * abs_r[1][0];
		if (abs(rot(2, 3) * rot(1, 0) - rot(1, 3) * rot(2, 0)) > ra + rb) return false;

		// Test axis L = A0 x B1
		ra = a_half_extents[1] * abs_r[1][2] + a_half_extents[2] * abs_r[1][1];
		rb = mHalfExtents[0] * abs_r[2][0] + mHalfExtents[2] * abs_r[0][0];
		if (abs(rot(2, 3) * rot(1, 1) - rot(1, 3) * rot(2, 1)) > ra + rb) return false;

		// Test axis L = A0 x B2
		ra = a_half_extents[1] * abs_r[2][2] + a_half_extents[2] * abs_r[2][1];
		rb = mHalfExtents[0] * abs_r[1][0] + mHalfExtents[1] * abs_r[0][0];
		if (abs(rot(2, 3) * rot(1, 2) - rot(1, 3) * rot(2, 2)) > ra + rb) return false;

		// Test axis L = A1 x B0
		ra = a_half_extents[0] * abs_r[0][2] + a_half_extents[2] * abs_r[0][0];
		rb = mHalfExtents[1] * abs_r[2][1] + mHalfExtents[2] * abs_r[1][1];
		if (abs(rot(0, 3) * rot(2, 0) - rot(2, 3) * rot(0, 0)) > ra + rb) return false;

		// Test axis L = A1 x B1
		ra = a_half_extents[0] * abs_r[1][2] + a_half_extents[2] * abs_r[1][0];
		rb = mHalfExtents[0] * abs_r[2][1] + mHalfExtents[2] * abs_r[0][1];
		if (abs(rot(0, 3) * rot(2, 1) - rot(2, 3) * rot(0, 1)) > ra + rb) return false;

		// Test axis L = A1 x B2
		ra = a_half_extents[0] * abs_r[2][2] + a_half_extents[2] * abs_r[2][0];
		rb = mHalfExtents[0] * abs_r[1][1] + mHalfExtents[1] * abs_r[0][1];
		if (abs(rot(0, 3) * rot(2, 2) - rot(2, 3) * rot(0, 2)) > ra + rb) return false;

		// Test axis L = A2 x B0
		ra = a_half_extents[0] * abs_r[0][1] + a_half_extents[1] * abs_r[0][0];
		rb = mHalfExtents[1] * abs_r[2][2] + mHalfExtents[2] * abs_r[1][2];
		if (abs(rot(1, 3) * rot(0, 0) - rot(0, 3) * rot(1, 0)) > ra + rb) return false;

		// Test axis L = A2 x B1
		ra = a_half_extents[0] * abs_r[1][1] + a_half_extents[1] * abs_r[1][0];
		rb = mHalfExtents[0] * abs_r[2][2] + mHalfExtents[2] * abs_r[0][2];
		if (abs(rot(1, 3) * rot(0, 1) - rot(0, 3) * rot(1, 1)) > ra + rb) return false;

		// Test axis L = A2 x B2
		ra = a_half_extents[0] * abs_r[2][1] + a_half_extents[1] * abs_r[2][0];
		rb = mHalfExtents[0] * abs_r[1][2] + mHalfExtents[1] * abs_r[0][2];
		if (abs(rot(1, 3) * rot(0, 2) - rot(0, 3) * rot(1, 2)) > ra + rb) return false;

		// Since no separating axis is found, the OBB and AAB must be intersecting
		return true;
	}

	/// Test if two oriented boxes overlap each other
	bool Overlaps(const OBB2& inBox, float inEpsilon = 1.0e-6f) const {
		// Taken from: Real Time Collision Detection - Christer Ericson
	// Chapter 4.4.1, page 103-105.
	// Note that A is this, B is inBox

	// Compute rotation matrix expressing b in a's coordinate frame
	Mat44 rot = mOrientation.InversedRotationTranslation() * inBox.mOrientation;

	// Compute common subexpressions. Add in an epsilon term to
	// counteract arithmetic errors when two edges are parallel and
	// their cross product is (near) null (see text for details)
	RVec2 epsilon = RVec2::sReplicate(inEpsilon);
	RVec2 abs_r[3] { rot.GetAxisX().Abs() + epsilon, rot.GetAxisY().Abs() + epsilon};

	// Test axes L = A0, L = A1, L = A2
	float ra, rb;
	for (int i = 0; i < 3; i++)
	{
		ra = mHalfExtents[i];
		rb = inBox.mHalfExtents[0] * abs_r[0][i] + inBox.mHalfExtents[1] * abs_r[1][i] + inBox.mHalfExtents[2] * abs_r[2][i];
		if (abs(rot(i, 3)) > ra + rb) return false;
	}

	// Test axes L = B0, L = B1, L = B2
	for (int i = 0; i < 3; i++)
	{
		ra = mHalfExtents.Dot(abs_r[i]);
		rb = inBox.mHalfExtents[i];
		if (abs(rot.GetTranslation().Dot(rot.GetColumn3(i))) > ra + rb) return false;
	}

	// Test axis L = A0 x B0
	ra = mHalfExtents[1] * abs_r[0][2] + mHalfExtents[2] * abs_r[0][1];
	rb = inBox.mHalfExtents[1] * abs_r[2][0] + inBox.mHalfExtents[2] * abs_r[1][0];
	if (abs(rot(2, 3) * rot(1, 0) - rot(1, 3) * rot(2, 0)) > ra + rb) return false;

	// Test axis L = A0 x B1
	ra = mHalfExtents[1] * abs_r[1][2] + mHalfExtents[2] * abs_r[1][1];
	rb = inBox.mHalfExtents[0] * abs_r[2][0] + inBox.mHalfExtents[2] * abs_r[0][0];
	if (abs(rot(2, 3) * rot(1, 1) - rot(1, 3) * rot(2, 1)) > ra + rb) return false;

	// Test axis L = A0 x B2
	ra = mHalfExtents[1] * abs_r[2][2] + mHalfExtents[2] * abs_r[2][1];
	rb = inBox.mHalfExtents[0] * abs_r[1][0] + inBox.mHalfExtents[1] * abs_r[0][0];
	if (abs(rot(2, 3) * rot(1, 2) - rot(1, 3) * rot(2, 2)) > ra + rb) return false;

	// Test axis L = A1 x B0
	ra = mHalfExtents[0] * abs_r[0][2] + mHalfExtents[2] * abs_r[0][0];
	rb = inBox.mHalfExtents[1] * abs_r[2][1] + inBox.mHalfExtents[2] * abs_r[1][1];
	if (abs(rot(0, 3) * rot(2, 0) - rot(2, 3) * rot(0, 0)) > ra + rb) return false;

	// Test axis L = A1 x B1
	ra = mHalfExtents[0] * abs_r[1][2] + mHalfExtents[2] * abs_r[1][0];
	rb = inBox.mHalfExtents[0] * abs_r[2][1] + inBox.mHalfExtents[2] * abs_r[0][1];
	if (abs(rot(0, 3) * rot(2, 1) - rot(2, 3) * rot(0, 1)) > ra + rb) return false;

	// Test axis L = A1 x B2
	ra = mHalfExtents[0] * abs_r[2][2] + mHalfExtents[2] * abs_r[2][0];
	rb = inBox.mHalfExtents[0] * abs_r[1][1] + inBox.mHalfExtents[1] * abs_r[0][1];
	if (abs(rot(0, 3) * rot(2, 2) - rot(2, 3) * rot(0, 2)) > ra + rb) return false;

	// Test axis L = A2 x B0
	ra = mHalfExtents[0] * abs_r[0][1] + mHalfExtents[1] * abs_r[0][0];
	rb = inBox.mHalfExtents[1] * abs_r[2][2] + inBox.mHalfExtents[2] * abs_r[1][2];
	if (abs(rot(1, 3) * rot(0, 0) - rot(0, 3) * rot(1, 0)) > ra + rb) return false;

	// Test axis L = A2 x B1
	ra = mHalfExtents[0] * abs_r[1][1] + mHalfExtents[1] * abs_r[1][0];
	rb = inBox.mHalfExtents[0] * abs_r[2][2] + inBox.mHalfExtents[2] * abs_r[0][2];
	if (abs(rot(1, 3) * rot(0, 1) - rot(0, 3) * rot(1, 1)) > ra + rb) return false;

	// Test axis L = A2 x B2
	ra = mHalfExtents[0] * abs_r[2][1] + mHalfExtents[1] * abs_r[2][0];
	rb = inBox.mHalfExtents[0] * abs_r[1][2] + inBox.mHalfExtents[1] * abs_r[0][2];
	if (abs(rot(1, 3) * rot(0, 2) - rot(0, 3) * rot(1, 2)) > ra + rb) return false;

	// Since no separating axis is found, the OBBs must be intersecting
	return true;
	}

	Mat44			mOrientation;														///< Transform that positions and rotates the local space axis aligned box into world space
	RVec2			mHalfExtents;														///< Half extents (half the size of the edge) of the local space axis aligned box
};

MOSS_WARNINGS_END

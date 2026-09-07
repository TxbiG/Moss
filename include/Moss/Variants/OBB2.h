#pragma once

#include <Moss/Geometry/Triangle.h>
#include <Moss/Variants/Matrix/Mat33.h>
#include <Moss/Variants/AABB2.h>

MOSS_SUPPRESS_WARNINGS_BEGIN

// Oriented box 2D
class MOSS_EXPORT_GCC_BUG_WORKAROUND [[nodiscard]] OBB2 {
public:
	MOSS_OVERRIDE_NEW_DELETE

	// Constructor
	OBB2() = default;
	OBB2(Mat33 inOrientation, RVec2 inHalfExtents) : mOrientation(inOrientation), mHalfExtents(inHalfExtents) { }

	// Construct from axis-aligned box and transform. Direct assignments to members fixed here:
	OBB2(Mat33 inOrientation, const AABB2& inBox) : mOrientation(inOrientation.PreTranslated(inBox.GetCenter())), mHalfExtents(inBox.GetExtent()) { }

	// Test if oriented box overlaps with axis aligned box each other
	bool Overlaps(const AABB2& inBox, float inEpsilon = 1.0e-6f) const {
		RVec2 a_center = inBox.GetCenter();
		RVec2 a_half_extents = inBox.GetExtent();

		// Get local axes and translation of the OBB (Column 2 is translation in 2D homogeneous Mat33)
		Vec3 b_pos = mOrientation.GetColumn2();
		RVec2 t = RVec2(b_pos.GetX(), b_pos.GetY()) - a_center;

		Vec3 c0 = mOrientation.GetColumn0();
		Vec3 c1 = mOrientation.GetColumn1();
		
		float r00 = abs(c0.GetX()) + inEpsilon;
		float r01 = abs(c1.GetX()) + inEpsilon;
		float r10 = abs(c0.GetY()) + inEpsilon;
		float r11 = abs(c1.GetY()) + inEpsilon;

		// Test Axis L = A0 (A's local X axis)
		if (abs(t.GetX()) > a_half_extents.GetX() + (mHalfExtents.GetX() * r00 + mHalfExtents.GetY() * r01)) return false;

		// Test Axis L = A1 (A's local Y axis)
		if (abs(t.GetY()) > a_half_extents.GetY() + (mHalfExtents.GetX() * r10 + mHalfExtents.GetY() * r11)) return false;

		// Test Axis L = B0 (B's local X axis)
		if (abs(t.GetX() * c0.GetX() + t.GetY() * c0.GetY()) > (a_half_extents.GetX() * r00 + a_half_extents.GetY() * r10) + mHalfExtents.GetX()) return false;

		// Test Axis L = B1 (B's local Y axis)
		if (abs(t.GetX() * c1.GetX() + t.GetY() * c1.GetY()) > (a_half_extents.GetX() * r01 + a_half_extents.GetY() * r11) + mHalfExtents.GetY()) return false;

		return true;
	}

	// Test if two oriented boxes overlap each other
	bool Overlaps(const OBB2& inBox, float inEpsilon = 1.0e-6f) const {
		Vec3 a_pos = mOrientation.GetColumn2();
		Vec3 b_pos = inBox.mOrientation.GetColumn2();
		RVec2 t_world = RVec2(b_pos.GetX(), b_pos.GetY()) - RVec2(a_pos.GetX(), a_pos.GetY());

		Vec3 a_c0 = mOrientation.GetColumn0();
		Vec3 a_c1 = mOrientation.GetColumn1();
		RVec2 t = RVec2(t_world.GetX() * a_c0.GetX() + t_world.GetY() * a_c0.GetY(),
		                t_world.GetX() * a_c1.GetX() + t_world.GetY() * a_c1.GetY());

		Vec3 b_c0 = inBox.mOrientation.GetColumn0();
		Vec3 b_c1 = inBox.mOrientation.GetColumn1();

		float r00 = abs(a_c0.GetX() * b_c0.GetX() + a_c0.GetY() * b_c0.GetY()) + inEpsilon;
		float r01 = abs(a_c0.GetX() * b_c1.GetX() + a_c0.GetY() * b_c1.GetY()) + inEpsilon;
		float r10 = abs(a_c1.GetX() * b_c0.GetX() + a_c1.GetY() * b_c0.GetY()) + inEpsilon;
		float r11 = abs(a_c1.GetX() * b_c1.GetX() + a_c1.GetY() * b_c1.GetY()) + inEpsilon;

		// Test Box A's axes
		if (abs(t.GetX()) > mHalfExtents.GetX() + (inBox.mHalfExtents.GetX() * r00 + inBox.mHalfExtents.GetY() * r01)) return false;
		if (abs(t.GetY()) > mHalfExtents.GetY() + (inBox.mHalfExtents.GetX() * r10 + inBox.mHalfExtents.GetY() * r11)) return false;

		// Test Box B's axes
		if (abs(t.GetX() * r00 + t.GetY() * r10) > (mHalfExtents.GetX() * r00 + mHalfExtents.GetY() * r10) + inBox.mHalfExtents.GetX()) return false;
		if (abs(t.GetX() * r01 + t.GetY() * r11) > (mHalfExtents.GetX() * r01 + mHalfExtents.GetY() * r11) + inBox.mHalfExtents.GetY()) return false;

		return true;
	}

	Mat33			mOrientation;		// Transform that positions and rotates the local space axis aligned box into world space
	RVec2			mHalfExtents;		// Half extents (half the size of the edge) of the local space axis aligned box
};

MOSS_SUPPRESS_WARNINGS_END
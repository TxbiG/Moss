#include <Moss/Moss_Renderer.h>



// Frustum2D

Frustum2D::Frustum2D(const Vec2& bottomLeft, const Vec2& topRight) { m_bounds = AABB2(Vec2(bottomLeft.x, bottomLeft.y), Vec2(topRight.x, topRight.y)); }

void Frustum2D::SetBounds(const Vec2& bottomLeft, const Vec2& topRight) { m_bounds = AABB2(Vec2(bottomLeft.x, bottomLeft.y), Vec2(topRight.x, topRight.y)); }

bool Frustum2D::Overlaps(const AABB2& box) const { return m_bounds.Overlaps(box); }

const AABB2& Frustum2D::GetBounds() const { return m_bounds; }



// Plane3D

explicit Plane::Plane(Vec4Arg inNormalAndConstant)	: mNormalAndConstant(inNormalAndConstant) { }
Plane::Plane(Vec3Arg inNormal, float inConstant)	: mNormalAndConstant(inNormal, inConstant) { }

// Create from point and normal
static Plane Plane::sFromPointAndNormal(Vec3Arg inPoint, Vec3Arg inNormal) { return Plane(Vec4(inNormal, -inNormal.Dot(inPoint))); }

// Create from point and normal, double precision version that more accurately calculates the plane constant
static Plane Plane::sFromPointAndNormal(DVec3Arg inPoint, Vec3Arg inNormal) { return Plane(Vec4(inNormal, -float(DVec3(inNormal).Dot(inPoint)))); }

// Create from 3 counter clockwise points
static Plane Plane::sFromPointsCCW(Vec3Arg inV1, Vec3Arg inV2, Vec3Arg inV3) { return sFromPointAndNormal(inV1, (inV2 - inV1).Cross(inV3 - inV1).Normalized()); }

// Properties
Vec3 Plane::GetNormal() const	 	{ return Vec3(mNormalAndConstant); }
void Plane::SetNormal(Vec3Arg inNormal)	 	{ mNormalAndConstant = Vec4(inNormal, mNormalAndConstant.GetW()); }
float Plane::GetConstant() const	  	{ return mNormalAndConstant.GetW(); }
void Plane::SetConstant(float inConstant) 	{ mNormalAndConstant.SetW(inConstant); }

// Offset the plane (positive value means move it in the direction of the plane normal)
Plane Offset(float inDistance) const	{ return Plane(mNormalAndConstant - Vec4(Vec3::sZero(), inDistance)); }

// Transform the plane by a matrix
inline Plane Plane::GetTransformed(Mat44Arg inTransform) const {
	Vec3 transformed_normal = inTransform.Multiply3x3(GetNormal());
	return Plane(transformed_normal, GetConstant() - inTransform.GetTranslation().Dot(transformed_normal));
}

	// Scale the plane, can handle non-uniform and negative scaling
inline Plane Plane::Scaled(Vec3Arg inScale) const {
	Vec3 scaled_normal = GetNormal() / inScale;
	float scaled_normal_length = scaled_normal.Length();
	return Plane(scaled_normal / scaled_normal_length, GetConstant() / scaled_normal_length);
}

	// Distance point to plane
float Plane::SignedDistance(Vec3Arg inPoint) const	{ return inPoint.Dot(GetNormal()) + GetConstant(); }

	// Project inPoint onto the plane
Vec3 Plane::ProjectPointOnPlane(Vec3Arg inPoint) const	{ return inPoint - GetNormal() * SignedDistance(inPoint); }

	// Returns intersection point between 3 planes
static bool	Plane::sIntersectPlanes(const Plane &inP1, const Plane &inP2, const Plane &inP3, Vec3 &outPoint) {
	// We solve the equation:
	// |ax, ay, az, aw|   | x |   | 0 |
	// |bx, by, bz, bw| * | y | = | 0 |
	// |cx, cy, cz, cw|   | z |   | 0 |
	// | 0,	 0,	 0,	 1|   | 1 |   | 1 |
	// Where normal of plane 1 = (ax, ay, az), plane constant of 1 = aw, normal of plane 2 = (bx, by, bz) etc.
	// This involves inverting the matrix and multiplying it with [0, 0, 0, 1]

	// Fetch the normals and plane constants for the three planes
	Vec4 a = inP1.mNormalAndConstant;
	Vec4 b = inP2.mNormalAndConstant;
	Vec4 c = inP3.mNormalAndConstant;

	// Result is a vector that we have to divide by:
	float denominator = Vec3(a).Dot(Vec3(b).Cross(Vec3(c)));
	if (denominator == 0.0f)
	return false;

	// The numerator is:
	// [aw*(bz*cy-by*cz)+ay*(bw*cz-bz*cw)+az*(by*cw-bw*cy)]
	// [aw*(bx*cz-bz*cx)+ax*(bz*cw-bw*cz)+az*(bw*cx-bx*cw)]
	// [aw*(by*cx-bx*cy)+ax*(bw*cy-by*cw)+ay*(bx*cw-bw*cx)]
	Vec4 numerator = a.SplatW() * (b.Swizzle<SWIZZLE_Z, SWIZZLE_X, SWIZZLE_Y, SWIZZLE_UNUSED>() * c.Swizzle<SWIZZLE_Y, SWIZZLE_Z, SWIZZLE_X, SWIZZLE_UNUSED>() - b.Swizzle<SWIZZLE_Y, SWIZZLE_Z, SWIZZLE_X, SWIZZLE_UNUSED>() * c.Swizzle<SWIZZLE_Z, SWIZZLE_X, SWIZZLE_Y, SWIZZLE_UNUSED>())
	+ a.Swizzle<SWIZZLE_Y, SWIZZLE_X, SWIZZLE_X, SWIZZLE_UNUSED>() * (b.Swizzle<SWIZZLE_W, SWIZZLE_Z, SWIZZLE_W, SWIZZLE_UNUSED>() * c.Swizzle<SWIZZLE_Z, SWIZZLE_W, SWIZZLE_Y, SWIZZLE_UNUSED>() - b.Swizzle<SWIZZLE_Z, SWIZZLE_W, SWIZZLE_Y, SWIZZLE_UNUSED>() * c.Swizzle<SWIZZLE_W, SWIZZLE_Z, SWIZZLE_W, SWIZZLE_UNUSED>())
	+ a.Swizzle<SWIZZLE_Z, SWIZZLE_Z, SWIZZLE_Y, SWIZZLE_UNUSED>() * (b.Swizzle<SWIZZLE_Y, SWIZZLE_W, SWIZZLE_X, SWIZZLE_UNUSED>() * c.Swizzle<SWIZZLE_W, SWIZZLE_X, SWIZZLE_W, SWIZZLE_UNUSED>() - b.Swizzle<SWIZZLE_W, SWIZZLE_X, SWIZZLE_W, SWIZZLE_UNUSED>() * c.Swizzle<SWIZZLE_Y, SWIZZLE_W, SWIZZLE_X, SWIZZLE_UNUSED>());

	outPoint = Vec3(numerator) / denominator;
	return true;
}


// Frustum3D

inline Frustum3D::Frustum3D(Vec3 inPosition, Vec3 inForward, Vec3 inUp, float inFOVX, float inFOVY, float inNear) {
	Vec3 right = inForward.Cross(inUp).Normalized();
	Vec3 up = right.Cross(inForward).Normalized(); // Calculate the real up vector (inUp does not need to be perpendicular to inForward)

	// Near plane
	m_Planes[0] = Plane::sFromPointAndNormal(inPosition + inNear * inForward, inForward);

	// Top and bottom planes
	m_Planes[1] = Plane::sFromPointAndNormal(inPosition, Mat44::sRotation(right, 0.5f * inFOVY) * -up);
	m_Planes[2] = Plane::sFromPointAndNormal(inPosition, Mat44::sRotation(right, -0.5f * inFOVY) * up);

	// Left and right planes
	m_Planes[3] = Plane::sFromPointAndNormal(inPosition, Mat44::sRotation(up, 0.5f * inFOVX) * right);
	m_Planes[4] = Plane::sFromPointAndNormal(inPosition, Mat44::sRotation(up, -0.5f * inFOVX) * -right);
}

inline bool	Frustum3D::Overlaps(const AABox &inBox) const {
	// Loop over all frustum planes
	for (const Plane &p : m_Planes)
	{
		// Get support point (the maximum extent) in the direction of our normal
		Vec3 support = inBox.GetSupport(p.GetNormal());

		// If this is behind our plane, the box is not inside the frustum
		if (p.SignedDistance(support) < 0.0f) { return false; }
	}

	return true;
}
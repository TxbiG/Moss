// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Core/Variants/Math/FindRoot.h>

MOSS_NAMESPACE_BEGIN

/// Tests a ray starting at inRayOrigin and extending infinitely in inRayDirection
/// against an infinite cylinder centered along the Y axis
/// @return FLT_MAX if there is no intersection, otherwise the fraction along the ray.
/// @param inRayDirection Direction of the ray. Does not need to be normalized.
/// @param inRayOrigin Origin of the ray. If the ray starts inside the cylinder, the returned fraction will be 0.
/// @param inCylinderRadius Radius of the infinite cylinder
MOSS_INLINE float RayCylinder(Vec3Arg inRayOrigin, Vec3Arg inRayDirection, float inCylinderRadius) {
	// Remove Y component of ray to see of ray intersects with infinite cylinder
	UVec4 mask_y = UVec4(0, 0xffffffff, 0, 0);
	Vec3 origin_xz = Vec3::sSelect(inRayOrigin, Vec3::sZero(), mask_y);
	float origin_xz_len_sq = origin_xz.LengthSq();
	float r_sq = Square(inCylinderRadius);
	if (origin_xz_len_sq > r_sq) {
		// Ray starts outside of the infinite cylinder
		// Solve: |RayOrigin_xz + fraction * RayDirection_xz|^2 = r^2 to find fraction
		Vec3 direction_xz = Vec3::sSelect(inRayDirection, Vec3::sZero(), mask_y);
		float a = direction_xz.LengthSq();
		float b = 2.0f * origin_xz.Dot(direction_xz);
		float c = origin_xz_len_sq - r_sq;
		float fraction1, fraction2;
		if (FindRoot(a, b, c, fraction1, fraction2) == 0)
			return FLT_MAX; // No intersection with infinite cylinder

		// Get fraction corresponding to the ray entering the circle
		float fraction = min(fraction1, fraction2);
		if (fraction >= 0.0f)
			return fraction;
	}
	else { return 0.0f; } // Ray starts inside the infinite cylinder

	// No collision
	return FLT_MAX;
}

/// Test a ray against a cylinder centered around the origin with its axis along the Y axis and half height specified.
/// @return FLT_MAX if there is no intersection, otherwise the fraction along the ray.
/// @param inRayDirection Ray direction. Does not need to be normalized.
/// @param inRayOrigin Origin of the ray. If the ray starts inside the cylinder, the returned fraction will be 0.
/// @param inCylinderRadius Radius of the cylinder
/// @param inCylinderHalfHeight Distance from the origin to the top (or bottom) of the cylinder
MOSS_INLINE float RayCylinder(Vec3Arg inRayOrigin, Vec3Arg inRayDirection, float inCylinderHalfHeight, float inCylinderRadius) {
	// Test infinite cylinder
	float fraction = RayCylinder(inRayOrigin, inRayDirection, inCylinderRadius);
	if (fraction == FLT_MAX)
		return FLT_MAX;

	// If this hit is in the finite cylinder we have our fraction
	if (abs(inRayOrigin.GetY() + fraction * inRayDirection.GetY()) <= inCylinderHalfHeight)
		return fraction;

	// Check if ray could hit the top or bottom plane of the cylinder
	float direction_y = inRayDirection.GetY();
	if (direction_y != 0.0f) {
		// Solving line equation: x = ray_origin + fraction * ray_direction
		// and plane equation: plane_normal . x + plane_constant = 0
		// fraction = (-plane_constant - plane_normal . ray_origin) / (plane_normal . ray_direction)
		// when the ray_direction.y < 0:
		// plane_constant = -cylinder_half_height, plane_normal = (0, 1, 0)
		// else
		// plane_constant = -cylinder_half_height, plane_normal = (0, -1, 0)
		float origin_y = inRayOrigin.GetY();
		float plane_fraction;
		if (direction_y < 0.0f)
			plane_fraction = (inCylinderHalfHeight - origin_y) / direction_y;
		else
			plane_fraction = -(inCylinderHalfHeight + origin_y) / direction_y;

		// Check if the hit is in front of the ray
		if (plane_fraction >= 0.0f) {
			// Test if this hit is inside the cylinder
			Vec3 point = inRayOrigin + plane_fraction * inRayDirection;
			float dist_sq = Square(point.GetX()) + Square(point.GetZ());
			if (dist_sq <= Square(inCylinderRadius))
				return plane_fraction;
		}
	}

	// No collision
	return FLT_MAX;
}

/// Tests a ray starting at inRayOrigin and extending infinitely in inRayDirection against a sphere,
/// @return FLT_MAX if there is no intersection, otherwise the fraction along the ray.
/// @param inRayOrigin Ray origin. If the ray starts inside the sphere, the returned fraction will be 0.
/// @param inRayDirection Ray direction. Does not need to be normalized.
/// @param inSphereCenter Position of the center of the sphere
/// @param inSphereRadius Radius of the sphere
MOSS_INLINE float RaySphere(Vec3Arg inRayOrigin, Vec3Arg inRayDirection, Vec3Arg inSphereCenter, float inSphereRadius) {
	// Solve: |RayOrigin + fraction * RayDirection - SphereCenter|^2 = SphereRadius^2 for fraction
	Vec3 center_origin = inRayOrigin - inSphereCenter;
	float a = inRayDirection.LengthSq();
	float b = 2.0f * inRayDirection.Dot(center_origin);
	float c = center_origin.LengthSq() - inSphereRadius * inSphereRadius;
	float fraction1, fraction2;
	if (FindRoot(a, b, c, fraction1, fraction2) == 0)
		return c <= 0.0f? 0.0f : FLT_MAX; // Return if origin is inside the sphere

	// Sort so that the smallest is first
	if (fraction1 > fraction2)
		std::swap(fraction1, fraction2);

	// Test solution with lowest fraction, this will be the ray entering the sphere
	if (fraction1 >= 0.0f)
		return fraction1; // Sphere is before the ray start

	// Test solution with highest fraction, this will be the ray leaving the sphere
	if (fraction2 >= 0.0f)
		return 0.0f; // We start inside the sphere

	// No solution
	return FLT_MAX;
}

/// Tests a ray starting at inRayOrigin and extending infinitely in inRayDirection against a sphere.
/// Outputs entry and exit points (outMinFraction and outMaxFraction) along the ray (which could be negative if the hit point is before the start of the ray).
/// @param inRayOrigin Ray origin. If the ray starts inside the sphere, the returned fraction will be 0.
/// @param inRayDirection Ray direction. Does not need to be normalized.
/// @param inSphereCenter Position of the center of the sphere.
/// @param inSphereRadius Radius of the sphere.
/// @param outMinFraction Returned lowest intersection fraction
/// @param outMaxFraction Returned highest intersection fraction
/// @return The amount of intersections with the sphere.
/// If 1 intersection is returned outMinFraction will be equal to outMaxFraction
MOSS_INLINE int RaySphere(Vec3Arg inRayOrigin, Vec3Arg inRayDirection, Vec3Arg inSphereCenter, float inSphereRadius, float &outMinFraction, float &outMaxFraction) {
	// Solve: |RayOrigin + fraction * RayDirection - SphereCenter|^2 = SphereRadius^2 for fraction
	Vec3 center_origin = inRayOrigin - inSphereCenter;
	float a = inRayDirection.LengthSq();
	float b = 2.0f * inRayDirection.Dot(center_origin);
	float c = center_origin.LengthSq() - inSphereRadius * inSphereRadius;
	float fraction1, fraction2;
	switch (FindRoot(a, b, c, fraction1, fraction2))
	{
	case 0:
		if (c <= 0.0f) {
			// Origin inside sphere
			outMinFraction = outMaxFraction = 0.0f;
			return 1;
		}
		else { return 0; } // Origin outside of the sphere
		break;

	case 1:
		// Ray is touching the sphere
		outMinFraction = outMaxFraction = fraction1;
		return 1;

	default:
		// Ray enters and exits the sphere
		// Sort so that the smallest is first
		if (fraction1 > fraction2)
			std::swap(fraction1, fraction2);

		outMinFraction = fraction1;
		outMaxFraction = fraction2;
		return 2;
	}
}

/// Tests a ray starting at inRayOrigin and extending infinitely in inRayDirection
/// against a capsule centered around the origin with its axis along the Y axis and half height specified.
/// @return FLT_MAX if there is no intersection, otherwise the fraction along the ray.
/// @param inRayDirection Ray direction. Does not need to be normalized.
/// @param inRayOrigin Origin of the ray. If the ray starts inside the capsule, the returned fraction will be 0.
/// @param inCapsuleHalfHeight Distance from the origin to the center of the top sphere (or that of the bottom)
/// @param inCapsuleRadius Radius of the top/bottom sphere
MOSS_INLINE float RayCapsule(Vec3Arg inRayOrigin, Vec3Arg inRayDirection, float inCapsuleHalfHeight, float inCapsuleRadius) {
	// Test infinite cylinder
	float cylinder = RayCylinder(inRayOrigin, inRayDirection, inCapsuleRadius);
	if (cylinder == FLT_MAX)
		return FLT_MAX;

	// If this hit is in the finite cylinder we have our fraction
	if (abs(inRayOrigin.GetY() + cylinder * inRayDirection.GetY()) <= inCapsuleHalfHeight)
		return cylinder;

	// Test upper and lower sphere
	Vec3 sphere_center(0, inCapsuleHalfHeight, 0);
	float upper = RaySphere(inRayOrigin, inRayDirection, sphere_center, inCapsuleRadius);
	float lower = RaySphere(inRayOrigin, inRayDirection, -sphere_center, inCapsuleRadius);
	return min(upper, lower);
}

/// Intersect ray with triangle, returns closest point or FLT_MAX if no hit (branch less version)
/// Adapted from: http://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
MOSS_INLINE float RayTriangle(Vec3Arg inOrigin, Vec3Arg inDirection, Vec3Arg inV0, Vec3Arg inV1, Vec3Arg inV2) {
	// Epsilon
	Vec3 epsilon = Vec3::sReplicate(1.0e-12f);

	// Zero & one
	Vec3 zero = Vec3::sZero();
	Vec3 one = Vec3::sOne();

	// Find vectors for two edges sharing inV0
	Vec3 e1 = inV1 - inV0;
	Vec3 e2 = inV2 - inV0;

	// Begin calculating determinant - also used to calculate u parameter
	Vec3 p = inDirection.Cross(e2);

	// if determinant is near zero, ray lies in plane of triangle
	Vec3 det = Vec3::sReplicate(e1.Dot(p));

	// Check if determinant is near zero
	UVec4 det_near_zero = Vec3::sLess(det.Abs(), epsilon);

	// When the determinant is near zero, set it to one to avoid dividing by zero
	det = Vec3::sSelect(det, Vec3::sOne(), det_near_zero);

	// Calculate distance from inV0 to ray origin
	Vec3 s = inOrigin - inV0;

	// Calculate u parameter
	Vec3 u = Vec3::sReplicate(s.Dot(p)) / det;

	// Prepare to test v parameter
	Vec3 q = s.Cross(e1);

	// Calculate v parameter
	Vec3 v = Vec3::sReplicate(inDirection.Dot(q)) / det;

	// Get intersection point
	Vec3 t = Vec3::sReplicate(e2.Dot(q)) / det;

	// Check if there is an intersection
	UVec4 no_intersection = UVec4::sOr(UVec4::sOr(UVec4::sOr(det_near_zero, Vec3::sLess(u, zero)), UVec4::sOr(Vec3::sLess(v, zero), Vec3::sGreater(u + v, one))), Vec3::sLess(t, zero));

	// Select intersection point or FLT_MAX based on if there is an intersection or not
	return Vec3::sSelect(t, Vec3::sReplicate(FLT_MAX), no_intersection).GetX();
}

/// Intersect ray with 4 triangles in SOA format, returns 4 vector of closest points or FLT_MAX if no hit (uses bit tricks to do less divisions)
MOSS_INLINE Vec4 RayTriangle4(Vec3Arg inOrigin, Vec3Arg inDirection, Vec4Arg inV0X, Vec4Arg inV0Y, Vec4Arg inV0Z, Vec4Arg inV1X, Vec4Arg inV1Y, Vec4Arg inV1Z, Vec4Arg inV2X, Vec4Arg inV2Y, Vec4Arg inV2Z) {
	// Epsilon
	Vec4 epsilon = Vec4::sReplicate(1.0e-12f);

	// Zero
	Vec4 zero = Vec4::sZero();

	// Find vectors for two edges sharing inV0
	Vec4 e1x = inV1X - inV0X;
	Vec4 e1y = inV1Y - inV0Y;
	Vec4 e1z = inV1Z - inV0Z;
	Vec4 e2x = inV2X - inV0X;
	Vec4 e2y = inV2Y - inV0Y;
	Vec4 e2z = inV2Z - inV0Z;

	// Get direction vector components
	Vec4 dx = inDirection.SplatX();
	Vec4 dy = inDirection.SplatY();
	Vec4 dz = inDirection.SplatZ();

	// Begin calculating determinant - also used to calculate u parameter
	Vec4 px = dy * e2z - dz * e2y;
	Vec4 py = dz * e2x - dx * e2z;
	Vec4 pz = dx * e2y - dy * e2x;

	// if determinant is near zero, ray lies in plane of triangle
	Vec4 det = e1x * px + e1y * py + e1z * pz;

	// Get sign bit for determinant and make positive
	Vec4 det_sign = Vec4::sAnd(det, UVec4::sReplicate(0x80000000).ReinterpretAsFloat());
	det = Vec4::sXor(det, det_sign);

	// Check which determinants are near zero
	UVec4 det_near_zero = Vec4::sLess(det, epsilon);

	// Set components of the determinant to 1 that are near zero to avoid dividing by zero
	det = Vec4::sSelect(det, Vec4::sOne(), det_near_zero);

	// Calculate distance from inV0 to ray origin
	Vec4 sx = inOrigin.SplatX() - inV0X;
	Vec4 sy = inOrigin.SplatY() - inV0Y;
	Vec4 sz = inOrigin.SplatZ() - inV0Z;

	// Calculate u parameter and flip sign if determinant was negative
	Vec4 u = Vec4::sXor(sx * px + sy * py + sz * pz, det_sign);

	// Prepare to test v parameter
	Vec4 qx = sy * e1z - sz * e1y;
	Vec4 qy = sz * e1x - sx * e1z;
	Vec4 qz = sx * e1y - sy * e1x;

	// Calculate v parameter and flip sign if determinant was negative
	Vec4 v = Vec4::sXor(dx * qx + dy * qy + dz * qz, det_sign);

	// Get intersection point and flip sign if determinant was negative
	Vec4 t = Vec4::sXor(e2x * qx + e2y * qy + e2z * qz, det_sign);

	// Check if there is an intersection
	UVec4 no_intersection = UVec4::sOr(UVec4::sOr(UVec4::sOr(det_near_zero,Vec4::sLess(u, zero)), UVec4::sOr(Vec4::sLess(v, zero),Vec4::sGreater(u + v, det))), Vec4::sLess(t, zero));

	// Select intersection point or FLT_MAX based on if there is an intersection or not
	return Vec4::sSelect(t / det, Vec4::sReplicate(FLT_MAX), no_intersection);
}



// Helper structure holding the reciprocal of a ray for Ray vs AABox testing
class RayInvDirection {
public:
	/// Constructors
	inline			RayInvDirection() = default;
	inline explicit	RayInvDirection(Vec3Arg inDirection) { Set(inDirection); }

	/// Set reciprocal from ray direction
	inline void Set(Vec3Arg inDirection) {
		// if (abs(inDirection) <= Epsilon) the ray is nearly parallel to the slab.
		mIsParallel = Vec3::sLessOrEqual(inDirection.Abs(), Vec3::sReplicate(1.0e-20f));

		// Calculate 1 / direction while avoiding division by zero
		mInvDirection = Vec3::sSelect(inDirection, Vec3::sOne(), mIsParallel).Reciprocal();
	}

	Vec3			mInvDirection;					///< 1 / ray direction
	UVec4			mIsParallel;					///< for each component if it is parallel to the coordinate axis
};

/// Intersect AABB with ray, returns minimal distance along ray or FLT_MAX if no hit
/// Note: Can return negative value if ray starts in box
MOSS_INLINE float RayAABox(Vec3Arg inOrigin, const RayInvDirection &inInvDirection, Vec3Arg inBoundsMin, Vec3Arg inBoundsMax) {
	// Constants
	Vec3 flt_min = Vec3::sReplicate(-FLT_MAX);
	Vec3 flt_max = Vec3::sReplicate(FLT_MAX);

	// Test against all three axes simultaneously.
	Vec3 t1 = (inBoundsMin - inOrigin) * inInvDirection.mInvDirection;
	Vec3 t2 = (inBoundsMax - inOrigin) * inInvDirection.mInvDirection;

	// Compute the max of min(t1,t2) and the min of max(t1,t2) ensuring we don't
	// use the results from any directions parallel to the slab.
	Vec3 t_min = Vec3::sSelect(Vec3::sMin(t1, t2), flt_min, inInvDirection.mIsParallel);
	Vec3 t_max = Vec3::sSelect(Vec3::sMax(t1, t2), flt_max, inInvDirection.mIsParallel);

	// t_min.xyz = maximum(t_min.x, t_min.y, t_min.z);
	t_min = Vec3::sMax(t_min, t_min.Swizzle<SWIZZLE_Y, SWIZZLE_Z, SWIZZLE_X>());
	t_min = Vec3::sMax(t_min, t_min.Swizzle<SWIZZLE_Z, SWIZZLE_X, SWIZZLE_Y>());

	// t_max.xyz = minimum(t_max.x, t_max.y, t_max.z);
	t_max = Vec3::sMin(t_max, t_max.Swizzle<SWIZZLE_Y, SWIZZLE_Z, SWIZZLE_X>());
	t_max = Vec3::sMin(t_max, t_max.Swizzle<SWIZZLE_Z, SWIZZLE_X, SWIZZLE_Y>());

	// if (t_min > t_max) return FLT_MAX;
	UVec4 no_intersection = Vec3::sGreater(t_min, t_max);

	// if (t_max < 0.0f) return FLT_MAX;
	no_intersection = UVec4::sOr(no_intersection, Vec3::sLess(t_max, Vec3::sZero()));

	// if (inInvDirection.mIsParallel && !(Min <= inOrigin && inOrigin <= Max)) return FLT_MAX; else return t_min;
	UVec4 no_parallel_overlap = UVec4::sOr(Vec3::sLess(inOrigin, inBoundsMin), Vec3::sGreater(inOrigin, inBoundsMax));
	no_intersection = UVec4::sOr(no_intersection, UVec4::sAnd(inInvDirection.mIsParallel, no_parallel_overlap));
	no_intersection = UVec4::sOr(no_intersection, no_intersection.SplatY());
	no_intersection = UVec4::sOr(no_intersection, no_intersection.SplatZ());
	return Vec3::sSelect(t_min, flt_max, no_intersection).GetX();
}

/// Intersect 4 AABBs with ray, returns minimal distance along ray or FLT_MAX if no hit
/// Note: Can return negative value if ray starts in box
MOSS_INLINE Vec4 RayAABox4(Vec3Arg inOrigin, const RayInvDirection &inInvDirection, Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ) {
	// Constants
	Vec4 flt_min = Vec4::sReplicate(-FLT_MAX);
	Vec4 flt_max = Vec4::sReplicate(FLT_MAX);

	// Origin
	Vec4 originx = inOrigin.SplatX();
	Vec4 originy = inOrigin.SplatY();
	Vec4 originz = inOrigin.SplatZ();

	// Parallel
	UVec4 parallelx = inInvDirection.mIsParallel.SplatX();
	UVec4 parallely = inInvDirection.mIsParallel.SplatY();
	UVec4 parallelz = inInvDirection.mIsParallel.SplatZ();

	// Inverse direction
	Vec4 invdirx = inInvDirection.mInvDirection.SplatX();
	Vec4 invdiry = inInvDirection.mInvDirection.SplatY();
	Vec4 invdirz = inInvDirection.mInvDirection.SplatZ();

	// Test against all three axes simultaneously.
	Vec4 t1x = (inBoundsMinX - originx) * invdirx;
	Vec4 t1y = (inBoundsMinY - originy) * invdiry;
	Vec4 t1z = (inBoundsMinZ - originz) * invdirz;
	Vec4 t2x = (inBoundsMaxX - originx) * invdirx;
	Vec4 t2y = (inBoundsMaxY - originy) * invdiry;
	Vec4 t2z = (inBoundsMaxZ - originz) * invdirz;

	// Compute the max of min(t1,t2) and the min of max(t1,t2) ensuring we don't
	// use the results from any directions parallel to the slab.
	Vec4 t_minx = Vec4::sSelect(Vec4::sMin(t1x, t2x), flt_min, parallelx);
	Vec4 t_miny = Vec4::sSelect(Vec4::sMin(t1y, t2y), flt_min, parallely);
	Vec4 t_minz = Vec4::sSelect(Vec4::sMin(t1z, t2z), flt_min, parallelz);
	Vec4 t_maxx = Vec4::sSelect(Vec4::sMax(t1x, t2x), flt_max, parallelx);
	Vec4 t_maxy = Vec4::sSelect(Vec4::sMax(t1y, t2y), flt_max, parallely);
	Vec4 t_maxz = Vec4::sSelect(Vec4::sMax(t1z, t2z), flt_max, parallelz);

	// t_min.xyz = maximum(t_min.x, t_min.y, t_min.z);
	Vec4 t_min = Vec4::sMax(Vec4::sMax(t_minx, t_miny), t_minz);

	// t_max.xyz = minimum(t_max.x, t_max.y, t_max.z);
	Vec4 t_max = Vec4::sMin(Vec4::sMin(t_maxx, t_maxy), t_maxz);

	// if (t_min > t_max) return FLT_MAX;
	UVec4 no_intersection = Vec4::sGreater(t_min, t_max);

	// if (t_max < 0.0f) return FLT_MAX;
	no_intersection = UVec4::sOr(no_intersection, Vec4::sLess(t_max, Vec4::sZero()));

	// if bounds are invalid return FLOAT_MAX;
	UVec4 bounds_invalid = UVec4::sOr(UVec4::sOr(Vec4::sGreater(inBoundsMinX, inBoundsMaxX), Vec4::sGreater(inBoundsMinY, inBoundsMaxY)), Vec4::sGreater(inBoundsMinZ, inBoundsMaxZ));
	no_intersection = UVec4::sOr(no_intersection, bounds_invalid);

	// if (inInvDirection.mIsParallel && !(Min <= inOrigin && inOrigin <= Max)) return FLT_MAX; else return t_min;
	UVec4 no_parallel_overlapx = UVec4::sAnd(parallelx, UVec4::sOr(Vec4::sLess(originx, inBoundsMinX), Vec4::sGreater(originx, inBoundsMaxX)));
	UVec4 no_parallel_overlapy = UVec4::sAnd(parallely, UVec4::sOr(Vec4::sLess(originy, inBoundsMinY), Vec4::sGreater(originy, inBoundsMaxY)));
	UVec4 no_parallel_overlapz = UVec4::sAnd(parallelz, UVec4::sOr(Vec4::sLess(originz, inBoundsMinZ), Vec4::sGreater(originz, inBoundsMaxZ)));
	no_intersection = UVec4::sOr(no_intersection, UVec4::sOr(UVec4::sOr(no_parallel_overlapx, no_parallel_overlapy), no_parallel_overlapz));
	return Vec4::sSelect(t_min, flt_max, no_intersection);
}

/// Intersect AABB with ray, returns minimal and maximal distance along ray or FLT_MAX, -FLT_MAX if no hit
/// Note: Can return negative value for outMin if ray starts in box
MOSS_INLINE void RayAABox(Vec3Arg inOrigin, const RayInvDirection &inInvDirection, Vec3Arg inBoundsMin, Vec3Arg inBoundsMax, float &outMin, float &outMax) {
	// Constants
	Vec3 flt_min = Vec3::sReplicate(-FLT_MAX);
	Vec3 flt_max = Vec3::sReplicate(FLT_MAX);

	// Test against all three axes simultaneously.
	Vec3 t1 = (inBoundsMin - inOrigin) * inInvDirection.mInvDirection;
	Vec3 t2 = (inBoundsMax - inOrigin) * inInvDirection.mInvDirection;

	// Compute the max of min(t1,t2) and the min of max(t1,t2) ensuring we don't
	// use the results from any directions parallel to the slab.
	Vec3 t_min = Vec3::sSelect(Vec3::sMin(t1, t2), flt_min, inInvDirection.mIsParallel);
	Vec3 t_max = Vec3::sSelect(Vec3::sMax(t1, t2), flt_max, inInvDirection.mIsParallel);

	// t_min.xyz = maximum(t_min.x, t_min.y, t_min.z);
	t_min = Vec3::sMax(t_min, t_min.Swizzle<SWIZZLE_Y, SWIZZLE_Z, SWIZZLE_X>());
	t_min = Vec3::sMax(t_min, t_min.Swizzle<SWIZZLE_Z, SWIZZLE_X, SWIZZLE_Y>());

	// t_max.xyz = minimum(t_max.x, t_max.y, t_max.z);
	t_max = Vec3::sMin(t_max, t_max.Swizzle<SWIZZLE_Y, SWIZZLE_Z, SWIZZLE_X>());
	t_max = Vec3::sMin(t_max, t_max.Swizzle<SWIZZLE_Z, SWIZZLE_X, SWIZZLE_Y>());

	// if (t_min > t_max) return FLT_MAX;
	UVec4 no_intersection = Vec3::sGreater(t_min, t_max);

	// if (t_max < 0.0f) return FLT_MAX;
	no_intersection = UVec4::sOr(no_intersection, Vec3::sLess(t_max, Vec3::sZero()));

	// if (inInvDirection.mIsParallel && !(Min <= inOrigin && inOrigin <= Max)) return FLT_MAX; else return t_min;
	UVec4 no_parallel_overlap = UVec4::sOr(Vec3::sLess(inOrigin, inBoundsMin), Vec3::sGreater(inOrigin, inBoundsMax));
	no_intersection = UVec4::sOr(no_intersection, UVec4::sAnd(inInvDirection.mIsParallel, no_parallel_overlap));
	no_intersection = UVec4::sOr(no_intersection, no_intersection.SplatY());
	no_intersection = UVec4::sOr(no_intersection, no_intersection.SplatZ());
	outMin = Vec3::sSelect(t_min, flt_max, no_intersection).GetX();
	outMax = Vec3::sSelect(t_max, flt_min, no_intersection).GetX();
}

/// Intersect AABB with ray, returns true if there is a hit closer than inClosest
MOSS_INLINE bool RayAABoxHits(Vec3Arg inOrigin, const RayInvDirection &inInvDirection, Vec3Arg inBoundsMin, Vec3Arg inBoundsMax, float inClosest) {
	// Constants
	Vec3 flt_min = Vec3::sReplicate(-FLT_MAX);
	Vec3 flt_max = Vec3::sReplicate(FLT_MAX);

	// Test against all three axes simultaneously.
	Vec3 t1 = (inBoundsMin - inOrigin) * inInvDirection.mInvDirection;
	Vec3 t2 = (inBoundsMax - inOrigin) * inInvDirection.mInvDirection;

	// Compute the max of min(t1,t2) and the min of max(t1,t2) ensuring we don't
	// use the results from any directions parallel to the slab.
	Vec3 t_min = Vec3::sSelect(Vec3::sMin(t1, t2), flt_min, inInvDirection.mIsParallel);
	Vec3 t_max = Vec3::sSelect(Vec3::sMax(t1, t2), flt_max, inInvDirection.mIsParallel);

	// t_min.xyz = maximum(t_min.x, t_min.y, t_min.z);
	t_min = Vec3::sMax(t_min, t_min.Swizzle<SWIZZLE_Y, SWIZZLE_Z, SWIZZLE_X>());
	t_min = Vec3::sMax(t_min, t_min.Swizzle<SWIZZLE_Z, SWIZZLE_X, SWIZZLE_Y>());

	// t_max.xyz = minimum(t_max.x, t_max.y, t_max.z);
	t_max = Vec3::sMin(t_max, t_max.Swizzle<SWIZZLE_Y, SWIZZLE_Z, SWIZZLE_X>());
	t_max = Vec3::sMin(t_max, t_max.Swizzle<SWIZZLE_Z, SWIZZLE_X, SWIZZLE_Y>());

	// if (t_min > t_max) return false;
	UVec4 no_intersection = Vec3::sGreater(t_min, t_max);

	// if (t_max < 0.0f) return false;
	no_intersection = UVec4::sOr(no_intersection, Vec3::sLess(t_max, Vec3::sZero()));

	// if (t_min > inClosest) return false;
	no_intersection = UVec4::sOr(no_intersection, Vec3::sGreater(t_min, Vec3::sReplicate(inClosest)));

	// if (inInvDirection.mIsParallel && !(Min <= inOrigin && inOrigin <= Max)) return false; else return true;
	UVec4 no_parallel_overlap = UVec4::sOr(Vec3::sLess(inOrigin, inBoundsMin), Vec3::sGreater(inOrigin, inBoundsMax));
	no_intersection = UVec4::sOr(no_intersection, UVec4::sAnd(inInvDirection.mIsParallel, no_parallel_overlap));

	return !no_intersection.TestAnyXYZTrue();
}

/// Intersect AABB with ray without hit fraction, based on separating axis test
/// @see http://www.codercorner.com/RayAABB.cpp
MOSS_INLINE bool RayAABoxHits(Vec3Arg inOrigin, Vec3Arg inDirection, Vec3Arg inBoundsMin, Vec3Arg inBoundsMax) {
	Vec3 extents = inBoundsMax - inBoundsMin;

	Vec3 diff = 2.0f * inOrigin - inBoundsMin - inBoundsMax;
	Vec3 abs_diff = diff.Abs();

	UVec4 no_intersection = UVec4::sAnd(Vec3::sGreater(abs_diff, extents), Vec3::sGreaterOrEqual(diff * inDirection, Vec3::sZero()));

	Vec3 abs_dir = inDirection.Abs();
	Vec3 abs_dir_yzz = abs_dir.Swizzle<SWIZZLE_Y, SWIZZLE_Z, SWIZZLE_Z>();
	Vec3 abs_dir_xyx = abs_dir.Swizzle<SWIZZLE_X, SWIZZLE_Y, SWIZZLE_X>();

	Vec3 extents_yzz = extents.Swizzle<SWIZZLE_Y, SWIZZLE_Z, SWIZZLE_Z>();
	Vec3 extents_xyx = extents.Swizzle<SWIZZLE_X, SWIZZLE_Y, SWIZZLE_X>();

	Vec3 diff_yzx = diff.Swizzle<SWIZZLE_Y, SWIZZLE_Z, SWIZZLE_X>();

	Vec3 dir_yzx = inDirection.Swizzle<SWIZZLE_Y, SWIZZLE_Z, SWIZZLE_X>();

	no_intersection = UVec4::sOr(no_intersection, Vec3::sGreater((inDirection * diff_yzx - dir_yzx * diff).Abs(), extents_xyx * abs_dir_yzz + extents_yzz * abs_dir_xyx));

	return !no_intersection.TestAnyXYZTrue();
}

MOSS_NAMESPACE_END

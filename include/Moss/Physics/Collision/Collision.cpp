// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT


#include <Moss/Physics/physics_intern.h>
#ifndef MOSS_DEBUG_RENDERER
	#include <Moss/Moss_Renderer.h>
#endif // MOSS_DEBUG_RENDERER

#include <Moss/Core/Profiler.h>

#ifdef MOSS_DUMP_BROADPHASE_TREE
MOSS_SUPPRESS_WARNINGS_STD_BEGIN
#include <fstream>
MOSS_SUPPRESS_WARNINGS_STD_END
#endif // MOSS_DUMP_BROADPHASE_TREE


MOSS_SUPRESS_WARNINGS_BEGIN

static constexpr uint8 sClosestFeatureToActiveEdgesMask[] = {
	0b000,		// 0b000: Invalid, guarded by an assert
	0b101,		// 0b001: Vertex 1 -> edge 1 or 3
	0b011,		// 0b010: Vertex 2 -> edge 1 or 2
	0b001,		// 0b011: Vertex 1 & 2 -> edge 1
	0b110,		// 0b100: Vertex 3 -> edge 2 or 3
	0b100,		// 0b101: Vertex 1 & 3 -> edge 3
	0b010,		// 0b110: Vertex 2 & 3 -> edge 2
	// 0b111: Vertex 1, 2 & 3 -> interior, guarded by an if
};

CollisionDispatch::CollideShape CollisionDispatch::sCollideShape[NumSubShapeTypes][NumSubShapeTypes];
CollisionDispatch::CastShape CollisionDispatch::sCastShape[NumSubShapeTypes][NumSubShapeTypes];


#ifdef MOSS_TRACK_NARROWPHASE_STATS
NarrowPhaseStat	NarrowPhaseStat::sCollideShape[NumSubShapeTypes][NumSubShapeTypes];
NarrowPhaseStat	NarrowPhaseStat::sCastShape[NumSubShapeTypes][NumSubShapeTypes];

thread_local TrackNarrowPhaseStat *TrackNarrowPhaseStat::sRoot = nullptr;
#endif

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(CollisionGroup)
{
	MOSS_ADD_ATTRIBUTE(CollisionGroup, mGroupFilter)
	MOSS_ADD_ATTRIBUTE(CollisionGroup, mGroupID)
	MOSS_ADD_ATTRIBUTE(CollisionGroup, mSubGroupID)
}

MOSS_IMPLEMENT_SERIALIZABLE_ABSTRACT_BASE(GroupFilter)
{
	MOSS_ADD_BASE_CLASS(GroupFilter, SerializableObject)
}

MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(GroupFilterTable)
{
	MOSS_ADD_BASE_CLASS(GroupFilterTable, GroupFilter)

	MOSS_ADD_ATTRIBUTE(GroupFilterTable, mNumSubGroups)
	MOSS_ADD_ATTRIBUTE(GroupFilterTable, mTable)
}

MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(PhysicsMaterialSimple)
{
	MOSS_ADD_BASE_CLASS(PhysicsMaterialSimple, PhysicsMaterial)

	MOSS_ADD_ATTRIBUTE(PhysicsMaterialSimple, mDebugName)
	MOSS_ADD_ATTRIBUTE(PhysicsMaterialSimple, mDebugColor)
}


/*													*/
bool TransformedShape::CastRay(const RRayCast &inRay, RayCastResult &ioHit) const
{
	if (mShape != nullptr)
	{
		// Transform the ray to local space, note that this drops precision which is possible because we're in local space now
		RayCast ray(inRay.Transformed(GetInverseCenterOfMassTransform()));

		// Scale the ray
		Vec3 inv_scale = GetShapeScale().Reciprocal();
		ray.mOrigin *= inv_scale;
		ray.mDirection *= inv_scale;

		// Cast the ray on the shape
		SubShapeIDCreator sub_shape_id(mSubShapeIDCreator);
		if (mShape->CastRay(ray, sub_shape_id, ioHit))
		{
			// Set body ID on the hit result
			ioHit.mBodyID = mBodyID;

			return true;
		}
	}

	return false;
}

void TransformedShape::CastRay(const RRayCast &inRay, const RayCastSettings &inRayCastSettings, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	if (mShape != nullptr)
	{
		// Set the context on the collector and filter
		ioCollector.SetContext(this);
		inShapeFilter.mBodyID2 = mBodyID;

		// Transform the ray to local space, note that this drops precision which is possible because we're in local space now
		RayCast ray(inRay.Transformed(GetInverseCenterOfMassTransform()));

		// Scale the ray
		Vec3 inv_scale = GetShapeScale().Reciprocal();
		ray.mOrigin *= inv_scale;
		ray.mDirection *= inv_scale;

		// Cast the ray on the shape
		SubShapeIDCreator sub_shape_id(mSubShapeIDCreator);
		mShape->CastRay(ray, inRayCastSettings, sub_shape_id, ioCollector, inShapeFilter);
	}
}

void TransformedShape::CollidePoint(RVec3Arg inPoint, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	if (mShape != nullptr)
	{
		// Set the context on the collector and filter
		ioCollector.SetContext(this);
		inShapeFilter.mBodyID2 = mBodyID;

		// Transform and scale the point to local space
		Vec3 point = Vec3(GetInverseCenterOfMassTransform() * inPoint) / GetShapeScale();

		// Do point collide on the shape
		SubShapeIDCreator sub_shape_id(mSubShapeIDCreator);
		mShape->CollidePoint(point, sub_shape_id, ioCollector, inShapeFilter);
	}
}

void TransformedShape::CollideShape(const Shape *inShape, Vec3Arg inShapeScale, RMat44Arg inCenterOfMassTransform, const CollideShapeSettings &inCollideShapeSettings, RVec3Arg inBaseOffset, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	if (mShape != nullptr)
	{
		// Set the context on the collector and filter
		ioCollector.SetContext(this);
		inShapeFilter.mBodyID2 = mBodyID;

		SubShapeIDCreator sub_shape_id1, sub_shape_id2(mSubShapeIDCreator);
		Mat44 transform1 = inCenterOfMassTransform.PostTranslated(-inBaseOffset).ToMat44();
		Mat44 transform2 = GetCenterOfMassTransform().PostTranslated(-inBaseOffset).ToMat44();
		CollisionDispatch::sCollideShapeVsShape(inShape, mShape, inShapeScale, GetShapeScale(), transform1, transform2, sub_shape_id1, sub_shape_id2, inCollideShapeSettings, ioCollector, inShapeFilter);
	}
}

void TransformedShape::CastShape(const RShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, RVec3Arg inBaseOffset, CastShapeCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	if (mShape != nullptr)
	{
		// Set the context on the collector and filter
		ioCollector.SetContext(this);
		inShapeFilter.mBodyID2 = mBodyID;

		// Get the shape cast relative to the base offset and convert it to floats
		ShapeCast shape_cast(inShapeCast.PostTranslated(-inBaseOffset));

		// Get center of mass of object we're casting against relative to the base offset and convert it to floats
		Mat44 center_of_mass_transform2 = GetCenterOfMassTransform().PostTranslated(-inBaseOffset).ToMat44();

		SubShapeIDCreator sub_shape_id1, sub_shape_id2(mSubShapeIDCreator);
		CollisionDispatch::sCastShapeVsShapeWorldSpace(shape_cast, inShapeCastSettings, mShape, GetShapeScale(), inShapeFilter, center_of_mass_transform2, sub_shape_id1, sub_shape_id2, ioCollector);
	}
}

void TransformedShape::CollectTransformedShapes(const AABox &inBox, TransformedShapeCollector &ioCollector, const ShapeFilter &inShapeFilter) const
{
	if (mShape != nullptr)
	{
		struct MyCollector : public TransformedShapeCollector
		{
										MyCollector(TransformedShapeCollector &ioCollector, RVec3 inShapePositionCOM) :
				TransformedShapeCollector(ioCollector),
				mCollector(ioCollector),
				mShapePositionCOM(inShapePositionCOM)
			{
			}

			virtual void				AddHit(const TransformedShape &inResult) override
			{
				// Apply the center of mass offset
				TransformedShape ts = inResult;
				ts.mShapePositionCOM += mShapePositionCOM;

				// Pass hit on to child collector
				mCollector.AddHit(ts);

				// Update early out fraction based on child collector
				UpdateEarlyOutFraction(mCollector.GetEarlyOutFraction());
			}

			TransformedShapeCollector &	mCollector;
			RVec3						mShapePositionCOM;
		};

		// Set the context on the collector
		ioCollector.SetContext(this);

		// Wrap the collector so we can add the center of mass precision, we do this to avoid losing precision because CollectTransformedShapes uses single precision floats
		MyCollector collector(ioCollector, mShapePositionCOM);

		// Take box to local space for the shape
		AABox box = inBox;
		box.Translate(-mShapePositionCOM);

		mShape->CollectTransformedShapes(box, Vec3::sZero(), mShapeRotation, GetShapeScale(), mSubShapeIDCreator, collector, inShapeFilter);
	}
}

void TransformedShape::GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, RVec3Arg inBaseOffset) const
{
	if (mShape != nullptr)
	{
		// Take box to local space for the shape
		AABox box = inBox;
		box.Translate(-inBaseOffset);

		mShape->GetTrianglesStart(ioContext, box, Vec3(mShapePositionCOM - inBaseOffset), mShapeRotation, GetShapeScale());
	}
}

int TransformedShape::GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials) const
{
	if (mShape != nullptr)
		return mShape->GetTrianglesNext(ioContext, inMaxTrianglesRequested, outTriangleVertices, outMaterials);
	else
		return 0;
}


/*													*/

CastConvexVsTriangles::CastConvexVsTriangles(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, CastShapeCollector &ioCollector) :
	mShapeCast(inShapeCast),
	mShapeCastSettings(inShapeCastSettings),
	mCenterOfMassTransform2(inCenterOfMassTransform2),
	mScale(inScale),
	mSubShapeIDCreator1(inSubShapeIDCreator1),
	mCollector(ioCollector)
{
	MOSS_ASSERT(inShapeCast.mShape->GetType() == EShapeType::Convex);

	// Determine if shape is inside out or not
	mScaleSign = ScaleHelpers::IsInsideOut(inScale)? -1.0f : 1.0f;
}

void CastConvexVsTriangles::Cast(Vec3Arg inV0, Vec3Arg inV1, Vec3Arg inV2, uint8 inActiveEdges, const SubShapeID &inSubShapeID2)
{
	MOSS_PROFILE_FUNCTION();

	// Scale triangle
	Vec3 v0 = mScale * inV0;
	Vec3 v1 = mScale * inV1;
	Vec3 v2 = mScale * inV2;

	// Calculate triangle normal
	Vec3 triangle_normal = mScaleSign * (v1 - v0).Cross(v2 - v0);

	// Backface check
	bool back_facing = triangle_normal.Dot(mShapeCast.mDirection) > 0.0f;
	if (mShapeCastSettings.mBackFaceModeTriangles == EBackFaceMode::IgnoreBackFaces && back_facing)
		return;

	// Create triangle support function
	TriangleConvexSupport triangle { v0, v1, v2 };

	// Check if we already created the cast shape support function
	if (mSupport == nullptr)
	{
		// Determine if we want to use the actual shape or a shrunken shape with convex radius
		ConvexShape::ESupportMode support_mode = mShapeCastSettings.mUseShrunkenShapeAndConvexRadius? ConvexShape::ESupportMode::ExcludeConvexRadius : ConvexShape::ESupportMode::Default;

		// Create support function
		mSupport = static_cast<const ConvexShape *>(mShapeCast.mShape)->GetSupportFunction(support_mode, mSupportBuffer, mShapeCast.mScale);
	}

	EPAPenetrationDepth epa;
	float fraction = mCollector.GetEarlyOutFraction();
	Vec3 contact_point_a, contact_point_b, contact_normal;
	if (epa.CastShape(mShapeCast.mCenterOfMassStart, mShapeCast.mDirection, mShapeCastSettings.mCollisionTolerance, mShapeCastSettings.mPenetrationTolerance, *mSupport, triangle, mSupport->GetConvexRadius(), 0.0f, mShapeCastSettings.mReturnDeepestPoint, fraction, contact_point_a, contact_point_b, contact_normal))
	{
		// Check if we have enabled active edge detection
		if (mShapeCastSettings.mActiveEdgeMode == EActiveEdgeMode::CollideOnlyWithActive && inActiveEdges != 0b111)
		{
			// Convert the active edge velocity hint to local space
			Vec3 active_edge_movement_direction = mCenterOfMassTransform2.Multiply3x3Transposed(mShapeCastSettings.mActiveEdgeMovementDirection);

			// Update the contact normal to account for active edges
			// Note that we flip the triangle normal as the penetration axis is pointing towards the triangle instead of away
			contact_normal = ActiveEdges::FixNormal(v0, v1, v2, back_facing? triangle_normal : -triangle_normal, inActiveEdges, contact_point_b, contact_normal, active_edge_movement_direction);
		}

		// Convert to world space
		contact_point_a = mCenterOfMassTransform2 * contact_point_a;
		contact_point_b = mCenterOfMassTransform2 * contact_point_b;
		Vec3 contact_normal_world = mCenterOfMassTransform2.Multiply3x3(contact_normal);

		// Its a hit, store the sub shape id's
		ShapeCastResult result(fraction, contact_point_a, contact_point_b, contact_normal_world, back_facing, mSubShapeIDCreator1.GetID(), inSubShapeID2, TransformedShape::sGetBodyID(mCollector.GetContext()));

		// Early out if this hit is deeper than the collector's early out value
		if (fraction == 0.0f && -result.mPenetrationDepth >= mCollector.GetEarlyOutFraction())
			return;

		// Gather faces
		if (mShapeCastSettings.mCollectFacesMode == ECollectFacesMode::CollectFaces)
		{
			// Get supporting face of shape 1
			Mat44 transform_1_to_2 = mShapeCast.mCenterOfMassStart;
			transform_1_to_2.SetTranslation(transform_1_to_2.GetTranslation() + fraction * mShapeCast.mDirection);
			static_cast<const ConvexShape *>(mShapeCast.mShape)->GetSupportingFace(SubShapeID(), transform_1_to_2.Multiply3x3Transposed(-contact_normal), mShapeCast.mScale, mCenterOfMassTransform2 * transform_1_to_2, result.mShape1Face);

			// Get face of the triangle
			triangle.GetSupportingFace(contact_normal, result.mShape2Face);

			// Convert to world space
			for (Vec3 &p : result.mShape2Face)
				p = mCenterOfMassTransform2 * p;
		}

		MOSS_IF_TRACK_NARROWPHASE_STATS(TrackNarrowPhaseCollector track;)
		mCollector.AddHit(result);
	}
}

/*													*/


CastSphereVsTriangles::CastSphereVsTriangles(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, CastShapeCollector &ioCollector) :
	mStart(inShapeCast.mCenterOfMassStart.GetTranslation()),
	mDirection(inShapeCast.mDirection),
	mShapeCastSettings(inShapeCastSettings),
	mCenterOfMassTransform2(inCenterOfMassTransform2),
	mScale(inScale),
	mSubShapeIDCreator1(inSubShapeIDCreator1),
	mCollector(ioCollector)
{
	// Cast to sphere shape
	MOSS_ASSERT(inShapeCast.mShape->GetSubType() == EShapeSubType::Sphere);
	const SphereShape *sphere = static_cast<const SphereShape *>(inShapeCast.mShape);

	// Scale the radius
	mRadius = sphere->GetRadius() * abs(inShapeCast.mScale.GetX());

	// Determine if shape is inside out or not
	mScaleSign = ScaleHelpers::IsInsideOut(inScale)? -1.0f : 1.0f;
}

void CastSphereVsTriangles::AddHit(bool inBackFacing, const SubShapeID &inSubShapeID2, float inFraction, Vec3Arg inContactPointA, Vec3Arg inContactPointB, Vec3Arg inContactNormal)
{
	// Convert to world space
	Vec3 contact_point_a = mCenterOfMassTransform2 * (mStart + inContactPointA);
	Vec3 contact_point_b = mCenterOfMassTransform2 * (mStart + inContactPointB);
	Vec3 contact_normal_world = mCenterOfMassTransform2.Multiply3x3(inContactNormal);

	// Its a hit, store the sub shape id's
	ShapeCastResult result(inFraction, contact_point_a, contact_point_b, contact_normal_world, inBackFacing, mSubShapeIDCreator1.GetID(), inSubShapeID2, TransformedShape::sGetBodyID(mCollector.GetContext()));

	// Note: We don't gather faces here because that's only useful if both shapes have a face. Since the sphere always has only 1 contact point, the manifold is always a point.

	MOSS_IF_TRACK_NARROWPHASE_STATS(TrackNarrowPhaseCollector track;)
	mCollector.AddHit(result);
}

void CastSphereVsTriangles::AddHitWithActiveEdgeDetection(Vec3Arg inV0, Vec3Arg inV1, Vec3Arg inV2, bool inBackFacing, Vec3Arg inTriangleNormal, uint8 inActiveEdges, const SubShapeID &inSubShapeID2, float inFraction, Vec3Arg inContactPointA, Vec3Arg inContactPointB, Vec3Arg inContactNormal)
{
	// Check if we have enabled active edge detection
	Vec3 contact_normal = inContactNormal;
	if (mShapeCastSettings.mActiveEdgeMode == EActiveEdgeMode::CollideOnlyWithActive && inActiveEdges != 0b111)
	{
		// Convert the active edge velocity hint to local space
		Vec3 active_edge_movement_direction = mCenterOfMassTransform2.Multiply3x3Transposed(mShapeCastSettings.mActiveEdgeMovementDirection);

		// Update the contact normal to account for active edges
		// Note that we flip the triangle normal as the penetration axis is pointing towards the triangle instead of away
		contact_normal = ActiveEdges::FixNormal(inV0, inV1, inV2, inBackFacing? inTriangleNormal : -inTriangleNormal, inActiveEdges, inContactPointB, inContactNormal, active_edge_movement_direction);
	}

	AddHit(inBackFacing, inSubShapeID2, inFraction, inContactPointA, inContactPointB, contact_normal);
}

// This is a simplified version of the ray cylinder test from: Real Time Collision Detection - Christer Ericson
// Chapter 5.3.7, page 194-197. Some conditions have been removed as we're not interested in hitting the caps of the cylinder.
// Note that the ray origin is assumed to be the origin here.
float CastSphereVsTriangles::RayCylinder(Vec3Arg inRayDirection, Vec3Arg inCylinderA, Vec3Arg inCylinderB, float inRadius) const
{
	// Calculate cylinder axis
	Vec3 axis = inCylinderB - inCylinderA;

	// Make ray start relative to cylinder side A (moving cylinder A to the origin)
	Vec3 start = -inCylinderA;

	// Test if segment is fully on the A side of the cylinder
	float start_dot_axis = start.Dot(axis);
	float direction_dot_axis = inRayDirection.Dot(axis);
	float end_dot_axis = start_dot_axis + direction_dot_axis;
	if (start_dot_axis < 0.0f && end_dot_axis < 0.0f)
		return FLT_MAX;

	// Test if segment is fully on the B side of the cylinder
	float axis_len_sq = axis.LengthSq();
	if (start_dot_axis > axis_len_sq && end_dot_axis > axis_len_sq)
		return FLT_MAX;

	// Calculate a, b and c, the factors for quadratic equation
	// We're basically solving the ray: x = start + direction * t
	// The closest point to x on the segment A B is: w = (x . axis) * axis / (axis . axis)
	// The distance between x and w should be radius: (x - w) . (x - w) = radius^2
	// Solving this gives the following:
	float a = axis_len_sq * inRayDirection.LengthSq() - Square(direction_dot_axis);
	if (abs(a) < 1.0e-6f)
		return FLT_MAX; // Segment runs parallel to cylinder axis, stop processing, we will either hit at fraction = 0 or we'll hit a vertex
	float b = axis_len_sq * start.Dot(inRayDirection) - direction_dot_axis * start_dot_axis; // should be multiplied by 2, instead we'll divide a and c by 2 when we solve the quadratic equation
	float c = axis_len_sq * (start.LengthSq() - Square(inRadius)) - Square(start_dot_axis);
	float det = Square(b) - a * c; // normally 4 * a * c but since both a and c need to be divided by 2 we lose the 4
	if (det < 0.0f)
		return FLT_MAX; // No solution to quadratic equation

	// Solve fraction t where the ray hits the cylinder
	float t = -(b + sqrt(det)) / a; // normally divided by 2 * a but since a should be divided by 2 we lose the 2
	if (t < 0.0f || t > 1.0f)
		return FLT_MAX; // Intersection lies outside segment
	if (start_dot_axis + t * direction_dot_axis < 0.0f || start_dot_axis + t * direction_dot_axis > axis_len_sq)
		return FLT_MAX; // Intersection outside the end point of the cylinder, stop processing, we will possibly hit a vertex
	return t;
}

void CastSphereVsTriangles::Cast(Vec3Arg inV0, Vec3Arg inV1, Vec3Arg inV2, uint8 inActiveEdges, const SubShapeID &inSubShapeID2)
{
	MOSS_PROFILE_FUNCTION();

	// Scale triangle and make it relative to the start of the cast
	Vec3 v0 = mScale * inV0 - mStart;
	Vec3 v1 = mScale * inV1 - mStart;
	Vec3 v2 = mScale * inV2 - mStart;

	// Calculate triangle normal
	Vec3 triangle_normal = mScaleSign * (v1 - v0).Cross(v2 - v0);
	float triangle_normal_len = triangle_normal.Length();
	if (triangle_normal_len == 0.0f)
		return; // Degenerate triangle
	triangle_normal /= triangle_normal_len;

	// Backface check
	float normal_dot_direction = triangle_normal.Dot(mDirection);
	bool back_facing = normal_dot_direction > 0.0f;
	if (mShapeCastSettings.mBackFaceModeTriangles == EBackFaceMode::IgnoreBackFaces && back_facing)
		return;

	// Test if distance between the sphere and plane of triangle is smaller or equal than the radius
	if (abs(v0.Dot(triangle_normal)) <= mRadius)
	{
		// Check if the sphere intersects at the start of the cast
		uint32 closest_feature;
		Vec3 q = ClosestPoint::GetClosestPointOnTriangle(v0, v1, v2, closest_feature);
		float q_len_sq = q.LengthSq();
		if (q_len_sq <= Square(mRadius))
		{
			// Early out if this hit is deeper than the collector's early out value
			float q_len = sqrt(q_len_sq);
			float penetration_depth = mRadius - q_len;
			if (-penetration_depth >= mCollector.GetEarlyOutFraction())
				return;

			// Generate contact point
			Vec3 contact_normal = q_len > 0.0f? q / q_len : Vec3::sAxisY();
			Vec3 contact_point_a = q + contact_normal * penetration_depth;
			Vec3 contact_point_b = q;
			AddHitWithActiveEdgeDetection(v0, v1, v2, back_facing, triangle_normal, inActiveEdges, inSubShapeID2, 0.0f, contact_point_a, contact_point_b, contact_normal);
			return;
		}
	}
	else
	{
		// Check if cast is not parallel to the plane of the triangle
		float abs_normal_dot_direction = abs(normal_dot_direction);
		if (abs_normal_dot_direction > 1.0e-6f)
		{
			// Calculate the point on the sphere that will hit the triangle's plane first and calculate a fraction where it will do so
			Vec3 d = Sign(normal_dot_direction) * mRadius * triangle_normal;
			float plane_intersection = (v0 - d).Dot(triangle_normal) / normal_dot_direction;

			// Check if sphere will hit in the interval that we're interested in
			if (plane_intersection * abs_normal_dot_direction < -mRadius	// Sphere hits the plane before the sweep, cannot intersect
				|| plane_intersection >= mCollector.GetEarlyOutFraction())	// Sphere hits the plane after the sweep / early out fraction, cannot intersect
				return;

			// We can only report an interior hit if we're hitting the plane during our sweep and not before
			if (plane_intersection >= 0.0f)
			{
				// Calculate the point of contact on the plane
				Vec3 p = d + plane_intersection * mDirection;

				// Check if this is an interior point
				float u, v, w;
				if (ClosestPoint::GetBaryCentricCoordinates(v0 - p, v1 - p, v2 - p, u, v, w)
					&& u >= 0.0f && v >= 0.0f && w >= 0.0f)
				{
					// Interior point, we found the collision point. We don't need to check active edges.
					AddHit(back_facing, inSubShapeID2, plane_intersection, p, p, back_facing? triangle_normal : -triangle_normal);
					return;
				}
			}
		}
	}

	// Test 3 edges
	float fraction = RayCylinder(mDirection, v0, v1, mRadius);
	fraction = min(fraction, RayCylinder(mDirection, v1, v2, mRadius));
	fraction = min(fraction, RayCylinder(mDirection, v2, v0, mRadius));

	// Test 3 vertices
	fraction = min(fraction, RaySphere(Vec3::sZero(), mDirection, v0, mRadius));
	fraction = min(fraction, RaySphere(Vec3::sZero(), mDirection, v1, mRadius));
	fraction = min(fraction, RaySphere(Vec3::sZero(), mDirection, v2, mRadius));

	// Check if we have a collision
	MOSS_ASSERT(fraction >= 0.0f);
	if (fraction < mCollector.GetEarlyOutFraction())
	{
		// Calculate the center of the sphere at the point of contact
		Vec3 p = fraction * mDirection;

		// Get contact point and normal
		uint32 closest_feature;
		Vec3 q = ClosestPoint::GetClosestPointOnTriangle(v0 - p, v1 - p, v2 - p, closest_feature);
		Vec3 contact_normal = q.Normalized();
		Vec3 contact_point_ab = p + q;
		AddHitWithActiveEdgeDetection(v0, v1, v2, back_facing, triangle_normal, inActiveEdges, inSubShapeID2, fraction, contact_point_ab, contact_point_ab, contact_normal);
	}
}

/*													*/

CollideConvexVsTriangles::CollideConvexVsTriangles(const ConvexShape *inShape1, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeID &inSubShapeID1, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector) :
	mCollideShapeSettings(inCollideShapeSettings),
	mCollector(ioCollector),
	mShape1(inShape1),
	mScale1(inScale1),
	mScale2(inScale2),
	mTransform1(inCenterOfMassTransform1),
	mSubShapeID1(inSubShapeID1)
{
	// Get transforms
	Mat44 inverse_transform2 = inCenterOfMassTransform2.InversedRotationTranslation();
	Mat44 transform1_to_2 = inverse_transform2 * inCenterOfMassTransform1;
	mTransform2To1 = transform1_to_2.InversedRotationTranslation();

	// Calculate bounds
	mBoundsOf1 = inShape1->GetLocalBounds().Scaled(inScale1);
	mBoundsOf1.ExpandBy(Vec3::sReplicate(inCollideShapeSettings.mMaxSeparationDistance));
	mBoundsOf1InSpaceOf2 = mBoundsOf1.Transformed(transform1_to_2);	// Convert bounding box of 1 into space of 2

	// Determine if shape 2 is inside out or not
	mScaleSign2 = ScaleHelpers::IsInsideOut(inScale2)? -1.0f : 1.0f;
}

void CollideConvexVsTriangles::Collide(Vec3Arg inV0, Vec3Arg inV1, Vec3Arg inV2, uint8 inActiveEdges, const SubShapeID &inSubShapeID2)
{
	MOSS_PROFILE_FUNCTION();

	// Scale triangle and transform it to the space of 1
	Vec3 v0 = mTransform2To1 * (mScale2 * inV0);
	Vec3 v1 = mTransform2To1 * (mScale2 * inV1);
	Vec3 v2 = mTransform2To1 * (mScale2 * inV2);

	// Calculate triangle normal
	Vec3 triangle_normal = mScaleSign2 * (v1 - v0).Cross(v2 - v0);

	// Backface check
	bool back_facing = triangle_normal.Dot(v0) > 0.0f;
	if (mCollideShapeSettings.mBackFaceMode == EBackFaceMode::IgnoreBackFaces && back_facing)
		return;

	// Get bounding box for triangle
	AABox triangle_bbox = AABox::sFromTwoPoints(v0, v1);
	triangle_bbox.Encapsulate(v2);

	// Get intersection between triangle and shape box, if there is none, we're done
	if (!triangle_bbox.Overlaps(mBoundsOf1))
		return;

	// Create triangle support function
	TriangleConvexSupport triangle(v0, v1, v2);

	// Perform collision detection
	// Note: As we don't remember the penetration axis from the last iteration, and it is likely that the shape (A) we're colliding the triangle (B) against is in front of the triangle,
	// and the penetration axis is the shortest distance along to push B out of collision, we use the inverse of the triangle normal as an initial penetration axis. This has been seen
	// to improve performance by approx. 5% over using a fixed axis like (1, 0, 0).
	Vec3 penetration_axis = -triangle_normal, point1, point2;
	EPAPenetrationDepth pen_depth;
	EPAPenetrationDepth::EStatus status;

	// Get the support function
	if (mShape1ExCvxRadius == nullptr)
		mShape1ExCvxRadius = mShape1->GetSupportFunction(ConvexShape::ESupportMode::ExcludeConvexRadius, mBufferExCvxRadius, mScale1);

	// Perform GJK step
	float max_separation_distance = mCollideShapeSettings.mMaxSeparationDistance;
	status = pen_depth.GetPenetrationDepthStepGJK(*mShape1ExCvxRadius, mShape1ExCvxRadius->GetConvexRadius() + max_separation_distance, triangle, 0.0f, mCollideShapeSettings.mCollisionTolerance, penetration_axis, point1, point2);

	// Check result of collision detection
	if (status == EPAPenetrationDepth::EStatus::NotColliding)
		return;
	else if (status == EPAPenetrationDepth::EStatus::Indeterminate)
	{
		// Need to run expensive EPA algorithm

		// We know we're overlapping at this point, so we can set the max separation distance to 0.
		// Numerically it is possible that GJK finds that the shapes are overlapping but EPA finds that they're separated.
		// In order to avoid this, we clamp the max separation distance to 1 so that we don't excessively inflate the shape,
		// but we still inflate it enough to avoid the case where EPA misses the collision.
		max_separation_distance = min(max_separation_distance, 1.0f);

		// Get the support function
		if (mShape1IncCvxRadius == nullptr)
			mShape1IncCvxRadius = mShape1->GetSupportFunction(ConvexShape::ESupportMode::IncludeConvexRadius, mBufferIncCvxRadius, mScale1);

		// Add convex radius
		AddConvexRadius shape1_add_max_separation_distance(*mShape1IncCvxRadius, max_separation_distance);

		// Perform EPA step
		if (!pen_depth.GetPenetrationDepthStepEPA(shape1_add_max_separation_distance, triangle, mCollideShapeSettings.mPenetrationTolerance, penetration_axis, point1, point2))
			return;
	}

	// Check if the penetration is bigger than the early out fraction
	float penetration_depth = (point2 - point1).Length() - max_separation_distance;
	if (-penetration_depth >= mCollector.GetEarlyOutFraction())
		return;

	// Correct point1 for the added separation distance
	float penetration_axis_len = penetration_axis.Length();
	if (penetration_axis_len > 0.0f)
		point1 -= penetration_axis * (max_separation_distance / penetration_axis_len);

	// Check if we have enabled active edge detection
	if (mCollideShapeSettings.mActiveEdgeMode == EActiveEdgeMode::CollideOnlyWithActive && inActiveEdges != 0b111)
	{
		// Convert the active edge velocity hint to local space
		Vec3 active_edge_movement_direction = mTransform1.Multiply3x3Transposed(mCollideShapeSettings.mActiveEdgeMovementDirection);

		// Update the penetration axis to account for active edges
		// Note that we flip the triangle normal as the penetration axis is pointing towards the triangle instead of away
		penetration_axis = ActiveEdges::FixNormal(v0, v1, v2, back_facing? triangle_normal : -triangle_normal, inActiveEdges, point2, penetration_axis, active_edge_movement_direction);
	}

	// Convert to world space
	point1 = mTransform1 * point1;
	point2 = mTransform1 * point2;
	Vec3 penetration_axis_world = mTransform1.Multiply3x3(penetration_axis);

	// Create collision result
	CollideShapeResult result(point1, point2, penetration_axis_world, penetration_depth, mSubShapeID1, inSubShapeID2, TransformedShape::sGetBodyID(mCollector.GetContext()));

	// Gather faces
	if (mCollideShapeSettings.mCollectFacesMode == ECollectFacesMode::CollectFaces)
	{
		// Get supporting face of shape 1
		mShape1->GetSupportingFace(SubShapeID(), -penetration_axis, mScale1, mTransform1, result.mShape1Face);

		// Get face of the triangle
		result.mShape2Face.resize(3);
		result.mShape2Face[0] = mTransform1 * v0;
		result.mShape2Face[1] = mTransform1 * v1;
		result.mShape2Face[2] = mTransform1 * v2;
	}

	// Notify the collector
	MOSS_IF_TRACK_NARROWPHASE_STATS(TrackNarrowPhaseCollector track;)
	mCollector.AddHit(result);
}


/*													*/

CollideSphereVsTriangles::CollideSphereVsTriangles(const SphereShape *inShape1, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeID &inSubShapeID1, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector) :
	mCollideShapeSettings(inCollideShapeSettings),
	mCollector(ioCollector),
	mShape1(inShape1),
	mScale2(inScale2),
	mTransform2(inCenterOfMassTransform2),
	mSubShapeID1(inSubShapeID1)
{
	// Calculate the center of the sphere in the space of 2
	mSphereCenterIn2 = inCenterOfMassTransform2.Multiply3x3Transposed(inCenterOfMassTransform1.GetTranslation() - inCenterOfMassTransform2.GetTranslation());

	// Determine if shape 2 is inside out or not
	mScaleSign2 = ScaleHelpers::IsInsideOut(inScale2)? -1.0f : 1.0f;

	// Check that the sphere is uniformly scaled
	MOSS_ASSERT(ScaleHelpers::IsUniformScale(inScale1.Abs()));
	mRadius = abs(inScale1.GetX()) * inShape1->GetRadius();
	mRadiusPlusMaxSeparationSq = Square(mRadius + inCollideShapeSettings.mMaxSeparationDistance);
}

void CollideSphereVsTriangles::Collide(Vec3Arg inV0, Vec3Arg inV1, Vec3Arg inV2, uint8 inActiveEdges, const SubShapeID &inSubShapeID2)
{
	MOSS_PROFILE_FUNCTION();

	// Scale triangle and make it relative to the center of the sphere
	Vec3 v0 = mScale2 * inV0 - mSphereCenterIn2;
	Vec3 v1 = mScale2 * inV1 - mSphereCenterIn2;
	Vec3 v2 = mScale2 * inV2 - mSphereCenterIn2;

	// Calculate triangle normal
	Vec3 triangle_normal = mScaleSign2 * (v1 - v0).Cross(v2 - v0);

	// Backface check
	bool back_facing = triangle_normal.Dot(v0) > 0.0f;
	if (mCollideShapeSettings.mBackFaceMode == EBackFaceMode::IgnoreBackFaces && back_facing)
		return;

	// Check if we collide with the sphere
	uint32 closest_feature;
	Vec3 point2 = ClosestPoint::GetClosestPointOnTriangle(v0, v1, v2, closest_feature);
	float point2_len_sq = point2.LengthSq();
	if (point2_len_sq > mRadiusPlusMaxSeparationSq)
		return;

	// Calculate penetration depth
	float penetration_depth = mRadius - sqrt(point2_len_sq);
	if (-penetration_depth >= mCollector.GetEarlyOutFraction())
		return;

	// Calculate penetration axis, direction along which to push 2 to move it out of collision (this is always away from the sphere center)
	Vec3 penetration_axis = point2.NormalizedOr(Vec3::sAxisY());

	// Calculate the point on the sphere
	Vec3 point1 = mRadius * penetration_axis;

	// Check if we have enabled active edge detection
	MOSS_ASSERT(closest_feature != 0);
	if (mCollideShapeSettings.mActiveEdgeMode == EActiveEdgeMode::CollideOnlyWithActive
		&& closest_feature != 0b111 // For an interior hit we should already have the right normal
		&& (inActiveEdges & sClosestFeatureToActiveEdgesMask[closest_feature]) == 0) // If we didn't hit an active edge we should take the triangle normal
	{
		// Convert the active edge velocity hint to local space
		Vec3 active_edge_movement_direction = mTransform2.Multiply3x3Transposed(mCollideShapeSettings.mActiveEdgeMovementDirection);

		// See ActiveEdges::FixNormal. If penetration_axis affects the movement less than the triangle normal we keep penetration_axis.
		Vec3 new_penetration_axis = back_facing? triangle_normal : -triangle_normal;
		if (active_edge_movement_direction.Dot(penetration_axis) * new_penetration_axis.Length() >= active_edge_movement_direction.Dot(new_penetration_axis))
			penetration_axis = new_penetration_axis;
	}

	// Convert to world space
	point1 = mTransform2 * (mSphereCenterIn2 + point1);
	point2 = mTransform2 * (mSphereCenterIn2 + point2);
	Vec3 penetration_axis_world = mTransform2.Multiply3x3(penetration_axis);

	// Create collision result
	CollideShapeResult result(point1, point2, penetration_axis_world, penetration_depth, mSubShapeID1, inSubShapeID2, TransformedShape::sGetBodyID(mCollector.GetContext()));

	// Gather faces
	if (mCollideShapeSettings.mCollectFacesMode == ECollectFacesMode::CollectFaces)
	{
		// The sphere doesn't have a supporting face

		// Get face of triangle 2
		result.mShape2Face.resize(3);
		result.mShape2Face[0] = mTransform2 * (mSphereCenterIn2 + v0);
		result.mShape2Face[1] = mTransform2 * (mSphereCenterIn2 + v1);
		result.mShape2Face[2] = mTransform2 * (mSphereCenterIn2 + v2);
	}

	// Notify the collector
	MOSS_IF_TRACK_NARROWPHASE_STATS(TrackNarrowPhaseCollector track;)
	mCollector.AddHit(result);
}

/*													*/

void CollisionDispatch::sInit()
{
	for (uint i = 0; i < NumSubShapeTypes; ++i)
		for (uint j = 0; j < NumSubShapeTypes; ++j)
		{
			if (sCollideShape[i][j] == nullptr)
				sCollideShape[i][j] = [](const Shape *, const Shape *, Vec3Arg, Vec3Arg, Mat44Arg, Mat44Arg, const SubShapeIDCreator &, const SubShapeIDCreator &, const CollideShapeSettings &, CollideShapeCollector &, const ShapeFilter &)
				{
					MOSS_ASSERT(false, "Unsupported shape pair");
				};

			if (sCastShape[i][j] == nullptr)
				sCastShape[i][j] = [](const ShapeCast &, const ShapeCastSettings &, const Shape *, Vec3Arg, const ShapeFilter &, Mat44Arg, const SubShapeIDCreator &, const SubShapeIDCreator &, CastShapeCollector &)
				{
					MOSS_ASSERT(false, "Unsupported shape pair");
				};
		}
}

void CollisionDispatch::sReversedCollideShape(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter)
{
	// A collision collector that flips the collision results
	class ReversedCollector : public CollideShapeCollector
	{
	public:
		explicit				ReversedCollector(CollideShapeCollector &ioCollector) :
			CollideShapeCollector(ioCollector),
			mCollector(ioCollector)
		{
		}

		virtual void			AddHit(const CollideShapeResult &inResult) override
		{
			// Add the reversed hit
			mCollector.AddHit(inResult.Reversed());

			// If our chained collector updated its early out fraction, we need to follow
			UpdateEarlyOutFraction(mCollector.GetEarlyOutFraction());
		}

	private:
		CollideShapeCollector &	mCollector;
	};

	ReversedShapeFilter shape_filter(inShapeFilter);
	ReversedCollector collector(ioCollector);
	sCollideShapeVsShape(inShape2, inShape1, inScale2, inScale1, inCenterOfMassTransform2, inCenterOfMassTransform1, inSubShapeIDCreator2, inSubShapeIDCreator1, inCollideShapeSettings, collector, shape_filter);
}

void CollisionDispatch::sReversedCastShape(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector)
{
	// A collision collector that flips the collision results
	class ReversedCollector : public CastShapeCollector
	{
	public:
		explicit				ReversedCollector(CastShapeCollector &ioCollector, Vec3Arg inWorldDirection) :
			CastShapeCollector(ioCollector),
			mCollector(ioCollector),
			mWorldDirection(inWorldDirection)
		{
		}

		virtual void			AddHit(const ShapeCastResult &inResult) override
		{
			// Add the reversed hit
			mCollector.AddHit(inResult.Reversed(mWorldDirection));

			// If our chained collector updated its early out fraction, we need to follow
			UpdateEarlyOutFraction(mCollector.GetEarlyOutFraction());
		}

	private:
		CastShapeCollector &	mCollector;
		Vec3					mWorldDirection;
	};

	// Reverse the shape cast (shape cast is in local space to shape 2)
	Mat44 com_start_inv = inShapeCast.mCenterOfMassStart.InversedRotationTranslation();
	ShapeCast local_shape_cast(inShape, inScale, com_start_inv, -com_start_inv.Multiply3x3(inShapeCast.mDirection));

	// Calculate the center of mass of shape 1 at start of sweep
	Mat44 shape1_com = inCenterOfMassTransform2 * inShapeCast.mCenterOfMassStart;

	// Calculate the world space direction vector of the shape cast
	Vec3 world_direction = -inCenterOfMassTransform2.Multiply3x3(inShapeCast.mDirection);

	// Forward the cast
	ReversedShapeFilter shape_filter(inShapeFilter);
	ReversedCollector collector(ioCollector, world_direction);
	sCastShapeVsShapeLocalSpace(local_shape_cast, inShapeCastSettings, inShapeCast.mShape, inShapeCast.mScale, shape_filter, shape1_com, inSubShapeIDCreator2, inSubShapeIDCreator1, collector);
}

/*													*/

void EstimateCollisionResponse(const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, CollisionEstimationResult &outResult, float inCombinedFriction, float inCombinedRestitution, float inMinVelocityForRestitution, uint inNumIterations)
{
	// Note this code is based on AxisConstraintPart, see that class for more comments on the math

	ContactPoints::size_type num_points = inManifold.mRelativeContactPointsOn1.size();
	MOSS_ASSERT(num_points == inManifold.mRelativeContactPointsOn2.size());

	// Start with zero impulses
	outResult.mImpulses.resize(num_points);
	memset(outResult.mImpulses.data(), 0, num_points * sizeof(CollisionEstimationResult::Impulse));

	// Calculate friction directions
	outResult.mTangent1 = inManifold.mWorldSpaceNormal.GetNormalizedPerpendicular();
	outResult.mTangent2 = inManifold.mWorldSpaceNormal.Cross(outResult.mTangent1);

	// Get body velocities
	EMotionType motion_type1 = inBody1.GetMotionType();
	const MotionProperties *motion_properties1 = inBody1.GetMotionPropertiesUnchecked();
	if (motion_type1 != EMotionType::Static)
	{
		outResult.mLinearVelocity1 = motion_properties1->GetLinearVelocity();
		outResult.mAngularVelocity1 = motion_properties1->GetAngularVelocity();
	}
	else
		outResult.mLinearVelocity1 = outResult.mAngularVelocity1 = Vec3::sZero();

	EMotionType motion_type2 = inBody2.GetMotionType();
	const MotionProperties *motion_properties2 = inBody2.GetMotionPropertiesUnchecked();
	if (motion_type2 != EMotionType::Static)
	{
		outResult.mLinearVelocity2 = motion_properties2->GetLinearVelocity();
		outResult.mAngularVelocity2 = motion_properties2->GetAngularVelocity();
	}
	else
		outResult.mLinearVelocity2 = outResult.mAngularVelocity2 = Vec3::sZero();

	// Get inverse mass and inertia
	float inv_m1, inv_m2;
	Mat44 inv_i1, inv_i2;
	if (motion_type1 == EMotionType::Dynamic)
	{
		inv_m1 = motion_properties1->GetInverseMass();
		inv_i1 = inBody1.GetInverseInertia();
	}
	else
	{
		inv_m1 = 0.0f;
		inv_i1 = Mat44::sZero();
	}

	if (motion_type2 == EMotionType::Dynamic)
	{
		inv_m2 = motion_properties2->GetInverseMass();
		inv_i2 = inBody2.GetInverseInertia();
	}
	else
	{
		inv_m2 = 0.0f;
		inv_i2 = Mat44::sZero();
	}

	// Get center of masses relative to the base offset
	Vec3 com1 = Vec3(inBody1.GetCenterOfMassPosition() - inManifold.mBaseOffset);
	Vec3 com2 = Vec3(inBody2.GetCenterOfMassPosition() - inManifold.mBaseOffset);

	struct AxisConstraint
	{
		inline void		Initialize(Vec3Arg inR1, Vec3Arg inR2, Vec3Arg inWorldSpaceNormal, float inInvM1, float inInvM2, Mat44Arg inInvI1, Mat44Arg inInvI2)
		{
			// Calculate effective mass: K^-1 = (J M^-1 J^T)^-1
			mR1PlusUxAxis = inR1.Cross(inWorldSpaceNormal);
			mR2xAxis = inR2.Cross(inWorldSpaceNormal);
			mInvI1_R1PlusUxAxis = inInvI1.Multiply3x3(mR1PlusUxAxis);
			mInvI2_R2xAxis = inInvI2.Multiply3x3(mR2xAxis);
			mEffectiveMass = 1.0f / (inInvM1 + mInvI1_R1PlusUxAxis.Dot(mR1PlusUxAxis) + inInvM2 + mInvI2_R2xAxis.Dot(mR2xAxis));
			mBias = 0.0f;
		}

		inline float	SolveGetLambda(Vec3Arg inWorldSpaceNormal, const CollisionEstimationResult &inResult) const
		{
			// Calculate jacobian multiplied by linear/angular velocity
			float jv = inWorldSpaceNormal.Dot(inResult.mLinearVelocity1 - inResult.mLinearVelocity2) + mR1PlusUxAxis.Dot(inResult.mAngularVelocity1) - mR2xAxis.Dot(inResult.mAngularVelocity2);

			// Lagrange multiplier is:
			//
			// lambda = -K^-1 (J v + b)
			return mEffectiveMass * (jv - mBias);
		}

		inline void		SolveApplyLambda(Vec3Arg inWorldSpaceNormal, float inInvM1, float inInvM2, float inLambda, CollisionEstimationResult &ioResult) const
		{
			// Apply impulse to body velocities
			ioResult.mLinearVelocity1 -= (inLambda * inInvM1) * inWorldSpaceNormal;
			ioResult.mAngularVelocity1 -= inLambda * mInvI1_R1PlusUxAxis;
			ioResult.mLinearVelocity2 += (inLambda * inInvM2) * inWorldSpaceNormal;
			ioResult.mAngularVelocity2 += inLambda * mInvI2_R2xAxis;
		}

		inline void		Solve(Vec3Arg inWorldSpaceNormal, float inInvM1, float inInvM2, float inMinLambda, float inMaxLambda, float &ioTotalLambda, CollisionEstimationResult &ioResult) const
		{
			// Calculate new total lambda
			float total_lambda = ioTotalLambda + SolveGetLambda(inWorldSpaceNormal, ioResult);

			// Clamp impulse
			total_lambda = Clamp(total_lambda, inMinLambda, inMaxLambda);

			SolveApplyLambda(inWorldSpaceNormal, inInvM1, inInvM2, total_lambda - ioTotalLambda, ioResult);

			ioTotalLambda = total_lambda;
		}

		Vec3			mR1PlusUxAxis;
		Vec3			mR2xAxis;
		Vec3			mInvI1_R1PlusUxAxis;
		Vec3			mInvI2_R2xAxis;
		float			mEffectiveMass;
		float			mBias;
	};

	struct Constraint
	{
		AxisConstraint	mContact;
		AxisConstraint	mFriction1;
		AxisConstraint	mFriction2;
	};

	// Initialize the constraint properties
	Constraint constraints[ContactPoints::Capacity];
	for (uint c = 0; c < num_points; ++c)
	{
		Constraint &constraint = constraints[c];

		// Calculate contact points relative to body 1 and 2
		Vec3 p = 0.5f * (inManifold.mRelativeContactPointsOn1[c] + inManifold.mRelativeContactPointsOn2[c]);
		Vec3 r1 = p - com1;
		Vec3 r2 = p - com2;

		// Initialize contact constraint
		constraint.mContact.Initialize(r1, r2, inManifold.mWorldSpaceNormal, inv_m1, inv_m2, inv_i1, inv_i2);

		// Handle elastic collisions
		if (inCombinedRestitution > 0.0f)
		{
			// Calculate velocity of contact point
			Vec3 relative_velocity = outResult.mLinearVelocity2 + outResult.mAngularVelocity2.Cross(r2) - outResult.mLinearVelocity1 - outResult.mAngularVelocity1.Cross(r1);
			float normal_velocity = relative_velocity.Dot(inManifold.mWorldSpaceNormal);

			// If it is big enough, apply restitution
			if (normal_velocity < -inMinVelocityForRestitution)
				constraint.mContact.mBias = inCombinedRestitution * normal_velocity;
		}

		if (inCombinedFriction > 0.0f)
		{
			// Initialize friction constraints
			constraint.mFriction1.Initialize(r1, r2, outResult.mTangent1, inv_m1, inv_m2, inv_i1, inv_i2);
			constraint.mFriction2.Initialize(r1, r2, outResult.mTangent2, inv_m1, inv_m2, inv_i1, inv_i2);
		}
	}

	// If there's only 1 contact point, we only need 1 iteration
	int num_iterations = inCombinedFriction <= 0.0f && num_points == 1? 1 : inNumIterations;

	// Solve iteratively
	for (int iteration = 0; iteration < num_iterations; ++iteration)
	{
		// Solve friction constraints first
		if (inCombinedFriction > 0.0f && iteration > 0) // For first iteration the contact impulse is zero so there's no point in applying friction
			for (uint c = 0; c < num_points; ++c)
			{
				const Constraint &constraint = constraints[c];
				CollisionEstimationResult::Impulse &impulse = outResult.mImpulses[c];

				float lambda1 = impulse.mFrictionImpulse1 + constraint.mFriction1.SolveGetLambda(outResult.mTangent1, outResult);
				float lambda2 = impulse.mFrictionImpulse2 + constraint.mFriction2.SolveGetLambda(outResult.mTangent2, outResult);

				// Calculate max impulse based on contact impulse
				float max_impulse = inCombinedFriction * impulse.mContactImpulse;

				// If the total lambda that we will apply is too large, scale it back
				float total_lambda_sq = Square(lambda1) + Square(lambda2);
				if (total_lambda_sq > Square(max_impulse))
				{
					float scale = max_impulse / sqrt(total_lambda_sq);
					lambda1 *= scale;
					lambda2 *= scale;
				}

				constraint.mFriction1.SolveApplyLambda(outResult.mTangent1, inv_m1, inv_m2, lambda1 - impulse.mFrictionImpulse1, outResult);
				constraint.mFriction2.SolveApplyLambda(outResult.mTangent2, inv_m1, inv_m2, lambda2 - impulse.mFrictionImpulse2, outResult);

				impulse.mFrictionImpulse1 = lambda1;
				impulse.mFrictionImpulse2 = lambda2;
			}

		// Solve contact constraints last
		for (uint c = 0; c < num_points; ++c)
			constraints[c].mContact.Solve(inManifold.mWorldSpaceNormal, inv_m1, inv_m2, 0.0f, FLT_MAX, outResult.mImpulses[c].mContactImpulse, outResult);
	}
}

/*													*/
void PruneContactPoints(Vec3Arg inPenetrationAxis, ContactPoints &ioContactPointsOn1, ContactPoints &ioContactPointsOn2 MOSS_IF_DEBUG_RENDERER(, RVec3Arg inCenterOfMass))
{
	// Makes no sense to call this with 4 or less points
	MOSS_ASSERT(ioContactPointsOn1.size() > 4);

	// Both arrays should have the same size
	MOSS_ASSERT(ioContactPointsOn1.size() == ioContactPointsOn2.size());

	// Penetration axis must be normalized
	MOSS_ASSERT(inPenetrationAxis.IsNormalized());

	// We use a heuristic of (distance to center of mass) * (penetration depth) to find the contact point that we should keep
	// Neither of those two terms should ever become zero, so we clamp against this minimum value
	constexpr float cMinDistanceSq = 1.0e-6f; // 1 mm

	ContactPoints projected;
	TStaticArray<float, 64> penetration_depth_sq;
	for (ContactPoints::size_type i = 0; i < ioContactPointsOn1.size(); ++i)
	{
		// Project contact points on the plane through inCenterOfMass with normal inPenetrationAxis and center around the center of mass of body 1
		// (note that since all points are relative to inCenterOfMass we can project onto the plane through the origin)
		Vec3 v1 = ioContactPointsOn1[i];
		projected.push_back(v1 - v1.Dot(inPenetrationAxis) * inPenetrationAxis);

		// Calculate penetration depth^2 of each point and clamp against the minimal distance
		Vec3 v2 = ioContactPointsOn2[i];
		penetration_depth_sq.push_back(max(cMinDistanceSq, (v2 - v1).LengthSq()));
	}

	// Find the point that is furthest away from the center of mass (its torque will have the biggest influence)
	// and the point that has the deepest penetration depth. Use the heuristic (distance to center of mass) * (penetration depth) for this.
	uint point1 = 0;
	float val = max(cMinDistanceSq, projected[0].LengthSq()) * penetration_depth_sq[0];
	for (uint i = 0; i < projected.size(); ++i)
	{
		float v = max(cMinDistanceSq, projected[i].LengthSq()) * penetration_depth_sq[i];
		if (v > val)
		{
			val = v;
			point1 = i;
		}
	}
	Vec3 point1v = projected[point1];

	// Find point furthest from the first point forming a line segment with point1. Again combine this with the heuristic
	// for deepest point as per above.
	uint point2 = uint(-1);
	val = -FLT_MAX;
	for (uint i = 0; i < projected.size(); ++i)
		if (i != point1)
		{
			float v = max(cMinDistanceSq, (projected[i] - point1v).LengthSq()) * penetration_depth_sq[i];
			if (v > val)
			{
				val = v;
				point2 = i;
			}
		}
	MOSS_ASSERT(point2 != uint(-1));
	Vec3 point2v = projected[point2];

	// Find furthest points on both sides of the line segment in order to maximize the area
	uint point3 = uint(-1);
	uint point4 = uint(-1);
	float min_val = 0.0f;
	float max_val = 0.0f;
	Vec3 perp = (point2v - point1v).Cross(inPenetrationAxis);
	for (uint i = 0; i < projected.size(); ++i)
		if (i != point1 && i != point2)
		{
			float v = perp.Dot(projected[i] - point1v);
			if (v < min_val)
			{
				min_val = v;
				point3 = i;
			}
			else if (v > max_val)
			{
				max_val = v;
				point4 = i;
			}
		}

	// Add points to array (in order so they form a polygon)
	TStaticArray<Vec3, 4> points_to_keep_on_1, points_to_keep_on_2;
	points_to_keep_on_1.push_back(ioContactPointsOn1[point1]);
	points_to_keep_on_2.push_back(ioContactPointsOn2[point1]);
	if (point3 != uint(-1))
	{
		points_to_keep_on_1.push_back(ioContactPointsOn1[point3]);
		points_to_keep_on_2.push_back(ioContactPointsOn2[point3]);
	}
	points_to_keep_on_1.push_back(ioContactPointsOn1[point2]);
	points_to_keep_on_2.push_back(ioContactPointsOn2[point2]);
	if (point4 != uint(-1))
	{
		MOSS_ASSERT(point3 != point4);
		points_to_keep_on_1.push_back(ioContactPointsOn1[point4]);
		points_to_keep_on_2.push_back(ioContactPointsOn2[point4]);
	}

#ifndef MOSS_DEBUG_RENDERER
	if (ContactConstraintManager::sDrawContactPointReduction)
	{
		// Draw input polygon
		DebugRenderer::sInstance->DrawWirePolygon(RMat44::sTranslation(inCenterOfMass), ioContactPointsOn1, Color::sOrange, 0.05f);

		// Draw primary axis
		DebugRenderer::sInstance->DrawArrow(inCenterOfMass + ioContactPointsOn1[point1], inCenterOfMass + ioContactPointsOn1[point2], Color::sRed, 0.05f);

		// Draw contact points we kept
		for (Vec3 p : points_to_keep_on_1)
			DebugRenderer::sInstance->DrawMarker(inCenterOfMass + p, Color::sGreen, 0.1f);
	}
#endif // MOSS_DEBUG_RENDERER

	// Copy the points back to the input buffer
	ioContactPointsOn1 = points_to_keep_on_1;
	ioContactPointsOn2 = points_to_keep_on_2;
}

void ManifoldBetweenTwoFaces(Vec3Arg inContactPoint1, Vec3Arg inContactPoint2, Vec3Arg inPenetrationAxis, float inMaxContactDistance, const ConvexShape::SupportingFace &inShape1Face, const ConvexShape::SupportingFace &inShape2Face, ContactPoints &outContactPoints1, ContactPoints &outContactPoints2 MOSS_IF_DEBUG_RENDERER(, RVec3Arg inCenterOfMass))
{
	MOSS_ASSERT(inMaxContactDistance > 0.0f);

#ifndef MOSS_DEBUG_RENDERER
	if (ContactConstraintManager::sDrawContactPoint)
	{
		RVec3 cp1 = inCenterOfMass + inContactPoint1;
		RVec3 cp2 = inCenterOfMass + inContactPoint2;

		// Draw contact points
		DebugRenderer::sInstance->DrawMarker(cp1, Color::sRed, 0.1f);
		DebugRenderer::sInstance->DrawMarker(cp2, Color::sGreen, 0.1f);

		// Draw contact normal
		DebugRenderer::sInstance->DrawArrow(cp1, cp1 + inPenetrationAxis.Normalized(), Color::sRed, 0.05f);
	}
#endif // MOSS_DEBUG_RENDERER

	// Remember size before adding new points, to check at the end if we added some
	ContactPoints::size_type old_size = outContactPoints1.size();

	// Check if both shapes have polygon faces
	if (inShape1Face.size() >= 2 // The dynamic shape needs to have at least 2 points or else there can never be more than 1 contact point
		&& inShape2Face.size() >= 3) // The dynamic/static shape needs to have at least 3 points (in the case that it has 2 points only if the edges match exactly you can have 2 contact points, but this situation is unstable anyhow)
	{
		// Clip the polygon of face 2 against that of 1
		ConvexShape::SupportingFace clipped_face;
		if (inShape1Face.size() >= 3)
			ClipPolyVsPoly(inShape2Face, inShape1Face, inPenetrationAxis, clipped_face);
		else if (inShape1Face.size() == 2)
			ClipPolyVsEdge(inShape2Face, inShape1Face[0], inShape1Face[1], inPenetrationAxis, clipped_face);

		// Determine plane origin and normal for shape 1
		Vec3 plane_origin = inShape1Face[0];
		Vec3 plane_normal;
		Vec3 first_edge = inShape1Face[1] - plane_origin;
		if (inShape1Face.size() >= 3)
		{
			// Three vertices, can just calculate the normal
			plane_normal = first_edge.Cross(inShape1Face[2] - plane_origin);
		}
		else
		{
			// Two vertices, first find a perpendicular to the edge and penetration axis and then use the perpendicular together with the edge to form a normal
			plane_normal = first_edge.Cross(inPenetrationAxis).Cross(first_edge);
		}

		// If penetration axis and plane normal are perpendicular, fall back to the contact points
		float penetration_axis_dot_plane_normal = inPenetrationAxis.Dot(plane_normal);
		if (penetration_axis_dot_plane_normal != 0.0f)
		{
			float penetration_axis_len = inPenetrationAxis.Length();

			for (Vec3 p2 : clipped_face)
			{
				// Project clipped face back onto the plane of face 1, we do this by solving:
				// p1 = p2 + distance * penetration_axis / |penetration_axis|
				// (p1 - plane_origin) . plane_normal = 0
				// This gives us:
				// distance = -|penetration_axis| * (p2 - plane_origin) . plane_normal / penetration_axis . plane_normal
				float distance = (p2 - plane_origin).Dot(plane_normal) / penetration_axis_dot_plane_normal; // note left out -|penetration_axis| term

				// If the point is less than inMaxContactDistance in front of the plane of face 2, add it as a contact point
				if (distance * penetration_axis_len < inMaxContactDistance)
				{
					Vec3 p1 = p2 - distance * inPenetrationAxis;
					outContactPoints1.push_back(p1);
					outContactPoints2.push_back(p2);
				}
			}
		}

	#ifndef MOSS_DEBUG_RENDERER
		if (ContactConstraintManager::sDrawSupportingFaces)
		{
			RMat44 com = RMat44::sTranslation(inCenterOfMass);

			// Draw clipped poly
			DebugRenderer::sInstance->DrawWirePolygon(com, clipped_face, Color::sOrange);

			// Draw supporting faces
			DebugRenderer::sInstance->DrawWirePolygon(com, inShape1Face, Color::sRed, 0.05f);
			DebugRenderer::sInstance->DrawWirePolygon(com, inShape2Face, Color::sGreen, 0.05f);

			// Draw normal
			float plane_normal_len = plane_normal.Length();
			if (plane_normal_len > 0.0f)
			{
				RVec3 plane_origin_ws = inCenterOfMass + plane_origin;
				DebugRenderer::sInstance->DrawArrow(plane_origin_ws, plane_origin_ws + plane_normal / plane_normal_len, Color::sYellow, 0.05f);
			}

			// Draw contact points that remain after distance check
			for (ContactPoints::size_type p = old_size; p < outContactPoints1.size(); ++p)
			{
				DebugRenderer::sInstance->DrawMarker(inCenterOfMass + outContactPoints1[p], Color::sYellow, 0.1f);
				DebugRenderer::sInstance->DrawMarker(inCenterOfMass + outContactPoints2[p], Color::sOrange, 0.1f);
			}
		}
	#endif // MOSS_DEBUG_RENDERER
	}

	// If the clipping result is empty, use the contact point itself
	if (outContactPoints1.size() == old_size)
	{
		outContactPoints1.push_back(inContactPoint1);
		outContactPoints2.push_back(inContactPoint2);
	}
}

/*													*/

bool NarrowPhaseQuery::CastRay(const RRayCast &inRay, RayCastResult &ioHit, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter, const BodyFilter &inBodyFilter) const
{
	MOSS_PROFILE_FUNCTION();

	class MyCollector : public RayCastBodyCollector
	{
	public:
							MyCollector(const RRayCast &inRay, RayCastResult &ioHit, const BodyLockInterface &inBodyLockInterface, const BodyFilter &inBodyFilter) :
			mRay(inRay),
			mHit(ioHit),
			mBodyLockInterface(inBodyLockInterface),
			mBodyFilter(inBodyFilter)
		{
			ResetEarlyOutFraction(ioHit.mFraction);
		}

		virtual void		AddHit(const ResultType &inResult) override
		{
			MOSS_ASSERT(inResult.mFraction < mHit.mFraction, "This hit should not have been passed on to the collector");

			// Only test shape if it passes the body filter
			if (mBodyFilter.ShouldCollide(inResult.mBodyID))
			{
				// Lock the body
				BodyLockRead lock(mBodyLockInterface, inResult.mBodyID);
				if (lock.SucceededAndIsInBroadPhase()) // Race condition: body could have been removed since it has been found in the broadphase, ensures body is in the broadphase while we call the callbacks
				{
					const Body &body = lock.GetBody();

					// Check body filter again now that we've locked the body
					if (mBodyFilter.ShouldCollideLocked(body))
					{
						// Collect the transformed shape
						TransformedShape ts = body.GetTransformedShape();

						// Release the lock now, we have all the info we need in the transformed shape
						lock.ReleaseLock();

						// Do narrow phase collision check
						if (ts.CastRay(mRay, mHit))
						{
							// Test that we didn't find a further hit by accident
							MOSS_ASSERT(mHit.mFraction >= 0.0f && mHit.mFraction < GetEarlyOutFraction());

							// Update early out fraction based on narrow phase collector
							UpdateEarlyOutFraction(mHit.mFraction);
						}
					}
				}
			}
		}

		RRayCast					mRay;
		RayCastResult &				mHit;
		const BodyLockInterface &	mBodyLockInterface;
		const BodyFilter &			mBodyFilter;
	};

	// Do broadphase test, note that the broadphase uses floats so we drop precision here
	MyCollector collector(inRay, ioHit, *mBodyLockInterface, inBodyFilter);
	mBroadPhaseQuery->CastRay(RayCast(inRay), collector, inBroadPhaseLayerFilter, inObjectLayerFilter);
	return ioHit.mFraction <= 1.0f;
}

void NarrowPhaseQuery::CastRay(const RRayCast &inRay, const RayCastSettings &inRayCastSettings, CastRayCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter, const BodyFilter &inBodyFilter, const ShapeFilter &inShapeFilter) const
{
	MOSS_PROFILE_FUNCTION();

	class MyCollector : public RayCastBodyCollector
	{
	public:
							MyCollector(const RRayCast &inRay, const RayCastSettings &inRayCastSettings, CastRayCollector &ioCollector, const BodyLockInterface &inBodyLockInterface, const BodyFilter &inBodyFilter, const ShapeFilter &inShapeFilter) :
			RayCastBodyCollector(ioCollector),
			mRay(inRay),
			mRayCastSettings(inRayCastSettings),
			mCollector(ioCollector),
			mBodyLockInterface(inBodyLockInterface),
			mBodyFilter(inBodyFilter),
			mShapeFilter(inShapeFilter)
		{
		}

		virtual void		AddHit(const ResultType &inResult) override
		{
			MOSS_ASSERT(inResult.mFraction < mCollector.GetEarlyOutFraction(), "This hit should not have been passed on to the collector");

			// Only test shape if it passes the body filter
			if (mBodyFilter.ShouldCollide(inResult.mBodyID))
			{
				// Lock the body
				BodyLockRead lock(mBodyLockInterface, inResult.mBodyID);
				if (lock.SucceededAndIsInBroadPhase()) // Race condition: body could have been removed since it has been found in the broadphase, ensures body is in the broadphase while we call the callbacks
				{
					const Body &body = lock.GetBody();

					// Check body filter again now that we've locked the body
					if (mBodyFilter.ShouldCollideLocked(body))
					{
						// Collect the transformed shape
						TransformedShape ts = body.GetTransformedShape();

						// Notify collector of new body
						mCollector.OnBody(body);

						// Release the lock now, we have all the info we need in the transformed shape
						lock.ReleaseLock();

						// Do narrow phase collision check
						ts.CastRay(mRay, mRayCastSettings, mCollector, mShapeFilter);

						// Notify collector of the end of this body
						// We do this before updating the early out fraction so that the collector can still modify it
						mCollector.OnBodyEnd();

						// Update early out fraction based on narrow phase collector
						UpdateEarlyOutFraction(mCollector.GetEarlyOutFraction());
					}
				}
			}
		}

		RRayCast					mRay;
		RayCastSettings				mRayCastSettings;
		CastRayCollector &			mCollector;
		const BodyLockInterface &	mBodyLockInterface;
		const BodyFilter &			mBodyFilter;
		const ShapeFilter &			mShapeFilter;
	};

	// Do broadphase test, note that the broadphase uses floats so we drop precision here
	MyCollector collector(inRay, inRayCastSettings, ioCollector, *mBodyLockInterface, inBodyFilter, inShapeFilter);
	mBroadPhaseQuery->CastRay(RayCast(inRay), collector, inBroadPhaseLayerFilter, inObjectLayerFilter);
}

void NarrowPhaseQuery::CollidePoint(RVec3Arg inPoint, CollidePointCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter, const BodyFilter &inBodyFilter, const ShapeFilter &inShapeFilter) const
{
	MOSS_PROFILE_FUNCTION();

	class MyCollector : public CollideShapeBodyCollector
	{
	public:
							MyCollector(RVec3Arg inPoint, CollidePointCollector &ioCollector, const BodyLockInterface &inBodyLockInterface, const BodyFilter &inBodyFilter, const ShapeFilter &inShapeFilter) :
			CollideShapeBodyCollector(ioCollector),
			mPoint(inPoint),
			mCollector(ioCollector),
			mBodyLockInterface(inBodyLockInterface),
			mBodyFilter(inBodyFilter),
			mShapeFilter(inShapeFilter)
		{
		}

		virtual void		AddHit(const ResultType &inResult) override
		{
			// Only test shape if it passes the body filter
			if (mBodyFilter.ShouldCollide(inResult))
			{
				// Lock the body
				BodyLockRead lock(mBodyLockInterface, inResult);
				if (lock.SucceededAndIsInBroadPhase()) // Race condition: body could have been removed since it has been found in the broadphase, ensures body is in the broadphase while we call the callbacks
				{
					const Body &body = lock.GetBody();

					// Check body filter again now that we've locked the body
					if (mBodyFilter.ShouldCollideLocked(body))
					{
						// Collect the transformed shape
						TransformedShape ts = body.GetTransformedShape();

						// Notify collector of new body
						mCollector.OnBody(body);

						// Release the lock now, we have all the info we need in the transformed shape
						lock.ReleaseLock();

						// Do narrow phase collision check
						ts.CollidePoint(mPoint, mCollector, mShapeFilter);

						// Notify collector of the end of this body
						// We do this before updating the early out fraction so that the collector can still modify it
						mCollector.OnBodyEnd();

						// Update early out fraction based on narrow phase collector
						UpdateEarlyOutFraction(mCollector.GetEarlyOutFraction());
					}
				}
			}
		}

		RVec3							mPoint;
		CollidePointCollector &			mCollector;
		const BodyLockInterface &		mBodyLockInterface;
		const BodyFilter &				mBodyFilter;
		const ShapeFilter &				mShapeFilter;
	};

	// Do broadphase test (note: truncates double to single precision since the broadphase uses single precision)
	MyCollector collector(inPoint, ioCollector, *mBodyLockInterface, inBodyFilter, inShapeFilter);
	mBroadPhaseQuery->CollidePoint(Vec3(inPoint), collector, inBroadPhaseLayerFilter, inObjectLayerFilter);
}

void NarrowPhaseQuery::CollideShape(const Shape *inShape, Vec3Arg inShapeScale, RMat44Arg inCenterOfMassTransform, const CollideShapeSettings &inCollideShapeSettings, RVec3Arg inBaseOffset, CollideShapeCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter, const BodyFilter &inBodyFilter, const ShapeFilter &inShapeFilter) const
{
	MOSS_PROFILE_FUNCTION();

	class MyCollector : public CollideShapeBodyCollector
	{
	public:
							MyCollector(const Shape *inShape, Vec3Arg inShapeScale, RMat44Arg inCenterOfMassTransform, const CollideShapeSettings &inCollideShapeSettings, RVec3Arg inBaseOffset, CollideShapeCollector &ioCollector, const BodyLockInterface &inBodyLockInterface, const BodyFilter &inBodyFilter, const ShapeFilter &inShapeFilter) :
			CollideShapeBodyCollector(ioCollector),
			mShape(inShape),
			mShapeScale(inShapeScale),
			mCenterOfMassTransform(inCenterOfMassTransform),
			mCollideShapeSettings(inCollideShapeSettings),
			mBaseOffset(inBaseOffset),
			mCollector(ioCollector),
			mBodyLockInterface(inBodyLockInterface),
			mBodyFilter(inBodyFilter),
			mShapeFilter(inShapeFilter)
		{
		}

		virtual void		AddHit(const ResultType &inResult) override
		{
			// Only test shape if it passes the body filter
			if (mBodyFilter.ShouldCollide(inResult))
			{
				// Lock the body
				BodyLockRead lock(mBodyLockInterface, inResult);
				if (lock.SucceededAndIsInBroadPhase()) // Race condition: body could have been removed since it has been found in the broadphase, ensures body is in the broadphase while we call the callbacks
				{
					const Body &body = lock.GetBody();

					// Check body filter again now that we've locked the body
					if (mBodyFilter.ShouldCollideLocked(body))
					{
						// Collect the transformed shape
						TransformedShape ts = body.GetTransformedShape();

						// Notify collector of new body
						mCollector.OnBody(body);

						// Release the lock now, we have all the info we need in the transformed shape
						lock.ReleaseLock();

						// Do narrow phase collision check
						ts.CollideShape(mShape, mShapeScale, mCenterOfMassTransform, mCollideShapeSettings, mBaseOffset, mCollector, mShapeFilter);

						// Notify collector of the end of this body
						// We do this before updating the early out fraction so that the collector can still modify it
						mCollector.OnBodyEnd();

						// Update early out fraction based on narrow phase collector
						UpdateEarlyOutFraction(mCollector.GetEarlyOutFraction());
					}
				}
			}
		}

		const Shape *					mShape;
		Vec3							mShapeScale;
		RMat44							mCenterOfMassTransform;
		const CollideShapeSettings &	mCollideShapeSettings;
		RVec3							mBaseOffset;
		CollideShapeCollector &			mCollector;
		const BodyLockInterface &		mBodyLockInterface;
		const BodyFilter &				mBodyFilter;
		const ShapeFilter &				mShapeFilter;
	};

	// Calculate bounds for shape and expand by max separation distance
	AABox bounds = inShape->GetWorldSpaceBounds(inCenterOfMassTransform, inShapeScale);
	bounds.ExpandBy(Vec3::sReplicate(inCollideShapeSettings.mMaxSeparationDistance));

	// Do broadphase test
	MyCollector collector(inShape, inShapeScale, inCenterOfMassTransform, inCollideShapeSettings, inBaseOffset, ioCollector, *mBodyLockInterface, inBodyFilter, inShapeFilter);
	mBroadPhaseQuery->CollideAABox(bounds, collector, inBroadPhaseLayerFilter, inObjectLayerFilter);
}

void NarrowPhaseQuery::CollideShapeWithInternalEdgeRemoval(const Shape *inShape, Vec3Arg inShapeScale, RMat44Arg inCenterOfMassTransform, const CollideShapeSettings &inCollideShapeSettings, RVec3Arg inBaseOffset, CollideShapeCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter, const BodyFilter &inBodyFilter, const ShapeFilter &inShapeFilter) const
{
	// We require these settings for internal edge removal to work
	CollideShapeSettings settings = inCollideShapeSettings;
	settings.mActiveEdgeMode = EActiveEdgeMode::CollideWithAll;
	settings.mCollectFacesMode = ECollectFacesMode::CollectFaces;

	InternalEdgeRemovingCollector wrapper(ioCollector);
	CollideShape(inShape, inShapeScale, inCenterOfMassTransform, settings, inBaseOffset, wrapper, inBroadPhaseLayerFilter, inObjectLayerFilter, inBodyFilter, inShapeFilter);
}

void NarrowPhaseQuery::CastShape(const RShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, RVec3Arg inBaseOffset, CastShapeCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter, const BodyFilter &inBodyFilter, const ShapeFilter &inShapeFilter) const
{
	MOSS_PROFILE_FUNCTION();

	class MyCollector : public CastShapeBodyCollector
	{
	public:
							MyCollector(const RShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, RVec3Arg inBaseOffset, CastShapeCollector &ioCollector, const BodyLockInterface &inBodyLockInterface, const BodyFilter &inBodyFilter, const ShapeFilter &inShapeFilter) :
			CastShapeBodyCollector(ioCollector),
			mShapeCast(inShapeCast),
			mShapeCastSettings(inShapeCastSettings),
			mBaseOffset(inBaseOffset),
			mCollector(ioCollector),
			mBodyLockInterface(inBodyLockInterface),
			mBodyFilter(inBodyFilter),
			mShapeFilter(inShapeFilter)
		{
		}

		virtual void		AddHit(const ResultType &inResult) override
		{
			MOSS_ASSERT(inResult.mFraction <= max(0.0f, mCollector.GetEarlyOutFraction()), "This hit should not have been passed on to the collector");

			// Only test shape if it passes the body filter
			if (mBodyFilter.ShouldCollide(inResult.mBodyID))
			{
				// Lock the body
				BodyLockRead lock(mBodyLockInterface, inResult.mBodyID);
				if (lock.SucceededAndIsInBroadPhase()) // Race condition: body could have been removed since it has been found in the broadphase, ensures body is in the broadphase while we call the callbacks
				{
					const Body &body = lock.GetBody();

					// Check body filter again now that we've locked the body
					if (mBodyFilter.ShouldCollideLocked(body))
					{
						// Collect the transformed shape
						TransformedShape ts = body.GetTransformedShape();

						// Notify collector of new body
						mCollector.OnBody(body);

						// Release the lock now, we have all the info we need in the transformed shape
						lock.ReleaseLock();

						// Do narrow phase collision check
						ts.CastShape(mShapeCast, mShapeCastSettings, mBaseOffset, mCollector, mShapeFilter);

						// Notify collector of the end of this body
						// We do this before updating the early out fraction so that the collector can still modify it
						mCollector.OnBodyEnd();

						// Update early out fraction based on narrow phase collector
						UpdateEarlyOutFraction(mCollector.GetEarlyOutFraction());
					}
				}
			}
		}

		RShapeCast					mShapeCast;
		const ShapeCastSettings &	mShapeCastSettings;
		RVec3						mBaseOffset;
		CastShapeCollector &		mCollector;
		const BodyLockInterface &	mBodyLockInterface;
		const BodyFilter &			mBodyFilter;
		const ShapeFilter &			mShapeFilter;
	};

	// Do broadphase test
	MyCollector collector(inShapeCast, inShapeCastSettings, inBaseOffset, ioCollector, *mBodyLockInterface, inBodyFilter, inShapeFilter);
	mBroadPhaseQuery->CastAABox({ inShapeCast.mShapeWorldBounds, inShapeCast.mDirection }, collector, inBroadPhaseLayerFilter, inObjectLayerFilter);
}

void NarrowPhaseQuery::CollectTransformedShapes(const AABox &inBox, TransformedShapeCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter, const BodyFilter &inBodyFilter, const ShapeFilter &inShapeFilter) const
{
	class MyCollector : public CollideShapeBodyCollector
	{
	public:
							MyCollector(const AABox &inBox, TransformedShapeCollector &ioCollector, const BodyLockInterface &inBodyLockInterface, const BodyFilter &inBodyFilter, const ShapeFilter &inShapeFilter) :
			CollideShapeBodyCollector(ioCollector),
			mBox(inBox),
			mCollector(ioCollector),
			mBodyLockInterface(inBodyLockInterface),
			mBodyFilter(inBodyFilter),
			mShapeFilter(inShapeFilter)
		{
		}

		virtual void		AddHit(const ResultType &inResult) override
		{
			// Only test shape if it passes the body filter
			if (mBodyFilter.ShouldCollide(inResult))
			{
				// Lock the body
				BodyLockRead lock(mBodyLockInterface, inResult);
				if (lock.SucceededAndIsInBroadPhase()) // Race condition: body could have been removed since it has been found in the broadphase, ensures body is in the broadphase while we call the callbacks
				{
					const Body &body = lock.GetBody();

					// Check body filter again now that we've locked the body
					if (mBodyFilter.ShouldCollideLocked(body))
					{
						// Collect the transformed shape
						TransformedShape ts = body.GetTransformedShape();

						// Notify collector of new body
						mCollector.OnBody(body);

						// Release the lock now, we have all the info we need in the transformed shape
						lock.ReleaseLock();

						// Do narrow phase collision check
						ts.CollectTransformedShapes(mBox, mCollector, mShapeFilter);

						// Notify collector of the end of this body
						// We do this before updating the early out fraction so that the collector can still modify it
						mCollector.OnBodyEnd();

						// Update early out fraction based on narrow phase collector
						UpdateEarlyOutFraction(mCollector.GetEarlyOutFraction());
					}
				}
			}
		}

		const AABox &					mBox;
		TransformedShapeCollector &		mCollector;
		const BodyLockInterface &		mBodyLockInterface;
		const BodyFilter &				mBodyFilter;
		const ShapeFilter &				mShapeFilter;
	};

	// Do broadphase test
	MyCollector collector(inBox, ioCollector, *mBodyLockInterface, inBodyFilter, inShapeFilter);
	mBroadPhaseQuery->CollideAABox(inBox, collector, inBroadPhaseLayerFilter, inObjectLayerFilter);
}

/*													*/
#ifdef MOSS_TRACK_NARROWPHASE_STATS
void NarrowPhaseStat::ReportStats(const char *inName, EShapeSubType inType1, EShapeSubType inType2, uint64 inTicks100Pct) const
{
	double total_pct = 100.0 * double(mTotalTicks) / double(inTicks100Pct);
	double total_pct_excl_children = 100.0 * double(mTotalTicks - mChildTicks) / double(inTicks100Pct);

	std::stringstream str;
	str << inName << ", " << sSubShapeTypeNames[(int)inType1] << ", " << sSubShapeTypeNames[(int)inType2] << ", " << mNumQueries << ", " << total_pct << ", " << total_pct_excl_children << ", " << total_pct_excl_children / mNumQueries << ", " << mHitsReported;
	MOSS_TRACE(str.str().c_str());
}

void NarrowPhaseStat::sReportStats()
{
	MOSS_TRACE("Query Type, Shape Type 1, Shape Type 2, Num Queries, Total Time (%%), Total Time Excl Children (%%), Total Time Excl. Children / Query (%%), Hits Reported");

	uint64 total_ticks = 0;
	for (EShapeSubType t1 : sAllSubShapeTypes)
		for (EShapeSubType t2 : sAllSubShapeTypes)
		{
			const NarrowPhaseStat &collide_stat = sCollideShape[(int)t1][(int)t2];
			total_ticks += collide_stat.mTotalTicks - collide_stat.mChildTicks;

			const NarrowPhaseStat &cast_stat = sCastShape[(int)t1][(int)t2];
			total_ticks += cast_stat.mTotalTicks - cast_stat.mChildTicks;
		}

	for (EShapeSubType t1 : sAllSubShapeTypes)
		for (EShapeSubType t2 : sAllSubShapeTypes)
		{
			const NarrowPhaseStat &stat = sCollideShape[(int)t1][(int)t2];
			if (stat.mNumQueries > 0)
				stat.ReportStats("CollideShape", t1, t2, total_ticks);
		}

	for (EShapeSubType t1 : sAllSubShapeTypes)
		for (EShapeSubType t2 : sAllSubShapeTypes)
		{
			const NarrowPhaseStat &stat = sCastShape[(int)t1][(int)t2];
			if (stat.mNumQueries > 0)
				stat.ReportStats("CastShape", t1, t2, total_ticks);
		}
}
#endif

// BoardPhase
/*													*/

BroadPhaseQuadTree::~BroadPhaseQuadTree()
{
	delete [] mLayers;
}

void BroadPhaseQuadTree::Init(BodyManager *inBodyManager, const BroadPhaseLayerInterface &inLayerInterface)
{
	BroadPhase::Init(inBodyManager, inLayerInterface);

	// Store input parameters
	mBroadPhaseLayerInterface = &inLayerInterface;
	mNumLayers = inLayerInterface.GetNumBroadPhaseLayers();
	MOSS_ASSERT(mNumLayers < (BroadPhaseLayer::Type)cBroadPhaseLayerInvalid);

#ifdef MOSS_DEBUG
	// Store lock context
	mLockContext = inBodyManager;
#endif // MOSS_DEBUG

	// Store max bodies
	mMaxBodies = inBodyManager->GetMaxBodies();

	// Initialize tracking data
	mTracking.resize(mMaxBodies);

	// Init allocator
	// Estimate the amount of nodes we're going to need
	uint32 num_leaves = (uint32)(mMaxBodies + 1) / 2; // Assume 50% fill
	uint32 num_leaves_plus_internal_nodes = num_leaves + (num_leaves + 2) / 3; // = Sum(num_leaves * 4^-i) with i = [0, Inf].
	mAllocator.Init(2 * num_leaves_plus_internal_nodes, 256); // We use double the amount of nodes while rebuilding the tree during Update()

	// Init sub trees
	mLayers = new QuadTree [mNumLayers];
	for (uint l = 0; l < mNumLayers; ++l)
	{
		mLayers[l].Init(mAllocator);

#if defined(MOSS_EXTERNAL_PROFILE) || defined(MOSS_PROFILE_ENABLED)
		// Set the name of the layer
		mLayers[l].SetName(inLayerInterface.GetBroadPhaseLayerName(BroadPhaseLayer(BroadPhaseLayer::Type(l))));
#endif // MOSS_EXTERNAL_PROFILE || MOSS_PROFILE_ENABLED
	}
}

void BroadPhaseQuadTree::FrameSync()
{
	MOSS_PROFILE_FUNCTION();

	// Take a unique lock on the old query lock so that we know no one is using the old nodes anymore.
	// Note that nothing should be locked at this point to avoid risking a lock inversion deadlock.
	// Note that in other places where we lock this mutex we don't use SharedLock to detect lock inversions. As long as
	// nothing else is locked this is safe. This is why BroadPhaseQuery should be the highest priority lock.
	UniqueLock root_lock(mQueryLocks[mQueryLockIdx ^ 1] MOSS_IF_ENABLE_ASSERTS(, mLockContext, EPhysicsLockTypes::BroadPhaseQuery));

	for (BroadPhaseLayer::Type l = 0; l < mNumLayers; ++l)
		mLayers[l].DiscardOldTree();
}

void BroadPhaseQuadTree::Optimize()
{
	MOSS_PROFILE_FUNCTION();

	FrameSync();

	LockModifications();

	for (uint l = 0; l < mNumLayers; ++l)
	{
		QuadTree &tree = mLayers[l];
		if (tree.HasBodies())
		{
			QuadTree::UpdateState update_state;
			tree.UpdatePrepare(mBodyManager->GetBodies(), mTracking, update_state, true);
			tree.UpdateFinalize(mBodyManager->GetBodies(), mTracking, update_state);
		}
	}

	UnlockModifications();

	mNextLayerToUpdate = 0;
}

void BroadPhaseQuadTree::LockModifications()
{
	// From this point on we prevent modifications to the tree
	PhysicsLock::sLock(mUpdateMutex MOSS_IF_ENABLE_ASSERTS(, mLockContext, EPhysicsLockTypes::BroadPhaseUpdate));
}

BroadPhase::UpdateState BroadPhaseQuadTree::UpdatePrepare()
{
	// LockModifications should have been called
	MOSS_ASSERT(mUpdateMutex.is_locked());

	// Create update state
	UpdateState update_state;
	UpdateStateImpl *update_state_impl = reinterpret_cast<UpdateStateImpl *>(&update_state);

	// Loop until we've seen all layers
	for (uint iteration = 0; iteration < mNumLayers; ++iteration)
	{
		// Get the layer
		QuadTree &tree = mLayers[mNextLayerToUpdate];
		mNextLayerToUpdate = (mNextLayerToUpdate + 1) % mNumLayers;

		// If it is dirty we update this one
		if (tree.HasBodies() && tree.IsDirty() && tree.CanBeUpdated())
		{
			update_state_impl->mTree = &tree;
			tree.UpdatePrepare(mBodyManager->GetBodies(), mTracking, update_state_impl->mUpdateState, false);
			return update_state;
		}
	}

	// Nothing to update
	update_state_impl->mTree = nullptr;
	return update_state;
}

void BroadPhaseQuadTree::UpdateFinalize(const UpdateState &inUpdateState)
{
	// LockModifications should have been called
	MOSS_ASSERT(mUpdateMutex.is_locked());

	// Test if a tree was updated
	const UpdateStateImpl *update_state_impl = reinterpret_cast<const UpdateStateImpl *>(&inUpdateState);
	if (update_state_impl->mTree == nullptr)
		return;

	update_state_impl->mTree->UpdateFinalize(mBodyManager->GetBodies(), mTracking, update_state_impl->mUpdateState);

	// Make all queries from now on use the new lock
	mQueryLockIdx = mQueryLockIdx ^ 1;
}

void BroadPhaseQuadTree::UnlockModifications()
{
	// From this point on we allow modifications to the tree again
	PhysicsLock::sUnlock(mUpdateMutex MOSS_IF_ENABLE_ASSERTS(, mLockContext, EPhysicsLockTypes::BroadPhaseUpdate));
}

BroadPhase::AddState BroadPhaseQuadTree::AddBodiesPrepare(BodyID *ioBodies, int inNumber)
{
	MOSS_PROFILE_FUNCTION();

	if (inNumber <= 0)
		return nullptr;

	const BodyVector &bodies = mBodyManager->GetBodies();
	MOSS_ASSERT(mMaxBodies == mBodyManager->GetMaxBodies());

	LayerState *state = new LayerState [mNumLayers];

	// Sort bodies on layer
	Body * const * const bodies_ptr = bodies.data(); // C pointer or else sort is incredibly slow in debug mode
	QuickSort(ioBodies, ioBodies + inNumber, [bodies_ptr](BodyID inLHS, BodyID inRHS) { return bodies_ptr[inLHS.GetIndex()]->GetBroadPhaseLayer() < bodies_ptr[inRHS.GetIndex()]->GetBroadPhaseLayer(); });

	BodyID *b_start = ioBodies, *b_end = ioBodies + inNumber;
	while (b_start < b_end)
	{
		// Get broadphase layer
		BroadPhaseLayer::Type broadphase_layer = (BroadPhaseLayer::Type)bodies[b_start->GetIndex()]->GetBroadPhaseLayer();
		MOSS_ASSERT(broadphase_layer < mNumLayers);

		// Find first body with different layer
		BodyID *b_mid = std::upper_bound(b_start, b_end, broadphase_layer, [bodies_ptr](BroadPhaseLayer::Type inLayer, BodyID inBodyID) { return inLayer < (BroadPhaseLayer::Type)bodies_ptr[inBodyID.GetIndex()]->GetBroadPhaseLayer(); });

		// Keep track of state for this layer
		LayerState &layer_state = state[broadphase_layer];
		layer_state.mBodyStart = b_start;
		layer_state.mBodyEnd = b_mid;

		// Insert all bodies of the same layer
		mLayers[broadphase_layer].AddBodiesPrepare(bodies, mTracking, b_start, int(b_mid - b_start), layer_state.mAddState);

		// Keep track in which tree we placed the object
		for (const BodyID *b = b_start; b < b_mid; ++b)
		{
			uint32 index = b->GetIndex();
			MOSS_ASSERT(bodies[index]->GetID() == *b, "Provided BodyID doesn't match BodyID in body manager");
			MOSS_ASSERT(!bodies[index]->IsInBroadPhase());
			Tracking &t = mTracking[index];
			MOSS_ASSERT(t.mBroadPhaseLayer == (BroadPhaseLayer::Type)cBroadPhaseLayerInvalid);
			t.mBroadPhaseLayer = broadphase_layer;
			MOSS_ASSERT(t.mObjectLayer == cObjectLayerInvalid);
			t.mObjectLayer = bodies[index]->GetObjectLayer();
		}

		// Repeat
		b_start = b_mid;
	}

	return state;
}

void BroadPhaseQuadTree::AddBodiesFinalize(BodyID *ioBodies, int inNumber, AddState inAddState)
{
	MOSS_PROFILE_FUNCTION();

	if (inNumber <= 0)
	{
		MOSS_ASSERT(inAddState == nullptr);
		return;
	}

	// This cannot run concurrently with UpdatePrepare()/UpdateFinalize()
	SharedLock lock(mUpdateMutex MOSS_IF_ENABLE_ASSERTS(, mLockContext, EPhysicsLockTypes::BroadPhaseUpdate));

	BodyVector &bodies = mBodyManager->GetBodies();
	MOSS_ASSERT(mMaxBodies == mBodyManager->GetMaxBodies());

	LayerState *state = (LayerState *)inAddState;

	for (BroadPhaseLayer::Type broadphase_layer = 0; broadphase_layer < mNumLayers; broadphase_layer++)
	{
		const LayerState &l = state[broadphase_layer];
		if (l.mBodyStart != nullptr)
		{
			// Insert all bodies of the same layer
			mLayers[broadphase_layer].AddBodiesFinalize(mTracking, int(l.mBodyEnd - l.mBodyStart), l.mAddState);

			// Mark added to broadphase
			for (const BodyID *b = l.mBodyStart; b < l.mBodyEnd; ++b)
			{
				uint32 index = b->GetIndex();
				MOSS_ASSERT(bodies[index]->GetID() == *b, "Provided BodyID doesn't match BodyID in body manager");
				MOSS_ASSERT(mTracking[index].mBroadPhaseLayer == broadphase_layer);
				MOSS_ASSERT(mTracking[index].mObjectLayer == bodies[index]->GetObjectLayer());
				MOSS_ASSERT(!bodies[index]->IsInBroadPhase());
				bodies[index]->SetInBroadPhaseInternal(true);
			}
		}
	}

	delete [] state;
}

void BroadPhaseQuadTree::AddBodiesAbort(BodyID *ioBodies, int inNumber, AddState inAddState)
{
	MOSS_PROFILE_FUNCTION();

	if (inNumber <= 0)
	{
		MOSS_ASSERT(inAddState == nullptr);
		return;
	}

	MOSS_IF_ENABLE_ASSERTS(const BodyVector &bodies = mBodyManager->GetBodies();)
	MOSS_ASSERT(mMaxBodies == mBodyManager->GetMaxBodies());

	LayerState *state = (LayerState *)inAddState;

	for (BroadPhaseLayer::Type broadphase_layer = 0; broadphase_layer < mNumLayers; broadphase_layer++)
	{
		const LayerState &l = state[broadphase_layer];
		if (l.mBodyStart != nullptr)
		{
			// Insert all bodies of the same layer
			mLayers[broadphase_layer].AddBodiesAbort(mTracking, l.mAddState);

			// Reset bookkeeping
			for (const BodyID *b = l.mBodyStart; b < l.mBodyEnd; ++b)
			{
				uint32 index = b->GetIndex();
				MOSS_ASSERT(bodies[index]->GetID() == *b, "Provided BodyID doesn't match BodyID in body manager");
				MOSS_ASSERT(!bodies[index]->IsInBroadPhase());
				Tracking &t = mTracking[index];
				MOSS_ASSERT(t.mBroadPhaseLayer == broadphase_layer);
				t.mBroadPhaseLayer = (BroadPhaseLayer::Type)cBroadPhaseLayerInvalid;
				t.mObjectLayer = cObjectLayerInvalid;
			}
		}
	}

	delete [] state;
}

void BroadPhaseQuadTree::RemoveBodies(BodyID *ioBodies, int inNumber)
{
	MOSS_PROFILE_FUNCTION();

	if (inNumber <= 0)
		return;

	// This cannot run concurrently with UpdatePrepare()/UpdateFinalize()
	SharedLock lock(mUpdateMutex MOSS_IF_ENABLE_ASSERTS(, mLockContext, EPhysicsLockTypes::BroadPhaseUpdate));

	BodyVector &bodies = mBodyManager->GetBodies();
	MOSS_ASSERT(mMaxBodies == mBodyManager->GetMaxBodies());

	// Sort bodies on layer
	Tracking *tracking = mTracking.data(); // C pointer or else sort is incredibly slow in debug mode
	QuickSort(ioBodies, ioBodies + inNumber, [tracking](BodyID inLHS, BodyID inRHS) { return tracking[inLHS.GetIndex()].mBroadPhaseLayer < tracking[inRHS.GetIndex()].mBroadPhaseLayer; });

	BodyID *b_start = ioBodies, *b_end = ioBodies + inNumber;
	while (b_start < b_end)
	{
		// Get broad phase layer
		BroadPhaseLayer::Type broadphase_layer = mTracking[b_start->GetIndex()].mBroadPhaseLayer;
		MOSS_ASSERT(broadphase_layer != (BroadPhaseLayer::Type)cBroadPhaseLayerInvalid);

		// Find first body with different layer
		BodyID *b_mid = std::upper_bound(b_start, b_end, broadphase_layer, [tracking](BroadPhaseLayer::Type inLayer, BodyID inBodyID) { return inLayer < tracking[inBodyID.GetIndex()].mBroadPhaseLayer; });

		// Remove all bodies of the same layer
		mLayers[broadphase_layer].RemoveBodies(bodies, mTracking, b_start, int(b_mid - b_start));

		for (const BodyID *b = b_start; b < b_mid; ++b)
		{
			// Reset bookkeeping
			uint32 index = b->GetIndex();
			Tracking &t = tracking[index];
			t.mBroadPhaseLayer = (BroadPhaseLayer::Type)cBroadPhaseLayerInvalid;
			t.mObjectLayer = cObjectLayerInvalid;

			// Mark removed from broadphase
			MOSS_ASSERT(bodies[index]->IsInBroadPhase());
			bodies[index]->SetInBroadPhaseInternal(false);
		}

		// Repeat
		b_start = b_mid;
	}
}

void BroadPhaseQuadTree::NotifyBodiesAABBChanged(BodyID *ioBodies, int inNumber, bool inTakeLock)
{
	MOSS_PROFILE_FUNCTION();

	if (inNumber <= 0)
		return;

	// This cannot run concurrently with UpdatePrepare()/UpdateFinalize()
	if (inTakeLock)
		PhysicsLock::sLockShared(mUpdateMutex MOSS_IF_ENABLE_ASSERTS(, mLockContext, EPhysicsLockTypes::BroadPhaseUpdate));
	else
		MOSS_ASSERT(mUpdateMutex.is_locked());

	const BodyVector &bodies = mBodyManager->GetBodies();
	MOSS_ASSERT(mMaxBodies == mBodyManager->GetMaxBodies());

	// Sort bodies on layer
	const Tracking *tracking = mTracking.data(); // C pointer or else sort is incredibly slow in debug mode
	QuickSort(ioBodies, ioBodies + inNumber, [tracking](BodyID inLHS, BodyID inRHS) { return tracking[inLHS.GetIndex()].mBroadPhaseLayer < tracking[inRHS.GetIndex()].mBroadPhaseLayer; });

	BodyID *b_start = ioBodies, *b_end = ioBodies + inNumber;
	while (b_start < b_end)
	{
		// Get broadphase layer
		BroadPhaseLayer::Type broadphase_layer = tracking[b_start->GetIndex()].mBroadPhaseLayer;
		MOSS_ASSERT(broadphase_layer != (BroadPhaseLayer::Type)cBroadPhaseLayerInvalid);

		// Find first body with different layer
		BodyID *b_mid = std::upper_bound(b_start, b_end, broadphase_layer, [tracking](BroadPhaseLayer::Type inLayer, BodyID inBodyID) { return inLayer < tracking[inBodyID.GetIndex()].mBroadPhaseLayer; });

		// Notify all bodies of the same layer changed
		mLayers[broadphase_layer].NotifyBodiesAABBChanged(bodies, mTracking, b_start, int(b_mid - b_start));

		// Repeat
		b_start = b_mid;
	}

	if (inTakeLock)
		PhysicsLock::sUnlockShared(mUpdateMutex MOSS_IF_ENABLE_ASSERTS(, mLockContext, EPhysicsLockTypes::BroadPhaseUpdate));
}

void BroadPhaseQuadTree::NotifyBodiesLayerChanged(BodyID *ioBodies, int inNumber)
{
	MOSS_PROFILE_FUNCTION();

	if (inNumber <= 0)
		return;

	// First sort the bodies that actually changed layer to beginning of the array
	const BodyVector &bodies = mBodyManager->GetBodies();
	MOSS_ASSERT(mMaxBodies == mBodyManager->GetMaxBodies());
	for (BodyID *body_id = ioBodies + inNumber - 1; body_id >= ioBodies; --body_id)
	{
		uint32 index = body_id->GetIndex();
		MOSS_ASSERT(bodies[index]->GetID() == *body_id, "Provided BodyID doesn't match BodyID in body manager");
		const Body *body = bodies[index];
		BroadPhaseLayer::Type broadphase_layer = (BroadPhaseLayer::Type)body->GetBroadPhaseLayer();
		MOSS_ASSERT(broadphase_layer < mNumLayers);
		if (mTracking[index].mBroadPhaseLayer == broadphase_layer)
		{
			// Update tracking information
			mTracking[index].mObjectLayer = body->GetObjectLayer();

			// Move the body to the end, layer didn't change
			std::swap(*body_id, ioBodies[inNumber - 1]);
			--inNumber;
		}
	}

	if (inNumber > 0)
	{
		// Changing layer requires us to remove from one tree and add to another, so this is equivalent to removing all bodies first and then adding them again
		RemoveBodies(ioBodies, inNumber);
		AddState add_state = AddBodiesPrepare(ioBodies, inNumber);
		AddBodiesFinalize(ioBodies, inNumber, add_state);
	}
}

void BroadPhaseQuadTree::CastRay(const RayCast &inRay, RayCastBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter) const
{
	MOSS_PROFILE_FUNCTION();

	MOSS_ASSERT(mMaxBodies == mBodyManager->GetMaxBodies());

	// Prevent this from running in parallel with node deletion in FrameSync(), see notes there
	shared_lock lock(mQueryLocks[mQueryLockIdx]);

	// Loop over all layers and test the ones that could hit
	for (BroadPhaseLayer::Type l = 0; l < mNumLayers; ++l)
	{
		const QuadTree &tree = mLayers[l];
		if (tree.HasBodies() && inBroadPhaseLayerFilter.ShouldCollide(BroadPhaseLayer(l)))
		{
			MOSS_PROFILE(tree.GetName());
			tree.CastRay(inRay, ioCollector, inObjectLayerFilter, mTracking);
			if (ioCollector.ShouldEarlyOut())
				break;
		}
	}
}

void BroadPhaseQuadTree::CollideAABox(const AABox &inBox, CollideShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter) const
{
	MOSS_PROFILE_FUNCTION();

	MOSS_ASSERT(mMaxBodies == mBodyManager->GetMaxBodies());

	// Prevent this from running in parallel with node deletion in FrameSync(), see notes there
	shared_lock lock(mQueryLocks[mQueryLockIdx]);

	// Loop over all layers and test the ones that could hit
	for (BroadPhaseLayer::Type l = 0; l < mNumLayers; ++l)
	{
		const QuadTree &tree = mLayers[l];
		if (tree.HasBodies() && inBroadPhaseLayerFilter.ShouldCollide(BroadPhaseLayer(l)))
		{
			MOSS_PROFILE(tree.GetName());
			tree.CollideAABox(inBox, ioCollector, inObjectLayerFilter, mTracking);
			if (ioCollector.ShouldEarlyOut())
				break;
		}
	}
}

void BroadPhaseQuadTree::CollideSphere(Vec3Arg inCenter, float inRadius, CollideShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter) const
{
	MOSS_PROFILE_FUNCTION();

	MOSS_ASSERT(mMaxBodies == mBodyManager->GetMaxBodies());

	// Prevent this from running in parallel with node deletion in FrameSync(), see notes there
	shared_lock lock(mQueryLocks[mQueryLockIdx]);

	// Loop over all layers and test the ones that could hit
	for (BroadPhaseLayer::Type l = 0; l < mNumLayers; ++l)
	{
		const QuadTree &tree = mLayers[l];
		if (tree.HasBodies() && inBroadPhaseLayerFilter.ShouldCollide(BroadPhaseLayer(l)))
		{
			MOSS_PROFILE(tree.GetName());
			tree.CollideSphere(inCenter, inRadius, ioCollector, inObjectLayerFilter, mTracking);
			if (ioCollector.ShouldEarlyOut())
				break;
		}
	}
}

void BroadPhaseQuadTree::CollidePoint(Vec3Arg inPoint, CollideShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter) const
{
	MOSS_PROFILE_FUNCTION();

	MOSS_ASSERT(mMaxBodies == mBodyManager->GetMaxBodies());

	// Prevent this from running in parallel with node deletion in FrameSync(), see notes there
	shared_lock lock(mQueryLocks[mQueryLockIdx]);

	// Loop over all layers and test the ones that could hit
	for (BroadPhaseLayer::Type l = 0; l < mNumLayers; ++l)
	{
		const QuadTree &tree = mLayers[l];
		if (tree.HasBodies() && inBroadPhaseLayerFilter.ShouldCollide(BroadPhaseLayer(l)))
		{
			MOSS_PROFILE(tree.GetName());
			tree.CollidePoint(inPoint, ioCollector, inObjectLayerFilter, mTracking);
			if (ioCollector.ShouldEarlyOut())
				break;
		}
	}
}

void BroadPhaseQuadTree::CollideOrientedBox(const OrientedBox &inBox, CollideShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter) const
{
	MOSS_PROFILE_FUNCTION();

	MOSS_ASSERT(mMaxBodies == mBodyManager->GetMaxBodies());

	// Prevent this from running in parallel with node deletion in FrameSync(), see notes there
	shared_lock lock(mQueryLocks[mQueryLockIdx]);

	// Loop over all layers and test the ones that could hit
	for (BroadPhaseLayer::Type l = 0; l < mNumLayers; ++l)
	{
		const QuadTree &tree = mLayers[l];
		if (tree.HasBodies() && inBroadPhaseLayerFilter.ShouldCollide(BroadPhaseLayer(l)))
		{
			MOSS_PROFILE(tree.GetName());
			tree.CollideOrientedBox(inBox, ioCollector, inObjectLayerFilter, mTracking);
			if (ioCollector.ShouldEarlyOut())
				break;
		}
	}
}

void BroadPhaseQuadTree::CastAABoxNoLock(const AABoxCast &inBox, CastShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter) const
{
	MOSS_PROFILE_FUNCTION();

	MOSS_ASSERT(mMaxBodies == mBodyManager->GetMaxBodies());

	// Loop over all layers and test the ones that could hit
	for (BroadPhaseLayer::Type l = 0; l < mNumLayers; ++l)
	{
		const QuadTree &tree = mLayers[l];
		if (tree.HasBodies() && inBroadPhaseLayerFilter.ShouldCollide(BroadPhaseLayer(l)))
		{
			MOSS_PROFILE(tree.GetName());
			tree.CastAABox(inBox, ioCollector, inObjectLayerFilter, mTracking);
			if (ioCollector.ShouldEarlyOut())
				break;
		}
	}
}

void BroadPhaseQuadTree::CastAABox(const AABoxCast &inBox, CastShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter) const
{
	// Prevent this from running in parallel with node deletion in FrameSync(), see notes there
	shared_lock lock(mQueryLocks[mQueryLockIdx]);

	CastAABoxNoLock(inBox, ioCollector, inBroadPhaseLayerFilter, inObjectLayerFilter);
}

void BroadPhaseQuadTree::FindCollidingPairs(BodyID *ioActiveBodies, int inNumActiveBodies, float inSpeculativeContactDistance, const ObjectVsBroadPhaseLayerFilter &inObjectVsBroadPhaseLayerFilter, const ObjectLayerPairFilter &inObjectLayerPairFilter, BodyPairCollector &ioPairCollector) const
{
	MOSS_PROFILE_FUNCTION();

	const BodyVector &bodies = mBodyManager->GetBodies();
	MOSS_ASSERT(mMaxBodies == mBodyManager->GetMaxBodies());

	// Note that we don't take any locks at this point. We know that the tree is not going to be swapped or deleted while finding collision pairs due to the way the jobs are scheduled in the PhysicsSystem::Update.

	// Sort bodies on layer
	const Tracking *tracking = mTracking.data(); // C pointer or else sort is incredibly slow in debug mode
	QuickSort(ioActiveBodies, ioActiveBodies + inNumActiveBodies, [tracking](BodyID inLHS, BodyID inRHS) { return tracking[inLHS.GetIndex()].mObjectLayer < tracking[inRHS.GetIndex()].mObjectLayer; });

	BodyID *b_start = ioActiveBodies, *b_end = ioActiveBodies + inNumActiveBodies;
	while (b_start < b_end)
	{
		// Get broadphase layer
		ObjectLayer object_layer = tracking[b_start->GetIndex()].mObjectLayer;
		MOSS_ASSERT(object_layer != cObjectLayerInvalid);

		// Find first body with different layer
		BodyID *b_mid = std::upper_bound(b_start, b_end, object_layer, [tracking](ObjectLayer inLayer, BodyID inBodyID) { return inLayer < tracking[inBodyID.GetIndex()].mObjectLayer; });

		// Loop over all layers and test the ones that could hit
		for (BroadPhaseLayer::Type l = 0; l < mNumLayers; ++l)
		{
			const QuadTree &tree = mLayers[l];
			if (tree.HasBodies() && inObjectVsBroadPhaseLayerFilter.ShouldCollide(object_layer, BroadPhaseLayer(l)))
			{
				MOSS_PROFILE(tree.GetName());
				tree.FindCollidingPairs(bodies, b_start, int(b_mid - b_start), inSpeculativeContactDistance, ioPairCollector, inObjectLayerPairFilter);
			}
		}

		// Repeat
		b_start = b_mid;
	}
}

AABox BroadPhaseQuadTree::GetBounds() const
{
	// Prevent this from running in parallel with node deletion in FrameSync(), see notes there
	shared_lock lock(mQueryLocks[mQueryLockIdx]);

	AABox bounds;
	for (BroadPhaseLayer::Type l = 0; l < mNumLayers; ++l)
		bounds.Encapsulate(mLayers[l].GetBounds());
	return bounds;
}

#ifdef MOSS_TRACK_BROADPHASE_STATS

void BroadPhaseQuadTree::ReportStats()
{
	MOSS_TRACE("Query Type, Filter Description, Tree Name, Num Queries, Total Time (%%), Total Time Excl. Collector (%%), Nodes Visited, Bodies Visited, Hits Reported, Hits Reported vs Bodies Visited (%%), Hits Reported vs Nodes Visited");

	uint64 total_ticks = 0;
	for (BroadPhaseLayer::Type l = 0; l < mNumLayers; ++l)
		total_ticks += mLayers[l].GetTicks100Pct();

	for (BroadPhaseLayer::Type l = 0; l < mNumLayers; ++l)
		mLayers[l].ReportStats(total_ticks);
}

#endif // MOSS_TRACK_BROADPHASE_STATS

/*													*/

void BroadPhaseBruteForce::AddBodiesFinalize(BodyID *ioBodies, int inNumber, AddState inAddState)
{
	lock_guard lock(mMutex);

	BodyVector &bodies = mBodyManager->GetBodies();

	// Allocate space
	uint32 idx = (uint32)mBodyIDs.size();
	mBodyIDs.resize(idx + inNumber);

	// Add bodies
	for (const BodyID *b = ioBodies, *b_end = ioBodies + inNumber; b < b_end; ++b)
	{
		Body &body = *bodies[b->GetIndex()];

		// Validate that body ID is consistent with array index
		MOSS_ASSERT(body.GetID() == *b);
		MOSS_ASSERT(!body.IsInBroadPhase());

		// Add it to the list
		mBodyIDs[idx] = body.GetID();
		++idx;

		// Indicate body is in the broadphase
		body.SetInBroadPhaseInternal(true);
	}

	// Resort
	QuickSort(mBodyIDs.begin(), mBodyIDs.end());
}

void BroadPhaseBruteForce::RemoveBodies(BodyID *ioBodies, int inNumber)
{
	lock_guard lock(mMutex);

	BodyVector &bodies = mBodyManager->GetBodies();

	MOSS_ASSERT((int)mBodyIDs.size() >= inNumber);

	// Remove bodies
	for (const BodyID *b = ioBodies, *b_end = ioBodies + inNumber; b < b_end; ++b)
	{
		Body &body = *bodies[b->GetIndex()];

		// Validate that body ID is consistent with array index
		MOSS_ASSERT(body.GetID() == *b);
		MOSS_ASSERT(body.IsInBroadPhase());

		// Find body id
		TArray<BodyID>::const_iterator it = std::lower_bound(mBodyIDs.begin(), mBodyIDs.end(), body.GetID());
		MOSS_ASSERT(it != mBodyIDs.end());

		// Remove element
		mBodyIDs.erase(it);

		// Indicate body is no longer in the broadphase
		body.SetInBroadPhaseInternal(false);
	}
}

void BroadPhaseBruteForce::NotifyBodiesAABBChanged(BodyID *ioBodies, int inNumber, bool inTakeLock)
{
	// Do nothing, we directly reference the body
}

void BroadPhaseBruteForce::NotifyBodiesLayerChanged(BodyID * ioBodies, int inNumber)
{
	// Do nothing, we directly reference the body
}

void BroadPhaseBruteForce::CastRay(const RayCast &inRay, RayCastBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter) const
{
	shared_lock lock(mMutex);

	// Load ray
	Vec3 origin(inRay.mOrigin);
	RayInvDirection inv_direction(inRay.mDirection);

	// For all bodies
	float early_out_fraction = ioCollector.GetEarlyOutFraction();
	for (BodyID b : mBodyIDs)
	{
		const Body &body = mBodyManager->GetBody(b);

		// Test layer
		if (inObjectLayerFilter.ShouldCollide(body.GetObjectLayer()))
		{
			// Test intersection with ray
			const AABox &bounds = body.GetWorldSpaceBounds();
			float fraction = RayAABox(origin, inv_direction, bounds.mMin, bounds.mMax);
			if (fraction < early_out_fraction)
			{
				// Store hit
				BroadPhaseCastResult result { b, fraction };
				ioCollector.AddHit(result);
				if (ioCollector.ShouldEarlyOut())
					break;
				early_out_fraction = ioCollector.GetEarlyOutFraction();
			}
		}
	}
}

void BroadPhaseBruteForce::CollideAABox(const AABox &inBox, CollideShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter) const
{
	shared_lock lock(mMutex);

	// For all bodies
	for (BodyID b : mBodyIDs)
	{
		const Body &body = mBodyManager->GetBody(b);

		// Test layer
		if (inObjectLayerFilter.ShouldCollide(body.GetObjectLayer()))
		{
			// Test intersection with box
			const AABox &bounds = body.GetWorldSpaceBounds();
			if (bounds.Overlaps(inBox))
			{
				// Store hit
				ioCollector.AddHit(b);
				if (ioCollector.ShouldEarlyOut())
					break;
			}
		}
	}
}

void BroadPhaseBruteForce::CollideSphere(Vec3Arg inCenter, float inRadius, CollideShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter) const
{
	shared_lock lock(mMutex);

	float radius_sq = Square(inRadius);

	// For all bodies
	for (BodyID b : mBodyIDs)
	{
		const Body &body = mBodyManager->GetBody(b);

		// Test layer
		if (inObjectLayerFilter.ShouldCollide(body.GetObjectLayer()))
		{
			// Test intersection with box
			const AABox &bounds = body.GetWorldSpaceBounds();
			if (bounds.GetSqDistanceTo(inCenter) <= radius_sq)
			{
				// Store hit
				ioCollector.AddHit(b);
				if (ioCollector.ShouldEarlyOut())
					break;
			}
		}
	}
}

void BroadPhaseBruteForce::CollidePoint(Vec3Arg inPoint, CollideShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter) const
{
	shared_lock lock(mMutex);

	// For all bodies
	for (BodyID b : mBodyIDs)
	{
		const Body &body = mBodyManager->GetBody(b);

		// Test layer
		if (inObjectLayerFilter.ShouldCollide(body.GetObjectLayer()))
		{
			// Test intersection with box
			const AABox &bounds = body.GetWorldSpaceBounds();
			if (bounds.Contains(inPoint))
			{
				// Store hit
				ioCollector.AddHit(b);
				if (ioCollector.ShouldEarlyOut())
					break;
			}
		}
	}
}

void BroadPhaseBruteForce::CollideOrientedBox(const OrientedBox &inBox, CollideShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter) const
{
	shared_lock lock(mMutex);

	// For all bodies
	for (BodyID b : mBodyIDs)
	{
		const Body &body = mBodyManager->GetBody(b);

		// Test layer
		if (inObjectLayerFilter.ShouldCollide(body.GetObjectLayer()))
		{
			// Test intersection with box
			const AABox &bounds = body.GetWorldSpaceBounds();
			if (inBox.Overlaps(bounds))
			{
				// Store hit
				ioCollector.AddHit(b);
				if (ioCollector.ShouldEarlyOut())
					break;
			}
		}
	}
}

void BroadPhaseBruteForce::CastAABoxNoLock(const AABoxCast &inBox, CastShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter) const
{
	shared_lock lock(mMutex);

	// Load box
	Vec3 origin(inBox.mBox.GetCenter());
	Vec3 extent(inBox.mBox.GetExtent());
	RayInvDirection inv_direction(inBox.mDirection);

	// For all bodies
	float early_out_fraction = ioCollector.GetPositiveEarlyOutFraction();
	for (BodyID b : mBodyIDs)
	{
		const Body &body = mBodyManager->GetBody(b);

		// Test layer
		if (inObjectLayerFilter.ShouldCollide(body.GetObjectLayer()))
		{
			// Test intersection with ray
			const AABox &bounds = body.GetWorldSpaceBounds();
			float fraction = RayAABox(origin, inv_direction, bounds.mMin - extent, bounds.mMax + extent);
			if (fraction < early_out_fraction)
			{
				// Store hit
				BroadPhaseCastResult result { b, fraction };
				ioCollector.AddHit(result);
				if (ioCollector.ShouldEarlyOut())
					break;
				early_out_fraction = ioCollector.GetPositiveEarlyOutFraction();
			}
		}
	}
}

void BroadPhaseBruteForce::CastAABox(const AABoxCast &inBox, CastShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter) const
{
	CastAABoxNoLock(inBox, ioCollector, inBroadPhaseLayerFilter, inObjectLayerFilter);
}

void BroadPhaseBruteForce::FindCollidingPairs(BodyID *ioActiveBodies, int inNumActiveBodies, float inSpeculativeContactDistance, const ObjectVsBroadPhaseLayerFilter &inObjectVsBroadPhaseLayerFilter, const ObjectLayerPairFilter &inObjectLayerPairFilter, BodyPairCollector &ioPairCollector) const
{
	shared_lock lock(mMutex);

	// Loop through all active bodies
	size_t num_bodies = mBodyIDs.size();
	for (int b1 = 0; b1 < inNumActiveBodies; ++b1)
	{
		BodyID b1_id = ioActiveBodies[b1];
		const Body &body1 = mBodyManager->GetBody(b1_id);
		const ObjectLayer layer1 = body1.GetObjectLayer();

		// Expand the bounding box by the speculative contact distance
		AABox bounds1 = body1.GetWorldSpaceBounds();
		bounds1.ExpandBy(Vec3::sReplicate(inSpeculativeContactDistance));

		// For all other bodies
		for (size_t b2 = 0; b2 < num_bodies; ++b2)
		{
			// Check if bodies can collide
			BodyID b2_id = mBodyIDs[b2];
			const Body &body2 = mBodyManager->GetBody(b2_id);
			if (!Body::sFindCollidingPairsCanCollide(body1, body2))
				continue;

			// Check if layers can collide
			const ObjectLayer layer2 = body2.GetObjectLayer();
			if (!inObjectLayerPairFilter.ShouldCollide(layer1, layer2))
				continue;

			// Check if bounds overlap
			const AABox &bounds2 = body2.GetWorldSpaceBounds();
			if (!bounds1.Overlaps(bounds2))
				continue;

			// Store overlapping pair
			ioPairCollector.AddHit({ b1_id, b2_id });
		}
	}
}

AABox BroadPhaseBruteForce::GetBounds() const
{
	shared_lock lock(mMutex);

	AABox bounds;
	for (BodyID b : mBodyIDs)
		bounds.Encapsulate(mBodyManager->GetBody(b).GetWorldSpaceBounds());
	return bounds;
}

/*													*/

void BroadPhase::Init(BodyManager *inBodyManager, const BroadPhaseLayerInterface &inLayerInterface)
{
	mBodyManager = inBodyManager;
}

/*													*/




////////////////////////////////////////////////////////////////////////////////////////////////////////
// QuadTree::Node
////////////////////////////////////////////////////////////////////////////////////////////////////////

QuadTree::Node::Node(bool inIsChanged) :
	mIsChanged(inIsChanged)
{
	// First reset bounds
	Vec4 val = Vec4::sReplicate(cLargeFloat);
	val.StoreFloat4((Float4 *)&mBoundsMinX);
	val.StoreFloat4((Float4 *)&mBoundsMinY);
	val.StoreFloat4((Float4 *)&mBoundsMinZ);
	val = Vec4::sReplicate(-cLargeFloat);
	val.StoreFloat4((Float4 *)&mBoundsMaxX);
	val.StoreFloat4((Float4 *)&mBoundsMaxY);
	val.StoreFloat4((Float4 *)&mBoundsMaxZ);

	// Reset child node ids
	mChildNodeID[0] = NodeID::sInvalid();
	mChildNodeID[1] = NodeID::sInvalid();
	mChildNodeID[2] = NodeID::sInvalid();
	mChildNodeID[3] = NodeID::sInvalid();
}

void QuadTree::Node::GetChildBounds(int inChildIndex, AABox &outBounds) const
{
	// Read bounding box in order min -> max
	outBounds.mMin = Vec3(mBoundsMinX[inChildIndex], mBoundsMinY[inChildIndex], mBoundsMinZ[inChildIndex]);
	outBounds.mMax = Vec3(mBoundsMaxX[inChildIndex], mBoundsMaxY[inChildIndex], mBoundsMaxZ[inChildIndex]);
}

void QuadTree::Node::SetChildBounds(int inChildIndex, const AABox &inBounds)
{
	// Bounding boxes provided to the quad tree should never be larger than cLargeFloat because this may trigger overflow exceptions
	// e.g. when squaring the value while testing sphere overlaps
	MOSS_ASSERT(inBounds.mMin.GetX() >= -cLargeFloat && inBounds.mMin.GetX() <= cLargeFloat
			   && inBounds.mMin.GetY() >= -cLargeFloat && inBounds.mMin.GetY() <= cLargeFloat
			   && inBounds.mMin.GetZ() >= -cLargeFloat && inBounds.mMin.GetZ() <= cLargeFloat
			   && inBounds.mMax.GetX() >= -cLargeFloat && inBounds.mMax.GetX() <= cLargeFloat
			   && inBounds.mMax.GetY() >= -cLargeFloat && inBounds.mMax.GetY() <= cLargeFloat
			   && inBounds.mMax.GetZ() >= -cLargeFloat && inBounds.mMax.GetZ() <= cLargeFloat);

	// Set max first (this keeps the bounding box invalid for reading threads)
	mBoundsMaxZ[inChildIndex] = inBounds.mMax.GetZ();
	mBoundsMaxY[inChildIndex] = inBounds.mMax.GetY();
	mBoundsMaxX[inChildIndex] = inBounds.mMax.GetX();

	// Then set min (and make box valid)
	mBoundsMinZ[inChildIndex] = inBounds.mMin.GetZ();
	mBoundsMinY[inChildIndex] = inBounds.mMin.GetY();
	mBoundsMinX[inChildIndex] = inBounds.mMin.GetX(); // Min X becomes valid last
}

void QuadTree::Node::InvalidateChildBounds(int inChildIndex)
{
	// First we make the box invalid by setting the min to cLargeFloat
	mBoundsMinX[inChildIndex] = cLargeFloat; // Min X becomes invalid first
	mBoundsMinY[inChildIndex] = cLargeFloat;
	mBoundsMinZ[inChildIndex] = cLargeFloat;

	// Then we reset the max values too
	mBoundsMaxX[inChildIndex] = -cLargeFloat;
	mBoundsMaxY[inChildIndex] = -cLargeFloat;
	mBoundsMaxZ[inChildIndex] = -cLargeFloat;
}

void QuadTree::Node::GetNodeBounds(AABox &outBounds) const
{
	// Get first child bounds
	GetChildBounds(0, outBounds);

	// Encapsulate other child bounds
	for (int child_idx = 1; child_idx < 4; ++child_idx)
	{
		AABox tmp;
		GetChildBounds(child_idx, tmp);
		outBounds.Encapsulate(tmp);
	}
}

bool QuadTree::Node::EncapsulateChildBounds(int inChildIndex, const AABox &inBounds)
{
	bool changed = AtomicMin(mBoundsMinX[inChildIndex], inBounds.mMin.GetX());
	changed |= AtomicMin(mBoundsMinY[inChildIndex], inBounds.mMin.GetY());
	changed |= AtomicMin(mBoundsMinZ[inChildIndex], inBounds.mMin.GetZ());
	changed |= AtomicMax(mBoundsMaxX[inChildIndex], inBounds.mMax.GetX());
	changed |= AtomicMax(mBoundsMaxY[inChildIndex], inBounds.mMax.GetY());
	changed |= AtomicMax(mBoundsMaxZ[inChildIndex], inBounds.mMax.GetZ());
	return changed;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// QuadTree
////////////////////////////////////////////////////////////////////////////////////////////////////////

const AABox QuadTree::cInvalidBounds(Vec3::sReplicate(cLargeFloat), Vec3::sReplicate(-cLargeFloat));

void QuadTree::GetBodyLocation(const TrackingVector &inTracking, BodyID inBodyID, uint32 &outNodeIdx, uint32 &outChildIdx) const
{
	uint32 body_location = inTracking[inBodyID.GetIndex()].mBodyLocation;
	MOSS_ASSERT(body_location != Tracking::cInvalidBodyLocation);
	outNodeIdx = body_location & 0x3fffffff;
	outChildIdx = body_location >> 30;
	MOSS_ASSERT(mAllocator->Get(outNodeIdx).mChildNodeID[outChildIdx] == inBodyID, "Make sure that the body is in the node where it should be");
}

void QuadTree::SetBodyLocation(TrackingVector &ioTracking, BodyID inBodyID, uint32 inNodeIdx, uint32 inChildIdx) const
{
	MOSS_ASSERT(inNodeIdx <= 0x3fffffff);
	MOSS_ASSERT(inChildIdx < 4);
	MOSS_ASSERT(mAllocator->Get(inNodeIdx).mChildNodeID[inChildIdx] == inBodyID, "Make sure that the body is in the node where it should be");
	ioTracking[inBodyID.GetIndex()].mBodyLocation = inNodeIdx + (inChildIdx << 30);

#ifdef MOSS_DEBUG
	uint32 v1, v2;
	GetBodyLocation(ioTracking, inBodyID, v1, v2);
	MOSS_ASSERT(v1 == inNodeIdx);
	MOSS_ASSERT(v2 == inChildIdx);
#endif
}

void QuadTree::sInvalidateBodyLocation(TrackingVector &ioTracking, BodyID inBodyID)
{
	ioTracking[inBodyID.GetIndex()].mBodyLocation = Tracking::cInvalidBodyLocation;
}

QuadTree::~QuadTree()
{
	// Get rid of any nodes that are still to be freed
	DiscardOldTree();

	// Get the current root node
	const RootNode &root_node = GetCurrentRoot();

	// Collect all bodies
	Allocator::Batch free_batch;
	TArray<NodeID, STLLocalAllocator<NodeID, cStackSize>> node_stack;
	node_stack.reserve(cStackSize);
	node_stack.push_back(root_node.GetNodeID());
	MOSS_ASSERT(node_stack.front().IsValid());
	if (node_stack.front().IsNode())
	{
		do
		{
			// Process node
			NodeID node_id = node_stack.back();
			node_stack.pop_back();
			MOSS_ASSERT(!node_id.IsBody());
			uint32 node_idx = node_id.GetNodeIndex();
			const Node &node = mAllocator->Get(node_idx);

			// Recurse and get all child nodes
			for (NodeID child_node_id : node.mChildNodeID)
				if (child_node_id.IsValid() && child_node_id.IsNode())
					node_stack.push_back(child_node_id);

			// Mark node to be freed
			mAllocator->AddObjectToBatch(free_batch, node_idx);
		}
		while (!node_stack.empty());
	}

	// Now free all nodes
	mAllocator->DestructObjectBatch(free_batch);
}

uint32 QuadTree::AllocateNode(bool inIsChanged)
{
	uint32 index = mAllocator->ConstructObject(inIsChanged);
	if (index == Allocator::cInvalidObjectIndex)
	{
		// If you're running out of nodes, you're most likely adding too many individual bodies to the tree.
		// Because of the lock free nature of this tree, any individual body is added to the root of the tree.
		// This means that if you add a lot of bodies individually, you will end up with a very deep tree and you'll be
		// using a lot more nodes than you would if you added them in batches.
		// Please look at BodyInterface::AddBodiesPrepare/AddBodiesFinalize.
		//
		// If you have created a wrapper around Jolt then a possible solution is to activate a mode during loading
		// that queues up any bodies that need to be added. When loading is done, insert all of them as a single batch.
		// This could be implemented as a 'start batching' / 'end batching' call to switch in and out of that mode.
		// The rest of the code can then just use the regular 'add single body' call on your wrapper and doesn't need to know
		// if this mode is active or not.
		//
		// Calling PhysicsSystem::Update or PhysicsSystem::OptimizeBroadPhase will perform maintenance
		// on the tree and will make it efficient again. If you're not calling these functions and are adding a lot of bodies
		// you could still be running out of nodes because the tree is not being maintained. If your application is paused,
		// consider still calling PhysicsSystem::Update with a delta time of 0 to keep the tree in good shape.
		//
		// The system keeps track of a previous and a current tree, this allows for queries to continue using the old tree
		// while the new tree is being built. If you completely clean the PhysicsSystem and rebuild it from scratch, you may
		// want to call PhysicsSystem::OptimizeBroadPhase two times after clearing to completely get rid of any lingering nodes.
		//
		// The number of nodes that is allocated is related to the max number of bodies that is passed in PhysicsSystem::Init.
		// For normal situations there are plenty of nodes available. If all else fails, you can increase the number of nodes
		// by increasing the maximum number of bodies.
		MOSS_TRACE("QuadTree: Out of nodes!");
		std::abort();
	}
	return index;
}

void QuadTree::Init(Allocator &inAllocator)
{
	// Store allocator
	mAllocator = &inAllocator;

	// Allocate root node
	mRootNode[mRootNodeIndex].mIndex = AllocateNode(false);
}

void QuadTree::DiscardOldTree()
{
	// Check if there is an old tree
	RootNode &old_root_node = mRootNode[mRootNodeIndex ^ 1];
	if (old_root_node.mIndex != cInvalidNodeIndex)
	{
		// Clear the root
		old_root_node.mIndex = cInvalidNodeIndex;

		// Now free all old nodes
		mAllocator->DestructObjectBatch(mFreeNodeBatch);

		// Clear the batch
		mFreeNodeBatch = Allocator::Batch();
	}
}

AABox QuadTree::GetBounds() const
{
	uint32 node_idx = GetCurrentRoot().mIndex;
	MOSS_ASSERT(node_idx != cInvalidNodeIndex);
	const Node &node = mAllocator->Get(node_idx);

	AABox bounds;
	node.GetNodeBounds(bounds);
	return bounds;
}

void QuadTree::UpdatePrepare(const BodyVector &inBodies, TrackingVector &ioTracking, UpdateState &outUpdateState, bool inFullRebuild)
{
#ifdef MOSS_DEBUG
	// We only read positions
	BodyAccess::Grant grant(BodyAccess::EAccess::None, BodyAccess::EAccess::Read);
#endif

	// Assert we have no nodes pending deletion, this means DiscardOldTree wasn't called yet
	MOSS_ASSERT(mFreeNodeBatch.mNumObjects == 0);

	// Mark tree non-dirty
	mIsDirty = false;

	// Get the current root node
	const RootNode &root_node = GetCurrentRoot();

#ifdef MOSS_DUMP_BROADPHASE_TREE
	DumpTree(root_node.GetNodeID(), StringFormat("%s_PRE", mName).c_str());
#endif

	// Assert sane data
#ifdef MOSS_DEBUG
	ValidateTree(inBodies, ioTracking, root_node.mIndex, mNumBodies);
#endif

	// Create space for all body ID's
	NodeID *node_ids = new NodeID [mNumBodies];
	NodeID *cur_node_id = node_ids;

	// Collect all bodies
	NodeID node_stack[cStackSize];
	node_stack[0] = root_node.GetNodeID();
	MOSS_ASSERT(node_stack[0].IsValid());
	int top = 0;
	do
	{
		// Check if node is a body
		NodeID node_id = node_stack[top];
		if (node_id.IsBody())
		{
			// Validate that we're still in the right layer
		#ifdef MOSS_DEBUG
			uint32 body_index = node_id.GetBodyID().GetIndex();
			MOSS_ASSERT(ioTracking[body_index].mObjectLayer == inBodies[body_index]->GetObjectLayer());
		#endif

			// Store body
			*cur_node_id = node_id;
			++cur_node_id;
		}
		else
		{
			// Process normal node
			uint32 node_idx = node_id.GetNodeIndex();
			const Node &node = mAllocator->Get(node_idx);

			if (!node.mIsChanged && !inFullRebuild)
			{
				// Node is unchanged, treat it as a whole
				*cur_node_id = node_id;
				++cur_node_id;
			}
			else
			{
				// Node is changed, recurse and get all children
				for (NodeID child_node_id : node.mChildNodeID)
					if (child_node_id.IsValid())
					{
						if (top < cStackSize)
						{
							node_stack[top] = child_node_id;
							top++;
						}
						else
						{
							MOSS_ASSERT(false, "Stack full!\n"
								"This must be a very deep tree. Are you batch adding bodies through BodyInterface::AddBodiesPrepare/AddBodiesFinalize?\n"
								"If you add lots of bodies through BodyInterface::AddBody you may need to call PhysicsSystem::OptimizeBroadPhase to rebuild the tree.");

							// Falling back to adding the node as a whole
							*cur_node_id = child_node_id;
							++cur_node_id;
						}
					}

				// Mark node to be freed
				mAllocator->AddObjectToBatch(mFreeNodeBatch, node_idx);
			}
		}
		--top;
	}
	while (top >= 0);

	// Check that our book keeping matches
	uint32 num_node_ids = uint32(cur_node_id - node_ids);
	MOSS_ASSERT(inFullRebuild? num_node_ids == mNumBodies : num_node_ids <= mNumBodies);

	// This will be the new root node id
	NodeID root_node_id;

	if (num_node_ids > 0)
	{
		// We mark the first 5 levels (max 1024 nodes) of the newly built tree as 'changed' so that
		// those nodes get recreated every time when we rebuild the tree. This balances the amount of
		// time we spend on rebuilding the tree ('unchanged' nodes will be put in the new tree as a whole)
		// vs the quality of the built tree.
		constexpr uint cMaxDepthMarkChanged = 5;

		// Build new tree
		AABox root_bounds;
		root_node_id = BuildTree(inBodies, ioTracking, node_ids, num_node_ids, cMaxDepthMarkChanged, root_bounds);

		if (root_node_id.IsBody())
		{
			// For a single body we need to allocate a new root node
			uint32 root_idx = AllocateNode(false);
			Node &root = mAllocator->Get(root_idx);
			root.SetChildBounds(0, root_bounds);
			root.mChildNodeID[0] = root_node_id;
			SetBodyLocation(ioTracking, root_node_id.GetBodyID(), root_idx, 0);
			root_node_id = NodeID::sFromNodeIndex(root_idx);
		}
	}
	else
	{
		// Empty tree, create root node
		uint32 root_idx = AllocateNode(false);
		root_node_id = NodeID::sFromNodeIndex(root_idx);
	}

	// Delete temporary data
	delete [] node_ids;

	outUpdateState.mRootNodeID = root_node_id;
}

void QuadTree::UpdateFinalize([[maybe_unused]] const BodyVector &inBodies, [[maybe_unused]] const TrackingVector &inTracking, const UpdateState &inUpdateState)
{
	// Tree building is complete, now we switch the old with the new tree
	uint32 new_root_idx = mRootNodeIndex ^ 1;
	RootNode &new_root_node = mRootNode[new_root_idx];
	{
		// Note: We don't need to lock here as the old tree stays available so any queries
		// that use it can continue using it until DiscardOldTree is called. This slot
		// should be empty and unused at this moment.
		MOSS_ASSERT(new_root_node.mIndex == cInvalidNodeIndex);
		new_root_node.mIndex = inUpdateState.mRootNodeID.GetNodeIndex();
	}

	// All queries that start from now on will use this new tree
	mRootNodeIndex = new_root_idx;

#ifdef MOSS_DUMP_BROADPHASE_TREE
	DumpTree(new_root_node.GetNodeID(), StringFormat("%s_POST", mName).c_str());
#endif

#ifdef MOSS_DEBUG
	ValidateTree(inBodies, inTracking, new_root_node.mIndex, mNumBodies);
#endif
}

void QuadTree::sPartition(NodeID *ioNodeIDs, Vec3 *ioNodeCenters, int inNumber, int &outMidPoint)
{
	// Handle trivial case
	if (inNumber <= 4)
	{
		outMidPoint = inNumber / 2;
		return;
	}

	// Calculate bounding box of box centers
	Vec3 center_min = Vec3::sReplicate(cLargeFloat);
	Vec3 center_max = Vec3::sReplicate(-cLargeFloat);
	for (const Vec3 *c = ioNodeCenters, *c_end = ioNodeCenters + inNumber; c < c_end; ++c)
	{
		Vec3 center = *c;
		center_min = Vec3::sMin(center_min, center);
		center_max = Vec3::sMax(center_max, center);
	}

	// Calculate split plane
	int dimension = (center_max - center_min).GetHighestComponentIndex();
	float split = 0.5f * (center_min + center_max)[dimension];

	// Divide bodies
	int start = 0, end = inNumber;
	while (start < end)
	{
		// Search for first element that is on the right hand side of the split plane
		while (start < end && ioNodeCenters[start][dimension] < split)
			++start;

		// Search for the first element that is on the left hand side of the split plane
		while (start < end && ioNodeCenters[end - 1][dimension] >= split)
			--end;

		if (start < end)
		{
			// Swap the two elements
			std::swap(ioNodeIDs[start], ioNodeIDs[end - 1]);
			std::swap(ioNodeCenters[start], ioNodeCenters[end - 1]);
			++start;
			--end;
		}
	}
	MOSS_ASSERT(start == end);

	if (start > 0 && start < inNumber)
	{
		// Success!
		outMidPoint = start;
	}
	else
	{
		// Failed to divide bodies
		outMidPoint = inNumber / 2;
	}
}

void QuadTree::sPartition4(NodeID *ioNodeIDs, Vec3 *ioNodeCenters, int inBegin, int inEnd, int *outSplit)
{
	NodeID *node_ids = ioNodeIDs + inBegin;
	Vec3 *node_centers = ioNodeCenters + inBegin;
	int number = inEnd - inBegin;

	// Partition entire range
	sPartition(node_ids, node_centers, number, outSplit[2]);

	// Partition lower half
	sPartition(node_ids, node_centers, outSplit[2], outSplit[1]);

	// Partition upper half
	sPartition(node_ids + outSplit[2], node_centers + outSplit[2], number - outSplit[2], outSplit[3]);

	// Convert to proper range
	outSplit[0] = inBegin;
	outSplit[1] += inBegin;
	outSplit[2] += inBegin;
	outSplit[3] += outSplit[2];
	outSplit[4] = inEnd;
}

AABox QuadTree::GetNodeOrBodyBounds(const BodyVector &inBodies, NodeID inNodeID) const
{
	if (inNodeID.IsNode())
	{
		// It is a node
		uint32 node_idx = inNodeID.GetNodeIndex();
		const Node &node = mAllocator->Get(node_idx);

		AABox bounds;
		node.GetNodeBounds(bounds);
		return bounds;
	}
	else
	{
		// It is a body
		return inBodies[inNodeID.GetBodyID().GetIndex()]->GetWorldSpaceBounds();
	}
}

QuadTree::NodeID QuadTree::BuildTree(const BodyVector &inBodies, TrackingVector &ioTracking, NodeID *ioNodeIDs, int inNumber, uint inMaxDepthMarkChanged, AABox &outBounds)
{
	// Trivial case: No bodies in tree
	if (inNumber == 0)
	{
		outBounds = cInvalidBounds;
		return NodeID::sInvalid();
	}

	// Trivial case: When we have 1 body or node, return it
	if (inNumber == 1)
	{
		if (ioNodeIDs->IsNode())
		{
			// When returning an existing node as root, ensure that no parent has been set
			Node &node = mAllocator->Get(ioNodeIDs->GetNodeIndex());
			node.mParentNodeIndex = cInvalidNodeIndex;
		}
		outBounds = GetNodeOrBodyBounds(inBodies, *ioNodeIDs);
		return *ioNodeIDs;
	}

	// Calculate centers of all bodies that are to be inserted
	Vec3 *centers = new Vec3 [inNumber];
	MOSS_ASSERT(IsAligned(centers, MOSS_VECTOR_ALIGNMENT));
	Vec3 *c = centers;
	for (const NodeID *n = ioNodeIDs, *n_end = ioNodeIDs + inNumber; n < n_end; ++n, ++c)
		*c = GetNodeOrBodyBounds(inBodies, *n).GetCenter();

	// The algorithm is a recursive tree build, but to avoid the call overhead we keep track of a stack here
	struct StackEntry
	{
		uint32			mNodeIdx;					// Node index of node that is generated
		int				mChildIdx;					// Index of child that we're currently processing
		int				mSplit[5];					// Indices where the node ID's have been split to form 4 partitions
		uint32			mDepth;						// Depth of this node in the tree
		Vec3			mNodeBoundsMin;				// Bounding box of this node, accumulated while iterating over children
		Vec3			mNodeBoundsMax;
	};
	static_assert(sizeof(StackEntry) == 64);
	StackEntry stack[cStackSize / 4]; // We don't process 4 at a time in this loop but 1, so the stack can be 4x as small
	int top = 0;

	// Create root node
	stack[0].mNodeIdx = AllocateNode(inMaxDepthMarkChanged > 0);
	stack[0].mChildIdx = -1;
	stack[0].mDepth = 0;
	stack[0].mNodeBoundsMin = Vec3::sReplicate(cLargeFloat);
	stack[0].mNodeBoundsMax = Vec3::sReplicate(-cLargeFloat);
	sPartition4(ioNodeIDs, centers, 0, inNumber, stack[0].mSplit);

	for (;;)
	{
		StackEntry &cur_stack = stack[top];

		// Next child
		cur_stack.mChildIdx++;

		// Check if all children processed
		if (cur_stack.mChildIdx >= 4)
		{
			// Terminate if there's nothing left to pop
			if (top <= 0)
				break;

			// Add our bounds to our parents bounds
			StackEntry &prev_stack = stack[top - 1];
			prev_stack.mNodeBoundsMin = Vec3::sMin(prev_stack.mNodeBoundsMin, cur_stack.mNodeBoundsMin);
			prev_stack.mNodeBoundsMax = Vec3::sMax(prev_stack.mNodeBoundsMax, cur_stack.mNodeBoundsMax);

			// Store parent node
			Node &node = mAllocator->Get(cur_stack.mNodeIdx);
			node.mParentNodeIndex = prev_stack.mNodeIdx;

			// Store this node's properties in the parent node
			Node &parent_node = mAllocator->Get(prev_stack.mNodeIdx);
			parent_node.mChildNodeID[prev_stack.mChildIdx] = NodeID::sFromNodeIndex(cur_stack.mNodeIdx);
			parent_node.SetChildBounds(prev_stack.mChildIdx, AABox(cur_stack.mNodeBoundsMin, cur_stack.mNodeBoundsMax));

			// Pop entry from stack
			--top;
		}
		else
		{
			// Get low and high index to bodies to process
			int low = cur_stack.mSplit[cur_stack.mChildIdx];
			int high = cur_stack.mSplit[cur_stack.mChildIdx + 1];
			int num_bodies = high - low;

			if (num_bodies == 1)
			{
				// Get body info
				NodeID child_node_id = ioNodeIDs[low];
				AABox bounds = GetNodeOrBodyBounds(inBodies, child_node_id);

				// Update node
				Node &node = mAllocator->Get(cur_stack.mNodeIdx);
				node.mChildNodeID[cur_stack.mChildIdx] = child_node_id;
				node.SetChildBounds(cur_stack.mChildIdx, bounds);

				if (child_node_id.IsNode())
				{
					// Update parent for this node
					Node &child_node = mAllocator->Get(child_node_id.GetNodeIndex());
					child_node.mParentNodeIndex = cur_stack.mNodeIdx;
				}
				else
				{
					// Set location in tracking
					SetBodyLocation(ioTracking, child_node_id.GetBodyID(), cur_stack.mNodeIdx, cur_stack.mChildIdx);
				}

				// Encapsulate bounding box in parent
				cur_stack.mNodeBoundsMin = Vec3::sMin(cur_stack.mNodeBoundsMin, bounds.mMin);
				cur_stack.mNodeBoundsMax = Vec3::sMax(cur_stack.mNodeBoundsMax, bounds.mMax);
			}
			else if (num_bodies > 1)
			{
				// Allocate new node
				StackEntry &new_stack = stack[++top];
				MOSS_ASSERT(top < cStackSize / 4);
				uint32 next_depth = cur_stack.mDepth + 1;
				new_stack.mNodeIdx = AllocateNode(inMaxDepthMarkChanged > next_depth);
				new_stack.mChildIdx = -1;
				new_stack.mDepth = next_depth;
				new_stack.mNodeBoundsMin = Vec3::sReplicate(cLargeFloat);
				new_stack.mNodeBoundsMax = Vec3::sReplicate(-cLargeFloat);
				sPartition4(ioNodeIDs, centers, low, high, new_stack.mSplit);
			}
		}
	}

	// Delete temporary data
	delete [] centers;

	// Store bounding box of root
	outBounds.mMin = stack[0].mNodeBoundsMin;
	outBounds.mMax = stack[0].mNodeBoundsMax;

	// Return root
	return NodeID::sFromNodeIndex(stack[0].mNodeIdx);
}

void QuadTree::MarkNodeAndParentsChanged(uint32 inNodeIndex)
{
	uint32 node_idx = inNodeIndex;

	do
	{
		// If node has changed, parent will be too
		Node &node = mAllocator->Get(node_idx);
		if (node.mIsChanged)
			break;

		// Mark node as changed
		node.mIsChanged = true;

		// Get our parent
		node_idx = node.mParentNodeIndex;
	}
	while (node_idx != cInvalidNodeIndex);
}

void QuadTree::WidenAndMarkNodeAndParentsChanged(uint32 inNodeIndex, const AABox &inNewBounds)
{
	uint32 node_idx = inNodeIndex;

	for (;;)
	{
		// Mark node as changed
		Node &node = mAllocator->Get(node_idx);
		node.mIsChanged = true;

		// Get our parent
		uint32 parent_idx = node.mParentNodeIndex;
		if (parent_idx == cInvalidNodeIndex)
			break;

		// Find which child of the parent we're in
		Node &parent_node = mAllocator->Get(parent_idx);
		NodeID node_id = NodeID::sFromNodeIndex(node_idx);
		int child_idx = -1;
		for (int i = 0; i < 4; ++i)
			if (parent_node.mChildNodeID[i] == node_id)
			{
				// Found one, set the node index and child index and update the bounding box too
				child_idx = i;
				break;
			}
		MOSS_ASSERT(child_idx != -1, "Nodes don't get removed from the tree, we must have found it");

		// To avoid any race conditions with other threads we only enlarge bounding boxes
		if (!parent_node.EncapsulateChildBounds(child_idx, inNewBounds))
		{
			// No changes to bounding box, only marking as changed remains to be done
			if (!parent_node.mIsChanged)
				MarkNodeAndParentsChanged(parent_idx);
			break;
		}

		// Update node index
		node_idx = parent_idx;
	}
}

bool QuadTree::TryInsertLeaf(TrackingVector &ioTracking, int inNodeIndex, NodeID inLeafID, const AABox &inLeafBounds, int inLeafNumBodies)
{
	// Tentatively assign the node as parent
	bool leaf_is_node = inLeafID.IsNode();
	if (leaf_is_node)
	{
		uint32 leaf_idx = inLeafID.GetNodeIndex();
		mAllocator->Get(leaf_idx).mParentNodeIndex = inNodeIndex;
	}

	// Fetch node that we're adding to
	Node &node = mAllocator->Get(inNodeIndex);

	// Find an empty child
	for (uint32 child_idx = 0; child_idx < 4; ++child_idx)
		if (node.mChildNodeID[child_idx].CompareExchange(NodeID::sInvalid(), inLeafID)) // Check if we can claim it
		{
			// We managed to add it to the node

			// If leaf was a body, we need to update its bookkeeping
			if (!leaf_is_node)
				SetBodyLocation(ioTracking, inLeafID.GetBodyID(), inNodeIndex, child_idx);

			// Now set the bounding box making the child valid for queries
			node.SetChildBounds(child_idx, inLeafBounds);

			// Widen the bounds for our parents too
			WidenAndMarkNodeAndParentsChanged(inNodeIndex, inLeafBounds);

			// Update body counter
			mNumBodies += inLeafNumBodies;

			// And we're done
			return true;
		}

	return false;
}

bool QuadTree::TryCreateNewRoot(TrackingVector &ioTracking, atomic<uint32> &ioRootNodeIndex, NodeID inLeafID, const AABox &inLeafBounds, int inLeafNumBodies)
{
	// Fetch old root
	uint32 root_idx = ioRootNodeIndex;
	Node &root = mAllocator->Get(root_idx);

	// Create new root, mark this new root as changed as we're not creating a very efficient tree at this point
	uint32 new_root_idx = AllocateNode(true);
	Node &new_root = mAllocator->Get(new_root_idx);

	// First child is current root, note that since the tree may be modified concurrently we cannot assume that the bounds of our child will be correct so we set a very large bounding box
	new_root.mChildNodeID[0] = NodeID::sFromNodeIndex(root_idx);
	new_root.SetChildBounds(0, AABox(Vec3::sReplicate(-cLargeFloat), Vec3::sReplicate(cLargeFloat)));

	// Second child is new leaf
	new_root.mChildNodeID[1] = inLeafID;
	new_root.SetChildBounds(1, inLeafBounds);

	// Tentatively assign new root as parent
	bool leaf_is_node = inLeafID.IsNode();
	if (leaf_is_node)
	{
		uint32 leaf_idx = inLeafID.GetNodeIndex();
		mAllocator->Get(leaf_idx).mParentNodeIndex = new_root_idx;
	}

	// Try to swap it
	if (ioRootNodeIndex.compare_exchange_strong(root_idx, new_root_idx))
	{
		// We managed to set the new root

		// If leaf was a body, we need to update its bookkeeping
		if (!leaf_is_node)
			SetBodyLocation(ioTracking, inLeafID.GetBodyID(), new_root_idx, 1);

		// Store parent node for old root
		root.mParentNodeIndex = new_root_idx;

		// Update body counter
		mNumBodies += inLeafNumBodies;

		// And we're done
		return true;
	}

	// Failed to swap, someone else must have created a new root, try again
	mAllocator->DestructObject(new_root_idx);
	return false;
}

void QuadTree::AddBodiesPrepare(const BodyVector &inBodies, TrackingVector &ioTracking, BodyID *ioBodyIDs, int inNumber, AddState &outState)
{
	// Assert sane input
	MOSS_ASSERT(ioBodyIDs != nullptr);
	MOSS_ASSERT(inNumber > 0);

#ifdef MOSS_DEBUG
	// Below we just cast the body ID's to node ID's, check here that that is valid
	for (const BodyID *b = ioBodyIDs, *b_end = ioBodyIDs + inNumber; b < b_end; ++b)
		NodeID::sFromBodyID(*b);
#endif

	// Build subtree for the new bodies, note that we mark all nodes as 'not changed'
	// so they will stay together as a batch and will make the tree rebuild cheaper
	outState.mLeafID = BuildTree(inBodies, ioTracking, (NodeID *)ioBodyIDs, inNumber, 0, outState.mLeafBounds);

#ifdef MOSS_DEBUG
	if (outState.mLeafID.IsNode())
		ValidateTree(inBodies, ioTracking, outState.mLeafID.GetNodeIndex(), inNumber);
#endif
}

void QuadTree::AddBodiesFinalize(TrackingVector &ioTracking, int inNumberBodies, const AddState &inState)
{
	// Assert sane input
	MOSS_ASSERT(inNumberBodies > 0);

	// Mark tree dirty
	mIsDirty = true;

	// Get the current root node
	RootNode &root_node = GetCurrentRoot();

	for (;;)
	{
		// Check if we can insert the body in the root
		if (TryInsertLeaf(ioTracking, root_node.mIndex, inState.mLeafID, inState.mLeafBounds, inNumberBodies))
			return;

		// Check if we can create a new root
		if (TryCreateNewRoot(ioTracking, root_node.mIndex, inState.mLeafID, inState.mLeafBounds, inNumberBodies))
			return;
	}
}

void QuadTree::AddBodiesAbort(TrackingVector &ioTracking, const AddState &inState)
{
	// Collect all bodies
	Allocator::Batch free_batch;
	NodeID node_stack[cStackSize];
	node_stack[0] = inState.mLeafID;
	MOSS_ASSERT(node_stack[0].IsValid());
	int top = 0;
	do
	{
		// Check if node is a body
		NodeID child_node_id = node_stack[top];
		if (child_node_id.IsBody())
		{
			// Reset location of body
			sInvalidateBodyLocation(ioTracking, child_node_id.GetBodyID());
		}
		else
		{
			// Process normal node
			uint32 node_idx = child_node_id.GetNodeIndex();
			const Node &node = mAllocator->Get(node_idx);
			for (NodeID sub_child_node_id : node.mChildNodeID)
				if (sub_child_node_id.IsValid())
				{
					MOSS_ASSERT(top < cStackSize);
					node_stack[top] = sub_child_node_id;
					top++;
				}

			// Mark it to be freed
			mAllocator->AddObjectToBatch(free_batch, node_idx);
		}
		--top;
	}
	while (top >= 0);

	// Now free all nodes as a single batch
	mAllocator->DestructObjectBatch(free_batch);
}

void QuadTree::RemoveBodies([[maybe_unused]] const BodyVector &inBodies, TrackingVector &ioTracking, const BodyID *ioBodyIDs, int inNumber)
{
	// Assert sane input
	MOSS_ASSERT(ioBodyIDs != nullptr);
	MOSS_ASSERT(inNumber > 0);

	// Mark tree dirty
	mIsDirty = true;

	for (const BodyID *cur = ioBodyIDs, *end = ioBodyIDs + inNumber; cur < end; ++cur)
	{
		// Check if BodyID is correct
		MOSS_ASSERT(inBodies[cur->GetIndex()]->GetID() == *cur, "Provided BodyID doesn't match BodyID in body manager");

		// Get location of body
		uint32 node_idx, child_idx;
		GetBodyLocation(ioTracking, *cur, node_idx, child_idx);

		// First we reset our internal bookkeeping
		sInvalidateBodyLocation(ioTracking, *cur);

		// Then we make the bounding box invalid, no queries can find this node anymore
		Node &node = mAllocator->Get(node_idx);
		node.InvalidateChildBounds(child_idx);

		// Finally we reset the child id, this makes the node available for adds again
		node.mChildNodeID[child_idx] = NodeID::sInvalid();

		// We don't need to bubble up our bounding box changes to our parents since we never make volumes smaller, only bigger
		// But we do need to mark the nodes as changed so that the tree can be rebuilt
		MarkNodeAndParentsChanged(node_idx);
	}

	mNumBodies -= inNumber;
}

void QuadTree::NotifyBodiesAABBChanged(const BodyVector &inBodies, const TrackingVector &inTracking, const BodyID *ioBodyIDs, int inNumber)
{
	// Assert sane input
	MOSS_ASSERT(ioBodyIDs != nullptr);
	MOSS_ASSERT(inNumber > 0);

	for (const BodyID *cur = ioBodyIDs, *end = ioBodyIDs + inNumber; cur < end; ++cur)
	{
		// Check if BodyID is correct
		const Body *body = inBodies[cur->GetIndex()];
		MOSS_ASSERT(body->GetID() == *cur, "Provided BodyID doesn't match BodyID in body manager");

		// Get the new bounding box
		const AABox &new_bounds = body->GetWorldSpaceBounds();

		// Get location of body
		uint32 node_idx, child_idx;
		GetBodyLocation(inTracking, *cur, node_idx, child_idx);

		// Widen bounds for node
		Node &node = mAllocator->Get(node_idx);
		if (node.EncapsulateChildBounds(child_idx, new_bounds))
		{
			// Mark tree dirty
			mIsDirty = true;

			// If bounds changed, widen the bounds for our parents too
			WidenAndMarkNodeAndParentsChanged(node_idx, new_bounds);
		}
	}
}

template <class Visitor>
MOSS_INLINE void QuadTree::WalkTree(const ObjectLayerFilter &inObjectLayerFilter, const TrackingVector &inTracking, Visitor &ioVisitor MOSS_IF_TRACK_BROADPHASE_STATS(, LayerToStats &ioStats)) const
{
	// Get the root
	const RootNode &root_node = GetCurrentRoot();

#ifdef MOSS_TRACK_BROADPHASE_STATS
	// Start tracking stats
	int bodies_visited = 0;
	int hits_collected = 0;
	int nodes_visited = 0;
	uint64 collector_ticks = 0;

	uint64 start = GetProcessorTickCount();
#endif // MOSS_TRACK_BROADPHASE_STATS

	NodeID node_stack[cStackSize];
	node_stack[0] = root_node.GetNodeID();
	int top = 0;
	do
	{
		// Check if node is a body
		NodeID child_node_id = node_stack[top];
		if (child_node_id.IsBody())
		{
			// Track amount of bodies visited
			MOSS_IF_TRACK_BROADPHASE_STATS(++bodies_visited;)

			BodyID body_id = child_node_id.GetBodyID();
			ObjectLayer object_layer = inTracking[body_id.GetIndex()].mObjectLayer; // We're not taking a lock on the body, so it may be in the process of being removed so check if the object layer is invalid
			if (object_layer != cObjectLayerInvalid && inObjectLayerFilter.ShouldCollide(object_layer))
			{
				MOSS_PROFILE("VisitBody");

				// Track amount of hits
				MOSS_IF_TRACK_BROADPHASE_STATS(++hits_collected;)

				// Start track time the collector takes
				MOSS_IF_TRACK_BROADPHASE_STATS(uint64 collector_start = GetProcessorTickCount();)

				// We found a body we collide with, call our visitor
				ioVisitor.VisitBody(body_id, top);

				// End track time the collector takes
				MOSS_IF_TRACK_BROADPHASE_STATS(collector_ticks += GetProcessorTickCount() - collector_start;)

				// Check if we're done
				if (ioVisitor.ShouldAbort())
					break;
			}
		}
		else if (child_node_id.IsValid())
		{
			MOSS_IF_TRACK_BROADPHASE_STATS(++nodes_visited;)

			// Check if stack can hold more nodes
			if (top + 4 < cStackSize)
			{
				// Process normal node
				const Node &node = mAllocator->Get(child_node_id.GetNodeIndex());
				MOSS_ASSERT(IsAligned(&node, MOSS_CACHE_LINE_SIZE));

				// Load bounds of 4 children
				Vec4 bounds_minx = Vec4::sLoadFloat4Aligned((const Float4 *)&node.mBoundsMinX);
				Vec4 bounds_miny = Vec4::sLoadFloat4Aligned((const Float4 *)&node.mBoundsMinY);
				Vec4 bounds_minz = Vec4::sLoadFloat4Aligned((const Float4 *)&node.mBoundsMinZ);
				Vec4 bounds_maxx = Vec4::sLoadFloat4Aligned((const Float4 *)&node.mBoundsMaxX);
				Vec4 bounds_maxy = Vec4::sLoadFloat4Aligned((const Float4 *)&node.mBoundsMaxY);
				Vec4 bounds_maxz = Vec4::sLoadFloat4Aligned((const Float4 *)&node.mBoundsMaxZ);

				// Load ids for 4 children
				UVec4 child_ids = UVec4::sLoadInt4Aligned((const uint32 *)&node.mChildNodeID[0]);

				// Check which sub nodes to visit
				int num_results = ioVisitor.VisitNodes(bounds_minx, bounds_miny, bounds_minz, bounds_maxx, bounds_maxy, bounds_maxz, child_ids, top);
				child_ids.StoreInt4((uint32 *)&node_stack[top]);
				top += num_results;
			}
			else
				MOSS_ASSERT(false, "Stack full!\n"
					"This must be a very deep tree. Are you batch adding bodies through BodyInterface::AddBodiesPrepare/AddBodiesFinalize?\n"
					"If you add lots of bodies through BodyInterface::AddBody you may need to call PhysicsSystem::OptimizeBroadPhase to rebuild the tree.");
		}

		// Fetch next node until we find one that the visitor wants to see
		do
			--top;
		while (top >= 0 && !ioVisitor.ShouldVisitNode(top));
	}
	while (top >= 0);

#ifdef MOSS_TRACK_BROADPHASE_STATS
	// Calculate total time the broadphase walk took
	uint64 total_ticks = GetProcessorTickCount() - start;

	// Update stats under lock protection (slow!)
	{
		unique_lock lock(mStatsMutex);
		Stat &s = ioStats[inObjectLayerFilter.GetDescription()];
		s.mNumQueries++;
		s.mNodesVisited += nodes_visited;
		s.mBodiesVisited += bodies_visited;
		s.mHitsReported += hits_collected;
		s.mTotalTicks += total_ticks;
		s.mCollectorTicks += collector_ticks;
	}
#endif // MOSS_TRACK_BROADPHASE_STATS
}

void QuadTree::CastRay(const RayCast &inRay, RayCastBodyCollector &ioCollector, const ObjectLayerFilter &inObjectLayerFilter, const TrackingVector &inTracking) const
{
	class Visitor
	{
	public:
		/// Constructor
		MOSS_INLINE				Visitor(const RayCast &inRay, RayCastBodyCollector &ioCollector) :
			mOrigin(inRay.mOrigin),
			mInvDirection(inRay.mDirection),
			mCollector(ioCollector)
		{
			mFractionStack[0] = -1;
		}

		/// Returns true if further processing of the tree should be aborted
		MOSS_INLINE bool			ShouldAbort() const
		{
			return mCollector.ShouldEarlyOut();
		}

		/// Returns true if this node / body should be visited, false if no hit can be generated
		MOSS_INLINE bool			ShouldVisitNode(int inStackTop) const
		{
			return mFractionStack[inStackTop] < mCollector.GetEarlyOutFraction();
		}

		/// Visit nodes, returns number of hits found and sorts ioChildNodeIDs so that they are at the beginning of the vector.
		MOSS_INLINE int			VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioChildNodeIDs, int inStackTop)
		{
			// Test the ray against 4 bounding boxes
			Vec4 fraction = RayAABox4(mOrigin, mInvDirection, inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);

			// Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
			return SortReverseAndStore(fraction, mCollector.GetEarlyOutFraction(), ioChildNodeIDs, &mFractionStack[inStackTop]);
		}

		/// Visit a body, returns false if the algorithm should terminate because no hits can be generated anymore
		MOSS_INLINE void			VisitBody(const BodyID &inBodyID, int inStackTop)
		{
			// Store potential hit with body
			BroadPhaseCastResult result { inBodyID, mFractionStack[inStackTop] };
			mCollector.AddHit(result);
		}

	private:
		Vec3					mOrigin;
		RayInvDirection			mInvDirection;
		RayCastBodyCollector &	mCollector;
		float					mFractionStack[cStackSize];
	};

	Visitor visitor(inRay, ioCollector);
	WalkTree(inObjectLayerFilter, inTracking, visitor MOSS_IF_TRACK_BROADPHASE_STATS(, mCastRayStats));
}

void QuadTree::CollideAABox(const AABox &inBox, CollideShapeBodyCollector &ioCollector, const ObjectLayerFilter &inObjectLayerFilter, const TrackingVector &inTracking) const
{
	class Visitor
	{
	public:
		/// Constructor
		MOSS_INLINE					Visitor(const AABox &inBox, CollideShapeBodyCollector &ioCollector) :
			mBox(inBox),
			mCollector(ioCollector)
		{
		}

		/// Returns true if further processing of the tree should be aborted
		MOSS_INLINE bool				ShouldAbort() const
		{
			return mCollector.ShouldEarlyOut();
		}

		/// Returns true if this node / body should be visited, false if no hit can be generated
		MOSS_INLINE bool				ShouldVisitNode(int inStackTop) const
		{
			return true;
		}

		/// Visit nodes, returns number of hits found and sorts ioChildNodeIDs so that they are at the beginning of the vector.
		MOSS_INLINE int				VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioChildNodeIDs, int inStackTop) const
		{
			// Test the box vs 4 boxes
			UVec4 hitting = AABox4VsBox(mBox, inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);
			return CountAndSortTrues(hitting, ioChildNodeIDs);
		}

		/// Visit a body, returns false if the algorithm should terminate because no hits can be generated anymore
		MOSS_INLINE void				VisitBody(const BodyID &inBodyID, int inStackTop)
		{
			// Store potential hit with body
			mCollector.AddHit(inBodyID);
		}

	private:
		const AABox &				mBox;
		CollideShapeBodyCollector &	mCollector;
	};

	Visitor visitor(inBox, ioCollector);
	WalkTree(inObjectLayerFilter, inTracking, visitor MOSS_IF_TRACK_BROADPHASE_STATS(, mCollideAABoxStats));
}

void QuadTree::CollideSphere(Vec3Arg inCenter, float inRadius, CollideShapeBodyCollector &ioCollector, const ObjectLayerFilter &inObjectLayerFilter, const TrackingVector &inTracking) const
{
	class Visitor
	{
	public:
		/// Constructor
		MOSS_INLINE					Visitor(Vec3Arg inCenter, float inRadius, CollideShapeBodyCollector &ioCollector) :
			mCenterX(inCenter.SplatX()),
			mCenterY(inCenter.SplatY()),
			mCenterZ(inCenter.SplatZ()),
			mRadiusSq(Vec4::sReplicate(Square(inRadius))),
			mCollector(ioCollector)
		{
		}

		/// Returns true if further processing of the tree should be aborted
		MOSS_INLINE bool				ShouldAbort() const
		{
			return mCollector.ShouldEarlyOut();
		}

		/// Returns true if this node / body should be visited, false if no hit can be generated
		MOSS_INLINE bool				ShouldVisitNode(int inStackTop) const
		{
			return true;
		}

		/// Visit nodes, returns number of hits found and sorts ioChildNodeIDs so that they are at the beginning of the vector.
		MOSS_INLINE int				VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioChildNodeIDs, int inStackTop) const
		{
			// Test 4 boxes vs sphere
			UVec4 hitting = AABox4VsSphere(mCenterX, mCenterY, mCenterZ, mRadiusSq, inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);
			return CountAndSortTrues(hitting, ioChildNodeIDs);
		}

		/// Visit a body, returns false if the algorithm should terminate because no hits can be generated anymore
		MOSS_INLINE void				VisitBody(const BodyID &inBodyID, int inStackTop)
		{
			// Store potential hit with body
			mCollector.AddHit(inBodyID);
		}

	private:
		Vec4						mCenterX;
		Vec4						mCenterY;
		Vec4						mCenterZ;
		Vec4						mRadiusSq;
		CollideShapeBodyCollector &	mCollector;
	};

	Visitor visitor(inCenter, inRadius, ioCollector);
	WalkTree(inObjectLayerFilter, inTracking, visitor MOSS_IF_TRACK_BROADPHASE_STATS(, mCollideSphereStats));
}

void QuadTree::CollidePoint(Vec3Arg inPoint, CollideShapeBodyCollector &ioCollector, const ObjectLayerFilter &inObjectLayerFilter, const TrackingVector &inTracking) const
{
	class Visitor
	{
	public:
		/// Constructor
		MOSS_INLINE					Visitor(Vec3Arg inPoint, CollideShapeBodyCollector &ioCollector) :
			mPoint(inPoint),
			mCollector(ioCollector)
		{
		}

		/// Returns true if further processing of the tree should be aborted
		MOSS_INLINE bool				ShouldAbort() const
		{
			return mCollector.ShouldEarlyOut();
		}

		/// Returns true if this node / body should be visited, false if no hit can be generated
		MOSS_INLINE bool				ShouldVisitNode(int inStackTop) const
		{
			return true;
		}

		/// Visit nodes, returns number of hits found and sorts ioChildNodeIDs so that they are at the beginning of the vector.
		MOSS_INLINE int				VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioChildNodeIDs, int inStackTop) const
		{
			// Test if point overlaps with box
			UVec4 hitting = AABox4VsPoint(mPoint, inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);
			return CountAndSortTrues(hitting, ioChildNodeIDs);
		}

		/// Visit a body, returns false if the algorithm should terminate because no hits can be generated anymore
		MOSS_INLINE void				VisitBody(const BodyID &inBodyID, int inStackTop)
		{
			// Store potential hit with body
			mCollector.AddHit(inBodyID);
		}

	private:
		Vec3						mPoint;
		CollideShapeBodyCollector &	mCollector;
	};

	Visitor visitor(inPoint, ioCollector);
	WalkTree(inObjectLayerFilter, inTracking, visitor MOSS_IF_TRACK_BROADPHASE_STATS(, mCollidePointStats));
}

void QuadTree::CollideOrientedBox(const OrientedBox &inBox, CollideShapeBodyCollector &ioCollector, const ObjectLayerFilter &inObjectLayerFilter, const TrackingVector &inTracking) const
{
	class Visitor
	{
	public:
		/// Constructor
		MOSS_INLINE					Visitor(const OrientedBox &inBox, CollideShapeBodyCollector &ioCollector) :
			mBox(inBox),
			mCollector(ioCollector)
		{
		}

		/// Returns true if further processing of the tree should be aborted
		MOSS_INLINE bool				ShouldAbort() const
		{
			return mCollector.ShouldEarlyOut();
		}

		/// Returns true if this node / body should be visited, false if no hit can be generated
		MOSS_INLINE bool				ShouldVisitNode(int inStackTop) const
		{
			return true;
		}

		/// Visit nodes, returns number of hits found and sorts ioChildNodeIDs so that they are at the beginning of the vector.
		MOSS_INLINE int				VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioChildNodeIDs, int inStackTop) const
		{
			// Test if point overlaps with box
			UVec4 hitting = AABox4VsBox(mBox, inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);
			return CountAndSortTrues(hitting, ioChildNodeIDs);
		}

		/// Visit a body, returns false if the algorithm should terminate because no hits can be generated anymore
		MOSS_INLINE void				VisitBody(const BodyID &inBodyID, int inStackTop)
		{
			// Store potential hit with body
			mCollector.AddHit(inBodyID);
		}

	private:
		OrientedBox					mBox;
		CollideShapeBodyCollector &	mCollector;
	};

	Visitor visitor(inBox, ioCollector);
	WalkTree(inObjectLayerFilter, inTracking, visitor MOSS_IF_TRACK_BROADPHASE_STATS(, mCollideOrientedBoxStats));
}

void QuadTree::CastAABox(const AABoxCast &inBox, CastShapeBodyCollector &ioCollector, const ObjectLayerFilter &inObjectLayerFilter, const TrackingVector &inTracking) const
{
	class Visitor
	{
	public:
		/// Constructor
		MOSS_INLINE					Visitor(const AABoxCast &inBox, CastShapeBodyCollector &ioCollector) :
			mOrigin(inBox.mBox.GetCenter()),
			mExtent(inBox.mBox.GetExtent()),
			mInvDirection(inBox.mDirection),
			mCollector(ioCollector)
		{
			mFractionStack[0] = -1;
		}

		/// Returns true if further processing of the tree should be aborted
		MOSS_INLINE bool				ShouldAbort() const
		{
			return mCollector.ShouldEarlyOut();
		}

		/// Returns true if this node / body should be visited, false if no hit can be generated
		MOSS_INLINE bool				ShouldVisitNode(int inStackTop) const
		{
			return mFractionStack[inStackTop] < mCollector.GetPositiveEarlyOutFraction();
		}

		/// Visit nodes, returns number of hits found and sorts ioChildNodeIDs so that they are at the beginning of the vector.
		MOSS_INLINE int				VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioChildNodeIDs, int inStackTop)
		{
			// Enlarge them by the casted aabox extents
			Vec4 bounds_min_x = inBoundsMinX, bounds_min_y = inBoundsMinY, bounds_min_z = inBoundsMinZ, bounds_max_x = inBoundsMaxX, bounds_max_y = inBoundsMaxY, bounds_max_z = inBoundsMaxZ;
			AABox4EnlargeWithExtent(mExtent, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			// Test 4 children
			Vec4 fraction = RayAABox4(mOrigin, mInvDirection, bounds_min_x, bounds_min_y, bounds_min_z, bounds_max_x, bounds_max_y, bounds_max_z);

			// Sort so that highest values are first (we want to first process closer hits and we process stack top to bottom)
			return SortReverseAndStore(fraction, mCollector.GetPositiveEarlyOutFraction(), ioChildNodeIDs, &mFractionStack[inStackTop]);
		}

		/// Visit a body, returns false if the algorithm should terminate because no hits can be generated anymore
		MOSS_INLINE void				VisitBody(const BodyID &inBodyID, int inStackTop)
		{
			// Store potential hit with body
			BroadPhaseCastResult result { inBodyID, mFractionStack[inStackTop] };
			mCollector.AddHit(result);
		}

	private:
		Vec3						mOrigin;
		Vec3						mExtent;
		RayInvDirection				mInvDirection;
		CastShapeBodyCollector &	mCollector;
		float						mFractionStack[cStackSize];
	};

	Visitor visitor(inBox, ioCollector);
	WalkTree(inObjectLayerFilter, inTracking, visitor MOSS_IF_TRACK_BROADPHASE_STATS(, mCastAABoxStats));
}

void QuadTree::FindCollidingPairs(const BodyVector &inBodies, const BodyID *inActiveBodies, int inNumActiveBodies, float inSpeculativeContactDistance, BodyPairCollector &ioPairCollector, const ObjectLayerPairFilter &inObjectLayerPairFilter) const
{
	// Note that we don't lock the tree at this point. We know that the tree is not going to be swapped or deleted while finding collision pairs due to the way the jobs are scheduled in the PhysicsSystem::Update.
	// We double check this at the end of the function.
	const RootNode &root_node = GetCurrentRoot();
	MOSS_ASSERT(root_node.mIndex != cInvalidNodeIndex);

	// Assert sane input
	MOSS_ASSERT(inActiveBodies != nullptr);
	MOSS_ASSERT(inNumActiveBodies > 0);

	NodeID node_stack[cStackSize];

	// Loop over all active bodies
	for (int b1 = 0; b1 < inNumActiveBodies; ++b1)
	{
		BodyID b1_id = inActiveBodies[b1];
		const Body &body1 = *inBodies[b1_id.GetIndex()];
		MOSS_ASSERT(!body1.IsStatic());

		// Expand the bounding box by the speculative contact distance
		AABox bounds1 = body1.GetWorldSpaceBounds();
		bounds1.ExpandBy(Vec3::sReplicate(inSpeculativeContactDistance));

		// Test each body with the tree
		node_stack[0] = root_node.GetNodeID();
		int top = 0;
		do
		{
			// Check if node is a body
			NodeID child_node_id = node_stack[top];
			if (child_node_id.IsBody())
			{
				// Don't collide with self
				BodyID b2_id = child_node_id.GetBodyID();
				if (b1_id != b2_id)
				{
					// Collision between dynamic pairs need to be picked up only once
					const Body &body2 = *inBodies[b2_id.GetIndex()];
					if (inObjectLayerPairFilter.ShouldCollide(body1.GetObjectLayer(), body2.GetObjectLayer())
						&& Body::sFindCollidingPairsCanCollide(body1, body2)
						&& bounds1.Overlaps(body2.GetWorldSpaceBounds())) // In the broadphase we widen the bounding box when a body moves, do a final check to see if the bounding boxes actually overlap
					{
						// Store potential hit between bodies
						ioPairCollector.AddHit({ b1_id, b2_id });
					}
				}
			}
			else if (child_node_id.IsValid())
			{
				// Process normal node
				const Node &node = mAllocator->Get(child_node_id.GetNodeIndex());
				MOSS_ASSERT(IsAligned(&node, MOSS_CACHE_LINE_SIZE));

				// Get bounds of 4 children
				Vec4 bounds_minx = Vec4::sLoadFloat4Aligned((const Float4 *)&node.mBoundsMinX);
				Vec4 bounds_miny = Vec4::sLoadFloat4Aligned((const Float4 *)&node.mBoundsMinY);
				Vec4 bounds_minz = Vec4::sLoadFloat4Aligned((const Float4 *)&node.mBoundsMinZ);
				Vec4 bounds_maxx = Vec4::sLoadFloat4Aligned((const Float4 *)&node.mBoundsMaxX);
				Vec4 bounds_maxy = Vec4::sLoadFloat4Aligned((const Float4 *)&node.mBoundsMaxY);
				Vec4 bounds_maxz = Vec4::sLoadFloat4Aligned((const Float4 *)&node.mBoundsMaxZ);

				// Test overlap
				UVec4 overlap = AABox4VsBox(bounds1, bounds_minx, bounds_miny, bounds_minz, bounds_maxx, bounds_maxy, bounds_maxz);
				int num_results = overlap.CountTrues();
				if (num_results > 0)
				{
					// Load ids for 4 children
					UVec4 child_ids = UVec4::sLoadInt4Aligned((const uint32 *)&node.mChildNodeID[0]);

					// Sort so that overlaps are first
					child_ids = UVec4::sSort4True(overlap, child_ids);

					// Push them onto the stack
					if (top + 4 < cStackSize)
					{
						child_ids.StoreInt4((uint32 *)&node_stack[top]);
						top += num_results;
					}
					else
						MOSS_ASSERT(false, "Stack full!\n"
							"This must be a very deep tree. Are you batch adding bodies through BodyInterface::AddBodiesPrepare/AddBodiesFinalize?\n"
							"If you add lots of bodies through BodyInterface::AddBody you may need to call PhysicsSystem::OptimizeBroadPhase to rebuild the tree.");
				}
			}
			--top;
		}
		while (top >= 0);
	}

	// Test that the root node was not swapped while finding collision pairs.
	// This would mean that UpdateFinalize/DiscardOldTree ran during collision detection which should not be possible due to the way the jobs are scheduled.
	MOSS_ASSERT(root_node.mIndex != cInvalidNodeIndex);
	MOSS_ASSERT(&root_node == &GetCurrentRoot());
}

#ifdef MOSS_DEBUG

void QuadTree::ValidateTree(const BodyVector &inBodies, const TrackingVector &inTracking, uint32 inNodeIndex, uint32 inNumExpectedBodies) const
{
	MOSS_PROFILE_FUNCTION();

	// Root should be valid
	MOSS_ASSERT(inNodeIndex != cInvalidNodeIndex);

	// To avoid call overhead, create a stack in place
	MOSS_SUPPRESS_WARNING_PUSH
	MOSS_CLANG_SUPPRESS_WARNING("-Wunused-member-function") // The default constructor of StackEntry is unused when using Jolt's Array class but not when using std::vector
	struct StackEntry
	{
						StackEntry() = default;
		inline			StackEntry(uint32 inNodeIndex, uint32 inParentNodeIndex) : mNodeIndex(inNodeIndex), mParentNodeIndex(inParentNodeIndex) { }

		uint32			mNodeIndex;
		uint32			mParentNodeIndex;
	};
	MOSS_SUPPRESS_WARNING_POP
	TArray<StackEntry, STLLocalAllocator<StackEntry, cStackSize>> stack;
	stack.reserve(cStackSize);
	stack.emplace_back(inNodeIndex, cInvalidNodeIndex);

	uint32 num_bodies = 0;

	do
	{
		// Copy entry from the stack
		StackEntry cur_stack = stack.back();
		stack.pop_back();

		// Validate parent
		const Node &node = mAllocator->Get(cur_stack.mNodeIndex);
		MOSS_ASSERT(node.mParentNodeIndex == cur_stack.mParentNodeIndex);

		// Validate that when a parent is not-changed that all of its children are also
		MOSS_ASSERT(cur_stack.mParentNodeIndex == cInvalidNodeIndex || mAllocator->Get(cur_stack.mParentNodeIndex).mIsChanged || !node.mIsChanged);

		// Loop children
		for (uint32 i = 0; i < 4; ++i)
		{
			NodeID child_node_id = node.mChildNodeID[i];
			if (child_node_id.IsValid())
			{
				if (child_node_id.IsNode())
				{
					// Child is a node, recurse
					uint32 child_idx = child_node_id.GetNodeIndex();
					stack.emplace_back(child_idx, cur_stack.mNodeIndex);

					// Validate that the bounding box is bigger or equal to the bounds in the tree
					// Bounding box could also be invalid if all children of our child were removed
					AABox child_bounds;
					node.GetChildBounds(i, child_bounds);
					AABox real_child_bounds;
					mAllocator->Get(child_idx).GetNodeBounds(real_child_bounds);
					MOSS_ASSERT(child_bounds.Contains(real_child_bounds) || !real_child_bounds.IsValid());
				}
				else
				{
					// Increment number of bodies found
					++num_bodies;

					// Check if tracker matches position of body
					uint32 node_idx, child_idx;
					GetBodyLocation(inTracking, child_node_id.GetBodyID(), node_idx, child_idx);
					MOSS_ASSERT(node_idx == cur_stack.mNodeIndex);
					MOSS_ASSERT(child_idx == i);

					// Validate that the body cached bounds still match the actual bounds
					const Body *body = inBodies[child_node_id.GetBodyID().GetIndex()];
					body->ValidateCachedBounds();

					// Validate that the node bounds are bigger or equal to the body bounds
					AABox body_bounds;
					node.GetChildBounds(i, body_bounds);
					MOSS_ASSERT(body_bounds.Contains(body->GetWorldSpaceBounds()));
				}
			}
		}
	}
	while (!stack.empty());

	// Check that the amount of bodies in the tree matches our counter
	MOSS_ASSERT(num_bodies == inNumExpectedBodies);
}

#endif

#ifdef MOSS_DUMP_BROADPHASE_TREE

void QuadTree::DumpTree(const NodeID &inRoot, const char *inFileNamePrefix) const
{
	// Open DOT file
	std::ofstream f;
	f.open(StringFormat("%s.dot", inFileNamePrefix).c_str(), std::ofstream::out | std::ofstream::trunc);
	if (!f.is_open())
		return;

	// Write header
	f << "digraph {\n";

	// Iterate the entire tree
	TArray<NodeID, STLLocalAllocator<NodeID, cStackSize>> node_stack;
	node_stack.push_back(inRoot);
	MOSS_ASSERT(inRoot.IsValid());
	do
	{
		// Check if node is a body
		NodeID node_id = node_stack.back();
		node_stack.pop_back();
		if (node_id.IsBody())
		{
			// Output body
			String body_id = ConvertToString(node_id.GetBodyID().GetIndex());
			f << "body" << body_id << "[label = \"Body " << body_id << "\"]\n";
		}
		else
		{
			// Process normal node
			uint32 node_idx = node_id.GetNodeIndex();
			const Node &node = mAllocator->Get(node_idx);

			// Get bounding box
			AABox bounds;
			node.GetNodeBounds(bounds);

			// Output node
			String node_str = ConvertToString(node_idx);
			f << "node" << node_str << "[label = \"Node " << node_str << "\nVolume: " << ConvertToString(bounds.GetVolume()) << "\" color=" << (node.mIsChanged? "red" : "black") << "]\n";

			// Recurse and get all children
			for (NodeID child_node_id : node.mChildNodeID)
				if (child_node_id.IsValid())
				{
					node_stack.push_back(child_node_id);

					// Output link
					f << "node" << node_str << " -> ";
					if (child_node_id.IsBody())
						f << "body" << ConvertToString(child_node_id.GetBodyID().GetIndex());
					else
						f << "node" << ConvertToString(child_node_id.GetNodeIndex());
					f << "\n";
				}
		}
	}
	while (!node_stack.empty());

	// Finish DOT file
	f << "}\n";
	f.close();

	// Convert to svg file
	String cmd = StringFormat("dot %s.dot -Tsvg -o %s.svg", inFileNamePrefix, inFileNamePrefix);
	system(cmd.c_str());
}

#endif // MOSS_DUMP_BROADPHASE_TREE

#ifdef MOSS_TRACK_BROADPHASE_STATS

uint64 QuadTree::GetTicks100Pct(const LayerToStats &inLayer) const
{
	uint64 total_ticks = 0;
	for (const LayerToStats::value_type &kv : inLayer)
		total_ticks += kv.second.mTotalTicks;
	return total_ticks;
}

void QuadTree::ReportStats(const char *inName, const LayerToStats &inLayer, uint64 inTicks100Pct) const
{
	for (const LayerToStats::value_type &kv : inLayer)
	{
		double total_pct = 100.0 * double(kv.second.mTotalTicks) / double(inTicks100Pct);
		double total_pct_excl_collector = 100.0 * double(kv.second.mTotalTicks - kv.second.mCollectorTicks) / double(inTicks100Pct);
		double hits_reported_vs_bodies_visited = kv.second.mBodiesVisited > 0? 100.0 * double(kv.second.mHitsReported) / double(kv.second.mBodiesVisited) : 100.0;
		double hits_reported_vs_nodes_visited = kv.second.mNodesVisited > 0? double(kv.second.mHitsReported) / double(kv.second.mNodesVisited) : -1.0;

		std::stringstream str;
		str << inName << ", " << kv.first << ", " << mName << ", " << kv.second.mNumQueries << ", " << total_pct << ", " << total_pct_excl_collector << ", " << kv.second.mNodesVisited << ", " << kv.second.mBodiesVisited << ", " << kv.second.mHitsReported << ", " << hits_reported_vs_bodies_visited << ", " << hits_reported_vs_nodes_visited;
		MOSS_TRACE(str.str().c_str());
	}
}

uint64 QuadTree::GetTicks100Pct() const
{
	uint64 total_ticks = 0;
	total_ticks += GetTicks100Pct(mCastRayStats);
	total_ticks += GetTicks100Pct(mCollideAABoxStats);
	total_ticks += GetTicks100Pct(mCollideSphereStats);
	total_ticks += GetTicks100Pct(mCollidePointStats);
	total_ticks += GetTicks100Pct(mCollideOrientedBoxStats);
	total_ticks += GetTicks100Pct(mCastAABoxStats);
	return total_ticks;
}

void QuadTree::ReportStats(uint64 inTicks100Pct) const
{
	unique_lock lock(mStatsMutex);
	ReportStats("RayCast", mCastRayStats, inTicks100Pct);
	ReportStats("CollideAABox", mCollideAABoxStats, inTicks100Pct);
	ReportStats("CollideSphere", mCollideSphereStats, inTicks100Pct);
	ReportStats("CollidePoint", mCollidePointStats, inTicks100Pct);
	ReportStats("CollideOrientedBox", mCollideOrientedBoxStats, inTicks100Pct);
	ReportStats("CastAABox", mCastAABoxStats, inTicks100Pct);
}

#endif // MOSS_TRACK_BROADPHASE_STATS

uint QuadTree::GetMaxTreeDepth(const NodeID &inNodeID) const
{
	// Reached a leaf?
	if (!inNodeID.IsValid() || inNodeID.IsBody())
		return 0;

	// Recurse to children
	uint max_depth = 0;
	const Node &node = mAllocator->Get(inNodeID.GetNodeIndex());
	for (NodeID child_node_id : node.mChildNodeID)
		max_depth = max(max_depth, GetMaxTreeDepth(child_node_id));
	return max_depth + 1;
}

MOSS_SUPRESS_WARNINGS_END
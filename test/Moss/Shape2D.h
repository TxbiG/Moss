#ifndef SHAPE2D_H
#define SHAPE2D_H


class [[nodiscard]] CircleShape2D {
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
	inline	Sphere() = default;
	inline	Sphere(const Float2 &inCenter, float inRadius)			: mCenter(inCenter), mRadius(inRadius) { }
	inline	Sphere(Vec2Arg inCenter, float inRadius)				: mRadius(inRadius) { inCenter.StoreFloat3(&mCenter); }

	/// Calculate the support vector for this convex shape.
	inline Vec2 GetSupport(Vec2Arg inDirection) const
	{
		float length = inDirection.Length();
		return length > 0.0f ? Vec3::sLoadFloat3Unsafe(mCenter) + (mRadius/ length) * inDirection : Vec3::sLoadFloat3Unsafe(mCenter);
	}

	// Properties
	inline Vec2	 GetCenter() const										{ return Vec2::sLoadFloat3Unsafe(mCenter); }
	inline float GetRadius() const										{ return mRadius; }

	/// Test if two spheres overlap
	inline bool Overlaps(const Sphere &inB) const { return (Vec2::sLoadFloat3Unsafe(mCenter) - Vec2::sLoadFloat3Unsafe(inB.mCenter)).LengthSq() <= Square(mRadius + inB.mRadius); }

	/// Check if this sphere overlaps with a box
	inline bool Overlaps(const AABox2 &inOther) const { return inOther.GetSqDistanceTo(GetCenter()) <= Square(mRadius); }

	/// Create the minimal sphere that encapsulates this sphere and inPoint
	inline void EncapsulatePoint(Vec2Arg inPoint) {
		// Calculate distance between point and center
		Vec2 center = GetCenter();
		Vec2 d_vec = inPoint - center;
		float d_sq = d_vec.LengthSq();
		if (d_sq > Square(mRadius))
		{
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
	Float2				mCenter;
	float				mRadius;
};

class BoxShape2D {
public:
	Box2D();
private:
	AABB2 point1;
};

class CapsuleShape2D {
public:
	Capsule2D();
private:
	AABB2 point1;
	AABB2 point2;
};

class PolygonShape2D;
class SeperationRayShape2D;
class WorldBoundaryShape2D;

// A line segment with two-sided collision.
class SegmentShape2D {
public:
	Segment2D();
	// The first point
	Vec2 point1;

	// The second point
	Vec2 point2;
};

// A line segment with one-sided collision. Only collides on the right side.
// Several of these are generated for a chain shape.
// ghost1 -> point1 -> point2 -> ghost2
class ChainSegment2D {
public:
	ChainSegment();

	// The tail ghost vertex
	Vec2 ghost1;

	// The line segment
	b2Segment segment;

	// The head ghost vertex
	Vec2 ghost2;

	// The owning chain shape index (internal usage only)
	int chainId;
};


class Chain2D {
public:
	Chain2D();
	// Use this to store application specific shape data.
	void* userData;

	// An array of at least 4 points. These are cloned and may be temporary.
	const b2Vec2* points;

	// The point count, must be 4 or more.
	int count;

	// Surface materials for each segment. These are cloned.
	const b2SurfaceMaterial* materials;

	// The material count. Must be 1 or count. This allows you to provide one
	// material for all segments or a unique material per segment.
	int materialCount;

	// Contact filtering data.
	b2Filter filter;

	// Indicates a closed chain formed by connecting the first and last points
	bool isLoop;

	// Enable sensors to detect this chain. False by default.
	bool enableSensorEvents;

	// Used internally to detect a valid definition. DO NOT SET.
	int internalValue;
};


class ConvexShape2D;
class ConvexHullShape2D;

/*
ShapeSettings
MOSS_API void ShapeSettings_Destroy(ShapeSettings* settings);
MOSS_API uint64_t ShapeSettings_GetUserData(const ShapeSettings* settings);
MOSS_API void ShapeSettings_SetUserData(ShapeSettings* settings, uint64_t userData);

 Shape
MOSS_API void Shape_Destroy(Shape* shape);
MOSS_API ShapeType Shape_GetType(const Shape* shape);
MOSS_API ShapeSubType Shape_GetSubType(const Shape* shape);
MOSS_API uint64_t Shape_GetUserData(const Shape* shape);
MOSS_API void Shape_SetUserData(Shape* shape, uint64_t userData);
MOSS_API bool Shape_MustBeStatic(const Shape* shape);
MOSS_API void Shape_GetCenterOfMass(const Shape* shape, Vec3* result);
MOSS_API void Shape_GetLocalBounds(const Shape* shape, AABB3* result);
MOSS_API uint32_t Shape_GetSubShapeIDBitsRecursive(const Shape* shape);
MOSS_API void Shape_GetWorldSpaceBounds(const Shape* shape, RMat44* centerOfMassTransform, Vec3* scale, AABB3* result);
MOSS_API float Shape_GetInnerRadius(const Shape* shape);
MOSS_API void Shape_GetMassProperties(const Shape* shape, MassProperties* result);
MOSS_API const Shape* Shape_GetLeafShape(const Shape* shape, SubShapeID subShapeID, SubShapeID* remainder);
MOSS_API const PhysicsMaterial* Shape_GetMaterial(const Shape* shape, SubShapeID subShapeID);
MOSS_API void Shape_GetSurfaceNormal(const Shape* shape, SubShapeID subShapeID, Vec3* localPosition, Vec3* normal);
MOSS_API void Shape_GetSupportingFace(const Shape* shape, const SubShapeID subShapeID, const Vec3* direction, const Vec3* scale, const Mat44* centerOfMassTransform, SupportingFace* outVertices);
MOSS_API float Shape_GetVolume(const Shape* shape);
MOSS_API bool Shape_IsValidScale(const Shape* shape, const Vec3* scale);
MOSS_API void Shape_MakeScaleValid(const Shape* shape, const Vec3* scale, Vec3* result);
MOSS_API Shape* Shape_ScaleShape(const Shape* shape, const Vec3* scale);
MOSS_API bool Shape_CastRay(const Shape* shape, const Vec3* origin, const Vec3* direction, RayCastResult* hit);
MOSS_API bool Shape_CastRay2(const Shape* shape, const Vec3* origin, const Vec3* direction, const RayCastSettings* rayCastSettings, CollisionCollectorType collectorType, CastRayResultCallback* callback, void* userData, const ShapeFilter* shapeFilter);
MOSS_API bool Shape_CollidePoint(const Shape* shape, const Vec3* point, const ShapeFilter* shapeFilter);
MOSS_API bool Shape_CollidePoint2(const Shape* shape, const Vec3* point, CollisionCollectorType collectorType, CollidePointResultCallback* callback, void* userData, const ShapeFilter* shapeFilter);

 CompoundShape
MOSS_API void CompoundShapeSettings_AddShape(CompoundShapeSettings* settings, const Vec3* position, const Quat* rotation, const ShapeSettings* shapeSettings, uint32_t userData);
MOSS_API void CompoundShapeSettings_AddShape2(CompoundShapeSettings* settings, const Vec3* position, const Quat* rotation, const Shape* shape, uint32_t userData);
MOSS_API uint32_t CompoundShape_GetNumSubShapes(const CompoundShape* shape);
MOSS_API void CompoundShape_GetSubShape(const CompoundShape* shape, uint32_t index, const Shape** subShape, Vec3* positionCOM, Quat* rotation, uint32_t* userData);
MOSS_API uint32_t CompoundShape_GetSubShapeIndexFromID(const CompoundShape* shape, SubShapeID id, SubShapeID* remainder);

StaticCompoundShape
MOSS_API StaticCompoundShapeSettings* StaticCompoundShapeSettings_Create(void);
MOSS_API StaticCompoundShape* StaticCompoundShape_Create(const StaticCompoundShapeSettings* settings);

MutableCompoundShape
MOSS_API MutableCompoundShapeSettings* MutableCompoundShapeSettings_Create(void);
MOSS_API MutableCompoundShape* MutableCompoundShape_Create(const MutableCompoundShapeSettings* settings);

MOSS_API uint32_t MutableCompoundShape_AddShape(MutableCompoundShape* shape, const Vec3* position, const Quat* rotation, const Shape* child, uint32_t userData, uint32_t index);
MOSS_API void MutableCompoundShape_RemoveShape(MutableCompoundShape* shape, uint32_t index);
MOSS_API void MutableCompoundShape_ModifyShape(MutableCompoundShape* shape, uint32_t index, const Vec3* position, const Quat* rotation);
MOSS_API void MutableCompoundShape_ModifyShape2(MutableCompoundShape* shape, uint32_t index, const Vec3* position, const Quat* rotation, const Shape* newShape);
MOSS_API void MutableCompoundShape_AdjustCenterOfMass(MutableCompoundShape* shape);

DecoratedShape 
MOSS_API const Shape* DecoratedShape_GetInnerShape(const DecoratedShape* shape);

RotatedTranslatedShape
MOSS_API RotatedTranslatedShapeSettings* RotatedTranslatedShapeSettings_Create(const Vec3* position, const Quat* rotation, const ShapeSettings* shapeSettings);
MOSS_API RotatedTranslatedShapeSettings* RotatedTranslatedShapeSettings_Create2(const Vec3* position, const Quat* rotation, const Shape* shape);
MOSS_API RotatedTranslatedShape* RotatedTranslatedShapeSettings_CreateShape(const RotatedTranslatedShapeSettings* settings);
MOSS_API RotatedTranslatedShape* RotatedTranslatedShape_Create(const Vec3* position, const Quat* rotation, const Shape* shape);
MOSS_API void RotatedTranslatedShape_GetPosition(const RotatedTranslatedShape* shape, Vec3* position);
MOSS_API void RotatedTranslatedShape_GetRotation(const RotatedTranslatedShape* shape, Quat* rotation);

 ScaledShape
MOSS_API ScaledShapeSettings* ScaledShapeSettings_Create(const ShapeSettings* shapeSettings, const Vec3* scale);
MOSS_API ScaledShapeSettings* ScaledShapeSettings_Create2(const Shape* shape, const Vec3* scale);
MOSS_API ScaledShape* ScaledShapeSettings_CreateShape(const ScaledShapeSettings* settings);
MOSS_API ScaledShape* ScaledShape_Create(const Shape* shape, const Vec3* scale);
MOSS_API void ScaledShape_GetScale(const ScaledShape* shape, Vec3* result);

OffsetCenterOfMassShape
MOSS_API OffsetCenterOfMassShapeSettings* OffsetCenterOfMassShapeSettings_Create(const Vec3* offset, const ShapeSettings* shapeSettings);
MOSS_API OffsetCenterOfMassShapeSettings* OffsetCenterOfMassShapeSettings_Create2(const Vec3* offset, const Shape* shape);
MOSS_API OffsetCenterOfMassShape* OffsetCenterOfMassShapeSettings_CreateShape(const OffsetCenterOfMassShapeSettings* settings);

MOSS_API OffsetCenterOfMassShape* OffsetCenterOfMassShape_Create(const Vec3* offset, const Shape* shape);
MOSS_API void OffsetCenterOfMassShape_GetOffset(const OffsetCenterOfMassShape* shape, Vec3* result);

 EmptyShape
MOSS_API EmptyShapeSettings* EmptyShapeSettings_Create(const Vec3* centerOfMass);
MOSS_API EmptyShape* EmptyShapeSettings_CreateShape(const EmptyShapeSettings* settings);
*/


MOSS_SUPRESS_WARNINGS_END
#endif
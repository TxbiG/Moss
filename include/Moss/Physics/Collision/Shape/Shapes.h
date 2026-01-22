// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Physics/Collision/Shape/ConvexShape.h>

MOSS_NAMESPACE_BEGIN
struct RayCast;
class RayCastSettings;
struct ShapeCast;
class ShapeCastSettings;
class RayCastResult;
class ShapeCastResult;
class CollidePointResult;
class CollideShapeResult;
class SubShapeIDCreator;
class SubShapeID;
class PhysicsMaterial;
class TransformedShape;
class Plane;
class CollideSoftBodyVertexIterator;
class Shape;
class StreamOut;
class StreamIn;
class Shape;
#ifndef MOSS_DEBUG_RENDERER
class DebugRenderer;
#endif // MOSS_DEBUG_RENDERER

using CastRayCollector = CollisionCollector<RayCastResult, CollisionCollectorTraitsCastRay>;
using CastShapeCollector = CollisionCollector<ShapeCastResult, CollisionCollectorTraitsCastShape>;
using CollidePointCollector = CollisionCollector<CollidePointResult, CollisionCollectorTraitsCollidePoint>;
using CollideShapeCollector = CollisionCollector<CollideShapeResult, CollisionCollectorTraitsCollideShape>;
using TransformedShapeCollector = CollisionCollector<TransformedShape, CollisionCollectorTraitsCollideShape>;

using ShapeRefC = RefConst<Shape>;
using ShapeList = TArray<ShapeRefC>;
using PhysicsMaterialRefC = RefConst<PhysicsMaterial>;
using PhysicsMaterialList = TArray<PhysicsMaterialRefC>;

/// Shapes are categorized in groups, each shape can return which group it belongs to through its Shape::GetType function.
enum class EShapeType : uint8
{
	Convex,							///< Used by ConvexShape, all shapes that use the generic convex vs convex collision detection system (box, sphere, capsule, tapered capsule, cylinder, triangle)
	Compound,						///< Used by CompoundShape
	Decorated,						///< Used by DecoratedShape
	Mesh,							///< Used by MeshShape
	HeightField,					///< Used by HeightFieldShape
	SoftBody,						///< Used by SoftBodyShape

	// User defined shapes
	User1,
	User2,
	User3,
	User4,

	Plane,							///< Used by PlaneShape
	Empty,							///< Used by EmptyShape
};

/// This enumerates all shape types, each shape can return its type through Shape::GetSubType
enum class EShapeSubType : uint8
{
	// Convex shapes
	Sphere,
	Box,
	Triangle,
	Capsule,
	TaperedCapsule,
	Cylinder,
	ConvexHull,

	// Compound shapes
	StaticCompound,
	MutableCompound,

	// Decorated shapes
	RotatedTranslated,
	Scaled,
	OffsetCenterOfMass,

	// Other shapes
	Mesh,
	HeightField,
	SoftBody,

	// User defined shapes
	User1,
	User2,
	User3,
	User4,
	User5,
	User6,
	User7,
	User8,

	// User defined convex shapes
	UserConvex1,
	UserConvex2,
	UserConvex3,
	UserConvex4,
	UserConvex5,
	UserConvex6,
	UserConvex7,
	UserConvex8,

	// Other shapes
	Plane,
	TaperedCylinder,
	Empty,
};

// Sets of shape sub types
static constexpr EShapeSubType sAllSubShapeTypes[] = { EShapeSubType::Sphere, EShapeSubType::Box, EShapeSubType::Triangle, EShapeSubType::Capsule, EShapeSubType::TaperedCapsule, EShapeSubType::Cylinder, EShapeSubType::ConvexHull, EShapeSubType::StaticCompound, EShapeSubType::MutableCompound, EShapeSubType::RotatedTranslated, EShapeSubType::Scaled, EShapeSubType::OffsetCenterOfMass, EShapeSubType::Mesh, EShapeSubType::HeightField, EShapeSubType::SoftBody, EShapeSubType::User1, EShapeSubType::User2, EShapeSubType::User3, EShapeSubType::User4, EShapeSubType::User5, EShapeSubType::User6, EShapeSubType::User7, EShapeSubType::User8, EShapeSubType::UserConvex1, EShapeSubType::UserConvex2, EShapeSubType::UserConvex3, EShapeSubType::UserConvex4, EShapeSubType::UserConvex5, EShapeSubType::UserConvex6, EShapeSubType::UserConvex7, EShapeSubType::UserConvex8, EShapeSubType::Plane, EShapeSubType::TaperedCylinder, EShapeSubType::Empty };
static constexpr EShapeSubType sConvexSubShapeTypes[] = { EShapeSubType::Sphere, EShapeSubType::Box, EShapeSubType::Triangle, EShapeSubType::Capsule, EShapeSubType::TaperedCapsule, EShapeSubType::Cylinder, EShapeSubType::ConvexHull, EShapeSubType::TaperedCylinder, EShapeSubType::UserConvex1, EShapeSubType::UserConvex2, EShapeSubType::UserConvex3, EShapeSubType::UserConvex4, EShapeSubType::UserConvex5, EShapeSubType::UserConvex6, EShapeSubType::UserConvex7, EShapeSubType::UserConvex8 };
static constexpr EShapeSubType sCompoundSubShapeTypes[] = { EShapeSubType::StaticCompound, EShapeSubType::MutableCompound };
static constexpr EShapeSubType sDecoratorSubShapeTypes[] = { EShapeSubType::RotatedTranslated, EShapeSubType::Scaled, EShapeSubType::OffsetCenterOfMass };

/// How many shape types we support
static constexpr uint NumSubShapeTypes = uint(std::size(sAllSubShapeTypes));

/// Names of sub shape types
static constexpr const char *sSubShapeTypeNames[] = { "Sphere", "Box", "Triangle", "Capsule", "TaperedCapsule", "Cylinder", "ConvexHull", "StaticCompound", "MutableCompound", "RotatedTranslated", "Scaled", "OffsetCenterOfMass", "Mesh", "HeightField", "SoftBody", "User1", "User2", "User3", "User4", "User5", "User6", "User7", "User8", "UserConvex1", "UserConvex2", "UserConvex3", "UserConvex4", "UserConvex5", "UserConvex6", "UserConvex7", "UserConvex8", "Plane", "TaperedCylinder", "Empty" };
static_assert(std::size(sSubShapeTypeNames) == NumSubShapeTypes);

/// Class that can construct shapes and that is serializable using the ObjectStream system.
/// Can be used to store shape data in 'uncooked' form (i.e. in a form that is still human readable and authorable).
/// Once the shape has been created using the Create() function, the data will be moved into the Shape class
/// in a form that is optimized for collision detection. After this, the ShapeSettings object is no longer needed
/// and can be destroyed. Each shape class has a derived class of the ShapeSettings object to store shape specific
/// data.
class MOSS_EXPORT ShapeSettings : public SerializableObject, public RefTarget<ShapeSettings>
{
	MOSS_DECLARE_SERIALIZABLE_ABSTRACT(MOSS_EXPORT, ShapeSettings)

public:
	using ShapeResult = Result<Ref<Shape>>;

	/// Create a shape according to the settings specified by this object.
	virtual ShapeResult				Create() const = 0;

	/// When creating a shape, the result is cached so that calling Create() again will return the same shape.
	/// If you make changes to the ShapeSettings you need to call this function to clear the cached result to allow Create() to build a new shape.
	void							ClearCachedResult()													{ mCachedResult.Clear(); }

	/// User data (to be used freely by the application)
	uint64							mUserData = 0;

protected:
	mutable ShapeResult				mCachedResult;
};

/// Function table for functions on shapes
class MOSS_EXPORT ShapeFunctions
{
public:
	/// Construct a shape
	Shape *							(*mConstruct)() = nullptr;

	/// Color of the shape when drawing
	Color							mColor = Color::sBlack;

	/// Get an entry in the registry for a particular sub type
	static inline ShapeFunctions &	sGet(EShapeSubType inSubType)										{ return sRegistry[int(inSubType)]; }

private:
	static ShapeFunctions			sRegistry[NumSubShapeTypes];
};

/// Base class for all shapes (collision volume of a body). Defines a virtual interface for collision detection.
class MOSS_EXPORT Shape : public RefTarget<Shape>, public NonCopyable
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	using ShapeResult = ShapeSettings::ShapeResult;

	/// Constructor
									Shape(EShapeType inType, EShapeSubType inSubType) : mShapeType(inType), mShapeSubType(inSubType) { }
									Shape(EShapeType inType, EShapeSubType inSubType, const ShapeSettings &inSettings, [[maybe_unused]] ShapeResult &outResult) : mUserData(inSettings.mUserData), mShapeType(inType), mShapeSubType(inSubType) { }

	/// Destructor
	virtual							~Shape() = default;

	/// Get type
	inline EShapeType				GetType() const														{ return mShapeType; }
	inline EShapeSubType			GetSubType() const													{ return mShapeSubType; }

	/// User data (to be used freely by the application)
	uint64							GetUserData() const													{ return mUserData; }
	void							SetUserData(uint64 inUserData)										{ mUserData = inUserData; }

	/// Check if this shape can only be used to create a static body or if it can also be dynamic/kinematic
	virtual bool					MustBeStatic() const												{ return false; }

	/// All shapes are centered around their center of mass. This function returns the center of mass position that needs to be applied to transform the shape to where it was created.
	virtual Vec3					GetCenterOfMass() const												{ return Vec3::sZero(); }

	/// Get local bounding box including convex radius, this box is centered around the center of mass rather than the world transform
	virtual AABox					GetLocalBounds() const = 0;

	/// Get the max number of sub shape ID bits that are needed to be able to address any leaf shape in this shape. Used mainly for checking that it is smaller or equal than SubShapeID::MaxBits.
	virtual uint					GetSubShapeIDBitsRecursive() const = 0;

	/// Get world space bounds including convex radius.
	/// This shape is scaled by inScale in local space first.
	/// This function can be overridden to return a closer fitting world space bounding box, by default it will just transform what GetLocalBounds() returns.
	virtual AABox					GetWorldSpaceBounds(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale) const { return GetLocalBounds().Scaled(inScale).Transformed(inCenterOfMassTransform); }

	/// Get world space bounds including convex radius.
	AABox							GetWorldSpaceBounds(DMat44Arg inCenterOfMassTransform, Vec3Arg inScale) const
	{
		// Use single precision version using the rotation only
		AABox bounds = GetWorldSpaceBounds(inCenterOfMassTransform.GetRotation(), inScale);

		// Apply translation
		bounds.Translate(inCenterOfMassTransform.GetTranslation());

		return bounds;
	}

	/// Returns the radius of the biggest sphere that fits entirely in the shape. In case this shape consists of multiple sub shapes, it returns the smallest sphere of the parts.
	/// This can be used as a measure of how far the shape can be moved without risking going through geometry.
	virtual float					GetInnerRadius() const = 0;

	/// Calculate the mass and inertia of this shape
	virtual MassProperties			GetMassProperties() const = 0;

	/// Get the leaf shape for a particular sub shape ID.
	/// @param inSubShapeID The full sub shape ID that indicates the path to the leaf shape
	/// @param outRemainder What remains of the sub shape ID after removing the path to the leaf shape (could e.g. refer to a triangle within a MeshShape)
	/// @return The shape or null if the sub shape ID is invalid
	virtual const Shape *			GetLeafShape([[maybe_unused]] const SubShapeID &inSubShapeID, SubShapeID &outRemainder) const;

	/// Get the material assigned to a particular sub shape ID
	virtual const PhysicsMaterial *	GetMaterial(const SubShapeID &inSubShapeID) const = 0;

	/// Get the surface normal of a particular sub shape ID and point on surface (all vectors are relative to center of mass for this shape).
	/// Note: When you have a CollideShapeResult or ShapeCastResult you should use -mPenetrationAxis.Normalized() as contact normal as GetSurfaceNormal will only return face normals (and not vertex or edge normals).
	virtual Vec3					GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const = 0;

	/// Type definition for a supporting face
	using SupportingFace = TStaticArray<Vec3, 32>;

	/// Get the vertices of the face that faces inDirection the most (includes any convex radius). Note that this function can only return faces of
	/// convex shapes or triangles, which is why a sub shape ID to get to that leaf must be provided.
	/// @param inSubShapeID Sub shape ID of target shape
	/// @param inDirection Direction that the face should be facing (in local space to this shape)
	/// @param inCenterOfMassTransform Transform to transform outVertices with
	/// @param inScale Scale in local space of the shape (scales relative to its center of mass)
	/// @param outVertices Resulting face. The returned face can be empty if the shape doesn't have polygons to return (e.g. because it's a sphere). The face will be returned in world space.
	virtual void					GetSupportingFace([[maybe_unused]] const SubShapeID &inSubShapeID, [[maybe_unused]] Vec3Arg inDirection, [[maybe_unused]] Vec3Arg inScale, [[maybe_unused]] Mat44Arg inCenterOfMassTransform, [[maybe_unused]] SupportingFace &outVertices) const { /* Nothing */ }

	/// Get the user data of a particular sub shape ID. Corresponds with the value stored in Shape::GetUserData of the leaf shape pointed to by inSubShapeID.
	virtual uint64					GetSubShapeUserData([[maybe_unused]] const SubShapeID &inSubShapeID) const			{ return mUserData; }

	/// Get the direct child sub shape and its transform for a sub shape ID.
	/// @param inSubShapeID Sub shape ID that indicates the path to the leaf shape
	/// @param inPositionCOM The position of the center of mass of this shape
	/// @param inRotation The orientation of this shape
	/// @param inScale Scale in local space of the shape (scales relative to its center of mass)
	/// @param outRemainder The remainder of the sub shape ID after removing the sub shape
	/// @return Direct child sub shape and its transform, note that the body ID and sub shape ID will be invalid
	virtual TransformedShape		GetSubShapeTransformedShape(const SubShapeID &inSubShapeID, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale, SubShapeID &outRemainder) const;

	/// Gets the properties needed to do buoyancy calculations for a body using this shape
	/// @param inCenterOfMassTransform Transform that takes this shape (centered around center of mass) to world space (or a desired other space)
	/// @param inScale Scale in local space of the shape (scales relative to its center of mass)
	/// @param inSurface The surface plane of the liquid relative to inCenterOfMassTransform
	/// @param outTotalVolume On return this contains the total volume of the shape
	/// @param outSubmergedVolume On return this contains the submerged volume of the shape
	/// @param outCenterOfBuoyancy On return this contains the world space center of mass of the submerged volume
#ifndef MOSS_DEBUG_RENDERER
	/// @param inBaseOffset The offset to transform inCenterOfMassTransform to world space (in double precision mode this can be used to shift the whole operation closer to the origin). Only used for debug drawing.
#endif
	virtual void					GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy
#ifndef MOSS_DEBUG_RENDERER // Not using MOSS_IF_DEBUG_RENDERER for Doxygen
		, RVec3Arg inBaseOffset
#endif
		) const = 0;

#ifndef MOSS_DEBUG_RENDERER
	/// Draw the shape at a particular location with a particular color (debugging purposes)
	virtual void					Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const = 0;

	/// Draw the results of the GetSupportFunction with the convex radius added back on to show any errors introduced by this process (only relevant for convex shapes)
	virtual void					DrawGetSupportFunction([[maybe_unused]] DebugRenderer *inRenderer, [[maybe_unused]] RMat44Arg inCenterOfMassTransform, [[maybe_unused]] Vec3Arg inScale, [[maybe_unused]] ColorArg inColor, [[maybe_unused]] bool inDrawSupportDirection) const { /* Only implemented for convex shapes */ }

	/// Draw the results of the GetSupportingFace function to show any errors introduced by this process (only relevant for convex shapes)
	virtual void					DrawGetSupportingFace([[maybe_unused]] DebugRenderer *inRenderer, [[maybe_unused]] RMat44Arg inCenterOfMassTransform, [[maybe_unused]] Vec3Arg inScale) const { /* Only implemented for convex shapes */ }
#endif // MOSS_DEBUG_RENDERER

	/// Cast a ray against this shape, returns true if it finds a hit closer than ioHit.mFraction and updates that fraction. Otherwise ioHit is left untouched and the function returns false.
	/// Note that the ray should be relative to the center of mass of this shape (i.e. subtract Shape::GetCenterOfMass() from RayCast::mOrigin if you want to cast against the shape in the space it was created).
	/// Convex objects will be treated as solid (meaning if the ray starts inside, you'll get a hit fraction of 0) and back face hits against triangles are returned.
	/// If you want the surface normal of the hit use GetSurfaceNormal(ioHit.mSubShapeID2, inRay.GetPointOnRay(ioHit.mFraction)).
	virtual bool					CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const = 0;

	/// Cast a ray against this shape. Allows returning multiple hits through ioCollector. Note that this version is more flexible but also slightly slower than the CastRay function that returns only a single hit.
	/// If you want the surface normal of the hit use GetSurfaceNormal(collected sub shape ID, inRay.GetPointOnRay(collected faction)).
	virtual void					CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const = 0;

	/// Check if inPoint is inside this shape. For this tests all shapes are treated as if they were solid.
	/// Note that inPoint should be relative to the center of mass of this shape (i.e. subtract Shape::GetCenterOfMass() from inPoint if you want to test against the shape in the space it was created).
	/// For a mesh shape, this test will only provide sensible information if the mesh is a closed manifold.
	/// For each shape that collides, ioCollector will receive a hit.
	virtual void					CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const = 0;

	/// Collides all vertices of a soft body with this shape and updates SoftBodyVertex::mCollisionPlane, SoftBodyVertex::mCollidingShapeIndex and SoftBodyVertex::mLargestPenetration if a collision with more penetration was found.
	/// @param inCenterOfMassTransform Center of mass transform for this shape relative to the vertices.
	/// @param inScale Scale in local space of the shape (scales relative to its center of mass)
	/// @param inVertices The vertices of the soft body
	/// @param inNumVertices The number of vertices in inVertices
	/// @param inCollidingShapeIndex Value to store in CollideSoftBodyVertexIterator::mCollidingShapeIndex when a collision was found
	virtual void					CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const = 0;

	/// Collect the leaf transformed shapes of all leaf shapes of this shape.
	/// inBox is the world space axis aligned box which leaf shapes should collide with.
	/// inPositionCOM/inRotation/inScale describes the transform of this shape.
	/// inSubShapeIDCreator represents the current sub shape ID of this shape.
	virtual void					CollectTransformedShapes(const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale, const SubShapeIDCreator &inSubShapeIDCreator, TransformedShapeCollector &ioCollector, const ShapeFilter &inShapeFilter) const;

	/// Transforms this shape and all of its children with inTransform, resulting shape(s) are passed to ioCollector.
	/// Note that not all shapes support all transforms (especially true for scaling), the resulting shape will try to match the transform as accurately as possible.
	/// @param inCenterOfMassTransform The transform (rotation, translation, scale) that the center of mass of the shape should get
	/// @param ioCollector The transformed shapes will be passed to this collector
	virtual void					TransformShape(Mat44Arg inCenterOfMassTransform, TransformedShapeCollector &ioCollector) const;

	/// Scale this shape. Note that not all shapes support all scales, this will return a shape that matches the scale as accurately as possible. See Shape::IsValidScale for more information.
	/// @param inScale The scale to use for this shape (note: this scale is applied to the entire shape in the space it was created, most other functions apply the scale in the space of the leaf shapes and from the center of mass!)
	ShapeResult						ScaleShape(Vec3Arg inScale) const;

	/// An opaque buffer that holds shape specific information during GetTrianglesStart/Next.
	struct alignas(16)				GetTrianglesContext { uint8 mData[4288]; };

	/// This is the minimum amount of triangles that should be requested through GetTrianglesNext.
	static constexpr int			cGetTrianglesMinTrianglesRequested = 32;

	/// To start iterating over triangles, call this function first.
	/// ioContext is a temporary buffer and should remain untouched until the last call to GetTrianglesNext.
	/// inBox is the world space bounding in which you want to get the triangles.
	/// inPositionCOM/inRotation/inScale describes the transform of this shape.
	/// To get the actual triangles call GetTrianglesNext.
	virtual void					GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const = 0;

	/// Call this repeatedly to get all triangles in the box.
	/// outTriangleVertices should be large enough to hold 3 * inMaxTriangleRequested entries.
	/// outMaterials (if it is not null) should contain inMaxTrianglesRequested entries.
	/// The function returns the amount of triangles that it found (which will be <= inMaxTrianglesRequested), or 0 if there are no more triangles.
	/// Note that the function can return a value < inMaxTrianglesRequested and still have more triangles to process (triangles can be returned in blocks).
	/// Note that the function may return triangles outside of the requested box, only coarse culling is performed on the returned triangles.
	virtual int						GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials = nullptr) const = 0;

	///@name Binary serialization of the shape. Note that this saves the 'cooked' shape in a format which will not be backwards compatible for newer library versions.
	/// In this case you need to recreate the shape from the ShapeSettings object and save it again. The user is expected to call SaveBinaryState followed by SaveMaterialState and SaveSubShapeState.
	/// The stream should be stored as is and the material and shape list should be saved using the applications own serialization system (e.g. by assigning an ID to each pointer).
	/// When restoring data, call sRestoreFromBinaryState to get the shape and then call RestoreMaterialState and RestoreSubShapeState to restore the pointers to the external objects.
	/// Alternatively you can use SaveWithChildren and sRestoreWithChildren to save and restore the shape and all its child shapes and materials in a single stream.
	///@{

	/// Saves the contents of the shape in binary form to inStream.
	virtual void					SaveBinaryState(StreamOut &inStream) const;

	/// Creates a Shape of the correct type and restores its contents from the binary stream inStream.
	static ShapeResult				sRestoreFromBinaryState(StreamIn &inStream);

	/// Outputs the material references that this shape has to outMaterials.
	virtual void					SaveMaterialState([[maybe_unused]] PhysicsMaterialList &outMaterials) const			{ /* By default do nothing */ }

	/// Restore the material references after calling sRestoreFromBinaryState. Note that the exact same materials need to be provided in the same order as returned by SaveMaterialState.
	virtual void					RestoreMaterialState([[maybe_unused]] const PhysicsMaterialRefC *inMaterials, [[maybe_unused]] uint inNumMaterials) { MOSS_ASSERT(inNumMaterials == 0); }

	/// Outputs the shape references that this shape has to outSubShapes.
	virtual void					SaveSubShapeState([[maybe_unused]] ShapeList &outSubShapes) const					{ /* By default do nothing */ }

	/// Restore the shape references after calling sRestoreFromBinaryState. Note that the exact same shapes need to be provided in the same order as returned by SaveSubShapeState.
	virtual void					RestoreSubShapeState([[maybe_unused]] const ShapeRefC *inSubShapes, [[maybe_unused]] uint inNumShapes) { MOSS_ASSERT(inNumShapes == 0); }

	using ShapeToIDMap = StreamUtils::ObjectToIDMap<Shape>;
	using IDToShapeMap = StreamUtils::IDToObjectMap<Shape>;
	using MaterialToIDMap = StreamUtils::ObjectToIDMap<PhysicsMaterial>;
	using IDToMaterialMap = StreamUtils::IDToObjectMap<PhysicsMaterial>;

	/// Save this shape, all its children and its materials. Pass in an empty map in ioShapeMap / ioMaterialMap or reuse the same map while saving multiple shapes to the same stream in order to avoid writing duplicates.
	void							SaveWithChildren(StreamOut &inStream, ShapeToIDMap &ioShapeMap, MaterialToIDMap &ioMaterialMap) const;

	/// Restore a shape, all its children and materials. Pass in an empty map in ioShapeMap / ioMaterialMap or reuse the same map while reading multiple shapes from the same stream in order to restore duplicates.
	static ShapeResult				sRestoreWithChildren(StreamIn &inStream, IDToShapeMap &ioShapeMap, IDToMaterialMap &ioMaterialMap);

	///@}

	/// Class that holds information about the shape that can be used for logging / data collection purposes
	struct Stats
	{
									Stats(size_t inSizeBytes, uint inNumTriangles) : mSizeBytes(inSizeBytes), mNumTriangles(inNumTriangles) { }

		size_t						mSizeBytes;				///< Amount of memory used by this shape (size in bytes)
		uint						mNumTriangles;			///< Number of triangles in this shape (when applicable)
	};

	/// Get stats of this shape. Use for logging / data collection purposes only. Does not add values from child shapes, use GetStatsRecursive for this.
	virtual Stats					GetStats() const = 0;

	using VisitedShapes = TSet<const Shape *>;

	/// Get the combined stats of this shape and its children.
	/// @param ioVisitedShapes is used to track which shapes have already been visited, to avoid calculating the wrong memory size.
	virtual Stats					GetStatsRecursive(VisitedShapes &ioVisitedShapes) const;

	///< Volume of this shape (m^3). Note that for compound shapes the volume may be incorrect since child shapes can overlap which is not accounted for.
	virtual float					GetVolume() const = 0;

	/// Test if inScale is a valid scale for this shape. Some shapes can only be scaled uniformly, compound shapes cannot handle shapes
	/// being rotated and scaled (this would cause shearing), scale can never be zero. When the scale is invalid, the function will return false.
	///
	/// Here's a list of supported scales:
	/// * SphereShape: Scale must be uniform (signs of scale are ignored).
	/// * BoxShape: Any scale supported (signs of scale are ignored).
	/// * TriangleShape: Any scale supported when convex radius is zero, otherwise only uniform scale supported.
	/// * CapsuleShape: Scale must be uniform (signs of scale are ignored).
	/// * TaperedCapsuleShape: Scale must be uniform (sign of Y scale can be used to flip the capsule).
	/// * CylinderShape: Scale must be uniform in XZ plane, Y can scale independently (signs of scale are ignored).
	/// * RotatedTranslatedShape: Scale must not cause shear in the child shape.
	/// * CompoundShape: Scale must not cause shear in any of the child shapes.
	virtual bool					IsValidScale(Vec3Arg inScale) const;

	/// This function will make sure that if you wrap this shape in a ScaledShape that the scale is valid.
	/// Note that this involves discarding components of the scale that are invalid, so the resulting scaled shape may be different than the requested scale.
	/// Compare the return value of this function with the scale you passed in to detect major inconsistencies and possibly warn the user.
	/// @param inScale Local space scale for this shape.
	/// @return Scale that can be used to wrap this shape in a ScaledShape. IsValidScale will return true for this scale.
	virtual Vec3					MakeScaleValid(Vec3Arg inScale) const;

#ifndef MOSS_DEBUG_RENDERER
	/// Debug helper which draws the intersection between water and the shapes, the center of buoyancy and the submerged volume
	static bool						sDrawSubmergedVolumes;
#endif // MOSS_DEBUG_RENDERER

protected:
	/// This function should not be called directly, it is used by sRestoreFromBinaryState.
	virtual void					RestoreBinaryState(StreamIn &inStream);

	/// A fallback version of CollidePoint that uses a ray cast and counts the number of hits to determine if the point is inside the shape. Odd number of hits means inside, even number of hits means outside.
	static void						sCollidePointUsingRayCast(const Shape &inShape, Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter);

private:
	uint64							mUserData = 0;
	EShapeType						mShapeType;
	EShapeSubType					mShapeSubType;
};


/// Class that constructs a CapsuleShape
class MOSS_EXPORT CapsuleShapeSettings final : public ConvexShapeSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, CapsuleShapeSettings)

public:
	/// Default constructor for deserialization
							CapsuleShapeSettings() = default;

	/// Create a capsule centered around the origin with one sphere cap at (0, -inHalfHeightOfCylinder, 0) and the other at (0, inHalfHeightOfCylinder, 0)
							CapsuleShapeSettings(float inHalfHeightOfCylinder, float inRadius, const PhysicsMaterial *inMaterial = nullptr) : ConvexShapeSettings(inMaterial), mRadius(inRadius), mHalfHeightOfCylinder(inHalfHeightOfCylinder) { }

	/// Check if this is a valid capsule shape
	bool					IsValid() const															{ return mRadius > 0.0f && mHalfHeightOfCylinder >= 0.0f; }

	/// Checks if the settings of this capsule make this shape a sphere
	bool					IsSphere() const														{ return mHalfHeightOfCylinder == 0.0f; }

	// See: ShapeSettings
	virtual ShapeResult		Create() const override;

	float					mRadius = 0.0f;
	float					mHalfHeightOfCylinder = 0.0f;
};

/// A capsule, implemented as a line segment with convex radius
class MOSS_EXPORT CapsuleShape final : public ConvexShape
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
							CapsuleShape() : ConvexShape(EShapeSubType::Capsule) { }
							CapsuleShape(const CapsuleShapeSettings &inSettings, ShapeResult &outResult);

	/// Create a capsule centered around the origin with one sphere cap at (0, -inHalfHeightOfCylinder, 0) and the other at (0, inHalfHeightOfCylinder, 0)
							CapsuleShape(float inHalfHeightOfCylinder, float inRadius, const PhysicsMaterial *inMaterial = nullptr) : ConvexShape(EShapeSubType::Capsule, inMaterial), mRadius(inRadius), mHalfHeightOfCylinder(inHalfHeightOfCylinder) { MOSS_ASSERT(inHalfHeightOfCylinder > 0.0f); MOSS_ASSERT(inRadius > 0.0f); }

	/// Radius of the cylinder
	float					GetRadius() const														{ return mRadius; }

	/// Get half of the height of the cylinder
	float					GetHalfHeightOfCylinder() const											{ return mHalfHeightOfCylinder; }

	// See Shape::GetLocalBounds
	virtual AABox			GetLocalBounds() const override;

	// See Shape::GetWorldSpaceBounds
	virtual AABox			GetWorldSpaceBounds(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale) const override;
	using Shape::GetWorldSpaceBounds;

	// See Shape::GetInnerRadius
	virtual float			GetInnerRadius() const override											{ return mRadius; }

	// See Shape::GetMassProperties
	virtual MassProperties	GetMassProperties() const override;

	// See Shape::GetSurfaceNormal
	virtual Vec3			GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const override;

	// See Shape::GetSupportingFace
	virtual void			GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const override;

	// See ConvexShape::GetSupportFunction
	virtual const Support *	GetSupportFunction(ESupportMode inMode, SupportBuffer &inBuffer, Vec3Arg inScale) const override;

#ifndef MOSS_DEBUG_RENDERER
	// See Shape::Draw
	virtual void			Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const override;
#endif // MOSS_DEBUG_RENDERER

	// See Shape::CastRay
	using ConvexShape::CastRay;
	virtual bool			CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const override;

	// See: Shape::CollidePoint
	virtual void			CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;

	// See: Shape::CollideSoftBodyVertices
	virtual void			CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const override;

	// See Shape::GetTrianglesStart
	virtual void			GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const override;

	// See Shape::GetTrianglesNext
	virtual int				GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials = nullptr) const override;

	// See Shape
	virtual void			SaveBinaryState(StreamOut &inStream) const override;

	// See Shape::GetStats
	virtual Stats			GetStats() const override												{ return Stats(sizeof(*this), 0); }

	// See Shape::GetVolume
	virtual float			GetVolume() const override												{ return 4.0f / 3.0f * MOSS_PI * Cubed(mRadius) + 2.0f * MOSS_PI * mHalfHeightOfCylinder * Square(mRadius); }

	// See Shape::IsValidScale
	virtual bool			IsValidScale(Vec3Arg inScale) const override;

	// See Shape::MakeScaleValid
	virtual Vec3			MakeScaleValid(Vec3Arg inScale) const override;

	// Register shape functions with the registry
	static void				sRegister();

protected:
	// See: Shape::RestoreBinaryState
	virtual void			RestoreBinaryState(StreamIn &inStream) override;

private:
	// Classes for GetSupportFunction
	class					CapsuleNoConvex;
	class					CapsuleWithConvex;

	float					mRadius = 0.0f;
	float					mHalfHeightOfCylinder = 0.0f;
};






















/// Class that constructs a CylinderShape
class MOSS_EXPORT CylinderShapeSettings final : public ConvexShapeSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, CylinderShapeSettings)

public:
	/// Default constructor for deserialization
							CylinderShapeSettings() = default;

	/// Create a shape centered around the origin with one top at (0, -inHalfHeight, 0) and the other at (0, inHalfHeight, 0) and radius inRadius.
	/// (internally the convex radius will be subtracted from the cylinder the total cylinder will not grow with the convex radius, but the edges of the cylinder will be rounded a bit).
							CylinderShapeSettings(float inHalfHeight, float inRadius, float inConvexRadius = cDefaultConvexRadius, const PhysicsMaterial *inMaterial = nullptr) : ConvexShapeSettings(inMaterial), mHalfHeight(inHalfHeight), mRadius(inRadius), mConvexRadius(inConvexRadius) { }

	// See: ShapeSettings
	virtual ShapeResult		Create() const override;

	float					mHalfHeight = 0.0f;
	float					mRadius = 0.0f;
	float					mConvexRadius = 0.0f;
};

/// A cylinder
class MOSS_EXPORT CylinderShape final : public ConvexShape
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
							CylinderShape() : ConvexShape(EShapeSubType::Cylinder) { }
							CylinderShape(const CylinderShapeSettings &inSettings, ShapeResult &outResult);

	/// Create a shape centered around the origin with one top at (0, -inHalfHeight, 0) and the other at (0, inHalfHeight, 0) and radius inRadius.
	/// (internally the convex radius will be subtracted from the cylinder the total cylinder will not grow with the convex radius, but the edges of the cylinder will be rounded a bit).
							CylinderShape(float inHalfHeight, float inRadius, float inConvexRadius = cDefaultConvexRadius, const PhysicsMaterial *inMaterial = nullptr);

	/// Get half height of cylinder
	float					GetHalfHeight() const														{ return mHalfHeight; }

	/// Get radius of cylinder
	float					GetRadius() const															{ return mRadius; }

	// See Shape::GetLocalBounds
	virtual AABox			GetLocalBounds() const override;

	// See Shape::GetInnerRadius
	virtual float			GetInnerRadius() const override												{ return min(mHalfHeight, mRadius); }

	// See Shape::GetMassProperties
	virtual MassProperties	GetMassProperties() const override;

	// See Shape::GetSurfaceNormal
	virtual Vec3			GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const override;

	// See Shape::GetSupportingFace
	virtual void			GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const override;

	// See ConvexShape::GetSupportFunction
	virtual const Support *	GetSupportFunction(ESupportMode inMode, SupportBuffer &inBuffer, Vec3Arg inScale) const override;

#ifndef MOSS_DEBUG_RENDERER
	// See Shape::Draw
	virtual void			Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const override;
#endif // MOSS_DEBUG_RENDERER

	// See Shape::CastRay
	using ConvexShape::CastRay;
	virtual bool			CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const override;

	// See: Shape::CollidePoint
	virtual void			CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;

	// See: Shape::CollideSoftBodyVertices
	virtual void			CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const override;

	// See Shape::GetTrianglesStart
	virtual void			GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const override;

	// See Shape::GetTrianglesNext
	virtual int				GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials = nullptr) const override;

	// See Shape
	virtual void			SaveBinaryState(StreamOut &inStream) const override;

	// See Shape::GetStats
	virtual Stats			GetStats() const override												{ return Stats(sizeof(*this), 0); }

	// See Shape::GetVolume
	virtual float			GetVolume() const override												{ return 2.0f * MOSS_PI * mHalfHeight * Square(mRadius); }

	/// Get the convex radius of this cylinder
	float					GetConvexRadius() const													{ return mConvexRadius; }

	// See Shape::IsValidScale
	virtual bool			IsValidScale(Vec3Arg inScale) const override;

	// See Shape::MakeScaleValid
	virtual Vec3			MakeScaleValid(Vec3Arg inScale) const override;

	// Register shape functions with the registry
	static void				sRegister();

protected:
	// See: Shape::RestoreBinaryState
	virtual void			RestoreBinaryState(StreamIn &inStream) override;

private:
	// Class for GetSupportFunction
	class					Cylinder;

	float					mHalfHeight = 0.0f;
	float					mRadius = 0.0f;
	float					mConvexRadius = 0.0f;
};

// Class that constructs a PlaneShape
class MOSS_EXPORT PlaneShapeSettings final : public ShapeSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, PlaneShapeSettings)

public:
	/// Default constructor for deserialization
									PlaneShapeSettings() = default;

	/// Create a plane shape.
									PlaneShapeSettings(const Plane &inPlane, const PhysicsMaterial *inMaterial = nullptr, float inHalfExtent = cDefaultHalfExtent) : mPlane(inPlane), mMaterial(inMaterial), mHalfExtent(inHalfExtent) { }

	// See: ShapeSettings
	virtual ShapeResult				Create() const override;

	Plane							mPlane;														///< Plane that describes the shape. The negative half space is considered solid.

	RefConst<PhysicsMaterial>		mMaterial;													///< Surface material of the plane

	static constexpr float			cDefaultHalfExtent = 1000.0f;								///< Default half-extent of the plane (total size along 1 axis will be 2 * half-extent)

	float							mHalfExtent = cDefaultHalfExtent;							///< The bounding box of this plane will run from [-half_extent, half_extent]. Keep this as low as possible for better broad phase performance.
};

/// A plane shape. The negative half space is considered solid. Planes cannot be dynamic objects, only static or kinematic.
/// The plane is considered an infinite shape, but testing collision outside of its bounding box (defined by the half-extent parameter) will not return a collision result.
/// At the edge of the bounding box collision with the plane will be inconsistent. If you need something of a well defined size, a box shape may be better.
class MOSS_EXPORT PlaneShape final : public Shape
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
									PlaneShape() : Shape(EShapeType::Plane, EShapeSubType::Plane) { }
									PlaneShape(const Plane &inPlane, const PhysicsMaterial *inMaterial = nullptr, float inHalfExtent = PlaneShapeSettings::cDefaultHalfExtent) : Shape(EShapeType::Plane, EShapeSubType::Plane), mPlane(inPlane), mMaterial(inMaterial), mHalfExtent(inHalfExtent) { CalculateLocalBounds(); }
									PlaneShape(const PlaneShapeSettings &inSettings, ShapeResult &outResult);

	/// Get the plane
	const Plane &					GetPlane() const											{ return mPlane; }

	/// Get the half-extent of the bounding box of the plane
	float							GetHalfExtent() const										{ return mHalfExtent; }

	// See Shape::MustBeStatic
	virtual bool					MustBeStatic() const override								{ return true; }

	// See Shape::GetLocalBounds
	virtual AABox					GetLocalBounds() const override								{ return mLocalBounds; }

	// See Shape::GetSubShapeIDBitsRecursive
	virtual uint					GetSubShapeIDBitsRecursive() const override					{ return 0; }

	// See Shape::GetInnerRadius
	virtual float					GetInnerRadius() const override								{ return 0.0f; }

	// See Shape::GetMassProperties
	virtual MassProperties			GetMassProperties() const override;

	// See Shape::GetMaterial
	virtual const PhysicsMaterial *	GetMaterial(const SubShapeID &inSubShapeID) const override	{ MOSS_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID"); return GetMaterial(); }

	// See Shape::GetSurfaceNormal
	virtual Vec3					GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const override { MOSS_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID"); return mPlane.GetNormal(); }

	// See Shape::GetSupportingFace
	virtual void					GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const override;

#ifndef MOSS_DEBUG_RENDERER
	// See Shape::Draw
	virtual void					Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const override;
#endif // MOSS_DEBUG_RENDERER

	// See Shape::CastRay
	virtual bool					CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const override;
	virtual void					CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;

	// See: Shape::CollidePoint
	virtual void					CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;

	// See: Shape::CollideSoftBodyVertices
	virtual void					CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const override;

	// See Shape::GetTrianglesStart
	virtual void					GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const override;

	// See Shape::GetTrianglesNext
	virtual int						GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials = nullptr) const override;

	// See Shape::GetSubmergedVolume
	virtual void					GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy MOSS_IF_DEBUG_RENDERER(, RVec3Arg inBaseOffset)) const override { MOSS_ASSERT(false, "Not supported"); }

	// See Shape
	virtual void					SaveBinaryState(StreamOut &inStream) const override;
	virtual void					SaveMaterialState(PhysicsMaterialList &outMaterials) const override;
	virtual void					RestoreMaterialState(const PhysicsMaterialRefC *inMaterials, uint inNumMaterials) override;

	// See Shape::GetStats
	virtual Stats					GetStats() const override									{ return Stats(sizeof(*this), 0); }

	// See Shape::GetVolume
	virtual float					GetVolume() const override									{ return 0; }

	/// Material of the shape
	void							SetMaterial(const PhysicsMaterial *inMaterial)				{ mMaterial = inMaterial; }
	const PhysicsMaterial *			GetMaterial() const											{ return mMaterial != nullptr? mMaterial : PhysicsMaterial::sDefault; }

	// Register shape functions with the registry
	static void						sRegister();

protected:
	// See: Shape::RestoreBinaryState
	virtual void					RestoreBinaryState(StreamIn &inStream) override;

private:
	struct							PSGetTrianglesContext;										///< Context class for GetTrianglesStart/Next

	// Get 4 vertices that form the plane
	void							GetVertices(Vec3 *outVertices) const;

	// Cache the local bounds
	void							CalculateLocalBounds();

	// Helper functions called by CollisionDispatch
	static void						sCollideConvexVsPlane(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter);
	static void						sCastConvexVsPlane(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector);

	Plane							mPlane;
	RefConst<PhysicsMaterial>		mMaterial;
	float							mHalfExtent;
	AABox							mLocalBounds;
};

// Class that constructs an OffsetCenterOfMassShape
class MOSS_EXPORT OffsetCenterOfMassShapeSettings final : public DecoratedShapeSettings {
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, OffsetCenterOfMassShapeSettings)
public:
	/// Constructor
									OffsetCenterOfMassShapeSettings() = default;

	/// Construct with shape settings, can be serialized.
									OffsetCenterOfMassShapeSettings(Vec3Arg inOffset, const ShapeSettings *inShape) : DecoratedShapeSettings(inShape), mOffset(inOffset) { }

	/// Variant that uses a concrete shape, which means this object cannot be serialized.
									OffsetCenterOfMassShapeSettings(Vec3Arg inOffset, const Shape *inShape): DecoratedShapeSettings(inShape), mOffset(inOffset) { }

	// See: ShapeSettings
	virtual ShapeResult				Create() const override;

	Vec3							mOffset;												///< Offset to be applied to the center of mass of the child shape
};

// Class that constructs a DecoratedShape
class MOSS_EXPORT DecoratedShapeSettings : public ShapeSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, DecoratedShapeSettings)

public:
	/// Default constructor for deserialization
									DecoratedShapeSettings() = default;

	/// Constructor that decorates another shape
	explicit						DecoratedShapeSettings(const ShapeSettings *inShape)	: mInnerShape(inShape) { }
	explicit						DecoratedShapeSettings(const Shape *inShape)			: mInnerShapePtr(inShape) { }

	RefConst<ShapeSettings>			mInnerShape;											///< Sub shape (either this or mShapePtr needs to be filled up)
	RefConst<Shape>					mInnerShapePtr;											///< Sub shape (either this or mShape needs to be filled up)
};

/// Base class for shapes that decorate another shape with extra functionality (e.g. scale, translation etc.)
class MOSS_EXPORT DecoratedShape : public Shape
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
	explicit						DecoratedShape(EShapeSubType inSubType) : Shape(EShapeType::Decorated, inSubType) { }
									DecoratedShape(EShapeSubType inSubType, const Shape *inInnerShape) : Shape(EShapeType::Decorated, inSubType), mInnerShape(inInnerShape) { }
									DecoratedShape(EShapeSubType inSubType, const DecoratedShapeSettings &inSettings, ShapeResult &outResult);

	/// Access to the decorated inner shape
	const Shape *					GetInnerShape() const									{ return mInnerShape; }

	// See Shape::MustBeStatic
	virtual bool					MustBeStatic() const override							{ return mInnerShape->MustBeStatic(); }

	// See Shape::GetCenterOfMass
	virtual Vec3					GetCenterOfMass() const override						{ return mInnerShape->GetCenterOfMass(); }

	// See Shape::GetSubShapeIDBitsRecursive
	virtual uint					GetSubShapeIDBitsRecursive() const override				{ return mInnerShape->GetSubShapeIDBitsRecursive(); }

	// See Shape::GetLeafShape
	virtual const Shape *			GetLeafShape(const SubShapeID &inSubShapeID, SubShapeID &outRemainder) const override { return mInnerShape->GetLeafShape(inSubShapeID, outRemainder); }

	// See Shape::GetMaterial
	virtual const PhysicsMaterial *	GetMaterial(const SubShapeID &inSubShapeID) const override;

	// See Shape::GetSupportingFace
	virtual void					GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const override;

	// See Shape::GetSubShapeUserData
	virtual uint64					GetSubShapeUserData(const SubShapeID &inSubShapeID) const override;

	// See Shape
	virtual void					SaveSubShapeState(ShapeList &outSubShapes) const override;
	virtual void					RestoreSubShapeState(const ShapeRefC *inSubShapes, uint inNumShapes) override;

	// See Shape::GetStatsRecursive
	virtual Stats					GetStatsRecursive(VisitedShapes &ioVisitedShapes) const override;

	// See Shape::IsValidScale
	virtual bool					IsValidScale(Vec3Arg inScale) const override			{ return mInnerShape->IsValidScale(inScale); }

	// See Shape::MakeScaleValid
	virtual Vec3					MakeScaleValid(Vec3Arg inScale) const override			{ return mInnerShape->MakeScaleValid(inScale); }

protected:
	RefConst<Shape>					mInnerShape;
};

/// This shape will shift the center of mass of a child shape, it can e.g. be used to lower the center of mass of an unstable object like a boat to make it stable
class MOSS_EXPORT OffsetCenterOfMassShape final : public DecoratedShape
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
									OffsetCenterOfMassShape() : DecoratedShape(EShapeSubType::OffsetCenterOfMass) { }
									OffsetCenterOfMassShape(const OffsetCenterOfMassShapeSettings &inSettings, ShapeResult &outResult);
									OffsetCenterOfMassShape(const Shape *inShape, Vec3Arg inOffset) : DecoratedShape(EShapeSubType::OffsetCenterOfMass, inShape), mOffset(inOffset) { }

	/// Access the offset that is applied to the center of mass
	Vec3							GetOffset() const										{ return mOffset; }

	// See Shape::GetCenterOfMass
	virtual Vec3					GetCenterOfMass() const override						{ return mInnerShape->GetCenterOfMass() + mOffset; }

	// See Shape::GetLocalBounds
	virtual AABox					GetLocalBounds() const override;

	// See Shape::GetWorldSpaceBounds
	virtual AABox					GetWorldSpaceBounds(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale) const override;
	using Shape::GetWorldSpaceBounds;

	// See Shape::GetInnerRadius
	virtual float					GetInnerRadius() const override							{ return mInnerShape->GetInnerRadius(); }

	// See Shape::GetMassProperties
	virtual MassProperties			GetMassProperties() const override
	{
		MassProperties mp = mInnerShape->GetMassProperties();
		mp.Translate(mOffset);
		return mp;
	}

	// See Shape::GetSubShapeTransformedShape
	virtual TransformedShape		GetSubShapeTransformedShape(const SubShapeID &inSubShapeID, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale, SubShapeID &outRemainder) const override;

	// See Shape::GetSurfaceNormal
	virtual Vec3					GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const override;

	// See Shape::GetSupportingFace
	virtual void					GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const override;

	// See Shape::GetSubmergedVolume
	virtual void					GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy MOSS_IF_DEBUG_RENDERER(, RVec3Arg inBaseOffset)) const override;

#ifndef MOSS_DEBUG_RENDERER
	// See Shape::Draw
	virtual void					Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const override;

	// See Shape::DrawGetSupportFunction
	virtual void					DrawGetSupportFunction(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inDrawSupportDirection) const override;

	// See Shape::DrawGetSupportingFace
	virtual void					DrawGetSupportingFace(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale) const override;
#endif // MOSS_DEBUG_RENDERER

	// See Shape::CastRay
	virtual bool					CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const override;
	virtual void					CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;

	// See: Shape::CollidePoint
	virtual void					CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;

	// See: Shape::CollideSoftBodyVertices
	virtual void					CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const override;

	// See Shape::CollectTransformedShapes
	virtual void					CollectTransformedShapes(const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale, const SubShapeIDCreator &inSubShapeIDCreator, TransformedShapeCollector &ioCollector, const ShapeFilter &inShapeFilter) const override;

	// See Shape::TransformShape
	virtual void					TransformShape(Mat44Arg inCenterOfMassTransform, TransformedShapeCollector &ioCollector) const override;

	// See Shape::GetTrianglesStart
	virtual void					GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const override { MOSS_ASSERT(false, "Cannot call on non-leaf shapes, use CollectTransformedShapes to collect the leaves first!"); }

	// See Shape::GetTrianglesNext
	virtual int						GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials = nullptr) const override { MOSS_ASSERT(false, "Cannot call on non-leaf shapes, use CollectTransformedShapes to collect the leaves first!"); return 0; }

	// See Shape
	virtual void					SaveBinaryState(StreamOut &inStream) const override;

	// See Shape::GetStats
	virtual Stats					GetStats() const override								{ return Stats(sizeof(*this), 0); }

	// See Shape::GetVolume
	virtual float					GetVolume() const override								{ return mInnerShape->GetVolume(); }

	// Register shape functions with the registry
	static void						sRegister();

protected:
	// See: Shape::RestoreBinaryState
	virtual void					RestoreBinaryState(StreamIn &inStream) override;

private:
	// Helper functions called by CollisionDispatch
	static void						sCollideOffsetCenterOfMassVsShape(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter);
	static void						sCollideShapeVsOffsetCenterOfMass(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter);
	static void						sCastOffsetCenterOfMassVsShape(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector);
	static void						sCastShapeVsOffsetCenterOfMass(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector);

	Vec3							mOffset;												///< Offset of the center of mass
};

// Constants for HeightFieldShape, this was moved out of the HeightFieldShape because of a linker bug
namespace HeightFieldShapeConstants
{
	/// Value used to create gaps in the height field
	constexpr float					cNoCollisionValue = FLT_MAX;

	/// Stack size to use during WalkHeightField
	constexpr int					cStackSize = 128;

	/// A position in the hierarchical grid is defined by a level (which grid), x and y position. We encode this in a single uint32 as: level << 28 | y << 14 | x
	constexpr uint					cNumBitsXY = 14;
	constexpr uint					cMaskBitsXY = (1 << cNumBitsXY) - 1;
	constexpr uint					cLevelShift = 2 * cNumBitsXY;

	/// When height samples are converted to 16 bit:
	constexpr uint16				cNoCollisionValue16 = 0xffff;				///< This is the magic value for 'no collision'
	constexpr uint16				cMaxHeightValue16 = 0xfffe;					///< This is the maximum allowed height value
};


/// Class that constructs a HeightFieldShape
class MOSS_EXPORT HeightFieldShapeSettings final : public ShapeSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, HeightFieldShapeSettings)

public:
	/// Default constructor for deserialization
									HeightFieldShapeSettings() = default;

	/// Create a height field shape of inSampleCount * inSampleCount vertices.
	/// The height field is a surface defined by: inOffset + inScale * (x, inSamples[y * inSampleCount + x], y).
	/// where x and y are integers in the range x and y e [0, inSampleCount - 1].
	/// inSampleCount: inSampleCount / mBlockSize must be minimally 2 and a power of 2 is the most efficient in terms of performance and storage.
	/// inSamples: inSampleCount^2 vertices.
	/// inMaterialIndices: (inSampleCount - 1)^2 indices that index into inMaterialList.
									HeightFieldShapeSettings(const float *inSamples, Vec3Arg inOffset, Vec3Arg inScale, uint32 inSampleCount, const uint8 *inMaterialIndices = nullptr, const PhysicsMaterialList &inMaterialList = PhysicsMaterialList());

	// See: ShapeSettings
	virtual ShapeResult				Create() const override;

	/// Determine the minimal and maximal value of mHeightSamples (will ignore cNoCollisionValue)
	/// @param outMinValue The minimal value of mHeightSamples or FLT_MAX if no samples have collision
	/// @param outMaxValue The maximal value of mHeightSamples or -FLT_MAX if no samples have collision
	/// @param outQuantizationScale (value - outMinValue) * outQuantizationScale quantizes a height sample to 16 bits
	void							DetermineMinAndMaxSample(float &outMinValue, float &outMaxValue, float &outQuantizationScale) const;

	/// Given mBlockSize, mSampleCount and mHeightSamples, calculate the amount of bits needed to stay below absolute error inMaxError
	/// @param inMaxError Maximum allowed error in mHeightSamples after compression (note that this does not take mScale.Y into account)
	/// @return Needed bits per sample in the range [1, 8].
	uint32							CalculateBitsPerSampleForError(float inMaxError) const;

	/// The height field is a surface defined by: mOffset + mScale * (x, mHeightSamples[y * mSampleCount + x], y).
	/// where x and y are integers in the range x and y e [0, mSampleCount - 1].
	Vec3							mOffset = Vec3::sZero();
	Vec3							mScale = Vec3::sOne();
	uint32							mSampleCount = 0;

	/// Artificial minimal value of mHeightSamples, used for compression and can be used to update the terrain after creating with lower height values. If there are any lower values in mHeightSamples, this value will be ignored.
	float							mMinHeightValue = cLargeFloat;

	/// Artificial maximum value of mHeightSamples, used for compression and can be used to update the terrain after creating with higher height values. If there are any higher values in mHeightSamples, this value will be ignored.
	float							mMaxHeightValue = -cLargeFloat;

	/// When bigger than mMaterials.size() the internal material list will be preallocated to support this number of materials.
	/// This avoids reallocations when calling HeightFieldShape::SetMaterials with new materials later.
	uint32							mMaterialsCapacity = 0;

	/// The heightfield is divided in blocks of mBlockSize * mBlockSize * 2 triangles and the acceleration structure culls blocks only,
	/// bigger block sizes reduce memory consumption but also reduce query performance. Sensible values are [2, 8], does not need to be
	/// a power of 2. Note that at run-time we'll perform one more grid subdivision, so the effective block size is half of what is provided here.
	uint32							mBlockSize = 2;

	/// How many bits per sample to use to compress the height field. Can be in the range [1, 8].
	/// Note that each sample is compressed relative to the min/max value of its block of mBlockSize * mBlockSize pixels so the effective precision is higher.
	/// Also note that increasing mBlockSize saves more memory than reducing the amount of bits per sample.
	uint32							mBitsPerSample = 8;

	/// An array of mSampleCount^2 height samples. Samples are stored in row major order, so the sample at (x, y) is at index y * mSampleCount + x.
	TArray<float>					mHeightSamples;

	/// An array of (mSampleCount - 1)^2 material indices.
	TArray<uint8>					mMaterialIndices;

	/// The materials of square at (x, y) is: mMaterials[mMaterialIndices[x + y * (mSampleCount - 1)]]
	PhysicsMaterialList				mMaterials;

	/// Cosine of the threshold angle (if the angle between the two triangles is bigger than this, the edge is active, note that a concave edge is always inactive).
	/// Setting this value too small can cause ghost collisions with edges, setting it too big can cause depenetration artifacts (objects not depenetrating quickly).
	/// Valid ranges are between cos(0 degrees) and cos(90 degrees). The default value is cos(5 degrees).
	float							mActiveEdgeCosThresholdAngle = 0.996195f;	// cos(5 degrees)
};

/// A height field shape. Cannot be used as a dynamic object.
///
/// Note: If you're using HeightFieldShape and are querying data while modifying the shape you'll have a race condition.
/// In this case it is best to create a new HeightFieldShape using the Clone function. You replace the shape on a body using BodyInterface::SetShape.
/// If a query is still working on the old shape, it will have taken a reference and keep the old shape alive until the query finishes.
class MOSS_EXPORT HeightFieldShape final : public Shape
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
									HeightFieldShape() : Shape(EShapeType::HeightField, EShapeSubType::HeightField) { }
									HeightFieldShape(const HeightFieldShapeSettings &inSettings, ShapeResult &outResult);
	virtual							~HeightFieldShape() override;

	/// Clone this shape. Can be used to avoid race conditions. See the documentation of this class for more information.
	Ref<HeightFieldShape>			Clone() const;

	// See Shape::MustBeStatic
	virtual bool					MustBeStatic() const override				{ return true; }

	/// Get the size of the height field. Note that this will always be rounded up to the nearest multiple of GetBlockSize().
	inline uint						GetSampleCount() const						{ return mSampleCount; }

	/// Get the size of a block
	inline uint						GetBlockSize() const						{ return mBlockSize; }

	// See Shape::GetLocalBounds
	virtual AABox					GetLocalBounds() const override;

	// See Shape::GetSubShapeIDBitsRecursive
	virtual uint					GetSubShapeIDBitsRecursive() const override	{ return GetSubShapeIDBits(); }

	// See Shape::GetInnerRadius
	virtual float					GetInnerRadius() const override				{ return 0.0f; }

	// See Shape::GetMassProperties
	virtual MassProperties			GetMassProperties() const override;

	// See Shape::GetMaterial
	virtual const PhysicsMaterial *	GetMaterial(const SubShapeID &inSubShapeID) const override;

	/// Overload to get the material at a particular location
	const PhysicsMaterial *			GetMaterial(uint inX, uint inY) const;

	// See Shape::GetSurfaceNormal
	virtual Vec3					GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const override;

	// See Shape::GetSupportingFace
	virtual void					GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const override;

	// See Shape::GetSubmergedVolume
	virtual void					GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy MOSS_IF_DEBUG_RENDERER(, RVec3Arg inBaseOffset)) const override { MOSS_ASSERT(false, "Not supported"); }

#ifndef MOSS_DEBUG_RENDERER
	// See Shape::Draw
	virtual void					Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const override;
#endif // MOSS_DEBUG_RENDERER

	// See Shape::CastRay
	virtual bool					CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const override;
	virtual void					CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;

	// See: Shape::CollidePoint
	virtual void					CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;

	// See: Shape::CollideSoftBodyVertices
	virtual void					CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const override;

	// See Shape::GetTrianglesStart
	virtual void					GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const override;

	// See Shape::GetTrianglesNext
	virtual int						GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials = nullptr) const override;

	/// Get height field position at sampled location (inX, inY).
	/// where inX and inY are integers in the range inX e [0, mSampleCount - 1] and inY e [0, mSampleCount - 1].
	Vec3							GetPosition(uint inX, uint inY) const;

	/// Check if height field at sampled location (inX, inY) has collision (has a hole or not)
	bool							IsNoCollision(uint inX, uint inY) const;

	/// Projects inLocalPosition (a point in the space of the shape) along the Y axis onto the surface and returns it in outSurfacePosition.
	/// When there is no surface position (because of a hole or because the point is outside the heightfield) the function will return false.
	bool							ProjectOntoSurface(Vec3Arg inLocalPosition, Vec3 &outSurfacePosition, SubShapeID &outSubShapeID) const;

	/// Returns the coordinates of the triangle that a sub shape ID represents
	/// @param inSubShapeID The sub shape ID to decode
	/// @param outX X coordinate of the triangle (in the range [0, mSampleCount - 2])
	/// @param outY Y coordinate of the triangle (in the range [0, mSampleCount - 2])
	/// @param outTriangleIndex Triangle within the quad (0 = lower triangle or 1 = upper triangle)
	void							GetSubShapeCoordinates(const SubShapeID &inSubShapeID, uint &outX, uint &outY, uint &outTriangleIndex) const;

	/// Get the range of height values that this height field can encode. Can be used to determine the allowed range when setting the height values with SetHeights.
	float							GetMinHeightValue() const					{ return mOffset.GetY(); }
	float							GetMaxHeightValue() const					{ return mOffset.GetY() + mScale.GetY() * HeightFieldShapeConstants::cMaxHeightValue16; }

	/// Get the height values of a block of data.
	/// Note that the height values are decompressed so will be slightly different from what the shape was originally created with.
	/// @param inX Start X position, must be a multiple of mBlockSize and in the range [0, mSampleCount - 1]
	/// @param inY Start Y position, must be a multiple of mBlockSize and in the range [0, mSampleCount - 1]
	/// @param inSizeX Number of samples in X direction, must be a multiple of mBlockSize and in the range [0, mSampleCount - inX]
	/// @param inSizeY Number of samples in Y direction, must be a multiple of mBlockSize and in the range [0, mSampleCount - inY]
	/// @param outHeights Returned height values, must be at least inSizeX * inSizeY floats. Values are returned in x-major order and can be cNoCollisionValue.
	/// @param inHeightsStride Stride in floats between two consecutive rows of outHeights (can be negative if the data is upside down).
	void							GetHeights(uint inX, uint inY, uint inSizeX, uint inSizeY, float *outHeights, intptr_t inHeightsStride) const;

	/// Set the height values of a block of data.
	/// Note that this requires decompressing and recompressing a border of size mBlockSize in the negative x/y direction so will cause some precision loss.
	/// Beware this can create a race condition if you're running collision queries in parallel. See class documentation for more information.
	/// @param inX Start X position, must be a multiple of mBlockSize and in the range [0, mSampleCount - 1]
	/// @param inY Start Y position, must be a multiple of mBlockSize and in the range [0, mSampleCount - 1]
	/// @param inSizeX Number of samples in X direction, must be a multiple of mBlockSize and in the range [0, mSampleCount - inX]
	/// @param inSizeY Number of samples in Y direction, must be a multiple of mBlockSize and in the range [0, mSampleCount - inY]
	/// @param inHeights The new height values to set, must be an array of inSizeX * inSizeY floats, can be cNoCollisionValue. Values outside of the range [GetMinHeightValue(), GetMaxHeightValue()] will be clamped.
	/// @param inHeightsStride Stride in floats between two consecutive rows of inHeights (can be negative if the data is upside down).
	/// @param inAllocator Allocator to use for temporary memory
	/// @param inActiveEdgeCosThresholdAngle Cosine of the threshold angle (if the angle between the two triangles is bigger than this, the edge is active, note that a concave edge is always inactive).
	void							SetHeights(uint inX, uint inY, uint inSizeX, uint inSizeY, const float *inHeights, intptr_t inHeightsStride, TempAllocator &inAllocator, float inActiveEdgeCosThresholdAngle = 0.996195f);

	/// Get the current list of materials, the indices returned by GetMaterials() will index into this list.
	const PhysicsMaterialList &		GetMaterialList() const						{ return mMaterials; }

	/// Get the material indices of a block of data.
	/// @param inX Start X position, must in the range [0, mSampleCount - 1]
	/// @param inY Start Y position, must in the range [0, mSampleCount - 1]
	/// @param inSizeX Number of samples in X direction
	/// @param inSizeY Number of samples in Y direction
	/// @param outMaterials Returned material indices, must be at least inSizeX * inSizeY uint8s. Values are returned in x-major order.
	/// @param inMaterialsStride Stride in uint8s between two consecutive rows of outMaterials (can be negative if the data is upside down).
	void							GetMaterials(uint inX, uint inY, uint inSizeX, uint inSizeY, uint8 *outMaterials, intptr_t inMaterialsStride) const;

	/// Set the material indices of a block of data.
	/// Beware this can create a race condition if you're running collision queries in parallel. See class documentation for more information.
	/// @param inX Start X position, must in the range [0, mSampleCount - 1]
	/// @param inY Start Y position, must in the range [0, mSampleCount - 1]
	/// @param inSizeX Number of samples in X direction
	/// @param inSizeY Number of samples in Y direction
	/// @param inMaterials The new material indices, must be at least inSizeX * inSizeY uint8s. Values are returned in x-major order.
	/// @param inMaterialsStride Stride in uint8s between two consecutive rows of inMaterials (can be negative if the data is upside down).
	/// @param inMaterialList The material list to use for the new material indices or nullptr if the material list should not be updated
	/// @param inAllocator Allocator to use for temporary memory
	/// @return True if the material indices were set, false if the total number of materials exceeded 256
	bool							SetMaterials(uint inX, uint inY, uint inSizeX, uint inSizeY, const uint8 *inMaterials, intptr_t inMaterialsStride, const PhysicsMaterialList *inMaterialList, TempAllocator &inAllocator);

	// See Shape
	virtual void					SaveBinaryState(StreamOut &inStream) const override;
	virtual void					SaveMaterialState(PhysicsMaterialList &outMaterials) const override;
	virtual void					RestoreMaterialState(const PhysicsMaterialRefC *inMaterials, uint inNumMaterials) override;

	// See Shape::GetStats
	virtual Stats					GetStats() const override;

	// See Shape::GetVolume
	virtual float					GetVolume() const override					{ return 0; }

#ifndef MOSS_DEBUG_RENDERER
	// Settings
	static bool						sDrawTriangleOutlines;
#endif // MOSS_DEBUG_RENDERER

	// Register shape functions with the registry
	static void						sRegister();

protected:
	// See: Shape::RestoreBinaryState
	virtual void					RestoreBinaryState(StreamIn &inStream) override;

private:
	class							DecodingContext;							///< Context class for walking through all nodes of a heightfield
	struct							HSGetTrianglesContext;						///< Context class for GetTrianglesStart/Next

	/// Calculate commonly used values and store them in the shape
	void							CacheValues();

	/// Allocate the mRangeBlocks, mHeightSamples and mActiveEdges buffers as a single data block
	void							AllocateBuffers();

	/// Calculate bit mask for all active edges in the heightfield for a specific region
	void							CalculateActiveEdges(uint inX, uint inY, uint inSizeX, uint inSizeY, const float *inHeights, uint inHeightsStartX, uint inHeightsStartY, intptr_t inHeightsStride, float inHeightsScale, float inActiveEdgeCosThresholdAngle, TempAllocator &inAllocator);

	/// Calculate bit mask for all active edges in the heightfield
	void							CalculateActiveEdges(const HeightFieldShapeSettings &inSettings);

	/// Store material indices in the least amount of bits per index possible
	void							StoreMaterialIndices(const HeightFieldShapeSettings &inSettings);

	/// Get the amount of horizontal/vertical blocks
	inline uint						GetNumBlocks() const						{ return mSampleCount / mBlockSize; }

	/// Get the maximum level (amount of grids) of the tree
	static inline uint				sGetMaxLevel(uint inNumBlocks)				{ return 32 - CountLeadingZeros(inNumBlocks - 1); }

	/// Get the range block offset and stride for GetBlockOffsetAndScale
	static inline void				sGetRangeBlockOffsetAndStride(uint inNumBlocks, uint inMaxLevel, uint &outRangeBlockOffset, uint &outRangeBlockStride);

	/// For block (inBlockX, inBlockY) get the offset and scale needed to decode a uint8 height sample to a uint16
	inline void						GetBlockOffsetAndScale(uint inBlockX, uint inBlockY, uint inRangeBlockOffset, uint inRangeBlockStride, float &outBlockOffset, float &outBlockScale) const;

	/// Get the height sample at position (inX, inY)
	inline uint8					GetHeightSample(uint inX, uint inY) const;

	/// Faster version of GetPosition when block offset and scale are already known
	inline Vec3						GetPosition(uint inX, uint inY, float inBlockOffset, float inBlockScale, bool &outNoCollision) const;

	/// Determine amount of bits needed to encode sub shape id
	uint							GetSubShapeIDBits() const;

	/// En/decode a sub shape ID. inX and inY specify the coordinate of the triangle. inTriangle == 0 is the lower triangle, inTriangle == 1 is the upper triangle.
	inline SubShapeID				EncodeSubShapeID(const SubShapeIDCreator &inCreator, uint inX, uint inY, uint inTriangle) const;
	inline void						DecodeSubShapeID(const SubShapeID &inSubShapeID, uint &outX, uint &outY, uint &outTriangle) const;

	/// Get the edge flags for a triangle
	inline uint8					GetEdgeFlags(uint inX, uint inY, uint inTriangle) const;

	// Helper functions called by CollisionDispatch
	static void						sCollideConvexVsHeightField(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter);
	static void						sCollideSphereVsHeightField(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter);
	static void						sCastConvexVsHeightField(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector);
	static void						sCastSphereVsHeightField(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector);

	/// Visit the entire height field using a visitor pattern
	/// Note: Used to be inlined but this triggers a bug in MSVC where it will not free the memory allocated by alloca which causes a stack overflow when WalkHeightField is called in a loop (clang does it correct)
	template <class Visitor>
	void							WalkHeightField(Visitor &ioVisitor) const;

	/// A block of 2x2 ranges used to form a hierarchical grid, ordered left top, right top, left bottom, right bottom
	struct alignas(16) RangeBlock
	{
		uint16						mMin[4];
		uint16						mMax[4];
	};

	/// For block (inBlockX, inBlockY) get the range block and the entry in the range block
	inline void						GetRangeBlock(uint inBlockX, uint inBlockY, uint inRangeBlockOffset, uint inRangeBlockStride, RangeBlock *&outBlock, uint &outIndexInBlock);

	/// Offset of first RangedBlock in grid per level
	static const uint				sGridOffsets[];

	/// The height field is a surface defined by: mOffset + mScale * (x, mHeightSamples[y * mSampleCount + x], y).
	/// where x and y are integers in the range x and y e [0, mSampleCount - 1].
	Vec3							mOffset = Vec3::sZero();
	Vec3							mScale = Vec3::sOne();

	/// Height data
	uint32							mSampleCount = 0;							///< See HeightFieldShapeSettings::mSampleCount
	uint32							mBlockSize = 2;								///< See HeightFieldShapeSettings::mBlockSize
	uint32							mHeightSamplesSize = 0;						///< Size of mHeightSamples in bytes
	uint32							mRangeBlocksSize = 0;						///< Size of mRangeBlocks in elements
	uint32							mActiveEdgesSize = 0;						///< Size of mActiveEdges in bytes
	uint8							mBitsPerSample = 8;							///< See HeightFieldShapeSettings::mBitsPerSample
	uint8							mSampleMask = 0xff;							///< All bits set for a sample: (1 << mBitsPerSample) - 1, used to indicate that there's no collision
	uint16							mMinSample = HeightFieldShapeConstants::cNoCollisionValue16; ///< Min and max value in mHeightSamples quantized to 16 bit, for calculating bounding box
	uint16							mMaxSample = HeightFieldShapeConstants::cNoCollisionValue16;
	RangeBlock *					mRangeBlocks = nullptr;						///< Hierarchical grid of range data describing the height variations within 1 block. The grid for level <level> starts at offset sGridOffsets[<level>]
	uint8 *							mHeightSamples = nullptr;					///< mBitsPerSample-bit height samples. Value [0, mMaxHeightValue] maps to highest detail grid in mRangeBlocks [mMin, mMax]. mNoCollisionValue is reserved to indicate no collision.
	uint8 *							mActiveEdges = nullptr;						///< (mSampleCount - 1)^2 * 3-bit active edge flags.

	/// Materials
	PhysicsMaterialList				mMaterials;									///< The materials of square at (x, y) is: mMaterials[mMaterialIndices[x + y * (mSampleCount - 1)]]
	TArray<uint8>					mMaterialIndices;							///< Compressed to the minimum amount of bits per material index (mSampleCount - 1) * (mSampleCount - 1) * mNumBitsPerMaterialIndex bits of data
	uint32							mNumBitsPerMaterialIndex = 0;				///< Number of bits per material index

#ifndef MOSS_DEBUG_RENDERER
	/// Temporary rendering data
	mutable TArray<DebugRenderer::GeometryRef> mGeometry;
	mutable bool					mCachedUseMaterialColors = false;			///< This is used to regenerate the triangle batch if the drawing settings change
#endif // MOSS_DEBUG_RENDERER
};


// Class that constructs a MutableCompoundShape.
class MOSS_EXPORT MutableCompoundShapeSettings final : public CompoundShapeSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, MutableCompoundShapeSettings)

public:
	// See: ShapeSettings
	virtual ShapeResult				Create() const override;
};

/// A compound shape, sub shapes can be rotated and translated.
/// This shape is optimized for adding / removing and changing the rotation / translation of sub shapes but is less efficient in querying.
/// Shifts all child objects so that they're centered around the center of mass (which needs to be kept up to date by calling AdjustCenterOfMass).
///
/// Note: If you're using MutableCompoundShape and are querying data while modifying the shape you'll have a race condition.
/// In this case it is best to create a new MutableCompoundShape using the Clone function. You replace the shape on a body using BodyInterface::SetShape.
/// If a query is still working on the old shape, it will have taken a reference and keep the old shape alive until the query finishes.
///
/// When you modify a MutableCompoundShape, beware that the SubShapeIDs of all other shapes can change. So be careful when storing SubShapeIDs.
class MOSS_EXPORT MutableCompoundShape final : public CompoundShape
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
									MutableCompoundShape() : CompoundShape(EShapeSubType::MutableCompound) { }
									MutableCompoundShape(const MutableCompoundShapeSettings &inSettings, ShapeResult &outResult);

	/// Clone this shape. Can be used to avoid race conditions. See the documentation of this class for more information.
	Ref<MutableCompoundShape>		Clone() const;

	// See Shape::CastRay
	virtual bool					CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const override;
	virtual void					CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;

	// See: Shape::CollidePoint
	virtual void					CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;

	// See Shape::CollectTransformedShapes
	virtual void					CollectTransformedShapes(const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale, const SubShapeIDCreator &inSubShapeIDCreator, TransformedShapeCollector &ioCollector, const ShapeFilter &inShapeFilter) const override;

	// See: CompoundShape::GetIntersectingSubShapes
	virtual int						GetIntersectingSubShapes(const AABox &inBox, uint *outSubShapeIndices, int inMaxSubShapeIndices) const override;

	// See: CompoundShape::GetIntersectingSubShapes
	virtual int						GetIntersectingSubShapes(const OrientedBox &inBox, uint *outSubShapeIndices, int inMaxSubShapeIndices) const override;

	// See Shape
	virtual void					SaveBinaryState(StreamOut &inStream) const override;

	// See Shape::GetStats
	virtual Stats					GetStats() const override								{ return Stats(sizeof(*this) + mSubShapes.size() * sizeof(SubShape) + mSubShapeBounds.size() * sizeof(Bounds), 0); }

	///@{
	/// @name Mutating shapes. Note that this is not thread safe, so you need to ensure that any bodies that use this shape are locked at the time of modification using BodyLockWrite. After modification you need to call BodyInterface::NotifyShapeChanged to update the broadphase and collision caches.

	/// Adding a new shape.
	/// Beware this can create a race condition if you're running collision queries in parallel. See class documentation for more information.
	/// @param inPosition The position of the new shape
	/// @param inRotation The orientation of the new shape
	/// @param inShape The shape to add
	/// @param inUserData User data that will be stored with the shape and can be retrieved using GetCompoundUserData
	/// @param inIndex Index where to insert the shape, UINT_MAX to add to the end
	/// @return The index of the newly added shape
	uint							AddShape(Vec3Arg inPosition, QuatArg inRotation, const Shape *inShape, uint32 inUserData = 0, uint inIndex = UINT_MAX);

	/// Remove a shape by index.
	/// Beware this can create a race condition if you're running collision queries in parallel. See class documentation for more information.
	void							RemoveShape(uint inIndex);

	/// Modify the position / orientation of a shape.
	/// Beware this can create a race condition if you're running collision queries in parallel. See class documentation for more information.
	void							ModifyShape(uint inIndex, Vec3Arg inPosition, QuatArg inRotation);

	/// Modify the position / orientation and shape at the same time.
	/// Beware this can create a race condition if you're running collision queries in parallel. See class documentation for more information.
	void							ModifyShape(uint inIndex, Vec3Arg inPosition, QuatArg inRotation, const Shape *inShape);

	/// @brief Batch set positions / orientations, this avoids duplicate work due to bounding box calculation.
	/// Beware this can create a race condition if you're running collision queries in parallel. See class documentation for more information.
	/// @param inStartIndex Index of first shape to update
	/// @param inNumber Number of shapes to update
	/// @param inPositions A list of positions with arbitrary stride
	/// @param inRotations A list of orientations with arbitrary stride
	/// @param inPositionStride The position stride (the number of bytes between the first and second element)
	/// @param inRotationStride The orientation stride (the number of bytes between the first and second element)
	void							ModifyShapes(uint inStartIndex, uint inNumber, const Vec3 *inPositions, const Quat *inRotations, uint inPositionStride = sizeof(Vec3), uint inRotationStride = sizeof(Quat));

	/// Recalculate the center of mass and shift all objects so they're centered around it
	/// (this needs to be done of dynamic bodies and if the center of mass changes significantly due to adding / removing / repositioning sub shapes or else the simulation will look unnatural)
	/// Note that after adjusting the center of mass of an object you need to call BodyInterface::NotifyShapeChanged and Constraint::NotifyShapeChanged on the relevant bodies / constraints.
	/// Beware this can create a race condition if you're running collision queries in parallel. See class documentation for more information.
	void							AdjustCenterOfMass();

	///@}

	// Register shape functions with the registry
	static void						sRegister();

protected:
	// See: Shape::RestoreBinaryState
	virtual void					RestoreBinaryState(StreamIn &inStream) override;

private:
	// Visitor for GetIntersectingSubShapes
	template <class BoxType>
	struct GetIntersectingSubShapesVisitorMC : public GetIntersectingSubShapesVisitor<BoxType>
	{
		using GetIntersectingSubShapesVisitor<BoxType>::GetIntersectingSubShapesVisitor;

		using Result = UVec4;

		MOSS_INLINE Result			TestBlock(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ) const
		{
			return GetIntersectingSubShapesVisitor<BoxType>::TestBounds(inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);
		}

		MOSS_INLINE bool				ShouldVisitBlock(UVec4Arg inResult) const
		{
			return inResult.TestAnyTrue();
		}

		MOSS_INLINE bool				ShouldVisitSubShape(UVec4Arg inResult, uint inIndexInBlock) const
		{
			return inResult[inIndexInBlock] != 0;
		}
	};

	/// Get the number of blocks of 4 bounding boxes
	inline uint						GetNumBlocks() const										{ return ((uint)mSubShapes.size() + 3) >> 2; }

	/// Ensure that the mSubShapeBounds has enough space to store bounding boxes equivalent to the number of shapes in mSubShapes
	void							EnsureSubShapeBoundsCapacity();

	/// Update mSubShapeBounds
	/// @param inStartIdx First sub shape to update
	/// @param inNumber Number of shapes to update
	void							CalculateSubShapeBounds(uint inStartIdx, uint inNumber);

	/// Calculate mLocalBounds from mSubShapeBounds
	void							CalculateLocalBounds();

	template <class Visitor>
	MOSS_INLINE void					WalkSubShapes(Visitor &ioVisitor) const;					///< Walk the sub shapes and call Visitor::VisitShape for each sub shape encountered

	// Helper functions called by CollisionDispatch
	static void						sCollideCompoundVsShape(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter);
	static void						sCollideShapeVsCompound(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter);
	static void						sCastShapeVsCompound(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector);

	struct Bounds
	{
		Vec4						mMinX;
		Vec4						mMinY;
		Vec4						mMinZ;
		Vec4						mMaxX;
		Vec4						mMaxY;
		Vec4						mMaxZ;
	};

	TArray<Bounds>					mSubShapeBounds;											///< Bounding boxes of all sub shapes in SOA format (in blocks of 4 boxes), MinX 0..3, MinY 0..3, MinZ 0..3, MaxX 0..3, MaxY 0..3, MaxZ 0..3, MinX 4..7, MinY 4..7, ...
};



// Class that constructs a MeshShape
class MOSS_EXPORT MeshShapeSettings final : public ShapeSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, MeshShapeSettings)

public:
	/// Default constructor for deserialization
									MeshShapeSettings() = default;

	/// Create a mesh shape.
									MeshShapeSettings(const TriangleList &inTriangles, PhysicsMaterialList inMaterials = PhysicsMaterialList());
									MeshShapeSettings(VertexList inVertices, IndexedTriangleList inTriangles, PhysicsMaterialList inMaterials = PhysicsMaterialList());

	/// Sanitize the mesh data. Remove duplicate and degenerate triangles. This is called automatically when constructing the MeshShapeSettings with a list of (indexed-) triangles.
	void							Sanitize();

	// See: ShapeSettings
	virtual ShapeResult				Create() const override;

	/// Vertices belonging to mIndexedTriangles
	VertexList						mTriangleVertices;

	/// Original list of indexed triangles (triangles will be reordered internally in the mesh shape).
	/// Triangles must be provided in counter clockwise order.
	/// Degenerate triangles will automatically be removed during mesh creation but no other mesh simplifications are performed, use an external library if this is desired.
	/// For simulation, the triangles are considered to be single sided.
	/// For ray casts you can choose to make triangles double sided by setting RayCastSettings::mBackFaceMode to EBackFaceMode::CollideWithBackFaces.
	/// For collide shape tests you can use CollideShapeSettings::mBackFaceMode and for shape casts you can use ShapeCastSettings::mBackFaceModeTriangles.
	IndexedTriangleList				mIndexedTriangles;

	/// Materials assigned to the triangles. Each triangle specifies which material it uses through its mMaterialIndex
	PhysicsMaterialList				mMaterials;

	/// Maximum number of triangles in each leaf of the axis aligned box tree. This is a balance between memory and performance. Can be in the range [1, MeshShape::MaxTrianglesPerLeaf].
	/// Sensible values are between 4 (for better performance) and 8 (for less memory usage).
	uint							mMaxTrianglesPerLeaf = 8;

	/// Cosine of the threshold angle (if the angle between the two triangles is bigger than this, the edge is active, note that a concave edge is always inactive).
	/// Setting this value too small can cause ghost collisions with edges, setting it too big can cause depenetration artifacts (objects not depenetrating quickly).
	/// Valid ranges are between cos(0 degrees) and cos(90 degrees). The default value is cos(5 degrees).
	/// Negative values will make all edges active and causes EActiveEdgeMode::CollideOnlyWithActive to behave as EActiveEdgeMode::CollideWithAll.
	/// This speeds up the build process but will require all bodies that can interact with the mesh to use BodyCreationSettings::mEnhancedInternalEdgeRemoval = true.
	float							mActiveEdgeCosThresholdAngle = 0.996195f;					// cos(5 degrees)

	/// When true, we store the user data coming from Triangle::mUserData or IndexedTriangle::mUserData in the mesh shape.
	/// This can be used to store additional data like the original index of the triangle in the mesh.
	/// Can be retrieved using MeshShape::GetTriangleUserData.
	/// Turning this on increases the memory used by the MeshShape by roughly 25%.
	bool							mPerTriangleUserData = false;

	enum class EBuildQuality
	{
		FavorRuntimePerformance,																///< Favor runtime performance, takes more time to build the MeshShape but performs better
		FavorBuildSpeed,																		///< Favor build speed, build the tree faster but the MeshShape will be slower
	};

	/// Determines the quality of the tree building process.
	EBuildQuality					mBuildQuality = EBuildQuality::FavorRuntimePerformance;
};

/// A mesh shape, consisting of triangles. Mesh shapes are mostly used for static geometry.
/// They can be used by dynamic or kinematic objects but only if they don't collide with other mesh or heightfield shapes as those collisions are currently not supported.
/// Note that if you make a mesh shape a dynamic or kinematic object, you need to provide a mass yourself as mesh shapes don't need to form a closed hull so don't have a well defined volume from which the mass can be calculated.
class MOSS_EXPORT MeshShape final : public Shape
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
									MeshShape() : Shape(EShapeType::Mesh, EShapeSubType::Mesh) { }
									MeshShape(const MeshShapeSettings &inSettings, ShapeResult &outResult);

	// See Shape::MustBeStatic
	virtual bool					MustBeStatic() const override								{ return true; }

	// See Shape::GetLocalBounds
	virtual AABox					GetLocalBounds() const override;

	// See Shape::GetSubShapeIDBitsRecursive
	virtual uint					GetSubShapeIDBitsRecursive() const override;

	// See Shape::GetInnerRadius
	virtual float					GetInnerRadius() const override								{ return 0.0f; }

	// See Shape::GetMassProperties
	virtual MassProperties			GetMassProperties() const override;

	// See Shape::GetMaterial
	virtual const PhysicsMaterial *	GetMaterial(const SubShapeID &inSubShapeID) const override;

	/// Get the list of all materials
	const PhysicsMaterialList &		GetMaterialList() const										{ return mMaterials; }

	/// Determine which material index a particular sub shape uses (note that if there are no materials this function will return 0 so check the array size)
	/// Note: This could for example be used to create a decorator shape around a mesh shape that overrides the GetMaterial call to replace a material with another material.
	uint							GetMaterialIndex(const SubShapeID &inSubShapeID) const;

	// See Shape::GetSurfaceNormal
	virtual Vec3					GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const override;

	// See Shape::GetSupportingFace
	virtual void					GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const override;

#ifndef MOSS_DEBUG_RENDERER
	// See Shape::Draw
	virtual void					Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const override;
#endif // MOSS_DEBUG_RENDERER

	// See Shape::CastRay
	virtual bool					CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const override;
	virtual void					CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;

	/// See: Shape::CollidePoint
	/// Note that for CollidePoint to work for a mesh shape, the mesh needs to be closed (a manifold) or multiple non-intersecting manifolds. Triangles may be facing the interior of the manifold.
	/// Insideness is tested by counting the amount of triangles encountered when casting an infinite ray from inPoint. If the number of hits is odd we're inside, if it's even we're outside.
	virtual void					CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;

	// See: Shape::CollideSoftBodyVertices
	virtual void					CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const override;

	// See Shape::GetTrianglesStart
	virtual void					GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const override;

	// See Shape::GetTrianglesNext
	virtual int						GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials = nullptr) const override;

	// See Shape::GetSubmergedVolume
	virtual void					GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy MOSS_IF_DEBUG_RENDERER(, RVec3Arg inBaseOffset)) const override { MOSS_ASSERT(false, "Not supported"); }

	// See Shape
	virtual void					SaveBinaryState(StreamOut &inStream) const override;
	virtual void					SaveMaterialState(PhysicsMaterialList &outMaterials) const override;
	virtual void					RestoreMaterialState(const PhysicsMaterialRefC *inMaterials, uint inNumMaterials) override;

	// See Shape::GetStats
	virtual Stats					GetStats() const override;

	// See Shape::GetVolume
	virtual float					GetVolume() const override									{ return 0; }

	// When MeshShape::mPerTriangleUserData is true, this function can be used to retrieve the user data that was stored in the mesh shape.
	uint32							GetTriangleUserData(const SubShapeID &inSubShapeID) const;

#ifndef MOSS_DEBUG_RENDERER
	// Settings
	static bool						sDrawTriangleGroups;
	static bool						sDrawTriangleOutlines;
#endif // MOSS_DEBUG_RENDERER

	// Register shape functions with the registry
	static void						sRegister();

protected:
	// See: Shape::RestoreBinaryState
	virtual void					RestoreBinaryState(StreamIn &inStream) override;

private:
	struct							MSGetTrianglesContext;										///< Context class for GetTrianglesStart/Next

	static constexpr int			NumTriangleBits = 3;										///< How many bits to reserve to encode the triangle index
	static constexpr int			MaxTrianglesPerLeaf = 1 << NumTriangleBits;					///< Number of triangles that are stored max per leaf aabb node

	/// Find and flag active edges
	static void						sFindActiveEdges(const MeshShapeSettings &inSettings, IndexedTriangleList &ioIndices);

	/// Visit the entire tree using a visitor pattern
	template <class Visitor>
	void							WalkTree(Visitor &ioVisitor) const;

	/// Same as above but with a callback per triangle instead of per block of triangles
	template <class Visitor>
	void							WalkTreePerTriangle(const SubShapeIDCreator &inSubShapeIDCreator2, Visitor &ioVisitor) const;

	/// Decode a sub shape ID
	inline void						DecodeSubShapeID(const SubShapeID &inSubShapeID, const void *&outTriangleBlock, uint32 &outTriangleIndex) const;

	// Helper functions called by CollisionDispatch
	static void						sCollideConvexVsMesh(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter);
	static void						sCollideSphereVsMesh(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter);
	static void						sCastConvexVsMesh(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector);
	static void						sCastSphereVsMesh(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector);

	/// Materials assigned to the triangles. Each triangle specifies which material it uses through its mMaterialIndex
	PhysicsMaterialList				mMaterials;

	ByteBuffer						mTree;														///< Resulting packed data structure

	/// 8 bit flags stored per triangle
	enum ETriangleFlags
	{
		/// Material index
		FLAGS_MATERIAL_BITS			= 5,
		FLAGS_MATERIAL_MASK			= (1 << FLAGS_MATERIAL_BITS) - 1,

		/// Active edge bits
		FLAGS_ACTIVE_EGDE_SHIFT		= FLAGS_MATERIAL_BITS,
		FLAGS_ACTIVE_EDGE_BITS		= 3,
		FLAGS_ACTIVE_EDGE_MASK		= (1 << FLAGS_ACTIVE_EDGE_BITS) - 1
	};

#ifndef MOSS_DEBUG_RENDERER
	mutable DebugRenderer::GeometryRef	mGeometry;												///< Debug rendering data
	mutable bool					mCachedTrianglesColoredPerGroup = false;					///< This is used to regenerate the triangle batch if the drawing settings change
	mutable bool					mCachedUseMaterialColors = false;							///< This is used to regenerate the triangle batch if the drawing settings change
#endif // MOSS_DEBUG_RENDERER
};


/// Class that constructs a ConvexShape (abstract)
class MOSS_EXPORT ConvexShapeSettings : public ShapeSettings
{
	MOSS_DECLARE_SERIALIZABLE_ABSTRACT(MOSS_EXPORT, ConvexShapeSettings)

public:
	/// Constructor
									ConvexShapeSettings() = default;
	explicit						ConvexShapeSettings(const PhysicsMaterial *inMaterial)		: mMaterial(inMaterial) { }

	/// Set the density of the object in kg / m^3
	void							SetDensity(float inDensity)									{ mDensity = inDensity; }

	// Properties
	RefConst<PhysicsMaterial>		mMaterial;													///< Material assigned to this shape
	float							mDensity = 1000.0f;											///< Uniform density of the interior of the convex object (kg / m^3)
};

/// Base class for all convex shapes. Defines a virtual interface.
class MOSS_EXPORT ConvexShape : public Shape
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
	explicit						ConvexShape(EShapeSubType inSubType) : Shape(EShapeType::Convex, inSubType) { }
									ConvexShape(EShapeSubType inSubType, const ConvexShapeSettings &inSettings, ShapeResult &outResult) : Shape(EShapeType::Convex, inSubType, inSettings, outResult), mMaterial(inSettings.mMaterial), mDensity(inSettings.mDensity) { }
									ConvexShape(EShapeSubType inSubType, const PhysicsMaterial *inMaterial) : Shape(EShapeType::Convex, inSubType), mMaterial(inMaterial) { }

	// See Shape::GetSubShapeIDBitsRecursive
	virtual uint					GetSubShapeIDBitsRecursive() const override					{ return 0; } // Convex shapes don't have sub shapes

	// See Shape::GetMaterial
	virtual const PhysicsMaterial *	GetMaterial([[maybe_unused]] const SubShapeID &inSubShapeID) const override	{ MOSS_ASSERT(inSubShapeID.IsEmpty(), "Invalid subshape ID"); return GetMaterial(); }

	// See Shape::CastRay
	virtual bool					CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const override;
	virtual void					CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;

	// See: Shape::CollidePoint
	virtual void					CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;

	// See Shape::GetTrianglesStart
	virtual void					GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const override;

	// See Shape::GetTrianglesNext
	virtual int						GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials = nullptr) const override;

	// See Shape::GetSubmergedVolume
	virtual void					GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy MOSS_IF_DEBUG_RENDERER(, RVec3Arg inBaseOffset)) const override;

	/// Function that provides an interface for GJK
	class Support
	{
	public:
		/// Warning: Virtual destructor will not be called on this object!
		virtual						~Support() = default;

		/// Calculate the support vector for this convex shape (includes / excludes the convex radius depending on how this was obtained).
		/// Support vector is relative to the center of mass of the shape.
		virtual Vec3				GetSupport(Vec3Arg inDirection) const = 0;

		/// Convex radius of shape. Collision detection on penetrating shapes is much more expensive,
		/// so you can add a radius around objects to increase the shape. This makes it far less likely that they will actually penetrate.
		virtual float				GetConvexRadius() const = 0;
	};

	/// Buffer to hold a Support object, used to avoid dynamic memory allocations
	class alignas(16) SupportBuffer
	{
	public:
		uint8						mData[4160];
	};

	/// How the GetSupport function should behave
	enum class ESupportMode
	{
		ExcludeConvexRadius,		///< Return the shape excluding the convex radius, Support::GetConvexRadius will return the convex radius if there is one, but adding this radius may not result in the most accurate/efficient representation of shapes with sharp edges
		IncludeConvexRadius,		///< Return the shape including the convex radius, Support::GetSupport includes the convex radius if there is one, Support::GetConvexRadius will return 0
		Default,					///< Use both Support::GetSupport add Support::GetConvexRadius to get a support point that matches the original shape as accurately/efficiently as possible
	};

	/// Returns an object that provides the GetSupport function for this shape.
	/// inMode determines if this support function includes or excludes the convex radius.
	/// of the values returned by the GetSupport function. This improves numerical accuracy of the results.
	/// inScale scales this shape in local space.
	virtual const Support *			GetSupportFunction(ESupportMode inMode, SupportBuffer &inBuffer, Vec3Arg inScale) const = 0;

	/// Material of the shape
	void							SetMaterial(const PhysicsMaterial *inMaterial)				{ mMaterial = inMaterial; }
	const PhysicsMaterial *			GetMaterial() const											{ return mMaterial != nullptr? mMaterial : PhysicsMaterial::sDefault; }

	/// Set density of the shape (kg / m^3)
	void							SetDensity(float inDensity)									{ mDensity = inDensity; }

	/// Get density of the shape (kg / m^3)
	float							GetDensity() const											{ return mDensity; }

#ifndef MOSS_DEBUG_RENDERER
	// See Shape::DrawGetSupportFunction
	virtual void					DrawGetSupportFunction(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inDrawSupportDirection) const override;

	// See Shape::DrawGetSupportingFace
	virtual void					DrawGetSupportingFace(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale) const override;
#endif // MOSS_DEBUG_RENDERER

	// See Shape
	virtual void					SaveBinaryState(StreamOut &inStream) const override;
	virtual void					SaveMaterialState(PhysicsMaterialList &outMaterials) const override;
	virtual void					RestoreMaterialState(const PhysicsMaterialRefC *inMaterials, uint inNumMaterials) override;

	// Register shape functions with the registry
	static void						sRegister();

protected:
	// See: Shape::RestoreBinaryState
	virtual void					RestoreBinaryState(StreamIn &inStream) override;

	/// Vertex list that forms a unit sphere
	static const TStaticArray<Vec3, 384> sUnitSphereTriangles;

private:
	// Class for GetTrianglesStart/Next
	class							CSGetTrianglesContext;

	// Helper functions called by CollisionDispatch
	static void						sCollideConvexVsConvex(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter);
	static void						sCastConvexVsConvex(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector);

	// Properties
	RefConst<PhysicsMaterial>		mMaterial;													///< Material assigned to this shape
	float							mDensity = 1000.0f;											///< Uniform density of the interior of the convex object (kg / m^3)
};


// Class that constructs a StaticCompoundShape. Note that if you only want a compound of 1 shape, use a RotatedTranslatedShape instead.
class MOSS_EXPORT StaticCompoundShapeSettings final : public CompoundShapeSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, StaticCompoundShapeSettings)

public:
	// See: ShapeSettings
	virtual ShapeResult				Create() const override;

	/// Specialization of Create() function that allows specifying a temp allocator to avoid temporary memory allocations on the heap
	ShapeResult						Create(TempAllocator &inTempAllocator) const;
};

/// A compound shape, sub shapes can be rotated and translated.
/// Sub shapes cannot be modified once the shape is constructed.
/// Shifts all child objects so that they're centered around the center of mass.
class MOSS_EXPORT StaticCompoundShape final : public CompoundShape
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
									StaticCompoundShape() : CompoundShape(EShapeSubType::StaticCompound) { }
									StaticCompoundShape(const StaticCompoundShapeSettings &inSettings, TempAllocator &inTempAllocator, ShapeResult &outResult);

	// See Shape::CastRay
	virtual bool					CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const override;
	virtual void					CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;

	// See: Shape::CollidePoint
	virtual void					CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;

	// See Shape::CollectTransformedShapes
	virtual void					CollectTransformedShapes(const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale, const SubShapeIDCreator &inSubShapeIDCreator, TransformedShapeCollector &ioCollector, const ShapeFilter &inShapeFilter) const override;

	// See: CompoundShape::GetIntersectingSubShapes
	virtual int						GetIntersectingSubShapes(const AABox &inBox, uint *outSubShapeIndices, int inMaxSubShapeIndices) const override;

	// See: CompoundShape::GetIntersectingSubShapes
	virtual int						GetIntersectingSubShapes(const OrientedBox &inBox, uint *outSubShapeIndices, int inMaxSubShapeIndices) const override;

	// See Shape
	virtual void					SaveBinaryState(StreamOut &inStream) const override;

	// See Shape::GetStats
	virtual Stats					GetStats() const override								{ return Stats(sizeof(*this) + mSubShapes.size() * sizeof(SubShape) + mNodes.size() * sizeof(Node), 0); }

	// Register shape functions with the registry
	static void						sRegister();

protected:
	// See: Shape::RestoreBinaryState
	virtual void					RestoreBinaryState(StreamIn &inStream) override;

private:
	// Visitor for GetIntersectingSubShapes
	template <class BoxType>
	struct GetIntersectingSubShapesVisitorSC : public GetIntersectingSubShapesVisitor<BoxType>
	{
		using GetIntersectingSubShapesVisitor<BoxType>::GetIntersectingSubShapesVisitor;

		MOSS_INLINE bool				ShouldVisitNode(int inStackTop) const
		{
			return true;
		}

		MOSS_INLINE int				VisitNodes(Vec4Arg inBoundsMinX, Vec4Arg inBoundsMinY, Vec4Arg inBoundsMinZ, Vec4Arg inBoundsMaxX, Vec4Arg inBoundsMaxY, Vec4Arg inBoundsMaxZ, UVec4 &ioProperties, int inStackTop)
		{
			// Test if point overlaps with box
			UVec4 collides = GetIntersectingSubShapesVisitor<BoxType>::TestBounds(inBoundsMinX, inBoundsMinY, inBoundsMinZ, inBoundsMaxX, inBoundsMaxY, inBoundsMaxZ);
			return CountAndSortTrues(collides, ioProperties);
		}
	};

	/// Sorts ioBodyIdx spatially into 2 groups. Second groups starts at ioBodyIdx + outMidPoint.
	/// After the function returns ioBodyIdx and ioBounds will be shuffled
	static void						sPartition(uint *ioBodyIdx, AABox *ioBounds, int inNumber, int &outMidPoint);

	/// Sorts ioBodyIdx from inBegin to (but excluding) inEnd spatially into 4 groups.
	/// outSplit needs to be 5 ints long, when the function returns each group runs from outSplit[i] to (but excluding) outSplit[i + 1]
	/// After the function returns ioBodyIdx and ioBounds will be shuffled
	static void						sPartition4(uint *ioBodyIdx, AABox *ioBounds, int inBegin, int inEnd, int *outSplit);

	// Helper functions called by CollisionDispatch
	static void						sCollideCompoundVsShape(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter);
	static void						sCollideShapeVsCompound(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter);
	static void						sCastShapeVsCompound(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector);

	// Maximum size of the stack during tree walk
	static constexpr int			cStackSize = 128;

	template <class Visitor>
	MOSS_INLINE void					WalkTree(Visitor &ioVisitor) const;						///< Walk the node tree calling the Visitor::VisitNodes for each node encountered and Visitor::VisitShape for each sub shape encountered

	/// Bits used in Node::mNodeProperties
	enum : uint32
	{
		IS_SUBSHAPE					= 0x80000000,											///< If this bit is set, the other bits index in mSubShape, otherwise in mNodes
		INVALID_NODE				= 0x7fffffff,											///< Signifies an invalid node
	};

	/// Node structure
	struct Node
	{
		void						SetChildBounds(uint inIndex, const AABox &inBounds);	///< Set bounding box for child inIndex to inBounds
		void						SetChildInvalid(uint inIndex);							///< Mark the child inIndex as invalid and set its bounding box to invalid

		HalfFloat					mBoundsMinX[4];											///< 4 child bounding boxes
		HalfFloat					mBoundsMinY[4];
		HalfFloat					mBoundsMinZ[4];
		HalfFloat					mBoundsMaxX[4];
		HalfFloat					mBoundsMaxY[4];
		HalfFloat					mBoundsMaxZ[4];
		uint32						mNodeProperties[4];										///< 4 child node properties
	};

	static_assert(sizeof(Node) == 64, "Node should be 64 bytes");

	using Nodes = TArray<Node>;

	Nodes							mNodes;													///< Quad tree node structure
};

// Class that constructs an EmptyShape
class MOSS_EXPORT EmptyShapeSettings final : public ShapeSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, EmptyShapeSettings)

public:
							EmptyShapeSettings() = default;
	explicit				EmptyShapeSettings(Vec3Arg inCenterOfMass) : mCenterOfMass(inCenterOfMass) { }

	ShapeResult				Create() const override;

	Vec3					mCenterOfMass = Vec3::sZero();									///< Determines the center of mass for this shape
};

/// An empty shape that has no volume and collides with nothing.
///
/// Possible use cases:
/// - As a placeholder for a shape that will be created later. E.g. if you first need to create a body and only then know what shape it will have.
/// - If you need a kinematic body to attach a constraint to, but you don't want the body to collide with anything.
///
/// Note that, if possible, you should also put your body in an ObjectLayer that doesn't collide with anything.
/// This ensures that collisions will be filtered out at broad phase level instead of at narrow phase level, this is more efficient.
class MOSS_EXPORT EmptyShape final : public Shape
{
public:
	// Constructor
							EmptyShape() : Shape(EShapeType::Empty, EShapeSubType::Empty) { }
	explicit				EmptyShape(Vec3Arg inCenterOfMass) : Shape(EShapeType::Empty, EShapeSubType::Empty), mCenterOfMass(inCenterOfMass) { }
							EmptyShape(const EmptyShapeSettings &inSettings, ShapeResult &outResult) : Shape(EShapeType::Empty, EShapeSubType::Empty, inSettings, outResult), mCenterOfMass(inSettings.mCenterOfMass) { outResult.Set(this); }

	// See: Shape
	Vec3					GetCenterOfMass() const override								{ return mCenterOfMass; }
	AABox					GetLocalBounds() const override									{ return { Vec3::sZero(), Vec3::sZero() }; }
	uint					GetSubShapeIDBitsRecursive() const override						{ return 0; }
	float					GetInnerRadius() const override									{ return 0.0f; }
	MassProperties			GetMassProperties() const override;
	const PhysicsMaterial *	GetMaterial([[maybe_unused]] const SubShapeID &inSubShapeID) const override { return PhysicsMaterial::sDefault; }
	virtual Vec3			GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const override { return Vec3::sZero(); }
	virtual void			GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy
#ifndef MOSS_DEBUG_RENDERER // Not using MOSS_IF_DEBUG_RENDERER for Doxygen
		, RVec3Arg inBaseOffset
#endif
		) const override																	{ outTotalVolume = 0.0f; outSubmergedVolume = 0.0f; outCenterOfBuoyancy = Vec3::sZero(); }
#ifndef MOSS_DEBUG_RENDERER
	virtual void			Draw([[maybe_unused]] DebugRenderer *inRenderer, [[maybe_unused]] RMat44Arg inCenterOfMassTransform, [[maybe_unused]] Vec3Arg inScale, [[maybe_unused]] ColorArg inColor, [[maybe_unused]] bool inUseMaterialColors, [[maybe_unused]] bool inDrawWireframe) const override;
#endif // MOSS_DEBUG_RENDERER
	virtual bool			CastRay([[maybe_unused]] const RayCast &inRay, [[maybe_unused]] const SubShapeIDCreator &inSubShapeIDCreator, [[maybe_unused]] RayCastResult &ioHit) const override { return false; }
	virtual void			CastRay([[maybe_unused]] const RayCast &inRay, [[maybe_unused]] const RayCastSettings &inRayCastSettings, [[maybe_unused]] const SubShapeIDCreator &inSubShapeIDCreator, [[maybe_unused]] CastRayCollector &ioCollector, [[maybe_unused]] const ShapeFilter &inShapeFilter = { }) const override { /* Do nothing */ }
	virtual void			CollidePoint([[maybe_unused]] Vec3Arg inPoint, [[maybe_unused]] const SubShapeIDCreator &inSubShapeIDCreator, [[maybe_unused]] CollidePointCollector &ioCollector, [[maybe_unused]] const ShapeFilter &inShapeFilter = { }) const override { /* Do nothing */ }
	virtual void			CollideSoftBodyVertices([[maybe_unused]] Mat44Arg inCenterOfMassTransform, [[maybe_unused]] Vec3Arg inScale, [[maybe_unused]] const CollideSoftBodyVertexIterator &inVertices, [[maybe_unused]] uint inNumVertices, [[maybe_unused]] int inCollidingShapeIndex) const override { /* Do nothing */ }
	virtual void			GetTrianglesStart([[maybe_unused]] GetTrianglesContext &ioContext, [[maybe_unused]] const AABox &inBox, [[maybe_unused]] Vec3Arg inPositionCOM, [[maybe_unused]] QuatArg inRotation, [[maybe_unused]] Vec3Arg inScale) const override { /* Do nothing */ }
	virtual int				GetTrianglesNext([[maybe_unused]] GetTrianglesContext &ioContext, [[maybe_unused]] int inMaxTrianglesRequested, [[maybe_unused]] Float3 *outTriangleVertices, [[maybe_unused]] const PhysicsMaterial **outMaterials = nullptr) const override { return 0; }
	Stats					GetStats() const override										{ return { sizeof(*this), 0 }; }
	float					GetVolume() const override										{ return 0.0f; }
	bool					IsValidScale([[maybe_unused]] Vec3Arg inScale) const override	{ return true; }

	// Register shape functions with the registry
	static void				sRegister();

private:
	Vec3					mCenterOfMass = Vec3::sZero();
};

// Class that constructs a BoxShape
class MOSS_EXPORT BoxShapeSettings final : public ConvexShapeSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, BoxShapeSettings)

public:
	/// Default constructor for deserialization
							BoxShapeSettings() = default;

	/// Create a box with half edge length inHalfExtent and convex radius inConvexRadius.
	/// (internally the convex radius will be subtracted from the half extent so the total box will not grow with the convex radius).
							BoxShapeSettings(Vec3Arg inHalfExtent, float inConvexRadius = cDefaultConvexRadius, const PhysicsMaterial *inMaterial = nullptr) : ConvexShapeSettings(inMaterial), mHalfExtent(inHalfExtent), mConvexRadius(inConvexRadius) { }

	// See: ShapeSettings
	virtual ShapeResult		Create() const override;

	Vec3					mHalfExtent = Vec3::sZero();								///< Half the size of the box (including convex radius)
	float					mConvexRadius = 0.0f;
};

/// A box, centered around the origin
class MOSS_EXPORT BoxShape final : public ConvexShape
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
							BoxShape() : ConvexShape(EShapeSubType::Box) { }
							BoxShape(const BoxShapeSettings &inSettings, ShapeResult &outResult);

	/// Create a box with half edge length inHalfExtent and convex radius inConvexRadius.
	/// (internally the convex radius will be subtracted from the half extent so the total box will not grow with the convex radius).
							BoxShape(Vec3Arg inHalfExtent, float inConvexRadius = cDefaultConvexRadius, const PhysicsMaterial *inMaterial = nullptr) : ConvexShape(EShapeSubType::Box, inMaterial), mHalfExtent(inHalfExtent), mConvexRadius(inConvexRadius) { MOSS_ASSERT(inConvexRadius >= 0.0f); MOSS_ASSERT(inHalfExtent.ReduceMin() >= inConvexRadius); }

	/// Get half extent of box
	Vec3					GetHalfExtent() const										{ return mHalfExtent; }

	// See Shape::GetLocalBounds
	virtual AABox			GetLocalBounds() const override								{ return AABox(-mHalfExtent, mHalfExtent); }

	// See Shape::GetInnerRadius
	virtual float			GetInnerRadius() const override								{ return mHalfExtent.ReduceMin(); }

	// See Shape::GetMassProperties
	virtual MassProperties	GetMassProperties() const override;

	// See Shape::GetSurfaceNormal
	virtual Vec3			GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const override;

	// See Shape::GetSupportingFace
	virtual void			GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const override;

	// See ConvexShape::GetSupportFunction
	virtual const Support *	GetSupportFunction(ESupportMode inMode, SupportBuffer &inBuffer, Vec3Arg inScale) const override;

#ifndef MOSS_DEBUG_RENDERER
	// See Shape::Draw
	virtual void			Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const override;
#endif // MOSS_DEBUG_RENDERER

	// See Shape::CastRay
	virtual bool			CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const override;
	virtual void			CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;

	// See: Shape::CollidePoint
	virtual void			CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;

	// See: Shape::CollideSoftBodyVertices
	virtual void			CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const override;

	// See Shape::GetTrianglesStart
	virtual void			GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const override;

	// See Shape::GetTrianglesNext
	virtual int				GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials = nullptr) const override;

	// See Shape
	virtual void			SaveBinaryState(StreamOut &inStream) const override;

	// See Shape::GetStats
	virtual Stats			GetStats() const override									{ return Stats(sizeof(*this), 12); }

	// See Shape::GetVolume
	virtual float			GetVolume() const override									{ return GetLocalBounds().GetVolume(); }

	/// Get the convex radius of this box
	float					GetConvexRadius() const										{ return mConvexRadius; }

	// Register shape functions with the registry
	static void				sRegister();

protected:
	// See: Shape::RestoreBinaryState
	virtual void			RestoreBinaryState(StreamIn &inStream) override;

private:
	// Class for GetSupportFunction
	class					Box;

	Vec3					mHalfExtent = Vec3::sZero();								///< Half the size of the box (including convex radius)
	float					mConvexRadius = 0.0f;
};


// Base class settings to construct a compound shape
class MOSS_EXPORT CompoundShapeSettings : public ShapeSettings
{
	MOSS_DECLARE_SERIALIZABLE_ABSTRACT(MOSS_EXPORT, CompoundShapeSettings)

public:
	/// Constructor. Use AddShape to add the parts.
									CompoundShapeSettings() = default;

	/// Add a shape to the compound.
	void							AddShape(Vec3Arg inPosition, QuatArg inRotation, const ShapeSettings *inShape, uint32 inUserData = 0);

	/// Add a shape to the compound. Variant that uses a concrete shape, which means this object cannot be serialized.
	void							AddShape(Vec3Arg inPosition, QuatArg inRotation, const Shape *inShape, uint32 inUserData = 0);

	struct SubShapeSettings
	{
		MOSS_DECLARE_SERIALIZABLE_NON_VIRTUAL(MOSS_EXPORT, SubShapeSettings)

		RefConst<ShapeSettings>		mShape;													///< Sub shape (either this or mShapePtr needs to be filled up)
		RefConst<Shape>				mShapePtr;												///< Sub shape (either this or mShape needs to be filled up)
		Vec3						mPosition;												///< Position of the sub shape
		Quat						mRotation;												///< Rotation of the sub shape

		/// User data value (can be used by the application for any purpose).
		/// Note this value can be retrieved through GetSubShape(...).mUserData, not through GetSubShapeUserData(...) as that returns Shape::GetUserData() of the leaf shape.
		/// Use GetSubShapeIndexFromID get a shape index from a SubShapeID to pass to GetSubShape.
		uint32						mUserData = 0;
	};

	using SubShapes = TArray<SubShapeSettings>;

	SubShapes						mSubShapes;
};

/// Base class for a compound shape
class MOSS_EXPORT CompoundShape : public Shape
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
	explicit						CompoundShape(EShapeSubType inSubType) : Shape(EShapeType::Compound, inSubType) { }
									CompoundShape(EShapeSubType inSubType, const ShapeSettings &inSettings, ShapeResult &outResult) : Shape(EShapeType::Compound, inSubType, inSettings, outResult) { }

	// See Shape::GetCenterOfMass
	virtual Vec3					GetCenterOfMass() const override						{ return mCenterOfMass; }

	// See Shape::MustBeStatic
	virtual bool					MustBeStatic() const override;

	// See Shape::GetLocalBounds
	virtual AABox					GetLocalBounds() const override							{ return mLocalBounds; }

	// See Shape::GetSubShapeIDBitsRecursive
	virtual uint					GetSubShapeIDBitsRecursive() const override;

	// See Shape::GetWorldSpaceBounds
	virtual AABox					GetWorldSpaceBounds(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale) const override;
	using Shape::GetWorldSpaceBounds;

	// See Shape::GetInnerRadius
	virtual float					GetInnerRadius() const override							{ return mInnerRadius; }

	// See Shape::GetMassProperties
	virtual MassProperties			GetMassProperties() const override;

	// See Shape::GetMaterial
	virtual const PhysicsMaterial *	GetMaterial(const SubShapeID &inSubShapeID) const override;

	// See Shape::GetLeafShape
	virtual const Shape *			GetLeafShape(const SubShapeID &inSubShapeID, SubShapeID &outRemainder) const override;

	// See Shape::GetSubShapeUserData
	virtual uint64					GetSubShapeUserData(const SubShapeID &inSubShapeID) const override;

	// See Shape::GetSubShapeTransformedShape
	virtual TransformedShape		GetSubShapeTransformedShape(const SubShapeID &inSubShapeID, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale, SubShapeID &outRemainder) const override;

	// See Shape::GetSurfaceNormal
	virtual Vec3					GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const override;

	// See Shape::GetSupportingFace
	virtual void					GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const override;

	// See Shape::GetSubmergedVolume
	virtual void					GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy MOSS_IF_DEBUG_RENDERER(, RVec3Arg inBaseOffset)) const override;

#ifndef MOSS_DEBUG_RENDERER
	// See Shape::Draw
	virtual void					Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const override;

	// See Shape::DrawGetSupportFunction
	virtual void					DrawGetSupportFunction(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inDrawSupportDirection) const override;

	// See Shape::DrawGetSupportingFace
	virtual void					DrawGetSupportingFace(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale) const override;
#endif // MOSS_DEBUG_RENDERER

	// See: Shape::CollideSoftBodyVertices
	virtual void					CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const override;

	// See Shape::TransformShape
	virtual void					TransformShape(Mat44Arg inCenterOfMassTransform, TransformedShapeCollector &ioCollector) const override;

	// See Shape::GetTrianglesStart
	virtual void					GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const override { MOSS_ASSERT(false, "Cannot call on non-leaf shapes, use CollectTransformedShapes to collect the leaves first!"); }

	// See Shape::GetTrianglesNext
	virtual int						GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials = nullptr) const override { MOSS_ASSERT(false, "Cannot call on non-leaf shapes, use CollectTransformedShapes to collect the leaves first!"); return 0; }

	/// Get which sub shape's bounding boxes overlap with an axis aligned box
	/// @param inBox The axis aligned box to test against (relative to the center of mass of this shape)
	/// @param outSubShapeIndices Buffer where to place the indices of the sub shapes that intersect
	/// @param inMaxSubShapeIndices How many indices will fit in the buffer (normally you'd provide a buffer of GetNumSubShapes() indices)
	/// @return How many indices were placed in outSubShapeIndices
	virtual int						GetIntersectingSubShapes(const AABox &inBox, uint *outSubShapeIndices, int inMaxSubShapeIndices) const = 0;

	/// Get which sub shape's bounding boxes overlap with an axis aligned box
	/// @param inBox The axis aligned box to test against (relative to the center of mass of this shape)
	/// @param outSubShapeIndices Buffer where to place the indices of the sub shapes that intersect
	/// @param inMaxSubShapeIndices How many indices will fit in the buffer (normally you'd provide a buffer of GetNumSubShapes() indices)
	/// @return How many indices were placed in outSubShapeIndices
	virtual int						GetIntersectingSubShapes(const OrientedBox &inBox, uint *outSubShapeIndices, int inMaxSubShapeIndices) const = 0;

	struct SubShape
	{
		/// Initialize sub shape from sub shape settings
		/// @param inSettings Settings object
		/// @param outResult Result object, only used in case of error
		/// @return True on success, false on failure
		bool						FromSettings(const CompoundShapeSettings::SubShapeSettings &inSettings, ShapeResult &outResult)
		{
			if (inSettings.mShapePtr != nullptr)
			{
				// Use provided shape
				mShape = inSettings.mShapePtr;
			}
			else
			{
				// Create child shape
				ShapeResult child_result = inSettings.mShape->Create();
				if (!child_result.IsValid())
				{
					outResult = child_result;
					return false;
				}
				mShape = child_result.Get();
			}

			// Copy user data
			mUserData = inSettings.mUserData;

			SetTransform(inSettings.mPosition, inSettings.mRotation, Vec3::sZero() /* Center of mass not yet calculated */);
			return true;
		}

		/// Update the transform of this sub shape
		/// @param inPosition New position
		/// @param inRotation New orientation
		/// @param inCenterOfMass The center of mass of the compound shape
		MOSS_INLINE void				SetTransform(Vec3Arg inPosition, QuatArg inRotation, Vec3Arg inCenterOfMass)
		{
			SetPositionCOM(inPosition - inCenterOfMass + inRotation * mShape->GetCenterOfMass());

			mIsRotationIdentity = inRotation.IsClose(Quat::sIdentity()) || inRotation.IsClose(-Quat::sIdentity());
			SetRotation(mIsRotationIdentity? Quat::sIdentity() : inRotation);
		}

		/// Get the local transform for this shape given the scale of the child shape
		/// The total transform of the child shape will be GetLocalTransformNoScale(inScale) * Mat44::sScaling(TransformScale(inScale))
		/// @param inScale The scale of the child shape (in local space of this shape)
		MOSS_INLINE Mat44			GetLocalTransformNoScale(Vec3Arg inScale) const
		{
			MOSS_ASSERT(IsValidScale(inScale));
			return Mat44::sRotationTranslation(GetRotation(), inScale * GetPositionCOM());
		}

		/// Test if inScale is valid for this sub shape
		inline bool					IsValidScale(Vec3Arg inScale) const
		{
			// We can always handle uniform scale or identity rotations
			if (mIsRotationIdentity || ScaleHelpers::IsUniformScale(inScale))
				return true;

			return ScaleHelpers::CanScaleBeRotated(GetRotation(), inScale);
		}

		/// Transform the scale to the local space of the child shape
		inline Vec3					TransformScale(Vec3Arg inScale) const
		{
			// We don't need to transform uniform scale or if the rotation is identity
			if (mIsRotationIdentity || ScaleHelpers::IsUniformScale(inScale))
				return inScale;

			return ScaleHelpers::RotateScale(GetRotation(), inScale);
		}

		/// Compress the center of mass position
		MOSS_INLINE void				SetPositionCOM(Vec3Arg inPositionCOM)
		{
			inPositionCOM.StoreFloat3(&mPositionCOM);
		}

		/// Uncompress the center of mass position
		MOSS_INLINE Vec3				GetPositionCOM() const
		{
			return Vec3::sLoadFloat3Unsafe(mPositionCOM);
		}

		/// Compress the rotation
		MOSS_INLINE void				SetRotation(QuatArg inRotation)
		{
			inRotation.StoreFloat3(&mRotation);
		}

		/// Uncompress the rotation
		MOSS_INLINE Quat				GetRotation() const
		{
			return mIsRotationIdentity? Quat::sIdentity() : Quat::sLoadFloat3Unsafe(mRotation);
		}

		RefConst<Shape>				mShape;
		Float3						mPositionCOM;											///< Note: Position of center of mass of sub shape!
		Float3						mRotation;												///< Note: X, Y, Z of rotation quaternion - note we read 4 bytes beyond this so make sure there's something there
		uint32						mUserData;												///< User data value (put here because it falls in padding bytes)
		bool						mIsRotationIdentity;									///< If mRotation is close to identity (put here because it falls in padding bytes)
		// 3 padding bytes left
	};

	static_assert(sizeof(SubShape) == (MOSS_CPU_ADDRESS_BITS == 64? 40 : 36), "Compiler added unexpected padding");

	using SubShapes = TArray<SubShape>;

	/// Access to the sub shapes of this compound
	const SubShapes &				GetSubShapes() const									{ return mSubShapes; }

	/// Get the total number of sub shapes
	uint							GetNumSubShapes() const									{ return uint(mSubShapes.size()); }

	/// Access to a particular sub shape
	const SubShape &				GetSubShape(uint inIdx) const							{ return mSubShapes[inIdx]; }

	/// Get the user data associated with a shape in this compound
	uint32							GetCompoundUserData(uint inIdx) const					{ return mSubShapes[inIdx].mUserData; }

	/// Set the user data associated with a shape in this compound
	void							SetCompoundUserData(uint inIdx, uint32 inUserData)		{ mSubShapes[inIdx].mUserData = inUserData; }

	/// Check if a sub shape ID is still valid for this shape
	/// @param inSubShapeID Sub shape id that indicates the leaf shape relative to this shape
	/// @return True if the ID is valid, false if not
	inline bool						IsSubShapeIDValid(SubShapeID inSubShapeID) const
	{
		SubShapeID remainder;
		return inSubShapeID.PopID(GetSubShapeIDBits(), remainder) < mSubShapes.size();
	}

	/// Convert SubShapeID to sub shape index
	/// @param inSubShapeID Sub shape id that indicates the leaf shape relative to this shape
	/// @param outRemainder This is the sub shape ID for the sub shape of the compound after popping off the index
	/// @return The index of the sub shape of this compound
	inline uint32					GetSubShapeIndexFromID(SubShapeID inSubShapeID, SubShapeID &outRemainder) const
	{
		uint32 idx = inSubShapeID.PopID(GetSubShapeIDBits(), outRemainder);
		MOSS_ASSERT(idx < mSubShapes.size(), "Invalid SubShapeID");
		return idx;
	}

	/// @brief Convert a sub shape index to a sub shape ID
	/// @param inIdx Index of the sub shape of this compound
	/// @param inParentSubShapeID Parent SubShapeID (describing the path to the compound shape)
	/// @return A sub shape ID creator that contains the full path to the sub shape with index inIdx
	inline SubShapeIDCreator		GetSubShapeIDFromIndex(int inIdx, const SubShapeIDCreator &inParentSubShapeID) const
	{
		return inParentSubShapeID.PushID(inIdx, GetSubShapeIDBits());
	}

	// See Shape
	virtual void					SaveBinaryState(StreamOut &inStream) const override;
	virtual void					SaveSubShapeState(ShapeList &outSubShapes) const override;
	virtual void					RestoreSubShapeState(const ShapeRefC *inSubShapes, uint inNumShapes) override;

	// See Shape::GetStatsRecursive
	virtual Stats					GetStatsRecursive(VisitedShapes &ioVisitedShapes) const override;

	// See Shape::GetVolume
	virtual float					GetVolume() const override;

	// See Shape::IsValidScale
	virtual bool					IsValidScale(Vec3Arg inScale) const override;

	// See Shape::MakeScaleValid
	virtual Vec3					MakeScaleValid(Vec3Arg inScale) const override;

	// Register shape functions with the registry
	static void						sRegister();

protected:
	// See: Shape::RestoreBinaryState
	virtual void					RestoreBinaryState(StreamIn &inStream) override;

	// Visitors for collision detection
	struct CastRayVisitor;
	struct CastRayVisitorCollector;
	struct CollidePointVisitor;
	struct CastShapeVisitor;
	struct CollectTransformedShapesVisitor;
	struct CollideCompoundVsShapeVisitor;
	struct CollideShapeVsCompoundVisitor;
	template <class BoxType> struct GetIntersectingSubShapesVisitor;

	/// Determine amount of bits needed to encode sub shape id
	inline uint						GetSubShapeIDBits() const
	{
		// Ensure we have enough bits to encode our shape [0, n - 1]
		uint32 n = uint32(mSubShapes.size()) - 1;
		return 32 - CountLeadingZeros(n);
	}

	/// Determine the inner radius of this shape
	inline void						CalculateInnerRadius()
	{
		mInnerRadius = FLT_MAX;
		for (const SubShape &s : mSubShapes)
			mInnerRadius = min(mInnerRadius, s.mShape->GetInnerRadius());
	}

	Vec3							mCenterOfMass { Vec3::sZero() };						///< Center of mass of the compound
	AABox							mLocalBounds { Vec3::sZero(), Vec3::sZero() };
	SubShapes						mSubShapes;
	float							mInnerRadius = FLT_MAX;									///< Smallest radius of GetInnerRadius() of child shapes

private:
	// Helper functions called by CollisionDispatch
	static void						sCastCompoundVsShape(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector);
};

// Class that constructs a RotatedTranslatedShape
class MOSS_EXPORT RotatedTranslatedShapeSettings final : public DecoratedShapeSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, RotatedTranslatedShapeSettings)

public:
	/// Constructor
									RotatedTranslatedShapeSettings() = default;

	/// Construct with shape settings, can be serialized.
									RotatedTranslatedShapeSettings(Vec3Arg inPosition, QuatArg inRotation, const ShapeSettings *inShape) : DecoratedShapeSettings(inShape), mPosition(inPosition), mRotation(inRotation) { }

	/// Variant that uses a concrete shape, which means this object cannot be serialized.
									RotatedTranslatedShapeSettings(Vec3Arg inPosition, QuatArg inRotation, const Shape *inShape): DecoratedShapeSettings(inShape), mPosition(inPosition), mRotation(inRotation) { }

	// See: ShapeSettings
	virtual ShapeResult				Create() const override;

	Vec3							mPosition;												///< Position of the sub shape
	Quat							mRotation;												///< Rotation of the sub shape
};

/// A rotated translated shape will rotate and translate a child shape.
/// Shifts the child object so that it is centered around the center of mass.
class MOSS_EXPORT RotatedTranslatedShape final : public DecoratedShape
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
									RotatedTranslatedShape() : DecoratedShape(EShapeSubType::RotatedTranslated) { }
									RotatedTranslatedShape(const RotatedTranslatedShapeSettings &inSettings, ShapeResult &outResult);
									RotatedTranslatedShape(Vec3Arg inPosition, QuatArg inRotation, const Shape *inShape);

	/// Access the rotation that is applied to the inner shape
	Quat							GetRotation() const										{ return mRotation; }

	/// Access the translation that has been applied to the inner shape
	Vec3							GetPosition() const										{ return mCenterOfMass - mRotation * mInnerShape->GetCenterOfMass(); }

	// See Shape::GetCenterOfMass
	virtual Vec3					GetCenterOfMass() const override						{ return mCenterOfMass; }

	// See Shape::GetLocalBounds
	virtual AABox					GetLocalBounds() const override;

	// See Shape::GetWorldSpaceBounds
	virtual AABox					GetWorldSpaceBounds(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale) const override;
	using Shape::GetWorldSpaceBounds;

	// See Shape::GetInnerRadius
	virtual float					GetInnerRadius() const override							{ return mInnerShape->GetInnerRadius(); }

	// See Shape::GetMassProperties
	virtual MassProperties			GetMassProperties() const override;

	// See Shape::GetSubShapeTransformedShape
	virtual TransformedShape		GetSubShapeTransformedShape(const SubShapeID &inSubShapeID, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale, SubShapeID &outRemainder) const override;

	// See Shape::GetSurfaceNormal
	virtual Vec3					GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const override;

	// See Shape::GetSupportingFace
	virtual void					GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const override;

	// See Shape::GetSubmergedVolume
	virtual void					GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy MOSS_IF_DEBUG_RENDERER(, RVec3Arg inBaseOffset)) const override;

#ifndef MOSS_DEBUG_RENDERER
	// See Shape::Draw
	virtual void					Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const override;

	// See Shape::DrawGetSupportFunction
	virtual void					DrawGetSupportFunction(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inDrawSupportDirection) const override;

	// See Shape::DrawGetSupportingFace
	virtual void					DrawGetSupportingFace(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale) const override;
#endif // MOSS_DEBUG_RENDERER

	// See Shape::CastRay
	virtual bool					CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const override;
	virtual void					CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;

	// See: Shape::CollidePoint
	virtual void					CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;

	// See: Shape::CollideSoftBodyVertices
	virtual void					CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const override;

	// See Shape::CollectTransformedShapes
	virtual void					CollectTransformedShapes(const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale, const SubShapeIDCreator &inSubShapeIDCreator, TransformedShapeCollector &ioCollector, const ShapeFilter &inShapeFilter) const override;

	// See Shape::TransformShape
	virtual void					TransformShape(Mat44Arg inCenterOfMassTransform, TransformedShapeCollector &ioCollector) const override;

	// See Shape::GetTrianglesStart
	virtual void					GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const override { MOSS_ASSERT(false, "Cannot call on non-leaf shapes, use CollectTransformedShapes to collect the leaves first!"); }

	// See Shape::GetTrianglesNext
	virtual int						GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials = nullptr) const override { MOSS_ASSERT(false, "Cannot call on non-leaf shapes, use CollectTransformedShapes to collect the leaves first!"); return 0; }

	// See Shape
	virtual void					SaveBinaryState(StreamOut &inStream) const override;

	// See Shape::GetStats
	virtual Stats					GetStats() const override								{ return Stats(sizeof(*this), 0); }

	// See Shape::GetVolume
	virtual float					GetVolume() const override								{ return mInnerShape->GetVolume(); }

	// See Shape::IsValidScale
	virtual bool					IsValidScale(Vec3Arg inScale) const override;

	// See Shape::MakeScaleValid
	virtual Vec3					MakeScaleValid(Vec3Arg inScale) const override;

	/// Transform the scale to the local space of the child shape
	inline Vec3						TransformScale(Vec3Arg inScale) const
	{
		// We don't need to transform uniform scale or if the rotation is identity
		if (mIsRotationIdentity || ScaleHelpers::IsUniformScale(inScale))
			return inScale;

		return ScaleHelpers::RotateScale(mRotation, inScale);
	}

	// Register shape functions with the registry
	static void						sRegister();

protected:
	// See: Shape::RestoreBinaryState
	virtual void					RestoreBinaryState(StreamIn &inStream) override;

private:
	// Helper functions called by CollisionDispatch
	static void						sCollideRotatedTranslatedVsShape(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter);
	static void						sCollideShapeVsRotatedTranslated(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter);
	static void						sCollideRotatedTranslatedVsRotatedTranslated(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter);
	static void						sCastRotatedTranslatedVsShape(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector);
	static void						sCastShapeVsRotatedTranslated(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector);
	static void						sCastRotatedTranslatedVsRotatedTranslated(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector);

	bool							mIsRotationIdentity;									///< If mRotation is close to identity (put here because it falls in padding bytes)
	Vec3							mCenterOfMass;											///< Position of the center of mass
	Quat							mRotation;												///< Rotation of the child shape
};

// Helper functions to get properties of a scaling vector
namespace ScaleHelpers
{
	/// Minimum valid scale value. This is used to prevent division by zero when scaling a shape with a zero scale.
	static constexpr float	cMinScale = 1.0e-6f;

	/// The tolerance used to check if components of the scale vector are the same
	static constexpr float	cScaleToleranceSq = 1.0e-8f;

	/// Test if a scale is identity
	inline bool				IsNotScaled(Vec3Arg inScale)									{ return inScale.IsClose(Vec3::sOne(), cScaleToleranceSq); }

	/// Test if a scale is uniform
	inline bool				IsUniformScale(Vec3Arg inScale)									{ return inScale.Swizzle<SWIZZLE_Y, SWIZZLE_Z, SWIZZLE_X>().IsClose(inScale, cScaleToleranceSq); }

	/// Test if a scale is uniform in XZ
	inline bool				IsUniformScaleXZ(Vec3Arg inScale)								{ return inScale.Swizzle<SWIZZLE_Z, SWIZZLE_Y, SWIZZLE_X>().IsClose(inScale, ScaleHelpers::cScaleToleranceSq); }

	/// Scale the convex radius of an object
	inline float			ScaleConvexRadius(float inConvexRadius, Vec3Arg inScale)		{ return min(inConvexRadius * inScale.Abs().ReduceMin(), cDefaultConvexRadius); }

	/// Test if a scale flips an object inside out (which requires flipping all normals and polygon windings)
	inline bool				IsInsideOut(Vec3Arg inScale)									{ return (CountBits(Vec3::sLess(inScale, Vec3::sZero()).GetTrues() & 0x7) & 1) != 0; }

	/// Test if any of the components of the scale have a value below cMinScale
	inline bool				IsZeroScale(Vec3Arg inScale)									{ return Vec3::sLess(inScale.Abs(), Vec3::sReplicate(cMinScale)).TestAnyXYZTrue(); }

	/// Ensure that the scale for each component is at least cMinScale
	inline Vec3				MakeNonZeroScale(Vec3Arg inScale)								{ return inScale.GetSign() * Vec3::sMax(inScale.Abs(), Vec3::sReplicate(cMinScale)); }

	/// Get the average scale if inScale, used to make the scale uniform when a shape doesn't support non-uniform scale
	inline Vec3				MakeUniformScale(Vec3Arg inScale)								{ return Vec3::sReplicate((inScale.GetX() + inScale.GetY() + inScale.GetZ()) / 3.0f); }

	/// Average the scale in XZ, used to make the scale uniform when a shape doesn't support non-uniform scale in the XZ plane
	inline Vec3				MakeUniformScaleXZ(Vec3Arg inScale)								{ return 0.5f * (inScale + inScale.Swizzle<SWIZZLE_Z, SWIZZLE_Y, SWIZZLE_X>()); }

	/// Checks in scale can be rotated to child shape
	/// @param inRotation Rotation of child shape
	/// @param inScale Scale in local space of parent shape
	/// @return True if the scale is valid (no shearing introduced)
	inline bool				CanScaleBeRotated(QuatArg inRotation, Vec3Arg inScale)
	{
		// inScale is a scale in local space of the shape, so the transform for the shape (ignoring translation) is: T = Mat44::sScale(inScale) * mRotation.
		// when we pass the scale to the child it needs to be local to the child, so we want T = mRotation * Mat44::sScale(ChildScale).
		// Solving for ChildScale: ChildScale = mRotation^-1 * Mat44::sScale(inScale) * mRotation = mRotation^T * Mat44::sScale(inScale) * mRotation
		// If any of the off diagonal elements are non-zero, it means the scale / rotation is not compatible.
		Mat44 r = Mat44::sRotation(inRotation);
		Mat44 child_scale = r.Multiply3x3LeftTransposed(r.PostScaled(inScale));

		// Get the columns, but zero the diagonal
		Vec4 zero = Vec4::sZero();
		Vec4 c0 = Vec4::sSelect(child_scale.GetColumn4(0), zero, UVec4(0xffffffff, 0, 0, 0)).Abs();
		Vec4 c1 = Vec4::sSelect(child_scale.GetColumn4(1), zero, UVec4(0, 0xffffffff, 0, 0)).Abs();
		Vec4 c2 = Vec4::sSelect(child_scale.GetColumn4(2), zero, UVec4(0, 0, 0xffffffff, 0)).Abs();

		// Check if all elements are less than epsilon
		Vec4 epsilon = Vec4::sReplicate(1.0e-6f);
		return UVec4::sAnd(UVec4::sAnd(Vec4::sLess(c0, epsilon), Vec4::sLess(c1, epsilon)), Vec4::sLess(c2, epsilon)).TestAllTrue();
	}

	/// Adjust scale for rotated child shape
	/// @param inRotation Rotation of child shape
	/// @param inScale Scale in local space of parent shape
	/// @return Rotated scale
	inline Vec3				RotateScale(QuatArg inRotation, Vec3Arg inScale)
	{
		// Get the diagonal of mRotation^T * Mat44::sScale(inScale) * mRotation (see comment at CanScaleBeRotated)
		Mat44 r = Mat44::sRotation(inRotation);
		return r.Multiply3x3LeftTransposed(r.PostScaled(inScale)).GetDiagonal3();
	}
}

// Class that constructs a ScaledShape
class MOSS_EXPORT ScaledShapeSettings final : public DecoratedShapeSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, ScaledShapeSettings)

public:
	/// Default constructor for deserialization
									ScaledShapeSettings() = default;

	/// Constructor that decorates another shape with a scale
									ScaledShapeSettings(const ShapeSettings *inShape, Vec3Arg inScale) : DecoratedShapeSettings(inShape), mScale(inScale) { }

	/// Variant that uses a concrete shape, which means this object cannot be serialized.
									ScaledShapeSettings(const Shape *inShape, Vec3Arg inScale) : DecoratedShapeSettings(inShape), mScale(inScale) { }

	// See: ShapeSettings
	virtual ShapeResult				Create() const override;

	Vec3							mScale = Vec3(1, 1, 1);
};

/// A shape that scales a child shape in local space of that shape. The scale can be non-uniform and can even turn it inside out when one or three components of the scale are negative.
class MOSS_EXPORT ScaledShape final : public DecoratedShape
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
									ScaledShape() : DecoratedShape(EShapeSubType::Scaled) { }
									ScaledShape(const ScaledShapeSettings &inSettings, ShapeResult &outResult);

	/// Constructor that decorates another shape with a scale
									ScaledShape(const Shape *inShape, Vec3Arg inScale)		: DecoratedShape(EShapeSubType::Scaled, inShape), mScale(inScale) { MOSS_ASSERT(!ScaleHelpers::IsZeroScale(mScale)); }

	/// Get the scale
	Vec3							GetScale() const										{ return mScale; }

	// See Shape::GetCenterOfMass
	virtual Vec3					GetCenterOfMass() const override						{ return mScale * mInnerShape->GetCenterOfMass(); }

	// See Shape::GetLocalBounds
	virtual AABox					GetLocalBounds() const override;

	// See Shape::GetWorldSpaceBounds
	virtual AABox					GetWorldSpaceBounds(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale) const override;
	using Shape::GetWorldSpaceBounds;

	// See Shape::GetInnerRadius
	virtual float					GetInnerRadius() const override							{ return mScale.ReduceMin() * mInnerShape->GetInnerRadius(); }

	// See Shape::GetMassProperties
	virtual MassProperties			GetMassProperties() const override;

	// See Shape::GetSubShapeTransformedShape
	virtual TransformedShape		GetSubShapeTransformedShape(const SubShapeID &inSubShapeID, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale, SubShapeID &outRemainder) const override;

	// See Shape::GetSurfaceNormal
	virtual Vec3					GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const override;

	// See Shape::GetSupportingFace
	virtual void					GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const override;

	// See Shape::GetSubmergedVolume
	virtual void					GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy MOSS_IF_DEBUG_RENDERER(, RVec3Arg inBaseOffset)) const override;

#ifndef MOSS_DEBUG_RENDERER
	// See Shape::Draw
	virtual void					Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const override;

	// See Shape::DrawGetSupportFunction
	virtual void					DrawGetSupportFunction(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inDrawSupportDirection) const override;

	// See Shape::DrawGetSupportingFace
	virtual void					DrawGetSupportingFace(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale) const override;
#endif // MOSS_DEBUG_RENDERER

	// See Shape::CastRay
	virtual bool					CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const override;
	virtual void					CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;

	// See: Shape::CollidePoint
	virtual void					CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;

	// See: Shape::CollideSoftBodyVertices
	virtual void					CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const override;

	// See Shape::CollectTransformedShapes
	virtual void					CollectTransformedShapes(const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale, const SubShapeIDCreator &inSubShapeIDCreator, TransformedShapeCollector &ioCollector, const ShapeFilter &inShapeFilter) const override;

	// See Shape::TransformShape
	virtual void					TransformShape(Mat44Arg inCenterOfMassTransform, TransformedShapeCollector &ioCollector) const override;

	// See Shape::GetTrianglesStart
	virtual void					GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const override { MOSS_ASSERT(false, "Cannot call on non-leaf shapes, use CollectTransformedShapes to collect the leaves first!"); }

	// See Shape::GetTrianglesNext
	virtual int						GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials = nullptr) const override { MOSS_ASSERT(false, "Cannot call on non-leaf shapes, use CollectTransformedShapes to collect the leaves first!"); return 0; }

	// See Shape
	virtual void					SaveBinaryState(StreamOut &inStream) const override;

	// See Shape::GetStats
	virtual Stats					GetStats() const override								{ return Stats(sizeof(*this), 0); }

	// See Shape::GetVolume
	virtual float					GetVolume() const override;

	// See Shape::IsValidScale
	virtual bool					IsValidScale(Vec3Arg inScale) const override;

	// See Shape::MakeScaleValid
	virtual Vec3					MakeScaleValid(Vec3Arg inScale) const override;

	// Register shape functions with the registry
	static void						sRegister();

protected:
	// See: Shape::RestoreBinaryState
	virtual void					RestoreBinaryState(StreamIn &inStream) override;

private:
	// Helper functions called by CollisionDispatch
	static void						sCollideScaledVsShape(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter);
	static void						sCollideShapeVsScaled(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter);
	static void						sCastScaledVsShape(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector);
	static void						sCastShapeVsScaled(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector);

	Vec3							mScale = Vec3(1, 1, 1);
};


// Class that constructs a SphereShape
class MOSS_EXPORT SphereShapeSettings final : public ConvexShapeSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, SphereShapeSettings)

public:
	/// Default constructor for deserialization
							SphereShapeSettings() = default;

	/// Create a sphere with radius inRadius
							SphereShapeSettings(float inRadius, const PhysicsMaterial *inMaterial = nullptr)	: ConvexShapeSettings(inMaterial), mRadius(inRadius) { }

	// See: ShapeSettings
	virtual ShapeResult		Create() const override;

	float					mRadius = 0.0f;
};

/// A sphere, centered around the origin.
/// Note that it is implemented as a point with convex radius.
class MOSS_EXPORT SphereShape final : public ConvexShape
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
							SphereShape() : ConvexShape(EShapeSubType::Sphere) { }
							SphereShape(const SphereShapeSettings &inSettings, ShapeResult &outResult);

	/// Create a sphere with radius inRadius
							SphereShape(float inRadius, const PhysicsMaterial *inMaterial = nullptr)			: ConvexShape(EShapeSubType::Sphere, inMaterial), mRadius(inRadius) { MOSS_ASSERT(inRadius > 0.0f); }

	/// Radius of the sphere
	float					GetRadius() const																	{ return mRadius; }

	// See Shape::GetLocalBounds
	virtual AABox			GetLocalBounds() const override;

	// See Shape::GetWorldSpaceBounds
	virtual AABox			GetWorldSpaceBounds(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale) const override;
	using Shape::GetWorldSpaceBounds;

	// See Shape::GetInnerRadius
	virtual float			GetInnerRadius() const override														{ return mRadius; }

	// See Shape::GetMassProperties
	virtual MassProperties	GetMassProperties() const override;

	// See Shape::GetSurfaceNormal
	virtual Vec3			GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const override;

	// See Shape::GetSupportingFace
	virtual void			GetSupportingFace([[maybe_unused]] const SubShapeID &inSubShapeID, [[maybe_unused]] Vec3Arg inDirection, [[maybe_unused]] Vec3Arg inScale, [[maybe_unused]] Mat44Arg inCenterOfMassTransform, [[maybe_unused]] SupportingFace &outVertices) const override { /* Hit is always a single point, no point in returning anything */ }

	// See ConvexShape::GetSupportFunction
	virtual const Support *	GetSupportFunction(ESupportMode inMode, SupportBuffer &inBuffer, Vec3Arg inScale) const override;

	// See Shape::GetSubmergedVolume
	virtual void			GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy MOSS_IF_DEBUG_RENDERER(, RVec3Arg inBaseOffset)) const override;

#ifndef MOSS_DEBUG_RENDERER
	// See Shape::Draw
	virtual void			Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const override;
#endif // MOSS_DEBUG_RENDERER

	// See Shape::CastRay
	virtual bool			CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const override;
	virtual void			CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;

	// See: Shape::CollidePoint
	virtual void			CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;

	// See: Shape::CollideSoftBodyVertices
	virtual void			CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const override;

	// See Shape::GetTrianglesStart
	virtual void			GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const override;

	// See Shape::GetTrianglesNext
	virtual int				GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials = nullptr) const override;

	// See Shape
	virtual void			SaveBinaryState(StreamOut &inStream) const override;

	// See Shape::GetStats
	virtual Stats			GetStats() const override															{ return Stats(sizeof(*this), 0); }

	// See Shape::GetVolume
	virtual float			GetVolume() const override															{ return 4.0f / 3.0f * MOSS_PI * Cubed(mRadius); }

	// See Shape::IsValidScale
	virtual bool			IsValidScale(Vec3Arg inScale) const override;

	// See Shape::MakeScaleValid
	virtual Vec3			MakeScaleValid(Vec3Arg inScale) const override;

	// Register shape functions with the registry
	static void				sRegister();

protected:
	// See: Shape::RestoreBinaryState
	virtual void			RestoreBinaryState(StreamIn &inStream) override;

private:
	// Get the radius of this sphere scaled by inScale
	inline float			GetScaledRadius(Vec3Arg inScale) const;

	// Classes for GetSupportFunction
	class					SphereNoConvex;
	class					SphereWithConvex;

	float					mRadius = 0.0f;
};

// Class that constructs a TriangleShape
class MOSS_EXPORT TriangleShapeSettings final : public ConvexShapeSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, TriangleShapeSettings)

public:
	/// Default constructor for deserialization
							TriangleShapeSettings() = default;

	/// Create a triangle with points (inV1, inV2, inV3) (counter clockwise) and convex radius inConvexRadius.
	/// Note that the convex radius is currently only used for shape vs shape collision, for all other purposes the triangle is infinitely thin.
							TriangleShapeSettings(Vec3Arg inV1, Vec3Arg inV2, Vec3Arg inV3, float inConvexRadius = 0.0f, const PhysicsMaterial *inMaterial = nullptr) : ConvexShapeSettings(inMaterial), mV1(inV1), mV2(inV2), mV3(inV3), mConvexRadius(inConvexRadius) { }

	// See: ShapeSettings
	virtual ShapeResult		Create() const override;

	Vec3					mV1;
	Vec3					mV2;
	Vec3					mV3;
	float					mConvexRadius = 0.0f;
};

/// A single triangle, not the most efficient way of creating a world filled with triangles but can be used as a query shape for example.
class MOSS_EXPORT TriangleShape final : public ConvexShape
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
							TriangleShape() : ConvexShape(EShapeSubType::Triangle) { }
							TriangleShape(const TriangleShapeSettings &inSettings, ShapeResult &outResult);

	/// Create a triangle with points (inV1, inV2, inV3) (counter clockwise) and convex radius inConvexRadius.
	/// Note that the convex radius is currently only used for shape vs shape collision, for all other purposes the triangle is infinitely thin.
							TriangleShape(Vec3Arg inV1, Vec3Arg inV2, Vec3Arg inV3, float inConvexRadius = 0.0f, const PhysicsMaterial *inMaterial = nullptr) : ConvexShape(EShapeSubType::Triangle, inMaterial), mV1(inV1), mV2(inV2), mV3(inV3), mConvexRadius(inConvexRadius) { MOSS_ASSERT(inConvexRadius >= 0.0f); }

	/// Get the vertices of the triangle
	inline Vec3				GetVertex1() const																	{ return mV1; }
	inline Vec3				GetVertex2() const																	{ return mV2; }
	inline Vec3				GetVertex3() const																	{ return mV3; }

	/// Convex radius
	float					GetConvexRadius() const																{ return mConvexRadius; }

	// See Shape::GetLocalBounds
	virtual AABox			GetLocalBounds() const override;

	// See Shape::GetWorldSpaceBounds
	virtual AABox			GetWorldSpaceBounds(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale) const override;
	using Shape::GetWorldSpaceBounds;

	// See Shape::GetInnerRadius
	virtual float			GetInnerRadius() const override														{ return mConvexRadius; }

	// See Shape::GetMassProperties
	virtual MassProperties	GetMassProperties() const override;

	// See Shape::GetSurfaceNormal
	virtual Vec3			GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const override;

	// See Shape::GetSupportingFace
	virtual void			GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const override;

	// See ConvexShape::GetSupportFunction
	virtual const Support *	GetSupportFunction(ESupportMode inMode, SupportBuffer &inBuffer, Vec3Arg inScale) const override;

	// See Shape::GetSubmergedVolume
	virtual void			GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy MOSS_IF_DEBUG_RENDERER(, RVec3Arg inBaseOffset)) const override;

#ifndef MOSS_DEBUG_RENDERER
	// See Shape::Draw
	virtual void			Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const override;
#endif // MOSS_DEBUG_RENDERER

	// See Shape::CastRay
	virtual bool			CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const override;
	virtual void			CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;

	// See: Shape::CollidePoint
	virtual void			CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;

	// See: Shape::CollideSoftBodyVertices
	virtual void			CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const override;

	// See Shape::GetTrianglesStart
	virtual void			GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const override;

	// See Shape::GetTrianglesNext
	virtual int				GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials = nullptr) const override;

	// See Shape
	virtual void			SaveBinaryState(StreamOut &inStream) const override;

	// See Shape::GetStats
	virtual Stats			GetStats() const override															{ return Stats(sizeof(*this), 1); }

	// See Shape::GetVolume
	virtual float			GetVolume() const override															{ return 0; }

	// See Shape::IsValidScale
	virtual bool			IsValidScale(Vec3Arg inScale) const override;

	// See Shape::MakeScaleValid
	virtual Vec3			MakeScaleValid(Vec3Arg inScale) const override;

	// Register shape functions with the registry
	static void				sRegister();

protected:
	// See: Shape::RestoreBinaryState
	virtual void			RestoreBinaryState(StreamIn &inStream) override;

private:
	// Helper functions called by CollisionDispatch
	static void				sCollideConvexVsTriangle(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter);
	static void				sCollideSphereVsTriangle(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter);
	static void				sCastConvexVsTriangle(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector);
	static void				sCastSphereVsTriangle(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector);

	// Context for GetTrianglesStart/Next
	class					TSGetTrianglesContext;

	// Classes for GetSupportFunction
	class					TriangleNoConvex;
	class					TriangleWithConvex;

	Vec3					mV1;
	Vec3					mV2;
	Vec3					mV3;
	float					mConvexRadius = 0.0f;
};


/// Class that constructs a TaperedCapsuleShape
class MOSS_EXPORT TaperedCapsuleShapeSettings final : public ConvexShapeSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, TaperedCapsuleShapeSettings)

public:
	/// Default constructor for deserialization
							TaperedCapsuleShapeSettings() = default;

	/// Create a tapered capsule centered around the origin with one sphere cap at (0, -inHalfHeightOfTaperedCylinder, 0) with radius inBottomRadius and the other at (0, inHalfHeightOfTaperedCylinder, 0) with radius inTopRadius
							TaperedCapsuleShapeSettings(float inHalfHeightOfTaperedCylinder, float inTopRadius, float inBottomRadius, const PhysicsMaterial *inMaterial = nullptr);

	/// Check if the settings are valid
	bool					IsValid() const															{ return mTopRadius > 0.0f && mBottomRadius > 0.0f && mHalfHeightOfTaperedCylinder >= 0.0f; }

	/// Checks if the settings of this tapered capsule make this shape a sphere
	bool					IsSphere() const;

	// See: ShapeSettings
	virtual ShapeResult		Create() const override;

	float					mHalfHeightOfTaperedCylinder = 0.0f;
	float					mTopRadius = 0.0f;
	float					mBottomRadius = 0.0f;
};

/// A capsule with different top and bottom radii
class MOSS_EXPORT TaperedCapsuleShape final : public ConvexShape
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
							TaperedCapsuleShape() : ConvexShape(EShapeSubType::TaperedCapsule) { }
							TaperedCapsuleShape(const TaperedCapsuleShapeSettings &inSettings, ShapeResult &outResult);

	/// Get top radius of the tapered capsule
	inline float			GetTopRadius() const													{ return mTopRadius; }

	/// Get bottom radius of the tapered capsule
	inline float			GetBottomRadius() const													{ return mBottomRadius; }

	/// Get half height between the top and bottom sphere center
	inline float			GetHalfHeight() const													{ return 0.5f * (mTopCenter - mBottomCenter); }

	// See Shape::GetCenterOfMass
	virtual Vec3			GetCenterOfMass() const override										{ return mCenterOfMass; }

	// See Shape::GetLocalBounds
	virtual AABox			GetLocalBounds() const override;

	// See Shape::GetWorldSpaceBounds
	virtual AABox			GetWorldSpaceBounds(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale) const override;
	using Shape::GetWorldSpaceBounds;

	// See Shape::GetInnerRadius
	virtual float			GetInnerRadius() const override											{ return min(mTopRadius, mBottomRadius); }

	// See Shape::GetMassProperties
	virtual MassProperties	GetMassProperties() const override;

	// See Shape::GetSurfaceNormal
	virtual Vec3			GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const override;

	// See Shape::GetSupportingFace
	virtual void			GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const override;

	// See ConvexShape::GetSupportFunction
	virtual const Support *	GetSupportFunction(ESupportMode inMode, SupportBuffer &inBuffer, Vec3Arg inScale) const override;

	// See: Shape::CollideSoftBodyVertices
	virtual void			CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const override;

#ifndef MOSS_DEBUG_RENDERER
	// See Shape::Draw
	virtual void			Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const override;
#endif // MOSS_DEBUG_RENDERER

	// See Shape
	virtual void			SaveBinaryState(StreamOut &inStream) const override;

	// See Shape::GetStats
	virtual Stats			GetStats() const override												{ return Stats(sizeof(*this), 0); }

	// See Shape::GetVolume
	virtual float			GetVolume() const override												{ return GetLocalBounds().GetVolume(); } // Volume is approximate!

	// See Shape::IsValidScale
	virtual bool			IsValidScale(Vec3Arg inScale) const override;

	// See Shape::MakeScaleValid
	virtual Vec3			MakeScaleValid(Vec3Arg inScale) const override;

	// Register shape functions with the registry
	static void				sRegister();

protected:
	// See: Shape::RestoreBinaryState
	virtual void			RestoreBinaryState(StreamIn &inStream) override;

private:
	// Class for GetSupportFunction
	class					TaperedCapsule;

	/// Returns box that approximates the inertia
	AABox					GetInertiaApproximation() const;

	Vec3					mCenterOfMass = Vec3::sZero();
	float					mTopRadius = 0.0f;
	float					mBottomRadius = 0.0f;
	float					mTopCenter = 0.0f;
	float					mBottomCenter = 0.0f;
	float					mConvexRadius = 0.0f;
	float					mSinAlpha = 0.0f;
	float					mTanAlpha = 0.0f;

#ifndef MOSS_DEBUG_RENDERER
	mutable DebugRenderer::GeometryRef mGeometry;
#endif // MOSS_DEBUG_RENDERER
};




/// Class that constructs a TaperedCylinderShape
class MOSS_EXPORT TaperedCylinderShapeSettings final : public ConvexShapeSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, TaperedCylinderShapeSettings)

public:
	/// Default constructor for deserialization
							TaperedCylinderShapeSettings() = default;

	/// Create a tapered cylinder centered around the origin with bottom at (0, -inHalfHeightOfTaperedCylinder, 0) with radius inBottomRadius and top at (0, inHalfHeightOfTaperedCylinder, 0) with radius inTopRadius
							TaperedCylinderShapeSettings(float inHalfHeightOfTaperedCylinder, float inTopRadius, float inBottomRadius, float inConvexRadius = cDefaultConvexRadius, const PhysicsMaterial *inMaterial = nullptr);

	// See: ShapeSettings
	virtual ShapeResult		Create() const override;

	float					mHalfHeight = 0.0f;
	float					mTopRadius = 0.0f;
	float					mBottomRadius = 0.0f;
	float					mConvexRadius = 0.0f;
};

/// A cylinder with different top and bottom radii
class MOSS_EXPORT TaperedCylinderShape final : public ConvexShape
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
							TaperedCylinderShape() : ConvexShape(EShapeSubType::TaperedCylinder) { }
							TaperedCylinderShape(const TaperedCylinderShapeSettings &inSettings, ShapeResult &outResult);

	/// Get top radius of the tapered cylinder
	inline float			GetTopRadius() const													{ return mTopRadius; }

	/// Get bottom radius of the tapered cylinder
	inline float			GetBottomRadius() const													{ return mBottomRadius; }

	/// Get convex radius of the tapered cylinder
	inline float			GetConvexRadius() const													{ return mConvexRadius; }

	/// Get half height of the tapered cylinder
	inline float			GetHalfHeight() const													{ return 0.5f * (mTop - mBottom); }

	// See Shape::GetCenterOfMass
	virtual Vec3			GetCenterOfMass() const override										{ return Vec3(0, -0.5f * (mTop + mBottom), 0); }

	// See Shape::GetLocalBounds
	virtual AABox			GetLocalBounds() const override;

	// See Shape::GetInnerRadius
	virtual float			GetInnerRadius() const override											{ return min(mTopRadius, mBottomRadius); }

	// See Shape::GetMassProperties
	virtual MassProperties	GetMassProperties() const override;

	// See Shape::GetSurfaceNormal
	virtual Vec3			GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const override;

	// See Shape::GetSupportingFace
	virtual void			GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const override;

	// See ConvexShape::GetSupportFunction
	virtual const Support *	GetSupportFunction(ESupportMode inMode, SupportBuffer &inBuffer, Vec3Arg inScale) const override;

	// See: Shape::CollidePoint
	virtual void			CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;

	// See: Shape::CollideSoftBodyVertices
	virtual void			CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const override;

	// See Shape::GetTrianglesStart
	virtual void			GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const override;

	// See Shape::GetTrianglesNext
	virtual int				GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials = nullptr) const override;

#ifndef MOSS_DEBUG_RENDERER
	// See Shape::Draw
	virtual void			Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const override;
#endif // MOSS_DEBUG_RENDERER

	// See Shape
	virtual void			SaveBinaryState(StreamOut &inStream) const override;

	// See Shape::GetStats
	virtual Stats			GetStats() const override												{ return Stats(sizeof(*this), 0); }

	// See Shape::GetVolume
	virtual float			GetVolume() const override;

	// See Shape::IsValidScale
	virtual bool			IsValidScale(Vec3Arg inScale) const override;

	// See Shape::MakeScaleValid
	virtual Vec3			MakeScaleValid(Vec3Arg inScale) const override;

	// Register shape functions with the registry
	static void				sRegister();

protected:
	// See: Shape::RestoreBinaryState
	virtual void			RestoreBinaryState(StreamIn &inStream) override;

private:
	// Class for GetSupportFunction
	class					TaperedCylinder;

	// Class for GetTrianglesTart
	class					TCSGetTrianglesContext;

	// Scale the cylinder
	MOSS_INLINE void			GetScaled(Vec3Arg inScale, float &outTop, float &outBottom, float &outTopRadius, float &outBottomRadius, float &outConvexRadius) const;

	float					mTop = 0.0f;
	float					mBottom = 0.0f;
	float					mTopRadius = 0.0f;
	float					mBottomRadius = 0.0f;
	float					mConvexRadius = 0.0f;
};
MOSS_NAMESPACE_END

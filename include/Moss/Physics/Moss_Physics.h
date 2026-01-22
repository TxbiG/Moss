//                        MIT License
//
//                  Copyright (c) 2026 Toby
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

/*!
 * @file Moss_Physics.h
 * @brief High-performance 2D and 3D physics simulation system powered by a modified version of Jolt Physics 5.3.0.
 * https://github.com/jrouwe/JoltPhysics
 *
 * The Moss Physics module provides a fast, deterministic, and multithreaded physics engine
 * designed for games, XR simulations, and large-scale environments.
 *
 * ---
 *
 * ### Core Features:
 * - **Jolt 5.3.0 Integration** — Lightweight, SIMD-accelerated, and cross-platform physics core.
 * - **2D & 3D Simulation** — Unified system supporting both 2D arcade-style and full 3D rigid-body dynamics.
 * - **Deterministic Simulation** — Ensures reproducible results across different platforms and frame rates.
 * - **Broadphase&  Narrowphase Collision** — Highly optimized multi-threaded broadphase with efficient shape queries.
 * - **Continuous Collision Detection (CCD)** — Prevents tunneling for fast-moving objects.
 * - **Constraint Solver** — Handles joints, springs, and ragdolls with stable stacking and accurate motion.
 * - **Trigger & Query Support** — Overlap queries, raycasts, sweeps, and shape casts.
 * - **Material System** — Custom friction, restitution, and physical material blending.
 *
 * ---
 *
 * ### Advanced Capabilities:
 * - **Multithreading**  
 *   Uses job system integration to distribute collision detection, constraint solving, and island building.
 * 
 * - **Scene Integration**  
 *   Physics components synchronize seamlessly with the entity system and transform hierarchy.
 *
 * - **Physics Queries**  
 *   - `Moss_Raycast(world, origin, direction, distance)`
 *   - `Moss_Sweep(world, shape, transform, direction, distance)`
 *   - `Moss_Overlap(world, shape, transform)`
 *
 * - **Dynamic Materials&  Layers**  
 *   - Define collision groups and layers for efficient filtering.  
 *   - Material blending for sound, haptics, and visual feedback.  
 *
 * ---
 *
 * ### Design Goals:
 * - Scalable from mobile to high-end XR devices.
 * - Deterministic, lock-step safe for multiplayer and replays.
 * - Fully decoupled from rendering and scene systems.
 * - Predictable and extensible API for custom physics behaviors.
 *
 * ---
 *
 * ### Future Extensions:
 * - Soft-body and cloth simulation.
 * - Fluid simulation with particle-based solvers.
 * - Async baking for collision meshes and static environments.
 * - Seperate 2D and 3D physics.
 */

#ifndef MOSS_PHYSICS_H
#define MOSS_PHYSICS_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include <Moss/Moss_stdinc.h>
#include <Moss/Moss_Renderer.h>

#include <Moss/Core/Variants/AABB2.h>
#include <Moss/Core/Variants/AABB3.h>
#include <Moss/Core/Variants/Color.h>
#include <Moss/Core/Variants/Quat.h>
#include <Moss/Core/Variants/OBB2.h>
#include <Moss/Core/Variants/OBB3.h>
#include <Moss/Core/Variants/TArray.h>

#include <Moss/Core/Variants/Vector/Vec2.h>
#include <Moss/Core/Variants/Vector/Vec3.h>
#include <Moss/Core/Variants/Vector/Vec4.h>
#include <Moss/Core/Variants/Matrix/Mat.h>


#define PHSICS_DEFAULT_COLLISION_TOLERANCE (1.0e-4f) // float cDefaultCollisionTolerance = 1.0e-4f
#define PHSICS_DEFAULT_PENETRATION_TOLERANCE (1.0e-4f) // float cDefaultPenetrationTolerance = 1.0e-4f
#define PHSICS_DEFAULT_CONVEX_RADIUS (0.05f) // float cDefaultConvexRadius = 0.05f
#define PHSICS_CAPSULE_PROJECTION_SLOP (0.02f) // float cCapsuleProjectionSlop = 0.02f
#define PHSICS_MAX_PHYSICS_JOBS (2048) // int cMaxPhysicsJobs = 2048
#define PHSICS_MAX_PHYSICS_BARRIERS (8) // int cMaxPhysicsBarriers = 8
#define PHSICS_INVALID_COLLISION_GROUP_ID (~0U)
#define PHSICS_INVALID_COLLISION_SUBGROUP_ID (~0U)

MOSS_SUPRESS_WARNINGS_BEGIN

static constexpr uint8_t cBodyTypeCount = 2;

class BodyID;
class CharacterID;
using uint32_t = SubShapeID;
using uint32_t = ObjectLayer;
using uint8_t = BroadPhaseLayer;
using uint32_t = CollisionGroupID;
using uint32_t = CollisionSubGroupID;

/* Forward declarations*/
typedef struct BroadPhaseLayerInterface				BroadPhaseLayerInterface;
typedef struct ObjectVsBroadPhaseLayerFilter		ObjectVsBroadPhaseLayerFilter;
typedef struct ObjectLayerPairFilter				ObjectLayerPairFilter;
typedef struct BroadPhaseLayerFilter				BroadPhaseLayerFilter;
typedef struct ObjectLayerFilter					ObjectLayerFilter;
class BodyFilter;
class ShapeFilter;
class SimShapeFilter;
class PhysicsSystem;

/* ShapeSettings*/
struct ShapeSettings;
struct ConvexShapeSettings;
struct SphereShapeSettings;
struct BoxShapeSettings;
struct PlaneShapeSettings;
struct TriangleShapeSettings;
struct CapsuleShapeSettings;
struct TaperedCapsuleShapeSettings;
struct CylinderShapeSettings;
struct TaperedCylinderShapeSettings;
struct ConvexHullShapeSettings;
struct CompoundShapeSettings;
struct StaticCompoundShapeSettings;
struct MutableCompoundShapeSettings;
struct MeshShapeSettings;
struct HeightFieldShapeSettings;
struct RotatedTranslatedShapeSettings;
struct ScaledShapeSettings;
struct OffsetCenterOfMassShapeSettings;
struct EmptyShapeSettings;

/* Shape*/
class Shape;
class ConvexShape;
class SphereShape;
class BoxShape;
class PlaneShape;
class CapsuleShape;
class CylinderShape;
class TaperedCylinderShape;
class TriangleShape;
class TaperedCapsuleShape;
class ConvexHullShape;
class CompoundShape;
class StaticCompoundShape;
class MutableCompoundShape;
class MeshShape;
class HeightFieldShape;
class DecoratedShape;
class RotatedTranslatedShape;
class ScaledShape;
class OffsetCenterOfMassShape;
class EmptyShape;

class BodyCreationSettings;
class SoftBodyCreationSettings;
class BodyInterface;
class BodyLockInterface;
class BroadPhaseQuery;
class NarrowPhaseQuery;
class MotionProperties;
class MassProperties;
class Body;

class CollideShapeResult;
class ContactListener;
class ContactManifold;

class GroupFilter;
class GroupFilterTable;  /* Inherits GroupFilter*/

class BodyActivationListener;
class BodyDrawFilter;

typedef struct SharedMutex                      SharedMutex;

typedef struct DebugRenderer                    DebugRenderer;

/* Constraint*/
typedef struct Constraint                       Constraint;
class TwoBodyConstraint;
typedef struct FixedConstraint                  FixedConstraint;
typedef struct DistanceConstraint               DistanceConstraint;
typedef struct PointConstraint                  PointConstraint;
typedef struct HingeConstraint                  HingeConstraint;
typedef struct SliderConstraint                 SliderConstraint;
typedef struct ConeConstraint                   ConeConstraint;
typedef struct SwingTwistConstraint             SwingTwistConstraint;
typedef struct SixDOFConstraint				    SixDOFConstraint;

/* Character, CharacterVirtual*/
typedef struct CharacterBase					CharacterBase;
typedef struct Character						Character;  /* Inherits CharacterBase*/
typedef struct CharacterVirtual                 CharacterVirtual;  /* Inherits CharacterBase*/
typedef struct CharacterContactListener			CharacterContactListener;
typedef struct CharacterVsCharacterCollision	CharacterVsCharacterCollision;

typedef struct JobSystem JobSystem;

class Skeleton;
class Ragdoll;

/* Enums*/
enum class EPhysicsUpdateError : uint32_t {
	None					= 0,			// No errors
	ManifoldCacheFull		= 1 << 0,		// The manifold cache is full, this means that the total number of contacts between bodies is too high. Some contacts were ignored. Increase inMaxContactConstraints in PhysicsSystem::Init.
	BodyPairCacheFull		= 1 << 1,		// The body pair cache is full, this means that too many bodies contacted. Some contacts were ignored. Increase inMaxBodyPairs in PhysicsSystem::Init.
	ContactConstraintsFull	= 1 << 2,		// The contact constraints buffer is full. Some contacts were ignored. Increase inMaxContactConstraints in PhysicsSystem::Init.
};

enum class EValidateResult {
	AcceptAllContactsForThisBodyPair = 0,
	AcceptContact = 1,
	RejectContact = 2,
	RejectAllContactsForThisBodyPair = 3
};

enum class EShapeType {
	Convex = 0,
	Compound = 1,
	Decorated = 2,
	Mesh = 3,
	HeightField = 4,
	SoftBody = 5,

	User1 = 6,
	User2 = 7,
	User3 = 8,
	User4 = 9
};

enum class EShapeSubType {
	Sphere = 0,
	Box = 1,
	Triangle = 2,
	Capsule = 3,
	TaperedCapsule = 4,
	Cylinder = 5,
	ConvexHull = 6,
	StaticCompound = 7,
	MutableCompound = 8,
	RotatedTranslated = 9,
	Scaled = 10,
	OffsetCenterOfMass = 11,
	Mesh = 12,
	HeightField = 13,
	SoftBody = 14
};

enum class EConstraintSubType
{
	Fixed,
	Point,
	Hinge,
	Slider,
	Distance,
	Cone,
	SwingTwist,
	SixDOF,
	Path,
	Vehicle,
	RackAndPinion,
	Gear,
	Pulley,

	// User defined constraint types start here
	User1,
	User2,
	User3,
	User4
};

enum class EAllowedDOFs {
	None				= 0b000000,									// No degrees of freedom are allowed. Note that this is not valid and will crash. Use a static body instead.
	All					= 0b111111,									// All degrees of freedom are allowed
	TranslationX		= 0b000001,									// Body can move in world space X axis
	TranslationY		= 0b000010,									// Body can move in world space Y axis
	TranslationZ		= 0b000100,									// Body can move in world space Z axis
	RotationX			= 0b001000,									// Body can rotate around world space X axis
	RotationY			= 0b010000,									// Body can rotate around world space Y axis
	RotationZ			= 0b100000,									// Body can rotate around world space Z axis
	Plane2D				= TranslationX | TranslationY | RotationZ,	// Body can only move in X and Y axis and rotate around Z axis
};

enum class EGroundState 				{ OnGround = 0, OnSteepGround = 1, NotSupported = 2, InAir = 3, };
enum class ETransmissionMode : uint8 	{ Auto, Manual };
enum class EBodyType : uint8 			{ Rigid, Soft };
enum class EMotionType : uint8 			{ Static, Kinematic, Dynamic };
enum class EActivation : uint8 			{ Activate, DontActivate };
enum class EConstraintType : uint8		{ Constraint, TwoBodyConstraint };
enum class EConstraintSpace : uint8 	{ LocalToBodyCOM, WorldSpace };
enum class EMotionQuality : uint8 		{ Discrete, LinearCast };
enum class EOverrideMassProperties 		{ CalculateMassAndInertia, CalculateInertia, MassAndInertiaProvided };
enum class EBackFaceMode : uint8 		{ IgnoreBackFaces, CollideWithBackFaces };
enum class EActiveEdgeMode : uint8 		{ CollideOnlyWithActive, CollideWithAll };
enum class ECollectFacesMode : uint8 	{ CollectFaces, NoFaces };
enum class EMotorState : uint8 			{ Off, Velocity, Position };
enum class ESwingType : uint8 			{ Cone, Pyramid };

enum class SoftBodyValidateResult
{
	AcceptContact,														// Accept this contact
	RejectContact,														// Reject this contact
};

enum class ECollisionCollectorType {
	AllHit = 0,
	AllHitSorted = 1,
	ClosestHit = 2,
	AnyHit = 3
};

enum class ETrackSide : uint8 { Left, Right, Num };

enum class ESixDOFConstraintAxis {
	TranslationX,
	TranslationY,
	TranslationZ,

	RotationX,
	RotationY,
	RotationZ,

	Num,
	NumTranslation = TranslationZ + 1,
	Force32 = 0x7FFFFFFF
};

enum class ESpringMode : uint8 { FrequencyAndDamping, StiffnessAndDamping };

// Defines how to color soft body constraints
enum class ESoftBodyConstraintColor {
	ConstraintType,				// Draw different types of constraints in different colors
	ConstraintGroup,			// Draw constraints in the same group in the same color, non-parallel group will be red
	ConstraintOrder,			// Draw constraints in the same group in the same color, non-parallel group will be red, and order within each group will be indicated with gradient
};

enum class EBodyManager_ShapeColor {
	InstanceColor,				// Random color per instance
	ShapeTypeColor,				// Convex = green, scaled = yellow, compound = orange, mesh = red
	MotionTypeColor,			// Static = grey, keyframed = green, dynamic = random color per instance
	SleepColor,					// Static = grey, keyframed = green, dynamic = yellow, sleeping = red
	IslandColor,				// Static = grey, active = random color per island, sleeping = light grey
	MaterialColor,				// Color as defined by the PhysicsMaterial of the shape
};

enum class CastShadow {
	CastShadow_On = 0,    // This shape should cast a shadow
	CastShadow_Off = 1,   // This shape should not cast a shadow
};

enum class DrawMode { Solid, Wireframe };

enum class Mesh_Shape_BuildQuality { FavorRuntimePerformance = 0, FavorBuildSpeed = 1 };

enum class ECanSleep { CannotSleep = 0, CanSleep = 1 };

struct AABoxCast {
	MOSS_OVERRIDE_NEW_DELETE

	AABox						mBox;						// Axis aligned box at starting location
	Vec3						mDirection;					// Direction and length of the cast (anything beyond this length will not be reported as a hit)
};

struct PhysicsMaterial {
	float staticFriction;
	float dynamicFriction;
	float restitution;
	float density;
};

class SoftBodyContactSettings
{
public:
	float							mInvMassScale1 = 1.0f;				// Scale factor for the inverse mass of the soft body (0 = infinite mass, 1 = use original mass, 2 = body has half the mass). For the same contact pair, you should strive to keep the value the same over time.
	float							mInvMassScale2 = 1.0f;				// Scale factor for the inverse mass of the other body (0 = infinite mass, 1 = use original mass, 2 = body has half the mass). For the same contact pair, you should strive to keep the value the same over time.
	float							mInvInertiaScale2 = 1.0f;			// Scale factor for the inverse inertia of the other body (usually same as mInvMassScale2)
	bool							mIsSensor;							// If the contact should be treated as a sensor vs body contact (no collision response)
};

class [[nodiscard]] Plane {
public:
	MOSS_OVERRIDE_NEW_DELETE

	// Constructor
	Plane() = default;
	explicit Plane(Vec4Arg inNormalAndConstant)										: mNormalAndConstant(inNormalAndConstant) { }
	Plane(Vec3Arg inNormal, float inConstant)								: mNormalAndConstant(inNormal, inConstant) { }

	// Create from point and normal
	static Plane	sFromPointAndNormal(Vec3Arg inPoint, Vec3Arg inNormal)					{ return Plane(Vec4(inNormal, -inNormal.Dot(inPoint))); }

	// Create from point and normal, double precision version that more accurately calculates the plane constant
	static Plane	sFromPointAndNormal(DVec3Arg inPoint, Vec3Arg inNormal)					{ return Plane(Vec4(inNormal, -float(DVec3(inNormal).Dot(inPoint)))); }

	// Create from 3 counter clockwise points
	static Plane	sFromPointsCCW(Vec3Arg inV1, Vec3Arg inV2, Vec3Arg inV3)				{ return sFromPointAndNormal(inV1, (inV2 - inV1).Cross(inV3 - inV1).Normalized()); }

	// Properties
	Vec3			GetNormal() const														{ return Vec3(mNormalAndConstant); }
	void			SetNormal(Vec3Arg inNormal)												{ mNormalAndConstant = Vec4(inNormal, mNormalAndConstant.GetW()); }
	float			GetConstant() const														{ return mNormalAndConstant.GetW(); }
	void			SetConstant(float inConstant)											{ mNormalAndConstant.SetW(inConstant); }

	// Offset the plane (positive value means move it in the direction of the plane normal)
	Plane			Offset(float inDistance) const											{ return Plane(mNormalAndConstant - Vec4(Vec3::sZero(), inDistance)); }

	// Transform the plane by a matrix
	inline Plane	GetTransformed(Mat44Arg inTransform) const
	{
		Vec3 transformed_normal = inTransform.Multiply3x3(GetNormal());
		return Plane(transformed_normal, GetConstant() - inTransform.GetTranslation().Dot(transformed_normal));
	}

	// Scale the plane, can handle non-uniform and negative scaling
	inline Plane	Scaled(Vec3Arg inScale) const
	{
		Vec3 scaled_normal = GetNormal() / inScale;
		float scaled_normal_length = scaled_normal.Length();
		return Plane(scaled_normal / scaled_normal_length, GetConstant() / scaled_normal_length);
	}

	// Distance point to plane
	float			SignedDistance(Vec3Arg inPoint) const									{ return inPoint.Dot(GetNormal()) + GetConstant(); }

	// Project inPoint onto the plane
	Vec3			ProjectPointOnPlane(Vec3Arg inPoint) const								{ return inPoint - GetNormal()* SignedDistance(inPoint); }

	// Returns intersection point between 3 planes
	static bool		sIntersectPlanes(const Plane& inP1, const Plane& inP2, const Plane& inP3, Vec3& outPoint)
	{
		// We solve the equation:
		// |ax, ay, az, aw|   | x |   | 0 |
		// |bx, by, bz, bw|* | y | = | 0 |
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
		Vec4 numerator =
			a.SplatW()* (b.Swizzle<SWIZZLE_Z, SWIZZLE_X, SWIZZLE_Y, SWIZZLE_UNUSED>()* c.Swizzle<SWIZZLE_Y, SWIZZLE_Z, SWIZZLE_X, SWIZZLE_UNUSED>() - b.Swizzle<SWIZZLE_Y, SWIZZLE_Z, SWIZZLE_X, SWIZZLE_UNUSED>()* c.Swizzle<SWIZZLE_Z, SWIZZLE_X, SWIZZLE_Y, SWIZZLE_UNUSED>())
			+ a.Swizzle<SWIZZLE_Y, SWIZZLE_X, SWIZZLE_X, SWIZZLE_UNUSED>()* (b.Swizzle<SWIZZLE_W, SWIZZLE_Z, SWIZZLE_W, SWIZZLE_UNUSED>()* c.Swizzle<SWIZZLE_Z, SWIZZLE_W, SWIZZLE_Y, SWIZZLE_UNUSED>() - b.Swizzle<SWIZZLE_Z, SWIZZLE_W, SWIZZLE_Y, SWIZZLE_UNUSED>()* c.Swizzle<SWIZZLE_W, SWIZZLE_Z, SWIZZLE_W, SWIZZLE_UNUSED>())
			+ a.Swizzle<SWIZZLE_Z, SWIZZLE_Z, SWIZZLE_Y, SWIZZLE_UNUSED>()* (b.Swizzle<SWIZZLE_Y, SWIZZLE_W, SWIZZLE_X, SWIZZLE_UNUSED>()* c.Swizzle<SWIZZLE_W, SWIZZLE_X, SWIZZLE_W, SWIZZLE_UNUSED>() - b.Swizzle<SWIZZLE_W, SWIZZLE_X, SWIZZLE_W, SWIZZLE_UNUSED>()* c.Swizzle<SWIZZLE_Y, SWIZZLE_W, SWIZZLE_X, SWIZZLE_UNUSED>());

		outPoint = Vec3(numerator) / denominator;
		return true;
	}

private:
#ifdef MOSS_OBJECT_STREAM
	friend void		CreateRTTIPlane(class RTTI& );										// For MOSS_IMPLEMENT_SERIALIZABLE_OUTSIDE_CLASS
#endif

	Vec4			mNormalAndConstant;													// XYZ = normal, W = constant, plane: x . normal + constant = 0
};

class Triangle
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	// Constructor
	Triangle() = default;
	Triangle(const Float3& inV1, const Float3& inV2, const Float3& inV3, uint32 inMaterialIndex = 0, uint32 inUserData = 0) : mV { inV1, inV2, inV3 }, mMaterialIndex(inMaterialIndex), mUserData(inUserData) { }
	Triangle(Vec3Arg inV1, Vec3Arg inV2, Vec3Arg inV3, uint32 inMaterialIndex = 0, uint32 inUserData = 0) : mMaterialIndex(inMaterialIndex), mUserData(inUserData) { inV1.StoreFloat3(&mV[0]); inV2.StoreFloat3(&mV[1]); inV3.StoreFloat3(&mV[2]); }

	// Get center of triangle
	Vec3 GetCentroid() const {
		return (Vec3::sLoadFloat3Unsafe(mV[0]) + Vec3::sLoadFloat3Unsafe(mV[1]) + Vec3::sLoadFloat3Unsafe(mV[2]))* (1.0f / 3.0f);
	}

	// Vertices
	Float3			mV[3];
	uint32			mMaterialIndex = 0;			// Follows mV[3] so that we can read mV as 4 vectors
	uint32			mUserData = 0;				// User data that can be used for anything by the application, e.g. for tracking the original index of the triangle
};

class IndexedTriangleNoMaterial
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	// Constructor
					IndexedTriangleNoMaterial() = default;
	constexpr		IndexedTriangleNoMaterial(uint32 inI1, uint32 inI2, uint32 inI3) : mIdx { inI1, inI2, inI3 } { }

	// Check if two triangles are identical
	bool			operator == (const IndexedTriangleNoMaterial& inRHS) const
	{
		return mIdx[0] == inRHS.mIdx[0]& & mIdx[1] == inRHS.mIdx[1]& & mIdx[2] == inRHS.mIdx[2];
	}

	// Check if two triangles are equivalent (using the same vertices)
	bool			IsEquivalent(const IndexedTriangleNoMaterial& inRHS) const
	{
		return (mIdx[0] == inRHS.mIdx[0]& & mIdx[1] == inRHS.mIdx[1]& & mIdx[2] == inRHS.mIdx[2])
			|| (mIdx[0] == inRHS.mIdx[1]& & mIdx[1] == inRHS.mIdx[2]& & mIdx[2] == inRHS.mIdx[0])
			|| (mIdx[0] == inRHS.mIdx[2]& & mIdx[1] == inRHS.mIdx[0]& & mIdx[2] == inRHS.mIdx[1]);
	}

	// Check if two triangles are opposite (using the same vertices but in opposing order)
	bool			IsOpposite(const IndexedTriangleNoMaterial& inRHS) const
	{
		return (mIdx[0] == inRHS.mIdx[0]& & mIdx[1] == inRHS.mIdx[2]& & mIdx[2] == inRHS.mIdx[1])
			|| (mIdx[0] == inRHS.mIdx[1]& & mIdx[1] == inRHS.mIdx[0]& & mIdx[2] == inRHS.mIdx[2])
			|| (mIdx[0] == inRHS.mIdx[2]& & mIdx[1] == inRHS.mIdx[1]& & mIdx[2] == inRHS.mIdx[0]);
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
		uint32 tmp = mIdx[0];
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
	uint64			GetHash() const
	{
		static_assert(sizeof(IndexedTriangleNoMaterial) == 3* sizeof(uint32), "Class should have no padding");
		return HashBytes(this, sizeof(IndexedTriangleNoMaterial));
	}

	uint32			mIdx[3];
};

// Triangle with 32-bit indices and material index
class IndexedTriangle : public IndexedTriangleNoMaterial
{
public:
	using IndexedTriangleNoMaterial::IndexedTriangleNoMaterial;

	// Constructor
	constexpr		IndexedTriangle(uint32 inI1, uint32 inI2, uint32 inI3, uint32 inMaterialIndex, uint inUserData = 0) : IndexedTriangleNoMaterial(inI1, inI2, inI3), mMaterialIndex(inMaterialIndex), mUserData(inUserData) { }

	// Check if two triangles are identical
	bool			operator == (const IndexedTriangle& inRHS) const
	{
		return mMaterialIndex == inRHS.mMaterialIndex& & mUserData == inRHS.mUserData& & IndexedTriangleNoMaterial::operator==(inRHS);
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
	uint64			GetHash() const
	{
		static_assert(sizeof(IndexedTriangle) == 5* sizeof(uint32), "Class should have no padding");
		return HashBytes(this, sizeof(IndexedTriangle));
	}

	uint32			mMaterialIndex = 0;
	uint32			mUserData = 0;				// User data that can be used for anything by the application, e.g. for tracking the original index of the triangle
};

typedef struct MassProperties {
	float mass;
	Mat44 inertia;
} MassProperties;

typedef struct ContactSettings {
	float				combinedFriction;
	float				combinedRestitution;
	float				invMassScale1;
	float				invInertiaScale1;
	float				invMassScale2;
	float				invInertiaScale2;
	bool				isSensor;
	Vec3				relativeLinearSurfaceVelocity;
	Vec3				relativeAngularSurfaceVelocity;
} ContactSettings;

typedef struct CollideSettingsBase {
	// How active edges (edges that a moving object should bump into) are handled
	EActiveEdgeMode			activeEdgeMode/* = ActiveEdgeMode_CollideOnlyWithActive*/;

	// If colliding faces should be collected or only the collision point
	CollectFacesMode		collectFacesMode/* = NoFaces*/;

	// If objects are closer than this distance, they are considered to be colliding (used for GJK) (unit: meter)
	float						collisionTolerance/* = DEFAULT_COLLISION_TOLERANCE*/;

	// A factor that determines the accuracy of the penetration depth calculation. If the change of the squared distance is less than tolerance* current_penetration_depth^2 the algorithm will terminate. (unit: dimensionless)
	float						penetrationTolerance/* = DEFAULT_PENETRATION_TOLERANCE*/;

	// When mActiveEdgeMode is CollideOnlyWithActive a movement direction can be provided. When hitting an inactive edge, the system will select the triangle normal as penetration depth only if it impedes the movement less than with the calculated penetration depth.
	Vec3					activeEdgeMovementDirection/* = Vec3::sZero()*/;
} CollideSettingsBase;

/* CollideShapeSettings*/
typedef struct CollideShapeSettings {
	CollideSettingsBase     base;    /* Inherits CollideSettingsBase*/
	// When > 0 contacts in the vicinity of the query shape can be found. All nearest contacts that are not further away than this distance will be found (unit: meter)
	float						maxSeparationDistance/* = 0.0f*/;

	// How backfacing triangles should be treated
	EBackFaceMode			backFaceMode/* = BackFaceMode_IgnoreBackFaces*/;
} CollideShapeSettings;

/* ShapeCastSettings*/
typedef struct ShapeCastSettings {
	CollideSettingsBase     base;    /* Inherits CollideSettingsBase*/

	// How backfacing triangles should be treated (should we report moving from back to front for triangle based shapes, e.g. for MeshShape/HeightFieldShape?)
	EBackFaceMode			backFaceModeTriangles/* = BackFaceMode_IgnoreBackFaces*/;

	// How backfacing convex objects should be treated (should we report starting inside an object and moving out?)
	EBackFaceMode			backFaceModeConvex/* = BackFaceMode_IgnoreBackFaces*/;

	// Indicates if we want to shrink the shape by the convex radius and then expand it again. This speeds up collision detection and gives a more accurate normal at the cost of a more 'rounded' shape.
	bool						useShrunkenShapeAndConvexRadius/* = false*/;

	// When true, and the shape is intersecting at the beginning of the cast (fraction = 0) then this will calculate the deepest penetration point (costing additional CPU time)
	bool						returnDeepestPoint/* = false*/;
} ShapeCastSettings;

typedef struct RayCastSettings {
	// How backfacing triangles should be treated (should we report back facing hits for triangle based shapes, e.g. MeshShape/HeightFieldShape?)
	EBackFaceMode backFaceModeTriangles/* = BackFaceMode_IgnoreBackFaces*/;

	// How backfacing convex objects should be treated (should we report back facing hits for convex shapes?)
	EBackFaceMode backFaceModeConvex/* = BackFaceMode_IgnoreBackFaces*/;

	// If convex shapes should be treated as solid. When true, a ray starting inside a convex shape will generate a hit at fraction 0.
	bool treatConvexAsSolid/* = true*/;
} RayCastSettings;

class MOSS_API SpringSettings {
public:
	// Constructor
	SpringSettings() = default;
	SpringSettings(const SpringSettings& ) = default;
	SpringSettings& operator = (const SpringSettings& ) = default;
	SpringSettings(ESpringMode inMode, float inFrequencyOrStiffness, float inDamping) : mMode(inMode), mFrequency(inFrequencyOrStiffness), mDamping(inDamping) { }

	// Saves the contents of the spring settings in binary form to inStream.
	void SaveBinaryState(StreamOut& inStream) const;

	// Restores contents from the binary stream inStream.
	void RestoreBinaryState(StreamIn& inStream);

	// Check if the spring has a valid frequency / stiffness, if not the spring will be hard
	inline bool	HasStiffness() const { return frequency > 0.0f; }

	// Selects the way in which the spring is defined
	// If the mode is StiffnessAndDamping then mFrequency becomes the stiffness (k) and mDamping becomes the damping ratio (c) in the spring equation F = -k* x - c* v. Otherwise the properties are as documented.
	ESpringMode mode = ESpringMode::FrequencyAndDamping;

	union {
		// Valid when mSpringMode = ESpringMode::FrequencyAndDamping.
		// If mFrequency > 0 the constraint will be soft and mFrequency specifies the oscillation frequency in Hz.
		// If mFrequency <= 0, mDamping is ignored and the constraint will have hard limits (as hard as the time step / the number of velocity / position solver steps allows).
		float frequency = 0.0f;

		// Valid when mSpringMode = ESpringMode::StiffnessAndDamping.
		// If mStiffness > 0 the constraint will be soft and mStiffness specifies the stiffness (k) in the spring equation F = -k* x - c* v for a linear or T = -k* theta - c* w for an angular spring.
		// If mStiffness <= 0, mDamping is ignored and the constraint will have hard limits (as hard as the time step / the number of velocity / position solver steps allows).
		//
		// Note that stiffness values are large numbers. To calculate a ballpark value for the needed stiffness you can use:
		// force = stiffness* delta_spring_length = mass* gravity <=> stiffness = mass* gravity / delta_spring_length.
		// So if your object weighs 1500 kg and the spring compresses by 2 meters, you need a stiffness in the order of 1500* 9.81 / 2 ~ 7500 N/m.
		float stiffness;
	};

	// When mSpringMode = ESpringMode::FrequencyAndDamping mDamping is the damping ratio (0 = no damping, 1 = critical damping).
	// When mSpringMode = ESpringMode::StiffnessAndDamping mDamping is the damping (c) in the spring equation F = -k* x - c* v for a linear or T = -k* theta - c* w for an angular spring.
	// Note that if you set mDamping = 0, you will not get an infinite oscillation. Because we integrate physics using an explicit Euler scheme, there is always energy loss.
	// This is done to keep the simulation from exploding, because with a damping of 0 and even the slightest rounding error, the oscillation could become bigger and bigger until the simulation explodes.
	float damping = 0.0f;
};

class MOSS_API MotorSettings {
public:
	// Constructor
	MotorSettings() = default;
	MotorSettings(const MotorSettings& ) = default;
	MotorSettings&	operator = (const MotorSettings& ) = default;
	MotorSettings(float inFrequency, float inDamping) : mSpringSettings(ESpringMode::FrequencyAndDamping, inFrequency, inDamping) { MOSS_ASSERT(IsValid()); }
	MotorSettings(float inFrequency, float inDamping, float inForceLimit, float inTorqueLimit) : mSpringSettings(ESpringMode::FrequencyAndDamping, inFrequency, inDamping), mMinForceLimit(-inForceLimit), mMaxForceLimit(inForceLimit), mMinTorqueLimit(-inTorqueLimit), mMaxTorqueLimit(inTorqueLimit) { MOSS_ASSERT(IsValid()); }

	// Set asymmetric force limits
	void SetForceLimits(float inMin, float inMax)	{ MOSS_ASSERT(inMin <= inMax); mMinForceLimit = inMin; mMaxForceLimit = inMax; }

	// Set asymmetric torque limits
	void SetTorqueLimits(float inMin, float inMax)	{ MOSS_ASSERT(inMin <= inMax); mMinTorqueLimit = inMin; mMaxTorqueLimit = inMax; }

	// Set symmetric force limits
	void SetForceLimit(float inLimit) { mMinForceLimit = -inLimit; mMaxForceLimit = inLimit; }

	// Set symmetric torque limits
	void SetTorqueLimit(float inLimit) { mMinTorqueLimit = -inLimit; mMaxTorqueLimit = inLimit; }

	// Check if settings are valid
	bool IsValid() const { return mSpringSettings.mFrequency >= 0.0f& & mSpringSettings.mDamping >= 0.0f& & mMinForceLimit <= mMaxForceLimit& & mMinTorqueLimit <= mMaxTorqueLimit; }

	// Saves the contents of the motor settings in binary form to inStream.
	void SaveBinaryState(StreamOut& inStream) const;

	// Restores contents from the binary stream inStream.
	void RestoreBinaryState(StreamIn& inStream);

	// Settings
	SpringSettings			mSpringSettings { ESpringMode::FrequencyAndDamping, 2.0f, 1.0f }; // Settings for the spring that is used to drive to the position target (not used when motor is a velocity motor).
	float					mMinForceLimit = -FLT_MAX;					// Minimum force to apply in case of a linear constraint (N). Usually this is -mMaxForceLimit unless you want a motor that can e.g. push but not pull. Not used when motor is an angular motor.
	float					mMaxForceLimit = FLT_MAX;					// Maximum force to apply in case of a linear constraint (N). Not used when motor is an angular motor.
	float					mMinTorqueLimit = -FLT_MAX;					// Minimum torque to apply in case of a angular constraint (N m). Usually this is -mMaxTorqueLimit unless you want a motor that can e.g. push but not pull. Not used when motor is a position motor.
	float					mMaxTorqueLimit = FLT_MAX;					// Maximum torque to apply in case of a angular constraint (N m). Not used when motor is a position motor.
};

typedef struct SubShapeIDPair {
	BodyID     Body1ID;
	SubShapeID subShapeID1;
	BodyID     Body2ID;
	SubShapeID subShapeID2;
} SubShapeIDPair;

typedef struct BroadPhaseCastResult {
	BodyID     bodyID;
	float      fraction;
};

typedef struct RayCastResult {
	BodyID     bodyID;
	float      fraction;
	SubShapeID subShapeID2;
} RayCastResult;

typedef struct CollidePointResult {
	BodyID bodyID;
	SubShapeID subShapeID2;
} CollidePointResult;

typedef struct CollideShapeResult {
	Vec3		contactPointOn1;
	Vec3		contactPointOn2;
	Vec3		penetrationAxis;
	float		penetrationDepth;
	SubShapeID	subShapeID1;
	SubShapeID	subShapeID2;
	BodyID		bodyID2;
	uint32_t	shape1FaceCount;
	Vec3*		shape1Faces;
	uint32_t	shape2FaceCount;
	Vec3*		shape2Faces;
} CollideShapeResult;

typedef struct ShapeCastResult {
	Vec3           contactPointOn1;
	Vec3           contactPointOn2;
	Vec3           penetrationAxis;
	float          penetrationDepth;
	SubShapeID     subShapeID1;
	SubShapeID     subShapeID2;
	BodyID         bodyID2;
	float          fraction;
	bool		   isBackFaceHit;
} ShapeCastResult;

typedef struct DrawSettings {
	bool						drawGetSupportFunction;				// Draw the GetSupport() function, used for convex collision detection
	bool						drawSupportDirection;				// When drawing the support function, also draw which direction mapped to a specific support point
	bool						drawGetSupportingFace;				// Draw the faces that were found colliding during collision detection
	bool						drawShape;							// Draw the shapes of all bodies
	bool						drawShapeWireframe;					// When mDrawShape is true and this is true, the shapes will be drawn in wireframe instead of solid.
	BodyManager_ShapeColor	drawShapeColor;                     // Coloring scheme to use for shapes
	bool						drawBoundingBox;					// Draw a bounding box per body
	bool						drawCenterOfMassTransform;			// Draw the center of mass for each body
	bool						drawWorldTransform;					// Draw the world transform (which may differ from its center of mass) of each body
	bool						drawVelocity;						// Draw the velocity vector for each body
	bool						drawMassAndInertia;					// Draw the mass and inertia (as the box equivalent) for each body
	bool						drawSleepStats;						// Draw stats regarding the sleeping algorithm of each body
	bool						drawSoftBodyVertices;				// Draw the vertices of soft bodies
	bool						drawSoftBodyVertexVelocities;		// Draw the velocities of the vertices of soft bodies
	bool						drawSoftBodyEdgeConstraints;		// Draw the edge constraints of soft bodies
	bool						drawSoftBodyBendConstraints;		// Draw the bend constraints of soft bodies
	bool						drawSoftBodyVolumeConstraints;		// Draw the volume constraints of soft bodies
	bool						drawSoftBodySkinConstraints;		// Draw the skin constraints of soft bodies
	bool						drawSoftBodyLRAConstraints;	        // Draw the LRA constraints of soft bodies
	bool						drawSoftBodyPredictedBounds;		// Draw the predicted bounds of soft bodies
	SoftBodyConstraintColor	drawSoftBodyConstraintColor;        // Coloring scheme to use for soft body constraints
} DrawSettings;

typedef struct SupportingFace {
    uint32_t count;
    Vec3 vertices[32];
} SupportingFace;

typedef struct CollisionGroup {
	const GroupFilter*	groupFilter;
	CollisionGroupID	groupID;
	CollisionSubGroupID	subGroupID;
} CollisionGroup;

typedef void CastRayResultCallback(void* context, const RayCastResult* result);
typedef void RayCastBodyResultCallback(void* context, const BroadPhaseCastResult* result);
typedef void CollideShapeBodyResultCallback(void* context, const BodyID result);
typedef void CollidePointResultCallback(void* context, const CollidePointResult* result);
typedef void CollideShapeResultCallback(void* context, const CollideShapeResult* result);
typedef void CastShapeResultCallback(void* context, const ShapeCastResult* result);

typedef float CastRayCollectorCallback(void* context, const RayCastResult* result);
typedef float RayCastBodyCollectorCallback(void* context, const BroadPhaseCastResult* result);
typedef float CollideShapeBodyCollectorCallback(void* context, const BodyID result);
typedef float CollidePointCollectorCallback(void* context, const CollidePointResult* result);
typedef float CollideShapeCollectorCallback(void* context, const CollideShapeResult* result);
typedef float CastShapeCollectorCallback(void* context, const ShapeCastResult* result);

typedef struct CollisionEstimationResultImpulse {
	float	contactImpulse;
	float	frictionImpulse1;
	float	frictionImpulse2;
} CollisionEstimationResultImpulse;

typedef struct CollisionEstimationResult {
	Vec3								linearVelocity1;
	Vec3								angularVelocity1;
	Vec3								linearVelocity2;
	Vec3								angularVelocity2;

	Vec3								tangent1;
	Vec3								tangent2;

	uint32_t								impulseCount;
	CollisionEstimationResultImpulse*	impulses;
} CollisionEstimationResult;

// Class used to store the configuration of a constraint. Allows run-time creation of constraints.
class MOSS_EXPORT ConstraintSettings : public SerializableObject, public RefTarget<ConstraintSettings>
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, ConstraintSettings)

public:
	using ConstraintResult = Result<Ref<ConstraintSettings>>;

	// Saves the contents of the constraint settings in binary form to inStream.
	virtual void				SaveBinaryState(StreamOut& inStream) const;

	// Creates a constraint of the correct type and restores its contents from the binary stream inStream.
	static ConstraintResult		sRestoreFromBinaryState(StreamIn& inStream);

	// If this constraint is enabled initially. Use Constraint::SetEnabled to toggle after creation.
	bool						mEnabled = true;

	// Priority of the constraint when solving. Higher numbers have are more likely to be solved correctly.
	// Note that if you want a deterministic simulation and you cannot guarantee the order in which constraints are added/removed, you can make the priority for all constraints unique to get a deterministic ordering.
	uint32						mConstraintPriority = 0;

	// Used only when the constraint is active. Override for the number of solver velocity iterations to run, 0 means use the default in PhysicsSettings::mNumVelocitySteps. The number of iterations to use is the max of all contacts and constraints in the island.
	uint						mNumVelocityStepsOverride = 0;

	// Used only when the constraint is active. Override for the number of solver position iterations to run, 0 means use the default in PhysicsSettings::mNumPositionSteps. The number of iterations to use is the max of all contacts and constraints in the island.
	uint						mNumPositionStepsOverride = 0;

	// Size of constraint when drawing it through the debug renderer
	float						mDrawConstraintSize = 1.0f;

	// User data value (can be used by application)
	uint64						mUserData = 0;

protected:
	// This function should not be called directly, it is used by sRestoreFromBinaryState.
	virtual void				RestoreBinaryState(StreamIn& inStream);
};

typedef struct BodyLockRead {
	const BodyLockInterface* lockInterface;
	SharedMutex* mutex;
	const Body* body;
} BodyLockRead;

typedef struct BodyLockWrite {
	const BodyLockInterface* lockInterface;
	SharedMutex* mutex;
	Body* body;
} BodyLockWrite;

typedef struct BodyLockMultiRead BodyLockMultiRead;
typedef struct BodyLockMultiWrite BodyLockMultiWrite;

typedef struct ExtendedUpdateSettings {
	Vec3	stickToFloorStepDown;
	Vec3	walkStairsStepUp;
	float	walkStairsMinStepForward;
	float	walkStairsStepForwardTest;
	float	walkStairsCosAngleForwardContact;
	Vec3	walkStairsStepDownExtra;
} ExtendedUpdateSettings;

// Base class for configuration of a character
class MOSS_API CharacterBaseSettings : public RefTarget<CharacterBaseSettings> {
public:
	MOSS_OVERRIDE_NEW_DELETE

	// Constructor
	CharacterBaseSettings() = default;
	CharacterBaseSettings(const CharacterBaseSettings& inSettings) = default;
	CharacterBaseSettings& operator = (const CharacterBaseSettings& inSettings) = default;

	// Virtual destructor
	virtual								~CharacterBaseSettings() = default;
	// Vector indicating the up direction of the character
	Vec3								mUp = Vec3::sAxisY();

	// Plane, defined in local space relative to the character. Every contact behind this plane can support the
	// character, every contact in front of this plane is treated as only colliding with the player.
	// Default: Accept any contact.
	Plane								mSupportingVolume { Vec3::sAxisY(), -1.0e10f };
	// Maximum angle of slope that character can still walk on (radians).
	float								mMaxSlopeAngle = DegreesToRadians(50.0f);
	// Set to indicate that extra effort should be made to try to remove ghost contacts (collisions with internal edges of a mesh). This is more expensive but makes bodies move smoother over a mesh with convex edges.
	bool								mEnhancedInternalEdgeRemoval = false;
	// Initial shape that represents the character's volume.
	// Usually this is a capsule, make sure the shape is made so that the bottom of the shape is at (0, 0, 0).
	RefConst<Shape>						mShape;
};

/* Character*/
struct MOSS_API CharacterSettings : public CharacterBaseSettings {
public:
	MOSS_OVERRIDE_NEW_DELETE
	ObjectLayer							mLayer = 0;				// Layer that this character will be added to
	float								mMass = 80.0f;			// Mass of the character
	float								mFriction = 0.2f;		// Friction for the character
	float								mGravityFactor = 1.0f;	// Value to multiply gravity with for this character
	EAllowedDOFs						mAllowedDOFs = EAllowedDOFs::TranslationX | EAllowedDOFs::TranslationY | EAllowedDOFs::TranslationZ;	// Allowed degrees of freedom for this character
};
/* CharacterVirtual*/
// Contains the configuration of a character
class MOSS_API CharacterVirtualSettings : public CharacterBaseSettings {
public:
	MOSS_OVERRIDE_NEW_DELETE

	// ID to give to this character. This is used for deterministically sorting and as an identifier to represent the character in the contact removal callback.
	CharacterID							mID = CharacterID::sNextCharacterID();

	// Character mass (kg). Used to push down objects with gravity when the character is standing on top.
	float								mMass = 70.0f;

	// Maximum force with which the character can push other bodies (N).
	float								mMaxStrength = 100.0f;

	// An extra offset applied to the shape in local space. This allows applying an extra offset to the shape in local space.
	Vec3								mShapeOffset = Vec3::sZero();

	//@name Movement settings
	EBackFaceMode						mBackFaceMode = EBackFaceMode::CollideWithBackFaces;	// When colliding with back faces, the character will not be able to move through back facing triangles. Use this if you have triangles that need to collide on both sides.
	float								mPredictiveContactDistance = 0.1f;						// How far to scan outside of the shape for predictive contacts. A value of 0 will most likely cause the character to get stuck as it cannot properly calculate a sliding direction anymore. A value that's too high will cause ghost collisions.
	uint8								mMaxCollisionIterations = 5;							// Max amount of collision loops
	uint8								mMaxConstraintIterations = 15;							// How often to try stepping in the constraint solving
	float								mMinTimeRemaining = 1.0e-4f;							// Early out condition: If this much time is left to simulate we are done
	float								mCollisionTolerance = 1.0e-3f;							// How far we're willing to penetrate geometry
	float								mCharacterPadding = 0.02f;								// How far we try to stay away from the geometry, this ensures that the sweep will hit as little as possible lowering the collision cost and reducing the risk of getting stuck
	uint8								mMaxNumHits = 256;										// Max num hits to collect in order to avoid excess of contact points collection
	float								mHitReductionCosMaxAngle = 0.999f;						// Cos(angle) where angle is the maximum angle between two hits contact normals that are allowed to be merged during hit reduction. Default is around 2.5 degrees. Set to -1 to turn off.
	float								mPenetrationRecoverySpeed = 1.0f;						// This value governs how fast a penetration will be resolved, 0 = nothing is resolved, 1 = everything in one update

	// This character can optionally have an inner rigid body. This rigid body can be used to give the character presence in the world. When set it means that:
	// - Regular collision checks (e.g. NarrowPhaseQuery::CastRay) will collide with the rigid body (they cannot collide with CharacterVirtual since it is not added to the broad phase)
	// - Regular contact callbacks will be called through the ContactListener (next to the ones that will be passed to the CharacterContactListener)
	// - Fast moving objects of motion quality LinearCast will not be able to pass through the CharacterVirtual in 1 time step
	RefConst<Shape>						mInnerBodyShape;

	// For a deterministic simulation, it is important to have a deterministic body ID. When set and when mInnerBodyShape is specified,
	// the inner body will be created with this specified ID instead of a generated ID.
	BodyID								mInnerBodyIDOverride;

	// Layer that the inner rigid body will be added to
	ObjectLayer							mInnerBodyLayer = 0;
};

struct CharacterContactSettings {
	bool canPushCharacter;
	bool canReceiveImpulses;
};

typedef struct CharacterVirtualContact {
	uint64_t						hash;
	BodyID						bodyB;
	CharacterID					characterIDB;
	SubShapeID					subShapeIDB;
	Vec3						position;
	Vec3						linearVelocity;
	Vec3						contactNormal;
	Vec3						surfaceNormal;
	float							distance;
	float							fraction;
	MotionType					motionTypeB;
	bool							isSensorB;
	const CharacterVirtual*		characterB;
	uint64_t						userData;
	const PhysicsMaterial*		material;
	bool							hadCollision;
	bool							wasDiscarded;
	bool							canPushCharacter;
} CharacterVirtualContact;

typedef struct JobSystemThreadPoolConfig {
	uint32_t maxJobs;
	uint32_t maxBarriers;
	int32_t numThreads;
} JobSystemThreadPoolConfig;

typedef struct JobSystemConfig {
	void* context;
	QueueJobCallback* queueJob;
	QueueJobsCallback* queueJobs;
	uint32_t maxConcurrency;
	uint32_t maxBarriers;
} JobSystemConfig;

typedef struct PhysicsStepListenerContext {
	float				deltaTime;
	bool				isFirstStep;
	bool				isLastStep;
	PhysicsSystem*		physicsSystem;
} PhysicsStepListenerContext;

typedef struct PhysicsSystemSettings {
	uint32_t maxBodies; /* 10240*/
	uint32_t numBodyMutexes; /* 0*/
	uint32_t maxBodyPairs; /* 65536*/
	uint32_t maxContactConstraints; /* 10240*/
	uint32_t _padding;
	BroadPhaseLayerInterface* broadPhaseLayerInterface;
	ObjectLayerPairFilter* objectLayerPairFilter;
	ObjectVsBroadPhaseLayerFilter* objectVsBroadPhaseLayerFilter;
} PhysicsSystemSettings;

typedef struct PhysicsSettings {
	int maxInFlightBodyPairs;
	int stepListenersBatchSize;
	int stepListenerBatchesPerJob;
	float baumgarte;
	float speculativeContactDistance;
	float penetrationSlop;
	float linearCastThreshold;
	float linearCastMaxPenetration;
	float manifoldTolerance;
	float maxPenetrationDistance;
	float bodyPairCacheMaxDeltaPositionSq;
	float bodyPairCacheCosMaxDeltaRotationDiv2;
	float contactNormalCosMaxDeltaRotation;
	float contactPointPreserveLambdaMaxDistSq;
	uint32_t numVelocitySteps;
	uint32_t numPositionSteps;
	float minVelocityForRestitution;
	float timeBeforeSleep;
	float pointVelocitySleepThreshold;
	bool deterministicSimulation;
	bool constraintWarmStart;
	bool useBodyPairContactCache;
	bool useManifoldReduction;
	bool useLargeIslandSplitter;
	bool allowSleeping;
	bool checkActiveEdges;
} PhysicsSettings;








typedef void(API_CALL* TraceFunc)(const char* message);
typedef bool(API_CALL* AssertFailureFunc)(const char* expression, const char* message, const char* file, uint32_t line);

typedef void JobFunction(void* arg);
typedef void QueueJobCallback(void* context, JobFunction* job, void* arg);
typedef void QueueJobsCallback(void* context, JobFunction* job, void** args, uint32_t count);

MOSS_API JobSystem* JobSystemThreadPool_Create(const JobSystemThreadPoolConfig* config);
MOSS_API JobSystem* JobSystemCallback_Create(const JobSystemConfig* config);
MOSS_API void JobSystem_Destroy(JobSystem* jobSystem);

MOSS_API bool Init(void);
MOSS_API void Shutdown(void);
MOSS_API void SetTraceHandler(TraceFunc handler);
MOSS_API void SetAssertFailureHandler(AssertFailureFunc handler);

/* Structs free members*/
MOSS_API void CollideShapeResult_FreeMembers(CollideShapeResult* result);
MOSS_API void CollisionEstimationResult_FreeMembers(CollisionEstimationResult* result);

/* BroadPhaseLayerInterface*/
MOSS_API BroadPhaseLayerInterface* BroadPhaseLayerInterfaceMask_Create(uint32_t numBroadPhaseLayers);
MOSS_API void BroadPhaseLayerInterfaceMask_ConfigureLayer(BroadPhaseLayerInterface* bpInterface, BroadPhaseLayer broadPhaseLayer, uint32_t groupsToInclude, uint32_t groupsToExclude);

MOSS_API BroadPhaseLayerInterface* BroadPhaseLayerInterfaceTable_Create(uint32_t numObjectLayers, uint32_t numBroadPhaseLayers);
MOSS_API void BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(BroadPhaseLayerInterface* bpInterface, ObjectLayer objectLayer, BroadPhaseLayer broadPhaseLayer);

/* ObjectLayerPairFilter*/
MOSS_API ObjectLayerPairFilter* ObjectLayerPairFilterMask_Create(void);
MOSS_API ObjectLayer ObjectLayerPairFilterMask_GetObjectLayer(uint32_t group, uint32_t mask);
MOSS_API uint32_t ObjectLayerPairFilterMask_GetGroup(ObjectLayer layer);
MOSS_API uint32_t ObjectLayerPairFilterMask_GetMask(ObjectLayer layer);

MOSS_API ObjectLayerPairFilter* ObjectLayerPairFilterTable_Create(uint32_t numObjectLayers);
MOSS_API void ObjectLayerPairFilterTable_DisableCollision(ObjectLayerPairFilter* objectFilter, ObjectLayer layer1, ObjectLayer layer2);
MOSS_API void ObjectLayerPairFilterTable_EnableCollision(ObjectLayerPairFilter* objectFilter, ObjectLayer layer1, ObjectLayer layer2);
MOSS_API bool ObjectLayerPairFilterTable_ShouldCollide(ObjectLayerPairFilter* objectFilter, ObjectLayer layer1, ObjectLayer layer2);

/* ObjectVsBroadPhaseLayerFilter*/
MOSS_API ObjectVsBroadPhaseLayerFilter* ObjectVsBroadPhaseLayerFilterMask_Create(const BroadPhaseLayerInterface* broadPhaseLayerInterface);

MOSS_API ObjectVsBroadPhaseLayerFilter* ObjectVsBroadPhaseLayerFilterTable_Create(
	BroadPhaseLayerInterface* broadPhaseLayerInterface, uint32_t numBroadPhaseLayers,
	ObjectLayerPairFilter* objectLayerPairFilter, uint32_t numObjectLayers);

MOSS_API void DrawSettings_InitDefault(DrawSettings* settings);




// A listener class that receives collision contact events for soft bodies against rigid bodies.
// It can be registered with the PhysicsSystem.
class SoftBodyContactListener
{
public:
	// Ensure virtual destructor
	virtual							~SoftBodyContactListener() = default;

	// Called whenever the soft body's aabox overlaps with another body's aabox (so receiving this callback doesn't tell if any of the vertices will collide).
	// This callback can be used to change the behavior of the collision response for all vertices in the soft body or to completely reject the contact.
	// Note that this callback is called when all bodies are locked, so don't use any locking functions!
	// @param inSoftBody The soft body that collided. It is safe to access this as the soft body is only updated on the current thread.
	// @param inOtherBody The other body that collided. Note that accessing the position/orientation/velocity of inOtherBody may result in a race condition as other threads may be modifying the body at the same time.
	// @param ioSettings The settings for all contact points that are generated by this collision.
	// @return Whether the contact should be processed or not.
	virtual SoftBodyValidateResult	OnSoftBodyContactValidate([[maybe_unused]] const Body& inSoftBody, [[maybe_unused]] const Body& inOtherBody, [[maybe_unused]] SoftBodyContactSettings& ioSettings) { return SoftBodyValidateResult::AcceptContact; }

	// Called after all contact points for a soft body have been handled. You only receive one callback per body pair per simulation step and can use inManifold to iterate through all contacts.
	// Note that this callback is called when all bodies are locked, so don't use any locking functions!
	// You will receive a single callback for a soft body per simulation step for performance reasons, this callback will apply to all vertices in the soft body.
	// @param inSoftBody The soft body that collided. It is safe to access this as the soft body is only updated on the current thread.
	// @param inManifold The manifold that describes the contact surface between the two bodies. Other bodies may be modified by other threads during this callback.
	virtual void					OnSoftBodyContactAdded([[maybe_unused]] const Body& inSoftBody, const SoftBodyManifold& inManifold) { /* Do nothing*/ }
};


/* PhysicsSystem*/
class MOSS_EXPORT PhysicsSystem : public NonCopyable
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	// Constructor / Destructor
	PhysicsSystem()	: mContactManager(mPhysicsSettings) MOSS_IF_ENABLE_ASSERTS(, mConstraintManager(&mBodyManager)) { }
	~PhysicsSystem();

	// The maximum value that can be passed to Init for inMaxBodies.
	static constexpr uint		cMaxBodiesLimit = BodyID::cMaxBodyIndex + 1;

	// The maximum value that can be passed to Init for inMaxBodyPairs.
	// Note you should really use a lower value, using this value will cost a lot of memory!
	// On a 32 bit platform, you'll run out of memory way before you reach this limit.
	static constexpr uint		cMaxBodyPairsLimit = ContactConstraintManager::cMaxBodyPairsLimit;

	// The maximum value that can be passed to Init for inMaxContactConstraints.
	// Note you should really use a lower value, using this value will cost a lot of memory!
	// On a 32 bit platform, you'll run out of memory way before you reach this limit.
	static constexpr uint		cMaxContactConstraintsLimit = ContactConstraintManager::cMaxContactConstraintsLimit;

	// Initialize the system.
	// @param inMaxBodies Maximum number of bodies to support.
	// @param inNumBodyMutexes Number of body mutexes to use. Should be a power of 2 in the range [1, 64], use 0 to auto detect.
	// @param inMaxBodyPairs Maximum amount of body pairs to process (anything else will fall through the world), this number should generally be much higher than the max amount of contact points as there will be lots of bodies close that are not actually touching.
	// @param inMaxContactConstraints Maximum amount of contact constraints to process (anything else will fall through the world).
	// @param inBroadPhaseLayerInterface Information on the mapping of object layers to broad phase layers. Since this is a virtual interface, the instance needs to stay alive during the lifetime of the PhysicsSystem.
	// @param inObjectVsBroadPhaseLayerFilter Filter callback function that is used to determine if an object layer collides with a broad phase layer. Since this is a virtual interface, the instance needs to stay alive during the lifetime of the PhysicsSystem.
	// @param inObjectLayerPairFilter Filter callback function that is used to determine if two object layers collide. Since this is a virtual interface, the instance needs to stay alive during the lifetime of the PhysicsSystem.
	void						Init(uint inMaxBodies, uint inNumBodyMutexes, uint inMaxBodyPairs, uint inMaxContactConstraints, const BroadPhaseLayerInterface& inBroadPhaseLayerInterface, const ObjectVsBroadPhaseLayerFilter& inObjectVsBroadPhaseLayerFilter, const ObjectLayerPairFilter& inObjectLayerPairFilter);

	// Listener that is notified whenever a body is activated/deactivated
	void						SetBodyActivationListener(BodyActivationListener*inListener) { mBodyManager.SetBodyActivationListener(inListener); }
	BodyActivationListener*	GetBodyActivationListener() const							{ return mBodyManager.GetBodyActivationListener(); }

	// Listener that is notified whenever a contact point between two bodies is added/updated/removed.
	// You can't change contact listener during PhysicsSystem::Update but it can be changed at any other time.
	void						SetContactListener(ContactListener*inListener)				{ mContactManager.SetContactListener(inListener); }
	ContactListener*			GetContactListener() const									{ return mContactManager.GetContactListener(); }

	// Listener that is notified whenever a contact point between a soft body and another body
	void						SetSoftBodyContactListener(SoftBodyContactListener*inListener) { mSoftBodyContactListener = inListener; }
	SoftBodyContactListener*	GetSoftBodyContactListener() const							{ return mSoftBodyContactListener; }

	// Set the function that combines the friction of two bodies and returns it
	// Default method is the geometric mean: sqrt(friction1* friction2).
	void						SetCombineFriction(ContactConstraintManager::CombineFunction inCombineFriction) { mContactManager.SetCombineFriction(inCombineFriction); }
	ContactConstraintManager::CombineFunction GetCombineFriction() const					{ return mContactManager.GetCombineFriction(); }

	// Set the function that combines the restitution of two bodies and returns it
	// Default method is max(restitution1, restitution1)
	void						SetCombineRestitution(ContactConstraintManager::CombineFunction inCombineRestitution) { mContactManager.SetCombineRestitution(inCombineRestitution); }
	ContactConstraintManager::CombineFunction GetCombineRestitution() const					{ return mContactManager.GetCombineRestitution(); }

	// Set/get the shape filter that will be used during simulation. This can be used to exclude shapes within a body from colliding with each other.
	// E.g. if you have a high detail and a low detail collision model, you can attach them to the same body in a StaticCompoundShape and use the ShapeFilter
	// to exclude the high detail collision model when simulating and exclude the low detail collision model when casting rays. Note that in this case
	// you would need to pass the inverse of inShapeFilter to the CastRay function. Pass a nullptr to disable the shape filter.
	// The PhysicsSystem does not own the ShapeFilter, make sure it stays alive during the lifetime of the PhysicsSystem.
	void						SetSimShapeFilter(const SimShapeFilter*inShapeFilter)		{ mSimShapeFilter = inShapeFilter; }
	const SimShapeFilter*		GetSimShapeFilter() const									{ return mSimShapeFilter; }

	// Advanced use only: This function is similar to CollisionDispatch::sCollideShapeVsShape but only used to collide bodies during simulation.
	// inBody1 The first body to collide.
	// inBody2 The second body to collide.
	// inCenterOfMassTransform1 The center of mass transform of the first body (note this will not be the actual world space position of the body, it will be made relative to some position so we can drop down to single precision).
	// inCenterOfMassTransform2 The center of mass transform of the second body.
	// ioCollideShapeSettings Settings that control the collision detection. Note that the implementation can freely overwrite the shape settings if needed, the caller provides a temporary that will not be used after the function returns.
	// ioCollector The collector that will receive the contact points.
	// inShapeFilter The shape filter that can be used to exclude shapes from colliding with each other.
	using SimCollideBodyVsBody = std::function<void(const Body& inBody1, const Body& inBody2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, CollideShapeSettings& ioCollideShapeSettings, CollideShapeCollector& ioCollector, const ShapeFilter& inShapeFilter)>;

	// Advanced use only: Set the function that will be used to collide two bodies during simulation.
	// This function is expected to eventually call CollideShapeCollector::AddHit all contact points between the shapes of body 1 and 2 in their given transforms.
	void						SetSimCollideBodyVsBody(const SimCollideBodyVsBody& inBodyVsBody) { mSimCollideBodyVsBody = inBodyVsBody; }
	const SimCollideBodyVsBody& GetSimCollideBodyVsBody() const								{ return mSimCollideBodyVsBody; }

	// Advanced use only: Default function that is used to collide two bodies during simulation.
	static void					sDefaultSimCollideBodyVsBody(const Body& inBody1, const Body& inBody2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, CollideShapeSettings& ioCollideShapeSettings, CollideShapeCollector& ioCollector, const ShapeFilter& inShapeFilter);

	// Control the main constants of the physics simulation
	void						SetPhysicsSettings(const PhysicsSettings& inSettings)		{ mPhysicsSettings = inSettings; }
	const PhysicsSettings& 		GetPhysicsSettings() const									{ return mPhysicsSettings; }

	// Access to the body interface. This interface allows to to create / remove bodies and to change their properties.
	const BodyInterface& 		GetBodyInterface() const									{ return mBodyInterfaceLocking; }
	BodyInterface& 				GetBodyInterface()											{ return mBodyInterfaceLocking; }
	const BodyInterface& 		GetBodyInterfaceNoLock() const								{ return mBodyInterfaceNoLock; } // Version that does not lock the bodies, use with great care!
	BodyInterface& 				GetBodyInterfaceNoLock()									{ return mBodyInterfaceNoLock; } // Version that does not lock the bodies, use with great care!

	// Access to the broadphase interface that allows coarse collision queries
	const BroadPhaseQuery& 		GetBroadPhaseQuery() const									{ return*mBroadPhase; }

	// Interface that allows fine collision queries against first the broad phase and then the narrow phase.
	const NarrowPhaseQuery& 	GetNarrowPhaseQuery() const									{ return mNarrowPhaseQueryLocking; }
	const NarrowPhaseQuery& 	GetNarrowPhaseQueryNoLock() const							{ return mNarrowPhaseQueryNoLock; } // Version that does not lock the bodies, use with great care!

	// Add constraint to the world
	void						AddConstraint(Constraint*inConstraint)						{ mConstraintManager.Add(&inConstraint, 1); }

	// Remove constraint from the world
	void						RemoveConstraint(Constraint*inConstraint)					{ mConstraintManager.Remove(&inConstraint, 1); }

	// Batch add constraints.
	void						AddConstraints(Constraint**inConstraints, int inNumber)	{ mConstraintManager.Add(inConstraints, inNumber); }

	// Batch remove constraints.
	void						RemoveConstraints(Constraint**inConstraints, int inNumber)	{ mConstraintManager.Remove(inConstraints, inNumber); }

	// Get a list of all constraints
	Constraints					GetConstraints() const										{ return mConstraintManager.GetConstraints(); }

	// Optimize the broadphase, needed only if you've added many bodies prior to calling Update() for the first time.
	// Don't call this every frame as PhysicsSystem::Update spreads out the same work over multiple frames.
	// If you add many bodies through BodyInterface::AddBodiesPrepare/AddBodiesFinalize and if the bodies in a batch are
	// in a roughly unoccupied space (e.g. a new level section) then a call to OptimizeBroadPhase is also not needed
	// as batch adding creates an efficient bounding volume hierarchy.
	// Don't call this function while bodies are being modified from another thread or use the locking BodyInterface to modify bodies.
	void						OptimizeBroadPhase();

	// Adds a new step listener
	void						AddStepListener(PhysicsStepListener*inListener);

	// Removes a step listener
	void						RemoveStepListener(PhysicsStepListener*inListener);

	// Simulate the system.
	// The world steps for a total of inDeltaTime seconds. This is divided in inCollisionSteps iterations.
	// Each iteration consists of collision detection followed by an integration step.
	// This function internally spawns jobs using inJobSystem and waits for them to complete, so no jobs will be running when this function returns.
	// The temp allocator is used, for example, to store the list of bodies that are in contact, how they form islands together
	// and data to solve the contacts between bodies. At the end of the Update call, all allocated memory will have been freed.
	EPhysicsUpdateError			Update(float inDeltaTime, int inCollisionSteps, TempAllocator*inTempAllocator, JobSystem*inJobSystem);

	// Saving state for replay
	void						SaveState(StateRecorder& inStream, EStateRecorderState inState = EStateRecorderState::All, const StateRecorderFilter*inFilter = nullptr) const;

	// Restoring state for replay. Returns false if failed.
	bool						RestoreState(StateRecorder& inStream, const StateRecorderFilter*inFilter = nullptr);

	// Saving state of a single body.
	void						SaveBodyState(const Body& inBody, StateRecorder& inStream) const;

	// Restoring state of a single body.
	void						RestoreBodyState(Body& ioBody, StateRecorder& inStream);

#ifndef MOSS_DEBUG_RENDERER
	// Drawing properties
	static bool					sDrawMotionQualityLinearCast;								// Draw debug info for objects that perform continuous collision detection through the linear cast motion quality

	// Draw the state of the bodies (debugging purposes)
	void						DrawBodies(const BodyManager::DrawSettings& inSettings, DebugRenderer*inRenderer, const BodyDrawFilter*inBodyFilter = nullptr) { mBodyManager.Draw(inSettings, mPhysicsSettings, inRenderer, inBodyFilter); }

	// Draw the constraints only (debugging purposes)
	void						DrawConstraints(DebugRenderer*inRenderer)					{ mConstraintManager.DrawConstraints(inRenderer); }

	// Draw the constraint limits only (debugging purposes)
	void						DrawConstraintLimits(DebugRenderer*inRenderer)				{ mConstraintManager.DrawConstraintLimits(inRenderer); }

	// Draw the constraint reference frames only (debugging purposes)
	void						DrawConstraintReferenceFrame(DebugRenderer*inRenderer)		{ mConstraintManager.DrawConstraintReferenceFrame(inRenderer); }
#endif // MOSS_DEBUG_RENDERER

	// Set gravity value
	void						SetGravity(Vec3Arg inGravity)								{ mGravity = inGravity; }
	Vec3						GetGravity() const											{ return mGravity; }

	// Returns a locking interface that won't actually lock the body. Use with great care!
	inline const BodyLockInterfaceNoLock& 	GetBodyLockInterfaceNoLock() const				{ return mBodyLockInterfaceNoLock; }

	// Returns a locking interface that locks the body so other threads cannot modify it.
	inline const BodyLockInterfaceLocking& 	GetBodyLockInterface() const					{ return mBodyLockInterfaceLocking; }

	// Get an broadphase layer filter that uses the default pair filter and a specified object layer to determine if broadphase layers collide
	DefaultBroadPhaseLayerFilter GetDefaultBroadPhaseLayerFilter(ObjectLayer inLayer) const	{ return DefaultBroadPhaseLayerFilter(*mObjectVsBroadPhaseLayerFilter, inLayer); }

	// Get an object layer filter that uses the default pair filter and a specified layer to determine if layers collide
	DefaultObjectLayerFilter	GetDefaultLayerFilter(ObjectLayer inLayer) const			{ return DefaultObjectLayerFilter(*mObjectLayerPairFilter, inLayer); }

	// Gets the current amount of bodies that are in the body manager
	uint						GetNumBodies() const										{ return mBodyManager.GetNumBodies(); }

	// Gets the current amount of active bodies that are in the body manager
	uint32						GetNumActiveBodies(EBodyType inType) const					{ return mBodyManager.GetNumActiveBodies(inType); }

	// Get the maximum amount of bodies that this physics system supports
	uint						GetMaxBodies() const										{ return mBodyManager.GetMaxBodies(); }

	// Helper struct that counts the number of bodies of each type
	using BodyStats = BodyManager::BodyStats;

	// Get stats about the bodies in the body manager (slow, iterates through all bodies)
	BodyStats					GetBodyStats() const										{ return mBodyManager.GetBodyStats(); }

	// Get copy of the list of all bodies under protection of a lock.
	// @param outBodyIDs On return, this will contain the list of BodyIDs
	void						GetBodies(BodyIDVector& outBodyIDs) const					{ return mBodyManager.GetBodyIDs(outBodyIDs); }

	// Get copy of the list of active bodies under protection of a lock.
	// @param inType The type of bodies to get
	// @param outBodyIDs On return, this will contain the list of BodyIDs
	void						GetActiveBodies(EBodyType inType, BodyIDVector& outBodyIDs) const { return mBodyManager.GetActiveBodies(inType, outBodyIDs); }

	// Get the list of active bodies, use GetNumActiveBodies() to find out how long the list is.
	// Note: Not thread safe. The active bodies list can change at any moment when other threads are doing work. Use GetActiveBodies() if you need a thread safe version.
	const BodyID*				GetActiveBodiesUnsafe(EBodyType inType) const				{ return mBodyManager.GetActiveBodiesUnsafe(inType); }

	// Check if 2 bodies were in contact during the last simulation step. Since contacts are only detected between active bodies, so at least one of the bodies must be active in order for this function to work.
	// It queries the state at the time of the last PhysicsSystem::Update and will return true if the bodies were in contact, even if one of the bodies was moved / removed afterwards.
	// This function can be called from any thread when the PhysicsSystem::Update is not running. During PhysicsSystem::Update this function is only valid during contact callbacks:
	// - During the ContactListener::OnContactAdded callback this function can be used to determine if a different contact pair between the bodies was active in the previous simulation step (function returns true) or if this is the first step that the bodies are touching (function returns false).
	// - During the ContactListener::OnContactRemoved callback this function can be used to determine if this is the last contact pair between the bodies (function returns false) or if there are other contacts still present (function returns true).
	bool						WereBodiesInContact(const BodyID& inBody1ID, const BodyID& inBody2ID) const { return mContactManager.WereBodiesInContact(inBody1ID, inBody2ID); }

	// Get the bounding box of all bodies in the physics system
	AABox						GetBounds() const											{ return mBroadPhase->GetBounds(); }

#ifdef MOSS_TRACK_BROADPHASE_STATS
	// Trace the accumulated broadphase stats to the TTY
	void						ReportBroadphaseStats()										{ mBroadPhase->ReportStats(); }
#endif // MOSS_TRACK_BROADPHASE_STATS

private:
	using CCDBody = PhysicsUpdateContext::Step::CCDBody;

	// Various job entry points
	void						JobStepListeners(PhysicsUpdateContext::Step*ioStep);
	void						JobDetermineActiveConstraints(PhysicsUpdateContext::Step*ioStep) const;
	void						JobApplyGravity(const PhysicsUpdateContext*ioContext, PhysicsUpdateContext::Step*ioStep);
	void						JobSetupVelocityConstraints(float inDeltaTime, PhysicsUpdateContext::Step*ioStep) const;
	void						JobBuildIslandsFromConstraints(PhysicsUpdateContext*ioContext, PhysicsUpdateContext::Step*ioStep);
	void						JobFindCollisions(PhysicsUpdateContext::Step*ioStep, int inJobIndex);
	void						JobFinalizeIslands(PhysicsUpdateContext*ioContext);
	void						JobBodySetIslandIndex();
	void						JobSolveVelocityConstraints(PhysicsUpdateContext*ioContext, PhysicsUpdateContext::Step*ioStep);
	void						JobPreIntegrateVelocity(PhysicsUpdateContext*ioContext, PhysicsUpdateContext::Step*ioStep);
	void						JobIntegrateVelocity(const PhysicsUpdateContext*ioContext, PhysicsUpdateContext::Step*ioStep);
	void						JobPostIntegrateVelocity(PhysicsUpdateContext*ioContext, PhysicsUpdateContext::Step*ioStep) const;
	void						JobFindCCDContacts(const PhysicsUpdateContext*ioContext, PhysicsUpdateContext::Step*ioStep);
	void						JobResolveCCDContacts(PhysicsUpdateContext*ioContext, PhysicsUpdateContext::Step*ioStep);
	void						JobContactRemovedCallbacks(const PhysicsUpdateContext::Step*ioStep);
	void						JobSolvePositionConstraints(PhysicsUpdateContext*ioContext, PhysicsUpdateContext::Step*ioStep);
	void						JobSoftBodyPrepare(PhysicsUpdateContext*ioContext, PhysicsUpdateContext::Step*ioStep);
	void						JobSoftBodyCollide(PhysicsUpdateContext*ioContext) const;
	void						JobSoftBodySimulate(PhysicsUpdateContext*ioContext, uint inThreadIndex) const;
	void						JobSoftBodyFinalize(PhysicsUpdateContext*ioContext);

	// Tries to spawn a new FindCollisions job if max concurrency hasn't been reached yet
	void						TrySpawnJobFindCollisions(PhysicsUpdateContext::Step*ioStep) const;

	using ContactAllocator = ContactConstraintManager::ContactAllocator;

	// Process narrow phase for a single body pair
	void						ProcessBodyPair(ContactAllocator& ioContactAllocator, const BodyPair& inBodyPair);

	// This helper batches up bodies that need to put to sleep to avoid contention on the activation mutex
	class BodiesToSleep;

	// Called at the end of JobSolveVelocityConstraints to check if bodies need to go to sleep and to update their bounding box in the broadphase
	void						CheckSleepAndUpdateBounds(uint32 inIslandIndex, const PhysicsUpdateContext*ioContext, const PhysicsUpdateContext::Step*ioStep, BodiesToSleep& ioBodiesToSleep);

	// Number of constraints to process at once in JobDetermineActiveConstraints
	static constexpr int		cDetermineActiveConstraintsBatchSize = 64;

	// Number of constraints to process at once in JobSetupVelocityConstraints, we want a low number of threads working on this so we take fairly large batches
	static constexpr int		cSetupVelocityConstraintsBatchSize = 256;

	// Number of bodies to process at once in JobApplyGravity
	static constexpr int		cApplyGravityBatchSize = 64;

	// Number of active bodies to test for collisions per batch
	static constexpr int		cActiveBodiesBatchSize = 16;

	// Number of active bodies to integrate velocities for
	static constexpr int		cIntegrateVelocityBatchSize = 64;

	// Number of contacts that need to be queued before another narrow phase job is started
	static constexpr int		cNarrowPhaseBatchSize = 16;

	// Number of continuous collision shape casts that need to be queued before another job is started
	static constexpr int		cNumCCDBodiesPerJob = 4;

	// Broadphase layer filter that decides if two objects can collide
	const ObjectVsBroadPhaseLayerFilter*mObjectVsBroadPhaseLayerFilter = nullptr;

	// Object layer filter that decides if two objects can collide
	const ObjectLayerPairFilter*mObjectLayerPairFilter = nullptr;

	// The body manager keeps track which bodies are in the simulation
	BodyManager					mBodyManager;

	// Body locking interfaces
	BodyLockInterfaceNoLock		mBodyLockInterfaceNoLock { mBodyManager };
	BodyLockInterfaceLocking	mBodyLockInterfaceLocking { mBodyManager };

	// Body interfaces
	BodyInterface				mBodyInterfaceNoLock;
	BodyInterface				mBodyInterfaceLocking;

	// Narrow phase query interface
	NarrowPhaseQuery			mNarrowPhaseQueryNoLock;
	NarrowPhaseQuery			mNarrowPhaseQueryLocking;

	// The broadphase does quick collision detection between body pairs
	BroadPhase*				mBroadPhase = nullptr;

	// The soft body contact listener
	SoftBodyContactListener*	mSoftBodyContactListener = nullptr;

	// The shape filter that is used to filter out sub shapes during simulation
	const SimShapeFilter*		mSimShapeFilter = nullptr;

	// The collision function that is used to collide two shapes during simulation
	SimCollideBodyVsBody		mSimCollideBodyVsBody =& sDefaultSimCollideBodyVsBody;

	// Simulation settings
	PhysicsSettings				mPhysicsSettings;

	// The contact manager resolves all contacts during a simulation step
	ContactConstraintManager	mContactManager;

	// All non-contact constraints
	ConstraintManager			mConstraintManager;

	// Keeps track of connected bodies and builds islands for multithreaded velocity/position update
	IslandBuilder				mIslandBuilder;

	// Will split large islands into smaller groups of bodies that can be processed in parallel
	LargeIslandSplitter			mLargeIslandSplitter;

	// Mutex protecting mStepListeners
	Mutex						mStepListenersMutex;

	// List of physics step listeners
	using StepListeners = TArray<PhysicsStepListener*>;
	StepListeners				mStepListeners;

	// This is the global gravity vector
	Vec3						mGravity = Vec3(0, -9.81f, 0);

	// Previous frame's delta time of one sub step to allow scaling previous frame's constraint impulses
	float						mPreviousStepDeltaTime = 0.0f;
};

/* PhysicsStepListener*/
class MOSS_API PhysicsStepListener
{
public:
	// Ensure virtual destructor
	virtual					~PhysicsStepListener() = default;

	// Called before every simulation step (received inCollisionSteps times for every PhysicsSystem::Update(...) call)
	// This is called while all body and constraint mutexes are locked. You can read/write bodies and constraints but not add/remove them.
	// Multiple listeners can be executed in parallel and it is the responsibility of the listener to avoid race conditions.
	// The best way to do this is to have each step listener operate on a subset of the bodies and constraints
	// and making sure that these bodies and constraints are not touched by any other step listener.
	// Note that this function is not called if there aren't any active bodies or when the physics system is updated with 0 delta time.
	virtual void			OnStep(const PhysicsStepListenerContext& inContext) = 0;
};

/* GroupFilter/GroupFilterTable*/
MOSS_API void GroupFilter_Destroy(GroupFilter* groupFilter);
MOSS_API bool GroupFilter_CanCollide(GroupFilter* groupFilter, const CollisionGroup* group1, const CollisionGroup* group2);

MOSS_API GroupFilterTable* GroupFilterTable_Create(uint32_t numSubGroups/* = 0*/);
MOSS_API void GroupFilterTable_DisableCollision(GroupFilterTable* table, CollisionSubGroupID subGroup1, CollisionSubGroupID subGroup2);
MOSS_API void GroupFilterTable_EnableCollision(GroupFilterTable* table, CollisionSubGroupID subGroup1, CollisionSubGroupID subGroup2);
MOSS_API bool GroupFilterTable_IsCollisionEnabled(GroupFilterTable* table, CollisionSubGroupID subGroup1, CollisionSubGroupID subGroup2);

/* ShapeSettings*/
MOSS_API void ShapeSettings_Destroy(ShapeSettings* settings);
MOSS_API uint64_t ShapeSettings_GetUserData(const ShapeSettings* settings);
MOSS_API void ShapeSettings_SetUserData(ShapeSettings* settings, uint64_t userData);

/* Shape*/
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

/* ConvexShape*/
MOSS_API float ConvexShapeSettings_GetDensity(const ConvexShapeSettings* shape);
MOSS_API void ConvexShapeSettings_SetDensity(ConvexShapeSettings* shape, float value);
MOSS_API float ConvexShape_GetDensity(const ConvexShape* shape);
MOSS_API void ConvexShape_SetDensity(ConvexShape* shape, float inDensity);

/* BoxShape*/
MOSS_API BoxShapeSettings* BoxShapeSettings_Create(const Vec3* halfExtent, float convexRadius);
MOSS_API BoxShape* BoxShapeSettings_CreateShape(const BoxShapeSettings* settings);

MOSS_API BoxShape* BoxShape_Create(const Vec3* halfExtent, float convexRadius);
MOSS_API void BoxShape_GetHalfExtent(const BoxShape* shape, Vec3* halfExtent);
MOSS_API float BoxShape_GetConvexRadius(const BoxShape* shape);

/* SphereShape*/
MOSS_API SphereShapeSettings* SphereShapeSettings_Create(float radius);
MOSS_API SphereShape* SphereShapeSettings_CreateShape(const SphereShapeSettings* settings);

MOSS_API float SphereShapeSettings_GetRadius(const SphereShapeSettings* settings);
MOSS_API void SphereShapeSettings_SetRadius(SphereShapeSettings* settings, float radius);
MOSS_API SphereShape* SphereShape_Create(float radius);
MOSS_API float SphereShape_GetRadius(const SphereShape* shape);

/* PlaneShape*/
MOSS_API PlaneShapeSettings* PlaneShapeSettings_Create(const Plane* plane, const PhysicsMaterial* material, float halfExtent);
MOSS_API PlaneShape* PlaneShapeSettings_CreateShape(const PlaneShapeSettings* settings);
MOSS_API PlaneShape* PlaneShape_Create(const Plane* plane, const PhysicsMaterial* material, float halfExtent);
MOSS_API void PlaneShape_GetPlane(const PlaneShape* shape, Plane* result);
MOSS_API float PlaneShape_GetHalfExtent(const PlaneShape* shape);

/* TriangleShape*/
MOSS_API TriangleShapeSettings* TriangleShapeSettings_Create(const Vec3* v1, const Vec3* v2, const Vec3* v3, float convexRadius);
MOSS_API TriangleShape* TriangleShapeSettings_CreateShape(const TriangleShapeSettings* settings);

MOSS_API TriangleShape* TriangleShape_Create(const Vec3* v1, const Vec3* v2, const Vec3* v3, float convexRadius);
MOSS_API float TriangleShape_GetConvexRadius(const TriangleShape* shape);
MOSS_API void TriangleShape_GetVertex1(const TriangleShape* shape, Vec3* result);
MOSS_API void TriangleShape_GetVertex2(const TriangleShape* shape, Vec3* result);
MOSS_API void TriangleShape_GetVertex3(const TriangleShape* shape, Vec3* result);

/* CapsuleShape*/
MOSS_API CapsuleShapeSettings* CapsuleShapeSettings_Create(float halfHeightOfCylinder, float radius);
MOSS_API CapsuleShape* CapsuleShapeSettings_CreateShape(const CapsuleShapeSettings* settings);
MOSS_API CapsuleShape* CapsuleShape_Create(float halfHeightOfCylinder, float radius);
MOSS_API float CapsuleShape_GetRadius(const CapsuleShape* shape);
MOSS_API float CapsuleShape_GetHalfHeightOfCylinder(const CapsuleShape* shape);

/* CylinderShape*/
MOSS_API CylinderShapeSettings* CylinderShapeSettings_Create(float halfHeight, float radius, float convexRadius);
MOSS_API CylinderShape* CylinderShapeSettings_CreateShape(const CylinderShapeSettings* settings);

MOSS_API CylinderShape* CylinderShape_Create(float halfHeight, float radius);
MOSS_API float CylinderShape_GetRadius(const CylinderShape* shape);
MOSS_API float CylinderShape_GetHalfHeight(const CylinderShape* shape);

/* TaperedCylinderShape*/
MOSS_API TaperedCylinderShapeSettings* TaperedCylinderShapeSettings_Create(float halfHeightOfTaperedCylinder, float topRadius, float bottomRadius, float convexRadius/* = cDefaultConvexRadius*/, const PhysicsMaterial* material /* = NULL*/);
MOSS_API TaperedCylinderShape* TaperedCylinderShapeSettings_CreateShape(const TaperedCylinderShapeSettings* settings);
MOSS_API float TaperedCylinderShape_GetTopRadius(const TaperedCylinderShape* shape);
MOSS_API float TaperedCylinderShape_GetBottomRadius(const TaperedCylinderShape* shape);
MOSS_API float TaperedCylinderShape_GetConvexRadius(const TaperedCylinderShape* shape);
MOSS_API float TaperedCylinderShape_GetHalfHeight(const TaperedCylinderShape* shape);

/* ConvexHullShape*/
MOSS_API ConvexHullShapeSettings* ConvexHullShapeSettings_Create(const Vec3* points, uint32_t pointsCount, float maxConvexRadius);
MOSS_API ConvexHullShape* ConvexHullShapeSettings_CreateShape(const ConvexHullShapeSettings* settings);
MOSS_API uint32_t ConvexHullShape_GetNumPoints(const ConvexHullShape* shape);
MOSS_API void ConvexHullShape_GetPoint(const ConvexHullShape* shape, uint32_t index, Vec3* result);
MOSS_API uint32_t ConvexHullShape_GetNumFaces(const ConvexHullShape* shape);
MOSS_API uint32_t ConvexHullShape_GetNumVerticesInFace(const ConvexHullShape* shape, uint32_t faceIndex);
MOSS_API uint32_t ConvexHullShape_GetFaceVertices(const ConvexHullShape* shape, uint32_t faceIndex, uint32_t maxVertices, uint32_t* vertices);

/* MeshShape*/
MOSS_API MeshShapeSettings* MeshShapeSettings_Create(const Triangle* triangles, uint32_t triangleCount);
MOSS_API MeshShapeSettings* MeshShapeSettings_Create2(const Vec3* vertices, uint32_t verticesCount, const IndexedTriangle* triangles, uint32_t triangleCount);
MOSS_API uint32_t MeshShapeSettings_GetMaxTrianglesPerLeaf(const MeshShapeSettings* settings);
MOSS_API void MeshShapeSettings_SetMaxTrianglesPerLeaf(MeshShapeSettings* settings, uint32_t value);
MOSS_API float MeshShapeSettings_GetActiveEdgeCosThresholdAngle(const MeshShapeSettings* settings);
MOSS_API void MeshShapeSettings_SetActiveEdgeCosThresholdAngle(MeshShapeSettings* settings, float value);
MOSS_API bool MeshShapeSettings_GetPerTriangleUserData(const MeshShapeSettings* settings);
MOSS_API void MeshShapeSettings_SetPerTriangleUserData(MeshShapeSettings* settings, bool value);
MOSS_API Mesh_Shape_BuildQuality MeshShapeSettings_GetBuildQuality(const MeshShapeSettings* settings);
MOSS_API void MeshShapeSettings_SetBuildQuality(MeshShapeSettings* settings, Mesh_Shape_BuildQuality value);

MOSS_API void MeshShapeSettings_Sanitize(MeshShapeSettings* settings);
MOSS_API MeshShape* MeshShapeSettings_CreateShape(const MeshShapeSettings* settings);
MOSS_API uint32_t MeshShape_GetTriangleUserData(const MeshShape* shape, SubShapeID id);

/* HeightFieldShape*/
MOSS_API HeightFieldShapeSettings* HeightFieldShapeSettings_Create(const float* samples, const Vec3* offset, const Vec3* scale, uint32_t sampleCount);
MOSS_API HeightFieldShape* HeightFieldShapeSettings_CreateShape(HeightFieldShapeSettings* settings);
MOSS_API void HeightFieldShapeSettings_DetermineMinAndMaxSample(const HeightFieldShapeSettings* settings, float* pOutMinValue, float* pOutMaxValue, float* pOutQuantizationScale);
MOSS_API uint32_t HeightFieldShapeSettings_CalculateBitsPerSampleForError(const HeightFieldShapeSettings* settings, float maxError);

MOSS_API uint32_t HeightFieldShape_GetSampleCount(const HeightFieldShape* shape);
MOSS_API uint32_t HeightFieldShape_GetBlockSize(const HeightFieldShape* shape);
MOSS_API const PhysicsMaterial* HeightFieldShape_GetMaterial(const HeightFieldShape* shape, uint32_t x, uint32_t y);
MOSS_API void HeightFieldShape_GetPosition(const HeightFieldShape* shape, uint32_t x, uint32_t y, Vec3* result);
MOSS_API bool HeightFieldShape_IsNoCollision(const HeightFieldShape* shape, uint32_t x, uint32_t y);
MOSS_API bool HeightFieldShape_ProjectOntoSurface(const HeightFieldShape* shape, const Vec3* localPosition, Vec3* outSurfacePosition, SubShapeID* outSubShapeID);
MOSS_API float HeightFieldShape_GetMinHeightValue(const HeightFieldShape* shape);
MOSS_API float HeightFieldShape_GetMaxHeightValue(const HeightFieldShape* shape);

/* TaperedCapsuleShape*/
MOSS_API TaperedCapsuleShapeSettings* TaperedCapsuleShapeSettings_Create(float halfHeightOfTaperedCylinder, float topRadius, float bottomRadius);
MOSS_API TaperedCapsuleShape* TaperedCapsuleShapeSettings_CreateShape(TaperedCapsuleShapeSettings* settings);

MOSS_API float TaperedCapsuleShape_GetTopRadius(const TaperedCapsuleShape* shape);
MOSS_API float TaperedCapsuleShape_GetBottomRadius(const TaperedCapsuleShape* shape);
MOSS_API float TaperedCapsuleShape_GetHalfHeight(const TaperedCapsuleShape* shape);

/* CompoundShape*/
MOSS_API void CompoundShapeSettings_AddShape(CompoundShapeSettings* settings, const Vec3* position, const Quat* rotation, const ShapeSettings* shapeSettings, uint32_t userData);
MOSS_API void CompoundShapeSettings_AddShape2(CompoundShapeSettings* settings, const Vec3* position, const Quat* rotation, const Shape* shape, uint32_t userData);
MOSS_API uint32_t CompoundShape_GetNumSubShapes(const CompoundShape* shape);
MOSS_API void CompoundShape_GetSubShape(const CompoundShape* shape, uint32_t index, const Shape** subShape, Vec3* positionCOM, Quat* rotation, uint32_t* userData);
MOSS_API uint32_t CompoundShape_GetSubShapeIndexFromID(const CompoundShape* shape, SubShapeID id, SubShapeID* remainder);

/* StaticCompoundShape*/
MOSS_API StaticCompoundShapeSettings* StaticCompoundShapeSettings_Create(void);
MOSS_API StaticCompoundShape* StaticCompoundShape_Create(const StaticCompoundShapeSettings* settings);

/* MutableCompoundShape*/
MOSS_API MutableCompoundShapeSettings* MutableCompoundShapeSettings_Create(void);
MOSS_API MutableCompoundShape* MutableCompoundShape_Create(const MutableCompoundShapeSettings* settings);

MOSS_API uint32_t MutableCompoundShape_AddShape(MutableCompoundShape* shape, const Vec3* position, const Quat* rotation, const Shape* child, uint32_t userData /* = 0*/, uint32_t index /* = UINT32_MAX*/);
MOSS_API void MutableCompoundShape_RemoveShape(MutableCompoundShape* shape, uint32_t index);
MOSS_API void MutableCompoundShape_ModifyShape(MutableCompoundShape* shape, uint32_t index, const Vec3* position, const Quat* rotation);
MOSS_API void MutableCompoundShape_ModifyShape2(MutableCompoundShape* shape, uint32_t index, const Vec3* position, const Quat* rotation, const Shape* newShape);
MOSS_API void MutableCompoundShape_AdjustCenterOfMass(MutableCompoundShape* shape);

/* DecoratedShape*/
MOSS_API const Shape* DecoratedShape_GetInnerShape(const DecoratedShape* shape);

/* RotatedTranslatedShape*/
MOSS_API RotatedTranslatedShapeSettings* RotatedTranslatedShapeSettings_Create(const Vec3* position, const Quat* rotation, const ShapeSettings* shapeSettings);
MOSS_API RotatedTranslatedShapeSettings* RotatedTranslatedShapeSettings_Create2(const Vec3* position, const Quat* rotation, const Shape* shape);
MOSS_API RotatedTranslatedShape* RotatedTranslatedShapeSettings_CreateShape(const RotatedTranslatedShapeSettings* settings);
MOSS_API RotatedTranslatedShape* RotatedTranslatedShape_Create(const Vec3* position, const Quat* rotation, const Shape* shape);
MOSS_API void RotatedTranslatedShape_GetPosition(const RotatedTranslatedShape* shape, Vec3* position);
MOSS_API void RotatedTranslatedShape_GetRotation(const RotatedTranslatedShape* shape, Quat* rotation);

/* ScaledShape*/
MOSS_API ScaledShapeSettings* ScaledShapeSettings_Create(const ShapeSettings* shapeSettings, const Vec3* scale);
MOSS_API ScaledShapeSettings* ScaledShapeSettings_Create2(const Shape* shape, const Vec3* scale);
MOSS_API ScaledShape* ScaledShapeSettings_CreateShape(const ScaledShapeSettings* settings);
MOSS_API ScaledShape* ScaledShape_Create(const Shape* shape, const Vec3* scale);
MOSS_API void ScaledShape_GetScale(const ScaledShape* shape, Vec3* result);

/* OffsetCenterOfMassShape*/
MOSS_API OffsetCenterOfMassShapeSettings* OffsetCenterOfMassShapeSettings_Create(const Vec3* offset, const ShapeSettings* shapeSettings);
MOSS_API OffsetCenterOfMassShapeSettings* OffsetCenterOfMassShapeSettings_Create2(const Vec3* offset, const Shape* shape);
MOSS_API OffsetCenterOfMassShape* OffsetCenterOfMassShapeSettings_CreateShape(const OffsetCenterOfMassShapeSettings* settings);

MOSS_API OffsetCenterOfMassShape* OffsetCenterOfMassShape_Create(const Vec3* offset, const Shape* shape);
MOSS_API void OffsetCenterOfMassShape_GetOffset(const OffsetCenterOfMassShape* shape, Vec3* result);

/* EmptyShape*/
MOSS_API EmptyShapeSettings* EmptyShapeSettings_Create(const Vec3* centerOfMass);
MOSS_API EmptyShape* EmptyShapeSettings_CreateShape(const EmptyShapeSettings* settings);

/* BodyCreationSettings*/
MOSS_API BodyCreationSettings* BodyCreationSettings_Create(void);
MOSS_API BodyCreationSettings* BodyCreationSettings_Create2(const ShapeSettings* settings,
	const Vec3* position,
	const Quat* rotation,
	MotionType motionType,
	ObjectLayer objectLayer);
MOSS_API BodyCreationSettings* BodyCreationSettings_Create3(const Shape* shape,
	const Vec3* position,
	const Quat* rotation,
	MotionType motionType,
	ObjectLayer objectLayer);
MOSS_API void BodyCreationSettings_Destroy(BodyCreationSettings* settings);

MOSS_API void BodyCreationSettings_GetPosition(BodyCreationSettings* settings, Vec3* result);
MOSS_API void BodyCreationSettings_SetPosition(BodyCreationSettings* settings, const Vec3* value);

MOSS_API void BodyCreationSettings_GetRotation(BodyCreationSettings* settings, Quat* result);
MOSS_API void BodyCreationSettings_SetRotation(BodyCreationSettings* settings, const Quat* value);

MOSS_API void BodyCreationSettings_GetLinearVelocity(BodyCreationSettings* settings, Vec3* velocity);
MOSS_API void BodyCreationSettings_SetLinearVelocity(BodyCreationSettings* settings, const Vec3* velocity);

MOSS_API void BodyCreationSettings_GetAngularVelocity(BodyCreationSettings* settings, Vec3* velocity);
MOSS_API void BodyCreationSettings_SetAngularVelocity(BodyCreationSettings* settings, const Vec3* velocity);

MOSS_API uint64_t BodyCreationSettings_GetUserData(const BodyCreationSettings* settings);
MOSS_API void BodyCreationSettings_SetUserData(BodyCreationSettings* settings, uint64_t value);

MOSS_API ObjectLayer BodyCreationSettings_GetObjectLayer(const BodyCreationSettings* settings);
MOSS_API void BodyCreationSettings_SetObjectLayer(BodyCreationSettings* settings, ObjectLayer value);

MOSS_API void BodyCreationSettings_GetCollisionGroup(const BodyCreationSettings* settings, CollisionGroup* result);
MOSS_API void BodyCreationSettings_SetCollisionGroup(BodyCreationSettings* settings, const CollisionGroup* value);

MOSS_API MotionType BodyCreationSettings_GetMotionType(const BodyCreationSettings* settings);
MOSS_API void BodyCreationSettings_SetMotionType(BodyCreationSettings* settings, MotionType value);

MOSS_API AllowedDOFs BodyCreationSettings_GetAllowedDOFs(const BodyCreationSettings* settings);
MOSS_API void BodyCreationSettings_SetAllowedDOFs(BodyCreationSettings* settings, AllowedDOFs value);

MOSS_API bool BodyCreationSettings_GetAllowDynamicOrKinematic(const BodyCreationSettings* settings);
MOSS_API void BodyCreationSettings_SetAllowDynamicOrKinematic(BodyCreationSettings* settings, bool value);

MOSS_API bool BodyCreationSettings_GetIsSensor(const BodyCreationSettings* settings);
MOSS_API void BodyCreationSettings_SetIsSensor(BodyCreationSettings* settings, bool value);

MOSS_API bool BodyCreationSettings_GetCollideKinematicVsNonDynamic(const BodyCreationSettings* settings);
MOSS_API void BodyCreationSettings_SetCollideKinematicVsNonDynamic(BodyCreationSettings* settings, bool value);

MOSS_API bool BodyCreationSettings_GetUseManifoldReduction(const BodyCreationSettings* settings);
MOSS_API void BodyCreationSettings_SetUseManifoldReduction(BodyCreationSettings* settings, bool value);

MOSS_API bool BodyCreationSettings_GetApplyGyroscopicForce(const BodyCreationSettings* settings);
MOSS_API void BodyCreationSettings_SetApplyGyroscopicForce(BodyCreationSettings* settings, bool value);

MOSS_API MotionQuality BodyCreationSettings_GetMotionQuality(const BodyCreationSettings* settings);
MOSS_API void BodyCreationSettings_SetMotionQuality(BodyCreationSettings* settings, MotionQuality value);

MOSS_API bool BodyCreationSettings_GetEnhancedInternalEdgeRemoval(const BodyCreationSettings* settings);
MOSS_API void BodyCreationSettings_SetEnhancedInternalEdgeRemoval(BodyCreationSettings* settings, bool value);

MOSS_API bool BodyCreationSettings_GetAllowSleeping(const BodyCreationSettings* settings);
MOSS_API void BodyCreationSettings_SetAllowSleeping(BodyCreationSettings* settings, bool value);

MOSS_API float BodyCreationSettings_GetFriction(const BodyCreationSettings* settings);
MOSS_API void BodyCreationSettings_SetFriction(BodyCreationSettings* settings, float value);

MOSS_API float BodyCreationSettings_GetRestitution(const BodyCreationSettings* settings);
MOSS_API void BodyCreationSettings_SetRestitution(BodyCreationSettings* settings, float value);

MOSS_API float BodyCreationSettings_GetLinearDamping(const BodyCreationSettings* settings);
MOSS_API void BodyCreationSettings_SetLinearDamping(BodyCreationSettings* settings, float value);

MOSS_API float BodyCreationSettings_GetAngularDamping(const BodyCreationSettings* settings);
MOSS_API void BodyCreationSettings_SetAngularDamping(BodyCreationSettings* settings, float value);

MOSS_API float BodyCreationSettings_GetMaxLinearVelocity(const BodyCreationSettings* settings);
MOSS_API void BodyCreationSettings_SetMaxLinearVelocity(BodyCreationSettings* settings, float value);

MOSS_API float BodyCreationSettings_GetMaxAngularVelocity(const BodyCreationSettings* settings);
MOSS_API void BodyCreationSettings_SetMaxAngularVelocity(BodyCreationSettings* settings, float value);

MOSS_API float BodyCreationSettings_GetGravityFactor(const BodyCreationSettings* settings);
MOSS_API void BodyCreationSettings_SetGravityFactor(BodyCreationSettings* settings, float value);

MOSS_API uint32_t BodyCreationSettings_GetNumVelocityStepsOverride(const BodyCreationSettings* settings);
MOSS_API void BodyCreationSettings_SetNumVelocityStepsOverride(BodyCreationSettings* settings, uint32_t value);

MOSS_API uint32_t BodyCreationSettings_GetNumPositionStepsOverride(const BodyCreationSettings* settings);
MOSS_API void BodyCreationSettings_SetNumPositionStepsOverride(BodyCreationSettings* settings, uint32_t value);

MOSS_API OverrideMassProperties BodyCreationSettings_GetOverrideMassProperties(const BodyCreationSettings* settings);
MOSS_API void BodyCreationSettings_SetOverrideMassProperties(BodyCreationSettings* settings, OverrideMassProperties value);

MOSS_API float BodyCreationSettings_GetInertiaMultiplier(const BodyCreationSettings* settings);
MOSS_API void BodyCreationSettings_SetInertiaMultiplier(BodyCreationSettings* settings, float value);

MOSS_API void BodyCreationSettings_GetMassPropertiesOverride(const BodyCreationSettings* settings, MassProperties* result);
MOSS_API void BodyCreationSettings_SetMassPropertiesOverride(BodyCreationSettings* settings, const MassProperties* massProperties);

/* BodyInterface*/
class MOSS_EXPORT BodyInterface : public NonCopyable {
public:
	// Initialize the interface (should only be called by PhysicsSystem)
	void						Init(BodyLockInterface& inBodyLockInterface, BodyManager& inBodyManager, BroadPhase& inBroadPhase) { mBodyLockInterface =& inBodyLockInterface; mBodyManager =& inBodyManager; mBroadPhase =& inBroadPhase; }

	// Create a rigid body
	// @return Created body or null when out of bodies
	Body*						CreateBody(const BodyCreationSettings& inSettings);

	// Create a soft body
	// @return Created body or null when out of bodies
	Body*						CreateSoftBody(const SoftBodyCreationSettings& inSettings);

	// Create a rigid body with specified ID. This function can be used if a simulation is to run in sync between clients or if a simulation needs to be restored exactly.
	// The ID created on the server can be replicated to the client and used to create a deterministic simulation.
	// @return Created body or null when the body ID is invalid or a body of the same ID already exists.
	Body*						CreateBodyWithID(const BodyID& inBodyID, const BodyCreationSettings& inSettings);

	// Create a soft body with specified ID. See comments at CreateBodyWithID.
	Body*						CreateSoftBodyWithID(const BodyID& inBodyID, const SoftBodyCreationSettings& inSettings);

	// Advanced use only. Creates a rigid body without specifying an ID. This body cannot be added to the physics system until it has been assigned a body ID.
	// This can be used to decouple allocation from registering the body. A call to CreateBodyWithoutID followed by AssignBodyID is equivalent to calling CreateBodyWithID.
	// @return Created body
	Body*						CreateBodyWithoutID(const BodyCreationSettings& inSettings) const;

	// Advanced use only. Creates a body without specifying an ID. See comments at CreateBodyWithoutID.
	Body*						CreateSoftBodyWithoutID(const SoftBodyCreationSettings& inSettings) const;

	// Advanced use only. Destroy a body previously created with CreateBodyWithoutID that hasn't gotten an ID yet through the AssignBodyID function,
	// or a body that has had its body ID unassigned through UnassignBodyIDs. Bodies that have an ID should be destroyed through DestroyBody.
	void						DestroyBodyWithoutID(Body*inBody) const;

	// Advanced use only. Assigns the next available body ID to a body that was created using CreateBodyWithoutID. After this call, the body can be added to the physics system.
	// @return false if the body already has an ID or out of body ids.
	bool						AssignBodyID(Body*ioBody);

	// Advanced use only. Assigns a body ID to a body that was created using CreateBodyWithoutID. After this call, the body can be added to the physics system.
	// @return false if the body already has an ID or if the ID is not valid.
	bool						AssignBodyID(Body*ioBody, const BodyID& inBodyID);

	// Advanced use only. See UnassignBodyIDs. Unassigns the ID of a single body.
	Body*						UnassignBodyID(const BodyID& inBodyID);

	// Advanced use only. Removes a number of body IDs from their bodies and returns the body pointers. Before calling this, the body should have been removed from the physics system.
	// The body can be destroyed through DestroyBodyWithoutID. This can be used to decouple deallocation. A call to UnassignBodyIDs followed by calls to DestroyBodyWithoutID is equivalent to calling DestroyBodies.
	// @param inBodyIDs A list of body IDs
	// @param inNumber Number of bodies in the list
	// @param outBodies If not null on input, this will contain a list of body pointers corresponding to inBodyIDs that can be destroyed afterwards (caller assumes ownership over these).
	void						UnassignBodyIDs(const BodyID*inBodyIDs, int inNumber, Body**outBodies);

	// Destroy a body.
	// Make sure that you remove the body from the physics system using BodyInterface::RemoveBody before calling this function.
	void						DestroyBody(const BodyID& inBodyID);

	// Destroy multiple bodies
	// Make sure that you remove the bodies from the physics system using BodyInterface::RemoveBody before calling this function.
	void						DestroyBodies(const BodyID*inBodyIDs, int inNumber);

	// Add body to the physics system.
	// Note that if you need to add multiple bodies, use the AddBodiesPrepare/AddBodiesFinalize function.
	// Adding many bodies, one at a time, results in a really inefficient broadphase until PhysicsSystem::OptimizeBroadPhase is called or when PhysicsSystem::Update rebuilds the tree!
	// After adding, to get a body by ID use the BodyLockRead or BodyLockWrite interface!
	void						AddBody(const BodyID& inBodyID, EActivation inActivationMode);

	// Remove body from the physics system.
	void						RemoveBody(const BodyID& inBodyID);

	// Check if a body has been added to the physics system.
	bool						IsAdded(const BodyID& inBodyID) const;

	// Combines CreateBody and AddBody
	// @return Created body ID or an invalid ID when out of bodies
	BodyID						CreateAndAddBody(const BodyCreationSettings& inSettings, EActivation inActivationMode);

	// Combines CreateSoftBody and AddBody
	// @return Created body ID or an invalid ID when out of bodies
	BodyID						CreateAndAddSoftBody(const SoftBodyCreationSettings& inSettings, EActivation inActivationMode);

	// Add state handle, used to keep track of a batch of bodies while adding them to the PhysicsSystem.
	using AddState = void*;

	//@name Batch adding interface
	//@{

	// Prepare adding inNumber bodies at ioBodies to the PhysicsSystem, returns a handle that should be used in AddBodiesFinalize/Abort.
	// This can be done on a background thread without influencing the PhysicsSystem.
	// ioBodies may be shuffled around by this function and should be kept that way until AddBodiesFinalize/Abort is called.
	AddState					AddBodiesPrepare(BodyID*ioBodies, int inNumber);

	// Finalize adding bodies to the PhysicsSystem, supply the return value of AddBodiesPrepare in inAddState.
	// Please ensure that the ioBodies array passed to AddBodiesPrepare is unmodified and passed again to this function.
	void						AddBodiesFinalize(BodyID*ioBodies, int inNumber, AddState inAddState, EActivation inActivationMode);

	// Abort adding bodies to the PhysicsSystem, supply the return value of AddBodiesPrepare in inAddState.
	// This can be done on a background thread without influencing the PhysicsSystem.
	// Please ensure that the ioBodies array passed to AddBodiesPrepare is unmodified and passed again to this function.
	void						AddBodiesAbort(BodyID*ioBodies, int inNumber, AddState inAddState);

	// Remove inNumber bodies in ioBodies from the PhysicsSystem.
	// ioBodies may be shuffled around by this function.
	void						RemoveBodies(BodyID*ioBodies, int inNumber);
	//@}

	//@name Activate / deactivate a body
	//@{
	void						ActivateBody(const BodyID& inBodyID);
	void						ActivateBodies(const BodyID*inBodyIDs, int inNumber);
	void						ActivateBodiesInAABox(const AABox& inBox, const BroadPhaseLayerFilter& inBroadPhaseLayerFilter, const ObjectLayerFilter& inObjectLayerFilter);
	void						DeactivateBody(const BodyID& inBodyID);
	void						DeactivateBodies(const BodyID*inBodyIDs, int inNumber);
	bool						IsActive(const BodyID& inBodyID) const;
	void						ResetSleepTimer(const BodyID& inBodyID);
	//@}

	// Create a two body constraint
	TwoBodyConstraint*			CreateConstraint(const TwoBodyConstraintSettings*inSettings, const BodyID& inBodyID1, const BodyID& inBodyID2);

	// Activate non-static bodies attached to a constraint
	void						ActivateConstraint(const TwoBodyConstraint*inConstraint);

	//@name Access to the shape of a body
	//@{

	// Get the current shape
	RefConst<Shape>				GetShape(const BodyID& inBodyID) const;

	// Set a new shape on the body
	// @param inBodyID Body ID of body that had its shape changed
	// @param inShape The new shape
	// @param inUpdateMassProperties When true, the mass and inertia tensor is recalculated
	// @param inActivationMode Whether or not to activate the body
	void						SetShape(const BodyID& inBodyID, const Shape*inShape, bool inUpdateMassProperties, EActivation inActivationMode) const;

	// Notify all systems to indicate that a shape has changed (usable for MutableCompoundShapes)
	// @param inBodyID Body ID of body that had its shape changed
	// @param inPreviousCenterOfMass Center of mass of the shape before the alterations
	// @param inUpdateMassProperties When true, the mass and inertia tensor is recalculated
	// @param inActivationMode Whether or not to activate the body
	void						NotifyShapeChanged(const BodyID& inBodyID, Vec3Arg inPreviousCenterOfMass, bool inUpdateMassProperties, EActivation inActivationMode) const;
	//@}

	//@name Object layer of a body
	//@{
	void						SetObjectLayer(const BodyID& inBodyID, ObjectLayer inLayer);
	ObjectLayer					GetObjectLayer(const BodyID& inBodyID) const;
	//@}

	//@name Position and rotation of a body
	//@{
	void						SetPositionAndRotation(const BodyID& inBodyID, RVec3Arg inPosition, QuatArg inRotation, EActivation inActivationMode);
	void						SetPositionAndRotationWhenChanged(const BodyID& inBodyID, RVec3Arg inPosition, QuatArg inRotation, EActivation inActivationMode); // Will only update the position/rotation and activate the body when the difference is larger than a very small number. This avoids updating the broadphase/waking up a body when the resulting position/orientation doesn't really change.
	void						GetPositionAndRotation(const BodyID& inBodyID, RVec3& outPosition, Quat& outRotation) const;
	void						SetPosition(const BodyID& inBodyID, RVec3Arg inPosition, EActivation inActivationMode);
	RVec3						GetPosition(const BodyID& inBodyID) const;
	RVec3						GetCenterOfMassPosition(const BodyID& inBodyID) const;
	void						SetRotation(const BodyID& inBodyID, QuatArg inRotation, EActivation inActivationMode);
	Quat						GetRotation(const BodyID& inBodyID) const;
	RMat44						GetWorldTransform(const BodyID& inBodyID) const;
	RMat44						GetCenterOfMassTransform(const BodyID& inBodyID) const;
	//@}

	// Set velocity of body such that it will be positioned at inTargetPosition/Rotation in inDeltaTime seconds (will activate body if needed)
	void						MoveKinematic(const BodyID& inBodyID, RVec3Arg inTargetPosition, QuatArg inTargetRotation, float inDeltaTime);

	// Linear or angular velocity (functions will activate body if needed).
	// Note that the linear velocity is the velocity of the center of mass, which may not coincide with the position of your object, to correct for this: \f$VelocityCOM = Velocity - AngularVelocity \times ShapeCOM\f$
	void						SetLinearAndAngularVelocity(const BodyID& inBodyID, Vec3Arg inLinearVelocity, Vec3Arg inAngularVelocity);
	void						GetLinearAndAngularVelocity(const BodyID& inBodyID, Vec3& outLinearVelocity, Vec3& outAngularVelocity) const;
	void						SetLinearVelocity(const BodyID& inBodyID, Vec3Arg inLinearVelocity);
	Vec3						GetLinearVelocity(const BodyID& inBodyID) const;
	void						AddLinearVelocity(const BodyID& inBodyID, Vec3Arg inLinearVelocity); // Add velocity to current velocity
	void						AddLinearAndAngularVelocity(const BodyID& inBodyID, Vec3Arg inLinearVelocity, Vec3Arg inAngularVelocity); // Add linear and angular to current velocities
	void						SetAngularVelocity(const BodyID& inBodyID, Vec3Arg inAngularVelocity);
	Vec3						GetAngularVelocity(const BodyID& inBodyID) const;
	Vec3						GetPointVelocity(const BodyID& inBodyID, RVec3Arg inPoint) const; // Velocity of point inPoint (in world space, e.g. on the surface of the body) of the body

	// Set the complete motion state of a body.
	// Note that the linear velocity is the velocity of the center of mass, which may not coincide with the position of your object, to correct for this: \f$VelocityCOM = Velocity - AngularVelocity \times ShapeCOM\f$
	void						SetPositionRotationAndVelocity(const BodyID& inBodyID, RVec3Arg inPosition, QuatArg inRotation, Vec3Arg inLinearVelocity, Vec3Arg inAngularVelocity);

	//@name Add forces to the body
	//@{
	void						AddForce(const BodyID& inBodyID, Vec3Arg inForce, EActivation inActivationMode = EActivation::Activate); // See Body::AddForce
	void						AddForce(const BodyID& inBodyID, Vec3Arg inForce, RVec3Arg inPoint, EActivation inActivationMode = EActivation::Activate); // Applied at inPoint
	void						AddTorque(const BodyID& inBodyID, Vec3Arg inTorque, EActivation inActivationMode = EActivation::Activate); // See Body::AddTorque
	void						AddForceAndTorque(const BodyID& inBodyID, Vec3Arg inForce, Vec3Arg inTorque, EActivation inActivationMode = EActivation::Activate); // A combination of Body::AddForce and Body::AddTorque
	//@}

	//@name Add an impulse to the body
	//@{
	void						AddImpulse(const BodyID& inBodyID, Vec3Arg inImpulse); // Applied at center of mass
	void						AddImpulse(const BodyID& inBodyID, Vec3Arg inImpulse, RVec3Arg inPoint); // Applied at inPoint
	void						AddAngularImpulse(const BodyID& inBodyID, Vec3Arg inAngularImpulse);
	bool						ApplyBuoyancyImpulse(const BodyID& inBodyID, RVec3Arg inSurfacePosition, Vec3Arg inSurfaceNormal, float inBuoyancy, float inLinearDrag, float inAngularDrag, Vec3Arg inFluidVelocity, Vec3Arg inGravity, float inDeltaTime);
	//@}

	//@name Body type
	//@{
	EBodyType					GetBodyType(const BodyID& inBodyID) const;
	//@}

	//@name Body motion type
	//@{
	void						SetMotionType(const BodyID& inBodyID, EMotionType inMotionType, EActivation inActivationMode);
	EMotionType					GetMotionType(const BodyID& inBodyID) const;
	//@}

	//@name Body motion quality
	//@{
	void						SetMotionQuality(const BodyID& inBodyID, EMotionQuality inMotionQuality);
	EMotionQuality				GetMotionQuality(const BodyID& inBodyID) const;
	//@}

	// Get inverse inertia tensor in world space
	Mat44						GetInverseInertia(const BodyID& inBodyID) const;

	//@name Restitution
	//@{
	void						SetRestitution(const BodyID& inBodyID, float inRestitution);
	float						GetRestitution(const BodyID& inBodyID) const;
	//@}

	//@name Friction
	//@{
	void						SetFriction(const BodyID& inBodyID, float inFriction);
	float						GetFriction(const BodyID& inBodyID) const;
	//@}

	//@name Gravity factor
	//@{
	void						SetGravityFactor(const BodyID& inBodyID, float inGravityFactor);
	float						GetGravityFactor(const BodyID& inBodyID) const;
	//@}

	//@name Manifold reduction
	//@{
	void						SetUseManifoldReduction(const BodyID& inBodyID, bool inUseReduction);
	bool						GetUseManifoldReduction(const BodyID& inBodyID) const;
	//@}

	//@name Collision group
	//@{
	void						SetCollisionGroup(const BodyID& inBodyID, const CollisionGroup& inCollisionGroup);
	const CollisionGroup& 		GetCollisionGroup(const BodyID& inBodyID) const;
	//@}

	// Get transform and shape for this body, used to perform collision detection
	TransformedShape			GetTransformedShape(const BodyID& inBodyID) const;

	// Get the user data for a body
	uint64						GetUserData(const BodyID& inBodyID) const;
	void						SetUserData(const BodyID& inBodyID, uint64 inUserData) const;

	// Get the material for a particular sub shape
	const PhysicsMaterial*		GetMaterial(const BodyID& inBodyID, const SubShapeID& inSubShapeID) const;

	// Set the Body::EFlags::InvalidateContactCache flag for the specified body. This means that the collision cache is invalid for any body pair involving that body until the next physics step.
	void						InvalidateContactCache(const BodyID& inBodyID);

private:
	// Helper function to activate a single body
	MOSS_INLINE void				ActivateBodyInternal(Body& ioBody) const;

	BodyLockInterface*			mBodyLockInterface = nullptr;
	BodyManager*				mBodyManager = nullptr;
	BroadPhase*				mBroadPhase = nullptr;
};

//--------------------------------------------------------------------------------------------------
// BodyLockInterface
//--------------------------------------------------------------------------------------------------
MOSS_API void BodyLockInterface_LockRead(const BodyLockInterface* lockInterface, BodyID bodyID, BodyLockRead* outLock);
MOSS_API void BodyLockInterface_UnlockRead(const BodyLockInterface* lockInterface, BodyLockRead* ioLock);

MOSS_API void BodyLockInterface_LockWrite(const BodyLockInterface* lockInterface, BodyID bodyID, BodyLockWrite* outLock);
MOSS_API void BodyLockInterface_UnlockWrite(const BodyLockInterface* lockInterface, BodyLockWrite* ioLock);

MOSS_API BodyLockMultiRead* BodyLockInterface_LockMultiRead(const BodyLockInterface* lockInterface, const BodyID* bodyIDs, uint32_t count);
MOSS_API void BodyLockMultiRead_Destroy(BodyLockMultiRead* ioLock);
MOSS_API const Body* BodyLockMultiRead_GetBody(BodyLockMultiRead* ioLock, uint32_t bodyIndex);

MOSS_API BodyLockMultiWrite* BodyLockInterface_LockMultiWrite(const BodyLockInterface* lockInterface, const BodyID* bodyIDs, uint32_t count);
MOSS_API void BodyLockMultiWrite_Destroy(BodyLockMultiWrite* ioLock);
MOSS_API Body* BodyLockMultiWrite_GetBody(BodyLockMultiWrite* ioLock, uint32_t bodyIndex);

//--------------------------------------------------------------------------------------------------
// MotionProperties
//--------------------------------------------------------------------------------------------------
// The Body class only keeps track of state for static bodies, the MotionProperties class keeps the additional state needed for a moving Body. It has a 1-on-1 relationship with the body.
class MOSS_API MotionProperties {
public:
	MOSS_OVERRIDE_NEW_DELETE

	// Motion quality, or how well it detects collisions when it has a high velocity
	EMotionQuality			GetMotionQuality() const										{ return mMotionQuality; }

	// Get the allowed degrees of freedom that this body has (this can be changed by calling SetMassProperties)
	inline EAllowedDOFs		GetAllowedDOFs() const											{ return mAllowedDOFs; }

	// If this body can go to sleep.
	inline bool				GetAllowSleeping() const										{ return mAllowSleeping; }

	// Get world space linear velocity of the center of mass
	inline Vec3				GetLinearVelocity() const										{ MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sVelocityAccess(), BodyAccess::EAccess::Read)); return mLinearVelocity; }

	// Set world space linear velocity of the center of mass
	void					SetLinearVelocity(Vec3Arg inLinearVelocity)						{ MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sVelocityAccess(), BodyAccess::EAccess::ReadWrite)); MOSS_ASSERT(inLinearVelocity.Length() <= mMaxLinearVelocity); mLinearVelocity = LockTranslation(inLinearVelocity); }

	// Set world space linear velocity of the center of mass, will make sure the value is clamped against the maximum linear velocity
	void					SetLinearVelocityClamped(Vec3Arg inLinearVelocity)				{ mLinearVelocity = LockTranslation(inLinearVelocity); ClampLinearVelocity(); }

	// Get world space angular velocity of the center of mass
	inline Vec3				GetAngularVelocity() const										{ MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sVelocityAccess(), BodyAccess::EAccess::Read)); return mAngularVelocity; }

	// Set world space angular velocity of the center of mass
	void					SetAngularVelocity(Vec3Arg inAngularVelocity)					{ MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sVelocityAccess(), BodyAccess::EAccess::ReadWrite)); MOSS_ASSERT(inAngularVelocity.Length() <= mMaxAngularVelocity); mAngularVelocity = LockAngular(inAngularVelocity); }

	// Set world space angular velocity of the center of mass, will make sure the value is clamped against the maximum angular velocity
	void					SetAngularVelocityClamped(Vec3Arg inAngularVelocity)			{ mAngularVelocity = LockAngular(inAngularVelocity); ClampAngularVelocity(); }

	// Set velocity of body such that it will be rotate/translate by inDeltaPosition/Rotation in inDeltaTime seconds.
	inline void				MoveKinematic(Vec3Arg inDeltaPosition, QuatArg inDeltaRotation, float inDeltaTime);

	//@name Velocity limits
	//@{

	// Maximum linear velocity that a body can achieve. Used to prevent the system from exploding.
	inline float			GetMaxLinearVelocity() const									{ return mMaxLinearVelocity; }
	inline void				SetMaxLinearVelocity(float inLinearVelocity)					{ MOSS_ASSERT(inLinearVelocity >= 0.0f); mMaxLinearVelocity = inLinearVelocity; }

	// Maximum angular velocity that a body can achieve. Used to prevent the system from exploding.
	inline float			GetMaxAngularVelocity() const									{ return mMaxAngularVelocity; }
	inline void				SetMaxAngularVelocity(float inAngularVelocity)					{ MOSS_ASSERT(inAngularVelocity >= 0.0f); mMaxAngularVelocity = inAngularVelocity; }
	//@}

	// Clamp velocity according to limit
	inline void				ClampLinearVelocity();
	inline void				ClampAngularVelocity();

	// Get linear damping: dv/dt = -c* v. c must be between 0 and 1 but is usually close to 0.
	inline float			GetLinearDamping() const										{ return mLinearDamping; }
	void					SetLinearDamping(float inLinearDamping)							{ MOSS_ASSERT(inLinearDamping >= 0.0f); mLinearDamping = inLinearDamping; }

	// Get angular damping: dw/dt = -c* w. c must be between 0 and 1 but is usually close to 0.
	inline float			GetAngularDamping() const										{ return mAngularDamping; }
	void					SetAngularDamping(float inAngularDamping)						{ MOSS_ASSERT(inAngularDamping >= 0.0f); mAngularDamping = inAngularDamping; }

	// Get gravity factor (1 = normal gravity, 0 = no gravity)
	inline float			GetGravityFactor() const										{ return mGravityFactor; }
	void					SetGravityFactor(float inGravityFactor)							{ mGravityFactor = inGravityFactor; }

	// Set the mass and inertia tensor
	void					SetMassProperties(EAllowedDOFs inAllowedDOFs, const MassProperties& inMassProperties);

	// Get inverse mass (1 / mass). Should only be called on a dynamic object (static or kinematic bodies have infinite mass so should be treated as 1 / mass = 0)
	inline float			GetInverseMass() const											{ MOSS_ASSERT(mCachedMotionType == EMotionType::Dynamic); return mInvMass; }
	inline float			GetInverseMassUnchecked() const									{ return mInvMass; }

	// Set the inverse mass (1 / mass).
	// Note that mass and inertia are linearly related (e.g. inertia of a sphere with mass m and radius r is \f$2/5 \: m \: r^2\f$).
	// If you change mass, inertia should probably change as well. You can use ScaleToMass to update mass and inertia at the same time.
	// If all your translation degrees of freedom are restricted, make sure this is zero (see EAllowedDOFs).
	void					SetInverseMass(float inInverseMass)								{ mInvMass = inInverseMass; }

	// Diagonal of inverse inertia matrix: D. Should only be called on a dynamic object (static or kinematic bodies have infinite mass so should be treated as D = 0)
	inline Vec3				GetInverseInertiaDiagonal() const								{ MOSS_ASSERT(mCachedMotionType == EMotionType::Dynamic); return mInvInertiaDiagonal; }

	// Rotation (R) that takes inverse inertia diagonal to local space: \f$I_{body}^{-1} = R \: D \: R^{-1}\f$
	inline Quat				GetInertiaRotation() const										{ return mInertiaRotation; }

	// Set the inverse inertia tensor in local space by setting the diagonal and the rotation: \f$I_{body}^{-1} = R \: D \: R^{-1}\f$.
	// Note that mass and inertia are linearly related (e.g. inertia of a sphere with mass m and radius r is \f$2/5 \: m \: r^2\f$).
	// If you change inertia, mass should probably change as well. You can use ScaleToMass to update mass and inertia at the same time.
	// If all your rotation degrees of freedom are restricted, make sure this is zero (see EAllowedDOFs).
	void					SetInverseInertia(Vec3Arg inDiagonal, QuatArg inRot)			{ mInvInertiaDiagonal = inDiagonal; mInertiaRotation = inRot; }

	// Sets the mass to inMass and scale the inertia tensor based on the ratio between the old and new mass.
	// Note that this only works when the current mass is finite (i.e. the body is dynamic and translational degrees of freedom are not restricted).
	void					ScaleToMass(float inMass);

	// Get inverse inertia matrix (\f$I_{body}^{-1}\f$). Will be a matrix of zeros for a static or kinematic object.
	inline Mat44			GetLocalSpaceInverseInertia() const;

	// Same as GetLocalSpaceInverseInertia() but doesn't check if the body is dynamic
	inline Mat44			GetLocalSpaceInverseInertiaUnchecked() const;

	// Get inverse inertia matrix (\f$I^{-1}\f$) for a given object rotation (translation will be ignored). Zero if object is static or kinematic.
	inline Mat44			GetInverseInertiaForRotation(Mat44Arg inRotation) const;

	// Multiply a vector with the inverse world space inertia tensor (\f$I_{world}^{-1}\f$). Zero if object is static or kinematic.
	MOSS_INLINE Vec3			MultiplyWorldSpaceInverseInertiaByVector(QuatArg inBodyRotation, Vec3Arg inV) const;

	// Velocity of point inPoint (in center of mass space, e.g. on the surface of the body) of the body (unit: m/s)
	MOSS_INLINE Vec3			GetPointVelocityCOM(Vec3Arg inPointRelativeToCOM) const			{ return mLinearVelocity + mAngularVelocity.Cross(inPointRelativeToCOM); }

	// Get the total amount of force applied to the center of mass this time step (through Body::AddForce calls). Note that it will reset to zero after PhysicsSystem::Update.
	MOSS_INLINE Vec3			GetAccumulatedForce() const										{ return Vec3::sLoadFloat3Unsafe(mForce); }

	// Get the total amount of torque applied to the center of mass this time step (through Body::AddForce/Body::AddTorque calls). Note that it will reset to zero after PhysicsSystem::Update.
	MOSS_INLINE Vec3			GetAccumulatedTorque() const									{ return Vec3::sLoadFloat3Unsafe(mTorque); }

	// Reset the total accumulated force, note that this will be done automatically after every time step.
	MOSS_INLINE void			ResetForce()													{ mForce = Float3(0, 0, 0); }

	// Reset the total accumulated torque, note that this will be done automatically after every time step.
	MOSS_INLINE void			ResetTorque()													{ mTorque = Float3(0, 0, 0); }

	// Reset the current velocity and accumulated force and torque.
	MOSS_INLINE void			ResetMotion()
	{
		MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sVelocityAccess(), BodyAccess::EAccess::ReadWrite));
		mLinearVelocity = mAngularVelocity = Vec3::sZero();
		mForce = mTorque = Float3(0, 0, 0);
	}

	// Returns a vector where the linear components that are not allowed by mAllowedDOFs are set to 0 and the rest to 0xffffffff
	MOSS_INLINE UVec4		GetLinearDOFsMask() const
	{
		UVec4 mask(uint32(EAllowedDOFs::TranslationX), uint32(EAllowedDOFs::TranslationY), uint32(EAllowedDOFs::TranslationZ), 0);
		return UVec4::sEquals(UVec4::sAnd(UVec4::sReplicate(uint32(mAllowedDOFs)), mask), mask);
	}

	// Takes a translation vector inV and returns a vector where the components that are not allowed by mAllowedDOFs are set to 0
	MOSS_INLINE Vec3			LockTranslation(Vec3Arg inV) const
	{
		return Vec3::sAnd(inV, Vec3(GetLinearDOFsMask().ReinterpretAsFloat()));
	}

	// Returns a vector where the angular components that are not allowed by mAllowedDOFs are set to 0 and the rest to 0xffffffff
	MOSS_INLINE UVec4		GetAngularDOFsMask() const
	{
		UVec4 mask(uint32(EAllowedDOFs::RotationX), uint32(EAllowedDOFs::RotationY), uint32(EAllowedDOFs::RotationZ), 0);
		return UVec4::sEquals(UVec4::sAnd(UVec4::sReplicate(uint32(mAllowedDOFs)), mask), mask);
	}

	// Takes an angular velocity / torque vector inV and returns a vector where the components that are not allowed by mAllowedDOFs are set to 0
	MOSS_INLINE Vec3			LockAngular(Vec3Arg inV) const
	{
		return Vec3::sAnd(inV, Vec3(GetAngularDOFsMask().ReinterpretAsFloat()));
	}

	// Used only when this body is dynamic and colliding. Override for the number of solver velocity iterations to run, 0 means use the default in PhysicsSettings::mNumVelocitySteps. The number of iterations to use is the max of all contacts and constraints in the island.
	void					SetNumVelocityStepsOverride(uint inN)							{ MOSS_ASSERT(inN < 256); mNumVelocityStepsOverride = uint8(inN); }
	uint					GetNumVelocityStepsOverride() const								{ return mNumVelocityStepsOverride; }

	// Used only when this body is dynamic and colliding. Override for the number of solver position iterations to run, 0 means use the default in PhysicsSettings::mNumPositionSteps. The number of iterations to use is the max of all contacts and constraints in the island.
	void					SetNumPositionStepsOverride(uint inN)							{ MOSS_ASSERT(inN < 256); mNumPositionStepsOverride = uint8(inN); }
	uint					GetNumPositionStepsOverride() const								{ return mNumPositionStepsOverride; }

	////////////////////////////////////////
	// FUNCTIONS BELOW THIS LINE ARE FOR INTERNAL USE ONLY
	////////////////////////////////////////

	//@name Update linear and angular velocity (used during constraint solving)
	//@{
	inline void				AddLinearVelocityStep(Vec3Arg inLinearVelocityChange)			{ MOSS_DET_LOG("AddLinearVelocityStep: " << inLinearVelocityChange); MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sVelocityAccess(), BodyAccess::EAccess::ReadWrite)); mLinearVelocity = LockTranslation(mLinearVelocity + inLinearVelocityChange); MOSS_ASSERT(!mLinearVelocity.IsNaN()); }
	inline void				SubLinearVelocityStep(Vec3Arg inLinearVelocityChange)			{ MOSS_DET_LOG("SubLinearVelocityStep: " << inLinearVelocityChange); MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sVelocityAccess(), BodyAccess::EAccess::ReadWrite)); mLinearVelocity = LockTranslation(mLinearVelocity - inLinearVelocityChange); MOSS_ASSERT(!mLinearVelocity.IsNaN()); }
	inline void				AddAngularVelocityStep(Vec3Arg inAngularVelocityChange)			{ MOSS_DET_LOG("AddAngularVelocityStep: " << inAngularVelocityChange); MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sVelocityAccess(), BodyAccess::EAccess::ReadWrite)); mAngularVelocity += inAngularVelocityChange; MOSS_ASSERT(!mAngularVelocity.IsNaN()); }
	inline void				SubAngularVelocityStep(Vec3Arg inAngularVelocityChange)			{ MOSS_DET_LOG("SubAngularVelocityStep: " << inAngularVelocityChange); MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sVelocityAccess(), BodyAccess::EAccess::ReadWrite)); mAngularVelocity -= inAngularVelocityChange; MOSS_ASSERT(!mAngularVelocity.IsNaN()); }
	//@}

	// Apply the gyroscopic force (aka Dzhanibekov effect, see https://en.wikipedia.org/wiki/Tennis_racket_theorem)
	inline void				ApplyGyroscopicForceInternal(QuatArg inBodyRotation, float inDeltaTime);

	// Apply all accumulated forces, torques and drag (should only be called by the PhysicsSystem)
	inline void				ApplyForceTorqueAndDragInternal(QuatArg inBodyRotation, Vec3Arg inGravity, float inDeltaTime);

	// Access to the island index
	uint32					GetIslandIndexInternal() const									{ return mIslandIndex; }
	void					SetIslandIndexInternal(uint32 inIndex)							{ mIslandIndex = inIndex; }

	// Access to the index in the active bodies array
	uint32					GetIndexInActiveBodiesInternal() const							{ return mIndexInActiveBodies; }

#ifdef MOSS_DOUBLE_PRECISION
	inline DVec3			GetSleepTestOffset() const										{ return DVec3::sLoadDouble3Unsafe(mSleepTestOffset); }
#endif // MOSS_DOUBLE_PRECISION

	// Reset spheres to center around inPoints with radius 0
	inline void				ResetSleepTestSpheres(const RVec3*inPoints);

	// Reset the sleep test timer without resetting the sleep test spheres
	inline void				ResetSleepTestTimer()											{ mSleepTestTimer = 0.0f; }

	// Accumulate sleep time and return if a body can go to sleep
	inline ECanSleep		AccumulateSleepTime(float inDeltaTime, float inTimeBeforeSleep);

	// Saving state for replay
	void					SaveState(StateRecorder& inStream) const;

	// Restoring state for replay
	void					RestoreState(StateRecorder& inStream);

	static constexpr uint32	cInactiveIndex = uint32(-1);									// Constant indicating that body is not active

private:
	friend class BodyManager;
	friend class Body;

	// 1st cache line
	// 16 byte aligned
	Vec3					mLinearVelocity { Vec3::sZero() };								// World space linear velocity of the center of mass (m/s)
	Vec3					mAngularVelocity { Vec3::sZero() };								// World space angular velocity (rad/s)
	Vec3					mInvInertiaDiagonal;											// Diagonal of inverse inertia matrix: D
	Quat					mInertiaRotation;												// Rotation (R) that takes inverse inertia diagonal to local space: Ibody^-1 = R* D* R^-1

	// 2nd cache line
	// 4 byte aligned
	Float3					mForce { 0, 0, 0 };												// Accumulated world space force (N). Note loaded through intrinsics so ensure that the 4 bytes after this are readable!
	Float3					mTorque { 0, 0, 0 };											// Accumulated world space torque (N m). Note loaded through intrinsics so ensure that the 4 bytes after this are readable!
	float					mInvMass;														// Inverse mass of the object (1/kg)
	float					mLinearDamping;													// Linear damping: dv/dt = -c* v. c must be between 0 and 1 but is usually close to 0.
	float					mAngularDamping;												// Angular damping: dw/dt = -c* w. c must be between 0 and 1 but is usually close to 0.
	float					mMaxLinearVelocity;												// Maximum linear velocity that this body can reach (m/s)
	float					mMaxAngularVelocity;											// Maximum angular velocity that this body can reach (rad/s)
	float					mGravityFactor;													// Factor to multiply gravity with
	uint32					mIndexInActiveBodies = cInactiveIndex;							// If the body is active, this is the index in the active body list or cInactiveIndex if it is not active (note that there are 2 lists, one for rigid and one for soft bodies)
	uint32					mIslandIndex = cInactiveIndex;									// Index of the island that this body is part of, when the body has not yet been updated or is not active this is cInactiveIndex

	// 1 byte aligned
	EMotionQuality			mMotionQuality;													// Motion quality, or how well it detects collisions when it has a high velocity
	bool					mAllowSleeping;													// If this body can go to sleep
	EAllowedDOFs			mAllowedDOFs = EAllowedDOFs::All;								// Allowed degrees of freedom for this body
	uint8					mNumVelocityStepsOverride = 0;									// Used only when this body is dynamic and colliding. Override for the number of solver velocity iterations to run, 0 means use the default in PhysicsSettings::mNumVelocitySteps. The number of iterations to use is the max of all contacts and constraints in the island.
	uint8					mNumPositionStepsOverride = 0;									// Used only when this body is dynamic and colliding. Override for the number of solver position iterations to run, 0 means use the default in PhysicsSettings::mNumPositionSteps. The number of iterations to use is the max of all contacts and constraints in the island.

	// 3rd cache line (least frequently used)
	// 4 byte aligned (or 8 byte if running in double precision)
#ifdef MOSS_DOUBLE_PRECISION
	Double3					mSleepTestOffset;												// mSleepTestSpheres are relative to this offset to prevent floating point inaccuracies. Warning: Loaded using sLoadDouble3Unsafe which will read 8 extra bytes.
#endif // MOSS_DOUBLE_PRECISION
	Sphere					mSleepTestSpheres[3];											// Measure motion for 3 points on the body to see if it is resting: COM, COM + largest bounding box axis, COM + second largest bounding box axis
	float					mSleepTestTimer;												// How long this body has been within the movement tolerance

#ifdef MOSS_DEBUG
	EBodyType				mCachedBodyType;												// Copied from Body::mBodyType and cached for asserting purposes
	EMotionType				mCachedMotionType;												// Copied from Body::mMotionType and cached for asserting purposes
#endif
};
//--------------------------------------------------------------------------------------------------
// RayCast
//--------------------------------------------------------------------------------------------------
MOSS_API void RayCast_GetPointOnRay(const Vec3* origin, const Vec3* direction, float fraction, Vec3* result);
MOSS_API void RRayCast_GetPointOnRay(const Vec3* origin, const Vec3* direction, float fraction, Vec3* result);

//--------------------------------------------------------------------------------------------------
// MassProperties
//--------------------------------------------------------------------------------------------------
MOSS_API void MassProperties_DecomposePrincipalMomentsOfInertia(MassProperties* properties, Mat44* rotation, Vec3* diagonal);
MOSS_API void MassProperties_ScaleToMass(MassProperties* properties, float mass);
MOSS_API void MassProperties_GetEquivalentSolidBoxSize(float mass, const Vec3* inertiaDiagonal, Vec3* result);

//--------------------------------------------------------------------------------------------------
// CollideShapeSettings
//--------------------------------------------------------------------------------------------------
MOSS_API void CollideShapeSettings_Init(CollideShapeSettings* settings);

//--------------------------------------------------------------------------------------------------
// ShapeCastSettings
//--------------------------------------------------------------------------------------------------
MOSS_API void ShapeCastSettings_Init(ShapeCastSettings* settings);

//--------------------------------------------------------------------------------------------------
// BroadPhaseQuery
//--------------------------------------------------------------------------------------------------
MOSS_API bool BroadPhaseQuery_CastRay(const BroadPhaseQuery* query,
	const Vec3* origin, const Vec3* direction,
	RayCastBodyCollectorCallback* callback, void* userData,
	BroadPhaseLayerFilter* broadPhaseLayerFilter,
	ObjectLayerFilter* objectLayerFilter);

MOSS_API bool BroadPhaseQuery_CastRay2(const BroadPhaseQuery* query,
	const Vec3* origin, const Vec3* direction,
	CollisionCollectorType collectorType,
	RayCastBodyResultCallback* callback, void* userData,
	BroadPhaseLayerFilter* broadPhaseLayerFilter,
	ObjectLayerFilter* objectLayerFilter);

MOSS_API bool BroadPhaseQuery_CollideAABB3(const BroadPhaseQuery* query,
	const AABB3* box, CollideShapeBodyCollectorCallback* callback, void* userData,
	BroadPhaseLayerFilter* broadPhaseLayerFilter,
	ObjectLayerFilter* objectLayerFilter);

MOSS_API bool BroadPhaseQuery_CollideSphere(const BroadPhaseQuery* query,
	const Vec3* center, float radius, CollideShapeBodyCollectorCallback* callback, void* userData,
	BroadPhaseLayerFilter* broadPhaseLayerFilter,
	ObjectLayerFilter* objectLayerFilter);

MOSS_API bool BroadPhaseQuery_CollidePoint(const BroadPhaseQuery* query,
	const Vec3* point, CollideShapeBodyCollectorCallback* callback, void* userData,
	BroadPhaseLayerFilter* broadPhaseLayerFilter,
	ObjectLayerFilter* objectLayerFilter);

//--------------------------------------------------------------------------------------------------
// NarrowPhaseQuery
//--------------------------------------------------------------------------------------------------
MOSS_API bool NarrowPhaseQuery_CastRay(const NarrowPhaseQuery* query,
	const Vec3* origin, const Vec3* direction,
	RayCastResult* hit,
	BroadPhaseLayerFilter* broadPhaseLayerFilter,
	ObjectLayerFilter* objectLayerFilter,
	const BodyFilter* bodyFilter);

MOSS_API bool NarrowPhaseQuery_CastRay2(const NarrowPhaseQuery* query,
	const Vec3* origin, const Vec3* direction,
	const RayCastSettings* rayCastSettings,
	CastRayCollectorCallback* callback, void* userData,
	BroadPhaseLayerFilter* broadPhaseLayerFilter,
	ObjectLayerFilter* objectLayerFilter,
	const BodyFilter* bodyFilter,
	const ShapeFilter* shapeFilter);

MOSS_API bool NarrowPhaseQuery_CastRay3(const NarrowPhaseQuery* query,
	const Vec3* origin, const Vec3* direction,
	const RayCastSettings* rayCastSettings,
	CollisionCollectorType collectorType,
	CastRayResultCallback* callback, void* userData,
	BroadPhaseLayerFilter* broadPhaseLayerFilter,
	ObjectLayerFilter* objectLayerFilter,
	const BodyFilter* bodyFilter,
	const ShapeFilter* shapeFilter);

MOSS_API bool NarrowPhaseQuery_CollidePoint(const NarrowPhaseQuery* query,
	const Vec3* point,
	CollidePointCollectorCallback* callback, void* userData,
	BroadPhaseLayerFilter* broadPhaseLayerFilter,
	ObjectLayerFilter* objectLayerFilter,
	const BodyFilter* bodyFilter,
	const ShapeFilter* shapeFilter);

MOSS_API bool NarrowPhaseQuery_CollidePoint2(const NarrowPhaseQuery* query,
	const Vec3* point,
	CollisionCollectorType collectorType,
	CollidePointResultCallback* callback, void* userData,
	BroadPhaseLayerFilter* broadPhaseLayerFilter,
	ObjectLayerFilter* objectLayerFilter,
	const BodyFilter* bodyFilter,
	const ShapeFilter* shapeFilter);

MOSS_API bool NarrowPhaseQuery_CollideShape(const NarrowPhaseQuery* query,
	const Shape* shape, const Vec3* scale, const RMat44* centerOfMassTransform,
	const CollideShapeSettings* settings,
	Vec3* baseOffset,
	CollideShapeCollectorCallback* callback, void* userData,
	BroadPhaseLayerFilter* broadPhaseLayerFilter,
	ObjectLayerFilter* objectLayerFilter,
	const BodyFilter* bodyFilter,
	const ShapeFilter* shapeFilter);

MOSS_API bool NarrowPhaseQuery_CollideShape2(const NarrowPhaseQuery* query,
	const Shape* shape, const Vec3* scale, const RMat44* centerOfMassTransform,
	const CollideShapeSettings* settings,
	Vec3* baseOffset,
	CollisionCollectorType collectorType,
	CollideShapeResultCallback* callback, void* userData,
	BroadPhaseLayerFilter* broadPhaseLayerFilter,
	ObjectLayerFilter* objectLayerFilter,
	const BodyFilter* bodyFilter,
	const ShapeFilter* shapeFilter);

MOSS_API bool NarrowPhaseQuery_CastShape(const NarrowPhaseQuery* query,
	const Shape* shape,
	const RMat44* worldTransform, const Vec3* direction,
	const ShapeCastSettings* settings,
	Vec3* baseOffset,
	CastShapeCollectorCallback* callback, void* userData,
	BroadPhaseLayerFilter* broadPhaseLayerFilter,
	ObjectLayerFilter* objectLayerFilter,
	const BodyFilter* bodyFilter,
	const ShapeFilter* shapeFilter);

MOSS_API bool NarrowPhaseQuery_CastShape2(const NarrowPhaseQuery* query,
	const Shape* shape,
	const RMat44* worldTransform, const Vec3* direction,
	const ShapeCastSettings* settings,
	Vec3* baseOffset,
	CollisionCollectorType collectorType,
	CastShapeResultCallback* callback, void* userData,
	BroadPhaseLayerFilter* broadPhaseLayerFilter,
	ObjectLayerFilter* objectLayerFilter,
	const BodyFilter* bodyFilter,
	const ShapeFilter* shapeFilter);

//--------------------------------------------------------------------------------------------------
// Body
//--------------------------------------------------------------------------------------------------
class MOSS_EXPORT_GCC_BUG_WORKAROUND alignas(MOSS_RVECTOR_ALIGNMENT) Body : public NonCopyable {
public:
	MOSS_OVERRIDE_NEW_DELETE

	// Get the id of this body
	inline const BodyID&	GetID() const													{ return mID; }

	// Get the type of body (rigid or soft)
	inline EBodyType		GetBodyType() const												{ return mBodyType; }

	// Check if this body is a rigid body
	inline bool				IsRigidBody() const												{ return mBodyType == EBodyType::RigidBody; }

	// Check if this body is a soft body
	inline bool				IsSoftBody() const												{ return mBodyType == EBodyType::SoftBody; }

	// See comment at GetIndexInActiveBodiesInternal for reasoning why TSAN is disabled here
	MOSS_TSAN_NO_SANITIZE
	// If this body is currently actively simulating (true) or sleeping (false)
	inline bool				IsActive() const												{ return mMotionProperties != nullptr& & mMotionProperties->mIndexInActiveBodies != cInactiveIndex; }

	// Check if this body is static (not movable)
	inline bool				IsStatic() const												{ return mMotionType == EMotionType::Static; }

	// Check if this body is kinematic (keyframed), which means that it will move according to its current velocity, but forces don't affect it
	inline bool				IsKinematic() const												{ return mMotionType == EMotionType::Kinematic; }

	// Check if this body is dynamic, which means that it moves and forces can act on it
	inline bool				IsDynamic() const												{ return mMotionType == EMotionType::Dynamic; }

	// Check if a body could be made kinematic or dynamic (if it was created dynamic or with mAllowDynamicOrKinematic set to true)
	inline bool				CanBeKinematicOrDynamic() const									{ return mMotionProperties != nullptr; }

	// Change the body to a sensor. A sensor will receive collision callbacks, but will not cause any collision responses and can be used as a trigger volume.
	// The cheapest sensor (in terms of CPU usage) is a sensor with motion type Static (they can be moved around using BodyInterface::SetPosition/SetPositionAndRotation).
	// These sensors will only detect collisions with active Dynamic or Kinematic bodies. As soon as a body go to sleep, the contact point with the sensor will be lost.
	// If you make a sensor Dynamic or Kinematic and activate them, the sensor will be able to detect collisions with sleeping bodies too. An active sensor will never go to sleep automatically.
	// When you make a Dynamic or Kinematic sensor, make sure it is in an ObjectLayer that does not collide with Static bodies or other sensors to avoid extra overhead in the broad phase.
	inline void				SetIsSensor(bool inIsSensor)									{ MOSS_ASSERT(IsRigidBody()); if (inIsSensor) mFlags.fetch_or(uint8(EFlags::IsSensor), memory_order_relaxed); else mFlags.fetch_and(uint8(~uint8(EFlags::IsSensor)), memory_order_relaxed); }

	// Check if this body is a sensor.
	inline bool				IsSensor() const												{ return (mFlags.load(memory_order_relaxed)&  uint8(EFlags::IsSensor)) != 0; }

	// If kinematic objects can generate contact points against other kinematic or static objects.
	// Note that turning this on can be CPU intensive as much more collision detection work will be done without any effect on the simulation (kinematic objects are not affected by other kinematic/static objects).
	// This can be used to make sensors detect static objects. Note that the sensor must be kinematic and active for it to detect static objects.
	inline void				SetCollideKinematicVsNonDynamic(bool inCollide)					{ MOSS_ASSERT(IsRigidBody()); if (inCollide) mFlags.fetch_or(uint8(EFlags::CollideKinematicVsNonDynamic), memory_order_relaxed); else mFlags.fetch_and(uint8(~uint8(EFlags::CollideKinematicVsNonDynamic)), memory_order_relaxed); }

	// Check if kinematic objects can generate contact points against other kinematic or static objects.
	inline bool				GetCollideKinematicVsNonDynamic() const							{ return (mFlags.load(memory_order_relaxed)&  uint8(EFlags::CollideKinematicVsNonDynamic)) != 0; }

	// If PhysicsSettings::mUseManifoldReduction is true, this allows turning off manifold reduction for this specific body.
	// Manifold reduction by default will combine contacts with similar normals that come from different SubShapeIDs (e.g. different triangles in a mesh shape or different compound shapes).
	// If the application requires tracking exactly which SubShapeIDs are in contact, you can turn off manifold reduction. Note that this comes at a performance cost.
	// Consider using BodyInterface::SetUseManifoldReduction if the body could already be in contact with other bodies to ensure that the contact cache is invalidated and you get the correct contact callbacks.
	inline void				SetUseManifoldReduction(bool inUseReduction)					{ MOSS_ASSERT(IsRigidBody()); if (inUseReduction) mFlags.fetch_or(uint8(EFlags::UseManifoldReduction), memory_order_relaxed); else mFlags.fetch_and(uint8(~uint8(EFlags::UseManifoldReduction)), memory_order_relaxed); }

	// Check if this body can use manifold reduction.
	inline bool				GetUseManifoldReduction() const									{ return (mFlags.load(memory_order_relaxed)&  uint8(EFlags::UseManifoldReduction)) != 0; }

	// Checks if the combination of this body and inBody2 should use manifold reduction
	inline bool				GetUseManifoldReductionWithBody(const Body& inBody2) const		{ return ((mFlags.load(memory_order_relaxed)&  inBody2.mFlags.load(memory_order_relaxed))&  uint8(EFlags::UseManifoldReduction)) != 0; }

	// Set to indicate that the gyroscopic force should be applied to this body (aka Dzhanibekov effect, see https://en.wikipedia.org/wiki/Tennis_racket_theorem)
	inline void				SetApplyGyroscopicForce(bool inApply)							{ MOSS_ASSERT(IsRigidBody()); if (inApply) mFlags.fetch_or(uint8(EFlags::ApplyGyroscopicForce), memory_order_relaxed); else mFlags.fetch_and(uint8(~uint8(EFlags::ApplyGyroscopicForce)), memory_order_relaxed); }

	// Check if the gyroscopic force is being applied for this body
	inline bool				GetApplyGyroscopicForce() const									{ return (mFlags.load(memory_order_relaxed)&  uint8(EFlags::ApplyGyroscopicForce)) != 0; }

	// Set to indicate that extra effort should be made to try to remove ghost contacts (collisions with internal edges of a mesh). This is more expensive but makes bodies move smoother over a mesh with convex edges.
	inline void				SetEnhancedInternalEdgeRemoval(bool inApply)					{ MOSS_ASSERT(IsRigidBody()); if (inApply) mFlags.fetch_or(uint8(EFlags::EnhancedInternalEdgeRemoval), memory_order_relaxed); else mFlags.fetch_and(uint8(~uint8(EFlags::EnhancedInternalEdgeRemoval)), memory_order_relaxed); }

	// Check if enhanced internal edge removal is turned on
	inline bool				GetEnhancedInternalEdgeRemoval() const							{ return (mFlags.load(memory_order_relaxed)&  uint8(EFlags::EnhancedInternalEdgeRemoval)) != 0; }

	// Checks if the combination of this body and inBody2 should use enhanced internal edge removal
	inline bool				GetEnhancedInternalEdgeRemovalWithBody(const Body& inBody2) const { return ((mFlags.load(memory_order_relaxed) | inBody2.mFlags.load(memory_order_relaxed))&  uint8(EFlags::EnhancedInternalEdgeRemoval)) != 0; }

	// Get the bodies motion type.
	inline EMotionType		GetMotionType() const											{ return mMotionType; }

	// Set the motion type of this body. Consider using BodyInterface::SetMotionType instead of this function if the body may be active or if it needs to be activated.
	void					SetMotionType(EMotionType inMotionType);

	// Get broadphase layer, this determines in which broad phase sub-tree the object is placed
	inline BroadPhaseLayer	GetBroadPhaseLayer() const										{ return mBroadPhaseLayer; }

	// Get object layer, this determines which other objects it collides with
	inline ObjectLayer		GetObjectLayer() const											{ return mObjectLayer; }

	// Collision group and sub-group ID, determines which other objects it collides with
	const CollisionGroup& 	GetCollisionGroup() const										{ return mCollisionGroup; }
	CollisionGroup& 		GetCollisionGroup()												{ return mCollisionGroup; }
	void					SetCollisionGroup(const CollisionGroup& inGroup)				{ mCollisionGroup = inGroup; }

	// If this body can go to sleep. Note that disabling sleeping on a sleeping object will not wake it up.
	bool					GetAllowSleeping() const										{ return mMotionProperties->mAllowSleeping; }
	void					SetAllowSleeping(bool inAllow);

	// Resets the sleep timer. This does not wake up the body if it is sleeping, but allows resetting the system that detects when a body is sleeping.
	inline void				ResetSleepTimer();

	// Friction (dimensionless number, usually between 0 and 1, 0 = no friction, 1 = friction force equals force that presses the two bodies together). Note that bodies can have negative friction but the combined friction (see PhysicsSystem::SetCombineFriction) should never go below zero.
	inline float			GetFriction() const												{ return mFriction; }
	void					SetFriction(float inFriction)									{ mFriction = inFriction; }

	// Restitution (dimensionless number, usually between 0 and 1, 0 = completely inelastic collision response, 1 = completely elastic collision response). Note that bodies can have negative restitution but the combined restitution (see PhysicsSystem::SetCombineRestitution) should never go below zero.
	inline float			GetRestitution() const											{ return mRestitution; }
	void					SetRestitution(float inRestitution)								{ mRestitution = inRestitution; }

	// Get world space linear velocity of the center of mass (unit: m/s)
	inline Vec3				GetLinearVelocity() const										{ return !IsStatic()? mMotionProperties->GetLinearVelocity() : Vec3::sZero(); }

	// Set world space linear velocity of the center of mass (unit: m/s).
	// If you want the body to wake up when it is sleeping, use BodyInterface::SetLinearVelocity instead.
	void					SetLinearVelocity(Vec3Arg inLinearVelocity)						{ MOSS_ASSERT(!IsStatic()); mMotionProperties->SetLinearVelocity(inLinearVelocity); }

	// Set world space linear velocity of the center of mass, will make sure the value is clamped against the maximum linear velocity.
	// If you want the body to wake up when it is sleeping, use BodyInterface::SetLinearVelocity instead.
	void					SetLinearVelocityClamped(Vec3Arg inLinearVelocity)				{ MOSS_ASSERT(!IsStatic()); mMotionProperties->SetLinearVelocityClamped(inLinearVelocity); }

	// Get world space angular velocity of the center of mass (unit: rad/s)
	inline Vec3				GetAngularVelocity() const										{ return !IsStatic()? mMotionProperties->GetAngularVelocity() : Vec3::sZero(); }

	// Set world space angular velocity of the center of mass (unit: rad/s).
	// If you want the body to wake up when it is sleeping, use BodyInterface::SetAngularVelocity instead.
	void					SetAngularVelocity(Vec3Arg inAngularVelocity)					{ MOSS_ASSERT(!IsStatic()); mMotionProperties->SetAngularVelocity(inAngularVelocity); }

	// Set world space angular velocity of the center of mass, will make sure the value is clamped against the maximum angular velocity.
	// If you want the body to wake up when it is sleeping, use BodyInterface::SetAngularVelocity instead.
	void					SetAngularVelocityClamped(Vec3Arg inAngularVelocity)			{ MOSS_ASSERT(!IsStatic()); mMotionProperties->SetAngularVelocityClamped(inAngularVelocity); }

	// Velocity of point inPoint (in center of mass space, e.g. on the surface of the body) of the body (unit: m/s)
	inline Vec3				GetPointVelocityCOM(Vec3Arg inPointRelativeToCOM) const			{ return !IsStatic()? mMotionProperties->GetPointVelocityCOM(inPointRelativeToCOM) : Vec3::sZero(); }

	// Velocity of point inPoint (in world space, e.g. on the surface of the body) of the body (unit: m/s)
	inline Vec3				GetPointVelocity(RVec3Arg inPoint) const						{ MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sPositionAccess(), BodyAccess::EAccess::Read)); return GetPointVelocityCOM(Vec3(inPoint - mPosition)); }

	// Add force (unit: N) at center of mass for the next time step, will be reset after the next call to PhysicsSystem::Update.
	// If you want the body to wake up when it is sleeping, use BodyInterface::AddForce instead.
	inline void				AddForce(Vec3Arg inForce)										{ MOSS_ASSERT(IsDynamic()); (Vec3::sLoadFloat3Unsafe(mMotionProperties->mForce) + inForce).StoreFloat3(&mMotionProperties->mForce); }

	// Add force (unit: N) at inPosition for the next time step, will be reset after the next call to PhysicsSystem::Update.
	// If you want the body to wake up when it is sleeping, use BodyInterface::AddForce instead.
	inline void				AddForce(Vec3Arg inForce, RVec3Arg inPosition);

	// Add torque (unit: N m) for the next time step, will be reset after the next call to PhysicsSystem::Update.
	// If you want the body to wake up when it is sleeping, use BodyInterface::AddTorque instead.
	inline void				AddTorque(Vec3Arg inTorque)										{ MOSS_ASSERT(IsDynamic()); (Vec3::sLoadFloat3Unsafe(mMotionProperties->mTorque) + inTorque).StoreFloat3(&mMotionProperties->mTorque); }

	// Get the total amount of force applied to the center of mass this time step (through AddForce calls). Note that it will reset to zero after PhysicsSystem::Update.
	inline Vec3				GetAccumulatedForce() const										{ MOSS_ASSERT(IsDynamic()); return mMotionProperties->GetAccumulatedForce(); }

	// Get the total amount of torque applied to the center of mass this time step (through AddForce/AddTorque calls). Note that it will reset to zero after PhysicsSystem::Update.
	inline Vec3				GetAccumulatedTorque() const									{ MOSS_ASSERT(IsDynamic()); return mMotionProperties->GetAccumulatedTorque(); }

	// Reset the total accumulated force, not that this will be done automatically after every time step.
	MOSS_INLINE void			ResetForce()													{ MOSS_ASSERT(IsDynamic()); return mMotionProperties->ResetForce(); }

	// Reset the total accumulated torque, not that this will be done automatically after every time step.
	MOSS_INLINE void			ResetTorque()													{ MOSS_ASSERT(IsDynamic()); return mMotionProperties->ResetTorque(); }

	// Reset the current velocity and accumulated force and torque.
	MOSS_INLINE void			ResetMotion()													{ MOSS_ASSERT(!IsStatic()); return mMotionProperties->ResetMotion(); }

	// Get inverse inertia tensor in world space
	inline Mat44			GetInverseInertia() const;

	// Add impulse to center of mass (unit: kg m/s).
	// If you want the body to wake up when it is sleeping, use BodyInterface::AddImpulse instead.
	inline void				AddImpulse(Vec3Arg inImpulse);

	// Add impulse to point in world space (unit: kg m/s).
	// If you want the body to wake up when it is sleeping, use BodyInterface::AddImpulse instead.
	inline void				AddImpulse(Vec3Arg inImpulse, RVec3Arg inPosition);

	// Add angular impulse in world space (unit: N m s).
	// If you want the body to wake up when it is sleeping, use BodyInterface::AddAngularImpulse instead.
	inline void				AddAngularImpulse(Vec3Arg inAngularImpulse);

	// Set velocity of body such that it will be positioned at inTargetPosition/Rotation in inDeltaTime seconds.
	// If you want the body to wake up when it is sleeping, use BodyInterface::MoveKinematic instead.
	void					MoveKinematic(RVec3Arg inTargetPosition, QuatArg inTargetRotation, float inDeltaTime);

	// Gets the properties needed to do buoyancy calculations
	// @param inSurfacePosition Position of the fluid surface in world space
	// @param inSurfaceNormal Normal of the fluid surface (should point up)
	// @param outTotalVolume On return this contains the total volume of the shape
	// @param outSubmergedVolume On return this contains the submerged volume of the shape
	// @param outRelativeCenterOfBuoyancy On return this contains the center of mass of the submerged volume relative to the center of mass of the body
	void					GetSubmergedVolume(RVec3Arg inSurfacePosition, Vec3Arg inSurfaceNormal, float& outTotalVolume, float& outSubmergedVolume, Vec3& outRelativeCenterOfBuoyancy) const;

	// Applies an impulse to the body that simulates fluid buoyancy and drag.
	// If you want the body to wake up when it is sleeping, use BodyInterface::ApplyBuoyancyImpulse instead.
	// @param inSurfacePosition Position of the fluid surface in world space
	// @param inSurfaceNormal Normal of the fluid surface (should point up)
	// @param inBuoyancy The buoyancy factor for the body. 1 = neutral body, < 1 sinks, > 1 floats. Note that we don't use the fluid density since it is harder to configure than a simple number between [0, 2]
	// @param inLinearDrag Linear drag factor that slows down the body when in the fluid (approx. 0.5)
	// @param inAngularDrag Angular drag factor that slows down rotation when the body is in the fluid (approx. 0.01)
	// @param inFluidVelocity The average velocity of the fluid (in m/s) in which the body resides
	// @param inGravity The gravity vector (pointing down)
	// @param inDeltaTime Delta time of the next simulation step (in s)
	// @return true if an impulse was applied, false if the body was not in the fluid
	bool					ApplyBuoyancyImpulse(RVec3Arg inSurfacePosition, Vec3Arg inSurfaceNormal, float inBuoyancy, float inLinearDrag, float inAngularDrag, Vec3Arg inFluidVelocity, Vec3Arg inGravity, float inDeltaTime);

	// Applies an impulse to the body that simulates fluid buoyancy and drag.
	// If you want the body to wake up when it is sleeping, use BodyInterface::ApplyBuoyancyImpulse instead.
	// @param inTotalVolume Total volume of the shape of this body (m^3)
	// @param inSubmergedVolume Submerged volume of the shape of this body (m^3)
	// @param inRelativeCenterOfBuoyancy The center of mass of the submerged volume relative to the center of mass of the body
	// @param inBuoyancy The buoyancy factor for the body. 1 = neutral body, < 1 sinks, > 1 floats. Note that we don't use the fluid density since it is harder to configure than a simple number between [0, 2]
	// @param inLinearDrag Linear drag factor that slows down the body when in the fluid (approx. 0.5)
	// @param inAngularDrag Angular drag factor that slows down rotation when the body is in the fluid (approx. 0.01)
	// @param inFluidVelocity The average velocity of the fluid (in m/s) in which the body resides
	// @param inGravity The gravity vector (pointing down)
	// @param inDeltaTime Delta time of the next simulation step (in s)
	// @return true if an impulse was applied, false if the body was not in the fluid
	bool					ApplyBuoyancyImpulse(float inTotalVolume, float inSubmergedVolume, Vec3Arg inRelativeCenterOfBuoyancy, float inBuoyancy, float inLinearDrag, float inAngularDrag, Vec3Arg inFluidVelocity, Vec3Arg inGravity, float inDeltaTime);

	// Check if this body has been added to the physics system
	inline bool				IsInBroadPhase() const											{ return (mFlags.load(memory_order_relaxed)&  uint8(EFlags::IsInBroadPhase)) != 0; }

	// Check if this body has been changed in such a way that the collision cache should be considered invalid for any body interacting with this body
	inline bool				IsCollisionCacheInvalid() const									{ return (mFlags.load(memory_order_relaxed)&  uint8(EFlags::InvalidateContactCache)) != 0; }

	// Get the shape of this body
	inline const Shape*	GetShape() const												{ return mShape; }

	// World space position of the body
	inline RVec3			GetPosition() const												{ MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sPositionAccess(), BodyAccess::EAccess::Read)); return mPosition - mRotation* mShape->GetCenterOfMass(); }

	// World space rotation of the body
	inline Quat				GetRotation() const												{ MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sPositionAccess(), BodyAccess::EAccess::Read)); return mRotation; }

	// Calculates the transform of this body
	inline RMat44			GetWorldTransform() const;

	// Gets the world space position of this body's center of mass
	inline RVec3			GetCenterOfMassPosition() const									{ MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sPositionAccess(), BodyAccess::EAccess::Read)); return mPosition; }

	// Calculates the transform for this body's center of mass
	inline RMat44			GetCenterOfMassTransform() const;

	// Calculates the inverse of the transform for this body's center of mass
	inline RMat44			GetInverseCenterOfMassTransform() const;

	// Get world space bounding box
	inline const AABox& 	GetWorldSpaceBounds() const										{ return mBounds; }

#ifdef MOSS_DEBUG
	// Validate that the cached bounding box of the body matches the actual bounding box of the body.
	// If this check fails then there are a number of possible causes:
	// 1. Shape is being modified without notifying the system of the change. E.g. if you modify a MutableCompoundShape
	// without calling BodyInterface::NotifyShapeChanged then there will be a mismatch between the cached bounding box
	// in the broad phase and the bounding box of the Shape.
	// 2. You are calling functions postfixed with 'Internal' which are not meant to be called by the application.
	// 3. If the actual bounds and cached bounds are very close, it could mean that you have a mismatch in floating
	// point unit state between threads. E.g. one thread has flush to zero (FTZ) or denormals are zero (DAZ) set and
	// the other thread does not. Or if the rounding mode differs between threads. This can cause small differences
	// in floating point calculations. If you are using JobSystemThreadPool you can use JobSystemThreadPool::SetThreadInitFunction
	// to initialize the floating point unit state.
	inline void	ValidateCachedBounds() const
	{
		AABox actual_body_bounds = mShape->GetWorldSpaceBounds(GetCenterOfMassTransform(), Vec3::sOne());
		MOSS_ASSERT(actual_body_bounds == mBounds, "Mismatch between cached bounding box and actual bounding box");
	}
#endif // MOSS_DEBUG

	// Access to the motion properties
	const MotionProperties*GetMotionProperties() const										{ MOSS_ASSERT(!IsStatic()); return mMotionProperties; }
	MotionProperties*		GetMotionProperties()											{ MOSS_ASSERT(!IsStatic()); return mMotionProperties; }

	// Access to the motion properties (version that does not check if the object is kinematic or dynamic)
	const MotionProperties*GetMotionPropertiesUnchecked() const							{ return mMotionProperties; }
	MotionProperties*		GetMotionPropertiesUnchecked()									{ return mMotionProperties; }

	// Access to the user data, can be used for anything by the application
	uint64					GetUserData() const												{ return mUserData; }
	void					SetUserData(uint64 inUserData)									{ mUserData = inUserData; }

	// Get surface normal of a particular sub shape and its world space surface position on this body
	inline Vec3				GetWorldSpaceSurfaceNormal(const SubShapeID& inSubShapeID, RVec3Arg inPosition) const;

	// Get the transformed shape of this body, which can be used to do collision detection outside of a body lock
	inline TransformedShape	GetTransformedShape() const										{ MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sPositionAccess(), BodyAccess::EAccess::Read)); return TransformedShape(mPosition, mRotation, mShape, mID); }

	// Debug function to convert a body back to a body creation settings object to be able to save/recreate the body later
	BodyCreationSettings	GetBodyCreationSettings() const;

	// Debug function to convert a soft body back to a soft body creation settings object to be able to save/recreate the body later
	SoftBodyCreationSettings GetSoftBodyCreationSettings() const;

	// A dummy body that can be used by constraints to attach a constraint to the world instead of another body
	static Body				sFixedToWorld;

	//@name THESE FUNCTIONS ARE FOR INTERNAL USE ONLY AND SHOULD NOT BE CALLED BY THE APPLICATION
	//@{

	// Helper function for BroadPhase::FindCollidingPairs that returns true when two bodies can collide
	// It assumes that body 1 is dynamic and active and guarantees that it body 1 collides with body 2 that body 2 will not collide with body 1 in order to avoid finding duplicate collision pairs
	static inline bool		sFindCollidingPairsCanCollide(const Body& inBody1, const Body& inBody2);

	// Update position using an Euler step (used during position integrate&  constraint solving)
	inline void				AddPositionStep(Vec3Arg inLinearVelocityTimesDeltaTime)			{ MOSS_ASSERT(IsRigidBody()); MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sPositionAccess(), BodyAccess::EAccess::ReadWrite)); mPosition += mMotionProperties->LockTranslation(inLinearVelocityTimesDeltaTime); MOSS_ASSERT(!mPosition.IsNaN()); }
	inline void				SubPositionStep(Vec3Arg inLinearVelocityTimesDeltaTime)			{ MOSS_ASSERT(IsRigidBody()); MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sPositionAccess(), BodyAccess::EAccess::ReadWrite)); mPosition -= mMotionProperties->LockTranslation(inLinearVelocityTimesDeltaTime); MOSS_ASSERT(!mPosition.IsNaN()); }

	// Update rotation using an Euler step (used during position integrate&  constraint solving)
	inline void				AddRotationStep(Vec3Arg inAngularVelocityTimesDeltaTime);
	inline void				SubRotationStep(Vec3Arg inAngularVelocityTimesDeltaTime);

	// Flag if body is in the broadphase (should only be called by the BroadPhase)
	inline void				SetInBroadPhaseInternal(bool inInBroadPhase)					{ if (inInBroadPhase) mFlags.fetch_or(uint8(EFlags::IsInBroadPhase), memory_order_relaxed); else mFlags.fetch_and(uint8(~uint8(EFlags::IsInBroadPhase)), memory_order_relaxed); }

	// Invalidate the contact cache (should only be called by the BodyManager), will be reset the next simulation step. Returns true if the contact cache was still valid.
	inline bool				InvalidateContactCacheInternal()								{ return (mFlags.fetch_or(uint8(EFlags::InvalidateContactCache), memory_order_relaxed)&  uint8(EFlags::InvalidateContactCache)) == 0; }

	// Reset the collision cache invalid flag (should only be called by the BodyManager).
	inline void				ValidateContactCacheInternal()									{ MOSS_IF_ENABLE_ASSERTS(uint8 old_val = ) mFlags.fetch_and(uint8(~uint8(EFlags::InvalidateContactCache)), memory_order_relaxed); MOSS_ASSERT((old_val&  uint8(EFlags::InvalidateContactCache)) != 0); }

	// Updates world space bounding box (should only be called by the PhysicsSystem)
	void					CalculateWorldSpaceBoundsInternal();

	// Function to update body's position (should only be called by the BodyInterface since it also requires updating the broadphase)
	void					SetPositionAndRotationInternal(RVec3Arg inPosition, QuatArg inRotation, bool inResetSleepTimer = true);

	// Updates the center of mass and optionally mass properties after shifting the center of mass or changes to the shape (should only be called by the BodyInterface since it also requires updating the broadphase)
	// @param inPreviousCenterOfMass Center of mass of the shape before the alterations
	// @param inUpdateMassProperties When true, the mass and inertia tensor is recalculated
	void					UpdateCenterOfMassInternal(Vec3Arg inPreviousCenterOfMass, bool inUpdateMassProperties);

	// Function to update a body's shape (should only be called by the BodyInterface since it also requires updating the broadphase)
	// @param inShape The new shape for this body
	// @param inUpdateMassProperties When true, the mass and inertia tensor is recalculated
	void					SetShapeInternal(const Shape*inShape, bool inUpdateMassProperties);

	// TSAN detects a race between BodyManager::AddBodyToActiveBodies coming from PhysicsSystem::ProcessBodyPair and Body::GetIndexInActiveBodiesInternal coming from PhysicsSystem::ProcessBodyPair.
	// When PhysicsSystem::ProcessBodyPair activates a body, it updates mIndexInActiveBodies and then updates BodyManager::mNumActiveBodies with release semantics. PhysicsSystem::ProcessBodyPair will
	// then finish its loop of active bodies and at the end of the loop it will read BodyManager::mNumActiveBodies with acquire semantics to see if any bodies were activated during the loop.
	// This means that changes to mIndexInActiveBodies must be visible to the thread, so TSANs report must be a false positive. We suppress the warning here.
	MOSS_TSAN_NO_SANITIZE
	// Access to the index in the BodyManager::mActiveBodies list
	uint32					GetIndexInActiveBodiesInternal() const							{ return mMotionProperties != nullptr? mMotionProperties->mIndexInActiveBodies : cInactiveIndex; }

	// Update eligibility for sleeping
	ECanSleep				UpdateSleepStateInternal(float inDeltaTime, float inMaxMovement, float inTimeBeforeSleep);

	// Saving state for replay
	void					SaveState(StateRecorder& inStream) const;

	// Restoring state for replay
	void					RestoreState(StateRecorder& inStream);

	//@}

	static constexpr uint32	cInactiveIndex = MotionProperties::cInactiveIndex;				// Constant indicating that body is not active

private:
	friend class BodyManager;
	friend class BodyWithMotionProperties;
	friend class SoftBodyWithMotionPropertiesAndShape;

							Body() = default;												// Bodies must be created through BodyInterface::CreateBody

	explicit				Body(bool);														// Alternative constructor that initializes all members

							~Body()															{ MOSS_ASSERT(mMotionProperties == nullptr); } // Bodies must be destroyed through BodyInterface::DestroyBody

	inline void				GetSleepTestPoints(RVec3*outPoints) const;						// Determine points to test for checking if body is sleeping: COM, COM + largest bounding box axis, COM + second largest bounding box axis

	enum class EFlags : uint8
	{
		IsSensor						= 1 << 0,											// If this object is a sensor. A sensor will receive collision callbacks, but will not cause any collision responses and can be used as a trigger volume.
		CollideKinematicVsNonDynamic	= 1 << 1,											// If kinematic objects can generate contact points against other kinematic or static objects.
		IsInBroadPhase					= 1 << 2,											// Set this bit to indicate that the body is in the broadphase
		InvalidateContactCache			= 1 << 3,											// Set this bit to indicate that all collision caches for this body are invalid, will be reset the next simulation step.
		UseManifoldReduction			= 1 << 4,											// Set this bit to indicate that this body can use manifold reduction (if PhysicsSettings::mUseManifoldReduction is true)
		ApplyGyroscopicForce			= 1 << 5,											// Set this bit to indicate that the gyroscopic force should be applied to this body (aka Dzhanibekov effect, see https://en.wikipedia.org/wiki/Tennis_racket_theorem)
		EnhancedInternalEdgeRemoval		= 1 << 6,											// Set this bit to indicate that enhanced internal edge removal should be used for this body (see BodyCreationSettings::mEnhancedInternalEdgeRemoval)
	};

	// 16 byte aligned
	RVec3					mPosition;														// World space position of center of mass
	Quat					mRotation;														// World space rotation of center of mass
	AABox					mBounds;														// World space bounding box of the body

	// 8 byte aligned
	RefConst<Shape>			mShape;															// Shape representing the volume of this body
	MotionProperties*		mMotionProperties = nullptr;									// If this is a keyframed or dynamic object, this object holds all information about the movement
	uint64					mUserData = 0;													// User data, can be used for anything by the application
	CollisionGroup			mCollisionGroup;												// The collision group this body belongs to (determines if two objects can collide)

	// 4 byte aligned
	float					mFriction;														// Friction of the body (dimensionless number, usually between 0 and 1, 0 = no friction, 1 = friction force equals force that presses the two bodies together). Note that bodies can have negative friction but the combined friction (see PhysicsSystem::SetCombineFriction) should never go below zero.
	float					mRestitution;													// Restitution of body (dimensionless number, usually between 0 and 1, 0 = completely inelastic collision response, 1 = completely elastic collision response). Note that bodies can have negative restitution but the combined restitution (see PhysicsSystem::SetCombineRestitution) should never go below zero.
	BodyID					mID;															// ID of the body (index in the bodies array)

	// 2 or 4 bytes aligned
	ObjectLayer				mObjectLayer;													// The collision layer this body belongs to (determines if two objects can collide)

	// 1 byte aligned
	EBodyType				mBodyType;														// Type of body (rigid or soft)
	BroadPhaseLayer			mBroadPhaseLayer;												// The broad phase layer this body belongs to
	EMotionType				mMotionType;													// Type of motion (static, dynamic or kinematic)
	atomic<uint8>			mFlags = 0;														// See EFlags for possible flags

	// 122 bytes up to here (64-bit mode, single precision, 16-bit ObjectLayer)
};

/* BroadPhaseLayerFilter_Procs*/
typedef struct BroadPhaseLayerFilter_Procs {
	bool(API_CALL* ShouldCollide)(void* userData, BroadPhaseLayer layer);
} BroadPhaseLayerFilter_Procs;

MOSS_API void BroadPhaseLayerFilter_SetProcs(const BroadPhaseLayerFilter_Procs* procs);
MOSS_API BroadPhaseLayerFilter* BroadPhaseLayerFilter_Create(void* userData);
MOSS_API void BroadPhaseLayerFilter_Destroy(BroadPhaseLayerFilter* filter);

/* ObjectLayerFilter*/
typedef struct ObjectLayerFilter_Procs {
	bool(API_CALL* ShouldCollide)(void* userData, ObjectLayer layer);
} ObjectLayerFilter_Procs;

MOSS_API void ObjectLayerFilter_SetProcs(const ObjectLayerFilter_Procs* procs);
MOSS_API ObjectLayerFilter* ObjectLayerFilter_Create(void* userData);
MOSS_API void ObjectLayerFilter_Destroy(ObjectLayerFilter* filter);

/* BodyFilter*/
class MOSS_API BodyFilter : public NonCopyable {
public:
	// Destructor
	virtual					~BodyFilter() = default;

	// Filter function. Returns true if we should collide with inBodyID
	virtual bool			ShouldCollide([[maybe_unused]] const BodyID& inBodyID) const { return true; }

	// Filter function. Returns true if we should collide with inBody (this is called after the body is locked and makes it possible to filter based on body members)
	virtual bool			ShouldCollideLocked([[maybe_unused]] const Body& inBody) const { return true; }
};

/* ShapeFilter*/
typedef struct ShapeFilter_Procs {
	bool(API_CALL* ShouldCollide)(void* userData, const Shape* shape2, const SubShapeID* subShapeIDOfShape2);
	bool(API_CALL* ShouldCollide2)(void* userData, const Shape* shape1, const SubShapeID* subShapeIDOfShape1, const Shape* shape2, const SubShapeID* subShapeIDOfShape2);
} ShapeFilter_Procs;

MOSS_API void ShapeFilter_SetProcs(const ShapeFilter_Procs* procs);
MOSS_API ShapeFilter* ShapeFilter_Create(void* userData);
MOSS_API void ShapeFilter_Destroy(ShapeFilter* filter);
MOSS_API BodyID ShapeFilter_GetBodyID2(ShapeFilter* filter);
MOSS_API void ShapeFilter_SetBodyID2(ShapeFilter* filter, BodyID id);

/* SimShapeFilter*/
typedef struct SimShapeFilter_Procs {
	bool(API_CALL* ShouldCollide)(void* userData, 
		const Body* body1, 
		const Shape* shape1, 
		const SubShapeID* subShapeIDOfShape1,
		const Body* body2,
		const Shape* shape2, 
		const SubShapeID* subShapeIDOfShape2
		);
} SimShapeFilter_Procs;

MOSS_API void SimShapeFilter_SetProcs(const SimShapeFilter_Procs* procs);
MOSS_API SimShapeFilter* SimShapeFilter_Create(void* userData);
MOSS_API void SimShapeFilter_Destroy(SimShapeFilter* filter);

/* Contact listener*/
typedef struct ContactListener_Procs {
	ValidateResult(API_CALL* OnContactValidate)(void* userData,
		const Body* body1,
		const Body* body2,
		const Vec3* baseOffset,
		const CollideShapeResult* collisionResult);

	void(API_CALL* OnContactAdded)(void* userData,
		const Body* body1,
		const Body* body2,
		const ContactManifold* manifold,
		ContactSettings* settings);

	void(API_CALL* OnContactPersisted)(void* userData,
		const Body* body1,
		const Body* body2,
		const ContactManifold* manifold,
		ContactSettings* settings);

	void(API_CALL* OnContactRemoved)(void* userData,
		const SubShapeIDPair* subShapePair
		);
} ContactListener_Procs;

MOSS_API void ContactListener_SetProcs(const ContactListener_Procs* procs);
MOSS_API ContactListener* ContactListener_Create(void* userData);
MOSS_API void ContactListener_Destroy(ContactListener* listener);

/* BodyActivationListener*/
class BodyActivationListener {
public:
	// Ensure virtual destructor
	virtual					~BodyActivationListener() = default;

	// Called whenever a body activates, note this can be called from any thread so make sure your code is thread safe.
	// At the time of the callback the body inBodyID will be locked and no bodies can be written/activated/deactivated from the callback.
	virtual void			OnBodyActivated(const BodyID& inBodyID, uint64 inBodyUserData) = 0;

	// Called whenever a body deactivates, note this can be called from any thread so make sure your code is thread safe.
	// At the time of the callback the body inBodyID will be locked and no bodies can be written/activated/deactivated from the callback.
	virtual void			OnBodyDeactivated(const BodyID& inBodyID, uint64 inBodyUserData) = 0;
};

/* BodyDrawFilter*/
typedef struct BodyDrawFilter_Procs {
	bool(API_CALL* ShouldDraw)(void* userData, const Body* body);
} BodyDrawFilter_Procs;

MOSS_API void BodyDrawFilter_SetProcs(const BodyDrawFilter_Procs* procs);
MOSS_API BodyDrawFilter* BodyDrawFilter_Create(void* userData);
MOSS_API void BodyDrawFilter_Destroy(BodyDrawFilter* filter);

/* ContactManifold*/
MOSS_API void ContactManifold_GetWorldSpaceNormal(const ContactManifold* manifold, Vec3* result);
MOSS_API float ContactManifold_GetPenetrationDepth(const ContactManifold* manifold);
MOSS_API SubShapeID ContactManifold_GetSubShapeID1(const ContactManifold* manifold);
MOSS_API SubShapeID ContactManifold_GetSubShapeID2(const ContactManifold* manifold);
MOSS_API uint32_t ContactManifold_GetPointCount(const ContactManifold* manifold);
MOSS_API void ContactManifold_GetWorldSpaceContactPointOn1(const ContactManifold* manifold, uint32_t index, Vec3* result);
MOSS_API void ContactManifold_GetWorldSpaceContactPointOn2(const ContactManifold* manifold, uint32_t index, Vec3* result);

/* CharacterBase*/
class MOSS_API CharacterBase : public RefTarget<CharacterBase>, public NonCopyable {
public:
	MOSS_OVERRIDE_NEW_DELETE

	CharacterBase(const CharacterBaseSettings*inSettings, PhysicsSystem*inSystem);
	virtual	~CharacterBase() = default;

	// Set the maximum angle of slope that character can still walk on (radians)
	void SetMaxSlopeAngle(float inMaxSlopeAngle) { mCosMaxSlopeAngle = Cos(inMaxSlopeAngle); }
	float GetCosMaxSlopeAngle() const { return mCosMaxSlopeAngle; }

	// Set the up vector for the character
	void SetUp(Vec3Arg inUp) { mUp = inUp; }
	Vec3 GetUp() const { return mUp; }

	// Check if the normal of the ground surface is too steep to walk on
	bool IsSlopeTooSteep(Vec3Arg inNormal) const
	{
		// If cos max slope angle is close to one the system is turned off,
		// otherwise check the angle between the up and normal vector
		return mCosMaxSlopeAngle < cNoMaxSlopeAngle& & inNormal.Dot(mUp) < mCosMaxSlopeAngle;
	}

	// Get the current shape that the character is using.
	const Shape* GetShape() const { return mShape; }

	enum class EGroundState
	{
		OnGround,						// Character is on the ground and can move freely.
		OnSteepGround,					// Character is on a slope that is too steep and can't climb up any further. The caller should start applying downward velocity if sliding from the slope is desired.
		NotSupported,					// Character is touching an object, but is not supported by it and should fall. The GetGroundXXX functions will return information about the touched object.
		InAir,							// Character is in the air and is not touching anything.
	};

	// Debug function to convert enum values to string
	static const char* sToString(EGroundState inState);

	//@name Properties of the ground this character is standing on

	// Current ground state
	EGroundState GetGroundState() const { return mGroundState; }

	// Returns true if the player is supported by normal or steep ground
	bool IsSupported() const { return mGroundState == EGroundState::OnGround || mGroundState == EGroundState::OnSteepGround; }

	// Get the contact point with the ground
	RVec3 GetGroundPosition() const	{ return mGroundPosition; }

	// Get the contact normal with the ground
	Vec3 GetGroundNormal() const { return mGroundNormal; }

	// Velocity in world space of ground
	Vec3 GetGroundVelocity() const { return mGroundVelocity; }

	// Material that the character is standing on
	const PhysicsMaterial* GetGroundMaterial() const { return mGroundMaterial; }

	// BodyID of the object the character is standing on. Note may have been removed!
	BodyID GetGroundBodyID() const { return mGroundBodyID; }

	// Sub part of the body that we're standing on.
	SubShapeID GetGroundSubShapeID() const { return mGroundBodySubShapeID; }

	// User data value of the body that we're standing on
	uint64 GetGroundUserData() const { return mGroundUserData; }

	// Saving / restoring state for replay
	virtual void						SaveState(StateRecorder& inStream) const;
	virtual void						RestoreState(StateRecorder& inStream);

protected:
	// Cached physics system
	PhysicsSystem* mSystem;

	// The shape that the body currently has
	RefConst<Shape>	mShape;

	// The character's world space up axis
	Vec3 mUp;

	// Every contact behind this plane can support the character
	Plane mSupportingVolume;

	// Beyond this value there is no max slope
	static constexpr float cNoMaxSlopeAngle = 0.9999f;

	// Cosine of the maximum angle of slope that character can still walk on
	float mCosMaxSlopeAngle;

	// Ground properties
	EGroundState						mGroundState = EGroundState::InAir;
	BodyID								mGroundBodyID;
	SubShapeID							mGroundBodySubShapeID;
	RVec3								mGroundPosition = RVec3::sZero();
	Vec3								mGroundNormal = Vec3::sZero();
	Vec3								mGroundVelocity = Vec3::sZero();
	RefConst<PhysicsMaterial>			mGroundMaterial = PhysicsMaterial::sDefault;
	uint64								mGroundUserData = 0;
};


/* Character*/
// Runtime character object.
// This object usually represents the player or a humanoid AI. It uses a single rigid body,
// usually with a capsule shape to simulate movement and collision for the character.
// The character is a keyframed object, the application controls it by setting the velocity.
class MOSS_API Character : public CharacterBase {
public:
	MOSS_OVERRIDE_NEW_DELETE

	// @param inSettings The settings for the character
	// @param inPosition Initial position for the character
	// @param inRotation Initial rotation for the character (usually only around Y)
	// @param inUserData Application specific value
	// @param inSystem Physics system that this character will be added to later
	Character(const CharacterSettings*inSettings, RVec3Arg inPosition, QuatArg inRotation, uint64 inUserData, PhysicsSystem*inSystem);
	virtual	~Character() override;

	// Add bodies and constraints to the system and optionally activate the bodies
	void AddToPhysicsSystem(EActivation inActivationMode = EActivation::Activate, bool inLockBodies = true);

	// Remove bodies and constraints from the system
	void RemoveFromPhysicsSystem(bool inLockBodies = true);

	// Wake up the character
	void Activate(bool inLockBodies = true);

	// Needs to be called after every PhysicsSystem::Update
	// @param inMaxSeparationDistance Max distance between the floor and the character to still consider the character standing on the floor
	// @param inLockBodies If the collision query should use the locking body interface (true) or the non locking body interface (false)
	void PostSimulation(float inMaxSeparationDistance, bool inLockBodies = true);

	// Control the velocity of the character
	void SetLinearAndAngularVelocity(Vec3Arg inLinearVelocity, Vec3Arg inAngularVelocity, bool inLockBodies = true);

	// Get the linear velocity of the character (m / s)
	Vec3 GetLinearVelocity(bool inLockBodies = true) const;

	// Set the linear velocity of the character (m / s)
	void SetLinearVelocity(Vec3Arg inLinearVelocity, bool inLockBodies = true);

	// Add world space linear velocity to current velocity (m / s)
	void AddLinearVelocity(Vec3Arg inLinearVelocity, bool inLockBodies = true);

	// Add impulse to the center of mass of the character
	void AddImpulse(Vec3Arg inImpulse, bool inLockBodies = true);

	// Get the body associated with this character
	BodyID GetBodyID() const										{ return mBodyID; }

	// Get position / rotation of the body
	void GetPositionAndRotation(RVec3& outPosition, Quat& outRotation, bool inLockBodies = true) const;

	// Set the position / rotation of the body, optionally activating it.
	void SetPositionAndRotation(RVec3Arg inPosition, QuatArg inRotation, EActivation inActivationMode = EActivation::Activate, bool inLockBodies = true) const;

	// Get the position of the character
	RVec3 GetPosition(bool inLockBodies = true) const;

	// Set the position of the character, optionally activating it.
	void SetPosition(RVec3Arg inPosition, EActivation inActivationMode = EActivation::Activate, bool inLockBodies = true);

	// Get the rotation of the character
	Quat GetRotation(bool inLockBodies = true) const;

	// Set the rotation of the character, optionally activating it.
	void SetRotation(QuatArg inRotation, EActivation inActivationMode = EActivation::Activate, bool inLockBodies = true);

	// Position of the center of mass of the underlying rigid body
	RVec3 GetCenterOfMassPosition(bool inLockBodies = true) const;

	// Calculate the world transform of the character
	RMat44 GetWorldTransform(bool inLockBodies = true) const;

	// Get the layer of the character
	ObjectLayer	GetLayer() const										{ return mLayer; }

	// Update the layer of the character
	void SetLayer(ObjectLayer inLayer, bool inLockBodies = true);

	// Switch the shape of the character (e.g. for stance). When inMaxPenetrationDepth is not FLT_MAX, it checks
	// if the new shape collides before switching shape. Returns true if the switch succeeded.
	bool SetShape(const Shape*inShape, float inMaxPenetrationDepth, bool inLockBodies = true);

	// Get the transformed shape that represents the volume of the character, can be used for collision checks.
	TransformedShape GetTransformedShape(bool inLockBodies = true) const;

	// @brief Get all contacts for the character at a particular location
	// @param inPosition Position to test.
	// @param inRotation Rotation at which to test the shape.
	// @param inMovementDirection A hint in which direction the character is moving, will be used to calculate a proper normal.
	// @param inMaxSeparationDistance How much distance around the character you want to report contacts in (can be 0 to match the character exactly).
	// @param inShape Shape to test collision with.
	// @param inBaseOffset All hit results will be returned relative to this offset, can be zero to get results in world position, but when you're testing far from the origin you get better precision by picking a position that's closer e.g. GetPosition() since floats are most accurate near the origin
	// @param ioCollector Collision collector that receives the collision results.
	// @param inLockBodies If the collision query should use the locking body interface (true) or the non locking body interface (false)
	void CheckCollision(RVec3Arg inPosition, QuatArg inRotation, Vec3Arg inMovementDirection, float inMaxSeparationDistance, const Shape*inShape, RVec3Arg inBaseOffset, CollideShapeCollector& ioCollector, bool inLockBodies = true) const;

private:
	// Check collisions between inShape and the world using the center of mass transform
	void CheckCollision(RMat44Arg inCenterOfMassTransform, Vec3Arg inMovementDirection, float inMaxSeparationDistance, const Shape*inShape, RVec3Arg inBaseOffset, CollideShapeCollector& ioCollector, bool inLockBodies) const;
	// Check collisions between inShape and the world using the current position / rotation of the character
	void CheckCollision(const Shape*inShape, float inMaxSeparationDistance, RVec3Arg inBaseOffset, CollideShapeCollector& ioCollector, bool inLockBodies) const;

	// The body of this character
	BodyID mBodyID;

	// The layer the body is in
	ObjectLayer	mLayer;
};

/* CharacterVirtualSettings*/
class MOSS_API CharacterVirtual : public CharacterBase {
public:
	MOSS_OVERRIDE_NEW_DELETE

	// @param inSettings The settings for the character
	// @param inPosition Initial position for the character
	// @param inRotation Initial rotation for the character (usually only around the up-axis)
	// @param inUserData Application specific value
	// @param inSystem Physics system that this character will be added to
	CharacterVirtual(const CharacterVirtualSettings*inSettings, RVec3Arg inPosition, QuatArg inRotation, uint64 inUserData, PhysicsSystem*inSystem);

	// Constructor without user data
	CharacterVirtual(const CharacterVirtualSettings*inSettings, RVec3Arg inPosition, QuatArg inRotation, PhysicsSystem*inSystem) : CharacterVirtual(inSettings, inPosition, inRotation, 0, inSystem) { }

	// Destructor
	virtual	~CharacterVirtual() override;

	// The ID of this character
	inline const CharacterID& GetID() const											{ return mID; }

	// Set the contact listener
	void SetListener(CharacterContactListener*inListener)		{ mListener = inListener; }

	// Get the current contact listener
	CharacterContactListener* GetListener() const										{ return mListener; }

	// Set the character vs character collision interface
	void SetCharacterVsCharacterCollision(CharacterVsCharacterCollision*inCharacterVsCharacterCollision) { mCharacterVsCharacterCollision = inCharacterVsCharacterCollision; }

	// Get the linear velocity of the character (m / s)
	Vec3 GetLinearVelocity() const								{ return mLinearVelocity; }

	// Set the linear velocity of the character (m / s)
	void SetLinearVelocity(Vec3Arg inLinearVelocity)				{ mLinearVelocity = inLinearVelocity; }

	// Get the position of the character
	RVec3 GetPosition() const										{ return mPosition; }

	// Set the position of the character
	void SetPosition(RVec3Arg inPosition)						{ mPosition = inPosition; UpdateInnerBodyTransform(); }

	// Get the rotation of the character
	Quat GetRotation() const										{ return mRotation; }

	// Set the rotation of the character
	void SetRotation(QuatArg inRotation)							{ mRotation = inRotation; UpdateInnerBodyTransform(); }

	// Get the center of mass position of the shape
	inline RVec3 GetCenterOfMassPosition() const							{ return mPosition + (mRotation* (mShapeOffset + mShape->GetCenterOfMass()) + mCharacterPadding* mUp); }

	// Calculate the world transform of the character
	RMat44 GetWorldTransform() const								{ return RMat44::sRotationTranslation(mRotation, mPosition); }

	// Calculates the transform for this character's center of mass
	RMat44 GetCenterOfMassTransform() const						{ return GetCenterOfMassTransform(mPosition, mRotation, mShape); }

	// Character mass (kg)
	float GetMass() const { return mMass; }
	void SetMass(float inMass) { mMass = inMass; }

	// Maximum force with which the character can push other bodies (N)
	float GetMaxStrength() const { return mMaxStrength; }
	void SetMaxStrength(float inMaxStrength) { mMaxStrength = inMaxStrength; }

	// This value governs how fast a penetration will be resolved, 0 = nothing is resolved, 1 = everything in one update
	float GetPenetrationRecoverySpeed() const { return mPenetrationRecoverySpeed; }
	void SetPenetrationRecoverySpeed(float inSpeed) { mPenetrationRecoverySpeed = inSpeed; }

	// Set to indicate that extra effort should be made to try to remove ghost contacts (collisions with internal edges of a mesh). This is more expensive but makes bodies move smoother over a mesh with convex edges.
	bool GetEnhancedInternalEdgeRemoval() const { return mEnhancedInternalEdgeRemoval; }
	void SetEnhancedInternalEdgeRemoval(bool inApply) { mEnhancedInternalEdgeRemoval = inApply; }

	// Character padding
	float GetCharacterPadding() const { return mCharacterPadding; }

	// Max num hits to collect in order to avoid excess of contact points collection
	uint8 GetMaxNumHits() const { return mMaxNumHits; }
	void SetMaxNumHits(uint8 inMaxHits) { mMaxNumHits = inMaxHits; }

	// Cos(angle) where angle is the maximum angle between two hits contact normals that are allowed to be merged during hit reduction. Default is around 2.5 degrees. Set to -1 to turn off.
	float GetHitReductionCosMaxAngle() const { return mHitReductionCosMaxAngle; }
	void SetHitReductionCosMaxAngle(float inCosMaxAngle) { mHitReductionCosMaxAngle = inCosMaxAngle; }

	// Returns if we exceeded the maximum number of hits during the last collision check and had to discard hits based on distance.
	// This can be used to find areas that have too complex geometry for the character to navigate properly.
	// To solve you can either increase the max number of hits or simplify the geometry. Note that the character simulation will
	// try to do its best to select the most relevant contacts to avoid the character from getting stuck.
	bool GetMaxHitsExceeded() const								{ return mMaxHitsExceeded; }

	// An extra offset applied to the shape in local space. This allows applying an extra offset to the shape in local space. Note that setting it on the fly can cause the shape to teleport into collision.
	Vec3 GetShapeOffset() const { return mShapeOffset; }
	void SetShapeOffset(Vec3Arg inShapeOffset) { mShapeOffset = inShapeOffset; UpdateInnerBodyTransform(); }

	// Access to the user data, can be used for anything by the application
	uint64 GetUserData() const { return mUserData; }
	void SetUserData(uint64 inUserData);

	// Optional inner rigid body that proxies the character in the world. Can be used to update body properties.
	BodyID GetInnerBodyID() const { return mInnerBodyID; }

	// This function can be called prior to calling Update() to convert a desired velocity into a velocity that won't make the character move further onto steep slopes.
	// This velocity can then be set on the character using SetLinearVelocity()
	// @param inDesiredVelocity Velocity to clamp against steep walls
	// @return A new velocity vector that won't make the character move up steep slopes
	Vec3 CancelVelocityTowardsSteepSlopes(Vec3Arg inDesiredVelocity) const;

	// This function is internally called by Update, WalkStairs, StickToFloor and ExtendedUpdate and is responsible for tracking if contacts are added, persisted or removed.
	// If you want to do multiple operations on a character (e.g. first Update then WalkStairs), you can surround the code with a StartTrackingContactChanges and FinishTrackingContactChanges pair
	// to only receive a single callback per contact on the CharacterContactListener. If you don't do this then you could for example receive a contact added callback during the Update and a
	// contact persisted callback during WalkStairs.
	void StartTrackingContactChanges();

	// This call triggers contact removal callbacks and is used in conjunction with StartTrackingContactChanges.
	void FinishTrackingContactChanges();

	// This is the main update function. It moves the character according to its current velocity (the character is similar to a kinematic body in the sense
	// that you set the velocity and the character will follow unless collision is blocking the way). Note it's your own responsibility to apply gravity to the character velocity!
	// Different surface materials (like ice) can be emulated by getting the ground material and adjusting the velocity and/or the max slope angle accordingly every frame.
	// @param inDeltaTime Time step to simulate.
	// @param inGravity Gravity vector (m/s^2). This gravity vector is only used when the character is standing on top of another object to apply downward force.
	// @param inBroadPhaseLayerFilter Filter that is used to check if the character collides with something in the broadphase.
	// @param inObjectLayerFilter Filter that is used to check if a character collides with a layer.
	// @param inBodyFilter Filter that is used to check if a character collides with a body.
	// @param inShapeFilter Filter that is used to check if a character collides with a subshape.
	// @param inAllocator An allocator for temporary allocations. All memory will be freed by the time this function returns.
	void Update(float inDeltaTime, Vec3Arg inGravity, const BroadPhaseLayerFilter& inBroadPhaseLayerFilter, const ObjectLayerFilter& inObjectLayerFilter, const BodyFilter& inBodyFilter, const ShapeFilter& inShapeFilter, TempAllocator& inAllocator);

	// This function will return true if the character has moved into a slope that is too steep (e.g. a vertical wall).
	// You would call WalkStairs to attempt to step up stairs.
	// @param inLinearVelocity The linear velocity that the player desired. This is used to determine if we're pushing into a step.
	bool CanWalkStairs(Vec3Arg inLinearVelocity) const;

	// When stair walking is needed, you can call the WalkStairs function to cast up, forward and down again to try to find a valid position
	// @param inDeltaTime Time step to simulate.
	// @param inStepUp The direction and distance to step up (this corresponds to the max step height)
	// @param inStepForward The direction and distance to step forward after the step up
	// @param inStepForwardTest When running at a high frequency, inStepForward can be very small and it's likely that you hit the side of the stairs on the way down. This could produce a normal that violates the max slope angle. If this happens, we test again using this distance from the up position to see if we find a valid slope.
	// @param inStepDownExtra An additional translation that is added when stepping down at the end. Allows you to step further down than up. Set to zero if you don't want this. Should be in the opposite direction of up.
	// @param inBroadPhaseLayerFilter Filter that is used to check if the character collides with something in the broadphase.
	// @param inObjectLayerFilter Filter that is used to check if a character collides with a layer.
	// @param inBodyFilter Filter that is used to check if a character collides with a body.
	// @param inShapeFilter Filter that is used to check if a character collides with a subshape.
	// @param inAllocator An allocator for temporary allocations. All memory will be freed by the time this function returns.
	// @return true if the stair walk was successful
	bool WalkStairs(float inDeltaTime, Vec3Arg inStepUp, Vec3Arg inStepForward, Vec3Arg inStepForwardTest, Vec3Arg inStepDownExtra, const BroadPhaseLayerFilter& inBroadPhaseLayerFilter, const ObjectLayerFilter& inObjectLayerFilter, const BodyFilter& inBodyFilter, const ShapeFilter& inShapeFilter, TempAllocator& inAllocator);

	// This function can be used to artificially keep the character to the floor. Normally when a character is on a small step and starts moving horizontally, the character will
	// lose contact with the floor because the initial vertical velocity is zero while the horizontal velocity is quite high. To prevent the character from losing contact with the floor,
	// we do an additional collision check downwards and if we find the floor within a certain distance, we project the character onto the floor.
	// @param inStepDown Max amount to project the character downwards (if no floor is found within this distance, the function will return false)
	// @param inBroadPhaseLayerFilter Filter that is used to check if the character collides with something in the broadphase.
	// @param inObjectLayerFilter Filter that is used to check if a character collides with a layer.
	// @param inBodyFilter Filter that is used to check if a character collides with a body.
	// @param inShapeFilter Filter that is used to check if a character collides with a subshape.
	// @param inAllocator An allocator for temporary allocations. All memory will be freed by the time this function returns.
	// @return True if the character was successfully projected onto the floor.
	bool StickToFloor(Vec3Arg inStepDown, const BroadPhaseLayerFilter& inBroadPhaseLayerFilter, const ObjectLayerFilter& inObjectLayerFilter, const BodyFilter& inBodyFilter, const ShapeFilter& inShapeFilter, TempAllocator& inAllocator);

	// Settings struct with settings for ExtendedUpdate
	struct ExtendedUpdateSettings
	{
		Vec3	mStickToFloorStepDown { 0, -0.5f, 0 };									// See StickToFloor inStepDown parameter. Can be zero to turn off.
		Vec3	mWalkStairsStepUp { 0, 0.4f, 0 };										// See WalkStairs inStepUp parameter. Can be zero to turn off.
		float	mWalkStairsMinStepForward { 0.02f };									// See WalkStairs inStepForward parameter. Note that the parameter only indicates a magnitude, direction is taken from current velocity.
		float	mWalkStairsStepForwardTest { 0.15f };									// See WalkStairs inStepForwardTest parameter. Note that the parameter only indicates a magnitude, direction is taken from current velocity.
		float	mWalkStairsCosAngleForwardContact { Cos(DegreesToRadians(75.0f)) };		// Cos(angle) where angle is the maximum angle between the ground normal in the horizontal plane and the character forward vector where we're willing to adjust the step forward test towards the contact normal.
		Vec3	mWalkStairsStepDownExtra { Vec3::sZero() };								// See WalkStairs inStepDownExtra
	};

	// This function combines Update, StickToFloor and WalkStairs. This function serves as an example of how these functions could be combined.
	// Before calling, call SetLinearVelocity to update the horizontal/vertical speed of the character, typically this is:
	// - When on OnGround and not moving away from ground: velocity = GetGroundVelocity() + horizontal speed as input by player + optional vertical jump velocity + delta time* gravity
	// - Else: velocity = current vertical velocity + horizontal speed as input by player + delta time* gravity
	// @param inDeltaTime Time step to simulate.
	// @param inGravity Gravity vector (m/s^2). This gravity vector is only used when the character is standing on top of another object to apply downward force.
	// @param inSettings A structure containing settings for the algorithm.
	// @param inBroadPhaseLayerFilter Filter that is used to check if the character collides with something in the broadphase.
	// @param inObjectLayerFilter Filter that is used to check if a character collides with a layer.
	// @param inBodyFilter Filter that is used to check if a character collides with a body.
	// @param inShapeFilter Filter that is used to check if a character collides with a subshape.
	// @param inAllocator An allocator for temporary allocations. All memory will be freed by the time this function returns.
	void ExtendedUpdate(float inDeltaTime, Vec3Arg inGravity, const ExtendedUpdateSettings& inSettings, const BroadPhaseLayerFilter& inBroadPhaseLayerFilter, const ObjectLayerFilter& inObjectLayerFilter, const BodyFilter& inBodyFilter, const ShapeFilter& inShapeFilter, TempAllocator& inAllocator);

	// This function can be used after a character has teleported to determine the new contacts with the world.
	void RefreshContacts(const BroadPhaseLayerFilter& inBroadPhaseLayerFilter, const ObjectLayerFilter& inObjectLayerFilter, const BodyFilter& inBodyFilter, const ShapeFilter& inShapeFilter, TempAllocator& inAllocator);

	// Use the ground body ID to get an updated estimate of the ground velocity. This function can be used if the ground body has moved / changed velocity and you want a new estimate of the ground velocity.
	// It will not perform collision detection, so is less accurate than RefreshContacts but a lot faster.
	void UpdateGroundVelocity();

	// Switch the shape of the character (e.g. for stance).
	// @param inShape The shape to switch to.
	// @param inMaxPenetrationDepth When inMaxPenetrationDepth is not FLT_MAX, it checks if the new shape collides before switching shape. This is the max penetration we're willing to accept after the switch.
	// @param inBroadPhaseLayerFilter Filter that is used to check if the character collides with something in the broadphase.
	// @param inObjectLayerFilter Filter that is used to check if a character collides with a layer.
	// @param inBodyFilter Filter that is used to check if a character collides with a body.
	// @param inShapeFilter Filter that is used to check if a character collides with a subshape.
	// @param inAllocator An allocator for temporary allocations. All memory will be freed by the time this function returns.
	// @return Returns true if the switch succeeded.
	bool SetShape(const Shape*inShape, float inMaxPenetrationDepth, const BroadPhaseLayerFilter& inBroadPhaseLayerFilter, const ObjectLayerFilter& inObjectLayerFilter, const BodyFilter& inBodyFilter, const ShapeFilter& inShapeFilter, TempAllocator& inAllocator);

	// Updates the shape of the inner rigid body. Should be called after a successful call to SetShape.
	void SetInnerBodyShape(const Shape*inShape);

	// Get the transformed shape that represents the volume of the character, can be used for collision checks.
	TransformedShape GetTransformedShape() const { return TransformedShape(GetCenterOfMassPosition(), mRotation, mShape, mInnerBodyID); }

	// @brief Get all contacts for the character at a particular location.
	// When colliding with another character virtual, this pointer will be provided through CollideShapeCollector::SetUserContext before adding a hit.
	// @param inPosition Position to test, note that this position will be corrected for the character padding.
	// @param inRotation Rotation at which to test the shape.
	// @param inMovementDirection A hint in which direction the character is moving, will be used to calculate a proper normal.
	// @param inMaxSeparationDistance How much distance around the character you want to report contacts in (can be 0 to match the character exactly).
	// @param inShape Shape to test collision with.
	// @param inBaseOffset All hit results will be returned relative to this offset, can be zero to get results in world position, but when you're testing far from the origin you get better precision by picking a position that's closer e.g. GetPosition() since floats are most accurate near the origin
	// @param ioCollector Collision collector that receives the collision results.
	// @param inBroadPhaseLayerFilter Filter that is used to check if the character collides with something in the broadphase.
	// @param inObjectLayerFilter Filter that is used to check if a character collides with a layer.
	// @param inBodyFilter Filter that is used to check if a character collides with a body.
	// @param inShapeFilter Filter that is used to check if a character collides with a subshape.
	void CheckCollision(RVec3Arg inPosition, QuatArg inRotation, Vec3Arg inMovementDirection, float inMaxSeparationDistance, const Shape*inShape, RVec3Arg inBaseOffset, CollideShapeCollector& ioCollector, const BroadPhaseLayerFilter& inBroadPhaseLayerFilter, const ObjectLayerFilter& inObjectLayerFilter, const BodyFilter& inBodyFilter, const ShapeFilter& inShapeFilter) const;

	// Saving / restoring state for replay
	virtual void SaveState(StateRecorder& inStream) const override;
	virtual void RestoreState(StateRecorder& inStream) override;

#ifndef MOSS_DEBUG_RENDERER
	static inline bool sDrawConstraints = false;								// Draw the current state of the constraints for iteration 0 when creating them
	static inline bool sDrawWalkStairs = false;								// Draw the state of the walk stairs algorithm
	static inline bool sDrawStickToFloor = false;								// Draw the state of the stick to floor algorithm
#endif

	// Uniquely identifies a contact between a character and another body or character
	class ContactKey {
	public:
		// Constructor
		ContactKey() = default;
		ContactKey(const ContactKey& inContact) = default;
		ContactKey(const BodyID& inBodyB, const SubShapeID& inSubShapeID) : mBodyB(inBodyB), mSubShapeIDB(inSubShapeID) { }
		ContactKey(const CharacterID& inCharacterIDB, const SubShapeID& inSubShapeID) : mCharacterIDB(inCharacterIDB), mSubShapeIDB(inSubShapeID) { }
		ContactKey&	operator = (const ContactKey& inContact) = default;

		// Checks if two contacts refer to the same body (or virtual character)
		inline bool	IsSameBody(const ContactKey& inOther) const { return mBodyB == inOther.mBodyB& & mCharacterIDB == inOther.mCharacterIDB; }

		// Equality operator
		bool operator == (const ContactKey& inRHS) const { return mBodyB == inRHS.mBodyB& & mCharacterIDB == inRHS.mCharacterIDB& & mSubShapeIDB == inRHS.mSubShapeIDB; }

		bool operator != (const ContactKey& inRHS) const { return !(*this == inRHS); }

		// Hash of this structure
		uint64 GetHash() const {
			static_assert(sizeof(BodyID) + sizeof(CharacterID) + sizeof(SubShapeID) == sizeof(ContactKey), "No padding expected");
			return HashBytes(this, sizeof(ContactKey));
		}

		// Saving / restoring state for replay
		void SaveState(StateRecorder& inStream) const;
		void RestoreState(StateRecorder& inStream);

		BodyID			mBodyB;													// ID of body we're colliding with (if not invalid)
		CharacterID		mCharacterIDB;											// Character we're colliding with (if not invalid)
		SubShapeID		mSubShapeIDB;											// Sub shape ID of body or character we're colliding with
	};

	// Encapsulates a collision contact
	struct Contact : public ContactKey {
		// Saving / restoring state for replay
		void							SaveState(StateRecorder& inStream) const;
		void							RestoreState(StateRecorder& inStream);

		RVec3							mPosition;							// Position where the character makes contact
		Vec3							mLinearVelocity;					// Velocity of the contact point
		Vec3							mContactNormal;						// Contact normal, pointing towards the character
		Vec3							mSurfaceNormal;						// Surface normal of the contact
		float							mDistance;							// Distance to the contact <= 0 means that it is an actual contact, > 0 means predictive
		float							mFraction;							// Fraction along the path where this contact takes place
		EMotionType						mMotionTypeB;						// Motion type of B, used to determine the priority of the contact
		bool							mIsSensorB;							// If B is a sensor
		const CharacterVirtual*		mCharacterB = nullptr;					// Character we're colliding with (if not nullptr). Note that this may be a dangling pointer when accessed through GetActiveContacts(), use mCharacterIDB instead.
		uint64							mUserData;							// User data of B
		const PhysicsMaterial*			mMaterial;							// Material of B
		bool							mHadCollision = false;				// If the character actually collided with the contact (can be false if a predictive contact never becomes a real one)
		bool							mWasDiscarded = false;				// If the contact validate callback chose to discard this contact or when the body is a sensor
		bool							mCanPushCharacter = true;			// When true, the velocity of the contact point can push the character
	};

	using TempContactList = TArray<Contact, STLTempAllocator<Contact>>;
	using ContactList = TArray<Contact>;

	// Access to the internal list of contacts that the character has found.
	// Note that only contacts that have their mHadCollision flag set are actual contacts.
	const ContactList&	GetActiveContacts() const { return mActiveContacts; }

	// Check if the character is currently in contact with or has collided with another body in the last operation (e.g. Update or WalkStairs)
	bool HasCollidedWith(const BodyID& inBody) const {
		for (const CharacterVirtual::Contact& c : mActiveContacts)
			if (c.mHadCollision& & c.mBodyB == inBody)
				return true;
		return false;
	}

	// Check if the character is currently in contact with or has collided with another character in the last time step (e.g. Update or WalkStairs)
	bool HasCollidedWith(const CharacterID& inCharacterID) const
	{
		for (const CharacterVirtual::Contact& c : mActiveContacts)
			if (c.mHadCollision& & c.mCharacterIDB == inCharacterID)
				return true;
		return false;
	}

	// Check if the character is currently in contact with or has collided with another character in the last time step (e.g. Update or WalkStairs)
	bool HasCollidedWith(const CharacterVirtual*inCharacter) const { return HasCollidedWith(inCharacter->GetID()); }

private:
	// Sorting predicate for making contact order deterministic
	struct ContactOrderingPredicate {
		inline bool	operator () (const Contact& inLHS, const Contact& inRHS) const {
			if (inLHS.mBodyB != inRHS.mBodyB)
				return inLHS.mBodyB < inRHS.mBodyB;

			if (inLHS.mCharacterIDB != inRHS.mCharacterIDB)
				return inLHS.mCharacterIDB < inRHS.mCharacterIDB;

			return inLHS.mSubShapeIDB.GetValue() < inRHS.mSubShapeIDB.GetValue();
		}
	};

	using IgnoredContactList = TArray<ContactKey, STLTempAllocator<ContactKey>>;

	// A constraint that limits the movement of the character
	struct Constraint {
		Contact*						mContact;												// Contact that this constraint was generated from
		float							mTOI;													// Calculated time of impact (can be negative if penetrating)
		float							mProjectedVelocity;										// Velocity of the contact projected on the contact normal (negative if separating)
		Vec3							mLinearVelocity;										// Velocity of the contact (can contain a corrective velocity to resolve penetration)
		Plane							mPlane;													// Plane around the origin that describes how far we can displace (from the origin)
		bool							mIsSteepSlope = false;									// If this constraint belongs to a steep slope
	};

	using ConstraintList = TArray<Constraint, STLTempAllocator<Constraint>>;

	// Collision collector that collects hits for CollideShape
	class ContactCollector : public CollideShapeCollector {
	public:
		 ContactCollector(PhysicsSystem*inSystem, const CharacterVirtual*inCharacter, uint8 inMaxHits, float inHitReductionCosMaxAngle, Vec3Arg inUp, RVec3Arg inBaseOffset, TempContactList& outContacts) : mBaseOffset(inBaseOffset), mUp(inUp), mSystem(inSystem), mCharacter(inCharacter), mContacts(outContacts), mMaxHits(inMaxHits), mHitReductionCosMaxAngle(inHitReductionCosMaxAngle) { }

		virtual void SetUserData(uint64 inUserData) override { mOtherCharacter = reinterpret_cast<CharacterVirtual*>(inUserData); }

		virtual void AddHit(const CollideShapeResult& inResult) override;

		RVec3							mBaseOffset;
		Vec3							mUp;
		PhysicsSystem*					mSystem;
		const CharacterVirtual*			mCharacter;
		CharacterVirtual*				mOtherCharacter = nullptr;
		TempContactList&				mContacts;
		uint8							mMaxHits;
		float							mHitReductionCosMaxAngle;
		bool							mMaxHitsExceeded = false;
	};

	// A collision collector that collects hits for CastShape
	class ContactCastCollector : public CastShapeCollector {
	public:
		ContactCastCollector(PhysicsSystem*inSystem, const CharacterVirtual*inCharacter, Vec3Arg inDisplacement, Vec3Arg inUp, const IgnoredContactList& inIgnoredContacts, RVec3Arg inBaseOffset, Contact& outContact) : mBaseOffset(inBaseOffset), mDisplacement(inDisplacement), mUp(inUp), mSystem(inSystem), mCharacter(inCharacter), mIgnoredContacts(inIgnoredContacts), mContact(outContact) { }

		virtual void SetUserData(uint64 inUserData) override { mOtherCharacter = reinterpret_cast<CharacterVirtual*>(inUserData); }

		virtual void AddHit(const ShapeCastResult& inResult) override;

		RVec3							mBaseOffset;
		Vec3							mDisplacement;
		Vec3							mUp;
		PhysicsSystem*					mSystem;
		const CharacterVirtual*		mCharacter;
		CharacterVirtual*				mOtherCharacter = nullptr;
		const IgnoredContactList&		mIgnoredContacts;
		Contact&						mContact;
	};

	// Helper function to convert a Jolt collision result into a contact
	template <class taCollector>
	inline static void sFillContactProperties(const CharacterVirtual*inCharacter, Contact& outContact, const Body& inBody, Vec3Arg inUp, RVec3Arg inBaseOffset, const taCollector& inCollector, const CollideShapeResult& inResult);
	inline static void sFillCharacterContactProperties(Contact& outContact, const CharacterVirtual*inOtherCharacter, RVec3Arg inBaseOffset, const CollideShapeResult& inResult);

	// Move the shape from ioPosition and try to displace it by inVelocity* inDeltaTime, this will try to slide the shape along the world geometry
	void MoveShape(RVec3& ioPosition, Vec3Arg inVelocity, float inDeltaTime, ContactList*outActiveContacts, const BroadPhaseLayerFilter& inBroadPhaseLayerFilter, const ObjectLayerFilter& inObjectLayerFilter, 
		const BodyFilter& inBodyFilter, const ShapeFilter& inShapeFilter, TempAllocator& inAllocator
	#ifndef MOSS_DEBUG_RENDERER
	, bool inDrawConstraints = false
	#endif // MOSS_DEBUG_RENDERER
	);

	// Ask the callback if inContact is a valid contact point
	bool ValidateContact(const Contact& inContact) const;

	// Trigger the contact callback for inContact and get the contact settings
	void ContactAdded(const Contact& inContact, CharacterContactSettings& ioSettings);

	// Tests the shape for collision around inPosition
	void GetContactsAtPosition(RVec3Arg inPosition, Vec3Arg inMovementDirection, const Shape*inShape, TempContactList& outContacts, const BroadPhaseLayerFilter& inBroadPhaseLayerFilter, 
		const ObjectLayerFilter& inObjectLayerFilter, const BodyFilter& inBodyFilter, const ShapeFilter& inShapeFilter) const;

	// Remove penetrating contacts with the same body that have conflicting normals, leaving these will make the character mover get stuck
	void RemoveConflictingContacts(TempContactList& ioContacts, IgnoredContactList& outIgnoredContacts) const;

	// Convert contacts into constraints. The character is assumed to start at the origin and the constraints are planes around the origin that confine the movement of the character.
	void DetermineConstraints(TempContactList& inContacts, float inDeltaTime, ConstraintList& outConstraints) const;

	// Use the constraints to solve the displacement of the character. This will slide the character on the planes around the origin for as far as possible.
	void SolveConstraints(Vec3Arg inVelocity, float inDeltaTime, float inTimeRemaining, ConstraintList& ioConstraints, IgnoredContactList& ioIgnoredContacts, float& outTimeSimulated, Vec3& outDisplacement, 
		TempAllocator& inAllocator
	#ifndef MOSS_DEBUG_RENDERER
		, bool inDrawConstraints = false
	#endif // MOSS_DEBUG_RENDERER
	);

	// Get the velocity of a body adjusted by the contact listener
	void GetAdjustedBodyVelocity(const Body& inBody, Vec3& outLinearVelocity, Vec3& outAngularVelocity) const;

	// Calculate the ground velocity of the character assuming it's standing on an object with specified linear and angular velocity and with specified center of mass.
	// Note that we don't just take the point velocity because a point on an object with angular velocity traces an arc,
	// so if you just take point velocity* delta time you get an error that accumulates over time
	Vec3 CalculateCharacterGroundVelocity(RVec3Arg inCenterOfMass, Vec3Arg inLinearVelocity, Vec3Arg inAngularVelocity, float inDeltaTime) const;

	// Handle contact with physics object that we're colliding against
	bool HandleContact(Vec3Arg inVelocity, Constraint& ioConstraint, float inDeltaTime);

	// Does a swept test of the shape from inPosition with displacement inDisplacement, returns true if there was a collision
	bool GetFirstContactForSweep(RVec3Arg inPosition, Vec3Arg inDisplacement, Contact& outContact, const IgnoredContactList& inIgnoredContacts, const BroadPhaseLayerFilter& inBroadPhaseLayerFilter, const ObjectLayerFilter& inObjectLayerFilter, const BodyFilter& inBodyFilter, const ShapeFilter& inShapeFilter) const;

	// Store contacts so that we have proper ground information
	void StoreActiveContacts(const TempContactList& inContacts, TempAllocator& inAllocator);

	// This function will determine which contacts are touching the character and will calculate the one that is supporting us
	void UpdateSupportingContact(bool inSkipContactVelocityCheck, TempAllocator& inAllocator);

	// This function can be called after moving the character to a new colliding position
	void MoveToContact(RVec3Arg inPosition, const Contact& inContact, const BroadPhaseLayerFilter& inBroadPhaseLayerFilter, const ObjectLayerFilter& inObjectLayerFilter, const BodyFilter& inBodyFilter, const ShapeFilter& inShapeFilter, TempAllocator& inAllocator);

	// This function returns the actual center of mass of the shape, not corrected for the character padding
	inline RMat44 GetCenterOfMassTransform(RVec3Arg inPosition, QuatArg inRotation, const Shape*inShape) const {
		return RMat44::sRotationTranslation(inRotation, inPosition).PreTranslated(mShapeOffset + inShape->GetCenterOfMass()).PostTranslated(mCharacterPadding* mUp);
	}

	// This function returns the position of the inner rigid body
	inline RVec3 GetInnerBodyPosition() const { return mPosition + (mRotation* mShapeOffset + mCharacterPadding* mUp); }

	// Move the inner rigid body to the current position
	void UpdateInnerBodyTransform();

	// ID
	CharacterID							mID;

	// Our main listener for contacts
	CharacterContactListener*			mListener = nullptr;

	// Interface to detect collision between characters
	CharacterVsCharacterCollision*		mCharacterVsCharacterCollision = nullptr;

	// Movement settings
	EBackFaceMode						mBackFaceMode;											// When colliding with back faces, the character will not be able to move through back facing triangles. Use this if you have triangles that need to collide on both sides.
	float								mPredictiveContactDistance;								// How far to scan outside of the shape for predictive contacts. A value of 0 will most likely cause the character to get stuck as it cannot properly calculate a sliding direction anymore. A value that's too high will cause ghost collisions.
	uint8								mMaxCollisionIterations;								// Max amount of collision loops
	uint8								mMaxConstraintIterations;								// How often to try stepping in the constraint solving
	float								mMinTimeRemaining;										// Early out condition: If this much time is left to simulate we are done
	float								mCollisionTolerance;									// How far we're willing to penetrate geometry
	float								mCharacterPadding;										// How far we try to stay away from the geometry, this ensures that the sweep will hit as little as possible lowering the collision cost and reducing the risk of getting stuck
	uint8								mMaxNumHits;											// Max num hits to collect in order to avoid excess of contact points collection
	float								mHitReductionCosMaxAngle;								// Cos(angle) where angle is the maximum angle between two hits contact normals that are allowed to be merged during hit reduction. Default is around 2.5 degrees. Set to -1 to turn off.
	float								mPenetrationRecoverySpeed;								// This value governs how fast a penetration will be resolved, 0 = nothing is resolved, 1 = everything in one update
	bool								mEnhancedInternalEdgeRemoval;							// Set to indicate that extra effort should be made to try to remove ghost contacts (collisions with internal edges of a mesh). This is more expensive but makes bodies move smoother over a mesh with convex edges.

	// Character mass (kg)
	float								mMass;

	// Maximum force with which the character can push other bodies (N)
	float								mMaxStrength;

	// An extra offset applied to the shape in local space. This allows applying an extra offset to the shape in local space.
	Vec3								mShapeOffset = Vec3::sZero();

	// Current position (of the base, not the center of mass)
	RVec3								mPosition = RVec3::sZero();

	// Current rotation (of the base, not of the center of mass)
	Quat								mRotation = Quat::sIdentity();

	// Current linear velocity
	Vec3								mLinearVelocity = Vec3::sZero();

	// List of contacts that were active in the last frame
	ContactList							mActiveContacts;

	// Remembers how often we called StartTrackingContactChanges
	int									mTrackingContactChanges = 0;

	// View from a contact listener perspective on which contacts have been added/removed
	struct ListenerContactValue
	{
										ListenerContactValue() = default;
		explicit						ListenerContactValue(const CharacterContactSettings& inSettings) : mSettings(inSettings) { }

		CharacterContactSettings		mSettings;
		int								mCount = 0;
	};

	using ListenerContacts = TMap<ContactKey, ListenerContactValue>;
	ListenerContacts					mListenerContacts;

	// Remembers the delta time of the last update
	float								mLastDeltaTime = 1.0f / 60.0f;

	// Remember if we exceeded the maximum number of hits and had to remove similar contacts
	mutable bool						mMaxHitsExceeded = false;

	// User data, can be used for anything by the application
	uint64								mUserData = 0;

	// The inner rigid body that proxies the character in the world
	BodyID								mInnerBodyID;
};

/* CharacterContactListener*/
// This class receives callbacks when a virtual character hits something.
class MOSS_API CharacterContactListener {
public:
	virtual	~CharacterContactListener() = default;

	// Callback to adjust the velocity of a body as seen by the character. Can be adjusted to e.g. implement a conveyor belt or an inertial dampener system of a sci-fi space ship.
	// Note that inBody2 is locked during the callback so you can read its properties freely.
	virtual void OnAdjustBodyVelocity(const CharacterVirtual*inCharacter, const Body& inBody2, Vec3& ioLinearVelocity, Vec3& ioAngularVelocity) { /* Do nothing, the linear and angular velocity are already filled in*/ }

	// Checks if a character can collide with specified body. Return true if the contact is valid.
	virtual bool OnContactValidate(const CharacterVirtual*inCharacter, const BodyID& inBodyID2, const SubShapeID& inSubShapeID2) { return true; }

	// Same as OnContactValidate but when colliding with a CharacterVirtual
	virtual bool OnCharacterContactValidate(const CharacterVirtual*inCharacter, const CharacterVirtual*inOtherCharacter, const SubShapeID& inSubShapeID2) { return true; }

	// Called whenever the character collides with a body for the first time.
	// @param inCharacter Character that is being solved
	// @param inBodyID2 Body ID of body that is being hit
	// @param inSubShapeID2 Sub shape ID of shape that is being hit
	// @param inContactPosition World space contact position
	// @param inContactNormal World space contact normal
	// @param ioSettings Settings returned by the contact callback to indicate how the character should behave
	virtual void OnContactAdded(const CharacterVirtual*inCharacter, const BodyID& inBodyID2, const SubShapeID& inSubShapeID2, RVec3Arg inContactPosition, Vec3Arg inContactNormal, CharacterContactSettings& ioSettings) { /* Default do nothing*/ }

	// Called whenever the character persists colliding with a body.
	// @param inCharacter Character that is being solved
	// @param inBodyID2 Body ID of body that is being hit
	// @param inSubShapeID2 Sub shape ID of shape that is being hit
	// @param inContactPosition World space contact position
	// @param inContactNormal World space contact normal
	// @param ioSettings Settings returned by the contact callback to indicate how the character should behave
	virtual void OnContactPersisted(const CharacterVirtual*inCharacter, const BodyID& inBodyID2, const SubShapeID& inSubShapeID2, RVec3Arg inContactPosition, Vec3Arg inContactNormal, CharacterContactSettings& ioSettings) { /* Default do nothing*/ }

	// Called whenever the character loses contact with a body.
	// Note that there is no guarantee that the body or its sub shape still exists at this point. The body may have been deleted since the last update.
	// @param inCharacter Character that is being solved
	// @param inBodyID2 Body ID of body that is being hit
	// @param inSubShapeID2 Sub shape ID of shape that is being hit
	virtual void OnContactRemoved(const CharacterVirtual*inCharacter, const BodyID& inBodyID2, const SubShapeID& inSubShapeID2) { /* Default do nothing*/ }

	// Same as OnContactAdded but when colliding with a CharacterVirtual
	virtual void OnCharacterContactAdded(const CharacterVirtual*inCharacter, const CharacterVirtual*inOtherCharacter, const SubShapeID& inSubShapeID2, RVec3Arg inContactPosition, Vec3Arg inContactNormal, CharacterContactSettings& ioSettings) { /* Default do nothing*/ }

	// Same as OnContactPersisted but when colliding with a CharacterVirtual
	virtual void OnCharacterContactPersisted(const CharacterVirtual*inCharacter, const CharacterVirtual*inOtherCharacter, const SubShapeID& inSubShapeID2, RVec3Arg inContactPosition, Vec3Arg inContactNormal, CharacterContactSettings& ioSettings) { /* Default do nothing*/ }

	// Same as OnContactRemoved but when colliding with a CharacterVirtual
	// Note that inOtherCharacterID can be the ID of a character that has been deleted. This happens if the character was in contact with this character during the last update, but has been deleted since.
	virtual void OnCharacterContactRemoved(const CharacterVirtual*inCharacter, const CharacterID& inOtherCharacterID, const SubShapeID& inSubShapeID2) { /* Default do nothing*/ }

	// Called whenever a contact is being used by the solver. Allows the listener to override the resulting character velocity (e.g. by preventing sliding along certain surfaces).
	// @param inCharacter Character that is being solved
	// @param inBodyID2 Body ID of body that is being hit
	// @param inSubShapeID2 Sub shape ID of shape that is being hit
	// @param inContactPosition World space contact position
	// @param inContactNormal World space contact normal
	// @param inContactVelocity World space velocity of contact point (e.g. for a moving platform)
	// @param inContactMaterial Material of contact point
	// @param inCharacterVelocity World space velocity of the character prior to hitting this contact
	// @param ioNewCharacterVelocity Contains the calculated world space velocity of the character after hitting this contact, this velocity slides along the surface of the contact. Can be modified by the listener to provide an alternative velocity.
	virtual void OnContactSolve(const CharacterVirtual*inCharacter, const BodyID& inBodyID2, const SubShapeID& inSubShapeID2, RVec3Arg inContactPosition, Vec3Arg inContactNormal, Vec3Arg inContactVelocity, const PhysicsMaterial*inContactMaterial, Vec3Arg inCharacterVelocity, Vec3& ioNewCharacterVelocity) { /* Default do nothing*/ }

	// Same as OnContactSolve but when colliding with a CharacterVirtual
	virtual void OnCharacterContactSolve(const CharacterVirtual*inCharacter, const CharacterVirtual*inOtherCharacter, const SubShapeID& inSubShapeID2, RVec3Arg inContactPosition, Vec3Arg inContactNormal, Vec3Arg inContactVelocity, const PhysicsMaterial*inContactMaterial, Vec3Arg inCharacterVelocity, Vec3& ioNewCharacterVelocity) { /* Default do nothing*/ }
};


/* CharacterVsCharacterCollision*/
class MOSS_API CharacterVsCharacterCollision : public NonCopyable {
public:
	virtual ~CharacterVsCharacterCollision() = default;

	// Collide a character against other CharacterVirtuals.
	// @param inCharacter The character to collide.
	// @param inCenterOfMassTransform Center of mass transform for this character.
	// @param inCollideShapeSettings Settings for the collision check.
	// @param inBaseOffset All hit results will be returned relative to this offset, can be zero to get results in world position, but when you're testing far from the origin you get better precision by picking a position that's closer e.g. GetPosition() since floats are most accurate near the origin
	// @param ioCollector Collision collector that receives the collision results.
	virtual void CollideCharacter(const CharacterVirtual*inCharacter, RMat44Arg inCenterOfMassTransform, const CollideShapeSettings& inCollideShapeSettings, RVec3Arg inBaseOffset, CollideShapeCollector& ioCollector) const = 0;

	// Cast a character against other CharacterVirtuals.
	// @param inCharacter The character to cast.
	// @param inCenterOfMassTransform Center of mass transform for this character.
	// @param inDirection Direction and length to cast in.
	// @param inShapeCastSettings Settings for the shape cast.
	// @param inBaseOffset All hit results will be returned relative to this offset, can be zero to get results in world position, but when you're testing far from the origin you get better precision by picking a position that's closer e.g. GetPosition() since floats are most accurate near the origin
	// @param ioCollector Collision collector that receives the collision results.
	virtual void CastCharacter(const CharacterVirtual*inCharacter, RMat44Arg inCenterOfMassTransform, Vec3Arg inDirection, const ShapeCastSettings& inShapeCastSettings, RVec3Arg inBaseOffset, CastShapeCollector& ioCollector) const = 0;
};

class MOSS_API CharacterVsCharacterCollisionSimple : public CharacterVsCharacterCollision {
public:
	// Add a character to the list of characters to check collision against.
	void Add(CharacterVirtual*inCharacter)						{ mCharacters.push_back(inCharacter); }

	// Remove a character from the list of characters to check collision against.
	void Remove(const CharacterVirtual*inCharacter);

	// See: CharacterVsCharacterCollision
	virtual void CollideCharacter(const CharacterVirtual*inCharacter, RMat44Arg inCenterOfMassTransform, const CollideShapeSettings& inCollideShapeSettings, RVec3Arg inBaseOffset, CollideShapeCollector& ioCollector) const override;
	virtual void CastCharacter(const CharacterVirtual*inCharacter, RMat44Arg inCenterOfMassTransform, Vec3Arg inDirection, const ShapeCastSettings& inShapeCastSettings, RVec3Arg inBaseOffset, CastShapeCollector& ioCollector) const override;

	TArray<CharacterVirtual*> mCharacters;	// The list of characters to check collision against
};

/* CollisionDispatch*/
MOSS_API bool CollisionDispatch_CollideShapeVsShape(
	const Shape* shape1, const Shape* shape2,
	const Vec3* scale1, const Vec3* scale2,
	const Mat44* centerOfMassTransform1, const Mat44* centerOfMassTransform2,
	const CollideShapeSettings* collideShapeSettings,
	CollideShapeCollectorCallback* callback, void* userData, const ShapeFilter* shapeFilter);

MOSS_API bool CollisionDispatch_CastShapeVsShapeLocalSpace(
	const Vec3* direction, const Shape* shape1, const Shape* shape2,
	const Vec3* scale1InShape2LocalSpace, const Vec3* scale2,
	Mat44* centerOfMassTransform1InShape2LocalSpace, Mat44* centerOfMassWorldTransform2,
	const ShapeCastSettings* shapeCastSettings,
	CastShapeCollectorCallback* callback, void* userData,
	const ShapeFilter* shapeFilter);

MOSS_API bool CollisionDispatch_CastShapeVsShapeWorldSpace(
	const Vec3* direction, const Shape* shape1, const Shape* shape2,
	const Vec3* scale1, const Vec3* inScale2,
	const Mat44* centerOfMassWorldTransform1, const Mat44* centerOfMassWorldTransform2,
	const ShapeCastSettings* shapeCastSettings,
	CastShapeCollectorCallback* callback, void* userData,
	const ShapeFilter* shapeFilter);

/* EstimateCollisionResponse*/
MOSS_API void EstimateCollisionResponse(const Body* body1, const Body* body2, const ContactManifold* manifold, float combinedFriction, float combinedRestitution, float minVelocityForRestitution, uint32_t numIterations, CollisionEstimationResult* result);



class MOSS_API MassProperties {
public:
	// Using eigendecomposition, decompose the inertia tensor into a diagonal matrix D and a right-handed rotation matrix R so that the inertia tensor is \f$R \: D \: R^{-1}\f$.
	// @see https://en.wikipedia.org/wiki/Moment_of_inertia section 'Principal axes'
	// @param outRotation The rotation matrix R
	// @param outDiagonal The diagonal of the diagonal matrix D
	// @return True if successful, false if failed
	bool					DecomposePrincipalMomentsOfInertia(Mat44& outRotation, Vec3& outDiagonal) const;

	// Set the mass and inertia of a box with edge size inBoxSize and density inDensity
	void					SetMassAndInertiaOfSolidBox(Vec3Arg inBoxSize, float inDensity);
	// Set the mass and scale the inertia tensor to match the mass
	void					ScaleToMass(float inMass);
	// Calculates the size of the solid box that has an inertia tensor diagonal inInertiaDiagonal
	static Vec3				sGetEquivalentSolidBoxSize(float inMass, Vec3Arg inInertiaDiagonal);
	// Rotate the inertia by 3x3 matrix inRotation
	void					Rotate(Mat44Arg inRotation);
	// Translate the inertia by a vector inTranslation
	void					Translate(Vec3Arg inTranslation);
	// Scale the mass and inertia by inScale, note that elements can be < 0 to flip the shape
	void					Scale(Vec3Arg inScale);
	// Saves the state of this object in binary form to inStream.
	void					SaveBinaryState(StreamOut& inStream) const;
	// Restore the state of this object from inStream.
	void					RestoreBinaryState(StreamIn& inStream);
	// Mass of the shape (kg)
	float					mMass = 0.0f;
	// Inertia tensor of the shape (kg m^2)
	Mat44					mInertia = Mat44::sZero();
};


// Temporary data used by the update of a soft body
class SoftBodyUpdateContext : public NonCopyable
{
public:
	static constexpr uint				cVertexCollisionBatch = 64;					// Number of vertices to process in a batch in DetermineCollisionPlanes
	static constexpr uint				cVertexConstraintBatch = 256;				// Number of vertices to group for processing batches of constraints in ApplyEdgeConstraints

	// Input
	Body*								mBody;										// Body that is being updated
	SoftBodyMotionProperties*			mMotionProperties;							// Motion properties of that body
	SoftBodyContactListener*			mContactListener;							// Contact listener to fire callbacks to
	const SimShapeFilter*				mSimShapeFilter;							// Shape filter to use for collision detection
	RMat44								mCenterOfMassTransform;						// Transform of the body relative to the soft body
	Vec3								mGravity;									// Gravity vector in local space of the soft body
	Vec3								mDisplacementDueToGravity;					// Displacement of the center of mass due to gravity in the current time step
	float								mDeltaTime;									// Delta time for the current time step
	float								mSubStepDeltaTime;							// Delta time for each sub step

	// Describes progress in the current update
	enum class EState
	{
		DetermineCollisionPlanes,													// Determine collision planes for vertices in parallel
		DetermineSensorCollisions,													// Determine collisions with sensors in parallel
		ApplyConstraints,															// Apply constraints in parallel
		Done																		// Update is finished
	};

	// State of the update
	atomic<EState>						mState { EState::DetermineCollisionPlanes };// Current state of the update
	atomic<uint>						mNextCollisionVertex { 0 };					// Next vertex to process for DetermineCollisionPlanes
	atomic<uint>						mNumCollisionVerticesProcessed { 0 };		// Number of vertices processed by DetermineCollisionPlanes, used to determine if we can go to the next step
	atomic<uint>						mNextSensorIndex { 0 };						// Next sensor to process for DetermineCollisionPlanes
	atomic<uint>						mNumSensorsProcessed { 0 };					// Number of sensors processed by DetermineSensorCollisions, used to determine if we can go to the next step
	atomic<uint>						mNextIteration { 0 };						// Next simulation iteration to process
	atomic<uint>						mNextConstraintGroup { 0 };					// Next constraint group to process
	atomic<uint>						mNumConstraintGroupsProcessed { 0 };		// Number of groups processed, used to determine if we can go to the next iteration

	// Output
	Vec3								mDeltaPosition;								// Delta position of the body in the current time step, should be applied after the update
	ECanSleep							mCanSleep;									// Can the body sleep? Should be applied after the update
};


// Run time information for a single particle of a soft body
// Note that at run-time you should only modify the inverse mass and/or velocity of a vertex to control the soft body.
// Modifying the position can lead to missed collisions.
// The other members are used internally by the soft body solver.
class SoftBodyVertex
{
public:
	// Reset collision information to prepare for a new collision check
	inline void		ResetCollision()
	{
		mLargestPenetration = -FLT_MAX;
		mCollidingShapeIndex = -1;
		mHasContact = false;
	}

	Vec3			mPreviousPosition;					// Internal use only. Position at the previous time step
	Vec3			mPosition;							// Position, relative to the center of mass of the soft body
	Vec3			mVelocity;							// Velocity, relative to the center of mass of the soft body
	Plane			mCollisionPlane;					// Internal use only. Nearest collision plane, relative to the center of mass of the soft body
	int				mCollidingShapeIndex;				// Internal use only. Index in the colliding shapes list of the body we may collide with
	bool			mHasContact;						// True if the vertex has collided with anything in the last update
	float			mLargestPenetration;				// Internal use only. Used while finding the collision plane, stores the largest penetration found so far
	float			mInvMass;							// Inverse mass (1 / mass)
};

// An interface to query which vertices of a soft body are colliding with other bodies
class SoftBodyManifold
{
public:
	// Get the vertices of the soft body for iterating
	const TArray<SoftBodyVertex>& 	GetVertices() const							{ return mVertices; }

	// Check if a vertex has collided with something in this update
	MOSS_INLINE bool					HasContact(const SoftBodyVertex& inVertex) const
	{
		return inVertex.mHasContact;
	}

	// Get the local space contact point (multiply by GetCenterOfMassTransform() of the soft body to get world space)
	MOSS_INLINE Vec3					GetLocalContactPoint(const SoftBodyVertex& inVertex) const
	{
		return inVertex.mPosition - inVertex.mCollisionPlane.SignedDistance(inVertex.mPosition)* inVertex.mCollisionPlane.GetNormal();
	}

	// Get the contact normal for the vertex (assumes there is a contact).
	MOSS_INLINE Vec3					GetContactNormal(const SoftBodyVertex& inVertex) const
	{
		return -inVertex.mCollisionPlane.GetNormal();
	}

	// Get the body with which the vertex has collided in this update
	MOSS_INLINE BodyID				GetContactBodyID(const SoftBodyVertex& inVertex) const
	{
		return inVertex.mHasContact? mCollidingShapes[inVertex.mCollidingShapeIndex].mBodyID : BodyID();
	}

	// Get the number of sensors that are in contact with the soft body
	MOSS_INLINE uint					GetNumSensorContacts() const
	{
		return (uint)mCollidingSensors.size();
	}

	// Get the i-th sensor that is in contact with the soft body
	MOSS_INLINE BodyID				GetSensorContactBodyID(uint inIndex) const
	{
		return mCollidingSensors[inIndex].mBodyID;
	}

private:
	// Allow SoftBodyMotionProperties to construct us
	friend class SoftBodyMotionProperties;

	// Constructor
	explicit						SoftBodyManifold(const SoftBodyMotionProperties*inMotionProperties) :
										mVertices(inMotionProperties->mVertices),
										mCollidingShapes(inMotionProperties->mCollidingShapes),
										mCollidingSensors(inMotionProperties->mCollidingSensors)
	{
	}

	using CollidingShape = SoftBodyMotionProperties::CollidingShape;
	using CollidingSensor = SoftBodyMotionProperties::CollidingSensor;

	const TArray<SoftBodyVertex>&	mVertices;
	const TArray<CollidingShape>&	mCollidingShapes;
	const TArray<CollidingSensor>&	mCollidingSensors;
};

class MOSS_EXPORT SoftBodyMotionProperties : public MotionProperties
{
public:
	using Vertex = SoftBodyVertex;
	using Edge = SoftBodySharedSettings::Edge;
	using Face = SoftBodySharedSettings::Face;
	using DihedralBend = SoftBodySharedSettings::DihedralBend;
	using Volume = SoftBodySharedSettings::Volume;
	using InvBind = SoftBodySharedSettings::InvBind;
	using SkinWeight = SoftBodySharedSettings::SkinWeight;
	using Skinned = SoftBodySharedSettings::Skinned;
	using LRA = SoftBodySharedSettings::LRA;

	// Initialize the soft body motion properties
	void								Initialize(const SoftBodyCreationSettings& inSettings);

	// Get the shared settings of the soft body
	const SoftBodySharedSettings*		GetSettings() const							{ return mSettings; }

	// Get the vertices of the soft body
	const TArray<Vertex>& 				GetVertices() const							{ return mVertices; }
	TArray<Vertex>& 						GetVertices()								{ return mVertices; }

	// Access an individual vertex
	const Vertex& 						GetVertex(uint inIndex) const				{ return mVertices[inIndex]; }
	Vertex& 							GetVertex(uint inIndex)						{ return mVertices[inIndex]; }

	// Get the materials of the soft body
	const PhysicsMaterialList& 			GetMaterials() const						{ return mSettings->mMaterials; }

	// Get the faces of the soft body
	const TArray<Face>& 					GetFaces() const							{ return mSettings->mFaces; }

	// Access to an individual face
	const Face& 						GetFace(uint inIndex) const					{ return mSettings->mFaces[inIndex]; }

	// Get the number of solver iterations
	uint32								GetNumIterations() const					{ return mNumIterations; }
	void								SetNumIterations(uint32 inNumIterations)	{ mNumIterations = inNumIterations; }

	// Get the pressure of the soft body
	float								GetPressure() const							{ return mPressure; }
	void								SetPressure(float inPressure)				{ mPressure = inPressure; }

	// Update the position of the body while simulating (set to false for something that is attached to the static world)
	bool								GetUpdatePosition() const					{ return mUpdatePosition; }
	void								SetUpdatePosition(bool inUpdatePosition)	{ mUpdatePosition = inUpdatePosition; }

	// Global setting to turn on/off skin constraints
	bool								GetEnableSkinConstraints() const			{ return mEnableSkinConstraints; }
	void								SetEnableSkinConstraints(bool inEnableSkinConstraints) { mEnableSkinConstraints = inEnableSkinConstraints; }

	// Multiplier applied to Skinned::mMaxDistance to allow tightening or loosening of the skin constraints. 0 to hard skin all vertices.
	float								GetSkinnedMaxDistanceMultiplier() const		{ return mSkinnedMaxDistanceMultiplier; }
	void								SetSkinnedMaxDistanceMultiplier(float inSkinnedMaxDistanceMultiplier) { mSkinnedMaxDistanceMultiplier = inSkinnedMaxDistanceMultiplier; }

	// Get local bounding box
	const AABox& 						GetLocalBounds() const						{ return mLocalBounds; }

	// Get the volume of the soft body. Note can become negative if the shape is inside out!
	float								GetVolume() const							{ return GetVolumeTimesSix() / 6.0f; }

	// Calculate the total mass and inertia of this body based on the current state of the vertices
	void								CalculateMassAndInertia();

#ifndef MOSS_DEBUG_RENDERER
	// Draw the state of a soft body
	void								DrawVertices(DebugRenderer*inRenderer, RMat44Arg inCenterOfMassTransform) const;
	void								DrawVertexVelocities(DebugRenderer*inRenderer, RMat44Arg inCenterOfMassTransform) const;
	void								DrawEdgeConstraints(DebugRenderer*inRenderer, RMat44Arg inCenterOfMassTransform, ESoftBodyConstraintColor inConstraintColor) const;
	void								DrawBendConstraints(DebugRenderer*inRenderer, RMat44Arg inCenterOfMassTransform, ESoftBodyConstraintColor inConstraintColor) const;
	void								DrawVolumeConstraints(DebugRenderer*inRenderer, RMat44Arg inCenterOfMassTransform, ESoftBodyConstraintColor inConstraintColor) const;
	void								DrawSkinConstraints(DebugRenderer*inRenderer, RMat44Arg inCenterOfMassTransform, ESoftBodyConstraintColor inConstraintColor) const;
	void								DrawLRAConstraints(DebugRenderer*inRenderer, RMat44Arg inCenterOfMassTransform, ESoftBodyConstraintColor inConstraintColor) const;
	void								DrawPredictedBounds(DebugRenderer*inRenderer, RMat44Arg inCenterOfMassTransform) const;
#endif // MOSS_DEBUG_RENDERER

	// Saving state for replay
	void								SaveState(StateRecorder& inStream) const;

	// Restoring state for replay
	void								RestoreState(StateRecorder& inStream);

	// Skin vertices to supplied joints, information is used by the skinned constraints.
	// @param inCenterOfMassTransform Value of Body::GetCenterOfMassTransform().
	// @param inJointMatrices The joint matrices must be expressed relative to inCenterOfMassTransform.
	// @param inNumJoints Indicates how large the inJointMatrices array is (used only for validating out of bounds).
	// @param inHardSkinAll Can be used to position all vertices on the skinned vertices and can be used to hard reset the soft body.
	// @param ioTempAllocator Allocator.
	void								SkinVertices(RMat44Arg inCenterOfMassTransform, const Mat44*inJointMatrices, uint inNumJoints, bool inHardSkinAll, TempAllocator& ioTempAllocator);

	// This function allows you to update the soft body immediately without going through the PhysicsSystem.
	// This is useful if the soft body is teleported and needs to 'settle' or it can be used if a the soft body
	// is not added to the PhysicsSystem and needs to be updated manually. One reason for not adding it to the
	// PhysicsSystem is that you might want to update a soft body immediately after updating an animated object
	// that has the soft body attached to it. If the soft body is added to the PhysicsSystem it will be updated
	// by it, so calling this function will effectively update it twice. Note that when you use this function,
	// only the current thread will be used, whereas if you update through the PhysicsSystem, multiple threads may
	// be used.
	// Note that this will bypass any sleep checks. Since the dynamic objects that the soft body touches
	// will not move during this call, there can be simulation artifacts if you call this function multiple times
	// without running the physics simulation step.
	void								CustomUpdate(float inDeltaTime, Body& ioSoftBody, PhysicsSystem& inSystem);

	////////////////////////////////////////
	// FUNCTIONS BELOW THIS LINE ARE FOR INTERNAL USE ONLY
	////////////////////////////////////////

	// Initialize the update context. Not part of the public API.
	void								InitializeUpdateContext(float inDeltaTime, Body& inSoftBody, const PhysicsSystem& inSystem, SoftBodyUpdateContext& ioContext);

	// Do a broad phase check and collect all bodies that can possibly collide with this soft body. Not part of the public API.
	void								DetermineCollidingShapes(const SoftBodyUpdateContext& inContext, const PhysicsSystem& inSystem, const BodyLockInterface& inBodyLockInterface);

	// Return code for ParallelUpdate
	enum class EStatus
	{
		NoWork	= 1 << 0,				// No work was done because other threads were still working on a batch that cannot run concurrently
		DidWork	= 1 << 1,				// Work was done to progress the update
		Done	= 1 << 2,				// All work is done
	};

	// Update the soft body, will process a batch of work. Not part of the public API.
	EStatus								ParallelUpdate(SoftBodyUpdateContext& ioContext, const PhysicsSettings& inPhysicsSettings);

	// Update the velocities of all rigid bodies that we collided with. Not part of the public API.
	void								UpdateRigidBodyVelocities(const SoftBodyUpdateContext& inContext, BodyInterface& inBodyInterface);

private:
	// SoftBodyManifold needs to have access to CollidingShape
	friend class SoftBodyManifold;

	// Information about a leaf shape that we're colliding with
	struct LeafShape
	{
										LeafShape() = default;
										LeafShape(Mat44Arg inTransform, Vec3Arg inScale, const Shape*inShape) : mTransform(inTransform), mScale(inScale), mShape(inShape) { }

		Mat44							mTransform;									// Transform of the shape relative to the soft body
		Vec3							mScale;										// Scale of the shape
		RefConst<Shape>					mShape;										// Shape
	};

	// Collect information about the colliding bodies
	struct CollidingShape
	{
		// Get the velocity of a point on this body
		Vec3							GetPointVelocity(Vec3Arg inPointRelativeToCOM) const
		{
			return mLinearVelocity + mAngularVelocity.Cross(inPointRelativeToCOM);
		}

		Mat44							mCenterOfMassTransform;						// Transform of the body relative to the soft body
		TArray<LeafShape>				mShapes;									// Leaf shapes of the body we hit
		BodyID							mBodyID;									// Body ID of the body we hit
		EMotionType						mMotionType;								// Motion type of the body we hit
		float							mInvMass;									// Inverse mass of the body we hit
		float							mFriction;									// Combined friction of the two bodies
		float							mRestitution;								// Combined restitution of the two bodies
		float							mSoftBodyInvMassScale;						// Scale factor for the inverse mass of the soft body vertices
		bool							mUpdateVelocities;							// If the linear/angular velocity changed and the body needs to be updated
		Mat44							mInvInertia;								// Inverse inertia in local space to the soft body
		Vec3							mLinearVelocity;							// Linear velocity of the body in local space to the soft body
		Vec3							mAngularVelocity;							// Angular velocity of the body in local space to the soft body
		Vec3							mOriginalLinearVelocity;					// Linear velocity of the body in local space to the soft body at start
		Vec3							mOriginalAngularVelocity;					// Angular velocity of the body in local space to the soft body at start
	};

	// Collect information about the colliding sensors
	struct CollidingSensor
	{
		Mat44							mCenterOfMassTransform;						// Transform of the body relative to the soft body
		TArray<LeafShape>				mShapes;									// Leaf shapes of the body we hit
		BodyID							mBodyID;									// Body ID of the body we hit
		bool							mHasContact;								// If the sensor collided with the soft body
	};

	// Information about the state of all skinned vertices
	struct SkinState
	{
		Vec3							mPreviousPosition = Vec3::sZero();			// Previous position of the skinned vertex, used to interpolate between the previous and current position
		Vec3							mPosition = Vec3::sNaN();					// Current position of the skinned vertex
		Vec3							mNormal = Vec3::sNaN();						// Normal of the skinned vertex
	};

	// Do a narrow phase check and determine the closest feature that we can collide with
	void								DetermineCollisionPlanes(uint inVertexStart, uint inNumVertices);

	// Do a narrow phase check between a single sensor and the soft body
	void								DetermineSensorCollisions(CollidingSensor& ioSensor);

	// Apply pressure force and update the vertex velocities
	void								ApplyPressure(const SoftBodyUpdateContext& inContext);

	// Integrate the positions of all vertices by 1 sub step
	void								IntegratePositions(const SoftBodyUpdateContext& inContext);

	// Enforce all bend constraints
	void								ApplyDihedralBendConstraints(const SoftBodyUpdateContext& inContext, uint inStartIndex, uint inEndIndex);

	// Enforce all volume constraints
	void								ApplyVolumeConstraints(const SoftBodyUpdateContext& inContext, uint inStartIndex, uint inEndIndex);

	// Enforce all skin constraints
	void								ApplySkinConstraints(const SoftBodyUpdateContext& inContext, uint inStartIndex, uint inEndIndex);

	// Enforce all edge constraints
	void								ApplyEdgeConstraints(const SoftBodyUpdateContext& inContext, uint inStartIndex, uint inEndIndex);

	// Enforce all LRA constraints
	void								ApplyLRAConstraints(uint inStartIndex, uint inEndIndex);

	// Enforce all collision constraints&  update all velocities according the XPBD algorithm
	void								ApplyCollisionConstraintsAndUpdateVelocities(const SoftBodyUpdateContext& inContext);

	// Update the state of the soft body (position, velocity, bounds)
	void								UpdateSoftBodyState(SoftBodyUpdateContext& ioContext, const PhysicsSettings& inPhysicsSettings);

	// Start the first solver iteration
	void								StartFirstIteration(SoftBodyUpdateContext& ioContext);

	// Executes tasks that need to run on the start of an iteration (i.e. the stuff that can't run in parallel)
	void								StartNextIteration(const SoftBodyUpdateContext& ioContext);

	// Helper function for ParallelUpdate that works on batches of collision planes
	EStatus								ParallelDetermineCollisionPlanes(SoftBodyUpdateContext& ioContext);

	// Helper function for ParallelUpdate that works on sensor collisions
	EStatus								ParallelDetermineSensorCollisions(SoftBodyUpdateContext& ioContext);

	// Helper function for ParallelUpdate that works on batches of constraints
	EStatus								ParallelApplyConstraints(SoftBodyUpdateContext& ioContext, const PhysicsSettings& inPhysicsSettings);

	// Helper function to update a single group of constraints
	void								ProcessGroup(const SoftBodyUpdateContext& ioContext, uint inGroupIndex);

	// Returns 6 times the volume of the soft body
	float								GetVolumeTimesSix() const;

#ifndef MOSS_DEBUG_RENDERER
	// Helper function to draw constraints
	template <typename GetEndIndex, typename DrawConstraint>
		inline void						DrawConstraints(ESoftBodyConstraintColor inConstraintColor, const GetEndIndex& inGetEndIndex, const DrawConstraint& inDrawConstraint, ColorArg inBaseColor) const;

	RMat44								mSkinStateTransform = RMat44::sIdentity();	// The matrix that transforms mSkinState to world space
#endif // MOSS_DEBUG_RENDERER

	RefConst<SoftBodySharedSettings>	mSettings;									// Configuration of the particles and constraints
	TArray<Vertex>						mVertices;									// Current state of all vertices in the simulation
	TArray<CollidingShape>				mCollidingShapes;							// List of colliding shapes retrieved during the last update
	TArray<CollidingSensor>				mCollidingSensors;							// List of colliding sensors retrieved during the last update
	TArray<SkinState>					mSkinState;									// List of skinned positions (1-on-1 with mVertices but only those that are used by the skinning constraints are filled in)
	AABox								mLocalBounds;								// Bounding box of all vertices
	AABox								mLocalPredictedBounds;						// Predicted bounding box for all vertices using extrapolation of velocity by last step delta time
	uint32								mNumIterations;								// Number of solver iterations
	uint								mNumSensors;								// Workaround for TSAN false positive: store mCollidingSensors.size() in a separate variable.
	float								mPressure;									// n* R* T, amount of substance* ideal gas constant* absolute temperature, see https://en.wikipedia.org/wiki/Pressure
	float								mSkinnedMaxDistanceMultiplier = 1.0f;		// Multiplier applied to Skinned::mMaxDistance to allow tightening or loosening of the skin constraints
	bool								mUpdatePosition;							// Update the position of the body while simulating (set to false for something that is attached to the static world)
	atomic<bool>						mNeedContactCallback = false;				// True if the soft body has collided with anything in the last update
	bool								mEnableSkinConstraints = true;				// If skin constraints are enabled
	bool								mSkinStatePreviousPositionValid = false;	// True if the skinning was updated in the last update so that the previous position of the skin state is valid
};

class MOSS_EXPORT SoftBodySharedSettings : public RefTarget<SoftBodySharedSettings>
{
	MOSS_DECLARE_SERIALIZABLE_NON_VIRTUAL(MOSS_EXPORT, SoftBodySharedSettings)

public:
	// Which type of bend constraint should be created
	enum class EBendType
	{
		None,														// No bend constraints will be created
		Distance,													// A simple distance constraint
		Dihedral,													// A dihedral bend constraint (most expensive, but also supports triangles that are initially not in the same plane)
	};

	// The type of long range attachment constraint to create
	enum class ELRAType
	{
		None,														// Don't create a LRA constraint
		EuclideanDistance,											// Create a LRA constraint based on Euclidean distance between the closest kinematic vertex and this vertex
		GeodesicDistance,											// Create a LRA constraint based on the geodesic distance between the closest kinematic vertex and this vertex (follows the edge constraints)
	};

	// Per vertex attributes used during the CreateConstraints function.
	// For an edge or shear constraint, the compliance is averaged between the two attached vertices.
	// For a bend constraint, the compliance is averaged between the two vertices on the shared edge.
	struct MOSS_EXPORT VertexAttributes
	{
		// Constructor
						VertexAttributes() = default;
						VertexAttributes(float inCompliance, float inShearCompliance, float inBendCompliance, ELRAType inLRAType = ELRAType::None, float inLRAMaxDistanceMultiplier = 1.0f) : mCompliance(inCompliance), mShearCompliance(inShearCompliance), mBendCompliance(inBendCompliance), mLRAType(inLRAType), mLRAMaxDistanceMultiplier(inLRAMaxDistanceMultiplier) { }

		float			mCompliance = 0.0f;							// The compliance of the normal edges. Set to FLT_MAX to disable regular edges for any edge involving this vertex.
		float			mShearCompliance = 0.0f;					// The compliance of the shear edges. Set to FLT_MAX to disable shear edges for any edge involving this vertex.
		float			mBendCompliance = FLT_MAX;					// The compliance of the bend edges. Set to FLT_MAX to disable bend edges for any bend constraint involving this vertex.
		ELRAType		mLRAType = ELRAType::None;					// The type of long range attachment constraint to create.
		float			mLRAMaxDistanceMultiplier = 1.0f;			// Multiplier for the max distance of the LRA constraint, e.g. 1.01 means the max distance is 1% longer than the calculated distance in the rest pose.
	};

	// Automatically create constraints based on the faces of the soft body
	// @param inVertexAttributes A list of attributes for each vertex (1-on-1 with mVertices, note that if the list is smaller than mVertices the last element will be repeated). This defines the properties of the constraints that are created.
	// @param inVertexAttributesLength The length of inVertexAttributes
	// @param inBendType The type of bend constraint to create
	// @param inAngleTolerance Shear edges are created when two connected triangles form a quad (are roughly in the same plane and form a square with roughly 90 degree angles). This defines the tolerance (in radians).
	void				CreateConstraints(const VertexAttributes*inVertexAttributes, uint inVertexAttributesLength, EBendType inBendType = EBendType::Distance, float inAngleTolerance = DegreesToRadians(8.0f));

	// Calculate the initial lengths of all springs of the edges of this soft body (if you use CreateConstraint, this is already done)
	void				CalculateEdgeLengths();

	// Calculate the max lengths for the long range attachment constraints based on Euclidean distance (if you use CreateConstraints, this is already done)
	// @param inMaxDistanceMultiplier Multiplier for the max distance of the LRA constraint, e.g. 1.01 means the max distance is 1% longer than the calculated distance in the rest pose.
	void				CalculateLRALengths(float inMaxDistanceMultiplier = 1.0f);

	// Calculate the constants for the bend constraints (if you use CreateConstraints, this is already done)
	void				CalculateBendConstraintConstants();

	// Calculates the initial volume of all tetrahedra of this soft body
	void				CalculateVolumeConstraintVolumes();

	// Calculate information needed to be able to calculate the skinned constraint normals at run-time
	void				CalculateSkinnedConstraintNormals();

	// Information about the optimization of the soft body, the indices of certain elements may have changed.
	class OptimizationResults
	{
	public:
		TArray<uint>		mEdgeRemap;									// Maps old edge index to new edge index
		TArray<uint>		mLRARemap;									// Maps old LRA index to new LRA index
		TArray<uint>		mDihedralBendRemap;							// Maps old dihedral bend index to new dihedral bend index
		TArray<uint>		mVolumeRemap;								// Maps old volume constraint index to new volume constraint index
		TArray<uint>		mSkinnedRemap;								// Maps old skinned constraint index to new skinned constraint index
	};

	// Optimize the soft body settings for simulation. This will reorder constraints so they can be executed in parallel.
	void				Optimize(OptimizationResults& outResults);

	// Optimize the soft body settings without results
	void				Optimize()									{ OptimizationResults results; Optimize(results); }

	// Clone this object
	Ref<SoftBodySharedSettings> Clone() const;

	// Saves the state of this object in binary form to inStream. Doesn't store the material list.
	void				SaveBinaryState(StreamOut& inStream) const;

	// Restore the state of this object from inStream. Doesn't restore the material list.
	void				RestoreBinaryState(StreamIn& inStream);

	using SharedSettingsToIDMap = StreamUtils::ObjectToIDMap<SoftBodySharedSettings>;
	using IDToSharedSettingsMap = StreamUtils::IDToObjectMap<SoftBodySharedSettings>;
	using MaterialToIDMap = StreamUtils::ObjectToIDMap<PhysicsMaterial>;
	using IDToMaterialMap = StreamUtils::IDToObjectMap<PhysicsMaterial>;

	// Save this shared settings and its materials. Pass in an empty map ioSettingsMap / ioMaterialMap or reuse the same map while saving multiple settings objects to the same stream in order to avoid writing duplicates.
	void				SaveWithMaterials(StreamOut& inStream, SharedSettingsToIDMap& ioSettingsMap, MaterialToIDMap& ioMaterialMap) const;

	using SettingsResult = Result<Ref<SoftBodySharedSettings>>;

	// Restore a shape and materials. Pass in an empty map in ioSettingsMap / ioMaterialMap or reuse the same map while reading multiple settings objects from the same stream in order to restore duplicates.
	static SettingsResult sRestoreWithMaterials(StreamIn& inStream, IDToSharedSettingsMap& ioSettingsMap, IDToMaterialMap& ioMaterialMap);

	// Create a cube. This can be used to create a simple soft body for testing purposes.
	// It will contain edge constraints, volume constraints and faces.
	// @param inGridSize Number of points along each axis
	// @param inGridSpacing Distance between points
	static Ref<SoftBodySharedSettings> sCreateCube(uint inGridSize, float inGridSpacing);

	// A vertex is a particle, the data in this structure is only used during creation of the soft body and not during simulation
	struct MOSS_EXPORT Vertex
	{
		MOSS_DECLARE_SERIALIZABLE_NON_VIRTUAL(MOSS_EXPORT, Vertex)

		// Constructor
						Vertex() = default;
						Vertex(const Float3& inPosition, const Float3& inVelocity = Float3(0, 0, 0), float inInvMass = 1.0f) : mPosition(inPosition), mVelocity(inVelocity), mInvMass(inInvMass) { }

		Float3			mPosition { 0, 0, 0 };						// Initial position of the vertex
		Float3			mVelocity { 0, 0, 0 };						// Initial velocity of the vertex
		float			mInvMass = 1.0f;							// Initial inverse of the mass of the vertex
	};

	// A face defines the surface of the body
	struct MOSS_EXPORT Face
	{
		MOSS_DECLARE_SERIALIZABLE_NON_VIRTUAL(MOSS_EXPORT, Face)

		// Constructor
						Face() = default;
						Face(uint32 inVertex1, uint32 inVertex2, uint32 inVertex3, uint32 inMaterialIndex = 0) : mVertex { inVertex1, inVertex2, inVertex3 }, mMaterialIndex(inMaterialIndex) { }

		// Check if this is a degenerate face (a face which points to the same vertex twice)
		bool			IsDegenerate() const						{ return mVertex[0] == mVertex[1] || mVertex[0] == mVertex[2] || mVertex[1] == mVertex[2]; }

		uint32			mVertex[3];									// Indices of the vertices that form the face
		uint32			mMaterialIndex = 0;							// Index of the material of the face in SoftBodySharedSettings::mMaterials
	};

	// An edge keeps two vertices at a constant distance using a spring: |x1 - x2| = rest length
	struct MOSS_EXPORT Edge
	{
		MOSS_DECLARE_SERIALIZABLE_NON_VIRTUAL(MOSS_EXPORT, Edge)

		// Constructor
						Edge() = default;
						Edge(uint32 inVertex1, uint32 inVertex2, float inCompliance = 0.0f) : mVertex { inVertex1, inVertex2 }, mCompliance(inCompliance) { }

		// Return the lowest vertex index of this constraint
		uint32			GetMinVertexIndex() const					{ return min(mVertex[0], mVertex[1]); }

		uint32			mVertex[2];									// Indices of the vertices that form the edge
		float			mRestLength = 1.0f;							// Rest length of the spring
		float			mCompliance = 0.0f;							// Inverse of the stiffness of the spring
	};

	/**
	* A dihedral bend constraint keeps the angle between two triangles constant along their shared edge.
	*
	*        x2
	*       /  \
	*      / t0 \
	*     x0----x1
	*      \ t1 /
	*       \  /
	*        x3
	*
	* x0..x3 are the vertices, t0 and t1 are the triangles that share the edge x0..x1
	*
	* Based on:
	* - "Position Based Dynamics" - Matthias Muller et al.
	* - "Strain Based Dynamics" - Matthias Muller et al.
	* - "Simulation of Clothing with Folds and Wrinkles" - R. Bridson et al.
	*/
	struct MOSS_EXPORT DihedralBend
	{
		MOSS_DECLARE_SERIALIZABLE_NON_VIRTUAL(MOSS_EXPORT, DihedralBend)

		// Constructor
						DihedralBend() = default;
						DihedralBend(uint32 inVertex1, uint32 inVertex2, uint32 inVertex3, uint32 inVertex4, float inCompliance = 0.0f) : mVertex { inVertex1, inVertex2, inVertex3, inVertex4 }, mCompliance(inCompliance) { }

		// Return the lowest vertex index of this constraint
		uint32			GetMinVertexIndex() const					{ return min(min(mVertex[0], mVertex[1]), min(mVertex[2], mVertex[3])); }

		uint32			mVertex[4];									// Indices of the vertices of the 2 triangles that share an edge (the first 2 vertices are the shared edge)
		float			mCompliance = 0.0f;							// Inverse of the stiffness of the constraint
		float			mInitialAngle = 0.0f;						// Initial angle between the normals of the triangles (pi - dihedral angle).
	};

	// Volume constraint, keeps the volume of a tetrahedron constant
	struct MOSS_EXPORT Volume
	{
		MOSS_DECLARE_SERIALIZABLE_NON_VIRTUAL(MOSS_EXPORT, Volume)

		// Constructor
						Volume() = default;
						Volume(uint32 inVertex1, uint32 inVertex2, uint32 inVertex3, uint32 inVertex4, float inCompliance = 0.0f) : mVertex { inVertex1, inVertex2, inVertex3, inVertex4 }, mCompliance(inCompliance) { }

		// Return the lowest vertex index of this constraint
		uint32			GetMinVertexIndex() const					{ return min(min(mVertex[0], mVertex[1]), min(mVertex[2], mVertex[3])); }

		uint32			mVertex[4];									// Indices of the vertices that form the tetrahedron
		float			mSixRestVolume = 1.0f;						// 6 times the rest volume of the tetrahedron (calculated by CalculateVolumeConstraintVolumes())
		float			mCompliance = 0.0f;							// Inverse of the stiffness of the constraint
	};

	// An inverse bind matrix take a skinned vertex from its bind pose into joint local space
	class MOSS_EXPORT InvBind {
		MOSS_DECLARE_SERIALIZABLE_NON_VIRTUAL(MOSS_EXPORT, InvBind)
	public:
		// Constructor
						InvBind() = default;
						InvBind(uint32 inJointIndex, Mat44Arg inInvBind) : mJointIndex(inJointIndex), mInvBind(inInvBind) { }

		uint32			mJointIndex = 0;							// Joint index to which this is attached
		Mat44			mInvBind = Mat44::sIdentity();				// The inverse bind matrix, this takes a vertex in its bind pose (Vertex::mPosition) to joint local space
	};

	// A joint and its skin weight
	class MOSS_EXPORT SkinWeight {
		MOSS_DECLARE_SERIALIZABLE_NON_VIRTUAL(MOSS_EXPORT, SkinWeight)
	public:
		// Constructor
		SkinWeight() = default;
		SkinWeight(uint32 inInvBindIndex, float inWeight) : mInvBindIndex(inInvBindIndex), mWeight(inWeight) { }

		uint32			mInvBindIndex = 0;							// Index in mInvBindMatrices
		float			mWeight = 0.0f;								// Weight with which it is skinned
	};

	// A constraint that skins a vertex to joints and limits the distance that the simulated vertex can travel from this vertex
	class MOSS_EXPORT Skinned {
		MOSS_DECLARE_SERIALIZABLE_NON_VIRTUAL(MOSS_EXPORT, Skinned)

	public:
		// Constructor
		Skinned() = default;
		Skinned(uint32 inVertex, float inMaxDistance, float inBackStopDistance, float inBackStopRadius) : mVertex(inVertex), mMaxDistance(inMaxDistance), mBackStopDistance(inBackStopDistance), mBackStopRadius(inBackStopRadius) { }

		// Normalize the weights so that they add up to 1
		void NormalizeWeights() {
			// Get the total weight
			float total = 0.0f;
			for (const SkinWeight& w : mWeights)
				total += w.mWeight;

			// Normalize
			if (total > 0.0f)
				for (SkinWeight& w : mWeights)
					w.mWeight /= total;
		}

		// Maximum number of skin weights
		static constexpr uint cMaxSkinWeights = 4;

		uint32			mVertex = 0;								// Index in mVertices which indicates which vertex is being skinned
		SkinWeight		mWeights[cMaxSkinWeights];					// Skin weights, the bind pose of the vertex is assumed to be stored in Vertex::mPosition. The first weight that is zero indicates the end of the list. Weights should add up to 1.
		float			mMaxDistance = FLT_MAX;						// Maximum distance that this vertex can reach from the skinned vertex, disabled when FLT_MAX. 0 when you want to hard skin the vertex to the skinned vertex.
		float			mBackStopDistance = FLT_MAX;				// Disabled if mBackStopDistance >= mMaxDistance. The faces surrounding mVertex determine an average normal. mBackStopDistance behind the vertex in the opposite direction of this normal, the back stop sphere starts. The simulated vertex will be pushed out of this sphere and it can be used to approximate the volume of the skinned mesh behind the skinned vertex.
		float			mBackStopRadius = 40.0f;					// Radius of the backstop sphere. By default this is a fairly large radius so the sphere approximates a plane.
		uint32			mNormalInfo = 0;							// Information needed to calculate the normal of this vertex, lowest 24 bit is start index in mSkinnedConstraintNormals, highest 8 bit is number of faces (generated by CalculateSkinnedConstraintNormals())
	};

	// A long range attachment constraint, this is a constraint that sets a max distance between a kinematic vertex and a dynamic vertex
	// See: "Long Range Attachments - A Method to Simulate Inextensible Clothing in Computer Games", Tae-Yong Kim, Nuttapong Chentanez and Matthias Mueller-Fischer
	class MOSS_EXPORT LRA
	{
		MOSS_DECLARE_SERIALIZABLE_NON_VIRTUAL(MOSS_EXPORT, LRA)

	public:
		// Constructor
						LRA() = default;
						LRA(uint32 inVertex1, uint32 inVertex2, float inMaxDistance) : mVertex { inVertex1, inVertex2 }, mMaxDistance(inMaxDistance) { }

		// Return the lowest vertex index of this constraint
		uint32			GetMinVertexIndex() const					{ return min(mVertex[0], mVertex[1]); }

		uint32			mVertex[2];									// The vertices that are connected. The first vertex should be kinematic, the 2nd dynamic.
		float			mMaxDistance = 0.0f;						// The maximum distance between the vertices
	};

	// Add a face to this soft body
	void				AddFace(const Face& inFace)					{ MOSS_ASSERT(!inFace.IsDegenerate()); mFaces.push_back(inFace); }

	TArray<Vertex>			mVertices;									// The list of vertices or particles of the body
	TArray<Face>			mFaces;										// The list of faces of the body
	TArray<Edge>			mEdgeConstraints;							// The list of edges or springs of the body
	TArray<DihedralBend>	mDihedralBendConstraints;					// The list of dihedral bend constraints of the body
	TArray<Volume>			mVolumeConstraints;							// The list of volume constraints of the body that keep the volume of tetrahedra in the soft body constant
	TArray<Skinned>			mSkinnedConstraints;						// The list of vertices that are constrained to a skinned vertex
	TArray<InvBind>			mInvBindMatrices;							// The list of inverse bind matrices for skinning vertices
	TArray<LRA>				mLRAConstraints;							// The list of long range attachment constraints
	PhysicsMaterialList 	mMaterials { PhysicsMaterial::sDefault };	// The materials of the faces of the body, referenced by Face::mMaterialIndex
	float					mVertexRadius = 0.0f;						// How big the particles are, can be used to push the vertices a little bit away from the surface of other bodies to prevent z-fighting

private:
	friend class SoftBodyMotionProperties;

	// Calculate the closest kinematic vertex array
	void				CalculateClosestKinematic();

	// Tracks the closest kinematic vertex
	struct ClosestKinematic {
		uint32			mVertex = 0xffffffff;						// Vertex index of closest kinematic vertex
		float			mDistance = FLT_MAX;						// Distance to the closest kinematic vertex
	};

	// Tracks the end indices of the various constraint groups
	struct UpdateGroup {
		uint			mEdgeEndIndex;								// The end index of the edge constraints in this group
		uint			mLRAEndIndex;								// The end index of the LRA constraints in this group
		uint			mDihedralBendEndIndex;						// The end index of the dihedral bend constraints in this group
		uint			mVolumeEndIndex;							// The end index of the volume constraints in this group
		uint			mSkinnedEndIndex;							// The end index of the skinned constraints in this group
	};

	TArray<ClosestKinematic> mClosestKinematic;						// The closest kinematic vertex to each vertex in mVertices
	TArray<UpdateGroup>	mUpdateGroups;								// The end indices for each group of constraints that can be updated in parallel
	TArray<uint32>		mSkinnedConstraintNormals;					// A list of indices in the mFaces array used by mSkinnedConstraints, calculated by CalculateSkinnedConstraintNormals()
};

class MOSS_API SoftBodyCreationSettings {
public:
	// Constructor
						SoftBodyCreationSettings() = default;
						SoftBodyCreationSettings(const SoftBodySharedSettings*inSettings, RVec3Arg inPosition, QuatArg inRotation, ObjectLayer inObjectLayer) : mSettings(inSettings), mPosition(inPosition), mRotation(inRotation), mObjectLayer(inObjectLayer) { }

	// Saves the state of this object in binary form to inStream. Doesn't store the shared settings nor the group filter.
	void				SaveBinaryState(StreamOut& inStream) const;

	// Restore the state of this object from inStream. Doesn't restore the shared settings nor the group filter.
	void				RestoreBinaryState(StreamIn& inStream);

	using GroupFilterToIDMap = StreamUtils::ObjectToIDMap<GroupFilter>;
	using IDToGroupFilterMap = StreamUtils::IDToObjectMap<GroupFilter>;
	using SharedSettingsToIDMap = SoftBodySharedSettings::SharedSettingsToIDMap;
	using IDToSharedSettingsMap = SoftBodySharedSettings::IDToSharedSettingsMap;
	using MaterialToIDMap = StreamUtils::ObjectToIDMap<PhysicsMaterial>;
	using IDToMaterialMap = StreamUtils::IDToObjectMap<PhysicsMaterial>;

	// Save this body creation settings, its shared settings and group filter. Pass in an empty map in ioSharedSettingsMap / ioMaterialMap / ioGroupFilterMap or reuse the same map while saving multiple shapes to the same stream in order to avoid writing duplicates.
	// Pass nullptr to ioSharedSettingsMap and ioMaterial map to skip saving shared settings and materials
	// Pass nullptr to ioGroupFilterMap to skip saving group filters
	void				SaveWithChildren(StreamOut& inStream, SharedSettingsToIDMap*ioSharedSettingsMap, MaterialToIDMap*ioMaterialMap, GroupFilterToIDMap*ioGroupFilterMap) const;

	using SBCSResult = Result<SoftBodyCreationSettings>;

	// Restore a shape, all its children and materials. Pass in an empty map in ioSharedSettingsMap / ioMaterialMap / ioGroupFilterMap or reuse the same map while reading multiple shapes from the same stream in order to restore duplicates.
	static SBCSResult	sRestoreWithChildren(StreamIn& inStream, IDToSharedSettingsMap& ioSharedSettingsMap, IDToMaterialMap& ioMaterialMap, IDToGroupFilterMap& ioGroupFilterMap);

	RefConst<SoftBodySharedSettings> mSettings;				// Defines the configuration of this soft body

	RVec3				mPosition { RVec3::sZero() };		// Initial position of the soft body
	Quat				mRotation { Quat::sIdentity() };	// Initial rotation of the soft body

	// User data value (can be used by application)
	uint64				mUserData = 0;

	//@name Collision settings
	ObjectLayer			mObjectLayer = 0;					// The collision layer this body belongs to (determines if two objects can collide)
	CollisionGroup		mCollisionGroup;					// The collision group this body belongs to (determines if two objects can collide)

	uint32				mNumIterations = 5;					// Number of solver iterations
	float				mLinearDamping = 0.1f;				// Linear damping: dv/dt = -mLinearDamping* v
	float				mMaxLinearVelocity = 500.0f;		// Maximum linear velocity that a vertex can reach (m/s)
	float				mRestitution = 0.0f;				// Restitution when colliding
	float				mFriction = 0.2f;					// Friction coefficient when colliding
	float				mPressure = 0.0f;					// n* R* T, amount of substance* ideal gas constant* absolute temperature, see https://en.wikipedia.org/wiki/Pressure
	float				mGravityFactor = 1.0f;				// Value to multiply gravity with for this body
	bool				mUpdatePosition = true;				// Update the position of the body while simulating (set to false for something that is attached to the static world)
	bool				mMakeRotationIdentity = true;		// Bake specified mRotation in the vertices and set the body rotation to identity (simulation is slightly more accurate if the rotation of a soft body is kept to identity)
	bool				mAllowSleeping = true;				// If this body can go to sleep or not
};




// Settings for constructing a rigid body
class MOSS_API BodyCreationSettings
{
	MOSS_DECLARE_SERIALIZABLE_NON_VIRTUAL(MOSS_API, BodyCreationSettings)

public:
	// Constructor
							BodyCreationSettings() = default;
							BodyCreationSettings(const ShapeSettings*inShape, RVec3Arg inPosition, QuatArg inRotation, EMotionType inMotionType, ObjectLayer inObjectLayer) : mPosition(inPosition), mRotation(inRotation), mObjectLayer(inObjectLayer), mMotionType(inMotionType), mShape(inShape) { }
							BodyCreationSettings(const Shape*inShape, RVec3Arg inPosition, QuatArg inRotation, EMotionType inMotionType, ObjectLayer inObjectLayer) : mPosition(inPosition), mRotation(inRotation), mObjectLayer(inObjectLayer), mMotionType(inMotionType), mShapePtr(inShape) { }

	// Access to the shape settings object. This contains serializable (non-runtime optimized) information about the Shape.
	const ShapeSettings*	GetShapeSettings() const										{ return mShape; }
	void					SetShapeSettings(const ShapeSettings*inShape)					{ mShape = inShape; mShapePtr = nullptr; }

	// Convert ShapeSettings object into a Shape object. This will free the ShapeSettings object and make the object ready for runtime. Serialization is no longer possible after this.
	Shape::ShapeResult		ConvertShapeSettings();

	// Access to the run-time shape object. Will convert from ShapeSettings object if needed.
	const Shape*			GetShape() const;
	void					SetShape(const Shape*inShape)									{ mShapePtr = inShape; mShape = nullptr; }

	// Check if the mass properties of this body will be calculated (only relevant for kinematic or dynamic objects that need a MotionProperties object)
	bool					HasMassProperties() const										{ return mAllowDynamicOrKinematic || mMotionType != EMotionType::Static; }

	// Calculate (or return when overridden) the mass and inertia for this body
	MassProperties			GetMassProperties() const;

	// Saves the state of this object in binary form to inStream. Doesn't store the shape nor the group filter.
	void					SaveBinaryState(StreamOut& inStream) const;

	// Restore the state of this object from inStream. Doesn't restore the shape nor the group filter.
	void					RestoreBinaryState(StreamIn& inStream);

	using GroupFilterToIDMap = StreamUtils::ObjectToIDMap<GroupFilter>;
	using IDToGroupFilterMap = StreamUtils::IDToObjectMap<GroupFilter>;
	using ShapeToIDMap = Shape::ShapeToIDMap;
	using IDToShapeMap = Shape::IDToShapeMap;
	using MaterialToIDMap = StreamUtils::ObjectToIDMap<PhysicsMaterial>;
	using IDToMaterialMap = StreamUtils::IDToObjectMap<PhysicsMaterial>;

	// Save body creation settings, its shape, materials and group filter. Pass in an empty map in ioShapeMap / ioMaterialMap / ioGroupFilterMap or reuse the same map while saving multiple shapes to the same stream in order to avoid writing duplicates.
	// Pass nullptr to ioShapeMap and ioMaterial map to skip saving shapes
	// Pass nullptr to ioGroupFilterMap to skip saving group filters
	void					SaveWithChildren(StreamOut& inStream, ShapeToIDMap*ioShapeMap, MaterialToIDMap*ioMaterialMap, GroupFilterToIDMap*ioGroupFilterMap) const;

	using BCSResult = Result<BodyCreationSettings>;

	// Restore body creation settings, its shape, materials and group filter. Pass in an empty map in ioShapeMap / ioMaterialMap / ioGroupFilterMap or reuse the same map while reading multiple shapes from the same stream in order to restore duplicates.
	static BCSResult		sRestoreWithChildren(StreamIn& inStream, IDToShapeMap& ioShapeMap, IDToMaterialMap& ioMaterialMap, IDToGroupFilterMap& ioGroupFilterMap);

	RVec3					mPosition = RVec3::sZero();										// Position of the body (not of the center of mass)
	Quat					mRotation = Quat::sIdentity();									// Rotation of the body
	Vec3					mLinearVelocity = Vec3::sZero();								// World space linear velocity of the center of mass (m/s)
	Vec3					mAngularVelocity = Vec3::sZero();								// World space angular velocity (rad/s)

	// User data value (can be used by application)
	uint64					mUserData = 0;

	//@name Collision settings
	ObjectLayer				mObjectLayer = 0;												// The collision layer this body belongs to (determines if two objects can collide)
	CollisionGroup			mCollisionGroup;												// The collision group this body belongs to (determines if two objects can collide)

	//@name Simulation properties
	EMotionType				mMotionType = EMotionType::Dynamic;								// Motion type, determines if the object is static, dynamic or kinematic
	EAllowedDOFs			mAllowedDOFs = EAllowedDOFs::All;								// Which degrees of freedom this body has (can be used to limit simulation to 2D)
	bool					mAllowDynamicOrKinematic = false;								// When this body is created as static, this setting tells the system to create a MotionProperties object so that the object can be switched to kinematic or dynamic
	bool					mIsSensor = false;												// If this body is a sensor. A sensor will receive collision callbacks, but will not cause any collision responses and can be used as a trigger volume. See description at Body::SetIsSensor.
	bool					mCollideKinematicVsNonDynamic = false;							// If kinematic objects can generate contact points against other kinematic or static objects. See description at Body::SetCollideKinematicVsNonDynamic.
	bool					mUseManifoldReduction = true;									// If this body should use manifold reduction (see description at Body::SetUseManifoldReduction)
	bool					mApplyGyroscopicForce = false;									// Set to indicate that the gyroscopic force should be applied to this body (aka Dzhanibekov effect, see https://en.wikipedia.org/wiki/Tennis_racket_theorem)
	EMotionQuality			mMotionQuality = EMotionQuality::Discrete;						// Motion quality, or how well it detects collisions when it has a high velocity
	bool					mEnhancedInternalEdgeRemoval = false;							// Set to indicate that extra effort should be made to try to remove ghost contacts (collisions with internal edges of a mesh). This is more expensive but makes bodies move smoother over a mesh with convex edges.
	bool					mAllowSleeping = true;											// If this body can go to sleep or not
	float					mFriction = 0.2f;												// Friction of the body (dimensionless number, usually between 0 and 1, 0 = no friction, 1 = friction force equals force that presses the two bodies together). Note that bodies can have negative friction but the combined friction (see PhysicsSystem::SetCombineFriction) should never go below zero.
	float					mRestitution = 0.0f;											// Restitution of body (dimensionless number, usually between 0 and 1, 0 = completely inelastic collision response, 1 = completely elastic collision response). Note that bodies can have negative restitution but the combined restitution (see PhysicsSystem::SetCombineRestitution) should never go below zero.
	float					mLinearDamping = 0.05f;											// Linear damping: dv/dt = -c* v. c must be between 0 and 1 but is usually close to 0.
	float					mAngularDamping = 0.05f;										// Angular damping: dw/dt = -c* w. c must be between 0 and 1 but is usually close to 0.
	float					mMaxLinearVelocity = 500.0f;									// Maximum linear velocity that this body can reach (m/s)
	float					mMaxAngularVelocity = 0.25f* MOSS_PI* 60.0f;					// Maximum angular velocity that this body can reach (rad/s)
	float					mGravityFactor = 1.0f;											// Value to multiply gravity with for this body
	uint					mNumVelocityStepsOverride = 0;									// Used only when this body is dynamic and colliding. Override for the number of solver velocity iterations to run, 0 means use the default in PhysicsSettings::mNumVelocitySteps. The number of iterations to use is the max of all contacts and constraints in the island.
	uint					mNumPositionStepsOverride = 0;									// Used only when this body is dynamic and colliding. Override for the number of solver position iterations to run, 0 means use the default in PhysicsSettings::mNumPositionSteps. The number of iterations to use is the max of all contacts and constraints in the island.

	//@name Mass properties of the body (by default calculated by the shape)
	EOverrideMassProperties	mOverrideMassProperties = EOverrideMassProperties::CalculateMassAndInertia; // Determines how mMassPropertiesOverride will be used
	float					mInertiaMultiplier = 1.0f;										// When calculating the inertia (not when it is provided) the calculated inertia will be multiplied by this value
	MassProperties			mMassPropertiesOverride;										// Contains replacement mass settings which override the automatically calculated values

private:
	// Collision volume for the body
	RefConst<ShapeSettings>	mShape;															// Shape settings, can be serialized. Mutually exclusive with mShapePtr
	RefConst<Shape>			mShapePtr;														// Actual shape, cannot be serialized. Mutually exclusive with mShape
};



















// Bitwise OR operator for EAllowedDOFs
constexpr EAllowedDOFs operator | (EAllowedDOFs inLHS, EAllowedDOFs inRHS)
{
	return EAllowedDOFs(uint8(inLHS) | uint8(inRHS));
}

// Bitwise AND operator for EAllowedDOFs
constexpr EAllowedDOFs operator&  (EAllowedDOFs inLHS, EAllowedDOFs inRHS)
{
	return EAllowedDOFs(uint8(inLHS)&  uint8(inRHS));
}

// Bitwise XOR operator for EAllowedDOFs
constexpr EAllowedDOFs operator ^ (EAllowedDOFs inLHS, EAllowedDOFs inRHS)
{
	return EAllowedDOFs(uint8(inLHS) ^ uint8(inRHS));
}

// Bitwise NOT operator for EAllowedDOFs
constexpr EAllowedDOFs operator ~ (EAllowedDOFs inAllowedDOFs)
{
	return EAllowedDOFs(~uint8(inAllowedDOFs));
}

// Bitwise OR assignment operator for EAllowedDOFs
constexpr EAllowedDOFs&  operator |= (EAllowedDOFs& ioLHS, EAllowedDOFs inRHS)
{
	ioLHS = ioLHS | inRHS;
	return ioLHS;
}

// Bitwise AND assignment operator for EAllowedDOFs
constexpr EAllowedDOFs&  operator& = (EAllowedDOFs& ioLHS, EAllowedDOFs inRHS)
{
	ioLHS = ioLHS&  inRHS;
	return ioLHS;
}

// Bitwise XOR assignment operator for EAllowedDOFs
constexpr EAllowedDOFs&  operator ^= (EAllowedDOFs& ioLHS, EAllowedDOFs inRHS)
{
	ioLHS = ioLHS ^ inRHS;
	return ioLHS;
}

inline EPhysicsUpdateError operator | (EPhysicsUpdateError inA, EPhysicsUpdateError inB) { return static_cast<EPhysicsUpdateError>(static_cast<uint32>(inA) | static_cast<uint32>(inB)); }

// OR operator for EPhysicsUpdateError
inline EPhysicsUpdateError operator |= (EPhysicsUpdateError& ioA, EPhysicsUpdateError inB) { ioA = ioA | inB; return ioA; }

// AND operator for EPhysicsUpdateError
inline EPhysicsUpdateError operator&  (EPhysicsUpdateError inA, EPhysicsUpdateError inB) { return static_cast<EPhysicsUpdateError>(static_cast<uint32>(inA)&  static_cast<uint32>(inB)); }


// Place Skeleton into renderer
// ===========================================================================================================
/* Skeleton*/
struct SkeletonJoint {
	const char*		name;
	const char*		parentName;
	int				parentJointIndex;
} SkeletonJoint;

MOSS_API Skeleton* Skeleton_Create(void);
MOSS_API void Skeleton_Destroy(Skeleton* skeleton);

MOSS_API uint32_t Skeleton_AddJoint(Skeleton* skeleton, const char* name);
MOSS_API uint32_t Skeleton_AddJoint2(Skeleton* skeleton, const char* name, int parentIndex);
MOSS_API uint32_t Skeleton_AddJoint3(Skeleton* skeleton, const char* name, const char* parentName);
MOSS_API int Skeleton_GetJointCount(const Skeleton* skeleton);
MOSS_API void Skeleton_GetJoint(const Skeleton* skeleton, int index, SkeletonJoint* joint);
MOSS_API int Skeleton_GetJointIndex(const Skeleton* skeleton, const char* name);
MOSS_API void Skeleton_CalculateParentJointIndices(Skeleton* skeleton);
MOSS_API bool Skeleton_AreJointsCorrectlyOrdered(const Skeleton* skeleton);


// ===========================================================================================================
using TriangleList = TArray<Triangle>;
using IndexedTriangleNoMaterialList = TArray<IndexedTriangleNoMaterial>;
using IndexedTriangleList = TArray<IndexedTriangle>;


MOSS_SUPRESS_WARNINGS_END


// Create a std::hash for IndexedTriangleNoMaterial and IndexedTriangle
MOSS_MAKE_STD_HASH(IndexedTriangleNoMaterial)
MOSS_MAKE_STD_HASH(IndexedTriangle)


#endif // MOSS_PHYSICS_H
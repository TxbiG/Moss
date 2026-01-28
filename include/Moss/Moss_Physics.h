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
 * - **Broadphase & Narrowphase Collision** — Highly optimized multi-threaded broadphase with efficient shape queries.
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
 * - **Dynamic Materials & Layers**  
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


/*
Deriving 3D Rigid Body Physics and implementing it in C/C++ (with intuitions) - https://youtu.be/4r_EvmPKOvY?si=I4PWbyXfvgudb4Cu
Building a Physics Engine with C++ and Simulating Machines - https://youtu.be/TtgS-b191V0?si=i74pfZTFMfHpZG4p
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

/* Forward declarations */
typedef struct BroadPhaseLayerInterface				BroadPhaseLayerInterface;
typedef struct ObjectVsBroadPhaseLayerFilter		ObjectVsBroadPhaseLayerFilter;
typedef struct ObjectLayerPairFilter				ObjectLayerPairFilter;
typedef struct BroadPhaseLayerFilter				BroadPhaseLayerFilter;
typedef struct ObjectLayerFilter					ObjectLayerFilter;
class BodyFilter;
class ShapeFilter;
class SimShapeFilter;
class PhysicsSystem;

/* ShapeSettings */
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
class GroupFilterTable;  /* Inherits GroupFilter */

class BodyActivationListener;
class BodyDrawFilter;

typedef struct DebugRenderer                    DebugRenderer;

/* Enums */
enum class EPhysicsUpdateError : uint32_t {
	None					= 0,			// No errors
	ManifoldCacheFull		= 1 << 0,		// The manifold cache is full, this means that the total number of contacts between bodies is too high. Some contacts were ignored. Increase inMaxContactConstraints in PhysicsSystem::Init.
	BodyPairCacheFull		= 1 << 1,		// The body pair cache is full, this means that too many bodies contacted. Some contacts were ignored. Increase inMaxBodyPairs in PhysicsSystem::Init.
	ContactConstraintsFull	= 1 << 2,		// The contact constraints buffer is full. Some contacts were ignored. Increase inMaxContactConstraints in PhysicsSystem::Init.
};

inline EPhysicsUpdateError operator | (EPhysicsUpdateError inA, EPhysicsUpdateError inB) { return static_cast<EPhysicsUpdateError>(static_cast<uint32>(inA) | static_cast<uint32>(inB)); }

// OR operator for EPhysicsUpdateError
inline EPhysicsUpdateError operator |= (EPhysicsUpdateError &ioA, EPhysicsUpdateError inB) { ioA = ioA | inB; return ioA; }

// AND operator for EPhysicsUpdateError
inline EPhysicsUpdateError operator & (EPhysicsUpdateError inA, EPhysicsUpdateError inB) { return static_cast<EPhysicsUpdateError>(static_cast<uint32>(inA) & static_cast<uint32>(inB)); }

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

enum class EConstraintSubType {
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

enum class EGroundState { 
	OnGround = 0, 
	OnSteepGround = 1, 
	NotSupported = 2, 
	InAir = 3, 
};
enum class ETransmissionMode : uint8 { 
	Auto, 
	Manual 
};

enum class EActivation : uint8 { 
	Activate, 
	DontActivate 
};
enum class EConstraintType : uint8 { 
	Constraint, 
	TwoBodyConstraint 
};
enum class EConstraintSpace : uint8 { 
	LocalToBodyCOM, 
	WorldSpace 
};
enum class EMotionQuality : uint8 { 
	Discrete, 
	LinearCast 
};
enum class EOverrideMassProperties { 
	CalculateMassAndInertia, 
	CalculateInertia, 
	MassAndInertiaProvided 
};
enum class EBackFaceMode : uint8 { 
	IgnoreBackFaces, 
	CollideWithBackFaces 
};
enum class EActiveEdgeMode : uint8 { 
	CollideOnlyWithActive, 
	CollideWithAll 
};
enum class ECollectFacesMode : uint8 { 
	CollectFaces, 
	NoFaces 
};
enum class EMotorState : uint8 { 
	Off, 
	Velocity, 
	Position 
};
enum class ESwingType : uint8 { 
	Cone, 
	Pyramid 
};

/*! @brief  Accept this contact or Reject this contact  */
enum class SoftBodyValidateResult { 
	AcceptContact, 
	RejectContact 
};

enum class ECollisionCollectorType { 
	AllHit = 0, 
	AllHitSorted = 1, 
	ClosestHit = 2, 
	AnyHit = 3 
};

enum class ETrackSide : uint8 { 
	Left, 
	Right, 
	Num 
};

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

enum class ESpringMode : uint8 { 
	FrequencyAndDamping, 
	StiffnessAndDamping 
};

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

struct SoftBodyContactSettings {
	float							mInvMassScale1 = 1.0f;				// Scale factor for the inverse mass of the soft body (0 = infinite mass, 1 = use original mass, 2 = body has half the mass). For the same contact pair, you should strive to keep the value the same over time.
	float							mInvMassScale2 = 1.0f;				// Scale factor for the inverse mass of the other body (0 = infinite mass, 1 = use original mass, 2 = body has half the mass). For the same contact pair, you should strive to keep the value the same over time.
	float							mInvInertiaScale2 = 1.0f;			// Scale factor for the inverse inertia of the other body (usually same as mInvMassScale2)
	bool							mIsSensor;							// If the contact should be treated as a sensor vs body contact (no collision response)
};

class Triangle {
public:
	MOSS_OVERRIDE_NEW_DELETE

	// Constructor
	Triangle() = default;
	Triangle(const Float3 &inV1, const Float3 &inV2, const Float3 &inV3, uint32 inMaterialIndex = 0, uint32 inUserData = 0) : mV { inV1, inV2, inV3 }, mMaterialIndex(inMaterialIndex), mUserData(inUserData) { }
	Triangle(Vec3Arg inV1, Vec3Arg inV2, Vec3Arg inV3, uint32 inMaterialIndex = 0, uint32 inUserData = 0) : mMaterialIndex(inMaterialIndex), mUserData(inUserData) { inV1.StoreFloat3(&mV[0]); inV2.StoreFloat3(&mV[1]); inV3.StoreFloat3(&mV[2]); }

	// Get center of triangle
	Vec3 GetCentroid() const { return (Vec3::sLoadFloat3Unsafe(mV[0]) + Vec3::sLoadFloat3Unsafe(mV[1]) + Vec3::sLoadFloat3Unsafe(mV[2])) * (1.0f / 3.0f); }

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
	constexpr IndexedTriangleNoMaterial(uint32 inI1, uint32 inI2, uint32 inI3) : mIdx { inI1, inI2, inI3 } { }

	// Check if two triangles are identical
	bool operator == (const IndexedTriangleNoMaterial &inRHS) const {
		return mIdx[0] == inRHS.mIdx[0] && mIdx[1] == inRHS.mIdx[1] && mIdx[2] == inRHS.mIdx[2];
	}

	// Check if two triangles are equivalent (using the same vertices)
	bool IsEquivalent(const IndexedTriangleNoMaterial &inRHS) const {
		return (mIdx[0] == inRHS.mIdx[0] && mIdx[1] == inRHS.mIdx[1] && mIdx[2] == inRHS.mIdx[2])
			|| (mIdx[0] == inRHS.mIdx[1] && mIdx[1] == inRHS.mIdx[2] && mIdx[2] == inRHS.mIdx[0])
			|| (mIdx[0] == inRHS.mIdx[2] && mIdx[1] == inRHS.mIdx[0] && mIdx[2] == inRHS.mIdx[1]);
	}

	// Check if two triangles are opposite (using the same vertices but in opposing order)
	bool IsOpposite(const IndexedTriangleNoMaterial &inRHS) const {
		return (mIdx[0] == inRHS.mIdx[0] && mIdx[1] == inRHS.mIdx[2] && mIdx[2] == inRHS.mIdx[1])
			|| (mIdx[0] == inRHS.mIdx[1] && mIdx[1] == inRHS.mIdx[0] && mIdx[2] == inRHS.mIdx[2])
			|| (mIdx[0] == inRHS.mIdx[2] && mIdx[1] == inRHS.mIdx[1] && mIdx[2] == inRHS.mIdx[0]);
	}

	// Check if triangle is degenerate
	bool IsDegenerate(const VertexList &inVertices) const {
		Vec3 v0(inVertices[mIdx[0]]);
		Vec3 v1(inVertices[mIdx[1]]);
		Vec3 v2(inVertices[mIdx[2]]);

		return (v1 - v0).Cross(v2 - v0).IsNearZero();
	}

	// Rotate the vertices so that the second vertex becomes first etc. This does not change the represented triangle.
	void Rotate() {
		uint32 tmp = mIdx[0];
		mIdx[0] = mIdx[1];
		mIdx[1] = mIdx[2];
		mIdx[2] = tmp;
	}

	// Get center of triangle
	Vec3 GetCentroid(const VertexList &inVertices) const {
		return (Vec3(inVertices[mIdx[0]]) + Vec3(inVertices[mIdx[1]]) + Vec3(inVertices[mIdx[2]])) / 3.0f;
	}

	// Get the hash value of this structure
	uint64 GetHash() const {
		static_assert(sizeof(IndexedTriangleNoMaterial) == 3 * sizeof(uint32), "Class should have no padding");
		return HashBytes(this, sizeof(IndexedTriangleNoMaterial));
	}

	uint32			mIdx[3];
};

// Triangle with 32-bit indices and material index
class IndexedTriangle : public IndexedTriangleNoMaterial {
public:
	using IndexedTriangleNoMaterial::IndexedTriangleNoMaterial;

	// Constructor
	constexpr		IndexedTriangle(uint32 inI1, uint32 inI2, uint32 inI3, uint32 inMaterialIndex, uint inUserData = 0) : IndexedTriangleNoMaterial(inI1, inI2, inI3), mMaterialIndex(inMaterialIndex), mUserData(inUserData) { }

	// Check if two triangles are identical
	bool operator == (const IndexedTriangle &inRHS) const {
		return mMaterialIndex == inRHS.mMaterialIndex && mUserData == inRHS.mUserData && IndexedTriangleNoMaterial::operator==(inRHS);
	}

	// Rotate the vertices so that the lowest vertex becomes the first. This does not change the represented triangle.
	IndexedTriangle	GetLowestIndexFirst() const {
		if (mIdx[0] < mIdx[1]) {
			if (mIdx[0] < mIdx[2])
				return IndexedTriangle(mIdx[0], mIdx[1], mIdx[2], mMaterialIndex, mUserData); // 0 is smallest
			else
				return IndexedTriangle(mIdx[2], mIdx[0], mIdx[1], mMaterialIndex, mUserData); // 2 is smallest
		}
		else {
			if (mIdx[1] < mIdx[2])
				return IndexedTriangle(mIdx[1], mIdx[2], mIdx[0], mMaterialIndex, mUserData); // 1 is smallest
			else
				return IndexedTriangle(mIdx[2], mIdx[0], mIdx[1], mMaterialIndex, mUserData); // 2 is smallest
		}
	}

	// Get the hash value of this structure
	uint64			GetHash() const
	{
		static_assert(sizeof(IndexedTriangle) == 5 * sizeof(uint32), "Class should have no padding");
		return HashBytes(this, sizeof(IndexedTriangle));
	}

	uint32			mMaterialIndex = 0;
	uint32			mUserData = 0;				// User data that can be used for anything by the application, e.g. for tracking the original index of the triangle
};


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

	// A factor that determines the accuracy of the penetration depth calculation. If the change of the squared distance is less than tolerance * current_penetration_depth^2 the algorithm will terminate. (unit: dimensionless)
	float						penetrationTolerance/* = DEFAULT_PENETRATION_TOLERANCE*/;

	// When mActiveEdgeMode is CollideOnlyWithActive a movement direction can be provided. When hitting an inactive edge, the system will select the triangle normal as penetration depth only if it impedes the movement less than with the calculated penetration depth.
	Vec3					activeEdgeMovementDirection/* = Vec3::sZero()*/;
} CollideSettingsBase;

/* CollideShapeSettings */
typedef struct CollideShapeSettings {
	CollideSettingsBase     base;    /* Inherits CollideSettingsBase */
	// When > 0 contacts in the vicinity of the query shape can be found. All nearest contacts that are not further away than this distance will be found (unit: meter)
	float						maxSeparationDistance/* = 0.0f*/;

	// How backfacing triangles should be treated
	EBackFaceMode			backFaceMode/* = BackFaceMode_IgnoreBackFaces*/;
} CollideShapeSettings;

/* ShapeCastSettings */
typedef struct ShapeCastSettings {
	CollideSettingsBase     base;    /* Inherits CollideSettingsBase */

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

class SpringSettings {
public:
	// Constructor
	SpringSettings() = default;
	SpringSettings(const SpringSettings &) = default;
	SpringSettings& operator = (const SpringSettings &) = default;
	SpringSettings(ESpringMode inMode, float inFrequencyOrStiffness, float inDamping) : mMode(inMode), mFrequency(inFrequencyOrStiffness), mDamping(inDamping) { }

	// Saves the contents of the spring settings in binary form to inStream.
	void SaveBinaryState(StreamOut &inStream) const;

	// Restores contents from the binary stream inStream.
	void RestoreBinaryState(StreamIn &inStream);

	// Check if the spring has a valid frequency / stiffness, if not the spring will be hard
	inline bool	HasStiffness() const { return frequency > 0.0f; }

	// Selects the way in which the spring is defined
	// If the mode is StiffnessAndDamping then mFrequency becomes the stiffness (k) and mDamping becomes the damping ratio (c) in the spring equation F = -k * x - c * v. Otherwise the properties are as documented.
	ESpringMode mode = ESpringMode::FrequencyAndDamping;

	union {
		// Valid when mSpringMode = ESpringMode::FrequencyAndDamping.
		// If mFrequency > 0 the constraint will be soft and mFrequency specifies the oscillation frequency in Hz.
		// If mFrequency <= 0, mDamping is ignored and the constraint will have hard limits (as hard as the time step / the number of velocity / position solver steps allows).
		float frequency = 0.0f;

		// Valid when mSpringMode = ESpringMode::StiffnessAndDamping.
		// If mStiffness > 0 the constraint will be soft and mStiffness specifies the stiffness (k) in the spring equation F = -k * x - c * v for a linear or T = -k * theta - c * w for an angular spring.
		// If mStiffness <= 0, mDamping is ignored and the constraint will have hard limits (as hard as the time step / the number of velocity / position solver steps allows).
		//
		// Note that stiffness values are large numbers. To calculate a ballpark value for the needed stiffness you can use:
		// force = stiffness * delta_spring_length = mass * gravity <=> stiffness = mass * gravity / delta_spring_length.
		// So if your object weighs 1500 kg and the spring compresses by 2 meters, you need a stiffness in the order of 1500 * 9.81 / 2 ~ 7500 N/m.
		float stiffness;
	};

	// When mSpringMode = ESpringMode::FrequencyAndDamping mDamping is the damping ratio (0 = no damping, 1 = critical damping).
	// When mSpringMode = ESpringMode::StiffnessAndDamping mDamping is the damping (c) in the spring equation F = -k * x - c * v for a linear or T = -k * theta - c * w for an angular spring.
	// Note that if you set mDamping = 0, you will not get an infinite oscillation. Because we integrate physics using an explicit Euler scheme, there is always energy loss.
	// This is done to keep the simulation from exploding, because with a damping of 0 and even the slightest rounding error, the oscillation could become bigger and bigger until the simulation explodes.
	float damping = 0.0f;
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

typedef void(API_CALL* TraceFunc)(const char* message);
typedef bool(API_CALL* AssertFailureFunc)(const char* expression, const char* message, const char* file, uint32_t line);

typedef void JobFunction(void* arg);
typedef void QueueJobCallback(void* context, JobFunction* job, void* arg);
typedef void QueueJobsCallback(void* context, JobFunction* job, void** args, uint32_t count);



/* Structs free members */
MOSS_API void CollideShapeResult_FreeMembers(CollideShapeResult* result);
MOSS_API void CollisionEstimationResult_FreeMembers(CollisionEstimationResult* result);

/* BroadPhaseLayerInterface */
MOSS_API BroadPhaseLayerInterface* BroadPhaseLayerInterfaceMask_Create(uint32_t numBroadPhaseLayers);
MOSS_API void BroadPhaseLayerInterfaceMask_ConfigureLayer(BroadPhaseLayerInterface* bpInterface, BroadPhaseLayer broadPhaseLayer, uint32_t groupsToInclude, uint32_t groupsToExclude);

MOSS_API BroadPhaseLayerInterface* BroadPhaseLayerInterfaceTable_Create(uint32_t numObjectLayers, uint32_t numBroadPhaseLayers);
MOSS_API void BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(BroadPhaseLayerInterface* bpInterface, ObjectLayer objectLayer, BroadPhaseLayer broadPhaseLayer);

/* ObjectLayerPairFilter */
MOSS_API ObjectLayerPairFilter* ObjectLayerPairFilterMask_Create(void);
MOSS_API ObjectLayer ObjectLayerPairFilterMask_GetObjectLayer(uint32_t group, uint32_t mask);
MOSS_API uint32_t ObjectLayerPairFilterMask_GetGroup(ObjectLayer layer);
MOSS_API uint32_t ObjectLayerPairFilterMask_GetMask(ObjectLayer layer);

MOSS_API ObjectLayerPairFilter* ObjectLayerPairFilterTable_Create(uint32_t numObjectLayers);
MOSS_API void ObjectLayerPairFilterTable_DisableCollision(ObjectLayerPairFilter* objectFilter, ObjectLayer layer1, ObjectLayer layer2);
MOSS_API void ObjectLayerPairFilterTable_EnableCollision(ObjectLayerPairFilter* objectFilter, ObjectLayer layer1, ObjectLayer layer2);
MOSS_API bool ObjectLayerPairFilterTable_ShouldCollide(ObjectLayerPairFilter* objectFilter, ObjectLayer layer1, ObjectLayer layer2);

/* ObjectVsBroadPhaseLayerFilter */
MOSS_API ObjectVsBroadPhaseLayerFilter* ObjectVsBroadPhaseLayerFilterMask_Create(const BroadPhaseLayerInterface* broadPhaseLayerInterface);

MOSS_API ObjectVsBroadPhaseLayerFilter* ObjectVsBroadPhaseLayerFilterTable_Create(
	BroadPhaseLayerInterface* broadPhaseLayerInterface, uint32_t numBroadPhaseLayers,
	ObjectLayerPairFilter* objectLayerPairFilter, uint32_t numObjectLayers);

MOSS_API void DrawSettings_InitDefault(DrawSettings* settings);



/* PhysicsStepListener */
class MOSS_API PhysicsStepListener {
public:
	// Ensure virtual destructor
	virtual					~PhysicsStepListener() = default;

	// Called before every simulation step (received inCollisionSteps times for every PhysicsSystem::Update(...) call)
	// This is called while all body and constraint mutexes are locked. You can read/write bodies and constraints but not add/remove them.
	// Multiple listeners can be executed in parallel and it is the responsibility of the listener to avoid race conditions.
	// The best way to do this is to have each step listener operate on a subset of the bodies and constraints
	// and making sure that these bodies and constraints are not touched by any other step listener.
	// Note that this function is not called if there aren't any active bodies or when the physics system is updated with 0 delta time.
	virtual void			OnStep(const PhysicsStepListenerContext &inContext) = 0;
};

/* ContactManifold */
MOSS_API void ContactManifold_GetWorldSpaceNormal(const ContactManifold* manifold, Vec3* result);
MOSS_API float ContactManifold_GetPenetrationDepth(const ContactManifold* manifold);
MOSS_API SubShapeID ContactManifold_GetSubShapeID1(const ContactManifold* manifold);
MOSS_API SubShapeID ContactManifold_GetSubShapeID2(const ContactManifold* manifold);
MOSS_API uint32_t ContactManifold_GetPointCount(const ContactManifold* manifold);
MOSS_API void ContactManifold_GetWorldSpaceContactPointOn1(const ContactManifold* manifold, uint32_t index, Vec3* result);
MOSS_API void ContactManifold_GetWorldSpaceContactPointOn2(const ContactManifold* manifold, uint32_t index, Vec3* result);

/* CollisionDispatch */
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


// Place Skeleton into renderer
// ===========================================================================================================
/* Skeleton */
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


// These need to be kept vvvvvvv

/// Take a list of triangles and get the unique set of vertices and use them to create indexed triangles.
/// Vertices that are less than inVertexWeldDistance apart will be combined to a single vertex.
MOSS_EXPORT void Indexify(const TriangleList &inTriangles, VertexList &outVertices, IndexedTriangleList &outTriangles, float inVertexWeldDistance = 1.0e-4f);

/// Take a list of indexed triangles and unpack them
MOSS_EXPORT void Deindexify(const VertexList &inVertices, const IndexedTriangleList &inTriangles, TriangleList &outTriangles);

// ===========================================================================================================

using TriangleList = TArray<Triangle>;
using IndexedTriangleNoMaterialList = TArray<IndexedTriangleNoMaterial>;
using IndexedTriangleList = TArray<IndexedTriangle>;


MOSS_SUPRESS_WARNINGS_END


// Create a std::hash for IndexedTriangleNoMaterial and IndexedTriangle
MOSS_MAKE_STD_HASH(IndexedTriangleNoMaterial)
MOSS_MAKE_STD_HASH(IndexedTriangle)


#endif // MOSS_PHYSICS_H
////////////////////////////////////////
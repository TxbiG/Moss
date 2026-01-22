#ifndef JOLT_BODY3D_H
#define JOLT_BODY3D_H


#include <Moss/Core/HashCombine.h>
#include <Moss/Core/Mutex.h>
#include <Moss/Core/MutexArray.h>


MOSS_NAMESPACE_BEGIN

enum class EBodyType : uint8 { 
	Rigid, 
	Soft 
};
enum class EMotionType : uint8 { 
	Static, 
	Kinematic, 
	Dynamic 
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

// Bitwise OR operator for EAllowedDOFs
constexpr EAllowedDOFs operator | (EAllowedDOFs inLHS, EAllowedDOFs inRHS) { return EAllowedDOFs(uint8(inLHS) | uint8(inRHS)); }

// Bitwise AND operator for EAllowedDOFs
constexpr EAllowedDOFs operator & (EAllowedDOFs inLHS, EAllowedDOFs inRHS) { return EAllowedDOFs(uint8(inLHS) & uint8(inRHS)); }

// Bitwise XOR operator for EAllowedDOFs
constexpr EAllowedDOFs operator ^ (EAllowedDOFs inLHS, EAllowedDOFs inRHS) { return EAllowedDOFs(uint8(inLHS) ^ uint8(inRHS)); }

// Bitwise NOT operator for EAllowedDOFs
constexpr EAllowedDOFs operator ~ (EAllowedDOFs inAllowedDOFs) { return EAllowedDOFs(~uint8(inAllowedDOFs)); }

// Bitwise OR assignment operator for EAllowedDOFs
constexpr EAllowedDOFs & operator |= (EAllowedDOFs &ioLHS, EAllowedDOFs inRHS) { ioLHS = ioLHS | inRHS; return ioLHS; }

// Bitwise AND assignment operator for EAllowedDOFs
constexpr EAllowedDOFs & operator &= (EAllowedDOFs &ioLHS, EAllowedDOFs inRHS) { ioLHS = ioLHS & inRHS; return ioLHS; }

// Bitwise XOR assignment operator for EAllowedDOFs
constexpr EAllowedDOFs & operator ^= (EAllowedDOFs &ioLHS, EAllowedDOFs inRHS) { ioLHS = ioLHS ^ inRHS; return ioLHS; }

class MOSS_EXPORT BodyAccess {
public:
	/// Access rules, used to detect race conditions during simulation
	enum class EAccess : uint8 {
		None		= 0,
		Read		= 1,
		ReadWrite	= 3,
	};

	/// Grant a scope specific access rights on the current thread
	class Grant {
	public:
		inline Grant(EAccess inVelocity, EAccess inPosition) {
			EAccess &velocity = sVelocityAccess();
			EAccess &position = sPositionAccess();

			MOSS_ASSERT(velocity == EAccess::ReadWrite);
			MOSS_ASSERT(position == EAccess::ReadWrite);

			velocity = inVelocity;
			position = inPosition;
		}

		inline ~Grant() {
			sVelocityAccess() = EAccess::ReadWrite;
			sPositionAccess() = EAccess::ReadWrite;
		}
	};

	/// Check if we have permission
	static inline bool					sCheckRights(EAccess inRights, EAccess inDesiredRights) {
		return (uint8(inRights) & uint8(inDesiredRights)) == uint8(inDesiredRights);
	}

	/// Access to read/write velocities
	static inline EAccess &				sVelocityAccess() {
		static thread_local EAccess sAccess = BodyAccess::EAccess::ReadWrite;
		return sAccess;
	}

	/// Access to read/write positions
	static inline EAccess &				sPositionAccess() {
		static thread_local EAccess sAccess = BodyAccess::EAccess::ReadWrite;
		return sAccess;
	}
};


/// Structure that holds a body pair
struct alignas(uint64) BodyPair {
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
							BodyPair() = default;
							BodyPair(BodyID inA, BodyID inB)							: mBodyA(inA), mBodyB(inB) { }

	/// Equals operator
	bool					operator == (const BodyPair &inRHS) const					{ return *reinterpret_cast<const uint64 *>(this) == *reinterpret_cast<const uint64 *>(&inRHS); }

	/// Smaller than operator, used for consistently ordering body pairs
	bool					operator < (const BodyPair &inRHS) const					{ return *reinterpret_cast<const uint64 *>(this) < *reinterpret_cast<const uint64 *>(&inRHS); }

	/// Get the hash value of this object
	uint64					GetHash() const												{ return Hash64(*reinterpret_cast<const uint64 *>(this)); }

	BodyID					mBodyA;
	BodyID					mBodyB;
};

static_assert(sizeof(BodyPair) == sizeof(uint64), "Mismatch in class size");



template <bool Write, class BodyType>
class BodyLockBase : public NonCopyable
{
public:
	/// Constructor will lock the body
								BodyLockBase(const BodyLockInterface &inBodyLockInterface, const BodyID &inBodyID) :
		mBodyLockInterface(inBodyLockInterface)
	{
		if (inBodyID == BodyID())
		{
			// Invalid body id
			mBodyLockMutex = nullptr;
			mBody = nullptr;
		}
		else
		{
			// Get mutex
			mBodyLockMutex = Write? inBodyLockInterface.LockWrite(inBodyID) : inBodyLockInterface.LockRead(inBodyID);

			// Get a reference to the body or nullptr when it is no longer valid
			mBody = inBodyLockInterface.TryGetBody(inBodyID);
		}
	}

	/// Explicitly release the lock (normally this is done in the destructor)
	inline void					ReleaseLock()
	{
		if (mBodyLockMutex != nullptr)
		{
			if (Write)
				mBodyLockInterface.UnlockWrite(mBodyLockMutex);
			else
				mBodyLockInterface.UnlockRead(mBodyLockMutex);

			mBodyLockMutex = nullptr;
			mBody = nullptr;
		}
	}

	/// Destructor will unlock the body
								~BodyLockBase()
	{
		ReleaseLock();
	}

	/// Test if the lock was successful (if the body ID was valid)
	inline bool					Succeeded() const
	{
		return mBody != nullptr;
	}

	/// Test if the lock was successful (if the body ID was valid) and the body is still in the broad phase
	inline bool					SucceededAndIsInBroadPhase() const
	{
		return mBody != nullptr && mBody->IsInBroadPhase();
	}

	/// Access the body
	inline BodyType &			GetBody() const
	{
		MOSS_ASSERT(mBody != nullptr, "Should check Succeeded() first");
		return *mBody;
	}

private:
	const BodyLockInterface &	mBodyLockInterface;
	SharedMutex *				mBodyLockMutex;
	BodyType *					mBody;
};

/// A body lock takes a body ID and locks the underlying body so that other threads cannot access its members
///
/// The common usage pattern is:
///
///		BodyLockInterface lock_interface = physics_system.GetBodyLockInterface(); // Or non-locking interface if the lock is already taken
///		BodyID body_id = ...; // Obtain ID to body
///
///		// Scoped lock
///		{
///			BodyLockRead lock(lock_interface, body_id);
///			if (lock.Succeeded()) // body_id may no longer be valid
///			{
///				const Body &body = lock.GetBody();
///
///				// Do something with body
///				...
///			}
///		}
class BodyLockRead : public BodyLockBase<false, const Body>
{
	using BodyLockBase::BodyLockBase;
};

/// Specialization that locks a body for writing to. @see BodyLockRead for usage patterns.
class BodyLockWrite : public BodyLockBase<true, Body>
{
	using BodyLockBase::BodyLockBase;
};


// Classes
class BodyCreationSettings;
class SoftBodyCreationSettings;
class BodyActivationListener;
class StateRecorderFilter;
struct PhysicsSettings;
#ifndef MOSS_DEBUG_RENDERER
class DebugRenderer;
class BodyDrawFilter;
#endif // MOSS_DEBUG_RENDERER

#ifndef MOSS_DEBUG_RENDERER

/// Defines how to color soft body constraints
enum class ESoftBodyConstraintColor
{
	ConstraintType,				/// Draw different types of constraints in different colors
	ConstraintGroup,			/// Draw constraints in the same group in the same color, non-parallel group will be red
	ConstraintOrder,			/// Draw constraints in the same group in the same color, non-parallel group will be red, and order within each group will be indicated with gradient
};

#endif // MOSS_DEBUG_RENDERER

/// Array of bodies
using BodyVector = TArray<Body *>;

/// Array of body ID's
using BodyIDVector = TArray<BodyID>;

class BodyActivationListener {
public:
	// Ensure virtual destructor
	virtual					~BodyActivationListener() = default;

	// Called whenever a body activates, note this can be called from any thread so make sure your code is thread safe.
	// At the time of the callback the body inBodyID will be locked and no bodies can be written/activated/deactivated from the callback.
	virtual void			OnBodyActivated(const BodyID &inBodyID, uint64 inBodyUserData) = 0;

	// Called whenever a body deactivates, note this can be called from any thread so make sure your code is thread safe.
	// At the time of the callback the body inBodyID will be locked and no bodies can be written/activated/deactivated from the callback.
	virtual void			OnBodyDeactivated(const BodyID &inBodyID, uint64 inBodyUserData) = 0;
};


/* BodyInterface */
class MOSS_EXPORT BodyInterface : public NonCopyable {
public:
	// Initialize the interface (should only be called by PhysicsSystem)
	void						Init(BodyLockInterface &inBodyLockInterface, BodyManager &inBodyManager, BroadPhase &inBroadPhase) { mBodyLockInterface = &inBodyLockInterface; mBodyManager = &inBodyManager; mBroadPhase = &inBroadPhase; }

	// Create a rigid body
	// @return Created body or null when out of bodies
	Body *						CreateBody(const BodyCreationSettings &inSettings);

	// Create a soft body
	// @return Created body or null when out of bodies
	Body *						CreateSoftBody(const SoftBodyCreationSettings &inSettings);

	// Create a rigid body with specified ID. This function can be used if a simulation is to run in sync between clients or if a simulation needs to be restored exactly.
	// The ID created on the server can be replicated to the client and used to create a deterministic simulation.
	// @return Created body or null when the body ID is invalid or a body of the same ID already exists.
	Body *						CreateBodyWithID(const BodyID &inBodyID, const BodyCreationSettings &inSettings);

	// Create a soft body with specified ID. See comments at CreateBodyWithID.
	Body *						CreateSoftBodyWithID(const BodyID &inBodyID, const SoftBodyCreationSettings &inSettings);

	// Advanced use only. Creates a rigid body without specifying an ID. This body cannot be added to the physics system until it has been assigned a body ID.
	// This can be used to decouple allocation from registering the body. A call to CreateBodyWithoutID followed by AssignBodyID is equivalent to calling CreateBodyWithID.
	// @return Created body
	Body *						CreateBodyWithoutID(const BodyCreationSettings &inSettings) const;

	// Advanced use only. Creates a body without specifying an ID. See comments at CreateBodyWithoutID.
	Body *						CreateSoftBodyWithoutID(const SoftBodyCreationSettings &inSettings) const;

	// Advanced use only. Destroy a body previously created with CreateBodyWithoutID that hasn't gotten an ID yet through the AssignBodyID function,
	// or a body that has had its body ID unassigned through UnassignBodyIDs. Bodies that have an ID should be destroyed through DestroyBody.
	void						DestroyBodyWithoutID(Body *inBody) const;

	// Advanced use only. Assigns the next available body ID to a body that was created using CreateBodyWithoutID. After this call, the body can be added to the physics system.
	// @return false if the body already has an ID or out of body ids.
	bool						AssignBodyID(Body *ioBody);

	// Advanced use only. Assigns a body ID to a body that was created using CreateBodyWithoutID. After this call, the body can be added to the physics system.
	// @return false if the body already has an ID or if the ID is not valid.
	bool						AssignBodyID(Body *ioBody, const BodyID &inBodyID);

	// Advanced use only. See UnassignBodyIDs. Unassigns the ID of a single body.
	Body *						UnassignBodyID(const BodyID &inBodyID);

	// Advanced use only. Removes a number of body IDs from their bodies and returns the body pointers. Before calling this, the body should have been removed from the physics system.
	// The body can be destroyed through DestroyBodyWithoutID. This can be used to decouple deallocation. A call to UnassignBodyIDs followed by calls to DestroyBodyWithoutID is equivalent to calling DestroyBodies.
	// @param inBodyIDs A list of body IDs
	// @param inNumber Number of bodies in the list
	// @param outBodies If not null on input, this will contain a list of body pointers corresponding to inBodyIDs that can be destroyed afterwards (caller assumes ownership over these).
	void						UnassignBodyIDs(const BodyID *inBodyIDs, int inNumber, Body **outBodies);

	// Destroy a body.
	// Make sure that you remove the body from the physics system using BodyInterface::RemoveBody before calling this function.
	void						DestroyBody(const BodyID &inBodyID);

	// Destroy multiple bodies
	// Make sure that you remove the bodies from the physics system using BodyInterface::RemoveBody before calling this function.
	void						DestroyBodies(const BodyID *inBodyIDs, int inNumber);

	// Add body to the physics system.
	// Note that if you need to add multiple bodies, use the AddBodiesPrepare/AddBodiesFinalize function.
	// Adding many bodies, one at a time, results in a really inefficient broadphase until PhysicsSystem::OptimizeBroadPhase is called or when PhysicsSystem::Update rebuilds the tree!
	// After adding, to get a body by ID use the BodyLockRead or BodyLockWrite interface!
	void						AddBody(const BodyID &inBodyID, EActivation inActivationMode);

	// Remove body from the physics system.
	void						RemoveBody(const BodyID &inBodyID);

	// Check if a body has been added to the physics system.
	bool						IsAdded(const BodyID &inBodyID) const;

	// Combines CreateBody and AddBody
	// @return Created body ID or an invalid ID when out of bodies
	BodyID						CreateAndAddBody(const BodyCreationSettings &inSettings, EActivation inActivationMode);

	// Combines CreateSoftBody and AddBody
	// @return Created body ID or an invalid ID when out of bodies
	BodyID						CreateAndAddSoftBody(const SoftBodyCreationSettings &inSettings, EActivation inActivationMode);

	// Add state handle, used to keep track of a batch of bodies while adding them to the PhysicsSystem.
	using AddState = void *;

	//@name Batch adding interface
	//@{

	// Prepare adding inNumber bodies at ioBodies to the PhysicsSystem, returns a handle that should be used in AddBodiesFinalize/Abort.
	// This can be done on a background thread without influencing the PhysicsSystem.
	// ioBodies may be shuffled around by this function and should be kept that way until AddBodiesFinalize/Abort is called.
	AddState					AddBodiesPrepare(BodyID *ioBodies, int inNumber);

	// Finalize adding bodies to the PhysicsSystem, supply the return value of AddBodiesPrepare in inAddState.
	// Please ensure that the ioBodies array passed to AddBodiesPrepare is unmodified and passed again to this function.
	void						AddBodiesFinalize(BodyID *ioBodies, int inNumber, AddState inAddState, EActivation inActivationMode);

	// Abort adding bodies to the PhysicsSystem, supply the return value of AddBodiesPrepare in inAddState.
	// This can be done on a background thread without influencing the PhysicsSystem.
	// Please ensure that the ioBodies array passed to AddBodiesPrepare is unmodified and passed again to this function.
	void						AddBodiesAbort(BodyID *ioBodies, int inNumber, AddState inAddState);

	// Remove inNumber bodies in ioBodies from the PhysicsSystem.
	// ioBodies may be shuffled around by this function.
	void						RemoveBodies(BodyID *ioBodies, int inNumber);
	//@}

	//@name Activate / deactivate a body
	//@{
	void						ActivateBody(const BodyID &inBodyID);
	void						ActivateBodies(const BodyID *inBodyIDs, int inNumber);
	void						ActivateBodiesInAABox(const AABox &inBox, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter);
	void						DeactivateBody(const BodyID &inBodyID);
	void						DeactivateBodies(const BodyID *inBodyIDs, int inNumber);
	bool						IsActive(const BodyID &inBodyID) const;
	void						ResetSleepTimer(const BodyID &inBodyID);
	//@}

	// Create a two body constraint
	TwoBodyConstraint *			CreateConstraint(const TwoBodyConstraintSettings *inSettings, const BodyID &inBodyID1, const BodyID &inBodyID2);

	// Activate non-static bodies attached to a constraint
	void						ActivateConstraint(const TwoBodyConstraint *inConstraint);

	//@name Access to the shape of a body
	//@{

	// Get the current shape
	RefConst<Shape>				GetShape(const BodyID &inBodyID) const;

	// Set a new shape on the body
	// @param inBodyID Body ID of body that had its shape changed
	// @param inShape The new shape
	// @param inUpdateMassProperties When true, the mass and inertia tensor is recalculated
	// @param inActivationMode Whether or not to activate the body
	void						SetShape(const BodyID &inBodyID, const Shape *inShape, bool inUpdateMassProperties, EActivation inActivationMode) const;

	// Notify all systems to indicate that a shape has changed (usable for MutableCompoundShapes)
	// @param inBodyID Body ID of body that had its shape changed
	// @param inPreviousCenterOfMass Center of mass of the shape before the alterations
	// @param inUpdateMassProperties When true, the mass and inertia tensor is recalculated
	// @param inActivationMode Whether or not to activate the body
	void						NotifyShapeChanged(const BodyID &inBodyID, Vec3Arg inPreviousCenterOfMass, bool inUpdateMassProperties, EActivation inActivationMode) const;
	//@}

	//@name Object layer of a body
	//@{
	void						SetObjectLayer(const BodyID &inBodyID, ObjectLayer inLayer);
	ObjectLayer					GetObjectLayer(const BodyID &inBodyID) const;
	//@}

	//@name Position and rotation of a body
	//@{
	void						SetPositionAndRotation(const BodyID &inBodyID, RVec3Arg inPosition, QuatArg inRotation, EActivation inActivationMode);
	void						SetPositionAndRotationWhenChanged(const BodyID &inBodyID, RVec3Arg inPosition, QuatArg inRotation, EActivation inActivationMode); // Will only update the position/rotation and activate the body when the difference is larger than a very small number. This avoids updating the broadphase/waking up a body when the resulting position/orientation doesn't really change.
	void						GetPositionAndRotation(const BodyID &inBodyID, RVec3 &outPosition, Quat &outRotation) const;
	void						SetPosition(const BodyID &inBodyID, RVec3Arg inPosition, EActivation inActivationMode);
	RVec3						GetPosition(const BodyID &inBodyID) const;
	RVec3						GetCenterOfMassPosition(const BodyID &inBodyID) const;
	void						SetRotation(const BodyID &inBodyID, QuatArg inRotation, EActivation inActivationMode);
	Quat						GetRotation(const BodyID &inBodyID) const;
	RMat44						GetWorldTransform(const BodyID &inBodyID) const;
	RMat44						GetCenterOfMassTransform(const BodyID &inBodyID) const;
	//@}

	// Set velocity of body such that it will be positioned at inTargetPosition/Rotation in inDeltaTime seconds (will activate body if needed)
	void						MoveKinematic(const BodyID &inBodyID, RVec3Arg inTargetPosition, QuatArg inTargetRotation, float inDeltaTime);

	// Linear or angular velocity (functions will activate body if needed).
	// Note that the linear velocity is the velocity of the center of mass, which may not coincide with the position of your object, to correct for this: \f$VelocityCOM = Velocity - AngularVelocity \times ShapeCOM\f$
	void						SetLinearAndAngularVelocity(const BodyID &inBodyID, Vec3Arg inLinearVelocity, Vec3Arg inAngularVelocity);
	void						GetLinearAndAngularVelocity(const BodyID &inBodyID, Vec3 &outLinearVelocity, Vec3 &outAngularVelocity) const;
	void						SetLinearVelocity(const BodyID &inBodyID, Vec3Arg inLinearVelocity);
	Vec3						GetLinearVelocity(const BodyID &inBodyID) const;
	void						AddLinearVelocity(const BodyID &inBodyID, Vec3Arg inLinearVelocity); // Add velocity to current velocity
	void						AddLinearAndAngularVelocity(const BodyID &inBodyID, Vec3Arg inLinearVelocity, Vec3Arg inAngularVelocity); // Add linear and angular to current velocities
	void						SetAngularVelocity(const BodyID &inBodyID, Vec3Arg inAngularVelocity);
	Vec3						GetAngularVelocity(const BodyID &inBodyID) const;
	Vec3						GetPointVelocity(const BodyID &inBodyID, RVec3Arg inPoint) const; // Velocity of point inPoint (in world space, e.g. on the surface of the body) of the body

	// Set the complete motion state of a body.
	// Note that the linear velocity is the velocity of the center of mass, which may not coincide with the position of your object, to correct for this: \f$VelocityCOM = Velocity - AngularVelocity \times ShapeCOM\f$
	void						SetPositionRotationAndVelocity(const BodyID &inBodyID, RVec3Arg inPosition, QuatArg inRotation, Vec3Arg inLinearVelocity, Vec3Arg inAngularVelocity);

	//@name Add forces to the body
	//@{
	void						AddForce(const BodyID &inBodyID, Vec3Arg inForce, EActivation inActivationMode = EActivation::Activate); // See Body::AddForce
	void						AddForce(const BodyID &inBodyID, Vec3Arg inForce, RVec3Arg inPoint, EActivation inActivationMode = EActivation::Activate); // Applied at inPoint
	void						AddTorque(const BodyID &inBodyID, Vec3Arg inTorque, EActivation inActivationMode = EActivation::Activate); // See Body::AddTorque
	void						AddForceAndTorque(const BodyID &inBodyID, Vec3Arg inForce, Vec3Arg inTorque, EActivation inActivationMode = EActivation::Activate); // A combination of Body::AddForce and Body::AddTorque
	//@}

	//@name Add an impulse to the body
	//@{
	void						AddImpulse(const BodyID &inBodyID, Vec3Arg inImpulse); // Applied at center of mass
	void						AddImpulse(const BodyID &inBodyID, Vec3Arg inImpulse, RVec3Arg inPoint); // Applied at inPoint
	void						AddAngularImpulse(const BodyID &inBodyID, Vec3Arg inAngularImpulse);
	bool						ApplyBuoyancyImpulse(const BodyID &inBodyID, RVec3Arg inSurfacePosition, Vec3Arg inSurfaceNormal, float inBuoyancy, float inLinearDrag, float inAngularDrag, Vec3Arg inFluidVelocity, Vec3Arg inGravity, float inDeltaTime);
	//@}

	//@name Body type
	//@{
	EBodyType					GetBodyType(const BodyID &inBodyID) const;
	//@}

	//@name Body motion type
	//@{
	void						SetMotionType(const BodyID &inBodyID, EMotionType inMotionType, EActivation inActivationMode);
	EMotionType					GetMotionType(const BodyID &inBodyID) const;
	//@}

	//@name Body motion quality
	//@{
	void						SetMotionQuality(const BodyID &inBodyID, EMotionQuality inMotionQuality);
	EMotionQuality				GetMotionQuality(const BodyID &inBodyID) const;
	//@}

	// Get inverse inertia tensor in world space
	Mat44						GetInverseInertia(const BodyID &inBodyID) const;

	//@name Restitution
	//@{
	void						SetRestitution(const BodyID &inBodyID, float inRestitution);
	float						GetRestitution(const BodyID &inBodyID) const;
	//@}

	//@name Friction
	//@{
	void						SetFriction(const BodyID &inBodyID, float inFriction);
	float						GetFriction(const BodyID &inBodyID) const;
	//@}

	//@name Gravity factor
	//@{
	void						SetGravityFactor(const BodyID &inBodyID, float inGravityFactor);
	float						GetGravityFactor(const BodyID &inBodyID) const;
	//@}

	//@name Manifold reduction
	//@{
	void						SetUseManifoldReduction(const BodyID &inBodyID, bool inUseReduction);
	bool						GetUseManifoldReduction(const BodyID &inBodyID) const;
	//@}

	//@name Collision group
	//@{
	void						SetCollisionGroup(const BodyID &inBodyID, const CollisionGroup &inCollisionGroup);
	const CollisionGroup &		GetCollisionGroup(const BodyID &inBodyID) const;
	//@}

	// Get transform and shape for this body, used to perform collision detection
	TransformedShape			GetTransformedShape(const BodyID &inBodyID) const;

	// Get the user data for a body
	uint64						GetUserData(const BodyID &inBodyID) const;
	void						SetUserData(const BodyID &inBodyID, uint64 inUserData) const;

	// Get the material for a particular sub shape
	const PhysicsMaterial *		GetMaterial(const BodyID &inBodyID, const SubShapeID &inSubShapeID) const;

	// Set the Body::EFlags::InvalidateContactCache flag for the specified body. This means that the collision cache is invalid for any body pair involving that body until the next physics step.
	void						InvalidateContactCache(const BodyID &inBodyID);

private:
	// Helper function to activate a single body
	MOSS_INLINE void				ActivateBodyInternal(Body &ioBody) const;

	BodyLockInterface *			mBodyLockInterface = nullptr;
	BodyManager *				mBodyManager = nullptr;
	BroadPhase *				mBroadPhase = nullptr;
};

/// Class that contains all bodies
class MOSS_EXPORT BodyManager : public NonCopyable
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Destructor
									~BodyManager();

	/// Initialize the manager
	void							Init(uint inMaxBodies, uint inNumBodyMutexes, const BroadPhaseLayerInterface &inLayerInterface);

	/// Gets the current amount of bodies that are in the body manager
	uint							GetNumBodies() const;

	/// Gets the max bodies that we can support
	uint							GetMaxBodies() const						{ return uint(mBodies.capacity()); }

	/// Helper struct that counts the number of bodies of each type
	struct BodyStats
	{
		uint						mNumBodies					= 0;			///< Total number of bodies in the body manager
		uint						mMaxBodies					= 0;			///< Max allowed number of bodies in the body manager (as configured in Init(...))

		uint						mNumBodiesStatic			= 0;			///< Number of static bodies

		uint						mNumBodiesDynamic			= 0;			///< Number of dynamic bodies
		uint						mNumActiveBodiesDynamic		= 0;			///< Number of dynamic bodies that are currently active

		uint						mNumBodiesKinematic			= 0;			///< Number of kinematic bodies
		uint						mNumActiveBodiesKinematic	= 0;			///< Number of kinematic bodies that are currently active

		uint						mNumSoftBodies				= 0;			///< Number of soft bodies
		uint						mNumActiveSoftBodies		= 0;			///< Number of soft bodies that are currently active
	};

	/// Get stats about the bodies in the body manager (slow, iterates through all bodies)
	BodyStats						GetBodyStats() const;

	/// Create a body using creation settings. The returned body will not be part of the body manager yet.
	Body *							AllocateBody(const BodyCreationSettings &inBodyCreationSettings) const;

	/// Create a soft body using creation settings. The returned body will not be part of the body manager yet.
	Body *							AllocateSoftBody(const SoftBodyCreationSettings &inSoftBodyCreationSettings) const;

	/// Free a body that has not been added to the body manager yet (if it has, use DestroyBodies).
	void							FreeBody(Body *inBody) const;

	/// Add a body to the body manager, assigning it the next available ID. Returns false if no more IDs are available.
	bool							AddBody(Body *ioBody);

	/// Add a body to the body manager, assigning it a custom ID. Returns false if the ID is not valid.
	bool							AddBodyWithCustomID(Body *ioBody, const BodyID &inBodyID);

	/// Remove a list of bodies from the body manager
	void							RemoveBodies(const BodyID *inBodyIDs, int inNumber, Body **outBodies);

	/// Remove a set of bodies from the body manager and frees them.
	void							DestroyBodies(const BodyID *inBodyIDs, int inNumber);

	/// Activate a list of bodies.
	/// This function should only be called when an exclusive lock for the bodies are held.
	void							ActivateBodies(const BodyID *inBodyIDs, int inNumber);

	/// Deactivate a list of bodies.
	/// This function should only be called when an exclusive lock for the bodies are held.
	void							DeactivateBodies(const BodyID *inBodyIDs, int inNumber);

	/// Update the motion quality for a body
	void							SetMotionQuality(Body &ioBody, EMotionQuality inMotionQuality);

	/// Get copy of the list of active bodies under protection of a lock.
	void							GetActiveBodies(EBodyType inType, BodyIDVector &outBodyIDs) const;

	/// Get the list of active bodies. Note: Not thread safe. The active bodies list can change at any moment.
	const BodyID *					GetActiveBodiesUnsafe(EBodyType inType) const { return mActiveBodies[int(inType)]; }

	/// Get the number of active bodies.
	uint32							GetNumActiveBodies(EBodyType inType) const	{ return mNumActiveBodies[int(inType)].load(memory_order_acquire); }

	/// Get the number of active bodies that are using continuous collision detection
	uint32							GetNumActiveCCDBodies() const				{ return mNumActiveCCDBodies; }

	/// Listener that is notified whenever a body is activated/deactivated
	void							SetBodyActivationListener(BodyActivationListener *inListener);
	BodyActivationListener *		GetBodyActivationListener() const			{ return mActivationListener; }

	/// Check if this is a valid body pointer. When a body is freed the memory that the pointer occupies is reused to store a freelist.
	static inline bool				sIsValidBodyPointer(const Body *inBody)		{ return (uintptr_t(inBody) & cIsFreedBody) == 0; }

	/// Get all bodies. Note that this can contain invalid body pointers, call sIsValidBodyPointer to check.
	const BodyVector &				GetBodies() const							{ return mBodies; }

	/// Get all bodies. Note that this can contain invalid body pointers, call sIsValidBodyPointer to check.
	BodyVector &					GetBodies()									{ return mBodies; }

	/// Get all body IDs under the protection of a lock
	void							GetBodyIDs(BodyIDVector &outBodies) const;

	/// Access a body (not protected by lock)
	const Body &					GetBody(const BodyID &inID) const			{ return *mBodies[inID.GetIndex()]; }

	/// Access a body (not protected by lock)
	Body &							GetBody(const BodyID &inID)					{ return *mBodies[inID.GetIndex()]; }

	/// Access a body, will return a nullptr if the body ID is no longer valid (not protected by lock)
	const Body *					TryGetBody(const BodyID &inID) const
	{
		uint32 idx = inID.GetIndex();
		if (idx >= mBodies.size())
			return nullptr;

		const Body *body = mBodies[idx];
		if (sIsValidBodyPointer(body) && body->GetID() == inID)
			return body;

		return nullptr;
	}

	/// Access a body, will return a nullptr if the body ID is no longer valid (not protected by lock)
	Body *							TryGetBody(const BodyID &inID)
	{
		uint32 idx = inID.GetIndex();
		if (idx >= mBodies.size())
			return nullptr;

		Body *body = mBodies[idx];
		if (sIsValidBodyPointer(body) && body->GetID() == inID)
			return body;

		return nullptr;
	}

	/// Access the mutex for a single body
	SharedMutex &					GetMutexForBody(const BodyID &inID) const	{ return mBodyMutexes.GetMutexByObjectIndex(inID.GetIndex()); }

	/// Bodies are protected using an array of mutexes (so a fixed number, not 1 per body). Each bit in this mask indicates a locked mutex.
	using MutexMask = uint64;

	///@name Batch body mutex access (do not use directly)
	///@{
	MutexMask						GetAllBodiesMutexMask() const				{ return mBodyMutexes.GetNumMutexes() == sizeof(MutexMask) * 8? ~MutexMask(0) : (MutexMask(1) << mBodyMutexes.GetNumMutexes()) - 1; }
	MutexMask						GetMutexMask(const BodyID *inBodies, int inNumber) const;
	void							LockRead(MutexMask inMutexMask) const;
	void							UnlockRead(MutexMask inMutexMask) const;
	void							LockWrite(MutexMask inMutexMask) const;
	void							UnlockWrite(MutexMask inMutexMask) const;
	///@}

	/// Lock all bodies. This should only be done during PhysicsSystem::Update().
	void							LockAllBodies() const;

	/// Unlock all bodies. This should only be done during PhysicsSystem::Update().
	void							UnlockAllBodies() const;

	/// Function to update body's layer (should only be called by the BodyInterface since it also requires updating the broadphase)
	inline void						SetBodyObjectLayerInternal(Body &ioBody, ObjectLayer inLayer) const { ioBody.mObjectLayer = inLayer; ioBody.mBroadPhaseLayer = mBroadPhaseLayerInterface->GetBroadPhaseLayer(inLayer); }

	/// Set the Body::EFlags::InvalidateContactCache flag for the specified body. This means that the collision cache is invalid for any body pair involving that body until the next physics step.
	void							InvalidateContactCacheForBody(Body &ioBody);

	/// Reset the Body::EFlags::InvalidateContactCache flag for all bodies. All contact pairs in the contact cache will now by valid again.
	void							ValidateContactCacheForAllBodies();

	/// Saving state for replay
	void							SaveState(StateRecorder &inStream, const StateRecorderFilter *inFilter) const;

	/// Restoring state for replay. Returns false if failed.
	bool							RestoreState(StateRecorder &inStream);

	/// Save the state of a single body for replay
	void							SaveBodyState(const Body &inBody, StateRecorder &inStream) const;

	/// Save the state of a single body for replay
	void							RestoreBodyState(Body &inBody, StateRecorder &inStream);

#ifndef MOSS_DEBUG_RENDERER
	enum class EShapeColor
	{
		InstanceColor,				///< Random color per instance
		ShapeTypeColor,				///< Convex = green, scaled = yellow, compound = orange, mesh = red
		MotionTypeColor,			///< Static = grey, keyframed = green, dynamic = random color per instance
		SleepColor,					///< Static = grey, keyframed = green, dynamic = yellow, sleeping = red
		IslandColor,				///< Static = grey, active = random color per island, sleeping = light grey
		MaterialColor,				///< Color as defined by the PhysicsMaterial of the shape
	};

	/// Draw settings
	struct DrawSettings
	{
		bool						mDrawGetSupportFunction = false;				///< Draw the GetSupport() function, used for convex collision detection
		bool						mDrawSupportDirection = false;					///< When drawing the support function, also draw which direction mapped to a specific support point
		bool						mDrawGetSupportingFace = false;					///< Draw the faces that were found colliding during collision detection
		bool						mDrawShape = true;								///< Draw the shapes of all bodies
		bool						mDrawShapeWireframe = false;					///< When mDrawShape is true and this is true, the shapes will be drawn in wireframe instead of solid.
		EShapeColor					mDrawShapeColor = EShapeColor::MotionTypeColor; ///< Coloring scheme to use for shapes
		bool						mDrawBoundingBox = false;						///< Draw a bounding box per body
		bool						mDrawCenterOfMassTransform = false;				///< Draw the center of mass for each body
		bool						mDrawWorldTransform = false;					///< Draw the world transform (which can be different than the center of mass) for each body
		bool						mDrawVelocity = false;							///< Draw the velocity vector for each body
		bool						mDrawMassAndInertia = false;					///< Draw the mass and inertia (as the box equivalent) for each body
		bool						mDrawSleepStats = false;						///< Draw stats regarding the sleeping algorithm of each body
		bool						mDrawSoftBodyVertices = false;					///< Draw the vertices of soft bodies
		bool						mDrawSoftBodyVertexVelocities = false;			///< Draw the velocities of the vertices of soft bodies
		bool						mDrawSoftBodyEdgeConstraints = false;			///< Draw the edge constraints of soft bodies
		bool						mDrawSoftBodyBendConstraints = false;			///< Draw the bend constraints of soft bodies
		bool						mDrawSoftBodyVolumeConstraints = false;			///< Draw the volume constraints of soft bodies
		bool						mDrawSoftBodySkinConstraints = false;			///< Draw the skin constraints of soft bodies
		bool						mDrawSoftBodyLRAConstraints = false;			///< Draw the LRA constraints of soft bodies
		bool						mDrawSoftBodyPredictedBounds = false;			///< Draw the predicted bounds of soft bodies
		ESoftBodyConstraintColor	mDrawSoftBodyConstraintColor = ESoftBodyConstraintColor::ConstraintType; ///< Coloring scheme to use for soft body constraints
	};

	/// Draw the state of the bodies (debugging purposes)
	void							Draw(const DrawSettings &inSettings, const PhysicsSettings &inPhysicsSettings, DebugRenderer *inRenderer, const BodyDrawFilter *inBodyFilter = nullptr);
#endif // MOSS_DEBUG_RENDERER

#ifdef MOSS_DEBUG
	/// Lock the active body list, asserts when Activate/DeactivateBody is called.
	void							SetActiveBodiesLocked(bool inLocked)		{ mActiveBodiesLocked = inLocked; }

	/// Per thread override of the locked state, to be used by the PhysicsSystem only!
	class GrantActiveBodiesAccess
	{
	public:
		inline GrantActiveBodiesAccess(bool inAllowActivation, bool inAllowDeactivation)
		{
			MOSS_ASSERT(!sGetOverrideAllowActivation());
			sSetOverrideAllowActivation(inAllowActivation);

			MOSS_ASSERT(!sGetOverrideAllowDeactivation());
			sSetOverrideAllowDeactivation(inAllowDeactivation);
		}

		inline ~GrantActiveBodiesAccess()
		{
			sSetOverrideAllowActivation(false);
			sSetOverrideAllowDeactivation(false);
		}
	};
#endif

#ifdef MOSS_DEBUG
	/// Validate if the cached bounding boxes are correct for all active bodies
	void							ValidateActiveBodyBounds();
#endif // MOSS_DEBUG

private:
	/// Increment and get the sequence number of the body
#ifdef MOSS_COMPILER_CLANG
	__attribute__((no_sanitize("implicit-conversion"))) // We intentionally overflow the uint8 sequence number
#endif
	inline uint8					GetNextSequenceNumber(int inBodyIndex)		{ return ++mBodySequenceNumbers[inBodyIndex]; }

	/// Add a single body to mActiveBodies, note doesn't lock the active body mutex!
	inline void						AddBodyToActiveBodies(Body &ioBody);

	/// Remove a single body from mActiveBodies, note doesn't lock the active body mutex!
	inline void						RemoveBodyFromActiveBodies(Body &ioBody);

	/// Helper function to remove a body from the manager
	MOSS_INLINE Body *				RemoveBodyInternal(const BodyID &inBodyID);

	/// Helper function to delete a body (which could actually be a BodyWithMotionProperties)
	inline static void				sDeleteBody(Body *inBody);

#if defined(MOSS_DEBUG)
	/// Function to check that the free list is not corrupted
	void							ValidateFreeList() const;
#endif // defined(MOSS_DEBUG)

	/// List of pointers to all bodies. Contains invalid pointers for deleted bodies, check with sIsValidBodyPointer. Note that this array is reserved to the max bodies that is passed in the Init function so that adding bodies will not reallocate the array.
	BodyVector						mBodies;

	/// Current number of allocated bodies
	uint							mNumBodies = 0;

	/// Indicates that there are no more freed body IDs
	static constexpr uintptr_t		cBodyIDFreeListEnd = ~uintptr_t(0);

	/// Bit that indicates a pointer in mBodies is actually the index of the next freed body. We use the lowest bit because we know that Bodies need to be 16 byte aligned so addresses can never end in a 1 bit.
	static constexpr uintptr_t		cIsFreedBody = uintptr_t(1);

	/// Amount of bits to shift to get an index to the next freed body
	static constexpr uint			cFreedBodyIndexShift = 1;

	/// Index of first entry in mBodies that is unused
	uintptr_t						mBodyIDFreeListStart = cBodyIDFreeListEnd;

	/// Protects mBodies array (but not the bodies it points to), mNumBodies and mBodyIDFreeListStart
	mutable Mutex					mBodiesMutex;

	/// An array of mutexes protecting the bodies in the mBodies array
	using BodyMutexes = MutexArray<SharedMutex>;
	mutable BodyMutexes				mBodyMutexes;

	/// List of next sequence number for a body ID
	TArray<uint8>					mBodySequenceNumbers;

	/// Mutex that protects the mActiveBodies array
	mutable Mutex					mActiveBodiesMutex;

	/// List of all active dynamic bodies (size is equal to max amount of bodies)
	BodyID *						mActiveBodies[cBodyTypeCount] = { };

	/// How many bodies there are in the list of active bodies
	atomic<uint32>					mNumActiveBodies[cBodyTypeCount] = { };

	/// How many of the active bodies have continuous collision detection enabled
	uint32							mNumActiveCCDBodies = 0;

	/// Mutex that protects the mBodiesCacheInvalid array
	mutable Mutex					mBodiesCacheInvalidMutex;

	/// List of all bodies that should have their cache invalidated
	BodyIDVector					mBodiesCacheInvalid;

	/// Listener that is notified whenever a body is activated/deactivated
	BodyActivationListener *		mActivationListener = nullptr;

	/// Cached broadphase layer interface
	const BroadPhaseLayerInterface *mBroadPhaseLayerInterface = nullptr;

#ifdef MOSS_DEBUG
	static bool						sGetOverrideAllowActivation();
	static void						sSetOverrideAllowActivation(bool inValue);

	static bool						sGetOverrideAllowDeactivation();
	static void						sSetOverrideAllowDeactivation(bool inValue);

	/// Debug system that tries to limit changes to active bodies during the PhysicsSystem::Update()
	bool							mActiveBodiesLocked = false;
#endif
};


/// Base class interface for locking a body. Usually you will use BodyLockRead / BodyLockWrite / BodyLockMultiRead / BodyLockMultiWrite instead.
class BodyLockInterface : public NonCopyable
{
public:
	/// Redefine MutexMask
	using MutexMask = BodyManager::MutexMask;

	/// Constructor
	explicit					BodyLockInterface(BodyManager &inBodyManager)		: mBodyManager(inBodyManager) { }
	virtual						~BodyLockInterface() = default;

	///@name Locking functions
	///@{
	virtual SharedMutex *		LockRead(const BodyID &inBodyID) const = 0;
	virtual void				UnlockRead(SharedMutex *inMutex) const = 0;
	virtual SharedMutex *		LockWrite(const BodyID &inBodyID) const = 0;
	virtual void				UnlockWrite(SharedMutex *inMutex) const = 0;
	///@}

	/// Get the mask needed to lock all bodies
	inline MutexMask			GetAllBodiesMutexMask() const
	{
		return mBodyManager.GetAllBodiesMutexMask();
	}

	///@name Batch locking functions
	///@{
	virtual MutexMask			GetMutexMask(const BodyID *inBodies, int inNumber) const = 0;
	virtual void				LockRead(MutexMask inMutexMask) const = 0;
	virtual void				UnlockRead(MutexMask inMutexMask) const = 0;
	virtual void				LockWrite(MutexMask inMutexMask) const = 0;
	virtual void				UnlockWrite(MutexMask inMutexMask) const = 0;
	///@}

	/// Convert body ID to body
	inline Body *				TryGetBody(const BodyID &inBodyID) const			{ return mBodyManager.TryGetBody(inBodyID); }

protected:
	BodyManager &				mBodyManager;
};

/// Implementation that performs no locking (assumes the lock has already been taken)
class BodyLockInterfaceNoLock final : public BodyLockInterface
{
public:
	using BodyLockInterface::BodyLockInterface;

	///@name Locking functions
	virtual SharedMutex *		LockRead([[maybe_unused]] const BodyID &inBodyID) const override	{ return nullptr; }
	virtual void				UnlockRead([[maybe_unused]] SharedMutex *inMutex) const override	{ /* Nothing to do */ }
	virtual SharedMutex *		LockWrite([[maybe_unused]] const BodyID &inBodyID) const override	{ return nullptr; }
	virtual void				UnlockWrite([[maybe_unused]] SharedMutex *inMutex) const override	{ /* Nothing to do */ }

	///@name Batch locking functions
	virtual MutexMask			GetMutexMask([[maybe_unused]] const BodyID *inBodies, [[maybe_unused]] int inNumber) const override { return 0; }
	virtual void				LockRead([[maybe_unused]] MutexMask inMutexMask) const override		{ /* Nothing to do */ }
	virtual void				UnlockRead([[maybe_unused]] MutexMask inMutexMask) const override	{ /* Nothing to do */ }
	virtual void				LockWrite([[maybe_unused]] MutexMask inMutexMask) const override	{ /* Nothing to do */ }
	virtual void				UnlockWrite([[maybe_unused]] MutexMask inMutexMask) const override	{ /* Nothing to do */ }
};

/// Implementation that uses the body manager to lock the correct mutex for a body
class BodyLockInterfaceLocking final : public BodyLockInterface
{
public:
	using BodyLockInterface::BodyLockInterface;

	///@name Locking functions
	virtual SharedMutex *		LockRead(const BodyID &inBodyID) const override
	{
		SharedMutex &mutex = mBodyManager.GetMutexForBody(inBodyID);
		PhysicsLock::sLockShared(mutex MOSS_IF_ENABLE_ASSERTS(, &mBodyManager, EPhysicsLockTypes::PerBody));
		return &mutex;
	}

	virtual void				UnlockRead(SharedMutex *inMutex) const override
	{
		PhysicsLock::sUnlockShared(*inMutex MOSS_IF_ENABLE_ASSERTS(, &mBodyManager, EPhysicsLockTypes::PerBody));
	}

	virtual SharedMutex *		LockWrite(const BodyID &inBodyID) const override
	{
		SharedMutex &mutex = mBodyManager.GetMutexForBody(inBodyID);
		PhysicsLock::sLock(mutex MOSS_IF_ENABLE_ASSERTS(, &mBodyManager, EPhysicsLockTypes::PerBody));
		return &mutex;
	}

	virtual void				UnlockWrite(SharedMutex *inMutex) const override
	{
		PhysicsLock::sUnlock(*inMutex MOSS_IF_ENABLE_ASSERTS(, &mBodyManager, EPhysicsLockTypes::PerBody));
	}

	///@name Batch locking functions
	virtual MutexMask			GetMutexMask(const BodyID *inBodies, int inNumber) const override
	{
		return mBodyManager.GetMutexMask(inBodies, inNumber);
	}

	virtual void				LockRead(MutexMask inMutexMask) const override
	{
		mBodyManager.LockRead(inMutexMask);
	}

	virtual void				UnlockRead(MutexMask inMutexMask) const override
	{
		mBodyManager.UnlockRead(inMutexMask);
	}

	virtual void				LockWrite(MutexMask inMutexMask) const override
	{
		mBodyManager.LockWrite(inMutexMask);
	}

	virtual void				UnlockWrite(MutexMask inMutexMask) const override
	{
		mBodyManager.UnlockWrite(inMutexMask);
	}
};



// Class function to filter out bodies, returns true if test should collide with body
class MOSS_EXPORT BodyFilter : public NonCopyable
{
public:
	/// Destructor
	virtual					~BodyFilter() = default;

	/// Filter function. Returns true if we should collide with inBodyID
	virtual bool			ShouldCollide([[maybe_unused]] const BodyID &inBodyID) const
	{
		return true;
	}

	/// Filter function. Returns true if we should collide with inBody (this is called after the body is locked and makes it possible to filter based on body members)
	virtual bool			ShouldCollideLocked([[maybe_unused]] const Body &inBody) const
	{
		return true;
	}
};

/// A simple body filter implementation that ignores a single, specified body
class MOSS_EXPORT IgnoreSingleBodyFilter : public BodyFilter
{
public:
	/// Constructor, pass the body you want to ignore
	explicit				IgnoreSingleBodyFilter(const BodyID &inBodyID) :
		mBodyID(inBodyID)
	{
	}

	/// Filter function. Returns true if we should collide with inBodyID
	virtual bool			ShouldCollide(const BodyID &inBodyID) const override
	{
		return mBodyID != inBodyID;
	}

private:
	BodyID					mBodyID;
};

/// A simple body filter implementation that ignores multiple, specified bodies
class MOSS_EXPORT IgnoreMultipleBodiesFilter : public BodyFilter
{
public:
	/// Remove all bodies from the filter
	void					Clear()
	{
		mBodyIDs.clear();
	}

	/// Reserve space for inSize body ID's
	void					Reserve(uint inSize)
	{
		mBodyIDs.reserve(inSize);
	}

	/// Add a body to be ignored
	void					IgnoreBody(const BodyID &inBodyID)
	{
		mBodyIDs.push_back(inBodyID);
	}

	/// Filter function. Returns true if we should collide with inBodyID
	virtual bool			ShouldCollide(const BodyID &inBodyID) const override
	{
		return std::find(mBodyIDs.begin(), mBodyIDs.end(), inBodyID) == mBodyIDs.end();
	}

private:
	TArray<BodyID>			mBodyIDs;
};

/// Ignores a single body and chains the filter to another filter
class MOSS_EXPORT IgnoreSingleBodyFilterChained : public BodyFilter
{
public:
	/// Constructor
	explicit				IgnoreSingleBodyFilterChained(const BodyID inBodyID, const BodyFilter &inFilter) :
		mBodyID(inBodyID),
		mFilter(inFilter)
	{
	}

	/// Filter function. Returns true if we should collide with inBodyID
	virtual bool			ShouldCollide(const BodyID &inBodyID) const override
	{
		return inBodyID != mBodyID && mFilter.ShouldCollide(inBodyID);
	}

	/// Filter function. Returns true if we should collide with inBody (this is called after the body is locked and makes it possible to filter based on body members)
	virtual bool			ShouldCollideLocked(const Body &inBody) const override
	{
		return mFilter.ShouldCollideLocked(inBody);
	}

private:
	BodyID					mBodyID;
	const BodyFilter &		mFilter;
};

#ifndef MOSS_DEBUG_RENDERER
/// Class function to filter out bodies for debug rendering, returns true if body should be rendered
class MOSS_EXPORT BodyDrawFilter : public NonCopyable
{
public:
	/// Destructor
	virtual					~BodyDrawFilter() = default;

	/// Filter function. Returns true if inBody should be rendered
	virtual bool			ShouldDraw([[maybe_unused]] const Body& inBody) const
	{
		return true;
	}
};
#endif // MOSS_DEBUG_RENDERER


// Base class for locking multiple bodies for the duration of the scope of this class (do not use directly)
template <bool Write, class BodyType>
class BodyLockMultiBase : public NonCopyable
{
public:
	/// Redefine MutexMask
	using MutexMask = BodyLockInterface::MutexMask;

	/// Constructor will lock the bodies
								BodyLockMultiBase(const BodyLockInterface &inBodyLockInterface, const BodyID *inBodyIDs, int inNumber) :
		mBodyLockInterface(inBodyLockInterface),
		mMutexMask(inBodyLockInterface.GetMutexMask(inBodyIDs, inNumber)),
		mBodyIDs(inBodyIDs),
		mNumBodyIDs(inNumber)
	{
		if (mMutexMask != 0)
		{
			// Get mutex
			if (Write)
				inBodyLockInterface.LockWrite(mMutexMask);
			else
				inBodyLockInterface.LockRead(mMutexMask);
		}
	}

	/// Destructor will unlock the bodies
								~BodyLockMultiBase()
	{
		if (mMutexMask != 0)
		{
			if (Write)
				mBodyLockInterface.UnlockWrite(mMutexMask);
			else
				mBodyLockInterface.UnlockRead(mMutexMask);
		}
	}

	/// Access the body (returns null if body was not properly locked)
	inline BodyType *			GetBody(int inBodyIndex) const
	{
		// Range check
		MOSS_ASSERT(inBodyIndex >= 0 && inBodyIndex < mNumBodyIDs);

		// Get body ID
		const BodyID &body_id = mBodyIDs[inBodyIndex];
		if (body_id.IsInvalid())
			return nullptr;

		// Get a reference to the body or nullptr when it is no longer valid
		return mBodyLockInterface.TryGetBody(body_id);
	}

private:
	const BodyLockInterface &	mBodyLockInterface;
	MutexMask					mMutexMask;
	const BodyID *				mBodyIDs;
	int							mNumBodyIDs;
};

/// A multi body lock takes a number of body IDs and locks the underlying bodies so that other threads cannot access its members
///
/// The common usage pattern is:
///
///		BodyLockInterface lock_interface = physics_system.GetBodyLockInterface(); // Or non-locking interface if the lock is already taken
///		const BodyID *body_id = ...; // Obtain IDs to bodies
///		int num_body_ids = ...;
///
///		// Scoped lock
///		{
///			BodyLockMultiRead lock(lock_interface, body_ids, num_body_ids);
///			for (int i = 0; i < num_body_ids; ++i)
///			{
///				const Body *body = lock.GetBody(i);
///				if (body != nullptr)
///				{
///					const Body &body = lock.Body();
///
///					// Do something with body
///					...
///				}
///			}
///		}
class BodyLockMultiRead : public BodyLockMultiBase<false, const Body>
{
	using BodyLockMultiBase::BodyLockMultiBase;
};

/// Specialization that locks multiple bodies for writing to. @see BodyLockMultiRead for usage patterns.
class BodyLockMultiWrite : public BodyLockMultiBase<true, Body>
{
	using BodyLockMultiBase::BodyLockMultiBase;
};


//--------------------------------------------------------------------------------------------------
// MotionProperties
//--------------------------------------------------------------------------------------------------
// The Body class only keeps track of state for static bodies, the MotionProperties class keeps the additional state needed for a moving Body. It has a 1-on-1 relationship with the body.
class MotionProperties {
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

	// Get linear damping: dv/dt = -c * v. c must be between 0 and 1 but is usually close to 0.
	inline float			GetLinearDamping() const										{ return mLinearDamping; }
	void					SetLinearDamping(float inLinearDamping)							{ MOSS_ASSERT(inLinearDamping >= 0.0f); mLinearDamping = inLinearDamping; }

	// Get angular damping: dw/dt = -c * w. c must be between 0 and 1 but is usually close to 0.
	inline float			GetAngularDamping() const										{ return mAngularDamping; }
	void					SetAngularDamping(float inAngularDamping)						{ MOSS_ASSERT(inAngularDamping >= 0.0f); mAngularDamping = inAngularDamping; }

	// Get gravity factor (1 = normal gravity, 0 = no gravity)
	inline float			GetGravityFactor() const										{ return mGravityFactor; }
	void					SetGravityFactor(float inGravityFactor)							{ mGravityFactor = inGravityFactor; }

	// Set the mass and inertia tensor
	void					SetMassProperties(EAllowedDOFs inAllowedDOFs, const MassProperties &inMassProperties);

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
	inline void				ResetSleepTestSpheres(const RVec3 *inPoints);

	// Reset the sleep test timer without resetting the sleep test spheres
	inline void				ResetSleepTestTimer()											{ mSleepTestTimer = 0.0f; }

	// Accumulate sleep time and return if a body can go to sleep
	inline ECanSleep		AccumulateSleepTime(float inDeltaTime, float inTimeBeforeSleep);

	// Saving state for replay
	void					SaveState(StateRecorder &inStream) const;

	// Restoring state for replay
	void					RestoreState(StateRecorder &inStream);

	static constexpr uint32	cInactiveIndex = uint32(-1);									// Constant indicating that body is not active

private:
	friend class BodyManager;
	friend class Body;

	// 1st cache line
	// 16 byte aligned
	Vec3					mLinearVelocity { Vec3::sZero() };								// World space linear velocity of the center of mass (m/s)
	Vec3					mAngularVelocity { Vec3::sZero() };								// World space angular velocity (rad/s)
	Vec3					mInvInertiaDiagonal;											// Diagonal of inverse inertia matrix: D
	Quat					mInertiaRotation;												// Rotation (R) that takes inverse inertia diagonal to local space: Ibody^-1 = R * D * R^-1

	// 2nd cache line
	// 4 byte aligned
	Float3					mForce { 0, 0, 0 };												// Accumulated world space force (N). Note loaded through intrinsics so ensure that the 4 bytes after this are readable!
	Float3					mTorque { 0, 0, 0 };											// Accumulated world space torque (N m). Note loaded through intrinsics so ensure that the 4 bytes after this are readable!
	float					mInvMass;														// Inverse mass of the object (1/kg)
	float					mLinearDamping;													// Linear damping: dv/dt = -c * v. c must be between 0 and 1 but is usually close to 0.
	float					mAngularDamping;												// Angular damping: dw/dt = -c * w. c must be between 0 and 1 but is usually close to 0.
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
	inline bool				IsActive() const												{ return mMotionProperties != nullptr && mMotionProperties->mIndexInActiveBodies != cInactiveIndex; }

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
	inline bool				IsSensor() const												{ return (mFlags.load(memory_order_relaxed) & uint8(EFlags::IsSensor)) != 0; }

	// If kinematic objects can generate contact points against other kinematic or static objects.
	// Note that turning this on can be CPU intensive as much more collision detection work will be done without any effect on the simulation (kinematic objects are not affected by other kinematic/static objects).
	// This can be used to make sensors detect static objects. Note that the sensor must be kinematic and active for it to detect static objects.
	inline void				SetCollideKinematicVsNonDynamic(bool inCollide)					{ MOSS_ASSERT(IsRigidBody()); if (inCollide) mFlags.fetch_or(uint8(EFlags::CollideKinematicVsNonDynamic), memory_order_relaxed); else mFlags.fetch_and(uint8(~uint8(EFlags::CollideKinematicVsNonDynamic)), memory_order_relaxed); }

	// Check if kinematic objects can generate contact points against other kinematic or static objects.
	inline bool				GetCollideKinematicVsNonDynamic() const							{ return (mFlags.load(memory_order_relaxed) & uint8(EFlags::CollideKinematicVsNonDynamic)) != 0; }

	// If PhysicsSettings::mUseManifoldReduction is true, this allows turning off manifold reduction for this specific body.
	// Manifold reduction by default will combine contacts with similar normals that come from different SubShapeIDs (e.g. different triangles in a mesh shape or different compound shapes).
	// If the application requires tracking exactly which SubShapeIDs are in contact, you can turn off manifold reduction. Note that this comes at a performance cost.
	// Consider using BodyInterface::SetUseManifoldReduction if the body could already be in contact with other bodies to ensure that the contact cache is invalidated and you get the correct contact callbacks.
	inline void				SetUseManifoldReduction(bool inUseReduction)					{ MOSS_ASSERT(IsRigidBody()); if (inUseReduction) mFlags.fetch_or(uint8(EFlags::UseManifoldReduction), memory_order_relaxed); else mFlags.fetch_and(uint8(~uint8(EFlags::UseManifoldReduction)), memory_order_relaxed); }

	// Check if this body can use manifold reduction.
	inline bool				GetUseManifoldReduction() const									{ return (mFlags.load(memory_order_relaxed) & uint8(EFlags::UseManifoldReduction)) != 0; }

	// Checks if the combination of this body and inBody2 should use manifold reduction
	inline bool				GetUseManifoldReductionWithBody(const Body &inBody2) const		{ return ((mFlags.load(memory_order_relaxed) & inBody2.mFlags.load(memory_order_relaxed)) & uint8(EFlags::UseManifoldReduction)) != 0; }

	// Set to indicate that the gyroscopic force should be applied to this body (aka Dzhanibekov effect, see https://en.wikipedia.org/wiki/Tennis_racket_theorem)
	inline void				SetApplyGyroscopicForce(bool inApply)							{ MOSS_ASSERT(IsRigidBody()); if (inApply) mFlags.fetch_or(uint8(EFlags::ApplyGyroscopicForce), memory_order_relaxed); else mFlags.fetch_and(uint8(~uint8(EFlags::ApplyGyroscopicForce)), memory_order_relaxed); }

	// Check if the gyroscopic force is being applied for this body
	inline bool				GetApplyGyroscopicForce() const									{ return (mFlags.load(memory_order_relaxed) & uint8(EFlags::ApplyGyroscopicForce)) != 0; }

	// Set to indicate that extra effort should be made to try to remove ghost contacts (collisions with internal edges of a mesh). This is more expensive but makes bodies move smoother over a mesh with convex edges.
	inline void				SetEnhancedInternalEdgeRemoval(bool inApply)					{ MOSS_ASSERT(IsRigidBody()); if (inApply) mFlags.fetch_or(uint8(EFlags::EnhancedInternalEdgeRemoval), memory_order_relaxed); else mFlags.fetch_and(uint8(~uint8(EFlags::EnhancedInternalEdgeRemoval)), memory_order_relaxed); }

	// Check if enhanced internal edge removal is turned on
	inline bool				GetEnhancedInternalEdgeRemoval() const							{ return (mFlags.load(memory_order_relaxed) & uint8(EFlags::EnhancedInternalEdgeRemoval)) != 0; }

	// Checks if the combination of this body and inBody2 should use enhanced internal edge removal
	inline bool				GetEnhancedInternalEdgeRemovalWithBody(const Body &inBody2) const { return ((mFlags.load(memory_order_relaxed) | inBody2.mFlags.load(memory_order_relaxed)) & uint8(EFlags::EnhancedInternalEdgeRemoval)) != 0; }

	// Get the bodies motion type.
	inline EMotionType		GetMotionType() const											{ return mMotionType; }

	// Set the motion type of this body. Consider using BodyInterface::SetMotionType instead of this function if the body may be active or if it needs to be activated.
	void					SetMotionType(EMotionType inMotionType);

	// Get broadphase layer, this determines in which broad phase sub-tree the object is placed
	inline BroadPhaseLayer	GetBroadPhaseLayer() const										{ return mBroadPhaseLayer; }

	// Get object layer, this determines which other objects it collides with
	inline ObjectLayer		GetObjectLayer() const											{ return mObjectLayer; }

	// Collision group and sub-group ID, determines which other objects it collides with
	const CollisionGroup &	GetCollisionGroup() const										{ return mCollisionGroup; }
	CollisionGroup &		GetCollisionGroup()												{ return mCollisionGroup; }
	void					SetCollisionGroup(const CollisionGroup &inGroup)				{ mCollisionGroup = inGroup; }

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
	void					GetSubmergedVolume(RVec3Arg inSurfacePosition, Vec3Arg inSurfaceNormal, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outRelativeCenterOfBuoyancy) const;

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
	inline bool				IsInBroadPhase() const											{ return (mFlags.load(memory_order_relaxed) & uint8(EFlags::IsInBroadPhase)) != 0; }

	// Check if this body has been changed in such a way that the collision cache should be considered invalid for any body interacting with this body
	inline bool				IsCollisionCacheInvalid() const									{ return (mFlags.load(memory_order_relaxed) & uint8(EFlags::InvalidateContactCache)) != 0; }

	// Get the shape of this body
	inline const Shape *	GetShape() const												{ return mShape; }

	// World space position of the body
	inline RVec3			GetPosition() const												{ MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sPositionAccess(), BodyAccess::EAccess::Read)); return mPosition - mRotation * mShape->GetCenterOfMass(); }

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
	inline const AABox &	GetWorldSpaceBounds() const										{ return mBounds; }

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
	const MotionProperties *GetMotionProperties() const										{ MOSS_ASSERT(!IsStatic()); return mMotionProperties; }
	MotionProperties *		GetMotionProperties()											{ MOSS_ASSERT(!IsStatic()); return mMotionProperties; }

	// Access to the motion properties (version that does not check if the object is kinematic or dynamic)
	const MotionProperties *GetMotionPropertiesUnchecked() const							{ return mMotionProperties; }
	MotionProperties *		GetMotionPropertiesUnchecked()									{ return mMotionProperties; }

	// Access to the user data, can be used for anything by the application
	uint64					GetUserData() const												{ return mUserData; }
	void					SetUserData(uint64 inUserData)									{ mUserData = inUserData; }

	// Get surface normal of a particular sub shape and its world space surface position on this body
	inline Vec3				GetWorldSpaceSurfaceNormal(const SubShapeID &inSubShapeID, RVec3Arg inPosition) const;

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
	static inline bool		sFindCollidingPairsCanCollide(const Body &inBody1, const Body &inBody2);

	// Update position using an Euler step (used during position integrate & constraint solving)
	inline void				AddPositionStep(Vec3Arg inLinearVelocityTimesDeltaTime)			{ MOSS_ASSERT(IsRigidBody()); MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sPositionAccess(), BodyAccess::EAccess::ReadWrite)); mPosition += mMotionProperties->LockTranslation(inLinearVelocityTimesDeltaTime); MOSS_ASSERT(!mPosition.IsNaN()); }
	inline void				SubPositionStep(Vec3Arg inLinearVelocityTimesDeltaTime)			{ MOSS_ASSERT(IsRigidBody()); MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sPositionAccess(), BodyAccess::EAccess::ReadWrite)); mPosition -= mMotionProperties->LockTranslation(inLinearVelocityTimesDeltaTime); MOSS_ASSERT(!mPosition.IsNaN()); }

	// Update rotation using an Euler step (used during position integrate & constraint solving)
	inline void				AddRotationStep(Vec3Arg inAngularVelocityTimesDeltaTime);
	inline void				SubRotationStep(Vec3Arg inAngularVelocityTimesDeltaTime);

	// Flag if body is in the broadphase (should only be called by the BroadPhase)
	inline void				SetInBroadPhaseInternal(bool inInBroadPhase)					{ if (inInBroadPhase) mFlags.fetch_or(uint8(EFlags::IsInBroadPhase), memory_order_relaxed); else mFlags.fetch_and(uint8(~uint8(EFlags::IsInBroadPhase)), memory_order_relaxed); }

	// Invalidate the contact cache (should only be called by the BodyManager), will be reset the next simulation step. Returns true if the contact cache was still valid.
	inline bool				InvalidateContactCacheInternal()								{ return (mFlags.fetch_or(uint8(EFlags::InvalidateContactCache), memory_order_relaxed) & uint8(EFlags::InvalidateContactCache)) == 0; }

	// Reset the collision cache invalid flag (should only be called by the BodyManager).
	inline void				ValidateContactCacheInternal()									{ MOSS_IF_ENABLE_ASSERTS(uint8 old_val = ) mFlags.fetch_and(uint8(~uint8(EFlags::InvalidateContactCache)), memory_order_relaxed); MOSS_ASSERT((old_val & uint8(EFlags::InvalidateContactCache)) != 0); }

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
	void					SetShapeInternal(const Shape *inShape, bool inUpdateMassProperties);

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
	void					SaveState(StateRecorder &inStream) const;

	// Restoring state for replay
	void					RestoreState(StateRecorder &inStream);

	//@}

	static constexpr uint32	cInactiveIndex = MotionProperties::cInactiveIndex;				// Constant indicating that body is not active

private:
	friend class BodyManager;
	friend class BodyWithMotionProperties;
	friend class SoftBodyWithMotionPropertiesAndShape;

							Body() = default;												// Bodies must be created through BodyInterface::CreateBody

	explicit				Body(bool);														// Alternative constructor that initializes all members

							~Body()															{ MOSS_ASSERT(mMotionProperties == nullptr); } // Bodies must be destroyed through BodyInterface::DestroyBody

	inline void				GetSleepTestPoints(RVec3 *outPoints) const;						// Determine points to test for checking if body is sleeping: COM, COM + largest bounding box axis, COM + second largest bounding box axis

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
	MotionProperties *		mMotionProperties = nullptr;									// If this is a keyframed or dynamic object, this object holds all information about the movement
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

class BodyFilter : public NonCopyable {
public:
	// Destructor
	virtual					~BodyFilter() = default;

	// Filter function. Returns true if we should collide with inBodyID
	virtual bool			ShouldCollide([[maybe_unused]] const BodyID &inBodyID) const { return true; }

	// Filter function. Returns true if we should collide with inBody (this is called after the body is locked and makes it possible to filter based on body members)
	virtual bool			ShouldCollideLocked([[maybe_unused]] const Body &inBody) const { return true; }
};

MOSS_API void EstimateCollisionResponse(const Body* body1, const Body* body2, const ContactManifold* manifold, float combinedFriction, float combinedRestitution, float minVelocityForRestitution, uint32_t numIterations, CollisionEstimationResult* result);

MOSS_NAMESPACE_END

#endif // JOLT_BODY3D_H
// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Physics/Body/Body.h>
#include <Moss/Physics/Collision/NarrowPhaseQuery.h>
#include <Moss/Physics/Collision/ContactListener.h>
#include <Moss/Physics/Constraints/Constraints.h>
#include <Moss/Physics/IslandBuilder.h>
#include <Moss/Physics/LargeIslandSplitter.h>
#include <Moss/Physics/PhysicsUpdateContext.h>

MOSS_SUPRESS_WARNINGS_BEGIN

class JobSystem;
class StateRecorder;
class TempAllocator;
class PhysicsStepListener;
class SoftBodyContactListener;
class SimShapeFilter;


// If objects are closer than this distance, they are considered to be colliding (used for GJK) (unit: meter)
constexpr float cDefaultCollisionTolerance = 1.0e-4f;

// A factor that determines the accuracy of the penetration depth calculation. If the change of the squared distance is less than tolerance * current_penetration_depth^2 the algorithm will terminate. (unit: dimensionless)
constexpr float cDefaultPenetrationTolerance = 1.0e-4f; //< Stop when there's less than 1% change

// How much padding to add around objects
constexpr float cDefaultConvexRadius = 0.05f;

// Used by (Tapered)CapsuleShape to determine when supporting face is an edge rather than a point (unit: meter)
static constexpr float cCapsuleProjectionSlop = 0.02f;

// Maximum amount of jobs to allow
constexpr int cMaxPhysicsJobs = 2048;

// Maximum amount of barriers to allow
constexpr int cMaxPhysicsBarriers = 8;

struct PhysicsSettings {
	MOSS_OVERRIDE_NEW_DELETE

	// Size of body pairs array, corresponds to the maximum amount of potential body pairs that can be in flight at any time.
	// Setting this to a low value will use less memory but slow down simulation as threads may run out of narrow phase work.
	int			mMaxInFlightBodyPairs = 16384;

	// How many PhysicsStepListeners to notify in 1 batch
	int			mStepListenersBatchSize = 8;

	// How many step listener batches are needed before spawning another job (set to INT_MAX if no parallelism is desired)
	int			mStepListenerBatchesPerJob = 1;

	// Baumgarte stabilization factor (how much of the position error to 'fix' in 1 update) (unit: dimensionless, 0 = nothing, 1 = 100%)
	float		mBaumgarte = 0.2f;

	// Radius around objects inside which speculative contact points will be detected. Note that if this is too big
	// you will get ghost collisions as speculative contacts are based on the closest points during the collision detection
	// step which may not be the actual closest points by the time the two objects hit (unit: meters)
	float		mSpeculativeContactDistance = 0.02f;

	// How much bodies are allowed to sink into each other (unit: meters)
	float		mPenetrationSlop = 0.02f;

	// Fraction of its inner radius a body must move per step to enable casting for the LinearCast motion quality
	float		mLinearCastThreshold = 0.75f;

	// Fraction of its inner radius a body may penetrate another body for the LinearCast motion quality
	float		mLinearCastMaxPenetration = 0.25f;

	// Max distance to use to determine if two points are on the same plane for determining the contact manifold between two shape faces (unit: meter)
	float		mManifoldTolerance = 1.0e-3f;

	// Maximum distance to correct in a single iteration when solving position constraints (unit: meters)
	float		mMaxPenetrationDistance = 0.2f;

	// Maximum relative delta position for body pairs to be able to reuse collision results from last frame (units: meter^2)
	float		mBodyPairCacheMaxDeltaPositionSq = Square(0.001f); //< 1 mm

	// Maximum relative delta orientation for body pairs to be able to reuse collision results from last frame, stored as cos(max angle / 2)
	float		mBodyPairCacheCosMaxDeltaRotationDiv2 = 0.99984769515639123915701155881391f; //< cos(2 degrees / 2)

	// Maximum angle between normals that allows manifolds between different sub shapes of the same body pair to be combined
	float		mContactNormalCosMaxDeltaRotation = 0.99619469809174553229501040247389f; //< cos(5 degree)

	// Maximum allowed distance between old and new contact point to preserve contact forces for warm start (units: meter^2)
	float		mContactPointPreserveLambdaMaxDistSq = Square(0.01f); //< 1 cm

	// Number of solver velocity iterations to run
	// Note that this needs to be >= 2 in order for friction to work (friction is applied using the non-penetration impulse from the previous iteration)
	uint		mNumVelocitySteps = 10;

	// Number of solver position iterations to run
	uint		mNumPositionSteps = 2;

	// Minimal velocity needed before a collision can be elastic. If the relative velocity between colliding objects
	// in the direction of the contact normal is lower than this, the restitution will be zero regardless of the configured
	// value. This lets an object settle sooner. Must be a positive number. (unit: m)
	float		mMinVelocityForRestitution = 1.0f;

	// Time before object is allowed to go to sleep (unit: seconds)
	float		mTimeBeforeSleep = 0.5f;

	// To detect if an object is sleeping, we use 3 points:
	// - The center of mass.
	// - The centers of the faces of the bounding box that are furthest away from the center.
	// The movement of these points is tracked and if the velocity of all 3 points is lower than this value,
	// the object is allowed to go to sleep. Must be a positive number. (unit: m/s)
	float		mPointVelocitySleepThreshold = 0.03f;

	// By default the simulation is deterministic, it is possible to turn this off by setting this setting to false. This will make the simulation run faster but it will no longer be deterministic.
	bool		mDeterministicSimulation = true;

	//@name These variables are mainly for debugging purposes, they allow turning on/off certain subsystems. You probably want to leave them alone.
	//@{

	// Whether or not to use warm starting for constraints (initially applying previous frames impulses)
	bool		mConstraintWarmStart = true;

	// Whether or not to use the body pair cache, which removes the need for narrow phase collision detection when orientation between two bodies didn't change
	bool		mUseBodyPairContactCache = true;

	// Whether or not to reduce manifolds with similar contact normals into one contact manifold (see description at Body::SetUseManifoldReduction)
	bool		mUseManifoldReduction = true;

	// If we split up large islands into smaller parallel batches of work (to improve performance)
	bool		mUseLargeIslandSplitter = true;

	// If objects can go to sleep or not
	bool		mAllowSleeping = true;

	// When false, we prevent collision against non-active (shared) edges. Mainly for debugging the algorithm.
	bool		mCheckActiveEdges = true;
};


/// The main class for the physics system. It contains all rigid bodies and simulates them.
///
/// The main simulation is performed by the Update() call on multiple threads (if the JobSystem is configured to use them). Please refer to the general architecture overview in the Docs folder for more information.
class MOSS_EXPORT PhysicsSystem : public NonCopyable
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor / Destructor
	PhysicsSystem()	: mContactManager(mPhysicsSettings) MOSS_IF_ENABLE_ASSERTS(, mConstraintManager(&mBodyManager)) { }
	~PhysicsSystem();

	/// The maximum value that can be passed to Init for inMaxBodies.
	static constexpr uint		cMaxBodiesLimit = BodyID::cMaxBodyIndex + 1;

	/// The maximum value that can be passed to Init for inMaxBodyPairs.
	/// Note you should really use a lower value, using this value will cost a lot of memory!
	/// On a 32 bit platform, you'll run out of memory way before you reach this limit.
	static constexpr uint		cMaxBodyPairsLimit = ContactConstraintManager::cMaxBodyPairsLimit;

	/// The maximum value that can be passed to Init for inMaxContactConstraints.
	/// Note you should really use a lower value, using this value will cost a lot of memory!
	/// On a 32 bit platform, you'll run out of memory way before you reach this limit.
	static constexpr uint		cMaxContactConstraintsLimit = ContactConstraintManager::cMaxContactConstraintsLimit;

	/// Initialize the system.
	/// @param inMaxBodies Maximum number of bodies to support.
	/// @param inNumBodyMutexes Number of body mutexes to use. Should be a power of 2 in the range [1, 64], use 0 to auto detect.
	/// @param inMaxBodyPairs Maximum amount of body pairs to process (anything else will fall through the world), this number should generally be much higher than the max amount of contact points as there will be lots of bodies close that are not actually touching.
	/// @param inMaxContactConstraints Maximum amount of contact constraints to process (anything else will fall through the world).
	/// @param inBroadPhaseLayerInterface Information on the mapping of object layers to broad phase layers. Since this is a virtual interface, the instance needs to stay alive during the lifetime of the PhysicsSystem.
	/// @param inObjectVsBroadPhaseLayerFilter Filter callback function that is used to determine if an object layer collides with a broad phase layer. Since this is a virtual interface, the instance needs to stay alive during the lifetime of the PhysicsSystem.
	/// @param inObjectLayerPairFilter Filter callback function that is used to determine if two object layers collide. Since this is a virtual interface, the instance needs to stay alive during the lifetime of the PhysicsSystem.
	void						Init(uint inMaxBodies, uint inNumBodyMutexes, uint inMaxBodyPairs, uint inMaxContactConstraints, const BroadPhaseLayerInterface &inBroadPhaseLayerInterface, const ObjectVsBroadPhaseLayerFilter &inObjectVsBroadPhaseLayerFilter, const ObjectLayerPairFilter &inObjectLayerPairFilter);

	/// Listener that is notified whenever a body is activated/deactivated
	void						SetBodyActivationListener(BodyActivationListener *inListener) { mBodyManager.SetBodyActivationListener(inListener); }
	BodyActivationListener *	GetBodyActivationListener() const							{ return mBodyManager.GetBodyActivationListener(); }

	/// Listener that is notified whenever a contact point between two bodies is added/updated/removed.
	/// You can't change contact listener during PhysicsSystem::Update but it can be changed at any other time.
	void						SetContactListener(ContactListener *inListener)				{ mContactManager.SetContactListener(inListener); }
	ContactListener *			GetContactListener() const									{ return mContactManager.GetContactListener(); }

	/// Listener that is notified whenever a contact point between a soft body and another body
	void						SetSoftBodyContactListener(SoftBodyContactListener *inListener) { mSoftBodyContactListener = inListener; }
	SoftBodyContactListener *	GetSoftBodyContactListener() const							{ return mSoftBodyContactListener; }

	/// Set the function that combines the friction of two bodies and returns it
	/// Default method is the geometric mean: sqrt(friction1 * friction2).
	void						SetCombineFriction(ContactConstraintManager::CombineFunction inCombineFriction) { mContactManager.SetCombineFriction(inCombineFriction); }
	ContactConstraintManager::CombineFunction GetCombineFriction() const					{ return mContactManager.GetCombineFriction(); }

	/// Set the function that combines the restitution of two bodies and returns it
	/// Default method is max(restitution1, restitution1)
	void						SetCombineRestitution(ContactConstraintManager::CombineFunction inCombineRestitution) { mContactManager.SetCombineRestitution(inCombineRestitution); }
	ContactConstraintManager::CombineFunction GetCombineRestitution() const					{ return mContactManager.GetCombineRestitution(); }

	/// Set/get the shape filter that will be used during simulation. This can be used to exclude shapes within a body from colliding with each other.
	/// E.g. if you have a high detail and a low detail collision model, you can attach them to the same body in a StaticCompoundShape and use the ShapeFilter
	/// to exclude the high detail collision model when simulating and exclude the low detail collision model when casting rays. Note that in this case
	/// you would need to pass the inverse of inShapeFilter to the CastRay function. Pass a nullptr to disable the shape filter.
	/// The PhysicsSystem does not own the ShapeFilter, make sure it stays alive during the lifetime of the PhysicsSystem.
	void						SetSimShapeFilter(const SimShapeFilter *inShapeFilter)		{ mSimShapeFilter = inShapeFilter; }
	const SimShapeFilter*		GetSimShapeFilter() const									{ return mSimShapeFilter; }

	/// Advanced use only: This function is similar to CollisionDispatch::sCollideShapeVsShape but only used to collide bodies during simulation.
	/// inBody1 The first body to collide.
	/// inBody2 The second body to collide.
	/// inCenterOfMassTransform1 The center of mass transform of the first body (note this will not be the actual world space position of the body, it will be made relative to some position so we can drop down to single precision).
	/// inCenterOfMassTransform2 The center of mass transform of the second body.
	/// ioCollideShapeSettings Settings that control the collision detection. Note that the implementation can freely overwrite the shape settings if needed, the caller provides a temporary that will not be used after the function returns.
	/// ioCollector The collector that will receive the contact points.
	/// inShapeFilter The shape filter that can be used to exclude shapes from colliding with each other.
	using SimCollideBodyVsBody = std::function<void(const Body &inBody1, const Body &inBody2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, CollideShapeSettings &ioCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter)>;

	/// Advanced use only: Set the function that will be used to collide two bodies during simulation.
	/// This function is expected to eventually call CollideShapeCollector::AddHit all contact points between the shapes of body 1 and 2 in their given transforms.
	void						SetSimCollideBodyVsBody(const SimCollideBodyVsBody &inBodyVsBody) { mSimCollideBodyVsBody = inBodyVsBody; }
	const SimCollideBodyVsBody &GetSimCollideBodyVsBody() const								{ return mSimCollideBodyVsBody; }

	/// Advanced use only: Default function that is used to collide two bodies during simulation.
	static void					sDefaultSimCollideBodyVsBody(const Body &inBody1, const Body &inBody2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, CollideShapeSettings &ioCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter);

	/// Control the main constants of the physics simulation
	void						SetPhysicsSettings(const PhysicsSettings &inSettings)		{ mPhysicsSettings = inSettings; }
	const PhysicsSettings &		GetPhysicsSettings() const									{ return mPhysicsSettings; }

	/// Access to the body interface. This interface allows to to create / remove bodies and to change their properties.
	const BodyInterface &		GetBodyInterface() const									{ return mBodyInterfaceLocking; }
	BodyInterface &				GetBodyInterface()											{ return mBodyInterfaceLocking; }
	const BodyInterface &		GetBodyInterfaceNoLock() const								{ return mBodyInterfaceNoLock; } ///< Version that does not lock the bodies, use with great care!
	BodyInterface &				GetBodyInterfaceNoLock()									{ return mBodyInterfaceNoLock; } ///< Version that does not lock the bodies, use with great care!

	/// Access to the broadphase interface that allows coarse collision queries
	const BroadPhaseQuery &		GetBroadPhaseQuery() const									{ return *mBroadPhase; }

	/// Interface that allows fine collision queries against first the broad phase and then the narrow phase.
	const NarrowPhaseQuery &	GetNarrowPhaseQuery() const									{ return mNarrowPhaseQueryLocking; }
	const NarrowPhaseQuery &	GetNarrowPhaseQueryNoLock() const							{ return mNarrowPhaseQueryNoLock; } ///< Version that does not lock the bodies, use with great care!

	/// Add constraint to the world
	void						AddConstraint(Constraint *inConstraint)						{ mConstraintManager.Add(&inConstraint, 1); }

	/// Remove constraint from the world
	void						RemoveConstraint(Constraint *inConstraint)					{ mConstraintManager.Remove(&inConstraint, 1); }

	/// Batch add constraints.
	void						AddConstraints(Constraint **inConstraints, int inNumber)	{ mConstraintManager.Add(inConstraints, inNumber); }

	/// Batch remove constraints.
	void						RemoveConstraints(Constraint **inConstraints, int inNumber)	{ mConstraintManager.Remove(inConstraints, inNumber); }

	/// Get a list of all constraints
	Constraints					GetConstraints() const										{ return mConstraintManager.GetConstraints(); }

	/// Optimize the broadphase, needed only if you've added many bodies prior to calling Update() for the first time.
	/// Don't call this every frame as PhysicsSystem::Update spreads out the same work over multiple frames.
	/// If you add many bodies through BodyInterface::AddBodiesPrepare/AddBodiesFinalize and if the bodies in a batch are
	/// in a roughly unoccupied space (e.g. a new level section) then a call to OptimizeBroadPhase is also not needed
	/// as batch adding creates an efficient bounding volume hierarchy.
	/// Don't call this function while bodies are being modified from another thread or use the locking BodyInterface to modify bodies.
	void						OptimizeBroadPhase();

	/// Adds a new step listener
	void						AddStepListener(PhysicsStepListener *inListener);

	/// Removes a step listener
	void						RemoveStepListener(PhysicsStepListener *inListener);

	/// Simulate the system.
	/// The world steps for a total of inDeltaTime seconds. This is divided in inCollisionSteps iterations.
	/// Each iteration consists of collision detection followed by an integration step.
	/// This function internally spawns jobs using inJobSystem and waits for them to complete, so no jobs will be running when this function returns.
	/// The temp allocator is used, for example, to store the list of bodies that are in contact, how they form islands together
	/// and data to solve the contacts between bodies. At the end of the Update call, all allocated memory will have been freed.
	EPhysicsUpdateError			Update(float inDeltaTime, int inCollisionSteps, TempAllocator *inTempAllocator, JobSystem *inJobSystem);

	/// Saving state for replay
	void						SaveState(StateRecorder &inStream, EStateRecorderState inState = EStateRecorderState::All, const StateRecorderFilter *inFilter = nullptr) const;

	/// Restoring state for replay. Returns false if failed.
	bool						RestoreState(StateRecorder &inStream, const StateRecorderFilter *inFilter = nullptr);

	/// Saving state of a single body.
	void						SaveBodyState(const Body &inBody, StateRecorder &inStream) const;

	/// Restoring state of a single body.
	void						RestoreBodyState(Body &ioBody, StateRecorder &inStream);

#ifndef MOSS_DEBUG_RENDERER
	// Drawing properties
	static bool					sDrawMotionQualityLinearCast;								///< Draw debug info for objects that perform continuous collision detection through the linear cast motion quality

	/// Draw the state of the bodies (debugging purposes)
	void						DrawBodies(const BodyManager::DrawSettings &inSettings, DebugRenderer *inRenderer, const BodyDrawFilter *inBodyFilter = nullptr) { mBodyManager.Draw(inSettings, mPhysicsSettings, inRenderer, inBodyFilter); }

	/// Draw the constraints only (debugging purposes)
	void						DrawConstraints(DebugRenderer *inRenderer)					{ mConstraintManager.DrawConstraints(inRenderer); }

	/// Draw the constraint limits only (debugging purposes)
	void						DrawConstraintLimits(DebugRenderer *inRenderer)				{ mConstraintManager.DrawConstraintLimits(inRenderer); }

	/// Draw the constraint reference frames only (debugging purposes)
	void						DrawConstraintReferenceFrame(DebugRenderer *inRenderer)		{ mConstraintManager.DrawConstraintReferenceFrame(inRenderer); }
#endif // MOSS_DEBUG_RENDERER

	/// Set gravity value
	void						SetGravity(Vec3Arg inGravity)								{ mGravity = inGravity; }
	Vec3						GetGravity() const											{ return mGravity; }

	/// Returns a locking interface that won't actually lock the body. Use with great care!
	inline const BodyLockInterfaceNoLock &	GetBodyLockInterfaceNoLock() const				{ return mBodyLockInterfaceNoLock; }

	/// Returns a locking interface that locks the body so other threads cannot modify it.
	inline const BodyLockInterfaceLocking &	GetBodyLockInterface() const					{ return mBodyLockInterfaceLocking; }

	/// Get an broadphase layer filter that uses the default pair filter and a specified object layer to determine if broadphase layers collide
	DefaultBroadPhaseLayerFilter GetDefaultBroadPhaseLayerFilter(ObjectLayer inLayer) const	{ return DefaultBroadPhaseLayerFilter(*mObjectVsBroadPhaseLayerFilter, inLayer); }

	/// Get an object layer filter that uses the default pair filter and a specified layer to determine if layers collide
	DefaultObjectLayerFilter	GetDefaultLayerFilter(ObjectLayer inLayer) const			{ return DefaultObjectLayerFilter(*mObjectLayerPairFilter, inLayer); }

	/// Gets the current amount of bodies that are in the body manager
	uint						GetNumBodies() const										{ return mBodyManager.GetNumBodies(); }

	/// Gets the current amount of active bodies that are in the body manager
	uint32						GetNumActiveBodies(EBodyType inType) const					{ return mBodyManager.GetNumActiveBodies(inType); }

	/// Get the maximum amount of bodies that this physics system supports
	uint						GetMaxBodies() const										{ return mBodyManager.GetMaxBodies(); }

	/// Helper struct that counts the number of bodies of each type
	using BodyStats = BodyManager::BodyStats;

	/// Get stats about the bodies in the body manager (slow, iterates through all bodies)
	BodyStats					GetBodyStats() const										{ return mBodyManager.GetBodyStats(); }

	/// Get copy of the list of all bodies under protection of a lock.
	/// @param outBodyIDs On return, this will contain the list of BodyIDs
	void						GetBodies(BodyIDVector &outBodyIDs) const					{ return mBodyManager.GetBodyIDs(outBodyIDs); }

	/// Get copy of the list of active bodies under protection of a lock.
	/// @param inType The type of bodies to get
	/// @param outBodyIDs On return, this will contain the list of BodyIDs
	void						GetActiveBodies(EBodyType inType, BodyIDVector &outBodyIDs) const { return mBodyManager.GetActiveBodies(inType, outBodyIDs); }

	/// Get the list of active bodies, use GetNumActiveBodies() to find out how long the list is.
	/// Note: Not thread safe. The active bodies list can change at any moment when other threads are doing work. Use GetActiveBodies() if you need a thread safe version.
	const BodyID *				GetActiveBodiesUnsafe(EBodyType inType) const				{ return mBodyManager.GetActiveBodiesUnsafe(inType); }

	/// Check if 2 bodies were in contact during the last simulation step. Since contacts are only detected between active bodies, so at least one of the bodies must be active in order for this function to work.
	/// It queries the state at the time of the last PhysicsSystem::Update and will return true if the bodies were in contact, even if one of the bodies was moved / removed afterwards.
	/// This function can be called from any thread when the PhysicsSystem::Update is not running. During PhysicsSystem::Update this function is only valid during contact callbacks:
	/// - During the ContactListener::OnContactAdded callback this function can be used to determine if a different contact pair between the bodies was active in the previous simulation step (function returns true) or if this is the first step that the bodies are touching (function returns false).
	/// - During the ContactListener::OnContactRemoved callback this function can be used to determine if this is the last contact pair between the bodies (function returns false) or if there are other contacts still present (function returns true).
	bool						WereBodiesInContact(const BodyID &inBody1ID, const BodyID &inBody2ID) const { return mContactManager.WereBodiesInContact(inBody1ID, inBody2ID); }

	/// Get the bounding box of all bodies in the physics system
	AABox						GetBounds() const											{ return mBroadPhase->GetBounds(); }

#ifdef MOSS_TRACK_BROADPHASE_STATS
	/// Trace the accumulated broadphase stats to the TTY
	void						ReportBroadphaseStats()										{ mBroadPhase->ReportStats(); }
#endif // MOSS_TRACK_BROADPHASE_STATS

private:
	using CCDBody = PhysicsUpdateContext::Step::CCDBody;

	// Various job entry points
	void						JobStepListeners(PhysicsUpdateContext::Step *ioStep);
	void						JobDetermineActiveConstraints(PhysicsUpdateContext::Step *ioStep) const;
	void						JobApplyGravity(const PhysicsUpdateContext *ioContext, PhysicsUpdateContext::Step *ioStep);
	void						JobSetupVelocityConstraints(float inDeltaTime, PhysicsUpdateContext::Step *ioStep) const;
	void						JobBuildIslandsFromConstraints(PhysicsUpdateContext *ioContext, PhysicsUpdateContext::Step *ioStep);
	void						JobFindCollisions(PhysicsUpdateContext::Step *ioStep, int inJobIndex);
	void						JobFinalizeIslands(PhysicsUpdateContext *ioContext);
	void						JobBodySetIslandIndex();
	void						JobSolveVelocityConstraints(PhysicsUpdateContext *ioContext, PhysicsUpdateContext::Step *ioStep);
	void						JobPreIntegrateVelocity(PhysicsUpdateContext *ioContext, PhysicsUpdateContext::Step *ioStep);
	void						JobIntegrateVelocity(const PhysicsUpdateContext *ioContext, PhysicsUpdateContext::Step *ioStep);
	void						JobPostIntegrateVelocity(PhysicsUpdateContext *ioContext, PhysicsUpdateContext::Step *ioStep) const;
	void						JobFindCCDContacts(const PhysicsUpdateContext *ioContext, PhysicsUpdateContext::Step *ioStep);
	void						JobResolveCCDContacts(PhysicsUpdateContext *ioContext, PhysicsUpdateContext::Step *ioStep);
	void						JobContactRemovedCallbacks(const PhysicsUpdateContext::Step *ioStep);
	void						JobSolvePositionConstraints(PhysicsUpdateContext *ioContext, PhysicsUpdateContext::Step *ioStep);
	void						JobSoftBodyPrepare(PhysicsUpdateContext *ioContext, PhysicsUpdateContext::Step *ioStep);
	void						JobSoftBodyCollide(PhysicsUpdateContext *ioContext) const;
	void						JobSoftBodySimulate(PhysicsUpdateContext *ioContext, uint inThreadIndex) const;
	void						JobSoftBodyFinalize(PhysicsUpdateContext *ioContext);

	/// Tries to spawn a new FindCollisions job if max concurrency hasn't been reached yet
	void						TrySpawnJobFindCollisions(PhysicsUpdateContext::Step *ioStep) const;

	using ContactAllocator = ContactConstraintManager::ContactAllocator;

	/// Process narrow phase for a single body pair
	void						ProcessBodyPair(ContactAllocator &ioContactAllocator, const BodyPair &inBodyPair);

	/// This helper batches up bodies that need to put to sleep to avoid contention on the activation mutex
	class BodiesToSleep;

	/// Called at the end of JobSolveVelocityConstraints to check if bodies need to go to sleep and to update their bounding box in the broadphase
	void						CheckSleepAndUpdateBounds(uint32 inIslandIndex, const PhysicsUpdateContext *ioContext, const PhysicsUpdateContext::Step *ioStep, BodiesToSleep &ioBodiesToSleep);

	/// Number of constraints to process at once in JobDetermineActiveConstraints
	static constexpr int		cDetermineActiveConstraintsBatchSize = 64;

	/// Number of constraints to process at once in JobSetupVelocityConstraints, we want a low number of threads working on this so we take fairly large batches
	static constexpr int		cSetupVelocityConstraintsBatchSize = 256;

	/// Number of bodies to process at once in JobApplyGravity
	static constexpr int		cApplyGravityBatchSize = 64;

	/// Number of active bodies to test for collisions per batch
	static constexpr int		cActiveBodiesBatchSize = 16;

	/// Number of active bodies to integrate velocities for
	static constexpr int		cIntegrateVelocityBatchSize = 64;

	/// Number of contacts that need to be queued before another narrow phase job is started
	static constexpr int		cNarrowPhaseBatchSize = 16;

	/// Number of continuous collision shape casts that need to be queued before another job is started
	static constexpr int		cNumCCDBodiesPerJob = 4;

	/// Broadphase layer filter that decides if two objects can collide
	const ObjectVsBroadPhaseLayerFilter *mObjectVsBroadPhaseLayerFilter = nullptr;

	/// Object layer filter that decides if two objects can collide
	const ObjectLayerPairFilter *mObjectLayerPairFilter = nullptr;

	/// The body manager keeps track which bodies are in the simulation
	BodyManager					mBodyManager;

	/// Body locking interfaces
	BodyLockInterfaceNoLock		mBodyLockInterfaceNoLock { mBodyManager };
	BodyLockInterfaceLocking	mBodyLockInterfaceLocking { mBodyManager };

	/// Body interfaces
	BodyInterface				mBodyInterfaceNoLock;
	BodyInterface				mBodyInterfaceLocking;

	/// Narrow phase query interface
	NarrowPhaseQuery			mNarrowPhaseQueryNoLock;
	NarrowPhaseQuery			mNarrowPhaseQueryLocking;

	/// The broadphase does quick collision detection between body pairs
	BroadPhase *				mBroadPhase = nullptr;

	/// The soft body contact listener
	SoftBodyContactListener *	mSoftBodyContactListener = nullptr;

	/// The shape filter that is used to filter out sub shapes during simulation
	const SimShapeFilter *		mSimShapeFilter = nullptr;

	/// The collision function that is used to collide two shapes during simulation
	SimCollideBodyVsBody		mSimCollideBodyVsBody = &sDefaultSimCollideBodyVsBody;

	/// Simulation settings
	PhysicsSettings				mPhysicsSettings;

	/// The contact manager resolves all contacts during a simulation step
	ContactConstraintManager	mContactManager;

	/// All non-contact constraints
	ConstraintManager			mConstraintManager;

	/// Keeps track of connected bodies and builds islands for multithreaded velocity/position update
	IslandBuilder				mIslandBuilder;

	/// Will split large islands into smaller groups of bodies that can be processed in parallel
	LargeIslandSplitter			mLargeIslandSplitter;

	/// Mutex protecting mStepListeners
	Mutex						mStepListenersMutex;

	/// List of physics step listeners
	using StepListeners = TArray<PhysicsStepListener *>;
	StepListeners				mStepListeners;

	/// This is the global gravity vector
	Vec3						mGravity = Vec3(0, -9.81f, 0);

	/// Previous frame's delta time of one sub step to allow scaling previous frame's constraint impulses
	float						mPreviousStepDeltaTime = 0.0f;
};

MOSS_SUPRESS_WARNINGS_END

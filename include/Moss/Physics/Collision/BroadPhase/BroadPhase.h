// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Core/Mutex.h>
#include <Moss/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Moss/Physics/Collision/ObjectLayer.h>
#include <Moss/Physics/Collision/CollisionCollector.h>
#include <Moss/Physics/Body/BodyID.h>
#include <Moss/Core/NonCopyable.h>

MOSS_NAMESPACE_BEGIN

// Shorthand function to ifdef out code if broadphase stats tracking is off
#ifdef MOSS_TRACK_BROADPHASE_STATS
	#define MOSS_IF_TRACK_BROADPHASE_STATS(...) __VA_ARGS__
#else
	#define MOSS_IF_TRACK_BROADPHASE_STATS(...)
#endif // MOSS_TRACK_BROADPHASE_STATS

class BodyManager;
struct BodyPair;

using BodyPairCollector = CollisionCollector<BodyPair, CollisionCollectorTraitsCollideShape>;

struct RayCast;
class BroadPhaseCastResult;
class AABox;
class OrientedBox;
struct AABoxCast;

// Various collector configurations
using RayCastBodyCollector = CollisionCollector<BroadPhaseCastResult, CollisionCollectorTraitsCastRay>;
using CastShapeBodyCollector = CollisionCollector<BroadPhaseCastResult, CollisionCollectorTraitsCastShape>;
using CollideShapeBodyCollector = CollisionCollector<BodyID, CollisionCollectorTraitsCollideShape>;

/// Interface to the broadphase that can perform collision queries. These queries will only test the bounding box of the body to quickly determine a potential set of colliding bodies.
/// The shapes of the bodies are not tested, if you want this then you should use the NarrowPhaseQuery interface.
class MOSS_EXPORT BroadPhaseQuery : public NonCopyable
{
public:
	/// Virtual destructor
	virtual				~BroadPhaseQuery() = default;

	/// Cast a ray and add any hits to ioCollector
	virtual void		CastRay(const RayCast &inRay, RayCastBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter = { }, const ObjectLayerFilter &inObjectLayerFilter = { }) const = 0;

	/// Get bodies intersecting with inBox and any hits to ioCollector
	virtual void		CollideAABox(const AABox &inBox, CollideShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter = { }, const ObjectLayerFilter &inObjectLayerFilter = { }) const = 0;

	/// Get bodies intersecting with a sphere and any hits to ioCollector
	virtual void		CollideSphere(Vec3Arg inCenter, float inRadius, CollideShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter = { }, const ObjectLayerFilter &inObjectLayerFilter = { }) const = 0;

	/// Get bodies intersecting with a point and any hits to ioCollector
	virtual void		CollidePoint(Vec3Arg inPoint, CollideShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter = { }, const ObjectLayerFilter &inObjectLayerFilter = { }) const = 0;

	/// Get bodies intersecting with an oriented box and any hits to ioCollector
	virtual void		CollideOrientedBox(const OrientedBox &inBox, CollideShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter = { }, const ObjectLayerFilter &inObjectLayerFilter = { }) const = 0;

	/// Cast a box and add any hits to ioCollector
	virtual void		CastAABox(const AABoxCast &inBox, CastShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter = { }, const ObjectLayerFilter &inObjectLayerFilter = { }) const = 0;
};

class BroadPhaseLayer {
public:
	using Type = uint8;

	MOSS_INLINE						BroadPhaseLayer() = default;
	MOSS_INLINE explicit constexpr	BroadPhaseLayer(Type inValue) : mValue(inValue) { }
	MOSS_INLINE constexpr			BroadPhaseLayer(const BroadPhaseLayer &) = default;
	MOSS_INLINE BroadPhaseLayer &	operator = (const BroadPhaseLayer &) = default;

	MOSS_INLINE constexpr bool		operator == (const BroadPhaseLayer &inRHS) const
	{
		return mValue == inRHS.mValue;
	}

	MOSS_INLINE constexpr bool		operator != (const BroadPhaseLayer &inRHS) const
	{
		return mValue != inRHS.mValue;
	}

	MOSS_INLINE constexpr bool		operator < (const BroadPhaseLayer &inRHS) const
	{
		return mValue < inRHS.mValue;
	}

	MOSS_INLINE explicit constexpr	operator Type() const
	{
		return mValue;
	}

	MOSS_INLINE Type					GetValue() const
	{
		return mValue;
	}

private:
	Type							mValue;
};

/// Constant value used to indicate an invalid broad phase layer
static constexpr BroadPhaseLayer cBroadPhaseLayerInvalid(0xff);

/// Interface that the application should implement to allow mapping object layers to broadphase layers
class MOSS_EXPORT BroadPhaseLayerInterface : public NonCopyable
{
public:
	/// Destructor
	virtual							~BroadPhaseLayerInterface() = default;

	/// Return the number of broadphase layers there are
	virtual uint					GetNumBroadPhaseLayers() const = 0;

	/// Convert an object layer to the corresponding broadphase layer
	virtual BroadPhaseLayer			GetBroadPhaseLayer(ObjectLayer inLayer) const = 0;

#if defined(MOSS_EXTERNAL_PROFILE) || defined(MOSS_PROFILE_ENABLED)
	/// Get the user readable name of a broadphase layer (debugging purposes)
	virtual const char *			GetBroadPhaseLayerName(BroadPhaseLayer inLayer) const = 0;
#endif // MOSS_EXTERNAL_PROFILE || MOSS_PROFILE_ENABLED
};

/// Class to test if an object can collide with a broadphase layer. Used while finding collision pairs.
class MOSS_EXPORT ObjectVsBroadPhaseLayerFilter : public NonCopyable
{
public:
	/// Destructor
	virtual							~ObjectVsBroadPhaseLayerFilter() = default;

	/// Returns true if an object layer should collide with a broadphase layer
	virtual bool					ShouldCollide([[maybe_unused]] ObjectLayer inLayer1, [[maybe_unused]] BroadPhaseLayer inLayer2) const
	{
		return true;
	}
};

/// Filter class for broadphase layers
class MOSS_EXPORT BroadPhaseLayerFilter : public NonCopyable
{
public:
	/// Destructor
	virtual							~BroadPhaseLayerFilter() = default;

	/// Function to filter out broadphase layers when doing collision query test (return true to allow testing against objects with this layer)
	virtual bool					ShouldCollide([[maybe_unused]] BroadPhaseLayer inLayer) const
	{
		return true;
	}
};

/// Default filter class that uses the pair filter in combination with a specified layer to filter layers
class MOSS_EXPORT DefaultBroadPhaseLayerFilter : public BroadPhaseLayerFilter
{
public:
	/// Constructor
									DefaultBroadPhaseLayerFilter(const ObjectVsBroadPhaseLayerFilter &inObjectVsBroadPhaseLayerFilter, ObjectLayer inLayer) :
		mObjectVsBroadPhaseLayerFilter(inObjectVsBroadPhaseLayerFilter),
		mLayer(inLayer)
	{
	}

	// See BroadPhaseLayerFilter::ShouldCollide
	virtual bool					ShouldCollide(BroadPhaseLayer inLayer) const override
	{
		return mObjectVsBroadPhaseLayerFilter.ShouldCollide(mLayer, inLayer);
	}

private:
	const ObjectVsBroadPhaseLayerFilter &mObjectVsBroadPhaseLayerFilter;
	ObjectLayer						mLayer;
};

/// Allows objects from a specific broad phase layer only
class MOSS_EXPORT SpecifiedBroadPhaseLayerFilter : public BroadPhaseLayerFilter
{
public:
	/// Constructor
	explicit						SpecifiedBroadPhaseLayerFilter(BroadPhaseLayer inLayer) :
		mLayer(inLayer)
	{
	}

	// See BroadPhaseLayerFilter::ShouldCollide
	virtual bool					ShouldCollide(BroadPhaseLayer inLayer) const override
	{
		return mLayer == inLayer;
	}

private:
	BroadPhaseLayer					mLayer;
};

/// Used to do coarse collision detection operations to quickly prune out bodies that will not collide.
class MOSS_EXPORT BroadPhase : public BroadPhaseQuery
{
public:
	/// Initialize the broadphase.
	/// @param inBodyManager The body manager singleton
	/// @param inLayerInterface Interface that maps object layers to broadphase layers.
	/// Note that the broadphase takes a pointer to the data inside inObjectToBroadPhaseLayer so this object should remain static.
	virtual void		Init(BodyManager *inBodyManager, const BroadPhaseLayerInterface &inLayerInterface);

	/// Should be called after many objects have been inserted to make the broadphase more efficient, usually done on startup only
	virtual void		Optimize()															{ /* Optionally overridden by implementation */ }

	/// Must be called just before updating the broadphase when none of the body mutexes are locked
	virtual void		FrameSync()															{ /* Optionally overridden by implementation */ }

	/// Must be called before UpdatePrepare to prevent modifications from being made to the tree
	virtual void		LockModifications()													{ /* Optionally overridden by implementation */ }

	/// Context used during broadphase update
	struct UpdateState { void *mData[4]; };

	/// Update the broadphase, needs to be called frequently to update the internal state when bodies have been modified.
	/// The UpdatePrepare() function can run in a background thread without influencing the broadphase
	virtual	UpdateState	UpdatePrepare()														{ return UpdateState(); }

	/// Finalizing the update will quickly apply the changes
	virtual void		UpdateFinalize([[maybe_unused]] const UpdateState &inUpdateState)	{ /* Optionally overridden by implementation */ }

	/// Must be called after UpdateFinalize to allow modifications to the broadphase
	virtual void		UnlockModifications()												{ /* Optionally overridden by implementation */ }

	/// Handle used during adding bodies to the broadphase
	using AddState = void *;

	/// Prepare adding inNumber bodies at ioBodies to the broadphase, returns a handle that should be used in AddBodiesFinalize/Abort.
	/// This can be done on a background thread without influencing the broadphase.
	/// ioBodies may be shuffled around by this function and should be kept that way until AddBodiesFinalize/Abort is called.
	virtual AddState	AddBodiesPrepare([[maybe_unused]] BodyID *ioBodies, [[maybe_unused]] int inNumber) { return nullptr; } // By default the broadphase doesn't support this

	/// Finalize adding bodies to the broadphase, supply the return value of AddBodiesPrepare in inAddState.
	/// Please ensure that the ioBodies array passed to AddBodiesPrepare is unmodified and passed again to this function.
	virtual void		AddBodiesFinalize(BodyID *ioBodies, int inNumber, AddState inAddState) = 0;

	/// Abort adding bodies to the broadphase, supply the return value of AddBodiesPrepare in inAddState.
	/// This can be done on a background thread without influencing the broadphase.
	/// Please ensure that the ioBodies array passed to AddBodiesPrepare is unmodified and passed again to this function.
	virtual void		AddBodiesAbort([[maybe_unused]] BodyID *ioBodies, [[maybe_unused]] int inNumber, [[maybe_unused]] AddState inAddState)	{ /* By default nothing needs to be done */ }

	/// Remove inNumber bodies in ioBodies from the broadphase.
	/// ioBodies may be shuffled around by this function.
	virtual void		RemoveBodies(BodyID *ioBodies, int inNumber) = 0;

	/// Call whenever the aabb of a body changes (can change order of ioBodies array)
	/// inTakeLock should be false if we're between LockModifications/UnlockModifications, in which case care needs to be taken to not call this between UpdatePrepare/UpdateFinalize
	virtual void		NotifyBodiesAABBChanged(BodyID *ioBodies, int inNumber, bool inTakeLock = true) = 0;

	/// Call whenever the layer (and optionally the aabb as well) of a body changes (can change order of ioBodies array)
	virtual void		NotifyBodiesLayerChanged(BodyID *ioBodies, int inNumber) = 0;

	/// Find all colliding pairs between dynamic bodies
	/// Note that this function is very specifically tailored for the PhysicsSystem::Update function, hence it is not part of the BroadPhaseQuery interface.
	/// One of the assumptions it can make is that no locking is needed during the query as it will only be called during a very particular part of the update.
	/// @param ioActiveBodies is a list of bodies for which we need to find colliding pairs (this function can change the order of the ioActiveBodies array). This can be a subset of the set of active bodies in the system.
	/// @param inNumActiveBodies is the size of the ioActiveBodies array.
	/// @param inSpeculativeContactDistance Distance at which speculative contact points will be created.
	/// @param inObjectVsBroadPhaseLayerFilter is the filter that determines if an object can collide with a broadphase layer.
	/// @param inObjectLayerPairFilter is the filter that determines if two objects can collide.
	/// @param ioPairCollector receives callbacks for every body pair found.
	virtual void		FindCollidingPairs(BodyID *ioActiveBodies, int inNumActiveBodies, float inSpeculativeContactDistance, const ObjectVsBroadPhaseLayerFilter &inObjectVsBroadPhaseLayerFilter, const ObjectLayerPairFilter &inObjectLayerPairFilter, BodyPairCollector &ioPairCollector) const = 0;

	/// Same as BroadPhaseQuery::CastAABox but can be implemented in a way to take no broad phase locks.
	virtual void		CastAABoxNoLock(const AABoxCast &inBox, CastShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter) const = 0;

	/// Get the bounding box of all objects in the broadphase
	virtual AABox		GetBounds() const = 0;

#ifdef MOSS_TRACK_BROADPHASE_STATS
	/// Trace the collected broadphase stats in CSV form.
	/// This report can be used to judge and tweak the efficiency of the broadphase.
	virtual void		ReportStats()														{ /* Can be implemented by derived classes */ }
#endif // MOSS_TRACK_BROADPHASE_STATS

protected:
	/// Link to the body manager that manages the bodies in this broadphase
	BodyManager *		mBodyManager = nullptr;
};


class MOSS_EXPORT BroadPhaseBruteForce final : public BroadPhase
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	// Implementing interface of BroadPhase (see BroadPhase for documentation)
	virtual void		AddBodiesFinalize(BodyID *ioBodies, int inNumber, AddState inAddState) override;
	virtual void		RemoveBodies(BodyID *ioBodies, int inNumber) override;
	virtual void		NotifyBodiesAABBChanged(BodyID *ioBodies, int inNumber, bool inTakeLock) override;
	virtual void		NotifyBodiesLayerChanged(BodyID *ioBodies, int inNumber) override;
	virtual void		CastRay(const RayCast &inRay, RayCastBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter) const override;
	virtual void		CollideAABox(const AABox &inBox, CollideShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter) const override;
	virtual void		CollideSphere(Vec3Arg inCenter, float inRadius, CollideShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter) const override;
	virtual void		CollidePoint(Vec3Arg inPoint, CollideShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter) const override;
	virtual void		CollideOrientedBox(const OrientedBox &inBox, CollideShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter) const override;
	virtual void		CastAABoxNoLock(const AABoxCast &inBox, CastShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter) const override;
	virtual void		CastAABox(const AABoxCast &inBox, CastShapeBodyCollector &ioCollector, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter) const override;
	virtual void		FindCollidingPairs(BodyID *ioActiveBodies, int inNumActiveBodies, float inSpeculativeContactDistance, const ObjectVsBroadPhaseLayerFilter &inObjectVsBroadPhaseLayerFilter, const ObjectLayerPairFilter &inObjectLayerPairFilter, BodyPairCollector &ioPairCollector) const override;
	virtual AABox		GetBounds() const override;

private:
	TArray<BodyID>		mBodyIDs;
	mutable SharedMutex	mMutex;
};

MOSS_NAMESPACE_END

// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT


#ifndef MOSS_PHYSICS_INTERN_H
#define MOSS_PHYSICS_INTERN_H

#include <Moss/Moss_Physics.h>

MOSS_SUPRESS_WARNINGS_BEGIN






// IDs

class MOSS_EXPORT CharacterID {
public:
	MOSS_OVERRIDE_NEW_DELETE

	static constexpr uint32	cInvalidCharacterID = 0xffffffff;	///< The value for an invalid character ID

	/// Construct invalid character ID
	CharacterID() : mID(cInvalidCharacterID) { }

	/// Construct with specific value, make sure you don't use the same value twice!
	explicit CharacterID(uint32 inID) : mID(inID) { }

	/// Get the numeric value of the ID
	inline uint32 GetValue() const { return mID; }

	/// Check if the ID is valid
	inline bool IsInvalid() const { return mID == cInvalidCharacterID; }

	/// Equals check
	inline bool operator == (const CharacterID &inRHS) const { return mID == inRHS.mID; }

	/// Not equals check
	inline bool operator != (const CharacterID &inRHS) const { return mID != inRHS.mID; }

	/// Smaller than operator, can be used for sorting characters
	inline bool	operator < (const CharacterID &inRHS) const { return mID < inRHS.mID; }

	/// Greater than operator, can be used for sorting characters
	inline bool operator > (const CharacterID &inRHS) const { return mID > inRHS.mID; }

	/// Get the hash for this character ID
	inline uint64 GetHash() const { return Hash<uint32>{} (mID); }

	/// Generate the next available character ID
	static CharacterID sNextCharacterID() {
		for (;;) {
			uint32 next = sNextID.fetch_add(1, std::memory_order_relaxed);
			if (next != cInvalidCharacterID)
				return CharacterID(next);
		}
	}

	/// Set the next available character ID, can be used after destroying all character to prepare for a second deterministic run
	static void sSetNextCharacterID(uint32 inNextValue = 1) { sNextID.store(inNextValue, std::memory_order_relaxed); }

private:
	inline static atomic<uint32> sNextID = 1;   // Next character ID to be assigned
	uint32					mID;                // ID value
};



class MOSS_EXPORT BodyID {
public:
	MOSS_OVERRIDE_NEW_DELETE

	static constexpr uint32	cInvalidBodyID = 0xffffffff;	///< The value for an invalid body ID
	static constexpr uint32	cBroadPhaseBit = 0x80000000;	///< This bit is used by the broadphase
	static constexpr uint32	cMaxBodyIndex = 0x7fffff;		///< Maximum value for body index (also the maximum amount of bodies supported - 1)
	static constexpr uint8	cMaxSequenceNumber = 0xff;		///< Maximum value for the sequence number
	static constexpr uint	cSequenceNumberShift = 23;		///< Number of bits to shift to get the sequence number

	/// Construct invalid body ID
	BodyID() : mID(cInvalidBodyID) {}

	/// Construct from index and sequence number combined in a single uint32 (use with care!)
	explicit BodyID(uint32 inID) : mID(inID) { MOSS_ASSERT((inID & cBroadPhaseBit) == 0 || inID == cInvalidBodyID); } // Check bit used by broadphase

	/// Construct from index and sequence number
	explicit BodyID(uint32 inID, uint8 inSequenceNumber) : mID((uint32(inSequenceNumber) << cSequenceNumberShift) | inID) { MOSS_ASSERT(inID <= cMaxBodyIndex); } // Should not overlap with broadphase bit or sequence number

	/// Get index in body array
	inline uint32 GetIndex() const { return mID & cMaxBodyIndex; }

	/// Get sequence number of body.
	/// The sequence number can be used to check if a body ID with the same body index has been reused by another body.
	/// It is mainly used in multi threaded situations where a body is removed and its body index is immediately reused by a body created from another thread.
	/// Functions querying the broadphase can (after acquiring a body lock) detect that the body has been removed (we assume that this won't happen more than 128 times in a row).
	inline uint8 GetSequenceNumber() const { return uint8(mID >> cSequenceNumberShift); }

	/// Returns the index and sequence number combined in an uint32
	inline uint32 GetIndexAndSequenceNumber() const { return mID; }

	/// Check if the ID is valid
	inline bool	IsInvalid() const { return mID == cInvalidBodyID; }

	/// Equals check
	inline bool	operator == (const BodyID &inRHS) const { return mID == inRHS.mID; }

	/// Not equals check
	inline bool	operator != (const BodyID &inRHS) const { return mID != inRHS.mID; }

	/// Smaller than operator, can be used for sorting bodies
	inline bool	operator < (const BodyID &inRHS) const { return mID < inRHS.mID; }

	/// Greater than operator, can be used for sorting bodies
	inline bool	operator > (const BodyID &inRHS) const { return mID > inRHS.mID; }

private:
	uint32					mID;
};


// Constraints


// Collision


// Geometry


// Bodies


// Body


// SoftBody
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

	/// Initialize the soft body motion properties
	void								Initialize(const SoftBodyCreationSettings &inSettings);

	/// Get the shared settings of the soft body
	const SoftBodySharedSettings *		GetSettings() const							{ return mSettings; }

	/// Get the vertices of the soft body
	const TArray<Vertex> &				GetVertices() const							{ return mVertices; }
	TArray<Vertex> &						GetVertices()								{ return mVertices; }

	/// Access an individual vertex
	const Vertex &						GetVertex(uint inIndex) const				{ return mVertices[inIndex]; }
	Vertex &							GetVertex(uint inIndex)						{ return mVertices[inIndex]; }

	/// Get the materials of the soft body
	const PhysicsMaterialList &			GetMaterials() const						{ return mSettings->mMaterials; }

	/// Get the faces of the soft body
	const TArray<Face> &					GetFaces() const							{ return mSettings->mFaces; }

	/// Access to an individual face
	const Face &						GetFace(uint inIndex) const					{ return mSettings->mFaces[inIndex]; }

	/// Get the number of solver iterations
	uint32								GetNumIterations() const					{ return mNumIterations; }
	void								SetNumIterations(uint32 inNumIterations)	{ mNumIterations = inNumIterations; }

	/// Get the pressure of the soft body
	float								GetPressure() const							{ return mPressure; }
	void								SetPressure(float inPressure)				{ mPressure = inPressure; }

	/// Update the position of the body while simulating (set to false for something that is attached to the static world)
	bool								GetUpdatePosition() const					{ return mUpdatePosition; }
	void								SetUpdatePosition(bool inUpdatePosition)	{ mUpdatePosition = inUpdatePosition; }

	/// Global setting to turn on/off skin constraints
	bool								GetEnableSkinConstraints() const			{ return mEnableSkinConstraints; }
	void								SetEnableSkinConstraints(bool inEnableSkinConstraints) { mEnableSkinConstraints = inEnableSkinConstraints; }

	/// Multiplier applied to Skinned::mMaxDistance to allow tightening or loosening of the skin constraints. 0 to hard skin all vertices.
	float								GetSkinnedMaxDistanceMultiplier() const		{ return mSkinnedMaxDistanceMultiplier; }
	void								SetSkinnedMaxDistanceMultiplier(float inSkinnedMaxDistanceMultiplier) { mSkinnedMaxDistanceMultiplier = inSkinnedMaxDistanceMultiplier; }

	/// Get local bounding box
	const AABox &						GetLocalBounds() const						{ return mLocalBounds; }

	/// Get the volume of the soft body. Note can become negative if the shape is inside out!
	float								GetVolume() const							{ return GetVolumeTimesSix() / 6.0f; }

	/// Calculate the total mass and inertia of this body based on the current state of the vertices
	void								CalculateMassAndInertia();

#ifndef MOSS_DEBUG_RENDERER
	/// Draw the state of a soft body
	void								DrawVertices(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform) const;
	void								DrawVertexVelocities(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform) const;
	void								DrawEdgeConstraints(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, ESoftBodyConstraintColor inConstraintColor) const;
	void								DrawBendConstraints(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, ESoftBodyConstraintColor inConstraintColor) const;
	void								DrawVolumeConstraints(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, ESoftBodyConstraintColor inConstraintColor) const;
	void								DrawSkinConstraints(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, ESoftBodyConstraintColor inConstraintColor) const;
	void								DrawLRAConstraints(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, ESoftBodyConstraintColor inConstraintColor) const;
	void								DrawPredictedBounds(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform) const;
#endif // MOSS_DEBUG_RENDERER

	/// Saving state for replay
	void								SaveState(StateRecorder &inStream) const;

	/// Restoring state for replay
	void								RestoreState(StateRecorder &inStream);

	/// Skin vertices to supplied joints, information is used by the skinned constraints.
	/// @param inCenterOfMassTransform Value of Body::GetCenterOfMassTransform().
	/// @param inJointMatrices The joint matrices must be expressed relative to inCenterOfMassTransform.
	/// @param inNumJoints Indicates how large the inJointMatrices array is (used only for validating out of bounds).
	/// @param inHardSkinAll Can be used to position all vertices on the skinned vertices and can be used to hard reset the soft body.
	/// @param ioTempAllocator Allocator.
	void								SkinVertices(RMat44Arg inCenterOfMassTransform, const Mat44 *inJointMatrices, uint inNumJoints, bool inHardSkinAll, TempAllocator &ioTempAllocator);

	/// This function allows you to update the soft body immediately without going through the PhysicsSystem.
	/// This is useful if the soft body is teleported and needs to 'settle' or it can be used if a the soft body
	/// is not added to the PhysicsSystem and needs to be updated manually. One reason for not adding it to the
	/// PhysicsSystem is that you might want to update a soft body immediately after updating an animated object
	/// that has the soft body attached to it. If the soft body is added to the PhysicsSystem it will be updated
	/// by it, so calling this function will effectively update it twice. Note that when you use this function,
	/// only the current thread will be used, whereas if you update through the PhysicsSystem, multiple threads may
	/// be used.
	/// Note that this will bypass any sleep checks. Since the dynamic objects that the soft body touches
	/// will not move during this call, there can be simulation artifacts if you call this function multiple times
	/// without running the physics simulation step.
	void								CustomUpdate(float inDeltaTime, Body &ioSoftBody, PhysicsSystem &inSystem);

	////////////////////////////////////////////////////////////
	// FUNCTIONS BELOW THIS LINE ARE FOR INTERNAL USE ONLY
	////////////////////////////////////////////////////////////

	/// Initialize the update context. Not part of the public API.
	void								InitializeUpdateContext(float inDeltaTime, Body &inSoftBody, const PhysicsSystem &inSystem, SoftBodyUpdateContext &ioContext);

	/// Do a broad phase check and collect all bodies that can possibly collide with this soft body. Not part of the public API.
	void								DetermineCollidingShapes(const SoftBodyUpdateContext &inContext, const PhysicsSystem &inSystem, const BodyLockInterface &inBodyLockInterface);

	/// Return code for ParallelUpdate
	enum class EStatus
	{
		NoWork	= 1 << 0,				///< No work was done because other threads were still working on a batch that cannot run concurrently
		DidWork	= 1 << 1,				///< Work was done to progress the update
		Done	= 1 << 2,				///< All work is done
	};

	/// Update the soft body, will process a batch of work. Not part of the public API.
	EStatus								ParallelUpdate(SoftBodyUpdateContext &ioContext, const PhysicsSettings &inPhysicsSettings);

	/// Update the velocities of all rigid bodies that we collided with. Not part of the public API.
	void								UpdateRigidBodyVelocities(const SoftBodyUpdateContext &inContext, BodyInterface &inBodyInterface);

private:
	// SoftBodyManifold needs to have access to CollidingShape
	friend class SoftBodyManifold;

	// Information about a leaf shape that we're colliding with
	struct LeafShape
	{
										LeafShape() = default;
										LeafShape(Mat44Arg inTransform, Vec3Arg inScale, const Shape *inShape) : mTransform(inTransform), mScale(inScale), mShape(inShape) { }

		Mat44							mTransform;									///< Transform of the shape relative to the soft body
		Vec3							mScale;										///< Scale of the shape
		RefConst<Shape>					mShape;										///< Shape
	};

	// Collect information about the colliding bodies
	struct CollidingShape
	{
		/// Get the velocity of a point on this body
		Vec3							GetPointVelocity(Vec3Arg inPointRelativeToCOM) const
		{
			return mLinearVelocity + mAngularVelocity.Cross(inPointRelativeToCOM);
		}

		Mat44							mCenterOfMassTransform;						///< Transform of the body relative to the soft body
		TArray<LeafShape>				mShapes;									///< Leaf shapes of the body we hit
		BodyID							mBodyID;									///< Body ID of the body we hit
		EMotionType						mMotionType;								///< Motion type of the body we hit
		float							mInvMass;									///< Inverse mass of the body we hit
		float							mFriction;									///< Combined friction of the two bodies
		float							mRestitution;								///< Combined restitution of the two bodies
		float							mSoftBodyInvMassScale;						///< Scale factor for the inverse mass of the soft body vertices
		bool							mUpdateVelocities;							///< If the linear/angular velocity changed and the body needs to be updated
		Mat44							mInvInertia;								///< Inverse inertia in local space to the soft body
		Vec3							mLinearVelocity;							///< Linear velocity of the body in local space to the soft body
		Vec3							mAngularVelocity;							///< Angular velocity of the body in local space to the soft body
		Vec3							mOriginalLinearVelocity;					///< Linear velocity of the body in local space to the soft body at start
		Vec3							mOriginalAngularVelocity;					///< Angular velocity of the body in local space to the soft body at start
	};

	// Collect information about the colliding sensors
	struct CollidingSensor
	{
		Mat44							mCenterOfMassTransform;						///< Transform of the body relative to the soft body
		TArray<LeafShape>				mShapes;									///< Leaf shapes of the body we hit
		BodyID							mBodyID;									///< Body ID of the body we hit
		bool							mHasContact;								///< If the sensor collided with the soft body
	};

	// Information about the state of all skinned vertices
	struct SkinState
	{
		Vec3							mPreviousPosition = Vec3::sZero();			///< Previous position of the skinned vertex, used to interpolate between the previous and current position
		Vec3							mPosition = Vec3::sNaN();					///< Current position of the skinned vertex
		Vec3							mNormal = Vec3::sNaN();						///< Normal of the skinned vertex
	};

	/// Do a narrow phase check and determine the closest feature that we can collide with
	void								DetermineCollisionPlanes(uint inVertexStart, uint inNumVertices);

	/// Do a narrow phase check between a single sensor and the soft body
	void								DetermineSensorCollisions(CollidingSensor &ioSensor);

	/// Apply pressure force and update the vertex velocities
	void								ApplyPressure(const SoftBodyUpdateContext &inContext);

	/// Integrate the positions of all vertices by 1 sub step
	void								IntegratePositions(const SoftBodyUpdateContext &inContext);

	/// Enforce all bend constraints
	void								ApplyDihedralBendConstraints(const SoftBodyUpdateContext &inContext, uint inStartIndex, uint inEndIndex);

	/// Enforce all volume constraints
	void								ApplyVolumeConstraints(const SoftBodyUpdateContext &inContext, uint inStartIndex, uint inEndIndex);

	/// Enforce all skin constraints
	void								ApplySkinConstraints(const SoftBodyUpdateContext &inContext, uint inStartIndex, uint inEndIndex);

	/// Enforce all edge constraints
	void								ApplyEdgeConstraints(const SoftBodyUpdateContext &inContext, uint inStartIndex, uint inEndIndex);

	/// Enforce all LRA constraints
	void								ApplyLRAConstraints(uint inStartIndex, uint inEndIndex);

	/// Enforce all collision constraints & update all velocities according the XPBD algorithm
	void								ApplyCollisionConstraintsAndUpdateVelocities(const SoftBodyUpdateContext &inContext);

	/// Update the state of the soft body (position, velocity, bounds)
	void								UpdateSoftBodyState(SoftBodyUpdateContext &ioContext, const PhysicsSettings &inPhysicsSettings);

	/// Start the first solver iteration
	void								StartFirstIteration(SoftBodyUpdateContext &ioContext);

	/// Executes tasks that need to run on the start of an iteration (i.e. the stuff that can't run in parallel)
	void								StartNextIteration(const SoftBodyUpdateContext &ioContext);

	/// Helper function for ParallelUpdate that works on batches of collision planes
	EStatus								ParallelDetermineCollisionPlanes(SoftBodyUpdateContext &ioContext);

	/// Helper function for ParallelUpdate that works on sensor collisions
	EStatus								ParallelDetermineSensorCollisions(SoftBodyUpdateContext &ioContext);

	/// Helper function for ParallelUpdate that works on batches of constraints
	EStatus								ParallelApplyConstraints(SoftBodyUpdateContext &ioContext, const PhysicsSettings &inPhysicsSettings);

	/// Helper function to update a single group of constraints
	void								ProcessGroup(const SoftBodyUpdateContext &ioContext, uint inGroupIndex);

	/// Returns 6 times the volume of the soft body
	float								GetVolumeTimesSix() const;

#ifndef MOSS_DEBUG_RENDERER
	/// Helper function to draw constraints
	template <typename GetEndIndex, typename DrawConstraint>
		inline void						DrawConstraints(ESoftBodyConstraintColor inConstraintColor, const GetEndIndex &inGetEndIndex, const DrawConstraint &inDrawConstraint, ColorArg inBaseColor) const;

	RMat44								mSkinStateTransform = RMat44::sIdentity();	///< The matrix that transforms mSkinState to world space
#endif // MOSS_DEBUG_RENDERER

	RefConst<SoftBodySharedSettings>	mSettings;									///< Configuration of the particles and constraints
	TArray<Vertex>						mVertices;									///< Current state of all vertices in the simulation
	TArray<CollidingShape>				mCollidingShapes;							///< List of colliding shapes retrieved during the last update
	TArray<CollidingSensor>				mCollidingSensors;							///< List of colliding sensors retrieved during the last update
	TArray<SkinState>					mSkinState;									///< List of skinned positions (1-on-1 with mVertices but only those that are used by the skinning constraints are filled in)
	AABox								mLocalBounds;								///< Bounding box of all vertices
	AABox								mLocalPredictedBounds;						///< Predicted bounding box for all vertices using extrapolation of velocity by last step delta time
	uint32								mNumIterations;								///< Number of solver iterations
	uint								mNumSensors;								///< Workaround for TSAN false positive: store mCollidingSensors.size() in a separate variable.
	float								mPressure;									///< n * R * T, amount of substance * ideal gas constant * absolute temperature, see https://en.wikipedia.org/wiki/Pressure
	float								mSkinnedMaxDistanceMultiplier = 1.0f;		///< Multiplier applied to Skinned::mMaxDistance to allow tightening or loosening of the skin constraints
	bool								mUpdatePosition;							///< Update the position of the body while simulating (set to false for something that is attached to the static world)
	atomic<bool>						mNeedContactCallback = false;				///< True if the soft body has collided with anything in the last update
	bool								mEnableSkinConstraints = true;				///< If skin constraints are enabled
	bool								mSkinStatePreviousPositionValid = false;	///< True if the skinning was updated in the last update so that the previous position of the skin state is valid
};

class MOSS_EXPORT SoftBodyShape final : public Shape {
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
									SoftBodyShape()											: Shape(EShapeType::SoftBody, EShapeSubType::SoftBody) { }

	/// Determine amount of bits needed to encode sub shape id
	uint							GetSubShapeIDBits() const;

	/// Convert a sub shape ID back to a face index
	uint32							GetFaceIndex(const SubShapeID &inSubShapeID) const;

	// See Shape
	virtual bool					MustBeStatic() const override							{ return false; }
	virtual Vec3					GetCenterOfMass() const override						{ return Vec3::sZero(); }
	virtual AABox					GetLocalBounds() const override;
	virtual uint					GetSubShapeIDBitsRecursive() const override				{ return GetSubShapeIDBits(); }
	virtual float					GetInnerRadius() const override							{ return 0.0f; }
	virtual MassProperties			GetMassProperties() const override						{ return MassProperties(); }
	virtual const PhysicsMaterial *	GetMaterial(const SubShapeID &inSubShapeID) const override;
	virtual Vec3					GetSurfaceNormal(const SubShapeID &inSubShapeID, Vec3Arg inLocalSurfacePosition) const override;
	virtual void					GetSupportingFace(const SubShapeID &inSubShapeID, Vec3Arg inDirection, Vec3Arg inScale, Mat44Arg inCenterOfMassTransform, SupportingFace &outVertices) const override;
	virtual void					GetSubmergedVolume(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const Plane &inSurface, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outCenterOfBuoyancy
#ifndef MOSS_DEBUG_RENDERER // Not using MOSS_IF_DEBUG_RENDERER for Doxygen
		, RVec3Arg inBaseOffset
#endif
		) const override;
#ifndef MOSS_DEBUG_RENDERER
	virtual void					Draw(DebugRenderer *inRenderer, RMat44Arg inCenterOfMassTransform, Vec3Arg inScale, ColorArg inColor, bool inUseMaterialColors, bool inDrawWireframe) const override;
#endif // MOSS_DEBUG_RENDERER
	virtual bool					CastRay(const RayCast &inRay, const SubShapeIDCreator &inSubShapeIDCreator, RayCastResult &ioHit) const override;
	virtual void					CastRay(const RayCast &inRay, const RayCastSettings &inRayCastSettings, const SubShapeIDCreator &inSubShapeIDCreator, CastRayCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;
	virtual void					CollidePoint(Vec3Arg inPoint, const SubShapeIDCreator &inSubShapeIDCreator, CollidePointCollector &ioCollector, const ShapeFilter &inShapeFilter = { }) const override;
	virtual void					CollideSoftBodyVertices(Mat44Arg inCenterOfMassTransform, Vec3Arg inScale, const CollideSoftBodyVertexIterator &inVertices, uint inNumVertices, int inCollidingShapeIndex) const override;
	virtual void					GetTrianglesStart(GetTrianglesContext &ioContext, const AABox &inBox, Vec3Arg inPositionCOM, QuatArg inRotation, Vec3Arg inScale) const override;
	virtual int						GetTrianglesNext(GetTrianglesContext &ioContext, int inMaxTrianglesRequested, Float3 *outTriangleVertices, const PhysicsMaterial **outMaterials = nullptr) const override;
	virtual Stats					GetStats() const override;
	virtual float					GetVolume() const override;

	// Register shape functions with the registry
	static void						sRegister();

private:
	// Helper functions called by CollisionDispatch
	static void						sCollideConvexVsSoftBody(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter);
	static void						sCollideSphereVsSoftBody(const Shape *inShape1, const Shape *inShape2, Vec3Arg inScale1, Vec3Arg inScale2, Mat44Arg inCenterOfMassTransform1, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, const CollideShapeSettings &inCollideShapeSettings, CollideShapeCollector &ioCollector, const ShapeFilter &inShapeFilter);
	static void						sCastConvexVsSoftBody(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector);
	static void						sCastSphereVsSoftBody(const ShapeCast &inShapeCast, const ShapeCastSettings &inShapeCastSettings, const Shape *inShape, Vec3Arg inScale, const ShapeFilter &inShapeFilter, Mat44Arg inCenterOfMassTransform2, const SubShapeIDCreator &inSubShapeIDCreator1, const SubShapeIDCreator &inSubShapeIDCreator2, CastShapeCollector &ioCollector);

	struct SBSGetTrianglesContext;

	friend class BodyManager;

	const SoftBodyMotionProperties *mSoftBodyMotionProperties;
};




//#define MOSS_ENABLE_DETERMINISM_LOG
#ifdef MOSS_ENABLE_DETERMINISM_LOG

#include <Moss/Physics/Body/BodyID.h>
#include <Moss/Physics/Collision/Shape/SubShapeID.h>

MOSS_SUPPRESS_WARNINGS_STD_BEGIN
#include <iomanip>
#include <fstream>
MOSS_SUPPRESS_WARNINGS_STD_END

/// A simple class that logs the state of the simulation. The resulting text file can be used to diff between platforms and find issues in determinism.
class DeterminismLog {
private:
	MOSS_INLINE uint32 Convert(float inValue) const { return *(uint32 *)&inValue; }

	MOSS_INLINE uint64 Convert(double inValue) const { return *(uint64 *)&inValue; }

public:
	DeterminismLog() {
		mLog.open("detlog.txt", std::ios::out | std::ios::trunc | std::ios::binary); // Binary because we don't want a difference between Unix and Windows line endings.
		mLog.fill('0');
	}

	DeterminismLog& operator << (char inValue) {
		mLog << inValue;
		return *this;
	}

	DeterminismLog & operator << (const char *inValue) {
		mLog << std::dec << inValue;
		return *this;
	}

	DeterminismLog &		operator << (const string &inValue) {
		mLog << std::dec << inValue;
		return *this;
	}

	DeterminismLog &		operator << (const BodyID &inValue) {
		mLog << std::hex << std::setw(8) << inValue.GetIndexAndSequenceNumber();
		return *this;
	}

	DeterminismLog &		operator << (const SubShapeID &inValue) {
		mLog << std::hex << std::setw(8) << inValue.GetValue();
		return *this;
	}

	DeterminismLog &		operator << (float inValue) {
		mLog << std::hex << std::setw(8) << Convert(inValue);
		return *this;
	}

	DeterminismLog &		operator << (int inValue)
	{
		mLog << inValue;
		return *this;
	}

	DeterminismLog &		operator << (uint32 inValue)
	{
		mLog << std::hex << std::setw(8) << inValue;
		return *this;
	}

	DeterminismLog &		operator << (uint64 inValue)
	{
		mLog << std::hex << std::setw(16) << inValue;
		return *this;
	}

	DeterminismLog &		operator << (Vec3Arg inValue)
	{
		mLog << std::hex << std::setw(8) << Convert(inValue.GetX()) << " " << std::setw(8) << Convert(inValue.GetY()) << " " << std::setw(8) << Convert(inValue.GetZ());
		return *this;
	}

	DeterminismLog &		operator << (DVec3Arg inValue)
	{
		mLog << std::hex << std::setw(16) << Convert(inValue.GetX()) << " " << std::setw(16) << Convert(inValue.GetY()) << " " << std::setw(16) << Convert(inValue.GetZ());
		return *this;
	}

	DeterminismLog &		operator << (Vec4Arg inValue)
	{
		mLog << std::hex << std::setw(8) << Convert(inValue.GetX()) << " " << std::setw(8) << Convert(inValue.GetY()) << " " << std::setw(8) << Convert(inValue.GetZ()) << " " << std::setw(8) << Convert(inValue.GetW());
		return *this;
	}

	DeterminismLog &		operator << (const Float3 &inValue)
	{
		mLog << std::hex << std::setw(8) << Convert(inValue.x) << " " << std::setw(8) << Convert(inValue.y) << " " << std::setw(8) << Convert(inValue.z);
		return *this;
	}

	DeterminismLog &		operator << (Mat44Arg inValue)
	{
		*this << inValue.GetColumn4(0) << " " << inValue.GetColumn4(1) << " " << inValue.GetColumn4(2) << " " << inValue.GetColumn4(3);
		return *this;
	}

	DeterminismLog &		operator << (DMat44Arg inValue)
	{
		*this << inValue.GetColumn4(0) << " " << inValue.GetColumn4(1) << " " << inValue.GetColumn4(2) << " " << inValue.GetTranslation();
		return *this;
	}

	DeterminismLog &		operator << (QuatArg inValue)
	{
		*this << inValue.GetXYZW();
		return *this;
	}

	// Singleton instance
	static DeterminismLog	sLog;

private:
	std::ofstream			mLog;
};

/// Will log something to the determinism log, usage: MOSS_DET_LOG("label " << value);
#define MOSS_DET_LOG(...)	DeterminismLog::sLog << __VA_ARGS__ << '\n'

MOSS_SUPRESS_WARNINGS_END

#else

MOSS_SUPRESS_WARNINGS_BEGIN

/// By default we log nothing
#define MOSS_DET_LOG(...)

MOSS_SUPRESS_WARNINGS_END

// Create a std::hash/MOSS::Hash for BodyID
MOSS_MAKE_HASHABLE(BodyID, t.GetIndexAndSequenceNumber())

#endif // MOSS_PHYSICS_INTERN_H
// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Core/Reference.h>
#include <Moss/Core/Result.h>
#include <Moss/Core/StreamUtils.h>

MOSS_NAMESPACE_BEGIN

class StreamIn;
class StreamOut;

/// Resource that contains the joint hierarchy for a skeleton
class MOSS_EXPORT Skeleton : public RefTarget<Skeleton>
{
	MOSS_DECLARE_SERIALIZABLE_NON_VIRTUAL(MOSS_EXPORT, Skeleton)

public:
	using SkeletonResult = Result<Ref<Skeleton>>;

	/// Declare internal structure for a joint
	class Joint
	{
		MOSS_DECLARE_SERIALIZABLE_NON_VIRTUAL(MOSS_EXPORT, Joint)

	public:
							Joint() = default;
							Joint(const string_view &inName, const string_view &inParentName, int inParentJointIndex) : mName(inName), mParentName(inParentName), mParentJointIndex(inParentJointIndex) { }

		String				mName;																		///< Name of the joint
		String				mParentName;																///< Name of parent joint
		int					mParentJointIndex = -1;														///< Index of parent joint (in mJoints) or -1 if it has no parent
	};

	using JointVector = TArray<Joint>;

	///@name Access to the joints
	///@{
	const JointVector &		GetJoints() const															{ return mJoints; }
	JointVector &			GetJoints()																	{ return mJoints; }
	int						GetJointCount() const														{ return (int)mJoints.size(); }
	const Joint &			GetJoint(int inJoint) const													{ return mJoints[inJoint]; }
	Joint &					GetJoint(int inJoint)														{ return mJoints[inJoint]; }
	uint					AddJoint(const string_view &inName, const string_view &inParentName = string_view()) { mJoints.emplace_back(inName, inParentName, -1); return (uint)mJoints.size() - 1; }
	uint					AddJoint(const string_view &inName, int inParentIndex)						{ mJoints.emplace_back(inName, inParentIndex >= 0? mJoints[inParentIndex].mName : String(), inParentIndex); return (uint)mJoints.size() - 1; }
	///@}

	/// Find joint by name
	int						GetJointIndex(const string_view &inName) const;

	/// Fill in parent joint indices based on name
	void					CalculateParentJointIndices();

	/// Many of the algorithms that use the Skeleton class require that parent joints are in the mJoints array before their children.
	/// This function returns true if this is the case, false if not.
	bool					AreJointsCorrectlyOrdered() const;

	/// Saves the state of this object in binary form to inStream.
	void					SaveBinaryState(StreamOut &inStream) const;

	/// Restore the state of this object from inStream.
	static SkeletonResult	sRestoreFromBinaryState(StreamIn &inStream);

private:
	/// Joints
	JointVector				mJoints;
};


// Class that is able to map a low detail (ragdoll) skeleton to a high detail (animation) skeleton and vice versa
class MOSS_EXPORT SkeletonMapper : public RefTarget<SkeletonMapper>
{
public:
	/// A joint that maps 1-on-1 to a joint in the other skeleton
	class Mapping
	{
	public:
							Mapping() = default;
							Mapping(int inJointIdx1, int inJointIdx2, Mat44Arg inJoint1To2) : mJointIdx1(inJointIdx1), mJointIdx2(inJointIdx2), mJoint1To2(inJoint1To2), mJoint2To1(inJoint1To2.Inversed())
		{
			// Ensure bottom right element is 1 (numerical imprecision in the inverse can make this not so)
			mJoint2To1(3, 3) = 1.0f;
		}

		int					mJointIdx1;																	///< Index of joint from skeleton 1
		int					mJointIdx2;																	///< Corresponding index of joint from skeleton 2
		Mat44				mJoint1To2;																	///< Transforms this joint from skeleton 1 to 2
		Mat44				mJoint2To1;																	///< Inverse of the transform above
	};

	/// A joint chain that starts with a 1-on-1 mapped joint and ends with a 1-on-1 mapped joint with intermediate joints that cannot be mapped
	class Chain
	{
	public:
							Chain() = default;
							Chain(TArray<int> &&inJointIndices1, TArray<int> &&inJointIndices2) : mJointIndices1(std::move(inJointIndices1)), mJointIndices2(std::move(inJointIndices2)) { }

		TArray<int>			mJointIndices1;																///< Joint chain from skeleton 1
		TArray<int>			mJointIndices2;																///< Corresponding joint chain from skeleton 2
	};

	/// Joints that could not be mapped from skeleton 1 to 2
	class Unmapped
	{
	public:
							Unmapped() = default;
							Unmapped(int inJointIdx, int inParentJointIdx) : mJointIdx(inJointIdx), mParentJointIdx(inParentJointIdx) { }

		int					mJointIdx;																	///< Joint index of unmappable joint
		int					mParentJointIdx;															///< Parent joint index of unmappable joint
	};

	/// Joints that should have their translation locked (fixed)
	class Locked
	{
	public:
		int					mJointIdx;																	///< Joint index of joint with locked translation (in skeleton 2)
		int					mParentJointIdx;															///< Parent joint index of joint with locked translation (in skeleton 2)
		Vec3				mTranslation;																///< Translation of neutral pose
	};

	/// A function that is called to determine if a joint can be mapped from source to target skeleton
	using CanMapJoint = function<bool (const Skeleton *, int, const Skeleton *, int)>;

	/// Default function that checks if the names of the joints are equal
	static bool				sDefaultCanMapJoint(const Skeleton *inSkeleton1, int inIndex1, const Skeleton *inSkeleton2, int inIndex2)
	{
		return inSkeleton1->GetJoint(inIndex1).mName == inSkeleton2->GetJoint(inIndex2).mName;
	}

	/// Initialize the skeleton mapper. Skeleton 1 should be the (low detail) ragdoll skeleton and skeleton 2 the (high detail) animation skeleton.
	/// We assume that each joint in skeleton 1 can be mapped to a joint in skeleton 2 (if not mapping from animation skeleton to ragdoll skeleton will be undefined).
	/// Skeleton 2 should have the same hierarchy as skeleton 1 but can contain extra joints between those in skeleton 1 and it can have extra joints at the root and leaves of the skeleton.
	/// @param inSkeleton1 Source skeleton to map from.
	/// @param inNeutralPose1 Neutral pose of the source skeleton (model space)
	/// @param inSkeleton2 Target skeleton to map to.
	/// @param inNeutralPose2 Neutral pose of the target skeleton (model space), inNeutralPose1 and inNeutralPose2 must match as closely as possible, preferably the position of the mappable joints should be identical.
	/// @param inCanMapJoint Function that checks if joints in skeleton 1 and skeleton 2 are equal.
	void					Initialize(const Skeleton *inSkeleton1, const Mat44 *inNeutralPose1, const Skeleton *inSkeleton2, const Mat44 *inNeutralPose2, const CanMapJoint &inCanMapJoint = sDefaultCanMapJoint);

	/// This can be called so lock the translation of a specified set of joints in skeleton 2.
	/// Because constraints are never 100% rigid, there's always a little bit of stretch in the ragdoll when the ragdoll is under stress.
	/// Locking the translations of the pose will remove the visual stretch from the ragdoll but will introduce a difference between the
	/// physical simulation and the visual representation.
	/// @param inSkeleton2 Target skeleton to map to.
	/// @param inLockedTranslations An array of bools the size of inSkeleton2->GetJointCount(), for each joint indicating if the joint is locked.
	/// @param inNeutralPose2 Neutral pose to take reference translations from
	void					LockTranslations(const Skeleton *inSkeleton2, const bool *inLockedTranslations, const Mat44 *inNeutralPose2);

	/// After Initialize(), this can be called to lock the translation of all joints in skeleton 2 below the first mapped joint to those of the neutral pose.
	/// Because constraints are never 100% rigid, there's always a little bit of stretch in the ragdoll when the ragdoll is under stress.
	/// Locking the translations of the pose will remove the visual stretch from the ragdoll but will introduce a difference between the
	/// physical simulation and the visual representation.
	/// @param inSkeleton2 Target skeleton to map to.
	/// @param inNeutralPose2 Neutral pose to take reference translations from
	void					LockAllTranslations(const Skeleton *inSkeleton2, const Mat44 *inNeutralPose2);

	/// Map a pose. Joints that were directly mappable will be copied in model space from pose 1 to pose 2. Any joints that are only present in skeleton 2
	/// will get their model space transform calculated through the local space transforms of pose 2. Joints that are part of a joint chain between two
	/// mapped joints will be reoriented towards the next joint in skeleton 1. This means that it is possible for unmapped joints to have some animation,
	/// but very extreme animation poses will show artifacts.
	/// @param inPose1ModelSpace Pose on skeleton 1 in model space
	/// @param inPose2LocalSpace Pose on skeleton 2 in local space (used for the joints that cannot be mapped)
	/// @param outPose2ModelSpace Model space pose on skeleton 2 (the output of the mapping)
	void					Map(const Mat44 *inPose1ModelSpace, const Mat44 *inPose2LocalSpace, Mat44 *outPose2ModelSpace) const;

	/// Reverse map a pose, this will only use the mappings and not the chains (it assumes that all joints in skeleton 1 are mapped)
	/// @param inPose2ModelSpace Model space pose on skeleton 2
	/// @param outPose1ModelSpace When the function returns this will contain the model space pose for skeleton 1
	void					MapReverse(const Mat44 *inPose2ModelSpace, Mat44 *outPose1ModelSpace) const;

	/// Search through the directly mapped joints (mMappings) and find inJoint1Idx, returns the corresponding Joint2Idx or -1 if not found.
	int						GetMappedJointIdx(int inJoint1Idx) const;

	/// Search through the locked translations (mLockedTranslations) and find if joint inJoint2Idx is locked.
	bool					IsJointTranslationLocked(int inJoint2Idx) const;

	using MappingVector = TArray<Mapping>;
	using ChainVector = TArray<Chain>;
	using UnmappedVector = TArray<Unmapped>;
	using LockedVector = TArray<Locked>;

	///@name Access to the mapped joints
	///@{
	const MappingVector &	GetMappings() const															{ return mMappings; }
	MappingVector &			GetMappings()																{ return mMappings; }
	const ChainVector &		GetChains() const															{ return mChains; }
	ChainVector &			GetChains()																	{ return mChains; }
	const UnmappedVector &	GetUnmapped() const															{ return mUnmapped; }
	UnmappedVector &		GetUnmapped()																{ return mUnmapped; }
	const LockedVector &	GetLockedTranslations() const												{ return mLockedTranslations; }
	LockedVector &			GetLockedTranslations()														{ return mLockedTranslations; }
	///@}

private:
	/// Joint mappings
	MappingVector			mMappings;
	ChainVector				mChains;
	UnmappedVector			mUnmapped;																	///< Joint indices that could not be mapped from 1 to 2 (these are indices in 2)
	LockedVector			mLockedTranslations;
};

// Instance of a skeleton, contains the pose the current skeleton is in
class MOSS_EXPORT SkeletonPose
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	using JointState = SkeletalAnimation::JointState;
	using JointStateVector = TArray<JointState>;
	using Mat44Vector = TArray<Mat44>;

	///@name Skeleton
	///@{
	void						SetSkeleton(const Skeleton *inSkeleton);
	const Skeleton *			GetSkeleton() const														{ return mSkeleton; }
	///@}

	/// Extra offset applied to the root (and therefore also to all of its children)
	void						SetRootOffset(RVec3Arg inOffset)										{ mRootOffset = inOffset; }
	RVec3						GetRootOffset() const													{ return mRootOffset; }

	///@name Properties of the joints
	///@{
	uint						GetJointCount() const													{ return (uint)mJoints.size(); }
	const JointStateVector &	GetJoints() const														{ return mJoints; }
	JointStateVector &			GetJoints()																{ return mJoints; }
	const JointState &			GetJoint(int inJoint) const												{ return mJoints[inJoint]; }
	JointState &				GetJoint(int inJoint)													{ return mJoints[inJoint]; }
	///@}

	///@name Joint matrices
	///@{
	const Mat44Vector &			GetJointMatrices() const												{ return mJointMatrices; }
	Mat44Vector &				GetJointMatrices()														{ return mJointMatrices; }
	const Mat44 &				GetJointMatrix(int inJoint) const										{ return mJointMatrices[inJoint]; }
	Mat44 &						GetJointMatrix(int inJoint)												{ return mJointMatrices[inJoint]; }
	///@}

	/// Convert the joint states to joint matrices
	void						CalculateJointMatrices();

	/// Convert joint matrices to joint states
	void						CalculateJointStates();

	/// Outputs the joint matrices in local space (ensure that outMatrices has GetJointCount() elements, assumes that values in GetJoints() is up to date)
	void						CalculateLocalSpaceJointMatrices(Mat44 *outMatrices) const;

#ifndef MOSS_DEBUG_RENDERER
	/// Draw settings
	struct DrawSettings
	{
		bool					mDrawJoints = true;
		bool					mDrawJointOrientations = true;
		bool					mDrawJointNames = false;
	};

	/// Draw current pose
	void						Draw(const DrawSettings &inDrawSettings, DebugRenderer *inRenderer, RMat44Arg inOffset = RMat44::sIdentity()) const;
#endif // MOSS_DEBUG_RENDERER

private:
	RefConst<Skeleton>			mSkeleton;																///< Skeleton definition
	RVec3						mRootOffset { RVec3::sZero() };											///< Extra offset applied to the root (and therefore also to all of its children)
	JointStateVector			mJoints;																///< Local joint orientations (local to parent Joint)
	Mat44Vector					mJointMatrices;															///< Local joint matrices (local to world matrix)
};

// Resource for a skinned animation
class MOSS_EXPORT SkeletalAnimation : public RefTarget<SkeletalAnimation>
{
	MOSS_DECLARE_SERIALIZABLE_NON_VIRTUAL(MOSS_EXPORT, SkeletalAnimation)

public:
	/// Contains the current state of a joint, a local space transformation relative to its parent joint
	class JointState
	{
		MOSS_DECLARE_SERIALIZABLE_NON_VIRTUAL(MOSS_EXPORT, JointState)

	public:
		/// Convert from a local space matrix
		void							FromMatrix(Mat44Arg inMatrix);

		/// Convert to matrix representation
		inline Mat44					ToMatrix() const									{ return Mat44::sRotationTranslation(mRotation, mTranslation); }

		Quat							mRotation = Quat::sIdentity();						///< Local space rotation of the joint
		Vec3							mTranslation = Vec3::sZero();						///< Local space translation of the joint
	};

	/// Contains the state of a single joint at a particular time
	class Keyframe : public JointState
	{
		MOSS_DECLARE_SERIALIZABLE_NON_VIRTUAL(MOSS_EXPORT, Keyframe)

	public:
		float							mTime = 0.0f;										///< Time of keyframe in seconds
	};

	using KeyframeVector = TArray<Keyframe>;

	/// Contains the animation for a single joint
	class AnimatedJoint
	{
		MOSS_DECLARE_SERIALIZABLE_NON_VIRTUAL(MOSS_EXPORT, AnimatedJoint)

	public:
		String							mJointName;											///< Name of the joint
		KeyframeVector					mKeyframes;											///< List of keyframes over time
	};

	using AnimatedJointVector = TArray<AnimatedJoint>;

	/// Get the length (in seconds) of this animation
	float								GetDuration() const;

	/// Scale the size of all joints by inScale
	void								ScaleJoints(float inScale);

	/// If the animation is looping or not. If an animation is looping, the animation will continue playing after completion
	void 								SetIsLooping(bool inIsLooping)						{ mIsLooping = inIsLooping; }
	bool								IsLooping() const									{ return mIsLooping; }

	/// Get the (interpolated) joint transforms at time inTime
	void								Sample(float inTime, SkeletonPose &ioPose) const;

	/// Get joint samples
	const AnimatedJointVector &			GetAnimatedJoints() const							{ return mAnimatedJoints; }
	AnimatedJointVector &				GetAnimatedJoints()									{ return mAnimatedJoints; }

	/// Saves the state of this animation in binary form to inStream.
	void								SaveBinaryState(StreamOut &inStream) const;

	using AnimationResult = Result<Ref<SkeletalAnimation>>;

	/// Restore a saved ragdoll from inStream
	static AnimationResult				sRestoreFromBinaryState(StreamIn &inStream);

private:
	AnimatedJointVector					mAnimatedJoints;									///< List of joints and keyframes
	bool								mIsLooping = true;									///< If this animation loops back to start
};

MOSS_NAMESPACE_END

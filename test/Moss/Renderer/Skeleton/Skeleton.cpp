// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT


#include <Moss/Skeleton/Skeleton.h>
#include <Moss/ObjectStream/TypeDeclarations.h>
#include <Moss/Core/StreamIn.h>
#include <Moss/Core/StreamOut.h>

MOSS_NAMESPACE_BEGIN

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(Skeleton::Joint)
{
	MOSS_ADD_ATTRIBUTE(Joint, mName)
	MOSS_ADD_ATTRIBUTE(Joint, mParentName)
}

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(Skeleton)
{
	MOSS_ADD_ATTRIBUTE(Skeleton, mJoints)
}



MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SkeletalAnimation::JointState)
{
	MOSS_ADD_ATTRIBUTE(JointState, mRotation)
	MOSS_ADD_ATTRIBUTE(JointState, mTranslation)
}

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SkeletalAnimation::Keyframe)
{
	MOSS_ADD_BASE_CLASS(Keyframe, JointState)

	MOSS_ADD_ATTRIBUTE(Keyframe, mTime)
}

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SkeletalAnimation::AnimatedJoint)
{
	MOSS_ADD_ATTRIBUTE(AnimatedJoint, mJointName)
	MOSS_ADD_ATTRIBUTE(AnimatedJoint, mKeyframes)
}

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SkeletalAnimation)
{
	MOSS_ADD_ATTRIBUTE(SkeletalAnimation, mAnimatedJoints)
	MOSS_ADD_ATTRIBUTE(SkeletalAnimation, mIsLooping)
}



int Skeleton::GetJointIndex(const string_view &inName) const
{
	for (int i = 0; i < (int)mJoints.size(); ++i)
		if (mJoints[i].mName == inName)
			return i;

	return -1;
}

void Skeleton::CalculateParentJointIndices()
{
	for (Joint &j : mJoints)
		j.mParentJointIndex = GetJointIndex(j.mParentName);
}

bool Skeleton::AreJointsCorrectlyOrdered() const
{
	for (int i = 0; i < (int)mJoints.size(); ++i)
		if (mJoints[i].mParentJointIndex >= i)
			return false;

	return true;
}

void Skeleton::SaveBinaryState(StreamOut &inStream) const
{
	inStream.Write((uint32)mJoints.size());
	for (const Joint &j : mJoints)
	{
		inStream.Write(j.mName);
		inStream.Write(j.mParentJointIndex);
		inStream.Write(j.mParentName);
	}
}

Skeleton::SkeletonResult Skeleton::sRestoreFromBinaryState(StreamIn &inStream)
{
	Ref<Skeleton> skeleton = new Skeleton;

	uint32 len = 0;
	inStream.Read(len);
	skeleton->mJoints.resize(len);
	for (Joint &j : skeleton->mJoints)
	{
		inStream.Read(j.mName);
		inStream.Read(j.mParentJointIndex);
		inStream.Read(j.mParentName);
	}

	SkeletonResult result;
	if (inStream.IsEOF() || inStream.IsFailed())
		result.SetError("Failed to read skeleton from stream");
	else
		result.Set(skeleton);
	return result;
}




void SkeletonPose::SetSkeleton(const Skeleton *inSkeleton)
{
	mSkeleton = inSkeleton;

	mJoints.resize(mSkeleton->GetJointCount());
	mJointMatrices.resize(mSkeleton->GetJointCount());
}

void SkeletonPose::CalculateJointMatrices()
{
	for (int i = 0; i < (int)mJoints.size(); ++i)
	{
		mJointMatrices[i] = mJoints[i].ToMatrix();

		int parent = mSkeleton->GetJoint(i).mParentJointIndex;
		if (parent >= 0)
		{
			MOSS_ASSERT(parent < i, "Joints must be ordered: parents first");
			mJointMatrices[i] = mJointMatrices[parent] * mJointMatrices[i];
		}
	}
}

void SkeletonPose::CalculateJointStates()
{
	for (int i = 0; i < (int)mJoints.size(); ++i)
	{
		Mat44 local_transform;
		int parent = mSkeleton->GetJoint(i).mParentJointIndex;
		if (parent >= 0)
			local_transform = mJointMatrices[parent].Inversed() * mJointMatrices[i];
		else
			local_transform = mJointMatrices[i];

		JointState &joint = mJoints[i];
		joint.mTranslation = local_transform.GetTranslation();
		joint.mRotation = local_transform.GetQuaternion();
	}
}

void SkeletonPose::CalculateLocalSpaceJointMatrices(Mat44 *outMatrices) const
{
	for (int i = 0; i < (int)mJoints.size(); ++i)
		outMatrices[i] = mJoints[i].ToMatrix();
}

#ifndef MOSS_DEBUG_RENDERER
void SkeletonPose::Draw(const DrawSettings &inDrawSettings, DebugRenderer *inRenderer, RMat44Arg inOffset) const
{
	RMat44 offset = inOffset * RMat44::sTranslation(mRootOffset);

	const Skeleton::JointVector &joints = mSkeleton->GetJoints();

	for (int b = 0; b < mSkeleton->GetJointCount(); ++b)
	{
		RMat44 joint_transform = offset * mJointMatrices[b];

		if (inDrawSettings.mDrawJoints)
		{
			int parent = joints[b].mParentJointIndex;
			if (parent >= 0)
				inRenderer->DrawLine(offset * mJointMatrices[parent].GetTranslation(), joint_transform.GetTranslation(), Color::sGreen);
		}

		if (inDrawSettings.mDrawJointOrientations)
			inRenderer->DrawCoordinateSystem(joint_transform, 0.05f);

		if (inDrawSettings.mDrawJointNames)
			inRenderer->DrawText3D(joint_transform.GetTranslation(), joints[b].mName, Color::sWhite, 0.05f);
	}
}
#endif // MOSS_DEBUG_RENDERER



void SkeletonMapper::Initialize(const Skeleton *inSkeleton1, const Mat44 *inNeutralPose1, const Skeleton *inSkeleton2, const Mat44 *inNeutralPose2, const CanMapJoint &inCanMapJoint)
{
	MOSS_ASSERT(mMappings.empty() && mChains.empty() && mUnmapped.empty()); // Should not be initialized yet

	// Count joints
	int n1 = inSkeleton1->GetJointCount();
	int n2 = inSkeleton2->GetJointCount();
	MOSS_ASSERT(n1 <= n2, "Skeleton 1 should be the low detail skeleton!");

	// Keep track of mapped joints (initialize to false)
	TArray<bool> mapped1(n1, false);
	TArray<bool> mapped2(n2, false);

	// Find joints that can be mapped directly
	for (int j1 = 0; j1 < n1; ++j1)
		for (int j2 = 0; j2 < n2; ++j2)
			if (inCanMapJoint(inSkeleton1, j1, inSkeleton2, j2))
			{
				// Calculate the transform that takes this joint from skeleton 1 to 2
				Mat44 joint_1_to_2 = inNeutralPose1[j1].Inversed() * inNeutralPose2[j2];

				// Ensure bottom right element is 1 (numerical imprecision in the inverse can make this not so)
				joint_1_to_2(3, 3) = 1.0f;

				mMappings.emplace_back(j1, j2, joint_1_to_2);
				mapped1[j1] = true;
				mapped2[j2] = true;
				break;
			}

	TArray<int> cur_chain; // Taken out of the loop to minimize amount of allocations

	// Find joint chains
	for (int m1 = 0; m1 < (int)mMappings.size(); ++m1)
	{
		TArray<int> chain2;
		int chain2_m = -1;

		for (int m2 = m1 + 1; m2 < (int)mMappings.size(); ++m2)
		{
			// Find the chain from back from m2 to m1
			int start = mMappings[m1].mJointIdx2;
			int end = mMappings[m2].mJointIdx2;
			int cur = end;
			cur_chain.clear(); // Should preserve memory
			do
			{
				cur_chain.push_back(cur);
				cur = inSkeleton2->GetJoint(cur).mParentJointIndex;
			}
			while (cur >= 0 && cur != start && !mapped2[cur]);
			cur_chain.push_back(start);

			if (cur == start // This should be the correct chain
				&& cur_chain.size() > 2 // It should have joints between the mapped joints
				&& cur_chain.size() > chain2.size()) // And it should be the longest so far
			{
				chain2.swap(cur_chain);
				chain2_m = m2;
			}
		}

		if (!chain2.empty())
		{
			// Get the chain for 1
			TArray<int> chain1;
			int start = mMappings[m1].mJointIdx1;
			int cur = mMappings[chain2_m].mJointIdx1;
			do
			{
				chain1.push_back(cur);
				cur = inSkeleton1->GetJoint(cur).mParentJointIndex;
			}
			while (cur >= 0 && cur != start && !mapped1[cur]);
			chain1.push_back(start);

			// If the chain exists in 1 too
			if (cur == start)
			{
				// Reverse the chains
				std::reverse(chain1.begin(), chain1.end());
				std::reverse(chain2.begin(), chain2.end());

				// Mark elements mapped
				for (int j1 : chain1)
					mapped1[j1] = true;
				for (int j2 : chain2)
					mapped2[j2] = true;

				// Insert the chain
				mChains.emplace_back(std::move(chain1), std::move(chain2));
			}
		}
	}

	// Collect unmapped joints from 2
	for (int j2 = 0; j2 < n2; ++j2)
		if (!mapped2[j2])
			mUnmapped.emplace_back(j2, inSkeleton2->GetJoint(j2).mParentJointIndex);
}

void SkeletonMapper::LockTranslations(const Skeleton *inSkeleton2, const bool *inLockedTranslations, const Mat44 *inNeutralPose2)
{
	MOSS_ASSERT(inSkeleton2->AreJointsCorrectlyOrdered());

	int n = inSkeleton2->GetJointCount();

	// Copy locked joints to array but don't actually include the first joint (this is physics driven)
	for (int i = 0; i < n; ++i)
		if (inLockedTranslations[i])
		{
			Locked l;
			l.mJointIdx = i;
			l.mParentJointIdx = inSkeleton2->GetJoint(i).mParentJointIndex;
			if (l.mParentJointIdx >= 0)
				l.mTranslation = inNeutralPose2[l.mParentJointIdx].Inversed() * inNeutralPose2[i].GetTranslation();
			else
				l.mTranslation = inNeutralPose2[i].GetTranslation();
			mLockedTranslations.push_back(l);
		}
}

void SkeletonMapper::LockAllTranslations(const Skeleton *inSkeleton2, const Mat44 *inNeutralPose2)
{
	MOSS_ASSERT(!mMappings.empty(), "Call Initialize first!");
	MOSS_ASSERT(inSkeleton2->AreJointsCorrectlyOrdered());

	// The first mapping is the top most one (remember that joints should be ordered so that parents go before children).
	// Because we created the mappings from the lowest joint first, this should contain the first mappable joint.
	int root_idx = mMappings[0].mJointIdx2;

	// Create temp array to hold locked joints
	int n = inSkeleton2->GetJointCount();
	bool *locked_translations = (bool *)MOSS_STACK_ALLOC(n * sizeof(bool));
	memset(locked_translations, 0, n * sizeof(bool));

	// Mark root as locked
	locked_translations[root_idx] = true;

	// Loop over all joints and propagate the locked flag to all children
	for (int i = root_idx + 1; i < n; ++i)
	{
		int parent_idx = inSkeleton2->GetJoint(i).mParentJointIndex;
		if (parent_idx >= 0)
			locked_translations[i] = locked_translations[parent_idx];
	}

	// Unmark root because we don't actually want to include this (this determines the position of the entire ragdoll)
	locked_translations[root_idx] = false;

	// Call the generic function
	LockTranslations(inSkeleton2, locked_translations, inNeutralPose2);
}

void SkeletonMapper::Map(const Mat44 *inPose1ModelSpace, const Mat44 *inPose2LocalSpace, Mat44 *outPose2ModelSpace) const
{
	// Apply direct mappings
	for (const Mapping &m : mMappings)
		outPose2ModelSpace[m.mJointIdx2] = inPose1ModelSpace[m.mJointIdx1] * m.mJoint1To2;

	// Apply chain mappings
	for (const Chain &c : mChains)
	{
		// Calculate end of chain given local space transforms of the joints of the chain
		Mat44 &chain_start = outPose2ModelSpace[c.mJointIndices2.front()];
		Mat44 chain_end = chain_start;
		for (int j = 1; j < (int)c.mJointIndices2.size(); ++j)
			chain_end = chain_end * inPose2LocalSpace[c.mJointIndices2[j]];

		// Calculate the direction in world space for skeleton 1 and skeleton 2 and the rotation between them
		Vec3 actual = chain_end.GetTranslation() - chain_start.GetTranslation();
		Vec3 desired = inPose1ModelSpace[c.mJointIndices1.back()].GetTranslation() - inPose1ModelSpace[c.mJointIndices1.front()].GetTranslation();
		Quat rotation = Quat::sFromTo(actual, desired);

		// Rotate the start of the chain
		chain_start.SetRotation(Mat44::sRotation(rotation) * chain_start.GetRotation());

		// Update all joints but the first and the last joint using their local space transforms
		for (int j = 1; j < (int)c.mJointIndices2.size() - 1; ++j)
		{
			int parent = c.mJointIndices2[j - 1];
			int child = c.mJointIndices2[j];
			outPose2ModelSpace[child] = outPose2ModelSpace[parent] * inPose2LocalSpace[child];
		}
	}

	// All unmapped joints take the local pose and convert it to model space
	for (const Unmapped &u : mUnmapped)
		if (u.mParentJointIdx >= 0)
		{
			MOSS_ASSERT(u.mParentJointIdx < u.mJointIdx, "Joints must be ordered: parents first");
			outPose2ModelSpace[u.mJointIdx] = outPose2ModelSpace[u.mParentJointIdx] * inPose2LocalSpace[u.mJointIdx];
		}
		else
			outPose2ModelSpace[u.mJointIdx] = inPose2LocalSpace[u.mJointIdx];

	// Update all locked joint translations
	for (const Locked &l : mLockedTranslations)
		outPose2ModelSpace[l.mJointIdx].SetTranslation(outPose2ModelSpace[l.mParentJointIdx] * l.mTranslation);
}

void SkeletonMapper::MapReverse(const Mat44 *inPose2ModelSpace, Mat44 *outPose1ModelSpace) const
{
	// Normally each joint in skeleton 1 should be present in the mapping, so we only need to apply the direct mappings
	for (const Mapping &m : mMappings)
		outPose1ModelSpace[m.mJointIdx1] = inPose2ModelSpace[m.mJointIdx2] * m.mJoint2To1;
}

int SkeletonMapper::GetMappedJointIdx(int inJoint1Idx) const
{
	for (const Mapping &m : mMappings)
		if (m.mJointIdx1 == inJoint1Idx)
			return m.mJointIdx2;

	return -1;
}

bool SkeletonMapper::IsJointTranslationLocked(int inJoint2Idx) const
{
	for (const Locked &l : mLockedTranslations)
		if (l.mJointIdx == inJoint2Idx)
			return true;

	return false;
}


void SkeletalAnimation::JointState::FromMatrix(Mat44Arg inMatrix)
{
	mRotation = inMatrix.GetQuaternion();
	mTranslation = inMatrix.GetTranslation();
}

float SkeletalAnimation::GetDuration() const
{
	if (!mAnimatedJoints.empty() && !mAnimatedJoints[0].mKeyframes.empty())
		return mAnimatedJoints[0].mKeyframes.back().mTime;
	else
		return 0.0f;
}

void SkeletalAnimation::ScaleJoints(float inScale)
{
	for (SkeletalAnimation::AnimatedJoint &j : mAnimatedJoints)
		for (SkeletalAnimation::Keyframe &k : j.mKeyframes)
			k.mTranslation *= inScale;
}

void SkeletalAnimation::Sample(float inTime, SkeletonPose &ioPose) const
{
	// Correct time when animation is looping
	MOSS_ASSERT(inTime >= 0.0f);
	float duration = GetDuration();
	float time = duration > 0.0f && mIsLooping? fmod(inTime, duration) : inTime;

	for (const AnimatedJoint &aj : mAnimatedJoints)
	{
		// Do binary search for keyframe
		int high = (int)aj.mKeyframes.size(), low = -1;
		while (high - low > 1)
		{
			int probe = (high + low) / 2;
			if (aj.mKeyframes[probe].mTime < time)
				low = probe;
			else
				high = probe;
		}

		JointState &state = ioPose.GetJoint(ioPose.GetSkeleton()->GetJointIndex(aj.mJointName));

		if (low == -1)
		{
			// Before first key, return first key
			state = static_cast<const JointState &>(aj.mKeyframes.front());
		}
		else if (high == (int)aj.mKeyframes.size())
		{
			// Beyond last key, return last key
			state = static_cast<const JointState &>(aj.mKeyframes.back());
		}
		else
		{
			// Interpolate
			const Keyframe &s1 = aj.mKeyframes[low];
			const Keyframe &s2 = aj.mKeyframes[low + 1];

			float fraction = (time - s1.mTime) / (s2.mTime - s1.mTime);
			MOSS_ASSERT(fraction >= 0.0f && fraction <= 1.0f);

			state.mTranslation = (1.0f - fraction) * s1.mTranslation + fraction * s2.mTranslation;
			MOSS_ASSERT(s1.mRotation.IsNormalized());
			MOSS_ASSERT(s2.mRotation.IsNormalized());
			state.mRotation = s1.mRotation.SLERP(s2.mRotation, fraction);
			MOSS_ASSERT(state.mRotation.IsNormalized());
		}
	}
}

void SkeletalAnimation::SaveBinaryState(StreamOut &inStream) const
{
	inStream.Write((uint32)mAnimatedJoints.size());
	for (const AnimatedJoint &j : mAnimatedJoints)
	{
		// Write Joint name and number of keyframes
		inStream.Write(j.mJointName);
		inStream.Write((uint32)j.mKeyframes.size());
		for (const Keyframe &k : j.mKeyframes)
		{
			inStream.Write(k.mTime);
			inStream.Write(k.mRotation);
			inStream.Write(k.mTranslation);
		}
	}

	// Save additional parameters
	inStream.Write(mIsLooping);
}

SkeletalAnimation::AnimationResult SkeletalAnimation::sRestoreFromBinaryState(StreamIn &inStream)
{
	AnimationResult result;

	Ref<SkeletalAnimation> animation = new SkeletalAnimation;

	// Restore animated joints
	uint32 len = 0;
	inStream.Read(len);
	animation->mAnimatedJoints.resize(len);
	for (AnimatedJoint &j : animation->mAnimatedJoints)
	{
		// Read joint name
		inStream.Read(j.mJointName);

		// Read keyframes
		len = 0;
		inStream.Read(len);
		j.mKeyframes.resize(len);
		for (Keyframe &k : j.mKeyframes)
		{
			inStream.Read(k.mTime);
			inStream.Read(k.mRotation);
			inStream.Read(k.mTranslation);
		}
	}

	// Read additional parameters
	inStream.Read(animation->mIsLooping);
	result.Set(animation);
	return result;
}
MOSS_NAMESPACE_END

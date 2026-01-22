
#include <Moss/Physics/physics_intern.h>
#ifndef MOSS_DEBUG_RENDERER
	#include <Moss/Moss_Renderer.h>
#endif // MOSS_DEBUG_RENDERER


MOSS_SUPRESS_WARNINGS_BEGIN

#ifndef MOSS_DEBUG_RENDERER
bool ContactConstraintManager::sDrawContactPoint = false;
bool ContactConstraintManager::sDrawSupportingFaces = false;
bool ContactConstraintManager::sDrawContactPointReduction = false;
bool ContactConstraintManager::sDrawContactManifolds = false;
#endif // MOSS_DEBUG_RENDERER


MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(ConeConstraintSettings) {
	MOSS_ADD_BASE_CLASS(ConeConstraintSettings, TwoBodyConstraintSettings)

	MOSS_ADD_ENUM_ATTRIBUTE(ConeConstraintSettings, mSpace)
	MOSS_ADD_ATTRIBUTE(ConeConstraintSettings, mPoint1)
	MOSS_ADD_ATTRIBUTE(ConeConstraintSettings, mTwistAxis1)
	MOSS_ADD_ATTRIBUTE(ConeConstraintSettings, mPoint2)
	MOSS_ADD_ATTRIBUTE(ConeConstraintSettings, mTwistAxis2)
	MOSS_ADD_ATTRIBUTE(ConeConstraintSettings, mHalfConeAngle)
}


MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(RackAndPinionConstraintSettings) {
	MOSS_ADD_BASE_CLASS(RackAndPinionConstraintSettings, TwoBodyConstraintSettings)

	MOSS_ADD_ENUM_ATTRIBUTE(RackAndPinionConstraintSettings, mSpace)
	MOSS_ADD_ATTRIBUTE(RackAndPinionConstraintSettings, mHingeAxis)
	MOSS_ADD_ATTRIBUTE(RackAndPinionConstraintSettings, mSliderAxis)
	MOSS_ADD_ATTRIBUTE(RackAndPinionConstraintSettings, mRatio)
}

MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(ConstraintSettings) {
	MOSS_ADD_BASE_CLASS(ConstraintSettings, SerializableObject)

	MOSS_ADD_ATTRIBUTE(ConstraintSettings, mEnabled)
	MOSS_ADD_ATTRIBUTE(ConstraintSettings, mDrawConstraintSize)
	MOSS_ADD_ATTRIBUTE(ConstraintSettings, mConstraintPriority)
	MOSS_ADD_ATTRIBUTE(ConstraintSettings, mNumVelocityStepsOverride)
	MOSS_ADD_ATTRIBUTE(ConstraintSettings, mNumPositionStepsOverride)
	MOSS_ADD_ATTRIBUTE(ConstraintSettings, mUserData)
}

MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(FixedConstraintSettings) {
	MOSS_ADD_BASE_CLASS(FixedConstraintSettings, TwoBodyConstraintSettings)

	MOSS_ADD_ENUM_ATTRIBUTE(FixedConstraintSettings, mSpace)
	MOSS_ADD_ATTRIBUTE(FixedConstraintSettings, mAutoDetectPoint)
	MOSS_ADD_ATTRIBUTE(FixedConstraintSettings, mPoint1)
	MOSS_ADD_ATTRIBUTE(FixedConstraintSettings, mAxisX1)
	MOSS_ADD_ATTRIBUTE(FixedConstraintSettings, mAxisY1)
	MOSS_ADD_ATTRIBUTE(FixedConstraintSettings, mPoint2)
	MOSS_ADD_ATTRIBUTE(FixedConstraintSettings, mAxisX2)
	MOSS_ADD_ATTRIBUTE(FixedConstraintSettings, mAxisY2)
}

MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(GearConstraintSettings) {
	MOSS_ADD_BASE_CLASS(GearConstraintSettings, TwoBodyConstraintSettings)

	MOSS_ADD_ENUM_ATTRIBUTE(GearConstraintSettings, mSpace)
	MOSS_ADD_ATTRIBUTE(GearConstraintSettings, mHingeAxis1)
	MOSS_ADD_ATTRIBUTE(GearConstraintSettings, mHingeAxis2)
	MOSS_ADD_ATTRIBUTE(GearConstraintSettings, mRatio)
}

MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(HingeConstraintSettings) {
	MOSS_ADD_BASE_CLASS(HingeConstraintSettings, TwoBodyConstraintSettings)

	MOSS_ADD_ENUM_ATTRIBUTE(HingeConstraintSettings, mSpace)
	MOSS_ADD_ATTRIBUTE(HingeConstraintSettings, mPoint1)
	MOSS_ADD_ATTRIBUTE(HingeConstraintSettings, mHingeAxis1)
	MOSS_ADD_ATTRIBUTE(HingeConstraintSettings, mNormalAxis1)
	MOSS_ADD_ATTRIBUTE(HingeConstraintSettings, mPoint2)
	MOSS_ADD_ATTRIBUTE(HingeConstraintSettings, mHingeAxis2)
	MOSS_ADD_ATTRIBUTE(HingeConstraintSettings, mNormalAxis2)
	MOSS_ADD_ATTRIBUTE(HingeConstraintSettings, mLimitsMin)
	MOSS_ADD_ATTRIBUTE(HingeConstraintSettings, mLimitsMax)
	MOSS_ADD_ATTRIBUTE(HingeConstraintSettings, mLimitsSpringSettings)
	MOSS_ADD_ATTRIBUTE(HingeConstraintSettings, mMaxFrictionTorque)
	MOSS_ADD_ATTRIBUTE(HingeConstraintSettings, mMotorSettings)
}

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(MotorSettings) {
	MOSS_ADD_ENUM_ATTRIBUTE_WITH_ALIAS(MotorSettings, mSpringSettings.mMode, "mSpringMode")
	MOSS_ADD_ATTRIBUTE_WITH_ALIAS(MotorSettings, mSpringSettings.mFrequency, "mFrequency") // Renaming attributes to stay compatible with old versions of the library
	MOSS_ADD_ATTRIBUTE_WITH_ALIAS(MotorSettings, mSpringSettings.mDamping, "mDamping")
	MOSS_ADD_ATTRIBUTE(MotorSettings, mMinForceLimit)
	MOSS_ADD_ATTRIBUTE(MotorSettings, mMaxForceLimit)
	MOSS_ADD_ATTRIBUTE(MotorSettings, mMinTorqueLimit)
	MOSS_ADD_ATTRIBUTE(MotorSettings, mMaxTorqueLimit)
}

MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(DistanceConstraintSettings) {
	MOSS_ADD_BASE_CLASS(DistanceConstraintSettings, TwoBodyConstraintSettings)

	MOSS_ADD_ENUM_ATTRIBUTE(DistanceConstraintSettings, mSpace)
	MOSS_ADD_ATTRIBUTE(DistanceConstraintSettings, mPoint1)
	MOSS_ADD_ATTRIBUTE(DistanceConstraintSettings, mPoint2)
	MOSS_ADD_ATTRIBUTE(DistanceConstraintSettings, mMinDistance)
	MOSS_ADD_ATTRIBUTE(DistanceConstraintSettings, mMaxDistance)
	MOSS_ADD_ENUM_ATTRIBUTE_WITH_ALIAS(DistanceConstraintSettings, mLimitsSpringSettings.mMode, "mSpringMode")
	MOSS_ADD_ATTRIBUTE_WITH_ALIAS(DistanceConstraintSettings, mLimitsSpringSettings.mFrequency, "mFrequency") // Renaming attributes to stay compatible with old versions of the library
	MOSS_ADD_ATTRIBUTE_WITH_ALIAS(DistanceConstraintSettings, mLimitsSpringSettings.mDamping, "mDamping")
}

MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(PathConstraintSettings) {
	MOSS_ADD_BASE_CLASS(PathConstraintSettings, TwoBodyConstraintSettings)

	MOSS_ADD_ATTRIBUTE(PathConstraintSettings, mPath)
	MOSS_ADD_ATTRIBUTE(PathConstraintSettings, mPathPosition)
	MOSS_ADD_ATTRIBUTE(PathConstraintSettings, mPathRotation)
	MOSS_ADD_ATTRIBUTE(PathConstraintSettings, mPathFraction)
	MOSS_ADD_ATTRIBUTE(PathConstraintSettings, mMaxFrictionForce)
	MOSS_ADD_ATTRIBUTE(PathConstraintSettings, mPositionMotorSettings)
	MOSS_ADD_ENUM_ATTRIBUTE(PathConstraintSettings, mRotationConstraintType)
}

MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(PulleyConstraintSettings) {
	MOSS_ADD_BASE_CLASS(PulleyConstraintSettings, TwoBodyConstraintSettings)

	MOSS_ADD_ENUM_ATTRIBUTE(PulleyConstraintSettings, mSpace)
	MOSS_ADD_ATTRIBUTE(PulleyConstraintSettings, mBodyPoint1)
	MOSS_ADD_ATTRIBUTE(PulleyConstraintSettings, mFixedPoint1)
	MOSS_ADD_ATTRIBUTE(PulleyConstraintSettings, mBodyPoint2)
	MOSS_ADD_ATTRIBUTE(PulleyConstraintSettings, mFixedPoint2)
	MOSS_ADD_ATTRIBUTE(PulleyConstraintSettings, mRatio)
	MOSS_ADD_ATTRIBUTE(PulleyConstraintSettings, mMinLength)
	MOSS_ADD_ATTRIBUTE(PulleyConstraintSettings, mMaxLength)
}

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(PathConstraintPathHermite::Point) {
	MOSS_ADD_ATTRIBUTE(PathConstraintPathHermite::Point, mPosition)
	MOSS_ADD_ATTRIBUTE(PathConstraintPathHermite::Point, mTangent)
	MOSS_ADD_ATTRIBUTE(PathConstraintPathHermite::Point, mNormal)
}

MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(PathConstraintPathHermite) {
	MOSS_ADD_BASE_CLASS(PathConstraintPathHermite, PathConstraintPath)

	MOSS_ADD_ATTRIBUTE(PathConstraintPathHermite, mPoints)
}

MOSS_IMPLEMENT_SERIALIZABLE_ABSTRACT(PathConstraintPath) {
	MOSS_ADD_BASE_CLASS(PathConstraintPath, SerializableObject)
}

MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(PointConstraintSettings) {
	MOSS_ADD_BASE_CLASS(PointConstraintSettings, TwoBodyConstraintSettings)

	MOSS_ADD_ENUM_ATTRIBUTE(PointConstraintSettings, mSpace)
	MOSS_ADD_ATTRIBUTE(PointConstraintSettings, mPoint1)
	MOSS_ADD_ATTRIBUTE(PointConstraintSettings, mPoint2)
}

MOSS_IMPLEMENT_SERIALIZABLE_ABSTRACT(TwoBodyConstraintSettings) {
	MOSS_ADD_BASE_CLASS(TwoBodyConstraintSettings, ConstraintSettings)
}

MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(SliderConstraintSettings) {
	MOSS_ADD_BASE_CLASS(SliderConstraintSettings, TwoBodyConstraintSettings)

	MOSS_ADD_ENUM_ATTRIBUTE(SliderConstraintSettings, mSpace)
	MOSS_ADD_ATTRIBUTE(SliderConstraintSettings, mAutoDetectPoint)
	MOSS_ADD_ATTRIBUTE(SliderConstraintSettings, mPoint1)
	MOSS_ADD_ATTRIBUTE(SliderConstraintSettings, mSliderAxis1)
	MOSS_ADD_ATTRIBUTE(SliderConstraintSettings, mNormalAxis1)
	MOSS_ADD_ATTRIBUTE(SliderConstraintSettings, mPoint2)
	MOSS_ADD_ATTRIBUTE(SliderConstraintSettings, mSliderAxis2)
	MOSS_ADD_ATTRIBUTE(SliderConstraintSettings, mNormalAxis2)
	MOSS_ADD_ATTRIBUTE(SliderConstraintSettings, mLimitsMin)
	MOSS_ADD_ATTRIBUTE(SliderConstraintSettings, mLimitsMax)
	MOSS_ADD_ENUM_ATTRIBUTE_WITH_ALIAS(SliderConstraintSettings, mLimitsSpringSettings.mMode, "mSpringMode")
	MOSS_ADD_ATTRIBUTE_WITH_ALIAS(SliderConstraintSettings, mLimitsSpringSettings.mFrequency, "mFrequency") // Renaming attributes to stay compatible with old versions of the library
	MOSS_ADD_ATTRIBUTE_WITH_ALIAS(SliderConstraintSettings, mLimitsSpringSettings.mDamping, "mDamping")
	MOSS_ADD_ATTRIBUTE(SliderConstraintSettings, mMaxFrictionForce)
	MOSS_ADD_ATTRIBUTE(SliderConstraintSettings, mMotorSettings)
}

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(SpringSettings) {
	MOSS_ADD_ENUM_ATTRIBUTE(SpringSettings, mMode)
	MOSS_ADD_ATTRIBUTE(SpringSettings, mFrequency)
	MOSS_ADD_ATTRIBUTE(SpringSettings, mDamping)
}


MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(SwingTwistConstraintSettings) {
	MOSS_ADD_BASE_CLASS(SwingTwistConstraintSettings, TwoBodyConstraintSettings)

	MOSS_ADD_ENUM_ATTRIBUTE(SwingTwistConstraintSettings, mSpace)
	MOSS_ADD_ATTRIBUTE(SwingTwistConstraintSettings, mPosition1)
	MOSS_ADD_ATTRIBUTE(SwingTwistConstraintSettings, mTwistAxis1)
	MOSS_ADD_ATTRIBUTE(SwingTwistConstraintSettings, mPlaneAxis1)
	MOSS_ADD_ATTRIBUTE(SwingTwistConstraintSettings, mPosition2)
	MOSS_ADD_ATTRIBUTE(SwingTwistConstraintSettings, mTwistAxis2)
	MOSS_ADD_ATTRIBUTE(SwingTwistConstraintSettings, mPlaneAxis2)
	MOSS_ADD_ENUM_ATTRIBUTE(SwingTwistConstraintSettings, mSwingType)
	MOSS_ADD_ATTRIBUTE(SwingTwistConstraintSettings, mNormalHalfConeAngle)
	MOSS_ADD_ATTRIBUTE(SwingTwistConstraintSettings, mPlaneHalfConeAngle)
	MOSS_ADD_ATTRIBUTE(SwingTwistConstraintSettings, mTwistMinAngle)
	MOSS_ADD_ATTRIBUTE(SwingTwistConstraintSettings, mTwistMaxAngle)
	MOSS_ADD_ATTRIBUTE(SwingTwistConstraintSettings, mMaxFrictionTorque)
	MOSS_ADD_ATTRIBUTE(SwingTwistConstraintSettings, mSwingMotorSettings)
	MOSS_ADD_ATTRIBUTE(SwingTwistConstraintSettings, mTwistMotorSettings)
}

MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(SixDOFConstraintSettings) {
	MOSS_ADD_BASE_CLASS(SixDOFConstraintSettings, TwoBodyConstraintSettings)

	MOSS_ADD_ENUM_ATTRIBUTE(SixDOFConstraintSettings, mSpace)
	MOSS_ADD_ATTRIBUTE(SixDOFConstraintSettings, mPosition1)
	MOSS_ADD_ATTRIBUTE(SixDOFConstraintSettings, mAxisX1)
	MOSS_ADD_ATTRIBUTE(SixDOFConstraintSettings, mAxisY1)
	MOSS_ADD_ATTRIBUTE(SixDOFConstraintSettings, mPosition2)
	MOSS_ADD_ATTRIBUTE(SixDOFConstraintSettings, mAxisX2)
	MOSS_ADD_ATTRIBUTE(SixDOFConstraintSettings, mAxisY2)
	MOSS_ADD_ATTRIBUTE(SixDOFConstraintSettings, mMaxFriction)
	MOSS_ADD_ENUM_ATTRIBUTE(SixDOFConstraintSettings, mSwingType)
	MOSS_ADD_ATTRIBUTE(SixDOFConstraintSettings, mLimitMin)
	MOSS_ADD_ATTRIBUTE(SixDOFConstraintSettings, mLimitMax)
	MOSS_ADD_ATTRIBUTE(SixDOFConstraintSettings, mLimitsSpringSettings)
	MOSS_ADD_ATTRIBUTE(SixDOFConstraintSettings, mMotorSettings)
}



TwoBodyConstraint *ConeConstraintSettings::Create(Body &inBody1, Body &inBody2) const { return new ConeConstraint(inBody1, inBody2, *this); }

ConeConstraint::ConeConstraint(Body &inBody1, Body &inBody2, const ConeConstraintSettings &inSettings) :
	TwoBodyConstraint(inBody1, inBody2, inSettings)
{
	// Store limits
	SetHalfConeAngle(inSettings.mHalfConeAngle);

	// Initialize rotation axis to perpendicular of twist axis in case the angle between the twist axis is 0 in the first frame
	mWorldSpaceRotationAxis = inSettings.mTwistAxis1.GetNormalizedPerpendicular();

	if (inSettings.mSpace == EConstraintSpace::WorldSpace)
	{
		// If all properties were specified in world space, take them to local space now
		RMat44 inv_transform1 = inBody1.GetInverseCenterOfMassTransform();
		mLocalSpacePosition1 = Vec3(inv_transform1 * inSettings.mPoint1);
		mLocalSpaceTwistAxis1 = inv_transform1.Multiply3x3(inSettings.mTwistAxis1);

		RMat44 inv_transform2 = inBody2.GetInverseCenterOfMassTransform();
		mLocalSpacePosition2 = Vec3(inv_transform2 * inSettings.mPoint2);
		mLocalSpaceTwistAxis2 = inv_transform2.Multiply3x3(inSettings.mTwistAxis2);
	}
	else
	{
		// Properties already in local space
		mLocalSpacePosition1 = Vec3(inSettings.mPoint1);
		mLocalSpacePosition2 = Vec3(inSettings.mPoint2);
		mLocalSpaceTwistAxis1 = inSettings.mTwistAxis1;
		mLocalSpaceTwistAxis2 = inSettings.mTwistAxis2;

		// If they were in local space, we need to take the initial rotation axis to world space
		mWorldSpaceRotationAxis = inBody1.GetRotation() * mWorldSpaceRotationAxis;
	}
}

void ConeConstraint::NotifyShapeChanged(const BodyID &inBodyID, Vec3Arg inDeltaCOM)
{
	if (mBody1->GetID() == inBodyID)
		mLocalSpacePosition1 -= inDeltaCOM;
	else if (mBody2->GetID() == inBodyID)
		mLocalSpacePosition2 -= inDeltaCOM;
}

void ConeConstraint::CalculateRotationConstraintProperties(Mat44Arg inRotation1, Mat44Arg inRotation2)
{
	// Rotation is along the cross product of both twist axis
	Vec3 twist1 = inRotation1.Multiply3x3(mLocalSpaceTwistAxis1);
	Vec3 twist2 = inRotation2.Multiply3x3(mLocalSpaceTwistAxis2);

	// Calculate dot product between twist axis, if it's smaller than the cone angle we need to correct
	mCosTheta = twist1.Dot(twist2);
	if (mCosTheta < mCosHalfConeAngle)
	{
		// Rotation axis is defined by the two twist axis
		Vec3 rot_axis = twist2.Cross(twist1);

		// If we can't find a rotation axis because the twist is too small, we'll use last frame's rotation axis
		float len = rot_axis.Length();
		if (len > 0.0f)
			mWorldSpaceRotationAxis = rot_axis / len;

		mAngleConstraintPart.CalculateConstraintProperties(*mBody1, *mBody2, mWorldSpaceRotationAxis);
	}
	else
		mAngleConstraintPart.Deactivate();
}

void ConeConstraint::SetupVelocityConstraint(float inDeltaTime)
{
	Mat44 rotation1 = Mat44::sRotation(mBody1->GetRotation());
	Mat44 rotation2 = Mat44::sRotation(mBody2->GetRotation());
	mPointConstraintPart.CalculateConstraintProperties(*mBody1, rotation1, mLocalSpacePosition1, *mBody2, rotation2, mLocalSpacePosition2);
	CalculateRotationConstraintProperties(rotation1, rotation2);
}

void ConeConstraint::ResetWarmStart()
{
	mPointConstraintPart.Deactivate();
	mAngleConstraintPart.Deactivate();
}

void ConeConstraint::WarmStartVelocityConstraint(float inWarmStartImpulseRatio)
{
	// Warm starting: Apply previous frame impulse
	mPointConstraintPart.WarmStart(*mBody1, *mBody2, inWarmStartImpulseRatio);
	mAngleConstraintPart.WarmStart(*mBody1, *mBody2, inWarmStartImpulseRatio);
}

bool ConeConstraint::SolveVelocityConstraint(float inDeltaTime)
{
	bool pos = mPointConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2);

	bool rot = false;
	if (mAngleConstraintPart.IsActive())
		rot = mAngleConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2, mWorldSpaceRotationAxis, 0, FLT_MAX);

	return pos || rot;
}

bool ConeConstraint::SolvePositionConstraint(float inDeltaTime, float inBaumgarte)
{
	mPointConstraintPart.CalculateConstraintProperties(*mBody1, Mat44::sRotation(mBody1->GetRotation()), mLocalSpacePosition1, *mBody2, Mat44::sRotation(mBody2->GetRotation()), mLocalSpacePosition2);
	bool pos = mPointConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, inBaumgarte);

	bool rot = false;
	CalculateRotationConstraintProperties(Mat44::sRotation(mBody1->GetRotation()), Mat44::sRotation(mBody2->GetRotation()));
	if (mAngleConstraintPart.IsActive())
		rot = mAngleConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, mCosTheta - mCosHalfConeAngle, inBaumgarte);

	return pos || rot;
}

#ifndef MOSS_DEBUG_RENDERER
void ConeConstraint::DrawConstraint(DebugRenderer *inRenderer) const
{
	RMat44 transform1 = mBody1->GetCenterOfMassTransform();
	RMat44 transform2 = mBody2->GetCenterOfMassTransform();

	RVec3 p1 = transform1 * mLocalSpacePosition1;
	RVec3 p2 = transform2 * mLocalSpacePosition2;

	// Draw constraint
	inRenderer->DrawMarker(p1, Color::sRed, 0.1f);
	inRenderer->DrawMarker(p2, Color::sGreen, 0.1f);

	// Draw twist axis
	inRenderer->DrawLine(p1, p1 + mDrawConstraintSize * transform1.Multiply3x3(mLocalSpaceTwistAxis1), Color::sRed);
	inRenderer->DrawLine(p2, p2 + mDrawConstraintSize * transform2.Multiply3x3(mLocalSpaceTwistAxis2), Color::sGreen);
}

void ConeConstraint::DrawConstraintLimits(DebugRenderer *inRenderer) const
{
	// Get constraint properties in world space
	RMat44 transform1 = mBody1->GetCenterOfMassTransform();
	RVec3 position1 = transform1 * mLocalSpacePosition1;
	Vec3 twist_axis1 = transform1.Multiply3x3(mLocalSpaceTwistAxis1);
	Vec3 normal_axis1 = transform1.Multiply3x3(mLocalSpaceTwistAxis1.GetNormalizedPerpendicular());

	inRenderer->DrawOpenCone(position1, twist_axis1, normal_axis1, ACos(mCosHalfConeAngle), mDrawConstraintSize * mCosHalfConeAngle, Color::sPurple, DebugRenderer::ECastShadow::Off);
}
#endif // MOSS_DEBUG_RENDERER

void ConeConstraint::SaveState(StateRecorder &inStream) const
{
	TwoBodyConstraint::SaveState(inStream);

	mPointConstraintPart.SaveState(inStream);
	mAngleConstraintPart.SaveState(inStream);
	inStream.Write(mWorldSpaceRotationAxis); // When twist is too small, the rotation is used from last frame so we need to store it
}

void ConeConstraint::RestoreState(StateRecorder &inStream)
{
	TwoBodyConstraint::RestoreState(inStream);

	mPointConstraintPart.RestoreState(inStream);
	mAngleConstraintPart.RestoreState(inStream);
	inStream.Read(mWorldSpaceRotationAxis);
}

Ref<ConstraintSettings> ConeConstraint::GetConstraintSettings() const
{
	ConeConstraintSettings *settings = new ConeConstraintSettings;
	ToConstraintSettings(*settings);
	settings->mSpace = EConstraintSpace::LocalToBodyCOM;
	settings->mPoint1 = RVec3(mLocalSpacePosition1);
	settings->mTwistAxis1 = mLocalSpaceTwistAxis1;
	settings->mPoint2 = RVec3(mLocalSpacePosition2);
	settings->mTwistAxis2 = mLocalSpaceTwistAxis2;
	settings->mHalfConeAngle = ACos(mCosHalfConeAngle);
	return settings;
}

Mat44 ConeConstraint::GetConstraintToBody1Matrix() const
{
	Vec3 perp = mLocalSpaceTwistAxis1.GetNormalizedPerpendicular();
	Vec3 perp2 = mLocalSpaceTwistAxis1.Cross(perp);
	return Mat44(Vec4(mLocalSpaceTwistAxis1, 0), Vec4(perp, 0), Vec4(perp2, 0), Vec4(mLocalSpacePosition1, 1));
}

Mat44 ConeConstraint::GetConstraintToBody2Matrix() const
{
	// Note: Incorrect in rotation around the twist axis (the perpendicular does not match that of body 1),
	// this should not matter as we're not limiting rotation around the twist axis.
	Vec3 perp = mLocalSpaceTwistAxis2.GetNormalizedPerpendicular();
	Vec3 perp2 = mLocalSpaceTwistAxis2.Cross(perp);
	return Mat44(Vec4(mLocalSpaceTwistAxis2, 0), Vec4(perp, 0), Vec4(perp2, 0), Vec4(mLocalSpacePosition2, 1));
}



void ConstraintManager::Add(Constraint **inConstraints, int inNumber)
{
	UniqueLock lock(mConstraintsMutex MOSS_IF_ENABLE_ASSERTS(, mLockContext, EPhysicsLockTypes::ConstraintsList));

	mConstraints.reserve(mConstraints.size() + inNumber);

	for (Constraint **c = inConstraints, **c_end = inConstraints + inNumber; c < c_end; ++c)
	{
		Constraint *constraint = *c;

		// Assume this constraint has not been added yet
		MOSS_ASSERT(constraint->mConstraintIndex == Constraint::cInvalidConstraintIndex);

		// Add to the list
		constraint->mConstraintIndex = uint32(mConstraints.size());
		mConstraints.push_back(constraint);
	}
}

void ConstraintManager::Remove(Constraint **inConstraints, int inNumber)
{
	UniqueLock lock(mConstraintsMutex MOSS_IF_ENABLE_ASSERTS(, mLockContext, EPhysicsLockTypes::ConstraintsList));

	for (Constraint **c = inConstraints, **c_end = inConstraints + inNumber; c < c_end; ++c)
	{
		Constraint *constraint = *c;

		// Reset constraint index for this constraint
		uint32 this_constraint_idx = constraint->mConstraintIndex;
		constraint->mConstraintIndex = Constraint::cInvalidConstraintIndex;
		MOSS_ASSERT(this_constraint_idx != Constraint::cInvalidConstraintIndex);

		// Check if this constraint is somewhere in the middle of the constraints, in this case we need to move the last constraint to this position
		uint32 last_constraint_idx = uint32(mConstraints.size() - 1);
		if (this_constraint_idx < last_constraint_idx)
		{
			Constraint *last_constraint = mConstraints[last_constraint_idx];
			last_constraint->mConstraintIndex = this_constraint_idx;
			mConstraints[this_constraint_idx] = last_constraint;
		}

		// Pop last constraint
		mConstraints.pop_back();
	}
}

Constraints ConstraintManager::GetConstraints() const
{
	UniqueLock lock(mConstraintsMutex MOSS_IF_ENABLE_ASSERTS(, mLockContext, EPhysicsLockTypes::ConstraintsList));

	Constraints copy = mConstraints;
	return copy;
}

void ConstraintManager::GetActiveConstraints(uint32 inStartConstraintIdx, uint32 inEndConstraintIdx, Constraint **outActiveConstraints, uint32 &outNumActiveConstraints) const
{
	MOSS_PROFILE_FUNCTION();

	MOSS_ASSERT(inEndConstraintIdx <= mConstraints.size());

	uint32 num_active_constraints = 0;
	for (uint32 constraint_idx = inStartConstraintIdx; constraint_idx < inEndConstraintIdx; ++constraint_idx)
	{
		Constraint *c = mConstraints[constraint_idx];
		MOSS_ASSERT(c->mConstraintIndex == constraint_idx);
		if (c->IsActive())
		{
			*(outActiveConstraints++) = c;
			num_active_constraints++;
		}
	}

	outNumActiveConstraints = num_active_constraints;
}

void ConstraintManager::sBuildIslands(Constraint **inActiveConstraints, uint32 inNumActiveConstraints, IslandBuilder &ioBuilder, BodyManager &inBodyManager)
{
	MOSS_PROFILE_FUNCTION();

	for (uint32 constraint_idx = 0; constraint_idx < inNumActiveConstraints; ++constraint_idx)
	{
		Constraint *c = inActiveConstraints[constraint_idx];
		c->BuildIslands(constraint_idx, ioBuilder, inBodyManager);
	}
}

void ConstraintManager::sSortConstraints(Constraint **inActiveConstraints, uint32 *inConstraintIdxBegin, uint32 *inConstraintIdxEnd)
{
	MOSS_PROFILE_FUNCTION();

	QuickSort(inConstraintIdxBegin, inConstraintIdxEnd, [inActiveConstraints](uint32 inLHS, uint32 inRHS) {
		const Constraint *lhs = inActiveConstraints[inLHS];
		const Constraint *rhs = inActiveConstraints[inRHS];

		if (lhs->GetConstraintPriority() != rhs->GetConstraintPriority())
			return lhs->GetConstraintPriority() < rhs->GetConstraintPriority();

		return lhs->mConstraintIndex < rhs->mConstraintIndex;
	});
}

void ConstraintManager::sSetupVelocityConstraints(Constraint **inActiveConstraints, uint32 inNumActiveConstraints, float inDeltaTime)
{
	MOSS_PROFILE_FUNCTION();

	for (Constraint **c = inActiveConstraints, **c_end = inActiveConstraints + inNumActiveConstraints; c < c_end; ++c)
		(*c)->SetupVelocityConstraint(inDeltaTime);
}

template <class ConstraintCallback>
void ConstraintManager::sWarmStartVelocityConstraints(Constraint **inActiveConstraints, const uint32 *inConstraintIdxBegin, const uint32 *inConstraintIdxEnd, float inWarmStartImpulseRatio, ConstraintCallback &ioCallback)
{
	MOSS_PROFILE_FUNCTION();

	for (const uint32 *constraint_idx = inConstraintIdxBegin; constraint_idx < inConstraintIdxEnd; ++constraint_idx)
	{
		Constraint *c = inActiveConstraints[*constraint_idx];
		ioCallback(c);
		c->WarmStartVelocityConstraint(inWarmStartImpulseRatio);
	}
}

// Specialize for the two constraint callback types
template void ConstraintManager::sWarmStartVelocityConstraints<CalculateSolverSteps>(Constraint **inActiveConstraints, const uint32 *inConstraintIdxBegin, const uint32 *inConstraintIdxEnd, float inWarmStartImpulseRatio, CalculateSolverSteps &ioCallback);
template void ConstraintManager::sWarmStartVelocityConstraints<DummyCalculateSolverSteps>(Constraint **inActiveConstraints, const uint32 *inConstraintIdxBegin, const uint32 *inConstraintIdxEnd, float inWarmStartImpulseRatio, DummyCalculateSolverSteps &ioCallback);

bool ConstraintManager::sSolveVelocityConstraints(Constraint **inActiveConstraints, const uint32 *inConstraintIdxBegin, const uint32 *inConstraintIdxEnd, float inDeltaTime)
{
	MOSS_PROFILE_FUNCTION();

	bool any_impulse_applied = false;

	for (const uint32 *constraint_idx = inConstraintIdxBegin; constraint_idx < inConstraintIdxEnd; ++constraint_idx)
	{
		Constraint *c = inActiveConstraints[*constraint_idx];
		any_impulse_applied |= c->SolveVelocityConstraint(inDeltaTime);
	}

	return any_impulse_applied;
}

bool ConstraintManager::sSolvePositionConstraints(Constraint **inActiveConstraints, const uint32 *inConstraintIdxBegin, const uint32 *inConstraintIdxEnd, float inDeltaTime, float inBaumgarte)
{
	MOSS_PROFILE_FUNCTION();

	bool any_impulse_applied = false;

	for (const uint32 *constraint_idx = inConstraintIdxBegin; constraint_idx < inConstraintIdxEnd; ++constraint_idx)
	{
		Constraint *c = inActiveConstraints[*constraint_idx];
		any_impulse_applied |= c->SolvePositionConstraint(inDeltaTime, inBaumgarte);
	}

	return any_impulse_applied;
}

#ifndef MOSS_DEBUG_RENDERER
void ConstraintManager::DrawConstraints(DebugRenderer *inRenderer) const
{
	MOSS_PROFILE_FUNCTION();

	UniqueLock lock(mConstraintsMutex MOSS_IF_ENABLE_ASSERTS(, mLockContext, EPhysicsLockTypes::ConstraintsList));

	for (const Ref<Constraint> &c : mConstraints)
		c->DrawConstraint(inRenderer);
}

void ConstraintManager::DrawConstraintLimits(DebugRenderer *inRenderer) const
{
	MOSS_PROFILE_FUNCTION();

	UniqueLock lock(mConstraintsMutex MOSS_IF_ENABLE_ASSERTS(, mLockContext, EPhysicsLockTypes::ConstraintsList));

	for (const Ref<Constraint> &c : mConstraints)
		c->DrawConstraintLimits(inRenderer);
}

void ConstraintManager::DrawConstraintReferenceFrame(DebugRenderer *inRenderer) const
{
	MOSS_PROFILE_FUNCTION();

	UniqueLock lock(mConstraintsMutex MOSS_IF_ENABLE_ASSERTS(, mLockContext, EPhysicsLockTypes::ConstraintsList));

	for (const Ref<Constraint> &c : mConstraints)
		c->DrawConstraintReferenceFrame(inRenderer);
}
#endif // MOSS_DEBUG_RENDERER

void ConstraintManager::SaveState(StateRecorder &inStream, const StateRecorderFilter *inFilter) const
{
	UniqueLock lock(mConstraintsMutex MOSS_IF_ENABLE_ASSERTS(, mLockContext, EPhysicsLockTypes::ConstraintsList));

	// Write state of constraints
	if (inFilter != nullptr)
	{
		// Determine which constraints to save
		TArray<Constraint *> constraints;
		constraints.reserve(mConstraints.size());
		for (const Ref<Constraint> &c : mConstraints)
			if (inFilter->ShouldSaveConstraint(*c))
				constraints.push_back(c);

		// Save them
		uint32 num_constraints = (uint32)constraints.size();
		inStream.Write(num_constraints);
		for (const Constraint *c : constraints)
		{
			inStream.Write(c->mConstraintIndex);
			c->SaveState(inStream);
		}
	}
	else
	{
		// Save all constraints
		uint32 num_constraints = (uint32)mConstraints.size();
		inStream.Write(num_constraints);
		for (const Ref<Constraint> &c : mConstraints)
		{
			inStream.Write(c->mConstraintIndex);
			c->SaveState(inStream);
		}
	}
}

bool ConstraintManager::RestoreState(StateRecorder &inStream)
{
	UniqueLock lock(mConstraintsMutex MOSS_IF_ENABLE_ASSERTS(, mLockContext, EPhysicsLockTypes::ConstraintsList));

	if (inStream.IsValidating())
	{
		// Read state of constraints
		uint32 num_constraints = (uint32)mConstraints.size(); // Initialize to current value for validation
		inStream.Read(num_constraints);
		if (num_constraints != mConstraints.size())
		{
			MOSS_ASSERT(false, "Cannot handle adding/removing constraints");
			return false;
		}
		for (const Ref<Constraint> &c : mConstraints)
		{
			uint32 constraint_index = c->mConstraintIndex;
			inStream.Read(constraint_index);
			if (constraint_index != c->mConstraintIndex)
			{
				MOSS_ASSERT(false, "Unexpected constraint index");
				return false;
			}
			c->RestoreState(inStream);
		}
	}
	else
	{
		// Not validating, use more flexible reading, read number of constraints
		uint32 num_constraints = 0;
		inStream.Read(num_constraints);

		for (uint32 idx = 0; idx < num_constraints; ++idx)
		{
			uint32 constraint_index;
			inStream.Read(constraint_index);
			if (mConstraints.size() <= constraint_index)
			{
				MOSS_ASSERT(false, "Restoring state for non-existing constraint");
				return false;
			}
			mConstraints[constraint_index]->RestoreState(inStream);
		}
	}

	return true;
}



void RackAndPinionConstraintSettings::SaveBinaryState(StreamOut &inStream) const
{
	ConstraintSettings::SaveBinaryState(inStream);

	inStream.Write(mSpace);
	inStream.Write(mHingeAxis);
	inStream.Write(mSliderAxis);
	inStream.Write(mRatio);
}

void RackAndPinionConstraintSettings::RestoreBinaryState(StreamIn &inStream)
{
	ConstraintSettings::RestoreBinaryState(inStream);

	inStream.Read(mSpace);
	inStream.Read(mHingeAxis);
	inStream.Read(mSliderAxis);
	inStream.Read(mRatio);
}

TwoBodyConstraint *RackAndPinionConstraintSettings::Create(Body &inBody1, Body &inBody2) const
{
	return new RackAndPinionConstraint(inBody1, inBody2, *this);
}

RackAndPinionConstraint::RackAndPinionConstraint(Body &inBody1, Body &inBody2, const RackAndPinionConstraintSettings &inSettings) :
	TwoBodyConstraint(inBody1, inBody2, inSettings),
	mLocalSpaceHingeAxis(inSettings.mHingeAxis),
	mLocalSpaceSliderAxis(inSettings.mSliderAxis),
	mRatio(inSettings.mRatio)
{
	if (inSettings.mSpace == EConstraintSpace::WorldSpace)
	{
		// If all properties were specified in world space, take them to local space now
		mLocalSpaceHingeAxis = inBody1.GetInverseCenterOfMassTransform().Multiply3x3(mLocalSpaceHingeAxis).Normalized();
		mLocalSpaceSliderAxis = inBody2.GetInverseCenterOfMassTransform().Multiply3x3(mLocalSpaceSliderAxis).Normalized();
	}
}

void RackAndPinionConstraint::CalculateConstraintProperties(Mat44Arg inRotation1, Mat44Arg inRotation2)
{
	// Calculate world space normals
	mWorldSpaceHingeAxis = inRotation1 * mLocalSpaceHingeAxis;
	mWorldSpaceSliderAxis = inRotation2 * mLocalSpaceSliderAxis;

	mRackAndPinionConstraintPart.CalculateConstraintProperties(*mBody1, mWorldSpaceHingeAxis, *mBody2, mWorldSpaceSliderAxis, mRatio);
}

void RackAndPinionConstraint::SetupVelocityConstraint(float inDeltaTime)
{
	// Calculate constraint properties that are constant while bodies don't move
	Mat44 rotation1 = Mat44::sRotation(mBody1->GetRotation());
	Mat44 rotation2 = Mat44::sRotation(mBody2->GetRotation());
	CalculateConstraintProperties(rotation1, rotation2);
}

void RackAndPinionConstraint::ResetWarmStart()
{
	mRackAndPinionConstraintPart.Deactivate();
}

void RackAndPinionConstraint::WarmStartVelocityConstraint(float inWarmStartImpulseRatio)
{
	// Warm starting: Apply previous frame impulse
	mRackAndPinionConstraintPart.WarmStart(*mBody1, *mBody2, inWarmStartImpulseRatio);
}

bool RackAndPinionConstraint::SolveVelocityConstraint(float inDeltaTime)
{
	return mRackAndPinionConstraintPart.SolveVelocityConstraint(*mBody1, mWorldSpaceHingeAxis, *mBody2, mWorldSpaceSliderAxis, mRatio);
}

bool RackAndPinionConstraint::SolvePositionConstraint(float inDeltaTime, float inBaumgarte)
{
	if (mRackConstraint == nullptr || mPinionConstraint == nullptr)
		return false;

	float rotation;
	if (mPinionConstraint->GetSubType() == EConstraintSubType::Hinge)
	{
		rotation = StaticCast<HingeConstraint>(mPinionConstraint)->GetCurrentAngle();
	}
	else
	{
		MOSS_ASSERT(false, "Unsupported");
		return false;
	}

	float translation;
	if (mRackConstraint->GetSubType() == EConstraintSubType::Slider)
	{
		translation = StaticCast<SliderConstraint>(mRackConstraint)->GetCurrentPosition();
	}
	else
	{
		MOSS_ASSERT(false, "Unsupported");
		return false;
	}

	float error = CenterAngleAroundZero(fmod(rotation - mRatio * translation, 2.0f * MOSS_PI));
	if (error == 0.0f)
		return false;

	Mat44 rotation1 = Mat44::sRotation(mBody1->GetRotation());
	Mat44 rotation2 = Mat44::sRotation(mBody2->GetRotation());
	CalculateConstraintProperties(rotation1, rotation2);
	return mRackAndPinionConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, error, inBaumgarte);
}

#ifndef MOSS_DEBUG_RENDERER
void RackAndPinionConstraint::DrawConstraint(DebugRenderer *inRenderer) const
{
	RMat44 transform1 = mBody1->GetCenterOfMassTransform();
	RMat44 transform2 = mBody2->GetCenterOfMassTransform();

	// Draw constraint axis
	inRenderer->DrawArrow(transform1.GetTranslation(), transform1 * mLocalSpaceHingeAxis, Color::sGreen, 0.01f);
	inRenderer->DrawArrow(transform2.GetTranslation(), transform2 * mLocalSpaceSliderAxis, Color::sBlue, 0.01f);
}

#endif // MOSS_DEBUG_RENDERER

void RackAndPinionConstraint::SaveState(StateRecorder &inStream) const
{
	TwoBodyConstraint::SaveState(inStream);

	mRackAndPinionConstraintPart.SaveState(inStream);
}

void RackAndPinionConstraint::RestoreState(StateRecorder &inStream)
{
	TwoBodyConstraint::RestoreState(inStream);

	mRackAndPinionConstraintPart.RestoreState(inStream);
}

Ref<ConstraintSettings> RackAndPinionConstraint::GetConstraintSettings() const
{
	RackAndPinionConstraintSettings *settings = new RackAndPinionConstraintSettings;
	ToConstraintSettings(*settings);
	settings->mSpace = EConstraintSpace::LocalToBodyCOM;
	settings->mHingeAxis = mLocalSpaceHingeAxis;
	settings->mSliderAxis = mLocalSpaceSliderAxis;
	settings->mRatio = mRatio;
	return settings;
}

Mat44 RackAndPinionConstraint::GetConstraintToBody1Matrix() const
{
	Vec3 perp = mLocalSpaceHingeAxis.GetNormalizedPerpendicular();
	return Mat44(Vec4(mLocalSpaceHingeAxis, 0), Vec4(perp, 0), Vec4(mLocalSpaceHingeAxis.Cross(perp), 0), Vec4(0, 0, 0, 1));
}

Mat44 RackAndPinionConstraint::GetConstraintToBody2Matrix() const
{
	Vec3 perp = mLocalSpaceSliderAxis.GetNormalizedPerpendicular();
	return Mat44(Vec4(mLocalSpaceSliderAxis, 0), Vec4(perp, 0), Vec4(mLocalSpaceSliderAxis.Cross(perp), 0), Vec4(0, 0, 0, 1));
}




////////////////////////////////////////////////////////////////////////////////////////////////////////
// ContactConstraintManager::WorldContactPoint
////////////////////////////////////////////////////////////////////////////////////////////////////////

void ContactConstraintManager::WorldContactPoint::CalculateNonPenetrationConstraintProperties(const Body &inBody1, float inInvMass1, float inInvInertiaScale1, const Body &inBody2, float inInvMass2, float inInvInertiaScale2, RVec3Arg inWorldSpacePosition1, RVec3Arg inWorldSpacePosition2, Vec3Arg inWorldSpaceNormal)
{
	// Calculate collision points relative to body
	RVec3 p = 0.5_r * (inWorldSpacePosition1 + inWorldSpacePosition2);
	Vec3 r1 = Vec3(p - inBody1.GetCenterOfMassPosition());
	Vec3 r2 = Vec3(p - inBody2.GetCenterOfMassPosition());

	mNonPenetrationConstraint.CalculateConstraintPropertiesWithMassOverride(inBody1, inInvMass1, inInvInertiaScale1, r1, inBody2, inInvMass2, inInvInertiaScale2, r2, inWorldSpaceNormal);
}

template <EMotionType Type1, EMotionType Type2>
MOSS_INLINE void ContactConstraintManager::WorldContactPoint::TemplatedCalculateFrictionAndNonPenetrationConstraintProperties(float inDeltaTime, float inGravityDeltaTimeDotNormal, const Body &inBody1, const Body &inBody2, float inInvM1, float inInvM2, Mat44Arg inInvI1, Mat44Arg inInvI2, RVec3Arg inWorldSpacePosition1, RVec3Arg inWorldSpacePosition2, Vec3Arg inWorldSpaceNormal, Vec3Arg inWorldSpaceTangent1, Vec3Arg inWorldSpaceTangent2, const ContactSettings &inSettings, float inMinVelocityForRestitution)
{
	MOSS_DET_LOG("TemplatedCalculateFrictionAndNonPenetrationConstraintProperties: p1: " << inWorldSpacePosition1 << " p2: " << inWorldSpacePosition2
		<< " normal: " << inWorldSpaceNormal << " tangent1: " << inWorldSpaceTangent1 << " tangent2: " << inWorldSpaceTangent2
		<< " restitution: " << inSettings.mCombinedRestitution << " friction: " << inSettings.mCombinedFriction << " minv: " << inMinVelocityForRestitution
		<< " surface_vel: " << inSettings.mRelativeLinearSurfaceVelocity << " surface_ang: " << inSettings.mRelativeAngularSurfaceVelocity);

	// Calculate collision points relative to body
	RVec3 p = 0.5_r * (inWorldSpacePosition1 + inWorldSpacePosition2);
	Vec3 r1 = Vec3(p - inBody1.GetCenterOfMassPosition());
	Vec3 r2 = Vec3(p - inBody2.GetCenterOfMassPosition());

	// The gravity is applied in the beginning of the time step. If we get here, there was a collision
	// at the beginning of the time step, so we've applied too much gravity. This means that our
	// calculated restitution can be too high, so when we apply restitution, we cancel the added
	// velocity due to gravity.
	float gravity_dt_dot_normal;

	// Calculate velocity of collision points
	Vec3 relative_velocity;
	if constexpr (Type1 != EMotionType::Static && Type2 != EMotionType::Static)
	{
		const MotionProperties *mp1 = inBody1.GetMotionPropertiesUnchecked();
		const MotionProperties *mp2 = inBody2.GetMotionPropertiesUnchecked();
		relative_velocity = mp2->GetPointVelocityCOM(r2) - mp1->GetPointVelocityCOM(r1);
		gravity_dt_dot_normal = inGravityDeltaTimeDotNormal * (mp2->GetGravityFactor() - mp1->GetGravityFactor());
	}
	else if constexpr (Type1 != EMotionType::Static)
	{
		const MotionProperties *mp1 = inBody1.GetMotionPropertiesUnchecked();
		relative_velocity = -mp1->GetPointVelocityCOM(r1);
		gravity_dt_dot_normal = inGravityDeltaTimeDotNormal * mp1->GetGravityFactor();
	}
	else if constexpr (Type2 != EMotionType::Static)
	{
		const MotionProperties *mp2 = inBody2.GetMotionPropertiesUnchecked();
		relative_velocity = mp2->GetPointVelocityCOM(r2);
		gravity_dt_dot_normal = inGravityDeltaTimeDotNormal * mp2->GetGravityFactor();
	}
	else
	{
		MOSS_ASSERT(false); // Static vs static makes no sense
		relative_velocity = Vec3::sZero();
		gravity_dt_dot_normal = 0.0f;
	}
	float normal_velocity = relative_velocity.Dot(inWorldSpaceNormal);

	// How much the shapes are penetrating (> 0 if penetrating, < 0 if separated)
	float penetration = Vec3(inWorldSpacePosition1 - inWorldSpacePosition2).Dot(inWorldSpaceNormal);

	// If there is no penetration, this is a speculative contact and we will apply a bias to the contact constraint
	// so that the constraint becomes relative_velocity . contact normal > -penetration / delta_time
	// instead of relative_velocity . contact normal > 0
	// See: GDC 2013: "Physics for Game Programmers; Continuous Collision" - Erin Catto
	float speculative_contact_velocity_bias = max(0.0f, -penetration / inDeltaTime);

	// Determine if the velocity is big enough for restitution
	float normal_velocity_bias;
	if (inSettings.mCombinedRestitution > 0.0f && normal_velocity < -inMinVelocityForRestitution)
	{
		// We have a velocity that is big enough for restitution. This is where speculative contacts don't work
		// great as we have to decide now if we're going to apply the restitution or not. If the relative
		// velocity is big enough for a hit, we apply the restitution (in the end, due to other constraints,
		// the objects may actually not collide and we will have applied restitution incorrectly). Another
		// artifact that occurs because of this approximation is that the object will bounce from its current
		// position rather than from a position where it is touching the other object. This causes the object
		// to appear to move faster for 1 frame (the opposite of time stealing).
		if (normal_velocity < -speculative_contact_velocity_bias)
			normal_velocity_bias = inSettings.mCombinedRestitution * (normal_velocity - gravity_dt_dot_normal);
		else
			// In this case we have predicted that we don't hit the other object, but if we do (due to other constraints changing velocities)
			// the speculative contact will prevent penetration but will not apply restitution leading to another artifact.
			normal_velocity_bias = speculative_contact_velocity_bias;
	}
	else
	{
		// No restitution. We can safely apply our contact velocity bias.
		normal_velocity_bias = speculative_contact_velocity_bias;
	}

	mNonPenetrationConstraint.TemplatedCalculateConstraintProperties<Type1, Type2>(inInvM1, inInvI1, r1, inInvM2, inInvI2, r2, inWorldSpaceNormal, normal_velocity_bias);

	// Calculate friction part
	if (inSettings.mCombinedFriction > 0.0f)
	{
		// Get surface velocity relative to tangents
		Vec3 ws_surface_velocity = inSettings.mRelativeLinearSurfaceVelocity + inSettings.mRelativeAngularSurfaceVelocity.Cross(r1);
		float surface_velocity1 = inWorldSpaceTangent1.Dot(ws_surface_velocity);
		float surface_velocity2 = inWorldSpaceTangent2.Dot(ws_surface_velocity);

		// Implement friction as 2 AxisConstraintParts
		mFrictionConstraint1.TemplatedCalculateConstraintProperties<Type1, Type2>(inInvM1, inInvI1, r1, inInvM2, inInvI2, r2, inWorldSpaceTangent1, surface_velocity1);
		mFrictionConstraint2.TemplatedCalculateConstraintProperties<Type1, Type2>(inInvM1, inInvI1, r1, inInvM2, inInvI2, r2, inWorldSpaceTangent2, surface_velocity2);
	}
	else
	{
		// Turn off friction constraint
		mFrictionConstraint1.Deactivate();
		mFrictionConstraint2.Deactivate();
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// ContactConstraintManager::ContactConstraint
////////////////////////////////////////////////////////////////////////////////////////////////////////

#ifndef MOSS_DEBUG_RENDERER
void ContactConstraintManager::ContactConstraint::Draw(DebugRenderer *inRenderer, ColorArg inManifoldColor) const
{
	if (mContactPoints.empty())
		return;

	// Get body transforms
	RMat44 transform_body1 = mBody1->GetCenterOfMassTransform();
	RMat44 transform_body2 = mBody2->GetCenterOfMassTransform();

	RVec3 prev_point = transform_body1 * Vec3::sLoadFloat3Unsafe(mContactPoints.back().mContactPoint->mPosition1);
	for (const WorldContactPoint &wcp : mContactPoints)
	{
		// Test if any lambda from the previous frame was transferred
		float radius = wcp.mNonPenetrationConstraint.GetTotalLambda() == 0.0f
					&& wcp.mFrictionConstraint1.GetTotalLambda() == 0.0f
					&& wcp.mFrictionConstraint2.GetTotalLambda() == 0.0f? 0.1f :  0.2f;

		RVec3 next_point = transform_body1 * Vec3::sLoadFloat3Unsafe(wcp.mContactPoint->mPosition1);
		inRenderer->DrawMarker(next_point, Color::sCyan, radius);
		inRenderer->DrawMarker(transform_body2 * Vec3::sLoadFloat3Unsafe(wcp.mContactPoint->mPosition2), Color::sPurple, radius);

		// Draw edge
		inRenderer->DrawArrow(prev_point, next_point, inManifoldColor, 0.05f);
		prev_point = next_point;
	}

	// Draw normal
	RVec3 wp = transform_body1 * Vec3::sLoadFloat3Unsafe(mContactPoints[0].mContactPoint->mPosition1);
	inRenderer->DrawArrow(wp, wp + GetWorldSpaceNormal(), Color::sRed, 0.05f);

	// Get tangents
	Vec3 t1, t2;
	GetTangents(t1, t2);

	// Draw tangents
	inRenderer->DrawLine(wp, wp + t1, Color::sGreen);
	inRenderer->DrawLine(wp, wp + t2, Color::sBlue);
}
#endif // MOSS_DEBUG_RENDERER

////////////////////////////////////////////////////////////////////////////////////////////////////////
// ContactConstraintManager::CachedContactPoint
////////////////////////////////////////////////////////////////////////////////////////////////////////

void ContactConstraintManager::CachedContactPoint::SaveState(StateRecorder &inStream) const
{
	inStream.Write(mPosition1);
	inStream.Write(mPosition2);
	inStream.Write(mNonPenetrationLambda);
	inStream.Write(mFrictionLambda);
}

void ContactConstraintManager::CachedContactPoint::RestoreState(StateRecorder &inStream)
{
	inStream.Read(mPosition1);
	inStream.Read(mPosition2);
	inStream.Read(mNonPenetrationLambda);
	inStream.Read(mFrictionLambda);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// ContactConstraintManager::CachedManifold
////////////////////////////////////////////////////////////////////////////////////////////////////////

void ContactConstraintManager::CachedManifold::SaveState(StateRecorder &inStream) const
{
	inStream.Write(mContactNormal);
}

void ContactConstraintManager::CachedManifold::RestoreState(StateRecorder &inStream)
{
	inStream.Read(mContactNormal);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// ContactConstraintManager::CachedBodyPair
////////////////////////////////////////////////////////////////////////////////////////////////////////

void ContactConstraintManager::CachedBodyPair::SaveState(StateRecorder &inStream) const
{
	inStream.Write(mDeltaPosition);
	inStream.Write(mDeltaRotation);
}

void ContactConstraintManager::CachedBodyPair::RestoreState(StateRecorder &inStream)
{
	inStream.Read(mDeltaPosition);
	inStream.Read(mDeltaRotation);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// ContactConstraintManager::ManifoldCache
////////////////////////////////////////////////////////////////////////////////////////////////////////

void ContactConstraintManager::ManifoldCache::Init(uint inMaxBodyPairs, uint inMaxContactConstraints, uint inCachedManifoldsSize)
{
	uint max_body_pairs = min(inMaxBodyPairs, cMaxBodyPairsLimit);
	MOSS_ASSERT(max_body_pairs == inMaxBodyPairs, "Cannot support this many body pairs!");
	MOSS_ASSERT(inMaxContactConstraints <= cMaxContactConstraintsLimit); // Should have been enforced by caller

	mAllocator.Init(uint(min(uint64(max_body_pairs) * sizeof(BodyPairMap::KeyValue) + inCachedManifoldsSize, uint64(~uint(0)))));

	mCachedManifolds.Init(GetNextPowerOf2(inMaxContactConstraints));
	mCachedBodyPairs.Init(GetNextPowerOf2(max_body_pairs));
}

void ContactConstraintManager::ManifoldCache::Clear()
{
	MOSS_PROFILE_FUNCTION();

	mCachedManifolds.Clear();
	mCachedBodyPairs.Clear();
	mAllocator.Clear();

#ifdef MOSS_DEBUG
	// Mark as incomplete
	mIsFinalized = false;
#endif
}

void ContactConstraintManager::ManifoldCache::Prepare(uint inExpectedNumBodyPairs, uint inExpectedNumManifolds)
{
	// Minimum amount of buckets to use in the hash map
	constexpr uint32 cMinBuckets = 1024;

	// Use the next higher power of 2 of amount of objects in the cache from last frame to determine the amount of buckets in this frame
	mCachedManifolds.SetNumBuckets(min(max(cMinBuckets, GetNextPowerOf2(inExpectedNumManifolds)), mCachedManifolds.GetMaxBuckets()));
	mCachedBodyPairs.SetNumBuckets(min(max(cMinBuckets, GetNextPowerOf2(inExpectedNumBodyPairs)), mCachedBodyPairs.GetMaxBuckets()));
}

const ContactConstraintManager::MKeyValue *ContactConstraintManager::ManifoldCache::Find(const SubShapeIDPair &inKey, uint64 inKeyHash) const
{
	MOSS_ASSERT(mIsFinalized);
	return mCachedManifolds.Find(inKey, inKeyHash);
}

ContactConstraintManager::MKeyValue *ContactConstraintManager::ManifoldCache::Create(ContactAllocator &ioContactAllocator, const SubShapeIDPair &inKey, uint64 inKeyHash, int inNumContactPoints)
{
	MOSS_ASSERT(!mIsFinalized);
	MKeyValue *kv = mCachedManifolds.Create(ioContactAllocator, inKey, inKeyHash, CachedManifold::sGetRequiredExtraSize(inNumContactPoints));
	if (kv == nullptr)
	{
		ioContactAllocator.mErrors |= EPhysicsUpdateError::ManifoldCacheFull;
		return nullptr;
	}
	kv->GetValue().mNumContactPoints = uint16(inNumContactPoints);
	++ioContactAllocator.mNumManifolds;
	return kv;
}

ContactConstraintManager::MKVAndCreated ContactConstraintManager::ManifoldCache::FindOrCreate(ContactAllocator &ioContactAllocator, const SubShapeIDPair &inKey, uint64 inKeyHash, int inNumContactPoints)
{
	MKeyValue *kv = const_cast<MKeyValue *>(mCachedManifolds.Find(inKey, inKeyHash));
	if (kv != nullptr)
		return { kv, false };

	return { Create(ioContactAllocator, inKey, inKeyHash, inNumContactPoints), true };
}

uint32 ContactConstraintManager::ManifoldCache::ToHandle(const MKeyValue *inKeyValue) const
{
	MOSS_ASSERT(!mIsFinalized);
	return mCachedManifolds.ToHandle(inKeyValue);
}

const ContactConstraintManager::MKeyValue *ContactConstraintManager::ManifoldCache::FromHandle(uint32 inHandle) const
{
	MOSS_ASSERT(mIsFinalized);
	return mCachedManifolds.FromHandle(inHandle);
}

const ContactConstraintManager::BPKeyValue *ContactConstraintManager::ManifoldCache::Find(const BodyPair &inKey, uint64 inKeyHash) const
{
	MOSS_ASSERT(mIsFinalized);
	return mCachedBodyPairs.Find(inKey, inKeyHash);
}

ContactConstraintManager::BPKeyValue *ContactConstraintManager::ManifoldCache::Create(ContactAllocator &ioContactAllocator, const BodyPair &inKey, uint64 inKeyHash)
{
	MOSS_ASSERT(!mIsFinalized);
	BPKeyValue *kv = mCachedBodyPairs.Create(ioContactAllocator, inKey, inKeyHash, 0);
	if (kv == nullptr)
	{
		ioContactAllocator.mErrors |= EPhysicsUpdateError::BodyPairCacheFull;
		return nullptr;
	}
	++ioContactAllocator.mNumBodyPairs;
	return kv;
}

void ContactConstraintManager::ManifoldCache::GetAllBodyPairsSorted(TArray<const BPKeyValue *> &outAll) const
{
	MOSS_ASSERT(mIsFinalized);
	mCachedBodyPairs.GetAllKeyValues(outAll);

	// Sort by key
	QuickSort(outAll.begin(), outAll.end(), [](const BPKeyValue *inLHS, const BPKeyValue *inRHS) {
		return inLHS->GetKey() < inRHS->GetKey();
	});
}

void ContactConstraintManager::ManifoldCache::GetAllManifoldsSorted(const CachedBodyPair &inBodyPair, TArray<const MKeyValue *> &outAll) const
{
	MOSS_ASSERT(mIsFinalized);

	// Iterate through the attached manifolds
	for (uint32 handle = inBodyPair.mFirstCachedManifold; handle != ManifoldMap::cInvalidHandle; handle = FromHandle(handle)->GetValue().mNextWithSameBodyPair)
	{
		const MKeyValue *kv = mCachedManifolds.FromHandle(handle);
		outAll.push_back(kv);
	}

	// Sort by key
	QuickSort(outAll.begin(), outAll.end(), [](const MKeyValue *inLHS, const MKeyValue *inRHS) {
		return inLHS->GetKey() < inRHS->GetKey();
	});
}

void ContactConstraintManager::ManifoldCache::GetAllCCDManifoldsSorted(TArray<const MKeyValue *> &outAll) const
{
	mCachedManifolds.GetAllKeyValues(outAll);

	for (int i = (int)outAll.size() - 1; i >= 0; --i)
		if ((outAll[i]->GetValue().mFlags & (uint16)CachedManifold::EFlags::CCDContact) == 0)
		{
			outAll[i] = outAll.back();
			outAll.pop_back();
		}

	// Sort by key
	QuickSort(outAll.begin(), outAll.end(), [](const MKeyValue *inLHS, const MKeyValue *inRHS) {
		return inLHS->GetKey() < inRHS->GetKey();
	});
}

void ContactConstraintManager::ManifoldCache::ContactPointRemovedCallbacks(ContactListener *inListener)
{
	MOSS_PROFILE_FUNCTION();

	for (MKeyValue &kv : mCachedManifolds)
		if ((kv.GetValue().mFlags & uint16(CachedManifold::EFlags::ContactPersisted)) == 0)
			inListener->OnContactRemoved(kv.GetKey());
}

#ifdef MOSS_DEBUG

void ContactConstraintManager::ManifoldCache::Finalize()
{
	mIsFinalized = true;

#ifdef MOSS_MANIFOLD_CACHE_DEBUG
	MOSS_TRACE("ManifoldMap:");
	mCachedManifolds.TraceStats();
	MOSS_TRACE("BodyPairMap:");
	mCachedBodyPairs.TraceStats();
#endif // MOSS_MANIFOLD_CACHE_DEBUG
}

#endif

void ContactConstraintManager::ManifoldCache::SaveState(StateRecorder &inStream, const StateRecorderFilter *inFilter) const
{
	MOSS_ASSERT(mIsFinalized);

	// Get contents of cache
	TArray<const BPKeyValue *> all_bp;
	GetAllBodyPairsSorted(all_bp);

	// Determine which ones to save
	TArray<const BPKeyValue *> selected_bp;
	if (inFilter == nullptr)
		selected_bp = std::move(all_bp);
	else
	{
		selected_bp.reserve(all_bp.size());
		for (const BPKeyValue *bp_kv : all_bp)
			if (inFilter->ShouldSaveContact(bp_kv->GetKey().mBodyA, bp_kv->GetKey().mBodyB))
				selected_bp.push_back(bp_kv);
	}

	// Write body pairs
	uint32 num_body_pairs = uint32(selected_bp.size());
	inStream.Write(num_body_pairs);
	for (const BPKeyValue *bp_kv : selected_bp)
	{
		// Write body pair key
		inStream.Write(bp_kv->GetKey());

		// Write body pair
		const CachedBodyPair &bp = bp_kv->GetValue();
		bp.SaveState(inStream);

		// Get attached manifolds
		TArray<const MKeyValue *> all_m;
		GetAllManifoldsSorted(bp, all_m);

		// Write num manifolds
		uint32 num_manifolds = uint32(all_m.size());
		inStream.Write(num_manifolds);

		// Write all manifolds
		for (const MKeyValue *m_kv : all_m)
		{
			// Write key
			inStream.Write(m_kv->GetKey());
			const CachedManifold &cm = m_kv->GetValue();
			MOSS_ASSERT((cm.mFlags & (uint16)CachedManifold::EFlags::CCDContact) == 0);

			// Write amount of contacts
			inStream.Write(cm.mNumContactPoints);

			// Write manifold
			cm.SaveState(inStream);

			// Write contact points
			for (uint32 i = 0; i < cm.mNumContactPoints; ++i)
				cm.mContactPoints[i].SaveState(inStream);
		}
	}

	// Get CCD manifolds
	TArray<const MKeyValue *> all_m;
	GetAllCCDManifoldsSorted(all_m);

	// Determine which ones to save
	TArray<const MKeyValue *> selected_m;
	if (inFilter == nullptr)
		selected_m = std::move(all_m);
	else
	{
		selected_m.reserve(all_m.size());
		for (const MKeyValue *m_kv : all_m)
			if (inFilter->ShouldSaveContact(m_kv->GetKey().GetBody1ID(), m_kv->GetKey().GetBody2ID()))
				selected_m.push_back(m_kv);
	}

	// Write all CCD manifold keys
	uint32 num_manifolds = uint32(selected_m.size());
	inStream.Write(num_manifolds);
	for (const MKeyValue *m_kv : selected_m)
		inStream.Write(m_kv->GetKey());
}

bool ContactConstraintManager::ManifoldCache::RestoreState(const ManifoldCache &inReadCache, StateRecorder &inStream, const StateRecorderFilter *inFilter)
{
	MOSS_ASSERT(!mIsFinalized);

	bool success = true;

	// Create a contact allocator for restoring the contact cache
	ContactAllocator contact_allocator(GetContactAllocator());

	// When validating, get all existing body pairs
	TArray<const BPKeyValue *> all_bp;
	if (inStream.IsValidating())
		inReadCache.GetAllBodyPairsSorted(all_bp);

	// Read amount of body pairs
	uint32 num_body_pairs;
	if (inStream.IsValidating())
		num_body_pairs = uint32(all_bp.size());
	inStream.Read(num_body_pairs);

	// Read entire cache
	for (uint32 i = 0; i < num_body_pairs; ++i)
	{
		// Read key
		BodyPair body_pair_key;
		if (inStream.IsValidating() && i < all_bp.size())
			body_pair_key = all_bp[i]->GetKey();
		inStream.Read(body_pair_key);

		// Check if we want to restore this contact
		if (inFilter == nullptr || inFilter->ShouldRestoreContact(body_pair_key.mBodyA, body_pair_key.mBodyB))
		{
			// Create new entry for this body pair
			uint64 body_pair_hash = body_pair_key.GetHash();
			BPKeyValue *bp_kv = Create(contact_allocator, body_pair_key, body_pair_hash);
			if (bp_kv == nullptr)
			{
				// Out of cache space
				success = false;
				break;
			}
			CachedBodyPair &bp = bp_kv->GetValue();

			// Read body pair
			if (inStream.IsValidating() && i < all_bp.size())
				memcpy(&bp, &all_bp[i]->GetValue(), sizeof(CachedBodyPair));
			bp.RestoreState(inStream);

			// When validating, get all existing manifolds
			TArray<const MKeyValue *> all_m;
			if (inStream.IsValidating())
				inReadCache.GetAllManifoldsSorted(all_bp[i]->GetValue(), all_m);

			// Read amount of manifolds
			uint32 num_manifolds = 0;
			if (inStream.IsValidating())
				num_manifolds = uint32(all_m.size());
			inStream.Read(num_manifolds);

			uint32 handle = ManifoldMap::cInvalidHandle;
			for (uint32 j = 0; j < num_manifolds; ++j)
			{
				// Read key
				SubShapeIDPair sub_shape_key;
				if (inStream.IsValidating() && j < all_m.size())
					sub_shape_key = all_m[j]->GetKey();
				inStream.Read(sub_shape_key);
				uint64 sub_shape_key_hash = sub_shape_key.GetHash();

				// Read amount of contact points
				uint16 num_contact_points = 0;
				if (inStream.IsValidating() && j < all_m.size())
					num_contact_points = all_m[j]->GetValue().mNumContactPoints;
				inStream.Read(num_contact_points);

				// Read manifold
				MKeyValue *m_kv = Create(contact_allocator, sub_shape_key, sub_shape_key_hash, num_contact_points);
				if (m_kv == nullptr)
				{
					// Out of cache space
					success = false;
					break;
				}
				CachedManifold &cm = m_kv->GetValue();
				if (inStream.IsValidating() && j < all_m.size())
				{
					memcpy(&cm, &all_m[j]->GetValue(), CachedManifold::sGetRequiredTotalSize(num_contact_points));
					cm.mNumContactPoints = uint16(num_contact_points); // Restore num contact points
				}
				cm.RestoreState(inStream);
				cm.mNextWithSameBodyPair = handle;
				handle = ToHandle(m_kv);

				// Read contact points
				for (uint32 k = 0; k < num_contact_points; ++k)
					cm.mContactPoints[k].RestoreState(inStream);
			}
			bp.mFirstCachedManifold = handle;
		}
		else
		{
			// Skip the contact
			CachedBodyPair bp;
			bp.RestoreState(inStream);
			uint32 num_manifolds = 0;
			inStream.Read(num_manifolds);
			for (uint32 j = 0; j < num_manifolds; ++j)
			{
				SubShapeIDPair sub_shape_key;
				inStream.Read(sub_shape_key);
				uint16 num_contact_points;
				inStream.Read(num_contact_points);
				CachedManifold cm;
				cm.RestoreState(inStream);
				for (uint32 k = 0; k < num_contact_points; ++k)
					cm.mContactPoints[0].RestoreState(inStream);
			}
		}
	}

	// When validating, get all existing CCD manifolds
	TArray<const MKeyValue *> all_m;
	if (inStream.IsValidating())
		inReadCache.GetAllCCDManifoldsSorted(all_m);

	// Read amount of CCD manifolds
	uint32 num_manifolds;
	if (inStream.IsValidating())
		num_manifolds = uint32(all_m.size());
	inStream.Read(num_manifolds);

	for (uint32 j = 0; j < num_manifolds; ++j)
	{
		// Read key
		SubShapeIDPair sub_shape_key;
		if (inStream.IsValidating() && j < all_m.size())
			sub_shape_key = all_m[j]->GetKey();
		inStream.Read(sub_shape_key);

		// Check if we want to restore this contact
		if (inFilter == nullptr || inFilter->ShouldRestoreContact(sub_shape_key.GetBody1ID(), sub_shape_key.GetBody2ID()))
		{
			// Create CCD manifold
			uint64 sub_shape_key_hash = sub_shape_key.GetHash();
			MKeyValue *m_kv = Create(contact_allocator, sub_shape_key, sub_shape_key_hash, 0);
			if (m_kv == nullptr)
			{
				// Out of cache space
				success = false;
				break;
			}
			CachedManifold &cm = m_kv->GetValue();
			cm.mFlags |= (uint16)CachedManifold::EFlags::CCDContact;
		}
	}

#ifdef MOSS_DEBUG
	// We don't finalize until the last part is restored
	if (inStream.IsLastPart())
		mIsFinalized = true;
#endif

	return success;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// ContactConstraintManager
////////////////////////////////////////////////////////////////////////////////////////////////////////

ContactConstraintManager::ContactConstraintManager(const PhysicsSettings &inPhysicsSettings) :
	mPhysicsSettings(inPhysicsSettings)
{
#ifdef MOSS_DEBUG
	// For the first frame mark this empty buffer as finalized
	mCache[mCacheWriteIdx ^ 1].Finalize();
#endif
}

ContactConstraintManager::~ContactConstraintManager()
{
	MOSS_ASSERT(mConstraints == nullptr);
}

void ContactConstraintManager::Init(uint inMaxBodyPairs, uint inMaxContactConstraints)
{
	// Limit the number of constraints so that the allocation size fits in an unsigned integer
	mMaxConstraints = min(inMaxContactConstraints, cMaxContactConstraintsLimit);
	MOSS_ASSERT(mMaxConstraints == inMaxContactConstraints, "Cannot support this many contact constraints!");

	// Calculate worst case cache usage
	constexpr uint cMaxManifoldSizePerConstraint = sizeof(CachedManifold) + (MaxContactPoints - 1) * sizeof(CachedContactPoint);
	static_assert(cMaxManifoldSizePerConstraint < sizeof(ContactConstraint)); // If not true, then the next line can overflow
	uint cached_manifolds_size = mMaxConstraints * cMaxManifoldSizePerConstraint;

	// Init the caches
	mCache[0].Init(inMaxBodyPairs, mMaxConstraints, cached_manifolds_size);
	mCache[1].Init(inMaxBodyPairs, mMaxConstraints, cached_manifolds_size);
}

void ContactConstraintManager::PrepareConstraintBuffer(PhysicsUpdateContext *inContext)
{
	// Store context
	mUpdateContext = inContext;

	// Allocate temporary constraint buffer
	MOSS_ASSERT(mConstraints == nullptr);
	mConstraints = (ContactConstraint *)inContext->mTempAllocator->Allocate(mMaxConstraints * sizeof(ContactConstraint));
}

template <EMotionType Type1, EMotionType Type2>
MOSS_INLINE void ContactConstraintManager::TemplatedCalculateFrictionAndNonPenetrationConstraintProperties(ContactConstraint &ioConstraint, const ContactSettings &inSettings, float inDeltaTime, Vec3Arg inGravityDeltaTime, RMat44Arg inTransformBody1, RMat44Arg inTransformBody2, const Body &inBody1, const Body &inBody2)
{
	// Calculate scaled mass and inertia
	Mat44 inv_i1;
	if constexpr (Type1 == EMotionType::Dynamic)
	{
		const MotionProperties *mp1 = inBody1.GetMotionPropertiesUnchecked();
		inv_i1 = inSettings.mInvInertiaScale1 * mp1->GetInverseInertiaForRotation(inTransformBody1.GetRotation());
	}
	else
	{
		inv_i1 = Mat44::sZero();
	}

	Mat44 inv_i2;
	if constexpr (Type2 == EMotionType::Dynamic)
	{
		const MotionProperties *mp2 = inBody2.GetMotionPropertiesUnchecked();
		inv_i2 = inSettings.mInvInertiaScale2 * mp2->GetInverseInertiaForRotation(inTransformBody2.GetRotation());
	}
	else
	{
		inv_i2 = Mat44::sZero();
	}

	// Calculate tangents
	Vec3 t1, t2;
	ioConstraint.GetTangents(t1, t2);

	Vec3 ws_normal = ioConstraint.GetWorldSpaceNormal();

	// Calculate value for restitution correction
	float gravity_dt_dot_normal = inGravityDeltaTime.Dot(ws_normal);

	// Setup velocity constraint properties
	float min_velocity_for_restitution = mPhysicsSettings.mMinVelocityForRestitution;
	for (WorldContactPoint &wcp : ioConstraint.mContactPoints)
	{
		RVec3 p1 = inTransformBody1 * Vec3::sLoadFloat3Unsafe(wcp.mContactPoint->mPosition1);
		RVec3 p2 = inTransformBody2 * Vec3::sLoadFloat3Unsafe(wcp.mContactPoint->mPosition2);
		wcp.TemplatedCalculateFrictionAndNonPenetrationConstraintProperties<Type1, Type2>(inDeltaTime, gravity_dt_dot_normal, inBody1, inBody2, ioConstraint.mInvMass1, ioConstraint.mInvMass2, inv_i1, inv_i2, p1, p2, ws_normal, t1, t2, inSettings, min_velocity_for_restitution);
	}
}

inline void ContactConstraintManager::CalculateFrictionAndNonPenetrationConstraintProperties(ContactConstraint &ioConstraint, const ContactSettings &inSettings, float inDeltaTime, Vec3Arg inGravityDeltaTime, RMat44Arg inTransformBody1, RMat44Arg inTransformBody2, const Body &inBody1, const Body &inBody2)
{
	// Dispatch to the correct templated form
	switch (inBody1.GetMotionType())
	{
	case EMotionType::Dynamic:
		switch (inBody2.GetMotionType())
		{
		case EMotionType::Dynamic:
			TemplatedCalculateFrictionAndNonPenetrationConstraintProperties<EMotionType::Dynamic, EMotionType::Dynamic>(ioConstraint, inSettings, inDeltaTime, inGravityDeltaTime, inTransformBody1, inTransformBody2, inBody1, inBody2);
			break;

		case EMotionType::Kinematic:
			TemplatedCalculateFrictionAndNonPenetrationConstraintProperties<EMotionType::Dynamic, EMotionType::Kinematic>(ioConstraint, inSettings, inDeltaTime, inGravityDeltaTime, inTransformBody1, inTransformBody2, inBody1, inBody2);
			break;

		case EMotionType::Static:
			TemplatedCalculateFrictionAndNonPenetrationConstraintProperties<EMotionType::Dynamic, EMotionType::Static>(ioConstraint, inSettings, inDeltaTime, inGravityDeltaTime, inTransformBody1, inTransformBody2, inBody1, inBody2);
			break;

		default:
			MOSS_ASSERT(false);
			break;
		}
		break;

	case EMotionType::Kinematic:
		MOSS_ASSERT(inBody2.IsDynamic());
		TemplatedCalculateFrictionAndNonPenetrationConstraintProperties<EMotionType::Kinematic, EMotionType::Dynamic>(ioConstraint, inSettings, inDeltaTime, inGravityDeltaTime, inTransformBody1, inTransformBody2, inBody1, inBody2);
		break;

	case EMotionType::Static:
		MOSS_ASSERT(inBody2.IsDynamic());
		TemplatedCalculateFrictionAndNonPenetrationConstraintProperties<EMotionType::Static, EMotionType::Dynamic>(ioConstraint, inSettings, inDeltaTime, inGravityDeltaTime, inTransformBody1, inTransformBody2, inBody1, inBody2);
		break;

	default:
		MOSS_ASSERT(false);
		break;
	}
}

void ContactConstraintManager::GetContactsFromCache(ContactAllocator &ioContactAllocator, Body &inBody1, Body &inBody2, bool &outPairHandled, bool &outConstraintCreated)
{
	MOSS_PROFILE_FUNCTION();

	// Start with nothing found and not handled
	outConstraintCreated = false;
	outPairHandled = false;

	// Swap bodies so that body 1 id < body 2 id
	Body *body1, *body2;
	if (inBody1.GetID() < inBody2.GetID())
	{
		body1 = &inBody1;
		body2 = &inBody2;
	}
	else
	{
		body1 = &inBody2;
		body2 = &inBody1;
	}

	// Find the cached body pair
	BodyPair body_pair_key(body1->GetID(), body2->GetID());
	uint64 body_pair_hash = body_pair_key.GetHash();
	const ManifoldCache &read_cache = mCache[mCacheWriteIdx ^ 1];
	const BPKeyValue *kv = read_cache.Find(body_pair_key, body_pair_hash);
	if (kv == nullptr)
		return;
	const CachedBodyPair &input_cbp = kv->GetValue();

	// Get relative translation
	Quat inv_r1 = body1->GetRotation().Conjugated();
	Vec3 delta_position = inv_r1 * Vec3(body2->GetCenterOfMassPosition() - body1->GetCenterOfMassPosition());

	// Get old position delta
	Vec3 old_delta_position = Vec3::sLoadFloat3Unsafe(input_cbp.mDeltaPosition);

	// Check if bodies are still roughly in the same relative position
	if ((delta_position - old_delta_position).LengthSq() > mPhysicsSettings.mBodyPairCacheMaxDeltaPositionSq)
		return;

	// Determine relative orientation
	Quat delta_rotation = inv_r1 * body2->GetRotation();

	// Reconstruct old quaternion delta
	Quat old_delta_rotation = Quat::sLoadFloat3Unsafe(input_cbp.mDeltaRotation);

	// Check if bodies are still roughly in the same relative orientation
	// The delta between 2 quaternions p and q is: p q^* = [rotation_axis * sin(angle / 2), cos(angle / 2)]
	// From the W component we can extract the angle: cos(angle / 2) = px * qx + py * qy + pz * qz + pw * qw = p . q
	// Since we want to abort if the rotation is smaller than -angle or bigger than angle, we can write the comparison as |p . q| < cos(angle / 2)
	if (abs(delta_rotation.Dot(old_delta_rotation)) < mPhysicsSettings.mBodyPairCacheCosMaxDeltaRotationDiv2)
		return;

	// The cache is valid, return that we've handled this body pair
	outPairHandled = true;

	// Copy the cached body pair to this frame
	ManifoldCache &write_cache = mCache[mCacheWriteIdx];
	BPKeyValue *output_bp_kv = write_cache.Create(ioContactAllocator, body_pair_key, body_pair_hash);
	if (output_bp_kv == nullptr)
		return; // Out of cache space
	CachedBodyPair *output_cbp = &output_bp_kv->GetValue();
	memcpy(output_cbp, &input_cbp, sizeof(CachedBodyPair));

	// If there were no contacts, we have handled the contact
	if (input_cbp.mFirstCachedManifold == ManifoldMap::cInvalidHandle)
		return;

	// Get body transforms
	RMat44 transform_body1 = body1->GetCenterOfMassTransform();
	RMat44 transform_body2 = body2->GetCenterOfMassTransform();

	// Get time step
	float delta_time = mUpdateContext->mStepDeltaTime;

	// Calculate value for restitution correction
	Vec3 gravity_dt = mUpdateContext->mPhysicsSystem->GetGravity() * delta_time;

	// Copy manifolds
	uint32 output_handle = ManifoldMap::cInvalidHandle;
	uint32 input_handle = input_cbp.mFirstCachedManifold;
	do
	{
		MOSS_PROFILE("Add Constraint From Cached Manifold");

		// Find the existing manifold
		const MKeyValue *input_kv = read_cache.FromHandle(input_handle);
		const SubShapeIDPair &input_key = input_kv->GetKey();
		const CachedManifold &input_cm = input_kv->GetValue();
		MOSS_ASSERT(input_cm.mNumContactPoints > 0); // There should be contact points in this manifold!

		// Create room for manifold in write buffer and copy data
		uint64 input_hash = input_key.GetHash();
		MKeyValue *output_kv = write_cache.Create(ioContactAllocator, input_key, input_hash, input_cm.mNumContactPoints);
		if (output_kv == nullptr)
			break; // Out of cache space
		CachedManifold *output_cm = &output_kv->GetValue();
		memcpy(output_cm, &input_cm, CachedManifold::sGetRequiredTotalSize(input_cm.mNumContactPoints));

		// Link the object under the body pairs
		output_cm->mNextWithSameBodyPair = output_handle;
		output_handle = write_cache.ToHandle(output_kv);

		// Calculate default contact settings
		ContactSettings settings;
		settings.mCombinedFriction = mCombineFriction(*body1, input_key.GetSubShapeID1(), *body2, input_key.GetSubShapeID2());
		settings.mCombinedRestitution = mCombineRestitution(*body1, input_key.GetSubShapeID1(), *body2, input_key.GetSubShapeID2());
		settings.mIsSensor = body1->IsSensor() || body2->IsSensor();

		// Calculate world space contact normal
		Vec3 world_space_normal = transform_body2.Multiply3x3(Vec3::sLoadFloat3Unsafe(output_cm->mContactNormal)).Normalized();

		// Call contact listener to update settings
		if (mContactListener != nullptr)
		{
			// Convert constraint to manifold structure for callback
			ContactManifold manifold;
			manifold.mWorldSpaceNormal = world_space_normal;
			manifold.mSubShapeID1 = input_key.GetSubShapeID1();
			manifold.mSubShapeID2 = input_key.GetSubShapeID2();
			manifold.mBaseOffset = transform_body1.GetTranslation();
			manifold.mRelativeContactPointsOn1.resize(output_cm->mNumContactPoints);
			manifold.mRelativeContactPointsOn2.resize(output_cm->mNumContactPoints);
			Mat44 local_transform_body2 = transform_body2.PostTranslated(-manifold.mBaseOffset).ToMat44();
			float penetration_depth = -FLT_MAX;
			for (uint32 i = 0; i < output_cm->mNumContactPoints; ++i)
			{
				const CachedContactPoint &ccp = output_cm->mContactPoints[i];
				manifold.mRelativeContactPointsOn1[i] = transform_body1.Multiply3x3(Vec3::sLoadFloat3Unsafe(ccp.mPosition1));
				manifold.mRelativeContactPointsOn2[i] = local_transform_body2 * Vec3::sLoadFloat3Unsafe(ccp.mPosition2);
				penetration_depth = max(penetration_depth, (manifold.mRelativeContactPointsOn1[0] - manifold.mRelativeContactPointsOn2[0]).Dot(world_space_normal));
			}
			manifold.mPenetrationDepth = penetration_depth; // We don't have the penetration depth anymore, estimate it

			// Notify callback
			mContactListener->OnContactPersisted(*body1, *body2, manifold, settings);
		}

		MOSS_ASSERT(settings.mIsSensor || !(body1->IsSensor() || body2->IsSensor()), "Sensors cannot be converted into regular bodies by a contact callback!");
		if (!settings.mIsSensor // If one of the bodies is a sensor, don't actually create the constraint
			&& ((body1->IsDynamic() && settings.mInvMassScale1 != 0.0f) // One of the bodies must have mass to be able to create a contact constraint
				|| (body2->IsDynamic() && settings.mInvMassScale2 != 0.0f)))
		{
			// Add contact constraint in world space for the solver
			uint32 constraint_idx = mNumConstraints++;
			if (constraint_idx >= mMaxConstraints)
			{
				ioContactAllocator.mErrors |= EPhysicsUpdateError::ContactConstraintsFull;
				break;
			}

			// A constraint will be created
			outConstraintCreated = true;

			ContactConstraint &constraint = mConstraints[constraint_idx];
			new (&constraint) ContactConstraint();
			constraint.mBody1 = body1;
			constraint.mBody2 = body2;
			constraint.mSortKey = input_hash;
			world_space_normal.StoreFloat3(&constraint.mWorldSpaceNormal);
			constraint.mCombinedFriction = settings.mCombinedFriction;
			constraint.mInvMass1 = body1->GetMotionPropertiesUnchecked() != nullptr? settings.mInvMassScale1 * body1->GetMotionPropertiesUnchecked()->GetInverseMassUnchecked() : 0.0f;
			constraint.mInvInertiaScale1 = settings.mInvInertiaScale1;
			constraint.mInvMass2 = body2->GetMotionPropertiesUnchecked() != nullptr? settings.mInvMassScale2 * body2->GetMotionPropertiesUnchecked()->GetInverseMassUnchecked() : 0.0f;
			constraint.mInvInertiaScale2 = settings.mInvInertiaScale2;
			constraint.mContactPoints.resize(output_cm->mNumContactPoints);
			for (uint32 i = 0; i < output_cm->mNumContactPoints; ++i)
			{
				CachedContactPoint &ccp = output_cm->mContactPoints[i];
				WorldContactPoint &wcp = constraint.mContactPoints[i];
				wcp.mNonPenetrationConstraint.SetTotalLambda(ccp.mNonPenetrationLambda);
				wcp.mFrictionConstraint1.SetTotalLambda(ccp.mFrictionLambda[0]);
				wcp.mFrictionConstraint2.SetTotalLambda(ccp.mFrictionLambda[1]);
				wcp.mContactPoint = &ccp;
			}

			MOSS_DET_LOG("GetContactsFromCache: id1: " << constraint.mBody1->GetID() << " id2: " << constraint.mBody2->GetID() << " key: " << constraint.mSortKey);

			// Calculate friction and non-penetration constraint properties for all contact points
			CalculateFrictionAndNonPenetrationConstraintProperties(constraint, settings, delta_time, gravity_dt, transform_body1, transform_body2, *body1, *body2);

			// Notify island builder
			mUpdateContext->mIslandBuilder->LinkContact(constraint_idx, body1->GetIndexInActiveBodiesInternal(), body2->GetIndexInActiveBodiesInternal());

		#ifndef MOSS_DEBUG_RENDERER
			// Draw the manifold
			if (sDrawContactManifolds)
				constraint.Draw(DebugRenderer::sInstance, Color::sYellow);
		#endif // MOSS_DEBUG_RENDERER
		}

		// Mark contact as persisted so that we won't fire OnContactRemoved callbacks
		input_cm.mFlags |= (uint16)CachedManifold::EFlags::ContactPersisted;

		// Fetch the next manifold
		input_handle = input_cm.mNextWithSameBodyPair;
	}
	while (input_handle != ManifoldMap::cInvalidHandle);
	output_cbp->mFirstCachedManifold = output_handle;
}

ContactConstraintManager::BodyPairHandle ContactConstraintManager::AddBodyPair(ContactAllocator &ioContactAllocator, const Body &inBody1, const Body &inBody2)
{
	MOSS_PROFILE_FUNCTION();

	// Swap bodies so that body 1 id < body 2 id
	const Body *body1, *body2;
	if (inBody1.GetID() < inBody2.GetID())
	{
		body1 = &inBody1;
		body2 = &inBody2;
	}
	else
	{
		body1 = &inBody2;
		body2 = &inBody1;
	}

	// Add an entry
	BodyPair body_pair_key(body1->GetID(), body2->GetID());
	uint64 body_pair_hash = body_pair_key.GetHash();
	BPKeyValue *body_pair_kv = mCache[mCacheWriteIdx].Create(ioContactAllocator, body_pair_key, body_pair_hash);
	if (body_pair_kv == nullptr)
		return nullptr; // Out of cache space
	CachedBodyPair *cbp = &body_pair_kv->GetValue();
	cbp->mFirstCachedManifold = ManifoldMap::cInvalidHandle;

	// Get relative translation
	Quat inv_r1 = body1->GetRotation().Conjugated();
	Vec3 delta_position = inv_r1 * Vec3(body2->GetCenterOfMassPosition() - body1->GetCenterOfMassPosition());

	// Store it
	delta_position.StoreFloat3(&cbp->mDeltaPosition);

	// Determine relative orientation
	Quat delta_rotation = inv_r1 * body2->GetRotation();

	// Store it
	delta_rotation.StoreFloat3(&cbp->mDeltaRotation);

	return cbp;
}

template <EMotionType Type1, EMotionType Type2>
bool ContactConstraintManager::TemplatedAddContactConstraint(ContactAllocator &ioContactAllocator, BodyPairHandle inBodyPairHandle, Body &inBody1, Body &inBody2, const ContactManifold &inManifold)
{
	// Calculate hash
	SubShapeIDPair key { inBody1.GetID(), inManifold.mSubShapeID1, inBody2.GetID(), inManifold.mSubShapeID2 };
	uint64 key_hash = key.GetHash();

	// Determine number of contact points
	int num_contact_points = (int)inManifold.mRelativeContactPointsOn1.size();
	MOSS_ASSERT(num_contact_points <= MaxContactPoints);
	MOSS_ASSERT(num_contact_points == (int)inManifold.mRelativeContactPointsOn2.size());

	// Reserve space for new contact cache entry
	// Note that for dynamic vs dynamic we always require the first body to have a lower body id to get a consistent key
	// under which to look up the contact
	ManifoldCache &write_cache = mCache[mCacheWriteIdx];
	MKeyValue *new_manifold_kv = write_cache.Create(ioContactAllocator, key, key_hash, num_contact_points);
	if (new_manifold_kv == nullptr)
		return false; // Out of cache space
	CachedManifold *new_manifold = &new_manifold_kv->GetValue();

	// Transform the world space normal to the space of body 2 (this is usually the static body)
	RMat44 inverse_transform_body2 = inBody2.GetInverseCenterOfMassTransform();
	inverse_transform_body2.Multiply3x3(inManifold.mWorldSpaceNormal).Normalized().StoreFloat3(&new_manifold->mContactNormal);

	// Settings object that gets passed to the callback
	ContactSettings settings;
	settings.mCombinedFriction = mCombineFriction(inBody1, inManifold.mSubShapeID1, inBody2, inManifold.mSubShapeID2);
	settings.mCombinedRestitution = mCombineRestitution(inBody1, inManifold.mSubShapeID1, inBody2, inManifold.mSubShapeID2);
	settings.mIsSensor = inBody1.IsSensor() || inBody2.IsSensor();

	// Get the contact points for the old cache entry
	const ManifoldCache &read_cache = mCache[mCacheWriteIdx ^ 1];
	const MKeyValue *old_manifold_kv = read_cache.Find(key, key_hash);
	const CachedContactPoint *ccp_start;
	const CachedContactPoint *ccp_end;
	if (old_manifold_kv != nullptr)
	{
		// Call point persisted listener
		if (mContactListener != nullptr)
			mContactListener->OnContactPersisted(inBody1, inBody2, inManifold, settings);

		// Fetch the contact points from the old manifold
		const CachedManifold *old_manifold = &old_manifold_kv->GetValue();
		ccp_start = old_manifold->mContactPoints;
		ccp_end = ccp_start + old_manifold->mNumContactPoints;

		// Mark contact as persisted so that we won't fire OnContactRemoved callbacks
		old_manifold->mFlags |= (uint16)CachedManifold::EFlags::ContactPersisted;
	}
	else
	{
		// Call point added listener
		if (mContactListener != nullptr)
			mContactListener->OnContactAdded(inBody1, inBody2, inManifold, settings);

		// No contact points available from old manifold
		ccp_start = nullptr;
		ccp_end = nullptr;
	}

	// Get inverse transform for body 1
	RMat44 inverse_transform_body1 = inBody1.GetInverseCenterOfMassTransform();

	bool contact_constraint_created = false;

	// If one of the bodies is a sensor, don't actually create the constraint
	MOSS_ASSERT(settings.mIsSensor || !(inBody1.IsSensor() || inBody2.IsSensor()), "Sensors cannot be converted into regular bodies by a contact callback!");
	if (!settings.mIsSensor
		&& ((inBody1.IsDynamic() && settings.mInvMassScale1 != 0.0f) // One of the bodies must have mass to be able to create a contact constraint
			|| (inBody2.IsDynamic() && settings.mInvMassScale2 != 0.0f)))
	{
		// Add contact constraint
		uint32 constraint_idx = mNumConstraints++;
		if (constraint_idx >= mMaxConstraints)
		{
			ioContactAllocator.mErrors |= EPhysicsUpdateError::ContactConstraintsFull;

			// Manifold has been created already, we're not filling it in, so we need to reset the contact number of points.
			// Note that we don't hook it up to the body pair cache so that it won't be used as a cache during the next simulation.
			new_manifold->mNumContactPoints = 0;
			return false;
		}

		// We will create a contact constraint
		contact_constraint_created = true;

		ContactConstraint &constraint = mConstraints[constraint_idx];
		new (&constraint) ContactConstraint();
		constraint.mBody1 = &inBody1;
		constraint.mBody2 = &inBody2;
		constraint.mSortKey = key_hash;
		inManifold.mWorldSpaceNormal.StoreFloat3(&constraint.mWorldSpaceNormal);
		constraint.mCombinedFriction = settings.mCombinedFriction;
		constraint.mInvMass1 = inBody1.GetMotionPropertiesUnchecked() != nullptr? settings.mInvMassScale1 * inBody1.GetMotionPropertiesUnchecked()->GetInverseMassUnchecked() : 0.0f;
		constraint.mInvInertiaScale1 = settings.mInvInertiaScale1;
		constraint.mInvMass2 = inBody2.GetMotionPropertiesUnchecked() != nullptr? settings.mInvMassScale2 * inBody2.GetMotionPropertiesUnchecked()->GetInverseMassUnchecked() : 0.0f;
		constraint.mInvInertiaScale2 = settings.mInvInertiaScale2;

		MOSS_DET_LOG("TemplatedAddContactConstraint: id1: " << constraint.mBody1->GetID() << " id2: " << constraint.mBody2->GetID() << " key: " << constraint.mSortKey);

		// Notify island builder
		mUpdateContext->mIslandBuilder->LinkContact(constraint_idx, inBody1.GetIndexInActiveBodiesInternal(), inBody2.GetIndexInActiveBodiesInternal());

		// Get time step
		float delta_time = mUpdateContext->mStepDeltaTime;

		// Calculate value for restitution correction
		float gravity_dt_dot_normal = inManifold.mWorldSpaceNormal.Dot(mUpdateContext->mPhysicsSystem->GetGravity() * delta_time);

		// Calculate scaled mass and inertia
		float inv_m1;
		Mat44 inv_i1;
		if constexpr (Type1 == EMotionType::Dynamic)
		{
			const MotionProperties *mp1 = inBody1.GetMotionPropertiesUnchecked();
			inv_m1 = settings.mInvMassScale1 * mp1->GetInverseMass();
			inv_i1 = settings.mInvInertiaScale1 * mp1->GetInverseInertiaForRotation(inverse_transform_body1.Transposed3x3());
		}
		else
		{
			inv_m1 = 0.0f;
			inv_i1 = Mat44::sZero();
		}

		float inv_m2;
		Mat44 inv_i2;
		if constexpr (Type2 == EMotionType::Dynamic)
		{
			const MotionProperties *mp2 = inBody2.GetMotionPropertiesUnchecked();
			inv_m2 = settings.mInvMassScale2 * mp2->GetInverseMass();
			inv_i2 = settings.mInvInertiaScale2 * mp2->GetInverseInertiaForRotation(inverse_transform_body2.Transposed3x3());
		}
		else
		{
			inv_m2 = 0.0f;
			inv_i2 = Mat44::sZero();
		}

		// Calculate tangents
		Vec3 t1, t2;
		constraint.GetTangents(t1, t2);

		constraint.mContactPoints.resize(num_contact_points);
		for (int i = 0; i < num_contact_points; ++i)
		{
			// Convert to world space and set positions
			WorldContactPoint &wcp = constraint.mContactPoints[i];
			RVec3 p1_ws = inManifold.mBaseOffset + inManifold.mRelativeContactPointsOn1[i];
			RVec3 p2_ws = inManifold.mBaseOffset + inManifold.mRelativeContactPointsOn2[i];

			// Convert to local space to the body
			Vec3 p1_ls = Vec3(inverse_transform_body1 * p1_ws);
			Vec3 p2_ls = Vec3(inverse_transform_body2 * p2_ws);

			// Check if we have a close contact point from last update
			bool lambda_set = false;
			for (const CachedContactPoint *ccp = ccp_start; ccp < ccp_end; ccp++)
				if (Vec3::sLoadFloat3Unsafe(ccp->mPosition1).IsClose(p1_ls, mPhysicsSettings.mContactPointPreserveLambdaMaxDistSq)
					&& Vec3::sLoadFloat3Unsafe(ccp->mPosition2).IsClose(p2_ls, mPhysicsSettings.mContactPointPreserveLambdaMaxDistSq))
				{
					// Get lambdas from previous frame
					wcp.mNonPenetrationConstraint.SetTotalLambda(ccp->mNonPenetrationLambda);
					wcp.mFrictionConstraint1.SetTotalLambda(ccp->mFrictionLambda[0]);
					wcp.mFrictionConstraint2.SetTotalLambda(ccp->mFrictionLambda[1]);
					lambda_set = true;
					break;
				}
			if (!lambda_set)
			{
				wcp.mNonPenetrationConstraint.SetTotalLambda(0.0f);
				wcp.mFrictionConstraint1.SetTotalLambda(0.0f);
				wcp.mFrictionConstraint2.SetTotalLambda(0.0f);
			}

			// Create new contact point
			CachedContactPoint &cp = new_manifold->mContactPoints[i];
			p1_ls.StoreFloat3(&cp.mPosition1);
			p2_ls.StoreFloat3(&cp.mPosition2);
			wcp.mContactPoint = &cp;

			// Setup velocity constraint
			wcp.TemplatedCalculateFrictionAndNonPenetrationConstraintProperties<Type1, Type2>(delta_time, gravity_dt_dot_normal, inBody1, inBody2, inv_m1, inv_m2, inv_i1, inv_i2, p1_ws, p2_ws, inManifold.mWorldSpaceNormal, t1, t2, settings, mPhysicsSettings.mMinVelocityForRestitution);
		}

	#ifndef MOSS_DEBUG_RENDERER
		// Draw the manifold
		if (sDrawContactManifolds)
			constraint.Draw(DebugRenderer::sInstance, Color::sOrange);
	#endif // MOSS_DEBUG_RENDERER
	}
	else
	{
		// Store the contact manifold in the cache
		for (int i = 0; i < num_contact_points; ++i)
		{
			// Convert to local space to the body
			Vec3 p1 = Vec3(inverse_transform_body1 * (inManifold.mBaseOffset + inManifold.mRelativeContactPointsOn1[i]));
			Vec3 p2 = Vec3(inverse_transform_body2 * (inManifold.mBaseOffset + inManifold.mRelativeContactPointsOn2[i]));

			// Create new contact point
			CachedContactPoint &cp = new_manifold->mContactPoints[i];
			p1.StoreFloat3(&cp.mPosition1);
			p2.StoreFloat3(&cp.mPosition2);

			// Reset contact impulses, we haven't applied any
			cp.mNonPenetrationLambda = 0.0f;
			cp.mFrictionLambda[0] = 0.0f;
			cp.mFrictionLambda[1] = 0.0f;
		}
	}

	// Store cached contact point in body pair cache
	CachedBodyPair *cbp = reinterpret_cast<CachedBodyPair *>(inBodyPairHandle);
	new_manifold->mNextWithSameBodyPair = cbp->mFirstCachedManifold;
	cbp->mFirstCachedManifold = write_cache.ToHandle(new_manifold_kv);

	// A contact constraint was added
	return contact_constraint_created;
}

bool ContactConstraintManager::AddContactConstraint(ContactAllocator &ioContactAllocator, BodyPairHandle inBodyPairHandle, Body &inBody1, Body &inBody2, const ContactManifold &inManifold)
{
	MOSS_PROFILE_FUNCTION();

	MOSS_DET_LOG("AddContactConstraint: id1: " << inBody1.GetID() << " id2: " << inBody2.GetID()
		<< " subshape1: " << inManifold.mSubShapeID1 << " subshape2: " << inManifold.mSubShapeID2
		<< " normal: " << inManifold.mWorldSpaceNormal << " pendepth: " << inManifold.mPenetrationDepth);

	MOSS_ASSERT(inManifold.mWorldSpaceNormal.IsNormalized());

	// Swap bodies so that body 1 id < body 2 id
	const ContactManifold *manifold;
	Body *body1, *body2;
	ContactManifold temp;
	if (inBody2.GetID() < inBody1.GetID())
	{
		body1 = &inBody2;
		body2 = &inBody1;
		temp = inManifold.SwapShapes();
		manifold = &temp;
	}
	else
	{
		body1 = &inBody1;
		body2 = &inBody2;
		manifold = &inManifold;
	}

	// Dispatch to the correct templated form
	// Note: Non-dynamic vs non-dynamic can happen in this case due to one body being a sensor, so we need to have an extended switch case here
	switch (body1->GetMotionType())
	{
	case EMotionType::Dynamic:
		{
			switch (body2->GetMotionType())
			{
			case EMotionType::Dynamic:
				return TemplatedAddContactConstraint<EMotionType::Dynamic, EMotionType::Dynamic>(ioContactAllocator, inBodyPairHandle, *body1, *body2, *manifold);

			case EMotionType::Kinematic:
				return TemplatedAddContactConstraint<EMotionType::Dynamic, EMotionType::Kinematic>(ioContactAllocator, inBodyPairHandle, *body1, *body2, *manifold);

			case EMotionType::Static:
				return TemplatedAddContactConstraint<EMotionType::Dynamic, EMotionType::Static>(ioContactAllocator, inBodyPairHandle, *body1, *body2, *manifold);

			default:
				MOSS_ASSERT(false);
				break;
			}
			break;
		}

	case EMotionType::Kinematic:
		switch (body2->GetMotionType())
		{
		case EMotionType::Dynamic:
			return TemplatedAddContactConstraint<EMotionType::Kinematic, EMotionType::Dynamic>(ioContactAllocator, inBodyPairHandle, *body1, *body2, *manifold);

		case EMotionType::Kinematic:
			return TemplatedAddContactConstraint<EMotionType::Kinematic, EMotionType::Kinematic>(ioContactAllocator, inBodyPairHandle, *body1, *body2, *manifold);

		case EMotionType::Static:
			return TemplatedAddContactConstraint<EMotionType::Kinematic, EMotionType::Static>(ioContactAllocator, inBodyPairHandle, *body1, *body2, *manifold);

		default:
			MOSS_ASSERT(false);
			break;
		}
		break;

	case EMotionType::Static:
		switch (body2->GetMotionType())
		{
		case EMotionType::Dynamic:
			return TemplatedAddContactConstraint<EMotionType::Static, EMotionType::Dynamic>(ioContactAllocator, inBodyPairHandle, *body1, *body2, *manifold);

		case EMotionType::Kinematic:
			return TemplatedAddContactConstraint<EMotionType::Static, EMotionType::Kinematic>(ioContactAllocator, inBodyPairHandle, *body1, *body2, *manifold);

		case EMotionType::Static: // Static vs static not possible
		default:
			MOSS_ASSERT(false);
			break;
		}
		break;

	default:
		MOSS_ASSERT(false);
		break;
	}

	return false;
}

void ContactConstraintManager::OnCCDContactAdded(ContactAllocator &ioContactAllocator, const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &outSettings)
{
	MOSS_ASSERT(inManifold.mWorldSpaceNormal.IsNormalized());

	// Calculate contact settings
	outSettings.mCombinedFriction = mCombineFriction(inBody1, inManifold.mSubShapeID1, inBody2, inManifold.mSubShapeID2);
	outSettings.mCombinedRestitution = mCombineRestitution(inBody1, inManifold.mSubShapeID1, inBody2, inManifold.mSubShapeID2);
	outSettings.mIsSensor = false; // For now, no sensors are supported during CCD

	// The remainder of this function only deals with calling contact callbacks, if there's no contact callback we also don't need to do this work
	if (mContactListener != nullptr)
	{
		// Swap bodies so that body 1 id < body 2 id
		const ContactManifold *manifold;
		const Body *body1, *body2;
		ContactManifold temp;
		if (inBody2.GetID() < inBody1.GetID())
		{
			body1 = &inBody2;
			body2 = &inBody1;
			temp = inManifold.SwapShapes();
			manifold = &temp;
		}
		else
		{
			body1 = &inBody1;
			body2 = &inBody2;
			manifold = &inManifold;
		}

		// Calculate hash
		SubShapeIDPair key { body1->GetID(), manifold->mSubShapeID1, body2->GetID(), manifold->mSubShapeID2 };
		uint64 key_hash = key.GetHash();

		// Check if we already created this contact this physics update
		ManifoldCache &write_cache = mCache[mCacheWriteIdx];
		MKVAndCreated new_manifold_kv = write_cache.FindOrCreate(ioContactAllocator, key, key_hash, 0);
		if (new_manifold_kv.second)
		{
			// This contact is new for this physics update, check if previous update we already had this contact.
			const ManifoldCache &read_cache = mCache[mCacheWriteIdx ^ 1];
			const MKeyValue *old_manifold_kv = read_cache.Find(key, key_hash);
			if (old_manifold_kv == nullptr)
			{
				// New contact
				mContactListener->OnContactAdded(*body1, *body2, *manifold, outSettings);
			}
			else
			{
				// Existing contact
				mContactListener->OnContactPersisted(*body1, *body2, *manifold, outSettings);

				// Mark contact as persisted so that we won't fire OnContactRemoved callbacks
				old_manifold_kv->GetValue().mFlags |= (uint16)CachedManifold::EFlags::ContactPersisted;
			}

			// Check if the cache is full
			if (new_manifold_kv.first != nullptr)
			{
				// We don't store any contact points in this manifold as it is not for caching impulses, we only need to know that the contact was created
				CachedManifold &new_manifold = new_manifold_kv.first->GetValue();
				new_manifold.mContactNormal = { 0, 0, 0 };
				new_manifold.mFlags |= (uint16)CachedManifold::EFlags::CCDContact;
			}
		}
		else
		{
			// Already found this contact this physics update.
			// Note that we can trigger OnContactPersisted multiple times per physics update, but otherwise we have no way of obtaining the settings
			mContactListener->OnContactPersisted(*body1, *body2, *manifold, outSettings);
		}

		// If we swapped body1 and body2 we need to swap the mass scales back
		if (manifold == &temp)
		{
			std::swap(outSettings.mInvMassScale1, outSettings.mInvMassScale2);
			std::swap(outSettings.mInvInertiaScale1, outSettings.mInvInertiaScale2);
			// Note we do not need to negate the relative surface velocity as it is not applied by the CCD collision constraint
		}
	}

	MOSS_ASSERT(outSettings.mIsSensor || !(inBody1.IsSensor() || inBody2.IsSensor()), "Sensors cannot be converted into regular bodies by a contact callback!");
}

void ContactConstraintManager::SortContacts(uint32 *inConstraintIdxBegin, uint32 *inConstraintIdxEnd) const
{
	MOSS_PROFILE_FUNCTION();

	QuickSort(inConstraintIdxBegin, inConstraintIdxEnd, [this](uint32 inLHS, uint32 inRHS) {
		const ContactConstraint &lhs = mConstraints[inLHS];
		const ContactConstraint &rhs = mConstraints[inRHS];

		// Most of the time the sort key will be different so we sort on that
		if (lhs.mSortKey != rhs.mSortKey)
			return lhs.mSortKey < rhs.mSortKey;

		// If they're equal we use the IDs of body 1 to order
		if (lhs.mBody1 != rhs.mBody1)
			return lhs.mBody1->GetID() < rhs.mBody1->GetID();

		// If they're still equal we use the IDs of body 2 to order
		if (lhs.mBody2 != rhs.mBody2)
			return lhs.mBody2->GetID() < rhs.mBody2->GetID();

		MOSS_ASSERT(inLHS == inRHS, "Hash collision, ordering will be inconsistent");
		return false;
	});
}

void ContactConstraintManager::FinalizeContactCacheAndCallContactPointRemovedCallbacks(uint inExpectedNumBodyPairs, uint inExpectedNumManifolds)
{
	MOSS_PROFILE_FUNCTION();

#ifdef MOSS_DEBUG
	// Mark cache as finalized
	ManifoldCache &old_write_cache = mCache[mCacheWriteIdx];
	old_write_cache.Finalize();

	// Check that the count of body pairs and manifolds that we tracked outside of the cache (to avoid contention on an atomic) is correct
	MOSS_ASSERT(old_write_cache.GetNumBodyPairs() == inExpectedNumBodyPairs);
	MOSS_ASSERT(old_write_cache.GetNumManifolds() == inExpectedNumManifolds);
#endif

	// Buffers are now complete, make write buffer the read buffer
	mCacheWriteIdx ^= 1;

	// Get the old read cache / new write cache
	ManifoldCache &old_read_cache = mCache[mCacheWriteIdx];

	// Call the contact point removal callbacks
	if (mContactListener != nullptr)
		old_read_cache.ContactPointRemovedCallbacks(mContactListener);

	// We're done with the old read cache now
	old_read_cache.Clear();

	// Use the amount of contacts from the last iteration to determine the amount of buckets to use in the hash map for the next iteration
	old_read_cache.Prepare(inExpectedNumBodyPairs, inExpectedNumManifolds);
}

bool ContactConstraintManager::WereBodiesInContact(const BodyID &inBody1ID, const BodyID &inBody2ID) const
{
	// The body pair needs to be in the cache and it needs to have a manifold (otherwise it's just a record indicating that there are no collisions)
	const ManifoldCache &read_cache = mCache[mCacheWriteIdx ^ 1];
	BodyPair key;
	if (inBody1ID < inBody2ID)
		key = BodyPair(inBody1ID, inBody2ID);
	else
		key = BodyPair(inBody2ID, inBody1ID);
	uint64 key_hash = key.GetHash();
	const BPKeyValue *kv = read_cache.Find(key, key_hash);
	return kv != nullptr && kv->GetValue().mFirstCachedManifold != ManifoldMap::cInvalidHandle;
}

template <EMotionType Type1, EMotionType Type2>
MOSS_INLINE void ContactConstraintManager::sWarmStartConstraint(ContactConstraint &ioConstraint, MotionProperties *ioMotionProperties1, MotionProperties *ioMotionProperties2, float inWarmStartImpulseRatio)
{
	// Calculate tangents
	Vec3 t1, t2;
	ioConstraint.GetTangents(t1, t2);

	Vec3 ws_normal = ioConstraint.GetWorldSpaceNormal();

	for (WorldContactPoint &wcp : ioConstraint.mContactPoints)
	{
		// Warm starting: Apply impulse from last frame
		if (wcp.mFrictionConstraint1.IsActive() || wcp.mFrictionConstraint2.IsActive())
		{
			wcp.mFrictionConstraint1.TemplatedWarmStart<Type1, Type2>(ioMotionProperties1, ioConstraint.mInvMass1, ioMotionProperties2, ioConstraint.mInvMass2, t1, inWarmStartImpulseRatio);
			wcp.mFrictionConstraint2.TemplatedWarmStart<Type1, Type2>(ioMotionProperties1, ioConstraint.mInvMass1, ioMotionProperties2, ioConstraint.mInvMass2, t2, inWarmStartImpulseRatio);
		}
		wcp.mNonPenetrationConstraint.TemplatedWarmStart<Type1, Type2>(ioMotionProperties1, ioConstraint.mInvMass1, ioMotionProperties2, ioConstraint.mInvMass2, ws_normal, inWarmStartImpulseRatio);
	}
}

template <class MotionPropertiesCallback>
void ContactConstraintManager::WarmStartVelocityConstraints(const uint32 *inConstraintIdxBegin, const uint32 *inConstraintIdxEnd, float inWarmStartImpulseRatio, MotionPropertiesCallback &ioCallback)
{
	MOSS_PROFILE_FUNCTION();

	for (const uint32 *constraint_idx = inConstraintIdxBegin; constraint_idx < inConstraintIdxEnd; ++constraint_idx)
	{
		ContactConstraint &constraint = mConstraints[*constraint_idx];

		// Fetch bodies
		Body &body1 = *constraint.mBody1;
		EMotionType motion_type1 = body1.GetMotionType();
		MotionProperties *motion_properties1 = body1.GetMotionPropertiesUnchecked();

		Body &body2 = *constraint.mBody2;
		EMotionType motion_type2 = body2.GetMotionType();
		MotionProperties *motion_properties2 = body2.GetMotionPropertiesUnchecked();

		// Dispatch to the correct templated form
		// Note: Warm starting doesn't differentiate between kinematic/static bodies so we handle both as static bodies
		if (motion_type1 == EMotionType::Dynamic)
		{
			if (motion_type2 == EMotionType::Dynamic)
			{
				sWarmStartConstraint<EMotionType::Dynamic, EMotionType::Dynamic>(constraint, motion_properties1, motion_properties2, inWarmStartImpulseRatio);

				ioCallback(motion_properties2);
			}
			else
				sWarmStartConstraint<EMotionType::Dynamic, EMotionType::Static>(constraint, motion_properties1, motion_properties2, inWarmStartImpulseRatio);

			ioCallback(motion_properties1);
		}
		else
		{
			MOSS_ASSERT(motion_type2 == EMotionType::Dynamic);

			sWarmStartConstraint<EMotionType::Static, EMotionType::Dynamic>(constraint, motion_properties1, motion_properties2, inWarmStartImpulseRatio);

			ioCallback(motion_properties2);
		}
	}
}

// Specialize for the two body callback types
template void ContactConstraintManager::WarmStartVelocityConstraints<CalculateSolverSteps>(const uint32 *inConstraintIdxBegin, const uint32 *inConstraintIdxEnd, float inWarmStartImpulseRatio, CalculateSolverSteps &ioCallback);
template void ContactConstraintManager::WarmStartVelocityConstraints<DummyCalculateSolverSteps>(const uint32 *inConstraintIdxBegin, const uint32 *inConstraintIdxEnd, float inWarmStartImpulseRatio, DummyCalculateSolverSteps &ioCallback);

template <EMotionType Type1, EMotionType Type2>
MOSS_INLINE bool ContactConstraintManager::sSolveVelocityConstraint(ContactConstraint &ioConstraint, MotionProperties *ioMotionProperties1, MotionProperties *ioMotionProperties2)
{
	bool any_impulse_applied = false;

	// Calculate tangents
	Vec3 t1, t2;
	ioConstraint.GetTangents(t1, t2);

	// First apply all friction constraints (non-penetration is more important than friction)
	for (WorldContactPoint &wcp : ioConstraint.mContactPoints)
	{
		// Check if friction is enabled
		if (wcp.mFrictionConstraint1.IsActive() || wcp.mFrictionConstraint2.IsActive())
		{
			// Calculate impulse to stop motion in tangential direction
			float lambda1 = wcp.mFrictionConstraint1.TemplatedSolveVelocityConstraintGetTotalLambda<Type1, Type2>(ioMotionProperties1, ioMotionProperties2, t1);
			float lambda2 = wcp.mFrictionConstraint2.TemplatedSolveVelocityConstraintGetTotalLambda<Type1, Type2>(ioMotionProperties1, ioMotionProperties2, t2);
			float total_lambda_sq = Square(lambda1) + Square(lambda2);

			// Calculate max impulse that can be applied. Note that we're using the non-penetration impulse from the previous iteration here.
			// We do this because non-penetration is more important so is solved last (the last things that are solved in an iterative solver
			// contribute the most).
			float max_lambda_f = ioConstraint.mCombinedFriction * wcp.mNonPenetrationConstraint.GetTotalLambda();

			// If the total lambda that we will apply is too large, scale it back
			if (total_lambda_sq > Square(max_lambda_f))
			{
				float scale = max_lambda_f / sqrt(total_lambda_sq);
				lambda1 *= scale;
				lambda2 *= scale;
			}

			// Apply the friction impulse
			if (wcp.mFrictionConstraint1.TemplatedSolveVelocityConstraintApplyLambda<Type1, Type2>(ioMotionProperties1, ioConstraint.mInvMass1, ioMotionProperties2, ioConstraint.mInvMass2, t1, lambda1))
				any_impulse_applied = true;
			if (wcp.mFrictionConstraint2.TemplatedSolveVelocityConstraintApplyLambda<Type1, Type2>(ioMotionProperties1, ioConstraint.mInvMass1, ioMotionProperties2, ioConstraint.mInvMass2, t2, lambda2))
				any_impulse_applied = true;
		}
	}

	Vec3 ws_normal = ioConstraint.GetWorldSpaceNormal();

	// Then apply all non-penetration constraints
	for (WorldContactPoint &wcp : ioConstraint.mContactPoints)
	{
		// Solve non penetration velocities
		if (wcp.mNonPenetrationConstraint.TemplatedSolveVelocityConstraint<Type1, Type2>(ioMotionProperties1, ioConstraint.mInvMass1, ioMotionProperties2, ioConstraint.mInvMass2, ws_normal, 0.0f, FLT_MAX))
			any_impulse_applied = true;
	}

	return any_impulse_applied;
}

bool ContactConstraintManager::SolveVelocityConstraints(const uint32 *inConstraintIdxBegin, const uint32 *inConstraintIdxEnd)
{
	MOSS_PROFILE_FUNCTION();

	bool any_impulse_applied = false;

	for (const uint32 *constraint_idx = inConstraintIdxBegin; constraint_idx < inConstraintIdxEnd; ++constraint_idx)
	{
		ContactConstraint &constraint = mConstraints[*constraint_idx];

		// Fetch bodies
		Body &body1 = *constraint.mBody1;
		EMotionType motion_type1 = body1.GetMotionType();
		MotionProperties *motion_properties1 = body1.GetMotionPropertiesUnchecked();

		Body &body2 = *constraint.mBody2;
		EMotionType motion_type2 = body2.GetMotionType();
		MotionProperties *motion_properties2 = body2.GetMotionPropertiesUnchecked();

		// Dispatch to the correct templated form
		switch (motion_type1)
		{
		case EMotionType::Dynamic:
			switch (motion_type2)
			{
			case EMotionType::Dynamic:
				any_impulse_applied |= sSolveVelocityConstraint<EMotionType::Dynamic, EMotionType::Dynamic>(constraint, motion_properties1, motion_properties2);
				break;

			case EMotionType::Kinematic:
				any_impulse_applied |= sSolveVelocityConstraint<EMotionType::Dynamic, EMotionType::Kinematic>(constraint, motion_properties1, motion_properties2);
				break;

			case EMotionType::Static:
				any_impulse_applied |= sSolveVelocityConstraint<EMotionType::Dynamic, EMotionType::Static>(constraint, motion_properties1, motion_properties2);
				break;

			default:
				MOSS_ASSERT(false);
				break;
			}
			break;

		case EMotionType::Kinematic:
			MOSS_ASSERT(motion_type2 == EMotionType::Dynamic);
			any_impulse_applied |= sSolveVelocityConstraint<EMotionType::Kinematic, EMotionType::Dynamic>(constraint, motion_properties1, motion_properties2);
			break;

		case EMotionType::Static:
			MOSS_ASSERT(motion_type2 == EMotionType::Dynamic);
			any_impulse_applied |= sSolveVelocityConstraint<EMotionType::Static, EMotionType::Dynamic>(constraint, motion_properties1, motion_properties2);
			break;

		default:
			MOSS_ASSERT(false);
			break;
		}
	}

	return any_impulse_applied;
}

void ContactConstraintManager::StoreAppliedImpulses(const uint32 *inConstraintIdxBegin, const uint32 *inConstraintIdxEnd) const
{
	// Copy back total applied impulse to cache for the next frame
	for (const uint32 *constraint_idx = inConstraintIdxBegin; constraint_idx < inConstraintIdxEnd; ++constraint_idx)
	{
		const ContactConstraint &constraint = mConstraints[*constraint_idx];

		for (const WorldContactPoint &wcp : constraint.mContactPoints)
		{
			wcp.mContactPoint->mNonPenetrationLambda = wcp.mNonPenetrationConstraint.GetTotalLambda();
			wcp.mContactPoint->mFrictionLambda[0] = wcp.mFrictionConstraint1.GetTotalLambda();
			wcp.mContactPoint->mFrictionLambda[1] = wcp.mFrictionConstraint2.GetTotalLambda();
		}
	}
}

bool ContactConstraintManager::SolvePositionConstraints(const uint32 *inConstraintIdxBegin, const uint32 *inConstraintIdxEnd)
{
	MOSS_PROFILE_FUNCTION();

	bool any_impulse_applied = false;

	for (const uint32 *constraint_idx = inConstraintIdxBegin; constraint_idx < inConstraintIdxEnd; ++constraint_idx)
	{
		ContactConstraint &constraint = mConstraints[*constraint_idx];

		// Fetch bodies
		Body &body1 = *constraint.mBody1;
		Body &body2 = *constraint.mBody2;

		// Get transforms
		RMat44 transform1 = body1.GetCenterOfMassTransform();
		RMat44 transform2 = body2.GetCenterOfMassTransform();

		Vec3 ws_normal = constraint.GetWorldSpaceNormal();

		for (WorldContactPoint &wcp : constraint.mContactPoints)
		{
			// Calculate new contact point positions in world space (the bodies may have moved)
			RVec3 p1 = transform1 * Vec3::sLoadFloat3Unsafe(wcp.mContactPoint->mPosition1);
			RVec3 p2 = transform2 * Vec3::sLoadFloat3Unsafe(wcp.mContactPoint->mPosition2);

			// Calculate separation along the normal (negative if interpenetrating)
			// Allow a little penetration by default (PhysicsSettings::mPenetrationSlop) to avoid jittering between contact/no-contact which wipes out the contact cache and warm start impulses
			// Clamp penetration to a max PhysicsSettings::mMaxPenetrationDistance so that we don't apply a huge impulse if we're penetrating a lot
			float separation = max(Vec3(p2 - p1).Dot(ws_normal) + mPhysicsSettings.mPenetrationSlop, -mPhysicsSettings.mMaxPenetrationDistance);

			// Only enforce constraint when separation < 0 (otherwise we're apart)
			if (separation < 0.0f)
			{
				// Update constraint properties (bodies may have moved)
				wcp.CalculateNonPenetrationConstraintProperties(body1, constraint.mInvMass1, constraint.mInvInertiaScale1, body2, constraint.mInvMass2, constraint.mInvInertiaScale2, p1, p2, ws_normal);

				// Solve position errors
				if (wcp.mNonPenetrationConstraint.SolvePositionConstraintWithMassOverride(body1, constraint.mInvMass1, body2, constraint.mInvMass2, ws_normal, separation, mPhysicsSettings.mBaumgarte))
					any_impulse_applied = true;
			}
		}
	}

	return any_impulse_applied;
}

void ContactConstraintManager::RecycleConstraintBuffer()
{
	// Reset constraint array
	mNumConstraints = 0;
}

void ContactConstraintManager::FinishConstraintBuffer()
{
	// Free constraints buffer
	mUpdateContext->mTempAllocator->Free(mConstraints, mMaxConstraints * sizeof(ContactConstraint));
	mConstraints = nullptr;
	mNumConstraints = 0;

	// Reset update context
	mUpdateContext = nullptr;
}

void ContactConstraintManager::SaveState(StateRecorder &inStream, const StateRecorderFilter *inFilter) const
{
	mCache[mCacheWriteIdx ^ 1].SaveState(inStream, inFilter);
}

bool ContactConstraintManager::RestoreState(StateRecorder &inStream, const StateRecorderFilter *inFilter)
{
	bool success = mCache[mCacheWriteIdx].RestoreState(mCache[mCacheWriteIdx ^ 1], inStream, inFilter);

	// If this is the last part, the cache is finalized
	if (inStream.IsLastPart())
	{
		mCacheWriteIdx ^= 1;
		mCache[mCacheWriteIdx].Clear();
	}

	return success;
}

ConstraintSettings::ConstraintResult ConstraintSettings::sRestoreFromBinaryState(StreamIn &inStream)
{
	return StreamUtils::RestoreObject<ConstraintSettings>(inStream, &ConstraintSettings::RestoreBinaryState);
}

void Constraint::SaveState(StateRecorder &inStream) const
{
	inStream.Write(mEnabled);
}

void Constraint::RestoreState(StateRecorder &inStream)
{
	inStream.Read(mEnabled);
}

void Constraint::ToConstraintSettings(ConstraintSettings &outSettings) const
{
	outSettings.mEnabled = mEnabled;
	outSettings.mConstraintPriority = mConstraintPriority;
	outSettings.mNumVelocityStepsOverride = mNumVelocityStepsOverride;
	outSettings.mNumPositionStepsOverride = mNumPositionStepsOverride;
	outSettings.mUserData = mUserData;
#ifndef MOSS_DEBUG_RENDERER
	outSettings.mDrawConstraintSize = mDrawConstraintSize;
#endif // MOSS_DEBUG_RENDERER
}

TwoBodyConstraint *FixedConstraintSettings::Create(Body &inBody1, Body &inBody2) const
{
	return new FixedConstraint(inBody1, inBody2, *this);
}

FixedConstraint::FixedConstraint(Body &inBody1, Body &inBody2, const FixedConstraintSettings &inSettings) :
	TwoBodyConstraint(inBody1, inBody2, inSettings)
{
	// Store inverse of initial rotation from body 1 to body 2 in body 1 space
	mInvInitialOrientation = RotationEulerConstraintPart::sGetInvInitialOrientationXY(inSettings.mAxisX1, inSettings.mAxisY1, inSettings.mAxisX2, inSettings.mAxisY2);

	if (inSettings.mSpace == EConstraintSpace::WorldSpace)
	{
		if (inSettings.mAutoDetectPoint)
		{
			// Determine anchor point: If any of the bodies can never be dynamic use the other body as anchor point
			RVec3 anchor;
			if (!inBody1.CanBeKinematicOrDynamic())
				anchor = inBody2.GetCenterOfMassPosition();
			else if (!inBody2.CanBeKinematicOrDynamic())
				anchor = inBody1.GetCenterOfMassPosition();
			else
			{
				// Otherwise use weighted anchor point towards the lightest body
				Real inv_m1 = Real(inBody1.GetMotionPropertiesUnchecked()->GetInverseMassUnchecked());
				Real inv_m2 = Real(inBody2.GetMotionPropertiesUnchecked()->GetInverseMassUnchecked());
				Real total_inv_mass = inv_m1 + inv_m2;
				if (total_inv_mass != 0.0_r)
					anchor = (inv_m1 * inBody1.GetCenterOfMassPosition() + inv_m2 * inBody2.GetCenterOfMassPosition()) / (inv_m1 + inv_m2);
				else
					anchor = inBody1.GetCenterOfMassPosition();
			}

			// Store local positions
			mLocalSpacePosition1 = Vec3(inBody1.GetInverseCenterOfMassTransform() * anchor);
			mLocalSpacePosition2 = Vec3(inBody2.GetInverseCenterOfMassTransform() * anchor);
		}
		else
		{
			// Store local positions
			mLocalSpacePosition1 = Vec3(inBody1.GetInverseCenterOfMassTransform() * inSettings.mPoint1);
			mLocalSpacePosition2 = Vec3(inBody2.GetInverseCenterOfMassTransform() * inSettings.mPoint2);
		}

		// Constraints were specified in world space, so we should have replaced c1 with q10^-1 c1 and c2 with q20^-1 c2
		// => r0^-1 = (q20^-1 c2) (q10^-1 c1)^1 = q20^-1 (c2 c1^-1) q10
		mInvInitialOrientation = inBody2.GetRotation().Conjugated() * mInvInitialOrientation * inBody1.GetRotation();
	}
	else
	{
		// Store local positions
		mLocalSpacePosition1 = Vec3(inSettings.mPoint1);
		mLocalSpacePosition2 = Vec3(inSettings.mPoint2);
	}
}

void FixedConstraint::NotifyShapeChanged(const BodyID &inBodyID, Vec3Arg inDeltaCOM)
{
	if (mBody1->GetID() == inBodyID)
		mLocalSpacePosition1 -= inDeltaCOM;
	else if (mBody2->GetID() == inBodyID)
		mLocalSpacePosition2 -= inDeltaCOM;
}

void FixedConstraint::SetupVelocityConstraint(float inDeltaTime)
{
	// Calculate constraint values that don't change when the bodies don't change position
	Mat44 rotation1 = Mat44::sRotation(mBody1->GetRotation());
	Mat44 rotation2 = Mat44::sRotation(mBody2->GetRotation());
	mRotationConstraintPart.CalculateConstraintProperties(*mBody1, rotation1, *mBody2, rotation2);
	mPointConstraintPart.CalculateConstraintProperties(*mBody1, rotation1, mLocalSpacePosition1, *mBody2, rotation2, mLocalSpacePosition2);
}

void FixedConstraint::ResetWarmStart()
{
	mRotationConstraintPart.Deactivate();
	mPointConstraintPart.Deactivate();
}

void FixedConstraint::WarmStartVelocityConstraint(float inWarmStartImpulseRatio)
{
	// Warm starting: Apply previous frame impulse
	mRotationConstraintPart.WarmStart(*mBody1, *mBody2, inWarmStartImpulseRatio);
	mPointConstraintPart.WarmStart(*mBody1, *mBody2, inWarmStartImpulseRatio);
}

bool FixedConstraint::SolveVelocityConstraint(float inDeltaTime)
{
	// Solve rotation constraint
	bool rot = mRotationConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2);

	// Solve position constraint
	bool pos = mPointConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2);

	return rot || pos;
}

bool FixedConstraint::SolvePositionConstraint(float inDeltaTime, float inBaumgarte)
{
	// Solve rotation constraint
	mRotationConstraintPart.CalculateConstraintProperties(*mBody1, Mat44::sRotation(mBody1->GetRotation()), *mBody2, Mat44::sRotation(mBody2->GetRotation()));
	bool rot = mRotationConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, mInvInitialOrientation, inBaumgarte);

	// Solve position constraint
	mPointConstraintPart.CalculateConstraintProperties(*mBody1, Mat44::sRotation(mBody1->GetRotation()), mLocalSpacePosition1, *mBody2, Mat44::sRotation(mBody2->GetRotation()), mLocalSpacePosition2);
	bool pos = mPointConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, inBaumgarte);

	return rot || pos;
}

#ifndef MOSS_DEBUG_RENDERER
void FixedConstraint::DrawConstraint(DebugRenderer *inRenderer) const
{
	RMat44 com1 = mBody1->GetCenterOfMassTransform();
	RMat44 com2 = mBody2->GetCenterOfMassTransform();

	RVec3 anchor1 = com1 * mLocalSpacePosition1;
	RVec3 anchor2 = com2 * mLocalSpacePosition2;

	// Draw constraint
	inRenderer->DrawLine(com1.GetTranslation(), anchor1, Color::sGreen);
	inRenderer->DrawLine(com2.GetTranslation(), anchor2, Color::sBlue);
}
#endif // MOSS_DEBUG_RENDERER

void FixedConstraint::SaveState(StateRecorder &inStream) const
{
	TwoBodyConstraint::SaveState(inStream);

	mRotationConstraintPart.SaveState(inStream);
	mPointConstraintPart.SaveState(inStream);
}

void FixedConstraint::RestoreState(StateRecorder &inStream)
{
	TwoBodyConstraint::RestoreState(inStream);

	mRotationConstraintPart.RestoreState(inStream);
	mPointConstraintPart.RestoreState(inStream);
}

Ref<ConstraintSettings> FixedConstraint::GetConstraintSettings() const
{
	FixedConstraintSettings *settings = new FixedConstraintSettings;
	ToConstraintSettings(*settings);
	settings->mSpace = EConstraintSpace::LocalToBodyCOM;
	settings->mPoint1 = RVec3(mLocalSpacePosition1);
	settings->mAxisX1 = Vec3::sAxisX();
	settings->mAxisY1 = Vec3::sAxisY();
	settings->mPoint2 = RVec3(mLocalSpacePosition2);
	settings->mAxisX2 = mInvInitialOrientation.RotateAxisX();
	settings->mAxisY2 = mInvInitialOrientation.RotateAxisY();
	return settings;
}

TwoBodyConstraint *GearConstraintSettings::Create(Body &inBody1, Body &inBody2) const
{
	return new GearConstraint(inBody1, inBody2, *this);
}

GearConstraint::GearConstraint(Body &inBody1, Body &inBody2, const GearConstraintSettings &inSettings) :
	TwoBodyConstraint(inBody1, inBody2, inSettings),
	mLocalSpaceHingeAxis1(inSettings.mHingeAxis1),
	mLocalSpaceHingeAxis2(inSettings.mHingeAxis2),
	mRatio(inSettings.mRatio)
{
	if (inSettings.mSpace == EConstraintSpace::WorldSpace)
	{
		// If all properties were specified in world space, take them to local space now
		mLocalSpaceHingeAxis1 = inBody1.GetInverseCenterOfMassTransform().Multiply3x3(mLocalSpaceHingeAxis1).Normalized();
		mLocalSpaceHingeAxis2 = inBody2.GetInverseCenterOfMassTransform().Multiply3x3(mLocalSpaceHingeAxis2).Normalized();
	}
}

void GearConstraint::CalculateConstraintProperties(Mat44Arg inRotation1, Mat44Arg inRotation2)
{
	// Calculate world space normals
	mWorldSpaceHingeAxis1 = inRotation1 * mLocalSpaceHingeAxis1;
	mWorldSpaceHingeAxis2 = inRotation2 * mLocalSpaceHingeAxis2;

	mGearConstraintPart.CalculateConstraintProperties(*mBody1, mWorldSpaceHingeAxis1, *mBody2, mWorldSpaceHingeAxis2, mRatio);
}

void GearConstraint::SetupVelocityConstraint(float inDeltaTime)
{
	// Calculate constraint properties that are constant while bodies don't move
	Mat44 rotation1 = Mat44::sRotation(mBody1->GetRotation());
	Mat44 rotation2 = Mat44::sRotation(mBody2->GetRotation());
	CalculateConstraintProperties(rotation1, rotation2);
}

void GearConstraint::ResetWarmStart()
{
	mGearConstraintPart.Deactivate();
}

void GearConstraint::WarmStartVelocityConstraint(float inWarmStartImpulseRatio)
{
	// Warm starting: Apply previous frame impulse
	mGearConstraintPart.WarmStart(*mBody1, *mBody2, inWarmStartImpulseRatio);
}

bool GearConstraint::SolveVelocityConstraint(float inDeltaTime)
{
	return mGearConstraintPart.SolveVelocityConstraint(*mBody1, mWorldSpaceHingeAxis1, *mBody2, mWorldSpaceHingeAxis2, mRatio);
}

bool GearConstraint::SolvePositionConstraint(float inDeltaTime, float inBaumgarte)
{
	if (mGear1Constraint == nullptr || mGear2Constraint == nullptr)
		return false;

	float gear1rot;
	if (mGear1Constraint->GetSubType() == EConstraintSubType::Hinge)
	{
		gear1rot = StaticCast<HingeConstraint>(mGear1Constraint)->GetCurrentAngle();
	}
	else
	{
		MOSS_ASSERT(false, "Unsupported");
		return false;
	}

	float gear2rot;
	if (mGear2Constraint->GetSubType() == EConstraintSubType::Hinge)
	{
		gear2rot = StaticCast<HingeConstraint>(mGear2Constraint)->GetCurrentAngle();
	}
	else
	{
		MOSS_ASSERT(false, "Unsupported");
		return false;
	}

	float error = CenterAngleAroundZero(fmod(gear1rot + mRatio * gear2rot, 2.0f * MOSS_PI));
	if (error == 0.0f)
		return false;

	Mat44 rotation1 = Mat44::sRotation(mBody1->GetRotation());
	Mat44 rotation2 = Mat44::sRotation(mBody2->GetRotation());
	CalculateConstraintProperties(rotation1, rotation2);
	return mGearConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, error, inBaumgarte);
}

#ifndef MOSS_DEBUG_RENDERER
void GearConstraint::DrawConstraint(DebugRenderer *inRenderer) const
{
	RMat44 transform1 = mBody1->GetCenterOfMassTransform();
	RMat44 transform2 = mBody2->GetCenterOfMassTransform();

	// Draw constraint axis
	inRenderer->DrawArrow(transform1.GetTranslation(), transform1 * mLocalSpaceHingeAxis1, Color::sGreen, 0.01f);
	inRenderer->DrawArrow(transform2.GetTranslation(), transform2 * mLocalSpaceHingeAxis2, Color::sBlue, 0.01f);
}

#endif // MOSS_DEBUG_RENDERER

void GearConstraint::SaveState(StateRecorder &inStream) const
{
	TwoBodyConstraint::SaveState(inStream);

	mGearConstraintPart.SaveState(inStream);
}

void GearConstraint::RestoreState(StateRecorder &inStream)
{
	TwoBodyConstraint::RestoreState(inStream);

	mGearConstraintPart.RestoreState(inStream);
}

Ref<ConstraintSettings> GearConstraint::GetConstraintSettings() const
{
	GearConstraintSettings *settings = new GearConstraintSettings;
	ToConstraintSettings(*settings);
	settings->mSpace = EConstraintSpace::LocalToBodyCOM;
	settings->mHingeAxis1 = mLocalSpaceHingeAxis1;
	settings->mHingeAxis2 = mLocalSpaceHingeAxis2;
	settings->mRatio = mRatio;
	return settings;
}

Mat44 GearConstraint::GetConstraintToBody1Matrix() const
{
	Vec3 perp = mLocalSpaceHingeAxis1.GetNormalizedPerpendicular();
	return Mat44(Vec4(mLocalSpaceHingeAxis1, 0), Vec4(perp, 0), Vec4(mLocalSpaceHingeAxis1.Cross(perp), 0), Vec4(0, 0, 0, 1));
}

Mat44 GearConstraint::GetConstraintToBody2Matrix() const
{
	Vec3 perp = mLocalSpaceHingeAxis2.GetNormalizedPerpendicular();
	return Mat44(Vec4(mLocalSpaceHingeAxis2, 0), Vec4(perp, 0), Vec4(mLocalSpaceHingeAxis2.Cross(perp), 0), Vec4(0, 0, 0, 1));
}

TwoBodyConstraint *HingeConstraintSettings::Create(Body &inBody1, Body &inBody2) const
{
	return new HingeConstraint(inBody1, inBody2, *this);
}

HingeConstraint::HingeConstraint(Body &inBody1, Body &inBody2, const HingeConstraintSettings &inSettings) :
	TwoBodyConstraint(inBody1, inBody2, inSettings),
	mMaxFrictionTorque(inSettings.mMaxFrictionTorque),
	mMotorSettings(inSettings.mMotorSettings)
{
	// Store limits
	MOSS_ASSERT(inSettings.mLimitsMin != inSettings.mLimitsMax || inSettings.mLimitsSpringSettings.mFrequency > 0.0f, "Better use a fixed constraint in this case");
	SetLimits(inSettings.mLimitsMin, inSettings.mLimitsMax);

	// Store inverse of initial rotation from body 1 to body 2 in body 1 space
	mInvInitialOrientation = RotationEulerConstraintPart::sGetInvInitialOrientationXZ(inSettings.mNormalAxis1, inSettings.mHingeAxis1, inSettings.mNormalAxis2, inSettings.mHingeAxis2);

	if (inSettings.mSpace == EConstraintSpace::WorldSpace)
	{
		// If all properties were specified in world space, take them to local space now
		RMat44 inv_transform1 = inBody1.GetInverseCenterOfMassTransform();
		mLocalSpacePosition1 = Vec3(inv_transform1 * inSettings.mPoint1);
		mLocalSpaceHingeAxis1 = inv_transform1.Multiply3x3(inSettings.mHingeAxis1).Normalized();
		mLocalSpaceNormalAxis1 = inv_transform1.Multiply3x3(inSettings.mNormalAxis1).Normalized();

		RMat44 inv_transform2 = inBody2.GetInverseCenterOfMassTransform();
		mLocalSpacePosition2 = Vec3(inv_transform2 * inSettings.mPoint2);
		mLocalSpaceHingeAxis2 = inv_transform2.Multiply3x3(inSettings.mHingeAxis2).Normalized();
		mLocalSpaceNormalAxis2 = inv_transform2.Multiply3x3(inSettings.mNormalAxis2).Normalized();

		// Constraints were specified in world space, so we should have replaced c1 with q10^-1 c1 and c2 with q20^-1 c2
		// => r0^-1 = (q20^-1 c2) (q10^-1 c1)^1 = q20^-1 (c2 c1^-1) q10
		mInvInitialOrientation = inBody2.GetRotation().Conjugated() * mInvInitialOrientation * inBody1.GetRotation();
	}
	else
	{
		mLocalSpacePosition1 = Vec3(inSettings.mPoint1);
		mLocalSpaceHingeAxis1 = inSettings.mHingeAxis1;
		mLocalSpaceNormalAxis1 = inSettings.mNormalAxis1;

		mLocalSpacePosition2 = Vec3(inSettings.mPoint2);
		mLocalSpaceHingeAxis2 = inSettings.mHingeAxis2;
		mLocalSpaceNormalAxis2 = inSettings.mNormalAxis2;
	}

	// Store spring settings
	SetLimitsSpringSettings(inSettings.mLimitsSpringSettings);
}

void HingeConstraint::NotifyShapeChanged(const BodyID &inBodyID, Vec3Arg inDeltaCOM)
{
	if (mBody1->GetID() == inBodyID)
		mLocalSpacePosition1 -= inDeltaCOM;
	else if (mBody2->GetID() == inBodyID)
		mLocalSpacePosition2 -= inDeltaCOM;
}

float HingeConstraint::GetCurrentAngle() const
{
	// See: CalculateA1AndTheta
	Quat rotation1 = mBody1->GetRotation();
	Quat diff = mBody2->GetRotation() * mInvInitialOrientation * rotation1.Conjugated();
	return diff.GetRotationAngle(rotation1 * mLocalSpaceHingeAxis1);
}

void HingeConstraint::SetLimits(float inLimitsMin, float inLimitsMax)
{
	MOSS_ASSERT(inLimitsMin <= 0.0f && inLimitsMin >= -MOSS_PI);
	MOSS_ASSERT(inLimitsMax >= 0.0f && inLimitsMax <= MOSS_PI);
	mLimitsMin = inLimitsMin;
	mLimitsMax = inLimitsMax;
	mHasLimits = mLimitsMin > -MOSS_PI || mLimitsMax < MOSS_PI;
}

void HingeConstraint::CalculateA1AndTheta()
{
	if (mHasLimits || mMotorState != EMotorState::Off || mMaxFrictionTorque > 0.0f)
	{
		Quat rotation1 = mBody1->GetRotation();

		// Calculate relative rotation in world space
		//
		// The rest rotation is:
		//
		// q2 = q1 r0
		//
		// But the actual rotation is
		//
		// q2 = diff q1 r0
		// <=> diff = q2 r0^-1 q1^-1
		//
		// Where:
		// q1 = current rotation of body 1
		// q2 = current rotation of body 2
		// diff = relative rotation in world space
		Quat diff = mBody2->GetRotation() * mInvInitialOrientation * rotation1.Conjugated();

		// Calculate hinge axis in world space
		mA1 = rotation1 * mLocalSpaceHingeAxis1;

		// Get rotation angle around the hinge axis
		mTheta = diff.GetRotationAngle(mA1);
	}
}

void HingeConstraint::CalculateRotationLimitsConstraintProperties(float inDeltaTime)
{
	// Apply constraint if outside of limits
	if (mHasLimits && (mTheta <= mLimitsMin || mTheta >= mLimitsMax))
		mRotationLimitsConstraintPart.CalculateConstraintPropertiesWithSettings(inDeltaTime, *mBody1, *mBody2, mA1, 0.0f, GetSmallestAngleToLimit(), mLimitsSpringSettings);
	else
		mRotationLimitsConstraintPart.Deactivate();
}

void HingeConstraint::CalculateMotorConstraintProperties(float inDeltaTime)
{
	switch (mMotorState)
	{
	case EMotorState::Off:
		if (mMaxFrictionTorque > 0.0f)
			mMotorConstraintPart.CalculateConstraintProperties(*mBody1, *mBody2, mA1);
		else
			mMotorConstraintPart.Deactivate();
		break;

	case EMotorState::Velocity:
		mMotorConstraintPart.CalculateConstraintProperties(*mBody1, *mBody2, mA1, -mTargetAngularVelocity);
		break;

	case EMotorState::Position:
		if (mMotorSettings.mSpringSettings.HasStiffness())
			mMotorConstraintPart.CalculateConstraintPropertiesWithSettings(inDeltaTime, *mBody1, *mBody2, mA1, 0.0f, CenterAngleAroundZero(mTheta - mTargetAngle), mMotorSettings.mSpringSettings);
		else
			mMotorConstraintPart.Deactivate();
		break;
	}
}

void HingeConstraint::SetupVelocityConstraint(float inDeltaTime)
{
	// Cache constraint values that are valid until the bodies move
	Mat44 rotation1 = Mat44::sRotation(mBody1->GetRotation());
	Mat44 rotation2 = Mat44::sRotation(mBody2->GetRotation());
	mPointConstraintPart.CalculateConstraintProperties(*mBody1, rotation1, mLocalSpacePosition1, *mBody2, rotation2, mLocalSpacePosition2);
	mRotationConstraintPart.CalculateConstraintProperties(*mBody1, rotation1, rotation1.Multiply3x3(mLocalSpaceHingeAxis1), *mBody2, rotation2, rotation2.Multiply3x3(mLocalSpaceHingeAxis2));
	CalculateA1AndTheta();
	CalculateRotationLimitsConstraintProperties(inDeltaTime);
	CalculateMotorConstraintProperties(inDeltaTime);
}

void HingeConstraint::ResetWarmStart()
{
	mMotorConstraintPart.Deactivate();
	mPointConstraintPart.Deactivate();
	mRotationConstraintPart.Deactivate();
	mRotationLimitsConstraintPart.Deactivate();
}

void HingeConstraint::WarmStartVelocityConstraint(float inWarmStartImpulseRatio)
{
	// Warm starting: Apply previous frame impulse
	mMotorConstraintPart.WarmStart(*mBody1, *mBody2, inWarmStartImpulseRatio);
	mPointConstraintPart.WarmStart(*mBody1, *mBody2, inWarmStartImpulseRatio);
	mRotationConstraintPart.WarmStart(*mBody1, *mBody2, inWarmStartImpulseRatio);
	mRotationLimitsConstraintPart.WarmStart(*mBody1, *mBody2, inWarmStartImpulseRatio);
}

float HingeConstraint::GetSmallestAngleToLimit() const
{
	float dist_to_min = CenterAngleAroundZero(mTheta - mLimitsMin);
	float dist_to_max = CenterAngleAroundZero(mTheta - mLimitsMax);
	return abs(dist_to_min) < abs(dist_to_max)? dist_to_min : dist_to_max;
}

bool HingeConstraint::IsMinLimitClosest() const
{
	float dist_to_min = CenterAngleAroundZero(mTheta - mLimitsMin);
	float dist_to_max = CenterAngleAroundZero(mTheta - mLimitsMax);
	return abs(dist_to_min) < abs(dist_to_max);
}

bool HingeConstraint::SolveVelocityConstraint(float inDeltaTime)
{
	// Solve motor
	bool motor = false;
	if (mMotorConstraintPart.IsActive())
	{
		switch (mMotorState)
		{
		case EMotorState::Off:
			{
				float max_lambda = mMaxFrictionTorque * inDeltaTime;
				motor = mMotorConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2, mA1, -max_lambda, max_lambda);
				break;
			}

		case EMotorState::Velocity:
		case EMotorState::Position:
			motor = mMotorConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2, mA1, inDeltaTime * mMotorSettings.mMinTorqueLimit, inDeltaTime * mMotorSettings.mMaxTorqueLimit);
			break;
		}
	}

	// Solve point constraint
	bool pos = mPointConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2);

	// Solve rotation constraint
	bool rot = mRotationConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2);

	// Solve rotation limits
	bool limit = false;
	if (mRotationLimitsConstraintPart.IsActive())
	{
		float min_lambda, max_lambda;
		if (mLimitsMin == mLimitsMax)
		{
			min_lambda = -FLT_MAX;
			max_lambda = FLT_MAX;
		}
		else if (IsMinLimitClosest())
		{
			min_lambda = 0.0f;
			max_lambda = FLT_MAX;
		}
		else
		{
			min_lambda = -FLT_MAX;
			max_lambda = 0.0f;
		}
		limit = mRotationLimitsConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2, mA1, min_lambda, max_lambda);
	}

	return motor || pos || rot || limit;
}

bool HingeConstraint::SolvePositionConstraint(float inDeltaTime, float inBaumgarte)
{
	// Motor operates on velocities only, don't call SolvePositionConstraint

	// Solve point constraint
	mPointConstraintPart.CalculateConstraintProperties(*mBody1, Mat44::sRotation(mBody1->GetRotation()), mLocalSpacePosition1, *mBody2, Mat44::sRotation(mBody2->GetRotation()), mLocalSpacePosition2);
	bool pos = mPointConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, inBaumgarte);

	// Solve rotation constraint
	Mat44 rotation1 = Mat44::sRotation(mBody1->GetRotation()); // Note that previous call to GetRotation() is out of date since the rotation has changed
	Mat44 rotation2 = Mat44::sRotation(mBody2->GetRotation());
	mRotationConstraintPart.CalculateConstraintProperties(*mBody1, rotation1, rotation1.Multiply3x3(mLocalSpaceHingeAxis1), *mBody2, rotation2, rotation2.Multiply3x3(mLocalSpaceHingeAxis2));
	bool rot = mRotationConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, inBaumgarte);

	// Solve rotation limits
	bool limit = false;
	if (mHasLimits && mLimitsSpringSettings.mFrequency <= 0.0f)
	{
		CalculateA1AndTheta();
		CalculateRotationLimitsConstraintProperties(inDeltaTime);
		if (mRotationLimitsConstraintPart.IsActive())
			limit = mRotationLimitsConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, GetSmallestAngleToLimit(), inBaumgarte);
	}

	return pos || rot || limit;
}

#ifndef MOSS_DEBUG_RENDERER
void HingeConstraint::DrawConstraint(DebugRenderer *inRenderer) const
{
	RMat44 transform1 = mBody1->GetCenterOfMassTransform();
	RMat44 transform2 = mBody2->GetCenterOfMassTransform();

	// Draw constraint
	RVec3 constraint_pos1 = transform1 * mLocalSpacePosition1;
	inRenderer->DrawMarker(constraint_pos1, Color::sRed, 0.1f);
	inRenderer->DrawLine(constraint_pos1, transform1 * (mLocalSpacePosition1 + mDrawConstraintSize * mLocalSpaceHingeAxis1), Color::sRed);

	RVec3 constraint_pos2 = transform2 * mLocalSpacePosition2;
	inRenderer->DrawMarker(constraint_pos2, Color::sGreen, 0.1f);
	inRenderer->DrawLine(constraint_pos2, transform2 * (mLocalSpacePosition2 + mDrawConstraintSize * mLocalSpaceHingeAxis2), Color::sGreen);
	inRenderer->DrawLine(constraint_pos2, transform2 * (mLocalSpacePosition2 + mDrawConstraintSize * mLocalSpaceNormalAxis2), Color::sWhite);
}

void HingeConstraint::DrawConstraintLimits(DebugRenderer *inRenderer) const
{
	if (mHasLimits && mLimitsMax > mLimitsMin)
	{
		// Get constraint properties in world space
		RMat44 transform1 = mBody1->GetCenterOfMassTransform();
		RVec3 position1 = transform1 * mLocalSpacePosition1;
		Vec3 hinge_axis1 = transform1.Multiply3x3(mLocalSpaceHingeAxis1);
		Vec3 normal_axis1 = transform1.Multiply3x3(mLocalSpaceNormalAxis1);

		inRenderer->DrawPie(position1, mDrawConstraintSize, hinge_axis1, normal_axis1, mLimitsMin, mLimitsMax, Color::sPurple, DebugRenderer::ECastShadow::Off);
	}
}
#endif // MOSS_DEBUG_RENDERER

void HingeConstraint::SaveState(StateRecorder &inStream) const
{
	TwoBodyConstraint::SaveState(inStream);

	mMotorConstraintPart.SaveState(inStream);
	mRotationConstraintPart.SaveState(inStream);
	mPointConstraintPart.SaveState(inStream);
	mRotationLimitsConstraintPart.SaveState(inStream);

	inStream.Write(mMotorState);
	inStream.Write(mTargetAngularVelocity);
	inStream.Write(mTargetAngle);
}

void HingeConstraint::RestoreState(StateRecorder &inStream)
{
	TwoBodyConstraint::RestoreState(inStream);

	mMotorConstraintPart.RestoreState(inStream);
	mRotationConstraintPart.RestoreState(inStream);
	mPointConstraintPart.RestoreState(inStream);
	mRotationLimitsConstraintPart.RestoreState(inStream);

	inStream.Read(mMotorState);
	inStream.Read(mTargetAngularVelocity);
	inStream.Read(mTargetAngle);
}


Ref<ConstraintSettings> HingeConstraint::GetConstraintSettings() const
{
	HingeConstraintSettings *settings = new HingeConstraintSettings;
	ToConstraintSettings(*settings);
	settings->mSpace = EConstraintSpace::LocalToBodyCOM;
	settings->mPoint1 = RVec3(mLocalSpacePosition1);
	settings->mHingeAxis1 = mLocalSpaceHingeAxis1;
	settings->mNormalAxis1 = mLocalSpaceNormalAxis1;
	settings->mPoint2 = RVec3(mLocalSpacePosition2);
	settings->mHingeAxis2 = mLocalSpaceHingeAxis2;
	settings->mNormalAxis2 = mLocalSpaceNormalAxis2;
	settings->mLimitsMin = mLimitsMin;
	settings->mLimitsMax = mLimitsMax;
	settings->mLimitsSpringSettings = mLimitsSpringSettings;
	settings->mMaxFrictionTorque = mMaxFrictionTorque;
	settings->mMotorSettings = mMotorSettings;
	return settings;
}

Mat44 HingeConstraint::GetConstraintToBody1Matrix() const
{
	return Mat44(Vec4(mLocalSpaceHingeAxis1, 0), Vec4(mLocalSpaceNormalAxis1, 0), Vec4(mLocalSpaceHingeAxis1.Cross(mLocalSpaceNormalAxis1), 0), Vec4(mLocalSpacePosition1, 1));
}

Mat44 HingeConstraint::GetConstraintToBody2Matrix() const
{
	return Mat44(Vec4(mLocalSpaceHingeAxis2, 0), Vec4(mLocalSpaceNormalAxis2, 0), Vec4(mLocalSpaceHingeAxis2.Cross(mLocalSpaceNormalAxis2), 0), Vec4(mLocalSpacePosition2, 1));
}

TwoBodyConstraint *DistanceConstraintSettings::Create(Body &inBody1, Body &inBody2) const
{
	return new DistanceConstraint(inBody1, inBody2, *this);
}

DistanceConstraint::DistanceConstraint(Body &inBody1, Body &inBody2, const DistanceConstraintSettings &inSettings) :
	TwoBodyConstraint(inBody1, inBody2, inSettings),
	mMinDistance(inSettings.mMinDistance),
	mMaxDistance(inSettings.mMaxDistance)
{
	if (inSettings.mSpace == EConstraintSpace::WorldSpace)
	{
		// If all properties were specified in world space, take them to local space now
		mLocalSpacePosition1 = Vec3(inBody1.GetInverseCenterOfMassTransform() * inSettings.mPoint1);
		mLocalSpacePosition2 = Vec3(inBody2.GetInverseCenterOfMassTransform() * inSettings.mPoint2);
		mWorldSpacePosition1 = inSettings.mPoint1;
		mWorldSpacePosition2 = inSettings.mPoint2;
	}
	else
	{
		// If properties were specified in local space, we need to calculate world space positions
		mLocalSpacePosition1 = Vec3(inSettings.mPoint1);
		mLocalSpacePosition2 = Vec3(inSettings.mPoint2);
		mWorldSpacePosition1 = inBody1.GetCenterOfMassTransform() * inSettings.mPoint1;
		mWorldSpacePosition2 = inBody2.GetCenterOfMassTransform() * inSettings.mPoint2;
	}

	// Store distance we want to keep between the world space points
	float distance = Vec3(mWorldSpacePosition2 - mWorldSpacePosition1).Length();
	float min_distance, max_distance;
	if (mMinDistance < 0.0f && mMaxDistance < 0.0f)
	{
		min_distance = max_distance = distance;
	}
	else
	{
		min_distance = mMinDistance < 0.0f? min(distance, mMaxDistance) : mMinDistance;
		max_distance = mMaxDistance < 0.0f? max(distance, mMinDistance) : mMaxDistance;
	}
	SetDistance(min_distance, max_distance);

	// Most likely gravity is going to tear us apart (this is only used when the distance between the points = 0)
	mWorldSpaceNormal = Vec3::sAxisY();

	// Store spring settings
	SetLimitsSpringSettings(inSettings.mLimitsSpringSettings);
}

void DistanceConstraint::NotifyShapeChanged(const BodyID &inBodyID, Vec3Arg inDeltaCOM)
{
	if (mBody1->GetID() == inBodyID)
		mLocalSpacePosition1 -= inDeltaCOM;
	else if (mBody2->GetID() == inBodyID)
		mLocalSpacePosition2 -= inDeltaCOM;
}

void DistanceConstraint::CalculateConstraintProperties(float inDeltaTime)
{
	// Update world space positions (the bodies may have moved)
	mWorldSpacePosition1 = mBody1->GetCenterOfMassTransform() * mLocalSpacePosition1;
	mWorldSpacePosition2 = mBody2->GetCenterOfMassTransform() * mLocalSpacePosition2;

	// Calculate world space normal
	Vec3 delta = Vec3(mWorldSpacePosition2 - mWorldSpacePosition1);
	float delta_len = delta.Length();
	if (delta_len > 0.0f)
		mWorldSpaceNormal = delta / delta_len;

	// Calculate points relative to body
	// r1 + u = (p1 - x1) + (p2 - p1) = p2 - x1
	Vec3 r1_plus_u = Vec3(mWorldSpacePosition2 - mBody1->GetCenterOfMassPosition());
	Vec3 r2 = Vec3(mWorldSpacePosition2 - mBody2->GetCenterOfMassPosition());

	if (mMinDistance == mMaxDistance)
	{
		mAxisConstraint.CalculateConstraintPropertiesWithSettings(inDeltaTime, *mBody1, r1_plus_u, *mBody2, r2, mWorldSpaceNormal, 0.0f, delta_len - mMinDistance, mLimitsSpringSettings);

		// Single distance, allow constraint forces in both directions
		mMinLambda = -FLT_MAX;
		mMaxLambda = FLT_MAX;
	}
	else if (delta_len <= mMinDistance)
	{
		mAxisConstraint.CalculateConstraintPropertiesWithSettings(inDeltaTime, *mBody1, r1_plus_u, *mBody2, r2, mWorldSpaceNormal, 0.0f, delta_len - mMinDistance, mLimitsSpringSettings);

		// Allow constraint forces to make distance bigger only
		mMinLambda = 0;
		mMaxLambda = FLT_MAX;
	}
	else if (delta_len >= mMaxDistance)
	{
		mAxisConstraint.CalculateConstraintPropertiesWithSettings(inDeltaTime, *mBody1, r1_plus_u, *mBody2, r2, mWorldSpaceNormal, 0.0f, delta_len - mMaxDistance, mLimitsSpringSettings);

		// Allow constraint forces to make distance smaller only
		mMinLambda = -FLT_MAX;
		mMaxLambda = 0;
	}
	else
		mAxisConstraint.Deactivate();
}

void DistanceConstraint::SetupVelocityConstraint(float inDeltaTime)
{
	CalculateConstraintProperties(inDeltaTime);
}

void DistanceConstraint::ResetWarmStart()
{
	mAxisConstraint.Deactivate();
}

void DistanceConstraint::WarmStartVelocityConstraint(float inWarmStartImpulseRatio)
{
	mAxisConstraint.WarmStart(*mBody1, *mBody2, mWorldSpaceNormal, inWarmStartImpulseRatio);
}

bool DistanceConstraint::SolveVelocityConstraint(float inDeltaTime)
{
	if (mAxisConstraint.IsActive())
		return mAxisConstraint.SolveVelocityConstraint(*mBody1, *mBody2, mWorldSpaceNormal, mMinLambda, mMaxLambda);
	else
		return false;
}

bool DistanceConstraint::SolvePositionConstraint(float inDeltaTime, float inBaumgarte)
{
	if (mLimitsSpringSettings.mFrequency <= 0.0f) // When the spring is active, we don't need to solve the position constraint
	{
		float distance = Vec3(mWorldSpacePosition2 - mWorldSpacePosition1).Dot(mWorldSpaceNormal);

		// Calculate position error
		float position_error = 0.0f;
		if (distance < mMinDistance)
			position_error = distance - mMinDistance;
		else if (distance > mMaxDistance)
			position_error = distance - mMaxDistance;

		if (position_error != 0.0f)
		{
			// Update constraint properties (bodies may have moved)
			CalculateConstraintProperties(inDeltaTime);

			return mAxisConstraint.SolvePositionConstraint(*mBody1, *mBody2, mWorldSpaceNormal, position_error, inBaumgarte);
		}
	}

	return false;
}

#ifndef MOSS_DEBUG_RENDERER
void DistanceConstraint::DrawConstraint(DebugRenderer *inRenderer) const
{
	// Draw constraint
	Vec3 delta = Vec3(mWorldSpacePosition2 - mWorldSpacePosition1);
	float len = delta.Length();
	if (len < mMinDistance)
	{
		RVec3 real_end_pos = mWorldSpacePosition1 + (len > 0.0f? delta * mMinDistance / len : Vec3(0, len, 0));
		inRenderer->DrawLine(mWorldSpacePosition1, mWorldSpacePosition2, Color::sGreen);
		inRenderer->DrawLine(mWorldSpacePosition2, real_end_pos, Color::sYellow);
	}
	else if (len > mMaxDistance)
	{
		RVec3 real_end_pos = mWorldSpacePosition1 + (len > 0.0f? delta * mMaxDistance / len : Vec3(0, len, 0));
		inRenderer->DrawLine(mWorldSpacePosition1, real_end_pos, Color::sGreen);
		inRenderer->DrawLine(real_end_pos, mWorldSpacePosition2, Color::sRed);
	}
	else
		inRenderer->DrawLine(mWorldSpacePosition1, mWorldSpacePosition2, Color::sGreen);

	// Draw constraint end points
	inRenderer->DrawMarker(mWorldSpacePosition1, Color::sWhite, 0.1f);
	inRenderer->DrawMarker(mWorldSpacePosition2, Color::sWhite, 0.1f);

	// Draw current length
	inRenderer->DrawText3D(0.5_r * (mWorldSpacePosition1 + mWorldSpacePosition2), StringFormat("%.2f", (double)len));
}
#endif // MOSS_DEBUG_RENDERER

void DistanceConstraint::SaveState(StateRecorder &inStream) const
{
	TwoBodyConstraint::SaveState(inStream);

	mAxisConstraint.SaveState(inStream);
	inStream.Write(mWorldSpaceNormal); // When distance = 0, the normal is used from last frame so we need to store it
}

void DistanceConstraint::RestoreState(StateRecorder &inStream)
{
	TwoBodyConstraint::RestoreState(inStream);

	mAxisConstraint.RestoreState(inStream);
	inStream.Read(mWorldSpaceNormal);
}

Ref<ConstraintSettings> DistanceConstraint::GetConstraintSettings() const
{
	DistanceConstraintSettings *settings = new DistanceConstraintSettings;
	ToConstraintSettings(*settings);
	settings->mSpace = EConstraintSpace::LocalToBodyCOM;
	settings->mPoint1 = RVec3(mLocalSpacePosition1);
	settings->mPoint2 = RVec3(mLocalSpacePosition2);
	settings->mMinDistance = mMinDistance;
	settings->mMaxDistance = mMaxDistance;
	settings->mLimitsSpringSettings = mLimitsSpringSettings;
	return settings;
}

TwoBodyConstraint *PathConstraintSettings::Create(Body &inBody1, Body &inBody2) const
{
	return new PathConstraint(inBody1, inBody2, *this);
}

PathConstraint::PathConstraint(Body &inBody1, Body &inBody2, const PathConstraintSettings &inSettings) :
	TwoBodyConstraint(inBody1, inBody2, inSettings),
	mRotationConstraintType(inSettings.mRotationConstraintType),
	mMaxFrictionForce(inSettings.mMaxFrictionForce),
	mPositionMotorSettings(inSettings.mPositionMotorSettings)
{
	// Calculate transform that takes us from the path start to center of mass space of body 1
	mPathToBody1 = Mat44::sRotationTranslation(inSettings.mPathRotation, inSettings.mPathPosition - inBody1.GetShape()->GetCenterOfMass());

	SetPath(inSettings.mPath, inSettings.mPathFraction);
}

void PathConstraint::NotifyShapeChanged(const BodyID &inBodyID, Vec3Arg inDeltaCOM)
{
	if (mBody1->GetID() == inBodyID)
		mPathToBody1.SetTranslation(mPathToBody1.GetTranslation() - inDeltaCOM);
	else if (mBody2->GetID() == inBodyID)
		mPathToBody2.SetTranslation(mPathToBody2.GetTranslation() - inDeltaCOM);
}

void PathConstraint::SetPath(const PathConstraintPath *inPath, float inPathFraction)
{
	mPath = inPath;
	mPathFraction = inPathFraction;

	if (mPath != nullptr)
	{
		// Get the point on the path for this fraction
		Vec3 path_point, path_tangent, path_normal, path_binormal;
		mPath->GetPointOnPath(mPathFraction, path_point, path_tangent, path_normal, path_binormal);

		// Construct the matrix that takes us from the closest point on the path to body 2 center of mass space
		Mat44 closest_point_to_path(Vec4(path_tangent, 0), Vec4(path_binormal, 0), Vec4(path_normal, 0), Vec4(path_point, 1));
		Mat44 cp_to_body1 = mPathToBody1 * closest_point_to_path;
		mPathToBody2 = (mBody2->GetInverseCenterOfMassTransform() * mBody1->GetCenterOfMassTransform()).ToMat44() * cp_to_body1;

		// Calculate initial orientation
		if (mRotationConstraintType == EPathRotationConstraintType::FullyConstrained)
			mInvInitialOrientation = RotationEulerConstraintPart::sGetInvInitialOrientation(*mBody1, *mBody2);
	}
}

void PathConstraint::CalculateConstraintProperties(float inDeltaTime)
{
	// Get transforms of body 1 and 2
	RMat44 transform1 = mBody1->GetCenterOfMassTransform();
	RMat44 transform2 = mBody2->GetCenterOfMassTransform();

	// Get the transform of the path transform as seen from body 1 in world space
	RMat44 path_to_world_1 = transform1 * mPathToBody1;

	// Get the transform of from the point on path that body 2 is attached to in world space
	RMat44 path_to_world_2 = transform2 * mPathToBody2;

	// Calculate new closest point on path
	RVec3 position2 = path_to_world_2.GetTranslation();
	Vec3 position2_local_to_path = Vec3(path_to_world_1.InversedRotationTranslation() * position2);
	mPathFraction = mPath->GetClosestPoint(position2_local_to_path, mPathFraction);

	// Get the point on the path for this fraction
	Vec3 path_point, path_tangent, path_normal, path_binormal;
	mPath->GetPointOnPath(mPathFraction, path_point, path_tangent, path_normal, path_binormal);

	// Calculate R1 and R2
	RVec3 path_point_ws = path_to_world_1 * path_point;
	mR1 = Vec3(path_point_ws - mBody1->GetCenterOfMassPosition());
	mR2 = Vec3(position2 - mBody2->GetCenterOfMassPosition());

	// Calculate U = X2 + R2 - X1 - R1
	mU = Vec3(position2 - path_point_ws);

	// Calculate world space normals
	mPathNormal = path_to_world_1.Multiply3x3(path_normal);
	mPathBinormal = path_to_world_1.Multiply3x3(path_binormal);

	// Calculate slide axis
	mPathTangent = path_to_world_1.Multiply3x3(path_tangent);

	// Prepare constraint part for position constraint to slide along the path
	mPositionConstraintPart.CalculateConstraintProperties(*mBody1, transform1.GetRotation(), mR1 + mU, *mBody2, transform2.GetRotation(), mR2, mPathNormal, mPathBinormal);

	// Check if closest point is on the boundary of the path and if so apply limit
	if (!mPath->IsLooping() && (mPathFraction <= 0.0f || mPathFraction >= mPath->GetPathMaxFraction()))
		mPositionLimitsConstraintPart.CalculateConstraintProperties(*mBody1, mR1 + mU, *mBody2, mR2, mPathTangent);
	else
		mPositionLimitsConstraintPart.Deactivate();

	// Prepare rotation constraint part
	switch (mRotationConstraintType)
	{
	case EPathRotationConstraintType::Free:
		// No rotational limits
		break;

	case EPathRotationConstraintType::ConstrainAroundTangent:
		mHingeConstraintPart.CalculateConstraintProperties(*mBody1, transform1.GetRotation(), mPathTangent, *mBody2, transform2.GetRotation(), path_to_world_2.GetAxisX());
		break;

	case EPathRotationConstraintType::ConstrainAroundNormal:
		mHingeConstraintPart.CalculateConstraintProperties(*mBody1, transform1.GetRotation(), mPathNormal, *mBody2, transform2.GetRotation(), path_to_world_2.GetAxisZ());
		break;

	case EPathRotationConstraintType::ConstrainAroundBinormal:
		mHingeConstraintPart.CalculateConstraintProperties(*mBody1, transform1.GetRotation(), mPathBinormal, *mBody2, transform2.GetRotation(), path_to_world_2.GetAxisY());
		break;

	case EPathRotationConstraintType::ConstrainToPath:
		// We need to calculate the inverse of the rotation from body 1 to body 2 for the current path position (see: RotationEulerConstraintPart::sGetInvInitialOrientation)
		// RotationBody2 = RotationBody1 * InitialOrientation <=> InitialOrientation^-1 = RotationBody2^-1 * RotationBody1
		// We can express RotationBody2 in terms of RotationBody1: RotationBody2 = RotationBody1 * PathToBody1 * RotationClosestPointOnPath * PathToBody2^-1
		// Combining these two: InitialOrientation^-1 = PathToBody2 * (PathToBody1 * RotationClosestPointOnPath)^-1
		mInvInitialOrientation = mPathToBody2.Multiply3x3RightTransposed(mPathToBody1.Multiply3x3(Mat44(Vec4(path_tangent, 0), Vec4(path_binormal, 0), Vec4(path_normal, 0), Vec4::sZero()))).GetQuaternion();
		[[fallthrough]];

	case EPathRotationConstraintType::FullyConstrained:
		mRotationConstraintPart.CalculateConstraintProperties(*mBody1, transform1.GetRotation(), *mBody2, transform2.GetRotation());
		break;
	}

	// Motor properties
	switch (mPositionMotorState)
	{
	case EMotorState::Off:
		if (mMaxFrictionForce > 0.0f)
			mPositionMotorConstraintPart.CalculateConstraintProperties(*mBody1, mR1 + mU, *mBody2, mR2, mPathTangent);
		else
			mPositionMotorConstraintPart.Deactivate();
		break;

	case EMotorState::Velocity:
		mPositionMotorConstraintPart.CalculateConstraintProperties(*mBody1, mR1 + mU, *mBody2, mR2, mPathTangent, -mTargetVelocity);
		break;

	case EMotorState::Position:
		if (mPositionMotorSettings.mSpringSettings.HasStiffness())
		{
			// Calculate constraint value to drive to
			float c;
			if (mPath->IsLooping())
			{
				float max_fraction = mPath->GetPathMaxFraction();
				c = fmod(mPathFraction - mTargetPathFraction, max_fraction);
				float half_max_fraction = 0.5f * max_fraction;
				if (c > half_max_fraction)
					c -= max_fraction;
				else if (c < -half_max_fraction)
					c += max_fraction;
			}
			else
				c = mPathFraction - mTargetPathFraction;
			mPositionMotorConstraintPart.CalculateConstraintPropertiesWithSettings(inDeltaTime, *mBody1, mR1 + mU, *mBody2, mR2, mPathTangent, 0.0f, c, mPositionMotorSettings.mSpringSettings);
		}
		else
			mPositionMotorConstraintPart.Deactivate();
		break;
	}
}

void PathConstraint::SetupVelocityConstraint(float inDeltaTime)
{
	CalculateConstraintProperties(inDeltaTime);
}

void PathConstraint::ResetWarmStart()
{
	mPositionMotorConstraintPart.Deactivate();
	mPositionConstraintPart.Deactivate();
	mPositionLimitsConstraintPart.Deactivate();
	mHingeConstraintPart.Deactivate();
	mRotationConstraintPart.Deactivate();
}

void PathConstraint::WarmStartVelocityConstraint(float inWarmStartImpulseRatio)
{
	// Warm starting: Apply previous frame impulse
	mPositionMotorConstraintPart.WarmStart(*mBody1, *mBody2, mPathTangent, inWarmStartImpulseRatio);
	mPositionConstraintPart.WarmStart(*mBody1, *mBody2, mPathNormal, mPathBinormal, inWarmStartImpulseRatio);
	mPositionLimitsConstraintPart.WarmStart(*mBody1, *mBody2, mPathTangent, inWarmStartImpulseRatio);

	switch (mRotationConstraintType)
	{
	case EPathRotationConstraintType::Free:
		// No rotational limits
		break;

	case EPathRotationConstraintType::ConstrainAroundTangent:
	case EPathRotationConstraintType::ConstrainAroundNormal:
	case EPathRotationConstraintType::ConstrainAroundBinormal:
		mHingeConstraintPart.WarmStart(*mBody1, *mBody2, inWarmStartImpulseRatio);
		break;

	case EPathRotationConstraintType::ConstrainToPath:
	case EPathRotationConstraintType::FullyConstrained:
		mRotationConstraintPart.WarmStart(*mBody1, *mBody2, inWarmStartImpulseRatio);
		break;
	}
}

bool PathConstraint::SolveVelocityConstraint(float inDeltaTime)
{
	// Solve motor
	bool motor = false;
	if (mPositionMotorConstraintPart.IsActive())
	{
		switch (mPositionMotorState)
		{
		case EMotorState::Off:
			{
				float max_lambda = mMaxFrictionForce * inDeltaTime;
				motor = mPositionMotorConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2, mPathTangent, -max_lambda, max_lambda);
				break;
			}

		case EMotorState::Velocity:
		case EMotorState::Position:
			motor = mPositionMotorConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2, mPathTangent, inDeltaTime * mPositionMotorSettings.mMinForceLimit, inDeltaTime * mPositionMotorSettings.mMaxForceLimit);
			break;
		}
	}

	// Solve position constraint along 2 axis
	bool pos = mPositionConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2, mPathNormal, mPathBinormal);

	// Solve limits along path axis
	bool limit = false;
	if (mPositionLimitsConstraintPart.IsActive())
	{
		if (mPathFraction <= 0.0f)
			limit = mPositionLimitsConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2, mPathTangent, 0, FLT_MAX);
		else
		{
			MOSS_ASSERT(mPathFraction >= mPath->GetPathMaxFraction());
			limit = mPositionLimitsConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2, mPathTangent, -FLT_MAX, 0);
		}
	}

	// Solve rotational constraint
	// Note, this is not entirely correct, we should apply a velocity constraint so that the body will actually follow the path
	// by looking at the derivative of the tangent, normal or binormal but we don't. This means the position constraint solver
	// will need to correct the orientation error that builds up, which in turn means that the simulation is not physically correct.
	bool rot = false;
	switch (mRotationConstraintType)
	{
	case EPathRotationConstraintType::Free:
		// No rotational limits
		break;

	case EPathRotationConstraintType::ConstrainAroundTangent:
	case EPathRotationConstraintType::ConstrainAroundNormal:
	case EPathRotationConstraintType::ConstrainAroundBinormal:
		rot = mHingeConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2);
		break;

	case EPathRotationConstraintType::ConstrainToPath:
	case EPathRotationConstraintType::FullyConstrained:
		rot = mRotationConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2);
		break;
	}

	return motor || pos || limit || rot;
}

bool PathConstraint::SolvePositionConstraint(float inDeltaTime, float inBaumgarte)
{
	// Update constraint properties (bodies may have moved)
	CalculateConstraintProperties(inDeltaTime);

	// Solve position constraint along 2 axis
	bool pos = mPositionConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, mU, mPathNormal, mPathBinormal, inBaumgarte);

	// Solve limits along path axis
	bool limit = false;
	if (mPositionLimitsConstraintPart.IsActive())
	{
		if (mPathFraction <= 0.0f)
			limit = mPositionLimitsConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, mPathTangent, mU.Dot(mPathTangent), inBaumgarte);
		else
		{
			MOSS_ASSERT(mPathFraction >= mPath->GetPathMaxFraction());
			limit = mPositionLimitsConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, mPathTangent, mU.Dot(mPathTangent), inBaumgarte);
		}
	}

	// Solve rotational constraint
	bool rot = false;
	switch (mRotationConstraintType)
	{
	case EPathRotationConstraintType::Free:
		// No rotational limits
		break;

	case EPathRotationConstraintType::ConstrainAroundTangent:
	case EPathRotationConstraintType::ConstrainAroundNormal:
	case EPathRotationConstraintType::ConstrainAroundBinormal:
		rot = mHingeConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, inBaumgarte);
		break;

	case EPathRotationConstraintType::ConstrainToPath:
	case EPathRotationConstraintType::FullyConstrained:
		rot = mRotationConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, mInvInitialOrientation, inBaumgarte);
		break;
	}

	return pos || limit || rot;
}

#ifndef MOSS_DEBUG_RENDERER
void PathConstraint::DrawConstraint(DebugRenderer *inRenderer) const
{
	if (mPath != nullptr)
	{
		// Draw the path in world space
		RMat44 path_to_world = mBody1->GetCenterOfMassTransform() * mPathToBody1;
		mPath->DrawPath(inRenderer, path_to_world);

		// Draw anchor point of both bodies in world space
		RVec3 x1 = mBody1->GetCenterOfMassPosition() + mR1;
		RVec3 x2 = mBody2->GetCenterOfMassPosition() + mR2;
		inRenderer->DrawMarker(x1, Color::sYellow, 0.1f);
		inRenderer->DrawMarker(x2, Color::sYellow, 0.1f);
		inRenderer->DrawArrow(x1, x1 + mPathTangent, Color::sBlue, 0.1f);
		inRenderer->DrawArrow(x1, x1 + mPathNormal, Color::sRed, 0.1f);
		inRenderer->DrawArrow(x1, x1 + mPathBinormal, Color::sGreen, 0.1f);
		inRenderer->DrawText3D(x1, StringFormat("%.1f", (double)mPathFraction));

		// Draw motor
		switch (mPositionMotorState)
		{
		case EMotorState::Position:
			{
				// Draw target marker
				Vec3 position, tangent, normal, binormal;
				mPath->GetPointOnPath(mTargetPathFraction, position, tangent, normal, binormal);
				inRenderer->DrawMarker(path_to_world * position, Color::sYellow, 1.0f);
				break;
			}

		case EMotorState::Velocity:
			{
				RVec3 position = mBody2->GetCenterOfMassPosition() + mR2;
				inRenderer->DrawArrow(position, position + mPathTangent * mTargetVelocity, Color::sRed, 0.1f);
				break;
			}

		case EMotorState::Off:
			break;
		}
	}
}
#endif // MOSS_DEBUG_RENDERER

void PathConstraint::SaveState(StateRecorder &inStream) const
{
	TwoBodyConstraint::SaveState(inStream);

	mPositionConstraintPart.SaveState(inStream);
	mPositionLimitsConstraintPart.SaveState(inStream);
	mPositionMotorConstraintPart.SaveState(inStream);
	mHingeConstraintPart.SaveState(inStream);
	mRotationConstraintPart.SaveState(inStream);

	inStream.Write(mMaxFrictionForce);
	inStream.Write(mPositionMotorSettings);
	inStream.Write(mPositionMotorState);
	inStream.Write(mTargetVelocity);
	inStream.Write(mTargetPathFraction);
	inStream.Write(mPathFraction);
}

void PathConstraint::RestoreState(StateRecorder &inStream)
{
	TwoBodyConstraint::RestoreState(inStream);

	mPositionConstraintPart.RestoreState(inStream);
	mPositionLimitsConstraintPart.RestoreState(inStream);
	mPositionMotorConstraintPart.RestoreState(inStream);
	mHingeConstraintPart.RestoreState(inStream);
	mRotationConstraintPart.RestoreState(inStream);

	inStream.Read(mMaxFrictionForce);
	inStream.Read(mPositionMotorSettings);
	inStream.Read(mPositionMotorState);
	inStream.Read(mTargetVelocity);
	inStream.Read(mTargetPathFraction);
	inStream.Read(mPathFraction);
}

Ref<ConstraintSettings> PathConstraint::GetConstraintSettings() const
{
	MOSS_ASSERT(false); // Not implemented yet
	return nullptr;
}


TwoBodyConstraint *PulleyConstraintSettings::Create(Body &inBody1, Body &inBody2) const
{
	return new PulleyConstraint(inBody1, inBody2, *this);
}

PulleyConstraint::PulleyConstraint(Body &inBody1, Body &inBody2, const PulleyConstraintSettings &inSettings) :
	TwoBodyConstraint(inBody1, inBody2, inSettings),
	mFixedPosition1(inSettings.mFixedPoint1),
	mFixedPosition2(inSettings.mFixedPoint2),
	mRatio(inSettings.mRatio),
	mMinLength(inSettings.mMinLength),
	mMaxLength(inSettings.mMaxLength)
{
	if (inSettings.mSpace == EConstraintSpace::WorldSpace)
	{
		// If all properties were specified in world space, take them to local space now
		mLocalSpacePosition1 = Vec3(inBody1.GetInverseCenterOfMassTransform() * inSettings.mBodyPoint1);
		mLocalSpacePosition2 = Vec3(inBody2.GetInverseCenterOfMassTransform() * inSettings.mBodyPoint2);
		mWorldSpacePosition1 = inSettings.mBodyPoint1;
		mWorldSpacePosition2 = inSettings.mBodyPoint2;
	}
	else
	{
		// If properties were specified in local space, we need to calculate world space positions
		mLocalSpacePosition1 = Vec3(inSettings.mBodyPoint1);
		mLocalSpacePosition2 = Vec3(inSettings.mBodyPoint2);
		mWorldSpacePosition1 = inBody1.GetCenterOfMassTransform() * inSettings.mBodyPoint1;
		mWorldSpacePosition2 = inBody2.GetCenterOfMassTransform() * inSettings.mBodyPoint2;
	}

	// Calculate min/max length if it was not provided
	float current_length = GetCurrentLength();
	if (mMinLength < 0.0f)
		mMinLength = current_length;
	if (mMaxLength < 0.0f)
		mMaxLength = current_length;

	// Initialize the normals to a likely valid axis in case the fixed points overlap with the attachment points (most likely the fixed points are above both bodies)
	mWorldSpaceNormal1 = mWorldSpaceNormal2 = -Vec3::sAxisY();
}

void PulleyConstraint::NotifyShapeChanged(const BodyID &inBodyID, Vec3Arg inDeltaCOM)
{
	if (mBody1->GetID() == inBodyID)
		mLocalSpacePosition1 -= inDeltaCOM;
	else if (mBody2->GetID() == inBodyID)
		mLocalSpacePosition2 -= inDeltaCOM;
}

float PulleyConstraint::CalculatePositionsNormalsAndLength()
{
	// Update world space positions (the bodies may have moved)
	mWorldSpacePosition1 = mBody1->GetCenterOfMassTransform() * mLocalSpacePosition1;
	mWorldSpacePosition2 = mBody2->GetCenterOfMassTransform() * mLocalSpacePosition2;

	// Calculate world space normals
	Vec3 delta1 = Vec3(mWorldSpacePosition1 - mFixedPosition1);
	float delta1_len = delta1.Length();
	if (delta1_len > 0.0f)
		mWorldSpaceNormal1 = delta1 / delta1_len;

	Vec3 delta2 = Vec3(mWorldSpacePosition2 - mFixedPosition2);
	float delta2_len = delta2.Length();
	if (delta2_len > 0.0f)
		mWorldSpaceNormal2 = delta2 / delta2_len;

	// Calculate length
	return delta1_len + mRatio * delta2_len;
}

void PulleyConstraint::CalculateConstraintProperties()
{
	// Calculate attachment points relative to COM
	Vec3 r1 = Vec3(mWorldSpacePosition1 - mBody1->GetCenterOfMassPosition());
	Vec3 r2 = Vec3(mWorldSpacePosition2 - mBody2->GetCenterOfMassPosition());

	mIndependentAxisConstraintPart.CalculateConstraintProperties(*mBody1, *mBody2, r1, mWorldSpaceNormal1, r2, mWorldSpaceNormal2, mRatio);
}

void PulleyConstraint::SetupVelocityConstraint(float inDeltaTime)
{
	// Determine if the constraint is active
	float current_length = CalculatePositionsNormalsAndLength();
	bool min_length_violation = current_length <= mMinLength;
	bool max_length_violation = current_length >= mMaxLength;
	if (min_length_violation || max_length_violation)
	{
		// Determine max lambda based on if the length is too big or small
		mMinLambda = max_length_violation? -FLT_MAX : 0.0f;
		mMaxLambda = min_length_violation? FLT_MAX : 0.0f;

		CalculateConstraintProperties();
	}
	else
		mIndependentAxisConstraintPart.Deactivate();
}

void PulleyConstraint::ResetWarmStart()
{
	mIndependentAxisConstraintPart.Deactivate();
}

void PulleyConstraint::WarmStartVelocityConstraint(float inWarmStartImpulseRatio)
{
	mIndependentAxisConstraintPart.WarmStart(*mBody1, *mBody2, mWorldSpaceNormal1, mWorldSpaceNormal2, mRatio, inWarmStartImpulseRatio);
}

bool PulleyConstraint::SolveVelocityConstraint(float inDeltaTime)
{
	if (mIndependentAxisConstraintPart.IsActive())
		return mIndependentAxisConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2, mWorldSpaceNormal1, mWorldSpaceNormal2, mRatio, mMinLambda, mMaxLambda);
	else
		return false;
}

bool PulleyConstraint::SolvePositionConstraint(float inDeltaTime, float inBaumgarte)
{
	// Calculate new length (bodies may have changed)
	float current_length = CalculatePositionsNormalsAndLength();

	float position_error = 0.0f;
	if (current_length < mMinLength)
		position_error = current_length - mMinLength;
	else if (current_length > mMaxLength)
		position_error = current_length - mMaxLength;

	if (position_error != 0.0f)
	{
		// Update constraint properties (bodies may have moved)
		CalculateConstraintProperties();

		return mIndependentAxisConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, mWorldSpaceNormal1, mWorldSpaceNormal2, mRatio, position_error, inBaumgarte);
	}

	return false;
}

#ifndef MOSS_DEBUG_RENDERER
void PulleyConstraint::DrawConstraint(DebugRenderer *inRenderer) const
{
	// Color according to length vs min/max length
	float current_length = GetCurrentLength();
	Color color = Color::sGreen;
	if (current_length < mMinLength)
		color = Color::sYellow;
	else if (current_length > mMaxLength)
		color = Color::sRed;

	// Draw constraint
	inRenderer->DrawLine(mWorldSpacePosition1, mFixedPosition1, color);
	inRenderer->DrawLine(mFixedPosition1, mFixedPosition2, color);
	inRenderer->DrawLine(mFixedPosition2, mWorldSpacePosition2, color);

	// Draw current length
	inRenderer->DrawText3D(0.5_r * (mFixedPosition1 + mFixedPosition2), StringFormat("%.2f", (double)current_length));
}
#endif // MOSS_DEBUG_RENDERER

void PulleyConstraint::SaveState(StateRecorder &inStream) const
{
	TwoBodyConstraint::SaveState(inStream);

	mIndependentAxisConstraintPart.SaveState(inStream);
	inStream.Write(mWorldSpaceNormal1); // When distance to fixed point = 0, the normal is used from last frame so we need to store it
	inStream.Write(mWorldSpaceNormal2);
}

void PulleyConstraint::RestoreState(StateRecorder &inStream)
{
	TwoBodyConstraint::RestoreState(inStream);

	mIndependentAxisConstraintPart.RestoreState(inStream);
	inStream.Read(mWorldSpaceNormal1);
	inStream.Read(mWorldSpaceNormal2);
}

Ref<ConstraintSettings> PulleyConstraint::GetConstraintSettings() const
{
	PulleyConstraintSettings *settings = new PulleyConstraintSettings;
	ToConstraintSettings(*settings);
	settings->mSpace = EConstraintSpace::LocalToBodyCOM;
	settings->mBodyPoint1 = RVec3(mLocalSpacePosition1);
	settings->mFixedPoint1 = mFixedPosition1;
	settings->mBodyPoint2 = RVec3(mLocalSpacePosition2);
	settings->mFixedPoint2 = mFixedPosition2;
	settings->mRatio = mRatio;
	settings->mMinLength = mMinLength;
	settings->mMaxLength = mMaxLength;
	return settings;
}

// Calculate position and tangent for a Cubic Hermite Spline segment
static inline void sCalculatePositionAndTangent(Vec3Arg inP1, Vec3Arg inM1, Vec3Arg inP2, Vec3Arg inM2, float inT, Vec3 &outPosition, Vec3 &outTangent)
{
	// Calculate factors for Cubic Hermite Spline
	// See: https://en.wikipedia.org/wiki/Cubic_Hermite_spline
	float t2 = inT * inT;
	float t3 = inT * t2;
	float h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
	float h10 = t3 - 2.0f * t2 + inT;
	float h01 = -2.0f * t3 + 3.0f * t2;
	float h11 = t3 - t2;

	// Calculate d/dt for factors to calculate the tangent
	float ddt_h00 = 6.0f * (t2 - inT);
	float ddt_h10 = 3.0f * t2 - 4.0f * inT + 1.0f;
	float ddt_h01 = -ddt_h00;
	float ddt_h11 = 3.0f * t2 - 2.0f * inT;

	outPosition = h00 * inP1 + h10 * inM1 + h01 * inP2 + h11 * inM2;
	outTangent = ddt_h00 * inP1 + ddt_h10 * inM1 + ddt_h01 * inP2 + ddt_h11 * inM2;
}

// Calculate the closest point to the origin for a Cubic Hermite Spline segment
// This is used to get an estimate for the interval in which the closest point can be found,
// the interval [0, 1] is too big for Newton Raphson to work on because it is solving a 5th degree polynomial which may
// have multiple local minima that are not the root. This happens especially when the path is straight (tangents aligned with inP2 - inP1).
// Based on the bisection method: https://en.wikipedia.org/wiki/Bisection_method
static inline void sCalculateClosestPointThroughBisection(Vec3Arg inP1, Vec3Arg inM1, Vec3Arg inP2, Vec3Arg inM2, float &outTMin, float &outTMax)
{
	outTMin = 0.0f;
	outTMax = 1.0f;

	// To get the closest point of the curve to the origin we need to solve:
	// d/dt P(t) . P(t) = 0 for t, where P(t) is the point on the curve segment
	// Using d/dt (a(t) . b(t)) = d/dt a(t) . b(t) + a(t) . d/dt b(t)
	// See: https://proofwiki.org/wiki/Derivative_of_Dot_Product_of_Vector-Valued_Functions
	// d/dt P(t) . P(t) = 2 P(t) d/dt P(t) = 2 P(t) . Tangent(t)

	// Calculate the derivative at t = 0, we know P(0) = inP1 and Tangent(0) = inM1
	float ddt_min = inP1.Dot(inM1); // Leaving out factor 2, we're only interested in the root
	if (abs(ddt_min) < 1.0e-6f)
	{
		// Derivative is near zero, we found our root
		outTMax = 0.0f;
		return;
	}
	bool ddt_min_negative = ddt_min < 0.0f;

	// Calculate derivative at t = 1, we know P(1) = inP2 and Tangent(1) = inM2
	float ddt_max = inP2.Dot(inM2);
	if (abs(ddt_max) < 1.0e-6f)
	{
		// Derivative is near zero, we found our root
		outTMin = 1.0f;
		return;
	}
	bool ddt_max_negative = ddt_max < 0.0f;

	// If the signs of the derivative are not different, this algorithm can't find the root
	if (ddt_min_negative == ddt_max_negative)
		return;

	// With 4 iterations we'll get a result accurate to 1 / 2^4 = 0.0625
	for (int iteration = 0; iteration < 4; ++iteration)
	{
		float t_mid = 0.5f * (outTMin + outTMax);
		Vec3 position, tangent;
		sCalculatePositionAndTangent(inP1, inM1, inP2, inM2, t_mid, position, tangent);
		float ddt_mid = position.Dot(tangent);
		if (abs(ddt_mid) < 1.0e-6f)
		{
			// Derivative is near zero, we found our root
			outTMin = outTMax = t_mid;
			return;
		}
		bool ddt_mid_negative = ddt_mid < 0.0f;

		// Update the search interval so that the signs of the derivative at both ends of the interval are still different
		if (ddt_mid_negative == ddt_min_negative)
			outTMin = t_mid;
		else
			outTMax = t_mid;
	}
}

// Calculate the closest point to the origin for a Cubic Hermite Spline segment
// Only considers the range t e [inTMin, inTMax] and will stop as soon as the closest point falls outside of that range
static inline float sCalculateClosestPointThroughNewtonRaphson(Vec3Arg inP1, Vec3Arg inM1, Vec3Arg inP2, Vec3Arg inM2, float inTMin, float inTMax, float &outDistanceSq)
{
	// This is the closest position on the curve to the origin that we found
	Vec3 position;

	// Calculate the size of the interval
	float interval = inTMax - inTMin;

	// Start in the middle of the interval
	float t = 0.5f * (inTMin + inTMax);

	// Do max 10 iterations to prevent taking too much CPU time
	for (int iteration = 0; iteration < 10; ++iteration)
	{
		// Calculate derivative at t, see comment at sCalculateClosestPointThroughBisection for derivation of the equations
		Vec3 tangent;
		sCalculatePositionAndTangent(inP1, inM1, inP2, inM2, t, position, tangent);
		float ddt = position.Dot(tangent); // Leaving out factor 2, we're only interested in the root

		// Calculate derivative of ddt: d^2/dt P(t) . P(t) = d/dt (2 P(t) . Tangent(t))
		// = 2 (d/dt P(t)) . Tangent(t) + P(t) . d/dt Tangent(t)) = 2 (Tangent(t) . Tangent(t) + P(t) . d/dt Tangent(t))
		float d2dt_h00 = 12.0f * t - 6.0f;
		float d2dt_h10 = 6.0f * t - 4.0f;
		float d2dt_h01 = -d2dt_h00;
		float d2dt_h11 = 6.0f * t - 2.0f;
		Vec3 ddt_tangent = d2dt_h00 * inP1 + d2dt_h10 * inM1 + d2dt_h01 * inP2 + d2dt_h11 * inM2;
		float d2dt = tangent.Dot(tangent) + position.Dot(ddt_tangent); // Leaving out factor 2, because we left it out above too

		// If d2dt is zero, the curve is flat and there are multiple t's for which we are closest to the origin, stop now
		if (d2dt == 0.0f)
			break;

		// Do a Newton Raphson step
		// See: https://en.wikipedia.org/wiki/Newton%27s_method
		// Clamp against [-interval, interval] to avoid overshooting too much, we're not interested outside the interval
		float delta = Clamp(-ddt / d2dt, -interval, interval);

		// If we're stepping away further from t e [inTMin, inTMax] stop now
		if ((t > inTMax && delta > 0.0f) || (t < inTMin && delta < 0.0f))
			break;

		// If we've converged, stop now
		t += delta;
		if (abs(delta) < 1.0e-4f)
			break;
	}

	// Calculate the distance squared for the origin to the curve
	outDistanceSq = position.LengthSq();
	return t;
}

void PathConstraintPathHermite::GetIndexAndT(float inFraction, int &outIndex, float &outT) const
{
	int num_points = int(mPoints.size());

	// Start by truncating the fraction to get the index and storing the remainder in t
	int index = int(trunc(inFraction));
	float t = inFraction - float(index);

	if (IsLooping())
	{
		MOSS_ASSERT(!mPoints.front().mPosition.IsClose(mPoints.back().mPosition), "A looping path should have a different first and last point!");

		// Make sure index is positive by adding a multiple of num_points
		if (index < 0)
			index += (-index / num_points + 1) * num_points;

		// Index needs to be modulo num_points
		index = index % num_points;
	}
	else
	{
		// Clamp against range of points
		if (index < 0)
		{
			index = 0;
			t = 0.0f;
		}
		else if (index >= num_points - 1)
		{
			index = num_points - 2;
			t = 1.0f;
		}
	}

	outIndex = index;
	outT = t;
}

float PathConstraintPathHermite::GetClosestPoint(Vec3Arg inPosition, float inFractionHint) const
{
	MOSS_PROFILE_FUNCTION();

	int num_points = int(mPoints.size());

	// Start with last point on the path, in the non-looping case we won't be visiting this point
	float best_dist_sq = (mPoints[num_points - 1].mPosition - inPosition).LengthSq();
	float best_t = float(num_points - 1);

	// Loop over all points
	for (int i = 0, max_i = IsLooping()? num_points : num_points - 1; i < max_i; ++i)
	{
		const Point &p1 = mPoints[i];
		const Point &p2 = mPoints[(i + 1) % num_points];

		// Make the curve relative to inPosition
		Vec3 p1_pos = p1.mPosition - inPosition;
		Vec3 p2_pos = p2.mPosition - inPosition;

		// Get distance to p1
		float dist_sq = p1_pos.LengthSq();
		if (dist_sq < best_dist_sq)
		{
			best_t = float(i);
			best_dist_sq = dist_sq;
		}

		// First find an interval for the closest point so that we can start doing Newton Raphson steps
		float t_min, t_max;
		sCalculateClosestPointThroughBisection(p1_pos, p1.mTangent, p2_pos, p2.mTangent, t_min, t_max);

		if (t_min == t_max)
		{
			// If the function above returned no interval then it found the root already and we can just calculate the distance
			Vec3 position, tangent;
			sCalculatePositionAndTangent(p1_pos, p1.mTangent, p2_pos, p2.mTangent, t_min, position, tangent);
			dist_sq = position.LengthSq();
			if (dist_sq < best_dist_sq)
			{
				best_t = float(i) + t_min;
				best_dist_sq = dist_sq;
			}
		}
		else
		{
			// Get closest distance along curve segment
			float t = sCalculateClosestPointThroughNewtonRaphson(p1_pos, p1.mTangent, p2_pos, p2.mTangent, t_min, t_max, dist_sq);
			if (t >= 0.0f && t <= 1.0f && dist_sq < best_dist_sq)
			{
				best_t = float(i) + t;
				best_dist_sq = dist_sq;
			}
		}
	}

	return best_t;
}

void PathConstraintPathHermite::GetPointOnPath(float inFraction, Vec3 &outPathPosition, Vec3 &outPathTangent, Vec3 &outPathNormal, Vec3 &outPathBinormal) const
{
	MOSS_PROFILE_FUNCTION();

	// Determine which hermite spline segment we need
	int index;
	float t;
	GetIndexAndT(inFraction, index, t);

	// Get the points on the segment
	const Point &p1 = mPoints[index];
	const Point &p2 = mPoints[(index + 1) % int(mPoints.size())];

	// Calculate the position and tangent on the path
	Vec3 tangent;
	sCalculatePositionAndTangent(p1.mPosition, p1.mTangent, p2.mPosition, p2.mTangent, t, outPathPosition, tangent);
	outPathTangent = tangent.Normalized();

	// Just linearly interpolate the normal
	Vec3 normal = (1.0f - t) * p1.mNormal + t * p2.mNormal;

	// Calculate binormal
	outPathBinormal = normal.Cross(outPathTangent).Normalized();

	// Recalculate normal so it is perpendicular to both (linear interpolation will cause it not to be)
	outPathNormal = outPathTangent.Cross(outPathBinormal);
	MOSS_ASSERT(outPathNormal.IsNormalized());
}


#ifndef MOSS_DEBUG_RENDERER
// Helper function to transform the results of GetPointOnPath to world space
static inline void sTransformPathPoint(RMat44Arg inTransform, Vec3Arg inPosition, RVec3 &outPosition, Vec3 &ioNormal, Vec3 &ioBinormal)
{
	outPosition = inTransform * inPosition;
	ioNormal = inTransform.Multiply3x3(ioNormal);
	ioBinormal = inTransform.Multiply3x3(ioBinormal);
}

// Helper function to draw a path segment
static inline void sDrawPathSegment(DebugRenderer *inRenderer, RVec3Arg inPrevPosition, RVec3Arg inPosition, Vec3Arg inNormal, Vec3Arg inBinormal)
{
	inRenderer->DrawLine(inPrevPosition, inPosition, Color::sWhite);
	inRenderer->DrawArrow(inPosition, inPosition + 0.1f * inNormal, Color::sRed, 0.02f);
	inRenderer->DrawArrow(inPosition, inPosition + 0.1f * inBinormal, Color::sGreen, 0.02f);
}

void PathConstraintPath::DrawPath(DebugRenderer *inRenderer, RMat44Arg inBaseTransform) const
{
	// Calculate first point
	Vec3 lfirst_pos, first_tangent, first_normal, first_binormal;
	GetPointOnPath(0.0f, lfirst_pos, first_tangent, first_normal, first_binormal);
	RVec3 first_pos;
	sTransformPathPoint(inBaseTransform, lfirst_pos, first_pos, first_normal, first_binormal);

	float t_max = GetPathMaxFraction();

	// Draw the segments
	RVec3 prev_pos = first_pos;
	for (float t = 0.1f; t < t_max; t += 0.1f)
	{
		Vec3 lpos, tangent, normal, binormal;
		GetPointOnPath(t, lpos, tangent, normal, binormal);
		RVec3 pos;
		sTransformPathPoint(inBaseTransform, lpos, pos, normal, binormal);
		sDrawPathSegment(inRenderer, prev_pos, pos, normal, binormal);
		prev_pos = pos;
	}

	// Draw last point
	Vec3 lpos, tangent, normal, binormal;
	GetPointOnPath(t_max, lpos, tangent, normal, binormal);
	RVec3 pos;
	sTransformPathPoint(inBaseTransform, lpos, pos, normal, binormal);
	sDrawPathSegment(inRenderer, prev_pos, pos, normal, binormal);
}
#endif // MOSS_DEBUG_RENDERER

void PathConstraintPath::SaveBinaryState(StreamOut &inStream) const
{
	inStream.Write(GetRTTI()->GetHash());
	inStream.Write(mIsLooping);
}

void PathConstraintPath::RestoreBinaryState(StreamIn &inStream)
{
	// Type hash read by sRestoreFromBinaryState
	inStream.Read(mIsLooping);
}

PathConstraintPath::PathResult PathConstraintPath::sRestoreFromBinaryState(StreamIn &inStream)
{
	return StreamUtils::RestoreObject<PathConstraintPath>(inStream, &PathConstraintPath::RestoreBinaryState);
}


TwoBodyConstraint *PointConstraintSettings::Create(Body &inBody1, Body &inBody2) const
{
	return new PointConstraint(inBody1, inBody2, *this);
}

PointConstraint::PointConstraint(Body &inBody1, Body &inBody2, const PointConstraintSettings &inSettings) :
	TwoBodyConstraint(inBody1, inBody2, inSettings)
{
	if (inSettings.mSpace == EConstraintSpace::WorldSpace)
	{
		// If all properties were specified in world space, take them to local space now
		mLocalSpacePosition1 = Vec3(inBody1.GetInverseCenterOfMassTransform() * inSettings.mPoint1);
		mLocalSpacePosition2 = Vec3(inBody2.GetInverseCenterOfMassTransform() * inSettings.mPoint2);
	}
	else
	{
		mLocalSpacePosition1 = Vec3(inSettings.mPoint1);
		mLocalSpacePosition2 = Vec3(inSettings.mPoint2);
	}
}

void PointConstraint::NotifyShapeChanged(const BodyID &inBodyID, Vec3Arg inDeltaCOM)
{
	if (mBody1->GetID() == inBodyID)
		mLocalSpacePosition1 -= inDeltaCOM;
	else if (mBody2->GetID() == inBodyID)
		mLocalSpacePosition2 -= inDeltaCOM;
}

void PointConstraint::SetPoint1(EConstraintSpace inSpace, RVec3Arg inPoint1)
{
	if (inSpace == EConstraintSpace::WorldSpace)
		mLocalSpacePosition1 = Vec3(mBody1->GetInverseCenterOfMassTransform() * inPoint1);
	else
		mLocalSpacePosition1 = Vec3(inPoint1);
}

void PointConstraint::SetPoint2(EConstraintSpace inSpace, RVec3Arg inPoint2)
{
	if (inSpace == EConstraintSpace::WorldSpace)
		mLocalSpacePosition2 = Vec3(mBody2->GetInverseCenterOfMassTransform() * inPoint2);
	else
		mLocalSpacePosition2 = Vec3(inPoint2);
}

void PointConstraint::CalculateConstraintProperties()
{
	mPointConstraintPart.CalculateConstraintProperties(*mBody1, Mat44::sRotation(mBody1->GetRotation()), mLocalSpacePosition1, *mBody2, Mat44::sRotation(mBody2->GetRotation()), mLocalSpacePosition2);
}

void PointConstraint::SetupVelocityConstraint(float inDeltaTime)
{
	CalculateConstraintProperties();
}

void PointConstraint::ResetWarmStart()
{
	mPointConstraintPart.Deactivate();
}

void PointConstraint::WarmStartVelocityConstraint(float inWarmStartImpulseRatio)
{
	// Warm starting: Apply previous frame impulse
	mPointConstraintPart.WarmStart(*mBody1, *mBody2, inWarmStartImpulseRatio);
}

bool PointConstraint::SolveVelocityConstraint(float inDeltaTime)
{
	return mPointConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2);
}

bool PointConstraint::SolvePositionConstraint(float inDeltaTime, float inBaumgarte)
{
	// Update constraint properties (bodies may have moved)
	CalculateConstraintProperties();

	return mPointConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, inBaumgarte);
}

#ifndef MOSS_DEBUG_RENDERER
void PointConstraint::DrawConstraint(DebugRenderer *inRenderer) const
{
	// Draw constraint
	inRenderer->DrawMarker(mBody1->GetCenterOfMassTransform() * mLocalSpacePosition1, Color::sRed, 0.1f);
	inRenderer->DrawMarker(mBody2->GetCenterOfMassTransform() * mLocalSpacePosition2, Color::sGreen, 0.1f);
}
#endif // MOSS_DEBUG_RENDERER

void PointConstraint::SaveState(StateRecorder &inStream) const
{
	TwoBodyConstraint::SaveState(inStream);

	mPointConstraintPart.SaveState(inStream);
}

void PointConstraint::RestoreState(StateRecorder &inStream)
{
	TwoBodyConstraint::RestoreState(inStream);

	mPointConstraintPart.RestoreState(inStream);
}

Ref<ConstraintSettings> PointConstraint::GetConstraintSettings() const
{
	PointConstraintSettings *settings = new PointConstraintSettings;
	ToConstraintSettings(*settings);
	settings->mSpace = EConstraintSpace::LocalToBodyCOM;
	settings->mPoint1 = RVec3(mLocalSpacePosition1);
	settings->mPoint2 = RVec3(mLocalSpacePosition2);
	return settings;
}


void TwoBodyConstraint::BuildIslands(uint32 inConstraintIndex, IslandBuilder &ioBuilder, BodyManager &inBodyManager)
{
	// Activate bodies
	BodyID body_ids[2];
	int num_bodies = 0;
	if (mBody1->IsDynamic() && !mBody1->IsActive())
		body_ids[num_bodies++] = mBody1->GetID();
	if (mBody2->IsDynamic() && !mBody2->IsActive())
		body_ids[num_bodies++] = mBody2->GetID();
	if (num_bodies > 0)
		inBodyManager.ActivateBodies(body_ids, num_bodies);

	// Link the bodies into the same island
	ioBuilder.LinkConstraint(inConstraintIndex, mBody1->GetIndexInActiveBodiesInternal(), mBody2->GetIndexInActiveBodiesInternal());
}

uint TwoBodyConstraint::BuildIslandSplits(LargeIslandSplitter &ioSplitter) const
{
	return ioSplitter.AssignSplit(mBody1, mBody2);
}

#ifndef MOSS_DEBUG_RENDERER

void TwoBodyConstraint::DrawConstraintReferenceFrame(DebugRenderer *inRenderer) const
{
	RMat44 transform1 = mBody1->GetCenterOfMassTransform() * GetConstraintToBody1Matrix();
	RMat44 transform2 = mBody2->GetCenterOfMassTransform() * GetConstraintToBody2Matrix();
	inRenderer->DrawCoordinateSystem(transform1, 1.1f * mDrawConstraintSize);
	inRenderer->DrawCoordinateSystem(transform2, mDrawConstraintSize);
}

#endif // MOSS_DEBUG_RENDERER


void SliderConstraintSettings::SetSliderAxis(Vec3Arg inSliderAxis)
{
	MOSS_ASSERT(mSpace == EConstraintSpace::WorldSpace);

	mSliderAxis1 = mSliderAxis2 = inSliderAxis;
	mNormalAxis1 = mNormalAxis2 = inSliderAxis.GetNormalizedPerpendicular();
}

void SliderConstraintSettings::SaveBinaryState(StreamOut &inStream) const
{
	ConstraintSettings::SaveBinaryState(inStream);

	inStream.Write(mSpace);
	inStream.Write(mAutoDetectPoint);
	inStream.Write(mPoint1);
	inStream.Write(mSliderAxis1);
	inStream.Write(mNormalAxis1);
	inStream.Write(mPoint2);
	inStream.Write(mSliderAxis2);
	inStream.Write(mNormalAxis2);
	inStream.Write(mLimitsMin);
	inStream.Write(mLimitsMax);
	inStream.Write(mMaxFrictionForce);
	mLimitsSpringSettings.SaveBinaryState(inStream);
	mMotorSettings.SaveBinaryState(inStream);
}

void SliderConstraintSettings::RestoreBinaryState(StreamIn &inStream)
{
	ConstraintSettings::RestoreBinaryState(inStream);

	inStream.Read(mSpace);
	inStream.Read(mAutoDetectPoint);
	inStream.Read(mPoint1);
	inStream.Read(mSliderAxis1);
	inStream.Read(mNormalAxis1);
	inStream.Read(mPoint2);
	inStream.Read(mSliderAxis2);
	inStream.Read(mNormalAxis2);
	inStream.Read(mLimitsMin);
	inStream.Read(mLimitsMax);
	inStream.Read(mMaxFrictionForce);
	mLimitsSpringSettings.RestoreBinaryState(inStream);
	mMotorSettings.RestoreBinaryState(inStream);
}

TwoBodyConstraint *SliderConstraintSettings::Create(Body &inBody1, Body &inBody2) const
{
	return new SliderConstraint(inBody1, inBody2, *this);
}

SliderConstraint::SliderConstraint(Body &inBody1, Body &inBody2, const SliderConstraintSettings &inSettings) :
	TwoBodyConstraint(inBody1, inBody2, inSettings),
	mMaxFrictionForce(inSettings.mMaxFrictionForce),
	mMotorSettings(inSettings.mMotorSettings)
{
	// Store inverse of initial rotation from body 1 to body 2 in body 1 space
	mInvInitialOrientation = RotationEulerConstraintPart::sGetInvInitialOrientationXY(inSettings.mSliderAxis1, inSettings.mNormalAxis1, inSettings.mSliderAxis2, inSettings.mNormalAxis2);

	if (inSettings.mSpace == EConstraintSpace::WorldSpace)
	{
		RMat44 inv_transform1 = inBody1.GetInverseCenterOfMassTransform();
		RMat44 inv_transform2 = inBody2.GetInverseCenterOfMassTransform();

		if (inSettings.mAutoDetectPoint)
		{
			// Determine anchor point: If any of the bodies can never be dynamic use the other body as anchor point
			RVec3 anchor;
			if (!inBody1.CanBeKinematicOrDynamic())
				anchor = inBody2.GetCenterOfMassPosition();
			else if (!inBody2.CanBeKinematicOrDynamic())
				anchor = inBody1.GetCenterOfMassPosition();
			else
			{
				// Otherwise use weighted anchor point towards the lightest body
				Real inv_m1 = Real(inBody1.GetMotionPropertiesUnchecked()->GetInverseMassUnchecked());
				Real inv_m2 = Real(inBody2.GetMotionPropertiesUnchecked()->GetInverseMassUnchecked());
				Real total_inv_mass = inv_m1 + inv_m2;
				if (total_inv_mass != 0.0_r)
					anchor = (inv_m1 * inBody1.GetCenterOfMassPosition() + inv_m2 * inBody2.GetCenterOfMassPosition()) / total_inv_mass;
				else
					anchor = inBody1.GetCenterOfMassPosition();
			}

			// Store local positions
			mLocalSpacePosition1 = Vec3(inv_transform1 * anchor);
			mLocalSpacePosition2 = Vec3(inv_transform2 * anchor);
		}
		else
		{
			// Store local positions
			mLocalSpacePosition1 = Vec3(inv_transform1 * inSettings.mPoint1);
			mLocalSpacePosition2 = Vec3(inv_transform2 * inSettings.mPoint2);
		}

		// If all properties were specified in world space, take them to local space now
		mLocalSpaceSliderAxis1 = inv_transform1.Multiply3x3(inSettings.mSliderAxis1).Normalized();
		mLocalSpaceNormal1 = inv_transform1.Multiply3x3(inSettings.mNormalAxis1).Normalized();

		// Constraints were specified in world space, so we should have replaced c1 with q10^-1 c1 and c2 with q20^-1 c2
		// => r0^-1 = (q20^-1 c2) (q10^-1 c1)^1 = q20^-1 (c2 c1^-1) q10
		mInvInitialOrientation = inBody2.GetRotation().Conjugated() * mInvInitialOrientation * inBody1.GetRotation();
	}
	else
	{
		// Store local positions
		mLocalSpacePosition1 = Vec3(inSettings.mPoint1);
		mLocalSpacePosition2 = Vec3(inSettings.mPoint2);

		// Store local space axis
		mLocalSpaceSliderAxis1 = inSettings.mSliderAxis1;
		mLocalSpaceNormal1 = inSettings.mNormalAxis1;
	}

	// Calculate 2nd local space normal
	mLocalSpaceNormal2 = mLocalSpaceSliderAxis1.Cross(mLocalSpaceNormal1);

	// Store limits
	MOSS_ASSERT(inSettings.mLimitsMin != inSettings.mLimitsMax || inSettings.mLimitsSpringSettings.mFrequency > 0.0f, "Better use a fixed constraint");
	SetLimits(inSettings.mLimitsMin, inSettings.mLimitsMax);

	// Store spring settings
	SetLimitsSpringSettings(inSettings.mLimitsSpringSettings);
}

void SliderConstraint::NotifyShapeChanged(const BodyID &inBodyID, Vec3Arg inDeltaCOM)
{
	if (mBody1->GetID() == inBodyID)
		mLocalSpacePosition1 -= inDeltaCOM;
	else if (mBody2->GetID() == inBodyID)
		mLocalSpacePosition2 -= inDeltaCOM;
}

float SliderConstraint::GetCurrentPosition() const
{
	// See: CalculateR1R2U and CalculateSlidingAxisAndPosition
	Vec3 r1 = mBody1->GetRotation() * mLocalSpacePosition1;
	Vec3 r2 = mBody2->GetRotation() * mLocalSpacePosition2;
	Vec3 u = Vec3(mBody2->GetCenterOfMassPosition() - mBody1->GetCenterOfMassPosition()) + r2 - r1;
	return u.Dot(mBody1->GetRotation() * mLocalSpaceSliderAxis1);
}

void SliderConstraint::SetLimits(float inLimitsMin, float inLimitsMax)
{
	MOSS_ASSERT(inLimitsMin <= 0.0f);
	MOSS_ASSERT(inLimitsMax >= 0.0f);
	mLimitsMin = inLimitsMin;
	mLimitsMax = inLimitsMax;
	mHasLimits = mLimitsMin != -FLT_MAX || mLimitsMax != FLT_MAX;
}

void SliderConstraint::CalculateR1R2U(Mat44Arg inRotation1, Mat44Arg inRotation2)
{
	// Calculate points relative to body
	mR1 = inRotation1 * mLocalSpacePosition1;
	mR2 = inRotation2 * mLocalSpacePosition2;

	// Calculate X2 + R2 - X1 - R1
	mU = Vec3(mBody2->GetCenterOfMassPosition() - mBody1->GetCenterOfMassPosition()) + mR2 - mR1;
}

void SliderConstraint::CalculatePositionConstraintProperties(Mat44Arg inRotation1, Mat44Arg inRotation2)
{
	// Calculate world space normals
	mN1 = inRotation1 * mLocalSpaceNormal1;
	mN2 = inRotation1 * mLocalSpaceNormal2;

	mPositionConstraintPart.CalculateConstraintProperties(*mBody1, inRotation1, mR1 + mU, *mBody2, inRotation2, mR2, mN1, mN2);
}

void SliderConstraint::CalculateSlidingAxisAndPosition(Mat44Arg inRotation1)
{
	if (mHasLimits || mMotorState != EMotorState::Off || mMaxFrictionForce > 0.0f)
	{
		// Calculate world space slider axis
		mWorldSpaceSliderAxis = inRotation1 * mLocalSpaceSliderAxis1;

		// Calculate slide distance along axis
		mD = mU.Dot(mWorldSpaceSliderAxis);
	}
}

void SliderConstraint::CalculatePositionLimitsConstraintProperties(float inDeltaTime)
{
	// Check if distance is within limits
	bool below_min = mD <= mLimitsMin;
	if (mHasLimits && (below_min || mD >= mLimitsMax))
		mPositionLimitsConstraintPart.CalculateConstraintPropertiesWithSettings(inDeltaTime, *mBody1, mR1 + mU, *mBody2, mR2, mWorldSpaceSliderAxis, 0.0f, mD - (below_min? mLimitsMin : mLimitsMax), mLimitsSpringSettings);
	else
		mPositionLimitsConstraintPart.Deactivate();
}

void SliderConstraint::CalculateMotorConstraintProperties(float inDeltaTime)
{
	switch (mMotorState)
	{
	case EMotorState::Off:
		if (mMaxFrictionForce > 0.0f)
			mMotorConstraintPart.CalculateConstraintProperties(*mBody1, mR1 + mU, *mBody2, mR2, mWorldSpaceSliderAxis);
		else
			mMotorConstraintPart.Deactivate();
		break;

	case EMotorState::Velocity:
		mMotorConstraintPart.CalculateConstraintProperties(*mBody1, mR1 + mU, *mBody2, mR2, mWorldSpaceSliderAxis, -mTargetVelocity);
		break;

	case EMotorState::Position:
		if (mMotorSettings.mSpringSettings.HasStiffness())
			mMotorConstraintPart.CalculateConstraintPropertiesWithSettings(inDeltaTime, *mBody1, mR1 + mU, *mBody2, mR2, mWorldSpaceSliderAxis, 0.0f, mD - mTargetPosition, mMotorSettings.mSpringSettings);
		else
			mMotorConstraintPart.Deactivate();
		break;
	}
}

void SliderConstraint::SetupVelocityConstraint(float inDeltaTime)
{
	// Calculate constraint properties that are constant while bodies don't move
	Mat44 rotation1 = Mat44::sRotation(mBody1->GetRotation());
	Mat44 rotation2 = Mat44::sRotation(mBody2->GetRotation());
	CalculateR1R2U(rotation1, rotation2);
	CalculatePositionConstraintProperties(rotation1, rotation2);
	mRotationConstraintPart.CalculateConstraintProperties(*mBody1, rotation1, *mBody2, rotation2);
	CalculateSlidingAxisAndPosition(rotation1);
	CalculatePositionLimitsConstraintProperties(inDeltaTime);
	CalculateMotorConstraintProperties(inDeltaTime);
}

void SliderConstraint::ResetWarmStart()
{
	mMotorConstraintPart.Deactivate();
	mPositionConstraintPart.Deactivate();
	mRotationConstraintPart.Deactivate();
	mPositionLimitsConstraintPart.Deactivate();
}

void SliderConstraint::WarmStartVelocityConstraint(float inWarmStartImpulseRatio)
{
	// Warm starting: Apply previous frame impulse
	mMotorConstraintPart.WarmStart(*mBody1, *mBody2, mWorldSpaceSliderAxis, inWarmStartImpulseRatio);
	mPositionConstraintPart.WarmStart(*mBody1, *mBody2, mN1, mN2, inWarmStartImpulseRatio);
	mRotationConstraintPart.WarmStart(*mBody1, *mBody2, inWarmStartImpulseRatio);
	mPositionLimitsConstraintPart.WarmStart(*mBody1, *mBody2, mWorldSpaceSliderAxis, inWarmStartImpulseRatio);
}

bool SliderConstraint::SolveVelocityConstraint(float inDeltaTime)
{
	// Solve motor
	bool motor = false;
	if (mMotorConstraintPart.IsActive())
	{
		switch (mMotorState)
		{
		case EMotorState::Off:
			{
				float max_lambda = mMaxFrictionForce * inDeltaTime;
				motor = mMotorConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2, mWorldSpaceSliderAxis, -max_lambda, max_lambda);
				break;
			}

		case EMotorState::Velocity:
		case EMotorState::Position:
			motor = mMotorConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2, mWorldSpaceSliderAxis, inDeltaTime * mMotorSettings.mMinForceLimit, inDeltaTime * mMotorSettings.mMaxForceLimit);
			break;
		}
	}

	// Solve position constraint along 2 axis
	bool pos = mPositionConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2, mN1, mN2);

	// Solve rotation constraint
	bool rot = mRotationConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2);

	// Solve limits along slider axis
	bool limit = false;
	if (mPositionLimitsConstraintPart.IsActive())
	{
		float min_lambda, max_lambda;
		if (mLimitsMin == mLimitsMax)
		{
			min_lambda = -FLT_MAX;
			max_lambda = FLT_MAX;
		}
		else if (mD <= mLimitsMin)
		{
			min_lambda = 0.0f;
			max_lambda = FLT_MAX;
		}
		else
		{
			min_lambda = -FLT_MAX;
			max_lambda = 0.0f;
		}
		limit = mPositionLimitsConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2, mWorldSpaceSliderAxis, min_lambda, max_lambda);
	}

	return motor || pos || rot || limit;
}

bool SliderConstraint::SolvePositionConstraint(float inDeltaTime, float inBaumgarte)
{
	// Motor operates on velocities only, don't call SolvePositionConstraint

	// Solve position constraint along 2 axis
	Mat44 rotation1 = Mat44::sRotation(mBody1->GetRotation());
	Mat44 rotation2 = Mat44::sRotation(mBody2->GetRotation());
	CalculateR1R2U(rotation1, rotation2);
	CalculatePositionConstraintProperties(rotation1, rotation2);
	bool pos = mPositionConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, mU, mN1, mN2, inBaumgarte);

	// Solve rotation constraint
	mRotationConstraintPart.CalculateConstraintProperties(*mBody1, Mat44::sRotation(mBody1->GetRotation()), *mBody2, Mat44::sRotation(mBody2->GetRotation()));
	bool rot = mRotationConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, mInvInitialOrientation, inBaumgarte);

	// Solve limits along slider axis
	bool limit = false;
	if (mHasLimits && mLimitsSpringSettings.mFrequency <= 0.0f)
	{
		rotation1 = Mat44::sRotation(mBody1->GetRotation());
		rotation2 = Mat44::sRotation(mBody2->GetRotation());
		CalculateR1R2U(rotation1, rotation2);
		CalculateSlidingAxisAndPosition(rotation1);
		CalculatePositionLimitsConstraintProperties(inDeltaTime);
		if (mPositionLimitsConstraintPart.IsActive())
		{
			if (mD <= mLimitsMin)
				limit = mPositionLimitsConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, mWorldSpaceSliderAxis, mD - mLimitsMin, inBaumgarte);
			else
			{
				MOSS_ASSERT(mD >= mLimitsMax);
				limit = mPositionLimitsConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, mWorldSpaceSliderAxis, mD - mLimitsMax, inBaumgarte);
			}
		}
	}

	return pos || rot || limit;
}

#ifndef MOSS_DEBUG_RENDERER
void SliderConstraint::DrawConstraint(DebugRenderer *inRenderer) const
{
	RMat44 transform1 = mBody1->GetCenterOfMassTransform();
	RMat44 transform2 = mBody2->GetCenterOfMassTransform();

	// Transform the local positions into world space
	Vec3 slider_axis = transform1.Multiply3x3(mLocalSpaceSliderAxis1);
	RVec3 position1 = transform1 * mLocalSpacePosition1;
	RVec3 position2 = transform2 * mLocalSpacePosition2;

	// Draw constraint
	inRenderer->DrawMarker(position1, Color::sRed, 0.1f);
	inRenderer->DrawMarker(position2, Color::sGreen, 0.1f);
	inRenderer->DrawLine(position1, position2, Color::sGreen);

	// Draw motor
	switch (mMotorState)
	{
	case EMotorState::Position:
		inRenderer->DrawMarker(position1 + mTargetPosition * slider_axis, Color::sYellow, 1.0f);
		break;

	case EMotorState::Velocity:
		{
			Vec3 cur_vel = (mBody2->GetLinearVelocity() - mBody1->GetLinearVelocity()).Dot(slider_axis) * slider_axis;
			inRenderer->DrawLine(position2, position2 + cur_vel, Color::sBlue);
			inRenderer->DrawArrow(position2 + cur_vel, position2 + mTargetVelocity * slider_axis, Color::sRed, 0.1f);
			break;
		}

	case EMotorState::Off:
		break;
	}
}

void SliderConstraint::DrawConstraintLimits(DebugRenderer *inRenderer) const
{
	if (mHasLimits)
	{
		RMat44 transform1 = mBody1->GetCenterOfMassTransform();
		RMat44 transform2 = mBody2->GetCenterOfMassTransform();

		// Transform the local positions into world space
		Vec3 slider_axis = transform1.Multiply3x3(mLocalSpaceSliderAxis1);
		RVec3 position1 = transform1 * mLocalSpacePosition1;
		RVec3 position2 = transform2 * mLocalSpacePosition2;

		// Calculate the limits in world space
		RVec3 limits_min = position1 + mLimitsMin * slider_axis;
		RVec3 limits_max = position1 + mLimitsMax * slider_axis;

		inRenderer->DrawLine(limits_min, position1, Color::sWhite);
		inRenderer->DrawLine(position2, limits_max, Color::sWhite);

		inRenderer->DrawMarker(limits_min, Color::sWhite, 0.1f);
		inRenderer->DrawMarker(limits_max, Color::sWhite, 0.1f);
	}
}
#endif // MOSS_DEBUG_RENDERER

void SliderConstraint::SaveState(StateRecorder &inStream) const
{
	TwoBodyConstraint::SaveState(inStream);

	mMotorConstraintPart.SaveState(inStream);
	mPositionConstraintPart.SaveState(inStream);
	mRotationConstraintPart.SaveState(inStream);
	mPositionLimitsConstraintPart.SaveState(inStream);

	inStream.Write(mMotorState);
	inStream.Write(mTargetVelocity);
	inStream.Write(mTargetPosition);
}

void SliderConstraint::RestoreState(StateRecorder &inStream)
{
	TwoBodyConstraint::RestoreState(inStream);

	mMotorConstraintPart.RestoreState(inStream);
	mPositionConstraintPart.RestoreState(inStream);
	mRotationConstraintPart.RestoreState(inStream);
	mPositionLimitsConstraintPart.RestoreState(inStream);

	inStream.Read(mMotorState);
	inStream.Read(mTargetVelocity);
	inStream.Read(mTargetPosition);
}

Ref<ConstraintSettings> SliderConstraint::GetConstraintSettings() const
{
	SliderConstraintSettings *settings = new SliderConstraintSettings;
	ToConstraintSettings(*settings);
	settings->mSpace = EConstraintSpace::LocalToBodyCOM;
	settings->mPoint1 = RVec3(mLocalSpacePosition1);
	settings->mSliderAxis1 = mLocalSpaceSliderAxis1;
	settings->mNormalAxis1 = mLocalSpaceNormal1;
	settings->mPoint2 = RVec3(mLocalSpacePosition2);
	Mat44 inv_initial_rotation = Mat44::sRotation(mInvInitialOrientation);
	settings->mSliderAxis2 = inv_initial_rotation.Multiply3x3(mLocalSpaceSliderAxis1);
	settings->mNormalAxis2 = inv_initial_rotation.Multiply3x3(mLocalSpaceNormal1);
	settings->mLimitsMin = mLimitsMin;
	settings->mLimitsMax = mLimitsMax;
	settings->mLimitsSpringSettings = mLimitsSpringSettings;
	settings->mMaxFrictionForce = mMaxFrictionForce;
	settings->mMotorSettings = mMotorSettings;
	return settings;
}

Mat44 SliderConstraint::GetConstraintToBody1Matrix() const
{
	return Mat44(Vec4(mLocalSpaceSliderAxis1, 0), Vec4(mLocalSpaceNormal1, 0), Vec4(mLocalSpaceNormal2, 0), Vec4(mLocalSpacePosition1, 1));
}

Mat44 SliderConstraint::GetConstraintToBody2Matrix() const
{
	Mat44 mat = Mat44::sRotation(mInvInitialOrientation).Multiply3x3(Mat44(Vec4(mLocalSpaceSliderAxis1, 0), Vec4(mLocalSpaceNormal1, 0), Vec4(mLocalSpaceNormal2, 0), Vec4(0, 0, 0, 1)));
	mat.SetTranslation(mLocalSpacePosition2);
	return mat;
}

TwoBodyConstraint *SwingTwistConstraintSettings::Create(Body &inBody1, Body &inBody2) const
{
	return new SwingTwistConstraint(inBody1, inBody2, *this);
}

void SwingTwistConstraint::UpdateLimits()
{
	// Pass limits on to swing twist constraint part
	mSwingTwistConstraintPart.SetLimits(mTwistMinAngle, mTwistMaxAngle, -mPlaneHalfConeAngle, mPlaneHalfConeAngle, -mNormalHalfConeAngle, mNormalHalfConeAngle);
}

SwingTwistConstraint::SwingTwistConstraint(Body &inBody1, Body &inBody2, const SwingTwistConstraintSettings &inSettings) :
	TwoBodyConstraint(inBody1, inBody2, inSettings),
	mNormalHalfConeAngle(inSettings.mNormalHalfConeAngle),
	mPlaneHalfConeAngle(inSettings.mPlaneHalfConeAngle),
	mTwistMinAngle(inSettings.mTwistMinAngle),
	mTwistMaxAngle(inSettings.mTwistMaxAngle),
	mMaxFrictionTorque(inSettings.mMaxFrictionTorque),
	mSwingMotorSettings(inSettings.mSwingMotorSettings),
	mTwistMotorSettings(inSettings.mTwistMotorSettings)
{
	// Override swing type
	mSwingTwistConstraintPart.SetSwingType(inSettings.mSwingType);

	// Calculate rotation needed to go from constraint space to body1 local space
	Vec3 normal_axis1 = inSettings.mPlaneAxis1.Cross(inSettings.mTwistAxis1);
	Mat44 c_to_b1(Vec4(inSettings.mTwistAxis1, 0), Vec4(normal_axis1, 0), Vec4(inSettings.mPlaneAxis1, 0), Vec4(0, 0, 0, 1));
	mConstraintToBody1 = c_to_b1.GetQuaternion();

	// Calculate rotation needed to go from constraint space to body2 local space
	Vec3 normal_axis2 = inSettings.mPlaneAxis2.Cross(inSettings.mTwistAxis2);
	Mat44 c_to_b2(Vec4(inSettings.mTwistAxis2, 0), Vec4(normal_axis2, 0), Vec4(inSettings.mPlaneAxis2, 0), Vec4(0, 0, 0, 1));
	mConstraintToBody2 = c_to_b2.GetQuaternion();

	if (inSettings.mSpace == EConstraintSpace::WorldSpace)
	{
		// If all properties were specified in world space, take them to local space now
		mLocalSpacePosition1 = Vec3(inBody1.GetInverseCenterOfMassTransform() * inSettings.mPosition1);
		mConstraintToBody1 = inBody1.GetRotation().Conjugated() * mConstraintToBody1;

		mLocalSpacePosition2 = Vec3(inBody2.GetInverseCenterOfMassTransform() * inSettings.mPosition2);
		mConstraintToBody2 = inBody2.GetRotation().Conjugated() * mConstraintToBody2;
	}
	else
	{
		mLocalSpacePosition1 = Vec3(inSettings.mPosition1);
		mLocalSpacePosition2 = Vec3(inSettings.mPosition2);
	}

	UpdateLimits();
}

void SwingTwistConstraint::NotifyShapeChanged(const BodyID &inBodyID, Vec3Arg inDeltaCOM)
{
	if (mBody1->GetID() == inBodyID)
		mLocalSpacePosition1 -= inDeltaCOM;
	else if (mBody2->GetID() == inBodyID)
		mLocalSpacePosition2 -= inDeltaCOM;
}

Quat SwingTwistConstraint::GetRotationInConstraintSpace() const
{
	// Let b1, b2 be the center of mass transform of body1 and body2 (For body1 this is mBody1->GetCenterOfMassTransform())
	// Let c1, c2 be the transform that takes a vector from constraint space to local space of body1 and body2 (For body1 this is Mat44::sRotationTranslation(mConstraintToBody1, mLocalSpacePosition1))
	// Let q be the rotation of the constraint in constraint space
	// b2 takes a vector from the local space of body2 to world space
	// To express this in terms of b1: b2 = b1 * c1 * q * c2^-1
	// c2^-1 goes from local body 2 space to constraint space
	// q rotates the constraint
	// c1 goes from constraint space to body 1 local space
	// b1 goes from body 1 local space to world space
	// So when the body rotations are given, q = (b1 * c1)^-1 * b2 c2
	// Or: q = (q1 * c1)^-1 * (q2 * c2) if we're only interested in rotations
	Quat constraint_body1_to_world = mBody1->GetRotation() * mConstraintToBody1;
	Quat constraint_body2_to_world = mBody2->GetRotation() * mConstraintToBody2;
	return constraint_body1_to_world.Conjugated() * constraint_body2_to_world;
}

void SwingTwistConstraint::SetSwingMotorState(EMotorState inState)
{
	MOSS_ASSERT(inState == EMotorState::Off || mSwingMotorSettings.IsValid());

	if (mSwingMotorState != inState)
	{
		mSwingMotorState = inState;

		// Ensure that warm starting next frame doesn't apply any impulses (motor parts are repurposed for different modes)
		for (AngleConstraintPart &c : mMotorConstraintPart)
			c.Deactivate();
	}
}

void SwingTwistConstraint::SetTwistMotorState(EMotorState inState)
{
	MOSS_ASSERT(inState == EMotorState::Off || mTwistMotorSettings.IsValid());

	if (mTwistMotorState != inState)
	{
		mTwistMotorState = inState;

		// Ensure that warm starting next frame doesn't apply any impulses (motor parts are repurposed for different modes)
		mMotorConstraintPart[0].Deactivate();
	}
}

void SwingTwistConstraint::SetTargetOrientationCS(QuatArg inOrientation)
{
	Quat q_swing, q_twist;
	inOrientation.GetSwingTwist(q_swing, q_twist);

	uint clamped_axis;
	mSwingTwistConstraintPart.ClampSwingTwist(q_swing, q_twist, clamped_axis);

	if (clamped_axis != 0)
		mTargetOrientation = q_swing * q_twist;
	else
		mTargetOrientation = inOrientation;
}

void SwingTwistConstraint::SetupVelocityConstraint(float inDeltaTime)
{
	// Setup point constraint
	Mat44 rotation1 = Mat44::sRotation(mBody1->GetRotation());
	Mat44 rotation2 = Mat44::sRotation(mBody2->GetRotation());
	mPointConstraintPart.CalculateConstraintProperties(*mBody1, rotation1, mLocalSpacePosition1, *mBody2, rotation2, mLocalSpacePosition2);

	// GetRotationInConstraintSpace written out since we reuse the sub expressions
	Quat constraint_body1_to_world = mBody1->GetRotation() * mConstraintToBody1;
	Quat constraint_body2_to_world = mBody2->GetRotation() * mConstraintToBody2;
	Quat q = constraint_body1_to_world.Conjugated() * constraint_body2_to_world;

	// Calculate constraint properties for the swing twist limit
	mSwingTwistConstraintPart.CalculateConstraintProperties(*mBody1, *mBody2, q, constraint_body1_to_world);

	if (mSwingMotorState != EMotorState::Off || mTwistMotorState != EMotorState::Off || mMaxFrictionTorque > 0.0f)
	{
		// Calculate rotation motor axis
		Mat44 ws_axis = Mat44::sRotation(constraint_body2_to_world);
		for (int i = 0; i < 3; ++i)
			mWorldSpaceMotorAxis[i] = ws_axis.GetColumn3(i);

		Vec3 rotation_error;
		if (mSwingMotorState == EMotorState::Position || mTwistMotorState == EMotorState::Position)
		{
			// Get target orientation along the shortest path from q
			Quat target_orientation = q.Dot(mTargetOrientation) > 0.0f? mTargetOrientation : -mTargetOrientation;

			// The definition of the constraint rotation q:
			// R2 * ConstraintToBody2 = R1 * ConstraintToBody1 * q (1)
			//
			// R2' is the rotation of body 2 when reaching the target_orientation:
			// R2' * ConstraintToBody2 = R1 * ConstraintToBody1 * target_orientation (2)
			//
			// The difference in body 2 space:
			// R2' = R2 * diff_body2 (3)
			//
			// We want to specify the difference in the constraint space of body 2:
			// diff_body2 = ConstraintToBody2 * diff * ConstraintToBody2^* (4)
			//
			// Extracting R2' from 2: R2' = R1 * ConstraintToBody1 * target_orientation * ConstraintToBody2^* (5)
			// Combining 3 & 4: R2' = R2 * ConstraintToBody2 * diff * ConstraintToBody2^* (6)
			// Combining 1 & 6: R2' = R1 * ConstraintToBody1 * q * diff * ConstraintToBody2^* (7)
			// Combining 5 & 7: R1 * ConstraintToBody1 * target_orientation * ConstraintToBody2^* = R1 * ConstraintToBody1 * q * diff * ConstraintToBody2^*
			// <=> target_orientation = q * diff
			// <=> diff = q^* * target_orientation
			Quat diff = q.Conjugated() * target_orientation;

			// Approximate error angles
			// The imaginary part of a quaternion is rotation_axis * sin(angle / 2)
			// If angle is small, sin(x) = x so angle[i] ~ 2.0f * rotation_axis[i]
			// We'll be making small time steps, so if the angle is not small at least the sign will be correct and we'll move in the right direction
			rotation_error = -2.0f * diff.GetXYZ();
		}

		// Swing motor
		switch (mSwingMotorState)
		{
		case EMotorState::Off:
			if (mMaxFrictionTorque > 0.0f)
			{
				// Enable friction
				for (int i = 1; i < 3; ++i)
					mMotorConstraintPart[i].CalculateConstraintProperties(*mBody1, *mBody2, mWorldSpaceMotorAxis[i], 0.0f);
			}
			else
			{
				// Disable friction
				for (AngleConstraintPart &c : mMotorConstraintPart)
					c.Deactivate();
			}
			break;

		case EMotorState::Velocity:
			// Use motor to create angular velocity around desired axis
			for (int i = 1; i < 3; ++i)
				mMotorConstraintPart[i].CalculateConstraintProperties(*mBody1, *mBody2, mWorldSpaceMotorAxis[i], -mTargetAngularVelocity[i]);
			break;

		case EMotorState::Position:
			// Use motor to drive rotation error to zero
			if (mSwingMotorSettings.mSpringSettings.HasStiffness())
			{
				for (int i = 1; i < 3; ++i)
					mMotorConstraintPart[i].CalculateConstraintPropertiesWithSettings(inDeltaTime, *mBody1, *mBody2, mWorldSpaceMotorAxis[i], 0.0f, rotation_error[i], mSwingMotorSettings.mSpringSettings);
			}
			else
			{
				for (int i = 1; i < 3; ++i)
					mMotorConstraintPart[i].Deactivate();
			}
			break;
		}

		// Twist motor
		switch (mTwistMotorState)
		{
		case EMotorState::Off:
			if (mMaxFrictionTorque > 0.0f)
			{
				// Enable friction
				mMotorConstraintPart[0].CalculateConstraintProperties(*mBody1, *mBody2, mWorldSpaceMotorAxis[0], 0.0f);
			}
			else
			{
				// Disable friction
				mMotorConstraintPart[0].Deactivate();
			}
			break;

		case EMotorState::Velocity:
			// Use motor to create angular velocity around desired axis
			mMotorConstraintPart[0].CalculateConstraintProperties(*mBody1, *mBody2, mWorldSpaceMotorAxis[0], -mTargetAngularVelocity[0]);
			break;

		case EMotorState::Position:
			// Use motor to drive rotation error to zero
			if (mTwistMotorSettings.mSpringSettings.HasStiffness())
				mMotorConstraintPart[0].CalculateConstraintPropertiesWithSettings(inDeltaTime, *mBody1, *mBody2, mWorldSpaceMotorAxis[0], 0.0f, rotation_error[0], mTwistMotorSettings.mSpringSettings);
			else
				mMotorConstraintPart[0].Deactivate();
			break;
		}
	}
	else
	{
		// Disable rotation motor
		for (AngleConstraintPart &c : mMotorConstraintPart)
			c.Deactivate();
	}
}

void SwingTwistConstraint::ResetWarmStart()
{
	for (AngleConstraintPart &c : mMotorConstraintPart)
		c.Deactivate();
	mSwingTwistConstraintPart.Deactivate();
	mPointConstraintPart.Deactivate();
}

void SwingTwistConstraint::WarmStartVelocityConstraint(float inWarmStartImpulseRatio)
{
	// Warm starting: Apply previous frame impulse
	for (AngleConstraintPart &c : mMotorConstraintPart)
		c.WarmStart(*mBody1, *mBody2, inWarmStartImpulseRatio);
	mSwingTwistConstraintPart.WarmStart(*mBody1, *mBody2, inWarmStartImpulseRatio);
	mPointConstraintPart.WarmStart(*mBody1, *mBody2, inWarmStartImpulseRatio);
}

bool SwingTwistConstraint::SolveVelocityConstraint(float inDeltaTime)
{
	bool impulse = false;

	// Solve twist rotation motor
	if (mMotorConstraintPart[0].IsActive())
	{
		// Twist limits
		float min_twist_limit, max_twist_limit;
		if (mTwistMotorState == EMotorState::Off)
		{
			max_twist_limit = inDeltaTime * mMaxFrictionTorque;
			min_twist_limit = -max_twist_limit;
		}
		else
		{
			min_twist_limit = inDeltaTime * mTwistMotorSettings.mMinTorqueLimit;
			max_twist_limit = inDeltaTime * mTwistMotorSettings.mMaxTorqueLimit;
		}

		impulse |= mMotorConstraintPart[0].SolveVelocityConstraint(*mBody1, *mBody2, mWorldSpaceMotorAxis[0], min_twist_limit, max_twist_limit);
	}

	// Solve swing rotation motor
	if (mMotorConstraintPart[1].IsActive())
	{
		// Swing parts should turn on / off together
		MOSS_ASSERT(mMotorConstraintPart[2].IsActive());

		// Swing limits
		float min_swing_limit, max_swing_limit;
		if (mSwingMotorState == EMotorState::Off)
		{
			max_swing_limit = inDeltaTime * mMaxFrictionTorque;
			min_swing_limit = -max_swing_limit;
		}
		else
		{
			min_swing_limit = inDeltaTime * mSwingMotorSettings.mMinTorqueLimit;
			max_swing_limit = inDeltaTime * mSwingMotorSettings.mMaxTorqueLimit;
		}

		for (int i = 1; i < 3; ++i)
			impulse |= mMotorConstraintPart[i].SolveVelocityConstraint(*mBody1, *mBody2, mWorldSpaceMotorAxis[i], min_swing_limit, max_swing_limit);
	}
	else
	{
		// Swing parts should turn on / off together
		MOSS_ASSERT(!mMotorConstraintPart[2].IsActive());
	}

	// Solve rotation limits
	impulse |= mSwingTwistConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2);

	// Solve position constraint
	impulse |= mPointConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2);

	return impulse;
}

bool SwingTwistConstraint::SolvePositionConstraint(float inDeltaTime, float inBaumgarte)
{
	bool impulse = false;

	// Solve rotation violations
	Quat q = GetRotationInConstraintSpace();
	impulse |= mSwingTwistConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, q, mConstraintToBody1, mConstraintToBody2, inBaumgarte);

	// Solve position violations
	mPointConstraintPart.CalculateConstraintProperties(*mBody1, Mat44::sRotation(mBody1->GetRotation()), mLocalSpacePosition1, *mBody2, Mat44::sRotation(mBody2->GetRotation()), mLocalSpacePosition2);
	impulse |= mPointConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, inBaumgarte);

	return impulse;
}

#ifndef MOSS_DEBUG_RENDERER
void SwingTwistConstraint::DrawConstraint(DebugRenderer *inRenderer) const
{
	// Get constraint properties in world space
	RMat44 transform1 = mBody1->GetCenterOfMassTransform();
	RVec3 position1 = transform1 * mLocalSpacePosition1;
	Quat rotation1 = mBody1->GetRotation() * mConstraintToBody1;
	Quat rotation2 = mBody2->GetRotation() * mConstraintToBody2;

	// Draw constraint orientation
	inRenderer->DrawCoordinateSystem(RMat44::sRotationTranslation(rotation1, position1), mDrawConstraintSize);

	// Draw current swing and twist
	Quat q = GetRotationInConstraintSpace();
	Quat q_swing, q_twist;
	q.GetSwingTwist(q_swing, q_twist);
	inRenderer->DrawLine(position1, position1 + mDrawConstraintSize * (rotation1 * q_twist).RotateAxisY(), Color::sWhite);
	inRenderer->DrawLine(position1, position1 + mDrawConstraintSize * (rotation1 * q_swing).RotateAxisX(), Color::sWhite);

	if (mSwingMotorState == EMotorState::Velocity || mTwistMotorState == EMotorState::Velocity)
	{
		// Draw target angular velocity
		inRenderer->DrawArrow(position1, position1 + rotation2 * mTargetAngularVelocity, Color::sRed, 0.1f);
	}
	if (mSwingMotorState == EMotorState::Position || mTwistMotorState == EMotorState::Position)
	{
		// Draw motor swing and twist
		Quat swing, twist;
		mTargetOrientation.GetSwingTwist(swing, twist);
		inRenderer->DrawLine(position1, position1 + mDrawConstraintSize * (rotation1 * twist).RotateAxisY(), Color::sYellow);
		inRenderer->DrawLine(position1, position1 + mDrawConstraintSize * (rotation1 * swing).RotateAxisX(), Color::sCyan);
	}
}

void SwingTwistConstraint::DrawConstraintLimits(DebugRenderer *inRenderer) const
{
	// Get matrix that transforms from constraint space to world space
	RMat44 constraint_to_world = RMat44::sRotationTranslation(mBody1->GetRotation() * mConstraintToBody1, mBody1->GetCenterOfMassTransform() * mLocalSpacePosition1);

	// Draw limits
	if (mSwingTwistConstraintPart.GetSwingType() == ESwingType::Pyramid)
		inRenderer->DrawSwingPyramidLimits(constraint_to_world, -mPlaneHalfConeAngle, mPlaneHalfConeAngle, -mNormalHalfConeAngle, mNormalHalfConeAngle, mDrawConstraintSize, Color::sGreen, DebugRenderer::ECastShadow::Off);
	else
		inRenderer->DrawSwingConeLimits(constraint_to_world, mPlaneHalfConeAngle, mNormalHalfConeAngle, mDrawConstraintSize, Color::sGreen, DebugRenderer::ECastShadow::Off);
	inRenderer->DrawPie(constraint_to_world.GetTranslation(), mDrawConstraintSize, constraint_to_world.GetAxisX(), constraint_to_world.GetAxisY(), mTwistMinAngle, mTwistMaxAngle, Color::sPurple, DebugRenderer::ECastShadow::Off);
}
#endif // MOSS_DEBUG_RENDERER

void SwingTwistConstraint::SaveState(StateRecorder &inStream) const
{
	TwoBodyConstraint::SaveState(inStream);

	mPointConstraintPart.SaveState(inStream);
	mSwingTwistConstraintPart.SaveState(inStream);
	for (const AngleConstraintPart &c : mMotorConstraintPart)
		c.SaveState(inStream);

	inStream.Write(mSwingMotorState);
	inStream.Write(mTwistMotorState);
	inStream.Write(mTargetAngularVelocity);
	inStream.Write(mTargetOrientation);
}

void SwingTwistConstraint::RestoreState(StateRecorder &inStream)
{
	TwoBodyConstraint::RestoreState(inStream);

	mPointConstraintPart.RestoreState(inStream);
	mSwingTwistConstraintPart.RestoreState(inStream);
	for (AngleConstraintPart &c : mMotorConstraintPart)
		c.RestoreState(inStream);

	inStream.Read(mSwingMotorState);
	inStream.Read(mTwistMotorState);
	inStream.Read(mTargetAngularVelocity);
	inStream.Read(mTargetOrientation);
}

Ref<ConstraintSettings> SwingTwistConstraint::GetConstraintSettings() const
{
	SwingTwistConstraintSettings *settings = new SwingTwistConstraintSettings;
	ToConstraintSettings(*settings);
	settings->mSpace = EConstraintSpace::LocalToBodyCOM;
	settings->mPosition1 = RVec3(mLocalSpacePosition1);
	settings->mTwistAxis1 = mConstraintToBody1.RotateAxisX();
	settings->mPlaneAxis1 = mConstraintToBody1.RotateAxisZ();
	settings->mPosition2 = RVec3(mLocalSpacePosition2);
	settings->mTwistAxis2 = mConstraintToBody2.RotateAxisX();
	settings->mPlaneAxis2 = mConstraintToBody2.RotateAxisZ();
	settings->mSwingType = mSwingTwistConstraintPart.GetSwingType();
	settings->mNormalHalfConeAngle = mNormalHalfConeAngle;
	settings->mPlaneHalfConeAngle = mPlaneHalfConeAngle;
	settings->mTwistMinAngle = mTwistMinAngle;
	settings->mTwistMaxAngle = mTwistMaxAngle;
	settings->mMaxFrictionTorque = mMaxFrictionTorque;
	settings->mSwingMotorSettings = mSwingMotorSettings;
	settings->mTwistMotorSettings = mTwistMotorSettings;
	return settings;
}


TwoBodyConstraint *SixDOFConstraintSettings::Create(Body &inBody1, Body &inBody2) const
{
	return new SixDOFConstraint(inBody1, inBody2, *this);
}

void SixDOFConstraint::UpdateTranslationLimits()
{
	// Set to zero if the limits are inversed
	for (int i = EAxis::TranslationX; i <= EAxis::TranslationZ; ++i)
		if (mLimitMin[i] > mLimitMax[i])
			mLimitMin[i] = mLimitMax[i] = 0.0f;
}

void SixDOFConstraint::UpdateRotationLimits()
{
	if (mSwingTwistConstraintPart.GetSwingType() == ESwingType::Cone)
	{
		// Cone swing upper limit needs to be positive
		mLimitMax[EAxis::RotationY] = max(0.0f, mLimitMax[EAxis::RotationY]);
		mLimitMax[EAxis::RotationZ] = max(0.0f, mLimitMax[EAxis::RotationZ]);

		// Cone swing limits only support symmetric ranges
		mLimitMin[EAxis::RotationY] = -mLimitMax[EAxis::RotationY];
		mLimitMin[EAxis::RotationZ] = -mLimitMax[EAxis::RotationZ];
	}

	for (int i = EAxis::RotationX; i <= EAxis::RotationZ; ++i)
	{
		// Clamp to [-PI, PI] range
		mLimitMin[i] = Clamp(mLimitMin[i], -MOSS_PI, MOSS_PI);
		mLimitMax[i] = Clamp(mLimitMax[i], -MOSS_PI, MOSS_PI);

		// Set to zero if the limits are inversed
		if (mLimitMin[i] > mLimitMax[i])
			mLimitMin[i] = mLimitMax[i] = 0.0f;
	}

	// Pass limits on to constraint part
	mSwingTwistConstraintPart.SetLimits(mLimitMin[EAxis::RotationX], mLimitMax[EAxis::RotationX], mLimitMin[EAxis::RotationY], mLimitMax[EAxis::RotationY], mLimitMin[EAxis::RotationZ], mLimitMax[EAxis::RotationZ]);
}

void SixDOFConstraint::UpdateFixedFreeAxis()
{
	uint8 old_free_axis = mFreeAxis;
	uint8 old_fixed_axis = mFixedAxis;

	// Cache which axis are fixed and which ones are free
	mFreeAxis = 0;
	mFixedAxis = 0;
	for (int a = 0; a < EAxis::Num; ++a)
	{
		float limit = a >= EAxis::RotationX? MOSS_PI : FLT_MAX;

		if (mLimitMin[a] >= mLimitMax[a])
			mFixedAxis |= 1 << a;
		else if (mLimitMin[a] <= -limit && mLimitMax[a] >= limit)
			mFreeAxis |= 1 << a;
	}

	// On change we deactivate all constraints to reset warm starting
	if (old_free_axis != mFreeAxis || old_fixed_axis != mFixedAxis)
	{
		for (AxisConstraintPart &c : mTranslationConstraintPart)
			c.Deactivate();
		mPointConstraintPart.Deactivate();
		mSwingTwistConstraintPart.Deactivate();
		mRotationConstraintPart.Deactivate();
		for (AxisConstraintPart &c : mMotorTranslationConstraintPart)
			c.Deactivate();
		for (AngleConstraintPart &c : mMotorRotationConstraintPart)
			c.Deactivate();
	}
}

SixDOFConstraint::SixDOFConstraint(Body &inBody1, Body &inBody2, const SixDOFConstraintSettings &inSettings) :
	TwoBodyConstraint(inBody1, inBody2, inSettings)
{
	// Override swing type
	mSwingTwistConstraintPart.SetSwingType(inSettings.mSwingType);

	// Calculate rotation needed to go from constraint space to body1 local space
	Vec3 axis_z1 = inSettings.mAxisX1.Cross(inSettings.mAxisY1);
	Mat44 c_to_b1(Vec4(inSettings.mAxisX1, 0), Vec4(inSettings.mAxisY1, 0), Vec4(axis_z1, 0), Vec4(0, 0, 0, 1));
	mConstraintToBody1 = c_to_b1.GetQuaternion();

	// Calculate rotation needed to go from constraint space to body2 local space
	Vec3 axis_z2 = inSettings.mAxisX2.Cross(inSettings.mAxisY2);
	Mat44 c_to_b2(Vec4(inSettings.mAxisX2, 0), Vec4(inSettings.mAxisY2, 0), Vec4(axis_z2, 0), Vec4(0, 0, 0, 1));
	mConstraintToBody2 = c_to_b2.GetQuaternion();

	if (inSettings.mSpace == EConstraintSpace::WorldSpace)
	{
		// If all properties were specified in world space, take them to local space now
		mLocalSpacePosition1 = Vec3(inBody1.GetInverseCenterOfMassTransform() * inSettings.mPosition1);
		mConstraintToBody1 = inBody1.GetRotation().Conjugated() * mConstraintToBody1;

		mLocalSpacePosition2 = Vec3(inBody2.GetInverseCenterOfMassTransform() * inSettings.mPosition2);
		mConstraintToBody2 = inBody2.GetRotation().Conjugated() * mConstraintToBody2;
	}
	else
	{
		mLocalSpacePosition1 = Vec3(inSettings.mPosition1);
		mLocalSpacePosition2 = Vec3(inSettings.mPosition2);
	}

	// Copy translation and rotation limits
	memcpy(mLimitMin, inSettings.mLimitMin, sizeof(mLimitMin));
	memcpy(mLimitMax, inSettings.mLimitMax, sizeof(mLimitMax));
	memcpy(mLimitsSpringSettings, inSettings.mLimitsSpringSettings, sizeof(mLimitsSpringSettings));
	UpdateTranslationLimits();
	UpdateRotationLimits();
	UpdateFixedFreeAxis();
	CacheHasSpringLimits();

	// Store friction settings
	memcpy(mMaxFriction, inSettings.mMaxFriction, sizeof(mMaxFriction));

	// Store motor settings
	for (int i = 0; i < EAxis::Num; ++i)
		mMotorSettings[i] = inSettings.mMotorSettings[i];

	// Cache if motors are active (motors are off initially, but we may have friction)
	CacheTranslationMotorActive();
	CacheRotationMotorActive();
}

void SixDOFConstraint::NotifyShapeChanged(const BodyID &inBodyID, Vec3Arg inDeltaCOM)
{
	if (mBody1->GetID() == inBodyID)
		mLocalSpacePosition1 -= inDeltaCOM;
	else if (mBody2->GetID() == inBodyID)
		mLocalSpacePosition2 -= inDeltaCOM;
}

void SixDOFConstraint::SetTranslationLimits(Vec3Arg inLimitMin, Vec3Arg inLimitMax)
{
	mLimitMin[EAxis::TranslationX] = inLimitMin.GetX();
	mLimitMin[EAxis::TranslationY] = inLimitMin.GetY();
	mLimitMin[EAxis::TranslationZ] = inLimitMin.GetZ();
	mLimitMax[EAxis::TranslationX] = inLimitMax.GetX();
	mLimitMax[EAxis::TranslationY] = inLimitMax.GetY();
	mLimitMax[EAxis::TranslationZ] = inLimitMax.GetZ();

	UpdateTranslationLimits();
	UpdateFixedFreeAxis();
}

void SixDOFConstraint::SetRotationLimits(Vec3Arg inLimitMin, Vec3Arg inLimitMax)
{
	mLimitMin[EAxis::RotationX] = inLimitMin.GetX();
	mLimitMin[EAxis::RotationY] = inLimitMin.GetY();
	mLimitMin[EAxis::RotationZ] = inLimitMin.GetZ();
	mLimitMax[EAxis::RotationX] = inLimitMax.GetX();
	mLimitMax[EAxis::RotationY] = inLimitMax.GetY();
	mLimitMax[EAxis::RotationZ] = inLimitMax.GetZ();

	UpdateRotationLimits();
	UpdateFixedFreeAxis();
}

void SixDOFConstraint::SetMaxFriction(EAxis inAxis, float inFriction)
{
	mMaxFriction[inAxis] = inFriction;

	if (inAxis >= EAxis::TranslationX && inAxis <= EAxis::TranslationZ)
		CacheTranslationMotorActive();
	else
		CacheRotationMotorActive();
}

void SixDOFConstraint::GetPositionConstraintProperties(Vec3 &outR1PlusU, Vec3 &outR2, Vec3 &outU) const
{
	RVec3 p1 = mBody1->GetCenterOfMassTransform() * mLocalSpacePosition1;
	RVec3 p2 = mBody2->GetCenterOfMassTransform() * mLocalSpacePosition2;
	outR1PlusU = Vec3(p2 - mBody1->GetCenterOfMassPosition()); // r1 + u = (p1 - x1) + (p2 - p1) = p2 - x1
	outR2 = Vec3(p2 - mBody2->GetCenterOfMassPosition());
	outU = Vec3(p2 - p1);
}

Quat SixDOFConstraint::GetRotationInConstraintSpace() const
{
	// Let b1, b2 be the center of mass transform of body1 and body2 (For body1 this is mBody1->GetCenterOfMassTransform())
	// Let c1, c2 be the transform that takes a vector from constraint space to local space of body1 and body2 (For body1 this is Mat44::sRotationTranslation(mConstraintToBody1, mLocalSpacePosition1))
	// Let q be the rotation of the constraint in constraint space
	// b2 takes a vector from the local space of body2 to world space
	// To express this in terms of b1: b2 = b1 * c1 * q * c2^-1
	// c2^-1 goes from local body 2 space to constraint space
	// q rotates the constraint
	// c1 goes from constraint space to body 1 local space
	// b1 goes from body 1 local space to world space
	// So when the body rotations are given, q = (b1 * c1)^-1 * b2 c2
	// Or: q = (q1 * c1)^-1 * (q2 * c2) if we're only interested in rotations
	return (mBody1->GetRotation() * mConstraintToBody1).Conjugated() * mBody2->GetRotation() * mConstraintToBody2;
}

void SixDOFConstraint::CacheTranslationMotorActive()
{
	mTranslationMotorActive = mMotorState[EAxis::TranslationX] != EMotorState::Off
		|| mMotorState[EAxis::TranslationY] != EMotorState::Off
		|| mMotorState[EAxis::TranslationZ] != EMotorState::Off
		|| HasFriction(EAxis::TranslationX)
		|| HasFriction(EAxis::TranslationY)
		|| HasFriction(EAxis::TranslationZ);
}

void SixDOFConstraint::CacheRotationMotorActive()
{
	mRotationMotorActive = mMotorState[EAxis::RotationX] != EMotorState::Off
		|| mMotorState[EAxis::RotationY] != EMotorState::Off
		|| mMotorState[EAxis::RotationZ] != EMotorState::Off
		|| HasFriction(EAxis::RotationX)
		|| HasFriction(EAxis::RotationY)
		|| HasFriction(EAxis::RotationZ);
}

void SixDOFConstraint::CacheRotationPositionMotorActive()
{
	mRotationPositionMotorActive = 0;
	for (int i = 0; i < 3; ++i)
		if (mMotorState[EAxis::RotationX + i] == EMotorState::Position)
			mRotationPositionMotorActive |= 1 << i;
}

void SixDOFConstraint::CacheHasSpringLimits()
{
	mHasSpringLimits = mLimitsSpringSettings[EAxis::TranslationX].mFrequency > 0.0f
		|| mLimitsSpringSettings[EAxis::TranslationY].mFrequency > 0.0f
		|| mLimitsSpringSettings[EAxis::TranslationZ].mFrequency > 0.0f;
}

void SixDOFConstraint::SetMotorState(EAxis inAxis, EMotorState inState)
{
	MOSS_ASSERT(inState == EMotorState::Off || mMotorSettings[inAxis].IsValid());

	if (mMotorState[inAxis] != inState)
	{
		mMotorState[inAxis] = inState;

		// Ensure that warm starting next frame doesn't apply any impulses (motor parts are repurposed for different modes)
		if (inAxis >= EAxis::TranslationX && inAxis <= EAxis::TranslationZ)
		{
			mMotorTranslationConstraintPart[inAxis - EAxis::TranslationX].Deactivate();

			CacheTranslationMotorActive();
		}
		else
		{
			MOSS_ASSERT(inAxis >= EAxis::RotationX && inAxis <= EAxis::RotationZ);

			mMotorRotationConstraintPart[inAxis - EAxis::RotationX].Deactivate();

			CacheRotationMotorActive();
			CacheRotationPositionMotorActive();
		}
	}
}

void SixDOFConstraint::SetTargetOrientationCS(QuatArg inOrientation)
{
	Quat q_swing, q_twist;
	inOrientation.GetSwingTwist(q_swing, q_twist);

	uint clamped_axis;
	mSwingTwistConstraintPart.ClampSwingTwist(q_swing, q_twist, clamped_axis);

	if (clamped_axis != 0)
		mTargetOrientation = q_swing * q_twist;
	else
		mTargetOrientation = inOrientation;
}

void SixDOFConstraint::SetupVelocityConstraint(float inDeltaTime)
{
	// Get body rotations
	Quat rotation1 = mBody1->GetRotation();
	Quat rotation2 = mBody2->GetRotation();

	// Quaternion that rotates from body1's constraint space to world space
	Quat constraint_body1_to_world = rotation1 * mConstraintToBody1;

	// Store world space axis of constraint space
	Mat44 translation_axis_mat = Mat44::sRotation(constraint_body1_to_world);
	for (int i = 0; i < 3; ++i)
		mTranslationAxis[i] = translation_axis_mat.GetColumn3(i);

	if (IsTranslationFullyConstrained())
	{
		// All translation locked: Setup point constraint
		mPointConstraintPart.CalculateConstraintProperties(*mBody1, Mat44::sRotation(rotation1), mLocalSpacePosition1, *mBody2, Mat44::sRotation(rotation2), mLocalSpacePosition2);
	}
	else if (IsTranslationConstrained() || mTranslationMotorActive)
	{
		// Update world space positions (the bodies may have moved)
		Vec3 r1_plus_u, r2, u;
		GetPositionConstraintProperties(r1_plus_u, r2, u);

		// Setup axis constraint parts
		for (int i = 0; i < 3; ++i)
		{
			EAxis axis = EAxis(EAxis::TranslationX + i);

			Vec3 translation_axis = mTranslationAxis[i];

			// Calculate displacement along this axis
			float d = translation_axis.Dot(u);
			mDisplacement[i] = d; // Store for SolveVelocityConstraint

			// Setup limit constraint
			bool constraint_active = false;
			float constraint_value = 0.0f;
			if (IsFixedAxis(axis))
			{
				// When constraint is fixed it is always active
				constraint_value = d - mLimitMin[i];
				constraint_active = true;
			}
			else if (!IsFreeAxis(axis))
			{
				// When constraint is limited, it is only active when outside of the allowed range
				if (d <= mLimitMin[i])
				{
					constraint_value = d - mLimitMin[i];
					constraint_active = true;
				}
				else if (d >= mLimitMax[i])
				{
					constraint_value = d - mLimitMax[i];
					constraint_active = true;
				}
			}

			if (constraint_active)
				mTranslationConstraintPart[i].CalculateConstraintPropertiesWithSettings(inDeltaTime, *mBody1, r1_plus_u, *mBody2, r2, translation_axis, 0.0f, constraint_value, mLimitsSpringSettings[i]);
			else
				mTranslationConstraintPart[i].Deactivate();

			// Setup motor constraint
			switch (mMotorState[i])
			{
			case EMotorState::Off:
				if (HasFriction(axis))
					mMotorTranslationConstraintPart[i].CalculateConstraintProperties(*mBody1, r1_plus_u, *mBody2, r2, translation_axis);
				else
					mMotorTranslationConstraintPart[i].Deactivate();
				break;

			case EMotorState::Velocity:
				mMotorTranslationConstraintPart[i].CalculateConstraintProperties(*mBody1, r1_plus_u, *mBody2, r2, translation_axis, -mTargetVelocity[i]);
				break;

			case EMotorState::Position:
				{
					const SpringSettings &spring_settings = mMotorSettings[i].mSpringSettings;
					if (spring_settings.HasStiffness())
						mMotorTranslationConstraintPart[i].CalculateConstraintPropertiesWithSettings(inDeltaTime, *mBody1, r1_plus_u, *mBody2, r2, translation_axis, 0.0f, translation_axis.Dot(u) - mTargetPosition[i], spring_settings);
					else
						mMotorTranslationConstraintPart[i].Deactivate();
					break;
				}
			}
		}
	}

	// Setup rotation constraints
	if (IsRotationFullyConstrained())
	{
		// All rotation locked: Setup rotation constraint
		mRotationConstraintPart.CalculateConstraintProperties(*mBody1, Mat44::sRotation(mBody1->GetRotation()), *mBody2, Mat44::sRotation(mBody2->GetRotation()));
	}
	else if (IsRotationConstrained() || mRotationMotorActive)
	{
		// GetRotationInConstraintSpace without redoing the calculation of constraint_body1_to_world
		Quat constraint_body2_to_world = mBody2->GetRotation() * mConstraintToBody2;
		Quat q = constraint_body1_to_world.Conjugated() * constraint_body2_to_world;

		// Use swing twist constraint part
		if (IsRotationConstrained())
			mSwingTwistConstraintPart.CalculateConstraintProperties(*mBody1, *mBody2, q, constraint_body1_to_world);
		else
			mSwingTwistConstraintPart.Deactivate();

		if (mRotationMotorActive)
		{
			// Calculate rotation motor axis
			Mat44 ws_axis = Mat44::sRotation(constraint_body2_to_world);
			for (int i = 0; i < 3; ++i)
				mRotationAxis[i] = ws_axis.GetColumn3(i);

			// Get target orientation along the shortest path from q
			Quat target_orientation = q.Dot(mTargetOrientation) > 0.0f? mTargetOrientation : -mTargetOrientation;

			// The definition of the constraint rotation q:
			// R2 * ConstraintToBody2 = R1 * ConstraintToBody1 * q (1)
			//
			// R2' is the rotation of body 2 when reaching the target_orientation:
			// R2' * ConstraintToBody2 = R1 * ConstraintToBody1 * target_orientation (2)
			//
			// The difference in body 2 space:
			// R2' = R2 * diff_body2 (3)
			//
			// We want to specify the difference in the constraint space of body 2:
			// diff_body2 = ConstraintToBody2 * diff * ConstraintToBody2^* (4)
			//
			// Extracting R2' from 2: R2' = R1 * ConstraintToBody1 * target_orientation * ConstraintToBody2^* (5)
			// Combining 3 & 4: R2' = R2 * ConstraintToBody2 * diff * ConstraintToBody2^* (6)
			// Combining 1 & 6: R2' = R1 * ConstraintToBody1 * q * diff * ConstraintToBody2^* (7)
			// Combining 5 & 7: R1 * ConstraintToBody1 * target_orientation * ConstraintToBody2^* = R1 * ConstraintToBody1 * q * diff * ConstraintToBody2^*
			// <=> target_orientation = q * diff
			// <=> diff = q^* * target_orientation
			Quat diff = q.Conjugated() * target_orientation;

			// Project diff so that only rotation around axis that have a position motor are remaining
			Quat projected_diff;
			switch (mRotationPositionMotorActive)
			{
			case 0b001:
				// Keep only rotation around X
				projected_diff = diff.GetTwist(Vec3::sAxisX());
				break;

			case 0b010:
				// Keep only rotation around Y
				projected_diff = diff.GetTwist(Vec3::sAxisY());
				break;

			case 0b100:
				// Keep only rotation around Z
				projected_diff = diff.GetTwist(Vec3::sAxisZ());
				break;

			case 0b011:
				// Remove rotation around Z
				// q = swing_xy * twist_z <=> swing_xy = q * twist_z^*
				projected_diff = diff * diff.GetTwist(Vec3::sAxisZ()).Conjugated();
				break;

			case 0b101:
				// Remove rotation around Y
				// q = swing_xz * twist_y <=> swing_xz = q * twist_y^*
				projected_diff = diff * diff.GetTwist(Vec3::sAxisY()).Conjugated();
				break;

			case 0b110:
				// Remove rotation around X
				// q = swing_yz * twist_x <=> swing_yz = q * twist_x^*
				projected_diff = diff * diff.GetTwist(Vec3::sAxisX()).Conjugated();
				break;

			case 0b111:
			default: // All motors off is handled here but the results are unused
				// Keep entire rotation
				projected_diff = diff;
				break;
			}

			// Approximate error angles
			// The imaginary part of a quaternion is rotation_axis * sin(angle / 2)
			// If angle is small, sin(x) = x so angle[i] ~ 2.0f * rotation_axis[i]
			// We'll be making small time steps, so if the angle is not small at least the sign will be correct and we'll move in the right direction
			Vec3 rotation_error = -2.0f * projected_diff.GetXYZ();

			// Setup motors
			for (int i = 0; i < 3; ++i)
			{
				EAxis axis = EAxis(EAxis::RotationX + i);

				Vec3 rotation_axis = mRotationAxis[i];

				switch (mMotorState[axis])
				{
				case EMotorState::Off:
					if (HasFriction(axis))
						mMotorRotationConstraintPart[i].CalculateConstraintProperties(*mBody1, *mBody2, rotation_axis);
					else
						mMotorRotationConstraintPart[i].Deactivate();
					break;

				case EMotorState::Velocity:
					mMotorRotationConstraintPart[i].CalculateConstraintProperties(*mBody1, *mBody2, rotation_axis, -mTargetAngularVelocity[i]);
					break;

				case EMotorState::Position:
					{
						const SpringSettings &spring_settings = mMotorSettings[axis].mSpringSettings;
						if (spring_settings.HasStiffness())
							mMotorRotationConstraintPart[i].CalculateConstraintPropertiesWithSettings(inDeltaTime, *mBody1, *mBody2, rotation_axis, 0.0f, rotation_error[i], spring_settings);
						else
							mMotorRotationConstraintPart[i].Deactivate();
						break;
					}
				}
			}
		}
	}
}

void SixDOFConstraint::ResetWarmStart()
{
	for (AxisConstraintPart &c : mMotorTranslationConstraintPart)
		c.Deactivate();
	for (AngleConstraintPart &c : mMotorRotationConstraintPart)
		c.Deactivate();
	mRotationConstraintPart.Deactivate();
	mSwingTwistConstraintPart.Deactivate();
	mPointConstraintPart.Deactivate();
	for (AxisConstraintPart &c : mTranslationConstraintPart)
		c.Deactivate();
}

void SixDOFConstraint::WarmStartVelocityConstraint(float inWarmStartImpulseRatio)
{
	// Warm start translation motors
	if (mTranslationMotorActive)
		for (int i = 0; i < 3; ++i)
			if (mMotorTranslationConstraintPart[i].IsActive())
				mMotorTranslationConstraintPart[i].WarmStart(*mBody1, *mBody2, mTranslationAxis[i], inWarmStartImpulseRatio);

	// Warm start rotation motors
	if (mRotationMotorActive)
		for (AngleConstraintPart &c : mMotorRotationConstraintPart)
			if (c.IsActive())
				c.WarmStart(*mBody1, *mBody2, inWarmStartImpulseRatio);

	// Warm start rotation constraints
	if (IsRotationFullyConstrained())
		mRotationConstraintPart.WarmStart(*mBody1, *mBody2, inWarmStartImpulseRatio);
	else if (IsRotationConstrained())
		mSwingTwistConstraintPart.WarmStart(*mBody1, *mBody2, inWarmStartImpulseRatio);

	// Warm start translation constraints
	if (IsTranslationFullyConstrained())
		mPointConstraintPart.WarmStart(*mBody1, *mBody2, inWarmStartImpulseRatio);
	else if (IsTranslationConstrained())
		for (int i = 0; i < 3; ++i)
			if (mTranslationConstraintPart[i].IsActive())
				mTranslationConstraintPart[i].WarmStart(*mBody1, *mBody2, mTranslationAxis[i], inWarmStartImpulseRatio);
}

bool SixDOFConstraint::SolveVelocityConstraint(float inDeltaTime)
{
	bool impulse = false;

	// Solve translation motor
	if (mTranslationMotorActive)
		for (int i = 0; i < 3; ++i)
			if (mMotorTranslationConstraintPart[i].IsActive())
				switch (mMotorState[i])
				{
				case EMotorState::Off:
				{
					// Apply friction only
					float max_lambda = mMaxFriction[i] * inDeltaTime;
					impulse |= mMotorTranslationConstraintPart[i].SolveVelocityConstraint(*mBody1, *mBody2, mTranslationAxis[i], -max_lambda, max_lambda);
					break;
				}

				case EMotorState::Velocity:
				case EMotorState::Position:
					// Drive motor
					impulse |= mMotorTranslationConstraintPart[i].SolveVelocityConstraint(*mBody1, *mBody2, mTranslationAxis[i], inDeltaTime * mMotorSettings[i].mMinForceLimit, inDeltaTime * mMotorSettings[i].mMaxForceLimit);
					break;
				}

	// Solve rotation motor
	if (mRotationMotorActive)
		for (int i = 0; i < 3; ++i)
		{
			EAxis axis = EAxis(EAxis::RotationX + i);
			if (mMotorRotationConstraintPart[i].IsActive())
				switch (mMotorState[axis])
				{
				case EMotorState::Off:
				{
					// Apply friction only
					float max_lambda = mMaxFriction[axis] * inDeltaTime;
					impulse |= mMotorRotationConstraintPart[i].SolveVelocityConstraint(*mBody1, *mBody2, mRotationAxis[i], -max_lambda, max_lambda);
					break;
				}

				case EMotorState::Velocity:
				case EMotorState::Position:
					// Drive motor
					impulse |= mMotorRotationConstraintPart[i].SolveVelocityConstraint(*mBody1, *mBody2, mRotationAxis[i], inDeltaTime * mMotorSettings[axis].mMinTorqueLimit, inDeltaTime * mMotorSettings[axis].mMaxTorqueLimit);
					break;
				}
		}

	// Solve rotation constraint
	if (IsRotationFullyConstrained())
		impulse |= mRotationConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2);
	else if (IsRotationConstrained())
		impulse |= mSwingTwistConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2);

	// Solve position constraint
	if (IsTranslationFullyConstrained())
		impulse |= mPointConstraintPart.SolveVelocityConstraint(*mBody1, *mBody2);
	else if (IsTranslationConstrained())
		for (int i = 0; i < 3; ++i)
			if (mTranslationConstraintPart[i].IsActive())
			{
				// If the axis is not fixed it must be limited (or else the constraint would not be active)
				// Calculate the min and max constraint force based on on which side we're limited
				float limit_min = -FLT_MAX, limit_max = FLT_MAX;
				if (!IsFixedAxis(EAxis(EAxis::TranslationX + i)))
				{
					MOSS_ASSERT(!IsFreeAxis(EAxis(EAxis::TranslationX + i)));
					if (mDisplacement[i] <= mLimitMin[i])
						limit_min = 0;
					else if (mDisplacement[i] >= mLimitMax[i])
						limit_max = 0;
				}

				impulse |= mTranslationConstraintPart[i].SolveVelocityConstraint(*mBody1, *mBody2, mTranslationAxis[i], limit_min, limit_max);
			}

	return impulse;
}

bool SixDOFConstraint::SolvePositionConstraint(float inDeltaTime, float inBaumgarte)
{
	bool impulse = false;

	if (IsRotationFullyConstrained())
	{
		// Rotation locked: Solve rotation constraint

		// Inverse of initial rotation from body 1 to body 2 in body 1 space
		// Definition of initial orientation r0: q2 = q1 r0
		// Initial rotation (see: GetRotationInConstraintSpace): q2 = q1 c1 c2^-1
		// So: r0^-1 = (c1 c2^-1)^-1 = c2 * c1^-1
		Quat constraint_to_body1 = mConstraintToBody1 * Quat::sEulerAngles(GetRotationLimitsMin());
		Quat inv_initial_orientation = mConstraintToBody2 * constraint_to_body1.Conjugated();

		// Solve rotation violations
		mRotationConstraintPart.CalculateConstraintProperties(*mBody1, Mat44::sRotation(mBody1->GetRotation()), *mBody2, Mat44::sRotation(mBody2->GetRotation()));
		impulse |= mRotationConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, inv_initial_orientation, inBaumgarte);
	}
	else if (IsRotationConstrained())
	{
		// Rotation partially constraint

		// Solve rotation violations
		Quat q = GetRotationInConstraintSpace();
		impulse |= mSwingTwistConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, q, mConstraintToBody1, mConstraintToBody2, inBaumgarte);
	}

	// Solve position violations
	if (IsTranslationFullyConstrained())
	{
		// Translation locked: Solve point constraint
		Vec3 local_space_position1 = mLocalSpacePosition1 + mConstraintToBody1 * GetTranslationLimitsMin();
		mPointConstraintPart.CalculateConstraintProperties(*mBody1, Mat44::sRotation(mBody1->GetRotation()), local_space_position1, *mBody2, Mat44::sRotation(mBody2->GetRotation()), mLocalSpacePosition2);
		impulse |= mPointConstraintPart.SolvePositionConstraint(*mBody1, *mBody2, inBaumgarte);
	}
	else if (IsTranslationConstrained())
	{
		// Translation partially locked: Solve per axis
		for (int i = 0; i < 3; ++i)
			if (mLimitsSpringSettings[i].mFrequency <= 0.0f) // If not soft limit
			{
				// Update world space positions (the bodies may have moved)
				Vec3 r1_plus_u, r2, u;
				GetPositionConstraintProperties(r1_plus_u, r2, u);

				// Quaternion that rotates from body1's constraint space to world space
				Quat constraint_body1_to_world = mBody1->GetRotation() * mConstraintToBody1;

				// Calculate axis
				Vec3 translation_axis;
				switch (i)
				{
				case 0:							translation_axis = constraint_body1_to_world.RotateAxisX(); break;
				case 1:							translation_axis = constraint_body1_to_world.RotateAxisY(); break;
				default:	MOSS_ASSERT(i == 2); translation_axis = constraint_body1_to_world.RotateAxisZ(); break;
				}

				// Determine position error
				float error = 0.0f;
				EAxis axis(EAxis(EAxis::TranslationX + i));
				if (IsFixedAxis(axis))
					error = u.Dot(translation_axis) - mLimitMin[axis];
				else if (!IsFreeAxis(axis))
				{
					float displacement = u.Dot(translation_axis);
					if (displacement <= mLimitMin[axis])
						error = displacement - mLimitMin[axis];
					else if (displacement >= mLimitMax[axis])
						error = displacement - mLimitMax[axis];
				}

				if (error != 0.0f)
				{
					// Setup axis constraint part and solve it
					mTranslationConstraintPart[i].CalculateConstraintProperties(*mBody1, r1_plus_u, *mBody2, r2, translation_axis);
					impulse |= mTranslationConstraintPart[i].SolvePositionConstraint(*mBody1, *mBody2, translation_axis, error, inBaumgarte);
				}
			}
	}

	return impulse;
}

#ifndef MOSS_DEBUG_RENDERER
void SixDOFConstraint::DrawConstraint(DebugRenderer *inRenderer) const
{
	// Get constraint properties in world space
	RVec3 position1 = mBody1->GetCenterOfMassTransform() * mLocalSpacePosition1;
	Quat rotation1 = mBody1->GetRotation() * mConstraintToBody1;
	Quat rotation2 = mBody2->GetRotation() * mConstraintToBody2;

	// Draw constraint orientation
	inRenderer->DrawCoordinateSystem(RMat44::sRotationTranslation(rotation1, position1), mDrawConstraintSize);

	if ((IsRotationConstrained() || mRotationPositionMotorActive != 0) && !IsRotationFullyConstrained())
	{
		// Draw current swing and twist
		Quat q = GetRotationInConstraintSpace();
		Quat q_swing, q_twist;
		q.GetSwingTwist(q_swing, q_twist);
		inRenderer->DrawLine(position1, position1 + mDrawConstraintSize * (rotation1 * q_twist).RotateAxisY(), Color::sWhite);
		inRenderer->DrawLine(position1, position1 + mDrawConstraintSize * (rotation1 * q_swing).RotateAxisX(), Color::sWhite);
	}

	// Draw target rotation
	Quat m_swing, m_twist;
	mTargetOrientation.GetSwingTwist(m_swing, m_twist);
	if (mMotorState[EAxis::RotationX] == EMotorState::Position)
		inRenderer->DrawLine(position1, position1 + mDrawConstraintSize * (rotation1 * m_twist).RotateAxisY(), Color::sYellow);
	if (mMotorState[EAxis::RotationY] == EMotorState::Position || mMotorState[EAxis::RotationZ] == EMotorState::Position)
		inRenderer->DrawLine(position1, position1 + mDrawConstraintSize * (rotation1 * m_swing).RotateAxisX(), Color::sYellow);

	// Draw target angular velocity
	Vec3 target_angular_velocity = Vec3::sZero();
	for (int i = 0; i < 3; ++i)
		if (mMotorState[EAxis::RotationX + i] == EMotorState::Velocity)
			target_angular_velocity.SetComponent(i, mTargetAngularVelocity[i]);
	if (target_angular_velocity != Vec3::sZero())
		inRenderer->DrawArrow(position1, position1 + rotation2 * target_angular_velocity, Color::sRed, 0.1f);
}

void SixDOFConstraint::DrawConstraintLimits(DebugRenderer *inRenderer) const
{
	// Get matrix that transforms from constraint space to world space
	RMat44 constraint_body1_to_world = RMat44::sRotationTranslation(mBody1->GetRotation() * mConstraintToBody1, mBody1->GetCenterOfMassTransform() * mLocalSpacePosition1);

	// Draw limits
	if (mSwingTwistConstraintPart.GetSwingType() == ESwingType::Pyramid)
		inRenderer->DrawSwingPyramidLimits(constraint_body1_to_world, mLimitMin[EAxis::RotationY], mLimitMax[EAxis::RotationY], mLimitMin[EAxis::RotationZ], mLimitMax[EAxis::RotationZ], mDrawConstraintSize, Color::sGreen, DebugRenderer::ECastShadow::Off);
	else
		inRenderer->DrawSwingConeLimits(constraint_body1_to_world, mLimitMax[EAxis::RotationY], mLimitMax[EAxis::RotationZ], mDrawConstraintSize, Color::sGreen, DebugRenderer::ECastShadow::Off);
	inRenderer->DrawPie(constraint_body1_to_world.GetTranslation(), mDrawConstraintSize, constraint_body1_to_world.GetAxisX(), constraint_body1_to_world.GetAxisY(), mLimitMin[EAxis::RotationX], mLimitMax[EAxis::RotationX], Color::sPurple, DebugRenderer::ECastShadow::Off);
}
#endif // MOSS_DEBUG_RENDERER

void SixDOFConstraint::SaveState(StateRecorder &inStream) const
{
	TwoBodyConstraint::SaveState(inStream);

	for (const AxisConstraintPart &c : mTranslationConstraintPart)
		c.SaveState(inStream);
	mPointConstraintPart.SaveState(inStream);
	mSwingTwistConstraintPart.SaveState(inStream);
	mRotationConstraintPart.SaveState(inStream);
	for (const AxisConstraintPart &c : mMotorTranslationConstraintPart)
		c.SaveState(inStream);
	for (const AngleConstraintPart &c : mMotorRotationConstraintPart)
		c.SaveState(inStream);

	inStream.Write(mMotorState);
	inStream.Write(mTargetVelocity);
	inStream.Write(mTargetAngularVelocity);
	inStream.Write(mTargetPosition);
	inStream.Write(mTargetOrientation);
}

void SixDOFConstraint::RestoreState(StateRecorder &inStream)
{
	TwoBodyConstraint::RestoreState(inStream);

	for (AxisConstraintPart &c : mTranslationConstraintPart)
		c.RestoreState(inStream);
	mPointConstraintPart.RestoreState(inStream);
	mSwingTwistConstraintPart.RestoreState(inStream);
	mRotationConstraintPart.RestoreState(inStream);
	for (AxisConstraintPart &c : mMotorTranslationConstraintPart)
		c.RestoreState(inStream);
	for (AngleConstraintPart &c : mMotorRotationConstraintPart)
		c.RestoreState(inStream);

	inStream.Read(mMotorState);
	inStream.Read(mTargetVelocity);
	inStream.Read(mTargetAngularVelocity);
	inStream.Read(mTargetPosition);
	inStream.Read(mTargetOrientation);

	CacheTranslationMotorActive();
	CacheRotationMotorActive();
	CacheRotationPositionMotorActive();
}

Ref<ConstraintSettings> SixDOFConstraint::GetConstraintSettings() const
{
	SixDOFConstraintSettings *settings = new SixDOFConstraintSettings;
	ToConstraintSettings(*settings);
	settings->mSpace = EConstraintSpace::LocalToBodyCOM;
	settings->mPosition1 = RVec3(mLocalSpacePosition1);
	settings->mAxisX1 = mConstraintToBody1.RotateAxisX();
	settings->mAxisY1 = mConstraintToBody1.RotateAxisY();
	settings->mPosition2 = RVec3(mLocalSpacePosition2);
	settings->mAxisX2 = mConstraintToBody2.RotateAxisX();
	settings->mAxisY2 = mConstraintToBody2.RotateAxisY();
	settings->mSwingType = mSwingTwistConstraintPart.GetSwingType();
	memcpy(settings->mLimitMin, mLimitMin, sizeof(mLimitMin));
	memcpy(settings->mLimitMax, mLimitMax, sizeof(mLimitMax));
	memcpy(settings->mMaxFriction, mMaxFriction, sizeof(mMaxFriction));
	for (int i = 0; i < EAxis::Num; ++i)
		settings->mMotorSettings[i] = mMotorSettings[i];
	return settings;
}


MOSS_SUPRESS_WARNINGS_END
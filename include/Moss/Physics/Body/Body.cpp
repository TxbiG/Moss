// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Moss/Physics/physics_intern.h>

#ifndef MOSS_DEBUG_RENDERER
	#include <Moss/Moss_Renderer.h>
#endif // MOSS_DEBUG_RENDERER


#include <Moss/Physics/Body/MotionProperties.h>
#include <Moss/Physics/Collision/BroadPhase/BroadPhase.h>
#include <Moss/Physics/Collision/CollisionCollectorImpl.h>
#include <Moss/Physics/Collision/PhysicsMaterial.h>
#include <Moss/Physics/Constraints/TwoBodyConstraint.h>
#include <Moss/Physics/StateRecorder.h>
#include <Moss/Core/Variants/Matrix/TMatrix.h>
#include <Moss/Core/Variants/Vector/TVec.h>
#include <Moss/Core/Variants/Math/EigenValueSymmetric.h>
#include <Moss/Core/StreamIn.h>
#include <Moss/Core/StreamOut.h>
#include <Moss/Core/InsertionSort.h>
#include <Moss/Physics/PhysicsSettings.h>
#include <Moss/Physics/StateRecorder.h>
#include <Moss/Physics/Collision/Shape/EmptyShape.h>
#include <Moss/Core/StringTools.h>
#include <Moss/Core/Profiler.h>
#include <Moss/Physics/PhysicsSettings.h>
#include <Moss/Physics/SoftBody.h>
#include <Moss/Physics/StateRecorder.h>
#include <Moss/Core/StringTools.h>
#include <Moss/Core/StreamIn.h>
#include <Moss/Core/StreamOut.h>

MOSS_SUPRESS_WARNINGS_BEGIN

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(MassProperties) {
	MOSS_ADD_ATTRIBUTE(MassProperties, mMass)
	MOSS_ADD_ATTRIBUTE(MassProperties, mInertia)
}

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(BodyCreationSettings) {
	MOSS_ADD_ATTRIBUTE(BodyCreationSettings, mPosition)
	MOSS_ADD_ATTRIBUTE(BodyCreationSettings, mRotation)
	MOSS_ADD_ATTRIBUTE(BodyCreationSettings, mLinearVelocity)
	MOSS_ADD_ATTRIBUTE(BodyCreationSettings, mAngularVelocity)
	MOSS_ADD_ATTRIBUTE(BodyCreationSettings, mUserData)
	MOSS_ADD_ATTRIBUTE(BodyCreationSettings, mShape)
	MOSS_ADD_ATTRIBUTE(BodyCreationSettings, mCollisionGroup)
	MOSS_ADD_ENUM_ATTRIBUTE(BodyCreationSettings, mObjectLayer)
	MOSS_ADD_ENUM_ATTRIBUTE(BodyCreationSettings, mMotionType)
	MOSS_ADD_ENUM_ATTRIBUTE(BodyCreationSettings, mAllowedDOFs)
	MOSS_ADD_ATTRIBUTE(BodyCreationSettings, mAllowDynamicOrKinematic)
	MOSS_ADD_ATTRIBUTE(BodyCreationSettings, mIsSensor)
	MOSS_ADD_ATTRIBUTE_WITH_ALIAS(BodyCreationSettings, mCollideKinematicVsNonDynamic, "mSensorDetectsStatic") // This is the old name to keep backwards compatibility
	MOSS_ADD_ATTRIBUTE(BodyCreationSettings, mUseManifoldReduction)
	MOSS_ADD_ATTRIBUTE(BodyCreationSettings, mApplyGyroscopicForce)
	MOSS_ADD_ENUM_ATTRIBUTE(BodyCreationSettings, mMotionQuality)
	MOSS_ADD_ATTRIBUTE(BodyCreationSettings, mEnhancedInternalEdgeRemoval)
	MOSS_ADD_ATTRIBUTE(BodyCreationSettings, mAllowSleeping)
	MOSS_ADD_ATTRIBUTE(BodyCreationSettings, mFriction)
	MOSS_ADD_ATTRIBUTE(BodyCreationSettings, mRestitution)
	MOSS_ADD_ATTRIBUTE(BodyCreationSettings, mLinearDamping)
	MOSS_ADD_ATTRIBUTE(BodyCreationSettings, mAngularDamping)
	MOSS_ADD_ATTRIBUTE(BodyCreationSettings, mMaxLinearVelocity)
	MOSS_ADD_ATTRIBUTE(BodyCreationSettings, mMaxAngularVelocity)
	MOSS_ADD_ATTRIBUTE(BodyCreationSettings, mGravityFactor)
	MOSS_ADD_ATTRIBUTE(BodyCreationSettings, mNumVelocityStepsOverride)
	MOSS_ADD_ATTRIBUTE(BodyCreationSettings, mNumPositionStepsOverride)
	MOSS_ADD_ENUM_ATTRIBUTE(BodyCreationSettings, mOverrideMassProperties)
	MOSS_ADD_ATTRIBUTE(BodyCreationSettings, mInertiaMultiplier)
	MOSS_ADD_ATTRIBUTE(BodyCreationSettings, mMassPropertiesOverride)
}

void MotionProperties::SetMassProperties(EAllowedDOFs inAllowedDOFs, const MassProperties &inMassProperties)
{
	// Store allowed DOFs
	mAllowedDOFs = inAllowedDOFs;

	// Decompose DOFs
	uint allowed_translation_axis = uint(inAllowedDOFs) & 0b111;
	uint allowed_rotation_axis = (uint(inAllowedDOFs) >> 3) & 0b111;

	// Set inverse mass
	if (allowed_translation_axis == 0)
	{
		// No translation possible
		mInvMass = 0.0f;
	}
	else
	{
		MOSS_ASSERT(inMassProperties.mMass > 0.0f, "Invalid mass. "
			"Some shapes like MeshShape or TriangleShape cannot calculate mass automatically, "
			"in this case you need to provide it by setting BodyCreationSettings::mOverrideMassProperties and mMassPropertiesOverride.");
		mInvMass = 1.0f / inMassProperties.mMass;
	}

	if (allowed_rotation_axis == 0)
	{
		// No rotation possible
		mInvInertiaDiagonal = Vec3::sZero();
		mInertiaRotation = Quat::sIdentity();
	}
	else
	{
		// Set inverse inertia
		Mat44 rotation;
		Vec3 diagonal;
		if (inMassProperties.DecomposePrincipalMomentsOfInertia(rotation, diagonal)
			&& !diagonal.IsNearZero())
		{
			mInvInertiaDiagonal = diagonal.Reciprocal();
			mInertiaRotation = rotation.GetQuaternion();
		}
		else
		{
			// Failed! Fall back to inertia tensor of sphere with radius 1.
			mInvInertiaDiagonal = Vec3::sReplicate(2.5f * mInvMass);
			mInertiaRotation = Quat::sIdentity();
		}
	}

	MOSS_ASSERT(mInvMass != 0.0f || mInvInertiaDiagonal != Vec3::sZero(), "Can't lock all axes, use a static body for this. This will crash with a division by zero later!");
}

void MotionProperties::SaveState(StateRecorder &inStream) const
{
	// Only write properties that can change at runtime
	inStream.Write(mLinearVelocity);
	inStream.Write(mAngularVelocity);
	inStream.Write(mForce);
	inStream.Write(mTorque);
#ifdef MOSS_DOUBLE_PRECISION
	inStream.Write(mSleepTestOffset);
#endif // MOSS_DOUBLE_PRECISION
	inStream.Write(mSleepTestSpheres);
	inStream.Write(mSleepTestTimer);
	inStream.Write(mAllowSleeping);
}

void MotionProperties::RestoreState(StateRecorder &inStream)
{
	inStream.Read(mLinearVelocity);
	inStream.Read(mAngularVelocity);
	inStream.Read(mForce);
	inStream.Read(mTorque);
#ifdef MOSS_DOUBLE_PRECISION
	inStream.Read(mSleepTestOffset);
#endif // MOSS_DOUBLE_PRECISION
	inStream.Read(mSleepTestSpheres);
	inStream.Read(mSleepTestTimer);
	inStream.Read(mAllowSleeping);
}


bool MassProperties::DecomposePrincipalMomentsOfInertia(Mat44 &outRotation, Vec3 &outDiagonal) const
{
	// Using eigendecomposition to get the principal components of the inertia tensor
	// See: https://en.wikipedia.org/wiki/Eigendecomposition_of_a_matrix
	Matrix<3, 3> inertia;
	inertia.CopyPart(mInertia, 0, 0, 3, 3, 0, 0);
	Matrix<3, 3> eigen_vec = Matrix<3, 3>::sIdentity();
	TVec<3> eigen_val;
	if (!EigenValueSymmetric(inertia, eigen_vec, eigen_val))
		return false;

	// Sort so that the biggest value goes first
	int indices[] = { 0, 1, 2 };
	InsertionSort(indices, indices + 3, [&eigen_val](int inLeft, int inRight) { return eigen_val[inLeft] > eigen_val[inRight]; });

	// Convert to a regular Mat44 and Vec3
	outRotation = Mat44::sIdentity();
	for (int i = 0; i < 3; ++i)
	{
		outRotation.SetColumn3(i, Vec3(reinterpret_cast<Float3 &>(eigen_vec.GetColumn(indices[i]))));
		outDiagonal.SetComponent(i, eigen_val[indices[i]]);
	}

	// Make sure that the rotation matrix is a right handed matrix
	if (outRotation.GetAxisX().Cross(outRotation.GetAxisY()).Dot(outRotation.GetAxisZ()) < 0.0f)
		outRotation.SetAxisZ(-outRotation.GetAxisZ());

#ifdef MOSS_DEBUG
	// Validate that the solution is correct, for each axis we want to make sure that the difference in inertia is
	// smaller than some fraction of the inertia itself in that axis
	Mat44 new_inertia = outRotation * Mat44::sScale(outDiagonal) * outRotation.Inversed();
	for (int i = 0; i < 3; ++i)
		MOSS_ASSERT(new_inertia.GetColumn3(i).IsClose(mInertia.GetColumn3(i), mInertia.GetColumn3(i).LengthSq() * 1.0e-10f));
#endif

	return true;
}

void MassProperties::SetMassAndInertiaOfSolidBox(Vec3Arg inBoxSize, float inDensity)
{
	// Calculate mass
	mMass = inBoxSize.GetX() * inBoxSize.GetY() * inBoxSize.GetZ() * inDensity;

	// Calculate inertia
	Vec3 size_sq = inBoxSize * inBoxSize;
	Vec3 scale = (size_sq.Swizzle<SWIZZLE_Y, SWIZZLE_X, SWIZZLE_X>() + size_sq.Swizzle<SWIZZLE_Z, SWIZZLE_Z, SWIZZLE_Y>()) * (mMass / 12.0f);
	mInertia = Mat44::sScale(scale);
}

void MassProperties::ScaleToMass(float inMass)
{
	if (mMass > 0.0f)
	{
		// Calculate how much we have to scale the inertia tensor
		float mass_scale = inMass / mMass;

		// Update mass
		mMass = inMass;

		// Update inertia tensor
		for (int i = 0; i < 3; ++i)
			mInertia.SetColumn4(i, mInertia.GetColumn4(i) * mass_scale);
	}
	else
	{
		// Just set the mass
		mMass = inMass;
	}
}

Vec3 MassProperties::sGetEquivalentSolidBoxSize(float inMass, Vec3Arg inInertiaDiagonal)
{
	// Moment of inertia of a solid box has diagonal:
	// mass / 12 * [size_y^2 + size_z^2, size_x^2 + size_z^2, size_x^2 + size_y^2]
	// Solving for size_x, size_y and size_y (diagonal and mass are known):
	Vec3 diagonal = inInertiaDiagonal * (12.0f / inMass);
	return Vec3(sqrt(0.5f * (-diagonal[0] + diagonal[1] + diagonal[2])), sqrt(0.5f * (diagonal[0] - diagonal[1] + diagonal[2])), sqrt(0.5f * (diagonal[0] + diagonal[1] - diagonal[2])));
}

void MassProperties::Scale(Vec3Arg inScale)
{
	// See: https://en.wikipedia.org/wiki/Moment_of_inertia#Inertia_tensor
	// The diagonal of the inertia tensor can be calculated like this:
	// Ixx = sum_{k = 1 to n}(m_k * (y_k^2 + z_k^2))
	// Iyy = sum_{k = 1 to n}(m_k * (x_k^2 + z_k^2))
	// Izz = sum_{k = 1 to n}(m_k * (x_k^2 + y_k^2))
	//
	// We want to isolate the terms x_k, y_k and z_k:
	// d = [0.5, 0.5, 0.5].[Ixx, Iyy, Izz]
	// [sum_{k = 1 to n}(m_k * x_k^2), sum_{k = 1 to n}(m_k * y_k^2), sum_{k = 1 to n}(m_k * z_k^2)] = [d, d, d] - [Ixx, Iyy, Izz]
	Vec3 diagonal = mInertia.GetDiagonal3();
	Vec3 xyz_sq = Vec3::sReplicate(Vec3::sReplicate(0.5f).Dot(diagonal)) - diagonal;

	// When scaling a shape these terms change like this:
	// sum_{k = 1 to n}(m_k * (scale_x * x_k)^2) = scale_x^2 * sum_{k = 1 to n}(m_k * x_k^2)
	// Same for y_k and z_k
	// Using these terms we can calculate the new diagonal of the inertia tensor:
	Vec3 xyz_scaled_sq = inScale * inScale * xyz_sq;
	float i_xx = xyz_scaled_sq.GetY() + xyz_scaled_sq.GetZ();
	float i_yy = xyz_scaled_sq.GetX() + xyz_scaled_sq.GetZ();
	float i_zz = xyz_scaled_sq.GetX() + xyz_scaled_sq.GetY();

	// The off diagonal elements are calculated like:
	// Ixy = -sum_{k = 1 to n}(x_k y_k)
	// Ixz = -sum_{k = 1 to n}(x_k z_k)
	// Iyz = -sum_{k = 1 to n}(y_k z_k)
	// Scaling these is simple:
	float i_xy = inScale.GetX() * inScale.GetY() * mInertia(0, 1);
	float i_xz = inScale.GetX() * inScale.GetZ() * mInertia(0, 2);
	float i_yz = inScale.GetY() * inScale.GetZ() * mInertia(1, 2);

	// Update inertia tensor
	mInertia(0, 0) = i_xx;
	mInertia(0, 1) = i_xy;
	mInertia(1, 0) = i_xy;
	mInertia(1, 1) = i_yy;
	mInertia(0, 2) = i_xz;
	mInertia(2, 0) = i_xz;
	mInertia(1, 2) = i_yz;
	mInertia(2, 1) = i_yz;
	mInertia(2, 2) = i_zz;

	// Mass scales linear with volume (note that the scaling can be negative and we don't want the mass to become negative)
	float mass_scale = abs(inScale.GetX() * inScale.GetY() * inScale.GetZ());
	mMass *= mass_scale;

	// Inertia scales linear with mass. This updates the m_k terms above.
	mInertia *= mass_scale;

	// Ensure that the bottom right element is a 1 again
	mInertia(3, 3) = 1.0f;
}

void MassProperties::Rotate(Mat44Arg inRotation)
{
	mInertia = inRotation.Multiply3x3(mInertia).Multiply3x3RightTransposed(inRotation);
}

void MassProperties::Translate(Vec3Arg inTranslation)
{
	// Transform the inertia using the parallel axis theorem: I' = I + m * (translation^2 E - translation translation^T)
	// Where I is the original body's inertia and E the identity matrix
	// See: https://en.wikipedia.org/wiki/Parallel_axis_theorem
	mInertia += mMass * (Mat44::sScale(inTranslation.Dot(inTranslation)) - Mat44::sOuterProduct(inTranslation, inTranslation));

	// Ensure that inertia is a 3x3 matrix, adding inertias causes the bottom right element to change
	mInertia.SetColumn4(3, Vec4(0, 0, 0, 1));
}

void MassProperties::SaveBinaryState(StreamOut &inStream) const
{
	inStream.Write(mMass);
	inStream.Write(mInertia);
}

void MassProperties::RestoreBinaryState(StreamIn &inStream)
{
	inStream.Read(mMass);
	inStream.Read(mInertia);
}

static const EmptyShape sFixedToWorldShape;
Body Body::sFixedToWorld(false);

Body::Body(bool) :
	mPosition(Vec3::sZero()),
	mRotation(Quat::sIdentity()),
	mShape(&sFixedToWorldShape), // Dummy shape
	mFriction(0.0f),
	mRestitution(0.0f),
	mObjectLayer(cObjectLayerInvalid),
	mMotionType(EMotionType::Static)
{
	sFixedToWorldShape.SetEmbedded();
}

void Body::SetMotionType(EMotionType inMotionType)
{
	if (mMotionType == inMotionType)
		return;

	MOSS_ASSERT(inMotionType == EMotionType::Static || mMotionProperties != nullptr, "Body needs to be created with mAllowDynamicOrKinematic set to true");
	MOSS_ASSERT(inMotionType != EMotionType::Static || !IsActive(), "Deactivate body first");
	MOSS_ASSERT(inMotionType == EMotionType::Dynamic || !IsSoftBody(), "Soft bodies can only be dynamic, you can make individual vertices kinematic by setting their inverse mass to 0");

	// Store new motion type
	mMotionType = inMotionType;

	if (mMotionProperties != nullptr)
	{
		// Update cache
		MOSS_IF_ENABLE_ASSERTS(mMotionProperties->mCachedMotionType = inMotionType;)

		switch (inMotionType)
		{
		case EMotionType::Static:
			// Stop the object
			mMotionProperties->mLinearVelocity = Vec3::sZero();
			mMotionProperties->mAngularVelocity = Vec3::sZero();
			[[fallthrough]];

		case EMotionType::Kinematic:
			// Cancel forces
			mMotionProperties->ResetForce();
			mMotionProperties->ResetTorque();
			break;

		case EMotionType::Dynamic:
			break;
		}
	}
}

void Body::SetAllowSleeping(bool inAllow)
{
	mMotionProperties->mAllowSleeping = inAllow;
	if (inAllow)
		ResetSleepTimer();
}

void Body::MoveKinematic(RVec3Arg inTargetPosition, QuatArg inTargetRotation, float inDeltaTime)
{
	MOSS_ASSERT(IsRigidBody()); // Only valid for rigid bodies
	MOSS_ASSERT(!IsStatic());
	MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sPositionAccess(), BodyAccess::EAccess::Read));

	// Calculate center of mass at end situation
	RVec3 new_com = inTargetPosition + inTargetRotation * mShape->GetCenterOfMass();

	// Calculate delta position and rotation
	Vec3 delta_pos = Vec3(new_com - mPosition);
	Quat delta_rotation = inTargetRotation * mRotation.Conjugated();

	mMotionProperties->MoveKinematic(delta_pos, delta_rotation, inDeltaTime);
}

void Body::CalculateWorldSpaceBoundsInternal()
{
	mBounds = mShape->GetWorldSpaceBounds(GetCenterOfMassTransform(), Vec3::sOne());
}

void Body::SetPositionAndRotationInternal(RVec3Arg inPosition, QuatArg inRotation, bool inResetSleepTimer)
{
	MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sPositionAccess(), BodyAccess::EAccess::ReadWrite));

	mPosition = inPosition + inRotation * mShape->GetCenterOfMass();
	mRotation = inRotation;

	// Initialize bounding box
	CalculateWorldSpaceBoundsInternal();

	// Reset sleeping test
	if (inResetSleepTimer && mMotionProperties != nullptr)
		ResetSleepTimer();
}

void Body::UpdateCenterOfMassInternal(Vec3Arg inPreviousCenterOfMass, bool inUpdateMassProperties)
{
	// Update center of mass position so the world position for this body stays the same
	mPosition += mRotation * (mShape->GetCenterOfMass() - inPreviousCenterOfMass);

	// Recalculate mass and inertia if requested
	if (inUpdateMassProperties && mMotionProperties != nullptr)
		mMotionProperties->SetMassProperties(mMotionProperties->GetAllowedDOFs(), mShape->GetMassProperties());
}

void Body::SetShapeInternal(const Shape *inShape, bool inUpdateMassProperties)
{
	MOSS_ASSERT(IsRigidBody()); // Only valid for rigid bodies
	MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sPositionAccess(), BodyAccess::EAccess::ReadWrite));

	// Get the old center of mass
	Vec3 old_com = mShape->GetCenterOfMass();

	// Update the shape
	mShape = inShape;

	// Update center of mass
	UpdateCenterOfMassInternal(old_com, inUpdateMassProperties);

	// Recalculate bounding box
	CalculateWorldSpaceBoundsInternal();
}

ECanSleep Body::UpdateSleepStateInternal(float inDeltaTime, float inMaxMovement, float inTimeBeforeSleep)
{
	// Check override & sensors will never go to sleep (they would stop detecting collisions with sleeping bodies)
	if (!mMotionProperties->mAllowSleeping || IsSensor())
		return ECanSleep::CannotSleep;

	// Get the points to test
	RVec3 points[3];
	GetSleepTestPoints(points);

#ifdef MOSS_DOUBLE_PRECISION
	// Get base offset for spheres
	DVec3 offset = mMotionProperties->GetSleepTestOffset();
#endif // MOSS_DOUBLE_PRECISION

	for (int i = 0; i < 3; ++i)
	{
		Sphere &sphere = mMotionProperties->mSleepTestSpheres[i];

		// Make point relative to base offset
#ifdef MOSS_DOUBLE_PRECISION
		Vec3 p = Vec3(points[i] - offset);
#else
		Vec3 p = points[i];
#endif // MOSS_DOUBLE_PRECISION

		// Encapsulate the point in a sphere
		sphere.EncapsulatePoint(p);

		// Test if it exceeded the max movement
		if (sphere.GetRadius() > inMaxMovement)
		{
			// Body is not sleeping, reset test
			mMotionProperties->ResetSleepTestSpheres(points);
			return ECanSleep::CannotSleep;
		}
	}

	return mMotionProperties->AccumulateSleepTime(inDeltaTime, inTimeBeforeSleep);
}

void Body::GetSubmergedVolume(RVec3Arg inSurfacePosition, Vec3Arg inSurfaceNormal, float &outTotalVolume, float &outSubmergedVolume, Vec3 &outRelativeCenterOfBuoyancy) const
{
	// For GetSubmergedVolume we transform the surface relative to the body position for increased precision
	Mat44 rotation = Mat44::sRotation(mRotation);
	Plane surface_relative_to_body = Plane::sFromPointAndNormal(inSurfacePosition - mPosition, inSurfaceNormal);

	// Calculate amount of volume that is submerged and what the center of buoyancy is
	mShape->GetSubmergedVolume(rotation, Vec3::sOne(), surface_relative_to_body, outTotalVolume, outSubmergedVolume, outRelativeCenterOfBuoyancy MOSS_IF_DEBUG_RENDERER(, mPosition));
}

bool Body::ApplyBuoyancyImpulse(float inTotalVolume, float inSubmergedVolume, Vec3Arg inRelativeCenterOfBuoyancy, float inBuoyancy, float inLinearDrag, float inAngularDrag, Vec3Arg inFluidVelocity, Vec3Arg inGravity, float inDeltaTime)
{
	MOSS_ASSERT(IsRigidBody()); // Only implemented for rigid bodies currently

	// We follow the approach from 'Game Programming Gems 6' 2.5 Exact Buoyancy for Polyhedra
	// All quantities below are in world space

	// If we're not submerged, there's no point in doing the rest of the calculations
	if (inSubmergedVolume > 0.0f)
	{
	#ifndef MOSS_DEBUG_RENDERER
		// Draw submerged volume properties
		if (Shape::sDrawSubmergedVolumes)
		{
			RVec3 center_of_buoyancy = mPosition + inRelativeCenterOfBuoyancy;
			DebugRenderer::sInstance->DrawMarker(center_of_buoyancy, Color::sWhite, 2.0f);
			DebugRenderer::sInstance->DrawText3D(center_of_buoyancy, StringFormat("%.3f / %.3f", (double)inSubmergedVolume, (double)inTotalVolume));
		}
	#endif // MOSS_DEBUG_RENDERER

		// When buoyancy is 1 we want neutral buoyancy, this means that the density of the liquid is the same as the density of the body at that point.
		// Buoyancy > 1 should make the object float, < 1 should make it sink.
		float inverse_mass = mMotionProperties->GetInverseMass();
		float fluid_density = inBuoyancy / (inTotalVolume * inverse_mass);

		// Buoyancy force = Density of Fluid * Submerged volume * Magnitude of gravity * Up direction (eq 2.5.1)
		// Impulse = Force * Delta time
		// We should apply this at the center of buoyancy (= center of mass of submerged volume)
		Vec3 buoyancy_impulse = -fluid_density * inSubmergedVolume * mMotionProperties->GetGravityFactor() * inGravity * inDeltaTime;

		// Calculate the velocity of the center of buoyancy relative to the fluid
		Vec3 linear_velocity = mMotionProperties->GetLinearVelocity();
		Vec3 angular_velocity = mMotionProperties->GetAngularVelocity();
		Vec3 center_of_buoyancy_velocity = linear_velocity + angular_velocity.Cross(inRelativeCenterOfBuoyancy);
		Vec3 relative_center_of_buoyancy_velocity = inFluidVelocity - center_of_buoyancy_velocity;

		// Here we deviate from the article, instead of eq 2.5.14 we use a quadratic drag formula: https://en.wikipedia.org/wiki/Drag_%28physics%29
		// Drag force = 0.5 * Fluid Density * (Velocity of fluid - Velocity of center of buoyancy)^2 * Linear Drag * Area Facing the Relative Fluid Velocity
		// Again Impulse = Force * Delta Time
		// We should apply this at the center of buoyancy (= center of mass for submerged volume with no center of mass offset)

		// Get size of local bounding box
		Vec3 size = mShape->GetLocalBounds().GetSize();

		// Determine area of the local space bounding box in the direction of the relative velocity between the fluid and the center of buoyancy
		float area = 0.0f;
		float relative_center_of_buoyancy_velocity_len_sq = relative_center_of_buoyancy_velocity.LengthSq();
		if (relative_center_of_buoyancy_velocity_len_sq > 1.0e-12f)
		{
			Vec3 local_relative_center_of_buoyancy_velocity = GetRotation().Conjugated() * relative_center_of_buoyancy_velocity;
			area = local_relative_center_of_buoyancy_velocity.Abs().Dot(size.Swizzle<SWIZZLE_Y, SWIZZLE_Z, SWIZZLE_X>() * size.Swizzle<SWIZZLE_Z, SWIZZLE_X, SWIZZLE_Y>()) / sqrt(relative_center_of_buoyancy_velocity_len_sq);
		}

		// Calculate the impulse
		Vec3 drag_impulse = (0.5f * fluid_density * inLinearDrag * area * inDeltaTime) * relative_center_of_buoyancy_velocity * relative_center_of_buoyancy_velocity.Length();

		// Clamp magnitude against current linear velocity to prevent overshoot
		float linear_velocity_len_sq = linear_velocity.LengthSq();
		float drag_delta_linear_velocity_len_sq = (drag_impulse * inverse_mass).LengthSq();
		if (drag_delta_linear_velocity_len_sq > linear_velocity_len_sq)
			drag_impulse *= sqrt(linear_velocity_len_sq / drag_delta_linear_velocity_len_sq);

		// Calculate the resulting delta linear velocity due to buoyancy and drag
		Vec3 delta_linear_velocity = (drag_impulse + buoyancy_impulse) * inverse_mass;
		mMotionProperties->AddLinearVelocityStep(delta_linear_velocity);

		// Determine average width of the body (across the three axis)
		float l = (size.GetX() + size.GetY() + size.GetZ()) / 3.0f;

		// Drag torque = -Angular Drag * Mass * Submerged volume / Total volume * (Average width of body)^2 * Angular velocity (eq 2.5.15)
		Vec3 drag_angular_impulse = (-inAngularDrag * inSubmergedVolume / inTotalVolume * inDeltaTime * Square(l) / inverse_mass) * angular_velocity;
		Mat44 inv_inertia = GetInverseInertia();
		Vec3 drag_delta_angular_velocity = inv_inertia * drag_angular_impulse;

		// Clamp magnitude against the current angular velocity to prevent overshoot
		float angular_velocity_len_sq = angular_velocity.LengthSq();
		float drag_delta_angular_velocity_len_sq = drag_delta_angular_velocity.LengthSq();
		if (drag_delta_angular_velocity_len_sq > angular_velocity_len_sq)
			drag_delta_angular_velocity *= sqrt(angular_velocity_len_sq / drag_delta_angular_velocity_len_sq);

		// Calculate total delta angular velocity due to drag and buoyancy
		Vec3 delta_angular_velocity = drag_delta_angular_velocity + inv_inertia * inRelativeCenterOfBuoyancy.Cross(buoyancy_impulse + drag_impulse);
		mMotionProperties->AddAngularVelocityStep(delta_angular_velocity);
		return true;
	}

	return false;
}

bool Body::ApplyBuoyancyImpulse(RVec3Arg inSurfacePosition, Vec3Arg inSurfaceNormal, float inBuoyancy, float inLinearDrag, float inAngularDrag, Vec3Arg inFluidVelocity, Vec3Arg inGravity, float inDeltaTime)
{
	MOSS_PROFILE_FUNCTION();

	float total_volume, submerged_volume;
	Vec3 relative_center_of_buoyancy;
	GetSubmergedVolume(inSurfacePosition, inSurfaceNormal, total_volume, submerged_volume, relative_center_of_buoyancy);

	return ApplyBuoyancyImpulse(total_volume, submerged_volume, relative_center_of_buoyancy, inBuoyancy, inLinearDrag, inAngularDrag, inFluidVelocity, inGravity, inDeltaTime);
}

void Body::SaveState(StateRecorder &inStream) const
{
	// Only write properties that can change at runtime
	inStream.Write(mPosition);
	inStream.Write(mRotation);

	if (mMotionProperties != nullptr)
	{
		if (IsSoftBody())
			static_cast<const SoftBodyMotionProperties *>(mMotionProperties)->SaveState(inStream);
		else
			mMotionProperties->SaveState(inStream);
	}
}

void Body::RestoreState(StateRecorder &inStream)
{
	inStream.Read(mPosition);
	inStream.Read(mRotation);

	if (mMotionProperties != nullptr)
	{
		if (IsSoftBody())
			static_cast<SoftBodyMotionProperties *>(mMotionProperties)->RestoreState(inStream);
		else
			mMotionProperties->RestoreState(inStream);

		MOSS_IF_ENABLE_ASSERTS(mMotionProperties->mCachedMotionType = mMotionType);
	}

	// Initialize bounding box
	CalculateWorldSpaceBoundsInternal();
}

BodyCreationSettings Body::GetBodyCreationSettings() const
{
	MOSS_ASSERT(IsRigidBody());

	BodyCreationSettings result;

	result.mPosition = GetPosition();
	result.mRotation = GetRotation();
	result.mLinearVelocity = mMotionProperties != nullptr? mMotionProperties->GetLinearVelocity() : Vec3::sZero();
	result.mAngularVelocity = mMotionProperties != nullptr? mMotionProperties->GetAngularVelocity() : Vec3::sZero();
	result.mObjectLayer = GetObjectLayer();
	result.mUserData = mUserData;
	result.mCollisionGroup = GetCollisionGroup();
	result.mMotionType = GetMotionType();
	result.mAllowedDOFs = mMotionProperties != nullptr? mMotionProperties->GetAllowedDOFs() : EAllowedDOFs::All;
	result.mAllowDynamicOrKinematic = mMotionProperties != nullptr;
	result.mIsSensor = IsSensor();
	result.mCollideKinematicVsNonDynamic = GetCollideKinematicVsNonDynamic();
	result.mUseManifoldReduction = GetUseManifoldReduction();
	result.mApplyGyroscopicForce = GetApplyGyroscopicForce();
	result.mMotionQuality = mMotionProperties != nullptr? mMotionProperties->GetMotionQuality() : EMotionQuality::Discrete;
	result.mEnhancedInternalEdgeRemoval = GetEnhancedInternalEdgeRemoval();
	result.mAllowSleeping = mMotionProperties != nullptr? GetAllowSleeping() : true;
	result.mFriction = GetFriction();
	result.mRestitution = GetRestitution();
	result.mLinearDamping = mMotionProperties != nullptr? mMotionProperties->GetLinearDamping() : 0.0f;
	result.mAngularDamping = mMotionProperties != nullptr? mMotionProperties->GetAngularDamping() : 0.0f;
	result.mMaxLinearVelocity = mMotionProperties != nullptr? mMotionProperties->GetMaxLinearVelocity() : 0.0f;
	result.mMaxAngularVelocity = mMotionProperties != nullptr? mMotionProperties->GetMaxAngularVelocity() : 0.0f;
	result.mGravityFactor = mMotionProperties != nullptr? mMotionProperties->GetGravityFactor() : 1.0f;
	result.mNumVelocityStepsOverride = mMotionProperties != nullptr? mMotionProperties->GetNumVelocityStepsOverride() : 0;
	result.mNumPositionStepsOverride = mMotionProperties != nullptr? mMotionProperties->GetNumPositionStepsOverride() : 0;
	result.mOverrideMassProperties = EOverrideMassProperties::MassAndInertiaProvided;

	// Invert inertia and mass
	if (mMotionProperties != nullptr)
	{
		float inv_mass = mMotionProperties->GetInverseMassUnchecked();
		Mat44 inv_inertia = mMotionProperties->GetLocalSpaceInverseInertiaUnchecked();

		// Get mass
		result.mMassPropertiesOverride.mMass = inv_mass != 0.0f? 1.0f / inv_mass : FLT_MAX;

		// Get inertia
		Mat44 inertia;
		if (inertia.SetInversed3x3(inv_inertia))
		{
			// Inertia was invertible, we can use it
			result.mMassPropertiesOverride.mInertia = inertia;
		}
		else
		{
			// Prevent division by zero
			Vec3 diagonal = Vec3::sMax(inv_inertia.GetDiagonal3(), Vec3::sReplicate(FLT_MIN));
			result.mMassPropertiesOverride.mInertia = Mat44::sScale(diagonal.Reciprocal());
		}
	}
	else
	{
		result.mMassPropertiesOverride.mMass = FLT_MAX;
		result.mMassPropertiesOverride.mInertia = Mat44::sScale(Vec3::sReplicate(FLT_MAX));
	}

	result.SetShape(GetShape());

	return result;
}

SoftBodyCreationSettings Body::GetSoftBodyCreationSettings() const
{
	MOSS_ASSERT(IsSoftBody());

	SoftBodyCreationSettings result;

	result.mPosition = GetPosition();
	result.mRotation = GetRotation();
	result.mUserData = mUserData;
	result.mObjectLayer = GetObjectLayer();
	result.mCollisionGroup = GetCollisionGroup();
	result.mFriction = GetFriction();
	result.mRestitution = GetRestitution();
	const SoftBodyMotionProperties *mp = static_cast<const SoftBodyMotionProperties *>(mMotionProperties);
	result.mNumIterations = mp->GetNumIterations();
	result.mLinearDamping = mp->GetLinearDamping();
	result.mMaxLinearVelocity = mp->GetMaxLinearVelocity();
	result.mGravityFactor = mp->GetGravityFactor();
	result.mPressure = mp->GetPressure();
	result.mUpdatePosition = mp->GetUpdatePosition();
	result.mSettings = mp->GetSettings();

	return result;
}

#ifdef MOSS_DEBUG
	static thread_local bool sOverrideAllowActivation = false;
	static thread_local bool sOverrideAllowDeactivation = false;

	bool BodyManager::sGetOverrideAllowActivation()
	{
		return sOverrideAllowActivation;
	}

	void BodyManager::sSetOverrideAllowActivation(bool inValue)
	{
		sOverrideAllowActivation = inValue;
	}

	bool BodyManager::sGetOverrideAllowDeactivation()
	{
		return sOverrideAllowDeactivation;
	}

	void BodyManager::sSetOverrideAllowDeactivation(bool inValue)
	{
		sOverrideAllowDeactivation = inValue;
	}
#endif

// Helper class that combines a body and its motion properties
class BodyWithMotionProperties : public Body
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	MotionProperties			mMotionProperties;
};

// Helper class that combines a soft body its motion properties and shape
class SoftBodyWithMotionPropertiesAndShape : public Body
{
public:
								SoftBodyWithMotionPropertiesAndShape()
	{
		mShape.SetEmbedded();
	}

	SoftBodyMotionProperties	mMotionProperties;
	SoftBodyShape				mShape;
};

inline void BodyManager::sDeleteBody(Body *inBody)
{
	if (inBody->mMotionProperties != nullptr)
	{
		MOSS_IF_ENABLE_ASSERTS(inBody->mMotionProperties = nullptr;)
		if (inBody->IsSoftBody())
		{
			inBody->mShape = nullptr; // Release the shape to avoid assertion on shape destruction because of embedded object with refcount > 0
			delete static_cast<SoftBodyWithMotionPropertiesAndShape *>(inBody);
		}
		else
			delete static_cast<BodyWithMotionProperties *>(inBody);
	}
	else
		delete inBody;
}

BodyManager::~BodyManager()
{
	UniqueLock lock(mBodiesMutex MOSS_IF_ENABLE_ASSERTS(, this, EPhysicsLockTypes::BodiesList));

	// Destroy any bodies that are still alive
	for (Body *b : mBodies)
		if (sIsValidBodyPointer(b))
			sDeleteBody(b);

	for (BodyID *active_bodies : mActiveBodies)
		delete [] active_bodies;
}

void BodyManager::Init(uint inMaxBodies, uint inNumBodyMutexes, const BroadPhaseLayerInterface &inLayerInterface)
{
	UniqueLock lock(mBodiesMutex MOSS_IF_ENABLE_ASSERTS(, this, EPhysicsLockTypes::BodiesList));

	// Num body mutexes must be a power of two and not bigger than our MutexMask
	uint num_body_mutexes = Clamp<uint>(GetNextPowerOf2(inNumBodyMutexes == 0? 2 * thread::hardware_concurrency() : inNumBodyMutexes), 1, sizeof(MutexMask) * 8);
#ifdef MOSS_TSAN_ENABLED
	num_body_mutexes = min(num_body_mutexes, 32U); // TSAN errors out when locking too many mutexes on the same thread, see: https://github.com/google/sanitizers/issues/950
#endif

	// Allocate the body mutexes
	mBodyMutexes.Init(num_body_mutexes);

	// Allocate space for bodies
	mBodies.reserve(inMaxBodies);

	// Allocate space for active bodies
	for (BodyID *&active_bodies : mActiveBodies)
	{
		MOSS_ASSERT(active_bodies == nullptr);
		active_bodies = new BodyID [inMaxBodies];
	}

	// Allocate space for sequence numbers
	mBodySequenceNumbers.resize(inMaxBodies, 0);

	// Keep layer interface
	mBroadPhaseLayerInterface = &inLayerInterface;
}

uint BodyManager::GetNumBodies() const
{
	UniqueLock lock(mBodiesMutex MOSS_IF_ENABLE_ASSERTS(, this, EPhysicsLockTypes::BodiesList));

	return mNumBodies;
}

BodyManager::BodyStats BodyManager::GetBodyStats() const
{
	UniqueLock lock(mBodiesMutex MOSS_IF_ENABLE_ASSERTS(, this, EPhysicsLockTypes::BodiesList));

	BodyStats stats;
	stats.mNumBodies = mNumBodies;
	stats.mMaxBodies = uint(mBodies.capacity());

	for (const Body *body : mBodies)
		if (sIsValidBodyPointer(body))
		{
			if (body->IsSoftBody())
			{
				stats.mNumSoftBodies++;
				if (body->IsActive())
					stats.mNumActiveSoftBodies++;
			}
			else
			{
				switch (body->GetMotionType())
				{
				case EMotionType::Static:
					stats.mNumBodiesStatic++;
					break;

				case EMotionType::Dynamic:
					stats.mNumBodiesDynamic++;
					if (body->IsActive())
						stats.mNumActiveBodiesDynamic++;
					break;

				case EMotionType::Kinematic:
					stats.mNumBodiesKinematic++;
					if (body->IsActive())
						stats.mNumActiveBodiesKinematic++;
					break;
				}
			}
		}

	return stats;
}

Body *BodyManager::AllocateBody(const BodyCreationSettings &inBodyCreationSettings) const
{
	// Fill in basic properties
	Body *body;
	if (inBodyCreationSettings.HasMassProperties())
	{
		BodyWithMotionProperties *bmp = new BodyWithMotionProperties;
		body = bmp;
		body->mMotionProperties = &bmp->mMotionProperties;
	}
	else
	{
		body = new Body;
	}
	body->mBodyType = EBodyType::RigidBody;
	body->mShape = inBodyCreationSettings.GetShape();
	body->mUserData = inBodyCreationSettings.mUserData;
	body->SetFriction(inBodyCreationSettings.mFriction);
	body->SetRestitution(inBodyCreationSettings.mRestitution);
	body->mMotionType = inBodyCreationSettings.mMotionType;
	if (inBodyCreationSettings.mIsSensor)
		body->SetIsSensor(true);
	if (inBodyCreationSettings.mCollideKinematicVsNonDynamic)
		body->SetCollideKinematicVsNonDynamic(true);
	if (inBodyCreationSettings.mUseManifoldReduction)
		body->SetUseManifoldReduction(true);
	if (inBodyCreationSettings.mApplyGyroscopicForce)
		body->SetApplyGyroscopicForce(true);
	if (inBodyCreationSettings.mEnhancedInternalEdgeRemoval)
		body->SetEnhancedInternalEdgeRemoval(true);
	SetBodyObjectLayerInternal(*body, inBodyCreationSettings.mObjectLayer);
	body->mObjectLayer = inBodyCreationSettings.mObjectLayer;
	body->mCollisionGroup = inBodyCreationSettings.mCollisionGroup;

	if (inBodyCreationSettings.HasMassProperties())
	{
		MotionProperties *mp = body->mMotionProperties;
		mp->SetLinearDamping(inBodyCreationSettings.mLinearDamping);
		mp->SetAngularDamping(inBodyCreationSettings.mAngularDamping);
		mp->SetMaxLinearVelocity(inBodyCreationSettings.mMaxLinearVelocity);
		mp->SetMaxAngularVelocity(inBodyCreationSettings.mMaxAngularVelocity);
		mp->SetMassProperties(inBodyCreationSettings.mAllowedDOFs, inBodyCreationSettings.GetMassProperties());
		mp->SetLinearVelocity(inBodyCreationSettings.mLinearVelocity); // Needs to happen after setting the max linear/angular velocity and setting allowed DOFs
		mp->SetAngularVelocity(inBodyCreationSettings.mAngularVelocity);
		mp->SetGravityFactor(inBodyCreationSettings.mGravityFactor);
		mp->SetNumVelocityStepsOverride(inBodyCreationSettings.mNumVelocityStepsOverride);
		mp->SetNumPositionStepsOverride(inBodyCreationSettings.mNumPositionStepsOverride);
		mp->mMotionQuality = inBodyCreationSettings.mMotionQuality;
		mp->mAllowSleeping = inBodyCreationSettings.mAllowSleeping;
		MOSS_IF_ENABLE_ASSERTS(mp->mCachedBodyType = body->mBodyType;)
		MOSS_IF_ENABLE_ASSERTS(mp->mCachedMotionType = body->mMotionType;)
	}

	// Position body
	body->SetPositionAndRotationInternal(inBodyCreationSettings.mPosition, inBodyCreationSettings.mRotation);

	return body;
}

/// Create a soft body using creation settings. The returned body will not be part of the body manager yet.
Body *BodyManager::AllocateSoftBody(const SoftBodyCreationSettings &inSoftBodyCreationSettings) const
{
	// Fill in basic properties
	SoftBodyWithMotionPropertiesAndShape *bmp = new SoftBodyWithMotionPropertiesAndShape;
	SoftBodyMotionProperties *mp = &bmp->mMotionProperties;
	SoftBodyShape *shape = &bmp->mShape;
	Body *body = bmp;
	shape->mSoftBodyMotionProperties = mp;
	body->mBodyType = EBodyType::SoftBody;
	body->mMotionProperties = mp;
	body->mShape = shape;
	body->mUserData = inSoftBodyCreationSettings.mUserData;
	body->SetFriction(inSoftBodyCreationSettings.mFriction);
	body->SetRestitution(inSoftBodyCreationSettings.mRestitution);
	body->mMotionType = EMotionType::Dynamic;
	SetBodyObjectLayerInternal(*body, inSoftBodyCreationSettings.mObjectLayer);
	body->mObjectLayer = inSoftBodyCreationSettings.mObjectLayer;
	body->mCollisionGroup = inSoftBodyCreationSettings.mCollisionGroup;
	mp->SetLinearDamping(inSoftBodyCreationSettings.mLinearDamping);
	mp->SetAngularDamping(0);
	mp->SetMaxLinearVelocity(inSoftBodyCreationSettings.mMaxLinearVelocity);
	mp->SetMaxAngularVelocity(FLT_MAX);
	mp->SetLinearVelocity(Vec3::sZero());
	mp->SetAngularVelocity(Vec3::sZero());
	mp->SetGravityFactor(inSoftBodyCreationSettings.mGravityFactor);
	mp->mMotionQuality = EMotionQuality::Discrete;
	mp->mAllowSleeping = inSoftBodyCreationSettings.mAllowSleeping;
	MOSS_IF_ENABLE_ASSERTS(mp->mCachedBodyType = body->mBodyType;)
	MOSS_IF_ENABLE_ASSERTS(mp->mCachedMotionType = body->mMotionType;)
	mp->Initialize(inSoftBodyCreationSettings);

	body->SetPositionAndRotationInternal(inSoftBodyCreationSettings.mPosition, inSoftBodyCreationSettings.mMakeRotationIdentity? Quat::sIdentity() : inSoftBodyCreationSettings.mRotation);

	return body;
}

void BodyManager::FreeBody(Body *inBody) const
{
	MOSS_ASSERT(inBody->GetID().IsInvalid(), "This function should only be called on a body that doesn't have an ID yet, use DestroyBody otherwise");

	sDeleteBody(inBody);
}

bool BodyManager::AddBody(Body *ioBody)
{
	// Return error when body was already added
	if (!ioBody->GetID().IsInvalid())
		return false;

	// Determine next free index
	uint32 idx;
	{
		UniqueLock lock(mBodiesMutex MOSS_IF_ENABLE_ASSERTS(, this, EPhysicsLockTypes::BodiesList));

		if (mBodyIDFreeListStart != cBodyIDFreeListEnd)
		{
			// Pop an item from the freelist
			MOSS_ASSERT(mBodyIDFreeListStart & cIsFreedBody);
			idx = uint32(mBodyIDFreeListStart >> cFreedBodyIndexShift);
			MOSS_ASSERT(!sIsValidBodyPointer(mBodies[idx]));
			mBodyIDFreeListStart = uintptr_t(mBodies[idx]);
			mBodies[idx] = ioBody;
		}
		else
		{
			if (mBodies.size() < mBodies.capacity())
			{
				// Allocate a new entry, note that the array should not actually resize since we've reserved it at init time
				idx = uint32(mBodies.size());
				mBodies.push_back(ioBody);
			}
			else
			{
				// Out of bodies
				return false;
			}
		}

		// Update cached number of bodies
		mNumBodies++;
	}

	// Get next sequence number and assign the ID
	uint8 seq_no = GetNextSequenceNumber(idx);
	ioBody->mID = BodyID(idx, seq_no);
	return true;
}

bool BodyManager::AddBodyWithCustomID(Body *ioBody, const BodyID &inBodyID)
{
	// Return error when body was already added
	if (!ioBody->GetID().IsInvalid())
		return false;

	{
		UniqueLock lock(mBodiesMutex MOSS_IF_ENABLE_ASSERTS(, this, EPhysicsLockTypes::BodiesList));

		// Check if index is beyond the max body ID
		uint32 idx = inBodyID.GetIndex();
		if (idx >= mBodies.capacity())
			return false; // Return error

		if (idx < mBodies.size())
		{
			// Body array entry has already been allocated, check if there's a free body here
			if (sIsValidBodyPointer(mBodies[idx]))
				return false; // Return error

			// Remove the entry from the freelist
			uintptr_t idx_start = mBodyIDFreeListStart >> cFreedBodyIndexShift;
			if (idx == idx_start)
			{
				// First entry, easy to remove, the start of the list is our next
				mBodyIDFreeListStart = uintptr_t(mBodies[idx]);
			}
			else
			{
				// Loop over the freelist and find the entry in the freelist pointing to our index
				// TODO: This is O(N), see if this becomes a performance problem (don't want to put the freed bodies in a double linked list)
				uintptr_t cur, next;
				for (cur = idx_start; cur != cBodyIDFreeListEnd >> cFreedBodyIndexShift; cur = next)
				{
					next = uintptr_t(mBodies[cur]) >> cFreedBodyIndexShift;
					if (next == idx)
					{
						mBodies[cur] = mBodies[idx];
						break;
					}
				}
				MOSS_ASSERT(cur != cBodyIDFreeListEnd >> cFreedBodyIndexShift);
			}

			// Put the body in the slot
			mBodies[idx] = ioBody;
		}
		else
		{
			// Ensure that all body IDs up to this body ID have been allocated and added to the free list
			while (idx > mBodies.size())
			{
				// Push the id onto the freelist
				mBodies.push_back((Body *)mBodyIDFreeListStart);
				mBodyIDFreeListStart = (uintptr_t(mBodies.size() - 1) << cFreedBodyIndexShift) | cIsFreedBody;
			}

			// Add the element to the list
			mBodies.push_back(ioBody);
		}

		// Update cached number of bodies
		mNumBodies++;
	}

	// Assign the ID
	ioBody->mID = inBodyID;
	return true;
}

Body *BodyManager::RemoveBodyInternal(const BodyID &inBodyID)
{
	// Get body
	uint32 idx = inBodyID.GetIndex();
	Body *body = mBodies[idx];

	// Validate that it can be removed
	MOSS_ASSERT(body->GetID() == inBodyID);
	MOSS_ASSERT(!body->IsActive());
	MOSS_ASSERT(!body->IsInBroadPhase(), "Use BodyInterface::RemoveBody to remove this body first!");

	// Push the id onto the freelist
	mBodies[idx] = (Body *)mBodyIDFreeListStart;
	mBodyIDFreeListStart = (uintptr_t(idx) << cFreedBodyIndexShift) | cIsFreedBody;

	return body;
}

#if defined(MOSS_DEBUG)

void BodyManager::ValidateFreeList() const
{
	// Check that the freelist is correct
	size_t num_freed = 0;
	for (uintptr_t start = mBodyIDFreeListStart; start != cBodyIDFreeListEnd; start = uintptr_t(mBodies[start >> cFreedBodyIndexShift]))
	{
		MOSS_ASSERT(start & cIsFreedBody);
		num_freed++;
	}
	MOSS_ASSERT(mNumBodies == mBodies.size() - num_freed);
}

#endif // defined(MOSS_DEBUG)

void BodyManager::RemoveBodies(const BodyID *inBodyIDs, int inNumber, Body **outBodies)
{
	// Don't take lock if no bodies are to be destroyed
	if (inNumber <= 0)
		return;

	UniqueLock lock(mBodiesMutex MOSS_IF_ENABLE_ASSERTS(, this, EPhysicsLockTypes::BodiesList));

	// Update cached number of bodies
	MOSS_ASSERT(mNumBodies >= (uint)inNumber);
	mNumBodies -= inNumber;

	for (const BodyID *b = inBodyIDs, *b_end = inBodyIDs + inNumber; b < b_end; b++)
	{
		// Remove body
		Body *body = RemoveBodyInternal(*b);

		// Clear the ID
		body->mID = BodyID();

		// Return the body to the caller
		if (outBodies != nullptr)
		{
			*outBodies = body;
			++outBodies;
		}
	}

#if defined(MOSS_DEBUG)
	ValidateFreeList();
#endif // defined(MOSS_DEBUG)
}

void BodyManager::DestroyBodies(const BodyID *inBodyIDs, int inNumber)
{
	// Don't take lock if no bodies are to be destroyed
	if (inNumber <= 0)
		return;

	UniqueLock lock(mBodiesMutex MOSS_IF_ENABLE_ASSERTS(, this, EPhysicsLockTypes::BodiesList));

	// Update cached number of bodies
	MOSS_ASSERT(mNumBodies >= (uint)inNumber);
	mNumBodies -= inNumber;

	for (const BodyID *b = inBodyIDs, *b_end = inBodyIDs + inNumber; b < b_end; b++)
	{
		// Remove body
		Body *body = RemoveBodyInternal(*b);

		// Free the body
		sDeleteBody(body);
	}

#if defined(MOSS_DEBUG)
	ValidateFreeList();
#endif // defined(MOSS_DEBUG)
}

void BodyManager::AddBodyToActiveBodies(Body &ioBody)
{
	// Select the correct array to use
	int type = (int)ioBody.GetBodyType();
	atomic<uint32> &num_active_bodies = mNumActiveBodies[type];
	BodyID *active_bodies = mActiveBodies[type];

	MotionProperties *mp = ioBody.mMotionProperties;
	uint32 num_active_bodies_val = num_active_bodies.load(memory_order_relaxed);
	mp->mIndexInActiveBodies = num_active_bodies_val;
	MOSS_ASSERT(num_active_bodies_val < GetMaxBodies());
	active_bodies[num_active_bodies_val] = ioBody.GetID();
	num_active_bodies.fetch_add(1, memory_order_release); // Increment atomic after setting the body ID so that PhysicsSystem::JobFindCollisions (which doesn't lock the mActiveBodiesMutex) will only read valid IDs

	// Count CCD bodies
	if (mp->GetMotionQuality() == EMotionQuality::LinearCast)
		mNumActiveCCDBodies++;
}

void BodyManager::RemoveBodyFromActiveBodies(Body &ioBody)
{
	// Select the correct array to use
	int type = (int)ioBody.GetBodyType();
	atomic<uint32> &num_active_bodies = mNumActiveBodies[type];
	BodyID *active_bodies = mActiveBodies[type];

	uint32 last_body_index = num_active_bodies.load(memory_order_relaxed) - 1;
	MotionProperties *mp = ioBody.mMotionProperties;
	if (mp->mIndexInActiveBodies != last_body_index)
	{
		// This is not the last body, use the last body to fill the hole
		BodyID last_body_id = active_bodies[last_body_index];
		active_bodies[mp->mIndexInActiveBodies] = last_body_id;

		// Update that body's index in the active list
		Body &last_body = *mBodies[last_body_id.GetIndex()];
		MOSS_ASSERT(last_body.mMotionProperties->mIndexInActiveBodies == last_body_index);
		last_body.mMotionProperties->mIndexInActiveBodies = mp->mIndexInActiveBodies;
	}

	// Mark this body as no longer active
	mp->mIndexInActiveBodies = Body::cInactiveIndex;

	// Remove unused element from active bodies list
	num_active_bodies.fetch_sub(1, memory_order_release);

	// Count CCD bodies
	if (mp->GetMotionQuality() == EMotionQuality::LinearCast)
		mNumActiveCCDBodies--;
}

void BodyManager::ActivateBodies(const BodyID *inBodyIDs, int inNumber)
{
	// Don't take lock if no bodies are to be activated
	if (inNumber <= 0)
		return;

	UniqueLock lock(mActiveBodiesMutex MOSS_IF_ENABLE_ASSERTS(, this, EPhysicsLockTypes::ActiveBodiesList));

	MOSS_ASSERT(!mActiveBodiesLocked || sOverrideAllowActivation);

	for (const BodyID *b = inBodyIDs, *b_end = inBodyIDs + inNumber; b < b_end; b++)
		if (!b->IsInvalid())
		{
			BodyID body_id = *b;
			Body &body = *mBodies[body_id.GetIndex()];

			MOSS_ASSERT(body.GetID() == body_id);
			MOSS_ASSERT(body.IsInBroadPhase(), "Use BodyInterface::AddBody to add the body first!");

			if (!body.IsStatic())
			{
				// Reset sleeping timer so that we don't immediately go to sleep again
				body.ResetSleepTimer();

				// Check if we're sleeping
				if (body.mMotionProperties->mIndexInActiveBodies == Body::cInactiveIndex)
				{
					AddBodyToActiveBodies(body);

					// Call activation listener
					if (mActivationListener != nullptr)
						mActivationListener->OnBodyActivated(body_id, body.GetUserData());
				}
			}
		}
}

void BodyManager::DeactivateBodies(const BodyID *inBodyIDs, int inNumber)
{
	// Don't take lock if no bodies are to be deactivated
	if (inNumber <= 0)
		return;

	UniqueLock lock(mActiveBodiesMutex MOSS_IF_ENABLE_ASSERTS(, this, EPhysicsLockTypes::ActiveBodiesList));

	MOSS_ASSERT(!mActiveBodiesLocked || sOverrideAllowDeactivation);

	for (const BodyID *b = inBodyIDs, *b_end = inBodyIDs + inNumber; b < b_end; b++)
		if (!b->IsInvalid())
		{
			BodyID body_id = *b;
			Body &body = *mBodies[body_id.GetIndex()];

			MOSS_ASSERT(body.GetID() == body_id);
			MOSS_ASSERT(body.IsInBroadPhase(), "Use BodyInterface::AddBody to add the body first!");

			if (body.mMotionProperties != nullptr
				&& body.mMotionProperties->mIndexInActiveBodies != Body::cInactiveIndex)
			{
				// Remove the body from the active bodies list
				RemoveBodyFromActiveBodies(body);

				// Mark this body as no longer active
				body.mMotionProperties->mIslandIndex = Body::cInactiveIndex;

				// Reset velocity
				body.mMotionProperties->mLinearVelocity = Vec3::sZero();
				body.mMotionProperties->mAngularVelocity = Vec3::sZero();

				// Call activation listener
				if (mActivationListener != nullptr)
					mActivationListener->OnBodyDeactivated(body_id, body.GetUserData());
			}
		}
}

void BodyManager::SetMotionQuality(Body &ioBody, EMotionQuality inMotionQuality)
{
	MotionProperties *mp = ioBody.GetMotionPropertiesUnchecked();
	if (mp != nullptr && mp->GetMotionQuality() != inMotionQuality)
	{
		UniqueLock lock(mActiveBodiesMutex MOSS_IF_ENABLE_ASSERTS(, this, EPhysicsLockTypes::ActiveBodiesList));

		MOSS_ASSERT(!mActiveBodiesLocked);

		bool is_active = ioBody.IsActive();
		if (is_active && mp->GetMotionQuality() == EMotionQuality::LinearCast)
			--mNumActiveCCDBodies;

		mp->mMotionQuality = inMotionQuality;

		if (is_active && mp->GetMotionQuality() == EMotionQuality::LinearCast)
			++mNumActiveCCDBodies;
	}
}

void BodyManager::GetActiveBodies(EBodyType inType, BodyIDVector &outBodyIDs) const
{
	MOSS_PROFILE_FUNCTION();

	UniqueLock lock(mActiveBodiesMutex MOSS_IF_ENABLE_ASSERTS(, this, EPhysicsLockTypes::ActiveBodiesList));

	const BodyID *active_bodies = mActiveBodies[(int)inType];
	outBodyIDs.assign(active_bodies, active_bodies + mNumActiveBodies[(int)inType].load(memory_order_relaxed));
}

void BodyManager::GetBodyIDs(BodyIDVector &outBodies) const
{
	MOSS_PROFILE_FUNCTION();

	UniqueLock lock(mBodiesMutex MOSS_IF_ENABLE_ASSERTS(, this, EPhysicsLockTypes::BodiesList));

	// Reserve space for all bodies
	outBodies.clear();
	outBodies.reserve(mNumBodies);

	// Iterate the list and find the bodies that are not null
	for (const Body *b : mBodies)
		if (sIsValidBodyPointer(b))
			outBodies.push_back(b->GetID());

	// Validate that our reservation was correct
	MOSS_ASSERT(outBodies.size() == mNumBodies);
}

void BodyManager::SetBodyActivationListener(BodyActivationListener *inListener)
{
	UniqueLock lock(mActiveBodiesMutex MOSS_IF_ENABLE_ASSERTS(, this, EPhysicsLockTypes::ActiveBodiesList));

	mActivationListener = inListener;
}

BodyManager::MutexMask BodyManager::GetMutexMask(const BodyID *inBodies, int inNumber) const
{
	MOSS_ASSERT(sizeof(MutexMask) * 8 >= mBodyMutexes.GetNumMutexes(), "MutexMask must have enough bits");

	if (inNumber >= (int)mBodyMutexes.GetNumMutexes())
	{
		// Just lock everything if there are too many bodies
		return GetAllBodiesMutexMask();
	}
	else
	{
		MutexMask mask = 0;
		for (const BodyID *b = inBodies, *b_end = inBodies + inNumber; b < b_end; ++b)
			if (!b->IsInvalid())
			{
				uint32 index = mBodyMutexes.GetMutexIndex(b->GetIndex());
				mask |= (MutexMask(1) << index);
			}
		return mask;
	}
}

void BodyManager::LockRead(MutexMask inMutexMask) const
{
	MOSS_IF_ENABLE_ASSERTS(PhysicsLock::sCheckLock(this, EPhysicsLockTypes::PerBody));

	int index = 0;
	for (MutexMask mask = inMutexMask; mask != 0; mask >>= 1, index++)
		if (mask & 1)
			mBodyMutexes.GetMutexByIndex(index).lock_shared();
}

void BodyManager::UnlockRead(MutexMask inMutexMask) const
{
	MOSS_IF_ENABLE_ASSERTS(PhysicsLock::sCheckUnlock(this, EPhysicsLockTypes::PerBody));

	int index = 0;
	for (MutexMask mask = inMutexMask; mask != 0; mask >>= 1, index++)
		if (mask & 1)
			mBodyMutexes.GetMutexByIndex(index).unlock_shared();
}

void BodyManager::LockWrite(MutexMask inMutexMask) const
{
	MOSS_IF_ENABLE_ASSERTS(PhysicsLock::sCheckLock(this, EPhysicsLockTypes::PerBody));

	int index = 0;
	for (MutexMask mask = inMutexMask; mask != 0; mask >>= 1, index++)
		if (mask & 1)
			mBodyMutexes.GetMutexByIndex(index).lock();
}

void BodyManager::UnlockWrite(MutexMask inMutexMask) const
{
	MOSS_IF_ENABLE_ASSERTS(PhysicsLock::sCheckUnlock(this, EPhysicsLockTypes::PerBody));

	int index = 0;
	for (MutexMask mask = inMutexMask; mask != 0; mask >>= 1, index++)
		if (mask & 1)
			mBodyMutexes.GetMutexByIndex(index).unlock();
}

void BodyManager::LockAllBodies() const
{
	MOSS_IF_ENABLE_ASSERTS(PhysicsLock::sCheckLock(this, EPhysicsLockTypes::PerBody));
	mBodyMutexes.LockAll();

	PhysicsLock::sLock(mBodiesMutex MOSS_IF_ENABLE_ASSERTS(, this, EPhysicsLockTypes::BodiesList));
}

void BodyManager::UnlockAllBodies() const
{
	PhysicsLock::sUnlock(mBodiesMutex MOSS_IF_ENABLE_ASSERTS(, this, EPhysicsLockTypes::BodiesList));

	MOSS_IF_ENABLE_ASSERTS(PhysicsLock::sCheckUnlock(this, EPhysicsLockTypes::PerBody));
	mBodyMutexes.UnlockAll();
}

void BodyManager::SaveState(StateRecorder &inStream, const StateRecorderFilter *inFilter) const
{
	{
		LockAllBodies();

		// Determine which bodies to save
		TArray<const Body *> bodies;
		bodies.reserve(mNumBodies);
		for (const Body *b : mBodies)
			if (sIsValidBodyPointer(b) && b->IsInBroadPhase() && (inFilter == nullptr || inFilter->ShouldSaveBody(*b)))
				bodies.push_back(b);

		// Write state of bodies
		uint32 num_bodies = (uint32)bodies.size();
		inStream.Write(num_bodies);
		for (const Body *b : bodies)
		{
			inStream.Write(b->GetID());
			inStream.Write(b->IsActive());
			b->SaveState(inStream);
		}

		UnlockAllBodies();
	}
}

bool BodyManager::RestoreState(StateRecorder &inStream)
{
	BodyIDVector bodies_to_activate, bodies_to_deactivate;

	{
		LockAllBodies();

		if (inStream.IsValidating())
		{
			// Read state of bodies, note this reads it in a way to be consistent with validation
			uint32 old_num_bodies = 0;
			for (const Body *b : mBodies)
				if (sIsValidBodyPointer(b) && b->IsInBroadPhase())
					++old_num_bodies;
			uint32 num_bodies = old_num_bodies; // Initialize to current value for validation
			inStream.Read(num_bodies);
			if (num_bodies != old_num_bodies)
			{
				MOSS_ASSERT(false, "Cannot handle adding/removing bodies");
				UnlockAllBodies();
				return false;
			}

			for (Body *b : mBodies)
				if (sIsValidBodyPointer(b) && b->IsInBroadPhase())
				{
					BodyID body_id = b->GetID(); // Initialize to current value for validation
					inStream.Read(body_id);
					if (body_id != b->GetID())
					{
						MOSS_ASSERT(false, "Cannot handle adding/removing bodies");
						UnlockAllBodies();
						return false;
					}
					bool is_active = b->IsActive(); // Initialize to current value for validation
					inStream.Read(is_active);
					if (is_active != b->IsActive())
					{
						if (is_active)
							bodies_to_activate.push_back(body_id);
						else
							bodies_to_deactivate.push_back(body_id);
					}
					b->RestoreState(inStream);
				}
		}
		else
		{
			// Not validating, we can be a bit more loose, read number of bodies
			uint32 num_bodies = 0;
			inStream.Read(num_bodies);

			// Iterate over the stored bodies and restore their state
			for (uint32 idx = 0; idx < num_bodies; ++idx)
			{
				BodyID body_id;
				inStream.Read(body_id);
				Body *b = TryGetBody(body_id);
				if (b == nullptr)
				{
					MOSS_ASSERT(false, "Restoring state for non-existing body");
					UnlockAllBodies();
					return false;
				}
				bool is_active;
				inStream.Read(is_active);
				if (is_active != b->IsActive())
				{
					if (is_active)
						bodies_to_activate.push_back(body_id);
					else
						bodies_to_deactivate.push_back(body_id);
				}
				b->RestoreState(inStream);
			}
		}

		UnlockAllBodies();
	}

	{
		UniqueLock lock(mActiveBodiesMutex MOSS_IF_ENABLE_ASSERTS(, this, EPhysicsLockTypes::ActiveBodiesList));

		for (BodyID body_id : bodies_to_activate)
		{
			Body *body = TryGetBody(body_id);
			AddBodyToActiveBodies(*body);
		}

		for (BodyID body_id : bodies_to_deactivate)
		{
			Body *body = TryGetBody(body_id);
			RemoveBodyFromActiveBodies(*body);
		}
	}

	return true;
}

void BodyManager::SaveBodyState(const Body &inBody, StateRecorder &inStream) const
{
	inStream.Write(inBody.IsActive());

	inBody.SaveState(inStream);
}

void BodyManager::RestoreBodyState(Body &ioBody, StateRecorder &inStream)
{
	bool is_active = ioBody.IsActive();
	inStream.Read(is_active);

	ioBody.RestoreState(inStream);

	if (is_active != ioBody.IsActive())
	{
		UniqueLock lock(mActiveBodiesMutex MOSS_IF_ENABLE_ASSERTS(, this, EPhysicsLockTypes::ActiveBodiesList));

		MOSS_ASSERT(!mActiveBodiesLocked || sOverrideAllowActivation);

		if (is_active)
			AddBodyToActiveBodies(ioBody);
		else
			RemoveBodyFromActiveBodies(ioBody);
	}
}

#ifndef MOSS_DEBUG_RENDERER
void BodyManager::Draw(const DrawSettings &inDrawSettings, const PhysicsSettings &inPhysicsSettings, DebugRenderer *inRenderer, const BodyDrawFilter *inBodyFilter)
{
	MOSS_PROFILE_FUNCTION();

	LockAllBodies();

	for (const Body *body : mBodies)
		if (sIsValidBodyPointer(body) && body->IsInBroadPhase() && (!inBodyFilter || inBodyFilter->ShouldDraw(*body)))
		{
			MOSS_ASSERT(mBodies[body->GetID().GetIndex()] == body);

			bool is_sensor = body->IsSensor();

			// Determine drawing mode
			Color color;
			if (is_sensor)
				color = Color::sYellow;
			else
				switch (inDrawSettings.mDrawShapeColor)
				{
				case EShapeColor::InstanceColor:
					// Each instance has own color
					color = Color::sGetDistinctColor(body->mID.GetIndex());
					break;

				case EShapeColor::ShapeTypeColor:
					color = ShapeFunctions::sGet(body->GetShape()->GetSubType()).mColor;
					break;

				case EShapeColor::MotionTypeColor:
					// Determine color based on motion type
					switch (body->mMotionType)
					{
					case EMotionType::Static:
						color = Color::sGrey;
						break;

					case EMotionType::Kinematic:
						color = Color::sGreen;
						break;

					case EMotionType::Dynamic:
						color = Color::sGetDistinctColor(body->mID.GetIndex());
						break;

					default:
						MOSS_ASSERT(false);
						color = Color::sBlack;
						break;
					}
					break;

				case EShapeColor::SleepColor:
					// Determine color based on motion type
					switch (body->mMotionType)
					{
					case EMotionType::Static:
						color = Color::sGrey;
						break;

					case EMotionType::Kinematic:
						color = body->IsActive()? Color::sGreen : Color::sRed;
						break;

					case EMotionType::Dynamic:
						color = body->IsActive()? Color::sYellow : Color::sRed;
						break;

					default:
						MOSS_ASSERT(false);
						color = Color::sBlack;
						break;
					}
					break;

				case EShapeColor::IslandColor:
					// Determine color based on motion type
					switch (body->mMotionType)
					{
					case EMotionType::Static:
						color = Color::sGrey;
						break;

					case EMotionType::Kinematic:
					case EMotionType::Dynamic:
						{
							uint32 idx = body->GetMotionProperties()->GetIslandIndexInternal();
							color = idx != Body::cInactiveIndex? Color::sGetDistinctColor(idx) : Color::sLightGrey;
						}
						break;

					default:
						MOSS_ASSERT(false);
						color = Color::sBlack;
						break;
					}
					break;

				case EShapeColor::MaterialColor:
					color = Color::sWhite;
					break;

				default:
					MOSS_ASSERT(false);
					color = Color::sBlack;
					break;
				}

			// Draw the results of GetSupportFunction
			if (inDrawSettings.mDrawGetSupportFunction)
				body->mShape->DrawGetSupportFunction(inRenderer, body->GetCenterOfMassTransform(), Vec3::sOne(), color, inDrawSettings.mDrawSupportDirection);

			// Draw the results of GetSupportingFace
			if (inDrawSettings.mDrawGetSupportingFace)
				body->mShape->DrawGetSupportingFace(inRenderer, body->GetCenterOfMassTransform(), Vec3::sOne());

			// Draw the shape
			if (inDrawSettings.mDrawShape)
				body->mShape->Draw(inRenderer, body->GetCenterOfMassTransform(), Vec3::sOne(), color, inDrawSettings.mDrawShapeColor == EShapeColor::MaterialColor, inDrawSettings.mDrawShapeWireframe || is_sensor);

			// Draw bounding box
			if (inDrawSettings.mDrawBoundingBox)
				inRenderer->DrawWireBox(body->mBounds, color);

			// Draw center of mass transform
			if (inDrawSettings.mDrawCenterOfMassTransform)
				inRenderer->DrawCoordinateSystem(body->GetCenterOfMassTransform(), 0.2f);

			// Draw world transform
			if (inDrawSettings.mDrawWorldTransform)
				inRenderer->DrawCoordinateSystem(body->GetWorldTransform(), 0.2f);

			// Draw world space linear and angular velocity
			if (inDrawSettings.mDrawVelocity)
			{
				RVec3 pos = body->GetCenterOfMassPosition();
				inRenderer->DrawArrow(pos, pos + body->GetLinearVelocity(), Color::sGreen, 0.1f);
				inRenderer->DrawArrow(pos, pos + body->GetAngularVelocity(), Color::sRed, 0.1f);
			}

			if (inDrawSettings.mDrawMassAndInertia && body->IsDynamic())
			{
				const MotionProperties *mp = body->GetMotionProperties();
				if (mp->GetInverseMass() > 0.0f
					&& !Vec3::sEquals(mp->GetInverseInertiaDiagonal(), Vec3::sZero()).TestAnyXYZTrue())
				{
					// Invert mass again
					float mass = 1.0f / mp->GetInverseMass();

					// Invert diagonal again
					Vec3 diagonal = mp->GetInverseInertiaDiagonal().Reciprocal();

					// Determine how big of a box has the equivalent inertia
					Vec3 box_size = MassProperties::sGetEquivalentSolidBoxSize(mass, diagonal);

					// Draw box with equivalent inertia
					inRenderer->DrawWireBox(body->GetCenterOfMassTransform() * Mat44::sRotation(mp->GetInertiaRotation()), AABox(-0.5f * box_size, 0.5f * box_size), Color::sOrange);

					// Draw mass
					inRenderer->DrawText3D(body->GetCenterOfMassPosition(), StringFormat("%.2f", (double)mass), Color::sOrange, 0.2f);
				}
			}

			if (inDrawSettings.mDrawSleepStats && body->IsDynamic() && body->IsActive())
			{
				// Draw stats to know which bodies could go to sleep
				String text = StringFormat("t: %.1f", (double)body->mMotionProperties->mSleepTestTimer);
				uint8 g = uint8(Clamp(255.0f * body->mMotionProperties->mSleepTestTimer / inPhysicsSettings.mTimeBeforeSleep, 0.0f, 255.0f));
				Color sleep_color = Color(0, 255 - g, g);
				inRenderer->DrawText3D(body->GetCenterOfMassPosition(), text, sleep_color, 0.2f);
				for (int i = 0; i < 3; ++i)
					inRenderer->DrawWireSphere(MOSS_IF_DOUBLE_PRECISION(body->mMotionProperties->GetSleepTestOffset() +) body->mMotionProperties->mSleepTestSpheres[i].GetCenter(), body->mMotionProperties->mSleepTestSpheres[i].GetRadius(), sleep_color);
			}

			if (body->IsSoftBody())
			{
				const SoftBodyMotionProperties *mp = static_cast<const SoftBodyMotionProperties *>(body->GetMotionProperties());
				RMat44 com = body->GetCenterOfMassTransform();

				if (inDrawSettings.mDrawSoftBodyVertices)
					mp->DrawVertices(inRenderer, com);

				if (inDrawSettings.mDrawSoftBodyVertexVelocities)
					mp->DrawVertexVelocities(inRenderer, com);

				if (inDrawSettings.mDrawSoftBodyEdgeConstraints)
					mp->DrawEdgeConstraints(inRenderer, com, inDrawSettings.mDrawSoftBodyConstraintColor);

				if (inDrawSettings.mDrawSoftBodyBendConstraints)
					mp->DrawBendConstraints(inRenderer, com, inDrawSettings.mDrawSoftBodyConstraintColor);

				if (inDrawSettings.mDrawSoftBodyVolumeConstraints)
					mp->DrawVolumeConstraints(inRenderer, com, inDrawSettings.mDrawSoftBodyConstraintColor);

				if (inDrawSettings.mDrawSoftBodySkinConstraints)
					mp->DrawSkinConstraints(inRenderer, com, inDrawSettings.mDrawSoftBodyConstraintColor);

				if (inDrawSettings.mDrawSoftBodyLRAConstraints)
					mp->DrawLRAConstraints(inRenderer, com, inDrawSettings.mDrawSoftBodyConstraintColor);

				if (inDrawSettings.mDrawSoftBodyPredictedBounds)
					mp->DrawPredictedBounds(inRenderer, com);
			}
		}

	UnlockAllBodies();
}
#endif // MOSS_DEBUG_RENDERER

void BodyManager::InvalidateContactCacheForBody(Body &ioBody)
{
	// If this is the first time we flip the collision cache invalid flag, we need to add it to an internal list to ensure we reset the flag at the end of the physics update
	if (ioBody.InvalidateContactCacheInternal())
	{
		lock_guard lock(mBodiesCacheInvalidMutex);
		mBodiesCacheInvalid.push_back(ioBody.GetID());
	}
}

void BodyManager::ValidateContactCacheForAllBodies()
{
	lock_guard lock(mBodiesCacheInvalidMutex);

	for (const BodyID &b : mBodiesCacheInvalid)
	{
		// The body may have been removed between the call to InvalidateContactCacheForBody and this call, so check if it still exists
		Body *body = TryGetBody(b);
		if (body != nullptr)
			body->ValidateContactCacheInternal();
	}
	mBodiesCacheInvalid.clear();
}

#ifdef MOSS_DEBUG
void BodyManager::ValidateActiveBodyBounds()
{
	UniqueLock lock(mActiveBodiesMutex MOSS_IF_ENABLE_ASSERTS(, this, EPhysicsLockTypes::ActiveBodiesList));

	for (uint type = 0; type < cBodyTypeCount; ++type)
		for (BodyID *id = mActiveBodies[type], *id_end = mActiveBodies[type] + mNumActiveBodies[type].load(memory_order_relaxed); id < id_end; ++id)
		{
			const Body *body = mBodies[id->GetIndex()];
			body->ValidateCachedBounds();
		}
}
#endif // MOSS_DEBUG


void BodyCreationSettings::SaveBinaryState(StreamOut &inStream) const
{
	inStream.Write(mPosition);
	inStream.Write(mRotation);
	inStream.Write(mLinearVelocity);
	inStream.Write(mAngularVelocity);
	mCollisionGroup.SaveBinaryState(inStream);
	inStream.Write(mObjectLayer);
	inStream.Write(mMotionType);
	inStream.Write(mAllowedDOFs);
	inStream.Write(mAllowDynamicOrKinematic);
	inStream.Write(mIsSensor);
	inStream.Write(mCollideKinematicVsNonDynamic);
	inStream.Write(mUseManifoldReduction);
	inStream.Write(mApplyGyroscopicForce);
	inStream.Write(mMotionQuality);
	inStream.Write(mEnhancedInternalEdgeRemoval);
	inStream.Write(mAllowSleeping);
	inStream.Write(mFriction);
	inStream.Write(mRestitution);
	inStream.Write(mLinearDamping);
	inStream.Write(mAngularDamping);
	inStream.Write(mMaxLinearVelocity);
	inStream.Write(mMaxAngularVelocity);
	inStream.Write(mGravityFactor);
	inStream.Write(mNumVelocityStepsOverride);
	inStream.Write(mNumPositionStepsOverride);
	inStream.Write(mOverrideMassProperties);
	inStream.Write(mInertiaMultiplier);
	mMassPropertiesOverride.SaveBinaryState(inStream);
}

void BodyCreationSettings::RestoreBinaryState(StreamIn &inStream)
{
	inStream.Read(mPosition);
	inStream.Read(mRotation);
	inStream.Read(mLinearVelocity);
	inStream.Read(mAngularVelocity);
	mCollisionGroup.RestoreBinaryState(inStream);
	inStream.Read(mObjectLayer);
	inStream.Read(mMotionType);
	inStream.Read(mAllowedDOFs);
	inStream.Read(mAllowDynamicOrKinematic);
	inStream.Read(mIsSensor);
	inStream.Read(mCollideKinematicVsNonDynamic);
	inStream.Read(mUseManifoldReduction);
	inStream.Read(mApplyGyroscopicForce);
	inStream.Read(mMotionQuality);
	inStream.Read(mEnhancedInternalEdgeRemoval);
	inStream.Read(mAllowSleeping);
	inStream.Read(mFriction);
	inStream.Read(mRestitution);
	inStream.Read(mLinearDamping);
	inStream.Read(mAngularDamping);
	inStream.Read(mMaxLinearVelocity);
	inStream.Read(mMaxAngularVelocity);
	inStream.Read(mGravityFactor);
	inStream.Read(mNumVelocityStepsOverride);
	inStream.Read(mNumPositionStepsOverride);
	inStream.Read(mOverrideMassProperties);
	inStream.Read(mInertiaMultiplier);
	mMassPropertiesOverride.RestoreBinaryState(inStream);
}

Shape::ShapeResult BodyCreationSettings::ConvertShapeSettings()
{
	// If we already have a shape, return it
	if (mShapePtr != nullptr)
	{
		mShape = nullptr;

		Shape::ShapeResult result;
		result.Set(const_cast<Shape *>(mShapePtr.GetPtr()));
		return result;
	}

	// Check if we have shape settings
	if (mShape == nullptr)
	{
		Shape::ShapeResult result;
		result.SetError("No shape present!");
		return result;
	}

	// Create the shape
	Shape::ShapeResult result = mShape->Create();
	if (result.IsValid())
		mShapePtr = result.Get();
	mShape = nullptr;
	return result;
}

const Shape *BodyCreationSettings::GetShape() const
{
	// If we already have a shape, return it
	if (mShapePtr != nullptr)
		return mShapePtr;

	// Check if we have shape settings
	if (mShape == nullptr)
		return nullptr;

	// Create the shape
	Shape::ShapeResult result = mShape->Create();
	if (result.IsValid())
		return result.Get();

	MOSS_TRACE("Error: %s", result.GetError().c_str());
	MOSS_ASSERT(false, "An error occurred during shape creation. Use ConvertShapeSettings() to convert the shape and get the error!");
	return nullptr;
}

MassProperties BodyCreationSettings::GetMassProperties() const
{
	// Calculate mass properties
	MassProperties mass_properties;
	switch (mOverrideMassProperties)
	{
	case EOverrideMassProperties::CalculateMassAndInertia:
		mass_properties = GetShape()->GetMassProperties();
		mass_properties.mInertia *= mInertiaMultiplier;
		mass_properties.mInertia(3, 3) = 1.0f;
		break;
	case EOverrideMassProperties::CalculateInertia:
		mass_properties = GetShape()->GetMassProperties();
		mass_properties.ScaleToMass(mMassPropertiesOverride.mMass);
		mass_properties.mInertia *= mInertiaMultiplier;
		mass_properties.mInertia(3, 3) = 1.0f;
		break;
	case EOverrideMassProperties::MassAndInertiaProvided:
		mass_properties = mMassPropertiesOverride;
		break;
	}
	return mass_properties;
}

void BodyCreationSettings::SaveWithChildren(StreamOut &inStream, ShapeToIDMap *ioShapeMap, MaterialToIDMap *ioMaterialMap, GroupFilterToIDMap *ioGroupFilterMap) const
{
	// Save creation settings
	SaveBinaryState(inStream);

	// Save shape
	if (ioShapeMap != nullptr && ioMaterialMap != nullptr)
		GetShape()->SaveWithChildren(inStream, *ioShapeMap, *ioMaterialMap);
	else
		inStream.Write(~uint32(0));

	// Save group filter
	StreamUtils::SaveObjectReference(inStream, mCollisionGroup.GetGroupFilter(), ioGroupFilterMap);
}

BodyCreationSettings::BCSResult BodyCreationSettings::sRestoreWithChildren(StreamIn &inStream, IDToShapeMap &ioShapeMap, IDToMaterialMap &ioMaterialMap, IDToGroupFilterMap &ioGroupFilterMap)
{
	BCSResult result;

	// Read creation settings
	BodyCreationSettings settings;
	settings.RestoreBinaryState(inStream);
	if (inStream.IsEOF() || inStream.IsFailed())
	{
		result.SetError("Error reading body creation settings");
		return result;
	}

	// Read shape
	Shape::ShapeResult shape_result = Shape::sRestoreWithChildren(inStream, ioShapeMap, ioMaterialMap);
	if (shape_result.HasError())
	{
		result.SetError(shape_result.GetError());
		return result;
	}
	settings.SetShape(shape_result.Get());

	// Read group filter
	Result gfresult = StreamUtils::RestoreObjectReference(inStream, ioGroupFilterMap);
	if (gfresult.HasError())
	{
		result.SetError(gfresult.GetError());
		return result;
	}
	settings.mCollisionGroup.SetGroupFilter(gfresult.Get());

	result.Set(settings);
	return result;
}


void BodyInterface::ActivateBodyInternal(Body &ioBody) const
{
	// Activate body or reset its sleep timer.
	// Note that BodyManager::ActivateBodies also resets the sleep timer internally, but we avoid a mutex lock if the body is already active by calling ResetSleepTimer directly.
	if (!ioBody.IsActive())
		mBodyManager->ActivateBodies(&ioBody.GetID(), 1);
	else
		ioBody.ResetSleepTimer();
}

Body *BodyInterface::CreateBody(const BodyCreationSettings &inSettings)
{
	Body *body = mBodyManager->AllocateBody(inSettings);
	if (!mBodyManager->AddBody(body))
	{
		mBodyManager->FreeBody(body);
		return nullptr;
	}
	return body;
}

Body *BodyInterface::CreateSoftBody(const SoftBodyCreationSettings &inSettings)
{
	Body *body = mBodyManager->AllocateSoftBody(inSettings);
	if (!mBodyManager->AddBody(body))
	{
		mBodyManager->FreeBody(body);
		return nullptr;
	}
	return body;
}

Body *BodyInterface::CreateBodyWithID(const BodyID &inBodyID, const BodyCreationSettings &inSettings)
{
	Body *body = mBodyManager->AllocateBody(inSettings);
	if (!mBodyManager->AddBodyWithCustomID(body, inBodyID))
	{
		mBodyManager->FreeBody(body);
		return nullptr;
	}
	return body;
}

Body *BodyInterface::CreateSoftBodyWithID(const BodyID &inBodyID, const SoftBodyCreationSettings &inSettings)
{
	Body *body = mBodyManager->AllocateSoftBody(inSettings);
	if (!mBodyManager->AddBodyWithCustomID(body, inBodyID))
	{
		mBodyManager->FreeBody(body);
		return nullptr;
	}
	return body;
}

Body *BodyInterface::CreateBodyWithoutID(const BodyCreationSettings &inSettings) const
{
	return mBodyManager->AllocateBody(inSettings);
}

Body *BodyInterface::CreateSoftBodyWithoutID(const SoftBodyCreationSettings &inSettings) const
{
	return mBodyManager->AllocateSoftBody(inSettings);
}

void BodyInterface::DestroyBodyWithoutID(Body *inBody) const
{
	mBodyManager->FreeBody(inBody);
}

bool BodyInterface::AssignBodyID(Body *ioBody)
{
	return mBodyManager->AddBody(ioBody);
}

bool BodyInterface::AssignBodyID(Body *ioBody, const BodyID &inBodyID)
{
	return mBodyManager->AddBodyWithCustomID(ioBody, inBodyID);
}

Body *BodyInterface::UnassignBodyID(const BodyID &inBodyID)
{
	Body *body = nullptr;
	mBodyManager->RemoveBodies(&inBodyID, 1, &body);
	return body;
}

void BodyInterface::UnassignBodyIDs(const BodyID *inBodyIDs, int inNumber, Body **outBodies)
{
	mBodyManager->RemoveBodies(inBodyIDs, inNumber, outBodies);
}

void BodyInterface::DestroyBody(const BodyID &inBodyID)
{
	mBodyManager->DestroyBodies(&inBodyID, 1);
}

void BodyInterface::DestroyBodies(const BodyID *inBodyIDs, int inNumber)
{
	mBodyManager->DestroyBodies(inBodyIDs, inNumber);
}

void BodyInterface::AddBody(const BodyID &inBodyID, EActivation inActivationMode)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		const Body &body = lock.GetBody();

		// Add to broadphase
		BodyID id = inBodyID;
		BroadPhase::AddState add_state = mBroadPhase->AddBodiesPrepare(&id, 1);
		mBroadPhase->AddBodiesFinalize(&id, 1, add_state);

		// Optionally activate body
		if (inActivationMode == EActivation::Activate && !body.IsStatic())
			mBodyManager->ActivateBodies(&inBodyID, 1);
	}
}

void BodyInterface::RemoveBody(const BodyID &inBodyID)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		const Body &body = lock.GetBody();

		// Deactivate body
		if (body.IsActive())
			mBodyManager->DeactivateBodies(&inBodyID, 1);

		// Remove from broadphase
		BodyID id = inBodyID;
		mBroadPhase->RemoveBodies(&id, 1);
	}
}

bool BodyInterface::IsAdded(const BodyID &inBodyID) const
{
	BodyLockRead lock(*mBodyLockInterface, inBodyID);
	return lock.SucceededAndIsInBroadPhase();
}

BodyID BodyInterface::CreateAndAddBody(const BodyCreationSettings &inSettings, EActivation inActivationMode)
{
	const Body *b = CreateBody(inSettings);
	if (b == nullptr)
		return BodyID(); // Out of bodies
	AddBody(b->GetID(), inActivationMode);
	return b->GetID();
}

BodyID BodyInterface::CreateAndAddSoftBody(const SoftBodyCreationSettings &inSettings, EActivation inActivationMode)
{
	const Body *b = CreateSoftBody(inSettings);
	if (b == nullptr)
		return BodyID(); // Out of bodies
	AddBody(b->GetID(), inActivationMode);
	return b->GetID();
}

BodyInterface::AddState BodyInterface::AddBodiesPrepare(BodyID *ioBodies, int inNumber)
{
	return mBroadPhase->AddBodiesPrepare(ioBodies, inNumber);
}

void BodyInterface::AddBodiesFinalize(BodyID *ioBodies, int inNumber, AddState inAddState, EActivation inActivationMode)
{
	BodyLockMultiWrite lock(*mBodyLockInterface, ioBodies, inNumber);

	// Add to broadphase
	mBroadPhase->AddBodiesFinalize(ioBodies, inNumber, inAddState);

	// Optionally activate bodies
	if (inActivationMode == EActivation::Activate)
		mBodyManager->ActivateBodies(ioBodies, inNumber);
}

void BodyInterface::AddBodiesAbort(BodyID *ioBodies, int inNumber, AddState inAddState)
{
	mBroadPhase->AddBodiesAbort(ioBodies, inNumber, inAddState);
}

void BodyInterface::RemoveBodies(BodyID *ioBodies, int inNumber)
{
	BodyLockMultiWrite lock(*mBodyLockInterface, ioBodies, inNumber);

	// Deactivate bodies
	mBodyManager->DeactivateBodies(ioBodies, inNumber);

	// Remove from broadphase
	mBroadPhase->RemoveBodies(ioBodies, inNumber);
}

void BodyInterface::ActivateBody(const BodyID &inBodyID)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		Body &body = lock.GetBody();
		ActivateBodyInternal(body);
	}
}

void BodyInterface::ActivateBodies(const BodyID *inBodyIDs, int inNumber)
{
	BodyLockMultiWrite lock(*mBodyLockInterface, inBodyIDs, inNumber);

	mBodyManager->ActivateBodies(inBodyIDs, inNumber);
}

void BodyInterface::ActivateBodiesInAABox(const AABox &inBox, const BroadPhaseLayerFilter &inBroadPhaseLayerFilter, const ObjectLayerFilter &inObjectLayerFilter)
{
	AllHitCollisionCollector<CollideShapeBodyCollector> collector;
	mBroadPhase->CollideAABox(inBox, collector, inBroadPhaseLayerFilter, inObjectLayerFilter);
	ActivateBodies(collector.mHits.data(), (int)collector.mHits.size());
}

void BodyInterface::DeactivateBody(const BodyID &inBodyID)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		const Body &body = lock.GetBody();

		if (body.IsActive())
			mBodyManager->DeactivateBodies(&inBodyID, 1);
	}
}

void BodyInterface::DeactivateBodies(const BodyID *inBodyIDs, int inNumber)
{
	BodyLockMultiWrite lock(*mBodyLockInterface, inBodyIDs, inNumber);

	mBodyManager->DeactivateBodies(inBodyIDs, inNumber);
}

bool BodyInterface::IsActive(const BodyID &inBodyID) const
{
	BodyLockRead lock(*mBodyLockInterface, inBodyID);
	return lock.Succeeded() && lock.GetBody().IsActive();
}

void BodyInterface::ResetSleepTimer(const BodyID &inBodyID)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
		lock.GetBody().ResetSleepTimer();
}

TwoBodyConstraint *BodyInterface::CreateConstraint(const TwoBodyConstraintSettings *inSettings, const BodyID &inBodyID1, const BodyID &inBodyID2)
{
	BodyID constraint_bodies[] = { inBodyID1, inBodyID2 };
	BodyLockMultiWrite lock(*mBodyLockInterface, constraint_bodies, 2);

	Body *body1 = lock.GetBody(0);
	Body *body2 = lock.GetBody(1);

	MOSS_ASSERT(body1 != body2);
	MOSS_ASSERT(body1 != nullptr || body2 != nullptr);

	return inSettings->Create(body1 != nullptr? *body1 : Body::sFixedToWorld, body2 != nullptr? *body2 : Body::sFixedToWorld);
}

void BodyInterface::ActivateConstraint(const TwoBodyConstraint *inConstraint)
{
	BodyID bodies[] = { inConstraint->GetBody1()->GetID(), inConstraint->GetBody2()->GetID() };
	ActivateBodies(bodies, 2);
}

RefConst<Shape> BodyInterface::GetShape(const BodyID &inBodyID) const
{
	RefConst<Shape> shape;
	BodyLockRead lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
		shape = lock.GetBody().GetShape();
	return shape;
}

void BodyInterface::SetShape(const BodyID &inBodyID, const Shape *inShape, bool inUpdateMassProperties, EActivation inActivationMode) const
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		Body &body = lock.GetBody();

		// Check if shape actually changed
		if (body.GetShape() != inShape)
		{
			// Update the shape
			body.SetShapeInternal(inShape, inUpdateMassProperties);

			// Flag collision cache invalid for this body
			mBodyManager->InvalidateContactCacheForBody(body);

			// Notify broadphase of change
			if (body.IsInBroadPhase())
			{
				BodyID id = body.GetID();
				mBroadPhase->NotifyBodiesAABBChanged(&id, 1);

				// Optionally activate body
				if (inActivationMode == EActivation::Activate && !body.IsStatic())
					ActivateBodyInternal(body);
			}
		}
	}
}

void BodyInterface::NotifyShapeChanged(const BodyID &inBodyID, Vec3Arg inPreviousCenterOfMass, bool inUpdateMassProperties, EActivation inActivationMode) const
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		Body &body = lock.GetBody();

		// Update center of mass, mass and inertia
		body.UpdateCenterOfMassInternal(inPreviousCenterOfMass, inUpdateMassProperties);

		// Recalculate bounding box
		body.CalculateWorldSpaceBoundsInternal();

		// Flag collision cache invalid for this body
		mBodyManager->InvalidateContactCacheForBody(body);

		// Notify broadphase of change
		if (body.IsInBroadPhase())
		{
			BodyID id = body.GetID();
			mBroadPhase->NotifyBodiesAABBChanged(&id, 1);

			// Optionally activate body
			if (inActivationMode == EActivation::Activate && !body.IsStatic())
				ActivateBodyInternal(body);
		}
	}
}

void BodyInterface::SetObjectLayer(const BodyID &inBodyID, ObjectLayer inLayer)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		Body &body = lock.GetBody();

		// Check if layer actually changed, updating the broadphase is rather expensive
		if (body.GetObjectLayer() != inLayer)
		{
			// Update the layer on the body
			mBodyManager->SetBodyObjectLayerInternal(body, inLayer);

			// Notify broadphase of change
			if (body.IsInBroadPhase())
			{
				BodyID id = body.GetID();
				mBroadPhase->NotifyBodiesLayerChanged(&id, 1);
			}
		}
	}
}

ObjectLayer BodyInterface::GetObjectLayer(const BodyID &inBodyID) const
{
	BodyLockRead lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
		return lock.GetBody().GetObjectLayer();
	else
		return cObjectLayerInvalid;
}

void BodyInterface::SetPositionAndRotation(const BodyID &inBodyID, RVec3Arg inPosition, QuatArg inRotation, EActivation inActivationMode)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		Body &body = lock.GetBody();

		// Update the position
		body.SetPositionAndRotationInternal(inPosition, inRotation);

		// Notify broadphase of change
		if (body.IsInBroadPhase())
		{
			BodyID id = body.GetID();
			mBroadPhase->NotifyBodiesAABBChanged(&id, 1);

			// Optionally activate body
			if (inActivationMode == EActivation::Activate && !body.IsStatic())
				ActivateBodyInternal(body);
		}
	}
}

void BodyInterface::SetPositionAndRotationWhenChanged(const BodyID &inBodyID, RVec3Arg inPosition, QuatArg inRotation, EActivation inActivationMode)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		Body &body = lock.GetBody();

		// Check if there is enough change
		if (!body.GetPosition().IsClose(inPosition)
			|| !body.GetRotation().IsClose(inRotation))
		{
			// Update the position
			body.SetPositionAndRotationInternal(inPosition, inRotation);

			// Notify broadphase of change
			if (body.IsInBroadPhase())
			{
				BodyID id = body.GetID();
				mBroadPhase->NotifyBodiesAABBChanged(&id, 1);

				// Optionally activate body
				if (inActivationMode == EActivation::Activate && !body.IsStatic())
					ActivateBodyInternal(body);
			}
		}
	}
}

void BodyInterface::GetPositionAndRotation(const BodyID &inBodyID, RVec3 &outPosition, Quat &outRotation) const
{
	BodyLockRead lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		const Body &body = lock.GetBody();
		outPosition = body.GetPosition();
		outRotation = body.GetRotation();
	}
	else
	{
		outPosition = RVec3::sZero();
		outRotation = Quat::sIdentity();
	}
}

void BodyInterface::SetPosition(const BodyID &inBodyID, RVec3Arg inPosition, EActivation inActivationMode)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		Body &body = lock.GetBody();

		// Update the position
		body.SetPositionAndRotationInternal(inPosition, body.GetRotation());

		// Notify broadphase of change
		if (body.IsInBroadPhase())
		{
			BodyID id = body.GetID();
			mBroadPhase->NotifyBodiesAABBChanged(&id, 1);

			// Optionally activate body
			if (inActivationMode == EActivation::Activate && !body.IsStatic())
				ActivateBodyInternal(body);
		}
	}
}

RVec3 BodyInterface::GetPosition(const BodyID &inBodyID) const
{
	BodyLockRead lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
		return lock.GetBody().GetPosition();
	else
		return RVec3::sZero();
}

RVec3 BodyInterface::GetCenterOfMassPosition(const BodyID &inBodyID) const
{
	BodyLockRead lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
		return lock.GetBody().GetCenterOfMassPosition();
	else
		return RVec3::sZero();
}

void BodyInterface::SetRotation(const BodyID &inBodyID, QuatArg inRotation, EActivation inActivationMode)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		Body &body = lock.GetBody();

		// Update the position
		body.SetPositionAndRotationInternal(body.GetPosition(), inRotation);

		// Notify broadphase of change
		if (body.IsInBroadPhase())
		{
			BodyID id = body.GetID();
			mBroadPhase->NotifyBodiesAABBChanged(&id, 1);

			// Optionally activate body
			if (inActivationMode == EActivation::Activate && !body.IsStatic())
				ActivateBodyInternal(body);
		}
	}
}

Quat BodyInterface::GetRotation(const BodyID &inBodyID) const
{
	BodyLockRead lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
		return lock.GetBody().GetRotation();
	else
		return Quat::sIdentity();
}

RMat44 BodyInterface::GetWorldTransform(const BodyID &inBodyID) const
{
	BodyLockRead lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
		return lock.GetBody().GetWorldTransform();
	else
		return RMat44::sIdentity();
}

RMat44 BodyInterface::GetCenterOfMassTransform(const BodyID &inBodyID) const
{
	BodyLockRead lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
		return lock.GetBody().GetCenterOfMassTransform();
	else
		return RMat44::sIdentity();
}

void BodyInterface::MoveKinematic(const BodyID &inBodyID, RVec3Arg inTargetPosition, QuatArg inTargetRotation, float inDeltaTime)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		Body &body = lock.GetBody();

		body.MoveKinematic(inTargetPosition, inTargetRotation, inDeltaTime);

		if (!body.IsActive() && (!body.GetLinearVelocity().IsNearZero() || !body.GetAngularVelocity().IsNearZero()))
			mBodyManager->ActivateBodies(&inBodyID, 1);
	}
}

void BodyInterface::SetLinearAndAngularVelocity(const BodyID &inBodyID, Vec3Arg inLinearVelocity, Vec3Arg inAngularVelocity)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		Body &body = lock.GetBody();
		if (!body.IsStatic())
		{
			body.SetLinearVelocityClamped(inLinearVelocity);
			body.SetAngularVelocityClamped(inAngularVelocity);

			if (!body.IsActive() && (!inLinearVelocity.IsNearZero() || !inAngularVelocity.IsNearZero()))
				mBodyManager->ActivateBodies(&inBodyID, 1);
		}
	}
}

void BodyInterface::GetLinearAndAngularVelocity(const BodyID &inBodyID, Vec3 &outLinearVelocity, Vec3 &outAngularVelocity) const
{
	BodyLockRead lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		const Body &body = lock.GetBody();
		if (!body.IsStatic())
		{
			outLinearVelocity = body.GetLinearVelocity();
			outAngularVelocity = body.GetAngularVelocity();
			return;
		}
	}

	outLinearVelocity = outAngularVelocity = Vec3::sZero();
}

void BodyInterface::SetLinearVelocity(const BodyID &inBodyID, Vec3Arg inLinearVelocity)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		Body &body = lock.GetBody();
		if (!body.IsStatic())
		{
			body.SetLinearVelocityClamped(inLinearVelocity);

			if (!body.IsActive() && !inLinearVelocity.IsNearZero())
				mBodyManager->ActivateBodies(&inBodyID, 1);
		}
	}
}

Vec3 BodyInterface::GetLinearVelocity(const BodyID &inBodyID) const
{
	BodyLockRead lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		const Body &body = lock.GetBody();
		if (!body.IsStatic())
			return body.GetLinearVelocity();
	}

	return Vec3::sZero();
}

void BodyInterface::AddLinearVelocity(const BodyID &inBodyID, Vec3Arg inLinearVelocity)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		Body &body = lock.GetBody();
		if (!body.IsStatic())
		{
			body.SetLinearVelocityClamped(body.GetLinearVelocity() + inLinearVelocity);

			if (!body.IsActive() && !body.GetLinearVelocity().IsNearZero())
				mBodyManager->ActivateBodies(&inBodyID, 1);
		}
	}
}

void BodyInterface::AddLinearAndAngularVelocity(const BodyID &inBodyID, Vec3Arg inLinearVelocity, Vec3Arg inAngularVelocity)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		Body &body = lock.GetBody();
		if (!body.IsStatic())
		{
			body.SetLinearVelocityClamped(body.GetLinearVelocity() + inLinearVelocity);
			body.SetAngularVelocityClamped(body.GetAngularVelocity() + inAngularVelocity);

			if (!body.IsActive() && (!body.GetLinearVelocity().IsNearZero() || !body.GetAngularVelocity().IsNearZero()))
				mBodyManager->ActivateBodies(&inBodyID, 1);
		}
	}
}

void BodyInterface::SetAngularVelocity(const BodyID &inBodyID, Vec3Arg inAngularVelocity)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		Body &body = lock.GetBody();
		if (!body.IsStatic())
		{
			body.SetAngularVelocityClamped(inAngularVelocity);

			if (!body.IsActive() && !inAngularVelocity.IsNearZero())
				mBodyManager->ActivateBodies(&inBodyID, 1);
		}
	}
}

Vec3 BodyInterface::GetAngularVelocity(const BodyID &inBodyID) const
{
	BodyLockRead lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		const Body &body = lock.GetBody();
		if (!body.IsStatic())
			return body.GetAngularVelocity();
	}

	return Vec3::sZero();
}

Vec3 BodyInterface::GetPointVelocity(const BodyID &inBodyID, RVec3Arg inPoint) const
{
	BodyLockRead lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		const Body &body = lock.GetBody();
		if (!body.IsStatic())
			return body.GetPointVelocity(inPoint);
	}

	return Vec3::sZero();
}

void BodyInterface::AddForce(const BodyID &inBodyID, Vec3Arg inForce, EActivation inActivationMode)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		Body &body = lock.GetBody();
		if (body.IsDynamic() && (inActivationMode == EActivation::Activate || body.IsActive()))
		{
			body.AddForce(inForce);

			if (inActivationMode == EActivation::Activate)
				ActivateBodyInternal(body);
		}
	}
}

void BodyInterface::AddForce(const BodyID &inBodyID, Vec3Arg inForce, RVec3Arg inPoint, EActivation inActivationMode)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		Body &body = lock.GetBody();
		if (body.IsDynamic() && (inActivationMode == EActivation::Activate || body.IsActive()))
		{
			body.AddForce(inForce, inPoint);

			if (inActivationMode == EActivation::Activate)
				ActivateBodyInternal(body);
		}
	}
}

void BodyInterface::AddTorque(const BodyID &inBodyID, Vec3Arg inTorque, EActivation inActivationMode)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		Body &body = lock.GetBody();
		if (body.IsDynamic() && (inActivationMode == EActivation::Activate || body.IsActive()))
		{
			body.AddTorque(inTorque);

			if (inActivationMode == EActivation::Activate)
				ActivateBodyInternal(body);
		}
	}
}

void BodyInterface::AddForceAndTorque(const BodyID &inBodyID, Vec3Arg inForce, Vec3Arg inTorque, EActivation inActivationMode)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		Body &body = lock.GetBody();
		if (body.IsDynamic() && (inActivationMode == EActivation::Activate || body.IsActive()))
		{
			body.AddForce(inForce);
			body.AddTorque(inTorque);

			if (inActivationMode == EActivation::Activate)
				ActivateBodyInternal(body);
		}
	}
}

void BodyInterface::AddImpulse(const BodyID &inBodyID, Vec3Arg inImpulse)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		Body &body = lock.GetBody();
		if (body.IsDynamic())
		{
			body.AddImpulse(inImpulse);

			if (!body.IsActive())
				mBodyManager->ActivateBodies(&inBodyID, 1);
		}
	}
}

void BodyInterface::AddImpulse(const BodyID &inBodyID, Vec3Arg inImpulse, RVec3Arg inPoint)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		Body &body = lock.GetBody();
		if (body.IsDynamic())
		{
			body.AddImpulse(inImpulse, inPoint);

			if (!body.IsActive())
				mBodyManager->ActivateBodies(&inBodyID, 1);
		}
	}
}

void BodyInterface::AddAngularImpulse(const BodyID &inBodyID, Vec3Arg inAngularImpulse)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		Body &body = lock.GetBody();
		if (body.IsDynamic())
		{
			body.AddAngularImpulse(inAngularImpulse);

			if (!body.IsActive())
				mBodyManager->ActivateBodies(&inBodyID, 1);
		}
	}
}

bool BodyInterface::ApplyBuoyancyImpulse(const BodyID &inBodyID, RVec3Arg inSurfacePosition, Vec3Arg inSurfaceNormal, float inBuoyancy, float inLinearDrag, float inAngularDrag, Vec3Arg inFluidVelocity, Vec3Arg inGravity, float inDeltaTime)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		Body &body = lock.GetBody();
		if (body.IsDynamic()
			&& body.ApplyBuoyancyImpulse(inSurfacePosition, inSurfaceNormal, inBuoyancy, inLinearDrag, inAngularDrag, inFluidVelocity, inGravity, inDeltaTime))
		{
			ActivateBodyInternal(body);
			return true;
		}
	}

	return false;
}

void BodyInterface::SetPositionRotationAndVelocity(const BodyID &inBodyID, RVec3Arg inPosition, QuatArg inRotation, Vec3Arg inLinearVelocity, Vec3Arg inAngularVelocity)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		Body &body = lock.GetBody();

		// Update the position
		body.SetPositionAndRotationInternal(inPosition, inRotation);

		// Notify broadphase of change
		if (body.IsInBroadPhase())
		{
			BodyID id = body.GetID();
			mBroadPhase->NotifyBodiesAABBChanged(&id, 1);
		}

		if (!body.IsStatic())
		{
			body.SetLinearVelocityClamped(inLinearVelocity);
			body.SetAngularVelocityClamped(inAngularVelocity);

			// Optionally activate body
			if (!body.IsActive() && (!inLinearVelocity.IsNearZero() || !inAngularVelocity.IsNearZero()))
				mBodyManager->ActivateBodies(&inBodyID, 1);
		}
	}
}

void BodyInterface::SetMotionType(const BodyID &inBodyID, EMotionType inMotionType, EActivation inActivationMode)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		Body &body = lock.GetBody();

		// Deactivate if we're making the body static
		if (body.IsActive() && inMotionType == EMotionType::Static)
			mBodyManager->DeactivateBodies(&inBodyID, 1);

		body.SetMotionType(inMotionType);

		// Activate body if requested
		if (inMotionType != EMotionType::Static && inActivationMode == EActivation::Activate)
			ActivateBodyInternal(body);
	}
}

EBodyType BodyInterface::GetBodyType(const BodyID &inBodyID) const
{
	BodyLockRead lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
		return lock.GetBody().GetBodyType();
	else
		return EBodyType::RigidBody;
}

EMotionType BodyInterface::GetMotionType(const BodyID &inBodyID) const
{
	BodyLockRead lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
		return lock.GetBody().GetMotionType();
	else
		return EMotionType::Static;
}

void BodyInterface::SetMotionQuality(const BodyID &inBodyID, EMotionQuality inMotionQuality)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
		mBodyManager->SetMotionQuality(lock.GetBody(), inMotionQuality);
}

EMotionQuality BodyInterface::GetMotionQuality(const BodyID &inBodyID) const
{
	BodyLockRead lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded() && !lock.GetBody().IsStatic())
		return lock.GetBody().GetMotionProperties()->GetMotionQuality();
	else
		return EMotionQuality::Discrete;
}

Mat44 BodyInterface::GetInverseInertia(const BodyID &inBodyID) const
{
	BodyLockRead lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
		return lock.GetBody().GetInverseInertia();
	else
		return Mat44::sIdentity();
}

void BodyInterface::SetRestitution(const BodyID &inBodyID, float inRestitution)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
		lock.GetBody().SetRestitution(inRestitution);
}

float BodyInterface::GetRestitution(const BodyID &inBodyID) const
{
	BodyLockRead lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
		return lock.GetBody().GetRestitution();
	else
		return 0.0f;
}

void BodyInterface::SetFriction(const BodyID &inBodyID, float inFriction)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
		lock.GetBody().SetFriction(inFriction);
}

float BodyInterface::GetFriction(const BodyID &inBodyID) const
{
	BodyLockRead lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
		return lock.GetBody().GetFriction();
	else
		return 0.0f;
}

void BodyInterface::SetGravityFactor(const BodyID &inBodyID, float inGravityFactor)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded() && lock.GetBody().GetMotionPropertiesUnchecked() != nullptr)
		lock.GetBody().GetMotionPropertiesUnchecked()->SetGravityFactor(inGravityFactor);
}

float BodyInterface::GetGravityFactor(const BodyID &inBodyID) const
{
	BodyLockRead lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded() && lock.GetBody().GetMotionPropertiesUnchecked() != nullptr)
		return lock.GetBody().GetMotionPropertiesUnchecked()->GetGravityFactor();
	else
		return 1.0f;
}

void BodyInterface::SetUseManifoldReduction(const BodyID &inBodyID, bool inUseReduction)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
	{
		Body &body = lock.GetBody();
		if (body.GetUseManifoldReduction() != inUseReduction)
		{
			body.SetUseManifoldReduction(inUseReduction);

			// Flag collision cache invalid for this body
			mBodyManager->InvalidateContactCacheForBody(body);
		}
	}
}

bool BodyInterface::GetUseManifoldReduction(const BodyID &inBodyID) const
{
	BodyLockRead lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
		return lock.GetBody().GetUseManifoldReduction();
	else
		return true;
}

void BodyInterface::SetCollisionGroup(const BodyID &inBodyID, const CollisionGroup &inCollisionGroup)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
		lock.GetBody().SetCollisionGroup(inCollisionGroup);
}

const CollisionGroup &BodyInterface::GetCollisionGroup(const BodyID &inBodyID) const
{
	BodyLockRead lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
		return lock.GetBody().GetCollisionGroup();
	else
		return CollisionGroup::sInvalid;
}

TransformedShape BodyInterface::GetTransformedShape(const BodyID &inBodyID) const
{
	BodyLockRead lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
		return lock.GetBody().GetTransformedShape();
	else
		return TransformedShape();
}

uint64 BodyInterface::GetUserData(const BodyID &inBodyID) const
{
	BodyLockRead lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
		return lock.GetBody().GetUserData();
	else
		return 0;
}

void BodyInterface::SetUserData(const BodyID &inBodyID, uint64 inUserData) const
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
		lock.GetBody().SetUserData(inUserData);
}

const PhysicsMaterial *BodyInterface::GetMaterial(const BodyID &inBodyID, const SubShapeID &inSubShapeID) const
{
	BodyLockRead lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
		return lock.GetBody().GetShape()->GetMaterial(inSubShapeID);
	else
		return PhysicsMaterial::sDefault;
}

void BodyInterface::InvalidateContactCache(const BodyID &inBodyID)
{
	BodyLockWrite lock(*mBodyLockInterface, inBodyID);
	if (lock.Succeeded())
		mBodyManager->InvalidateContactCacheForBody(lock.GetBody());
}

MOSS_SUPRESS_WARNINGS_END

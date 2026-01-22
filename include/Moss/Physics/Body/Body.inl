// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

MOSS_NAMESPACE_BEGIN

RMat44 Body::GetWorldTransform() const
{
	MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sPositionAccess(), BodyAccess::EAccess::Read));

	return RMat44::sRotationTranslation(mRotation, mPosition).PreTranslated(-mShape->GetCenterOfMass());
}

RMat44 Body::GetCenterOfMassTransform() const
{
	MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sPositionAccess(), BodyAccess::EAccess::Read));

	return RMat44::sRotationTranslation(mRotation, mPosition);
}

RMat44 Body::GetInverseCenterOfMassTransform() const
{
	MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sPositionAccess(), BodyAccess::EAccess::Read));

	return RMat44::sInverseRotationTranslation(mRotation, mPosition);
}

inline bool Body::sFindCollidingPairsCanCollide(const Body &inBody1, const Body &inBody2)
{
	// First body should never be a soft body
	MOSS_ASSERT(!inBody1.IsSoftBody());

	// One of these conditions must be true
	// - We always allow detecting collisions between kinematic and non-dynamic bodies
	// - One of the bodies must be dynamic to collide
	// - A kinematic object can collide with a sensor
	if (!inBody1.GetCollideKinematicVsNonDynamic()
		&& !inBody2.GetCollideKinematicVsNonDynamic()
		&& (!inBody1.IsDynamic() && !inBody2.IsDynamic())
		&& !(inBody1.IsKinematic() && inBody2.IsSensor())
		&& !(inBody2.IsKinematic() && inBody1.IsSensor()))
		return false;

	// Check that body 1 is active
	uint32 body1_index_in_active_bodies = inBody1.GetIndexInActiveBodiesInternal();
	MOSS_ASSERT(!inBody1.IsStatic() && body1_index_in_active_bodies != Body::cInactiveIndex, "This function assumes that Body 1 is active");

	// If the pair A, B collides we need to ensure that the pair B, A does not collide or else we will handle the collision twice.
	// If A is the same body as B we don't want to collide (1)
	// If A is dynamic / kinematic and B is static we should collide (2)
	// If A is dynamic / kinematic and B is dynamic / kinematic we should only collide if
	//	- A is active and B is not active (3)
	//	- A is active and B will become active during this simulation step (4)
	//	- A is active and B is active, we require a condition that makes A, B collide and B, A not (5)
	//
	// In order to implement this we use the index in the active body list and make use of the fact that
	// a body not in the active list has Body.Index = 0xffffffff which is the highest possible value for an uint32.
	//
	// Because we know that A is active we know that A.Index != 0xffffffff:
	// (1) Because A.Index != 0xffffffff, if A.Index = B.Index then A = B, so to collide A.Index != B.Index
	// (2) A.Index != 0xffffffff, B.Index = 0xffffffff (because it's static and cannot be in the active list), so to collide A.Index != B.Index
	// (3) A.Index != 0xffffffff, B.Index = 0xffffffff (because it's not yet active), so to collide A.Index != B.Index
	// (4) A.Index != 0xffffffff, B.Index = 0xffffffff currently. But it can activate during the Broad/NarrowPhase step at which point it
	//     will be added to the end of the active list which will make B.Index > A.Index (this holds only true when we don't deactivate
	//     bodies during the Broad/NarrowPhase step), so to collide A.Index < B.Index.
	// (5) As tie breaker we can use the same condition A.Index < B.Index to collide, this means that if A, B collides then B, A won't
	static_assert(Body::cInactiveIndex == 0xffffffff, "The algorithm below uses this value");
	if (!inBody2.IsSoftBody() && body1_index_in_active_bodies >= inBody2.GetIndexInActiveBodiesInternal())
		return false;
	MOSS_ASSERT(inBody1.GetID() != inBody2.GetID(), "Read the comment above, A and B are the same body which should not be possible!");

	// Check collision group filter
	if (!inBody1.GetCollisionGroup().CanCollide(inBody2.GetCollisionGroup()))
		return false;

	return true;
}

void Body::AddRotationStep(Vec3Arg inAngularVelocityTimesDeltaTime)
{
	MOSS_ASSERT(IsRigidBody());
	MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sPositionAccess(), BodyAccess::EAccess::ReadWrite));

	// This used to use the equation: d/dt R(t) = 1/2 * w(t) * R(t) so that R(t + dt) = R(t) + 1/2 * w(t) * R(t) * dt
	// See: Appendix B of An Introduction to Physically Based Modeling: Rigid Body Simulation II-Nonpenetration Constraints
	// URL: https://www.cs.cmu.edu/~baraff/sigcourse/notesd2.pdf
	// But this is a first order approximation and does not work well for kinematic ragdolls that are driven to a new
	// pose if the poses differ enough. So now we split w(t) * dt into an axis and angle part and create a quaternion with it.
	// Note that the resulting quaternion is normalized since otherwise numerical drift will eventually make the rotation non-normalized.
	float len = inAngularVelocityTimesDeltaTime.Length();
	if (len > 1.0e-6f)
	{
		mRotation = (Quat::sRotation(inAngularVelocityTimesDeltaTime / len, len) * mRotation).Normalized();
		MOSS_ASSERT(!mRotation.IsNaN());
	}
}

void Body::SubRotationStep(Vec3Arg inAngularVelocityTimesDeltaTime)
{
	MOSS_ASSERT(IsRigidBody());
	MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sPositionAccess(), BodyAccess::EAccess::ReadWrite));

	// See comment at Body::AddRotationStep
	float len = inAngularVelocityTimesDeltaTime.Length();
	if (len > 1.0e-6f)
	{
		mRotation = (Quat::sRotation(inAngularVelocityTimesDeltaTime / len, -len) * mRotation).Normalized();
		MOSS_ASSERT(!mRotation.IsNaN());
	}
}

Vec3 Body::GetWorldSpaceSurfaceNormal(const SubShapeID &inSubShapeID, RVec3Arg inPosition) const
{
	RMat44 inv_com = GetInverseCenterOfMassTransform();
	return inv_com.Multiply3x3Transposed(mShape->GetSurfaceNormal(inSubShapeID, Vec3(inv_com * inPosition))).Normalized();
}

Mat44 Body::GetInverseInertia() const
{
	MOSS_ASSERT(IsDynamic());

	return GetMotionProperties()->GetInverseInertiaForRotation(Mat44::sRotation(mRotation));
}

void Body::AddForce(Vec3Arg inForce, RVec3Arg inPosition)
{
	AddForce(inForce);
	AddTorque(Vec3(inPosition - mPosition).Cross(inForce));
}

void Body::AddImpulse(Vec3Arg inImpulse)
{
	MOSS_ASSERT(IsDynamic());

	SetLinearVelocityClamped(mMotionProperties->GetLinearVelocity() + inImpulse * mMotionProperties->GetInverseMass());
}

void Body::AddImpulse(Vec3Arg inImpulse, RVec3Arg inPosition)
{
	MOSS_ASSERT(IsDynamic());

	SetLinearVelocityClamped(mMotionProperties->GetLinearVelocity() + inImpulse * mMotionProperties->GetInverseMass());

	SetAngularVelocityClamped(mMotionProperties->GetAngularVelocity() + mMotionProperties->MultiplyWorldSpaceInverseInertiaByVector(mRotation, Vec3(inPosition - mPosition).Cross(inImpulse)));
}

void Body::AddAngularImpulse(Vec3Arg inAngularImpulse)
{
	MOSS_ASSERT(IsDynamic());

	SetAngularVelocityClamped(mMotionProperties->GetAngularVelocity() + mMotionProperties->MultiplyWorldSpaceInverseInertiaByVector(mRotation, inAngularImpulse));
}

void Body::GetSleepTestPoints(RVec3 *outPoints) const
{
	MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sPositionAccess(), BodyAccess::EAccess::Read));

	// Center of mass is the first position
	outPoints[0] = mPosition;

	// The second and third position are on the largest axis of the bounding box
	Vec3 extent = mShape->GetLocalBounds().GetExtent();
	int lowest_component = extent.GetLowestComponentIndex();
	Mat44 rotation = Mat44::sRotation(mRotation);
	switch (lowest_component)
	{
	case 0:
		outPoints[1] = mPosition + extent.GetY() * rotation.GetColumn3(1);
		outPoints[2] = mPosition + extent.GetZ() * rotation.GetColumn3(2);
		break;

	case 1:
		outPoints[1] = mPosition + extent.GetX() * rotation.GetColumn3(0);
		outPoints[2] = mPosition + extent.GetZ() * rotation.GetColumn3(2);
		break;

	case 2:
		outPoints[1] = mPosition + extent.GetX() * rotation.GetColumn3(0);
		outPoints[2] = mPosition + extent.GetY() * rotation.GetColumn3(1);
		break;

	default:
		MOSS_ASSERT(false);
		break;
	}
}

void Body::ResetSleepTimer()
{
	RVec3 points[3];
	GetSleepTestPoints(points);
	mMotionProperties->ResetSleepTestSpheres(points);
}

void MotionProperties::MoveKinematic(Vec3Arg inDeltaPosition, QuatArg inDeltaRotation, float inDeltaTime)
{
	MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sVelocityAccess(), BodyAccess::EAccess::ReadWrite));
	MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sPositionAccess(), BodyAccess::EAccess::Read));
	MOSS_ASSERT(mCachedBodyType == EBodyType::RigidBody);
	MOSS_ASSERT(mCachedMotionType != EMotionType::Static);

	// Calculate required linear velocity
	mLinearVelocity = LockTranslation(inDeltaPosition / inDeltaTime);

	// Calculate required angular velocity
	Vec3 axis;
	float angle;
	inDeltaRotation.GetAxisAngle(axis, angle);
	mAngularVelocity = LockAngular(axis * (angle / inDeltaTime));
}

void MotionProperties::ClampLinearVelocity()
{
	MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sVelocityAccess(), BodyAccess::EAccess::ReadWrite));

	float len_sq = mLinearVelocity.LengthSq();
	MOSS_ASSERT(isfinite(len_sq));
	if (len_sq > Square(mMaxLinearVelocity))
		mLinearVelocity *= mMaxLinearVelocity / sqrt(len_sq);
}

void MotionProperties::ClampAngularVelocity()
{
	MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sVelocityAccess(), BodyAccess::EAccess::ReadWrite));

	float len_sq = mAngularVelocity.LengthSq();
	MOSS_ASSERT(isfinite(len_sq));
	if (len_sq > Square(mMaxAngularVelocity))
		mAngularVelocity *= mMaxAngularVelocity / sqrt(len_sq);
}

inline Mat44 MotionProperties::GetLocalSpaceInverseInertiaUnchecked() const
{
	Mat44 rotation = Mat44::sRotation(mInertiaRotation);
	Mat44 rotation_mul_scale_transposed(mInvInertiaDiagonal.SplatX() * rotation.GetColumn4(0), mInvInertiaDiagonal.SplatY() * rotation.GetColumn4(1), mInvInertiaDiagonal.SplatZ() * rotation.GetColumn4(2), Vec4(0, 0, 0, 1));
	return rotation.Multiply3x3RightTransposed(rotation_mul_scale_transposed);
}

inline void MotionProperties::ScaleToMass(float inMass)
{
	MOSS_ASSERT(mInvMass > 0.0f, "Body must have finite mass");
	MOSS_ASSERT(inMass > 0.0f, "New mass cannot be zero");

	float new_inv_mass = 1.0f / inMass;
	mInvInertiaDiagonal *= new_inv_mass / mInvMass;
	mInvMass = new_inv_mass;
}

inline Mat44 MotionProperties::GetLocalSpaceInverseInertia() const
{
	MOSS_ASSERT(mCachedMotionType == EMotionType::Dynamic);
	return GetLocalSpaceInverseInertiaUnchecked();
}

Mat44 MotionProperties::GetInverseInertiaForRotation(Mat44Arg inRotation) const
{
	MOSS_ASSERT(mCachedMotionType == EMotionType::Dynamic);

	Mat44 rotation = inRotation.Multiply3x3(Mat44::sRotation(mInertiaRotation));
	Mat44 rotation_mul_scale_transposed(mInvInertiaDiagonal.SplatX() * rotation.GetColumn4(0), mInvInertiaDiagonal.SplatY() * rotation.GetColumn4(1), mInvInertiaDiagonal.SplatZ() * rotation.GetColumn4(2), Vec4(0, 0, 0, 1));
	Mat44 inverse_inertia = rotation.Multiply3x3RightTransposed(rotation_mul_scale_transposed);

	// We need to mask out both the rows and columns of DOFs that are not allowed
	Vec4 angular_dofs_mask = GetAngularDOFsMask().ReinterpretAsFloat();
	inverse_inertia.SetColumn4(0, Vec4::sAnd(inverse_inertia.GetColumn4(0), Vec4::sAnd(angular_dofs_mask, angular_dofs_mask.SplatX())));
	inverse_inertia.SetColumn4(1, Vec4::sAnd(inverse_inertia.GetColumn4(1), Vec4::sAnd(angular_dofs_mask, angular_dofs_mask.SplatY())));
	inverse_inertia.SetColumn4(2, Vec4::sAnd(inverse_inertia.GetColumn4(2), Vec4::sAnd(angular_dofs_mask, angular_dofs_mask.SplatZ())));

	return inverse_inertia;
}

Vec3 MotionProperties::MultiplyWorldSpaceInverseInertiaByVector(QuatArg inBodyRotation, Vec3Arg inV) const
{
	MOSS_ASSERT(mCachedMotionType == EMotionType::Dynamic);

	// Mask out columns of DOFs that are not allowed
	Vec3 angular_dofs_mask = Vec3(GetAngularDOFsMask().ReinterpretAsFloat());
	Vec3 v = Vec3::sAnd(inV, angular_dofs_mask);

	// Multiply vector by inverse inertia
	Mat44 rotation = Mat44::sRotation(inBodyRotation * mInertiaRotation);
	Vec3 result = rotation.Multiply3x3(mInvInertiaDiagonal * rotation.Multiply3x3Transposed(v));

	// Mask out rows of DOFs that are not allowed
	return Vec3::sAnd(result, angular_dofs_mask);
}

void MotionProperties::ApplyGyroscopicForceInternal(QuatArg inBodyRotation, float inDeltaTime)
{
	MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sVelocityAccess(), BodyAccess::EAccess::ReadWrite));
	MOSS_ASSERT(mCachedBodyType == EBodyType::RigidBody);
	MOSS_ASSERT(mCachedMotionType == EMotionType::Dynamic);

	// Calculate local space inertia tensor (a diagonal in local space)
	UVec4 is_zero = Vec3::sEquals(mInvInertiaDiagonal, Vec3::sZero());
	Vec3 denominator = Vec3::sSelect(mInvInertiaDiagonal, Vec3::sOne(), is_zero);
	Vec3 nominator = Vec3::sSelect(Vec3::sOne(), Vec3::sZero(), is_zero);
	Vec3 local_inertia = nominator / denominator; // Avoid dividing by zero, inertia in this axis will be zero

	// Calculate local space angular momentum
	Quat inertia_space_to_world_space = inBodyRotation * mInertiaRotation;
	Vec3 local_angular_velocity = inertia_space_to_world_space.Conjugated() * mAngularVelocity;
	Vec3 local_momentum = local_inertia * local_angular_velocity;

	// The gyroscopic force applies a torque: T = -w x I w where w is angular velocity and I the inertia tensor
	// Calculate the new angular momentum by applying the gyroscopic force and make sure the new magnitude is the same as the old one
	// to avoid introducing energy into the system due to the Euler step
	Vec3 new_local_momentum = local_momentum - inDeltaTime * local_angular_velocity.Cross(local_momentum);
	float new_local_momentum_len_sq = new_local_momentum.LengthSq();
	new_local_momentum = new_local_momentum_len_sq > 0.0f? new_local_momentum * sqrt(local_momentum.LengthSq() / new_local_momentum_len_sq) : Vec3::sZero();

	// Convert back to world space angular velocity
	mAngularVelocity = inertia_space_to_world_space * (mInvInertiaDiagonal * new_local_momentum);
}

void MotionProperties::ApplyForceTorqueAndDragInternal(QuatArg inBodyRotation, Vec3Arg inGravity, float inDeltaTime)
{
	MOSS_ASSERT(BodyAccess::sCheckRights(BodyAccess::sVelocityAccess(), BodyAccess::EAccess::ReadWrite));
	MOSS_ASSERT(mCachedBodyType == EBodyType::RigidBody);
	MOSS_ASSERT(mCachedMotionType == EMotionType::Dynamic);

	// Update linear velocity
	mLinearVelocity = LockTranslation(mLinearVelocity + inDeltaTime * (mGravityFactor * inGravity + mInvMass * GetAccumulatedForce()));

	// Update angular velocity
	mAngularVelocity += inDeltaTime * MultiplyWorldSpaceInverseInertiaByVector(inBodyRotation, GetAccumulatedTorque());

	// Linear damping: dv/dt = -c * v
	// Solution: v(t) = v(0) * e^(-c * t) or v2 = v1 * e^(-c * dt)
	// Taylor expansion of e^(-c * dt) = 1 - c * dt + ...
	// Since dt is usually in the order of 1/60 and c is a low number too this approximation is good enough
	mLinearVelocity *= max(0.0f, 1.0f - mLinearDamping * inDeltaTime);
	mAngularVelocity *= max(0.0f, 1.0f - mAngularDamping * inDeltaTime);

	// Clamp velocities
	ClampLinearVelocity();
	ClampAngularVelocity();
}

void MotionProperties::ResetSleepTestSpheres(const RVec3 *inPoints)
{
#ifdef MOSS_DOUBLE_PRECISION
	// Make spheres relative to the first point and initialize them to zero radius
	DVec3 offset = inPoints[0];
	offset.StoreDouble3(&mSleepTestOffset);
	mSleepTestSpheres[0] = Sphere(Vec3::sZero(), 0.0f);
	for (int i = 1; i < 3; ++i)
		mSleepTestSpheres[i] = Sphere(Vec3(inPoints[i] - offset), 0.0f);
#else
	// Initialize the spheres to zero radius around the supplied points
	for (int i = 0; i < 3; ++i)
		mSleepTestSpheres[i] = Sphere(inPoints[i], 0.0f);
#endif

	mSleepTestTimer = 0.0f;
}

ECanSleep MotionProperties::AccumulateSleepTime(float inDeltaTime, float inTimeBeforeSleep)
{
	mSleepTestTimer += inDeltaTime;
	return mSleepTestTimer >= inTimeBeforeSleep? ECanSleep::CanSleep : ECanSleep::CannotSleep;
}

MOSS_NAMESPACE_END

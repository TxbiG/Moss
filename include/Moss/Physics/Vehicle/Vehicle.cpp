// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT


#include <Moss/Physics/physics_intern.h>
#ifndef MOSS_DEBUG_RENDERER
	#include <Moss/Moss_Renderer.h>
#endif // MOSS_DEBUG_RENDERER


#include <Moss/Physics/Vehicle/TrackedVehicleController.h>
#include <Moss/Physics/PhysicsSystem.h>
#include <Moss/ObjectStream/TypeDeclarations.h>
#include <Moss/Physics/Vehicle/VehicleController.h>
#include <Moss/Core/Factory.h>
#include <Moss/Physics/Vehicle/VehicleConstraint.h>
#include <Moss/Physics/Vehicle/VehicleController.h>

MOSS_SUPRESS_WARNINGS_BEGIN

MOSS_IMPLEMENT_SERIALIZABLE_ABSTRACT(VehicleControllerSettings) { MOSS_ADD_BASE_CLASS(VehicleControllerSettings, SerializableObject) }

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(VehicleAntiRollBar) {
	MOSS_ADD_ATTRIBUTE(VehicleAntiRollBar, mLeftWheel)
	MOSS_ADD_ATTRIBUTE(VehicleAntiRollBar, mRightWheel)
	MOSS_ADD_ATTRIBUTE(VehicleAntiRollBar, mStiffness)
}

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(VehicleDifferentialSettings) {
	MOSS_ADD_ATTRIBUTE(VehicleDifferentialSettings, mLeftWheel)
	MOSS_ADD_ATTRIBUTE(VehicleDifferentialSettings, mRightWheel)
	MOSS_ADD_ATTRIBUTE(VehicleDifferentialSettings, mDifferentialRatio)
	MOSS_ADD_ATTRIBUTE(VehicleDifferentialSettings, mLeftRightSplit)
	MOSS_ADD_ATTRIBUTE(VehicleDifferentialSettings, mLimitedSlipRatio)
	MOSS_ADD_ATTRIBUTE(VehicleDifferentialSettings, mEngineTorqueRatio)
}

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(VehicleDifferentialSettings) {
	MOSS_ADD_ATTRIBUTE(VehicleDifferentialSettings, mLeftWheel)
	MOSS_ADD_ATTRIBUTE(VehicleDifferentialSettings, mRightWheel)
	MOSS_ADD_ATTRIBUTE(VehicleDifferentialSettings, mDifferentialRatio)
	MOSS_ADD_ATTRIBUTE(VehicleDifferentialSettings, mLeftRightSplit)
	MOSS_ADD_ATTRIBUTE(VehicleDifferentialSettings, mLimitedSlipRatio)
	MOSS_ADD_ATTRIBUTE(VehicleDifferentialSettings, mEngineTorqueRatio)
}

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(VehicleEngineSettings) {
	MOSS_ADD_ATTRIBUTE(VehicleEngineSettings, mMaxTorque)
	MOSS_ADD_ATTRIBUTE(VehicleEngineSettings, mMinRPM)
	MOSS_ADD_ATTRIBUTE(VehicleEngineSettings, mMaxRPM)
	MOSS_ADD_ATTRIBUTE(VehicleEngineSettings, mNormalizedTorque)
}

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(VehicleTrackSettings) {
	MOSS_ADD_ATTRIBUTE(VehicleTrackSettings, mDrivenWheel)
	MOSS_ADD_ATTRIBUTE(VehicleTrackSettings, mWheels)
	MOSS_ADD_ATTRIBUTE(VehicleTrackSettings, mInertia)
	MOSS_ADD_ATTRIBUTE(VehicleTrackSettings, mAngularDamping)
	MOSS_ADD_ATTRIBUTE(VehicleTrackSettings, mMaxBrakeTorque)
	MOSS_ADD_ATTRIBUTE(VehicleTrackSettings, mDifferentialRatio)
}

MOSS_IMPLEMENT_SERIALIZABLE_NON_VIRTUAL(VehicleTransmissionSettings) {
	MOSS_ADD_ENUM_ATTRIBUTE(VehicleTransmissionSettings, mMode)
	MOSS_ADD_ATTRIBUTE(VehicleTransmissionSettings, mGearRatios)
	MOSS_ADD_ATTRIBUTE(VehicleTransmissionSettings, mReverseGearRatios)
	MOSS_ADD_ATTRIBUTE(VehicleTransmissionSettings, mSwitchTime)
	MOSS_ADD_ATTRIBUTE(VehicleTransmissionSettings, mClutchReleaseTime)
	MOSS_ADD_ATTRIBUTE(VehicleTransmissionSettings, mSwitchLatency)
	MOSS_ADD_ATTRIBUTE(VehicleTransmissionSettings, mShiftUpRPM)
	MOSS_ADD_ATTRIBUTE(VehicleTransmissionSettings, mShiftDownRPM)
	MOSS_ADD_ATTRIBUTE(VehicleTransmissionSettings, mClutchStrength)
}

MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(TrackedVehicleControllerSettings) {
	MOSS_ADD_BASE_CLASS(TrackedVehicleControllerSettings, VehicleControllerSettings)
	MOSS_ADD_ATTRIBUTE(TrackedVehicleControllerSettings, mEngine)
	MOSS_ADD_ATTRIBUTE(TrackedVehicleControllerSettings, mTransmission)
	MOSS_ADD_ATTRIBUTE(TrackedVehicleControllerSettings, mTracks)
}

MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(WheelSettingsTV) {
	MOSS_ADD_ATTRIBUTE(WheelSettingsTV, mLongitudinalFriction)
	MOSS_ADD_ATTRIBUTE(WheelSettingsTV, mLateralFriction)
}

MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(VehicleConstraintSettings) {
	MOSS_ADD_BASE_CLASS(VehicleConstraintSettings, ConstraintSettings)

	MOSS_ADD_ATTRIBUTE(VehicleConstraintSettings, mUp)
	MOSS_ADD_ATTRIBUTE(VehicleConstraintSettings, mForward)
	MOSS_ADD_ATTRIBUTE(VehicleConstraintSettings, mMaxPitchRollAngle)
	MOSS_ADD_ATTRIBUTE(VehicleConstraintSettings, mWheels)
	MOSS_ADD_ATTRIBUTE(VehicleConstraintSettings, mAntiRollBars)
	MOSS_ADD_ATTRIBUTE(VehicleConstraintSettings, mController)
}

MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(WheelSettings) {
	MOSS_ADD_ATTRIBUTE(WheelSettings, mSuspensionForcePoint)
	MOSS_ADD_ATTRIBUTE(WheelSettings, mPosition)
	MOSS_ADD_ATTRIBUTE(WheelSettings, mSuspensionDirection)
	MOSS_ADD_ATTRIBUTE(WheelSettings, mSteeringAxis)
	MOSS_ADD_ATTRIBUTE(WheelSettings, mWheelForward)
	MOSS_ADD_ATTRIBUTE(WheelSettings, mWheelUp)
	MOSS_ADD_ATTRIBUTE(WheelSettings, mSuspensionMinLength)
	MOSS_ADD_ATTRIBUTE(WheelSettings, mSuspensionMaxLength)
	MOSS_ADD_ATTRIBUTE(WheelSettings, mSuspensionPreloadLength)
	MOSS_ADD_ENUM_ATTRIBUTE_WITH_ALIAS(WheelSettings, mSuspensionSpring.mMode, "mSuspensionSpringMode")
	MOSS_ADD_ATTRIBUTE_WITH_ALIAS(WheelSettings, mSuspensionSpring.mFrequency, "mSuspensionFrequency") // Renaming attributes to stay compatible with old versions of the library
	MOSS_ADD_ATTRIBUTE_WITH_ALIAS(WheelSettings, mSuspensionSpring.mDamping, "mSuspensionDamping")
	MOSS_ADD_ATTRIBUTE(WheelSettings, mRadius)
	MOSS_ADD_ATTRIBUTE(WheelSettings, mWidth)
	MOSS_ADD_ATTRIBUTE(WheelSettings, mEnableSuspensionForcePoint)
}

MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(WheeledVehicleControllerSettings) {
	MOSS_ADD_BASE_CLASS(WheeledVehicleControllerSettings, VehicleControllerSettings)

	MOSS_ADD_ATTRIBUTE(WheeledVehicleControllerSettings, mEngine)
	MOSS_ADD_ATTRIBUTE(WheeledVehicleControllerSettings, mTransmission)
	MOSS_ADD_ATTRIBUTE(WheeledVehicleControllerSettings, mDifferentials)
	MOSS_ADD_ATTRIBUTE(WheeledVehicleControllerSettings, mDifferentialLimitedSlipRatio)
}

MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(WheelSettingsWV) {
	MOSS_ADD_ATTRIBUTE(WheelSettingsWV, mInertia)
	MOSS_ADD_ATTRIBUTE(WheelSettingsWV, mAngularDamping)
	MOSS_ADD_ATTRIBUTE(WheelSettingsWV, mMaxSteerAngle)
	MOSS_ADD_ATTRIBUTE(WheelSettingsWV, mLongitudinalFriction)
	MOSS_ADD_ATTRIBUTE(WheelSettingsWV, mLateralFriction)
	MOSS_ADD_ATTRIBUTE(WheelSettingsWV, mMaxBrakeTorque)
	MOSS_ADD_ATTRIBUTE(WheelSettingsWV, mMaxHandBrakeTorque)
}

MOSS_IMPLEMENT_SERIALIZABLE_VIRTUAL(MotorcycleControllerSettings) {
	MOSS_ADD_BASE_CLASS(MotorcycleControllerSettings, VehicleControllerSettings)

	MOSS_ADD_ATTRIBUTE(MotorcycleControllerSettings, mMaxLeanAngle)
	MOSS_ADD_ATTRIBUTE(MotorcycleControllerSettings, mLeanSpringConstant)
	MOSS_ADD_ATTRIBUTE(MotorcycleControllerSettings, mLeanSpringDamping)
	MOSS_ADD_ATTRIBUTE(MotorcycleControllerSettings, mLeanSpringIntegrationCoefficient)
	MOSS_ADD_ATTRIBUTE(MotorcycleControllerSettings, mLeanSpringIntegrationCoefficientDecay)
	MOSS_ADD_ATTRIBUTE(MotorcycleControllerSettings, mLeanSmoothingFactor)
}

WheelTV::WheelTV(const WheelSettingsTV &inSettings) : Wheel(inSettings) { }

void WheelTV::CalculateAngularVelocity(const VehicleConstraint &inConstraint)
{
	const WheelSettingsTV *settings = GetSettings();
	const Wheels &wheels = inConstraint.GetWheels();
	const VehicleTrack &track = static_cast<const TrackedVehicleController *>(inConstraint.GetController())->GetTracks()[mTrackIndex];

	// Calculate angular velocity of this wheel
	mAngularVelocity = track.mAngularVelocity * wheels[track.mDrivenWheel]->GetSettings()->mRadius / settings->mRadius;
}

void WheelTV::Update(uint inWheelIndex, float inDeltaTime, const VehicleConstraint &inConstraint)
{
	CalculateAngularVelocity(inConstraint);

	// Update rotation of wheel
	mAngle = fmod(mAngle + mAngularVelocity * inDeltaTime, 2.0f * MOSS_PI);

	// Reset brake impulse, will be set during post collision again
	mBrakeImpulse = 0.0f;

	if (mContactBody != nullptr)
	{
		// Friction at the point of this wheel between track and floor
		const WheelSettingsTV *settings = GetSettings();
		VehicleConstraint::CombineFunction combine_friction = inConstraint.GetCombineFriction();
		mCombinedLongitudinalFriction = settings->mLongitudinalFriction;
		mCombinedLateralFriction = settings->mLateralFriction;
		combine_friction(inWheelIndex, mCombinedLongitudinalFriction, mCombinedLateralFriction, *mContactBody, mContactSubShapeID);
	}
	else
	{
		// No collision
		mCombinedLongitudinalFriction = mCombinedLateralFriction = 0.0f;
	}
}

VehicleController *TrackedVehicleControllerSettings::ConstructController(VehicleConstraint &inConstraint) const
{
	return new TrackedVehicleController(*this, inConstraint);
}

TrackedVehicleControllerSettings::TrackedVehicleControllerSettings()
{
	// Numbers guestimated from: https://en.wikipedia.org/wiki/M1_Abrams
	mEngine.mMinRPM = 500.0f;
	mEngine.mMaxRPM = 4000.0f;
	mEngine.mMaxTorque = 500.0f; // Note actual torque for M1 is around 5000 but we need a reduced mass in order to keep the simulation sane

	mTransmission.mShiftDownRPM = 1000.0f;
	mTransmission.mShiftUpRPM = 3500.0f;
	mTransmission.mGearRatios = { 4.0f, 3.0f, 2.0f, 1.0f };
	mTransmission.mReverseGearRatios = { -4.0f, -3.0f };
}

TrackedVehicleController::TrackedVehicleController(const TrackedVehicleControllerSettings &inSettings, VehicleConstraint &inConstraint) :
	VehicleController(inConstraint)
{
	// Copy engine settings
	static_cast<VehicleEngineSettings &>(mEngine) = inSettings.mEngine;
	MOSS_ASSERT(inSettings.mEngine.mMinRPM >= 0.0f);
	MOSS_ASSERT(inSettings.mEngine.mMinRPM <= inSettings.mEngine.mMaxRPM);
	mEngine.SetCurrentRPM(mEngine.mMinRPM);

	// Copy transmission settings
	static_cast<VehicleTransmissionSettings &>(mTransmission) = inSettings.mTransmission;
#ifdef MOSS_DEBUG
	for (float r : inSettings.mTransmission.mGearRatios)
		MOSS_ASSERT(r > 0.0f);
	for (float r : inSettings.mTransmission.mReverseGearRatios)
		MOSS_ASSERT(r < 0.0f);
#endif // MOSS_DEBUG
	MOSS_ASSERT(inSettings.mTransmission.mSwitchTime >= 0.0f);
	MOSS_ASSERT(inSettings.mTransmission.mShiftDownRPM > 0.0f);
	MOSS_ASSERT(inSettings.mTransmission.mMode != ETransmissionMode::Auto || inSettings.mTransmission.mShiftUpRPM < inSettings.mEngine.mMaxRPM);
	MOSS_ASSERT(inSettings.mTransmission.mShiftUpRPM > inSettings.mTransmission.mShiftDownRPM);

	// Copy track settings
	for (uint i = 0; i < std::size(mTracks); ++i)
	{
		const VehicleTrackSettings &d = inSettings.mTracks[i];
		static_cast<VehicleTrackSettings &>(mTracks[i]) = d;
		MOSS_ASSERT(d.mInertia >= 0.0f);
		MOSS_ASSERT(d.mAngularDamping >= 0.0f);
		MOSS_ASSERT(d.mMaxBrakeTorque >= 0.0f);
		MOSS_ASSERT(d.mDifferentialRatio > 0.0f);
	}
}

bool TrackedVehicleController::AllowSleep() const
{
	return mForwardInput == 0.0f								// No user input
		&& mTransmission.AllowSleep()							// Transmission is not shifting
		&& mEngine.AllowSleep();								// Engine is idling
}

void TrackedVehicleController::PreCollide(float inDeltaTime, PhysicsSystem &inPhysicsSystem)
{
	Wheels &wheels = mConstraint.GetWheels();

	// Fill in track index
	for (size_t t = 0; t < std::size(mTracks); ++t)
		for (uint w : mTracks[t].mWheels)
			static_cast<WheelTV *>(wheels[w])->mTrackIndex = (uint)t;

	// Angular damping: dw/dt = -c * w
	// Solution: w(t) = w(0) * e^(-c * t) or w2 = w1 * e^(-c * dt)
	// Taylor expansion of e^(-c * dt) = 1 - c * dt + ...
	// Since dt is usually in the order of 1/60 and c is a low number too this approximation is good enough
	for (VehicleTrack &t : mTracks)
		t.mAngularVelocity *= max(0.0f, 1.0f - t.mAngularDamping * inDeltaTime);
}

void TrackedVehicleController::SyncLeftRightTracks()
{
	// Apply left to right ratio according to track inertias
	VehicleTrack &tl = mTracks[(int)ETrackSide::Left];
	VehicleTrack &tr = mTracks[(int)ETrackSide::Right];

	if (mLeftRatio * mRightRatio > 0.0f)
	{
		// Solve: (tl.mAngularVelocity + dl) / (tr.mAngularVelocity + dr) = mLeftRatio / mRightRatio and dl * tr.mInertia = -dr * tl.mInertia, where dl/dr are the delta angular velocities for left and right tracks
		float impulse = (mLeftRatio * tr.mAngularVelocity - mRightRatio * tl.mAngularVelocity) / (mLeftRatio * tr.mInertia + mRightRatio * tl.mInertia);
		tl.mAngularVelocity += impulse * tl.mInertia;
		tr.mAngularVelocity -= impulse * tr.mInertia;
	}
	else
	{
		// Solve: (tl.mAngularVelocity + dl) / (tr.mAngularVelocity + dr) = mLeftRatio / mRightRatio and dl * tr.mInertia = dr * tl.mInertia, where dl/dr are the delta angular velocities for left and right tracks
		float impulse = (mLeftRatio * tr.mAngularVelocity - mRightRatio * tl.mAngularVelocity) / (mRightRatio * tl.mInertia - mLeftRatio * tr.mInertia);
		tl.mAngularVelocity += impulse * tl.mInertia;
		tr.mAngularVelocity += impulse * tr.mInertia;
	}
}

void TrackedVehicleController::PostCollide(float inDeltaTime, PhysicsSystem &inPhysicsSystem)
{
	MOSS_PROFILE_FUNCTION();

	Wheels &wheels = mConstraint.GetWheels();

	// Update wheel angle, do this before applying torque to the wheels (as friction will slow them down again)
	for (uint wheel_index = 0, num_wheels = (uint)wheels.size(); wheel_index < num_wheels; ++wheel_index)
	{
		WheelTV *w = static_cast<WheelTV *>(wheels[wheel_index]);
		w->Update(wheel_index, inDeltaTime, mConstraint);
	}

	// First calculate engine speed based on speed of all wheels
	bool can_engine_apply_torque = false;
	if (mTransmission.GetCurrentGear() != 0 && mTransmission.GetClutchFriction() > 1.0e-3f)
	{
		float transmission_ratio = mTransmission.GetCurrentRatio();
		bool forward = transmission_ratio >= 0.0f;
		float fastest_wheel_speed = forward? -FLT_MAX : FLT_MAX;
		for (const VehicleTrack &t : mTracks)
		{
			if (forward)
				fastest_wheel_speed = max(fastest_wheel_speed, t.mAngularVelocity * t.mDifferentialRatio);
			else
				fastest_wheel_speed = min(fastest_wheel_speed, t.mAngularVelocity * t.mDifferentialRatio);
			for (uint w : t.mWheels)
				if (wheels[w]->HasContact())
				{
					can_engine_apply_torque = true;
					break;
				}
		}

		// Update RPM only if the tracks are connected to the engine
		if (fastest_wheel_speed > -FLT_MAX && fastest_wheel_speed < FLT_MAX)
			mEngine.SetCurrentRPM(fastest_wheel_speed * mTransmission.GetCurrentRatio() * VehicleEngine::cAngularVelocityToRPM);
	}
	else
	{
		// Update engine with damping
		mEngine.ApplyDamping(inDeltaTime);

		// In auto transmission mode, don't accelerate the engine when switching gears
		float forward_input = mTransmission.mMode == ETransmissionMode::Manual? abs(mForwardInput) : 0.0f;

		// Engine not connected to wheels, update RPM based on engine inertia alone
		mEngine.ApplyTorque(mEngine.GetTorque(forward_input), inDeltaTime);
	}

	// Update transmission
	// Note: only allow switching gears up when the tracks are rolling in the same direction
	mTransmission.Update(inDeltaTime, mEngine.GetCurrentRPM(), mForwardInput, mLeftRatio * mRightRatio > 0.0f && can_engine_apply_torque);

	// Calculate the amount of torque the transmission gives to the differentials
	float transmission_ratio = mTransmission.GetCurrentRatio();
	float transmission_torque = mTransmission.GetClutchFriction() * transmission_ratio * mEngine.GetTorque(abs(mForwardInput));
	if (transmission_torque != 0.0f)
	{
		// Apply the transmission torque to the wheels
		for (uint i = 0; i < std::size(mTracks); ++i)
		{
			VehicleTrack &t = mTracks[i];

			// Get wheel rotation ratio for this track
			float ratio = i == 0? mLeftRatio : mRightRatio;

			// Calculate the max angular velocity of the driven wheel of the track given current engine RPM
			// Note this adds 0.1% slop to avoid numerical accuracy issues
			float track_max_angular_velocity = mEngine.GetCurrentRPM() / (transmission_ratio * t.mDifferentialRatio * ratio * VehicleEngine::cAngularVelocityToRPM) * 1.001f;

			// Calculate torque on the driven wheel
			float differential_torque = t.mDifferentialRatio * ratio * transmission_torque;

			// Apply torque to driven wheel
			if (t.mAngularVelocity * track_max_angular_velocity < 0.0f || abs(t.mAngularVelocity) < abs(track_max_angular_velocity))
				t.mAngularVelocity += differential_torque * inDeltaTime / t.mInertia;
		}
	}

	// Ensure that we have the correct ratio between the two tracks
	SyncLeftRightTracks();

	// Braking
	for (VehicleTrack &t : mTracks)
	{
		// Calculate brake torque
		float brake_torque = mBrakeInput * t.mMaxBrakeTorque;
		if (brake_torque > 0.0f)
		{
			// Calculate how much torque is needed to stop the track from rotating in this time step
			float brake_torque_to_lock_track = abs(t.mAngularVelocity) * t.mInertia / inDeltaTime;
			if (brake_torque > brake_torque_to_lock_track)
			{
				// Wheels are locked
				t.mAngularVelocity = 0.0f;
				brake_torque -= brake_torque_to_lock_track;
			}
			else
			{
				// Slow down the track
				t.mAngularVelocity -= Sign(t.mAngularVelocity) * brake_torque * inDeltaTime / t.mInertia;
			}
		}

		if (brake_torque > 0.0f)
		{
			// Sum the radius of all wheels touching the floor
			float total_radius = 0.0f;
			for (uint wheel_index : t.mWheels)
			{
				const WheelTV *w = static_cast<WheelTV *>(wheels[wheel_index]);

				if (w->HasContact())
					total_radius += w->GetSettings()->mRadius;
			}

			if (total_radius > 0.0f)
			{
				brake_torque /= total_radius;
				for (uint wheel_index : t.mWheels)
				{
					WheelTV *w = static_cast<WheelTV *>(wheels[wheel_index]);
					if (w->HasContact())
					{
						// Impulse: p = F * dt = Torque / Wheel_Radius * dt, Torque = Total_Torque * Wheel_Radius / Summed_Radius => p = Total_Torque * dt / Summed_Radius
						w->mBrakeImpulse = brake_torque * inDeltaTime;
					}
				}
			}
		}
	}

	// Update wheel angular velocity based on that of the track
	for (Wheel *w_base : wheels)
	{
		WheelTV *w = static_cast<WheelTV *>(w_base);
		w->CalculateAngularVelocity(mConstraint);
	}
}

bool TrackedVehicleController::SolveLongitudinalAndLateralConstraints(float inDeltaTime)
{
	bool impulse = false;

	for (Wheel *w_base : mConstraint.GetWheels())
		if (w_base->HasContact())
		{
			WheelTV *w = static_cast<WheelTV *>(w_base);
			const WheelSettingsTV *settings = w->GetSettings();
			VehicleTrack &track = mTracks[w->mTrackIndex];

			// Calculate max impulse that we can apply on the ground
			float max_longitudinal_friction_impulse = w->mCombinedLongitudinalFriction * w->GetSuspensionLambda();

			// Calculate relative velocity between wheel contact point and floor in longitudinal direction
			Vec3 relative_velocity = mConstraint.GetVehicleBody()->GetPointVelocity(w->GetContactPosition()) - w->GetContactPointVelocity();
			float relative_longitudinal_velocity = relative_velocity.Dot(w->GetContactLongitudinal());

			// Calculate brake force to apply
			float min_longitudinal_impulse, max_longitudinal_impulse;
			if (w->mBrakeImpulse != 0.0f)
			{
				// Limit brake force by max tire friction
				float brake_impulse = min(w->mBrakeImpulse, max_longitudinal_friction_impulse);

				// Check which direction the brakes should be applied (we don't want to apply an impulse that would accelerate the vehicle)
				if (relative_longitudinal_velocity >= 0.0f)
				{
					min_longitudinal_impulse = -brake_impulse;
					max_longitudinal_impulse = 0.0f;
				}
				else
				{
					min_longitudinal_impulse = 0.0f;
					max_longitudinal_impulse = brake_impulse;
				}

				// Longitudinal impulse, note that we assume that once the wheels are locked that the brakes have more than enough torque to keep the wheels locked so we exclude any rotation deltas
				impulse |= w->SolveLongitudinalConstraintPart(mConstraint, min_longitudinal_impulse, max_longitudinal_impulse);
			}
			else
			{
				// Assume we want to apply an angular impulse that makes the delta velocity between track and ground zero in one time step, calculate the amount of linear impulse needed to do that
				float desired_angular_velocity = relative_longitudinal_velocity / settings->mRadius;
				float linear_impulse = (track.mAngularVelocity - desired_angular_velocity) * track.mInertia / settings->mRadius;

				// Limit the impulse by max track friction
				float prev_lambda = w->GetLongitudinalLambda();
				min_longitudinal_impulse = max_longitudinal_impulse = Clamp(prev_lambda + linear_impulse, -max_longitudinal_friction_impulse, max_longitudinal_friction_impulse);

				// Longitudinal impulse
				impulse |= w->SolveLongitudinalConstraintPart(mConstraint, min_longitudinal_impulse, max_longitudinal_impulse);

				// Update the angular velocity of the track according to the lambda that was applied
				track.mAngularVelocity -= (w->GetLongitudinalLambda() - prev_lambda) * settings->mRadius / track.mInertia;
				SyncLeftRightTracks();
			}
		}

	for (Wheel *w_base : mConstraint.GetWheels())
		if (w_base->HasContact())
		{
			WheelTV *w = static_cast<WheelTV *>(w_base);

			// Update angular velocity of wheel for the next iteration
			w->CalculateAngularVelocity(mConstraint);

			// Lateral friction
			float max_lateral_friction_impulse = w->mCombinedLateralFriction * w->GetSuspensionLambda();
			impulse |= w->SolveLateralConstraintPart(mConstraint, -max_lateral_friction_impulse, max_lateral_friction_impulse);
		}

	return impulse;
}

#ifndef MOSS_DEBUG_RENDERER

void TrackedVehicleController::Draw(DebugRenderer *inRenderer) const
{
	float constraint_size = mConstraint.GetDrawConstraintSize();

	// Draw RPM
	Body *body = mConstraint.GetVehicleBody();
	Vec3 rpm_meter_up = body->GetRotation() * mConstraint.GetLocalUp();
	RVec3 rpm_meter_pos = body->GetPosition() + body->GetRotation() * mRPMMeterPosition;
	Vec3 rpm_meter_fwd = body->GetRotation() * mConstraint.GetLocalForward();
	mEngine.DrawRPM(inRenderer, rpm_meter_pos, rpm_meter_fwd, rpm_meter_up, mRPMMeterSize, mTransmission.mShiftDownRPM, mTransmission.mShiftUpRPM);

	// Draw current vehicle state
	String status = StringFormat("Forward: %.1f, LRatio: %.1f, RRatio: %.1f, Brake: %.1f\n"
								 "Gear: %d, Clutch: %.1f, EngineRPM: %.0f, V: %.1f km/h",
								 (double)mForwardInput, (double)mLeftRatio, (double)mRightRatio, (double)mBrakeInput,
								 mTransmission.GetCurrentGear(), (double)mTransmission.GetClutchFriction(), (double)mEngine.GetCurrentRPM(), (double)body->GetLinearVelocity().Length() * 3.6);
	inRenderer->DrawText3D(body->GetPosition(), status, Color::sWhite, constraint_size);

	for (const VehicleTrack &t : mTracks)
	{
		const WheelTV *w = static_cast<const WheelTV *>(mConstraint.GetWheels()[t.mDrivenWheel]);
		const WheelSettings *settings = w->GetSettings();

		// Calculate where the suspension attaches to the body in world space
		RVec3 ws_position = body->GetCenterOfMassPosition() + body->GetRotation() * (settings->mPosition - body->GetShape()->GetCenterOfMass());

		DebugRenderer::sInstance->DrawText3D(ws_position, StringFormat("W: %.1f", (double)t.mAngularVelocity), Color::sWhite, constraint_size);
	}

	RMat44 body_transform = body->GetWorldTransform();

	for (const Wheel *w_base : mConstraint.GetWheels())
	{
		const WheelTV *w = static_cast<const WheelTV *>(w_base);
		const WheelSettings *settings = w->GetSettings();

		// Calculate where the suspension attaches to the body in world space
		RVec3 ws_position = body_transform * settings->mPosition;
		Vec3 ws_direction = body_transform.Multiply3x3(settings->mSuspensionDirection);

		// Draw suspension
		RVec3 min_suspension_pos = ws_position + ws_direction * settings->mSuspensionMinLength;
		RVec3 max_suspension_pos = ws_position + ws_direction * settings->mSuspensionMaxLength;
		inRenderer->DrawLine(ws_position, min_suspension_pos, Color::sRed);
		inRenderer->DrawLine(min_suspension_pos, max_suspension_pos, Color::sGreen);

		// Draw current length
		RVec3 wheel_pos = ws_position + ws_direction * w->GetSuspensionLength();
		inRenderer->DrawMarker(wheel_pos, w->GetSuspensionLength() < settings->mSuspensionMinLength? Color::sRed : Color::sGreen, constraint_size);

		// Draw wheel basis
		Vec3 wheel_forward, wheel_up, wheel_right;
		mConstraint.GetWheelLocalBasis(w, wheel_forward, wheel_up, wheel_right);
		wheel_forward = body_transform.Multiply3x3(wheel_forward);
		wheel_up = body_transform.Multiply3x3(wheel_up);
		wheel_right = body_transform.Multiply3x3(wheel_right);
		Vec3 steering_axis = body_transform.Multiply3x3(settings->mSteeringAxis);
		inRenderer->DrawLine(wheel_pos, wheel_pos + wheel_forward, Color::sRed);
		inRenderer->DrawLine(wheel_pos, wheel_pos + wheel_up, Color::sGreen);
		inRenderer->DrawLine(wheel_pos, wheel_pos + wheel_right, Color::sBlue);
		inRenderer->DrawLine(wheel_pos, wheel_pos + steering_axis, Color::sYellow);

		// Draw wheel
		RMat44 wheel_transform(Vec4(wheel_up, 0.0f), Vec4(wheel_right, 0.0f), Vec4(wheel_forward, 0.0f), wheel_pos);
		wheel_transform.SetRotation(wheel_transform.GetRotation() * Mat44::sRotationY(-w->GetRotationAngle()));
		inRenderer->DrawCylinder(wheel_transform, settings->mWidth * 0.5f, settings->mRadius, w->GetSuspensionLength() <= settings->mSuspensionMinLength? Color::sRed : Color::sGreen, DebugRenderer::ECastShadow::Off, DebugRenderer::EDrawMode::Wireframe);

		if (w->HasContact())
		{
			// Draw contact
			inRenderer->DrawLine(w->GetContactPosition(), w->GetContactPosition() + w->GetContactNormal(), Color::sYellow);
			inRenderer->DrawLine(w->GetContactPosition(), w->GetContactPosition() + w->GetContactLongitudinal(), Color::sRed);
			inRenderer->DrawLine(w->GetContactPosition(), w->GetContactPosition() + w->GetContactLateral(), Color::sBlue);

			DebugRenderer::sInstance->DrawText3D(w->GetContactPosition(), StringFormat("S: %.2f", (double)w->GetSuspensionLength()), Color::sWhite, constraint_size);
		}
	}
}

#endif // MOSS_DEBUG_RENDERER

void TrackedVehicleController::SaveState(StateRecorder &inStream) const
{
	inStream.Write(mForwardInput);
	inStream.Write(mLeftRatio);
	inStream.Write(mRightRatio);
	inStream.Write(mBrakeInput);

	mEngine.SaveState(inStream);
	mTransmission.SaveState(inStream);

	for (const VehicleTrack &t : mTracks)
		t.SaveState(inStream);
}

void TrackedVehicleController::RestoreState(StateRecorder &inStream)
{
	inStream.Read(mForwardInput);
	inStream.Read(mLeftRatio);
	inStream.Read(mRightRatio);
	inStream.Read(mBrakeInput);

	mEngine.RestoreState(inStream);
	mTransmission.RestoreState(inStream);

	for (VehicleTrack &t : mTracks)
		t.RestoreState(inStream);
}



void VehicleDifferentialSettings::CalculateTorqueRatio(float inLeftAngularVelocity, float inRightAngularVelocity, float &outLeftTorqueFraction, float &outRightTorqueFraction) const
{
	// Start with the default torque ratio
	outLeftTorqueFraction = 1.0f - mLeftRightSplit;
	outRightTorqueFraction = mLeftRightSplit;

	if (mLimitedSlipRatio < FLT_MAX)
	{
		MOSS_ASSERT(mLimitedSlipRatio > 1.0f);

		// This is a limited slip differential, adjust torque ratios according to wheel speeds
		float omega_l = max(1.0e-3f, abs(inLeftAngularVelocity)); // prevent div by zero by setting a minimum velocity and ignoring that the wheels may be rotating in different directions
		float omega_r = max(1.0e-3f, abs(inRightAngularVelocity));
		float omega_min = min(omega_l, omega_r);
		float omega_max = max(omega_l, omega_r);

		// Map into a value that is 0 when the wheels are turning at an equal rate and 1 when the wheels are turning at mLimitedSlipRotationRatio
		float alpha = min((omega_max / omega_min - 1.0f) / (mLimitedSlipRatio - 1.0f), 1.0f);
		MOSS_ASSERT(alpha >= 0.0f);
		float one_min_alpha = 1.0f - alpha;

		if (omega_l < omega_r)
		{
			// Redirect more power to the left wheel
			outLeftTorqueFraction = outLeftTorqueFraction * one_min_alpha + alpha;
			outRightTorqueFraction = outRightTorqueFraction * one_min_alpha;
		}
		else
		{
			// Redirect more power to the right wheel
			outLeftTorqueFraction = outLeftTorqueFraction * one_min_alpha;
			outRightTorqueFraction = outRightTorqueFraction * one_min_alpha + alpha;
		}
	}

	// Assert the values add up to 1
	MOSS_ASSERT(abs(outLeftTorqueFraction + outRightTorqueFraction - 1.0f) < 1.0e-6f);
}


void VehicleDifferentialSettings::CalculateTorqueRatio(float inLeftAngularVelocity, float inRightAngularVelocity, float &outLeftTorqueFraction, float &outRightTorqueFraction) const
{
	// Start with the default torque ratio
	outLeftTorqueFraction = 1.0f - mLeftRightSplit;
	outRightTorqueFraction = mLeftRightSplit;

	if (mLimitedSlipRatio < FLT_MAX)
	{
		MOSS_ASSERT(mLimitedSlipRatio > 1.0f);

		// This is a limited slip differential, adjust torque ratios according to wheel speeds
		float omega_l = max(1.0e-3f, abs(inLeftAngularVelocity)); // prevent div by zero by setting a minimum velocity and ignoring that the wheels may be rotating in different directions
		float omega_r = max(1.0e-3f, abs(inRightAngularVelocity));
		float omega_min = min(omega_l, omega_r);
		float omega_max = max(omega_l, omega_r);

		// Map into a value that is 0 when the wheels are turning at an equal rate and 1 when the wheels are turning at mLimitedSlipRotationRatio
		float alpha = min((omega_max / omega_min - 1.0f) / (mLimitedSlipRatio - 1.0f), 1.0f);
		MOSS_ASSERT(alpha >= 0.0f);
		float one_min_alpha = 1.0f - alpha;

		if (omega_l < omega_r)
		{
			// Redirect more power to the left wheel
			outLeftTorqueFraction = outLeftTorqueFraction * one_min_alpha + alpha;
			outRightTorqueFraction = outRightTorqueFraction * one_min_alpha;
		}
		else
		{
			// Redirect more power to the right wheel
			outLeftTorqueFraction = outLeftTorqueFraction * one_min_alpha;
			outRightTorqueFraction = outRightTorqueFraction * one_min_alpha + alpha;
		}
	}

	// Assert the values add up to 1
	MOSS_ASSERT(abs(outLeftTorqueFraction + outRightTorqueFraction - 1.0f) < 1.0e-6f);
}

VehicleConstraint::VehicleConstraint(Body &inVehicleBody, const VehicleConstraintSettings &inSettings) :
	Constraint(inSettings),
	mBody(&inVehicleBody),
	mForward(inSettings.mForward),
	mUp(inSettings.mUp),
	mWorldUp(inSettings.mUp),
	mAntiRollBars(inSettings.mAntiRollBars)
{
	// Check sanity of incoming settings
	MOSS_ASSERT(inSettings.mUp.IsNormalized());
	MOSS_ASSERT(inSettings.mForward.IsNormalized());
	MOSS_ASSERT(!inSettings.mWheels.empty());

	// Store max pitch/roll angle
	SetMaxPitchRollAngle(inSettings.mMaxPitchRollAngle);

	// Construct our controller class
	mController = inSettings.mController->ConstructController(*this);

	// Create wheels
	mWheels.resize(inSettings.mWheels.size());
	for (uint i = 0; i < mWheels.size(); ++i)
		mWheels[i] = mController->ConstructWheel(*inSettings.mWheels[i]);

	// Use the body ID as a seed for the step counter so that not all vehicles will update at the same time
	mCurrentStep = uint32(Hash64(inVehicleBody.GetID().GetIndex()));
}

VehicleConstraint::~VehicleConstraint()
{
	// Destroy controller
	delete mController;

	// Destroy our wheels
	for (Wheel *w : mWheels)
		delete w;
}

void VehicleConstraint::GetWheelLocalBasis(const Wheel *inWheel, Vec3 &outForward, Vec3 &outUp, Vec3 &outRight) const
{
	const WheelSettings *settings = inWheel->mSettings;

	Quat steer_rotation = Quat::sRotation(settings->mSteeringAxis, inWheel->mSteerAngle);
	outUp = steer_rotation * settings->mWheelUp;
	outForward = steer_rotation * settings->mWheelForward;
	outRight = outForward.Cross(outUp).Normalized();
	outForward = outUp.Cross(outRight).Normalized();
}

Mat44 VehicleConstraint::GetWheelLocalTransform(uint inWheelIndex, Vec3Arg inWheelRight, Vec3Arg inWheelUp) const
{
	MOSS_ASSERT(inWheelIndex < mWheels.size());

	const Wheel *wheel = mWheels[inWheelIndex];
	const WheelSettings *settings = wheel->mSettings;

	// Use the two vectors provided to calculate a matrix that takes us from wheel model space to X = right, Y = up, Z = forward (the space where we will rotate the wheel)
	Mat44 wheel_to_rotational = Mat44(Vec4(inWheelRight, 0), Vec4(inWheelUp, 0), Vec4(inWheelUp.Cross(inWheelRight), 0), Vec4(0, 0, 0, 1)).Transposed();

	// Calculate the matrix that takes us from the rotational space to vehicle local space
	Vec3 local_forward, local_up, local_right;
	GetWheelLocalBasis(wheel, local_forward, local_up, local_right);
	Vec3 local_wheel_pos = settings->mPosition + settings->mSuspensionDirection * wheel->mSuspensionLength;
	Mat44 rotational_to_local(Vec4(local_right, 0), Vec4(local_up, 0), Vec4(local_forward, 0), Vec4(local_wheel_pos, 1));

	// Calculate transform of rotated wheel
	return rotational_to_local * Mat44::sRotationX(wheel->mAngle) * wheel_to_rotational;
}

RMat44 VehicleConstraint::GetWheelWorldTransform(uint inWheelIndex, Vec3Arg inWheelRight, Vec3Arg inWheelUp) const
{
	return mBody->GetWorldTransform() * GetWheelLocalTransform(inWheelIndex, inWheelRight, inWheelUp);
}

void VehicleConstraint::OnStep(const PhysicsStepListenerContext &inContext)
{
	MOSS_PROFILE_FUNCTION();

	// Callback to higher-level systems. We do it before PreCollide, in case steering changes.
	if (mPreStepCallback != nullptr)
		mPreStepCallback(*this, inContext);

	if (mIsGravityOverridden)
	{
		// If gravity is overridden, we replace the normal gravity calculations
		if (mBody->IsActive())
		{
			MotionProperties *mp = mBody->GetMotionProperties();
			mp->SetGravityFactor(0.0f);
			mBody->AddForce(mGravityOverride / mp->GetInverseMass());
		}

		// And we calculate the world up using the custom gravity
		mWorldUp = (-mGravityOverride).NormalizedOr(mWorldUp);
	}
	else
	{
		// Calculate new world up vector by inverting gravity
		mWorldUp = (-inContext.mPhysicsSystem->GetGravity()).NormalizedOr(mWorldUp);
	}

	// Callback on our controller
	mController->PreCollide(inContext.mDeltaTime, *inContext.mPhysicsSystem);

	// Calculate if this constraint is active by checking if our main vehicle body is active or any of the bodies we touch are active
	mIsActive = mBody->IsActive();

	// Test how often we need to update the wheels
	uint num_steps_between_collisions = mIsActive? mNumStepsBetweenCollisionTestActive : mNumStepsBetweenCollisionTestInactive;

	RMat44 body_transform = mBody->GetWorldTransform();

	// Test collision for wheels
	for (uint wheel_index = 0; wheel_index < mWheels.size(); ++wheel_index)
	{
		Wheel *w = mWheels[wheel_index];
		const WheelSettings *settings = w->mSettings;

		// Calculate suspension origin and direction
		RVec3 ws_origin = body_transform * settings->mPosition;
		Vec3 ws_direction = body_transform.Multiply3x3(settings->mSuspensionDirection);

		// Test if we need to update this wheel
		if (num_steps_between_collisions == 0
			|| (mCurrentStep + wheel_index) % num_steps_between_collisions != 0)
		{
			// Simplified wheel contact test
			if (!w->mContactBodyID.IsInvalid())
			{
				// Test if the body is still valid
				w->mContactBody = inContext.mPhysicsSystem->GetBodyLockInterfaceNoLock().TryGetBody(w->mContactBodyID);
				if (w->mContactBody == nullptr)
				{
					// It's not, forget the contact
					w->mContactBodyID = BodyID();
					w->mContactSubShapeID = SubShapeID();
					w->mSuspensionLength = settings->mSuspensionMaxLength;
				}
				else
				{
					// Extrapolate the wheel contact properties
					mVehicleCollisionTester->PredictContactProperties(*inContext.mPhysicsSystem, *this, wheel_index, ws_origin, ws_direction, mBody->GetID(), w->mContactBody, w->mContactSubShapeID, w->mContactPosition, w->mContactNormal, w->mSuspensionLength);
				}
			}
		}
		else
		{
			// Full wheel contact test, start by resetting the contact data
			w->mContactBodyID = BodyID();
			w->mContactBody = nullptr;
			w->mContactSubShapeID = SubShapeID();
			w->mSuspensionLength = settings->mSuspensionMaxLength;

			// Test collision to find the floor
			if (mVehicleCollisionTester->Collide(*inContext.mPhysicsSystem, *this, wheel_index, ws_origin, ws_direction, mBody->GetID(), w->mContactBody, w->mContactSubShapeID, w->mContactPosition, w->mContactNormal, w->mSuspensionLength))
			{
				// Store ID (pointer is not valid outside of the simulation step)
				w->mContactBodyID = w->mContactBody->GetID();
			}
		}

		if (w->mContactBody != nullptr)
		{
			// Store contact velocity, cache this as the contact body may be removed
			w->mContactPointVelocity = w->mContactBody->GetPointVelocity(w->mContactPosition);

			// Determine plane constant for axle contact plane
			w->mAxlePlaneConstant = RVec3(w->mContactNormal).Dot(ws_origin + w->mSuspensionLength * ws_direction);

			// Check if body is active, if so the entire vehicle should be active
			mIsActive |= w->mContactBody->IsActive();

			// Determine world space forward using steering angle and body rotation
			Vec3 forward, up, right;
			GetWheelLocalBasis(w, forward, up, right);
			forward = body_transform.Multiply3x3(forward);
			right = body_transform.Multiply3x3(right);

			// The longitudinal axis is in the up/forward plane
			w->mContactLongitudinal = w->mContactNormal.Cross(right);

			// Make sure that the longitudinal axis is aligned with the forward axis
			if (w->mContactLongitudinal.Dot(forward) < 0.0f)
				w->mContactLongitudinal = -w->mContactLongitudinal;

			// Normalize it
			w->mContactLongitudinal = w->mContactLongitudinal.NormalizedOr(w->mContactNormal.GetNormalizedPerpendicular());

			// The lateral axis is perpendicular to contact normal and longitudinal axis
			w->mContactLateral = w->mContactLongitudinal.Cross(w->mContactNormal).Normalized();
		}
	}

	// Callback to higher-level systems. We do it immediately after wheel collision.
	if (mPostCollideCallback != nullptr)
		mPostCollideCallback(*this, inContext);

	// Calculate anti-rollbar impulses
	for (const VehicleAntiRollBar &r : mAntiRollBars)
	{
		MOSS_ASSERT(r.mStiffness >= 0.0f);

		Wheel *lw = mWheels[r.mLeftWheel];
		Wheel *rw = mWheels[r.mRightWheel];

		if (lw->mContactBody != nullptr && rw->mContactBody != nullptr)
		{
			// Calculate the impulse to apply based on the difference in suspension length
			float difference = rw->mSuspensionLength - lw->mSuspensionLength;
			float impulse = difference * r.mStiffness * inContext.mDeltaTime;
			lw->mAntiRollBarImpulse = -impulse;
			rw->mAntiRollBarImpulse = impulse;
		}
		else
		{
			// When one of the wheels is not on the ground we don't apply any impulses
			lw->mAntiRollBarImpulse = rw->mAntiRollBarImpulse = 0.0f;
		}
	}

	// Callback on our controller
	mController->PostCollide(inContext.mDeltaTime, *inContext.mPhysicsSystem);

	// Callback to higher-level systems. We do it before the sleep section, in case velocities change.
	if (mPostStepCallback != nullptr)
		mPostStepCallback(*this, inContext);

	// If the wheels are rotating, we don't want to go to sleep yet
	if (mBody->GetAllowSleeping())
	{
		bool allow_sleep = mController->AllowSleep();
		if (allow_sleep)
			for (const Wheel *w : mWheels)
				if (abs(w->mAngularVelocity) > DegreesToRadians(10.0f))
				{
					allow_sleep = false;
					break;
				}
		if (!allow_sleep)
			mBody->ResetSleepTimer();
	}

	// Increment step counter
	++mCurrentStep;
}

void VehicleConstraint::BuildIslands(uint32 inConstraintIndex, IslandBuilder &ioBuilder, BodyManager &inBodyManager)
{
	// Find dynamic bodies that our wheels are touching
	BodyID *body_ids = (BodyID *)MOSS_STACK_ALLOC((mWheels.size() + 1) * sizeof(BodyID));
	int num_bodies = 0;
	bool needs_to_activate = false;
	for (const Wheel *w : mWheels)
		if (w->mContactBody != nullptr)
		{
			// Avoid adding duplicates
			bool duplicate = false;
			BodyID id = w->mContactBody->GetID();
			for (int i = 0; i < num_bodies; ++i)
				if (body_ids[i] == id)
				{
					duplicate = true;
					break;
				}
			if (duplicate)
				continue;

			if (w->mContactBody->IsDynamic())
			{
				body_ids[num_bodies++] = id;
				needs_to_activate |= !w->mContactBody->IsActive();
			}
		}

	// Activate bodies, note that if we get here we have already told the system that we're active so that means our main body needs to be active too
	if (!mBody->IsActive())
	{
		// Our main body is not active, activate it too
		body_ids[num_bodies] = mBody->GetID();
		inBodyManager.ActivateBodies(body_ids, num_bodies + 1);
	}
	else if (needs_to_activate)
	{
		// Only activate bodies the wheels are touching
		inBodyManager.ActivateBodies(body_ids, num_bodies);
	}

	// Link the bodies into the same island
	uint32 min_active_index = Body::cInactiveIndex;
	for (int i = 0; i < num_bodies; ++i)
	{
		const Body &body = inBodyManager.GetBody(body_ids[i]);
		min_active_index = min(min_active_index, body.GetIndexInActiveBodiesInternal());
		ioBuilder.LinkBodies(mBody->GetIndexInActiveBodiesInternal(), body.GetIndexInActiveBodiesInternal());
	}

	// Link the constraint in the island
	ioBuilder.LinkConstraint(inConstraintIndex, mBody->GetIndexInActiveBodiesInternal(), min_active_index);
}

uint VehicleConstraint::BuildIslandSplits(LargeIslandSplitter &ioSplitter) const
{
	return ioSplitter.AssignToNonParallelSplit(mBody);
}

void VehicleConstraint::CalculateSuspensionForcePoint(const Wheel &inWheel, Vec3 &outR1PlusU, Vec3 &outR2) const
{
	// Determine point to apply force to
	RVec3 force_point;
	if (inWheel.mSettings->mEnableSuspensionForcePoint)
		force_point = mBody->GetWorldTransform() * inWheel.mSettings->mSuspensionForcePoint;
	else
		force_point = inWheel.mContactPosition;

	// Calculate r1 + u and r2
	outR1PlusU = Vec3(force_point - mBody->GetCenterOfMassPosition());
	outR2 = Vec3(force_point - inWheel.mContactBody->GetCenterOfMassPosition());
}

void VehicleConstraint::CalculatePitchRollConstraintProperties(RMat44Arg inBodyTransform)
{
	// Check if a limit was specified
	if (mCosMaxPitchRollAngle > -1.0f)
	{
		// Calculate cos of angle between world up vector and vehicle up vector
		Vec3 vehicle_up = inBodyTransform.Multiply3x3(mUp);
		mCosPitchRollAngle = mWorldUp.Dot(vehicle_up);
		if (mCosPitchRollAngle < mCosMaxPitchRollAngle)
		{
			// Calculate rotation axis to rotate vehicle towards up
			Vec3 rotation_axis = mWorldUp.Cross(vehicle_up);
			float len = rotation_axis.Length();
			if (len > 0.0f)
				mPitchRollRotationAxis = rotation_axis / len;

			mPitchRollPart.CalculateConstraintProperties(*mBody, Body::sFixedToWorld, mPitchRollRotationAxis);
		}
		else
			mPitchRollPart.Deactivate();
	}
	else
		mPitchRollPart.Deactivate();
}

void VehicleConstraint::SetupVelocityConstraint(float inDeltaTime)
{
	RMat44 body_transform = mBody->GetWorldTransform();

	for (Wheel *w : mWheels)
		if (w->mContactBody != nullptr)
		{
			const WheelSettings *settings = w->mSettings;

			Vec3 neg_contact_normal = -w->mContactNormal;

			Vec3 r1_plus_u, r2;
			CalculateSuspensionForcePoint(*w, r1_plus_u, r2);

			// Suspension spring
			if (settings->mSuspensionMaxLength > settings->mSuspensionMinLength)
			{
				float stiffness, damping;
				if (settings->mSuspensionSpring.mMode == ESpringMode::FrequencyAndDamping)
				{
					// Calculate effective mass based on vehicle configuration (the stiffness of the spring should not be affected by the dynamics of the vehicle): K = 1 / (J M^-1 J^T)
					// Note that if no suspension force point is supplied we don't know where the force is applied so we assume it is applied at average suspension length
					Vec3 force_point = settings->mEnableSuspensionForcePoint? settings->mSuspensionForcePoint : settings->mPosition + 0.5f * (settings->mSuspensionMinLength + settings->mSuspensionMaxLength) * settings->mSuspensionDirection;
					Vec3 force_point_x_neg_up = force_point.Cross(-mUp);
					const MotionProperties *mp = mBody->GetMotionProperties();
					float effective_mass = 1.0f / (mp->GetInverseMass() + force_point_x_neg_up.Dot(mp->GetLocalSpaceInverseInertia().Multiply3x3(force_point_x_neg_up)));

					// Convert frequency and damping to stiffness and damping
					float omega = 2.0f * MOSS_PI * settings->mSuspensionSpring.mFrequency;
					stiffness = effective_mass * Square(omega);
					damping = 2.0f * effective_mass * settings->mSuspensionSpring.mDamping * omega;
				}
				else
				{
					// In this case we can simply copy the properties
					stiffness = settings->mSuspensionSpring.mStiffness;
					damping = settings->mSuspensionSpring.mDamping;
				}

				// Calculate the damping and frequency of the suspension spring given the angle between the suspension direction and the contact normal
				// If the angle between the suspension direction and the inverse of the contact normal is alpha then the force on the spring relates to the force along the contact normal as:
				//
				// Fspring = Fnormal * cos(alpha)
				//
				// The spring force is:
				//
				// Fspring = -k * x
				//
				// where k is the spring constant and x is the displacement of the spring. So we have:
				//
				// Fnormal * cos(alpha) = -k * x <=> Fnormal = -k / cos(alpha) * x
				//
				// So we can see this as a spring with spring constant:
				//
				// k' = k / cos(alpha)
				//
				// In the same way the velocity relates like:
				//
				// Vspring = Vnormal * cos(alpha)
				//
				// Which results in the modified damping constant c:
				//
				// c' = c / cos(alpha)
				//
				// Note that we clamp 1 / cos(alpha) to the range [0.1, 1] in order not to increase the stiffness / damping by too much.
				Vec3 ws_direction = body_transform.Multiply3x3(settings->mSuspensionDirection);
				float cos_angle = max(0.1f, ws_direction.Dot(neg_contact_normal));
				stiffness /= cos_angle;
				damping /= cos_angle;

				// Get the value of the constraint equation
				float c = w->mSuspensionLength - settings->mSuspensionMaxLength - settings->mSuspensionPreloadLength;

				w->mSuspensionPart.CalculateConstraintPropertiesWithStiffnessAndDamping(inDeltaTime, *mBody, r1_plus_u, *w->mContactBody, r2, neg_contact_normal, w->mAntiRollBarImpulse, c, stiffness, damping);
			}
			else
				w->mSuspensionPart.Deactivate();

			// Check if we reached the 'max up' position and if so add a hard velocity constraint that stops any further movement in the normal direction
			if (w->mSuspensionLength < settings->mSuspensionMinLength)
				w->mSuspensionMaxUpPart.CalculateConstraintProperties(*mBody, r1_plus_u, *w->mContactBody, r2, neg_contact_normal);
			else
				w->mSuspensionMaxUpPart.Deactivate();

			// Friction and propulsion
			w->mLongitudinalPart.CalculateConstraintProperties(*mBody, r1_plus_u, *w->mContactBody, r2, -w->mContactLongitudinal);
			w->mLateralPart.CalculateConstraintProperties(*mBody, r1_plus_u, *w->mContactBody, r2, -w->mContactLateral);
		}
		else
		{
			// No contact -> disable everything
			w->mSuspensionPart.Deactivate();
			w->mSuspensionMaxUpPart.Deactivate();
			w->mLongitudinalPart.Deactivate();
			w->mLateralPart.Deactivate();
		}

	CalculatePitchRollConstraintProperties(body_transform);
}

void VehicleConstraint::ResetWarmStart()
{
	for (Wheel *w : mWheels)
	{
		w->mSuspensionPart.Deactivate();
		w->mSuspensionMaxUpPart.Deactivate();
		w->mLongitudinalPart.Deactivate();
		w->mLateralPart.Deactivate();
	}

	mPitchRollPart.Deactivate();
}

void VehicleConstraint::WarmStartVelocityConstraint(float inWarmStartImpulseRatio)
{
	for (Wheel *w : mWheels)
		if (w->mContactBody != nullptr)
		{
			Vec3 neg_contact_normal = -w->mContactNormal;

			w->mSuspensionPart.WarmStart(*mBody, *w->mContactBody, neg_contact_normal, inWarmStartImpulseRatio);
			w->mSuspensionMaxUpPart.WarmStart(*mBody, *w->mContactBody, neg_contact_normal, inWarmStartImpulseRatio);
			w->mLongitudinalPart.WarmStart(*mBody, *w->mContactBody, -w->mContactLongitudinal, 0.0f); // Don't warm start the longitudinal part (the engine/brake force, we don't want to preserve anything from the last frame)
			w->mLateralPart.WarmStart(*mBody, *w->mContactBody, -w->mContactLateral, inWarmStartImpulseRatio);
		}

	mPitchRollPart.WarmStart(*mBody, Body::sFixedToWorld, inWarmStartImpulseRatio);
}

bool VehicleConstraint::SolveVelocityConstraint(float inDeltaTime)
{
	bool impulse = false;

	// Solve suspension
	for (Wheel *w : mWheels)
		if (w->mContactBody != nullptr)
		{
			Vec3 neg_contact_normal = -w->mContactNormal;

			// Suspension spring, note that it can only push and not pull
			if (w->mSuspensionPart.IsActive())
				impulse |= w->mSuspensionPart.SolveVelocityConstraint(*mBody, *w->mContactBody, neg_contact_normal, 0.0f, FLT_MAX);

			// When reaching the minimal suspension length only allow forces pushing the bodies away
			if (w->mSuspensionMaxUpPart.IsActive())
				impulse |= w->mSuspensionMaxUpPart.SolveVelocityConstraint(*mBody, *w->mContactBody, neg_contact_normal, 0.0f, FLT_MAX);
		}

	// Solve the horizontal movement of the vehicle
	impulse |= mController->SolveLongitudinalAndLateralConstraints(inDeltaTime);

	// Apply the pitch / roll constraint to avoid the vehicle from toppling over
	if (mPitchRollPart.IsActive())
		impulse |= mPitchRollPart.SolveVelocityConstraint(*mBody, Body::sFixedToWorld, mPitchRollRotationAxis, 0, FLT_MAX);

	return impulse;
}

bool VehicleConstraint::SolvePositionConstraint(float inDeltaTime, float inBaumgarte)
{
	bool impulse = false;

	RMat44 body_transform = mBody->GetWorldTransform();

	for (Wheel *w : mWheels)
		if (w->mContactBody != nullptr)
		{
			const WheelSettings *settings = w->mSettings;

			// Check if we reached the 'max up' position now that the body has possibly moved
			// We do this by calculating the axle position at minimum suspension length and making sure it does not go through the
			// plane defined by the contact normal and the axle position when the contact happened
			// TODO: This assumes that only the vehicle moved and not the ground as we kept the axle contact plane in world space
			Vec3 ws_direction = body_transform.Multiply3x3(settings->mSuspensionDirection);
			RVec3 ws_position = body_transform * settings->mPosition;
			RVec3 min_suspension_pos = ws_position + settings->mSuspensionMinLength * ws_direction;
			float max_up_error = float(RVec3(w->mContactNormal).Dot(min_suspension_pos) - w->mAxlePlaneConstant);
			if (max_up_error < 0.0f)
			{
				Vec3 neg_contact_normal = -w->mContactNormal;

				// Recalculate constraint properties since the body may have moved
				Vec3 r1_plus_u, r2;
				CalculateSuspensionForcePoint(*w, r1_plus_u, r2);
				w->mSuspensionMaxUpPart.CalculateConstraintProperties(*mBody, r1_plus_u, *w->mContactBody, r2, neg_contact_normal);

				impulse |= w->mSuspensionMaxUpPart.SolvePositionConstraint(*mBody, *w->mContactBody, neg_contact_normal, max_up_error, inBaumgarte);
			}
		}

	// Apply the pitch / roll constraint to avoid the vehicle from toppling over
	CalculatePitchRollConstraintProperties(body_transform);
	if (mPitchRollPart.IsActive())
		impulse |= mPitchRollPart.SolvePositionConstraint(*mBody, Body::sFixedToWorld, mCosPitchRollAngle - mCosMaxPitchRollAngle, inBaumgarte);

	return impulse;
}

#ifndef MOSS_DEBUG_RENDERER

void VehicleConstraint::DrawConstraint(DebugRenderer *inRenderer) const
{
	mController->Draw(inRenderer);
}

void VehicleConstraint::DrawConstraintLimits(DebugRenderer *inRenderer) const
{
}

#endif // MOSS_DEBUG_RENDERER

void VehicleConstraint::SaveState(StateRecorder &inStream) const
{
	Constraint::SaveState(inStream);

	mController->SaveState(inStream);

	for (const Wheel *w : mWheels)
	{
		inStream.Write(w->mAngularVelocity);
		inStream.Write(w->mAngle);
		inStream.Write(w->mContactBodyID); // Used by MotorcycleController::PreCollide
		inStream.Write(w->mContactPosition); // Used by VehicleCollisionTester::PredictContactProperties
		inStream.Write(w->mContactNormal); // Used by MotorcycleController::PreCollide
		inStream.Write(w->mContactLateral); // Used by MotorcycleController::PreCollide
		inStream.Write(w->mSuspensionLength); // Used by VehicleCollisionTester::PredictContactProperties

		w->mSuspensionPart.SaveState(inStream);
		w->mSuspensionMaxUpPart.SaveState(inStream);
		w->mLongitudinalPart.SaveState(inStream);
		w->mLateralPart.SaveState(inStream);
	}

	inStream.Write(mPitchRollRotationAxis); // When rotation is too small we use last frame so we need to store it
	mPitchRollPart.SaveState(inStream);
	inStream.Write(mCurrentStep);
}

void VehicleConstraint::RestoreState(StateRecorder &inStream)
{
	Constraint::RestoreState(inStream);

	mController->RestoreState(inStream);

	for (Wheel *w : mWheels)
	{
		inStream.Read(w->mAngularVelocity);
		inStream.Read(w->mAngle);
		inStream.Read(w->mContactBodyID);
		inStream.Read(w->mContactPosition);
		inStream.Read(w->mContactNormal);
		inStream.Read(w->mContactLateral);
		inStream.Read(w->mSuspensionLength);
		w->mContactBody = nullptr; // No longer valid

		w->mSuspensionPart.RestoreState(inStream);
		w->mSuspensionMaxUpPart.RestoreState(inStream);
		w->mLongitudinalPart.RestoreState(inStream);
		w->mLateralPart.RestoreState(inStream);
	}

	inStream.Read(mPitchRollRotationAxis);
	mPitchRollPart.RestoreState(inStream);
	inStream.Read(mCurrentStep);
}

Ref<ConstraintSettings> VehicleConstraint::GetConstraintSettings() const
{
	MOSS_ASSERT(false); // Not implemented yet
	return nullptr;
}

Wheel::Wheel(const WheelSettings &inSettings) :
	mSettings(&inSettings),
	mSuspensionLength(inSettings.mSuspensionMaxLength)
{
	MOSS_ASSERT(inSettings.mSuspensionDirection.IsNormalized());
	MOSS_ASSERT(inSettings.mSteeringAxis.IsNormalized());
	MOSS_ASSERT(inSettings.mWheelForward.IsNormalized());
	MOSS_ASSERT(inSettings.mWheelUp.IsNormalized());
	MOSS_ASSERT(inSettings.mSuspensionMinLength >= 0.0f);
	MOSS_ASSERT(inSettings.mSuspensionMaxLength >= inSettings.mSuspensionMinLength);
	MOSS_ASSERT(inSettings.mSuspensionPreloadLength >= 0.0f);
	MOSS_ASSERT(inSettings.mSuspensionSpring.mFrequency > 0.0f);
	MOSS_ASSERT(inSettings.mSuspensionSpring.mDamping >= 0.0f);
	MOSS_ASSERT(inSettings.mRadius > 0.0f);
	MOSS_ASSERT(inSettings.mWidth >= 0.0f);
}

bool Wheel::SolveLongitudinalConstraintPart(const VehicleConstraint &inConstraint, float inMinImpulse, float inMaxImpulse)
{
	return mLongitudinalPart.SolveVelocityConstraint(*inConstraint.GetVehicleBody(), *mContactBody, -mContactLongitudinal, inMinImpulse, inMaxImpulse);
}

bool Wheel::SolveLateralConstraintPart(const VehicleConstraint &inConstraint, float inMinImpulse, float inMaxImpulse)
{
	return mLateralPart.SolveVelocityConstraint(*inConstraint.GetVehicleBody(), *mContactBody, -mContactLateral, inMinImpulse, inMaxImpulse);
}


VehicleEngineSettings::VehicleEngineSettings()
{
	mNormalizedTorque.Reserve(3);
	mNormalizedTorque.AddPoint(0.0f, 0.8f);
	mNormalizedTorque.AddPoint(0.66f, 1.0f);
	mNormalizedTorque.AddPoint(1.0f, 0.8f);
}

void VehicleEngine::ApplyTorque(float inTorque, float inDeltaTime)
{
	// Accelerate engine using torque
	mCurrentRPM += cAngularVelocityToRPM * inTorque * inDeltaTime / mInertia;
	ClampRPM();
}

void VehicleEngine::ApplyDamping(float inDeltaTime)
{
	// Angular damping: dw/dt = -c * w
	// Solution: w(t) = w(0) * e^(-c * t) or w2 = w1 * e^(-c * dt)
	// Taylor expansion of e^(-c * dt) = 1 - c * dt + ...
	// Since dt is usually in the order of 1/60 and c is a low number too this approximation is good enough
	mCurrentRPM *= max(0.0f, 1.0f - mAngularDamping * inDeltaTime);
	ClampRPM();
}

#ifndef MOSS_DEBUG_RENDERER

void VehicleEngine::DrawRPM(DebugRenderer *inRenderer, RVec3Arg inPosition, Vec3Arg inForward, Vec3Arg inUp, float inSize, float inShiftDownRPM, float inShiftUpRPM) const
{
	// Function to draw part of a pie
	auto draw_pie = [this, inRenderer, inSize, inPosition, inForward, inUp](float inMinRPM, float inMaxRPM, Color inColor) {
		inRenderer->DrawPie(inPosition, inSize, inForward, inUp, ConvertRPMToAngle(inMinRPM), ConvertRPMToAngle(inMaxRPM), inColor, DebugRenderer::ECastShadow::Off);
	};

	// Draw segment under min RPM
	draw_pie(0, mMinRPM, Color::sGrey);

	// Draw segment until inShiftDownRPM
	if (mCurrentRPM < inShiftDownRPM)
	{
		draw_pie(mMinRPM, mCurrentRPM, Color::sRed);
		draw_pie(mCurrentRPM, inShiftDownRPM, Color::sDarkRed);
	}
	else
	{
		draw_pie(mMinRPM, inShiftDownRPM, Color::sRed);
	}

	// Draw segment between inShiftDownRPM and inShiftUpRPM
	if (mCurrentRPM > inShiftDownRPM && mCurrentRPM < inShiftUpRPM)
	{
		draw_pie(inShiftDownRPM, mCurrentRPM, Color::sOrange);
		draw_pie(mCurrentRPM, inShiftUpRPM, Color::sDarkOrange);
	}
	else
	{
		draw_pie(inShiftDownRPM, inShiftUpRPM, mCurrentRPM <= inShiftDownRPM? Color::sDarkOrange : Color::sOrange);
	}

	// Draw segment above inShiftUpRPM
	if (mCurrentRPM > inShiftUpRPM)
	{
		draw_pie(inShiftUpRPM, mCurrentRPM, Color::sGreen);
		draw_pie(mCurrentRPM, mMaxRPM, Color::sDarkGreen);
	}
	else
	{
		draw_pie(inShiftUpRPM, mMaxRPM, Color::sDarkGreen);
	}
}

#endif // MOSS_DEBUG_RENDERER

WheelSettingsWV::WheelSettingsWV()
{
	mLongitudinalFriction.Reserve(3);
	mLongitudinalFriction.AddPoint(0.0f, 0.0f);
	mLongitudinalFriction.AddPoint(0.06f, 1.2f);
	mLongitudinalFriction.AddPoint(0.2f, 1.0f);

	mLateralFriction.Reserve(3);
	mLateralFriction.AddPoint(0.0f, 0.0f);
	mLateralFriction.AddPoint(3.0f, 1.2f);
	mLateralFriction.AddPoint(20.0f, 1.0f);
}

WheelWV::WheelWV(const WheelSettingsWV &inSettings) :
	Wheel(inSettings)
{
	MOSS_ASSERT(inSettings.mInertia >= 0.0f);
	MOSS_ASSERT(inSettings.mAngularDamping >= 0.0f);
	MOSS_ASSERT(abs(inSettings.mMaxSteerAngle) <= 0.5f * MOSS_PI);
	MOSS_ASSERT(inSettings.mMaxBrakeTorque >= 0.0f);
	MOSS_ASSERT(inSettings.mMaxHandBrakeTorque >= 0.0f);
}

void WheelWV::Update(uint inWheelIndex, float inDeltaTime, const VehicleConstraint &inConstraint)
{
	const WheelSettingsWV *settings = GetSettings();

	// Angular damping: dw/dt = -c * w
	// Solution: w(t) = w(0) * e^(-c * t) or w2 = w1 * e^(-c * dt)
	// Taylor expansion of e^(-c * dt) = 1 - c * dt + ...
	// Since dt is usually in the order of 1/60 and c is a low number too this approximation is good enough
	mAngularVelocity *= max(0.0f, 1.0f - settings->mAngularDamping * inDeltaTime);

	// Update rotation of wheel
	mAngle = fmod(mAngle + mAngularVelocity * inDeltaTime, 2.0f * MOSS_PI);

	if (mContactBody != nullptr)
	{
		const Body *body = inConstraint.GetVehicleBody();

		// Calculate relative velocity between wheel contact point and floor
		Vec3 relative_velocity = body->GetPointVelocity(mContactPosition) - mContactPointVelocity;

		// Cancel relative velocity in the normal plane
		relative_velocity -= mContactNormal.Dot(relative_velocity) * mContactNormal;
		float relative_longitudinal_velocity = relative_velocity.Dot(mContactLongitudinal);

		// Calculate longitudinal friction based on difference between velocity of rolling wheel and drive surface
		float relative_longitudinal_velocity_denom = Sign(relative_longitudinal_velocity) * max(1.0e-3f, abs(relative_longitudinal_velocity)); // Ensure we don't divide by zero
		mLongitudinalSlip = abs((mAngularVelocity * settings->mRadius - relative_longitudinal_velocity) / relative_longitudinal_velocity_denom);
		float longitudinal_slip_friction = settings->mLongitudinalFriction.GetValue(mLongitudinalSlip);

		// Calculate lateral friction based on slip angle
		float relative_velocity_len = relative_velocity.Length();
		mLateralSlip = relative_velocity_len < 1.0e-3f ? 0.0f : ACos(abs(relative_longitudinal_velocity) / relative_velocity_len);
		float lateral_slip_angle = RadiansToDegrees(mLateralSlip);
		float lateral_slip_friction = settings->mLateralFriction.GetValue(lateral_slip_angle);

		// Tire friction
		VehicleConstraint::CombineFunction combine_friction = inConstraint.GetCombineFriction();
		mCombinedLongitudinalFriction = longitudinal_slip_friction;
		mCombinedLateralFriction = lateral_slip_friction;
		combine_friction(inWheelIndex, mCombinedLongitudinalFriction, mCombinedLateralFriction, *mContactBody, mContactSubShapeID);
	}
	else
	{
		// No collision
		mLongitudinalSlip = 0.0f;
		mLateralSlip = 0.0f;
		mCombinedLongitudinalFriction = mCombinedLateralFriction = 0.0f;
	}
}

VehicleController *WheeledVehicleControllerSettings::ConstructController(VehicleConstraint &inConstraint) const
{
	return new WheeledVehicleController(*this, inConstraint);
}

WheeledVehicleController::WheeledVehicleController(const WheeledVehicleControllerSettings &inSettings, VehicleConstraint &inConstraint) :
	VehicleController(inConstraint)
{
	// Copy engine settings
	static_cast<VehicleEngineSettings &>(mEngine) = inSettings.mEngine;
	MOSS_ASSERT(inSettings.mEngine.mMinRPM >= 0.0f);
	MOSS_ASSERT(inSettings.mEngine.mMinRPM <= inSettings.mEngine.mMaxRPM);
	mEngine.SetCurrentRPM(mEngine.mMinRPM);

	// Copy transmission settings
	static_cast<VehicleTransmissionSettings &>(mTransmission) = inSettings.mTransmission;
#ifndef MOSS_DEBUG
	for (float r : inSettings.mTransmission.mGearRatios)
		MOSS_ASSERT(r > 0.0f);
	for (float r : inSettings.mTransmission.mReverseGearRatios)
		MOSS_ASSERT(r < 0.0f);
#endif // MOSS_DEBUG
	MOSS_ASSERT(inSettings.mTransmission.mSwitchTime >= 0.0f);
	MOSS_ASSERT(inSettings.mTransmission.mShiftDownRPM > 0.0f);
	MOSS_ASSERT(inSettings.mTransmission.mMode != ETransmissionMode::Auto || inSettings.mTransmission.mShiftUpRPM < inSettings.mEngine.mMaxRPM);
	MOSS_ASSERT(inSettings.mTransmission.mShiftUpRPM > inSettings.mTransmission.mShiftDownRPM);
	MOSS_ASSERT(inSettings.mTransmission.mClutchStrength > 0.0f);

	// Copy differential settings
	mDifferentials.resize(inSettings.mDifferentials.size());
	for (uint i = 0; i < mDifferentials.size(); ++i)
	{
		const VehicleDifferentialSettings &d = inSettings.mDifferentials[i];
		mDifferentials[i] = d;
		MOSS_ASSERT(d.mDifferentialRatio > 0.0f);
		MOSS_ASSERT(d.mLeftRightSplit >= 0.0f && d.mLeftRightSplit <= 1.0f);
		MOSS_ASSERT(d.mEngineTorqueRatio >= 0.0f);
		MOSS_ASSERT(d.mLimitedSlipRatio > 1.0f);
	}

	mDifferentialLimitedSlipRatio = inSettings.mDifferentialLimitedSlipRatio;
	MOSS_ASSERT(mDifferentialLimitedSlipRatio > 1.0f);
}

float WheeledVehicleController::GetWheelSpeedAtClutch() const
{
	float wheel_speed_at_clutch = 0.0f;
	int num_driven_wheels = 0;
	for (const VehicleDifferentialSettings &d : mDifferentials)
	{
		int wheels[] = { d.mLeftWheel, d.mRightWheel };
		for (int w : wheels)
			if (w >= 0)
			{
				wheel_speed_at_clutch += mConstraint.GetWheel(w)->GetAngularVelocity() * d.mDifferentialRatio;
				num_driven_wheels++;
			}
	}
	return wheel_speed_at_clutch / float(num_driven_wheels) * VehicleEngine::cAngularVelocityToRPM * mTransmission.GetCurrentRatio();
}

bool WheeledVehicleController::AllowSleep() const
{
	return mForwardInput == 0.0f								// No user input
		&& mTransmission.AllowSleep()							// Transmission is not shifting
		&& mEngine.AllowSleep();								// Engine is idling
}

void WheeledVehicleController::PreCollide(float inDeltaTime, PhysicsSystem &inPhysicsSystem)
{
	MOSS_PROFILE_FUNCTION();

#ifndef MOSS_TRACE_VEHICLE_STATS
	static bool sTracedHeader = false;
	if (!sTracedHeader)
	{
		MOSS_TRACE("Time, ForwardInput, Gear, ClutchFriction, EngineRPM, WheelRPM, Velocity (km/h)");
		sTracedHeader = true;
	}
	static float sTime = 0.0f;
	sTime += inDeltaTime;
	MOSS_TRACE("%.3f, %.1f, %d, %.1f, %.1f, %.1f, %.1f", sTime, mForwardInput, mTransmission.GetCurrentGear(), mTransmission.GetClutchFriction(), mEngine.GetCurrentRPM(), GetWheelSpeedAtClutch(), mConstraint.GetVehicleBody()->GetLinearVelocity().Length() * 3.6f);
#endif // MOSS_TRACE_VEHICLE_STATS

	for (Wheel *w_base : mConstraint.GetWheels())
	{
		WheelWV *w = static_cast<WheelWV *>(w_base);

		// Set steering angle
		w->SetSteerAngle(-mRightInput * w->GetSettings()->mMaxSteerAngle);
	}
}

void WheeledVehicleController::PostCollide(float inDeltaTime, PhysicsSystem &inPhysicsSystem)
{
	MOSS_PROFILE_FUNCTION();

	// Remember old RPM so we can detect if we're increasing or decreasing
	float old_engine_rpm = mEngine.GetCurrentRPM();

	Wheels &wheels = mConstraint.GetWheels();

	// Update wheel angle, do this before applying torque to the wheels (as friction will slow them down again)
	for (uint wheel_index = 0, num_wheels = (uint)wheels.size(); wheel_index < num_wheels; ++wheel_index)
	{
		WheelWV *w = static_cast<WheelWV *>(wheels[wheel_index]);
		w->Update(wheel_index, inDeltaTime, mConstraint);
	}

	// In auto transmission mode, don't accelerate the engine when switching gears
	float forward_input = abs(mForwardInput);
	if (mTransmission.mMode == ETransmissionMode::Auto)
		forward_input *= mTransmission.GetClutchFriction();

	// Apply engine damping
	mEngine.ApplyDamping(inDeltaTime);

	// Calculate engine torque
	float engine_torque = mEngine.GetTorque(forward_input);

	// Define a struct that contains information about driven differentials (i.e. that have wheels connected)
	struct DrivenDifferential
	{
		const VehicleDifferentialSettings *	mDifferential;
		float								mAngularVelocity;
		float								mClutchToDifferentialTorqueRatio;
		float								mTempTorqueFactor;
	};

	// Collect driven differentials and their speeds
	TArray<DrivenDifferential> driven_differentials;
	driven_differentials.reserve(mDifferentials.size());
	float differential_omega_min = FLT_MAX, differential_omega_max = 0.0f;
	for (const VehicleDifferentialSettings &d : mDifferentials)
	{
		float avg_omega = 0.0f;
		int avg_omega_denom = 0;
		int indices[] = { d.mLeftWheel, d.mRightWheel };
		for (int idx : indices)
			if (idx != -1)
			{
				avg_omega += wheels[idx]->GetAngularVelocity();
				avg_omega_denom++;
			}

		if (avg_omega_denom > 0)
		{
			avg_omega = abs(avg_omega * d.mDifferentialRatio / float(avg_omega_denom)); // ignoring that the differentials may be rotating in different directions
			driven_differentials.push_back({ &d, avg_omega, d.mEngineTorqueRatio, 0 });

			// Remember min and max velocity
			differential_omega_min = min(differential_omega_min, avg_omega);
			differential_omega_max = max(differential_omega_max, avg_omega);
		}
	}

	if (mDifferentialLimitedSlipRatio < FLT_MAX					// Limited slip differential needs to be turned on
		&& differential_omega_max > differential_omega_min)		// There needs to be a velocity difference
	{
		// Calculate factor based on relative speed of a differential
		float sum_factor = 0.0f;
		for (DrivenDifferential &d : driven_differentials)
		{
			// Differential with max velocity gets factor 0, differential with min velocity 1
			d.mTempTorqueFactor = (differential_omega_max - d.mAngularVelocity) / (differential_omega_max - differential_omega_min);
			sum_factor += d.mTempTorqueFactor;
		}

		// Normalize the result
		for (DrivenDifferential &d : driven_differentials)
			d.mTempTorqueFactor /= sum_factor;

		// Prevent div by zero
		differential_omega_min = max(1.0e-3f, differential_omega_min);
		differential_omega_max = max(1.0e-3f, differential_omega_max);

		// Map into a value that is 0 when the wheels are turning at an equal rate and 1 when the wheels are turning at mDifferentialLimitedSlipRatio
		float alpha = min((differential_omega_max / differential_omega_min - 1.0f) / (mDifferentialLimitedSlipRatio - 1.0f), 1.0f);
		MOSS_ASSERT(alpha >= 0.0f);
		float one_min_alpha = 1.0f - alpha;

		// Update torque ratio for all differentials
		for (DrivenDifferential &d : driven_differentials)
			d.mClutchToDifferentialTorqueRatio = one_min_alpha * d.mClutchToDifferentialTorqueRatio + alpha * d.mTempTorqueFactor;
	}

#ifndef MOSS_ENAMOSS_DEBUGBLE_ASSERTS
	// Assert the values add up to 1
	float sum_torque_factors = 0.0f;
	for (DrivenDifferential &d : driven_differentials)
		sum_torque_factors += d.mClutchToDifferentialTorqueRatio;
	MOSS_ASSERT(abs(sum_torque_factors - 1.0f) < 1.0e-6f);
#endif // MOSS_DEBUG

	// Define a struct that collects information about the wheels that connect to the engine
	struct DrivenWheel
	{
		WheelWV *				mWheel;
		float					mClutchToWheelRatio;
		float					mClutchToWheelTorqueRatio;
		float					mEstimatedAngularImpulse;
	};
	TArray<DrivenWheel> driven_wheels;
	driven_wheels.reserve(wheels.size());

	// Collect driven wheels
	float transmission_ratio = mTransmission.GetCurrentRatio();
	for (const DrivenDifferential &dd : driven_differentials)
	{
		VehicleDifferentialSettings d = *dd.mDifferential;

		WheelWV *wl = d.mLeftWheel != -1? static_cast<WheelWV *>(wheels[d.mLeftWheel]) : nullptr;
		WheelWV *wr = d.mRightWheel != -1? static_cast<WheelWV *>(wheels[d.mRightWheel]) : nullptr;

		float clutch_to_wheel_ratio = transmission_ratio * d.mDifferentialRatio;

		if (wl != nullptr && wr != nullptr)
		{
			// Calculate torque ratio
			float ratio_l, ratio_r;
			d.CalculateTorqueRatio(wl->GetAngularVelocity(), wr->GetAngularVelocity(), ratio_l, ratio_r);

			// Add both wheels
			driven_wheels.push_back({ wl, clutch_to_wheel_ratio, dd.mClutchToDifferentialTorqueRatio * ratio_l, 0.0f });
			driven_wheels.push_back({ wr, clutch_to_wheel_ratio, dd.mClutchToDifferentialTorqueRatio * ratio_r, 0.0f });
		}
		else if (wl != nullptr)
		{
			// Only left wheel, all power to left
			driven_wheels.push_back({ wl, clutch_to_wheel_ratio, dd.mClutchToDifferentialTorqueRatio, 0.0f });
		}
		else if (wr != nullptr)
		{
			// Only right wheel, all power to right
			driven_wheels.push_back({ wr, clutch_to_wheel_ratio, dd.mClutchToDifferentialTorqueRatio, 0.0f });
		}
	}

	bool solved = false;
	if (!driven_wheels.empty())
	{
		// Define the torque at the clutch at time t as:
		//
		// tc(t):=S*(we(t)-sum(R(j)*ww(j,t),j,1,N)/N)
		//
		// Where:
		// S is the total strength of clutch (= friction * strength)
		// we(t) is the engine angular velocity at time t
		// R(j) is the total gear ratio of clutch to wheel for wheel j
		// ww(j,t) is the angular velocity of wheel j at time t
		// N is the amount of wheels
		//
		// The torque that increases the engine angular velocity at time t is:
		//
		// te(t):=TE-tc(t)
		//
		// Where:
		// TE is the torque delivered by the engine
		//
		// The torque that increases the wheel angular velocity for wheel i at time t is:
		//
		// tw(i,t):=TW(i)+R(i)*F(i)*tc(t)
		//
		// Where:
		// TW(i) is the torque applied to the wheel outside of the engine (brake + torque due to friction with the ground)
		// F(i) is the fraction of the engine torque applied from engine to wheel i
		//
		// Because the angular acceleration and torque are connected through: Torque = I * dw/dt
		//
		// We have the angular acceleration of the engine at time t:
		//
		// ddt_we(t):=te(t)/Ie
		//
		// Where:
		// Ie is the inertia of the engine
		//
		// We have the angular acceleration of wheel i at time t:
		//
		// ddt_ww(i,t):=tw(i,t)/Iw(i)
		//
		// Where:
		// Iw(i) is the inertia of wheel i
		//
		// We could take a simple Euler step to calculate the resulting accelerations but because the system is very stiff this turns out to be unstable, so we need to use implicit Euler instead:
		//
		// we(t+dt)=we(t)+dt*ddt_we(t+dt)
		//
		// and:
		//
		// ww(i,t+dt)=ww(i,t)+dt*ddt_ww(i,t+dt)
		//
		// Expanding both equations (the equations above are in wxMaxima format and this can easily be done by expand(%)):
		//
		// For wheel:
		//
		// ww(i,t+dt) + (S*dt*F(i)*R(i)*sum(R(j)*ww(j,t+dt),j,1,N))/(N*Iw(i)) - (S*dt*F(i)*R(i)*we(t+dt))/Iw(i) = ww(i,t)+(dt*TW(i))/Iw(i)
		//
		// For engine:
		//
		// we(t+dt) + (S*dt*we(t+dt))/Ie - (S*dt*sum(R(j)*ww(j,t+dt),j,1,N))/(Ie*N) = we(t)+(TE*dt)/Ie
		//
		// Defining a vector w(t) = (ww(1, t), ww(2, t), ..., ww(N, t), we(t)) we can write both equations as a matrix multiplication:
		//
		// a * w(t + dt) = b
		//
		// We then invert the matrix to get the new angular velocities.

		// Dimension of matrix is N + 1
		int n = (int)driven_wheels.size() + 1;

		// Last column of w is for the engine angular velocity
		int engine = n - 1;

		// Define a and b
		DynMatrix a(n, n);
		DynMatrix b(n, 1);

		// Get number of driven wheels as a float
		float num_driven_wheels_float = float(driven_wheels.size());

		// Angular velocity of engine
		float w_engine = mEngine.GetAngularVelocity();

		// Calculate the total strength of the clutch
		float clutch_strength = transmission_ratio != 0.0f? mTransmission.GetClutchFriction() * mTransmission.mClutchStrength : 0.0f;

		// dt / Ie
		float dt_div_ie = inDeltaTime / mEngine.mInertia;

		// Calculate scale factor for impulses based on previous delta time
		float impulse_scale = mPreviousDeltaTime > 0.0f? inDeltaTime / mPreviousDeltaTime : 0.0f;

		// Iterate the rows for the wheels
		for (int i = 0; i < (int)driven_wheels.size(); ++i)
		{
			DrivenWheel &w_i = driven_wheels[i];
			const WheelSettingsWV *settings = w_i.mWheel->GetSettings();

			// Get wheel inertia
			float inertia = settings->mInertia;

			// S * R(i)
			float s_r = clutch_strength * w_i.mClutchToWheelRatio;

			// dt * S * R(i) * F(i) / Iw
			float dt_s_r_f_div_iw = inDeltaTime * s_r * w_i.mClutchToWheelTorqueRatio / inertia;

			// Fill in the columns of a for wheel j
			for (int j = 0; j < (int)driven_wheels.size(); ++j)
			{
				const DrivenWheel &w_j = driven_wheels[j];
				a(i, j) = dt_s_r_f_div_iw * w_j.mClutchToWheelRatio / num_driven_wheels_float;
			}

			// Add ww(i, t+dt)
			a(i, i) += 1.0f;

			// Add the column for the engine
			a(i, engine) = -dt_s_r_f_div_iw;

			// Calculate external angular impulse operating on the wheel: TW(i) * dt
			float dt_tw = 0.0f;

			// Combine brake with hand brake torque
			float brake_torque = mBrakeInput * settings->mMaxBrakeTorque + mHandBrakeInput * settings->mMaxHandBrakeTorque;
			if (brake_torque > 0.0f)
			{
				// We're braking
				// Calculate brake angular impulse
				float sign;
				if (w_i.mWheel->GetAngularVelocity() != 0.0f)
					sign = Sign(w_i.mWheel->GetAngularVelocity());
				else
					sign = Sign(mTransmission.GetCurrentRatio()); // When wheels have locked up use the transmission ratio to determine the sign
				dt_tw = sign * inDeltaTime * brake_torque;
			}

			if (w_i.mWheel->HasContact())
			{
				// We have wheel contact with the floor
				// Note that we don't know the torque due to the ground contact yet, so we use the impulse applied from the last frame to estimate it
				// Wheel torque TW = force * radius = lambda / dt * radius
				dt_tw += impulse_scale * w_i.mWheel->GetLongitudinalLambda() * settings->mRadius;
			}

			w_i.mEstimatedAngularImpulse = dt_tw;

			// Fill in the constant b = ww(i,t)+(dt*TW(i))/Iw(i)
			b(i, 0) = w_i.mWheel->GetAngularVelocity() - dt_tw / inertia;

			// To avoid looping over the wheels again, we also fill in the wheel columns of the engine row here
			a(engine, i) = -dt_div_ie * s_r / num_driven_wheels_float;
		}

		// Finalize the engine row
		a(engine, engine) = (1.0f + dt_div_ie * clutch_strength);
		b(engine, 0) = w_engine + dt_div_ie * engine_torque;

		// Solve the linear equation
		if (GaussianElimination(a, b))
		{
			// Update the angular velocities for the wheels
			for (int i = 0; i < (int)driven_wheels.size(); ++i)
			{
				DrivenWheel &w_i = driven_wheels[i];
				const WheelSettingsWV *settings = w_i.mWheel->GetSettings();

				// Get solved wheel angular velocity
				float angular_velocity = b(i, 0);

				// We estimated TW and applied it in the equation above, but we haven't actually applied this torque yet so we undo it here.
				// It will be applied when we solve the actual braking / the constraints with the floor.
				angular_velocity += w_i.mEstimatedAngularImpulse / settings->mInertia;

				// Update angular velocity
				w_i.mWheel->SetAngularVelocity(angular_velocity);
			}

			// Update the engine RPM
			mEngine.SetCurrentRPM(b(engine, 0) * VehicleEngine::cAngularVelocityToRPM);

			// The speeds have been solved
			solved = true;
		}
		else
		{
			MOSS_ASSERT(false, "New engine/wheel speeds could not be calculated!");
		}
	}

	if (!solved)
	{
		// Engine not connected to wheels, apply all torque to engine rotation
		mEngine.ApplyTorque(engine_torque, inDeltaTime);
	}

	// Calculate if any of the wheels are slipping, this is used to prevent gear switching
	bool wheels_slipping = false;
	for (const DrivenWheel &w : driven_wheels)
		wheels_slipping |= w.mClutchToWheelTorqueRatio > 0.0f && (!w.mWheel->HasContact() || w.mWheel->mLongitudinalSlip > 0.1f);

	// Only allow shifting up when we're not slipping and we're increasing our RPM.
	// After a jump, we have a very high engine RPM but once we hit the ground the RPM should be decreasing and we don't want to shift up
	// during that time.
	bool can_shift_up = !wheels_slipping && mEngine.GetCurrentRPM() >= old_engine_rpm;

	// Update transmission
	mTransmission.Update(inDeltaTime, mEngine.GetCurrentRPM(), mForwardInput, can_shift_up);

	// Braking
	for (Wheel *w_base : wheels)
	{
		WheelWV *w = static_cast<WheelWV *>(w_base);
		const WheelSettingsWV *settings = w->GetSettings();

		// Combine brake with hand brake torque
		float brake_torque = mBrakeInput * settings->mMaxBrakeTorque + mHandBrakeInput * settings->mMaxHandBrakeTorque;
		if (brake_torque > 0.0f)
		{
			// Calculate how much torque is needed to stop the wheels from rotating in this time step
			float brake_torque_to_lock_wheels = abs(w->GetAngularVelocity()) * settings->mInertia / inDeltaTime;
			if (brake_torque > brake_torque_to_lock_wheels)
			{
				// Wheels are locked
				w->SetAngularVelocity(0.0f);
				w->mBrakeImpulse = (brake_torque - brake_torque_to_lock_wheels) * inDeltaTime / settings->mRadius;
			}
			else
			{
				// Slow down the wheels
				w->ApplyTorque(-Sign(w->GetAngularVelocity()) * brake_torque, inDeltaTime);
				w->mBrakeImpulse = 0.0f;
			}
		}
		else
		{
			// Not braking
			w->mBrakeImpulse = 0.0f;
		}
	}

	// Remember previous delta time so we can scale the impulses correctly
	mPreviousDeltaTime = inDeltaTime;
}

bool WheeledVehicleController::SolveLongitudinalAndLateralConstraints(float inDeltaTime)
{
	bool impulse = false;

	float *max_lateral_friction_impulse = (float *)MOSS_STACK_ALLOC(mConstraint.GetWheels().size() * sizeof(float));

	uint wheel_index = 0;
	for (Wheel *w_base : mConstraint.GetWheels())
	{
		if (w_base->HasContact())
		{
			WheelWV *w = static_cast<WheelWV *>(w_base);
			const WheelSettingsWV *settings = w->GetSettings();

			// Calculate max impulse that we can apply on the ground
			float max_longitudinal_friction_impulse;
			mTireMaxImpulseCallback(wheel_index,
				max_longitudinal_friction_impulse, max_lateral_friction_impulse[wheel_index], w->GetSuspensionLambda(),
				w->mCombinedLongitudinalFriction, w->mCombinedLateralFriction, w->mLongitudinalSlip, w->mLateralSlip, inDeltaTime);

			// Calculate relative velocity between wheel contact point and floor in longitudinal direction
			Vec3 relative_velocity = mConstraint.GetVehicleBody()->GetPointVelocity(w->GetContactPosition()) - w->GetContactPointVelocity();
			float relative_longitudinal_velocity = relative_velocity.Dot(w->GetContactLongitudinal());

			// Calculate brake force to apply
			float min_longitudinal_impulse, max_longitudinal_impulse;
			if (w->mBrakeImpulse != 0.0f)
			{
				// Limit brake force by max tire friction
				float brake_impulse = min(w->mBrakeImpulse, max_longitudinal_friction_impulse);

				// Check which direction the brakes should be applied (we don't want to apply an impulse that would accelerate the vehicle)
				if (relative_longitudinal_velocity >= 0.0f)
				{
					min_longitudinal_impulse = -brake_impulse;
					max_longitudinal_impulse = 0.0f;
				}
				else
				{
					min_longitudinal_impulse = 0.0f;
					max_longitudinal_impulse = brake_impulse;
				}

				// Longitudinal impulse, note that we assume that once the wheels are locked that the brakes have more than enough torque to keep the wheels locked so we exclude any rotation deltas
				impulse |= w->SolveLongitudinalConstraintPart(mConstraint, min_longitudinal_impulse, max_longitudinal_impulse);
			}
			else
			{
				// Assume we want to apply an angular impulse that makes the delta velocity between wheel and ground zero in one time step, calculate the amount of linear impulse needed to do that
				float desired_angular_velocity = relative_longitudinal_velocity / settings->mRadius;
				float linear_impulse = (w->GetAngularVelocity() - desired_angular_velocity) * settings->mInertia / settings->mRadius;

				// Limit the impulse by max tire friction
				float prev_lambda = w->GetLongitudinalLambda();
				min_longitudinal_impulse = max_longitudinal_impulse = Clamp(prev_lambda + linear_impulse, -max_longitudinal_friction_impulse, max_longitudinal_friction_impulse);

				// Longitudinal impulse
				impulse |= w->SolveLongitudinalConstraintPart(mConstraint, min_longitudinal_impulse, max_longitudinal_impulse);

				// Update the angular velocity of the wheels according to the lambda that was applied
				w->SetAngularVelocity(w->GetAngularVelocity() - (w->GetLongitudinalLambda() - prev_lambda) * settings->mRadius / settings->mInertia);
			}
		}
		++wheel_index;
	}

	wheel_index = 0;
	for (Wheel *w_base : mConstraint.GetWheels())
	{
		if (w_base->HasContact())
		{
			WheelWV *w = static_cast<WheelWV *>(w_base);

			// Lateral friction
			float max_lateral_impulse = max_lateral_friction_impulse[wheel_index];
			impulse |= w->SolveLateralConstraintPart(mConstraint, -max_lateral_impulse, max_lateral_impulse);
		}
		++wheel_index;
	}

	return impulse;
}

#ifndef MOSS_DEBUG_RENDERER

void WheeledVehicleController::Draw(DebugRenderer *inRenderer) const
{
	float constraint_size = mConstraint.GetDrawConstraintSize();

	// Draw RPM
	Body *body = mConstraint.GetVehicleBody();
	Vec3 rpm_meter_up = body->GetRotation() * mConstraint.GetLocalUp();
	RVec3 rpm_meter_pos = body->GetPosition() + body->GetRotation() * mRPMMeterPosition;
	Vec3 rpm_meter_fwd = body->GetRotation() * mConstraint.GetLocalForward();
	mEngine.DrawRPM(inRenderer, rpm_meter_pos, rpm_meter_fwd, rpm_meter_up, mRPMMeterSize, mTransmission.mShiftDownRPM, mTransmission.mShiftUpRPM);

	if (mTransmission.GetCurrentRatio() != 0.0f)
	{
		// Calculate average wheel speed at clutch
		float wheel_speed_at_clutch = GetWheelSpeedAtClutch();

		// Draw the average wheel speed measured at clutch to compare engine RPM with wheel RPM
		inRenderer->DrawLine(rpm_meter_pos, rpm_meter_pos + Quat::sRotation(rpm_meter_fwd, mEngine.ConvertRPMToAngle(wheel_speed_at_clutch)) * (rpm_meter_up * 1.1f * mRPMMeterSize), Color::sYellow);
	}

	// Draw current vehicle state
	String status = StringFormat("Forward: %.1f, Right: %.1f\nBrake: %.1f, HandBrake: %.1f\n"
								 "Gear: %d, Clutch: %.1f\nEngineRPM: %.0f, V: %.1f km/h",
								 (double)mForwardInput, (double)mRightInput, (double)mBrakeInput, (double)mHandBrakeInput,
								 mTransmission.GetCurrentGear(), (double)mTransmission.GetClutchFriction(), (double)mEngine.GetCurrentRPM(), (double)body->GetLinearVelocity().Length() * 3.6);
	inRenderer->DrawText3D(body->GetPosition(), status, Color::sWhite, constraint_size);

	RMat44 body_transform = body->GetWorldTransform();

	for (const Wheel *w_base : mConstraint.GetWheels())
	{
		const WheelWV *w = static_cast<const WheelWV *>(w_base);
		const WheelSettings *settings = w->GetSettings();

		// Calculate where the suspension attaches to the body in world space
		RVec3 ws_position = body_transform * settings->mPosition;
		Vec3 ws_direction = body_transform.Multiply3x3(settings->mSuspensionDirection);

		// Draw suspension
		RVec3 min_suspension_pos = ws_position + ws_direction * settings->mSuspensionMinLength;
		RVec3 max_suspension_pos = ws_position + ws_direction * settings->mSuspensionMaxLength;
		inRenderer->DrawLine(ws_position, min_suspension_pos, Color::sRed);
		inRenderer->DrawLine(min_suspension_pos, max_suspension_pos, Color::sGreen);

		// Draw current length
		RVec3 wheel_pos = ws_position + ws_direction * w->GetSuspensionLength();
		inRenderer->DrawMarker(wheel_pos, w->GetSuspensionLength() < settings->mSuspensionMinLength? Color::sRed : Color::sGreen, constraint_size);

		// Draw wheel basis
		Vec3 wheel_forward, wheel_up, wheel_right;
		mConstraint.GetWheelLocalBasis(w, wheel_forward, wheel_up, wheel_right);
		wheel_forward = body_transform.Multiply3x3(wheel_forward);
		wheel_up = body_transform.Multiply3x3(wheel_up);
		wheel_right = body_transform.Multiply3x3(wheel_right);
		Vec3 steering_axis = body_transform.Multiply3x3(settings->mSteeringAxis);
		inRenderer->DrawLine(wheel_pos, wheel_pos + wheel_forward, Color::sRed);
		inRenderer->DrawLine(wheel_pos, wheel_pos + wheel_up, Color::sGreen);
		inRenderer->DrawLine(wheel_pos, wheel_pos + wheel_right, Color::sBlue);
		inRenderer->DrawLine(wheel_pos, wheel_pos + steering_axis, Color::sYellow);

		// Draw wheel
		RMat44 wheel_transform(Vec4(wheel_up, 0.0f), Vec4(wheel_right, 0.0f), Vec4(wheel_forward, 0.0f), wheel_pos);
		wheel_transform.SetRotation(wheel_transform.GetRotation() * Mat44::sRotationY(-w->GetRotationAngle()));
		inRenderer->DrawCylinder(wheel_transform, settings->mWidth * 0.5f, settings->mRadius, w->GetSuspensionLength() <= settings->mSuspensionMinLength? Color::sRed : Color::sGreen, DebugRenderer::ECastShadow::Off, DebugRenderer::EDrawMode::Wireframe);

		if (w->HasContact())
		{
			// Draw contact
			inRenderer->DrawLine(w->GetContactPosition(), w->GetContactPosition() + w->GetContactNormal(), Color::sYellow);
			inRenderer->DrawLine(w->GetContactPosition(), w->GetContactPosition() + w->GetContactLongitudinal(), Color::sRed);
			inRenderer->DrawLine(w->GetContactPosition(), w->GetContactPosition() + w->GetContactLateral(), Color::sBlue);

			DebugRenderer::sInstance->DrawText3D(wheel_pos, StringFormat("W: %.1f, S: %.2f\nSlipLateral: %.1f, SlipLong: %.2f\nFrLateral: %.1f, FrLong: %.1f", (double)w->GetAngularVelocity(), (double)w->GetSuspensionLength(), (double)RadiansToDegrees(w->mLateralSlip), (double)w->mLongitudinalSlip, (double)w->mCombinedLateralFriction, (double)w->mCombinedLongitudinalFriction), Color::sWhite, constraint_size);
		}
		else
		{
			// Draw 'no hit'
			DebugRenderer::sInstance->DrawText3D(wheel_pos, StringFormat("W: %.1f", (double)w->GetAngularVelocity()), Color::sRed, constraint_size);
		}
	}
}

#endif // MOSS_DEBUG_RENDERER

void WheeledVehicleController::SaveState(StateRecorder &inStream) const
{
	inStream.Write(mForwardInput);
	inStream.Write(mRightInput);
	inStream.Write(mBrakeInput);
	inStream.Write(mHandBrakeInput);
	inStream.Write(mPreviousDeltaTime);

	mEngine.SaveState(inStream);
	mTransmission.SaveState(inStream);
}

void WheeledVehicleController::RestoreState(StateRecorder &inStream)
{
	inStream.Read(mForwardInput);
	inStream.Read(mRightInput);
	inStream.Read(mBrakeInput);
	inStream.Read(mHandBrakeInput);
	inStream.Read(mPreviousDeltaTime);

	mEngine.RestoreState(inStream);
	mTransmission.RestoreState(inStream);
}

void VehicleTrack::SaveState(StateRecorder &inStream) const
{
	inStream.Write(mAngularVelocity);
}

void VehicleTrack::RestoreState(StateRecorder &inStream)
{
	inStream.Read(mAngularVelocity);
}

VehicleController *MotorcycleControllerSettings::ConstructController(VehicleConstraint &inConstraint) const
{
	return new MotorcycleController(*this, inConstraint);
}

MotorcycleController::MotorcycleController(const MotorcycleControllerSettings &inSettings, VehicleConstraint &inConstraint) :
	WheeledVehicleController(inSettings, inConstraint),
	mMaxLeanAngle(inSettings.mMaxLeanAngle),
	mLeanSpringConstant(inSettings.mLeanSpringConstant),
	mLeanSpringDamping(inSettings.mLeanSpringDamping),
	mLeanSpringIntegrationCoefficient(inSettings.mLeanSpringIntegrationCoefficient),
	mLeanSpringIntegrationCoefficientDecay(inSettings.mLeanSpringIntegrationCoefficientDecay),
	mLeanSmoothingFactor(inSettings.mLeanSmoothingFactor)
{
}

float MotorcycleController::GetWheelBase() const
{
	float low = FLT_MAX, high = -FLT_MAX;

	for (const Wheel *w : mConstraint.GetWheels())
	{
		const WheelSettings *s = w->GetSettings();

		// Measure distance along the forward axis by looking at the fully extended suspension.
		// If the suspension force point is active, use that instead.
		Vec3 force_point = s->mEnableSuspensionForcePoint? s->mSuspensionForcePoint : s->mPosition + s->mSuspensionDirection * s->mSuspensionMaxLength;
		float value = force_point.Dot(mConstraint.GetLocalForward());

		// Update min and max
		low = min(low, value);
		high = max(high, value);
	}

	return high - low;
}

void MotorcycleController::PreCollide(float inDeltaTime, PhysicsSystem &inPhysicsSystem)
{
	WheeledVehicleController::PreCollide(inDeltaTime, inPhysicsSystem);

	const Body *body = mConstraint.GetVehicleBody();
	Vec3 forward = body->GetRotation() * mConstraint.GetLocalForward();
	float wheel_base = GetWheelBase();
	Vec3 world_up = mConstraint.GetWorldUp();

	if (mEnableLeanController)
	{
		// Calculate the target lean vector, this is in the direction of the total applied impulse by the ground on the wheels
		Vec3 target_lean = Vec3::sZero();
		for (const Wheel *w : mConstraint.GetWheels())
			if (w->HasContact())
				target_lean += w->GetContactNormal() * w->GetSuspensionLambda() + w->GetContactLateral() * w->GetLateralLambda();

		// Normalize the impulse
		target_lean = target_lean.NormalizedOr(world_up);

		// Smooth the impulse to avoid jittery behavior
		mTargetLean = mLeanSmoothingFactor * mTargetLean + (1.0f - mLeanSmoothingFactor) * target_lean;

		// Remove forward component, we can only lean sideways
		mTargetLean -= forward * mTargetLean.Dot(forward);
		mTargetLean = mTargetLean.NormalizedOr(world_up);

		// Clamp the target lean against the max lean angle
		Vec3 adjusted_world_up = world_up - forward * world_up.Dot(forward);
		adjusted_world_up = adjusted_world_up.NormalizedOr(world_up);
		float w_angle = -Sign(mTargetLean.Cross(adjusted_world_up).Dot(forward)) * ACos(mTargetLean.Dot(adjusted_world_up));
		if (abs(w_angle) > mMaxLeanAngle)
			mTargetLean = Quat::sRotation(forward, Sign(w_angle) * mMaxLeanAngle) * adjusted_world_up;

		// Integrate the delta angle
		Vec3 up = body->GetRotation() * mConstraint.GetLocalUp();
		float d_angle = -Sign(mTargetLean.Cross(up).Dot(forward)) * ACos(mTargetLean.Dot(up));
		mLeanSpringIntegratedDeltaAngle += d_angle * inDeltaTime;
	}
	else
	{
		// Controller not enabled, reset target lean
		mTargetLean = world_up;

		// Reset integrated delta angle
		mLeanSpringIntegratedDeltaAngle = 0;
	}

	MOSS_DET_LOG("WheeledVehicleController::PreCollide: mTargetLean: " << mTargetLean);

	// Calculate max steering angle based on the max lean angle we're willing to take
	// See: https://en.wikipedia.org/wiki/Bicycle_and_motorcycle_dynamics#Leaning
	// LeanAngle = Atan(Velocity^2 / (Gravity * TurnRadius))
	// And: https://en.wikipedia.org/wiki/Turning_radius (we're ignoring the tire width)
	// The CasterAngle is the added according to https://en.wikipedia.org/wiki/Bicycle_and_motorcycle_dynamics#Turning (this is the same formula but without small angle approximation)
	// TurnRadius = WheelBase / (Sin(SteerAngle) * Cos(CasterAngle))
	// => SteerAngle = ASin(WheelBase * Tan(LeanAngle) * Gravity / (Velocity^2 * Cos(CasterAngle))
	// The caster angle is different for each wheel so we can only calculate part of the equation here
	float max_steer_angle_factor = wheel_base * Tan(mMaxLeanAngle) * (mConstraint.IsGravityOverridden()? mConstraint.GetGravityOverride() : inPhysicsSystem.GetGravity()).Length();

	// Calculate forward velocity
	float velocity = body->GetLinearVelocity().Dot(forward);
	float velocity_sq = Square(velocity);

	// Decompose steering into sign and direction
	float steer_strength = abs(mRightInput);
	float steer_sign = -Sign(mRightInput);

	for (Wheel *w_base : mConstraint.GetWheels())
	{
		WheelWV *w = static_cast<WheelWV *>(w_base);
		const WheelSettingsWV *s = w->GetSettings();

		// Check if this wheel can steer
		if (s->mMaxSteerAngle != 0.0f)
		{
			// Calculate cos(caster angle), the angle between the steering axis and the up vector
			float cos_caster_angle = s->mSteeringAxis.Dot(mConstraint.GetLocalUp());

			// Calculate steer angle
			float steer_angle = steer_strength * w->GetSettings()->mMaxSteerAngle;

			// Clamp to max steering angle
			if (mEnableLeanSteeringLimit
				&& velocity_sq > 1.0e-6f && cos_caster_angle > 1.0e-6f)
			{
				float max_steer_angle = ASin(max_steer_angle_factor / (velocity_sq * cos_caster_angle));
				steer_angle = min(steer_angle, max_steer_angle);
			}

			// Set steering angle
			w->SetSteerAngle(steer_sign * steer_angle);
		}
	}

	// Reset applied impulse
	mAppliedImpulse = 0;
}

bool MotorcycleController::SolveLongitudinalAndLateralConstraints(float inDeltaTime)
{
	bool impulse = WheeledVehicleController::SolveLongitudinalAndLateralConstraints(inDeltaTime);

	if (mEnableLeanController)
	{
		// Only apply a lean impulse if all wheels are in contact, otherwise we can easily spin out
		bool all_in_contact = true;
		for (const Wheel *w : mConstraint.GetWheels())
			if (!w->HasContact() || w->GetSuspensionLambda() <= 0.0f)
			{
				all_in_contact = false;
				break;
			}

		if (all_in_contact)
		{
			Body *body = mConstraint.GetVehicleBody();
			const MotionProperties *mp = body->GetMotionProperties();

			Vec3 forward = body->GetRotation() * mConstraint.GetLocalForward();
			Vec3 up = body->GetRotation() * mConstraint.GetLocalUp();

			// Calculate delta to target angle and derivative
			float d_angle = -Sign(mTargetLean.Cross(up).Dot(forward)) * ACos(mTargetLean.Dot(up));
			float ddt_angle = body->GetAngularVelocity().Dot(forward);

			// Calculate impulse to apply to get to target lean angle
			float total_impulse = (mLeanSpringConstant * d_angle - mLeanSpringDamping * ddt_angle + mLeanSpringIntegrationCoefficient * mLeanSpringIntegratedDeltaAngle) * inDeltaTime;

			// Remember angular velocity pre angular impulse
			Vec3 old_w = mp->GetAngularVelocity();

			// Apply impulse taking into account the impulse we've applied earlier
			float delta_impulse = total_impulse - mAppliedImpulse;
			body->AddAngularImpulse(delta_impulse * forward);
			mAppliedImpulse = total_impulse;

			// Calculate delta angular velocity due to angular impulse
			Vec3 dw = mp->GetAngularVelocity() - old_w;
			Vec3 linear_acceleration = Vec3::sZero();
			float total_lambda = 0.0f;
			for (Wheel *w_base : mConstraint.GetWheels())
			{
				const WheelWV *w = static_cast<WheelWV *>(w_base);

				// We weigh the importance of each contact point according to the contact force
				float lambda = w->GetSuspensionLambda();
				total_lambda += lambda;

				// Linear acceleration of contact point is dw x com_to_contact
				Vec3 r = Vec3(w->GetContactPosition() - body->GetCenterOfMassPosition());
				linear_acceleration += lambda * dw.Cross(r);
			}

			// Apply linear impulse to COM to cancel the average velocity change on the wheels due to the angular impulse
			Vec3 linear_impulse = -linear_acceleration / (total_lambda * mp->GetInverseMass());
			body->AddImpulse(linear_impulse);

			// Return true if we applied an impulse
			impulse |= delta_impulse != 0.0f;
		}
		else
		{
			// Decay the integrated angle because we won't be applying a torque this frame
			// Uses 1st order Taylor approximation of e^(-decay * dt) = 1 - decay * dt
			mLeanSpringIntegratedDeltaAngle *= max(0.0f, 1.0f - mLeanSpringIntegrationCoefficientDecay * inDeltaTime);
		}
	}

	return impulse;
}

void MotorcycleController::SaveState(StateRecorder &inStream) const
{
	WheeledVehicleController::SaveState(inStream);

	inStream.Write(mTargetLean);
}

void MotorcycleController::RestoreState(StateRecorder &inStream)
{
	WheeledVehicleController::RestoreState(inStream);

	inStream.Read(mTargetLean);
}

#ifndef MOSS_DEBUG_RENDERER

void MotorcycleController::Draw(DebugRenderer *inRenderer) const
{
	WheeledVehicleController::Draw(inRenderer);

	// Draw current and desired lean angle
	Body *body = mConstraint.GetVehicleBody();
	RVec3 center_of_mass = body->GetCenterOfMassPosition();
	Vec3 up = body->GetRotation() * mConstraint.GetLocalUp();
	inRenderer->DrawArrow(center_of_mass, center_of_mass + up, Color::sYellow, 0.1f);
	inRenderer->DrawArrow(center_of_mass, center_of_mass + mTargetLean, Color::sRed, 0.1f);
}

#endif // MOSS_DEBUG_RENDERER

void VehicleTransmission::Update(float inDeltaTime, float inCurrentRPM, float inForwardInput, bool inCanShiftUp)
{
	// Update current gear and calculate clutch friction
	if (mMode == ETransmissionMode::Auto)
	{
		// Switch gears based on rpm
		int old_gear = mCurrentGear;
		if (mCurrentGear == 0 // In neutral
			|| inForwardInput * float(mCurrentGear) < 0.0f) // Changing between forward / reverse
		{
			// Switch to first gear or reverse depending on input
			mCurrentGear = inForwardInput > 0.0f? 1 : (inForwardInput < 0.0f? -1 : 0);
		}
		else if (mGearSwitchLatencyTimeLeft == 0.0f) // If not in the timout after switching gears
		{
			if (inCanShiftUp && inCurrentRPM > mShiftUpRPM)
			{
				if (mCurrentGear < 0)
				{
					// Shift up, reverse
					if (mCurrentGear > -(int)mReverseGearRatios.size())
						mCurrentGear--;
				}
				else
				{
					// Shift up, forward
					if (mCurrentGear < (int)mGearRatios.size())
						mCurrentGear++;
				}
			}
			else if (inCurrentRPM < mShiftDownRPM)
			{
				if (mCurrentGear < 0)
				{
					// Shift down, reverse
					int max_gear = inForwardInput != 0.0f? -1 : 0;
					if (mCurrentGear < max_gear)
						mCurrentGear++;
				}
				else
				{
					// Shift down, forward
					int min_gear = inForwardInput != 0.0f? 1 : 0;
					if (mCurrentGear > min_gear)
						mCurrentGear--;
				}
			}
		}

		if (old_gear != mCurrentGear)
		{
			// We've shifted gear, start switch countdown
			mGearSwitchTimeLeft = old_gear != 0? mSwitchTime : 0.0f;
			mClutchReleaseTimeLeft = mClutchReleaseTime;
			mGearSwitchLatencyTimeLeft = mSwitchLatency;
			mClutchFriction = 0.0f;
		}
		else if (mGearSwitchTimeLeft > 0.0f)
		{
			// If still switching gears, count down
			mGearSwitchTimeLeft = max(0.0f, mGearSwitchTimeLeft - inDeltaTime);
			mClutchFriction = 0.0f;
		}
		else if (mClutchReleaseTimeLeft > 0.0f)
		{
			// After switching the gears we slowly release the clutch
			mClutchReleaseTimeLeft = max(0.0f, mClutchReleaseTimeLeft - inDeltaTime);
			mClutchFriction = 1.0f - mClutchReleaseTimeLeft / mClutchReleaseTime;
		}
		else
		{
			// Clutch has full friction
			mClutchFriction = 1.0f;

			// Count down switch latency
			mGearSwitchLatencyTimeLeft = max(0.0f, mGearSwitchLatencyTimeLeft - inDeltaTime);
		}
	}
}

float VehicleTransmission::GetCurrentRatio() const
{
	if (mCurrentGear < 0)
		return mReverseGearRatios[-mCurrentGear - 1];
	else if (mCurrentGear == 0)
		return 0.0f;
	else
		return mGearRatios[mCurrentGear - 1];
}

void VehicleTransmission::SaveState(StateRecorder &inStream) const
{
	inStream.Write(mCurrentGear);
	inStream.Write(mClutchFriction);
	inStream.Write(mGearSwitchTimeLeft);
	inStream.Write(mClutchReleaseTimeLeft);
	inStream.Write(mGearSwitchLatencyTimeLeft);
}

void VehicleTransmission::RestoreState(StateRecorder &inStream)
{
	inStream.Read(mCurrentGear);
	inStream.Read(mClutchFriction);
	inStream.Read(mGearSwitchTimeLeft);
	inStream.Read(mClutchReleaseTimeLeft);
	inStream.Read(mGearSwitchLatencyTimeLeft);
}

bool VehicleCollisionTesterRay::Collide(PhysicsSystem &inPhysicsSystem, const VehicleConstraint &inVehicleConstraint, uint inWheelIndex, RVec3Arg inOrigin, Vec3Arg inDirection, const BodyID &inVehicleBodyID, Body *&outBody, SubShapeID &outSubShapeID, RVec3 &outContactPosition, Vec3 &outContactNormal, float &outSuspensionLength) const
{
	const DefaultBroadPhaseLayerFilter default_broadphase_layer_filter = inPhysicsSystem.GetDefaultBroadPhaseLayerFilter(mObjectLayer);
	const BroadPhaseLayerFilter &broadphase_layer_filter = mBroadPhaseLayerFilter != nullptr? *mBroadPhaseLayerFilter : default_broadphase_layer_filter;

	const DefaultObjectLayerFilter default_object_layer_filter = inPhysicsSystem.GetDefaultLayerFilter(mObjectLayer);
	const ObjectLayerFilter &object_layer_filter = mObjectLayerFilter != nullptr? *mObjectLayerFilter : default_object_layer_filter;

	const IgnoreSingleBodyFilter default_body_filter(inVehicleBodyID);
	const BodyFilter &body_filter = mBodyFilter != nullptr? *mBodyFilter : default_body_filter;

	const WheelSettings *wheel_settings = inVehicleConstraint.GetWheel(inWheelIndex)->GetSettings();
	float wheel_radius = wheel_settings->mRadius;
	float ray_length = wheel_settings->mSuspensionMaxLength + wheel_radius;
	RRayCast ray { inOrigin, ray_length * inDirection };

	class MyCollector : public CastRayCollector
	{
	public:
							MyCollector(PhysicsSystem &inPhysicsSystem, const RRayCast &inRay, Vec3Arg inUpDirection, float inCosMaxSlopeAngle) :
			mPhysicsSystem(inPhysicsSystem),
			mRay(inRay),
			mUpDirection(inUpDirection),
			mCosMaxSlopeAngle(inCosMaxSlopeAngle)
		{
		}

		virtual void		AddHit(const RayCastResult &inResult) override
		{
			// Test if this collision is closer than the previous one
			if (inResult.mFraction < GetEarlyOutFraction())
			{
				// Lock the body
				BodyLockRead lock(mPhysicsSystem.GetBodyLockInterfaceNoLock(), inResult.mBodyID);
				MOSS_ASSERT(lock.Succeeded()); // When this runs all bodies are locked so this should not fail
				const Body *body = &lock.GetBody();

				if (body->IsSensor())
					return;

				// Test that we're not hitting a vertical wall
				RVec3 contact_pos = mRay.GetPointOnRay(inResult.mFraction);
				Vec3 normal = body->GetWorldSpaceSurfaceNormal(inResult.mSubShapeID2, contact_pos);
				if (normal.Dot(mUpDirection) > mCosMaxSlopeAngle)
				{
					// Update early out fraction to this hit
					UpdateEarlyOutFraction(inResult.mFraction);

					// Get the contact properties
					mBody = body;
					mSubShapeID2 = inResult.mSubShapeID2;
					mContactPosition = contact_pos;
					mContactNormal = normal;
				}
			}
		}

		// Configuration
		PhysicsSystem &		mPhysicsSystem;
		RRayCast			mRay;
		Vec3				mUpDirection;
		float				mCosMaxSlopeAngle;

		// Resulting closest collision
		const Body *		mBody = nullptr;
		SubShapeID			mSubShapeID2;
		RVec3				mContactPosition;
		Vec3				mContactNormal;
	};

	RayCastSettings settings;

	MyCollector collector(inPhysicsSystem, ray, mUp, mCosMaxSlopeAngle);
	inPhysicsSystem.GetNarrowPhaseQueryNoLock().CastRay(ray, settings, collector, broadphase_layer_filter, object_layer_filter, body_filter);
	if (collector.mBody == nullptr)
		return false;

	outBody = const_cast<Body *>(collector.mBody);
	outSubShapeID = collector.mSubShapeID2;
	outContactPosition = collector.mContactPosition;
	outContactNormal = collector.mContactNormal;
	outSuspensionLength = max(0.0f, ray_length * collector.GetEarlyOutFraction() - wheel_radius);

	return true;
}

void VehicleCollisionTesterRay::PredictContactProperties(PhysicsSystem &inPhysicsSystem, const VehicleConstraint &inVehicleConstraint, uint inWheelIndex, RVec3Arg inOrigin, Vec3Arg inDirection, const BodyID &inVehicleBodyID, Body *&ioBody, SubShapeID &ioSubShapeID, RVec3 &ioContactPosition, Vec3 &ioContactNormal, float &ioSuspensionLength) const
{
	// Recalculate the contact points assuming the contact point is on an infinite plane
	const WheelSettings *wheel_settings = inVehicleConstraint.GetWheel(inWheelIndex)->GetSettings();
	float d_dot_n = inDirection.Dot(ioContactNormal);
	if (d_dot_n < -1.0e-6f)
	{
		// Reproject the contact position using the suspension ray and the plane formed by the contact position and normal
		ioContactPosition = inOrigin + Vec3(ioContactPosition - inOrigin).Dot(ioContactNormal) / d_dot_n * inDirection;

		// The suspension length is simply the distance between the contact position and the suspension origin excluding the wheel radius
		ioSuspensionLength = Clamp(Vec3(ioContactPosition - inOrigin).Dot(inDirection) - wheel_settings->mRadius, 0.0f, wheel_settings->mSuspensionMaxLength);
	}
	else
	{
		// If the normal is pointing away we assume there's no collision anymore
		ioSuspensionLength = wheel_settings->mSuspensionMaxLength;
	}
}

bool VehicleCollisionTesterCastSphere::Collide(PhysicsSystem &inPhysicsSystem, const VehicleConstraint &inVehicleConstraint, uint inWheelIndex, RVec3Arg inOrigin, Vec3Arg inDirection, const BodyID &inVehicleBodyID, Body *&outBody, SubShapeID &outSubShapeID, RVec3 &outContactPosition, Vec3 &outContactNormal, float &outSuspensionLength) const
{
	const DefaultBroadPhaseLayerFilter default_broadphase_layer_filter = inPhysicsSystem.GetDefaultBroadPhaseLayerFilter(mObjectLayer);
	const BroadPhaseLayerFilter &broadphase_layer_filter = mBroadPhaseLayerFilter != nullptr? *mBroadPhaseLayerFilter : default_broadphase_layer_filter;

	const DefaultObjectLayerFilter default_object_layer_filter = inPhysicsSystem.GetDefaultLayerFilter(mObjectLayer);
	const ObjectLayerFilter &object_layer_filter = mObjectLayerFilter != nullptr? *mObjectLayerFilter : default_object_layer_filter;

	const IgnoreSingleBodyFilter default_body_filter(inVehicleBodyID);
	const BodyFilter &body_filter = mBodyFilter != nullptr? *mBodyFilter : default_body_filter;

	SphereShape sphere(mRadius);
	sphere.SetEmbedded();

	const WheelSettings *wheel_settings = inVehicleConstraint.GetWheel(inWheelIndex)->GetSettings();
	float wheel_radius = wheel_settings->mRadius;
	float shape_cast_length = wheel_settings->mSuspensionMaxLength + wheel_radius - mRadius;
	RShapeCast shape_cast(&sphere, Vec3::sOne(), RMat44::sTranslation(inOrigin), inDirection * shape_cast_length);

	ShapeCastSettings settings;
	settings.mUseShrunkenShapeAndConvexRadius = true;
	settings.mReturnDeepestPoint = true;

	class MyCollector : public CastShapeCollector
	{
	public:
							MyCollector(PhysicsSystem &inPhysicsSystem, const RShapeCast &inShapeCast, Vec3Arg inUpDirection, float inCosMaxSlopeAngle) :
			mPhysicsSystem(inPhysicsSystem),
			mShapeCast(inShapeCast),
			mUpDirection(inUpDirection),
			mCosMaxSlopeAngle(inCosMaxSlopeAngle)
		{
		}

		virtual void		AddHit(const ShapeCastResult &inResult) override
		{
			// Test if this collision is closer/deeper than the previous one
			float early_out = inResult.GetEarlyOutFraction();
			if (early_out < GetEarlyOutFraction())
			{
				// Lock the body
				BodyLockRead lock(mPhysicsSystem.GetBodyLockInterfaceNoLock(), inResult.mBodyID2);
				MOSS_ASSERT(lock.Succeeded()); // When this runs all bodies are locked so this should not fail
				const Body *body = &lock.GetBody();

				if (body->IsSensor())
					return;

				// Test that we're not hitting a vertical wall
				Vec3 normal = -inResult.mPenetrationAxis.Normalized();
				if (normal.Dot(mUpDirection) > mCosMaxSlopeAngle)
				{
					// Update early out fraction to this hit
					UpdateEarlyOutFraction(early_out);

					// Get the contact properties
					mBody = body;
					mSubShapeID2 = inResult.mSubShapeID2;
					mContactPosition = mShapeCast.mCenterOfMassStart.GetTranslation() + inResult.mContactPointOn2;
					mContactNormal = normal;
					mFraction = inResult.mFraction;
				}
			}
		}

		// Configuration
		PhysicsSystem &		mPhysicsSystem;
		const RShapeCast &	mShapeCast;
		Vec3				mUpDirection;
		float				mCosMaxSlopeAngle;

		// Resulting closest collision
		const Body *		mBody = nullptr;
		SubShapeID			mSubShapeID2;
		RVec3				mContactPosition;
		Vec3				mContactNormal;
		float				mFraction;
	};

	MyCollector collector(inPhysicsSystem, shape_cast, mUp, mCosMaxSlopeAngle);
	inPhysicsSystem.GetNarrowPhaseQueryNoLock().CastShape(shape_cast, settings, shape_cast.mCenterOfMassStart.GetTranslation(), collector, broadphase_layer_filter, object_layer_filter, body_filter);
	if (collector.mBody == nullptr)
		return false;

	outBody = const_cast<Body *>(collector.mBody);
	outSubShapeID = collector.mSubShapeID2;
	outContactPosition = collector.mContactPosition;
	outContactNormal = collector.mContactNormal;
	outSuspensionLength = max(0.0f, shape_cast_length * collector.mFraction + mRadius - wheel_radius);

	return true;
}

void VehicleCollisionTesterCastSphere::PredictContactProperties(PhysicsSystem &inPhysicsSystem, const VehicleConstraint &inVehicleConstraint, uint inWheelIndex, RVec3Arg inOrigin, Vec3Arg inDirection, const BodyID &inVehicleBodyID, Body *&ioBody, SubShapeID &ioSubShapeID, RVec3 &ioContactPosition, Vec3 &ioContactNormal, float &ioSuspensionLength) const
{
	// Recalculate the contact points assuming the contact point is on an infinite plane
	const WheelSettings *wheel_settings = inVehicleConstraint.GetWheel(inWheelIndex)->GetSettings();
	float d_dot_n = inDirection.Dot(ioContactNormal);
	if (d_dot_n < -1.0e-6f)
	{
		// Reproject the contact position using the suspension cast sphere and the plane formed by the contact position and normal
		// This solves x = inOrigin + fraction * inDirection and (x - ioContactPosition) . ioContactNormal = mRadius for fraction
		float oc_dot_n = Vec3(ioContactPosition - inOrigin).Dot(ioContactNormal);
		float fraction = (mRadius + oc_dot_n) / d_dot_n;
		ioContactPosition = inOrigin + fraction * inDirection - mRadius * ioContactNormal;

		// Calculate the new suspension length in the same way as the cast sphere normally does
		ioSuspensionLength = Clamp(fraction + mRadius - wheel_settings->mRadius, 0.0f, wheel_settings->mSuspensionMaxLength);
	}
	else
	{
		// If the normal is pointing away we assume there's no collision anymore
		ioSuspensionLength = wheel_settings->mSuspensionMaxLength;
	}
}

bool VehicleCollisionTesterCastCylinder::Collide(PhysicsSystem &inPhysicsSystem, const VehicleConstraint &inVehicleConstraint, uint inWheelIndex, RVec3Arg inOrigin, Vec3Arg inDirection, const BodyID &inVehicleBodyID, Body *&outBody, SubShapeID &outSubShapeID, RVec3 &outContactPosition, Vec3 &outContactNormal, float &outSuspensionLength) const
{
	const DefaultBroadPhaseLayerFilter default_broadphase_layer_filter = inPhysicsSystem.GetDefaultBroadPhaseLayerFilter(mObjectLayer);
	const BroadPhaseLayerFilter &broadphase_layer_filter = mBroadPhaseLayerFilter != nullptr? *mBroadPhaseLayerFilter : default_broadphase_layer_filter;

	const DefaultObjectLayerFilter default_object_layer_filter = inPhysicsSystem.GetDefaultLayerFilter(mObjectLayer);
	const ObjectLayerFilter &object_layer_filter = mObjectLayerFilter != nullptr? *mObjectLayerFilter : default_object_layer_filter;

	const IgnoreSingleBodyFilter default_body_filter(inVehicleBodyID);
	const BodyFilter &body_filter = mBodyFilter != nullptr? *mBodyFilter : default_body_filter;

	const WheelSettings *wheel_settings = inVehicleConstraint.GetWheel(inWheelIndex)->GetSettings();
	float max_suspension_length = wheel_settings->mSuspensionMaxLength;

	// Get the wheel transform given that the cylinder rotates around the Y axis
	RMat44 shape_cast_start = inVehicleConstraint.GetWheelWorldTransform(inWheelIndex, Vec3::sAxisY(), Vec3::sAxisX());
	shape_cast_start.SetTranslation(inOrigin);

	// Construct a cylinder with the dimensions of the wheel
	float wheel_half_width = 0.5f * wheel_settings->mWidth;
	CylinderShape cylinder(wheel_half_width, wheel_settings->mRadius, min(wheel_half_width, wheel_settings->mRadius) * mConvexRadiusFraction);
	cylinder.SetEmbedded();

	RShapeCast shape_cast(&cylinder, Vec3::sOne(), shape_cast_start, inDirection * max_suspension_length);

	ShapeCastSettings settings;
	settings.mUseShrunkenShapeAndConvexRadius = true;
	settings.mReturnDeepestPoint = true;

	class MyCollector : public CastShapeCollector
	{
	public:
							MyCollector(PhysicsSystem &inPhysicsSystem, const RShapeCast &inShapeCast) :
			mPhysicsSystem(inPhysicsSystem),
			mShapeCast(inShapeCast)
		{
		}

		virtual void		AddHit(const ShapeCastResult &inResult) override
		{
			// Test if this collision is closer/deeper than the previous one
			float early_out = inResult.GetEarlyOutFraction();
			if (early_out < GetEarlyOutFraction())
			{
				// Lock the body
				BodyLockRead lock(mPhysicsSystem.GetBodyLockInterfaceNoLock(), inResult.mBodyID2);
				MOSS_ASSERT(lock.Succeeded()); // When this runs all bodies are locked so this should not fail
				const Body *body = &lock.GetBody();

				if (body->IsSensor())
					return;

				// Update early out fraction to this hit
				UpdateEarlyOutFraction(early_out);

				// Get the contact properties
				mBody = body;
				mSubShapeID2 = inResult.mSubShapeID2;
				mContactPosition = mShapeCast.mCenterOfMassStart.GetTranslation() + inResult.mContactPointOn2;
				mContactNormal = -inResult.mPenetrationAxis.Normalized();
				mFraction = inResult.mFraction;
			}
		}

		// Configuration
		PhysicsSystem &		mPhysicsSystem;
		const RShapeCast &	mShapeCast;

		// Resulting closest collision
		const Body *		mBody = nullptr;
		SubShapeID			mSubShapeID2;
		RVec3				mContactPosition;
		Vec3				mContactNormal;
		float				mFraction;
	};

	MyCollector collector(inPhysicsSystem, shape_cast);
	inPhysicsSystem.GetNarrowPhaseQueryNoLock().CastShape(shape_cast, settings, shape_cast.mCenterOfMassStart.GetTranslation(), collector, broadphase_layer_filter, object_layer_filter, body_filter);
	if (collector.mBody == nullptr)
		return false;

	outBody = const_cast<Body *>(collector.mBody);
	outSubShapeID = collector.mSubShapeID2;
	outContactPosition = collector.mContactPosition;
	outContactNormal = collector.mContactNormal;
	outSuspensionLength = max_suspension_length * collector.mFraction;

	return true;
}

void VehicleCollisionTesterCastCylinder::PredictContactProperties(PhysicsSystem &inPhysicsSystem, const VehicleConstraint &inVehicleConstraint, uint inWheelIndex, RVec3Arg inOrigin, Vec3Arg inDirection, const BodyID &inVehicleBodyID, Body *&ioBody, SubShapeID &ioSubShapeID, RVec3 &ioContactPosition, Vec3 &ioContactNormal, float &ioSuspensionLength) const
{
	// Recalculate the contact points assuming the contact point is on an infinite plane
	const WheelSettings *wheel_settings = inVehicleConstraint.GetWheel(inWheelIndex)->GetSettings();
	float d_dot_n = inDirection.Dot(ioContactNormal);
	if (d_dot_n < -1.0e-6f)
	{
		// Wheel size
		float half_width = 0.5f * wheel_settings->mWidth;
		float radius = wheel_settings->mRadius;

		// Get the inverse local space contact normal for a cylinder pointing along Y
		RMat44 wheel_transform = inVehicleConstraint.GetWheelWorldTransform(inWheelIndex, Vec3::sAxisY(), Vec3::sAxisX());
		Vec3 inverse_local_normal = -wheel_transform.Multiply3x3Transposed(ioContactNormal);

		// Get the support point of this normal in local space of the cylinder
		// See CylinderShape::Cylinder::GetSupport
		float x = inverse_local_normal.GetX(), y = inverse_local_normal.GetY(), z = inverse_local_normal.GetZ();
		float o = sqrt(Square(x) + Square(z));
		Vec3 support_point;
		if (o > 0.0f)
			support_point = Vec3((radius * x) / o, Sign(y) * half_width, (radius * z) / o);
		else
			support_point = Vec3(0, Sign(y) * half_width, 0);

		// Rotate back to world space
		support_point = wheel_transform.Multiply3x3(support_point);

		// Now we can use inOrigin + support_point as the start of a ray of our suspension to the contact plane
		// as know that it is the first point on the wheel that will hit the plane
		RVec3 origin = inOrigin + support_point;

		// Calculate contact position and suspension length, the is the same as VehicleCollisionTesterRay
		// but we don't need to take the radius into account anymore
		Vec3 oc(ioContactPosition - origin);
		ioContactPosition = origin + oc.Dot(ioContactNormal) / d_dot_n * inDirection;
		ioSuspensionLength = Clamp(oc.Dot(inDirection), 0.0f, wheel_settings->mSuspensionMaxLength);
	}
	else
	{
		// If the normal is pointing away we assume there's no collision anymore
		ioSuspensionLength = wheel_settings->mSuspensionMaxLength;
	}
}

MOSS_SUPRESS_WARNINGS_END
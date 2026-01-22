#ifndef MOSS_PHYSICS_CONTRAINTS_H
#define MOSS_PHYSICS_CONTRAINTS_H

class Body;


enum class EPathRotationConstraintType {
	Free,							///< Do not constrain the rotation of the body at all
	ConstrainAroundTangent,			///< Only allow rotation around the tangent vector (following the path)
	ConstrainAroundNormal,			///< Only allow rotation around the normal vector (perpendicular to the path)
	ConstrainAroundBinormal,		///< Only allow rotation around the binormal vector (perpendicular to the path)
	ConstrainToPath,				///< Fully constrain the rotation of body 2 to the path (following the tangent and normal of the path)
	FullyConstrained,				///< Fully constrain the rotation of the body 2 to the rotation of body 1
};

class AngleConstraintPart {
	/// Internal helper function to update velocities of bodies after Lagrange multiplier is calculated
	MOSS_INLINE bool ApplyVelocityStep(Body& ioBody1, Body& ioBody2, float inLambda) const {
		// Apply impulse if delta is not zero
		if (inLambda != 0.0f) {
			// Calculate velocity change due to constraint
			//
			// Impulse:
			// P = J^T lambda
			//
			// Euler velocity integration:
			// v' = v + M^-1 P
			if (ioBody1.IsDynamic())
				ioBody1.GetMotionProperties()->SubAngularVelocityStep(inLambda* mInvI1_Axis);
			if (ioBody2.IsDynamic())
				ioBody2.GetMotionProperties()->AddAngularVelocityStep(inLambda* mInvI2_Axis);
			return true;
		}

		return false;
	}

	/// Internal helper function to calculate the inverse effective mass
	MOSS_INLINE float CalculateInverseEffectiveMass(const Body& inBody1, const Body& inBody2, Vec3Arg inWorldSpaceAxis) {
		MOSS_ASSERT(inWorldSpaceAxis.IsNormalized(1.0e-4f));

		// Calculate properties used below
		mInvI1_Axis = inBody1.IsDynamic()? inBody1.GetMotionProperties()->MultiplyWorldSpaceInverseInertiaByVector(inBody1.GetRotation(), inWorldSpaceAxis) : Vec3::sZero();
		mInvI2_Axis = inBody2.IsDynamic()? inBody2.GetMotionProperties()->MultiplyWorldSpaceInverseInertiaByVector(inBody2.GetRotation(), inWorldSpaceAxis) : Vec3::sZero();

		// Calculate inverse effective mass: K = J M^-1 J^T
		return inWorldSpaceAxis.Dot(mInvI1_Axis + mInvI2_Axis);
	}

public:
	/// Calculate properties used during the functions below
	/// @param inBody1 The first body that this constraint is attached to
	/// @param inBody2 The second body that this constraint is attached to
	/// @param inWorldSpaceAxis The axis of rotation along which the constraint acts (normalized)
	/// Set the following terms to zero if you don't want to drive the constraint to zero with a spring:
	/// @param inBias Bias term (b) for the constraint impulse: lambda = J v + b
	inline void	CalculateConstraintProperties(const Body& inBody1, const Body& inBody2, Vec3Arg inWorldSpaceAxis, float inBias = 0.0f) {
		float inv_effective_mass = CalculateInverseEffectiveMass(inBody1, inBody2, inWorldSpaceAxis);

		if (inv_effective_mass == 0.0f)
			Deactivate();
		else {
			mEffectiveMass = 1.0f / inv_effective_mass;
			mSpringPart.CalculateSpringPropertiesWithBias(inBias);
		}
	}

	/// Calculate properties used during the functions below
	/// @param inDeltaTime Time step
	/// @param inBody1 The first body that this constraint is attached to
	/// @param inBody2 The second body that this constraint is attached to
	/// @param inWorldSpaceAxis The axis of rotation along which the constraint acts (normalized)
	/// Set the following terms to zero if you don't want to drive the constraint to zero with a spring:
	/// @param inBias Bias term (b) for the constraint impulse: lambda = J v + b
	///	@param inC Value of the constraint equation (C)
	///	@param inFrequency Oscillation frequency (Hz)
	///	@param inDamping Damping factor (0 = no damping, 1 = critical damping)
	inline void	CalculateConstraintPropertiesWithFrequencyAndDamping(float inDeltaTime, const Body& inBody1, const Body& inBody2, Vec3Arg inWorldSpaceAxis, float inBias, float inC, float inFrequency, float inDamping) {
		float inv_effective_mass = CalculateInverseEffectiveMass(inBody1, inBody2, inWorldSpaceAxis);

		if (inv_effective_mass == 0.0f)
			Deactivate();
		else
			mSpringPart.CalculateSpringPropertiesWithFrequencyAndDamping(inDeltaTime, inv_effective_mass, inBias, inC, inFrequency, inDamping, mEffectiveMass);
	}

	/// Calculate properties used during the functions below
	/// @param inDeltaTime Time step
	/// @param inBody1 The first body that this constraint is attached to
	/// @param inBody2 The second body that this constraint is attached to
	/// @param inWorldSpaceAxis The axis of rotation along which the constraint acts (normalized)
	/// Set the following terms to zero if you don't want to drive the constraint to zero with a spring:
	/// @param inBias Bias term (b) for the constraint impulse: lambda = J v + b
	///	@param inC Value of the constraint equation (C)
	///	@param inStiffness Spring stiffness k.
	///	@param inDamping Spring damping coefficient c.
	inline void CalculateConstraintPropertiesWithStiffnessAndDamping(float inDeltaTime, const Body& inBody1, const Body& inBody2, Vec3Arg inWorldSpaceAxis, float inBias, float inC, float inStiffness, float inDamping) {
		float inv_effective_mass = CalculateInverseEffectiveMass(inBody1, inBody2, inWorldSpaceAxis);

		if (inv_effective_mass == 0.0f)
			Deactivate();
		else
			mSpringPart.CalculateSpringPropertiesWithStiffnessAndDamping(inDeltaTime, inv_effective_mass, inBias, inC, inStiffness, inDamping, mEffectiveMass);
	}

	/// Selects one of the above functions based on the spring settings
	inline void CalculateConstraintPropertiesWithSettings(float inDeltaTime, const Body& inBody1, const Body& inBody2, Vec3Arg inWorldSpaceAxis, float inBias, float inC, const SpringSettings& inSpringSettings) {
		float inv_effective_mass = CalculateInverseEffectiveMass(inBody1, inBody2, inWorldSpaceAxis);

		if (inv_effective_mass == 0.0f)
			Deactivate();
		else if (inSpringSettings.mMode == ESpringMode::FrequencyAndDamping)
			mSpringPart.CalculateSpringPropertiesWithFrequencyAndDamping(inDeltaTime, inv_effective_mass, inBias, inC, inSpringSettings.mFrequency, inSpringSettings.mDamping, mEffectiveMass);
		else
			mSpringPart.CalculateSpringPropertiesWithStiffnessAndDamping(inDeltaTime, inv_effective_mass, inBias, inC, inSpringSettings.mStiffness, inSpringSettings.mDamping, mEffectiveMass);
	}

	/// Deactivate this constraint
	inline void Deactivate() {
		mEffectiveMass = 0.0f;
		mTotalLambda = 0.0f;
	}

	/// Check if constraint is active
	inline bool IsActive() const {
		return mEffectiveMass != 0.0f;
	}

	/// Must be called from the WarmStartVelocityConstraint call to apply the previous frame's impulses
	/// @param ioBody1 The first body that this constraint is attached to
	/// @param ioBody2 The second body that this constraint is attached to
	/// @param inWarmStartImpulseRatio Ratio of new step to old time step (dt_new / dt_old) for scaling the lagrange multiplier of the previous frame
	inline void WarmStart(Body& ioBody1, Body& ioBody2, float inWarmStartImpulseRatio) {
		mTotalLambda*= inWarmStartImpulseRatio;
		ApplyVelocityStep(ioBody1, ioBody2, mTotalLambda);
	}

	/// Iteratively update the velocity constraint. Makes sure d/dt C(...) = 0, where C is the constraint equation.
	/// @param ioBody1 The first body that this constraint is attached to
	/// @param ioBody2 The second body that this constraint is attached to
	/// @param inWorldSpaceAxis The axis of rotation along which the constraint acts (normalized)
	/// @param inMinLambda Minimum angular impulse to apply (N m s)
	/// @param inMaxLambda Maximum angular impulse to apply (N m s)
	inline bool	SolveVelocityConstraint(Body& ioBody1, Body& ioBody2, Vec3Arg inWorldSpaceAxis, float inMinLambda, float inMaxLambda) {
		// Lagrange multiplier is:
		//
		// lambda = -K^-1 (J v + b)
		float lambda = mEffectiveMass* (inWorldSpaceAxis.Dot(ioBody1.GetAngularVelocity() - ioBody2.GetAngularVelocity()) - mSpringPart.GetBias(mTotalLambda));
		float new_lambda = Clamp(mTotalLambda + lambda, inMinLambda, inMaxLambda); // Clamp impulse
		lambda = new_lambda - mTotalLambda; // Lambda potentially got clamped, calculate the new impulse to apply
		mTotalLambda = new_lambda; // Store accumulated impulse

		return ApplyVelocityStep(ioBody1, ioBody2, lambda);
	}

	/// Return lagrange multiplier
	float GetTotalLambda() const {
		return mTotalLambda;
	}

	/// Iteratively update the position constraint. Makes sure C(...) == 0.
	/// @param ioBody1 The first body that this constraint is attached to
	/// @param ioBody2 The second body that this constraint is attached to
	/// @param inC Value of the constraint equation (C)
	/// @param inBaumgarte Baumgarte constant (fraction of the error to correct)
	inline bool SolvePositionConstraint(Body& ioBody1, Body& ioBody2, float inC, float inBaumgarte) const {
		// Only apply position constraint when the constraint is hard, otherwise the velocity bias will fix the constraint
		if (inC != 0.0f& & !mSpringPart.IsActive()) {
			// Calculate lagrange multiplier (lambda) for Baumgarte stabilization:
			//
			// lambda = -K^-1* beta / dt* C
			//
			// We should divide by inDeltaTime, but we should multiply by inDeltaTime in the Euler step below so they're cancelled out
			float lambda = -mEffectiveMass* inBaumgarte* inC;

			// Directly integrate velocity change for one time step
			//
			// Euler velocity integration:
			// dv = M^-1 P
			//
			// Impulse:
			// P = J^T lambda
			//
			// Euler position integration:
			// x' = x + dv* dt
			//
			// Note we don't accumulate velocities for the stabilization. This is using the approach described in 'Modeling and
			// Solving Constraints' by Erin Catto presented at GDC 2007. On slide 78 it is suggested to split up the Baumgarte
			// stabilization for positional drift so that it does not actually add to the momentum. We combine an Euler velocity
			// integrate + a position integrate and then discard the velocity change.
			if (ioBody1.IsDynamic())
				ioBody1.SubRotationStep(lambda* mInvI1_Axis);
			if (ioBody2.IsDynamic())
				ioBody2.AddRotationStep(lambda* mInvI2_Axis);
			return true;
		}

		return false;
	}

	/// Save state of this constraint part
	void SaveState(StateRecorder& inStream) const {
		inStream.Write(mTotalLambda);
	}

	/// Restore state of this constraint part
	void RestoreState(StateRecorder& inStream) {
		inStream.Read(mTotalLambda);
	}

private:
	Vec3						mInvI1_Axis;
	Vec3						mInvI2_Axis;
	float						mEffectiveMass = 0.0f;
	SpringPart					mSpringPart;
	float						mTotalLambda = 0.0f;
};

class DualAxisConstraintPart {
public:
	using Vec2 = TVec<2>;
	using Mat22 = Matrix<2, 2>;
private:
	/// Internal helper function to update velocities of bodies after Lagrange multiplier is calculated
	MOSS_INLINE bool ApplyVelocityStep(Body& ioBody1, Body& ioBody2, Vec3Arg inN1, Vec3Arg inN2, const Vec2& inLambda) const {
		// Apply impulse if delta is not zero
		if (!inLambda.IsZero()) {
			// Calculate velocity change due to constraint
			//
			// Impulse:
			// P = J^T lambda
			//
			// Euler velocity integration:
			// v' = v + M^-1 P
			Vec3 impulse = inN1* inLambda[0] + inN2* inLambda[1];
			if (ioBody1.IsDynamic()) {
				MotionProperties*mp1 = ioBody1.GetMotionProperties();
				mp1->SubLinearVelocityStep(mp1->GetInverseMass()* impulse);
				mp1->SubAngularVelocityStep(mInvI1_R1PlusUxN1* inLambda[0] + mInvI1_R1PlusUxN2* inLambda[1]);
			}
			if (ioBody2.IsDynamic()) {
				MotionProperties*mp2 = ioBody2.GetMotionProperties();
				mp2->AddLinearVelocityStep(mp2->GetInverseMass()* impulse);
				mp2->AddAngularVelocityStep(mInvI2_R2xN1* inLambda[0] + mInvI2_R2xN2* inLambda[1]);
			}
			return true;
		}

		return false;
	}

	/// Internal helper function to calculate the lagrange multiplier
	inline void	CalculateLagrangeMultiplier(const Body& inBody1, const Body& inBody2, Vec3Arg inN1, Vec3Arg inN2, Vec2& outLambda) const {
		// Calculate lagrange multiplier:
		//
		// lambda = -K^-1 (J v + b)
		Vec3 delta_lin = inBody1.GetLinearVelocity() - inBody2.GetLinearVelocity();
		Vec2 jv;
		jv[0] = inN1.Dot(delta_lin) + mR1PlusUxN1.Dot(inBody1.GetAngularVelocity()) - mR2xN1.Dot(inBody2.GetAngularVelocity());
		jv[1] = inN2.Dot(delta_lin) + mR1PlusUxN2.Dot(inBody1.GetAngularVelocity()) - mR2xN2.Dot(inBody2.GetAngularVelocity());
		outLambda = mEffectiveMass* jv;
	}

public:
	/// Calculate properties used during the functions below
	/// All input vectors are in world space
	inline void CalculateConstraintProperties(const Body& inBody1, Mat44Arg inRotation1, Vec3Arg inR1PlusU, const Body& inBody2, Mat44Arg inRotation2, Vec3Arg inR2, Vec3Arg inN1, Vec3Arg inN2) {
		MOSS_ASSERT(inN1.IsNormalized(1.0e-5f));
		MOSS_ASSERT(inN2.IsNormalized(1.0e-5f));

		// Calculate properties used during constraint solving
		mR1PlusUxN1 = inR1PlusU.Cross(inN1);
		mR1PlusUxN2 = inR1PlusU.Cross(inN2);
		mR2xN1 = inR2.Cross(inN1);
		mR2xN2 = inR2.Cross(inN2);

		// Calculate effective mass: K^-1 = (J M^-1 J^T)^-1, eq 59
		Mat22 inv_effective_mass;
		if (inBody1.IsDynamic()) {
			const MotionProperties*mp1 = inBody1.GetMotionProperties();
			Mat44 inv_i1 = mp1->GetInverseInertiaForRotation(inRotation1);
			mInvI1_R1PlusUxN1 = inv_i1.Multiply3x3(mR1PlusUxN1);
			mInvI1_R1PlusUxN2 = inv_i1.Multiply3x3(mR1PlusUxN2);

			inv_effective_mass(0, 0) = mp1->GetInverseMass() + mR1PlusUxN1.Dot(mInvI1_R1PlusUxN1);
			inv_effective_mass(0, 1) = mR1PlusUxN1.Dot(mInvI1_R1PlusUxN2);
			inv_effective_mass(1, 0) = mR1PlusUxN2.Dot(mInvI1_R1PlusUxN1);
			inv_effective_mass(1, 1) = mp1->GetInverseMass() + mR1PlusUxN2.Dot(mInvI1_R1PlusUxN2);
		}
		else {
			mInvI1_R1PlusUxN1 = Vec3::sNaN();
			mInvI1_R1PlusUxN2 = Vec3::sNaN();

			MOSS_ERROR("mInvI1_R1PlusUxN1 and mInvI1_R1PlusUxN2 set to NaN.");

			inv_effective_mass = Mat22::sZero();
		}

		if (inBody2.IsDynamic()) {
			const MotionProperties*mp2 = inBody2.GetMotionProperties();
			Mat44 inv_i2 = mp2->GetInverseInertiaForRotation(inRotation2);
			mInvI2_R2xN1 = inv_i2.Multiply3x3(mR2xN1);
			mInvI2_R2xN2 = inv_i2.Multiply3x3(mR2xN2);

			inv_effective_mass(0, 0) += mp2->GetInverseMass() + mR2xN1.Dot(mInvI2_R2xN1);
			inv_effective_mass(0, 1) += mR2xN1.Dot(mInvI2_R2xN2);
			inv_effective_mass(1, 0) += mR2xN2.Dot(mInvI2_R2xN1);
			inv_effective_mass(1, 1) += mp2->GetInverseMass() + mR2xN2.Dot(mInvI2_R2xN2);
		}
		else {	
			mInvI2_R2xN1 = Vec3::sNaN();
			mInvI2_R2xN2 = Vec3::sNaN();
			MOSS_ERROR("mInvI2_R2xN1 and mInvI2_R2xN2set to NaN.");
		}

		if (!mEffectiveMass.SetInversed(inv_effective_mass))
			Deactivate();
	}

	/// Deactivate this constraint
	inline void Deactivate() {
		mEffectiveMass.SetZero();
		mTotalLambda.SetZero();
	}

	/// Check if constraint is active
	inline bool IsActive() const {
		return !mEffectiveMass.IsZero();
	}

	/// Must be called from the WarmStartVelocityConstraint call to apply the previous frame's impulses
	/// All input vectors are in world space
	inline void WarmStart(Body& ioBody1, Body& ioBody2, Vec3Arg inN1, Vec3Arg inN2, float inWarmStartImpulseRatio) {
		mTotalLambda*= inWarmStartImpulseRatio;
		ApplyVelocityStep(ioBody1, ioBody2, inN1, inN2, mTotalLambda);
	}

	/// Iteratively update the velocity constraint. Makes sure d/dt C(...) = 0, where C is the constraint equation.
	/// All input vectors are in world space
	inline bool SolveVelocityConstraint(Body& ioBody1, Body& ioBody2, Vec3Arg inN1, Vec3Arg inN2) {
		Vec2 lambda;
		CalculateLagrangeMultiplier(ioBody1, ioBody2, inN1, inN2, lambda);

		// Store accumulated lambda
		mTotalLambda += lambda;

		return ApplyVelocityStep(ioBody1, ioBody2, inN1, inN2, lambda);
	}

	/// Iteratively update the position constraint. Makes sure C(...) = 0.
	/// All input vectors are in world space
	inline bool SolvePositionConstraint(Body& ioBody1, Body& ioBody2, Vec3Arg inU, Vec3Arg inN1, Vec3Arg inN2, float inBaumgarte) const {
		Vec2 c;
		c[0] = inU.Dot(inN1);
		c[1] = inU.Dot(inN2);
		if (!c.IsZero())
		{
			// Calculate lagrange multiplier (lambda) for Baumgarte stabilization:
			//
			// lambda = -K^-1* beta / dt* C
			//
			// We should divide by inDeltaTime, but we should multiply by inDeltaTime in the Euler step below so they're cancelled out
			Vec2 lambda = -inBaumgarte* (mEffectiveMass* c);

			// Directly integrate velocity change for one time step
			//
			// Euler velocity integration:
			// dv = M^-1 P
			//
			// Impulse:
			// P = J^T lambda
			//
			// Euler position integration:
			// x' = x + dv* dt
			//
			// Note we don't accumulate velocities for the stabilization. This is using the approach described in 'Modeling and
			// Solving Constraints' by Erin Catto presented at GDC 2007. On slide 78 it is suggested to split up the Baumgarte
			// stabilization for positional drift so that it does not actually add to the momentum. We combine an Euler velocity
			// integrate + a position integrate and then discard the velocity change.
			Vec3 impulse = inN1* lambda[0] + inN2* lambda[1];
			if (ioBody1.IsDynamic())
			{
				ioBody1.SubPositionStep(ioBody1.GetMotionProperties()->GetInverseMass()* impulse);
				ioBody1.SubRotationStep(mInvI1_R1PlusUxN1* lambda[0] + mInvI1_R1PlusUxN2* lambda[1]);
			}
			if (ioBody2.IsDynamic())
			{
				ioBody2.AddPositionStep(ioBody2.GetMotionProperties()->GetInverseMass()* impulse);
				ioBody2.AddRotationStep(mInvI2_R2xN1* lambda[0] + mInvI2_R2xN2* lambda[1]);
			}
			return true;
		}
		return false;
	}

	/// Override total lagrange multiplier, can be used to set the initial value for warm starting
	inline void SetTotalLambda(const Vec2& inLambda) {
		mTotalLambda = inLambda;
	}

	/// Return lagrange multiplier
	inline const Vec2& GetTotalLambda() const {
		return mTotalLambda;
	}

	/// Save state of this constraint part
	void SaveState(StateRecorder& inStream) const {
		inStream.Write(mTotalLambda);
	}

	/// Restore state of this constraint part
	void RestoreState(StateRecorder& inStream) {
		inStream.Read(mTotalLambda);
	}

private:
	Vec3						mR1PlusUxN1;
	Vec3						mR1PlusUxN2;
	Vec3						mR2xN1;
	Vec3						mR2xN2;
	Vec3						mInvI1_R1PlusUxN1;
	Vec3						mInvI1_R1PlusUxN2;
	Vec3						mInvI2_R2xN1;
	Vec3						mInvI2_R2xN2;
	Mat22						mEffectiveMass;
	Vec2						mTotalLambda { Vec2::sZero() };
};

class GearConstraintPart {
	/// Internal helper function to update velocities of bodies after Lagrange multiplier is calculated
	MOSS_INLINE bool ApplyVelocityStep(Body& ioBody1, Body& ioBody2, float inLambda) const {
		// Apply impulse if delta is not zero
		if (inLambda != 0.0f) {
			// Calculate velocity change due to constraint
			//
			// Impulse:
			// P = J^T lambda
			//
			// Euler velocity integration:
			// v' = v + M^-1 P
			ioBody1.GetMotionProperties()->AddAngularVelocityStep(inLambda* mInvI1_A);
			ioBody2.GetMotionProperties()->AddAngularVelocityStep(inLambda* mInvI2_B);
			return true;
		}

		return false;
	}

public:
	/// Calculate properties used during the functions below
	/// @param inBody1 The first body that this constraint is attached to
	/// @param inBody2 The second body that this constraint is attached to
	/// @param inWorldSpaceHingeAxis1 The axis around which body 1 rotates
	/// @param inWorldSpaceHingeAxis2 The axis around which body 2 rotates
	/// @param inRatio The ratio between rotation and translation
	inline void CalculateConstraintProperties(const Body& inBody1, Vec3Arg inWorldSpaceHingeAxis1, const Body& inBody2, Vec3Arg inWorldSpaceHingeAxis2, float inRatio) {
		MOSS_ASSERT(inWorldSpaceHingeAxis1.IsNormalized(1.0e-4f));
		MOSS_ASSERT(inWorldSpaceHingeAxis2.IsNormalized(1.0e-4f));

		// Calculate: I1^-1 a
		mInvI1_A = inBody1.GetMotionProperties()->MultiplyWorldSpaceInverseInertiaByVector(inBody1.GetRotation(), inWorldSpaceHingeAxis1);

		// Calculate: I2^-1 b
		mInvI2_B = inBody2.GetMotionProperties()->MultiplyWorldSpaceInverseInertiaByVector(inBody2.GetRotation(), inWorldSpaceHingeAxis2);

		// K^-1 = 1 / (J M^-1 J^T) = 1 / (a^T I1^-1 a + r^2* b^T I2^-1 b)
		float inv_effective_mass = (inWorldSpaceHingeAxis1.Dot(mInvI1_A) + inWorldSpaceHingeAxis2.Dot(mInvI2_B)* Square(inRatio));
		if (inv_effective_mass == 0.0f)
			Deactivate();
		else
			mEffectiveMass = 1.0f / inv_effective_mass;
	}

	/// Deactivate this constraint
	inline void	Deactivate() {
		mEffectiveMass = 0.0f;
		mTotalLambda = 0.0f;
	}

	/// Check if constraint is active
	inline bool	IsActive() const {
		return mEffectiveMass != 0.0f;
	}

	/// Must be called from the WarmStartVelocityConstraint call to apply the previous frame's impulses
	/// @param ioBody1 The first body that this constraint is attached to
	/// @param ioBody2 The second body that this constraint is attached to
	/// @param inWarmStartImpulseRatio Ratio of new step to old time step (dt_new / dt_old) for scaling the lagrange multiplier of the previous frame
	inline void	WarmStart(Body& ioBody1, Body& ioBody2, float inWarmStartImpulseRatio) {
		mTotalLambda*= inWarmStartImpulseRatio;
		ApplyVelocityStep(ioBody1, ioBody2, mTotalLambda);
	}

	/// Iteratively update the velocity constraint. Makes sure d/dt C(...) = 0, where C is the constraint equation.
	/// @param ioBody1 The first body that this constraint is attached to
	/// @param ioBody2 The second body that this constraint is attached to
	/// @param inWorldSpaceHingeAxis1 The axis around which body 1 rotates
	/// @param inWorldSpaceHingeAxis2 The axis around which body 2 rotates
	/// @param inRatio The ratio between rotation and translation
	inline bool	SolveVelocityConstraint(Body& ioBody1, Vec3Arg inWorldSpaceHingeAxis1, Body& ioBody2, Vec3Arg inWorldSpaceHingeAxis2, float inRatio) {
		// Lagrange multiplier is:
		//
		// lambda = -K^-1 (J v + b)
		float lambda = -mEffectiveMass* (inWorldSpaceHingeAxis1.Dot(ioBody1.GetAngularVelocity()) + inRatio* inWorldSpaceHingeAxis2.Dot(ioBody2.GetAngularVelocity()));
		mTotalLambda += lambda; // Store accumulated impulse

		return ApplyVelocityStep(ioBody1, ioBody2, lambda);
	}

	/// Return lagrange multiplier
	float GetTotalLambda() const {
		return mTotalLambda;
	}

	/// Iteratively update the position constraint. Makes sure C(...) == 0.
	/// @param ioBody1 The first body that this constraint is attached to
	/// @param ioBody2 The second body that this constraint is attached to
	/// @param inC Value of the constraint equation (C)
	/// @param inBaumgarte Baumgarte constant (fraction of the error to correct)
	inline bool SolvePositionConstraint(Body& ioBody1, Body& ioBody2, float inC, float inBaumgarte) const {
		// Only apply position constraint when the constraint is hard, otherwise the velocity bias will fix the constraint
		if (inC != 0.0f) {
			// Calculate lagrange multiplier (lambda) for Baumgarte stabilization:
			//
			// lambda = -K^-1* beta / dt* C
			//
			// We should divide by inDeltaTime, but we should multiply by inDeltaTime in the Euler step below so they're cancelled out
			float lambda = -mEffectiveMass* inBaumgarte* inC;

			// Directly integrate velocity change for one time step
			//
			// Euler velocity integration:
			// dv = M^-1 P
			//
			// Impulse:
			// P = J^T lambda
			//
			// Euler position integration:
			// x' = x + dv* dt
			//
			// Note we don't accumulate velocities for the stabilization. This is using the approach described in 'Modeling and
			// Solving Constraints' by Erin Catto presented at GDC 2007. On slide 78 it is suggested to split up the Baumgarte
			// stabilization for positional drift so that it does not actually add to the momentum. We combine an Euler velocity
			// integrate + a position integrate and then discard the velocity change.
			if (ioBody1.IsDynamic())
				ioBody1.AddRotationStep(lambda* mInvI1_A);
			if (ioBody2.IsDynamic())
				ioBody2.AddRotationStep(lambda* mInvI2_B);
			return true;
		}
		return false;
	}

	/// Save state of this constraint part
	void SaveState(StateRecorder& inStream) const {
		inStream.Write(mTotalLambda);
	}

	/// Restore state of this constraint part
	void RestoreState(StateRecorder& inStream) {
		inStream.Read(mTotalLambda);
	}

private:
	Vec3						mInvI1_A;
	Vec3						mInvI2_B;
	float						mEffectiveMass = 0.0f;
	float						mTotalLambda = 0.0f;
};


class PointConstraintPart {
	MOSS_INLINE bool ApplyVelocityStep(Body& ioBody1, Body& ioBody2, Vec3Arg inLambda) const {
		// Apply impulse if delta is not zero
		if (inLambda != Vec3::sZero()) {
			// Calculate velocity change due to constraint
			//
			// Impulse:
			// P = J^T lambda
			//
			// Euler velocity integration:
			// v' = v + M^-1 P
			if (ioBody1.IsDynamic()) {
				MotionProperties*mp1 = ioBody1.GetMotionProperties();
				mp1->SubLinearVelocityStep(mp1->GetInverseMass()* inLambda);
				mp1->SubAngularVelocityStep(mInvI1_R1X* inLambda);
			}
			if (ioBody2.IsDynamic()) {
				MotionProperties*mp2 = ioBody2.GetMotionProperties();
				mp2->AddLinearVelocityStep(mp2->GetInverseMass()* inLambda);
				mp2->AddAngularVelocityStep(mInvI2_R2X* inLambda);
			}
			return true;
		}

		return false;
	}

public:
	/// Calculate properties used during the functions below
	/// @param inBody1 The first body that this constraint is attached to
	/// @param inBody2 The second body that this constraint is attached to
	/// @param inRotation1 The 3x3 rotation matrix for body 1 (translation part is ignored)
	/// @param inRotation2 The 3x3 rotation matrix for body 2 (translation part is ignored)
	/// @param inR1 Local space vector from center of mass to constraint point for body 1
	/// @param inR2 Local space vector from center of mass to constraint point for body 2
	inline void CalculateConstraintProperties(const Body& inBody1, Mat44Arg inRotation1, Vec3Arg inR1, const Body& inBody2, Mat44Arg inRotation2, Vec3Arg inR2) {
		// Positions where the point constraint acts on (middle point between center of masses) in world space
		mR1 = inRotation1.Multiply3x3(inR1);
		mR2 = inRotation2.Multiply3x3(inR2);

		// Calculate effective mass: K^-1 = (J M^-1 J^T)^-1
		// Using: I^-1 = R* Ibody^-1* R^T
		float summed_inv_mass;
		Mat44 inv_effective_mass;
		if (inBody1.IsDynamic()) {
			const MotionProperties*mp1 = inBody1.GetMotionProperties();
			Mat44 inv_i1 = mp1->GetInverseInertiaForRotation(inRotation1);
			summed_inv_mass = mp1->GetInverseMass();

			Mat44 r1x = Mat44::sCrossProduct(mR1);
			mInvI1_R1X = inv_i1.Multiply3x3(r1x);
			inv_effective_mass = r1x.Multiply3x3(inv_i1).Multiply3x3RightTransposed(r1x);
		}
		else {
			mInvI1_R1X = Mat44::sNaN();
			MOSS_ERROR("had to store as NaN.")

			summed_inv_mass = 0.0f;
			inv_effective_mass = Mat44::sZero();
		}

		if (inBody2.IsDynamic()) {
			const MotionProperties*mp2 = inBody2.GetMotionProperties();
			Mat44 inv_i2 = mp2->GetInverseInertiaForRotation(inRotation2);
			summed_inv_mass += mp2->GetInverseMass();

			Mat44 r2x = Mat44::sCrossProduct(mR2);
			mInvI2_R2X = inv_i2.Multiply3x3(r2x);
			inv_effective_mass += r2x.Multiply3x3(inv_i2).Multiply3x3RightTransposed(r2x);
		}
		else {
			mInvI2_R2X = Mat44::sNaN();
			MOSS_ERROR("had to store as NaN.")
		}

		inv_effective_mass += Mat44::sScale(summed_inv_mass);
		if (!mEffectiveMass.SetInversed3x3(inv_effective_mass))
			Deactivate();
	}

	/// Deactivate this constraint
	inline void	Deactivate() {
		mEffectiveMass = Mat44::sZero();
		mTotalLambda = Vec3::sZero();
	}

	/// Check if constraint is active
	inline bool	IsActive() const {
		return mEffectiveMass(3, 3) != 0.0f;
	}

	/// Must be called from the WarmStartVelocityConstraint call to apply the previous frame's impulses
	/// @param ioBody1 The first body that this constraint is attached to
	/// @param ioBody2 The second body that this constraint is attached to
	/// @param inWarmStartImpulseRatio Ratio of new step to old time step (dt_new / dt_old) for scaling the lagrange multiplier of the previous frame
	inline void	WarmStart(Body& ioBody1, Body& ioBody2, float inWarmStartImpulseRatio) {
		mTotalLambda*= inWarmStartImpulseRatio;
		ApplyVelocityStep(ioBody1, ioBody2, mTotalLambda);
	}

	/// Iteratively update the velocity constraint. Makes sure d/dt C(...) = 0, where C is the constraint equation.
	/// @param ioBody1 The first body that this constraint is attached to
	/// @param ioBody2 The second body that this constraint is attached to
	inline bool SolveVelocityConstraint(Body& ioBody1, Body& ioBody2) {
		// Calculate lagrange multiplier:
		//
		// lambda = -K^-1 (J v + b)
		Vec3 lambda = mEffectiveMass* (ioBody1.GetLinearVelocity() - mR1.Cross(ioBody1.GetAngularVelocity()) - ioBody2.GetLinearVelocity() + mR2.Cross(ioBody2.GetAngularVelocity()));
		mTotalLambda += lambda; // Store accumulated lambda
		return ApplyVelocityStep(ioBody1, ioBody2, lambda);
	}

	/// Iteratively update the position constraint. Makes sure C(...) = 0.
	/// @param ioBody1 The first body that this constraint is attached to
	/// @param ioBody2 The second body that this constraint is attached to
	/// @param inBaumgarte Baumgarte constant (fraction of the error to correct)
	inline bool	SolvePositionConstraint(Body& ioBody1, Body& ioBody2, float inBaumgarte) const {
		Vec3 separation = (Vec3(ioBody2.GetCenterOfMassPosition() - ioBody1.GetCenterOfMassPosition()) + mR2 - mR1);
		if (separation != Vec3::sZero()) {
			// Calculate lagrange multiplier (lambda) for Baumgarte stabilization:
			//
			// lambda = -K^-1* beta / dt* C
			//
			// We should divide by inDeltaTime, but we should multiply by inDeltaTime in the Euler step below so they're cancelled out
			Vec3 lambda = mEffectiveMass* -inBaumgarte* separation;

			// Directly integrate velocity change for one time step
			//
			// Euler velocity integration:
			// dv = M^-1 P
			//
			// Impulse:
			// P = J^T lambda
			//
			// Euler position integration:
			// x' = x + dv* dt
			//
			// Note we don't accumulate velocities for the stabilization. This is using the approach described in 'Modeling and
			// Solving Constraints' by Erin Catto presented at GDC 2007. On slide 78 it is suggested to split up the Baumgarte
			// stabilization for positional drift so that it does not actually add to the momentum. We combine an Euler velocity
			// integrate + a position integrate and then discard the velocity change.
			if (ioBody1.IsDynamic()) {
				ioBody1.SubPositionStep(ioBody1.GetMotionProperties()->GetInverseMass()* lambda);
				ioBody1.SubRotationStep(mInvI1_R1X* lambda);
			}
			if (ioBody2.IsDynamic()) {
				ioBody2.AddPositionStep(ioBody2.GetMotionProperties()->GetInverseMass()* lambda);
				ioBody2.AddRotationStep(mInvI2_R2X* lambda);
			}

			return true;
		}

		return false;
	}

	/// Return lagrange multiplier
	Vec3 GetTotalLambda() const {
		return mTotalLambda;
	}

	/// Save state of this constraint part
	void SaveState(StateRecorder& inStream) const {
		inStream.Write(mTotalLambda);
	}

	/// Restore state of this constraint part
	void RestoreState(StateRecorder& inStream) {
		inStream.Read(mTotalLambda);
	}

private:
	Vec3						mR1;
	Vec3						mR2;
	Mat44						mInvI1_R1X;
	Mat44						mInvI2_R2X;
	Mat44						mEffectiveMass;
	Vec3						mTotalLambda { Vec3::sZero() };
};


class HingeRotationConstraintPart {
public:
	using Vec2 = TVec<2>;
	using Mat22 = Matrix<2, 2>;

private:
	/// Internal helper function to update velocities of bodies after Lagrange multiplier is calculated
	MOSS_INLINE bool ApplyVelocityStep(Body& ioBody1, Body& ioBody2, const Vec2& inLambda) const {
		// Apply impulse if delta is not zero
		if (!inLambda.IsZero()) {
			// Calculate velocity change due to constraint
			//
			// Impulse:
			// P = J^T lambda
			//
			// Euler velocity integration:
			// v' = v + M^-1 P
			Vec3 impulse = mB2xA1* inLambda[0] + mC2xA1* inLambda[1];
			if (ioBody1.IsDynamic())
				ioBody1.GetMotionProperties()->SubAngularVelocityStep(mInvI1.Multiply3x3(impulse));
			if (ioBody2.IsDynamic())
				ioBody2.GetMotionProperties()->AddAngularVelocityStep(mInvI2.Multiply3x3(impulse));
			return true;
		}

		return false;
	}

public:
	/// Calculate properties used during the functions below
	inline void	CalculateConstraintProperties(const Body& inBody1, Mat44Arg inRotation1, Vec3Arg inWorldSpaceHingeAxis1, const Body& inBody2, Mat44Arg inRotation2, Vec3Arg inWorldSpaceHingeAxis2) {
		MOSS_ASSERT(inWorldSpaceHingeAxis1.IsNormalized(1.0e-5f));
		MOSS_ASSERT(inWorldSpaceHingeAxis2.IsNormalized(1.0e-5f));

		// Calculate hinge axis in world space
		mA1 = inWorldSpaceHingeAxis1;
		Vec3 a2 = inWorldSpaceHingeAxis2;
		float dot = mA1.Dot(a2);
		if (dot <= 1.0e-3f) {
			// World space axes are more than 90 degrees apart, get a perpendicular vector in the plane formed by mA1 and a2 as hinge axis until the rotation is less than 90 degrees
			Vec3 perp = a2 - dot* mA1;
			if (perp.LengthSq() < 1.0e-6f) {
				// mA1 ~ -a2, take random perpendicular
				perp = mA1.GetNormalizedPerpendicular();
			}

			// Blend in a little bit from mA1 so we're less than 90 degrees apart
			a2 = (0.99f* perp.Normalized() + 0.01f* mA1).Normalized();
		}
		mB2 = a2.GetNormalizedPerpendicular();
		mC2 = a2.Cross(mB2);

		// Calculate properties used during constraint solving
		mInvI1 = inBody1.IsDynamic()? inBody1.GetMotionProperties()->GetInverseInertiaForRotation(inRotation1) : Mat44::sZero();
		mInvI2 = inBody2.IsDynamic()? inBody2.GetMotionProperties()->GetInverseInertiaForRotation(inRotation2) : Mat44::sZero();
		mB2xA1 = mB2.Cross(mA1);
		mC2xA1 = mC2.Cross(mA1);

		// Calculate effective mass: K^-1 = (J M^-1 J^T)^-1
		Mat44 summed_inv_inertia = mInvI1 + mInvI2;
		Mat22 inv_effective_mass;
		inv_effective_mass(0, 0) = mB2xA1.Dot(summed_inv_inertia.Multiply3x3(mB2xA1));
		inv_effective_mass(0, 1) = mB2xA1.Dot(summed_inv_inertia.Multiply3x3(mC2xA1));
		inv_effective_mass(1, 0) = mC2xA1.Dot(summed_inv_inertia.Multiply3x3(mB2xA1));
		inv_effective_mass(1, 1) = mC2xA1.Dot(summed_inv_inertia.Multiply3x3(mC2xA1));
		if (!mEffectiveMass.SetInversed(inv_effective_mass))
			Deactivate();
	}

	/// Deactivate this constraint
	inline void					Deactivate() {
		mEffectiveMass.SetZero();
		mTotalLambda.SetZero();
	}

	/// Must be called from the WarmStartVelocityConstraint call to apply the previous frame's impulses
	inline void	WarmStart(Body& ioBody1, Body& ioBody2, float inWarmStartImpulseRatio) {
		mTotalLambda*= inWarmStartImpulseRatio;
		ApplyVelocityStep(ioBody1, ioBody2, mTotalLambda);
	}

	/// Iteratively update the velocity constraint. Makes sure d/dt C(...) = 0, where C is the constraint equation.
	inline bool	SolveVelocityConstraint(Body& ioBody1, Body& ioBody2) {
		// Calculate lagrange multiplier:
		//
		// lambda = -K^-1 (J v + b)
		Vec3 delta_ang = ioBody1.GetAngularVelocity() - ioBody2.GetAngularVelocity();
		Vec2 jv;
		jv[0] = mB2xA1.Dot(delta_ang);
		jv[1] = mC2xA1.Dot(delta_ang);
		Vec2 lambda = mEffectiveMass* jv;

		// Store accumulated lambda
		mTotalLambda += lambda;

		return ApplyVelocityStep(ioBody1, ioBody2, lambda);
	}

	/// Iteratively update the position constraint. Makes sure C(...) = 0.
	inline bool	SolvePositionConstraint(Body& ioBody1, Body& ioBody2, float inBaumgarte) const
	{
		// Constraint needs Axis of body 1 perpendicular to both B and C from body 2 (which are both perpendicular to the Axis of body 2)
		Vec2 c;
		c[0] = mA1.Dot(mB2);
		c[1] = mA1.Dot(mC2);
		if (!c.IsZero()) {
			// Calculate lagrange multiplier (lambda) for Baumgarte stabilization:
			//
			// lambda = -K^-1* beta / dt* C
			//
			// We should divide by inDeltaTime, but we should multiply by inDeltaTime in the Euler step below so they're cancelled out
			Vec2 lambda = -inBaumgarte* (mEffectiveMass* c);

			// Directly integrate velocity change for one time step
			//
			// Euler velocity integration:
			// dv = M^-1 P
			//
			// Impulse:
			// P = J^T lambda
			//
			// Euler position integration:
			// x' = x + dv* dt
			//
			// Note we don't accumulate velocities for the stabilization. This is using the approach described in 'Modeling and
			// Solving Constraints' by Erin Catto presented at GDC 2007. On slide 78 it is suggested to split up the Baumgarte
			// stabilization for positional drift so that it does not actually add to the momentum. We combine an Euler velocity
			// integrate + a position integrate and then discard the velocity change.
			Vec3 impulse = mB2xA1* lambda[0] + mC2xA1* lambda[1];
			if (ioBody1.IsDynamic())
				ioBody1.SubRotationStep(mInvI1.Multiply3x3(impulse));
			if (ioBody2.IsDynamic())
				ioBody2.AddRotationStep(mInvI2.Multiply3x3(impulse));
			return true;
		}

		return false;
	}

	/// Return lagrange multiplier
	const Vec2& GetTotalLambda() const {
		return mTotalLambda;
	}

	/// Save state of this constraint part
	void SaveState(StateRecorder& inStream) const {
		inStream.Write(mTotalLambda);
	}

	/// Restore state of this constraint part
	void RestoreState(StateRecorder& inStream) {
		inStream.Read(mTotalLambda);
	}

private:
	Vec3						mA1;						///< World space hinge axis for body 1
	Vec3						mB2;						///< World space perpendiculars of hinge axis for body 2
	Vec3						mC2;
	Mat44						mInvI1;
	Mat44						mInvI2;
	Vec3						mB2xA1;
	Vec3						mC2xA1;
	Mat22						mEffectiveMass;
	Vec2						mTotalLambda { Vec2::sZero() };
};


class AxisConstraintPart {
	/// Internal helper function to update velocities of bodies after Lagrange multiplier is calculated
	template <EMotionType Type1, EMotionType Type2>
	MOSS_INLINE bool ApplyVelocityStep(MotionProperties* ioMotionProperties1, float inInvMass1, MotionProperties* ioMotionProperties2, float inInvMass2, Vec3Arg inWorldSpaceAxis, float inLambda) const {
		// Apply impulse if delta is not zero
		if (inLambda != 0.0f) {
			// Calculate velocity change due to constraint
			//
			// Impulse:
			// P = J^T lambda
			//
			// Euler velocity integration:
			// v' = v + M^-1 P
			if constexpr (Type1 == EMotionType::Dynamic) {
				ioMotionProperties1->SubLinearVelocityStep((inLambda* inInvMass1)* inWorldSpaceAxis);
				ioMotionProperties1->SubAngularVelocityStep(inLambda* Vec3::sLoadFloat3Unsafe(mInvI1_R1PlusUxAxis));
			}
			if constexpr (Type2 == EMotionType::Dynamic) {
				ioMotionProperties2->AddLinearVelocityStep((inLambda* inInvMass2)* inWorldSpaceAxis);
				ioMotionProperties2->AddAngularVelocityStep(inLambda* Vec3::sLoadFloat3Unsafe(mInvI2_R2xAxis));
			}
			return true;
		}

		return false;
	}

	/// Internal helper function to calculate the inverse effective mass
	template <EMotionType Type1, EMotionType Type2>
	MOSS_INLINE float TemplatedCalculateInverseEffectiveMass(float inInvMass1, Mat44Arg inInvI1, Vec3Arg inR1PlusU, float inInvMass2, Mat44Arg inInvI2, Vec3Arg inR2, Vec3Arg inWorldSpaceAxis) {
		MOSS_ASSERT(inWorldSpaceAxis.IsNormalized(1.0e-5f));

		// Calculate properties used below
		Vec3 r1_plus_u_x_axis;
		if constexpr (Type1 != EMotionType::Static) {
			r1_plus_u_x_axis = inR1PlusU.Cross(inWorldSpaceAxis);
			r1_plus_u_x_axis.StoreFloat3(&mR1PlusUxAxis);
		}
		else {
		#ifdef MOSS_DEBUG
			Vec3::sNaN().StoreFloat3(&mR1PlusUxAxis);
		#endif
		}

		Vec3 r2_x_axis;
		if constexpr (Type2 != EMotionType::Static) {
			r2_x_axis = inR2.Cross(inWorldSpaceAxis);
			r2_x_axis.StoreFloat3(&mR2xAxis);
		}
		else {
		#ifdef MOSS_DEBUG
			Vec3::sNaN().StoreFloat3(&mR2xAxis);
		#endif
		}

		// Calculate inverse effective mass: K = J M^-1 J^T
		float inv_effective_mass;

		if constexpr (Type1 == EMotionType::Dynamic) {
			Vec3 invi1_r1_plus_u_x_axis = inInvI1.Multiply3x3(r1_plus_u_x_axis);
			invi1_r1_plus_u_x_axis.StoreFloat3(&mInvI1_R1PlusUxAxis);
			inv_effective_mass = inInvMass1 + invi1_r1_plus_u_x_axis.Dot(r1_plus_u_x_axis);
		}
		else {
			(void)r1_plus_u_x_axis; // Fix compiler warning: Not using this (it's not calculated either)
			Vec3::sNaN().StoreFloat3(&mInvI1_R1PlusUxAxis);
			MOSS_ERROR("mInvI1_R1PlusUxAxis set to NaN.")
			inv_effective_mass = 0.0f;
		}

		if constexpr (Type2 == EMotionType::Dynamic) {
			Vec3 invi2_r2_x_axis = inInvI2.Multiply3x3(r2_x_axis);
			invi2_r2_x_axis.StoreFloat3(&mInvI2_R2xAxis);
			inv_effective_mass += inInvMass2 + invi2_r2_x_axis.Dot(r2_x_axis);
		}
		else {
			(void)r2_x_axis; // Fix compiler warning: Not using this (it's not calculated either)
			Vec3::sNaN().StoreFloat3(&mInvI2_R2xAxis);
			MOSS_ERROR("had to store as NaN.")
		}

		return inv_effective_mass;
	}

	/// Internal helper function to calculate the inverse effective mass
	MOSS_INLINE float CalculateInverseEffectiveMass(const Body& inBody1, Vec3Arg inR1PlusU, const Body& inBody2, Vec3Arg inR2, Vec3Arg inWorldSpaceAxis) {
		// Dispatch to the correct templated form
		switch (inBody1.GetMotionType()) {
		case EMotionType::Dynamic: {
				const MotionProperties*mp1 = inBody1.GetMotionPropertiesUnchecked();
				float inv_m1 = mp1->GetInverseMass();
				Mat44 inv_i1 = inBody1.GetInverseInertia();
				switch (inBody2.GetMotionType())
				{
				case EMotionType::Dynamic:
					return TemplatedCalculateInverseEffectiveMass<EMotionType::Dynamic, EMotionType::Dynamic>(inv_m1, inv_i1, inR1PlusU, inBody2.GetMotionPropertiesUnchecked()->GetInverseMass(), inBody2.GetInverseInertia(), inR2, inWorldSpaceAxis);

				case EMotionType::Kinematic:
					return TemplatedCalculateInverseEffectiveMass<EMotionType::Dynamic, EMotionType::Kinematic>(inv_m1, inv_i1, inR1PlusU, 0 /* Will not be used*/, Mat44() /* Will not be used*/, inR2, inWorldSpaceAxis);

				case EMotionType::Static:
					return TemplatedCalculateInverseEffectiveMass<EMotionType::Dynamic, EMotionType::Static>(inv_m1, inv_i1, inR1PlusU, 0 /* Will not be used*/, Mat44() /* Will not be used*/, inR2, inWorldSpaceAxis);

				default:
					break;
				}
				break;
			}

		case EMotionType::Kinematic:
			MOSS_ASSERT(inBody2.IsDynamic());
			return TemplatedCalculateInverseEffectiveMass<EMotionType::Kinematic, EMotionType::Dynamic>(0 /* Will not be used*/, Mat44() /* Will not be used*/, inR1PlusU, inBody2.GetMotionPropertiesUnchecked()->GetInverseMass(), inBody2.GetInverseInertia(), inR2, inWorldSpaceAxis);

		case EMotionType::Static:
			MOSS_ASSERT(inBody2.IsDynamic());
			return TemplatedCalculateInverseEffectiveMass<EMotionType::Static, EMotionType::Dynamic>(0 /* Will not be used*/, Mat44() /* Will not be used*/, inR1PlusU, inBody2.GetMotionPropertiesUnchecked()->GetInverseMass(), inBody2.GetInverseInertia(), inR2, inWorldSpaceAxis);

		default:
			break;
		}

		MOSS_ASSERT(false);
		return 0.0f;
	}

	/// Internal helper function to calculate the inverse effective mass, version that supports mass scaling
	MOSS_INLINE float CalculateInverseEffectiveMassWithMassOverride(const Body& inBody1, float inInvMass1, float inInvInertiaScale1, Vec3Arg inR1PlusU, const Body& inBody2, float inInvMass2, float inInvInertiaScale2, Vec3Arg inR2, Vec3Arg inWorldSpaceAxis) {
		// Dispatch to the correct templated form
		switch (inBody1.GetMotionType()) {
		case EMotionType::Dynamic: {
				Mat44 inv_i1 = inInvInertiaScale1* inBody1.GetInverseInertia();
				switch (inBody2.GetMotionType())
				{
				case EMotionType::Dynamic:
					return TemplatedCalculateInverseEffectiveMass<EMotionType::Dynamic, EMotionType::Dynamic>(inInvMass1, inv_i1, inR1PlusU, inInvMass2, inInvInertiaScale2* inBody2.GetInverseInertia(), inR2, inWorldSpaceAxis);

				case EMotionType::Kinematic:
					return TemplatedCalculateInverseEffectiveMass<EMotionType::Dynamic, EMotionType::Kinematic>(inInvMass1, inv_i1, inR1PlusU, 0 /* Will not be used*/, Mat44() /* Will not be used*/, inR2, inWorldSpaceAxis);

				case EMotionType::Static:
					return TemplatedCalculateInverseEffectiveMass<EMotionType::Dynamic, EMotionType::Static>(inInvMass1, inv_i1, inR1PlusU, 0 /* Will not be used*/, Mat44() /* Will not be used*/, inR2, inWorldSpaceAxis);

				default:
					break;
				}
				break;
			}

		case EMotionType::Kinematic:
			MOSS_ASSERT(inBody2.IsDynamic());
			return TemplatedCalculateInverseEffectiveMass<EMotionType::Kinematic, EMotionType::Dynamic>(0 /* Will not be used*/, Mat44() /* Will not be used*/, inR1PlusU, inInvMass2, inInvInertiaScale2* inBody2.GetInverseInertia(), inR2, inWorldSpaceAxis);

		case EMotionType::Static:
			MOSS_ASSERT(inBody2.IsDynamic());
			return TemplatedCalculateInverseEffectiveMass<EMotionType::Static, EMotionType::Dynamic>(0 /* Will not be used*/, Mat44() /* Will not be used*/, inR1PlusU, inInvMass2, inInvInertiaScale2* inBody2.GetInverseInertia(), inR2, inWorldSpaceAxis);

		default:
			break;
		}

		MOSS_ASSERT(false);
		return 0.0f;
	}

public:
	/// Templated form of CalculateConstraintProperties with the motion types baked in
	template <EMotionType Type1, EMotionType Type2>
	MOSS_INLINE void TemplatedCalculateConstraintProperties(float inInvMass1, Mat44Arg inInvI1, Vec3Arg inR1PlusU, float inInvMass2, Mat44Arg inInvI2, Vec3Arg inR2, Vec3Arg inWorldSpaceAxis, float inBias = 0.0f) {
		float inv_effective_mass = TemplatedCalculateInverseEffectiveMass<Type1, Type2>(inInvMass1, inInvI1, inR1PlusU, inInvMass2, inInvI2, inR2, inWorldSpaceAxis);

		if (inv_effective_mass == 0.0f)
			Deactivate();
		else {
			mEffectiveMass = 1.0f / inv_effective_mass;
			mSpringPart.CalculateSpringPropertiesWithBias(inBias);
		}
		MOSS_DET_LOG("TemplatedCalculateConstraintProperties: invM1: " << inInvMass1 << " invI1: " << inInvI1 << " r1PlusU: " << inR1PlusU << " invM2: " << inInvMass2 << " invI2: " << inInvI2 << " r2: " << inR2 << " bias: " << inBias << " r1PlusUxAxis: " << mR1PlusUxAxis << " r2xAxis: " << mR2xAxis << " invI1_R1PlusUxAxis: " << mInvI1_R1PlusUxAxis << " invI2_R2xAxis: " << mInvI2_R2xAxis << " effectiveMass: " << mEffectiveMass << " totalLambda: " << mTotalLambda);
	}

	/// Calculate properties used during the functions below
	/// @param inBody1 The first body that this constraint is attached to
	/// @param inBody2 The second body that this constraint is attached to
	/// @param inR1PlusU See equations above (r1 + u)
	/// @param inR2 See equations above (r2)
	/// @param inWorldSpaceAxis Axis along which the constraint acts (normalized, pointing from body 1 to 2)
	/// @param inBias Bias term (b) for the constraint impulse: lambda = J v + b
	inline void	CalculateConstraintProperties(const Body& inBody1, Vec3Arg inR1PlusU, const Body& inBody2, Vec3Arg inR2, Vec3Arg inWorldSpaceAxis, float inBias = 0.0f)	{
		float inv_effective_mass = CalculateInverseEffectiveMass(inBody1, inR1PlusU, inBody2, inR2, inWorldSpaceAxis);

		if (inv_effective_mass == 0.0f)
			Deactivate();
		else {
			mEffectiveMass = 1.0f / inv_effective_mass;
			mSpringPart.CalculateSpringPropertiesWithBias(inBias);
		}
	}

	/// Calculate properties used during the functions below, version that supports mass scaling
	/// @param inBody1 The first body that this constraint is attached to
	/// @param inBody2 The second body that this constraint is attached to
	/// @param inInvMass1 The inverse mass of body 1 (only used when body 1 is dynamic)
	/// @param inInvMass2 The inverse mass of body 2 (only used when body 2 is dynamic)
	/// @param inInvInertiaScale1 Scale factor for the inverse inertia of body 1
	/// @param inInvInertiaScale2 Scale factor for the inverse inertia of body 2
	/// @param inR1PlusU See equations above (r1 + u)
	/// @param inR2 See equations above (r2)
	/// @param inWorldSpaceAxis Axis along which the constraint acts (normalized, pointing from body 1 to 2)
	/// @param inBias Bias term (b) for the constraint impulse: lambda = J v + b
	inline void	CalculateConstraintPropertiesWithMassOverride(const Body& inBody1, float inInvMass1, float inInvInertiaScale1, Vec3Arg inR1PlusU, const Body& inBody2, float inInvMass2, float inInvInertiaScale2, Vec3Arg inR2, Vec3Arg inWorldSpaceAxis, float inBias = 0.0f) {
		float inv_effective_mass = CalculateInverseEffectiveMassWithMassOverride(inBody1, inInvMass1, inInvInertiaScale1, inR1PlusU, inBody2, inInvMass2, inInvInertiaScale2, inR2, inWorldSpaceAxis);

		if (inv_effective_mass == 0.0f)
			Deactivate();
		else {
			mEffectiveMass = 1.0f / inv_effective_mass;
			mSpringPart.CalculateSpringPropertiesWithBias(inBias);
		}
	}

	/// Calculate properties used during the functions below
	/// @param inDeltaTime Time step
	/// @param inBody1 The first body that this constraint is attached to
	/// @param inBody2 The second body that this constraint is attached to
	/// @param inR1PlusU See equations above (r1 + u)
	/// @param inR2 See equations above (r2)
	/// @param inWorldSpaceAxis Axis along which the constraint acts (normalized, pointing from body 1 to 2)
	/// @param inBias Bias term (b) for the constraint impulse: lambda = J v + b
	///	@param inC Value of the constraint equation (C).
	///	@param inFrequency Oscillation frequency (Hz).
	///	@param inDamping Damping factor (0 = no damping, 1 = critical damping).
	inline void	CalculateConstraintPropertiesWithFrequencyAndDamping(float inDeltaTime, const Body& inBody1, Vec3Arg inR1PlusU, const Body& inBody2, Vec3Arg inR2, Vec3Arg inWorldSpaceAxis, float inBias, float inC, float inFrequency, float inDamping) {
		float inv_effective_mass = CalculateInverseEffectiveMass(inBody1, inR1PlusU, inBody2, inR2, inWorldSpaceAxis);

		if (inv_effective_mass == 0.0f)
			Deactivate();
		else
			mSpringPart.CalculateSpringPropertiesWithFrequencyAndDamping(inDeltaTime, inv_effective_mass, inBias, inC, inFrequency, inDamping, mEffectiveMass);
	}

	/// Calculate properties used during the functions below
	/// @param inDeltaTime Time step
	/// @param inBody1 The first body that this constraint is attached to
	/// @param inBody2 The second body that this constraint is attached to
	/// @param inR1PlusU See equations above (r1 + u)
	/// @param inR2 See equations above (r2)
	/// @param inWorldSpaceAxis Axis along which the constraint acts (normalized, pointing from body 1 to 2)
	/// @param inBias Bias term (b) for the constraint impulse: lambda = J v + b
	///	@param inC Value of the constraint equation (C).
	///	@param inStiffness Spring stiffness k.
	///	@param inDamping Spring damping coefficient c.
	inline void	CalculateConstraintPropertiesWithStiffnessAndDamping(float inDeltaTime, const Body& inBody1, Vec3Arg inR1PlusU, const Body& inBody2, Vec3Arg inR2, Vec3Arg inWorldSpaceAxis, float inBias, float inC, float inStiffness, float inDamping) {
		float inv_effective_mass = CalculateInverseEffectiveMass(inBody1, inR1PlusU, inBody2, inR2, inWorldSpaceAxis);

		if (inv_effective_mass == 0.0f)
			Deactivate();
		else
			mSpringPart.CalculateSpringPropertiesWithStiffnessAndDamping(inDeltaTime, inv_effective_mass, inBias, inC, inStiffness, inDamping, mEffectiveMass);
	}

	/// Selects one of the above functions based on the spring settings
	inline void CalculateConstraintPropertiesWithSettings(float inDeltaTime, const Body& inBody1, Vec3Arg inR1PlusU, const Body& inBody2, Vec3Arg inR2, Vec3Arg inWorldSpaceAxis, float inBias, float inC, const SpringSettings& inSpringSettings) {
		float inv_effective_mass = CalculateInverseEffectiveMass(inBody1, inR1PlusU, inBody2, inR2, inWorldSpaceAxis);

		if (inv_effective_mass == 0.0f)
			Deactivate();
		else if (inSpringSettings.mMode == ESpringMode::FrequencyAndDamping)
			mSpringPart.CalculateSpringPropertiesWithFrequencyAndDamping(inDeltaTime, inv_effective_mass, inBias, inC, inSpringSettings.mFrequency, inSpringSettings.mDamping, mEffectiveMass);
		else
			mSpringPart.CalculateSpringPropertiesWithStiffnessAndDamping(inDeltaTime, inv_effective_mass, inBias, inC, inSpringSettings.mStiffness, inSpringSettings.mDamping, mEffectiveMass);
	}

	/// Deactivate this constraint
	inline void					Deactivate() {
		mEffectiveMass = 0.0f;
		mTotalLambda = 0.0f;
	}

	/// Check if constraint is active
	inline bool					IsActive() const {
		return mEffectiveMass != 0.0f;
	}

	/// Templated form of WarmStart with the motion types baked in
	template <EMotionType Type1, EMotionType Type2>
	inline void TemplatedWarmStart(MotionProperties*ioMotionProperties1, float inInvMass1, MotionProperties*ioMotionProperties2, float inInvMass2, Vec3Arg inWorldSpaceAxis, float inWarmStartImpulseRatio) {
		mTotalLambda*= inWarmStartImpulseRatio;

		ApplyVelocityStep<Type1, Type2>(ioMotionProperties1, inInvMass1, ioMotionProperties2, inInvMass2, inWorldSpaceAxis, mTotalLambda);
	}

	/// Must be called from the WarmStartVelocityConstraint call to apply the previous frame's impulses
	/// @param ioBody1 The first body that this constraint is attached to
	/// @param ioBody2 The second body that this constraint is attached to
	/// @param inWorldSpaceAxis Axis along which the constraint acts (normalized)
	/// @param inWarmStartImpulseRatio Ratio of new step to old time step (dt_new / dt_old) for scaling the lagrange multiplier of the previous frame
	inline void	WarmStart(Body& ioBody1, Body& ioBody2, Vec3Arg inWorldSpaceAxis, float inWarmStartImpulseRatio) {
		EMotionType motion_type1 = ioBody1.GetMotionType();
		MotionProperties*motion_properties1 = ioBody1.GetMotionPropertiesUnchecked();

		EMotionType motion_type2 = ioBody2.GetMotionType();
		MotionProperties*motion_properties2 = ioBody2.GetMotionPropertiesUnchecked();

		// Dispatch to the correct templated form
		// Note: Warm starting doesn't differentiate between kinematic/static bodies so we handle both as static bodies
		if (motion_type1 == EMotionType::Dynamic) {
			if (motion_type2 == EMotionType::Dynamic)
				TemplatedWarmStart<EMotionType::Dynamic, EMotionType::Dynamic>(motion_properties1, motion_properties1->GetInverseMass(), motion_properties2, motion_properties2->GetInverseMass(), inWorldSpaceAxis, inWarmStartImpulseRatio);
			else
				TemplatedWarmStart<EMotionType::Dynamic, EMotionType::Static>(motion_properties1, motion_properties1->GetInverseMass(), motion_properties2, 0.0f /* Unused*/, inWorldSpaceAxis, inWarmStartImpulseRatio);
		}
		else {
			MOSS_ASSERT(motion_type2 == EMotionType::Dynamic);
			TemplatedWarmStart<EMotionType::Static, EMotionType::Dynamic>(motion_properties1, 0.0f /* Unused*/, motion_properties2, motion_properties2->GetInverseMass(), inWorldSpaceAxis, inWarmStartImpulseRatio);
		}
	}

	/// Templated form of SolveVelocityConstraint with the motion types baked in, part 1: get the total lambda
	template <EMotionType Type1, EMotionType Type2>
	MOSS_INLINE float TemplatedSolveVelocityConstraintGetTotalLambda(const MotionProperties*ioMotionProperties1, const MotionProperties*ioMotionProperties2, Vec3Arg inWorldSpaceAxis) const {
		// Calculate jacobian multiplied by linear velocity
		float jv;
		if constexpr (Type1 != EMotionType::Static& & Type2 != EMotionType::Static)
			jv = inWorldSpaceAxis.Dot(ioMotionProperties1->GetLinearVelocity() - ioMotionProperties2->GetLinearVelocity());
		else if constexpr (Type1 != EMotionType::Static)
			jv = inWorldSpaceAxis.Dot(ioMotionProperties1->GetLinearVelocity());
		else if constexpr (Type2 != EMotionType::Static)
			jv = inWorldSpaceAxis.Dot(-ioMotionProperties2->GetLinearVelocity());
		else
			MOSS_ASSERT(false); // Static vs static is nonsensical!

		// Calculate jacobian multiplied by angular velocity
		if constexpr (Type1 != EMotionType::Static)
			jv += Vec3::sLoadFloat3Unsafe(mR1PlusUxAxis).Dot(ioMotionProperties1->GetAngularVelocity());
		if constexpr (Type2 != EMotionType::Static)
			jv -= Vec3::sLoadFloat3Unsafe(mR2xAxis).Dot(ioMotionProperties2->GetAngularVelocity());

		// Lagrange multiplier is:
		//
		// lambda = -K^-1 (J v + b)
		float lambda = mEffectiveMass* (jv - mSpringPart.GetBias(mTotalLambda));

		// Return the total accumulated lambda
		return mTotalLambda + lambda;
	}

	/// Templated form of SolveVelocityConstraint with the motion types baked in, part 2: apply new lambda
	template <EMotionType Type1, EMotionType Type2>
	MOSS_INLINE bool TemplatedSolveVelocityConstraintApplyLambda(MotionProperties*ioMotionProperties1, float inInvMass1, MotionProperties*ioMotionProperties2, float inInvMass2, Vec3Arg inWorldSpaceAxis, float inTotalLambda) {
		float delta_lambda = inTotalLambda - mTotalLambda; // Calculate change in lambda
		mTotalLambda = inTotalLambda; // Store accumulated impulse

		return ApplyVelocityStep<Type1, Type2>(ioMotionProperties1, inInvMass1, ioMotionProperties2, inInvMass2, inWorldSpaceAxis, delta_lambda);
	}

	/// Templated form of SolveVelocityConstraint with the motion types baked in
	template <EMotionType Type1, EMotionType Type2>
	inline bool	TemplatedSolveVelocityConstraint(MotionProperties*ioMotionProperties1, float inInvMass1, MotionProperties*ioMotionProperties2, float inInvMass2, Vec3Arg inWorldSpaceAxis, float inMinLambda, float inMaxLambda) {
		float total_lambda = TemplatedSolveVelocityConstraintGetTotalLambda<Type1, Type2>(ioMotionProperties1, ioMotionProperties2, inWorldSpaceAxis);

		// Clamp impulse to specified range
		total_lambda = Clamp(total_lambda, inMinLambda, inMaxLambda);

		return TemplatedSolveVelocityConstraintApplyLambda<Type1, Type2>(ioMotionProperties1, inInvMass1, ioMotionProperties2, inInvMass2, inWorldSpaceAxis, total_lambda);
	}

	/// Iteratively update the velocity constraint. Makes sure d/dt C(...) = 0, where C is the constraint equation.
	/// @param ioBody1 The first body that this constraint is attached to
	/// @param ioBody2 The second body that this constraint is attached to
	/// @param inWorldSpaceAxis Axis along which the constraint acts (normalized)
	/// @param inMinLambda Minimum value of constraint impulse to apply (N s)
	/// @param inMaxLambda Maximum value of constraint impulse to apply (N s)
	inline bool SolveVelocityConstraint(Body& ioBody1, Body& ioBody2, Vec3Arg inWorldSpaceAxis, float inMinLambda, float inMaxLambda) {
		EMotionType motion_type1 = ioBody1.GetMotionType();
		MotionProperties*motion_properties1 = ioBody1.GetMotionPropertiesUnchecked();

		EMotionType motion_type2 = ioBody2.GetMotionType();
		MotionProperties*motion_properties2 = ioBody2.GetMotionPropertiesUnchecked();

		// Dispatch to the correct templated form
		switch (motion_type1) {
		case EMotionType::Dynamic:
			switch (motion_type2) {
			case EMotionType::Dynamic:
				return TemplatedSolveVelocityConstraint<EMotionType::Dynamic, EMotionType::Dynamic>(motion_properties1, motion_properties1->GetInverseMass(), motion_properties2, motion_properties2->GetInverseMass(), inWorldSpaceAxis, inMinLambda, inMaxLambda);

			case EMotionType::Kinematic:
				return TemplatedSolveVelocityConstraint<EMotionType::Dynamic, EMotionType::Kinematic>(motion_properties1, motion_properties1->GetInverseMass(), motion_properties2, 0.0f /* Unused*/, inWorldSpaceAxis, inMinLambda, inMaxLambda);

			case EMotionType::Static:
				return TemplatedSolveVelocityConstraint<EMotionType::Dynamic, EMotionType::Static>(motion_properties1, motion_properties1->GetInverseMass(), motion_properties2, 0.0f /* Unused*/, inWorldSpaceAxis, inMinLambda, inMaxLambda);

			default:
				MOSS_ASSERT(false);
				break;
			}
			break;

		case EMotionType::Kinematic:
			MOSS_ASSERT(motion_type2 == EMotionType::Dynamic);
			return TemplatedSolveVelocityConstraint<EMotionType::Kinematic, EMotionType::Dynamic>(motion_properties1, 0.0f /* Unused*/, motion_properties2, motion_properties2->GetInverseMass(), inWorldSpaceAxis, inMinLambda, inMaxLambda);

		case EMotionType::Static:
			MOSS_ASSERT(motion_type2 == EMotionType::Dynamic);
			return TemplatedSolveVelocityConstraint<EMotionType::Static, EMotionType::Dynamic>(motion_properties1, 0.0f /* Unused*/, motion_properties2, motion_properties2->GetInverseMass(), inWorldSpaceAxis, inMinLambda, inMaxLambda);

		default:
			MOSS_ASSERT(false);
			break;
		}

		return false;
	}

	/// Iteratively update the velocity constraint. Makes sure d/dt C(...) = 0, where C is the constraint equation.
	/// @param ioBody1 The first body that this constraint is attached to
	/// @param ioBody2 The second body that this constraint is attached to
	/// @param inInvMass1 The inverse mass of body 1 (only used when body 1 is dynamic)
	/// @param inInvMass2 The inverse mass of body 2 (only used when body 2 is dynamic)
	/// @param inWorldSpaceAxis Axis along which the constraint acts (normalized)
	/// @param inMinLambda Minimum value of constraint impulse to apply (N s)
	/// @param inMaxLambda Maximum value of constraint impulse to apply (N s)
	inline bool SolveVelocityConstraintWithMassOverride(Body& ioBody1, float inInvMass1, Body& ioBody2, float inInvMass2, Vec3Arg inWorldSpaceAxis, float inMinLambda, float inMaxLambda) {
		EMotionType motion_type1 = ioBody1.GetMotionType();
		MotionProperties*motion_properties1 = ioBody1.GetMotionPropertiesUnchecked();

		EMotionType motion_type2 = ioBody2.GetMotionType();
		MotionProperties*motion_properties2 = ioBody2.GetMotionPropertiesUnchecked();

		// Dispatch to the correct templated form
		switch (motion_type1) {
		case EMotionType::Dynamic:
			switch (motion_type2) {
			case EMotionType::Dynamic:
				return TemplatedSolveVelocityConstraint<EMotionType::Dynamic, EMotionType::Dynamic>(motion_properties1, inInvMass1, motion_properties2, inInvMass2, inWorldSpaceAxis, inMinLambda, inMaxLambda);

			case EMotionType::Kinematic:
				return TemplatedSolveVelocityConstraint<EMotionType::Dynamic, EMotionType::Kinematic>(motion_properties1, inInvMass1, motion_properties2, 0.0f /* Unused*/, inWorldSpaceAxis, inMinLambda, inMaxLambda);

			case EMotionType::Static:
				return TemplatedSolveVelocityConstraint<EMotionType::Dynamic, EMotionType::Static>(motion_properties1, inInvMass1, motion_properties2, 0.0f /* Unused*/, inWorldSpaceAxis, inMinLambda, inMaxLambda);

			default:
				MOSS_ASSERT(false);
				break;
			}
			break;

		case EMotionType::Kinematic:
			MOSS_ASSERT(motion_type2 == EMotionType::Dynamic);
			return TemplatedSolveVelocityConstraint<EMotionType::Kinematic, EMotionType::Dynamic>(motion_properties1, 0.0f /* Unused*/, motion_properties2, inInvMass2, inWorldSpaceAxis, inMinLambda, inMaxLambda);

		case EMotionType::Static:
			MOSS_ASSERT(motion_type2 == EMotionType::Dynamic);
			return TemplatedSolveVelocityConstraint<EMotionType::Static, EMotionType::Dynamic>(motion_properties1, 0.0f /* Unused*/, motion_properties2, inInvMass2, inWorldSpaceAxis, inMinLambda, inMaxLambda);

		default:
			MOSS_ASSERT(false);
			break;
		}

		return false;
	}

	/// Iteratively update the position constraint. Makes sure C(...) = 0.
	/// @param ioBody1 The first body that this constraint is attached to
	/// @param ioBody2 The second body that this constraint is attached to
	/// @param inWorldSpaceAxis Axis along which the constraint acts (normalized)
	/// @param inC Value of the constraint equation (C)
	/// @param inBaumgarte Baumgarte constant (fraction of the error to correct)
	inline bool SolvePositionConstraint(Body& ioBody1, Body& ioBody2, Vec3Arg inWorldSpaceAxis, float inC, float inBaumgarte) const {
		// Only apply position constraint when the constraint is hard, otherwise the velocity bias will fix the constraint
		if (inC != 0.0f& & !mSpringPart.IsActive()) {
			// Calculate lagrange multiplier (lambda) for Baumgarte stabilization:
			//
			// lambda = -K^-1* beta / dt* C
			//
			// We should divide by inDeltaTime, but we should multiply by inDeltaTime in the Euler step below so they're cancelled out
			float lambda = -mEffectiveMass* inBaumgarte* inC;

			// Directly integrate velocity change for one time step
			//
			// Euler velocity integration:
			// dv = M^-1 P
			//
			// Impulse:
			// P = J^T lambda
			//
			// Euler position integration:
			// x' = x + dv* dt
			//
			// Note we don't accumulate velocities for the stabilization. This is using the approach described in 'Modeling and
			// Solving Constraints' by Erin Catto presented at GDC 2007. On slide 78 it is suggested to split up the Baumgarte
			// stabilization for positional drift so that it does not actually add to the momentum. We combine an Euler velocity
			// integrate + a position integrate and then discard the velocity change.
			if (ioBody1.IsDynamic()) {
				ioBody1.SubPositionStep((lambda* ioBody1.GetMotionProperties()->GetInverseMass())* inWorldSpaceAxis);
				ioBody1.SubRotationStep(lambda* Vec3::sLoadFloat3Unsafe(mInvI1_R1PlusUxAxis));
			}
			if (ioBody2.IsDynamic()) {
				ioBody2.AddPositionStep((lambda* ioBody2.GetMotionProperties()->GetInverseMass())* inWorldSpaceAxis);
				ioBody2.AddRotationStep(lambda* Vec3::sLoadFloat3Unsafe(mInvI2_R2xAxis));
			}
			return true;
		}
		return false;
	}

	/// Iteratively update the position constraint. Makes sure C(...) = 0.
	/// @param ioBody1 The first body that this constraint is attached to
	/// @param ioBody2 The second body that this constraint is attached to
	/// @param inInvMass1 The inverse mass of body 1 (only used when body 1 is dynamic)
	/// @param inInvMass2 The inverse mass of body 2 (only used when body 2 is dynamic)
	/// @param inWorldSpaceAxis Axis along which the constraint acts (normalized)
	/// @param inC Value of the constraint equation (C)
	/// @param inBaumgarte Baumgarte constant (fraction of the error to correct)
	inline bool	SolvePositionConstraintWithMassOverride(Body& ioBody1, float inInvMass1, Body& ioBody2, float inInvMass2, Vec3Arg inWorldSpaceAxis, float inC, float inBaumgarte) const {
		// Only apply position constraint when the constraint is hard, otherwise the velocity bias will fix the constraint
		if (inC != 0.0f& & !mSpringPart.IsActive()) {
			// Calculate lagrange multiplier (lambda) for Baumgarte stabilization:
			//
			// lambda = -K^-1* beta / dt* C
			//
			// We should divide by inDeltaTime, but we should multiply by inDeltaTime in the Euler step below so they're cancelled out
			float lambda = -mEffectiveMass* inBaumgarte* inC;

			// Directly integrate velocity change for one time step
			//
			// Euler velocity integration:
			// dv = M^-1 P
			//
			// Impulse:
			// P = J^T lambda
			//
			// Euler position integration:
			// x' = x + dv* dt
			//
			// Note we don't accumulate velocities for the stabilization. This is using the approach described in 'Modeling and
			// Solving Constraints' by Erin Catto presented at GDC 2007. On slide 78 it is suggested to split up the Baumgarte
			// stabilization for positional drift so that it does not actually add to the momentum. We combine an Euler velocity
			// integrate + a position integrate and then discard the velocity change.
			if (ioBody1.IsDynamic()) {
				ioBody1.SubPositionStep((lambda* inInvMass1)* inWorldSpaceAxis);
				ioBody1.SubRotationStep(lambda* Vec3::sLoadFloat3Unsafe(mInvI1_R1PlusUxAxis));
			}
			if (ioBody2.IsDynamic()) {
				ioBody2.AddPositionStep((lambda* inInvMass2)* inWorldSpaceAxis);
				ioBody2.AddRotationStep(lambda* Vec3::sLoadFloat3Unsafe(mInvI2_R2xAxis));
			}
			return true;
		}
		return false;
	}

	/// Override total lagrange multiplier, can be used to set the initial value for warm starting
	inline void SetTotalLambda(float inLambda) {
		mTotalLambda = inLambda;
	}

	/// Return lagrange multiplier
	inline float GetTotalLambda() const {
		return mTotalLambda;
	}

	/// Save state of this constraint part
	void SaveState(StateRecorder& inStream) const {
		inStream.Write(mTotalLambda);
	}

	/// Restore state of this constraint part
	void RestoreState(StateRecorder& inStream) {
		inStream.Read(mTotalLambda);
	}

private:
	Float3						mR1PlusUxAxis;
	Float3						mR2xAxis;
	Float3						mInvI1_R1PlusUxAxis;
	Float3						mInvI2_R2xAxis;
	float						mEffectiveMass = 0.0f;
	SpringPart					mSpringPart;
	float						mTotalLambda = 0.0f;
};


class IndependentAxisConstraintPart {
	/// Internal helper function to update velocities of bodies after Lagrange multiplier is calculated
	MOSS_INLINE bool ApplyVelocityStep(Body& ioBody1, Body& ioBody2, Vec3Arg inN1, Vec3Arg inN2, float inRatio, float inLambda) const {
		// Apply impulse if delta is not zero
		if (inLambda != 0.0f) {
			// Calculate velocity change due to constraint
			//
			// Impulse:
			// P = J^T lambda
			//
			// Euler velocity integration:
			// v' = v + M^-1 P
			if (ioBody1.IsDynamic()) {
				MotionProperties*mp1 = ioBody1.GetMotionProperties();
				mp1->AddLinearVelocityStep((mp1->GetInverseMass()* inLambda)* inN1);
				mp1->AddAngularVelocityStep(mInvI1_R1xN1* inLambda);
			}
			if (ioBody2.IsDynamic()) {
				MotionProperties*mp2 = ioBody2.GetMotionProperties();
				mp2->AddLinearVelocityStep((inRatio* mp2->GetInverseMass()* inLambda)* inN2);
				mp2->AddAngularVelocityStep(mInvI2_RatioR2xN2* inLambda);
			}
			return true;
		}

		return false;
	}

public:
	/// Calculate properties used during the functions below
	/// @param inBody1 The first body that this constraint is attached to
	/// @param inBody2 The second body that this constraint is attached to
	/// @param inR1 The position on which the constraint operates on body 1 relative to COM
	/// @param inN1 The world space normal in which the constraint operates for body 1
	/// @param inR2 The position on which the constraint operates on body 1 relative to COM
	/// @param inN2 The world space normal in which the constraint operates for body 2
	/// @param inRatio The ratio how forces are applied between bodies
	inline void CalculateConstraintProperties(const Body& inBody1, const Body& inBody2, Vec3Arg inR1, Vec3Arg inN1, Vec3Arg inR2, Vec3Arg inN2, float inRatio) {
		MOSS_ASSERT(inN1.IsNormalized(1.0e-4f)& & inN2.IsNormalized(1.0e-4f));

		float inv_effective_mass = 0.0f;

		if (!inBody1.IsStatic()) {
			const MotionProperties*mp1 = inBody1.GetMotionProperties();

			mR1xN1 = inR1.Cross(inN1);
			mInvI1_R1xN1 = mp1->MultiplyWorldSpaceInverseInertiaByVector(inBody1.GetRotation(), mR1xN1);

			inv_effective_mass += mp1->GetInverseMass() + mInvI1_R1xN1.Dot(mR1xN1);
		}

		if (!inBody2.IsStatic()) {
			const MotionProperties*mp2 = inBody2.GetMotionProperties();

			mRatioR2xN2 = inRatio* inR2.Cross(inN2);
			mInvI2_RatioR2xN2 = mp2->MultiplyWorldSpaceInverseInertiaByVector(inBody2.GetRotation(), mRatioR2xN2);

			inv_effective_mass += Square(inRatio)* mp2->GetInverseMass() + mInvI2_RatioR2xN2.Dot(mRatioR2xN2);
		}

		// Calculate inverse effective mass: K = J M^-1 J^T
		if (inv_effective_mass == 0.0f)
			Deactivate();
		else
			mEffectiveMass = 1.0f / inv_effective_mass;
	}

	/// Deactivate this constraint
	inline void					Deactivate() {
		mEffectiveMass = 0.0f;
		mTotalLambda = 0.0f;
	}

	/// Check if constraint is active
	inline bool					IsActive() const {
		return mEffectiveMass != 0.0f;
	}

	/// Must be called from the WarmStartVelocityConstraint call to apply the previous frame's impulses
	/// @param ioBody1 The first body that this constraint is attached to
	/// @param ioBody2 The second body that this constraint is attached to
	/// @param inN1 The world space normal in which the constraint operates for body 1
	/// @param inN2 The world space normal in which the constraint operates for body 2
	/// @param inRatio The ratio how forces are applied between bodies
	/// @param inWarmStartImpulseRatio Ratio of new step to old time step (dt_new / dt_old) for scaling the lagrange multiplier of the previous frame
	inline void	WarmStart(Body& ioBody1, Body& ioBody2, Vec3Arg inN1, Vec3Arg inN2, float inRatio, float inWarmStartImpulseRatio) {
		mTotalLambda*= inWarmStartImpulseRatio;
		ApplyVelocityStep(ioBody1, ioBody2, inN1, inN2, inRatio, mTotalLambda);
	}

	/// Iteratively update the velocity constraint. Makes sure d/dt C(...) = 0, where C is the constraint equation.
	/// @param ioBody1 The first body that this constraint is attached to
	/// @param ioBody2 The second body that this constraint is attached to
	/// @param inN1 The world space normal in which the constraint operates for body 1
	/// @param inN2 The world space normal in which the constraint operates for body 2
	/// @param inRatio The ratio how forces are applied between bodies
	/// @param inMinLambda Minimum angular impulse to apply (N m s)
	/// @param inMaxLambda Maximum angular impulse to apply (N m s)
	inline bool	SolveVelocityConstraint(Body& ioBody1, Body& ioBody2, Vec3Arg inN1, Vec3Arg inN2, float inRatio, float inMinLambda, float inMaxLambda) {
		// Lagrange multiplier is:
		//
		// lambda = -K^-1 (J v + b)
		float lambda = -mEffectiveMass* (inN1.Dot(ioBody1.GetLinearVelocity()) + mR1xN1.Dot(ioBody1.GetAngularVelocity()) + inRatio* inN2.Dot(ioBody2.GetLinearVelocity()) + mRatioR2xN2.Dot(ioBody2.GetAngularVelocity()));
		float new_lambda = Clamp(mTotalLambda + lambda, inMinLambda, inMaxLambda); // Clamp impulse
		lambda = new_lambda - mTotalLambda; // Lambda potentially got clamped, calculate the new impulse to apply
		mTotalLambda = new_lambda; // Store accumulated impulse

		return ApplyVelocityStep(ioBody1, ioBody2, inN1, inN2, inRatio, lambda);
	}

	/// Return lagrange multiplier
	float GetTotalLambda() const {
		return mTotalLambda;
	}

	/// Iteratively update the position constraint. Makes sure C(...) == 0.
	/// @param ioBody1 The first body that this constraint is attached to
	/// @param ioBody2 The second body that this constraint is attached to
	/// @param inN1 The world space normal in which the constraint operates for body 1
	/// @param inN2 The world space normal in which the constraint operates for body 2
	/// @param inRatio The ratio how forces are applied between bodies
	/// @param inC Value of the constraint equation (C)
	/// @param inBaumgarte Baumgarte constant (fraction of the error to correct)
	inline bool SolvePositionConstraint(Body& ioBody1, Body& ioBody2, Vec3Arg inN1, Vec3Arg inN2, float inRatio, float inC, float inBaumgarte) const {
		if (inC != 0.0f) {
			// Calculate lagrange multiplier (lambda) for Baumgarte stabilization:
			//
			// lambda = -K^-1* beta / dt* C
			//
			// We should divide by inDeltaTime, but we should multiply by inDeltaTime in the Euler step below so they're cancelled out
			float lambda = -mEffectiveMass* inBaumgarte* inC;

			// Directly integrate velocity change for one time step
			//
			// Euler velocity integration:
			// dv = M^-1 P
			//
			// Impulse:
			// P = J^T lambda
			//
			// Euler position integration:
			// x' = x + dv* dt
			//
			// Note we don't accumulate velocities for the stabilization. This is using the approach described in 'Modeling and
			// Solving Constraints' by Erin Catto presented at GDC 2007. On slide 78 it is suggested to split up the Baumgarte
			// stabilization for positional drift so that it does not actually add to the momentum. We combine an Euler velocity
			// integrate + a position integrate and then discard the velocity change.
			if (ioBody1.IsDynamic()) {
				ioBody1.AddPositionStep((lambda* ioBody1.GetMotionPropertiesUnchecked()->GetInverseMass())* inN1);
				ioBody1.AddRotationStep(lambda* mInvI1_R1xN1);
			}
			if (ioBody2.IsDynamic()) {
				ioBody2.AddPositionStep((lambda* inRatio* ioBody2.GetMotionPropertiesUnchecked()->GetInverseMass())* inN2);
				ioBody2.AddRotationStep(lambda* mInvI2_RatioR2xN2);
			}
			return true;
		}

		return false;
	}

	/// Save state of this constraint part
	void SaveState(StateRecorder& inStream) const {
		inStream.Write(mTotalLambda);
	}

	/// Restore state of this constraint part
	void RestoreState(StateRecorder& inStream) {
		inStream.Read(mTotalLambda);
	}

private:
	Vec3						mR1xN1;
	Vec3						mInvI1_R1xN1;
	Vec3						mRatioR2xN2;
	Vec3						mInvI2_RatioR2xN2;
	float						mEffectiveMass = 0.0f;
	float						mTotalLambda = 0.0f;
};

class RotationQuatConstraintPart
{
private:
	/// Internal helper function to update velocities of bodies after Lagrange multiplier is calculated
	MOSS_INLINE bool ApplyVelocityStep(Body& ioBody1, Body& ioBody2, Vec3Arg inLambda) const {
		// Apply impulse if delta is not zero
		if (inLambda != Vec3::sZero()) {
			// Calculate velocity change due to constraint
			//
			// Impulse:
			// P = J^T lambda
			//
			// Euler velocity integration:
			// v' = v + M^-1 P
			if (ioBody1.IsDynamic())
				ioBody1.GetMotionProperties()->SubAngularVelocityStep(mInvI1_JPT.Multiply3x3(inLambda));
			if (ioBody2.IsDynamic())
				ioBody2.GetMotionProperties()->AddAngularVelocityStep(mInvI2_JPT.Multiply3x3(inLambda));
			return true;
		}

		return false;
	}

public:
	/// Return inverse of initial rotation from body 1 to body 2 in body 1 space
	static Quat					sGetInvInitialOrientation(const Body& inBody1, const Body& inBody2) {
		// q20 = q10 r0
		// <=> r0 = q10^-1 q20
		// <=> r0^-1 = q20^-1 q10
		//
		// where:
		//
		// q20 = initial orientation of body 2
		// q10 = initial orientation of body 1
		// r0 = initial rotation from body 1 to body 2
		return inBody2.GetRotation().Conjugated()* inBody1.GetRotation();
	}

	/// Calculate properties used during the functions below
	inline void	CalculateConstraintProperties(const Body& inBody1, Mat44Arg inRotation1, const Body& inBody2, Mat44Arg inRotation2, QuatArg inInvInitialOrientation) {
		// Calculate: JP = 1/2 A ML(q1^*) MR(q2 r0^*) A^T
		Mat44 jp = (Mat44::sQuatLeftMultiply(0.5f* inBody1.GetRotation().Conjugated())* Mat44::sQuatRightMultiply(inBody2.GetRotation()* inInvInitialOrientation)).GetRotationSafe();

		// Calculate properties used during constraint solving
		Mat44 inv_i1 = inBody1.IsDynamic()? inBody1.GetMotionProperties()->GetInverseInertiaForRotation(inRotation1) : Mat44::sZero();
		Mat44 inv_i2 = inBody2.IsDynamic()? inBody2.GetMotionProperties()->GetInverseInertiaForRotation(inRotation2) : Mat44::sZero();
		mInvI1_JPT = inv_i1.Multiply3x3RightTransposed(jp);
		mInvI2_JPT = inv_i2.Multiply3x3RightTransposed(jp);

		// Calculate effective mass: K^-1 = (J M^-1 J^T)^-1
		// = (JP* I1^-1* JP^T + JP* I2^-1* JP^T)^-1
		// = (JP* (I1^-1 + I2^-1)* JP^T)^-1
		if (!mEffectiveMass.SetInversed3x3(jp.Multiply3x3(inv_i1 + inv_i2).Multiply3x3RightTransposed(jp)))
			Deactivate();
		else
			mEffectiveMass_JP = mEffectiveMass.Multiply3x3(jp);
	}

	/// Deactivate this constraint
	inline void	Deactivate() {
		mEffectiveMass = Mat44::sZero();
		mEffectiveMass_JP = Mat44::sZero();
		mTotalLambda = Vec3::sZero();
	}

	/// Check if constraint is active
	inline bool	IsActive() const {
		return mEffectiveMass(3, 3) != 0.0f;
	}

	/// Must be called from the WarmStartVelocityConstraint call to apply the previous frame's impulses
	inline void WarmStart(Body& ioBody1, Body& ioBody2, float inWarmStartImpulseRatio) {
		mTotalLambda*= inWarmStartImpulseRatio;
		ApplyVelocityStep(ioBody1, ioBody2, mTotalLambda);
	}

	/// Iteratively update the velocity constraint. Makes sure d/dt C(...) = 0, where C is the constraint equation.
	inline bool	SolveVelocityConstraint(Body& ioBody1, Body& ioBody2) {
		// Calculate lagrange multiplier:
		//
		// lambda = -K^-1 (J v + b)
		Vec3 lambda = mEffectiveMass_JP.Multiply3x3(ioBody1.GetAngularVelocity() - ioBody2.GetAngularVelocity());
		mTotalLambda += lambda;
		return ApplyVelocityStep(ioBody1, ioBody2, lambda);
	}

	/// Iteratively update the position constraint. Makes sure C(...) = 0.
	inline bool	SolvePositionConstraint(Body& ioBody1, Body& ioBody2, QuatArg inInvInitialOrientation, float inBaumgarte) const {
		// Calculate constraint equation
		Vec3 c = (ioBody1.GetRotation().Conjugated()* ioBody2.GetRotation()* inInvInitialOrientation).GetXYZ();
		if (c != Vec3::sZero()) {
			// Calculate lagrange multiplier (lambda) for Baumgarte stabilization:
			//
			// lambda = -K^-1* beta / dt* C
			//
			// We should divide by inDeltaTime, but we should multiply by inDeltaTime in the Euler step below so they're cancelled out
			Vec3 lambda = -inBaumgarte* mEffectiveMass* c;

			// Directly integrate velocity change for one time step
			//
			// Euler velocity integration:
			// dv = M^-1 P
			//
			// Impulse:
			// P = J^T lambda
			//
			// Euler position integration:
			// x' = x + dv* dt
			//
			// Note we don't accumulate velocities for the stabilization. This is using the approach described in 'Modeling and
			// Solving Constraints' by Erin Catto presented at GDC 2007. On slide 78 it is suggested to split up the Baumgarte
			// stabilization for positional drift so that it does not actually add to the momentum. We combine an Euler velocity
			// integrate + a position integrate and then discard the velocity change.
			if (ioBody1.IsDynamic())
				ioBody1.SubRotationStep(mInvI1_JPT.Multiply3x3(lambda));
			if (ioBody2.IsDynamic())
				ioBody2.AddRotationStep(mInvI2_JPT.Multiply3x3(lambda));
			return true;
		}

		return false;
	}

	/// Return lagrange multiplier
	Vec3 GetTotalLambda() const {
		return mTotalLambda;
	}

	/// Save state of this constraint part
	void SaveState(StateRecorder& inStream) const {
		inStream.Write(mTotalLambda);
	}

	/// Restore state of this constraint part
	void RestoreState(StateRecorder& inStream) {
		inStream.Read(mTotalLambda);
	}

private:
	Mat44						mInvI1_JPT;
	Mat44						mInvI2_JPT;
	Mat44						mEffectiveMass;
	Mat44						mEffectiveMass_JP;
	Vec3						mTotalLambda { Vec3::sZero() };
};


#ifndef MOSS_PLATFORM_DOXYGEN // Somehow Doxygen gets confused and thinks the parameters to CalculateSpringProperties belong to this macro
MOSS_MSVC_SUPPRESS_WARNING(4723) // potential divide by 0 - caused by line: outEffectiveMass = 1.0f / inInvEffectiveMass, note that MOSS_SUPRESS_WARNINGS_BEGIN already pushes the warning state
#endif // !MOSS_PLATFORM_DOXYGEN

/// Class used in other constraint parts to calculate the required bias factor in the lagrange multiplier for creating springs
class SpringPart {
private:
	MOSS_INLINE void CalculateSpringPropertiesHelper(float inDeltaTime, float inInvEffectiveMass, float inBias, float inC, float inStiffness, float inDamping, float& outEffectiveMass) {
		// Soft constraints as per: Soft Constraints: Reinventing The Spring - Erin Catto - GDC 2011

		// Note that the calculation of beta and gamma below are based on the solution of an implicit Euler integration scheme
		// This scheme is unconditionally stable but has built in damping, so even when you set the damping ratio to 0 there will still
		// be damping. See page 16 and 32.

		// Calculate softness (gamma in the slides)
		// See page 34 and note that the gamma needs to be divided by delta time since we're working with impulses rather than forces:
		// softness = 1 / (dt* (c + dt* k))
		// Note that the spring stiffness is k and the spring damping is c
		mSoftness = 1.0f / (inDeltaTime* (inDamping + inDeltaTime* inStiffness));

		// Calculate bias factor (baumgarte stabilization):
		// beta = dt* k / (c + dt* k) = dt* k^2* softness
		// b = beta / dt* C = dt* k* softness* C
		mBias = inBias + inDeltaTime* inStiffness* mSoftness* inC;

		// Update the effective mass, see post by Erin Catto: http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=4&t=1354
		//
		// Newton's Law:
		// M* (v2 - v1) = J^T* lambda
		//
		// Velocity constraint with softness and Baumgarte:
		// J* v2 + softness* lambda + b = 0
		//
		// where b = beta* C / dt
		//
		// We know everything except v2 and lambda.
		//
		// First solve Newton's law for v2 in terms of lambda:
		//
		// v2 = v1 + M^-1* J^T* lambda
		//
		// Substitute this expression into the velocity constraint:
		//
		// J* (v1 + M^-1* J^T* lambda) + softness* lambda + b = 0
		//
		// Now collect coefficients of lambda:
		//
		// (J* M^-1* J^T + softness)* lambda = - J* v1 - b
		//
		// Now we define:
		//
		// K = J* M^-1* J^T + softness
		//
		// So our new effective mass is K^-1
		outEffectiveMass = 1.0f / (inInvEffectiveMass + mSoftness);
	}

public:
	/// Turn off the spring and set a bias only
	///
	/// @param inBias Bias term (b) for the constraint impulse: lambda = J v + b
	inline void	CalculateSpringPropertiesWithBias(float inBias) {
		mSoftness = 0.0f;
		mBias = inBias;
	}

	/// Calculate spring properties based on frequency and damping ratio
	///
	/// @param inDeltaTime Time step
	/// @param inInvEffectiveMass Inverse effective mass K
	/// @param inBias Bias term (b) for the constraint impulse: lambda = J v + b
	///	@param inC Value of the constraint equation (C). Set to zero if you don't want to drive the constraint to zero with a spring.
	///	@param inFrequency Oscillation frequency (Hz). Set to zero if you don't want to drive the constraint to zero with a spring.
	///	@param inDamping Damping factor (0 = no damping, 1 = critical damping). Set to zero if you don't want to drive the constraint to zero with a spring.
	/// @param outEffectiveMass On return, this contains the new effective mass K^-1
	inline void	CalculateSpringPropertiesWithFrequencyAndDamping(float inDeltaTime, float inInvEffectiveMass, float inBias, float inC, float inFrequency, float inDamping, float& outEffectiveMass) {
		outEffectiveMass = 1.0f / inInvEffectiveMass;

		if (inFrequency > 0.0f) {
			// Calculate angular frequency
			float omega = 2.0f* MOSS_PI* inFrequency;

			// Calculate spring stiffness k and damping constant c (page 45)
			float k = outEffectiveMass* Square(omega);
			float c = 2.0f* outEffectiveMass* inDamping* omega;

			CalculateSpringPropertiesHelper(inDeltaTime, inInvEffectiveMass, inBias, inC, k, c, outEffectiveMass);
		}
		else {
			CalculateSpringPropertiesWithBias(inBias);
		}
	}

	/// Calculate spring properties with spring Stiffness (k) and damping (c), this is based on the spring equation: F = -k* x - c* v
	///
	/// @param inDeltaTime Time step
	/// @param inInvEffectiveMass Inverse effective mass K
	/// @param inBias Bias term (b) for the constraint impulse: lambda = J v + b
	///	@param inC Value of the constraint equation (C). Set to zero if you don't want to drive the constraint to zero with a spring.
	///	@param inStiffness Spring stiffness k. Set to zero if you don't want to drive the constraint to zero with a spring.
	///	@param inDamping Spring damping coefficient c. Set to zero if you don't want to drive the constraint to zero with a spring.
	/// @param outEffectiveMass On return, this contains the new effective mass K^-1
	inline void	CalculateSpringPropertiesWithStiffnessAndDamping(float inDeltaTime, float inInvEffectiveMass, float inBias, float inC, float inStiffness, float inDamping, float& outEffectiveMass) {
		if (inStiffness > 0.0f) {
			CalculateSpringPropertiesHelper(inDeltaTime, inInvEffectiveMass, inBias, inC, inStiffness, inDamping, outEffectiveMass);
		}
		else {
			outEffectiveMass = 1.0f / inInvEffectiveMass;

			CalculateSpringPropertiesWithBias(inBias);
		}
	}

	/// Returns if this spring is active
	inline bool IsActive() const {
		return mSoftness != 0.0f;
	}

	/// Get total bias b, including supplied bias and bias for spring: lambda = J v + b
	inline float GetBias(float inTotalLambda) const {
		// Remainder of post by Erin Catto: http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=4&t=1354
		//
		// Each iteration we are not computing the whole impulse, we are computing an increment to the impulse and we are updating the velocity.
		// Also, as we solve each constraint we get a perfect v2, but then some other constraint will come along and mess it up.
		// So we want to patch up the constraint while acknowledging the accumulated impulse and the damaged velocity.
		// To help with that we use P for the accumulated impulse and lambda as the update. Mathematically we have:
		//
		// M* (v2new - v2damaged) = J^T* lambda
		// J* v2new + softness* (total_lambda + lambda) + b = 0
		//
		// If we solve this we get:
		//
		// v2new = v2damaged + M^-1* J^T* lambda
		// J* (v2damaged + M^-1* J^T* lambda) + softness* total_lambda + softness* lambda + b = 0
		//
		// (J* M^-1* J^T + softness)* lambda = -(J* v2damaged + softness* total_lambda + b)
		//
		// So our lagrange multiplier becomes:
		//
		// lambda = -K^-1 (J v + softness* total_lambda + b)
		//
		// So we return the bias: softness* total_lambda + b
		return mSoftness* inTotalLambda + mBias;
	}

private:
	float						mBias  = 0.0f;
	float						mSoftness  = 0.0f;
};


class SwingTwistConstraintPart
{
public:
	/// Override the swing type
	void SetSwingType(ESwingType inSwingType) {
		mSwingType = inSwingType;
	}

	/// Get the swing type for this part
	ESwingType GetSwingType() const {
		return mSwingType;
	}

	/// Set limits for this constraint (see description above for parameters)
	void SetLimits(float inTwistMinAngle, float inTwistMaxAngle, float inSwingYMinAngle, float inSwingYMaxAngle, float inSwingZMinAngle, float inSwingZMaxAngle) {
		constexpr float cLockedAngle = DegreesToRadians(0.5f);
		constexpr float cFreeAngle = DegreesToRadians(179.5f);

		// Assume sane input
		MOSS_ASSERT(inTwistMinAngle <= inTwistMaxAngle);
		MOSS_ASSERT(inSwingYMinAngle <= inSwingYMaxAngle);
		MOSS_ASSERT(inSwingZMinAngle <= inSwingZMaxAngle);
		MOSS_ASSERT(inSwingYMinAngle >= -MOSS_PI& & inSwingYMaxAngle <= MOSS_PI);
		MOSS_ASSERT(inSwingZMinAngle >= -MOSS_PI& & inSwingZMaxAngle <= MOSS_PI);

		// Calculate the sine and cosine of the half angles
		Vec4 half_twist = 0.5f* Vec4(inTwistMinAngle, inTwistMaxAngle, 0, 0);
		Vec4 twist_s, twist_c;
		half_twist.SinCos(twist_s, twist_c);
		Vec4 half_swing = 0.5f* Vec4(inSwingYMinAngle, inSwingYMaxAngle, inSwingZMinAngle, inSwingZMaxAngle);
		Vec4 swing_s, swing_c;
		half_swing.SinCos(swing_s, swing_c);

		// Store half angles for pyramid limit
		mSwingYHalfMinAngle = half_swing.GetX();
		mSwingYHalfMaxAngle = half_swing.GetY();
		mSwingZHalfMinAngle = half_swing.GetZ();
		mSwingZHalfMaxAngle = half_swing.GetW();

		// Store axis flags which are used at runtime to quickly decided which constraints to apply
		mRotationFlags = 0;
		if (inTwistMinAngle > -cLockedAngle& & inTwistMaxAngle < cLockedAngle) {
			mRotationFlags |= TwistXLocked;
			mSinTwistHalfMinAngle = 0.0f;
			mSinTwistHalfMaxAngle = 0.0f;
			mCosTwistHalfMinAngle = 1.0f;
			mCosTwistHalfMaxAngle = 1.0f;
		}
		else if (inTwistMinAngle < -cFreeAngle& & inTwistMaxAngle > cFreeAngle) {
			mRotationFlags |= TwistXFree;
			mSinTwistHalfMinAngle = -1.0f;
			mSinTwistHalfMaxAngle = 1.0f;
			mCosTwistHalfMinAngle = 0.0f;
			mCosTwistHalfMaxAngle = 0.0f;
		}
		else {
			mSinTwistHalfMinAngle = twist_s.GetX();
			mSinTwistHalfMaxAngle = twist_s.GetY();
			mCosTwistHalfMinAngle = twist_c.GetX();
			mCosTwistHalfMaxAngle = twist_c.GetY();
		}

		if (inSwingYMinAngle > -cLockedAngle& & inSwingYMaxAngle < cLockedAngle) {
			mRotationFlags |= SwingYLocked;
			mSinSwingYHalfMinAngle = 0.0f;
			mSinSwingYHalfMaxAngle = 0.0f;
			mCosSwingYHalfMinAngle = 1.0f;
			mCosSwingYHalfMaxAngle = 1.0f;
		}
		else if (inSwingYMinAngle < -cFreeAngle& & inSwingYMaxAngle > cFreeAngle) {
			mRotationFlags |= SwingYFree;
			mSinSwingYHalfMinAngle = -1.0f;
			mSinSwingYHalfMaxAngle = 1.0f;
			mCosSwingYHalfMinAngle = 0.0f;
			mCosSwingYHalfMaxAngle = 0.0f;
		}
		else {
			mSinSwingYHalfMinAngle = swing_s.GetX();
			mSinSwingYHalfMaxAngle = swing_s.GetY();
			mCosSwingYHalfMinAngle = swing_c.GetX();
			mCosSwingYHalfMaxAngle = swing_c.GetY();
			MOSS_ASSERT(mSinSwingYHalfMinAngle <= mSinSwingYHalfMaxAngle);
		}

		if (inSwingZMinAngle > -cLockedAngle& & inSwingZMaxAngle < cLockedAngle) {
			mRotationFlags |= SwingZLocked;
			mSinSwingZHalfMinAngle = 0.0f;
			mSinSwingZHalfMaxAngle = 0.0f;
			mCosSwingZHalfMinAngle = 1.0f;
			mCosSwingZHalfMaxAngle = 1.0f;
		}
		else if (inSwingZMinAngle < -cFreeAngle& & inSwingZMaxAngle > cFreeAngle) {
			mRotationFlags |= SwingZFree;
			mSinSwingZHalfMinAngle = -1.0f;
			mSinSwingZHalfMaxAngle = 1.0f;
			mCosSwingZHalfMinAngle = 0.0f;
			mCosSwingZHalfMaxAngle = 0.0f;
		}
		else {
			mSinSwingZHalfMinAngle = swing_s.GetZ();
			mSinSwingZHalfMaxAngle = swing_s.GetW();
			mCosSwingZHalfMinAngle = swing_c.GetZ();
			mCosSwingZHalfMaxAngle = swing_c.GetW();
			MOSS_ASSERT(mSinSwingZHalfMinAngle <= mSinSwingZHalfMaxAngle);
		}
	}

	/// Flags to indicate which axis got clamped by ClampSwingTwist
	static constexpr uint		cClampedTwistMin = 1 << 0;
	static constexpr uint		cClampedTwistMax = 1 << 1;
	static constexpr uint		cClampedSwingYMin = 1 << 2;
	static constexpr uint		cClampedSwingYMax = 1 << 3;
	static constexpr uint		cClampedSwingZMin = 1 << 4;
	static constexpr uint		cClampedSwingZMax = 1 << 5;

	/// Helper function to determine if we're clamped against the min or max limit
	static MOSS_INLINE bool		sDistanceToMinShorter(float inDeltaMin, float inDeltaMax) {
		// We're outside of the limits, get actual delta to min/max range
		// Note that a swing/twist of -1 and 1 represent the same angle, so if the difference is bigger than 1, the shortest angle is the other way around (2 - difference)
		// We should actually be working with angles rather than sin(angle / 2). When the difference is small the approximation is accurate, but
		// when working with extreme values the calculation is off and e.g. when the limit is between 0 and 180 a value of approx -60 will clamp
		// to 180 rather than 0 (you'd expect anything > -90 to go to 0).
		inDeltaMin = abs(inDeltaMin);
		if (inDeltaMin > 1.0f) inDeltaMin = 2.0f - inDeltaMin;
		inDeltaMax = abs(inDeltaMax);
		if (inDeltaMax > 1.0f) inDeltaMax = 2.0f - inDeltaMax;
		return inDeltaMin < inDeltaMax;
	}

	/// Clamp twist and swing against the constraint limits, returns which parts were clamped (everything assumed in constraint space)
	inline void					ClampSwingTwist(Quat& ioSwing, Quat& ioTwist, uint& outClampedAxis) const {
		// Start with not clamped
		outClampedAxis = 0;

		// Check that swing and twist quaternions don't contain rotations around the wrong axis
		MOSS_ASSERT(ioSwing.GetX() == 0.0f);
		MOSS_ASSERT(ioTwist.GetY() == 0.0f);
		MOSS_ASSERT(ioTwist.GetZ() == 0.0f);

		// Ensure quaternions have w > 0
		bool negate_swing = ioSwing.GetW() < 0.0f;
		if (negate_swing)
			ioSwing = -ioSwing;
		bool negate_twist = ioTwist.GetW() < 0.0f;
		if (negate_twist)
			ioTwist = -ioTwist;

		if (mRotationFlags&  TwistXLocked) {
			// Twist axis is locked, clamp whenever twist is not identity
			outClampedAxis |= ioTwist.GetX() != 0.0f? (cClampedTwistMin | cClampedTwistMax) : 0;
			ioTwist = Quat::sIdentity();
		}
		else if ((mRotationFlags&  TwistXFree) == 0) {
			// Twist axis has limit, clamp whenever out of range
			float delta_min = mSinTwistHalfMinAngle - ioTwist.GetX();
			float delta_max = ioTwist.GetX() - mSinTwistHalfMaxAngle;
			if (delta_min > 0.0f || delta_max > 0.0f) {
				// Pick the twist that corresponds to the smallest delta
				if (sDistanceToMinShorter(delta_min, delta_max)) {
					ioTwist = Quat(mSinTwistHalfMinAngle, 0, 0, mCosTwistHalfMinAngle);
					outClampedAxis |= cClampedTwistMin;
				}
				else {
					ioTwist = Quat(mSinTwistHalfMaxAngle, 0, 0, mCosTwistHalfMaxAngle);
					outClampedAxis |= cClampedTwistMax;
				}
			}
		}

		// Clamp swing
		if (mRotationFlags&  SwingYLocked) {
			if (mRotationFlags&  SwingZLocked) {
				// Both swing Y and Z are disabled, no degrees of freedom in swing
				outClampedAxis |= ioSwing.GetY() != 0.0f? (cClampedSwingYMin | cClampedSwingYMax) : 0;
				outClampedAxis |= ioSwing.GetZ() != 0.0f? (cClampedSwingZMin | cClampedSwingZMax) : 0;
				ioSwing = Quat::sIdentity();
			}
			else {
				// Swing Y angle disabled, only 1 degree of freedom in swing
				outClampedAxis |= ioSwing.GetY() != 0.0f? (cClampedSwingYMin | cClampedSwingYMax) : 0;
				float delta_min = mSinSwingZHalfMinAngle - ioSwing.GetZ();
				float delta_max = ioSwing.GetZ() - mSinSwingZHalfMaxAngle;
				if (delta_min > 0.0f || delta_max > 0.0f) {
					// Pick the swing that corresponds to the smallest delta
					if (sDistanceToMinShorter(delta_min, delta_max)) {
						ioSwing = Quat(0, 0, mSinSwingZHalfMinAngle, mCosSwingZHalfMinAngle);
						outClampedAxis |= cClampedSwingZMin;
					}
					else {
						ioSwing = Quat(0, 0, mSinSwingZHalfMaxAngle, mCosSwingZHalfMaxAngle);
						outClampedAxis |= cClampedSwingZMax;
					}
				}
				else if ((outClampedAxis&  cClampedSwingYMin) != 0) {
					float z = ioSwing.GetZ();
					ioSwing = Quat(0, 0, z, sqrt(1.0f - Square(z)));
				}
			}
		}
		else if (mRotationFlags & SwingZLocked) {
			// Swing Z angle disabled, only 1 degree of freedom in swing
			outClampedAxis |= ioSwing.GetZ() != 0.0f? (cClampedSwingZMin | cClampedSwingZMax) : 0;
			float delta_min = mSinSwingYHalfMinAngle - ioSwing.GetY();
			float delta_max = ioSwing.GetY() - mSinSwingYHalfMaxAngle;
			if (delta_min > 0.0f || delta_max > 0.0f) {
				// Pick the swing that corresponds to the smallest delta
				if (sDistanceToMinShorter(delta_min, delta_max)) {
					ioSwing = Quat(0, mSinSwingYHalfMinAngle, 0, mCosSwingYHalfMinAngle);
					outClampedAxis |= cClampedSwingYMin;
				}
				else {
					ioSwing = Quat(0, mSinSwingYHalfMaxAngle, 0, mCosSwingYHalfMaxAngle);
					outClampedAxis |= cClampedSwingYMax;
				}
			}
			else if ((outClampedAxis & cClampedSwingZMin) != 0) {
				float y = ioSwing.GetY();
				ioSwing = Quat(0, y, 0, sqrt(1.0f - Square(y)));
			}
		}
		else {
			// Two degrees of freedom
			if (mSwingType == ESwingType::Cone) {
				// Use ellipse to solve limits
				Ellipsee ellipse(mSinSwingYHalfMaxAngle, mSinSwingZHalfMaxAngle);
				Float2 point(ioSwing.GetY(), ioSwing.GetZ());
				if (!ellipse.IsInside(point)) {
					Float2 closest = ellipse.GetClosestPoint(point);
					ioSwing = Quat(0, closest.x, closest.y, sqrt(max(0.0f, 1.0f - Square(closest.x) - Square(closest.y))));
					outClampedAxis |= cClampedSwingYMin | cClampedSwingYMax | cClampedSwingZMin | cClampedSwingZMax; // We're not using the flags on which side we got clamped here
				}
			}
			else {
				// Use pyramid to solve limits
				// The quaternion rotating by angle y around the Y axis then rotating by angle z around the Z axis is:
				// q = Quat::sRotation(Vec3::sAxisZ(), z)* Quat::sRotation(Vec3::sAxisY(), y)
				// [q.x, q.y, q.z, q.w] = [-sin(y / 2)* sin(z / 2), sin(y / 2)* cos(z / 2), cos(y / 2)* sin(z / 2), cos(y / 2)* cos(z / 2)]
				// So we can calculate y / 2 = atan2(q.y, q.w) and z / 2 = atan2(q.z, q.w)
				Vec4 half_angle = Vec4::sATan2(ioSwing.GetXYZW().Swizzle<SWIZZLE_Y, SWIZZLE_Y, SWIZZLE_Z, SWIZZLE_Z>(), ioSwing.GetXYZW().SplatW());
				Vec4 min_half_angle(mSwingYHalfMinAngle, mSwingYHalfMinAngle, mSwingZHalfMinAngle, mSwingZHalfMinAngle);
				Vec4 max_half_angle(mSwingYHalfMaxAngle, mSwingYHalfMaxAngle, mSwingZHalfMaxAngle, mSwingZHalfMaxAngle);
				Vec4 clamped_half_angle = Vec4::sMin(Vec4::sMax(half_angle, min_half_angle), max_half_angle);
				UVec4 unclamped = Vec4::sEquals(half_angle, clamped_half_angle);
				if (!unclamped.TestAllTrue()) {
					// We now calculate the quaternion again using the formula for q above,
					// but we leave out the x component in order to not introduce twist
					Vec4 s, c;
					clamped_half_angle.SinCos(s, c);
					ioSwing = Quat(0, s.GetY()* c.GetZ(), c.GetY()* s.GetZ(), c.GetY()* c.GetZ()).Normalized();
					outClampedAxis |= cClampedSwingYMin | cClampedSwingYMax | cClampedSwingZMin | cClampedSwingZMax; // We're not using the flags on which side we got clamped here
				}
			}
		}

		// Flip sign back
		if (negate_swing)
			ioSwing = -ioSwing;
		if (negate_twist)
			ioTwist = -ioTwist;

		MOSS_ASSERT(ioSwing.IsNormalized());
		MOSS_ASSERT(ioTwist.IsNormalized());
	}

	/// Calculate properties used during the functions below
	/// @param inBody1 The first body that this constraint is attached to
	/// @param inBody2 The second body that this constraint is attached to
	/// @param inConstraintRotation The current rotation of the constraint in constraint space
	/// @param inConstraintToWorld Rotates from constraint space into world space
	inline void	CalculateConstraintProperties(const Body& inBody1, const Body& inBody2, QuatArg inConstraintRotation, QuatArg inConstraintToWorld) {
		// Decompose into swing and twist
		Quat q_swing, q_twist;
		inConstraintRotation.GetSwingTwist(q_swing, q_twist);

		// Clamp against joint limits
		Quat q_clamped_swing = q_swing, q_clamped_twist = q_twist;
		uint clamped_axis;
		ClampSwingTwist(q_clamped_swing, q_clamped_twist, clamped_axis);

		if (mRotationFlags&  SwingYLocked) {
			Quat twist_to_world = inConstraintToWorld* q_swing;
			mWorldSpaceSwingLimitYRotationAxis = twist_to_world.RotateAxisY();
			mWorldSpaceSwingLimitZRotationAxis = twist_to_world.RotateAxisZ();

			if (mRotationFlags&  SwingZLocked) {
				// Swing fully locked
				mSwingLimitYConstraintPart.CalculateConstraintProperties(inBody1, inBody2, mWorldSpaceSwingLimitYRotationAxis);
				mSwingLimitZConstraintPart.CalculateConstraintProperties(inBody1, inBody2, mWorldSpaceSwingLimitZRotationAxis);
			}
			else {
				// Swing only locked around Y
				mSwingLimitYConstraintPart.CalculateConstraintProperties(inBody1, inBody2, mWorldSpaceSwingLimitYRotationAxis);
				if ((clamped_axis&  (cClampedSwingZMin | cClampedSwingZMax)) != 0) {
					if ((clamped_axis&  cClampedSwingZMin) != 0)
						mWorldSpaceSwingLimitZRotationAxis = -mWorldSpaceSwingLimitZRotationAxis; // Flip axis if hitting min limit because the impulse limit is going to be between [-FLT_MAX, 0]
					mSwingLimitZConstraintPart.CalculateConstraintProperties(inBody1, inBody2, mWorldSpaceSwingLimitZRotationAxis);
				}
				else
					mSwingLimitZConstraintPart.Deactivate();
			}
		}
		else if (mRotationFlags&  SwingZLocked) {
			// Swing only locked around Z
			Quat twist_to_world = inConstraintToWorld* q_swing;
			mWorldSpaceSwingLimitYRotationAxis = twist_to_world.RotateAxisY();
			mWorldSpaceSwingLimitZRotationAxis = twist_to_world.RotateAxisZ();

			if ((clamped_axis&  (cClampedSwingYMin | cClampedSwingYMax)) != 0) {
				if ((clamped_axis&  cClampedSwingYMin) != 0)
					mWorldSpaceSwingLimitYRotationAxis = -mWorldSpaceSwingLimitYRotationAxis; // Flip axis if hitting min limit because the impulse limit is going to be between [-FLT_MAX, 0]
				mSwingLimitYConstraintPart.CalculateConstraintProperties(inBody1, inBody2, mWorldSpaceSwingLimitYRotationAxis);
			}
			else
				mSwingLimitYConstraintPart.Deactivate();
			mSwingLimitZConstraintPart.CalculateConstraintProperties(inBody1, inBody2, mWorldSpaceSwingLimitZRotationAxis);
		}
		else if ((mRotationFlags&  SwingYZFree) != SwingYZFree) {
			// Swing has limits around Y and Z
			if ((clamped_axis&  (cClampedSwingYMin | cClampedSwingYMax | cClampedSwingZMin | cClampedSwingZMax)) != 0)
			{
				// Calculate axis of rotation from clamped swing to swing
				Vec3 current = (inConstraintToWorld* q_swing).RotateAxisX();
				Vec3 desired = (inConstraintToWorld* q_clamped_swing).RotateAxisX();
				mWorldSpaceSwingLimitYRotationAxis = desired.Cross(current);
				float len = mWorldSpaceSwingLimitYRotationAxis.Length();
				if (len != 0.0f)
				{
					mWorldSpaceSwingLimitYRotationAxis /= len;
					mSwingLimitYConstraintPart.CalculateConstraintProperties(inBody1, inBody2, mWorldSpaceSwingLimitYRotationAxis);
				}
				else
					mSwingLimitYConstraintPart.Deactivate();
			}
			else
				mSwingLimitYConstraintPart.Deactivate();
			mSwingLimitZConstraintPart.Deactivate();
		}
		else {
			// No swing limits
			mSwingLimitYConstraintPart.Deactivate();
			mSwingLimitZConstraintPart.Deactivate();
		}

		if (mRotationFlags&  TwistXLocked) {
			// Twist locked, always activate constraint
			mWorldSpaceTwistLimitRotationAxis = (inConstraintToWorld* q_swing).RotateAxisX();
			mTwistLimitConstraintPart.CalculateConstraintProperties(inBody1, inBody2, mWorldSpaceTwistLimitRotationAxis);
		}
		else if ((mRotationFlags&  TwistXFree) == 0) {
			// Twist has limits
			if ((clamped_axis&  (cClampedTwistMin | cClampedTwistMax)) != 0) {
				mWorldSpaceTwistLimitRotationAxis = (inConstraintToWorld* q_swing).RotateAxisX();
				if ((clamped_axis&  cClampedTwistMin) != 0)
					mWorldSpaceTwistLimitRotationAxis = -mWorldSpaceTwistLimitRotationAxis; // Flip axis if hitting min limit because the impulse limit is going to be between [-FLT_MAX, 0]
				mTwistLimitConstraintPart.CalculateConstraintProperties(inBody1, inBody2, mWorldSpaceTwistLimitRotationAxis);
			}
			else
				mTwistLimitConstraintPart.Deactivate();
		}
		else {
			// No twist limits
			mTwistLimitConstraintPart.Deactivate();
		}
	}

	/// Deactivate this constraint
	void Deactivate() {
		mSwingLimitYConstraintPart.Deactivate();
		mSwingLimitZConstraintPart.Deactivate();
		mTwistLimitConstraintPart.Deactivate();
	}

	/// Check if constraint is active
	inline bool IsActive() const {
		return mSwingLimitYConstraintPart.IsActive() || mSwingLimitZConstraintPart.IsActive() || mTwistLimitConstraintPart.IsActive();
	}

	/// Must be called from the WarmStartVelocityConstraint call to apply the previous frame's impulses
	inline void	WarmStart(Body& ioBody1, Body& ioBody2, float inWarmStartImpulseRatio) {
		mSwingLimitYConstraintPart.WarmStart(ioBody1, ioBody2, inWarmStartImpulseRatio);
		mSwingLimitZConstraintPart.WarmStart(ioBody1, ioBody2, inWarmStartImpulseRatio);
		mTwistLimitConstraintPart.WarmStart(ioBody1, ioBody2, inWarmStartImpulseRatio);
	}

	/// Iteratively update the velocity constraint. Makes sure d/dt C(...) = 0, where C is the constraint equation.
	inline bool SolveVelocityConstraint(Body& ioBody1, Body& ioBody2) {
		bool impulse = false;

		// Solve swing constraint
		if (mSwingLimitYConstraintPart.IsActive())
			impulse |= mSwingLimitYConstraintPart.SolveVelocityConstraint(ioBody1, ioBody2, mWorldSpaceSwingLimitYRotationAxis, -FLT_MAX, mSinSwingYHalfMinAngle == mSinSwingYHalfMaxAngle? FLT_MAX : 0.0f);

		if (mSwingLimitZConstraintPart.IsActive())
			impulse |= mSwingLimitZConstraintPart.SolveVelocityConstraint(ioBody1, ioBody2, mWorldSpaceSwingLimitZRotationAxis, -FLT_MAX, mSinSwingZHalfMinAngle == mSinSwingZHalfMaxAngle? FLT_MAX : 0.0f);

		// Solve twist constraint
		if (mTwistLimitConstraintPart.IsActive())
			impulse |= mTwistLimitConstraintPart.SolveVelocityConstraint(ioBody1, ioBody2, mWorldSpaceTwistLimitRotationAxis, -FLT_MAX, mSinTwistHalfMinAngle == mSinTwistHalfMaxAngle? FLT_MAX : 0.0f);

		return impulse;
	}

	/// Iteratively update the position constraint. Makes sure C(...) = 0.
	/// @param ioBody1 The first body that this constraint is attached to
	/// @param ioBody2 The second body that this constraint is attached to
	/// @param inConstraintRotation The current rotation of the constraint in constraint space
	/// @param inConstraintToBody1 , inConstraintToBody2 Rotates from constraint space to body 1/2 space
	/// @param inBaumgarte Baumgarte constant (fraction of the error to correct)
	inline bool SolvePositionConstraint(Body& ioBody1, Body& ioBody2, QuatArg inConstraintRotation, QuatArg inConstraintToBody1, QuatArg inConstraintToBody2, float inBaumgarte) const {
		Quat q_swing, q_twist;
		inConstraintRotation.GetSwingTwist(q_swing, q_twist);

		uint clamped_axis;
		ClampSwingTwist(q_swing, q_twist, clamped_axis);

		// Solve rotation violations
		if (clamped_axis != 0) {
			RotationEulerConstraintPart part;
			Quat inv_initial_orientation = inConstraintToBody2* (inConstraintToBody1* q_swing* q_twist).Conjugated();
			part.CalculateConstraintProperties(ioBody1, Mat44::sRotation(ioBody1.GetRotation()), ioBody2, Mat44::sRotation(ioBody2.GetRotation()));
			return part.SolvePositionConstraint(ioBody1, ioBody2, inv_initial_orientation, inBaumgarte);
		}

		return false;
	}

	/// Return lagrange multiplier for swing
	inline float GetTotalSwingYLambda() const {
		return mSwingLimitYConstraintPart.GetTotalLambda();
	}

	inline float GetTotalSwingZLambda() const {
		return mSwingLimitZConstraintPart.GetTotalLambda();
	}

	/// Return lagrange multiplier for twist
	inline float GetTotalTwistLambda() const {
		return mTwistLimitConstraintPart.GetTotalLambda();
	}

	/// Save state of this constraint part
	void SaveState(StateRecorder& inStream) const {
		mSwingLimitYConstraintPart.SaveState(inStream);
		mSwingLimitZConstraintPart.SaveState(inStream);
		mTwistLimitConstraintPart.SaveState(inStream);
	}

	/// Restore state of this constraint part
	void RestoreState(StateRecorder& inStream) {
		mSwingLimitYConstraintPart.RestoreState(inStream);
		mSwingLimitZConstraintPart.RestoreState(inStream);
		mTwistLimitConstraintPart.RestoreState(inStream);
	}

private:
	// CONFIGURATION PROPERTIES FOLLOW

	enum ERotationFlags {
		/// Indicates that axis is completely locked (cannot rotate around this axis)
		TwistXLocked			= 1 << 0,
		SwingYLocked			= 1 << 1,
		SwingZLocked			= 1 << 2,

		/// Indicates that axis is completely free (can rotate around without limits)
		TwistXFree				= 1 << 3,
		SwingYFree				= 1 << 4,
		SwingZFree				= 1 << 5,
		SwingYZFree				= SwingYFree | SwingZFree
	};

	uint8						mRotationFlags;

	// Constants
	ESwingType					mSwingType = ESwingType::Cone;
	float						mSinTwistHalfMinAngle;
	float						mSinTwistHalfMaxAngle;
	float						mCosTwistHalfMinAngle;
	float						mCosTwistHalfMaxAngle;
	float						mSwingYHalfMinAngle;
	float						mSwingYHalfMaxAngle;
	float						mSwingZHalfMinAngle;
	float						mSwingZHalfMaxAngle;
	float						mSinSwingYHalfMinAngle;
	float						mSinSwingYHalfMaxAngle;
	float						mSinSwingZHalfMinAngle;
	float						mSinSwingZHalfMaxAngle;
	float						mCosSwingYHalfMinAngle;
	float						mCosSwingYHalfMaxAngle;
	float						mCosSwingZHalfMinAngle;
	float						mCosSwingZHalfMaxAngle;

	// RUN TIME PROPERTIES FOLLOW

	/// Rotation axis for the angle constraint parts
	Vec3						mWorldSpaceSwingLimitYRotationAxis;
	Vec3						mWorldSpaceSwingLimitZRotationAxis;
	Vec3						mWorldSpaceTwistLimitRotationAxis;

	/// The constraint parts
	AngleConstraintPart			mSwingLimitYConstraintPart;
	AngleConstraintPart			mSwingLimitZConstraintPart;
	AngleConstraintPart			mTwistLimitConstraintPart;
};


class RackAndPinionConstraintPart {
	/// Internal helper function to update velocities of bodies after Lagrange multiplier is calculated
	MOSS_INLINE bool ApplyVelocityStep(Body& ioBody1, Body& ioBody2, float inLambda) const {
		// Apply impulse if delta is not zero
		if (inLambda != 0.0f) {
			// Calculate velocity change due to constraint
			//
			// Impulse:
			// P = J^T lambda
			//
			// Euler velocity integration:
			// v' = v + M^-1 P
			ioBody1.GetMotionProperties()->AddAngularVelocityStep(inLambda* mInvI1_A);
			ioBody2.GetMotionProperties()->SubLinearVelocityStep(inLambda* mRatio_InvM2_B);
			return true;
		}

		return false;
	}

public:
	/// Calculate properties used during the functions below
	/// @param inBody1 The first body that this constraint is attached to
	/// @param inBody2 The second body that this constraint is attached to
	/// @param inWorldSpaceHingeAxis The axis around which body 1 rotates
	/// @param inWorldSpaceSliderAxis The axis along which body 2 slides
	/// @param inRatio The ratio between rotation and translation
	inline void CalculateConstraintProperties(const Body& inBody1, Vec3Arg inWorldSpaceHingeAxis, const Body& inBody2, Vec3Arg inWorldSpaceSliderAxis, float inRatio) {
		MOSS_ASSERT(inWorldSpaceHingeAxis.IsNormalized(1.0e-4f));
		MOSS_ASSERT(inWorldSpaceSliderAxis.IsNormalized(1.0e-4f));

		// Calculate: I1^-1 a
		mInvI1_A = inBody1.GetMotionProperties()->MultiplyWorldSpaceInverseInertiaByVector(inBody1.GetRotation(), inWorldSpaceHingeAxis);

		// Calculate: r/m2 b
		float inv_m2 = inBody2.GetMotionProperties()->GetInverseMass();
		mRatio_InvM2_B = inRatio* inv_m2* inWorldSpaceSliderAxis;

		// K^-1 = 1 / (J M^-1 J^T) = 1 / (a^T I1^-1 a + 1/m2* r^2* b . b)
		float inv_effective_mass = (inWorldSpaceHingeAxis.Dot(mInvI1_A) + inv_m2* Square(inRatio));
		if (inv_effective_mass == 0.0f)
			Deactivate();
		else
			mEffectiveMass = 1.0f / inv_effective_mass;
	}

	/// Deactivate this constraint
	inline void Deactivate() {
		mEffectiveMass = 0.0f;
		mTotalLambda = 0.0f;
	}

	/// Check if constraint is active
	inline bool	IsActive() const {
		return mEffectiveMass != 0.0f;
	}

	/// Must be called from the WarmStartVelocityConstraint call to apply the previous frame's impulses
	/// @param ioBody1 The first body that this constraint is attached to
	/// @param ioBody2 The second body that this constraint is attached to
	/// @param inWarmStartImpulseRatio Ratio of new step to old time step (dt_new / dt_old) for scaling the lagrange multiplier of the previous frame
	inline void	WarmStart(Body& ioBody1, Body& ioBody2, float inWarmStartImpulseRatio) {
		mTotalLambda*= inWarmStartImpulseRatio;
		ApplyVelocityStep(ioBody1, ioBody2, mTotalLambda);
	}

	/// Iteratively update the velocity constraint. Makes sure d/dt C(...) = 0, where C is the constraint equation.
	/// @param ioBody1 The first body that this constraint is attached to
	/// @param ioBody2 The second body that this constraint is attached to
	/// @param inWorldSpaceHingeAxis The axis around which body 1 rotates
	/// @param inWorldSpaceSliderAxis The axis along which body 2 slides
	/// @param inRatio The ratio between rotation and translation
	inline bool SolveVelocityConstraint(Body& ioBody1, Vec3Arg inWorldSpaceHingeAxis, Body& ioBody2, Vec3Arg inWorldSpaceSliderAxis, float inRatio) {
		// Lagrange multiplier is:
		//
		// lambda = -K^-1 (J v + b)
		float lambda = mEffectiveMass* (inRatio* inWorldSpaceSliderAxis.Dot(ioBody2.GetLinearVelocity()) - inWorldSpaceHingeAxis.Dot(ioBody1.GetAngularVelocity()));
		mTotalLambda += lambda; // Store accumulated impulse

		return ApplyVelocityStep(ioBody1, ioBody2, lambda);
	}

	/// Return lagrange multiplier
	float GetTotalLambda() const { return mTotalLambda; }

	/// Iteratively update the position constraint. Makes sure C(...) == 0.
	/// @param ioBody1 The first body that this constraint is attached to
	/// @param ioBody2 The second body that this constraint is attached to
	/// @param inC Value of the constraint equation (C)
	/// @param inBaumgarte Baumgarte constant (fraction of the error to correct)
	inline bool SolvePositionConstraint(Body& ioBody1, Body& ioBody2, float inC, float inBaumgarte) const {
		// Only apply position constraint when the constraint is hard, otherwise the velocity bias will fix the constraint
		if (inC != 0.0f) {
			// Calculate lagrange multiplier (lambda) for Baumgarte stabilization:
			//
			// lambda = -K^-1* beta / dt* C
			//
			// We should divide by inDeltaTime, but we should multiply by inDeltaTime in the Euler step below so they're cancelled out
			float lambda = -mEffectiveMass* inBaumgarte* inC;

			// Directly integrate velocity change for one time step
			//
			// Euler velocity integration:
			// dv = M^-1 P
			//
			// Impulse:
			// P = J^T lambda
			//
			// Euler position integration:
			// x' = x + dv* dt
			//
			// Note we don't accumulate velocities for the stabilization. This is using the approach described in 'Modeling and
			// Solving Constraints' by Erin Catto presented at GDC 2007. On slide 78 it is suggested to split up the Baumgarte
			// stabilization for positional drift so that it does not actually add to the momentum. We combine an Euler velocity
			// integrate + a position integrate and then discard the velocity change.
			if (ioBody1.IsDynamic())
				ioBody1.AddRotationStep(lambda* mInvI1_A);
			if (ioBody2.IsDynamic())
				ioBody2.SubPositionStep(lambda* mRatio_InvM2_B);
			return true;
		}

		return false;
	}

	/// Save state of this constraint part
	void SaveState(StateRecorder& inStream) const {
		inStream.Write(mTotalLambda);
	}

	/// Restore state of this constraint part
	void RestoreState(StateRecorder& inStream) {
		inStream.Read(mTotalLambda);
	}

private:
	Vec3						mInvI1_A;
	Vec3						mRatio_InvM2_B;
	float						mEffectiveMass = 0.0f;
	float						mTotalLambda = 0.0f;
};

class RotationEulerConstraintPart {
private:
	/// Internal helper function to update velocities of bodies after Lagrange multiplier is calculated
	MOSS_INLINE bool ApplyVelocityStep(Body& ioBody1, Body& ioBody2, Vec3Arg inLambda) const {
		// Apply impulse if delta is not zero
		if (inLambda != Vec3::sZero())
		{
			// Calculate velocity change due to constraint
			//
			// Impulse:
			// P = J^T lambda
			//
			// Euler velocity integration:
			// v' = v + M^-1 P
			if (ioBody1.IsDynamic())
				ioBody1.GetMotionProperties()->SubAngularVelocityStep(mInvI1.Multiply3x3(inLambda));
			if (ioBody2.IsDynamic())
				ioBody2.GetMotionProperties()->AddAngularVelocityStep(mInvI2.Multiply3x3(inLambda));
			return true;
		}
		return false;
	}

public:
	/// Return inverse of initial rotation from body 1 to body 2 in body 1 space
	static Quat sGetInvInitialOrientation(const Body& inBody1, const Body& inBody2) {
		// q20 = q10 r0
		// <=> r0 = q10^-1 q20
		// <=> r0^-1 = q20^-1 q10
		//
		// where:
		//
		// q20 = initial orientation of body 2
		// q10 = initial orientation of body 1
		// r0 = initial rotation from body 1 to body 2
		return inBody2.GetRotation().Conjugated()* inBody1.GetRotation();
	}

	/// @brief Return inverse of initial rotation from body 1 to body 2 in body 1 space
	/// @param inAxisX1 Reference axis X for body 1
	/// @param inAxisY1 Reference axis Y for body 1
	/// @param inAxisX2 Reference axis X for body 2
	/// @param inAxisY2 Reference axis Y for body 2
	static Quat sGetInvInitialOrientationXY(Vec3Arg inAxisX1, Vec3Arg inAxisY1, Vec3Arg inAxisX2, Vec3Arg inAxisY2) {
		// Store inverse of initial rotation from body 1 to body 2 in body 1 space:
		//
		// q20 = q10 r0
		// <=> r0 = q10^-1 q20
		// <=> r0^-1 = q20^-1 q10
		//
		// where:
		//
		// q10, q20 = world space initial orientation of body 1 and 2
		// r0 = initial rotation from body 1 to body 2 in local space of body 1
		//
		// We can also write this in terms of the constraint matrices:
		//
		// q20 c2 = q10 c1
		// <=> q20 = q10 c1 c2^-1
		// => r0 = c1 c2^-1
		// <=> r0^-1 = c2 c1^-1
		//
		// where:
		//
		// c1, c2 = matrix that takes us from body 1 and 2 COM to constraint space 1 and 2
		if (inAxisX1 == inAxisX2& & inAxisY1 == inAxisY2) {
			// Axis are the same -> identity transform
			return Quat::sIdentity();
		}
		else {
			Mat44 constraint1(Vec4(inAxisX1, 0), Vec4(inAxisY1, 0), Vec4(inAxisX1.Cross(inAxisY1), 0), Vec4(0, 0, 0, 1));
			Mat44 constraint2(Vec4(inAxisX2, 0), Vec4(inAxisY2, 0), Vec4(inAxisX2.Cross(inAxisY2), 0), Vec4(0, 0, 0, 1));
			return constraint2.GetQuaternion()* constraint1.GetQuaternion().Conjugated();
		}
	}

	/// @brief Return inverse of initial rotation from body 1 to body 2 in body 1 space
	/// @param inAxisX1 Reference axis X for body 1
	/// @param inAxisZ1 Reference axis Z for body 1
	/// @param inAxisX2 Reference axis X for body 2
	/// @param inAxisZ2 Reference axis Z for body 2
	static Quat sGetInvInitialOrientationXZ(Vec3Arg inAxisX1, Vec3Arg inAxisZ1, Vec3Arg inAxisX2, Vec3Arg inAxisZ2) {
		// See comment at sGetInvInitialOrientationXY
		if (inAxisX1 == inAxisX2& & inAxisZ1 == inAxisZ2) {
			return Quat::sIdentity();
		}
		else {
			Mat44 constraint1(Vec4(inAxisX1, 0), Vec4(inAxisZ1.Cross(inAxisX1), 0), Vec4(inAxisZ1, 0), Vec4(0, 0, 0, 1));
			Mat44 constraint2(Vec4(inAxisX2, 0), Vec4(inAxisZ2.Cross(inAxisX2), 0), Vec4(inAxisZ2, 0), Vec4(0, 0, 0, 1));
			return constraint2.GetQuaternion()* constraint1.GetQuaternion().Conjugated();
		}
	}

	/// Calculate properties used during the functions below
	inline void CalculateConstraintProperties(const Body& inBody1, Mat44Arg inRotation1, const Body& inBody2, Mat44Arg inRotation2) {
		// Calculate properties used during constraint solving
		mInvI1 = inBody1.IsDynamic()? inBody1.GetMotionProperties()->GetInverseInertiaForRotation(inRotation1) : Mat44::sZero();
		mInvI2 = inBody2.IsDynamic()? inBody2.GetMotionProperties()->GetInverseInertiaForRotation(inRotation2) : Mat44::sZero();

		// Calculate effective mass: K^-1 = (J M^-1 J^T)^-1
		if (!mEffectiveMass.SetInversed3x3(mInvI1 + mInvI2))
			Deactivate();
	}

	/// Deactivate this constraint
	inline void Deactivate() {
		mEffectiveMass = Mat44::sZero();
		mTotalLambda = Vec3::sZero();
	}

	/// Check if constraint is active
	inline bool IsActive() const {
		return mEffectiveMass(3, 3) != 0.0f;
	}

	/// Must be called from the WarmStartVelocityConstraint call to apply the previous frame's impulses
	inline void	WarmStart(Body& ioBody1, Body& ioBody2, float inWarmStartImpulseRatio) {
		mTotalLambda*= inWarmStartImpulseRatio;
		ApplyVelocityStep(ioBody1, ioBody2, mTotalLambda);
	}

	/// Iteratively update the velocity constraint. Makes sure d/dt C(...) = 0, where C is the constraint equation.
	inline bool	SolveVelocityConstraint(Body& ioBody1, Body& ioBody2) {
		// Calculate lagrange multiplier:
		//
		// lambda = -K^-1 (J v + b)
		Vec3 lambda = mEffectiveMass.Multiply3x3(ioBody1.GetAngularVelocity() - ioBody2.GetAngularVelocity());
		mTotalLambda += lambda;
		return ApplyVelocityStep(ioBody1, ioBody2, lambda);
	}

	/// Iteratively update the position constraint. Makes sure C(...) = 0.
	inline bool SolvePositionConstraint(Body& ioBody1, Body& ioBody2, QuatArg inInvInitialOrientation, float inBaumgarte) const {
		// Calculate difference in rotation
		//
		// The rotation should be:
		//
		// q2 = q1 r0
		//
		// But because of drift the actual rotation is
		//
		// q2 = diff q1 r0
		// <=> diff = q2 r0^-1 q1^-1
		//
		// Where:
		// q1 = current rotation of body 1
		// q2 = current rotation of body 2
		// diff = error that needs to be reduced to zero
		Quat diff = ioBody2.GetRotation()* inInvInitialOrientation* ioBody1.GetRotation().Conjugated();

		// A quaternion can be seen as:
		//
		// q = [sin(theta / 2)* v, cos(theta/2)]
		//
		// Where:
		// v = rotation vector
		// theta = rotation angle
		//
		// If we assume theta is small (error is small) then sin(x) = x so an approximation of the error angles is:
		Vec3 error = 2.0f* diff.EnsureWPositive().GetXYZ();
		if (error != Vec3::sZero()) {
			// Calculate lagrange multiplier (lambda) for Baumgarte stabilization:
			//
			// lambda = -K^-1* beta / dt* C
			//
			// We should divide by inDeltaTime, but we should multiply by inDeltaTime in the Euler step below so they're cancelled out
			Vec3 lambda = -inBaumgarte* mEffectiveMass* error;

			// Directly integrate velocity change for one time step
			//
			// Euler velocity integration:
			// dv = M^-1 P
			//
			// Impulse:
			// P = J^T lambda
			//
			// Euler position integration:
			// x' = x + dv* dt
			//
			// Note we don't accumulate velocities for the stabilization. This is using the approach described in 'Modeling and
			// Solving Constraints' by Erin Catto presented at GDC 2007. On slide 78 it is suggested to split up the Baumgarte
			// stabilization for positional drift so that it does not actually add to the momentum. We combine an Euler velocity
			// integrate + a position integrate and then discard the velocity change.
			if (ioBody1.IsDynamic())
				ioBody1.SubRotationStep(mInvI1.Multiply3x3(lambda));
			if (ioBody2.IsDynamic())
				ioBody2.AddRotationStep(mInvI2.Multiply3x3(lambda));
			return true;
		}
		return false;
	}

	/// Return lagrange multiplier
	Vec3 GetTotalLambda() const {
		return mTotalLambda;
	}

	/// Save state of this constraint part
	void SaveState(StateRecorder& inStream) const {
		inStream.Write(mTotalLambda);
	}

	/// Restore state of this constraint part
	void RestoreState(StateRecorder& inStream) {
		inStream.Read(mTotalLambda);
	}

private:
	Mat44						mInvI1;
	Mat44						mInvI2;
	Mat44						mEffectiveMass;
	Vec3						mTotalLambda { Vec3::sZero() };
};



/// A constraint manager manages all constraints of the same type
class ConstraintManager : public NonCopyable
{
public:
	MOSS_OVERRIDE_NEW_DELETE

#ifdef MOSS_DEBUG
	/// Constructor
							ConstraintManager(PhysicsLockContext inContext) : mLockContext(inContext) { }
#endif // MOSS_DEBUG

	/// Add a new constraint. This is thread safe.
	void					Add(Constraint **inConstraints, int inNumber);

	/// Remove a constraint. This is thread safe.
	void					Remove(Constraint **inConstraint, int inNumber);

	/// Get a list of all constraints
	Constraints				GetConstraints() const;

	/// Get total number of constraints
	inline uint32			GetNumConstraints() const					{ return uint32(mConstraints.size()); }

	/// Determine the active constraints of a subset of the constraints
	void					GetActiveConstraints(uint32 inStartConstraintIdx, uint32 inEndConstraintIdx, Constraint **outActiveConstraints, uint32 &outNumActiveConstraints) const;

	/// Link bodies to form islands
	static void				sBuildIslands(Constraint **inActiveConstraints, uint32 inNumActiveConstraints, IslandBuilder &ioBuilder, BodyManager &inBodyManager);

	/// In order to have a deterministic simulation, we need to sort the constraints of an island before solving them
	static void				sSortConstraints(Constraint **inActiveConstraints, uint32 *inConstraintIdxBegin, uint32 *inConstraintIdxEnd);

	/// Prior to solving the velocity constraints, you must call SetupVelocityConstraints once to precalculate values that are independent of velocity
	static void				sSetupVelocityConstraints(Constraint **inActiveConstraints, uint32 inNumActiveConstraints, float inDeltaTime);

	/// Apply last frame's impulses, must be called prior to SolveVelocityConstraints
	template <class ConstraintCallback>
	static void				sWarmStartVelocityConstraints(Constraint **inActiveConstraints, const uint32 *inConstraintIdxBegin, const uint32 *inConstraintIdxEnd, float inWarmStartImpulseRatio, ConstraintCallback &ioCallback);

	/// This function is called multiple times to iteratively come to a solution that meets all velocity constraints
	static bool				sSolveVelocityConstraints(Constraint **inActiveConstraints, const uint32 *inConstraintIdxBegin, const uint32 *inConstraintIdxEnd, float inDeltaTime);

	/// This function is called multiple times to iteratively come to a solution that meets all position constraints
	static bool				sSolvePositionConstraints(Constraint **inActiveConstraints, const uint32 *inConstraintIdxBegin, const uint32 *inConstraintIdxEnd, float inDeltaTime, float inBaumgarte);

#ifndef MOSS_DEBUG_RENDERER
	/// Draw all constraints
	void					DrawConstraints(DebugRenderer *inRenderer) const;

	/// Draw all constraint limits
	void					DrawConstraintLimits(DebugRenderer *inRenderer) const;

	/// Draw all constraint reference frames
	void					DrawConstraintReferenceFrame(DebugRenderer *inRenderer) const;
#endif // MOSS_DEBUG_RENDERER

	/// Save state of constraints
	void					SaveState(StateRecorder &inStream, const StateRecorderFilter *inFilter) const;

	/// Restore the state of constraints. Returns false if failed.
	bool					RestoreState(StateRecorder &inStream);

	/// Lock all constraints. This should only be done during PhysicsSystem::Update().
	void					LockAllConstraints()						{ PhysicsLock::sLock(mConstraintsMutex MOSS_IF_ENABLE_ASSERTS(, mLockContext, EPhysicsLockTypes::ConstraintsList)); }
	void					UnlockAllConstraints()						{ PhysicsLock::sUnlock(mConstraintsMutex MOSS_IF_ENABLE_ASSERTS(, mLockContext, EPhysicsLockTypes::ConstraintsList)); }

private:
#ifdef MOSS_DEBUG
	PhysicsLockContext		mLockContext;
#endif // MOSS_DEBUG
	Constraints				mConstraints;
	mutable Mutex			mConstraintsMutex;
};

/// Base class for all physics constraints. A constraint removes one or more degrees of freedom for a rigid body.
class MOSS_EXPORT Constraint : public RefTarget<Constraint>, public NonCopyable
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
	explicit					Constraint(const ConstraintSettings& inSettings) :
#ifndef MOSS_DEBUG_RENDERER
		mDrawConstraintSize(inSettings.mDrawConstraintSize),
#endif // MOSS_DEBUG_RENDERER
		mConstraintPriority(inSettings.mConstraintPriority),
		mNumVelocityStepsOverride(uint8(inSettings.mNumVelocityStepsOverride)),
		mNumPositionStepsOverride(uint8(inSettings.mNumPositionStepsOverride)),
		mEnabled(inSettings.mEnabled),
		mUserData(inSettings.mUserData)
	{
		MOSS_ASSERT(inSettings.mNumVelocityStepsOverride < 256);
		MOSS_ASSERT(inSettings.mNumPositionStepsOverride < 256);
	}

	/// Virtual destructor
	virtual						~Constraint() = default;

	/// Get the type of a constraint
	virtual EConstraintType		GetType() const								{ return EConstraintType::Constraint; }

	/// Get the sub type of a constraint
	virtual EConstraintSubType	GetSubType() const = 0;

	/// Priority of the constraint when solving. Higher numbers have are more likely to be solved correctly.
	/// Note that if you want a deterministic simulation and you cannot guarantee the order in which constraints are added/removed, you can make the priority for all constraints unique to get a deterministic ordering.
	uint32						GetConstraintPriority() const				{ return mConstraintPriority; }
	void						SetConstraintPriority(uint32 inPriority)	{ mConstraintPriority = inPriority; }

	/// Used only when the constraint is active. Override for the number of solver velocity iterations to run, 0 means use the default in PhysicsSettings::mNumVelocitySteps. The number of iterations to use is the max of all contacts and constraints in the island.
	void						SetNumVelocityStepsOverride(uint inN)		{ MOSS_ASSERT(inN < 256); mNumVelocityStepsOverride = uint8(inN); }
	uint						GetNumVelocityStepsOverride() const			{ return mNumVelocityStepsOverride; }

	/// Used only when the constraint is active. Override for the number of solver position iterations to run, 0 means use the default in PhysicsSettings::mNumPositionSteps. The number of iterations to use is the max of all contacts and constraints in the island.
	void						SetNumPositionStepsOverride(uint inN)		{ MOSS_ASSERT(inN < 256); mNumPositionStepsOverride = uint8(inN); }
	uint						GetNumPositionStepsOverride() const			{ return mNumPositionStepsOverride; }

	/// Enable / disable this constraint. This can e.g. be used to implement a breakable constraint by detecting that the constraint impulse
	/// (see e.g. PointConstraint::GetTotalLambdaPosition) went over a certain limit and then disabling the constraint.
	/// Note that although a disabled constraint will not affect the simulation in any way anymore, it does incur some processing overhead.
	/// Alternatively you can remove a constraint from the constraint manager (which may be more costly if you want to disable the constraint for a short while).
	void						SetEnabled(bool inEnabled)					{ mEnabled = inEnabled; }

	/// Test if a constraint is enabled.
	bool						GetEnabled() const							{ return mEnabled; }

	/// Access to the user data, can be used for anything by the application
	uint64						GetUserData() const							{ return mUserData; }
	void						SetUserData(uint64 inUserData)				{ mUserData = inUserData; }

	/// Notify the constraint that the shape of a body has changed and that its center of mass has moved by inDeltaCOM.
	/// Bodies don't know which constraints are connected to them so the user is responsible for notifying the relevant constraints when a body changes.
	/// @param inBodyID ID of the body that has changed
	/// @param inDeltaCOM The delta of the center of mass of the body (shape->GetCenterOfMass() - shape_before_change->GetCenterOfMass())
	virtual void				NotifyShapeChanged(const BodyID& inBodyID, Vec3Arg inDeltaCOM) = 0;

	/// Notify the system that the configuration of the bodies and/or constraint has changed enough so that the warm start impulses should not be applied the next frame.
	/// You can use this function for example when repositioning a ragdoll through Ragdoll::SetPose in such a way that the orientation of the bodies completely changes so that
	/// the previous frame impulses are no longer a good approximation of what the impulses will be in the next frame. Calling this function when there are no big changes
	/// will result in the constraints being much 'softer' than usual so they are more easily violated (e.g. a long chain of bodies might sag a bit if you call this every frame).
	virtual void				ResetWarmStart() = 0;

	///@name Solver interface
	///@{
	virtual bool				IsActive() const							{ return mEnabled; }
	virtual void				SetupVelocityConstraint(float inDeltaTime) = 0;
	virtual void				WarmStartVelocityConstraint(float inWarmStartImpulseRatio) = 0;
	virtual bool				SolveVelocityConstraint(float inDeltaTime) = 0;
	virtual bool				SolvePositionConstraint(float inDeltaTime, float inBaumgarte) = 0;
	///@}

	/// Link bodies that are connected by this constraint in the island builder
	virtual void				BuildIslands(uint32 inConstraintIndex, IslandBuilder& ioBuilder, BodyManager& inBodyManager) = 0;

	/// Link bodies that are connected by this constraint in the same split. Returns the split index.
	virtual uint				BuildIslandSplits(LargeIslandSplitter& ioSplitter) const = 0;

#ifndef MOSS_DEBUG_RENDERER
	// Drawing interface
	virtual void				DrawConstraint(DebugRenderer*inRenderer) const = 0;
	virtual void				DrawConstraintLimits([[maybe_unused]] DebugRenderer*inRenderer) const { }
	virtual void				DrawConstraintReferenceFrame([[maybe_unused]] DebugRenderer*inRenderer) const { }

	/// Size of constraint when drawing it through the debug renderer
	float						GetDrawConstraintSize() const				{ return mDrawConstraintSize; }
	void						SetDrawConstraintSize(float inSize)			{ mDrawConstraintSize = inSize; }
#endif // MOSS_DEBUG_RENDERER

	/// Saving state for replay
	virtual void				SaveState(StateRecorder& inStream) const;

	/// Restoring state for replay
	virtual void				RestoreState(StateRecorder& inStream);

	/// Debug function to convert a constraint to its settings, note that this will not save to which bodies the constraint is connected to
	virtual Ref<ConstraintSettings> GetConstraintSettings() const = 0;

protected:
	/// Helper function to copy settings back to constraint settings for this base class
	void						ToConstraintSettings(ConstraintSettings& outSettings) const;

#ifndef MOSS_DEBUG_RENDERER
	/// Size of constraint when drawing it through the debug renderer
	float						mDrawConstraintSize;
#endif // MOSS_DEBUG_RENDERER

private:
	friend class ConstraintManager;

	/// Index that indicates this constraint is not in the constraint manager
	static constexpr uint32		cInvalidConstraintIndex = 0xffffffff;

	/// Index in the mConstraints list of the ConstraintManager for easy finding
	uint32						mConstraintIndex = cInvalidConstraintIndex;

	/// Priority of the constraint when solving. Higher numbers have are more likely to be solved correctly.
	uint32						mConstraintPriority = 0;

	/// Used only when the constraint is active. Override for the number of solver velocity iterations to run, 0 means use the default in PhysicsSettings::mNumVelocitySteps. The number of iterations to use is the max of all contacts and constraints in the island.
	uint8						mNumVelocityStepsOverride = 0;

	/// Used only when the constraint is active. Override for the number of solver position iterations to run, 0 means use the default in PhysicsSettings::mNumPositionSteps. The number of iterations to use is the max of all contacts and constraints in the island.
	uint8						mNumPositionStepsOverride = 0;

	/// If this constraint is currently enabled
	bool						mEnabled = true;

	/// User data value (can be used by application)
	uint64						mUserData;
};

class MOSS_EXPORT TwoBodyConstraintSettings : public ConstraintSettings {
	MOSS_DECLARE_SERIALIZABLE_ABSTRACT(MOSS_EXPORT, TwoBodyConstraintSettings)
public:
	/// Create an instance of this constraint
	/// You can use Body::sFixedToWorld for inBody1 if you want to attach inBody2 to the world
	virtual TwoBodyConstraint*	Create(Body& inBody1, Body& inBody2) const = 0;
};

// Fixed constraint settings, used to create a fixed constraint
class MOSS_EXPORT FixedConstraintSettings final : public TwoBodyConstraintSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, FixedConstraintSettings)

public:
	// See: ConstraintSettings::SaveBinaryState
	virtual void				SaveBinaryState(StreamOut& inStream) const override;

	/// Create an instance of this constraint
	virtual TwoBodyConstraint*	Create(Body& inBody1, Body& inBody2) const override;

	/// This determines in which space the constraint is setup, all properties below should be in the specified space
	EConstraintSpace			mSpace = EConstraintSpace::WorldSpace;

	/// When mSpace is WorldSpace mPoint1 and mPoint2 can be automatically calculated based on the positions of the bodies when the constraint is created (they will be fixated in their current relative position/orientation). Set this to false if you want to supply the attachment points yourself.
	bool						mAutoDetectPoint = false;

	/// Body 1 constraint reference frame (space determined by mSpace)
	RVec3						mPoint1 = RVec3::sZero();
	Vec3						mAxisX1 = Vec3::sAxisX();
	Vec3						mAxisY1 = Vec3::sAxisY();

	/// Body 2 constraint reference frame (space determined by mSpace)
	RVec3						mPoint2 = RVec3::sZero();
	Vec3						mAxisX2 = Vec3::sAxisX();
	Vec3						mAxisY2 = Vec3::sAxisY();

protected:
	// See: ConstraintSettings::RestoreBinaryState
	virtual void				RestoreBinaryState(StreamIn& inStream) override;
};

/// A fixed constraint welds two bodies together removing all degrees of freedom between them.
/// This variant uses Euler angles for the rotation constraint.
class MOSS_EXPORT FixedConstraint final : public TwoBodyConstraint
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
								FixedConstraint(Body& inBody1, Body& inBody2, const FixedConstraintSettings& inSettings);

	// Generic interface of a constraint
	virtual EConstraintSubType	GetSubType() const override									{ return EConstraintSubType::Fixed; }
	virtual void				NotifyShapeChanged(const BodyID& inBodyID, Vec3Arg inDeltaCOM) override;
	virtual void				SetupVelocityConstraint(float inDeltaTime) override;
	virtual void				ResetWarmStart() override;
	virtual void				WarmStartVelocityConstraint(float inWarmStartImpulseRatio) override;
	virtual bool				SolveVelocityConstraint(float inDeltaTime) override;
	virtual bool				SolvePositionConstraint(float inDeltaTime, float inBaumgarte) override;
#ifndef MOSS_DEBUG_RENDERER
	virtual void				DrawConstraint(DebugRenderer*inRenderer) const override;
#endif // MOSS_DEBUG_RENDERER
	virtual void				SaveState(StateRecorder& inStream) const override;
	virtual void				RestoreState(StateRecorder& inStream) override;
	virtual Ref<ConstraintSettings> GetConstraintSettings() const override;

	// See: TwoBodyConstraint
	virtual Mat44				GetConstraintToBody1Matrix() const override					{ return Mat44::sTranslation(mLocalSpacePosition1); }
	virtual Mat44				GetConstraintToBody2Matrix() const override					{ return Mat44::sRotationTranslation(mInvInitialOrientation, mLocalSpacePosition2); }

	///@name Get Lagrange multiplier from last physics update (the linear/angular impulse applied to satisfy the constraint)
	inline Vec3					GetTotalLambdaPosition() const								{ return mPointConstraintPart.GetTotalLambda(); }
	inline Vec3					GetTotalLambdaRotation() const								{ return mRotationConstraintPart.GetTotalLambda(); }

private:
	// CONFIGURATION PROPERTIES FOLLOW

	// Local space constraint positions
	Vec3						mLocalSpacePosition1;
	Vec3						mLocalSpacePosition2;

	// Inverse of initial rotation from body 1 to body 2 in body 1 space
	Quat						mInvInitialOrientation;

	// RUN TIME PROPERTIES FOLLOW

	// The constraint parts
	RotationEulerConstraintPart	mRotationConstraintPart;
	PointConstraintPart			mPointConstraintPart;
};

class MOSS_EXPORT DistanceConstraintSettings final : public TwoBodyConstraintSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, DistanceConstraintSettings)

public:
	// See: ConstraintSettings::SaveBinaryState
	virtual void				SaveBinaryState(StreamOut& inStream) const override;

	/// Create an instance of this constraint
	virtual TwoBodyConstraint*	Create(Body& inBody1, Body& inBody2) const override;

	/// This determines in which space the constraint is setup, all properties below should be in the specified space
	EConstraintSpace			mSpace = EConstraintSpace::WorldSpace;

	/// Body 1 constraint reference frame (space determined by mSpace).
	/// Constraint will keep mPoint1 (a point on body 1) and mPoint2 (a point on body 2) at the same distance.
	/// Note that this constraint can be used as a cheap PointConstraint by setting mPoint1 = mPoint2 (but this removes only 1 degree of freedom instead of 3).
	RVec3						mPoint1 = RVec3::sZero();

	/// Body 2 constraint reference frame (space determined by mSpace)
	RVec3						mPoint2 = RVec3::sZero();

	/// Ability to override the distance range at which the two points are kept apart. If the value is negative, it will be replaced by the distance between mPoint1 and mPoint2 (works only if mSpace is world space).
	float						mMinDistance = -1.0f;
	float						mMaxDistance = -1.0f;

	/// When enabled, this makes the limits soft. When the constraint exceeds the limits, a spring force will pull it back.
	SpringSettings				mLimitsSpringSettings;

protected:
	// See: ConstraintSettings::RestoreBinaryState
	virtual void				RestoreBinaryState(StreamIn& inStream) override;
};


class MOSS_EXPORT DistanceConstraintSettings final : public TwoBodyConstraintSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, DistanceConstraintSettings)

public:
	// See: ConstraintSettings::SaveBinaryState
	virtual void				SaveBinaryState(StreamOut& inStream) const override;

	/// Create an instance of this constraint
	virtual TwoBodyConstraint*	Create(Body& inBody1, Body& inBody2) const override;

	/// This determines in which space the constraint is setup, all properties below should be in the specified space
	EConstraintSpace			mSpace = EConstraintSpace::WorldSpace;

	/// Body 1 constraint reference frame (space determined by mSpace).
	/// Constraint will keep mPoint1 (a point on body 1) and mPoint2 (a point on body 2) at the same distance.
	/// Note that this constraint can be used as a cheap PointConstraint by setting mPoint1 = mPoint2 (but this removes only 1 degree of freedom instead of 3).
	RVec3						mPoint1 = RVec3::sZero();

	/// Body 2 constraint reference frame (space determined by mSpace)
	RVec3						mPoint2 = RVec3::sZero();

	/// Ability to override the distance range at which the two points are kept apart. If the value is negative, it will be replaced by the distance between mPoint1 and mPoint2 (works only if mSpace is world space).
	float						mMinDistance = -1.0f;
	float						mMaxDistance = -1.0f;

	/// When enabled, this makes the limits soft. When the constraint exceeds the limits, a spring force will pull it back.
	SpringSettings				mLimitsSpringSettings;

protected:
	// See: ConstraintSettings::RestoreBinaryState
	virtual void				RestoreBinaryState(StreamIn& inStream) override;
};
/// This constraint is a stiff spring that holds 2 points at a fixed distance from each other
class MOSS_EXPORT DistanceConstraint final : public TwoBodyConstraint
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Construct distance constraint
								DistanceConstraint(Body& inBody1, Body& inBody2, const DistanceConstraintSettings& inSettings);

	// Generic interface of a constraint
	virtual EConstraintSubType	GetSubType() const override									{ return EConstraintSubType::Distance; }
	virtual void				NotifyShapeChanged(const BodyID& inBodyID, Vec3Arg inDeltaCOM) override;
	virtual void				SetupVelocityConstraint(float inDeltaTime) override;
	virtual void				ResetWarmStart() override;
	virtual void				WarmStartVelocityConstraint(float inWarmStartImpulseRatio) override;
	virtual bool				SolveVelocityConstraint(float inDeltaTime) override;
	virtual bool				SolvePositionConstraint(float inDeltaTime, float inBaumgarte) override;
#ifndef MOSS_DEBUG_RENDERER
	virtual void				DrawConstraint(DebugRenderer*inRenderer) const override;
#endif // MOSS_DEBUG_RENDERER
	virtual void				SaveState(StateRecorder& inStream) const override;
	virtual void				RestoreState(StateRecorder& inStream) override;
	virtual Ref<ConstraintSettings> GetConstraintSettings() const override;

	// See: TwoBodyConstraint
	virtual Mat44				GetConstraintToBody1Matrix() const override					{ return Mat44::sTranslation(mLocalSpacePosition1); }
	virtual Mat44				GetConstraintToBody2Matrix() const override					{ return Mat44::sTranslation(mLocalSpacePosition2); } // Note: Incorrect rotation as we don't track the original rotation difference, should not matter though as the constraint is not limiting rotation.

	/// Update the minimum and maximum distance for the constraint
	void						SetDistance(float inMinDistance, float inMaxDistance)		{ MOSS_ASSERT(inMinDistance <= inMaxDistance); mMinDistance = inMinDistance; mMaxDistance = inMaxDistance; }
	float						GetMinDistance() const										{ return mMinDistance; }
	float						GetMaxDistance() const										{ return mMaxDistance; }

	/// Update the limits spring settings
	const SpringSettings& 		GetLimitsSpringSettings() const								{ return mLimitsSpringSettings; }
	SpringSettings& 			GetLimitsSpringSettings()									{ return mLimitsSpringSettings; }
	void						SetLimitsSpringSettings(const SpringSettings& inLimitsSpringSettings) { mLimitsSpringSettings = inLimitsSpringSettings; }

	///@name Get Lagrange multiplier from last physics update (the linear impulse applied to satisfy the constraint)
	inline float				GetTotalLambdaPosition() const								{ return mAxisConstraint.GetTotalLambda(); }

private:
	// Internal helper function to calculate the values below
	void						CalculateConstraintProperties(float inDeltaTime);

	// CONFIGURATION PROPERTIES FOLLOW

	// Local space constraint positions
	Vec3						mLocalSpacePosition1;
	Vec3						mLocalSpacePosition2;

	// Min/max distance that must be kept between the world space points
	float						mMinDistance;
	float						mMaxDistance;

	// Soft constraint limits
	SpringSettings				mLimitsSpringSettings;

	// RUN TIME PROPERTIES FOLLOW

	// World space positions and normal
	RVec3						mWorldSpacePosition1;
	RVec3						mWorldSpacePosition2;
	Vec3						mWorldSpaceNormal;

	// Depending on if the distance < min or distance > max we can apply forces to prevent further violations
	float						mMinLambda;
	float						mMaxLambda;

	// The constraint part
	AxisConstraintPart			mAxisConstraint;
};

/// Point constraint settings, used to create a point constraint
class MOSS_EXPORT PointConstraintSettings final : public TwoBodyConstraintSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, PointConstraintSettings)

public:
	// See: ConstraintSettings::SaveBinaryState
	virtual void				SaveBinaryState(StreamOut& inStream) const override;

	/// Create an instance of this constraint
	virtual TwoBodyConstraint*	Create(Body& inBody1, Body& inBody2) const override;

	/// This determines in which space the constraint is setup, all properties below should be in the specified space
	EConstraintSpace			mSpace = EConstraintSpace::WorldSpace;

	/// Body 1 constraint position (space determined by mSpace).
	RVec3						mPoint1 = RVec3::sZero();

	/// Body 2 constraint position (space determined by mSpace).
	/// Note: Normally you would set mPoint1 = mPoint2 if the bodies are already placed how you want to constrain them (if mSpace = world space).
	RVec3						mPoint2 = RVec3::sZero();

protected:
	// See: ConstraintSettings::RestoreBinaryState
	virtual void				RestoreBinaryState(StreamIn& inStream) override;
};

/// A point constraint constrains 2 bodies on a single point (removing 3 degrees of freedom)
class MOSS_EXPORT PointConstraint final : public TwoBodyConstraint
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Construct point constraint
								PointConstraint(Body& inBody1, Body& inBody2, const PointConstraintSettings& inSettings);

	// Generic interface of a constraint
	virtual EConstraintSubType	GetSubType() const override									{ return EConstraintSubType::Point; }
	virtual void				NotifyShapeChanged(const BodyID& inBodyID, Vec3Arg inDeltaCOM) override;
	virtual void				SetupVelocityConstraint(float inDeltaTime) override;
	virtual void				ResetWarmStart() override;
	virtual void				WarmStartVelocityConstraint(float inWarmStartImpulseRatio) override;
	virtual bool				SolveVelocityConstraint(float inDeltaTime) override;
	virtual bool				SolvePositionConstraint(float inDeltaTime, float inBaumgarte) override;
#ifndef MOSS_DEBUG_RENDERER
	virtual void				DrawConstraint(DebugRenderer*inRenderer) const override;
#endif // MOSS_DEBUG_RENDERER
	virtual void				SaveState(StateRecorder& inStream) const override;
	virtual void				RestoreState(StateRecorder& inStream) override;
	virtual Ref<ConstraintSettings> GetConstraintSettings() const override;

	/// Update the attachment point for body 1
	void						SetPoint1(EConstraintSpace inSpace, RVec3Arg inPoint1);

	/// Update the attachment point for body 2
	void						SetPoint2(EConstraintSpace inSpace, RVec3Arg inPoint2);

	/// Get the attachment point for body 1 relative to body 1 COM (transform by Body::GetCenterOfMassTransform to take to world space)
	inline Vec3					GetLocalSpacePoint1() const									{ return mLocalSpacePosition1; }

	/// Get the attachment point for body 2 relative to body 2 COM (transform by Body::GetCenterOfMassTransform to take to world space)
	inline Vec3					GetLocalSpacePoint2() const									{ return mLocalSpacePosition2; }

	// See: TwoBodyConstraint
	virtual Mat44				GetConstraintToBody1Matrix() const override					{ return Mat44::sTranslation(mLocalSpacePosition1); }
	virtual Mat44				GetConstraintToBody2Matrix() const override					{ return Mat44::sTranslation(mLocalSpacePosition2); } // Note: Incorrect rotation as we don't track the original rotation difference, should not matter though as the constraint is not limiting rotation.

	///@name Get Lagrange multiplier from last physics update (the linear impulse applied to satisfy the constraint)
	inline Vec3					GetTotalLambdaPosition() const								{ return mPointConstraintPart.GetTotalLambda(); }

private:
	// Internal helper function to calculate the values below
	void						CalculateConstraintProperties();

	// Local space constraint positions
	Vec3						mLocalSpacePosition1;
	Vec3						mLocalSpacePosition2;

	// The constraint part
	PointConstraintPart			mPointConstraintPart;
};

class MOSS_EXPORT HingeConstraintSettings final : public TwoBodyConstraintSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, HingeConstraintSettings)

public:
	// See: ConstraintSettings::SaveBinaryState
	virtual void				SaveBinaryState(StreamOut& inStream) const override;

	/// Create an instance of this constraint
	virtual TwoBodyConstraint*	Create(Body& inBody1, Body& inBody2) const override;

	/// This determines in which space the constraint is setup, all properties below should be in the specified space
	EConstraintSpace			mSpace = EConstraintSpace::WorldSpace;

	/// Body 1 constraint reference frame (space determined by mSpace).
	/// Hinge axis is the axis where rotation is allowed.
	/// When the normal axis of both bodies align in world space, the hinge angle is defined to be 0.
	/// mHingeAxis1 and mNormalAxis1 should be perpendicular. mHingeAxis2 and mNormalAxis2 should also be perpendicular.
	/// If you configure the joint in world space and create both bodies with a relative rotation you want to be defined as zero,
	/// you can simply set mHingeAxis1 = mHingeAxis2 and mNormalAxis1 = mNormalAxis2.
	RVec3						mPoint1 = RVec3::sZero();
	Vec3						mHingeAxis1 = Vec3::sAxisY();
	Vec3						mNormalAxis1 = Vec3::sAxisX();

	/// Body 2 constraint reference frame (space determined by mSpace)
	RVec3						mPoint2 = RVec3::sZero();
	Vec3						mHingeAxis2 = Vec3::sAxisY();
	Vec3						mNormalAxis2 = Vec3::sAxisX();

	/// Rotation around the hinge axis will be limited between [mLimitsMin, mLimitsMax] where mLimitsMin e [-pi, 0] and mLimitsMax e [0, pi].
	/// Both angles are in radians.
	float						mLimitsMin = -MOSS_PI;
	float						mLimitsMax = MOSS_PI;

	/// When enabled, this makes the limits soft. When the constraint exceeds the limits, a spring force will pull it back.
	SpringSettings				mLimitsSpringSettings;

	/// Maximum amount of torque (N m) to apply as friction when the constraint is not powered by a motor
	float						mMaxFrictionTorque = 0.0f;

	/// In case the constraint is powered, this determines the motor settings around the hinge axis
	MotorSettings				mMotorSettings;

protected:
	// See: ConstraintSettings::RestoreBinaryState
	virtual void				RestoreBinaryState(StreamIn& inStream) override;
};

/// A hinge constraint constrains 2 bodies on a single point and allows only a single axis of rotation
class MOSS_EXPORT HingeConstraint final : public TwoBodyConstraint
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Construct hinge constraint
								HingeConstraint(Body& inBody1, Body& inBody2, const HingeConstraintSettings& inSettings);

	// Generic interface of a constraint
	virtual EConstraintSubType	GetSubType() const override								{ return EConstraintSubType::Hinge; }
	virtual void				NotifyShapeChanged(const BodyID& inBodyID, Vec3Arg inDeltaCOM) override;
	virtual void				SetupVelocityConstraint(float inDeltaTime) override;
	virtual void				ResetWarmStart() override;
	virtual void				WarmStartVelocityConstraint(float inWarmStartImpulseRatio) override;
	virtual bool				SolveVelocityConstraint(float inDeltaTime) override;
	virtual bool				SolvePositionConstraint(float inDeltaTime, float inBaumgarte) override;
#ifndef MOSS_DEBUG_RENDERER
	virtual void				DrawConstraint(DebugRenderer* inRenderer) const override;
	virtual void				DrawConstraintLimits(DebugRenderer* inRenderer) const override;
#endif // MOSS_DEBUG_RENDERER
	virtual void				SaveState(StateRecorder& inStream) const override;
	virtual void				RestoreState(StateRecorder& inStream) override;
	virtual Ref<ConstraintSettings> GetConstraintSettings() const override;

	// See: TwoBodyConstraint
	virtual Mat44				GetConstraintToBody1Matrix() const override;
	virtual Mat44				GetConstraintToBody2Matrix() const override;

	/// Get the attachment point for body 1 relative to body 1 COM (transform by Body::GetCenterOfMassTransform to take to world space)
	inline Vec3					GetLocalSpacePoint1() const								{ return mLocalSpacePosition1; }

	/// Get the attachment point for body 2 relative to body 2 COM (transform by Body::GetCenterOfMassTransform to take to world space)
	inline Vec3					GetLocalSpacePoint2() const								{ return mLocalSpacePosition2; }

	// Local space hinge directions (transform direction by Body::GetCenterOfMassTransform to take to world space)
	Vec3						GetLocalSpaceHingeAxis1() const							{ return mLocalSpaceHingeAxis1; }
	Vec3						GetLocalSpaceHingeAxis2() const							{ return mLocalSpaceHingeAxis2; }

	// Local space normal directions (transform direction by Body::GetCenterOfMassTransform to take to world space)
	Vec3						GetLocalSpaceNormalAxis1() const						{ return mLocalSpaceNormalAxis1; }
	Vec3						GetLocalSpaceNormalAxis2() const						{ return mLocalSpaceNormalAxis2; }

	/// Get the current rotation angle from the rest position
	float						GetCurrentAngle() const;

	// Friction control
	void						SetMaxFrictionTorque(float inFrictionTorque)			{ mMaxFrictionTorque = inFrictionTorque; }
	float						GetMaxFrictionTorque() const							{ return mMaxFrictionTorque; }

	// Motor settings
	MotorSettings& 				GetMotorSettings()										{ return mMotorSettings; }
	const MotorSettings& 		GetMotorSettings() const								{ return mMotorSettings; }

	// Motor controls
	void						SetMotorState(EMotorState inState)						{ MOSS_ASSERT(inState == EMotorState::Off || mMotorSettings.IsValid()); mMotorState = inState; }
	EMotorState					GetMotorState() const									{ return mMotorState; }
	void						SetTargetAngularVelocity(float inAngularVelocity)		{ mTargetAngularVelocity = inAngularVelocity; } ///< rad/s
	float						GetTargetAngularVelocity() const						{ return mTargetAngularVelocity; }
	void						SetTargetAngle(float inAngle)							{ mTargetAngle = mHasLimits? Clamp(inAngle, mLimitsMin, mLimitsMax) : inAngle; } ///< rad
	float						GetTargetAngle() const									{ return mTargetAngle; }

	/// Update the rotation limits of the hinge, value in radians (see HingeConstraintSettings)
	void						SetLimits(float inLimitsMin, float inLimitsMax);
	float						GetLimitsMin() const									{ return mLimitsMin; }
	float						GetLimitsMax() const									{ return mLimitsMax; }
	bool						HasLimits() const										{ return mHasLimits; }

	/// Update the limits spring settings
	const SpringSettings& 		GetLimitsSpringSettings() const							{ return mLimitsSpringSettings; }
	SpringSettings& 			GetLimitsSpringSettings()								{ return mLimitsSpringSettings; }
	void						SetLimitsSpringSettings(const SpringSettings& inLimitsSpringSettings) { mLimitsSpringSettings = inLimitsSpringSettings; }

	///@name Get Lagrange multiplier from last physics update (the linear/angular impulse applied to satisfy the constraint)
	inline Vec3					GetTotalLambdaPosition() const							{ return mPointConstraintPart.GetTotalLambda(); }
	inline TVec<2>			GetTotalLambdaRotation() const							{ return mRotationConstraintPart.GetTotalLambda(); }
	inline float				GetTotalLambdaRotationLimits() const					{ return mRotationLimitsConstraintPart.GetTotalLambda(); }
	inline float				GetTotalLambdaMotor() const								{ return mMotorConstraintPart.GetTotalLambda(); }

private:
	// Internal helper function to calculate the values below
	void						CalculateA1AndTheta();
	void						CalculateRotationLimitsConstraintProperties(float inDeltaTime);
	void						CalculateMotorConstraintProperties(float inDeltaTime);
	inline float				GetSmallestAngleToLimit() const;
	inline bool					IsMinLimitClosest() const;

	// CONFIGURATION PROPERTIES FOLLOW

	// Local space constraint positions
	Vec3						mLocalSpacePosition1;
	Vec3						mLocalSpacePosition2;

	// Local space hinge directions
	Vec3						mLocalSpaceHingeAxis1;
	Vec3						mLocalSpaceHingeAxis2;

	// Local space normal direction (direction relative to which to draw constraint limits)
	Vec3						mLocalSpaceNormalAxis1;
	Vec3						mLocalSpaceNormalAxis2;

	// Inverse of initial relative orientation between bodies (which defines hinge angle = 0)
	Quat						mInvInitialOrientation;

	// Hinge limits
	bool						mHasLimits;
	float						mLimitsMin;
	float						mLimitsMax;

	// Soft constraint limits
	SpringSettings				mLimitsSpringSettings;

	// Friction
	float						mMaxFrictionTorque;

	// Motor controls
	MotorSettings				mMotorSettings;
	EMotorState					mMotorState = EMotorState::Off;
	float						mTargetAngularVelocity = 0.0f;
	float						mTargetAngle = 0.0f;

	// RUN TIME PROPERTIES FOLLOW

	// Current rotation around the hinge axis
	float						mTheta = 0.0f;

	// World space hinge axis for body 1
	Vec3						mA1;

	// The constraint parts
	PointConstraintPart			mPointConstraintPart;
	HingeRotationConstraintPart mRotationConstraintPart;
	AngleConstraintPart			mRotationLimitsConstraintPart;
	AngleConstraintPart			mMotorConstraintPart;
};

class MOSS_EXPORT SliderConstraintSettings final : public TwoBodyConstraintSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, SliderConstraintSettings)

public:
	// See: ConstraintSettings::SaveBinaryState
	virtual void				SaveBinaryState(StreamOut& inStream) const override;

	/// Create an instance of this constraint.
	/// Note that the rotation constraint will be solved from body 1. This means that if body 1 and body 2 have different masses / inertias (kinematic body = infinite mass / inertia), body 1 should be the heaviest body.
	virtual TwoBodyConstraint*	Create(Body& inBody1, Body& inBody2) const override;

	/// Simple way of setting the slider and normal axis in world space (assumes the bodies are already oriented correctly when the constraint is created)
	void						SetSliderAxis(Vec3Arg inSliderAxis);

	/// This determines in which space the constraint is setup, all properties below should be in the specified space
	EConstraintSpace			mSpace = EConstraintSpace::WorldSpace;

	/// When mSpace is WorldSpace mPoint1 and mPoint2 can be automatically calculated based on the positions of the bodies when the constraint is created (the current relative position/orientation is chosen as the '0' position). Set this to false if you want to supply the attachment points yourself.
	bool						mAutoDetectPoint = false;

	/// Body 1 constraint reference frame (space determined by mSpace).
	/// Slider axis is the axis along which movement is possible (direction), normal axis is a perpendicular vector to define the frame.
	RVec3						mPoint1 = RVec3::sZero();
	Vec3						mSliderAxis1 = Vec3::sAxisX();
	Vec3						mNormalAxis1 = Vec3::sAxisY();

	/// Body 2 constraint reference frame (space determined by mSpace)
	RVec3						mPoint2 = RVec3::sZero();
	Vec3						mSliderAxis2 = Vec3::sAxisX();
	Vec3						mNormalAxis2 = Vec3::sAxisY();

	/// When the bodies move so that mPoint1 coincides with mPoint2 the slider position is defined to be 0, movement will be limited between [mLimitsMin, mLimitsMax] where mLimitsMin e [-inf, 0] and mLimitsMax e [0, inf]
	float						mLimitsMin = -FLT_MAX;
	float						mLimitsMax = FLT_MAX;

	/// When enabled, this makes the limits soft. When the constraint exceeds the limits, a spring force will pull it back.
	SpringSettings				mLimitsSpringSettings;

	/// Maximum amount of friction force to apply (N) when not driven by a motor.
	float						mMaxFrictionForce = 0.0f;

	/// In case the constraint is powered, this determines the motor settings around the sliding axis
	MotorSettings				mMotorSettings;

protected:
	// See: ConstraintSettings::RestoreBinaryState
	virtual void				RestoreBinaryState(StreamIn& inStream) override;
};

/// A slider constraint allows movement in only 1 axis (and no rotation). Also known as a prismatic constraint.
class MOSS_EXPORT SliderConstraint final : public TwoBodyConstraint
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Construct slider constraint
								SliderConstraint(Body& inBody1, Body& inBody2, const SliderConstraintSettings& inSettings);

	// Generic interface of a constraint
	virtual EConstraintSubType	GetSubType() const override								{ return EConstraintSubType::Slider; }
	virtual void				NotifyShapeChanged(const BodyID& inBodyID, Vec3Arg inDeltaCOM) override;
	virtual void				SetupVelocityConstraint(float inDeltaTime) override;
	virtual void				ResetWarmStart() override;
	virtual void				WarmStartVelocityConstraint(float inWarmStartImpulseRatio) override;
	virtual bool				SolveVelocityConstraint(float inDeltaTime) override;
	virtual bool				SolvePositionConstraint(float inDeltaTime, float inBaumgarte) override;
#ifndef MOSS_DEBUG_RENDERER
	virtual void				DrawConstraint(DebugRenderer*inRenderer) const override;
	virtual void				DrawConstraintLimits(DebugRenderer*inRenderer) const override;
#endif // MOSS_DEBUG_RENDERER
	virtual void				SaveState(StateRecorder& inStream) const override;
	virtual void				RestoreState(StateRecorder& inStream) override;
	virtual Ref<ConstraintSettings> GetConstraintSettings() const override;

	// See: TwoBodyConstraint
	virtual Mat44				GetConstraintToBody1Matrix() const override;
	virtual Mat44				GetConstraintToBody2Matrix() const override;

	/// Get the current distance from the rest position
	float						GetCurrentPosition() const;

	/// Friction control
	void						SetMaxFrictionForce(float inFrictionForce)				{ mMaxFrictionForce = inFrictionForce; }
	float						GetMaxFrictionForce() const								{ return mMaxFrictionForce; }

	/// Motor settings
	MotorSettings& 				GetMotorSettings()										{ return mMotorSettings; }
	const MotorSettings& 		GetMotorSettings() const								{ return mMotorSettings; }

	// Motor controls
	void						SetMotorState(EMotorState inState)						{ MOSS_ASSERT(inState == EMotorState::Off || mMotorSettings.IsValid()); mMotorState = inState; }
	EMotorState					GetMotorState() const									{ return mMotorState; }
	void						SetTargetVelocity(float inVelocity)						{ mTargetVelocity = inVelocity; }
	float						GetTargetVelocity() const								{ return mTargetVelocity; }
	void						SetTargetPosition(float inPosition)						{ mTargetPosition = mHasLimits? Clamp(inPosition, mLimitsMin, mLimitsMax) : inPosition; }
	float						GetTargetPosition() const								{ return mTargetPosition; }

	/// Update the limits of the slider constraint (see SliderConstraintSettings)
	void						SetLimits(float inLimitsMin, float inLimitsMax);
	float						GetLimitsMin() const									{ return mLimitsMin; }
	float						GetLimitsMax() const									{ return mLimitsMax; }
	bool						HasLimits() const										{ return mHasLimits; }

	/// Update the limits spring settings
	const SpringSettings& 		GetLimitsSpringSettings() const							{ return mLimitsSpringSettings; }
	SpringSettings& 			GetLimitsSpringSettings()								{ return mLimitsSpringSettings; }
	void						SetLimitsSpringSettings(const SpringSettings& inLimitsSpringSettings) { mLimitsSpringSettings = inLimitsSpringSettings; }

	///@name Get Lagrange multiplier from last physics update (the linear/angular impulse applied to satisfy the constraint)
	inline TVec<2>				GetTotalLambdaPosition() const							{ return mPositionConstraintPart.GetTotalLambda(); }
	inline float				GetTotalLambdaPositionLimits() const					{ return mPositionLimitsConstraintPart.GetTotalLambda(); }
	inline Vec3					GetTotalLambdaRotation() const							{ return mRotationConstraintPart.GetTotalLambda(); }
	inline float				GetTotalLambdaMotor() const								{ return mMotorConstraintPart.GetTotalLambda(); }

private:
	// Internal helper function to calculate the values below
	void						CalculateR1R2U(Mat44Arg inRotation1, Mat44Arg inRotation2);
	void						CalculateSlidingAxisAndPosition(Mat44Arg inRotation1);
	void						CalculatePositionConstraintProperties(Mat44Arg inRotation1, Mat44Arg inRotation2);
	void						CalculatePositionLimitsConstraintProperties(float inDeltaTime);
	void						CalculateMotorConstraintProperties(float inDeltaTime);

	// CONFIGURATION PROPERTIES FOLLOW

	// Local space constraint positions
	Vec3						mLocalSpacePosition1;
	Vec3						mLocalSpacePosition2;

	// Local space sliding direction
	Vec3						mLocalSpaceSliderAxis1;

	// Local space normals to the sliding direction (in body 1 space)
	Vec3						mLocalSpaceNormal1;
	Vec3						mLocalSpaceNormal2;

	// Inverse of initial rotation from body 1 to body 2 in body 1 space
	Quat						mInvInitialOrientation;

	// Slider limits
	bool						mHasLimits;
	float						mLimitsMin;
	float						mLimitsMax;

	// Soft constraint limits
	SpringSettings				mLimitsSpringSettings;

	// Friction
	float						mMaxFrictionForce;

	// Motor controls
	MotorSettings				mMotorSettings;
	EMotorState					mMotorState = EMotorState::Off;
	float						mTargetVelocity = 0.0f;
	float						mTargetPosition = 0.0f;

	// RUN TIME PROPERTIES FOLLOW

	// Positions where the point constraint acts on (middle point between center of masses)
	Vec3						mR1;
	Vec3						mR2;

	// X2 + R2 - X1 - R1
	Vec3						mU;

	// World space sliding direction
	Vec3						mWorldSpaceSliderAxis;

	// Normals to the slider axis
	Vec3						mN1;
	Vec3						mN2;

	// Distance along the slide axis
	float						mD = 0.0f;

	// The constraint parts
	DualAxisConstraintPart		mPositionConstraintPart;
	RotationEulerConstraintPart	mRotationConstraintPart;
	AxisConstraintPart			mPositionLimitsConstraintPart;
	AxisConstraintPart			mMotorConstraintPart;
};

// Cone constraint settings, used to create a cone constraint
class MOSS_EXPORT ConeConstraintSettings final : public TwoBodyConstraintSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, ConeConstraintSettings)

public:
	// See: ConstraintSettings::SaveBinaryState
	virtual void				SaveBinaryState(StreamOut& inStream) const override;

	/// Create an instance of this constraint
	virtual TwoBodyConstraint*	Create(Body& inBody1, Body& inBody2) const override;

	/// This determines in which space the constraint is setup, all properties below should be in the specified space
	EConstraintSpace			mSpace = EConstraintSpace::WorldSpace;

	/// Body 1 constraint reference frame (space determined by mSpace)
	RVec3						mPoint1 = RVec3::sZero();
	Vec3						mTwistAxis1 = Vec3::sAxisX();

	/// Body 2 constraint reference frame (space determined by mSpace)
	RVec3						mPoint2 = RVec3::sZero();
	Vec3						mTwistAxis2 = Vec3::sAxisX();

	/// Half of maximum angle between twist axis of body 1 and 2
	float						mHalfConeAngle = 0.0f;

protected:
	// See: ConstraintSettings::RestoreBinaryState
	virtual void				RestoreBinaryState(StreamIn& inStream) override;
};

/// A cone constraint constraints 2 bodies to a single point and limits the swing between the twist axis within a cone:
///
/// t1 . t2 <= cos(theta)
///
/// Where:
///
/// t1 = twist axis of body 1.
/// t2 = twist axis of body 2.
/// theta = half cone angle (angle from the principal axis of the cone to the edge).
///
/// Calculating the Jacobian:
///
/// Constraint equation:
///
/// C = t1 . t2 - cos(theta)
///
/// Derivative:
///
/// d/dt C = d/dt (t1 . t2) = (d/dt t1) . t2 + t1 . (d/dt t2) = (w1 x t1) . t2 + t1 . (w2 x t2) = (t1 x t2) . w1 + (t2 x t1) . w2
///
/// d/dt C = J v = [0, -t2 x t1, 0, t2 x t1] [v1, w1, v2, w2]
///
/// Where J is the Jacobian.
///
/// Note that this is the exact same equation as used in AngleConstraintPart if we use t2 x t1 as the world space axis
class MOSS_EXPORT ConeConstraint final : public TwoBodyConstraint
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Construct cone constraint
								ConeConstraint(Body& inBody1, Body& inBody2, const ConeConstraintSettings& inSettings);

	// Generic interface of a constraint
	virtual EConstraintSubType	GetSubType() const override					{ return EConstraintSubType::Cone; }
	virtual void				NotifyShapeChanged(const BodyID& inBodyID, Vec3Arg inDeltaCOM) override;
	virtual void				SetupVelocityConstraint(float inDeltaTime) override;
	virtual void				ResetWarmStart() override;
	virtual void				WarmStartVelocityConstraint(float inWarmStartImpulseRatio) override;
	virtual bool				SolveVelocityConstraint(float inDeltaTime) override;
	virtual bool				SolvePositionConstraint(float inDeltaTime, float inBaumgarte) override;
#ifndef MOSS_DEBUG_RENDERER
	virtual void				DrawConstraint(DebugRenderer*inRenderer) const override;
	virtual void				DrawConstraintLimits(DebugRenderer*inRenderer) const override;
#endif // MOSS_DEBUG_RENDERER
	virtual void				SaveState(StateRecorder& inStream) const override;
	virtual void				RestoreState(StateRecorder& inStream) override;
	virtual Ref<ConstraintSettings> GetConstraintSettings() const override;

	// See: TwoBodyConstraint
	virtual Mat44				GetConstraintToBody1Matrix() const override;
	virtual Mat44				GetConstraintToBody2Matrix() const override;

	/// Update maximum angle between body 1 and 2 (see ConeConstraintSettings)
	void						SetHalfConeAngle(float inHalfConeAngle)		{ MOSS_ASSERT(inHalfConeAngle >= 0.0f& & inHalfConeAngle <= MOSS_PI); mCosHalfConeAngle = Cos(inHalfConeAngle); }
	float						GetCosHalfConeAngle() const					{ return mCosHalfConeAngle; }

	///@name Get Lagrange multiplier from last physics update (the linear/angular impulse applied to satisfy the constraint)
	inline Vec3					GetTotalLambdaPosition() const				{ return mPointConstraintPart.GetTotalLambda(); }
	inline float				GetTotalLambdaRotation() const				{ return mAngleConstraintPart.GetTotalLambda(); }

private:
	// Internal helper function to calculate the values below
	void						CalculateRotationConstraintProperties(Mat44Arg inRotation1, Mat44Arg inRotation2);

	// CONFIGURATION PROPERTIES FOLLOW

	// Local space constraint positions
	Vec3						mLocalSpacePosition1;
	Vec3						mLocalSpacePosition2;

	// Local space constraint axis
	Vec3						mLocalSpaceTwistAxis1;
	Vec3						mLocalSpaceTwistAxis2;

	// Angular limits
	float						mCosHalfConeAngle;

	// RUN TIME PROPERTIES FOLLOW

	// Axis and angle of rotation between the two bodies
	Vec3						mWorldSpaceRotationAxis;
	float						mCosTheta;

	// The constraint parts
	PointConstraintPart			mPointConstraintPart;
	AngleConstraintPart			mAngleConstraintPart;
};

/// Swing twist constraint settings, used to create a swing twist constraint
/// All values in this structure are copied to the swing twist constraint and the settings object is no longer needed afterwards.
///
/// This image describes the limit settings:
/// @image html Docs/SwingTwistConstraint.png
class MOSS_EXPORT SwingTwistConstraintSettings final : public TwoBodyConstraintSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, SwingTwistConstraintSettings)

public:
	// See: ConstraintSettings::SaveBinaryState
	virtual void				SaveBinaryState(StreamOut& inStream) const override;

	/// Create an instance of this constraint
	virtual TwoBodyConstraint*	Create(Body& inBody1, Body& inBody2) const override;

	/// This determines in which space the constraint is setup, all properties below should be in the specified space
	EConstraintSpace			mSpace = EConstraintSpace::WorldSpace;

	///@name Body 1 constraint reference frame (space determined by mSpace)
	RVec3						mPosition1 = RVec3::sZero();
	Vec3						mTwistAxis1 = Vec3::sAxisX();
	Vec3						mPlaneAxis1 = Vec3::sAxisY();

	///@name Body 2 constraint reference frame (space determined by mSpace)
	RVec3						mPosition2 = RVec3::sZero();
	Vec3						mTwistAxis2 = Vec3::sAxisX();
	Vec3						mPlaneAxis2 = Vec3::sAxisY();

	/// The type of swing constraint that we want to use.
	ESwingType					mSwingType = ESwingType::Cone;

	///@name Swing rotation limits
	float						mNormalHalfConeAngle = 0.0f;								///< See image at Detailed Description. Angle in radians.
	float						mPlaneHalfConeAngle = 0.0f;									///< See image at Detailed Description. Angle in radians.

	///@name Twist rotation limits
	float						mTwistMinAngle = 0.0f;										///< See image at Detailed Description. Angle in radians. Should be \f$\in [-\pi, \pi]\f$.
	float						mTwistMaxAngle = 0.0f;										///< See image at Detailed Description. Angle in radians. Should be \f$\in [-\pi, \pi]\f$.

	///@name Friction
	float						mMaxFrictionTorque = 0.0f;									///< Maximum amount of torque (N m) to apply as friction when the constraint is not powered by a motor

	///@name In case the constraint is powered, this determines the motor settings around the swing and twist axis
	MotorSettings				mSwingMotorSettings;
	MotorSettings				mTwistMotorSettings;

protected:
	// See: ConstraintSettings::RestoreBinaryState
	virtual void				RestoreBinaryState(StreamIn& inStream) override;
};

/// A swing twist constraint is a specialized constraint for humanoid ragdolls that allows limited rotation only
///
/// @see SwingTwistConstraintSettings for a description of the limits
class MOSS_EXPORT SwingTwistConstraint final : public TwoBodyConstraint
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Construct swing twist constraint
								SwingTwistConstraint(Body& inBody1, Body& inBody2, const SwingTwistConstraintSettings& inSettings);

	///@name Generic interface of a constraint
	virtual EConstraintSubType	GetSubType() const override									{ return EConstraintSubType::SwingTwist; }
	virtual void				NotifyShapeChanged(const BodyID& inBodyID, Vec3Arg inDeltaCOM) override;
	virtual void				SetupVelocityConstraint(float inDeltaTime) override;
	virtual void				ResetWarmStart() override;
	virtual void				WarmStartVelocityConstraint(float inWarmStartImpulseRatio) override;
	virtual bool				SolveVelocityConstraint(float inDeltaTime) override;
	virtual bool				SolvePositionConstraint(float inDeltaTime, float inBaumgarte) override;
#ifndef MOSS_DEBUG_RENDERER
	virtual void				DrawConstraint(DebugRenderer*inRenderer) const override;
	virtual void				DrawConstraintLimits(DebugRenderer*inRenderer) const override;
#endif // MOSS_DEBUG_RENDERER
	virtual void				SaveState(StateRecorder& inStream) const override;
	virtual void				RestoreState(StateRecorder& inStream) override;
	virtual Ref<ConstraintSettings> GetConstraintSettings() const override;

	// See: TwoBodyConstraint
	virtual Mat44				GetConstraintToBody1Matrix() const override					{ return Mat44::sRotationTranslation(mConstraintToBody1, mLocalSpacePosition1); }
	virtual Mat44				GetConstraintToBody2Matrix() const override					{ return Mat44::sRotationTranslation(mConstraintToBody2, mLocalSpacePosition2); }

	///@name Constraint reference frame
	inline Vec3					GetLocalSpacePosition1() const								{ return mLocalSpacePosition1; }
	inline Vec3					GetLocalSpacePosition2() const								{ return mLocalSpacePosition2; }
	inline Quat					GetConstraintToBody1() const								{ return mConstraintToBody1; }
	inline Quat					GetConstraintToBody2() const								{ return mConstraintToBody2; }

	///@name Constraint limits
	inline float				GetNormalHalfConeAngle() const								{ return mNormalHalfConeAngle; }
	inline void					SetNormalHalfConeAngle(float inAngle)						{ mNormalHalfConeAngle = inAngle; UpdateLimits(); }
	inline float				GetPlaneHalfConeAngle() const								{ return mPlaneHalfConeAngle; }
	inline void					SetPlaneHalfConeAngle(float inAngle)						{ mPlaneHalfConeAngle = inAngle; UpdateLimits(); }
	inline float				GetTwistMinAngle() const									{ return mTwistMinAngle; }
	inline void					SetTwistMinAngle(float inAngle)								{ mTwistMinAngle = inAngle; UpdateLimits(); }
	inline float				GetTwistMaxAngle() const									{ return mTwistMaxAngle; }
	inline void					SetTwistMaxAngle(float inAngle)								{ mTwistMaxAngle = inAngle; UpdateLimits(); }

	///@name Motor settings
	const MotorSettings& 		GetSwingMotorSettings() const								{ return mSwingMotorSettings; }
	MotorSettings& 				GetSwingMotorSettings()										{ return mSwingMotorSettings; }
	const MotorSettings& 		GetTwistMotorSettings() const								{ return mTwistMotorSettings; }
	MotorSettings& 				GetTwistMotorSettings()										{ return mTwistMotorSettings; }

	///@name Friction control
	void						SetMaxFrictionTorque(float inFrictionTorque)				{ mMaxFrictionTorque = inFrictionTorque; }
	float						GetMaxFrictionTorque() const								{ return mMaxFrictionTorque; }

	///@name Motor controls

	/// Controls if the motors are on or off
	void						SetSwingMotorState(EMotorState inState);
	EMotorState					GetSwingMotorState() const									{ return mSwingMotorState; }
	void						SetTwistMotorState(EMotorState inState);
	EMotorState					GetTwistMotorState() const									{ return mTwistMotorState; }

	/// Set the target angular velocity of body 2 in constraint space of body 2
	void						SetTargetAngularVelocityCS(Vec3Arg inAngularVelocity)		{ mTargetAngularVelocity = inAngularVelocity; }
	Vec3						GetTargetAngularVelocityCS() const							{ return mTargetAngularVelocity; }

	/// Set the target orientation in constraint space (drives constraint to: GetRotationInConstraintSpace() == inOrientation)
	void						SetTargetOrientationCS(QuatArg inOrientation);
	Quat						GetTargetOrientationCS() const								{ return mTargetOrientation; }

	/// Set the target orientation in body space (R2 = R1* inOrientation, where R1 and R2 are the world space rotations for body 1 and 2).
	/// Solve: R2* ConstraintToBody2 = R1* ConstraintToBody1* q (see SwingTwistConstraint::GetSwingTwist) and R2 = R1* inOrientation for q.
	void						SetTargetOrientationBS(QuatArg inOrientation)				{ SetTargetOrientationCS(mConstraintToBody1.Conjugated()* inOrientation* mConstraintToBody2); }

	/// Get current rotation of constraint in constraint space.
	/// Solve: R2* ConstraintToBody2 = R1* ConstraintToBody1* q for q.
	Quat						GetRotationInConstraintSpace() const;

	///@name Get Lagrange multiplier from last physics update (the linear/angular impulse applied to satisfy the constraint)
	inline Vec3					GetTotalLambdaPosition() const								{ return mPointConstraintPart.GetTotalLambda(); }
	inline float				GetTotalLambdaTwist() const									{ return mSwingTwistConstraintPart.GetTotalTwistLambda(); }
	inline float				GetTotalLambdaSwingY() const								{ return mSwingTwistConstraintPart.GetTotalSwingYLambda(); }
	inline float				GetTotalLambdaSwingZ() const								{ return mSwingTwistConstraintPart.GetTotalSwingZLambda(); }
	inline Vec3					GetTotalLambdaMotor() const									{ return Vec3(mMotorConstraintPart[0].GetTotalLambda(), mMotorConstraintPart[1].GetTotalLambda(), mMotorConstraintPart[2].GetTotalLambda()); }

private:
	// Update the limits in the swing twist constraint part
	void						UpdateLimits();

	// CONFIGURATION PROPERTIES FOLLOW

	// Local space constraint positions
	Vec3						mLocalSpacePosition1;
	Vec3						mLocalSpacePosition2;

	// Transforms from constraint space to body space
	Quat						mConstraintToBody1;
	Quat						mConstraintToBody2;

	// Limits
	float						mNormalHalfConeAngle;
	float						mPlaneHalfConeAngle;
	float						mTwistMinAngle;
	float						mTwistMaxAngle;

	// Friction
	float						mMaxFrictionTorque;

	// Motor controls
	MotorSettings				mSwingMotorSettings;
	MotorSettings				mTwistMotorSettings;
	EMotorState					mSwingMotorState = EMotorState::Off;
	EMotorState					mTwistMotorState = EMotorState::Off;
	Vec3						mTargetAngularVelocity = Vec3::sZero();
	Quat						mTargetOrientation = Quat::sIdentity();

	// RUN TIME PROPERTIES FOLLOW

	// Rotation axis for motor constraint parts
	Vec3						mWorldSpaceMotorAxis[3];

	// The constraint parts
	PointConstraintPart			mPointConstraintPart;
	SwingTwistConstraintPart	mSwingTwistConstraintPart;
	AngleConstraintPart			mMotorConstraintPart[3];
};

class MOSS_EXPORT SixDOFConstraintSettings final : public TwoBodyConstraintSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, SixDOFConstraintSettings)

public:
	/// Constraint is split up into translation/rotation around X, Y and Z axis.
	enum EAxis
	{
		TranslationX,
		TranslationY,
		TranslationZ,

		RotationX,
		RotationY,
		RotationZ,

		Num,
		NumTranslation = TranslationZ + 1,
	};

	// See: ConstraintSettings::SaveBinaryState
	virtual void				SaveBinaryState(StreamOut& inStream) const override;

	/// Create an instance of this constraint
	virtual TwoBodyConstraint*	Create(Body& inBody1, Body& inBody2) const override;

	/// This determines in which space the constraint is setup, all properties below should be in the specified space
	EConstraintSpace			mSpace = EConstraintSpace::WorldSpace;

	/// Body 1 constraint reference frame (space determined by mSpace)
	RVec3						mPosition1 = RVec3::sZero();
	Vec3						mAxisX1 = Vec3::sAxisX();
	Vec3						mAxisY1 = Vec3::sAxisY();

	/// Body 2 constraint reference frame (space determined by mSpace)
	RVec3						mPosition2 = RVec3::sZero();
	Vec3						mAxisX2 = Vec3::sAxisX();
	Vec3						mAxisY2 = Vec3::sAxisY();

	/// Friction settings.
	/// For translation: Max friction force in N. 0 = no friction.
	/// For rotation: Max friction torque in Nm. 0 = no friction.
	float						mMaxFriction[EAxis::Num] = { 0, 0, 0, 0, 0, 0 };

	/// The type of swing constraint that we want to use.
	ESwingType					mSwingType = ESwingType::Cone;

	/// Limits.
	/// For translation: Min and max linear limits in m (0 is frame of body 1 and 2 coincide).
	/// For rotation: Min and max angular limits in rad (0 is frame of body 1 and 2 coincide). See comments at Axis enum for limit ranges.
	///
	/// Remove degree of freedom by setting min = FLT_MAX and max = -FLT_MAX. The constraint will be driven to 0 for this axis.
	///
	/// Free movement over an axis is allowed when min = -FLT_MAX and max = FLT_MAX.
	///
	/// Rotation limit around X-Axis: When limited, should be \f$\in [-\pi, \pi]\f$. Can be asymmetric around zero.
	///
	/// Rotation limit around Y-Z Axis: Forms a pyramid or cone shaped limit:
	///* For pyramid, should be \f$\in [-\pi, \pi]\f$ and does not need to be symmetrical around zero.
	///* For cone should be \f$\in [0, \pi]\f$ and needs to be symmetrical around zero (min limit is assumed to be -max limit).
	float						mLimitMin[EAxis::Num] = { -FLT_MAX, -FLT_MAX, -FLT_MAX, -FLT_MAX, -FLT_MAX, -FLT_MAX };
	float						mLimitMax[EAxis::Num] = { FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX };

	/// When enabled, this makes the limits soft. When the constraint exceeds the limits, a spring force will pull it back.
	/// Only soft translation limits are supported, soft rotation limits are not currently supported.
	SpringSettings				mLimitsSpringSettings[EAxis::NumTranslation];

	/// Make axis free (unconstrained)
	void						MakeFreeAxis(EAxis inAxis)									{ mLimitMin[inAxis] = -FLT_MAX; mLimitMax[inAxis] = FLT_MAX; }
	bool						IsFreeAxis(EAxis inAxis) const								{ return mLimitMin[inAxis] == -FLT_MAX& & mLimitMax[inAxis] == FLT_MAX; }

	/// Make axis fixed (fixed at value 0)
	void						MakeFixedAxis(EAxis inAxis)									{ mLimitMin[inAxis] = FLT_MAX; mLimitMax[inAxis] = -FLT_MAX; }
	bool						IsFixedAxis(EAxis inAxis) const								{ return mLimitMin[inAxis] >= mLimitMax[inAxis]; }

	/// Set a valid range for the constraint (if inMax < inMin, the axis will become fixed)
	void						SetLimitedAxis(EAxis inAxis, float inMin, float inMax)		{ mLimitMin[inAxis] = inMin; mLimitMax[inAxis] = inMax; }

	/// Motor settings for each axis
	MotorSettings				mMotorSettings[EAxis::Num];

protected:
	// See: ConstraintSettings::RestoreBinaryState
	virtual void				RestoreBinaryState(StreamIn& inStream) override;
};

/// 6 Degree Of Freedom Constraint. Allows control over each of the 6 degrees of freedom.
class MOSS_EXPORT SixDOFConstraint final : public TwoBodyConstraint
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Get Axis from settings class
	using EAxis = SixDOFConstraintSettings::EAxis;

	/// Construct six DOF constraint
								SixDOFConstraint(Body& inBody1, Body& inBody2, const SixDOFConstraintSettings& inSettings);

	/// Generic interface of a constraint
	virtual EConstraintSubType	GetSubType() const override									{ return EConstraintSubType::SixDOF; }
	virtual void				NotifyShapeChanged(const BodyID& inBodyID, Vec3Arg inDeltaCOM) override;
	virtual void				SetupVelocityConstraint(float inDeltaTime) override;
	virtual void				ResetWarmStart() override;
	virtual void				WarmStartVelocityConstraint(float inWarmStartImpulseRatio) override;
	virtual bool				SolveVelocityConstraint(float inDeltaTime) override;
	virtual bool				SolvePositionConstraint(float inDeltaTime, float inBaumgarte) override;
#ifndef MOSS_DEBUG_RENDERER
	virtual void				DrawConstraint(DebugRenderer*inRenderer) const override;
	virtual void				DrawConstraintLimits(DebugRenderer*inRenderer) const override;
#endif // MOSS_DEBUG_RENDERER
	virtual void				SaveState(StateRecorder& inStream) const override;
	virtual void				RestoreState(StateRecorder& inStream) override;
	virtual Ref<ConstraintSettings> GetConstraintSettings() const override;

	// See: TwoBodyConstraint
	virtual Mat44				GetConstraintToBody1Matrix() const override					{ return Mat44::sRotationTranslation(mConstraintToBody1, mLocalSpacePosition1); }
	virtual Mat44				GetConstraintToBody2Matrix() const override					{ return Mat44::sRotationTranslation(mConstraintToBody2, mLocalSpacePosition2); }

	/// Update the translation limits for this constraint
	void						SetTranslationLimits(Vec3Arg inLimitMin, Vec3Arg inLimitMax);

	/// Update the rotational limits for this constraint
	void						SetRotationLimits(Vec3Arg inLimitMin, Vec3Arg inLimitMax);

	/// Get constraint Limits
	float						GetLimitsMin(EAxis inAxis) const							{ return mLimitMin[inAxis]; }
	float						GetLimitsMax(EAxis inAxis) const							{ return mLimitMax[inAxis]; }
	Vec3						GetTranslationLimitsMin() const								{ return Vec3::sLoadFloat3Unsafe(*reinterpret_cast<const Float3*>(&mLimitMin[EAxis::TranslationX])); }
	Vec3						GetTranslationLimitsMax() const								{ return Vec3::sLoadFloat3Unsafe(*reinterpret_cast<const Float3*>(&mLimitMax[EAxis::TranslationX])); }
	Vec3						GetRotationLimitsMin() const								{ return Vec3::sLoadFloat3Unsafe(*reinterpret_cast<const Float3*>(&mLimitMin[EAxis::RotationX])); }
	Vec3						GetRotationLimitsMax() const								{ return Vec3::sLoadFloat3Unsafe(*reinterpret_cast<const Float3*>(&mLimitMax[EAxis::RotationX])); }

	/// Check which axis are fixed/free
	inline bool					IsFixedAxis(EAxis inAxis) const								{ return (mFixedAxis&  (1 << inAxis)) != 0; }
	inline bool					IsFreeAxis(EAxis inAxis) const								{ return (mFreeAxis&  (1 << inAxis)) != 0; }

	/// Update the limits spring settings
	const SpringSettings& 		GetLimitsSpringSettings(EAxis inAxis) const					{ MOSS_ASSERT(inAxis < EAxis::NumTranslation); return mLimitsSpringSettings[inAxis]; }
	void						SetLimitsSpringSettings(EAxis inAxis, const SpringSettings& inLimitsSpringSettings) { MOSS_ASSERT(inAxis < EAxis::NumTranslation); mLimitsSpringSettings[inAxis] = inLimitsSpringSettings; CacheHasSpringLimits(); }

	/// Set the max friction for each axis
	void						SetMaxFriction(EAxis inAxis, float inFriction);
	float						GetMaxFriction(EAxis inAxis) const							{ return mMaxFriction[inAxis]; }

	/// Get rotation of constraint in constraint space
	Quat						GetRotationInConstraintSpace() const;

	/// Motor settings
	MotorSettings& 				GetMotorSettings(EAxis inAxis)								{ return mMotorSettings[inAxis]; }
	const MotorSettings& 		GetMotorSettings(EAxis inAxis) const						{ return mMotorSettings[inAxis]; }

	/// Motor controls.
	/// Translation motors work in constraint space of body 1.
	/// Rotation motors work in constraint space of body 2 (!).
	void						SetMotorState(EAxis inAxis, EMotorState inState);
	EMotorState					GetMotorState(EAxis inAxis) const							{ return mMotorState[inAxis]; }

	/// Set the target velocity in body 1 constraint space
	Vec3						GetTargetVelocityCS() const									{ return mTargetVelocity; }
	void						SetTargetVelocityCS(Vec3Arg inVelocity)						{ mTargetVelocity = inVelocity; }

	/// Set the target angular velocity in body 2 constraint space (!)
	void						SetTargetAngularVelocityCS(Vec3Arg inAngularVelocity)		{ mTargetAngularVelocity = inAngularVelocity; }
	Vec3						GetTargetAngularVelocityCS() const							{ return mTargetAngularVelocity; }

	/// Set the target position in body 1 constraint space
	Vec3						GetTargetPositionCS() const									{ return mTargetPosition; }
	void						SetTargetPositionCS(Vec3Arg inPosition)						{ mTargetPosition = inPosition; }

	/// Set the target orientation in body 1 constraint space
	void						SetTargetOrientationCS(QuatArg inOrientation);
	Quat						GetTargetOrientationCS() const								{ return mTargetOrientation; }

	/// Set the target orientation in body space (R2 = R1* inOrientation, where R1 and R2 are the world space rotations for body 1 and 2).
	/// Solve: R2* ConstraintToBody2 = R1* ConstraintToBody1* q (see SwingTwistConstraint::GetSwingTwist) and R2 = R1* inOrientation for q.
	void						SetTargetOrientationBS(QuatArg inOrientation)				{ SetTargetOrientationCS(mConstraintToBody1.Conjugated()* inOrientation* mConstraintToBody2); }

	///@name Get Lagrange multiplier from last physics update (the linear/angular impulse applied to satisfy the constraint)
	inline Vec3					GetTotalLambdaPosition() const								{ return IsTranslationFullyConstrained()? mPointConstraintPart.GetTotalLambda() : Vec3(mTranslationConstraintPart[0].GetTotalLambda(), mTranslationConstraintPart[1].GetTotalLambda(), mTranslationConstraintPart[2].GetTotalLambda()); }
	inline Vec3					GetTotalLambdaRotation() const								{ return IsRotationFullyConstrained()? mRotationConstraintPart.GetTotalLambda() : Vec3(mSwingTwistConstraintPart.GetTotalTwistLambda(), mSwingTwistConstraintPart.GetTotalSwingYLambda(), mSwingTwistConstraintPart.GetTotalSwingZLambda()); }
	inline Vec3					GetTotalLambdaMotorTranslation() const						{ return Vec3(mMotorTranslationConstraintPart[0].GetTotalLambda(), mMotorTranslationConstraintPart[1].GetTotalLambda(), mMotorTranslationConstraintPart[2].GetTotalLambda()); }
	inline Vec3					GetTotalLambdaMotorRotation() const							{ return Vec3(mMotorRotationConstraintPart[0].GetTotalLambda(), mMotorRotationConstraintPart[1].GetTotalLambda(), mMotorRotationConstraintPart[2].GetTotalLambda()); }

private:
	// Calculate properties needed for the position constraint
	inline void					GetPositionConstraintProperties(Vec3& outR1PlusU, Vec3& outR2, Vec3& outU) const;

	// Sanitize the translation limits
	inline void					UpdateTranslationLimits();

	// Propagate the rotation limits to the constraint part
	inline void					UpdateRotationLimits();

	// Update the cached state of which axis are free and which ones are fixed
	inline void					UpdateFixedFreeAxis();

	// Cache the state of mTranslationMotorActive
	void						CacheTranslationMotorActive();

	// Cache the state of mRotationMotorActive
	void						CacheRotationMotorActive();

	// Cache the state of mRotationPositionMotorActive
	void						CacheRotationPositionMotorActive();

	/// Cache the state of mHasSpringLimits
	void						CacheHasSpringLimits();

	// Constraint settings helper functions
	inline bool					IsTranslationConstrained() const							{ return (mFreeAxis&  0b111) != 0b111; }
	inline bool					IsTranslationFullyConstrained() const						{ return (mFixedAxis&  0b111) == 0b111& & !mHasSpringLimits; }
	inline bool					IsRotationConstrained() const								{ return (mFreeAxis&  0b111000) != 0b111000; }
	inline bool					IsRotationFullyConstrained() const							{ return (mFixedAxis&  0b111000) == 0b111000; }
	inline bool					HasFriction(EAxis inAxis) const								{ return !IsFixedAxis(inAxis)& & mMaxFriction[inAxis] > 0.0f; }

	// CONFIGURATION PROPERTIES FOLLOW

	// Local space constraint positions
	Vec3						mLocalSpacePosition1;
	Vec3						mLocalSpacePosition2;

	// Transforms from constraint space to body space
	Quat						mConstraintToBody1;
	Quat						mConstraintToBody2;

	// Limits
	uint8						mFreeAxis = 0;												// Bitmask of free axis (bit 0 = TranslationX)
	uint8						mFixedAxis = 0;												// Bitmask of fixed axis (bit 0 = TranslationX)
	bool						mTranslationMotorActive = false;							// If any of the translational frictions / motors are active
	bool						mRotationMotorActive = false;								// If any of the rotational frictions / motors are active
	uint8						mRotationPositionMotorActive = 0;							// Bitmask of axis that have position motor active (bit 0 = RotationX)
	bool						mHasSpringLimits = false;									// If any of the limit springs have a non-zero frequency/stiffness
	float						mLimitMin[EAxis::Num];
	float						mLimitMax[EAxis::Num];
	SpringSettings				mLimitsSpringSettings[EAxis::NumTranslation];

	// Motor settings for each axis
	MotorSettings				mMotorSettings[EAxis::Num];

	// Friction settings for each axis
	float						mMaxFriction[EAxis::Num];

	// Motor controls
	EMotorState					mMotorState[EAxis::Num] = { EMotorState::Off, EMotorState::Off, EMotorState::Off, EMotorState::Off, EMotorState::Off, EMotorState::Off };
	Vec3						mTargetVelocity = Vec3::sZero();
	Vec3						mTargetAngularVelocity = Vec3::sZero();
	Vec3						mTargetPosition = Vec3::sZero();
	Quat						mTargetOrientation = Quat::sIdentity();

	// RUN TIME PROPERTIES FOLLOW

	// Constraint space axis in world space
	Vec3						mTranslationAxis[3];
	Vec3						mRotationAxis[3];

	// Translation displacement (valid when translation axis has a range limit)
	float						mDisplacement[3];

	// Individual constraint parts for translation, or a combined point constraint part if all axis are fixed
	AxisConstraintPart			mTranslationConstraintPart[3];
	PointConstraintPart			mPointConstraintPart;

	// Individual constraint parts for rotation or a combined constraint part if rotation is fixed
	SwingTwistConstraintPart	mSwingTwistConstraintPart;
	RotationEulerConstraintPart	mRotationConstraintPart;

	// Motor or friction constraints
	AxisConstraintPart			mMotorTranslationConstraintPart[3];
	AngleConstraintPart			mMotorRotationConstraintPart[3];
};

class MOSS_EXPORT GearConstraintSettings final : public TwoBodyConstraintSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, GearConstraintSettings)

public:
	// See: ConstraintSettings::SaveBinaryState
	virtual void				SaveBinaryState(StreamOut& inStream) const override;

	/// Create an instance of this constraint.
	virtual TwoBodyConstraint*	Create(Body& inBody1, Body& inBody2) const override;

	/// Defines the ratio between the rotation of both gears
	/// The ratio is defined as: Gear1Rotation(t) = -ratio* Gear2Rotation(t)
	/// @param inNumTeethGear1 Number of teeth that body 1 has
	/// @param inNumTeethGear2 Number of teeth that body 2 has
	void						SetRatio(int inNumTeethGear1, int inNumTeethGear2)
	{
		mRatio = float(inNumTeethGear2) / float(inNumTeethGear1);
	}

	/// This determines in which space the constraint is setup, all properties below should be in the specified space
	EConstraintSpace			mSpace = EConstraintSpace::WorldSpace;

	/// Body 1 constraint reference frame (space determined by mSpace).
	Vec3						mHingeAxis1 = Vec3::sAxisX();

	/// Body 2 constraint reference frame (space determined by mSpace)
	Vec3						mHingeAxis2 = Vec3::sAxisX();

	/// Ratio between both gears, see SetRatio.
	float						mRatio = 1.0f;

protected:
	// See: ConstraintSettings::RestoreBinaryState
	virtual void				RestoreBinaryState(StreamIn& inStream) override;
};

/// A gear constraint constrains the rotation of body1 to the rotation of body 2 using a gear.
/// Note that this constraint needs to be used in conjunction with a two hinge constraints.
class MOSS_EXPORT GearConstraint final : public TwoBodyConstraint {
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Construct gear constraint
								GearConstraint(Body& inBody1, Body& inBody2, const GearConstraintSettings& inSettings);

	// Generic interface of a constraint
	virtual EConstraintSubType	GetSubType() const override								{ return EConstraintSubType::Gear; }
	virtual void				NotifyShapeChanged(const BodyID& inBodyID, Vec3Arg inDeltaCOM) override { /* Do nothing*/ }
	virtual void				SetupVelocityConstraint(float inDeltaTime) override;
	virtual void				ResetWarmStart() override;
	virtual void				WarmStartVelocityConstraint(float inWarmStartImpulseRatio) override;
	virtual bool				SolveVelocityConstraint(float inDeltaTime) override;
	virtual bool				SolvePositionConstraint(float inDeltaTime, float inBaumgarte) override;
#ifndef MOSS_DEBUG_RENDERER
	virtual void				DrawConstraint(DebugRenderer*inRenderer) const override;
#endif // MOSS_DEBUG_RENDERER
	virtual void				SaveState(StateRecorder& inStream) const override;
	virtual void				RestoreState(StateRecorder& inStream) override;
	virtual Ref<ConstraintSettings> GetConstraintSettings() const override;

	// See: TwoBodyConstraint
	virtual Mat44				GetConstraintToBody1Matrix() const override;
	virtual Mat44				GetConstraintToBody2Matrix() const override;

	/// The constraints that constrain both gears (2 hinges), optional and used to calculate the rotation error and fix numerical drift.
	void						SetConstraints(const Constraint*inGear1, const Constraint*inGear2)	{ mGear1Constraint = inGear1; mGear2Constraint = inGear2; }

	///@name Get Lagrange multiplier from last physics update (the angular impulse applied to satisfy the constraint)
	inline float				GetTotalLambda() const									{ return mGearConstraintPart.GetTotalLambda(); }

private:
	// Internal helper function to calculate the values below
	void						CalculateConstraintProperties(Mat44Arg inRotation1, Mat44Arg inRotation2);

	// CONFIGURATION PROPERTIES FOLLOW

	// Local space hinge axis for body 1
	Vec3						mLocalSpaceHingeAxis1;

	// Local space hinge axis for body 2
	Vec3						mLocalSpaceHingeAxis2;

	// Ratio between gear 1 and 2
	float						mRatio;

	// The constraints that constrain both gears (2 hinges), optional and used to calculate the rotation error and fix numerical drift.
	RefConst<Constraint>		mGear1Constraint;
	RefConst<Constraint>		mGear2Constraint;

	// RUN TIME PROPERTIES FOLLOW

	// World space hinge axis for body 1
	Vec3						mWorldSpaceHingeAxis1;

	// World space hinge axis for body 2
	Vec3						mWorldSpaceHingeAxis2;

	// The constraint parts
	GearConstraintPart			mGearConstraintPart;
};


/* TwoBodyConstraint */
// Base class for all constraints that involve 2 bodies. Body1 is usually considered the parent, Body2 the child.
class MOSS_EXPORT TwoBodyConstraint : public Constraint
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	// Constructor
								TwoBodyConstraint(Body &inBody1, Body &inBody2, const TwoBodyConstraintSettings &inSettings) : Constraint(inSettings), mBody1(&inBody1), mBody2(&inBody2) { }

	// Get the type of a constraint
	virtual EConstraintType		GetType() const override				{ return EConstraintType::TwoBodyConstraint; }

	// Solver interface
	virtual bool				IsActive() const override				{ return Constraint::IsActive() && (mBody1->IsActive() || mBody2->IsActive()) && (mBody2->IsDynamic() || mBody1->IsDynamic()); }
#ifndef MOSS_DEBUG_RENDERER
	virtual void				DrawConstraintReferenceFrame(DebugRenderer *inRenderer) const override;
#endif // MOSS_DEBUG_RENDERER

	// Access to the connected bodies
	Body*						GetBody1() const						{ return mBody1; }
	Body*						GetBody2() const						{ return mBody2; }

	// Calculates the transform that transforms from constraint space to body 1 space. The first column of the matrix is the primary constraint axis (e.g. the hinge axis / slider direction), second column the secondary etc.
	virtual Mat44				GetConstraintToBody1Matrix() const = 0;

	// Calculates the transform that transforms from constraint space to body 2 space. The first column of the matrix is the primary constraint axis (e.g. the hinge axis / slider direction), second column the secondary etc.
	virtual Mat44				GetConstraintToBody2Matrix() const = 0;

	// Link bodies that are connected by this constraint in the island builder
	virtual void				BuildIslands(uint32 inConstraintIndex, IslandBuilder &ioBuilder, BodyManager &inBodyManager) override;

	// Link bodies that are connected by this constraint in the same split. Returns the split index.
	virtual uint				BuildIslandSplits(LargeIslandSplitter &ioSplitter) const override;

protected:
	// The two bodies involved
	Body*						mBody1;
	Body*						mBody2;
};



/// Path constraint settings, used to constrain the degrees of freedom between two bodies to a path
///
/// The requirements of the path are that:
/// * Tangent, normal and bi-normal form an orthonormal basis with: tangent cross bi-normal = normal
/// * The path points along the tangent vector
/// * The path is continuous so doesn't contain any sharp corners
///
/// The reason for all this is that the constraint acts like a slider constraint with the sliding axis being the tangent vector (the assumption here is that delta time will be small enough so that the path is linear for that delta time).
class MOSS_EXPORT PathConstraintSettings final : public TwoBodyConstraintSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, PathConstraintSettings)

public:
	// See: ConstraintSettings::SaveBinaryState
	virtual void					SaveBinaryState(StreamOut &inStream) const override;

	/// Create an instance of this constraint
	virtual TwoBodyConstraint *		Create(Body &inBody1, Body &inBody2) const override;

	/// The path that constrains the two bodies
	RefConst<PathConstraintPath>	mPath;

	/// The position of the path start relative to world transform of body 1
	Vec3							mPathPosition = Vec3::sZero();

	/// The rotation of the path start relative to world transform of body 1
	Quat							mPathRotation = Quat::sIdentity();

	/// The fraction along the path that corresponds to the initial position of body 2. Usually this is 0, the beginning of the path. But if you want to start an object halfway the path you can calculate this with mPath->GetClosestPoint(point on path to attach body to).
	float							mPathFraction = 0.0f;

	/// Maximum amount of friction force to apply (N) when not driven by a motor.
	float							mMaxFrictionForce = 0.0f;

	/// In case the constraint is powered, this determines the motor settings along the path
	MotorSettings					mPositionMotorSettings;

	/// How to constrain the rotation of the body to the path
	EPathRotationConstraintType		mRotationConstraintType = EPathRotationConstraintType::Free;

protected:
	// See: ConstraintSettings::RestoreBinaryState
	virtual void					RestoreBinaryState(StreamIn &inStream) override;
};

/// Path constraint, used to constrain the degrees of freedom between two bodies to a path
class MOSS_EXPORT PathConstraint final : public TwoBodyConstraint
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Construct point constraint
									PathConstraint(Body &inBody1, Body &inBody2, const PathConstraintSettings &inSettings);

	// Generic interface of a constraint
	virtual EConstraintSubType		GetSubType() const override								{ return EConstraintSubType::Path; }
	virtual void					NotifyShapeChanged(const BodyID &inBodyID, Vec3Arg inDeltaCOM) override;
	virtual void					SetupVelocityConstraint(float inDeltaTime) override;
	virtual void					ResetWarmStart() override;
	virtual void					WarmStartVelocityConstraint(float inWarmStartImpulseRatio) override;
	virtual bool					SolveVelocityConstraint(float inDeltaTime) override;
	virtual bool					SolvePositionConstraint(float inDeltaTime, float inBaumgarte) override;
#ifndef MOSS_DEBUG_RENDERER
	virtual void					DrawConstraint(DebugRenderer *inRenderer) const override;
#endif // MOSS_DEBUG_RENDERER
	virtual void					SaveState(StateRecorder &inStream) const override;
	virtual void					RestoreState(StateRecorder &inStream) override;
	virtual bool					IsActive() const override								{ return TwoBodyConstraint::IsActive() && mPath != nullptr; }
	virtual Ref<ConstraintSettings> GetConstraintSettings() const override;

	// See: TwoBodyConstraint
	virtual Mat44					GetConstraintToBody1Matrix() const override				{ return mPathToBody1; }
	virtual Mat44					GetConstraintToBody2Matrix() const override				{ return mPathToBody2; }

	/// Update the path for this constraint
	void							SetPath(const PathConstraintPath *inPath, float inPathFraction);

	/// Access to the current path
	const PathConstraintPath *		GetPath() const											{ return mPath; }

	/// Access to the current fraction along the path e [0, GetPath()->GetMaxPathFraction()]
	float							GetPathFraction() const									{ return mPathFraction; }

	/// Friction control
	void							SetMaxFrictionForce(float inFrictionForce)				{ mMaxFrictionForce = inFrictionForce; }
	float							GetMaxFrictionForce() const								{ return mMaxFrictionForce; }

	/// Position motor settings
	MotorSettings &					GetPositionMotorSettings()								{ return mPositionMotorSettings; }
	const MotorSettings &			GetPositionMotorSettings() const						{ return mPositionMotorSettings; }

	// Position motor controls (drives body 2 along the path)
	void							SetPositionMotorState(EMotorState inState)				{ MOSS_ASSERT(inState == EMotorState::Off || mPositionMotorSettings.IsValid()); mPositionMotorState = inState; }
	EMotorState						GetPositionMotorState() const							{ return mPositionMotorState; }
	void							SetTargetVelocity(float inVelocity)						{ mTargetVelocity = inVelocity; }
	float							GetTargetVelocity() const								{ return mTargetVelocity; }
	void							SetTargetPathFraction(float inFraction)					{ MOSS_ASSERT(mPath->IsLooping() || (inFraction >= 0.0f && inFraction <= mPath->GetPathMaxFraction())); mTargetPathFraction = inFraction; }
	float							GetTargetPathFraction() const							{ return mTargetPathFraction; }

	///@name Get Lagrange multiplier from last physics update (the linear/angular impulse applied to satisfy the constraint)
	inline TVec<2>				GetTotalLambdaPosition() const							{ return mPositionConstraintPart.GetTotalLambda(); }
	inline float					GetTotalLambdaPositionLimits() const					{ return mPositionLimitsConstraintPart.GetTotalLambda(); }
	inline float					GetTotalLambdaMotor() const								{ return mPositionMotorConstraintPart.GetTotalLambda(); }
	inline TVec<2>				GetTotalLambdaRotationHinge() const						{ return mHingeConstraintPart.GetTotalLambda(); }
	inline Vec3						GetTotalLambdaRotation() const							{ return mRotationConstraintPart.GetTotalLambda(); }

private:
	// Internal helper function to calculate the values below
	void							CalculateConstraintProperties(float inDeltaTime);

	// CONFIGURATION PROPERTIES FOLLOW

	RefConst<PathConstraintPath>	mPath;													///< The path that attaches the two bodies
	Mat44							mPathToBody1;											///< Transform that takes a quantity from path space to body 1 center of mass space
	Mat44							mPathToBody2;											///< Transform that takes a quantity from path space to body 2 center of mass space
	EPathRotationConstraintType		mRotationConstraintType;								///< How to constrain the rotation of the path

	// Friction
	float							mMaxFrictionForce;

	// Motor controls
	MotorSettings					mPositionMotorSettings;
	EMotorState						mPositionMotorState = EMotorState::Off;
	float							mTargetVelocity = 0.0f;
	float							mTargetPathFraction = 0.0f;

	// RUN TIME PROPERTIES FOLLOW

	// Positions where the point constraint acts on in world space
	Vec3							mR1;
	Vec3							mR2;

	// X2 + R2 - X1 - R1
	Vec3							mU;

	// World space path tangent
	Vec3							mPathTangent;

	// Normals to the path tangent
	Vec3							mPathNormal;
	Vec3							mPathBinormal;

	// Inverse of initial rotation from body 1 to body 2 in body 1 space (only used when rotation constraint type is FullyConstrained)
	Quat							mInvInitialOrientation;

	// Current fraction along the path where body 2 is attached
	float							mPathFraction = 0.0f;

	// Translation constraint parts
	DualAxisConstraintPart			mPositionConstraintPart;								///< Constraint part that keeps the movement along the tangent of the path
	AxisConstraintPart				mPositionLimitsConstraintPart;							///< Constraint part that prevents movement beyond the beginning and end of the path
	AxisConstraintPart				mPositionMotorConstraintPart;							///< Constraint to drive the object along the path or to apply friction

	// Rotation constraint parts
	HingeRotationConstraintPart		mHingeConstraintPart;									///< Constraint part that removes 2 degrees of rotation freedom
	RotationEulerConstraintPart		mRotationConstraintPart;								///< Constraint part that removes all rotational freedom
};








// The path for a path constraint. It allows attaching two bodies to each other while giving the second body the freedom to move along a path relative to the first.
class MOSS_EXPORT PathConstraintPath : public SerializableObject, public RefTarget<PathConstraintPath>
{
	MOSS_DECLARE_SERIALIZABLE_ABSTRACT(MOSS_EXPORT, PathConstraintPath)

public:
	using PathResult = Result<Ref<PathConstraintPath>>;

	/// Virtual destructor to ensure that derived types get their destructors called
	virtual				~PathConstraintPath() override = default;

	/// Gets the max fraction along the path. I.e. sort of the length of the path.
	virtual float		GetPathMaxFraction() const = 0;

	/// Get the globally closest point on the curve (Could be slow!)
	/// @param inPosition Position to find closest point for
	/// @param inFractionHint Last known fraction along the path (can be used to speed up the search)
	/// @return Fraction of closest point along the path
	virtual float		GetClosestPoint(Vec3Arg inPosition, float inFractionHint) const = 0;

	/// Given the fraction along the path, get the point, tangent and normal.
	/// @param inFraction Fraction along the path [0, GetPathMaxFraction()].
	/// @param outPathPosition Returns the closest position to inSearchPosition on the path.
	/// @param outPathTangent Returns the tangent to the path at outPathPosition (the vector that follows the direction of the path)
	/// @param outPathNormal Return the normal to the path at outPathPosition (a vector that's perpendicular to outPathTangent)
	/// @param outPathBinormal Returns the binormal to the path at outPathPosition (a vector so that normal cross tangent = binormal)
	virtual void		GetPointOnPath(float inFraction, Vec3 &outPathPosition, Vec3 &outPathTangent, Vec3 &outPathNormal, Vec3 &outPathBinormal) const = 0;

	/// If the path is looping or not. If a path is looping, the first and last point are automatically connected to each other. They should not be the same points.
	void				SetIsLooping(bool inIsLooping)						{ mIsLooping = inIsLooping; }
	bool				IsLooping() const									{ return mIsLooping; }

#ifndef MOSS_DEBUG_RENDERER
	/// Draw the path relative to inBaseTransform. Used for debug purposes.
	void				DrawPath(DebugRenderer *inRenderer, RMat44Arg inBaseTransform) const;
#endif // MOSS_DEBUG_RENDERER

	/// Saves the contents of the path in binary form to inStream.
	virtual void		SaveBinaryState(StreamOut &inStream) const;

	/// Creates a Shape of the correct type and restores its contents from the binary stream inStream.
	static PathResult	sRestoreFromBinaryState(StreamIn &inStream);

protected:
	/// This function should not be called directly, it is used by sRestoreFromBinaryState.
	virtual void		RestoreBinaryState(StreamIn &inStream);

private:
	/// If the path is looping or not. If a path is looping, the first and last point are automatically connected to each other. They should not be the same points.
	bool				mIsLooping = false;
};



// Fixed constraint settings, used to create a fixed constraint
class MOSS_EXPORT FixedConstraintSettings final : public TwoBodyConstraintSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, FixedConstraintSettings)

public:
	// See: ConstraintSettings::SaveBinaryState
	virtual void				SaveBinaryState(StreamOut &inStream) const override;

	/// Create an instance of this constraint
	virtual TwoBodyConstraint *	Create(Body &inBody1, Body &inBody2) const override;

	/// This determines in which space the constraint is setup, all properties below should be in the specified space
	EConstraintSpace			mSpace = EConstraintSpace::WorldSpace;

	/// When mSpace is WorldSpace mPoint1 and mPoint2 can be automatically calculated based on the positions of the bodies when the constraint is created (they will be fixated in their current relative position/orientation). Set this to false if you want to supply the attachment points yourself.
	bool						mAutoDetectPoint = false;

	/// Body 1 constraint reference frame (space determined by mSpace)
	RVec3						mPoint1 = RVec3::sZero();
	Vec3						mAxisX1 = Vec3::sAxisX();
	Vec3						mAxisY1 = Vec3::sAxisY();

	/// Body 2 constraint reference frame (space determined by mSpace)
	RVec3						mPoint2 = RVec3::sZero();
	Vec3						mAxisX2 = Vec3::sAxisX();
	Vec3						mAxisY2 = Vec3::sAxisY();

protected:
	// See: ConstraintSettings::RestoreBinaryState
	virtual void				RestoreBinaryState(StreamIn &inStream) override;
};

/// A fixed constraint welds two bodies together removing all degrees of freedom between them.
/// This variant uses Euler angles for the rotation constraint.
class MOSS_EXPORT FixedConstraint final : public TwoBodyConstraint
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
								FixedConstraint(Body &inBody1, Body &inBody2, const FixedConstraintSettings &inSettings);

	// Generic interface of a constraint
	virtual EConstraintSubType	GetSubType() const override									{ return EConstraintSubType::Fixed; }
	virtual void				NotifyShapeChanged(const BodyID &inBodyID, Vec3Arg inDeltaCOM) override;
	virtual void				SetupVelocityConstraint(float inDeltaTime) override;
	virtual void				ResetWarmStart() override;
	virtual void				WarmStartVelocityConstraint(float inWarmStartImpulseRatio) override;
	virtual bool				SolveVelocityConstraint(float inDeltaTime) override;
	virtual bool				SolvePositionConstraint(float inDeltaTime, float inBaumgarte) override;
#ifndef MOSS_DEBUG_RENDERER
	virtual void				DrawConstraint(DebugRenderer *inRenderer) const override;
#endif // MOSS_DEBUG_RENDERER
	virtual void				SaveState(StateRecorder &inStream) const override;
	virtual void				RestoreState(StateRecorder &inStream) override;
	virtual Ref<ConstraintSettings> GetConstraintSettings() const override;

	// See: TwoBodyConstraint
	virtual Mat44				GetConstraintToBody1Matrix() const override					{ return Mat44::sTranslation(mLocalSpacePosition1); }
	virtual Mat44				GetConstraintToBody2Matrix() const override					{ return Mat44::sRotationTranslation(mInvInitialOrientation, mLocalSpacePosition2); }

	///@name Get Lagrange multiplier from last physics update (the linear/angular impulse applied to satisfy the constraint)
	inline Vec3					GetTotalLambdaPosition() const								{ return mPointConstraintPart.GetTotalLambda(); }
	inline Vec3					GetTotalLambdaRotation() const								{ return mRotationConstraintPart.GetTotalLambda(); }

private:
	// CONFIGURATION PROPERTIES FOLLOW

	// Local space constraint positions
	Vec3						mLocalSpacePosition1;
	Vec3						mLocalSpacePosition2;

	// Inverse of initial rotation from body 1 to body 2 in body 1 space
	Quat						mInvInitialOrientation;

	// RUN TIME PROPERTIES FOLLOW

	// The constraint parts
	RotationEulerConstraintPart	mRotationConstraintPart;
	PointConstraintPart			mPointConstraintPart;
};



// A path that follows a Hermite spline
class MOSS_EXPORT PathConstraintPathHermite final : public PathConstraintPath
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, PathConstraintPathHermite)

public:
	// See PathConstraintPath::GetPathMaxFraction
	virtual float		GetPathMaxFraction() const override									{ return float(IsLooping()? mPoints.size() : mPoints.size() - 1); }

	// See PathConstraintPath::GetClosestPoint
	virtual float		GetClosestPoint(Vec3Arg inPosition, float inFractionHint) const override;

	// See PathConstraintPath::GetPointOnPath
	virtual void		GetPointOnPath(float inFraction, Vec3 &outPathPosition, Vec3 &outPathTangent, Vec3 &outPathNormal, Vec3 &outPathBinormal) const override;

	/// Adds a point to the path
	void				AddPoint(Vec3Arg inPosition, Vec3Arg inTangent, Vec3Arg inNormal)	{ mPoints.push_back({ inPosition, inTangent, inNormal}); }

	// See: PathConstraintPath::SaveBinaryState
	virtual void		SaveBinaryState(StreamOut &inStream) const override;

	struct Point
	{
		MOSS_DECLARE_SERIALIZABLE_NON_VIRTUAL(MOSS_EXPORT, Point)

		Vec3			mPosition;															///< Position on the path
		Vec3			mTangent;															///< Tangent of the path, does not need to be normalized (in the direction of the path)
		Vec3			mNormal;															///< Normal of the path (together with the tangent along the curve this forms a basis for the constraint)
	};

protected:
	// See: PathConstraintPath::RestoreBinaryState
	virtual void		RestoreBinaryState(StreamIn &inStream) override;

private:
	/// Helper function that returns the index of the path segment and the fraction t on the path segment based on the full path fraction
	inline void			GetIndexAndT(float inFraction, int &outIndex, float &outT) const;

	using Points = TArray<Point>;

	Points				mPoints;															///< Points on the Hermite spline
};





/// Pulley constraint settings, used to create a pulley constraint.
/// A pulley connects two bodies via two fixed world points to each other similar to a distance constraint.
/// We define Length1 = |BodyPoint1 - FixedPoint1| where Body1 is a point on body 1 in world space and FixedPoint1 a fixed point in world space
/// Length2 = |BodyPoint2 - FixedPoint2|
/// The constraint keeps the two line segments constrained so that
/// MinDistance <= Length1 + Ratio * Length2 <= MaxDistance
class MOSS_EXPORT PulleyConstraintSettings final : public TwoBodyConstraintSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, PulleyConstraintSettings)

public:
	// See: ConstraintSettings::SaveBinaryState
	virtual void				SaveBinaryState(StreamOut &inStream) const override;

	/// Create an instance of this constraint
	virtual TwoBodyConstraint *	Create(Body &inBody1, Body &inBody2) const override;

	/// This determines in which space the constraint is setup, specified properties below should be in the specified space
	EConstraintSpace			mSpace = EConstraintSpace::WorldSpace;

	/// Body 1 constraint attachment point (space determined by mSpace).
	RVec3						mBodyPoint1 = RVec3::sZero();

	/// Fixed world point to which body 1 is connected (always world space)
	RVec3						mFixedPoint1 = RVec3::sZero();

	/// Body 2 constraint attachment point (space determined by mSpace)
	RVec3						mBodyPoint2 = RVec3::sZero();

	/// Fixed world point to which body 2 is connected (always world space)
	RVec3						mFixedPoint2 = RVec3::sZero();

	/// Ratio between the two line segments (see formula above), can be used to create a block and tackle
	float						mRatio = 1.0f;

	/// The minimum length of the line segments (see formula above), use -1 to calculate the length based on the positions of the objects when the constraint is created.
	float						mMinLength = 0.0f;

	/// The maximum length of the line segments (see formula above), use -1 to calculate the length based on the positions of the objects when the constraint is created.
	float						mMaxLength = -1.0f;

protected:
	// See: ConstraintSettings::RestoreBinaryState
	virtual void				RestoreBinaryState(StreamIn &inStream) override;
};

/// A pulley constraint.
class MOSS_EXPORT PulleyConstraint final : public TwoBodyConstraint
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Construct distance constraint
								PulleyConstraint(Body &inBody1, Body &inBody2, const PulleyConstraintSettings &inSettings);

	// Generic interface of a constraint
	virtual EConstraintSubType	GetSubType() const override									{ return EConstraintSubType::Pulley; }
	virtual void				NotifyShapeChanged(const BodyID &inBodyID, Vec3Arg inDeltaCOM) override;
	virtual void				SetupVelocityConstraint(float inDeltaTime) override;
	virtual void				ResetWarmStart() override;
	virtual void				WarmStartVelocityConstraint(float inWarmStartImpulseRatio) override;
	virtual bool				SolveVelocityConstraint(float inDeltaTime) override;
	virtual bool				SolvePositionConstraint(float inDeltaTime, float inBaumgarte) override;
#ifndef MOSS_DEBUG_RENDERER
	virtual void				DrawConstraint(DebugRenderer *inRenderer) const override;
#endif // MOSS_DEBUG_RENDERER
	virtual void				SaveState(StateRecorder &inStream) const override;
	virtual void				RestoreState(StateRecorder &inStream) override;
	virtual Ref<ConstraintSettings> GetConstraintSettings() const override;

	// See: TwoBodyConstraint
	virtual Mat44				GetConstraintToBody1Matrix() const override					{ return Mat44::sTranslation(mLocalSpacePosition1); }
	virtual Mat44				GetConstraintToBody2Matrix() const override					{ return Mat44::sTranslation(mLocalSpacePosition2); } // Note: Incorrect rotation as we don't track the original rotation difference, should not matter though as the constraint is not limiting rotation.

	/// Update the minimum and maximum length for the constraint
	void						SetLength(float inMinLength, float inMaxLength)				{ MOSS_ASSERT(inMinLength >= 0.0f && inMinLength <= inMaxLength); mMinLength = inMinLength; mMaxLength = inMaxLength; }
	float						GetMinLength() const										{ return mMinLength; }
	float						GetMaxLength() const										{ return mMaxLength; }

	/// Get the current length of both segments (multiplied by the ratio for segment 2)
	float						GetCurrentLength() const									{ return Vec3(mWorldSpacePosition1 - mFixedPosition1).Length() + mRatio * Vec3(mWorldSpacePosition2 - mFixedPosition2).Length(); }

	///@name Get Lagrange multiplier from last physics update (the linear impulse applied to satisfy the constraint)
	inline float				GetTotalLambdaPosition() const								{ return mIndependentAxisConstraintPart.GetTotalLambda(); }

private:
	// Calculates world positions and normals and returns current length
	float						CalculatePositionsNormalsAndLength();

	// Internal helper function to calculate the values below
	void						CalculateConstraintProperties();

	// CONFIGURATION PROPERTIES FOLLOW

	// Local space constraint positions on the bodies
	Vec3						mLocalSpacePosition1;
	Vec3						mLocalSpacePosition2;

	// World space fixed positions
	RVec3						mFixedPosition1;
	RVec3						mFixedPosition2;

	/// Ratio between the two line segments
	float						mRatio;

	// The minimum/maximum length of the line segments
	float						mMinLength;
	float						mMaxLength;

	// RUN TIME PROPERTIES FOLLOW

	// World space positions and normal
	RVec3						mWorldSpacePosition1;
	RVec3						mWorldSpacePosition2;
	Vec3						mWorldSpaceNormal1;
	Vec3						mWorldSpaceNormal2;

	// Depending on if the length < min or length > max we can apply forces to prevent further violations
	float						mMinLambda;
	float						mMaxLambda;

	// The constraint part
	IndependentAxisConstraintPart mIndependentAxisConstraintPart;
};


// Rack and pinion constraint (slider & gear) settings
class MOSS_EXPORT RackAndPinionConstraintSettings final : public TwoBodyConstraintSettings
{
	MOSS_DECLARE_SERIALIZABLE_VIRTUAL(MOSS_EXPORT, RackAndPinionConstraintSettings)

public:
	// See: ConstraintSettings::SaveBinaryState
	virtual void				SaveBinaryState(StreamOut &inStream) const override;

	/// Create an instance of this constraint.
	/// Body1 should be the pinion (gear) and body 2 the rack (slider).
	virtual TwoBodyConstraint *	Create(Body &inBody1, Body &inBody2) const override;

	/// Defines the ratio between the rotation of the pinion and the translation of the rack.
	/// The ratio is defined as: PinionRotation(t) = ratio * RackTranslation(t)
	/// @param inNumTeethRack Number of teeth that the rack has
	/// @param inRackLength Length of the rack
	/// @param inNumTeethPinion Number of teeth the pinion has
	void						SetRatio(int inNumTeethRack, float inRackLength, int inNumTeethPinion)
	{
		mRatio = 2.0f * MOSS_PI * inNumTeethRack / (inRackLength * inNumTeethPinion);
	}

	/// This determines in which space the constraint is setup, all properties below should be in the specified space
	EConstraintSpace			mSpace = EConstraintSpace::WorldSpace;

	/// Body 1 (pinion) constraint reference frame (space determined by mSpace).
	Vec3						mHingeAxis = Vec3::sAxisX();

	/// Body 2 (rack) constraint reference frame (space determined by mSpace)
	Vec3						mSliderAxis = Vec3::sAxisX();

	/// Ratio between the rack and pinion, see SetRatio.
	float						mRatio = 1.0f;

protected:
	// See: ConstraintSettings::RestoreBinaryState
	virtual void				RestoreBinaryState(StreamIn &inStream) override;
};

/// A rack and pinion constraint constrains the rotation of body1 to the translation of body 2.
/// Note that this constraint needs to be used in conjunction with a hinge constraint for body 1 and a slider constraint for body 2.
class MOSS_EXPORT RackAndPinionConstraint final : public TwoBodyConstraint
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Construct gear constraint
								RackAndPinionConstraint(Body &inBody1, Body &inBody2, const RackAndPinionConstraintSettings &inSettings);

	// Generic interface of a constraint
	virtual EConstraintSubType	GetSubType() const override												{ return EConstraintSubType::RackAndPinion; }
	virtual void				NotifyShapeChanged(const BodyID &inBodyID, Vec3Arg inDeltaCOM) override { /* Nothing */ }
	virtual void				SetupVelocityConstraint(float inDeltaTime) override;
	virtual void				ResetWarmStart() override;
	virtual void				WarmStartVelocityConstraint(float inWarmStartImpulseRatio) override;
	virtual bool				SolveVelocityConstraint(float inDeltaTime) override;
	virtual bool				SolvePositionConstraint(float inDeltaTime, float inBaumgarte) override;
#ifndef MOSS_DEBUG_RENDERER
	virtual void				DrawConstraint(DebugRenderer *inRenderer) const override;
#endif // MOSS_DEBUG_RENDERER
	virtual void				SaveState(StateRecorder &inStream) const override;
	virtual void				RestoreState(StateRecorder &inStream) override;
	virtual Ref<ConstraintSettings> GetConstraintSettings() const override;

	// See: TwoBodyConstraint
	virtual Mat44				GetConstraintToBody1Matrix() const override;
	virtual Mat44				GetConstraintToBody2Matrix() const override;

	/// The constraints that constrain the rack and pinion (a slider and a hinge), optional and used to calculate the position error and fix numerical drift.
	void						SetConstraints(const Constraint *inPinion, const Constraint *inRack)	{ mPinionConstraint = inPinion; mRackConstraint = inRack; }

	///@name Get Lagrange multiplier from last physics update (the linear/angular impulse applied to satisfy the constraint)
	inline float				GetTotalLambda() const													{ return mRackAndPinionConstraintPart.GetTotalLambda(); }

private:
	// Internal helper function to calculate the values below
	void						CalculateConstraintProperties(Mat44Arg inRotation1, Mat44Arg inRotation2);

	// CONFIGURATION PROPERTIES FOLLOW

	// Local space hinge axis
	Vec3						mLocalSpaceHingeAxis;

	// Local space sliding direction
	Vec3						mLocalSpaceSliderAxis;

	// Ratio between rack and pinion
	float						mRatio;

	// The constraints that constrain the rack and pinion (a slider and a hinge), optional and used to calculate the position error and fix numerical drift.
	RefConst<Constraint>		mPinionConstraint;
	RefConst<Constraint>		mRackConstraint;

	// RUN TIME PROPERTIES FOLLOW

	// World space hinge axis
	Vec3						mWorldSpaceHingeAxis;

	// World space sliding direction
	Vec3						mWorldSpaceSliderAxis;

	// The constraint parts
	RackAndPinionConstraintPart	mRackAndPinionConstraintPart;
};






using Constraints = TArray<Ref<Constraint>>;

#endif
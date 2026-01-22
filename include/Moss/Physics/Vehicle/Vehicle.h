#ifndef VEHICLE3D_H
#define VEHICLE3D_H

class VehicleControllerSettings : public SerializableObject, public RefTarget<VehicleControllerSettings> {
public:
	MOSS_OVERRIDE_NEW_DELETE
	// Saves the contents of the controller settings in binary form to inStream.
	virtual void				SaveBinaryState(StreamOut& inStream) const = 0;

	// Restore the contents of the controller settings in binary form from inStream.
	virtual void				RestoreBinaryState(StreamIn& inStream) = 0;

	// Create an instance of the vehicle controller class
	virtual VehicleController*	ConstructController(VehicleConstraint& inConstraint) const = 0;
};

class VehicleConstraintSettings : public ConstraintSettings {
	MOSS_OVERRIDE_NEW_DELETE
	Vec3						mUp { 0, 1, 0 };							// Vector indicating the up direction of the vehicle (in local space to the body)
	Vec3						mForward { 0, 0, 1 };						// Vector indicating forward direction of the vehicle (in local space to the body)
	float						mMaxPitchRollAngle = MOSS_PI;				// Defines the maximum pitch/roll angle (rad), can be used to avoid the car from getting upside down. The vehicle up direction will stay within a cone centered around the up axis with half top angle mMaxPitchRollAngle, set to pi to turn off.
	TArray<Ref<WheelSettings>>	mWheels;									// List of wheels and their properties
	VehicleAntiRollBars			mAntiRollBars;								// List of anti rollbars and their properties
	Ref<VehicleControllerSettings> mController;								// Defines how the vehicle can accelerate / decelerate

protected:
	// This function should not be called directly, it is used by sRestoreFromBinaryState.
	virtual void				RestoreBinaryState(StreamIn& inStream) override;
};

class MOSS_API MotorSettings {
public:
	// Constructor
	MotorSettings() = default;
	MotorSettings(const MotorSettings &) = default;
	MotorSettings&	operator = (const MotorSettings &) = default;
	MotorSettings(float inFrequency, float inDamping) : mSpringSettings(ESpringMode::FrequencyAndDamping, inFrequency, inDamping) { MOSS_ASSERT(IsValid()); }
	MotorSettings(float inFrequency, float inDamping, float inForceLimit, float inTorqueLimit) : mSpringSettings(ESpringMode::FrequencyAndDamping, inFrequency, inDamping), mMinForceLimit(-inForceLimit), mMaxForceLimit(inForceLimit), mMinTorqueLimit(-inTorqueLimit), mMaxTorqueLimit(inTorqueLimit) { MOSS_ASSERT(IsValid()); }

	// Set asymmetric force limits
	void SetForceLimits(float inMin, float inMax)	{ MOSS_ASSERT(inMin <= inMax); mMinForceLimit = inMin; mMaxForceLimit = inMax; }

	// Set asymmetric torque limits
	void SetTorqueLimits(float inMin, float inMax)	{ MOSS_ASSERT(inMin <= inMax); mMinTorqueLimit = inMin; mMaxTorqueLimit = inMax; }

	// Set symmetric force limits
	void SetForceLimit(float inLimit) { mMinForceLimit = -inLimit; mMaxForceLimit = inLimit; }

	// Set symmetric torque limits
	void SetTorqueLimit(float inLimit) { mMinTorqueLimit = -inLimit; mMaxTorqueLimit = inLimit; }

	// Check if settings are valid
	bool IsValid() const { return mSpringSettings.mFrequency >= 0.0f && mSpringSettings.mDamping >= 0.0f && mMinForceLimit <= mMaxForceLimit && mMinTorqueLimit <= mMaxTorqueLimit; }

	// Saves the contents of the motor settings in binary form to inStream.
	void SaveBinaryState(StreamOut &inStream) const;

	// Restores contents from the binary stream inStream.
	void RestoreBinaryState(StreamIn &inStream);

	// Settings
	SpringSettings			mSpringSettings { ESpringMode::FrequencyAndDamping, 2.0f, 1.0f }; // Settings for the spring that is used to drive to the position target (not used when motor is a velocity motor).
	float					mMinForceLimit = -FLT_MAX;					// Minimum force to apply in case of a linear constraint (N). Usually this is -mMaxForceLimit unless you want a motor that can e.g. push but not pull. Not used when motor is an angular motor.
	float					mMaxForceLimit = FLT_MAX;					// Maximum force to apply in case of a linear constraint (N). Not used when motor is an angular motor.
	float					mMinTorqueLimit = -FLT_MAX;					// Minimum torque to apply in case of a angular constraint (N m). Usually this is -mMaxTorqueLimit unless you want a motor that can e.g. push but not pull. Not used when motor is a position motor.
	float					mMaxTorqueLimit = FLT_MAX;					// Maximum torque to apply in case of a angular constraint (N m). Not used when motor is a position motor.
};

struct WheelSettings {
	MOSS_OVERRIDE_NEW_DELETE
	Vec3					mPosition { 0, 0, 0 };						// Attachment point of wheel suspension in local space of the body
	Vec3					mSuspensionForcePoint { 0, 0, 0 };			// Where tire forces (suspension and traction) are applied, in local space of the body. A good default is the center of the wheel in its neutral pose. See mEnableSuspensionForcePoint.
	Vec3					mSuspensionDirection { 0, -1, 0 };			// Direction of the suspension in local space of the body, should point down
	Vec3					mSteeringAxis { 0, 1, 0 };					// Direction of the steering axis in local space of the body, should point up (e.g. for a bike would be -mSuspensionDirection)
	Vec3					mWheelUp { 0, 1, 0 };						// Up direction when the wheel is in the neutral steering position (usually VehicleConstraintSettings::mUp but can be used to give the wheel camber or for a bike would be -mSuspensionDirection)
	Vec3					mWheelForward { 0, 0, 1 };					// Forward direction when the wheel is in the neutral steering position (usually VehicleConstraintSettings::mForward but can be used to give the wheel toe, does not need to be perpendicular to mWheelUp)
	float					mSuspensionMinLength = 0.3f;				// How long the suspension is in max raised position relative to the attachment point (m)
	float					mSuspensionMaxLength = 0.5f;				// How long the suspension is in max droop position relative to the attachment point (m)
	float					mSuspensionPreloadLength = 0.0f;			// The natural length (m) of the suspension spring is defined as mSuspensionMaxLength + mSuspensionPreloadLength. Can be used to preload the suspension as the spring is compressed by mSuspensionPreloadLength when the suspension is in max droop position. Note that this means when the vehicle touches the ground there is a discontinuity so it will also make the vehicle more bouncy as we're updating with discrete time steps.
	SpringSettings			mSuspensionSpring { ESpringMode::FrequencyAndDamping, 1.5f, 0.5f }; // Settings for the suspension spring
	float					mRadius = 0.3f;								// Radius of the wheel (m)
	float					mWidth = 0.1f;								// Width of the wheel (m)
	bool					mEnableSuspensionForcePoint = false;		// Enables mSuspensionForcePoint, if disabled, the forces are applied at the collision contact point. This leads to a more accurate simulation when interacting with dynamic objects but makes the vehicle less stable. When setting this to true, all forces will be applied to a fixed point on the vehicle body.
};

struct WheelSettingsWV : public WheelSettings {
	MOSS_OVERRIDE_NEW_DELETE
	float						mInertia = 0.9f;							// Moment of inertia (kg m^2), for a cylinder this would be 0.5* M* R^2 which is 0.9 for a wheel with a mass of 20 kg and radius 0.3 m
	float						mAngularDamping = 0.2f;						// Angular damping factor of the wheel: dw/dt = -c* w
	float						mMaxSteerAngle = DegreesToRadians(70.0f);	// How much this wheel can steer (radians)
	LinearCurve					mLongitudinalFriction;						// On the Y-axis: friction in the forward direction of the tire. Friction is normally between 0 (no friction) and 1 (full friction) although friction can be a little bit higher than 1 because of the profile of a tire. On the X-axis: the slip ratio (fraction) defined as (omega_wheel* r_wheel - v_longitudinal) / |v_longitudinal|. You can see slip ratio as the amount the wheel is spinning relative to the floor: 0 means the wheel has full traction and is rolling perfectly in sync with the ground, 1 is for example when the wheel is locked and sliding over the ground.
	LinearCurve					mLateralFriction;							// On the Y-axis: friction in the sideways direction of the tire. Friction is normally between 0 (no friction) and 1 (full friction) although friction can be a little bit higher than 1 because of the profile of a tire. On the X-axis: the slip angle (degrees) defined as angle between relative contact velocity and tire direction.
	float						mMaxBrakeTorque = 1500.0f;					// How much torque (Nm) the brakes can apply to this wheel
	float						mMaxHandBrakeTorque = 4000.0f;				// How much torque (Nm) the hand brake can apply to this wheel (usually only applied to the rear wheels)
};
 WheelSettingsTV : public WheelSettings {
	MOSS_OVERRIDE_NEW_DELETE
	// See: WheelSettings
	virtual void				SaveBinaryState(StreamOut& inStream) const override;
	virtual void				RestoreBinaryState(StreamIn& inStream) override;

	float						mLongitudinalFriction = 4.0f;				// Friction in forward direction of tire
	float						mLateralFriction = 2.0f;					// Friction in sideways direction of tire
};


class VehicleConstraint : public Constraint, public PhysicsStepListener {
public:
	// Constructor / destructor
								VehicleConstraint(Body& inVehicleBody, const VehicleConstraintSettings& inSettings);
	virtual						~VehicleConstraint() override;

	// Get the type of a constraint
	virtual EConstraintSubType	GetSubType() const override					{ return EConstraintSubType::Vehicle; }

	// Defines the maximum pitch/roll angle (rad), can be used to avoid the car from getting upside down. The vehicle up direction will stay within a cone centered around the up axis with half top angle mMaxPitchRollAngle, set to pi to turn off.
	void						SetMaxPitchRollAngle(float inMaxPitchRollAngle) { mCosMaxPitchRollAngle = Cos(inMaxPitchRollAngle); }

	// Set the interface that tests collision between wheel and ground
	void						SetVehicleCollisionTester(const VehicleCollisionTester*inTester) { mVehicleCollisionTester = inTester; }

	// Callback function to combine the friction of a tire with the friction of the body it is colliding with.
	// On input ioLongitudinalFriction and ioLateralFriction contain the friction of the tire, on output they should contain the combined friction with inBody2.
	using CombineFunction = function<void(uint8 inWheelIndex, float& ioLongitudinalFriction, float& ioLateralFriction, const Body& inBody2, const SubShapeID& inSubShapeID2)>;

	// Set the function that combines the friction of two bodies and returns it
	// Default method is the geometric mean: sqrt(friction1* friction2).
	void						SetCombineFriction(const CombineFunction& inCombineFriction) { mCombineFriction = inCombineFriction; }
	const CombineFunction& 		GetCombineFriction() const					{ return mCombineFriction; }

	// Callback function to notify of current stage in PhysicsStepListener::OnStep.
	using StepCallback = function<void(VehicleConstraint& inVehicle, const PhysicsStepListenerContext& inContext)>;

	// Callback function to notify that PhysicsStepListener::OnStep has started for this vehicle. Default is to do nothing.
	// Can be used to allow higher-level code to e.g. control steering. This is the last moment that the position/orientation of the vehicle can be changed.
	// Wheel collision checks have not been performed yet.
	const StepCallback& 		GetPreStepCallback() const					{ return mPreStepCallback; }
	void						SetPreStepCallback(const StepCallback& inPreStepCallback) { mPreStepCallback = inPreStepCallback; }

	// Callback function to notify that PhysicsStepListener::OnStep has just completed wheel collision checks. Default is to do nothing.
	// Can be used to allow higher-level code to e.g. detect tire contact or to modify the velocity of the vehicle based on the wheel contacts.
	// You should not change the position of the vehicle in this callback as the wheel collision checks have already been performed.
	const StepCallback& 		GetPostCollideCallback() const				{ return mPostCollideCallback; }
	void						SetPostCollideCallback(const StepCallback& inPostCollideCallback) { mPostCollideCallback = inPostCollideCallback; }

	// Callback function to notify that PhysicsStepListener::OnStep has completed for this vehicle. Default is to do nothing.
	// Can be used to allow higher-level code to e.g. control the vehicle in the air.
	// You should not change the position of the vehicle in this callback as the wheel collision checks have already been performed.
	const StepCallback& 		GetPostStepCallback() const					{ return mPostStepCallback; }
	void						SetPostStepCallback(const StepCallback& inPostStepCallback) { mPostStepCallback = inPostStepCallback; }

	// Override gravity for this vehicle. Note that overriding gravity will set the gravity factor of the vehicle body to 0 and apply gravity in the PhysicsStepListener instead.
	void						OverrideGravity(Vec3Arg inGravity)			{ mGravityOverride = inGravity; mIsGravityOverridden = true; }
	bool						IsGravityOverridden() const					{ return mIsGravityOverridden; }
	Vec3						GetGravityOverride() const					{ return mGravityOverride; }
	void						ResetGravityOverride()						{ mIsGravityOverridden = false; mBody->GetMotionProperties()->SetGravityFactor(1.0f); } // Note that resetting the gravity override will restore the gravity factor of the vehicle body to 1.

	// Get the local space forward vector of the vehicle
	Vec3						GetLocalForward() const						{ return mForward; }

	// Get the local space up vector of the vehicle
	Vec3						GetLocalUp() const							{ return mUp; }

	// Vector indicating the world space up direction (used to limit vehicle pitch/roll), calculated every frame by inverting gravity
	Vec3						GetWorldUp() const							{ return mWorldUp; }

	// Access to the vehicle body
	Body*						GetVehicleBody() const						{ return mBody; }

	// Access to the vehicle controller interface (determines acceleration / deceleration)
	const VehicleController*	GetController() const						{ return mController; }

	// Access to the vehicle controller interface (determines acceleration / deceleration)
	VehicleController*			GetController()								{ return mController; }

	// Get the state of the wheels
	const Wheels& 				GetWheels() const							{ return mWheels; }

	// Get the state of a wheels (writable interface, allows you to make changes to the configuration which will take effect the next time step)
	Wheels& 					GetWheels()									{ return mWheels; }

	// Get the state of a wheel
	Wheel*						GetWheel(uint8 inIdx)						{ return mWheels[inIdx]; }
	const Wheel*				GetWheel(uint8 inIdx) const					{ return mWheels[inIdx]; }

	// Get the basis vectors for the wheel in local space to the vehicle body (note: basis does not rotate when the wheel rotates around its axis)
	// @param inWheel Wheel to fetch basis for
	// @param outForward Forward vector for the wheel
	// @param outUp Up vector for the wheel
	// @param outRight Right vector for the wheel
	void						GetWheelLocalBasis(const Wheel*inWheel, Vec3& outForward, Vec3& outUp, Vec3& outRight) const;

	// Get the transform of a wheel in local space to the vehicle body, returns a matrix that transforms a cylinder aligned with the Y axis in body space (not COM space)
	// @param inWheelIndex Index of the wheel to fetch
	// @param inWheelRight Unit vector that indicates right in model space of the wheel (so if you only have 1 wheel model, you probably want to specify the opposite direction for the left and right wheels)
	// @param inWheelUp Unit vector that indicates up in model space of the wheel
	Mat44						GetWheelLocalTransform(uint8 inWheelIndex, Vec3Arg inWheelRight, Vec3Arg inWheelUp) const;

	// Get the transform of a wheel in world space, returns a matrix that transforms a cylinder aligned with the Y axis in world space
	// @param inWheelIndex Index of the wheel to fetch
	// @param inWheelRight Unit vector that indicates right in model space of the wheel (so if you only have 1 wheel model, you probably want to specify the opposite direction for the left and right wheels)
	// @param inWheelUp Unit vector that indicates up in model space of the wheel
	RMat44						GetWheelWorldTransform(uint8 inWheelIndex, Vec3Arg inWheelRight, Vec3Arg inWheelUp) const;

	// Access to the vehicle's anti roll bars
	const VehicleAntiRollBars& 	GetAntiRollBars() const						{ return mAntiRollBars; }
	VehicleAntiRollBars& 		GetAntiRollBars()							{ return mAntiRollBars; }

	// Number of simulation steps between wheel collision tests when the vehicle is active. Default is 1. 0 = never, 1 = every step, 2 = every other step, etc.
	// Note that if a vehicle has multiple wheels and the number of steps > 1, the wheels will be tested in a round robin fashion.
	// If there are multiple vehicles, the tests will be spread out based on the BodyID of the vehicle.
	// If you set this to test less than every step, you may see simulation artifacts. This setting can be used to reduce the cost of simulating vehicles in the distance.
	void						SetNumStepsBetweenCollisionTestActive(uint8 inSteps) { mNumStepsBetweenCollisionTestActive = inSteps; }
	uint8						GetNumStepsBetweenCollisionTestActive() const { return mNumStepsBetweenCollisionTestActive; }

	// Number of simulation steps between wheel collision tests when the vehicle is inactive. Default is 1. 0 = never, 1 = every step, 2 = every other step, etc.
	// Note that if a vehicle has multiple wheels and the number of steps > 1, the wheels will be tested in a round robin fashion.
	// If there are multiple vehicles, the tests will be spread out based on the BodyID of the vehicle.
	// This number can be lower than the number of steps when the vehicle is active as the only purpose of this test is
	// to allow the vehicle to wake up in response to bodies moving into the wheels but not touching the body of the vehicle.
	void						SetNumStepsBetweenCollisionTestInactive(uint8 inSteps) { mNumStepsBetweenCollisionTestInactive = inSteps; }
	uint8						GetNumStepsBetweenCollisionTestInactive() const { return mNumStepsBetweenCollisionTestInactive; }

	// Generic interface of a constraint
	virtual bool				IsActive() const override					{ return mIsActive& & Constraint::IsActive(); }
	virtual void				NotifyShapeChanged(const BodyID& inBodyID, Vec3Arg inDeltaCOM) override { /* Do nothing*/ }
	virtual void				SetupVelocityConstraint(float inDeltaTime) override;
	virtual void				ResetWarmStart() override;
	virtual void				WarmStartVelocityConstraint(float inWarmStartImpulseRatio) override;
	virtual bool				SolveVelocityConstraint(float inDeltaTime) override;
	virtual bool				SolvePositionConstraint(float inDeltaTime, float inBaumgarte) override;
	virtual void				BuildIslands(uint32 inConstraintIndex, IslandBuilder& ioBuilder, BodyManager& inBodyManager) override;
	virtual uint8				BuildIslandSplits(LargeIslandSplitter& ioSplitter) const override;
#ifndef MOSS_DEBUG_RENDERER
	virtual void				DrawConstraint(DebugRenderer*inRenderer) const override;
	virtual void				DrawConstraintLimits(DebugRenderer*inRenderer) const override;
#endif // MOSS_DEBUG_RENDERER
	virtual void				SaveState(StateRecorder& inStream) const override;
	virtual void				RestoreState(StateRecorder& inStream) override;
	virtual Ref<ConstraintSettings> GetConstraintSettings() const override;

private:
	// See: PhysicsStepListener
	virtual void				OnStep(const PhysicsStepListenerContext& inContext) override;

	// Calculate the position where the suspension and traction forces should be applied in world space, relative to the center of mass of both bodies
	void						CalculateSuspensionForcePoint(const Wheel& inWheel, Vec3& outR1PlusU, Vec3& outR2) const;

	// Calculate the constraint properties for mPitchRollPart
	void						CalculatePitchRollConstraintProperties(RMat44Arg inBodyTransform);

	// Gravity override
	bool						mIsGravityOverridden = false;				// If the gravity is currently overridden
	Vec3						mGravityOverride = Vec3::sZero();			// Gravity override value, replaces PhysicsSystem::GetGravity() when mIsGravityOverridden is true

	// Simulation information
	Body*						mBody;										// Body of the vehicle
	Vec3						mForward;									// Local space forward vector for the vehicle
	Vec3						mUp;										// Local space up vector for the vehicle
	Vec3						mWorldUp;									// Vector indicating the world space up direction (used to limit vehicle pitch/roll)
	Wheels						mWheels;									// Wheel states of the vehicle
	VehicleAntiRollBars			mAntiRollBars;								// Anti rollbars of the vehicle
	VehicleController*			mController;								// Controls the acceleration / deceleration of the vehicle
	bool						mIsActive = false;							// If this constraint is active
	uint8						mNumStepsBetweenCollisionTestActive = 1;	// Number of simulation steps between wheel collision tests when the vehicle is active
	uint8						mNumStepsBetweenCollisionTestInactive = 1;	// Number of simulation steps between wheel collision tests when the vehicle is inactive
	uint8						mCurrentStep = 0;							// Current step number, used to determine when to test a wheel

	// Prevent vehicle from toppling over
	float						mCosMaxPitchRollAngle;						// Cos of the max pitch/roll angle
	float						mCosPitchRollAngle;							// Cos of the current pitch/roll angle
	Vec3						mPitchRollRotationAxis { 0, 1, 0 };			// Current axis along which to apply torque to prevent the car from toppling over
	AngleConstraintPart			mPitchRollPart;								// Constraint part that prevents the car from toppling over

	// Interfaces
	RefConst<VehicleCollisionTester> mVehicleCollisionTester;				// Class that performs testing of collision for the wheels
	CombineFunction				mCombineFriction = [](uint8, float& ioLongitudinalFriction, float& ioLateralFriction, const Body& inBody2, const SubShapeID& )
	{
		float body_friction = inBody2.GetFriction();

		ioLongitudinalFriction = sqrt(ioLongitudinalFriction* body_friction);
		ioLateralFriction = sqrt(ioLateralFriction* body_friction);
	};

	// Callbacks
	StepCallback				mPreStepCallback;
	StepCallback				mPostCollideCallback;
	StepCallback				mPostStepCallback;
};

class Wheel : public NonCopyable {
public:
	MOSS_OVERRIDE_NEW_DELETE

	// Constructor / destructor
	explicit Wheel(const WheelSettings& inSettings);
	virtual	~Wheel() = default;

	// Get settings for the wheel
	const WheelSettings* GetSettings() const { return mSettings; }

	// Get the angular velocity (rad/s) for this wheel, note that positive means the wheel is rotating such that the car moves forward
	float GetAngularVelocity() const { return mAngularVelocity; }

	// Update the angular velocity (rad/s)
	void SetAngularVelocity(float inVel) { mAngularVelocity = inVel; }

	// Get the current rotation angle of the wheel in radians [0, 2 pi]
	float GetRotationAngle() const { return mAngle; }

	// Set the current rotation angle of the wheel in radians [0, 2 pi]
	void SetRotationAngle(float inAngle) { mAngle = inAngle; }

	// Get the current steer angle of the wheel in radians [-pi, pi], positive is to the left
	float GetSteerAngle() const { return mSteerAngle; }

	// Set the current steer angle of the wheel in radians [-pi, pi]
	void SetSteerAngle(float inAngle) { mSteerAngle = inAngle; }

	// Returns true if the wheel is touching an object
	inline bool	HasContact() const { return !mContactBodyID.IsInvalid(); }

	// Returns the body ID of the body that this wheel is touching
	BodyID GetContactBodyID() const { return mContactBodyID; }

	// Returns the sub shape ID where we're contacting the body
	SubShapeID GetContactSubShapeID() const { return mContactSubShapeID; }

	// Returns the current contact position in world space (note by the time you call this the vehicle has moved)
	Vec3 GetContactPosition() const	{ MOSS_ASSERT(HasContact()); return mContactPosition; }

	// Velocity of the contact point (m / s, not relative to the wheel but in world space)
	Vec3 GetContactPointVelocity() const { MOSS_ASSERT(HasContact()); return mContactPointVelocity; }

	// Returns the current contact normal in world space (note by the time you call this the vehicle has moved)
	Vec3 GetContactNormal() const { MOSS_ASSERT(HasContact()); return mContactNormal; }

	// Returns longitudinal direction (direction along the wheel relative to floor) in world space (note by the time you call this the vehicle has moved)
	Vec3 GetContactLongitudinal() const	{ MOSS_ASSERT(HasContact()); return mContactLongitudinal; }

	// Returns lateral direction (sideways direction) in world space (note by the time you call this the vehicle has moved)
	Vec3 GetContactLateral() const { MOSS_ASSERT(HasContact()); return mContactLateral; }

	// Get the length of the suspension for a wheel (m) relative to the suspension attachment point (hard point)
	float GetSuspensionLength() const { return mSuspensionLength; }

	// Check if the suspension hit its upper limit
	bool HasHitHardPoint() const { return mSuspensionMaxUpPart.IsActive(); }

	// Get the total impulse (N s) that was applied by the suspension
	float GetSuspensionLambda() const { return mSuspensionPart.GetTotalLambda() + mSuspensionMaxUpPart.GetTotalLambda(); }

	// Get total impulse (N s) applied along the forward direction of the wheel
	float GetLongitudinalLambda() const { return mLongitudinalPart.GetTotalLambda(); }

	// Get total impulse (N s) applied along the sideways direction of the wheel
	float GetLateralLambda() const { return mLateralPart.GetTotalLambda(); }

	// Internal function that should only be called by the controller. Used to apply impulses in the forward direction of the vehicle.
	bool SolveLongitudinalConstraintPart(const VehicleConstraint& inConstraint, float inMinImpulse, float inMaxImpulse);

	// Internal function that should only be called by the controller. Used to apply impulses in the sideways direction of the vehicle.
	bool SolveLateralConstraintPart(const VehicleConstraint& inConstraint, float inMinImpulse, float inMaxImpulse);

protected:
	friend class VehicleConstraint;

	RefConst<WheelSettings>	mSettings;									/ Configuration settings for this wheel
	BodyID					mContactBodyID;								/ ID of body for ground
	SubShapeID				mContactSubShapeID;							/ Sub shape ID for ground
	Body*					mContactBody = nullptr;						/ Body for ground
	float					mSuspensionLength;							/ Current length of the suspension
	Vec3					mContactPosition;							/ Position of the contact point between wheel and ground
	Vec3					mContactPointVelocity;						/ Velocity of the contact point (m / s, not relative to the wheel but in world space)
	Vec3					mContactNormal;								/ Normal of the contact point between wheel and ground
	Vec3					mContactLongitudinal;						/ Vector perpendicular to normal in the forward direction
	Vec3					mContactLateral;							/ Vector perpendicular to normal and longitudinal direction in the right direction
	Real					mAxlePlaneConstant;							/ Constant for the contact plane of the axle, defined as ContactNormal . (WorldSpaceSuspensionPoint + SuspensionLength* WorldSpaceSuspensionDirection)
	float					mAntiRollBarImpulse = 0.0f;					/ Amount of impulse applied to the suspension from the anti-rollbars

	float					mSteerAngle = 0.0f;							/ Rotation around the suspension direction, positive is to the left
	float					mAngularVelocity = 0.0f;					/ Rotation speed of wheel, positive when the wheels cause the vehicle to move forwards (rad/s)
	float					mAngle = 0.0f;								/ Current rotation of the wheel (rad, [0, 2 pi])

	AxisConstraintPart		mSuspensionPart;							/ Controls movement up/down along the contact normal
	AxisConstraintPart		mSuspensionMaxUpPart;						/ Adds a hard limit when reaching the minimal suspension length
	AxisConstraintPart		mLongitudinalPart;							/ Controls movement forward/backward
	AxisConstraintPart		mLateralPart;								/ Controls movement sideways (slip)
};

class WheelWV : public Wheel {
public:
	MOSS_OVERRIDE_NEW_DELETE

	explicit WheelWV(const WheelSettingsWV& inWheel);

	// Override GetSettings and cast to the correct class
	const WheelSettingsWV* GetSettings() const { return StaticCast<WheelSettingsWV>(mSettings); }

	// Apply a torque (N m) to the wheel for a particular delta time
	void ApplyTorque(float inTorque, float inDeltaTime) { mAngularVelocity += inTorque* inDeltaTime / GetSettings()->mInertia; }

	// Update the wheel rotation based on the current angular velocity
	void Update(uint8 inWheelIndex, float inDeltaTime, const VehicleConstraint& inConstraint);

	float						mLongitudinalSlip = 0.0f;					// Velocity difference between ground and wheel relative to ground velocity
	float						mLateralSlip = 0.0f;						// Angular difference (in radians) between ground and wheel relative to ground velocity
	float						mCombinedLongitudinalFriction = 0.0f;		// Combined friction coefficient in longitudinal direction (combines terrain and tires)
	float						mCombinedLateralFriction = 0.0f;			// Combined friction coefficient in lateral direction (combines terrain and tires)
	float						mBrakeImpulse = 0.0f;						// Amount of impulse that the brakes can apply to the floor (excluding friction)
};

class WheelTV : public Wheel {
public:
	MOSS_OVERRIDE_NEW_DELETE

	explicit					WheelTV(const WheelSettingsTV& inWheel);
	const WheelSettingsTV*		GetSettings() const							{ return StaticCast<WheelSettingsTV>(mSettings); }

	// Update the angular velocity of the wheel based on the angular velocity of the track
	void						CalculateAngularVelocity(const VehicleConstraint& inConstraint);

	// Update the wheel rotation based on the current angular velocity
	void						Update(uint8 inWheelIndex, float inDeltaTime, const VehicleConstraint& inConstraint);

	int							mTrackIndex = -1;							// Index in mTracks to which this wheel is attached (calculated on initialization)
	float						mCombinedLongitudinalFriction = 0.0f;		// Combined friction coefficient in longitudinal direction (combines terrain and track)
	float						mCombinedLateralFriction = 0.0f;			// Combined friction coefficient in lateral direction (combines terrain and track)
	float						mBrakeImpulse = 0.0f;						// Amount of impulse that the brakes can apply to the floor (excluding friction), spread out from brake impulse applied on track
};

class VehicleTransmissionSettings {
public:
	// Saves the contents in binary form to inStream.
	void					SaveBinaryState(StreamOut& inStream) const;

	// Restores the contents in binary form to inStream.
	void					RestoreBinaryState(StreamIn& inStream);

	ETransmissionMode		mMode = ETransmissionMode::Auto;			// How to switch gears
	TArray<float>			mGearRatios { 2.66f, 1.78f, 1.3f, 1.0f, 0.74f }; // Ratio in rotation rate between engine and gear box, first element is 1st gear, 2nd element 2nd gear etc.
	TArray<float>			mReverseGearRatios { -2.90f };				// Ratio in rotation rate between engine and gear box when driving in reverse
	float					mSwitchTime = 0.5f;							// How long it takes to switch gears (s), only used in auto mode
	float					mClutchReleaseTime = 0.3f;					// How long it takes to release the clutch (go to full friction), only used in auto mode
	float					mSwitchLatency = 0.5f;						// How long to wait after releasing the clutch before another switch is attempted (s), only used in auto mode
	float					mShiftUpRPM = 4000.0f;						// If RPM of engine is bigger then this we will shift a gear up, only used in auto mode
	float					mShiftDownRPM = 2000.0f;					// If RPM of engine is smaller then this we will shift a gear down, only used in auto mode
	float					mClutchStrength = 10.0f;					// Strength of the clutch when fully engaged. Total torque a clutch applies is Torque = ClutchStrength* (Velocity Engine - Avg Velocity Wheels At Clutch) (units: k m^2 s^-1)
};

class VehicleTransmission : public VehicleTransmissionSettings
{
public:
	// Set input from driver regarding the transmission (only relevant when transmission is set to manual mode)
	// @param inCurrentGear Current gear, -1 = reverse, 0 = neutral, 1 = 1st gear etc.
	// @param inClutchFriction Value between 0 and 1 indicating how much friction the clutch gives (0 = no friction, 1 = full friction)
	void					Set(int inCurrentGear, float inClutchFriction) { mCurrentGear = inCurrentGear; mClutchFriction = inClutchFriction; }

	// Update the current gear and clutch friction if the transmission is in auto mode
	// @param inDeltaTime Time step delta time in s
	// @param inCurrentRPM Current RPM for engine
	// @param inForwardInput Hint if the user wants to drive forward (> 0) or backwards (< 0)
	// @param inCanShiftUp Indicates if we want to allow the transmission to shift up (e.g. pass false if wheels are slipping)
	void					Update(float inDeltaTime, float inCurrentRPM, float inForwardInput, bool inCanShiftUp);

	// Current gear, -1 = reverse, 0 = neutral, 1 = 1st gear etc.
	int						GetCurrentGear() const						{ return mCurrentGear; }

	// Value between 0 and 1 indicating how much friction the clutch gives (0 = no friction, 1 = full friction)
	float					GetClutchFriction() const					{ return mClutchFriction; }

	// If the auto box is currently switching gears
	bool					IsSwitchingGear() const						{ return mGearSwitchTimeLeft > 0.0f; }

	// Return the transmission ratio based on the current gear (ratio between engine and differential)
	float					GetCurrentRatio() const;

	// Only allow sleeping when the transmission is idle
	bool					AllowSleep() const							{ return mGearSwitchTimeLeft <= 0.0f& & mClutchReleaseTimeLeft <= 0.0f& & mGearSwitchLatencyTimeLeft <= 0.0f; }

	// Saving state for replay
	void					SaveState(StateRecorder& inStream) const;
	void					RestoreState(StateRecorder& inStream);

private:
	int						mCurrentGear = 0;							// Current gear, -1 = reverse, 0 = neutral, 1 = 1st gear etc.
	float					mClutchFriction = 1.0f;						// Value between 0 and 1 indicating how much friction the clutch gives (0 = no friction, 1 = full friction)
	float					mGearSwitchTimeLeft = 0.0f;					// When switching gears this will be > 0 and will cause the engine to not provide any torque to the wheels for a short time (used for automatic gear switching only)
	float					mClutchReleaseTimeLeft = 0.0f;				// After switching gears this will be > 0 and will cause the clutch friction to go from 0 to 1 (used for automatic gear switching only)
	float					mGearSwitchLatencyTimeLeft = 0.0f;			// After releasing the clutch this will be > 0 and will prevent another gear switch (used for automatic gear switching only)
};

class VehicleCollisionTester : public RefTarget<VehicleCollisionTester>, public NonCopyable {
public:
	MOSS_OVERRIDE_NEW_DELETE

	// Constructors
									VehicleCollisionTester() = default;
	explicit						VehicleCollisionTester(ObjectLayer inObjectLayer) : mObjectLayer(inObjectLayer) { }

	// Virtual destructor
	virtual							~VehicleCollisionTester() = default;

	// Object layer to use for collision detection, this is used when the filters are not overridden
	ObjectLayer						GetObjectLayer() const												{ return mObjectLayer; }
	void							SetObjectLayer(ObjectLayer inObjectLayer)							{ mObjectLayer = inObjectLayer; }

	// Access to the broad phase layer filter, when set this overrides the object layer supplied in the constructor
	void							SetBroadPhaseLayerFilter(const BroadPhaseLayerFilter*inFilter)		{ mBroadPhaseLayerFilter = inFilter; }
	const BroadPhaseLayerFilter*	GetBroadPhaseLayerFilter() const									{ return mBroadPhaseLayerFilter; }

	// Access to the object layer filter, when set this overrides the object layer supplied in the constructor
	void							SetObjectLayerFilter(const ObjectLayerFilter*inFilter)				{ mObjectLayerFilter = inFilter; }
	const ObjectLayerFilter*		GetObjectLayerFilter() const										{ return mObjectLayerFilter; }

	// Access to the body filter, when set this overrides the default filter that filters out the vehicle body
	void							SetBodyFilter(const BodyFilter*inFilter)							{ mBodyFilter = inFilter; }
	const BodyFilter*				GetBodyFilter() const												{ return mBodyFilter; }

	// Do a collision test with the world
	// @param inPhysicsSystem The physics system that should be tested against
	// @param inVehicleConstraint The vehicle constraint
	// @param inWheelIndex Index of the wheel that we're testing collision for
	// @param inOrigin Origin for the test, corresponds to the world space position for the suspension attachment point
	// @param inDirection Direction for the test (unit vector, world space)
	// @param inVehicleBodyID This body should be filtered out during collision detection to avoid self collisions
	// @param outBody Body that the wheel collided with
	// @param outSubShapeID Sub shape ID that the wheel collided with
	// @param outContactPosition Contact point between wheel and floor, in world space
	// @param outContactNormal Contact normal between wheel and floor, pointing away from the floor
	// @param outSuspensionLength New length of the suspension [0, inSuspensionMaxLength]
	// @return True when collision found, false if not
	virtual bool					Collide(PhysicsSystem& inPhysicsSystem, const VehicleConstraint& inVehicleConstraint, uint8 inWheelIndex, RVec3Arg inOrigin, Vec3Arg inDirection, const BodyID& inVehicleBodyID, Body*&outBody, SubShapeID& outSubShapeID, RVec3& outContactPosition, Vec3& outContactNormal, float& outSuspensionLength) const = 0;

	// Do a cheap contact properties prediction based on the contact properties from the last collision test (provided as input parameters)
	// @param inPhysicsSystem The physics system that should be tested against
	// @param inVehicleConstraint The vehicle constraint
	// @param inWheelIndex Index of the wheel that we're testing collision for
	// @param inOrigin Origin for the test, corresponds to the world space position for the suspension attachment point
	// @param inDirection Direction for the test (unit vector, world space)
	// @param inVehicleBodyID The body ID for the vehicle itself
	// @param ioBody Body that the wheel previously collided with
	// @param ioSubShapeID Sub shape ID that the wheel collided with during the last check
	// @param ioContactPosition Contact point between wheel and floor during the last check, in world space
	// @param ioContactNormal Contact normal between wheel and floor during the last check, pointing away from the floor
	// @param ioSuspensionLength New length of the suspension [0, inSuspensionMaxLength]
	virtual void					PredictContactProperties(PhysicsSystem& inPhysicsSystem, const VehicleConstraint& inVehicleConstraint, uint8 inWheelIndex, RVec3Arg inOrigin, Vec3Arg inDirection, const BodyID& inVehicleBodyID, Body*&ioBody, SubShapeID& ioSubShapeID, RVec3& ioContactPosition, Vec3& ioContactNormal, float& ioSuspensionLength) const = 0;

protected:
	const BroadPhaseLayerFilter	*	mBroadPhaseLayerFilter = nullptr;
	const ObjectLayerFilter*		mObjectLayerFilter = nullptr;
	const BodyFilter*				mBodyFilter = nullptr;
	ObjectLayer						mObjectLayer = cObjectLayerInvalid;
};

// Collision tester that tests collision using a raycast
class VehicleCollisionTesterRay : public VehicleCollisionTester {
public:
	MOSS_OVERRIDE_NEW_DELETE

	// Constructor
	// @param inObjectLayer Object layer to test collision with
	// @param inUp World space up vector, used to avoid colliding with vertical walls.
	// @param inMaxSlopeAngle Max angle (rad) that is considered for colliding wheels. This is to avoid colliding with vertical walls.
	VehicleCollisionTesterRay(ObjectLayer inObjectLayer, Vec3Arg inUp = Vec3::sAxisY(), float inMaxSlopeAngle = DegreesToRadians(80.0f)) : VehicleCollisionTester(inObjectLayer), mUp(inUp), mCosMaxSlopeAngle(Cos(inMaxSlopeAngle)) { }

	// See: VehicleCollisionTester
	virtual bool					Collide(PhysicsSystem& inPhysicsSystem, const VehicleConstraint& inVehicleConstraint, uint8 inWheelIndex, RVec3Arg inOrigin, Vec3Arg inDirection, const BodyID& inVehicleBodyID, Body*&outBody, SubShapeID& outSubShapeID, RVec3& outContactPosition, Vec3& outContactNormal, float& outSuspensionLength) const override;
	virtual void					PredictContactProperties(PhysicsSystem& inPhysicsSystem, const VehicleConstraint& inVehicleConstraint, uint8 inWheelIndex, RVec3Arg inOrigin, Vec3Arg inDirection, const BodyID& inVehicleBodyID, Body*&ioBody, SubShapeID& ioSubShapeID, RVec3& ioContactPosition, Vec3& ioContactNormal, float& ioSuspensionLength) const override;

private:
	Vec3							mUp;
	float							mCosMaxSlopeAngle;
};

// Collision tester that tests collision using a sphere cast
class VehicleCollisionTesterCastSphere : public VehicleCollisionTester {
public:
	MOSS_OVERRIDE_NEW_DELETE

	// Constructor
	// @param inObjectLayer Object layer to test collision with
	// @param inUp World space up vector, used to avoid colliding with vertical walls.
	// @param inRadius Radius of sphere
	// @param inMaxSlopeAngle Max angle (rad) that is considered for colliding wheels. This is to avoid colliding with vertical walls.
									VehicleCollisionTesterCastSphere(ObjectLayer inObjectLayer, float inRadius, Vec3Arg inUp = Vec3::sAxisY(), float inMaxSlopeAngle = DegreesToRadians(80.0f)) : VehicleCollisionTester(inObjectLayer), mRadius(inRadius), mUp(inUp), mCosMaxSlopeAngle(Cos(inMaxSlopeAngle)) { }

	// See: VehicleCollisionTester
	virtual bool					Collide(PhysicsSystem& inPhysicsSystem, const VehicleConstraint& inVehicleConstraint, uint8 inWheelIndex, RVec3Arg inOrigin, Vec3Arg inDirection, const BodyID& inVehicleBodyID, Body*&outBody, SubShapeID& outSubShapeID, RVec3& outContactPosition, Vec3& outContactNormal, float& outSuspensionLength) const override;
	virtual void					PredictContactProperties(PhysicsSystem& inPhysicsSystem, const VehicleConstraint& inVehicleConstraint, uint8 inWheelIndex, RVec3Arg inOrigin, Vec3Arg inDirection, const BodyID& inVehicleBodyID, Body*&ioBody, SubShapeID& ioSubShapeID, RVec3& ioContactPosition, Vec3& ioContactNormal, float& ioSuspensionLength) const override;

private:
	float							mRadius;
	Vec3							mUp;
	float							mCosMaxSlopeAngle;
};

// Collision tester that tests collision using a cylinder shape
class VehicleCollisionTesterCastCylinder : public VehicleCollisionTester {
public:
	MOSS_OVERRIDE_NEW_DELETE

	// Constructor
	// @param inObjectLayer Object layer to test collision with
	// @param inConvexRadiusFraction Fraction of half the wheel width (or wheel radius if it is smaller) that is used as the convex radius
									VehicleCollisionTesterCastCylinder(ObjectLayer inObjectLayer, float inConvexRadiusFraction = 0.1f) : VehicleCollisionTester(inObjectLayer), mConvexRadiusFraction(inConvexRadiusFraction) { MOSS_ASSERT(mConvexRadiusFraction >= 0.0f& & mConvexRadiusFraction <= 1.0f); }

	// See: VehicleCollisionTester
	virtual bool					Collide(PhysicsSystem& inPhysicsSystem, const VehicleConstraint& inVehicleConstraint, uint8 inWheelIndex, RVec3Arg inOrigin, Vec3Arg inDirection, const BodyID& inVehicleBodyID, Body*&outBody, SubShapeID& outSubShapeID, RVec3& outContactPosition, Vec3& outContactNormal, float& outSuspensionLength) const override;
	virtual void					PredictContactProperties(PhysicsSystem& inPhysicsSystem, const VehicleConstraint& inVehicleConstraint, uint8 inWheelIndex, RVec3Arg inOrigin, Vec3Arg inDirection, const BodyID& inVehicleBodyID, Body*&ioBody, SubShapeID& ioSubShapeID, RVec3& ioContactPosition, Vec3& ioContactNormal, float& ioSuspensionLength) const override;

private:
	float							mConvexRadiusFraction;
};

struct WheeledVehicleControllerSettings : public VehicleControllerSettings {
	VehicleEngineSettings		mEngine;									// The properties of the engine
	VehicleTransmissionSettings	mTransmission;								// The properties of the transmission (aka gear box)
	TArray<VehicleDifferentialSettings> mDifferentials;						// List of differentials and their properties
	float						mDifferentialLimitedSlipRatio = 1.4f;		// Ratio max / min average wheel speed of each differential (measured at the clutch). When the ratio is exceeded all torque gets distributed to the differential with the minimal average velocity. This allows implementing a limited slip differential between differentials. Set to FLT_MAX for an open differential. Value should be > 1.
};

struct MotorcycleControllerSettings : public WheeledVehicleControllerSettings {
	float						mMaxLeanAngle = DegreesToRadians(45.0f);	// How far we're willing to make the bike lean over in turns (in radians)
	float						mLeanSpringConstant = 5000.0f;				// Spring constant for the lean spring
	float						mLeanSpringDamping = 1000.0f;				// Spring damping constant for the lean spring
	float						mLeanSpringIntegrationCoefficient = 0.0f;	// The lean spring applies an additional force equal to this coefficient* Integral(delta angle, 0, t), this effectively makes the lean spring a PID controller
	float						mLeanSpringIntegrationCoefficientDecay = 4.0f;// How much to decay the angle integral when the wheels are not touching the floor: new_value = e^(-decay* t)* initial_value
	// How much to smooth the lean angle (0 = no smoothing, 1 = lean angle never changes)
	// Note that this is frame rate dependent because the formula is: smoothing_factor* previous + (1 - smoothing_factor)* current
	float						mLeanSmoothingFactor = 0.8f;
};

struct VehicleTrackSettings {
	uint8					mDrivenWheel;								// Which wheel on the track is connected to the engine
	TArray<uint8>			mWheels;									// Indices of wheels that are inside this track, should include the driven wheel too
	float					mInertia = 10.0f;							// Moment of inertia (kg m^2) of the track and its wheels as seen on the driven wheel
	float					mAngularDamping = 0.5f;						// Damping factor of track and its wheels: dw/dt = -c* w as seen on the driven wheel
	float					mMaxBrakeTorque = 15000.0f;					// How much torque (Nm) the brakes can apply on the driven wheel
	float					mDifferentialRatio = 6.0f;					// Ratio between rotation speed of gear box and driven wheel of track
};

struct VehicleEngineSettings {
	float					MaxTorque = 500.0f;						// Max amount of torque (Nm) that the engine can deliver
	float					MinRPM = 1000.0f;							// Min amount of revolutions per minute (rpm) the engine can produce without stalling
	float					MaxRPM = 6000.0f;							// Max amount of revolutions per minute (rpm) the engine can generate
	LinearCurve				NormalizedTorque;							// Y-axis: Curve that describes a ratio of the max torque the engine can produce (0 = 0, 1 = mMaxTorque). X-axis: the fraction of the RPM of the engine (0 = mMinRPM, 1 = mMaxRPM)
	float					Inertia = 0.5f;							// Moment of inertia (kg m^2) of the engine
	float					AngularDamping = 0.2f;						// Angular damping factor of the wheel: dw/dt = -c* w
};

struct TrackedVehicleControllerSettings : public VehicleControllerSettings {
	VehicleEngineSettings		mEngine;									// The properties of the engine
	VehicleTransmissionSettings	mTransmission;								// The properties of the transmission (aka gear box)
	VehicleTrackSettings		mTracks[(int)ETrackSide::Num];				// List of tracks and their properties
};

struct VehicleDifferentialSettings {
public:
	// Calculate the torque ratio between left and right wheel
	// @param inLeftAngularVelocity Angular velocity of left wheel (rad / s)
	// @param inRightAngularVelocity Angular velocity of right wheel (rad / s)
	// @param outLeftTorqueFraction Fraction of torque that should go to the left wheel
	// @param outRightTorqueFraction Fraction of torque that should go to the right wheel
	void					CalculateTorqueRatio(float inLeftAngularVelocity, float inRightAngularVelocity, float& outLeftTorqueFraction, float& outRightTorqueFraction) const;

	int						mLeftWheel = -1;							// Index (in mWheels) that represents the left wheel of this differential (can be -1 to indicate no wheel)
	int						mRightWheel = -1;							// Index (in mWheels) that represents the right wheel of this differential (can be -1 to indicate no wheel)
	float					mDifferentialRatio = 3.42f;					// Ratio between rotation speed of gear box and wheels
	float					mLeftRightSplit = 0.5f;						// Defines how the engine torque is split across the left and right wheel (0 = left, 0.5 = center, 1 = right)
	float					mLimitedSlipRatio = 1.4f;					// Ratio max / min wheel speed. When this ratio is exceeded, all torque gets distributed to the slowest moving wheel. This allows implementing a limited slip differential. Set to FLT_MAX for an open differential. Value should be > 1.
	float					mEngineTorqueRatio = 1.0f;					// How much of the engines torque is applied to this differential (0 = none, 1 = full), make sure the sum of all differentials is 1.
};


struct VehicleTrack : public VehicleTrackSettings {
	float					mAngularVelocity = 0.0f;					// Angular velocity of the driven wheel, will determine the speed of the entire track
};

class VehicleController : public NonCopyable {
public:
	MOSS_OVERRIDE_NEW_DELETE

	// Constructor / destructor
	explicit VehicleController(VehicleConstraint& inConstraint) : mConstraint(inConstraint) { }
	virtual ~VehicleController() = default;

	// Access the vehicle constraint that this controller is part of
	VehicleConstraint& GetConstraint() { return mConstraint; }
	const VehicleConstraint& GetConstraint() const { return mConstraint; }

protected:
	// The functions below are only for the VehicleConstraint
	friend class VehicleConstraint;

	// Create a new instance of wheel
	virtual Wheel* ConstructWheel(const WheelSettings& inWheel) const = 0;

	// If the vehicle is allowed to go to sleep
	virtual bool AllowSleep() const = 0;

	// Called before the wheel probes have been done
	virtual void PreCollide(float inDeltaTime, PhysicsSystem& inPhysicsSystem) = 0;

	// Called after the wheel probes have been done
	virtual void PostCollide(float inDeltaTime, PhysicsSystem& inPhysicsSystem) = 0;

	// Solve longitudinal and lateral constraint parts for all of the wheels
	virtual bool SolveLongitudinalAndLateralConstraints(float inDeltaTime) = 0;

	// Saving state for replay
	virtual void SaveState(StateRecorder& inStream) const = 0;
	virtual void RestoreState(StateRecorder& inStream) = 0;

#ifndef MOSS_DEBUG_RENDERER
	// Drawing interface
	virtual void Draw(DebugRenderer*inRenderer) const = 0;
#endif // MOSS_DEBUG_RENDERER

	VehicleConstraint&			mConstraint;								// The vehicle constraint we belong to
};


class VehicleEngine : public VehicleEngineSettings
{
public:
	// Multiply an angular velocity (rad/s) with this value to get rounds per minute (RPM)
	static constexpr float	cAngularVelocityToRPM = 60.0f / (2.0f* MOSS_PI);

	// Clamp the RPM between min and max RPM
	inline void				ClampRPM()									{ mCurrentRPM = Clamp(mCurrentRPM, mMinRPM, mMaxRPM); }

	// Current rotation speed of engine in rounds per minute
	float					GetCurrentRPM() const						{ return mCurrentRPM; }

	// Update rotation speed of engine in rounds per minute
	void					SetCurrentRPM(float inRPM)					{ mCurrentRPM = inRPM; ClampRPM(); }

	// Get current angular velocity of the engine in radians / second
	inline float			GetAngularVelocity() const					{ return mCurrentRPM / cAngularVelocityToRPM; }

	// Get the amount of torque (N m) that the engine can supply
	// @param inAcceleration How much the gas pedal is pressed [0, 1]
	float					GetTorque(float inAcceleration) const		{ return inAcceleration* mMaxTorque* mNormalizedTorque.GetValue(mCurrentRPM / mMaxRPM); }

	// Apply a torque to the engine rotation speed
	// @param inTorque Torque in N m
	// @param inDeltaTime Delta time in seconds
	void					ApplyTorque(float inTorque, float inDeltaTime);

	// Update the engine RPM for damping
	// @param inDeltaTime Delta time in seconds
	void					ApplyDamping(float inDeltaTime);

#ifndef MOSS_DEBUG_RENDERER
	// Function that converts RPM to an angle in radians for debugging purposes
	float					ConvertRPMToAngle(float inRPM) const		{ return (-0.75f + 1.5f* inRPM / mMaxRPM)* MOSS_PI; }

	// Debug draw a RPM meter
	void					DrawRPM(DebugRenderer*inRenderer, RVec3Arg inPosition, Vec3Arg inForward, Vec3Arg inUp, float inSize, float inShiftDownRPM, float inShiftUpRPM) const;
#endif // MOSS_DEBUG_RENDERER

	// If the engine is idle we allow the vehicle to sleep
	bool					AllowSleep() const							{ return mCurrentRPM <= 1.01f* mMinRPM; }

	// Saving state for replay
	void					SaveState(StateRecorder& inStream) const;
	void					RestoreState(StateRecorder& inStream);

private:
	float					mCurrentRPM = mMinRPM;						// Current rotation speed of engine in rounds per minute
};

class TrackedVehicleController : public VehicleController {
public:
	MOSS_OVERRIDE_NEW_DELETE

	// Constructor
	TrackedVehicleController(const TrackedVehicleControllerSettings& inSettings, VehicleConstraint& inConstraint);

	// Set input from driver
	// @param inForward Value between -1 and 1 for auto transmission and value between 0 and 1 indicating desired driving direction and amount the gas pedal is pressed
	// @param inLeftRatio Value between -1 and 1 indicating an extra multiplier to the rotation rate of the left track (used for steering)
	// @param inRightRatio Value between -1 and 1 indicating an extra multiplier to the rotation rate of the right track (used for steering)
	// @param inBrake Value between 0 and 1 indicating how strong the brake pedal is pressed
	void SetDriverInput(float inForward, float inLeftRatio, float inRightRatio, float inBrake) { MOSS_ASSERT(inLeftRatio != 0.0f& & inRightRatio != 0.0f); mForwardInput = inForward; mLeftRatio = inLeftRatio; mRightRatio = inRightRatio; mBrakeInput = inBrake; }

	// Value between -1 and 1 for auto transmission and value between 0 and 1 indicating desired driving direction and amount the gas pedal is pressed
	void SetForwardInput(float inForward) { mForwardInput = inForward; }
	float GetForwardInput() const { return mForwardInput; }

	// Value between -1 and 1 indicating an extra multiplier to the rotation rate of the left track (used for steering)
	void SetLeftRatio(float inLeftRatio) { MOSS_ASSERT(inLeftRatio != 0.0f); mLeftRatio = inLeftRatio; }
	float GetLeftRatio() const { return mLeftRatio; }

	// Value between -1 and 1 indicating an extra multiplier to the rotation rate of the right track (used for steering)
	void SetRightRatio(float inRightRatio) { MOSS_ASSERT(inRightRatio != 0.0f); mRightRatio = inRightRatio; }
	float GetRightRatio() const	{ return mRightRatio; }

	// Value between 0 and 1 indicating how strong the brake pedal is pressed
	void SetBrakeInput(float inBrake) { mBrakeInput = inBrake; }
	float GetBrakeInput() const	{ return mBrakeInput; }

	// Get current engine state
	const VehicleEngine&		GetEngine() const { return mEngine; }

	// Get current engine state (writable interface, allows you to make changes to the configuration which will take effect the next time step)
	VehicleEngine&				GetEngine() { return mEngine; }

	// Get current transmission state
	const VehicleTransmission&	GetTransmission() const { return mTransmission; }

	// Get current transmission state (writable interface, allows you to make changes to the configuration which will take effect the next time step)
	VehicleTransmission& GetTransmission() { return mTransmission; }

	// Get the tracks this vehicle has
	const VehicleTracks& GetTracks() const { return mTracks; }

	// Get the tracks this vehicle has (writable interface, allows you to make changes to the configuration which will take effect the next time step)
	VehicleTracks& GetTracks() { return mTracks; }

#ifndef MOSS_DEBUG_RENDERER
	// Debug drawing of RPM meter
	void SetRPMMeter(Vec3Arg inPosition, float inSize) { mRPMMeterPosition = inPosition; mRPMMeterSize = inSize; }
#endif // MOSS_DEBUG_RENDERER

protected:
	// Synchronize angular velocities of left and right tracks according to their ratios
	void SyncLeftRightTracks();

	// See: VehicleController
	virtual Wheel* ConstructWheel(const WheelSettings& inWheel) const override { MOSS_ASSERT(IsKindOf(&inWheel, MOSS_RTTI(WheelSettingsTV))); return new WheelTV(static_cast<const WheelSettingsTV& >(inWheel)); }
	virtual bool AllowSleep() const override;
	virtual void PreCollide(float inDeltaTime, PhysicsSystem& inPhysicsSystem) override;
	virtual void PostCollide(float inDeltaTime, PhysicsSystem& inPhysicsSystem) override;
	virtual bool SolveLongitudinalAndLateralConstraints(float inDeltaTime) override;
	virtual void SaveState(StateRecorder& inStream) const override;
	virtual void RestoreState(StateRecorder& inStream) override;
#ifndef MOSS_DEBUG_RENDERER
	virtual void Draw(DebugRenderer*inRenderer) const override;
#endif // MOSS_DEBUG_RENDERER

	// Control information
	float			mForwardInput = 0.0f;	// Value between -1 and 1 for auto transmission and value between 0 and 1 indicating desired driving direction and amount the gas pedal is pressed
	float			mLeftRatio = 1.0f;		// Value between -1 and 1 indicating an extra multiplier to the rotation rate of the left track (used for steering)
	float			mRightRatio = 1.0f;		// Value between -1 and 1 indicating an extra multiplier to the rotation rate of the right track (used for steering)
	float			mBrakeInput = 0.0f;		// Value between 0 and 1 indicating how strong the brake pedal is pressed

	// Simulation information
	VehicleEngine				mEngine;									// Engine state of the vehicle
	VehicleTransmission			mTransmission;								// Transmission state of the vehicle
	VehicleTracks				mTracks;									// Tracks of the vehicle

#ifndef MOSS_DEBUG_RENDERER
	// Debug settings
	Vec3						mRPMMeterPosition { 0, 1, 0 };				// Position (in local space of the body) of the RPM meter when drawing the constraint
	float						mRPMMeterSize = 0.5f;						// Size of the RPM meter when drawing the constraint
#endif // MOSS_DEBUG_RENDERER
};

struct VehicleAntiRollBar {
	int						leftWheel;
	int						rightWheel = 1;
	float					stiffness = 1000.0f;
};

class WheeledVehicleController : public VehicleController {
public:
	MOSS_OVERRIDE_NEW_DELETE

	// Constructor
	WheeledVehicleController(const WheeledVehicleControllerSettings& inSettings, VehicleConstraint& inConstraint);

	// Typedefs
	using Differentials = TArray<VehicleDifferentialSettings>;

	// Set input from driver
	// @param inForward Value between -1 and 1 for auto transmission and value between 0 and 1 indicating desired driving direction and amount the gas pedal is pressed
	// @param inRight Value between -1 and 1 indicating desired steering angle (1 = right)
	// @param inBrake Value between 0 and 1 indicating how strong the brake pedal is pressed
	// @param inHandBrake Value between 0 and 1 indicating how strong the hand brake is pulled
	void SetDriverInput(float inForward, float inRight, float inBrake, float inHandBrake) { mForwardInput = inForward; mRightInput = inRight; mBrakeInput = inBrake; mHandBrakeInput = inHandBrake; }

	// Value between -1 and 1 for auto transmission and value between 0 and 1 indicating desired driving direction and amount the gas pedal is pressed
	void SetForwardInput(float inForward) { mForwardInput = inForward; }
	float GetForwardInput() const { return mForwardInput; }

	// Value between -1 and 1 indicating desired steering angle (1 = right)
	void SetRightInput(float inRight) { mRightInput = inRight; }
	float GetRightInput() const { return mRightInput; }

	// Value between 0 and 1 indicating how strong the brake pedal is pressed
	void SetBrakeInput(float inBrake) { mBrakeInput = inBrake; }
	float GetBrakeInput() const { return mBrakeInput; }

	// Value between 0 and 1 indicating how strong the hand brake is pulled
	void SetHandBrakeInput(float inHandBrake) { mHandBrakeInput = inHandBrake; }
	float GetHandBrakeInput() const	{ return mHandBrakeInput; }

	// Get current engine state
	const VehicleEngine& GetEngine() const { return mEngine; }

	// Get current engine state (writable interface, allows you to make changes to the configuration which will take effect the next time step)
	VehicleEngine& GetEngine() { return mEngine; }

	// Get current transmission state
	const VehicleTransmission& GetTransmission() const { return mTransmission; }

	// Get current transmission state (writable interface, allows you to make changes to the configuration which will take effect the next time step)
	VehicleTransmission& GetTransmission() { return mTransmission; }

	// Get the differentials this vehicle has
	const Differentials& GetDifferentials() const { return mDifferentials; }

	// Get the differentials this vehicle has (writable interface, allows you to make changes to the configuration which will take effect the next time step)
	Differentials& GetDifferentials() { return mDifferentials; }

	// Ratio max / min average wheel speed of each differential (measured at the clutch).
	float GetDifferentialLimitedSlipRatio() const { return mDifferentialLimitedSlipRatio; }
	void SetDifferentialLimitedSlipRatio(float inV)	{ mDifferentialLimitedSlipRatio = inV; }

	// Get the average wheel speed of all driven wheels (measured at the clutch)
	float GetWheelSpeedAtClutch() const;

	// Calculate max tire impulses by combining friction, slip, and suspension impulse. Note that the actual applied impulse may be lower (e.g. when the vehicle is stationary on a horizontal surface the actual impulse applied will be 0).
	using TireMaxImpulseCallback = function<void(uint8 inWheelIndex, float& outLongitudinalImpulse, float& outLateralImpulse, float inSuspensionImpulse, float inLongitudinalFriction, float inLateralFriction, float inLongitudinalSlip, float inLateralSlip, float inDeltaTime)>;
	const TireMaxImpulseCallback& GetTireMaxImpulseCallback() const { return mTireMaxImpulseCallback; }
	void SetTireMaxImpulseCallback(const TireMaxImpulseCallback& inTireMaxImpulseCallback)	{ mTireMaxImpulseCallback = inTireMaxImpulseCallback; }

#ifndef MOSS_DEBUG_RENDERER
	// Debug drawing of RPM meter
	void SetRPMMeter(Vec3Arg inPosition, float inSize) { mRPMMeterPosition = inPosition; mRPMMeterSize = inSize; }
#endif // MOSS_DEBUG_RENDERER

protected:
	// See: VehicleController
	virtual Wheel* ConstructWheel(const WheelSettings& inWheel) const override { MOSS_ASSERT(IsKindOf(&inWheel, MOSS_RTTI(WheelSettingsWV))); return new WheelWV(static_cast<const WheelSettingsWV& >(inWheel)); }
	virtual bool AllowSleep() const override;
	virtual void PreCollide(float inDeltaTime, PhysicsSystem& inPhysicsSystem) override;
	virtual void PostCollide(float inDeltaTime, PhysicsSystem& inPhysicsSystem) override;
	virtual bool SolveLongitudinalAndLateralConstraints(float inDeltaTime) override;
	virtual void SaveState(StateRecorder& inStream) const override;
	virtual void RestoreState(StateRecorder& inStream) override;
#ifndef MOSS_DEBUG_RENDERER
	virtual void Draw(DebugRenderer*inRenderer) const override;
#endif // MOSS_DEBUG_RENDERER

	// Control information
	float						mForwardInput = 0.0f;						// Value between -1 and 1 for auto transmission and value between 0 and 1 indicating desired driving direction and amount the gas pedal is pressed
	float						mRightInput = 0.0f;							// Value between -1 and 1 indicating desired steering angle
	float						mBrakeInput = 0.0f;							// Value between 0 and 1 indicating how strong the brake pedal is pressed
	float						mHandBrakeInput = 0.0f;						// Value between 0 and 1 indicating how strong the hand brake is pulled

	// Simulation information
	VehicleEngine				mEngine;									// Engine state of the vehicle
	VehicleTransmission			mTransmission;								// Transmission state of the vehicle
	Differentials				mDifferentials;								// Differential states of the vehicle
	float						mDifferentialLimitedSlipRatio;				// Ratio max / min average wheel speed of each differential (measured at the clutch).
	float						mPreviousDeltaTime = 0.0f;					// Delta time of the last step

	// Callback that calculates the max impulse that the tire can apply to the ground
	TireMaxImpulseCallback		mTireMaxImpulseCallback = 
	[](uint8, float& outLongitudinalImpulse, float& outLateralImpulse, float inSuspensionImpulse, float inLongitudinalFriction, float inLateralFriction, float, float, float)
		{
			outLongitudinalImpulse = inLongitudinalFriction* inSuspensionImpulse;
			outLateralImpulse = inLateralFriction* inSuspensionImpulse;
		};

#ifndef MOSS_DEBUG_RENDERER
	// Debug settings
	Vec3						mRPMMeterPosition { 0, 1, 0 };				// Position (in local space of the body) of the RPM meter when drawing the constraint
	float						mRPMMeterSize = 0.5f;						// Size of the RPM meter when drawing the constraint
#endif // MOSS_DEBUG_RENDERER
};

class MotorcycleController : public WheeledVehicleController {
public:
	MOSS_OVERRIDE_NEW_DELETE

	MotorcycleController(const MotorcycleControllerSettings& inSettings, VehicleConstraint& inConstraint);

	// Get the distance between the front and back wheels
	float GetWheelBase() const;
	// Enable or disable the lean spring. This allows you to temporarily disable the lean spring to allow the motorcycle to fall over.
	void EnableLeanController(bool inEnable) { mEnableLeanController = inEnable; }
	// Check if the lean spring is enabled.
	bool IsLeanControllerEnabled() const { return mEnableLeanController; }
	// Enable or disable the lean steering limit. When enabled (default) the steering angle is limited based on the vehicle speed to prevent steering that would cause an inertial force that causes the motorcycle to topple over.
	void EnableLeanSteeringLimit(bool inEnable) { mEnableLeanSteeringLimit = inEnable; }
	bool IsLeanSteeringLimitEnabled() const { return mEnableLeanSteeringLimit; }
	// Spring constant for the lean spring
	void SetLeanSpringConstant(float inConstant) { mLeanSpringConstant = inConstant; }
	float GetLeanSpringConstant() const { return mLeanSpringConstant; }
	// Spring damping constant for the lean spring
	void SetLeanSpringDamping(float inDamping) { mLeanSpringDamping = inDamping; }
	float GetLeanSpringDamping() const { return mLeanSpringDamping; }
	// The lean spring applies an additional force equal to this coefficient* Integral(delta angle, 0, t), this effectively makes the lean spring a PID controller
	void SetLeanSpringIntegrationCoefficient(float inCoefficient) { mLeanSpringIntegrationCoefficient = inCoefficient; }
	float GetLeanSpringIntegrationCoefficient() const { return mLeanSpringIntegrationCoefficient; }
	// How much to decay the angle integral when the wheels are not touching the floor: new_value = e^(-decay* t)* initial_value
	void SetLeanSpringIntegrationCoefficientDecay(float inDecay) { mLeanSpringIntegrationCoefficientDecay = inDecay; }
	float GetLeanSpringIntegrationCoefficientDecay() const	{ return mLeanSpringIntegrationCoefficientDecay; }
	// How much to smooth the lean angle (0 = no smoothing, 1 = lean angle never changes)
	// Note that this is frame rate dependent because the formula is: smoothing_factor* previous + (1 - smoothing_factor)* current
	void SetLeanSmoothingFactor(float inFactor) { mLeanSmoothingFactor = inFactor; }
	float GetLeanSmoothingFactor() const { return mLeanSmoothingFactor; }

protected:
	// See: VehicleController
	virtual void				PreCollide(float inDeltaTime, PhysicsSystem& inPhysicsSystem) override;
	virtual bool				SolveLongitudinalAndLateralConstraints(float inDeltaTime) override;
	virtual void				SaveState(StateRecorder& inStream) const override;
	virtual void				RestoreState(StateRecorder& inStream) override;
#ifndef MOSS_DEBUG_RENDERER
	virtual void				Draw(DebugRenderer*inRenderer) const override;
#endif // MOSS_DEBUG_RENDERER
	// Configuration properties
	bool mEnableLeanController = true;
	bool mEnableLeanSteeringLimit = true;
	float mMaxLeanAngle;
	float mLeanSpringConstant;
	float mLeanSpringDamping;
	float mLeanSpringIntegrationCoefficient;
	float mLeanSpringIntegrationCoefficientDecay;
	float mLeanSmoothingFactor;

	Vec3 mTargetLean = Vec3::sZero();			// Run-time calculated target lean vector
	float mLeanSpringIntegratedDeltaAngle = 0.0f;	// Integrated error for the lean spring
	float mAppliedImpulse = 0.0f;					// Run-time total angular impulse applied to turn the cycle towards the target lean angle
};

static_assert(MOSS_CPU_ADDRESS_BITS != 64 || sizeof(Body) == MOSS_IF_SINGLE_PRECISION_ELSE(128, 160), "Body size is incorrect");
static_assert(alignof(Body) == MOSS_RVECTOR_ALIGNMENT, "Body should properly align");


using Wheels = TArray<Wheel*>;
using VehicleTracks = VehicleTrack[(int)ETrackSide::Num];
using VehicleAntiRollBars = TArray<VehicleAntiRollBar>;


MOSS_SUPRESS_WARNINGS_END

#endif
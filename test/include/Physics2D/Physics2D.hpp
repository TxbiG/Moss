#pragma once
#include "BroadPhase2D.hpp"
#include "NarrowPhase2D.hpp"
#include "Shapes2D.hpp"
#include "Skeleton2D.hpp"
#include "Joints2D.hpp"
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <optional>
#include <limits>
#include <utility>

enum class MOSS_API BodyType : uint8_t { Static, Kinematic, Dynamic };

struct MOSS_API CollisionFilter {
    uint16_t categoryBits = 0x0001;
    uint16_t maskBits = 0xffff;
    int16_t groupIndex = 0;
};
struct MOSS_API Material2D {
    float friction = 0.6f;
    float restitution = 0.0f;
    float density = 1.0f;
};
struct MOSS_API BodyDef {
    BodyType type = BodyType::Static;
    Shape2D shape = Shape2D::Box({0.5f,0.5f});
    Vec2 position{};
    float angle = 0.0f;
    Vec2 linearVelocity{};
    float angularVelocity = 0.0f;
    float gravityScale = 1.0f;
    float linearDamping = -1.0f; // negative uses world default
    float angularDamping = -1.0f; // negative uses world default
    bool fixedRotation = false;
    bool bullet = false;
    bool isSensor = false;
    bool allowSleep = true;
    bool initiallyAwake = true;
    bool enabled = true;
    bool oneWayPlatform = false;
    Vec2 oneWayNormal{0,1};
    CollisionFilter filter{};
    Material2D material{};
    void* userData = nullptr;
    std::string name;
};

struct MOSS_API BodyState {
    BodyID id = InvalidBodyID;
    BodyType type = BodyType::Static;
    Transform2D transform{};
    Vec2 linearVelocity{};
    float angularVelocity=0;
    Vec2 force{};
    float torque=0;
    float mass=0, inverseMass=0, inertia=0, inverseInertia=0;
    Vec2 localCenter{}, worldCenter{};
    bool awake=true, enabled=true, isSensor=false, bullet=false;
    void* userData=nullptr;
    std::string name;
};

struct MOSS_API ContactPoint2D { Vec2 point{}; float separation=0; float normalImpulse=0; float tangentImpulse=0; float velocityBias=0; uint32_t featureId=0; };
struct MOSS_API SubShapePath2D {
    static constexpr uint8_t Capacity=8;
    uint8_t depth=0;
    uint32_t ids[Capacity]{};
    bool Push(uint32_t id) { if(depth>=Capacity)return false;ids[depth++]=id;return true; }
    uint32_t Hash() const { uint32_t h=2166136261u;for(uint8_t i=0;i<depth;++i){h=(h^ids[i])*16777619u;}return h; }
};
struct MOSS_API Manifold2D {
    Vec2 normal{0,1}; // from A to B
    float penetration=0;
    uint16_t childIndexA=0xffffu, childIndexB=0xffffu;
    uint32_t childIdA=0xffffffffu, childIdB=0xffffffffu;
    void* childUserDataA=nullptr;
    void* childUserDataB=nullptr;
    SubShapePath2D childPathA{}, childPathB{};
    bool isSensor=false;
    ContactPoint2D points[2]{};
    uint8_t pointCount=0;
};
struct MOSS_API ContactEvent {
    BodyID bodyA=InvalidBodyID, bodyB=InvalidBodyID;
    Manifold2D manifold{}; // first manifold retained for compatibility
    bool sensor=false;
    std::vector<Manifold2D> manifolds; // all child-pair manifolds for compound contacts
};
struct MOSS_API RayCastHit {
    BodyID body=InvalidBodyID;
    Vec2 point{}, normal{};
    float fraction=1.0f;
    void* userData=nullptr;
};
struct MOSS_API ShapeCastHit2D { BodyID body=InvalidBodyID; Vec2 point{},normal{}; float fraction=1.0f; uint32_t iterations=0; void* userData=nullptr; };
struct MOSS_API TimeOfImpact2D { bool hit=false; float fraction=1.0f; Vec2 point{},normal{}; uint32_t iterations=0; };

struct MOSS_API WorldSettings {
    Vec2 gravity = Vec2(0, -9.81f);
    int velocityIterations=8;
    int positionIterations=3;
    float fixedTimeStep=1.0f/60.0f;
    int maxSubSteps=4;
    float linearDamping=0.01f;
    float angularDamping=0.01f;
    float contactSlop=0.005f;
    float baumgarte=0.2f;
    float sleepLinearThreshold=0.03f;
    float sleepAngularThreshold=0.03f;
    float timeToSleep=0.5f;
    float broadPhaseCellSize=1.0f;
    float maxFrameTime=0.25f;
    int maxCCDSubSteps=64;
    int maxCCDImpactsPerStep=32;
    float ccdMotionThreshold=0.5f;
    bool deterministicMode=false;
    bool enableMultithreading=true;
    uint32_t workerCount=0; // zero selects hardware concurrency
};

struct MOSS_API PhysicsStats2D {
    uint64_t stepCount=0;
    uint32_t bodyCount=0, enabledBodyCount=0, contactCount=0;
    uint32_t broadPhaseCandidateCount=0, ccdSubStepCount=0, ccdImpactCount=0, ccdEventOverflowCount=0;
    BroadPhaseTreeStats2D broadPhaseTree{};
    uint32_t islandCount=0;
};
struct MOSS_API PhysicsProfile2D {
    double broadPhaseMilliseconds=0.0;
    double broadPhasePairGenerationMilliseconds=0.0;
    double narrowPhaseMilliseconds=0.0;
    double velocitySolveMilliseconds=0.0;
    double positionSolveMilliseconds=0.0;
    double ccdMilliseconds=0.0;
    double totalStepMilliseconds=0.0;
    uint64_t broadPhaseQueries=0,gjkIterations=0,shapeCastCount=0,ccdImpactCount=0,simdBatchCount=0;
};
struct MOSS_API BodySnapshot2D {
    BodyID id=InvalidBodyID;
    Transform2D transform{};
    Vec2 linearVelocity{};
    float angularVelocity=0.0f;
    Vec2 force{};
    float torque=0.0f;
    bool awake=true, enabled=true;
};
struct MOSS_API WorldSnapshot2D {
    uint64_t stepCount=0;
    std::vector<BodySnapshot2D> bodies;
};

// Stable, pointer-free world archive records. Body IDs are remapped when an
// archive is restored into a new world; use sourceId -> returned ID mapping.
struct MOSS_API ArchivedBody2D {
    BodyID sourceId=InvalidBodyID;
    BodyDef definition{};
    BodySnapshot2D state{};
};
struct MOSS_API ArchivedJoint2D {
    JointID sourceId=InvalidJointID;
    JointType type=JointType::Distance;
    BodyID bodyA=InvalidBodyID,bodyB=InvalidBodyID;
    Vec2 anchorA{},anchorB{},axisA{1,0},target{};
    Vec2 groundAnchorA{},groundAnchorB{},linearOffset{};
    float length=0,minLength=-1,maxLength=-1,frequency=0,damping=0.7f;
    float lower=0,upper=0,motorSpeed=0,maxMotor=0,referenceAngle=0;
    float ratio=1,constant=0,angularOffset=0,correctionFactor=0.3f,maxTorque=0,maxForce=1000;
    float breakForce=std::numeric_limits<float>::infinity(),breakTorque=std::numeric_limits<float>::infinity();
    bool enableLimit=false,enableMotor=false,collideConnected=false,enabled=true;
};
struct MOSS_API WorldArchive2D {
    uint32_t formatVersion=1;
    WorldSettings settings{};
    uint64_t stepCount=0;
    std::vector<ArchivedBody2D> bodies;
    std::vector<ArchivedJoint2D> joints;
};
struct MOSS_API JointDebugInfo2D {
    JointID id=InvalidJointID;
    JointType type=JointType::Distance;
    BodyID bodyA=InvalidBodyID,bodyB=InvalidBodyID;
    Vec2 anchorA{},anchorB{};
    bool enabled=false;
};
struct MOSS_API PhysicsDebugOptions2D {
    bool drawAABBs=true;
    bool drawFatAABBs=false;
    bool drawCentersOfMass=false;
    bool drawVelocities=false;
    bool drawContacts=true;
    bool drawJoints=true;
    bool drawBodyNames=true;
    float velocityScale=0.15f;
    float contactNormalScale=0.25f;
};

// Optional deformable/rigid coupling callbacks. Hooks run inside the world's
// fixed-step schedule: prepare after rigid velocity integration, velocity once
// after each rigid velocity iteration, position once after each rigid position
// iteration, and finish before state/cache publication. Callbacks must not call
// Step/StepFixed recursively. They may apply impulses/position corrections.
struct MOSS_API PhysicsStepHooks2D {
    std::function<void(float)> prepare;
    // Dynamic rigid bodies connected through external constraints are merged
    // into the same island before the solver graph is finalized.
    std::function<void(std::vector<std::pair<BodyID,BodyID>>&)> gatherIslandLinks;
    std::function<void(float,int,const std::vector<BodyID>&)> solveVelocityIsland;
    std::function<void(float,int,const std::vector<BodyID>&)> solvePositionIsland;
    // External components with no dynamic rigid-body anchor are independent
    // islands and are advanced serially after each rigid-island pass.
    std::function<void(float,int)> solveVelocityDetached;
    std::function<void(float,int)> solvePositionDetached;
    // Global callbacks retained for simple integrations.
    std::function<void(float,int)> solveVelocityIteration;
    std::function<void(float,int)> solvePositionIteration;
    std::function<void(float)> finish;
};

class MOSS_API PhysicsWorld2D {
public:
    explicit PhysicsWorld2D(const WorldSettings& settings = {});
    ~PhysicsWorld2D();
    PhysicsWorld2D(PhysicsWorld2D&&) noexcept;
    PhysicsWorld2D& operator=(PhysicsWorld2D&&) noexcept;
    PhysicsWorld2D(const PhysicsWorld2D&) = delete;
    PhysicsWorld2D& operator=(const PhysicsWorld2D&) = delete;

    BodyID CreateBody(const BodyDef& def);
    bool DestroyBody(BodyID id);
    BodyState* GetBody(BodyID id);
    const BodyState* GetBody(BodyID id) const;
    const Shape2D* GetBodyShape(BodyID id) const;
    AABB2 GetBodyAABB(BodyID id) const;
    AABB2 GetBroadPhaseFatAABB(BodyID id) const;
    Transform2D GetInterpolatedTransform(BodyID id) const;
    bool SetTransform(BodyID id, Vec2 position, float angle, bool wake=true);
    bool SetLinearVelocity(BodyID id, Vec2 velocity, bool wake=true);
    bool SetAngularVelocity(BodyID id, float velocity, bool wake=true);
    bool ApplyForce(BodyID id, Vec2 force, Vec2 worldPoint, bool wake=true);
    bool ApplyForceToCenter(BodyID id, Vec2 force, bool wake=true);
    bool ApplyTorque(BodyID id, float torque, bool wake=true);
    bool ApplyLinearImpulse(BodyID id, Vec2 impulse, Vec2 worldPoint, bool wake=true);
    bool ApplyPositionImpulse(BodyID id, Vec2 impulse, Vec2 worldPoint, bool wake=true);
    bool ApplyAngularImpulse(BodyID id, float impulse, bool wake=true);
    bool SetBodyEnabled(BodyID id, bool enabled);
    bool SetBodyGravityScale(BodyID id, float scale);
    bool SetBodyDamping(BodyID id, float linear, float angular);
    bool SetBodyShape(BodyID id, const Shape2D& shape, float density=1.0f);
    bool SetBodyMaterial(BodyID id, const Material2D& material);
    bool SetBodyFilter(BodyID id, const CollisionFilter& filter);
    void SetGravity(Vec2 gravity);
    Vec2 GetGravity() const;
    const WorldSettings& GetSettings() const;
    void SetSettings(const WorldSettings& settings);
    void Step(float frameDelta);
    void StepFixed(float dt, const PhysicsStepHooks2D* hooks=nullptr);
    void ClearForces();

    JointID CreateDistanceJoint(const DistanceJointDef& def);
    JointID CreateRevoluteJoint(const RevoluteJointDef& def);
    JointID CreateWeldJoint(const WeldJointDef& def);
    JointID CreatePrismaticJoint(const PrismaticJointDef& def);
    JointID CreateWheelJoint(const WheelJointDef& def);
    JointID CreateRopeJoint(const RopeJointDef& def);
    JointID CreateMouseJoint(const MouseJointDef& def);
    JointID CreateFrictionJoint(const FrictionJointDef& def);
    JointID CreateMotorJoint(const MotorJointDef& def);
    JointID CreatePulleyJoint(const PulleyJointDef& def);
    JointID CreateGearJoint(const GearJointDef& def);
    bool SetJointMotor(JointID id,float speed,float maxForce,bool enabled=true);
    bool SetMouseJointTarget(JointID id, Vec2 target, bool wake=true);
    bool SetJointBreakThreshold(JointID id, float force, float torque=std::numeric_limits<float>::infinity());
    Vec2 GetJointReactionForce(JointID id, float inverseDt) const;
    float GetJointReactionTorque(JointID id, float inverseDt) const;
    bool DestroyJoint(JointID id);
    const std::vector<JointBreakEvent2D>& GetJointBreakEvents() const;

    std::vector<BodyID> QueryAABB(const AABB2& bounds, const CollisionFilter& filter = {}) const;
    BroadPhaseTreeStats2D GetBroadPhaseStats() const;
    bool ValidateBroadPhase() const;
    std::vector<BodyID> QueryPoint(Vec2 point, const CollisionFilter& filter = {}) const;
    std::vector<BodyID> QueryShape(const Shape2D& shape, const Transform2D& transform, const CollisionFilter& filter = {}) const;
    std::optional<RayCastHit> RayCast(Vec2 from, Vec2 to, const CollisionFilter& filter = {}, BodyID ignore = InvalidBodyID) const;
    std::vector<RayCastHit> RayCastAll(Vec2 from, Vec2 to, const CollisionFilter& filter = {}, BodyID ignore = InvalidBodyID) const;
    std::optional<ShapeCastHit2D> ShapeCast(const Shape2D& shape, const Transform2D& start, Vec2 translation,
        float rotationDelta=0.0f, const CollisionFilter& filter={}, BodyID ignore=InvalidBodyID, float targetDistance=0.001f) const;
    TimeOfImpact2D ComputeTimeOfImpact(BodyID bodyA, BodyID bodyB, float dt) const;

    const std::vector<ContactEvent>& GetBeginContactEvents() const;
    const std::vector<ContactEvent>& GetEndContactEvents() const;
    const std::vector<ContactEvent>& GetActiveContactEvents() const;
    std::vector<JointDebugInfo2D> GetJointDebugInfo() const;
    void SetContactListener(std::function<void(const ContactEvent&, bool began)> listener);
    void SetPreSolveFilter(std::function<bool(BodyID, BodyID, const Manifold2D&)> filter);

    float GetInterpolationAlpha() const;
    uint64_t GetStepCount() const;
    std::vector<BodyID> GetBodies() const;
    PhysicsStats2D GetStats() const;
    PhysicsProfile2D GetProfile() const;
    void ResetProfile();
    WorldSnapshot2D CaptureSnapshot() const;
    bool RestoreSnapshot(const WorldSnapshot2D& snapshot, bool requireExactBodySet=true);
    WorldArchive2D CaptureArchive() const;
    bool RestoreArchive(const WorldArchive2D& archive, std::vector<std::pair<BodyID,BodyID>>* bodyIdRemap=nullptr, std::vector<std::pair<JointID,JointID>>* jointIdRemap=nullptr);

    // Change a body between Static / Kinematic / Dynamic at runtime. Fails if Dynamic would end up massless.
    bool SetBodyType(BodyID id, BodyType type);
    // Dynamic bodies only. Sleeping zeroes velocity (same as the automatic sleep path).
    bool SetBodyAwake(BodyID id, bool awake);
    bool SetBodyBullet(BodyID id, bool bullet);
    bool SetBodyFixedRotation(BodyID id, bool fixedRotation);
    bool SetBodySleepingAllowed(BodyID id, bool allow);
    // Overrides the mass computed from the shape. `inertia` is about the body's local origin
    // (same convention as ComputeMass). Survives SetBodyShape/SetBodyMaterial until reset.
    bool SetBodyMassData(BodyID id, const MassData& mass);
    bool ResetBodyMassData(BodyID id);

    // ---- Joint state ----
    // Getters return nullopt for invalid/destroyed joints or joint types the value doesn't apply to.
    bool IsJointValid(JointID id) const;
    std::optional<JointType> GetJointType(JointID id) const;
    std::optional<float> GetJointAngle(JointID id) const;         // Revolute, Weld (radians, relative to reference angle)
    std::optional<float> GetJointAngularSpeed(JointID id) const;  // Revolute, Wheel (wB - wA)
    std::optional<float> GetJointTranslation(JointID id) const;   // Prismatic, Wheel (along axis A)
    std::optional<float> GetJointLinearSpeed(JointID id) const;   // Prismatic, Wheel
    bool SetJointLimits(JointID id, bool enable, float lower, float upper);   // Revolute, Prismatic
    bool SetJointSpring(JointID id, float frequencyHz, float dampingRatio);   // Distance, Weld, Wheel, Mouse
    bool SetJointEnabled(JointID id, bool enabled);                           // reversible, unlike DestroyJoint
    bool SetDistanceJointLength(JointID id, float length, float minLength=-1.0f, float maxLength=-1.0f);

private:
    struct MOSS_API Impl;
    std::unique_ptr<Impl> impl_;
};

// Ragdoll: an articulated 2D collection of rigid bodies joined by revolute joints.
struct MOSS_API RagdollBoneDef {
    std::string name;
    int parent=-1;
    Shape2D shape=Shape2D::Box({0.12f,0.35f});
    float mass=1.0f;
    Vec2 localPosition{};       // relative to ragdoll origin for root, parent bone for children
    Vec2 parentAnchor{};        // local anchor on parent
    Vec2 childAnchor{};         // local anchor on this bone
    float lowerAngle=-0.8f, upperAngle=0.8f;
    bool enableLimit=true;
};
struct MOSS_API RagdollDef {
    Vec2 position{};
    float angle=0.0f;
    std::vector<RagdollBoneDef> bones;
    float friction=0.7f;
    float restitution=0.0f;
    CollisionFilter filter{};
    bool collideSelf=false; // same negative group suppresses internal bone contacts when false
};
struct MOSS_API ActiveRagdollSettings2D {
    float stiffness=18.0f;
    float damping=3.0f;
    float maxTorque=80.0f;
    float maxAngularSpeed=18.0f;
    float blend=1.0f;
    std::vector<float> perJointTorqueScale{};
};
struct MOSS_API Ragdoll2D {
    std::vector<BodyID> bones;
    std::vector<JointID> joints;
    std::vector<uint32_t> jointParentBone,jointChildBone;
    std::vector<std::string> boneNames;
    bool valid=false;
    void Destroy(PhysicsWorld2D& world);
};
Ragdoll2D CreateRagdoll2D(PhysicsWorld2D& world, const RagdollDef& def);
RagdollDef MakeHumanoidRagdoll(Vec2 position, float scale=1.0f);
void ApplyRagdollPoseToSkeleton(const PhysicsWorld2D& world, const Ragdoll2D& ragdoll, Skeleton2D& skeleton, float blend=1.0f);
void DriveRagdollPose(PhysicsWorld2D& world,const Ragdoll2D& ragdoll,const std::vector<float>& targetRelativeAngles, float stiffness=18.0f,float damping=3.0f,float maxTorque=80.0f);
void DriveRagdollPoseAdvanced(PhysicsWorld2D& world,const Ragdoll2D& ragdoll, const std::vector<float>& targetRelativeAngles, const ActiveRagdollSettings2D& settings);

// Lightweight platformer controller helper. The host owns the body and rendering.
struct MOSS_API PlatformerControllerSettings {
    float moveSpeed=6.0f;
    float jumpSpeed=8.0f;
    float groundProbe=0.16f;
    float groundNormalThreshold=0.55f;
    float airControl=0.8f;
    float groundAcceleration=55.0f, airAcceleration=22.0f, groundDeceleration=65.0f;
    float coyoteTime=0.10f, jumpBufferTime=0.12f;
    float jumpCutMultiplier=0.45f, maxFallSpeed=28.0f, groundSnapDistance=0.04f;
};
class MOSS_API PlatformerController2D {
public:
    PlatformerController2D(PhysicsWorld2D& world, BodyID body, PlatformerControllerSettings settings={});
    bool IsGrounded() const { return grounded_; }
    Vec2 GetGroundNormal() const { return groundNormal_; }
    void Update(float horizontalInput, bool jumpPressed, float dt);
    void SetJumpHeld(bool held) { jumpHeld_=held; }
    void Reset();
private:
    PhysicsWorld2D* world_;
    BodyID body_;
    PlatformerControllerSettings settings_;
    bool grounded_=false;
    Vec2 groundNormal_{0,1};
    float coyoteTimer_=0.0f, jumpBufferTimer_=0.0f;
    bool jumpHeld_=false, wasGrounded_=false, jumpCutApplied_=false;
};

struct MOSS_API CharacterSettings2D {
    float skinWidth=0.02f;
    float maxSlopeAngle=50.0f*kPi/180.0f;
    float stepHeight=0.3f;
    float snapDistance=0.08f;
    int maxSlideIterations=5;
    bool enableStepClimbing=true;
    bool enableGroundSnap=true;
    Vec2 upDirection{0.0f,1.0f};
    bool carryMovingPlatforms=true;
    float platformCarryScale=1.0f;
};
struct MOSS_API CharacterMoveResult2D {
    Vec2 position{},actualDisplacement{},groundNormal{0,1};
    BodyID groundBody=InvalidBodyID;
    bool grounded=false,hitCeiling=false,hitWall=false,stepped=false;
    uint32_t sweepCount=0;
};
class MOSS_API CharacterController2D {
public:
    CharacterController2D(PhysicsWorld2D& world,BodyID body,KinematicCharacterSettings2D settings={});
    CharacterMoveResult2D Move(Vec2 displacement);
    CharacterMoveResult2D MoveAndSlide(Vec2 desiredVelocity,float dt);
    void SetVelocity(Vec2 velocity){velocity_=velocity;}
    Vec2 GetVelocity()const{return velocity_;}
    bool IsGrounded()const{return lastResult_.grounded;}
    const CharacterMoveResult2D& GetLastMoveResult()const{return lastResult_;}
    void SetSettings(const KinematicCharacterSettings2D& settings){settings_=settings;}
private:
    bool TryStep(Vec2 remaining,Vec2& position,CharacterMoveResult2D& result);
    PhysicsWorld2D* world_=nullptr; BodyID body_=InvalidBodyID;
    CharacterSettings2D settings_{}; 
    Vec2 velocity_{}; 
    CharacterMoveResult2D lastResult_{};
    BodyID lastGroundBody_=InvalidBodyID;
    Vec2 lastGroundLocalAnchor_{};
    Transform2D lastGroundTransform_{};
    bool hasGroundAnchor_=false;
};

// Debug drawing is renderer-agnostic; implement these callbacks with Moss's renderer.
struct MOSS_API DebugDraw2D {
    std::function<void(Vec2,Vec2,uint32_t)> line;
    std::function<void(Vec2,float,uint32_t)> circle;
    std::function<void(const std::vector<Vec2>&,uint32_t)> polygon;
    std::function<void(Vec2,const std::string&,uint32_t)> text;
};
void DrawDebug(const PhysicsWorld2D& world, const DebugDraw2D& draw, const PhysicsDebugOptions2D& options);
void DrawDebug(const PhysicsWorld2D& world, const DebugDraw2D& draw, bool drawAABBs=true);

// Tilemap helper: converts each solid row into merged horizontal edge spans.
// Use one static chain per chunk and rebuild only modified chunks.
std::vector<Shape2D> BuildTilemapEdges(const std::vector<uint8_t>& solid, int width, int height, float cellSize=1.0f, Vec2 origin={}, bool mergeCollinear=true);

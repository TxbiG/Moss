#pragma once
#include "Shapes2D.hpp"
#include <cstdint>
#include <limits>

#include <Moss/Physics/Body/BodyID.h>

using BodyID = uint32_t;
using JointID = uint32_t;
constexpr BodyID InvalidBodyID = 0xffffffffu;
constexpr JointID InvalidJointID = 0xffffffffu;

struct MOSS_API DistanceJointDef {
    BodyID bodyA=InvalidBodyID, bodyB=InvalidBodyID;
    Vec2 localAnchorA{}, localAnchorB{};
    float length=-1.0f;
    float frequencyHz=0.0f;
    float dampingRatio=0.7f;
    float minLength=-1.0f, maxLength=-1.0f;
    bool collideConnected=false;
};
struct MOSS_API RevoluteJointDef {
    BodyID bodyA=InvalidBodyID, bodyB=InvalidBodyID;
    Vec2 localAnchorA{}, localAnchorB{};
    bool enableLimit=false;
    float lowerAngle=-MOSS_PI, upperAngle=MOSS_PI;
    bool enableMotor=false;
    float motorSpeed=0.0f;
    float maxMotorTorque=0.0f;
    bool collideConnected=false;
};
struct MOSS_API WeldJointDef {
    BodyID bodyA=InvalidBodyID, bodyB=InvalidBodyID;
    Vec2 localAnchorA{}, localAnchorB{};
    float referenceAngle=std::numeric_limits<float>::quiet_NaN();
    float frequencyHz=0.0f;
    float dampingRatio=0.7f;
    bool collideConnected=false;
};
struct MOSS_API PrismaticJointDef {
    BodyID bodyA=InvalidBodyID, bodyB=InvalidBodyID;
    Vec2 localAnchorA{}, localAnchorB{};
    Vec2 localAxisA{1,0};
    bool enableLimit=false;
    float lowerTranslation=0.0f, upperTranslation=0.0f;
    bool enableMotor=false;
    float motorSpeed=0.0f, maxMotorForce=0.0f;
    bool collideConnected=false;
};
struct MOSS_API WheelJointDef {
    BodyID bodyA=InvalidBodyID, bodyB=InvalidBodyID;
    Vec2 localAnchorA{}, localAnchorB{};
    Vec2 localAxisA{0,1};
    float frequencyHz=4.0f, dampingRatio=0.7f;
    bool enableMotor=false;
    float motorSpeed=0.0f, maxMotorTorque=0.0f;
    bool collideConnected=false;
};
struct MOSS_API RopeJointDef {
    BodyID bodyA=InvalidBodyID, bodyB=InvalidBodyID;
    Vec2 localAnchorA{}, localAnchorB{};
    float maxLength=1.0f;
    bool collideConnected=false;
};
struct MOSS_API MouseJointDef {
    BodyID body=InvalidBodyID;
    Vec2 localAnchor{};
    Vec2 target{};
    float frequencyHz=5.0f, dampingRatio=0.7f, maxForce=1000.0f;
};
struct MOSS_API FrictionJointDef {
    BodyID bodyA=InvalidBodyID,bodyB=InvalidBodyID;
    Vec2 localAnchorA{},localAnchorB{};
    float maxForce=0.0f,maxTorque=0.0f;
    bool collideConnected=false;
};
struct MOSS_API MotorJointDef {
    BodyID bodyA=InvalidBodyID,bodyB=InvalidBodyID;
    Vec2 linearOffset{}; // desired B origin relative to A, in A local space
    float angularOffset=0.0f;
    float maxForce=1.0f,maxTorque=1.0f,correctionFactor=0.3f;
    bool collideConnected=true;
};
struct MOSS_API PulleyJointDef {
    BodyID bodyA=InvalidBodyID,bodyB=InvalidBodyID;
    Vec2 groundAnchorA{-1,1},groundAnchorB{1,1};
    Vec2 localAnchorA{},localAnchorB{};
    float ratio=1.0f;
    float constantLength=-1.0f;
    bool collideConnected=true;
};
struct MOSS_API GearJointDef {
    BodyID bodyA=InvalidBodyID,bodyB=InvalidBodyID;
    float ratio=1.0f;
    float constant=std::numeric_limits<float>::quiet_NaN();
    float motorSpeed=0.0f,maxMotorTorque=0.0f;
    bool enableMotor=false;
    bool collideConnected=true;
};

struct MOSS_API JointBreakEvent2D {
    JointID joint=InvalidJointID;
    BodyID bodyA=InvalidBodyID, bodyB=InvalidBodyID;
    Vec2 reactionForce{};
    float reactionTorque=0.0f;
};
enum class JointType : uint8_t { Distance, Revolute, Weld, Prismatic, Wheel, Rope, Mouse, Friction, Motor, Pulley, Gear };

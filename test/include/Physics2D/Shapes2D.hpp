#pragma once
#include <vector>
#include <memory>
#include <cstddef>

#include <Moss/Moss_stdinc.h>
#include <Moss/Variants/Vector/Vec2.h>

// Capsule() and RoundedBox() are built as polygon approximations, not true capsules
constexpr float MOSS_PI = 3.14159265358979323846f;
constexpr float kEpsilon = 1.0e-6f;

enum class ShapeType : uint8_t { Circle, Polygon, Edge, Chain, Compound };


struct [[nodiscard]] alignas(MOSS_VECTOR_ALIGNMENT) Rot2 {
    float c = 1.0f, s = 0.0f;
    Rot2() = default;
    explicit Rot2(float angle) : c(std::cos(angle)), s(std::sin(angle)) {}
    Vec2 Apply(Vec2 v) const { return {c*v.GetX() - s*v..GetY(), s*v..GetX() + c*v.GetY()}; }
    Vec2 ApplyInverse(Vec2 v) const { return {c*v.x + s*v.y, -s*v.x + c*v.y}; }
};

struct [[nodiscard]] alignas(MOSS_VECTOR_ALIGNMENT) Transform2D {
    Vec2 position;
    float angle = 0.0f;
    Vec2 Apply(Vec2 local) const { return position + Rot2(angle).Apply(local); }
    Vec2 ApplyInverse(Vec2 world) const { return Rot2(angle).ApplyInverse(world-position); }
};

struct MOSS_API CircleShape {
    Vec2 center{};
    float radius = 0.5f;
};
struct MOSS_API PolygonShape {
    std::vector<Vec2> vertices; // convex, counter-clockwise, local space
    std::vector<Vec2> normals;  // outward normals
    Vec2 centroid{};
};
struct MOSS_API EdgeShape {
    Vec2 a{}, b{};              // local-space segment
    Vec2 ghostA{}, ghostB{};    // optional adjacent vertices for chain smoothing
    bool hasGhostA=false, hasGhostB=false;
    bool oneSided=false;
};
struct MOSS_API ChainShape {
    std::vector<Vec2> vertices;
    bool loop=false;
    bool oneSided=false;
};

struct MOSS_API CompoundChild2D;
struct MOSS_API CompoundShape2D;

struct MOSS_API Shape2D {
    ShapeType type = ShapeType::Circle;
    CircleShape circle{};
    PolygonShape polygon{};
    EdgeShape edge{};
    ChainShape chain{};
    std::shared_ptr<CompoundShape2D> compound;

    static Shape2D Circle(float radius, Vec2 center = {}) {
        Shape2D s; s.type=ShapeType::Circle; s.circle={center,std::max(radius,kEpsilon)}; return s;
    }
    static Shape2D Box(Vec2 halfExtents, Vec2 center = {}, float angle = 0.0f) {
        halfExtents.x=std::max(halfExtents.x,kEpsilon); halfExtents.y=std::max(halfExtents.y,kEpsilon);
        const Rot2 r(angle);
        std::vector<Vec2> v = { {-halfExtents.x,-halfExtents.y}, {halfExtents.x,-halfExtents.y},
                                {halfExtents.x,halfExtents.y}, {-halfExtents.x,halfExtents.y} };
        for (auto& p : v) p=center+r.Apply(p);
        return Polygon(v);
    }
    static Shape2D Polygon(const std::vector<Vec2>& points);
    static Shape2D Capsule(Vec2 endpointA, Vec2 endpointB, float radius, int arcSegments=6);
    static Shape2D RoundedBox(Vec2 halfExtents, float cornerRadius, Vec2 center = {}, float angle = 0.0f, int arcSegments=3);
    static Shape2D Edge(Vec2 a, Vec2 b, bool oneSided = false) {
        Shape2D s; s.type=ShapeType::Edge; s.edge.a=a; s.edge.b=b; s.edge.oneSided=oneSided; return s;
    }
    static Shape2D Chain(const std::vector<Vec2>& points, bool loop = false, bool oneSided = false) {
        Shape2D s; s.type=ShapeType::Chain; s.chain.vertices=points; s.chain.loop=loop; s.chain.oneSided=oneSided; return s;
    }
    static Shape2D Compound(const std::vector<CompoundChild2D>& children);
    bool IsValid() const;
};

struct MOSS_API CompoundChild2D {
    Shape2D shape{};
    Transform2D transform{};
    uint32_t id=0xffffffffu;
    void* userData=nullptr;
    bool isSensor=false;
    uint16_t categoryBits=0x0001, maskBits=0xffff;
    int16_t groupIndex=0;
    float friction=-1.0f, restitution=-1.0f, density=-1.0f;
};
struct MOSS_API CompoundShape2D { std::vector<CompoundChild2D> children; };

struct MOSS_API MassData {
    float mass=0.0f;
    Vec2 center{};
    float inertia=0.0f; // about the local origin
};
MassData ComputeMass(const Shape2D& shape, float density);
AABB2 ComputeAABB(const Shape2D& shape, const Transform2D& transform);
bool TestPoint(const Shape2D& shape, const Transform2D& transform, Vec2 point);
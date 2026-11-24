#pragma once
#include "Shapes2D.hpp"
#include <optional>

struct MOSS_API ShapeDistance2D {
    bool valid = false;
    bool overlapping = false;
    float distance = 0.0f;
    Vec2 pointA{}, pointB{};
    Vec2 normal{1.0f, 0.0f}; // from A toward B
    uint32_t iterations = 0;
};

struct MOSS_API ShapeCastResult2D {
    bool hit = false;
    float fraction = 1.0f;
    Vec2 point{}, normal{0.0f, 1.0f};
    uint32_t iterations = 0;
};

// GJK distance query for convex circle/polygon/edge shapes.
MOSS_API ShapeDistance2D DistanceBetweenShapes2D(const Shape2D& a, const Transform2D& transformA, const Shape2D& b, const Transform2D& transformB);

// Conservative-advancement shape cast. Translation and rotation are parameterized over [0, 1].
// Compound and chain shapes are decomposed into convex children.
MOSS_API ShapeCastResult2D CastShapeAgainstShape2D(const Shape2D& movingShape, const Transform2D& start, Vec2 translation, float rotationDelta, const Shape2D& targetShape, 
    const Transform2D& target, float targetDistance = 0.001f, uint32_t maxIterations = 32, Vec2 targetTranslation = {}, float targetRotationDelta = 0.0f);

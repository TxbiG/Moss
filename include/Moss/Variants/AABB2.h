#pragma once

#include <Moss/Core/Variants/Vector/Vec4.h>

MOSS_WARNINGS_BEGIN

// AABB2 (Axis aligned box 2D) is used for 2D Collisions
class [[nodiscard]] AABB2 {
public:
    // Constructors
    AABB2() : mBox(Vec4(FLT_MAX, FLT_MAX, -FLT_MAX, -FLT_MAX)) {}
    AABB2(Vec2Arg inMin, Vec2Arg inMax) : mBox(inMin.GetX(), inMin.GetY(), inMax.GetX(), inMax.GetY()) {}
    AABB2(Vec2Arg inCenter, float inRadius) {
        Vec2 r   = Vec2::sReplicate(inRadius);
        Vec2 min = inCenter - r;
        Vec2 max = inCenter + r;
        mBox     = Vec4(min.GetX(), min.GetY(), max.GetX(), max.GetY());
    }

    static AABB2 sFromTwoPoints(Vec2Arg a, Vec2Arg b) { return AABB2(Vec2::sMin(a, b), Vec2::sMax(a, b)); }

    // Reset
    void SetEmpty() { mBox = Vec4(FLT_MAX, FLT_MAX, -FLT_MAX, -FLT_MAX); }

    // Validation
    bool IsValid() const { return (mBox.GetX() <= mBox.GetZ()) && (mBox.GetY() <= mBox.GetW()); }

    // Encapsulation
    void Encapsulate(Vec2Arg p) {
        // Broadcast p to Vec4: (px, py, px, py)
        RVec4 point(p.GetX(), p.GetY(), p.GetX(), p.GetY());

        // Min for lower half, Max for upper half
        mBox = Vec4( std::min(mBox.GetX(), point.GetX()), std::min(mBox.GetY(), point.GetY()), std::max(mBox.GetZ(), point.GetZ()), std::max(mBox.GetW(), point.GetW()));
    }

    void Encapsulate(const AABB2 &other) { mBox = Vec4(std::min(mBox.GetX(), other.mBox.GetX()), std::min(mBox.GetY(), other.mBox.GetY()), std::max(mBox.GetZ(), 
		other.mBox.GetZ()), std::max(mBox.GetW(), other.mBox.GetW())); }

    // Query
    bool Contains(Vec2Arg p) const { return (p.GetX() >= mBox.GetX() && p.GetY() >= mBox.GetY()) && (p.GetX() <= mBox.GetZ() && p.GetY() <= mBox.GetW()); }
    bool Overlaps(const AABB2 &other) const { return (other.mBox.GetX() < mBox.GetZ() && other.mBox.GetY() < mBox.GetW() && other.mBox.GetZ() > mBox.GetX() && other.mBox.GetW() > mBox.GetY()); }

    // Math
    RVec2 GetCenter() const { return Vec2((mBox.GetX() + mBox.GetZ()) * 0.5f, (mBox.GetY() + mBox.GetW()) * 0.5f); }
    RVec2 GetExtent() const { return Vec2( (mBox.GetZ() - mBox.GetX()) * 0.5f, (mBox.GetW() - mBox.GetY()) * 0.5f); }
    RVec2 GetSize() const { return Vec2(mBox.GetZ() - mBox.GetX(), mBox.GetW() - mBox.GetY()); }
    float GetArea() const { Vec2 size = GetSize(); return size.GetX() * size.GetY(); }

    RVec2 GetClosestPoint(Vec2Arg p) const { return Vec2( std::clamp(p.GetX(), mBox.GetX(), mBox.GetZ()), std::clamp(p.GetY(), mBox.GetY(), mBox.GetW())); }
    float GetSqDistanceTo(Vec2Arg p) const { Vec2 delta = GetClosestPoint(p) - p; return delta.LengthSq(); }

    // Grow/shrink
    void ExpandBy(Vec2Arg amount) { mBox = Vec4( mBox.GetX() - amount.GetX(), mBox.GetY() - amount.GetY(), mBox.GetZ() + amount.GetX(), mBox.GetW() + amount.GetY() ); }

    void EnsureMinimalEdgeLength(float min_edge) {
        float sizeX = mBox.GetZ() - mBox.GetX();
        float sizeY = mBox.GetW() - mBox.GetY();

        if (sizeX < min_edge) mBox.SetZ(mBox.GetX() + min_edge);
        if (sizeY < min_edge) mBox.SetW(mBox.GetY() + min_edge);
    }

    // Accessors
    RVec2 GetMin() const { return Vec2(mBox.GetX(), mBox.GetY()); }
    RVec2 GetMax() const { return Vec2(mBox.GetZ(), mBox.GetW()); }


	bool operator == (const AABB2 &inRHS) const				{ return mMin == inRHS.GetMin() && mMax == inRHS.GetMax(); }
	bool operator != (const AABB2 &inRHS) const				{ return mMin != inRHS.GetMin() || mMax != inRHS.GetMax(); }

private:
    RVec4 mBox; // (x, y = min, z, w = max)
};

MOSS_WARNINGS_END
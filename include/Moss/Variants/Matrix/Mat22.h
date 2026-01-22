// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Core/Variants/Math/MathTypes.h>

MOSS_WARNINGS_BEGIN

/// Holds a 4x4 matrix of floats, but supports also operations on the 3x3 upper left part of the matrix.

class [[nodiscard]] alignas(MOSS_VECTOR_ALIGNMENT) Mat22 {
public:
    MOSS_OVERRIDE_NEW_DELETE

    Vec4 m; // (m00, m01, m10, m11)

    Mat22() = default;

    MOSS_INLINE Mat22(float m00, float m01, float m10, float m11) : m(m00, m01, m10, m11) {}
    MOSS_INLINE Mat22(const Vec2 &c0, const Vec2 &c1) : m(c0.x, c1.x, c0.y, c1.y) {}

    static MOSS_INLINE Mat22 sZero() { return Mat22(0.0f, 0.0f, 0.0f, 0.0f); }

    static MOSS_INLINE Mat22 sIdentity() { return Mat22(1.0f, 0.0f, 0.0f, 1.0f); }

    static MOSS_INLINE Mat22 sNaN() { return Mat22(JPH_NAN, JPH_NAN, JPH_NAN, JPH_NAN); }

    MOSS_INLINE float Determinant() const { return m.GetX() * m.GetW() - m.GetY() * m.GetZ();}

    MOSS_INLINE Mat22 Inverse() const {
        float det = Determinant();
        MOSS_ASSERT(det != 0.0f);

        float invDet = 1.0f / det;

        return Mat22(m.GetW() * invDet, -m.GetY() * invDet, -m.GetZ() * invDet, m.GetX() * invDet);
    }

    MOSS_INLINE Vec2 operator * (const Vec2 &v) const {
        Vec4 xy(v.x, v.y, v.x, v.y);
        Vec4 r = m * xy;

        return Vec2(r.GetX() + r.GetY(), r.GetZ() + r.GetW());
    }

    MOSS_INLINE Mat22 operator * (float s) const { return Mat22(m * Vec4::sReplicate(s)); }

    MOSS_INLINE Mat22 operator * (const Mat22 &rhs) const {
        return Mat22(
            m.GetX() * rhs.m.GetX() + m.GetY() * rhs.m.GetZ(),
            m.GetX() * rhs.m.GetY() + m.GetY() * rhs.m.GetW(),
            m.GetZ() * rhs.m.GetX() + m.GetW() * rhs.m.GetZ(),
            m.GetZ() * rhs.m.GetY() + m.GetW() * rhs.m.GetW());
    }

private:
    explicit Mat22(const Vec4 &v) : m(v) {}
};

static_assert(std::is_trivial<Mat22>(), "Is supposed to be a trivial type!");

MOSS_WARNINGS_END
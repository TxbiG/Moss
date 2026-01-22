// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Core/HashCombine.h>
#include <Moss/Core/Variants/TArray.h>
MOSS_SUPRESS_WARNINGS_BEGIN

/// Class that holds 3 floats. Used as a storage class. Convert to Vec3 for calculations.
class [[nodiscard]] Float3
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/* 			Constructors		*/
	Float3() = default;
	Float3(const Float3 &inRHS) = default;
	constexpr Float3(float x, float y, float z) : x(x), y(y), z(z) { }

	/* 			Operators		*/
	Float3&	operator = (const Float3 &inRHS) = default;
	float	operator [] (int inCoordinate) const { MOSS_ASSERT(inCoordinate < 3); return *(&x + inCoordinate); }
	bool	operator == (const Float3 &inRHS) const { return x == inRHS.x && y == inRHS.y && z == inRHS.z; }
	bool	operator != (const Float3 &inRHS) const { return x != inRHS.x || y != inRHS.y || z != inRHS.z; }
	inline Float3 operator-() const { return Float3{-x, -y, -z}; }
	MOSS_INLINE Float3 operator-(const Float3 &rhs) const noexcept { return { x - rhs.x, y - rhs.y, z - rhs.z }; }
    MOSS_INLINE Float3 operator+(const Float3 &rhs) const noexcept { return { x + rhs.x, y + rhs.y, z + rhs.z }; }

	float		x;
	float		y;
	float		z;
};

using VertexList = TArray<Float3>;

static_assert(std::is_trivial<Float3>(), "Is supposed to be a trivial type!");

MOSS_SUPRESS_WARNINGS_END

// Create a std::hash for Float3
MOSS_MAKE_HASHABLE(Float3, t.x, t.y, t.z)
// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Core/HashCombine.h>
#include <Moss/Variants/TArray.h>

MOSS_SUPRESS_WARNINGS_BEGIN

/// Class that holds 4 float values. Convert to Vec4 to perform calculations.
class [[nodiscard]] Float4 {
public:
	MOSS_OVERRIDE_NEW_DELETE

	/* 			Constructors		*/
	Float4() = default; // Intentionally not initialized for performance reasons
	Float4(const Float4 &inRHS) = default;
	constexpr Float4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) { }

	/* 			Operators		*/
	Float4&	operator = (const Float4 &inRHS) = default;
	//float	operator [] (int inCoordinate) const { MOSS_ASSERT(inCoordinate < 4); return *(&x + inCoordinate); }
	bool	operator == (const Float4 &inRHS) const { return x == inRHS.x && y == inRHS.y && z == inRHS.z && w == inRHS.w; }
	bool	operator != (const Float4 &inRHS) const { return x != inRHS.x || y != inRHS.y || z != inRHS.z || w != inRHS.w; }
	inline Float4 operator-() const { return Float4{-x, -y, -z, -w}; }
	MOSS_INLINE Float4 operator-(const Float4 &rhs) const noexcept { return { x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w }; }
    MOSS_INLINE Float4 operator+(const Float4 &rhs) const noexcept { return { x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w }; }

	float x, y, z, w;
};

static_assert(std::is_trivial<Float4>(), "Is supposed to be a trivial type!");

MOSS_SUPRESS_WARNINGS_END

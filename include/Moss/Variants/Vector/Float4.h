// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Core/HashCombine.h>

MOSS_SUPRESS_WARNINGS_BEGIN

/// Class that holds 4 float values. Convert to Vec4 to perform calculations.
class [[nodiscard]] Float4
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/* 			Constructors		*/
	Float4() = default; ///< Intentionally not initialized for performance reasons
	Float4(const Float4 &inRHS) = default;
	constexpr Float4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) { }

	/* 			Operators		*/
	float		operator [] (int inCoordinate) const { MOSS_ASSERT(inCoordinate < 4); return *(&x + inCoordinate); }

	float		x;
	float		y;
	float		z;
	float		w;
};

static_assert(std::is_trivial<Float4>(), "Is supposed to be a trivial type!");

MOSS_SUPRESS_WARNINGS_END

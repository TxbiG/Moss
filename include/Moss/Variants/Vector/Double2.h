// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

MOSS_SUPRESS_WARNINGS_BEGIN

/// Class that holds 3 doubles. Used as a storage class. Convert to DVec3 for calculations.
class [[nodiscard]] Double2
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	Double2() = default; ///< Intentionally not initialized for performance reasons
	Double2(const Double2 &inRHS) = default;
	Double2(double inX, double inY) : x(inX), y(inY) { }


	Double2&	operator = (const Double2 &inRHS) = default;
	
	double		operator [] (int inCoordinate) const
	{
		MOSS_ASSERT(inCoordinate < 2);
		return *(&x + inCoordinate);
	}

	bool		operator == (const Double2 &inRHS) const { return x == inRHS.x && y == inRHS.y; }
	bool		operator != (const Double2 &inRHS) const { return x != inRHS.x || y != inRHS.y; }

	double		x;
	double		y;
};

static_assert(std::is_trivial<Double2>(), "Is supposed to be a trivial type!");

MOSS_SUPRESS_WARNINGS_END

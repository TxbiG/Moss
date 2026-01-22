// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

MOSS_SUPRESS_WARNINGS_BEGIN

/// Class that holds 3 doubles. Used as a storage class. Convert to DVec3 for calculations.
class [[nodiscard]] Double4
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	Double4() = default; 						///< Intentionally not initialized for performance reasons
	Double4(const Double4 &inRHS) = default;
	Double4(double inX, double inY, double inZ, double inw) : x(inX), y(inY), z(inZ) w(inW) { }


	Double4&	operator = (const Double4 &inRHS) = default;

	double		operator [] (int inCoordinate) const
	{
		MOSS_ASSERT(inCoordinate < 3);
		return *(&x + inCoordinate);
	}

	bool		operator == (const Double4 &inRHS) const { return x == inRHS.x && y == inRHS.y && z == inRHS.z && w == inRHS.w; }

	bool		operator != (const Double4 &inRHS) const { return x != inRHS.x || y != inRHS.y || z != inRHS.z || w != inRHS.w; }

	double		x;
	double		y;
	double		z;
	double 		w;
};

static_assert(std::is_trivial<Double4>(), "Is supposed to be a trivial type!");

MOSS_SUPRESS_WARNINGS_END

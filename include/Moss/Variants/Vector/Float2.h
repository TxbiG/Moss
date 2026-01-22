// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

MOSS_SUPRESS_WARNINGS_BEGIN

/// Class that holds 2 floats, used as a storage class mainly.
class [[nodiscard]] Float2
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/* 			Constructors		*/
	Float2() = default;
	Float2(const Float2 &inRHS) = default;
	constexpr Float2(float inX, float inY) : x(inX), y(inY) {}

	/* 			Operators		*/
	Float2& operator = (const Float2 &inRHS) = default;
	bool	operator == (const Float2 &inRHS) const			{ return x == inRHS.x && y == inRHS.y; }
	bool	operator != (const Float2 &inRHS) const			{ return x != inRHS.x || y != inRHS.y; }

	/// To String
	friend ostream&	operator << (ostream &inStream, const Float2 &inV) { inStream << inV.x << ", " << inV.y; return inStream; }

	float				x;
	float				y;
};

inline Float2 operator-(const Float2 &lhs, const Float2 &rhs) noexcept {
    return Float2{lhs.x - rhs.x, lhs.y - rhs.y};
}

MOSS_SUPRESS_WARNINGS_END

MOSS_MAKE_HASHABLE(Float2, t.x, t.y)
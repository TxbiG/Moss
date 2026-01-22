#pragma once

MOSS_SUPRESS_WARNINGS_BEGIN

/// Class that holds 4 int values. Convert to Vec4 to perform calculations.
class [[nodiscard]] Int4
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/* 			Constructors		*/
	Int4() = default; 
	Int4(const Int4 &inRHS) = default;
	Int4(int x, int y, int z, int w) : x(x), y(y), z(z), w(w) { }

	/* 			Operators		*/
	int		operator [] (int inCoordinate) const { MOSS_ASSERT(inCoordinate < 4); return *(&x + inCoordinate); }

	int		x;
	int		y;
	int		z;
	int		w;
};

static_assert(std::is_trivial<Int4>(), "Is supposed to be a trivial type!");

MOSS_SUPRESS_WARNINGS_END
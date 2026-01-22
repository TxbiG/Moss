#pragma once

MOSS_SUPRESS_WARNINGS_BEGIN

/// Class that holds 3 ints. Used as a storage class. Convert to Vec3 for calculations.
class [[nodiscard]] Int3
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/* 			Constructors		*/
	Int3() = default; 
	Int3(const Int3 &inRHS) = default;
	constexpr Int3(int x, int y, int z) : x(x), y(y), z(z) { }

	/* 			Operators		*/
	Int3&	operator = (const Int3 &inRHS) = default;
	int		operator [] (int inCoordinate) const { MOSS_ASSERT(inCoordinate < 3); return *(&x + inCoordinate); }
	bool	operator == (const Int3 &inRHS) const { return x == inRHS.x && y == inRHS.y && z == inRHS.z; }
	bool	operator != (const Int3 &inRHS) const { return x != inRHS.x || y != inRHS.y || z != inRHS.z; }

	int		x;
	int		y;
	int		z;
};

//using VertexList = TArray<Int3>;

static_assert(std::is_trivial<Int3>(), "Is supposed to be a trivial type!");

MOSS_SUPRESS_WARNINGS_END

#pragma once

MOSS_SUPRESS_WARNINGS_BEGIN

/// Class that holds 2 ints, used as a storage class mainly.
class [[nodiscard]] Int2
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/* 			Constructors		*/
	Int2() = default;
	Int2(const Int2 &inRHS) = default;
	Int2(int inX, int inY) : x(inX), y(inY) { }

	/* 			Operators		*/
	Int2& operator = (const Int2 &inRHS) = default;
	bool	operator == (const Int2 &inRHS) const			{ return x == inRHS.x && y == inRHS.y; }
	bool	operator != (const Int2 &inRHS) const			{ return x != inRHS.x || y != inRHS.y; }

	/// To String
	friend ostream&	operator << (ostream &inStream, const Int2 &inV) { inStream << inV.x << ", " << inV.y; return inStream; }

	int				x;
	int				y;
};

static_assert(std::is_trivial<Int2>(), "Is supposed to be a trivial type!");

MOSS_SUPRESS_WARNINGS_END

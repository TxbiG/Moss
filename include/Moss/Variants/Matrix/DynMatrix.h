// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2022 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

MOSS_SUPPRESS_WARNINGS_BEGIN

/// Dynamic resizable matrix class
class [[nodiscard]] DynMatrix
{
public:
	/// Constructor
	DynMatrix(const DynMatrix &) = default;
	DynMatrix(uint32_t inRows, uint32_t inCols)			: mRows(inRows), mCols(inCols) { mElements.resize(inRows * inCols); }

	/// Access an element
	float			operator () (uint32_t inRow, uint32_t inCol) const	{ MOSS_ASSERT(inRow < mRows && inCol < mCols); return mElements[inRow * mCols + inCol]; }
	float&			operator () (uint32_t inRow, uint32_t inCol)		{ MOSS_ASSERT(inRow < mRows && inCol < mCols); return mElements[inRow * mCols + inCol]; }

	/// Get dimensions
	uint32_t			GetCols() const								{ return mCols; }
	uint32_t			GetRows() const								{ return mRows; }

private:
	uint32_t			mRows;
	uint32_t			mCols;
	TArray<float>	mElements;
};

MOSS_SUPPRESS_WARNINGS_END

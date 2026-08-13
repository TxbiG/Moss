// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2022 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

MOSS_SUPRESS_WARNINGS_BEGIN

/// Dynamic resizable matrix class
class [[nodiscard]] DynMatrix
{
public:
	/// Constructor
	DynMatrix(const DynMatrix &) = default;
	DynMatrix(uint32 inRows, uint32 inCols)			: mRows(inRows), mCols(inCols) { mElements.resize(inRows * inCols); }

	/// Access an element
	float			operator () (uint32 inRow, uint32 inCol) const	{ MOSS_ASSERT(inRow < mRows && inCol < mCols); return mElements[inRow * mCols + inCol]; }
	float&			operator () (uint32 inRow, uint32 inCol)		{ MOSS_ASSERT(inRow < mRows && inCol < mCols); return mElements[inRow * mCols + inCol]; }

	/// Get dimensions
	uint32			GetCols() const								{ return mCols; }
	uint32			GetRows() const								{ return mRows; }

private:
	uint32			mRows;
	uint32			mCols;
	TArray<float>	mElements;
};

MOSS_SUPRESS_WARNINGS_END

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
					DynMatrix(uint inRows, uint inCols)			: mRows(inRows), mCols(inCols) { mElements.resize(inRows * inCols); }

	/// Access an element
	float			operator () (uint inRow, uint inCol) const	{ MOSS_ASSERT(inRow < mRows && inCol < mCols); return mElements[inRow * mCols + inCol]; }
	float &			operator () (uint inRow, uint inCol)		{ MOSS_ASSERT(inRow < mRows && inCol < mCols); return mElements[inRow * mCols + inCol]; }

	/// Get dimensions
	uint			GetCols() const								{ return mCols; }
	uint			GetRows() const								{ return mRows; }

private:
	uint			mRows;
	uint			mCols;
	TArray<float>	mElements;
};

MOSS_SUPRESS_WARNINGS_END

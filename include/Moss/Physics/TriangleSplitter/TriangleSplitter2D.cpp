// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Moss/TriangleSplitter/TriangleSplitter.h>

MOSS_SUPRESS_WARNINGS_BEGIN

TriangleSplitter::TriangleSplitter(const VertexList &inVertices, const IndexedTriangleList &inTriangles) :
	mVertices(inVertices),
	mTriangles(inTriangles)
{
	mSortedTriangleIdx.resize(inTriangles.size());
	mCentroids.resize(inTriangles.size() + 1); // Add 1 so we can load with Vec3::sLoadFloat3Unsafe

	for (uint t = 0; t < inTriangles.size(); ++t)
	{
		// Initially triangles start unsorted
		mSortedTriangleIdx[t] = t;

		// Calculate centroid
		inTriangles[t].GetCentroid(inVertices).StoreFloat3(&mCentroids[t]);
	}

	// Make sure Vec3::sLoatFloat3Unsafe doesn't read uninitialized data
	mCentroids.back() = Float3(0, 0, 0);
}

bool TriangleSplitter::SplitInternal(const Range &inTriangles, uint inDimension, float inSplit, Range &outLeft, Range &outRight)
{
	// Divide triangles
	uint *start = mSortedTriangleIdx.data() + inTriangles.mBegin;
	uint *end = mSortedTriangleIdx.data() + inTriangles.mEnd;
	while (start < end)
	{
		// Search for first element that is on the right hand side of the split plane
		while (start < end && mCentroids[*start][inDimension] < inSplit)
			++start;

		// Search for the first element that is on the left hand side of the split plane
		while (start < end && mCentroids[*(end - 1)][inDimension] >= inSplit)
			--end;

		if (start < end)
		{
			// Swap the two elements
			--end;
			std::swap(*start, *end);
			++start;
		}
	}
	MOSS_ASSERT(start == end);

	uint start_idx = uint(start - mSortedTriangleIdx.data());

#ifdef MOSS_DEBUG
	// Validate division algorithm
	MOSS_ASSERT(inTriangles.mBegin <= start_idx);
	MOSS_ASSERT(start_idx <= inTriangles.mEnd);
	for (uint i = inTriangles.mBegin; i < start_idx; ++i)
		MOSS_ASSERT(mCentroids[mSortedTriangleIdx[i]][inDimension] < inSplit);
	for (uint i = start_idx; i < inTriangles.mEnd; ++i)
		MOSS_ASSERT(mCentroids[mSortedTriangleIdx[i]][inDimension] >= inSplit);
#endif

	outLeft = Range(inTriangles.mBegin, start_idx);
	outRight = Range(start_idx, inTriangles.mEnd);
	return outLeft.Count() > 0 && outRight.Count() > 0;
}










TriangleSplitterMean::TriangleSplitterMean(const VertexList &inVertices, const IndexedTriangleList &inTriangles) :
	TriangleSplitter(inVertices, inTriangles)
{
}

bool TriangleSplitterMean::Split(const Range &inTriangles, Range &outLeft, Range &outRight)
{
	const uint *begin = mSortedTriangleIdx.data() + inTriangles.mBegin;
	const uint *end = mSortedTriangleIdx.data() + inTriangles.mEnd;

	// Calculate mean value for these triangles
	Vec3 mean = Vec3::sZero();
	for (const uint *t = begin; t < end; ++t)
		mean += Vec3::sLoadFloat3Unsafe(mCentroids[*t]);
	mean *= 1.0f / inTriangles.Count();

	// Calculate deviation
	Vec3 deviation = Vec3::sZero();
	for (const uint *t = begin; t < end; ++t)
	{
		Vec3 delta = Vec3::sLoadFloat3Unsafe(mCentroids[*t]) - mean;
		deviation += delta * delta;
	}
	deviation *= 1.0f / inTriangles.Count();

	// Calculate split plane
	uint dimension = deviation.GetHighestComponentIndex();
	float split = mean[dimension];

	return SplitInternal(inTriangles, dimension, split, outLeft, outRight);
}




TriangleSplitterBinning::TriangleSplitterBinning(const VertexList &inVertices, const IndexedTriangleList &inTriangles, uint inMinNumBins, uint inMaxNumBins, uint inNumTrianglesPerBin) :
	TriangleSplitter(inVertices, inTriangles),
	mMinNumBins(inMinNumBins),
	mMaxNumBins(inMaxNumBins),
	mNumTrianglesPerBin(inNumTrianglesPerBin)
{
	mBins.resize(mMaxNumBins * 3); // mMaxNumBins per dimension
}

bool TriangleSplitterBinning::Split(const Range &inTriangles, Range &outLeft, Range &outRight)
{
	const uint *begin = mSortedTriangleIdx.data() + inTriangles.mBegin;
	const uint *end = mSortedTriangleIdx.data() + inTriangles.mEnd;

	// Calculate bounds for this range
	AABox centroid_bounds;
	for (const uint *t = begin; t < end; ++t)
		centroid_bounds.Encapsulate(Vec3::sLoadFloat3Unsafe(mCentroids[*t]));

	// Convert bounds to min coordinate and size
	// Prevent division by zero if one of the dimensions is zero
	constexpr float cMinSize = 1.0e-5f;
	Vec3 bounds_min = centroid_bounds.mMin;
	Vec3 bounds_size = Vec3::sMax(centroid_bounds.mMax - bounds_min, Vec3::sReplicate(cMinSize));

	float best_cp = FLT_MAX;
	uint best_dim = 0xffffffff;
	float best_split = 0;

	// Bin in all dimensions
	uint num_bins = Clamp(inTriangles.Count() / mNumTrianglesPerBin, mMinNumBins, mMaxNumBins);

	// Initialize bins
	for (uint dim = 0; dim < 3; ++dim)
	{
		// Get bounding box size for this dimension
		float bounds_min_dim = bounds_min[dim];
		float bounds_size_dim = bounds_size[dim];

		// Get the bins for this dimension
		Bin *bins_dim = &mBins[num_bins * dim];

		for (uint b = 0; b < num_bins; ++b)
		{
			Bin &bin = bins_dim[b];
			bin.mBounds.SetEmpty();
			bin.mMinCentroid = bounds_min_dim + bounds_size_dim * (b + 1) / num_bins;
			bin.mNumTriangles = 0;
		}
	}

	// Bin all triangles in all dimensions at once
	for (const uint *t = begin; t < end; ++t)
	{
		Vec3 centroid_pos = Vec3::sLoadFloat3Unsafe(mCentroids[*t]);

		AABox triangle_bounds = AABox::sFromTriangle(mVertices, mTriangles[*t]);

		Vec3 bin_no_f = (centroid_pos - bounds_min) / bounds_size * float(num_bins);
		UVec4 bin_no = UVec4::sMin(bin_no_f.ToInt(), UVec4::sReplicate(num_bins - 1));

		for (uint dim = 0; dim < 3; ++dim)
		{
			// Select bin
			Bin &bin = mBins[num_bins * dim + bin_no[dim]];

			// Accumulate triangle in bin
			bin.mBounds.Encapsulate(triangle_bounds);
			bin.mMinCentroid = min(bin.mMinCentroid, centroid_pos[dim]);
			bin.mNumTriangles++;
		}
	}

	for (uint dim = 0; dim < 3; ++dim)
	{
		// Skip axis if too small
		if (bounds_size[dim] <= cMinSize)
			continue;

		// Get the bins for this dimension
		Bin *bins_dim = &mBins[num_bins * dim];

		// Calculate totals left to right
		AABox prev_bounds;
		int prev_triangles = 0;
		for (uint b = 0; b < num_bins; ++b)
		{
			Bin &bin = bins_dim[b];
			bin.mBoundsAccumulatedLeft = prev_bounds; // Don't include this node as we'll take a split on the left side of the bin
			bin.mNumTrianglesAccumulatedLeft = prev_triangles;
			prev_bounds.Encapsulate(bin.mBounds);
			prev_triangles += bin.mNumTriangles;
		}

		// Calculate totals right to left
		prev_bounds.SetEmpty();
		prev_triangles = 0;
		for (int b = num_bins - 1; b >= 0; --b)
		{
			Bin &bin = bins_dim[b];
			prev_bounds.Encapsulate(bin.mBounds);
			prev_triangles += bin.mNumTriangles;
			bin.mBoundsAccumulatedRight = prev_bounds;
			bin.mNumTrianglesAccumulatedRight = prev_triangles;
		}

		// Get best splitting plane
		for (uint b = 1; b < num_bins; ++b) // Start at 1 since selecting bin 0 would result in everything ending up on the right side
		{
			// Calculate surface area heuristic and see if it is better than the current best
			const Bin &bin = bins_dim[b];
			float cp = bin.mBoundsAccumulatedLeft.GetSurfaceArea() * bin.mNumTrianglesAccumulatedLeft + bin.mBoundsAccumulatedRight.GetSurfaceArea() * bin.mNumTrianglesAccumulatedRight;
			if (cp < best_cp)
			{
				best_cp = cp;
				best_dim = dim;
				best_split = bin.mMinCentroid;
			}
		}
	}

	// No split found?
	if (best_dim == 0xffffffff)
		return false;

	return SplitInternal(inTriangles, best_dim, best_split, outLeft, outRight);
}


MOSS_SUPRESS_WARNINGS_END

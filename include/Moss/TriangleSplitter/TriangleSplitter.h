// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Geometry/Triangle.h>
#include <Moss/Core/NonCopyable.h>
#include <Moss/Variants/AABB3.h>
#include <Moss/Variants/AABB2.h>

MOSS_SUPPRESS_WARNINGS_BEGIN

/// A class that splits a triangle list into two parts for building a tree
class MOSS_API TriangleSplitter : public NonCopyable {
public:
	/// Constructor
	TriangleSplitter(const VertexList &inVertices, const IndexedTriangleList &inTriangles);

	/// Virtual destructor
	virtual						~TriangleSplitter() = default;

	struct Stats {
		const char*			mSplitterName = nullptr;
		int					mLeafSize = 0;
	};

	/// Get stats of splitter
	virtual void				GetStats(Stats &outStats) const = 0;

	/// Helper struct to indicate triangle range before and after the split
	struct Range {
		/// Constructor
		Range() = default;
		Range(uint32_t inBegin, uint32_t inEnd) : mBegin(inBegin), mEnd(inEnd) { }

		/// Get number of triangles in range
		uint32_t Count() const {
			return mEnd - mBegin;
		}

		/// Start and end index (end = 1 beyond end)
		uint32_t	mBegin;
		uint32_t	mEnd;
	};

	/// Range of triangles to start with
	Range						GetInitialRange() const
	{
		return Range(0, (uint32_t)mSortedTriangleIdx.size());
	}

	/// Split triangles into two groups left and right, returns false if no split could be made
	/// @param inTriangles The range of triangles (in mSortedTriangleIdx) to process
	/// @param outLeft On return this will contain the ranges for the left subpart. mSortedTriangleIdx may have been shuffled.
	/// @param outRight On return this will contain the ranges for the right subpart. mSortedTriangleIdx may have been shuffled.
	/// @return Returns true when a split was found
	virtual bool Split(const Range &inTriangles, Range &outLeft, Range &outRight) = 0;

	/// Get the list of vertices
	const VertexList& GetVertices() const { return mVertices; }

	/// Get triangle by index
	const IndexedTriangle& GetTriangle(uint32_t inIdx) const { return mTriangles[mSortedTriangleIdx[inIdx]]; }

protected:
	/// Helper function to split triangles based on dimension and split value
	bool SplitInternal(const Range &inTriangles, uint32_t inDimension, float inSplit, Range &outLeft, Range &outRight);

	const VertexList&			mVertices;				// Vertices of the indexed triangles
	const IndexedTriangleList&	mTriangles;				// Unsorted triangles
	TArray<Float3>				mCentroids;				// Unsorted centroids of triangles
	TArray<uint32_t>					mSortedTriangleIdx;	// Indices to sort triangles
};




/// Binning splitter approach taken from: Realtime Ray Tracing on GPU with BVH-based Packet Traversal by Johannes Gunther et al.
class MOSS_API TriangleSplitterBinning : public TriangleSplitter {
public:
	/// Constructor
							TriangleSplitterBinning(const VertexList &inVertices, const IndexedTriangleList &inTriangles, uint32_t inMinNumBins = 8, uint32_t inMaxNumBins = 128, uint32_t inNumTrianglesPerBin = 6);

	// See TriangleSplitter::GetStats
	virtual void GetStats(Stats &outStats) const override { outStats.mSplitterName = "TriangleSplitterBinning"; }

	// See TriangleSplitter::Split
	virtual bool Split(const Range &inTriangles, Range &outLeft, Range &outRight) override;

private:
	// Configuration
	const uint32_t				mMinNumBins;
	const uint32_t				mMaxNumBins;
	const uint32_t				mNumTrianglesPerBin;

	struct Bin {
		// Properties of this bin
		AABox				mBounds;
		float				mMinCentroid;
		uint32_t				mNumTriangles;

		// Accumulated data from left most / right most bin to current (including this bin)
		AABox				mBoundsAccumulatedLeft;
		AABox				mBoundsAccumulatedRight;
		uint32_t				mNumTrianglesAccumulatedLeft;
		uint32_t				mNumTrianglesAccumulatedRight;
	};

	// Scratch area to store the bins
	TArray<Bin>				mBins;
};



/// Splitter using mean of axis with biggest centroid deviation
class MOSS_API TriangleSplitterMean : public TriangleSplitter {
public:
	/// Constructor
	TriangleSplitterMean(const VertexList &inVertices, const IndexedTriangleList &inTriangles);

	// See TriangleSplitter::GetStats
	virtual void GetStats(Stats &outStats) const override { outStats.mSplitterName = "TriangleSplitterMean"; }

	// See TriangleSplitter::Split
	virtual bool Split(const Range &inTriangles, Range &outLeft, Range &outRight) override;
};

MOSS_SUPPRESS_WARNINGS_END

// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/TriangleSplitter/TriangleSplitter.h>
#include <Moss/Variants/AABB3.h>
#include <Moss/Core/NonCopyable.h>

MOSS_SUPPRESS_WARNINGS_BEGIN

struct AABBTreeBuilderStats {
	///@name Splitter stats
	TriangleSplitter::Stats	mSplitterStats;							// Stats returned by the triangle splitter algorithm

	///@name Tree structure
	float					mSAHCost = 0.0f;						// Surface Area Heuristic cost of this tree
	int						mMinDepth = 0;							// Minimal depth of tree (number of nodes)
	int						mMaxDepth = 0;							// Maximum depth of tree (number of nodes)
	int						mNodeCount = 0;							// Number of nodes in the tree
	int						mLeafNodeCount = 0;						// Number of leaf nodes (that contain triangles)

	///@name Configured stats
	int						mMaxTrianglesPerLeaf = 0;				// Configured max triangles per leaf

	///@name Actual stats
	int						mTreeMinTrianglesPerLeaf = 0;			// Minimal amount of triangles in a leaf
	int						mTreeMaxTrianglesPerLeaf = 0;			// Maximal amount of triangles in a leaf
	float					mTreeAvgTrianglesPerLeaf = 0.0f;		// Average amount of triangles in leaf nodes
};

/// Helper class to build an AABB tree
class MOSS_API AABBTreeBuilder {
public:
	/// A node in the tree, contains the AABox for the tree and any child nodes or triangles
	class Node {
	public:
		MOSS_OVERRIDE_NEW_DELETE

		/// Indicates that there is no child
		static constexpr uint32_t cInvalidNodeIndex = ~uint32_t(0);

		/// Get number of triangles in this node
		inline uint32_t GetTriangleCount() const				{ return mNumTriangles; }

		/// Check if this node has any children
		inline bool HasChildren() const						{ return mChild[0] != cInvalidNodeIndex || mChild[1] != cInvalidNodeIndex; }

		/// Min depth of tree
		uint32_t GetMinDepth(const TArray<Node> &inNodes) const;

		/// Max depth of tree
		uint32_t GetMaxDepth(const TArray<Node> &inNodes) const;

		/// Number of nodes in tree
		uint32_t GetNodeCount(const TArray<Node> &inNodes) const;

		/// Number of leaf nodes in tree
		uint32_t GetLeafNodeCount(const TArray<Node> &inNodes) const;

		/// Get triangle count in tree
		uint32_t GetTriangleCountInTree(const TArray<Node> &inNodes) const;

		/// Calculate min and max triangles per node
		void GetTriangleCountPerNode(const TArray<Node> &inNodes, float &outAverage, uint32_t &outMin, uint32_t &outMax) const;

		/// Calculate the total cost of the tree using the surface area heuristic
		float CalculateSAHCost(const TArray<Node> &inNodes, float inCostTraversal, float inCostLeaf) const;

		/// Recursively get children (breadth first) to get in total inN children (or less if there are no more)
		void GetNChildren(const TArray<Node> &inNodes, uint32_t inN, TArray<const Node *> &outChildren) const;

		/// Bounding box
		AABox mBounds;

		/// Triangles (if no child nodes)
		uint32_t mTrianglesBegin; // Index into mTriangles
		uint32_t mNumTriangles = 0;

		/// Child node indices (if no triangles)
		uint32_t mChild[2] = { cInvalidNodeIndex, cInvalidNodeIndex };

	private:
		friend class AABBTreeBuilder;

		/// Recursive helper function to calculate cost of the tree
		float CalculateSAHCostInternal(const TArray<Node> &inNodes, float inCostTraversalDivSurfaceArea, float inCostLeafDivSurfaceArea) const;

		/// Recursive helper function to calculate min and max triangles per node
		void GetTriangleCountPerNodeInternal(const TArray<Node> &inNodes, float &outAverage, uint32_t &outAverageDivisor, uint32_t &outMin, uint32_t &outMax) const;
	};

	/// Constructor
	AABBTreeBuilder(TriangleSplitter &inSplitter, uint32_t inMaxTrianglesPerLeaf = 16);

	/// Recursively build tree, returns the root node of the tree
	Node*					Build(AABBTreeBuilderStats &outStats);

	/// Get all nodes
	const TArray<Node>& GetNodes() const { return mNodes; }

	/// Get all triangles
	const TArray<IndexedTriangle>& GetTriangles() const { return mTriangles; }

private:
	uint32_t					BuildInternal(const TriangleSplitter::Range &inTriangles);

	TriangleSplitter &		mTriangleSplitter;
	const uint32_t				mMaxTrianglesPerLeaf;
	TArray<Node>				mNodes;
	TArray<IndexedTriangle>	mTriangles;
};

MOSS_SUPPRESS_WARNINGS_END

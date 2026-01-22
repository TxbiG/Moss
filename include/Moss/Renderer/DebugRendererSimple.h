// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2024 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#ifndef MOSS_DEBUG_RENDERER

#include <Moss/Renderer/DebugRenderer.h>

MOSS_SUPRESS_WARNINGS_BEGIN

/// Inherit from this class to simplify implementing a debug renderer, start with this implementation:
///
///		class MyDebugRenderer : public MOSS::DebugRendererSimple
///		{
///		public:
///			virtual void DrawLine(MOSS::RVec3Arg inFrom, MOSS::RVec3Arg inTo, MOSS::ColorArg inColor) override
///			{
///				// Implement
///			}
///
///			virtual void DrawTriangle(MOSS::RVec3Arg inV1, MOSS::RVec3Arg inV2, MOSS::RVec3Arg inV3, MOSS::ColorArg inColor, ECastShadow inCastShadow) override
///			{
///				// Implement
///			}
///
///			virtual void DrawText3D(MOSS::RVec3Arg inPosition, const string_view &inString, MOSS::ColorArg inColor, float inHeight) override
///			{
///				// Implement
///			}
///		};
///
/// Note that this class is meant to be a quick start for implementing a debug renderer, it is not the most efficient way to implement a debug renderer.
class MOSS_DEBUG_RENDERER_EXPORT DebugRendererSimple : public DebugRenderer
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
								DebugRendererSimple();

	/// Should be called every frame by the application to provide the camera position.
	/// This is used to determine the correct LOD for rendering.
	void						SetCameraPos(RVec3Arg inCameraPos)
	{
		mCameraPos = inCameraPos;
		mCameraPosSet = true;
	}

	/// Fallback implementation that uses DrawLine to draw a triangle (override this if you have a version that renders solid triangles)
	virtual void				DrawTriangle(RVec3Arg inV1, RVec3Arg inV2, RVec3Arg inV3, ColorArg inColor, ECastShadow inCastShadow) override
	{
		DrawLine(inV1, inV2, inColor);
		DrawLine(inV2, inV3, inColor);
		DrawLine(inV3, inV1, inColor);
	}

protected:
	/// Implementation of DebugRenderer interface
	virtual Batch				CreateTriangleBatch(const Triangle *inTriangles, int inTriangleCount) override;
	virtual Batch				CreateTriangleBatch(const Vertex *inVertices, int inVertexCount, const uint32 *inIndices, int inIndexCount) override;
	virtual void				DrawGeometry(RMat44Arg inModelMatrix, const AABox &inWorldSpaceBounds, float inLODScaleSq, ColorArg inModelColor, const GeometryRef &inGeometry, ECullMode inCullMode, ECastShadow inCastShadow, EDrawMode inDrawMode) override;

private:
	/// Implementation specific batch object
	class BatchImpl : public RefTargetVirtual
	{
	public:
		MOSS_OVERRIDE_NEW_DELETE

		virtual void			AddRef() override			{ ++mRefCount; }
		virtual void			Release() override			{ if (--mRefCount == 0) delete this; }

		TArray<Triangle>			mTriangles;

	private:
		atomic<uint32>			mRefCount = 0;
	};

	/// Last provided camera position
	RVec3						mCameraPos;
	bool						mCameraPosSet = false;
};

MOSS_SUPRESS_WARNINGS_END
#endif // MOSS_DEBUG_RENDERER
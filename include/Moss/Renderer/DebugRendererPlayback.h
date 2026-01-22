// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#ifndef MOSS_DEBUG_RENDERER

#include <Moss/Renderer/DebugRendererRecorder.h>
#include <Moss/Core/StreamIn.h>
#include <Moss/Core/Variants/TMap.h>

MOSS_SUPRESS_WARNINGS_BEGIN

/// Class that can read a recorded stream from DebugRendererRecorder and plays it back trough a DebugRenderer
class MOSS_DEBUG_RENDERER_EXPORT DebugRendererPlayback
{
public:
	/// Constructor
										DebugRendererPlayback(DebugRenderer &inRenderer) : mRenderer(inRenderer) { }

	/// Parse a stream of frames
	void								Parse(StreamIn &inStream);

	/// Get the number of parsed frames
	uint								GetNumFrames() const				{ return (uint)mFrames.size(); }

	/// Draw a frame
	void								DrawFrame(uint inFrameNumber) const;

private:
	/// The debug renderer we're using to do the actual rendering
	DebugRenderer &						mRenderer;

	/// Mapping of ID to batch
	TMap<uint32, DebugRenderer::Batch> mBatches;

	/// Mapping of ID to geometry
	TMap<uint32, DebugRenderer::GeometryRef> mGeometries;

	/// The list of parsed frames
	using Frame = DebugRendererRecorder::Frame;
	TArray<Frame>						mFrames;
};

MOSS_SUPRESS_WARNINGS_END
#endif // MOSS_DEBUG_RENDERER
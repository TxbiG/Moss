// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2025 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Renderer/PixelShader.h>
#include <Renderer/VertexShader.h>

#include <MetalKit/MetalKit.h>

/// Pixel shader handle for Metal
class PixelShaderMTL : public PixelShader
{
public:
	PixelShaderMTL(id<MTLFunction> inFunction) : mFunction(inFunction) { }
	virtual	~PixelShaderMTL() override { [mFunction release]; }

	/// Access to the function
	id<MTLFunction>	GetFunction() const	{ return mFunction; }

private:
	id<MTLFunction>	mFunction;
};

/// Vertex shader handle for Metal
class VertexShaderMTL : public VertexShader
{
public:
	VertexShaderMTL(id<MTLFunction> inFunction) : mFunction(inFunction) { }
	virtual	~VertexShaderMTL() override { [mFunction release]; }

	/// Access to the function
	id<MTLFunction>	GetFunction() const	{ return mFunction; }

private:
	id<MTLFunction>	mFunction;
};


/// Compute shader handle for Metal
class ComputeShaderMTL : public ComputeShader
{
public:
	ComputeShaderMTL(id<MTLFunction> inFunction) : mFunction(inFunction) { }
	virtual	~ComputeShaderMTL() override { [mFunction release]; }

	/// Access to the function
	id<MTLFunction>	GetFunction() const	{ return mFunction; }

private:
	id<MTLFunction>	mFunction;
};

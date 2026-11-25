// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2024 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Renderer/PixelShader.h>
#include <Renderer/VertexShader.h>

/// Pixel shader handle for DirectX
class PixelShaderDX12 : public PixelShader {
public:
	PixelShaderDX12(ComPtr<ID3DBlob> inShader) : mShader(inShader) { }

	ComPtr<ID3DBlob>		mShader;	// The compiled shader
};


/// Vertex shader handle for DirectX
class VertexShaderDX12 : public VertexShader {
public:
	VertexShaderDX12(ComPtr<ID3DBlob> inShader) : mShader(inShader) { }

	ComPtr<ID3DBlob>		mShader;	// The compiled shader
};


class ComputeShaderDX12 : public VertexShader {
public:
	ComputeShaderDX12(ComPtr<ID3DBlob> inShader) : mShader(inShader) { }

	ComPtr<ID3DBlob>		mShader;	// The compiled shader
};

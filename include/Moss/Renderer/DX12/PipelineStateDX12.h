// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2024 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Renderer/PipelineState.h>

class RendererDX12;
class VertexShaderDX12;
class PixelShaderDX12;

/// DirectX 12 pipeline state object
class PipelineStateDX12 : public PipelineState
{
public:
	/// Constructor
	PipelineStateDX12(RendererDX12 *inRenderer, const VertexShaderDX12 *inVertexShader, const EInputDescription *inInputDescription, uint inInputDescriptionCount, 
		const PixelShaderDX12 *inPixelShader, EDrawPass inDrawPass, EFillMode inFillMode, ETopology inTopology, EDepthTest inDepthTest, EBlendMode inBlendMode, ECullMode inCullMode);
	virtual	~PipelineStateDX12() override;

	/// Make this pipeline state active (any primitives rendered after this will use this state)
	virtual void bind() override;
	virtual void unbind() override { return; }

	virtual void SetUniform(const char* name, const float value) override { SetUniformInternal(name, &value, sizeof(float)); }
    virtual void SetUniform(const char* name, const int value) override { SetUniformInternal(name, &value, sizeof(int)); }
    
    virtual void SetUniform(const char* name, const Color& value) override { SetUniformInternal(name, &value, sizeof(Color)); }

    // Vector Uniforms
    virtual void SetUniform(const char* name, const Float2& value) override { SetUniforms(name, &value, sizeof(Float2));}
    virtual void SetUniform(const char* name, const Int2& value) override;
    virtual void SetUniform(const char* name, const Float3& value) override { SetUniforms(name, &value, sizeof(Float3)); }
    virtual void SetUniform(const char* name, const Int3& value) override;
    virtual void SetUniform(const char* name, const Float4& value) override { SetUniforms(name, &value, sizeof(Float4)); }
    virtual void SetUniform(const char* name, const Int4& value) override;

    // Matrix Uniforms
    //virtual void SetUniform(const char* name, const Mat2& value) override;
    //virtual void SetUniform(const char* name, const Mat2x3& value) override;
    //virtual void SetUniform(const char* name, const Mat2x4& value) override;
    //virtual void SetUniform(const char* name, const Mat3& value) override;
    //virtual void SetUniform(const char* name, const Mat3x2& value) override;
    //virtual void SetUniform(const char* name, const Mat3x4& value) override;
    virtual void SetUniform(const char* name, const Mat44& value) override { SetUniforms(name, &value, sizeof(Mat44)); }
    //virtual void SetUniform(const char* name, const Mat4x2& value) override;
    //virtual void SetUniform(const char *name, const Mat4x3& value) override;

    // Unsigned int
    //virtual void SetUniform(const char* name, unsigned int value) override;
    //virtual void SetUniform(const char* name, unsigned int value0, unsigned int value1);
    //virtual void SetUniform(const char* name, unsigned int value0, unsigned int value1, unsigned int value2);
    //virtual void SetUniform(const char* name, unsigned int value0, unsigned int value1, unsigned int value2, unsigned int value3);

    // Unsigned Integer Array Uniforms
    //virtual void SetUniform(const char* name, int count, uint32* value) override;
    //virtual void SetUniform(const char* name, int count, uint32* value) override;
    //virtual void SetUniform(const char* name, int count, uint32* value) override;
    //virtual void SetUniform(const char* name, int count, uint32* value) override;

    // Float Array Uniforms
    //virtual void SetUniform(const char* name, int count, const float* value) override;
    //virtual void SetUniform(const char* name, int count, const float* value) override;
    //virtual void SetUniform(const char* name, int count, const float* value) override;
    //virtual void SetUniform(const char* name, int count, const float* value) override;

    // Integer Array Uniforms
    //virtual void SetUniform(const char* name, int count, const int* value) override;
    //virtual void SetUniform(const char* name, int count, const int* value) override;
    //virtual void SetUniform(const char* name, int count, const int* value) override;
    //virtual void SetUniform(const char* name, int count, const int* value) override;

    void SetUniformBlock(const std::string& name, const void* data, size_t size, bool useBuffer = false) override {
        auto& entry = pendingUniforms[name];
        entry.data.assign((uint8_t*)data, (uint8_t*)data + size);
        entry.size = size;
        entry.useBuffer = useBuffer;
    }

    // Flush push constant uniforms (32-bit constants)
    void FlushUniforms(ID3D12GraphicsCommandList* cmdList) {
        for (auto& [name, entry] : pendingUniforms) {
            UINT rootIndex = GetRootIndexFor(name);
            if (!entry.useBuffer) {
                // Push constants (root constants)
                UINT num32BitValues = static_cast<UINT>(entry.size / 4);
                cmdList->SetGraphicsRoot32BitConstants(rootIndex, num32BitValues, entry.data.data(), 0);
            } else {
                // Constant buffer views (CBVs)
                ID3D12Resource* cb = GetOrCreateConstantBuffer(name, entry.size);

                uint8_t* mappedData = nullptr;
                D3D12_RANGE readRange{0, 0};
                cb->Map(0, &readRange, reinterpret_cast<void**>(&mappedData));
                memcpy(mappedData, entry.data.data(), entry.size);
                cb->Unmap(0, nullptr);

                D3D12_GPU_VIRTUAL_ADDRESS gpuAddress = cb->GetGPUVirtualAddress();
                cmdList->SetGraphicsRootConstantBufferView(rootIndex, gpuAddress);
            }
        }
        pendingUniforms.clear();
    }


private:
    void SetUniformInternal(const char* name, const void* data, size_t size) {
        auto it = uniformLayout.find(name);
        if (it == uniformLayout.end()) return;
        const UniformEntry& entry = it->second;

        auto& uniform = pendingUniforms[name];
        uniform.data.resize(size);
        memcpy(uniform.data.data(), data, size);
        uniform.size = size;
        uniform.useBuffer = entry.useBuffer;  // Reflect whether uniform uses CBV or push constant
    }
    struct PendingUniform {
        std::vector<uint8_t> data;
        size_t size = 0;
        bool useBuffer = false; // true = CBV, false = push constant
    };

    // Map uniform name -> reflection info (offset, size, useBuffer)
    std::unordered_map<std::string, UniformEntry> uniformLayout;
    // Pending uniforms waiting to be flushed to GPU
    std::unordered_map<std::string, PendingUniform> pendingUniforms;

	RendererDX12 *						mRenderer;
	ComPtr<ID3D12PipelineState>			mPSO;
};

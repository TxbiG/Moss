// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2025 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Renderer/PipelineState.h>
#include <Renderer/MTL/VertexShaderMTL.h>
#include <Renderer/MTL/PixelShaderMTL.h>

class RendererMTL;

/// Metal pipeline state object
class PipelineStateMTL : public PipelineState
{
public:
	/// Constructor
										PipelineStateMTL(RendererMTL *inRenderer, const VertexShaderMTL *inVertexShader, const EInputDescription *inInputDescription, uint inInputDescriptionCount, const PixelShaderMTL *inPixelShader, EDrawPass inDrawPass, EFillMode inFillMode, ETopology inTopology, EDepthTest inDepthTest, EBlendMode inBlendMode, ECullMode inCullMode);
	virtual								~PipelineStateMTL() override;

	/// Make this pipeline state active (any primitives rendered after this will use this state)
	virtual void bind() override;
	virtual void unbind() override { return; };

	virtual void SetUniform(const char* name, const float value) override;
    virtual void SetUniform(const char* name, const int value) override;

    virtual void SetUniform(const char* name, const Color& value) override;

    // Vector Uniforms
    virtual void SetUniform(const char* name, const Float2& value) override;
    virtual void SetUniform(const char* name, const Int2& value) override;
    virtual void SetUniform(const char* name, const Float3& value) override;
    virtual void SetUniform(const char* name, const Int3& value) override;
    virtual void SetUniform(const char* name, const Float4& value) override;
    virtual void SetUniform(const char* name, const Int4& value) override;

    // Matrix Uniforms
    //virtual void SetUniform(const char* name, const Mat2& value) override;
    //virtual void SetUniform(const char* name, const Mat2x3& value) override;
    //virtual void SetUniform(const char* name, const Mat2x4& value) override;
    //virtual void SetUniform(const char* name, const Mat3& value) override;
    //virtual void SetUniform(const char* name, const Mat3x2& value) override;
    //virtual void SetUniform(const char* name, const Mat3x4& value) override;
    virtual void SetUniform(const char* name, const Mat44& value) override;
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


    void SetUniformBlock(const std::string& name, const void* data, size_t size, bool useBuffer = false) {
        auto& entry = pending.items[name];
        entry.data.assign((uint8_t*)data, (uint8_t*)data + size);
        entry.size = size;
        entry.useBuffer = useBuffer;
    }

    void FlushUniforms(id<MTLRenderCommandEncoder> encoder) {
        for (auto& [name, entry] : pending.items) {
            NSUInteger index = GetUniformIndex(name); // reflection-based or manual

            [encoder setVertexBytes:entry.data.data() length:entry.size atIndex:index];
            [encoder setFragmentBytes:entry.data.data() length:entry.size atIndex:index];
        }
        pending.items.clear();
    }


    void SetUniformBuffer(const std::string& blockName, const void* data, size_t size) {
        auto& entry = pending.items[name];
        entry.data.assign((uint8_t*)data, (uint8_t*)data + size);
        entry.size = size;
        entry.useBuffer = useBuffer;
    }


    void FlushUniformBuffers(id<MTLRenderCommandEncoder> encoder) {
        for (auto& [name, entry] : pending.items) {
            NSUInteger index = GetUniformIndex(name);
            id<MTLBuffer> buffer = GetOrCreateBuffer(name, entry.size);
            memcpy([buffer contents], entry.data.data(), entry.size);
            [encoder setVertexBuffer:buffer offset:0 atIndex:index];
            [encoder setFragmentBuffer:buffer offset:0 atIndex:index];
        }
        pending.items.clear();
    }

private:
    void SetUniform(const char* name, const void* data, size_t size) override {
        auto it = uniformLayout.find(name);
        if (it == uniformLayout.end()) return;
        const UniformEntry& entry = it->second;

        auto& buffer = pending[entry.binding];
        if (buffer.size() < entry.offset + size)
            buffer.resize(entry.offset + size);

        memcpy(buffer.data() + entry.offset, data, size);
    }
    
    struct PendingUniform {
        std::vector<uint8_t> data;
        size_t size;
        bool useBuffer; // false = setBytes, true = MTLBuffer
    };
    struct PendingUniforms { std::unordered_map<std::string, PendingUniform> items; };

    PendingUniforms  blocks;
    PendingUniforms  buffers;

	RendererMTL *						mRenderer;
	RefConst<VertexShaderMTL>			mVertexShader;
	RefConst<PixelShaderMTL>			mPixelShader;
	id<MTLRenderPipelineState> 			mPipelineState;
	id<MTLDepthStencilState>			mDepthState;
	MTLCullMode							mCullMode;
	MTLTriangleFillMode					mFillMode;
};

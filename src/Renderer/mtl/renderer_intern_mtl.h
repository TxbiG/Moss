#ifndef MOSS_RENDERER_INTERNAL_H
#define MOSS_RENDERER_INTERNAL_H
#include <Moss/Moss_Renderer.h>

struct Moss_Renderer {
    MTKView*                     mView;
    id<MTLDevice>                mDevice;
    id<MTLLibrary>               mShaderLibrary;
    id<MTLCommandQueue>          mCommandQueue;
    id<MTLCommandBuffer>         mCommandBuffer;
    id<MTLRenderCommandEncoder>  mRenderEncoder;
    MTLRenderPassDescriptor*     mCurrentPass;

    // Swapchain / framebuffers
    id<CAMetalDrawable>          mCurrentDrawable;
    Ref<TextureMTL>              mMainColorTexture;
    Ref<TextureMTL>              mDepthTexture;
    Ref<TextureMTL>              mShadowMap;

    // Post-processing / FSR
    bool enableFSR1;
    float fsrSharpness;
    Ref<TextureMTL> fsrIntermediateTex;
    id<MTLRenderPipelineState> fsrEASUPipeline;
    id<MTLRenderPipelineState> fsrRCASPipeline;

    Ref<TextureMTL> fsrHistoryTex[2];
    int fsrHistoryIndex;
    Ref<TextureMTL> fsrMotionVectorsTex;
    Ref<TextureMTL> fsrDepthTex;
    id<MTLComputePipelineState> fsr2Pipeline;

    // Multi-frame
    uint32_t frameIndex;
    static constexpr uint32_t MaxFramesInFlight = 2;
    id<MTLCommandBuffer> commandBuffers[MaxFramesInFlight];

    // Render state
    bool enableDepthTest;
    bool enableAlphaBlending;
    bool enableCullFace;
    AntiAliasing aaMode;
    int msaaSamples;
    Ref<TextureMTL> msaaColorTexture;
    Ref<TextureMTL> msaaResolveTexture;


#if defined(MOSS_RENDERER_FORWARD)
    bool enableFSR2;
    Ref<TextureMTL> lightIndexTexture;
    id<MTLBuffer>  lightGridBuffer;
    uint32_t       numLights;
#endif
};

struct Moss_GPUBuffer {
    Moss_GPUDevice* device;

    id<MTLBuffer> buffer;

    uint64_t size;
    MTLResourceOptions options;

    void* mapped; // CPU pointer if shared
};

struct Moss_GPUDevice {
    GraphicsBackend backend;   // METAL

    id<MTLDevice> device;
    id<MTLCommandQueue> commandQueue;

    uint32_t frameIndex;
    uint32_t framesInFlight;

    bool supportsCompute;
    bool supportsTileShaders;
    bool supportsMSAA;

    uint32_t maxThreadsPerThreadgroup;
};

struct Moss_Shader {
    Moss_GPUDevice* device;

    id<MTLFunction> function;
    Moss_ShaderStage stage; // Vertex / Fragment / Compute

    const char* entryPoint;
};

struct Moss_PipelineState {
    Moss_GPUDevice* device;

    id<MTLRenderPipelineState> renderPSO;
    id<MTLComputePipelineState> computePSO;

    MTLDepthStencilState* depthStencil;

    bool blending;
    bool depthTest;
    bool cullFace;

    MTLCullMode cullMode;
    MTLWinding frontFace;
};

struct Moss_Framebuffer {
    Moss_GPUDevice* device;

    MTLRenderPassDescriptor* passDescriptor;

    Moss_TextureView* colorAttachments[8];
    uint32_t colorCount;

    Moss_TextureView* depthAttachment;

    uint32_t width;
    uint32_t height;
};

#define MOSS_MAX_RESOURCES 16

struct Moss_ResourceSet {
    Moss_GPUDevice* device;

    uint32_t bufferCount;
    Moss_GPUBuffer* buffers[MOSS_MAX_RESOURCES];
    uint32_t bufferSlots[MOSS_MAX_RESOURCES];

    uint32_t textureCount;
    Moss_TextureView* textures[MOSS_MAX_RESOURCES];
    uint32_t textureSlots[MOSS_MAX_RESOURCES];

    uint32_t samplerCount;
    Moss_GPUSampler* samplers[MOSS_MAX_RESOURCES];
    uint32_t samplerSlots[MOSS_MAX_RESOURCES];
};

struct Moss_TextureView {
    Moss_GPUDevice* device;

    id<MTLTexture> texture;

    uint32_t width;
    uint32_t height;
    uint32_t mipLevels;

    MTLPixelFormat format;
};

struct Moss_GPUSampler {
    Moss_GPUDevice* device;

    id<MTLSamplerState> sampler;

    MTLSamplerMinMagFilter minFilter;
    MTLSamplerMinMagFilter magFilter;

    MTLSamplerAddressMode addressU;
    MTLSamplerAddressMode addressV;
    MTLSamplerAddressMode addressW;
};


struct Moss_RenderGraph {
    Moss_GPUDevice* device;

    Moss_RGPass** passes;
    uint32_t passCount;
};

struct Moss_RGPass {
    const char* name;

    Moss_Framebuffer* framebuffer;

    void (*execute)(Moss_GPUCommandBuffer*, void* userData);
};

struct Moss_RGTexture {
    Moss_TextureView* texture;
};

struct Moss_RGBuffer {
    Moss_GPUBuffer* buffer;
};

struct Moss_GPUCommandBuffer {
    Moss_GPUDevice* device;

    id<MTLCommandBuffer> commandBuffer;
    id<MTLRenderCommandEncoder> renderEncoder;
    id<MTLComputeCommandEncoder> computeEncoder;

    bool recording;
};


#endif // MOSS_RENDERER_INTERNAL_H
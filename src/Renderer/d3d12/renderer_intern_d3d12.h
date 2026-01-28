#ifndef MOSS_RENDERER_INTERNAL_H
#define MOSS_RENDERER_INTERNAL_H
#include <Moss/Moss_Renderer.h>

#define MOSS_MAX_COLOR_ATTACHMENTS 8
#define MOSS_MAX_BINDINGS 16

struct Moss_Renderer {
    Moss_Window* mWindow;

    // Core DX12
    ComPtr<IDXGIFactory4>           mDXGIFactory;
    ComPtr<ID3D12Device>            mDevice;

    // Descriptor heaps
    DescriptorHeapDX12               mRTVHeap;     // Render target views
    DescriptorHeapDX12               mDSVHeap;     // Depth stencil view
    DescriptorHeapDX12               mSRVHeap;     // Shader resource views

    // Swapchain + render targets
    ComPtr<IDXGISwapChain3>         mSwapChain;
    ComPtr<ID3D12Resource>          mRenderTargets[cFrameCount];
    D3D12_CPU_DESCRIPTOR_HANDLE      mRenderTargetViews[cFrameCount];
    ComPtr<ID3D12Resource>          mDepthStencilBuffer;
    D3D12_CPU_DESCRIPTOR_HANDLE      mDepthStencilView;

    // Command objects
    ComPtr<ID3D12CommandAllocator>  mCommandAllocators[cFrameCount];
    ComPtr<ID3D12GraphicsCommandList> mCommandList;
    ComPtr<ID3D12CommandQueue>      mCommandQueue;
    CommandQueueDX12                 mUploadQueue; // staging uploads

    // Constant buffers per frame
    std::unique_ptr<ConstantBufferDX12> mVertexShaderConstantBufferProjection[cFrameCount];
    std::unique_ptr<ConstantBufferDX12> mVertexShaderConstantBufferOrtho[cFrameCount];
    std::unique_ptr<ConstantBufferDX12> mPixelShaderConstantBuffer[cFrameCount];

    // Synchronization
    HANDLE                           mFenceEvent;
    ComPtr<ID3D12Fence>              mFence;
    UINT64                            mFenceValues[cFrameCount] = {};
    uint32_t                          mFrameIndex = 0;

    // Post-processing / FSR
    bool                              enableFSR1 = false;
    float                             fsrSharpness = 1.0f;
    ComPtr<ID3D12Resource>            fsrIntermediateTex;
    ComPtr<ID3D12PipelineState>       fsrEASUPSO;
    ComPtr<ID3D12PipelineState>       fsrRCASPSO;

    ComPtr<ID3D12Resource>            fsrHistoryTex[2];
    ComPtr<ID3D12Resource>            fsrMotionVectorsTex;
    ComPtr<ID3D12Resource>            fsrDepthTex;
    ComPtr<ID3D12PipelineState>       fsr2PSO;
    int                               fsrHistoryIndex = 0;

    // Shadow mapping
    Ref<TextureDX12>                  mShadowMap;
    ComPtr<ID3D12PipelineState>       mShadowPSO;

    // Resource management
    using ResourceCache = UnorderedMap<uint64_t, Array<ComPtr<ID3D12Resource>>>;
    ResourceCache                      mResourceCache;
    ResourceCache                      mDelayCached[cFrameCount];
    TArray<ComPtr<ID3D12Object>>       mDelayReleased[cFrameCount];
    bool                               mIsExiting = false;

    // Renderer state
    bool                               enableDepthTest = true;
    bool                               enableAlphaBlending = true;
    bool                               enableCullFace = false;
    AntiAliasing                       aaMode = AntiAliasing::None;
    int                                msaaSamples = 4;

#if defined(MOSS_RENDERER_FORWARD)
    // Forward+ specific
    bool                               enableFSR2 = false;
    Ref<TextureDX12>                   lightIndexTexture;
    ComPtr<ID3D12Resource>             lightGridBuffer;
    uint32_t                            numLights = 0;
#endif
};

struct Moss_GPUDevice {
    GraphicsBackend backend; // DIRECTX12

    ComPtr<IDXGIFactory4> factory;
    ComPtr<ID3D12Device> device;

    ComPtr<ID3D12CommandQueue> graphicsQueue;
    ComPtr<ID3D12CommandQueue> computeQueue;
    ComPtr<ID3D12CommandQueue> copyQueue;

    uint32_t framesInFlight;
    uint32_t frameIndex;

    // Descriptor heaps (global)
    DescriptorHeapDX12 rtvHeap;
    DescriptorHeapDX12 dsvHeap;
    DescriptorHeapDX12 srvHeap;

    // Synchronization
    ComPtr<ID3D12Fence> fence;
    HANDLE fenceEvent;
    uint64_t fenceValues[3];
};

struct Moss_GPUDevice {
    GraphicsBackend backend; // DIRECTX12

    ComPtr<IDXGIFactory4> factory;
    ComPtr<ID3D12Device> device;

    ComPtr<ID3D12CommandQueue> graphicsQueue;
    ComPtr<ID3D12CommandQueue> computeQueue;
    ComPtr<ID3D12CommandQueue> copyQueue;

    uint32_t framesInFlight;
    uint32_t frameIndex;

    // Descriptor heaps (global)
    DescriptorHeapDX12 rtvHeap;
    DescriptorHeapDX12 dsvHeap;
    DescriptorHeapDX12 srvHeap;

    // Synchronization
    ComPtr<ID3D12Fence> fence;
    HANDLE fenceEvent;
    uint64_t fenceValues[3];
};

struct Moss_Shader {
    D3D12_SHADER_BYTECODE bytecode;
    D3D12_SHADER_VISIBILITY visibility;
};

struct Moss_PipelineState {
    Moss_GPUDevice* device;

    ComPtr<ID3D12PipelineState> pipelineState;
    ComPtr<ID3D12RootSignature> rootSignature;

    bool depthTest;
    bool depthWrite;
    bool blending;
    bool cullFace;

    D3D12_CULL_MODE cullMode;
    D3D12_FILL_MODE fillMode;
};

struct Moss_Framebuffer {
    Moss_GPUDevice* device;

    Moss_TextureView* colorAttachments[MOSS_MAX_COLOR_ATTACHMENTS];
    uint32_t colorCount;

    Moss_TextureView* depthAttachment;

    uint32_t width;
    uint32_t height;
};

struct Moss_ResourceSet {
    Moss_GPUDevice* device;

    uint32_t bindingCount;

    D3D12_GPU_DESCRIPTOR_HANDLE gpuHandle;
    D3D12_CPU_DESCRIPTOR_HANDLE cpuHandle;
};

struct Moss_TextureView {
    Moss_GPUDevice* device;

    ComPtr<ID3D12Resource> resource;

    DXGI_FORMAT format;
    uint32_t width;
    uint32_t height;
    uint32_t mipLevels;

    D3D12_RESOURCE_STATES currentState;

    D3D12_CPU_DESCRIPTOR_HANDLE srv;
    D3D12_CPU_DESCRIPTOR_HANDLE rtv;
    D3D12_CPU_DESCRIPTOR_HANDLE dsv;
    D3D12_CPU_DESCRIPTOR_HANDLE uav;
};

struct Moss_GPUSampler {
    Moss_GPUDevice* device;

    D3D12_SAMPLER_DESC desc;
    D3D12_CPU_DESCRIPTOR_HANDLE samplerHandle;
};


struct Moss_RenderGraph {
    Moss_GPUDevice* device;

    Moss_RGPass** passes;
    uint32_t passCount;
};

typedef struct Moss_RGPass Moss_RGPass;
struct Moss_RGTexture {
    Moss_TextureView* texture;
};

struct Moss_RGBuffer {
    Moss_GPUBuffer* buffer;
};

struct Moss_GPUCommandBuffer {
    Moss_GPUDevice* device;

    ComPtr<ID3D12CommandAllocator> allocator;
    ComPtr<ID3D12GraphicsCommandList> commandList;

    bool recording;
};




#endif // MOSS_RENDERER_INTERNAL_H
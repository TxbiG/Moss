//                        MIT License
//
//                  Copyright (c) 2026 Toby
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/*!
 * @file Moss_Renderer.h
 * @brief Core rendering interface for the Moss Framework.
 *
 * The renderer module provides the abstraction layer for all 2D and 3D graphics
 * operations. It serves as a unified, API-agnostic rendering backend capable of
 * targeting multiple graphics APIs (Vulkan, OpenGL, OpenGL ES, Metal, and DirectX 12).
 * 
 * ### Renderer Types
 * - Forward+       - Vulkan, DirectX 12, Metal (Modern Gen PC and consoles)
 * - Mobile         - Vulkan, OpenGL ES v2.0 & v3.0, Metal (IOS & Android)
 * - Compatibility  - OpenGL v3.3 (Older Gen PC and consoles)
 * 
 * ### Anti Aliasing
 * - Forward+       - MSAA, SSAA, FXAA, SMAA, SSRL, TAA, FSR2
 * - Mobile         - MSAA, SSAA, FXAA, SMAA, SSRL
 * - Compatibility  - MSAA, SSAA
 * 
 * ### Macros
 * - MOSS_GRAPHICS_OPENGL 	- OpenGL v3.3
 * - MOSS_GRAPHICS_OPENGLES - OpenGL ES v2.0 or v3.0
 * - MOSS_GRAPHICS_VULKAN 	- Vulkan
 * - MOSS_GRAPHICS_DIRECTX 	- DirectX 12
 * - MOSS_GRAPHICS_METAL 	- Metal
 * - MOSS_GRAPHICS_MOLTENVK — Vulkan API abstraction for Metal (cross-platform translation).
 * - MOSS_RENDERER_FORWARD 	- Forward+ Renderer
 * - MOSS_RENDERER_MOBILE 	- Mobile Renderer
 * - MOSS_RENDERER_COMPATI 	- Compatibility Renderer
 * 
 * ### Custom Renderer Support:
 * While Moss provides a default renderer implementation Moss_Renderer, developers
 * can build their own custom renderer using the tools and abstractions provided:
 * - Cross-platform resource management (textures, buffers, framebuffers)
 * - Multi-frame synchronization and command submission
 * - Post-processing and upscaling tools (FSR1 / FSR2)
 * - Shadow mapping and lighting pipelines
 * - Descriptor heaps / render pipelines (Vulkan, DX12, Metal)
 *
 * This allows creating fully customized rendering pipelines while leveraging
 * Moss’s low-level GPU abstractions and helper utilities.
 *
 * ### Planned Features / TODO:
 * - **Multithreaded Render Submission** — Asynchronous job-based rendering pipeline (future optimization).
 * - Reflections and improved GI support.
 * - Recording gameplay or screen capture.
 * - Display video playback within the renderer.
 *
 * The renderer acts as the core visual system of Moss, tightly integrated with
 * physics visualization, UI rendering layers, and scene navigation tools.
 */

#ifndef MOSS_RENDERER_H
#define MOSS_RENDERER_H

#ifdef MOSS_GRAPHICS_VULKAN
#include <vulkan/vulkan.h>
#endif // MOSS_GRAPHICS_VULKAN

#include <Moss/Moss_stdinc.h>
#include <Moss/Moss_Platform.h>

#define MOSS_API		// < Delete this

/* ======================================================
 * Forward Declerations
 * =================================================== */

// GPU Side
typedef struct Moss_GPUBuffer Moss_GPUBuffer;
typedef struct Moss_GPUDevice Moss_GPUDevice;
typedef struct Moss_Shader Moss_Shader;
typedef struct Moss_ResourceSet Moss_ResourceSet;
typedef struct Moss_TextureView Moss_TextureView;
typedef struct Moss_GPUSampler Moss_GPUSampler;

typedef struct Moss_RenderGraph Moss_RenderGraph;
typedef struct Moss_RGPass Moss_RGPass;
typedef struct Moss_RGTexture Moss_RGTexture;
typedef struct Moss_RGBuffer Moss_RGBuffer;
typedef struct Moss_GPUCommandBuffer Moss_GPUCommandBuffer;

typedef uint32_t Moss_BindlessHandle;


// Renderer side
typedef struct Moss_Renderer Moss_Renderer;
typedef struct Moss_PipelineState Moss_PipelineState;
typedef struct Moss_Framebuffer Moss_Framebuffer;

typedef uint8_t Moss_CullMask;
typedef uint8_t Moss_LightMask;
typedef uint8_t Moss_VisibleMask;

struct Font;
struct GPUTexture;
struct SkyBox;
struct Viewport;
struct SubViewport;
struct FogVolume;
struct Surface;
struct SurfaceInstance;
struct Mesh;
struct MeshInstance;
struct Model;
struct ModelInstance;
struct Texture;

struct Frustum2D;
struct Frustum3D;

// In which draw pass to use this pipeline state
enum class EDrawPass { 
    SHADOW, 
    DEPTH, 
    OPAQUE, 
    LIGHTING, 
    TRANSPARENT, 
    POSTPROCESS 
};
// The type of topology to emit
enum class ETopology { 
    POINT, 
    LINE, 
    TRIANGLE 
};
// Fill mode of the triangles (<-Should be in Renderer Settings)
enum class EFillMode {
    SOLID, 
    WIREFRAME 
};
// Culling triangles
enum class ECullMode { 
    NONE,
    BACKFACE,
    FRONTFACE
};
//
enum class EFrontFace { 
    CLOCKWISE, 
    COUNTER_CLOCKWISE 
};
//
enum class ECompareOp { 
    NEVER, 
    LESS, 
    EQUAL, 
    LESS_EQUAL, 
    GREATER, 
    NOT_EQUAL, 
    GREATER_EQUAL, 
    ALWAYS 
};
//
enum class EStencilOp { 
    KEEP, 
    ZERO, 
    REPLACE, 
    INC_CLAMP, 
    DEC_CLAMP, 
    INVERT, 
    INC_WRAP, 
    DEC_WRAP 
};
//
enum class EBlendFactor { 
    ZERO, 
    ONE, 
    SRC_COLOR, 
    ONE_MINUS_SRC_COLOR, 
    DST_COLOR, 
    ONE_MINUS_DST_COLOR, 
    SRC_ALPHA, 
    ONE_MINUS_SRC_ALPHA, 
    DST_ALPHA, 
    ONE_MINUS_DST_ALPHA, 
    CONSTANT_COLOR, 
    ONE_MINUS_CONSTANT_COLOR, 
    SRC_ALPHA_SATURATE
};
//
enum class EBlendOp { 
    OP_ADD, 
    OP_SUBTRACT, 
    OP_REVERSE_SUBTRACT, 
    OP_MIN, 
    OP_MAX 
};
// How to blend the pixel from the shader in the back buffer
enum class EBlendMode { 
    OPAQUE, 
    ALPHA, 
    ADDITIVE, 
    MULTIPLY, 
    CUSTOM 
};
//
enum class EShaderStage { 
    VERTEX      = 1 << 0,
    FRAGMENT    = 1 << 1,
    COMPUTE     = 1 << 2,
    GEOMETRY    = 1 << 3,
    TESS_CTRL   = 1 << 4,
    TESS_EVAL   = 1 << 5,
    MESH        = 1 << 6,
    TASK        = 1 << 7
};
//
enum class Moss_ShaderResourceType {
    UNIFORM_BUFFER,
    STORAGE_BUFFER,
    SAMPLED_TEXTURE,
    STORAGE_TEXTURE,
    SAMPLER,
    INPUT_ATTACHMENT
};
//
enum class ERendererPresentation {
    DISABLED,           // There is no logical size in effect
    STRETCH,            //  The rendered content is stretched to the output resolution
    LETTERBOX,          // The rendered content is fit to the largest dimension and the other dimension is letterboxed with the clear color
    OVERSCAN,           //  The rendered content is fit to the smallest dimension and the other dimension extends beyond the output bounds
    INTEGER_SCALE       // The rendered content is scaled up by integer multiples to fit the output resolution
};
//
enum class EShaderDataType {
    FLOAT,
    FLOAT2,
    FLOAT3,
    FLOAT4,
    INT,
    INT2,
    INT3,
    INT4,
    UINT,
    MAT3,
    MAT4,
    SAMPLER_2D,
    SAMPLER_CUBE
};
//
enum class EGPUPresentMode {
    VSYNC,
    IMMEDIATE,
    MAILBOX
};
//
enum class EVertexFormat {
    FLOAT,
    FLOAT2,
    FLOAT3,
    FLOAT4,
    UBYTE4_NORM,
    SHORT2_NORM,
    HALF2,
    HALF4
};
//
enum class EVertexInputRate { 
    PER_VERTEX, 
    PER_INSTANCE 
};
//
enum class ETextureType { 
    TEXTURE_2D, 
    TEXTURE_2D_ARRAY, 
    TEXTURE_3D, 
    TEXTURE_CUBE, 
    TEXTURE_CUBE_ARRAY 
};
//
enum class ETextureFormat {
    // Unsigned normalized color formats
    R8, 
    RG8, 
    RGB8, 
    RGBA8,
    
    // Signed normalized formats
    R8_SNORM, 
    RG8_SNORM, 
    RGB8_SNORM, 
    RGBA8_SNORM,
    
    // Floating-point formats
    R16F, 
    RG16F, 
    RGB16F, 
    RGBA16F,
    R32F, 
    RG32F, 
    RGB32F, 
    RGBA32F,
    
    // Integer formats
    R8UI, 
    RG8UI, 
    RGBA8UI, 
    R16UI, 
    RG16UI,
    RGBA16UI, 
    R32UI, 
    RG32UI, 
    RGBA32UI,
    
    // Depth formats
    Depth16, 
    Depth24, 
    Depth32F, 
    Depth24Stencil8, 
    Depth32FStencil8,
    
    // Compressed (optional support)
    DXT1, 
    DXT3, 
    DXT5, 
    BC4, 
    BC5, 
    BC6H, 
    BC7,
    
    // sRGB formats
    SRGB8, 
    SRGBA8
};
//
enum class ETextureAddressMode { 
    CLAMP, 
    WRAP, 
    MIRROR 
};
//
enum class ETextureFilter { 
    NEAREST, 
    LINEAR 
};

enum class EGPUBufferUsage {
    VERTEX,
    INDEX,
    UNIFORM,
    STORAGE,
    INDIRECT,
    TRANSFER_SRC,
    TRANSFER_DST
};
enum class MaterialTextureType {
    Color,
    Roughness, // packed g
    Metalness, // packed b
    Normal,
    Occlusion, // packed r
    Emission,
    Height,    // packed a
    AlphaMask, // packed into color a
    Packed,    // occlusion, roughness, metalness, height
    Max
};
enum class EAttachmentType {
    COLOR,
    DEPTH,
    DEPTH_STENCIL
};

enum class ELoadOp {
    LOAD,
    CLEAR,
    DONT_CARE
};

enum class EStoreOp {
    STORE,
    DONT_CARE
};

enum class EStencilPass {
    KEEP,
    REPLACE,
    INCREMENT,
    DECREMENT,
};

enum class EStencilMode {
    DISABLED = 0,

    /* Mask creation */
    WRITE,        // write 1s
    INCREMENT,    // nesting
    DECREMENT,

    /* Mask usage */
    TEST_EQUAL,
    TEST_LEQUAL,
    TEST_GEQUAL,
};

enum class EResourceState {
    SHADER_READ,
    RENDER_TARGET,
    DEPTH_WRITE,
    TRANSFER_SRC,
    TRANSFER_DST
};
enum class ECommandQueue {
    GRAPHICS,
    COMPUTE,
    TRANSFER
};
enum class ERGPassType {
    GRAPHICS,
    COMPUTE,
    TRANSFER
};
enum class EResidencyState {
    RESIDENT,
    EVICTED,
    STREAMING
};


typedef struct Moss_GPUBufferDesc {
    uint64_t size;
    EGPUBufferUsage usage;
    int cpu_visible; /* staging vs device local */
};

typedef struct Moss_VertexAttribute {
    uint32_t location;
    EVertexFormat format;
    uint32_t offset;
    EVertexInputRate rate;
};

struct VertexShader;
struct FragmentShader;
struct ComputeShader;
struct GeometryShader;
struct TessCTRLShader;
struct TessEvalShader;
struct MeshShader;
struct TaskShader;

typedef struct Moss_ShaderPushConstant {
    uint32_t size;          /* In bytes */
    EShaderStage stage_mask;
};

typedef struct Moss_ShaderCreateInfo {
    EShaderStage stage;

    const void* bytecode;     /* Source or compiled binary */
    uint32_t bytecode_size;

    const char* entry_point;  /* Usually \"main\" */

    int enable_reflection;    /* Required for materials & pipelines */
};

typedef struct Moss_ShaderResourceBinding {
    const char* name;

    Moss_ShaderResourceType type;
    EShaderStage stage_mask;

    uint32_t set;       /* Descriptor set / argument buffer */
    uint32_t binding;  /* Binding slot */
    uint32_t count;    /* Array size */

};

typedef struct Moss_PipelineShaderStage {
    EShaderStage stage;
    Moss_Shader* shader;
};

typedef struct Moss_PipelineDesc {
    EDrawPass draw_pass;
    ETopology topology;
    EFillMode fill_mode;
    ECullMode cull_mode;
    EFrontFace front_face;

    int depth_test_enable;
    int depth_write_enable;
    ECompareOp depth_compare;

    EBlendMode blend_mode;

    const Moss_VertexAttribute* vertex_attributes;
    uint32_t vertex_attribute_count;

    const Moss_PipelineShaderStage* shader_stages;
    uint32_t shader_stage_count;
};

typedef struct Moss_ResourceSetLayoutBinding {
    Moss_ShaderResourceType type;
    uint32_t binding;
    uint32_t count;
    EShaderStage stage_mask;
};

typedef struct Moss_ShaderReflection {
    const Moss_ShaderResourceBinding* resources;
    uint32_t resource_count;

    Moss_ShaderPushConstant push_constants;

};

typedef struct Moss_TextureDesc {
    ETextureType type;
    ETextureFormat format;
    ETextureUsage usage;

    uint32_t width;
    uint32_t height;
    uint32_t depth;     /* For 3D textures */
    uint32_t layers;    /* Array / cube */
    uint32_t mip_levels;

    int generate_mips;
};

typedef struct Moss_GPUSamplerDesc {
    ETextureFilter min_filter;
    ETextureFilter mag_filter;
    ETextureFilter mip_filter;

    ETextureAddressMode address_u;
    ETextureAddressMode address_v;
    ETextureAddressMode address_w;

    float mip_lod_bias;
    float max_anisotropy;
};

typedef struct Moss_TextureViewDesc {
    Moss_Texture* texture;
    ETextureFormat format;

    uint32_t base_mip;
    uint32_t mip_count;

    uint32_t base_layer;
    uint32_t layer_count;
};

typedef struct Moss_ResourceSetLayoutDesc {
    uint32_t set_index;

    const Moss_ResourceSetLayoutBinding* bindings;
    uint32_t binding_count;

    int bindless; /* enables bindless for this set */
};

typedef struct Moss_RGTextureDesc {
    uint32_t width, height;
    ETextureFormat format;
    uint32_t mip_levels;
    uint32_t layers;
};

typedef struct Moss_RGPassDesc {
    ERGPassType type;
    const char* name;

    void (*record)(Moss_GPUCommandBuffer* cmd, void* user_data);
    void* user_data;
};

typedef struct Moss_StreamedTextureDesc {
    Moss_TextureDesc desc;
    uint32_t max_resident_mips;
};


typedef struct Viewport {
    float x;
    float y;
    float width;
    float height;

    float min_depth;
    float max_depth;
};

typedef struct Moss_FramebufferAttachmentDesc
{
    EAttachmentType type;
    ETextureFormat format;

    ELoadOp  load_op;
    EStoreOp store_op;

    float clear_color[4];   /* Only for color */
    float clear_depth;      /* Only for depth */
    uint32_t clear_stencil;

};

typedef struct Moss_FramebufferDesc {
    uint32_t width;
    uint32_t height;

    const Moss_FramebufferAttachmentDesc* attachments;
    uint32_t attachment_count;

    int sampled; /* Can attachments be sampled in shaders? */
};
typedef struct Moss_PostProcessPass {
    Moss_PipelineState* pipeline;
    Moss_ResourceSet* resource_set;
    Moss_Framebuffer* target;
};
typedef struct SubViewport {
    Viewport viewport;
    Moss_Framebuffer* target;
};



typedef struct Moss_DepthStencilState {
    bool depth_test;
    bool depth_write;
    ECompareOp depth_compare;

    EStencilMode stencil;
};

struct ComputePipeline;

typedef void (*Moss_ShaderReloadCallback)(Moss_Shader* shader, void* user_data);
typedef void (*Moss_SubViewportRecordFn)(Moss_GPUCommandBuffer* cmd, void* user_data);



MOSS_API Moss_AcquireGPUCommandBuffer();
MOSS_API Moss_BeginGPUComputePass();
MOSS_API Moss_BeginGPUCopyPass();
MOSS_API Moss_BeginGPURenderPass();
MOSS_API Moss_BindGPUComputeSamplers();
MOSS_API Moss_BindGPUFragmentSamplers();
MOSS_API Moss_BindGPUIndexBuffer();
MOSS_API Moss_BindGPUVertexBuffers();
MOSS_API Moss_BindGPUVertexSamplers();
MOSS_API Moss_BlitGPUTexture();
MOSS_API Moss_CalculateGPUTextureFormatSize();
MOSS_API Moss_CancelGPUCommandBuffer();
MOSS_API Moss_CopyGPUBufferToBuffer();
MOSS_API Moss_CopyGPUTextureToTexture();
MOSS_API Moss_CreateGPUBuffer();
MOSS_API Moss_CreateGPUSampler();
MOSS_API Moss_CreateGPUShader();
MOSS_API Moss_CreateGPUTexture();
MOSS_API Moss_CreateGPUTransferBuffer();
MOSS_API Moss_DispatchGPUCompute();
MOSS_API Moss_DispatchGPUComputeIndirect();
MOSS_API Moss_DownloadFromGPUBuffer();
MOSS_API Moss_DownloadFromGPUTexture();
MOSS_API Moss_DrawGPUIndexedPrimitives();
MOSS_API Moss_DrawGPUIndexedPrimitivesIndirect();
MOSS_API Moss_DrawGPUPrimitives();
MOSS_API Moss_DrawGPUPrimitivesIndirect();
MOSS_API Moss_EndGPUComputePass();
MOSS_API Moss_EndGPUCopyPass();
MOSS_API Moss_EndGPURenderPass();
MOSS_API Moss_GDKResumeGPU();
MOSS_API Moss_GDKSuspendGPU();
MOSS_API Moss_GenerateMipmapsForGPUTexture();
MOSS_API Moss_GetGPUDriver();
MOSS_API Moss_GetGPUShaderFormats();
MOSS_API Moss_GetGPUTextureFormatFromPixelFormat();
MOSS_API Moss_GetNumGPUDrivers();
MOSS_API Moss_GetPixelFormatFromGPUTextureFormat();
MOSS_API Moss_GPUSupportsProperties();
MOSS_API Moss_GPUSupportsShaderFormats();
MOSS_API Moss_GPUTextureFormatTexelBlockSize();
MOSS_API Moss_GPUTextureSupportsFormat();
MOSS_API Moss_GPUTextureSupportsSampleCount();
MOSS_API Moss_InsertGPUDebugLabel();
MOSS_API Moss_MapGPUTransferBuffer();
MOSS_API Moss_PopGPUDebugGroup();
MOSS_API Moss_PushGPUComputeUniformData();
MOSS_API Moss_PushGPUDebugGroup();
MOSS_API Moss_PushGPUFragmentUniformData();
MOSS_API Moss_PushGPUVertexUniformData();
MOSS_API Moss_QueryGPUFence();
MOSS_API Moss_ReleaseGPUBuffer();
MOSS_API Moss_ReleaseGPUFence();
MOSS_API Moss_ReleaseGPUSampler();
MOSS_API Moss_ReleaseGPUShader();
MOSS_API Moss_ReleaseGPUTexture();
MOSS_API Moss_ReleaseGPUTransferBuffer();
MOSS_API Moss_SetGPUAllowedFramesInFlight();
MOSS_API Moss_SetGPUBlendConstants();
MOSS_API Moss_SetGPUBufferName();
MOSS_API Moss_SetGPUScissor();
MOSS_API Moss_SetGPUStencilReference();
MOSS_API Moss_SetGPUTextureName();
MOSS_API Moss_SetGPUViewport();
MOSS_API Moss_SubmitGPUCommandBuffer();
MOSS_API Moss_SubmitGPUCommandBufferAndAcquireFence();
MOSS_API Moss_UnmapGPUTransferBuffer();
MOSS_API Moss_UploadToGPUBuffer();
MOSS_API Moss_UploadToGPUTexture();

MOSS_API Moss_WaitForGPUFences();
MOSS_API Moss_WaitForGPUIdle();
MOSS_API Moss_WindowSupportsGPUPresentMode();

MOSS_API Moss_GPUBlitInfo();
MOSS_API Moss_GPUBlitRegion();
MOSS_API Moss_GPUBufferBinding();
MOSS_API Moss_GPUBufferCreateInfo();
MOSS_API Moss_GPUBufferLocation();
MOSS_API Moss_GPUBufferRegion();
MOSS_API Moss_GPUColorTargetBlendState();
MOSS_API Moss_GPUColorTargetDescription();
MOSS_API Moss_GPUColorTargetInfo();
MOSS_API Moss_GPUDepthStencilState();
MOSS_API Moss_GPUDepthStencilTargetInfo();
MOSS_API Moss_GPUIndexedIndirectDrawCommand();
MOSS_API Moss_GPUIndirectDispatchCommand();
MOSS_API Moss_GPUIndirectDrawCommand();
MOSS_API Moss_GPUMultisampleState();
MOSS_API Moss_GPURasterizerState();
MOSS_API Moss_GPUSamplerCreateInfo();
MOSS_API Moss_GPUShaderCreateInfo();
MOSS_API Moss_GPUStencilOpState();
MOSS_API Moss_GPUTextureCreateInfo();
MOSS_API Moss_GPUTextureLocation();
MOSS_API Moss_GPUTextureRegion();
MOSS_API Moss_GPUTextureSamplerBinding();
MOSS_API Moss_GPUTextureTransferInfo();
MOSS_API Moss_GPUTransferBufferCreateInfo();
MOSS_API Moss_GPUTransferBufferLocation();
MOSS_API Moss_GPUVertexAttribute();
MOSS_API Moss_GPUVertexBufferDescription();
MOSS_API Moss_GPUVertexInputState();
MOSS_API Moss_GPUViewport();


MOSS_API Moss_BindGPUComputeStorageBuffers();
MOSS_API Moss_BindGPUComputeStorageTextures();
MOSS_API Moss_BindGPUFragmentStorageBuffers();
MOSS_API Moss_BindGPUFragmentStorageTextures();
MOSS_API Moss_BindGPUVertexStorageBuffers();
MOSS_API Moss_BindGPUVertexStorageTextures();
MOSS_API Moss_GPUStorageBufferReadWriteBinding();
MOSS_API Moss_GPUStorageTextureReadWriteBinding();

MOSS_API Moss_GPUVulkanOptions();
MOSS_API Moss_GetGPUDeviceDriver();
MOSS_API Moss_GetGPUDeviceProperties();
MOSS_API Moss_ReleaseWindowFromGPUDevice();
MOSS_API Moss_ClaimWindowForGPUDevice();
MOSS_API Moss_DestroyGPUDevice();
MOSS_API Moss_CreateGPUDevice();
MOSS_API Moss_CreateGPUDeviceWithProperties();


MOSS_API Moss_AcquireGPUSwapchainTexture();
MOSS_API Moss_GetGPUSwapchainTextureFormat();
MOSS_API Moss_SetGPUSwapchainParameters();
MOSS_API Moss_WaitAndAcquireGPUSwapchainTexture();
MOSS_API Moss_WaitForGPUSwapchain();
MOSS_API Moss_WindowSupportsGPUSwapchainComposition();



MOSS_API Moss_GPUBuffer* Moss_GPUBufferCreate(Moss_Renderer* renderer, const Moss_GPUBufferDesc* desc);
MOSS_API void Moss_GPUBufferUpload(Moss_Renderer* renderer, Moss_GPUBuffer* buffer, const void* data, uint64_t size, uint64_t offset);
MOSS_API void Moss_GPUBufferDestroy(Moss_Renderer* renderer, Moss_GPUBuffer* buffer);
MOSS_API void* Moss_GPUBufferMap(Moss_Renderer* renderer, Moss_GPUBuffer* buffer);
MOSS_API void Moss_GPUBufferUnmap(Moss_Renderer* renderer, Moss_GPUBuffer* buffer);

// Shader
MOSS_API Moss_Shader* Moss_ShaderCreate(Moss_Renderer* renderer, const Moss_ShaderCreateInfo* create_info);
MOSS_API void Moss_ShaderDestroy(Moss_Shader* shader);
MOSS_API const Moss_ShaderReflection* Moss_ShaderGetReflection(const Moss_Shader* shader);
MOSS_API void Moss_ShaderSetReloadCallback(Moss_Shader* shader, Moss_ShaderReloadCallback callback, void* user_data);
MOSS_API int Moss_ShaderReloadFromSource(Moss_Shader* shader, const void* new_bytecode, uint32_t bytecode_size);

// Bindless Textures
MOSS_API Moss_BindlessHandle Moss_BindlessRegisterTexture(Moss_Renderer* renderer, void* texture);
MOSS_API Moss_BindlessHandle Moss_BindlessRegisterBuffer(Moss_Renderer* renderer, Moss_GPUBuffer* buffer);
MOSS_API void Moss_BindlessUnregister(Moss_Renderer* renderer, Moss_BindlessHandle handle);
MOSS_API Moss_GPUSampler* Moss_GPUSamplerCreate(Moss_Renderer* renderer, const Moss_GPUSamplerDesc* desc);
MOSS_API void Moss_GPUSamplerDestroy(Moss_GPUSampler* sampler);


MOSS_API Moss_TextureView* Moss_TextureViewCreate(Moss_Renderer* renderer, const Moss_TextureViewDesc* desc);
MOSS_API void Moss_TextureViewDestroy(Moss_TextureView* view);


MOSS_API Moss_Texture* Moss_TextureCreate(Moss_Renderer* renderer, const Moss_TextureDesc* desc);
MOSS_API void Moss_TextureDestroy(Moss_Texture* texture);


MOSS_API void Moss_TextureUpload(...);
MOSS_API void Moss_TextureGenerateMips(Moss_Texture*);
MOSS_API void Moss_DrawFullscreenTriangle(Moss_Renderer*);

MOSS_API Moss_GPUCommandBuffer* Moss_GPUCommandBufferBegin( Moss_Renderer* renderer, ECommandQueue queue);
MOSS_API void Moss_GPUCommandBufferEnd(Moss_GPUCommandBuffer* cmd);
MOSS_API void Moss_GPUCommandBufferSubmit(Moss_Renderer* renderer, Moss_GPUCommandBuffer* cmd);
MOSS_API void Moss_CmdBindPipeline(Moss_GPUCommandBuffer* cmd, Moss_PipelineState* pipeline);
MOSS_API void Moss_CmdBindResourceSet(Moss_GPUCommandBuffer* cmd, Moss_PipelineState* pipeline, uint32_t set_index, Moss_ResourceSet* set);
MOSS_API void Moss_CmdBindVertexBuffer(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* buffer, uint64_t offset);
MOSS_API void Moss_CmdBindIndexBuffer(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* buffer, uint64_t offset);
MOSS_API void Moss_CmdDraw(Moss_GPUCommandBuffer* cmd, uint32_t vertex_count, uint32_t first_vertex);
MOSS_API void Moss_CmdDispatch(Moss_GPUCommandBuffer* cmd, uint32_t x, uint32_t y, uint32_t z);


MOSS_API Moss_RGTexture* Moss_RGPassReadTexture(Moss_RGPass* pass, Moss_RGTexture* texture, EResourceState state);
MOSS_API Moss_RGTexture* Moss_RGPassWriteTexture(Moss_RGPass* pass, Moss_RGTexture* texture, EResourceState state);


MOSS_API int Moss_RendererSupportsAsyncCompute(Moss_Renderer* renderer);
MOSS_API void Moss_TextureSetResidency(Moss_Texture* texture, EResidencyState state);




MOSS_API void Moss_RendererClear();

/* ======================================================
 * Renderer Side
 * =================================================== */

typedef struct Camera2D {
    RVec2 position;
	RVec2 offset;
    float zoom = 1.0f;
    float rotation = 0.0f;

    RMat44 u_viewProj;
	Frustum2D m_frustum;

    LightMask light_mask;
	VisibleLayer visible_layer;
};

typedef struct Camera3D {
    RVec3 position;
	Vec3 up = Vec3(0.0f, 1.0f, 0.0f);    // Y is up
    Vec3 target;
	Quat Orientation = Quat(0.0f, 0.0f, -1.0f);

    float speed = 0.1f;
    float sensitivity = 100;

    float fov = 45.0f;          // Field of view (degrees)
    float aspectRatio;          // avoid divide-by-zero
    float nearPlane = 0.1f;
    float farPlane = 1000.0f;

    RMat44 u_viewProj;
	Frustum3D m_frustum;

    CullMask cullMask;
	LightMask light_mask;
};

typedef struct Material2D {
    Color* albedo;
    Texture* albedoMap;
    Texture* normalMap;
};

typedef struct Material3D {
    Color* albedo;
    float* metallic;
    float* roughness;
    float* ao;
    float* specular;
    Texture* albedoMap;
    Texture* normalMap;
    Texture* roughnessMap;
    Texture* metallicMap;
    Texture* aoMap;
};


// Lighting
// TextureLight2D is a Light that uses a texture as its emission
struct TextureLight2D { 
    float intensity, rotation; 
    Texture* texture; 
    Float2 position; 
    Color* color; 
    LightMask filter; 
};
// DirectionalLight2D
struct DirectionalLight2D { 
    float intensity, rotation; 
    Color* color; 
    LightMask filter; 
};
// PointLight2D
struct PointLight2D { 
    float intensity, rotation, radius; 
    Float2 position; 
    Color* color; 
    LightMask filter; 
};

struct TextureLight3D { 
    float intensity; 
    Texture* texture;
    Float3 position, rotation; 
    Color* color; 
    LightMask filter;
};
struct DirectionalLight3D { 
    float intensity;
    Float3 rotation;
    Color* color;
    LightMask filter;
};
struct SpotLight3D { 
    float intensity, radius, angle, penumbra; 
    Float3 position, rotation; 
    Color* color; 
    LightMask filter; 
};
struct OmniLight3D { 
    float intensity, radius; 
    Float3 position; 
    Color* color; 
    LightMask filter; 
};

struct SurfaceColor {
    Color* color;
    Rect* resource_set;
    Moss_PipelineState* pipeline;
};
struct SurfaceRect {
    Color* color;
    Texture* texture;
    Rect* resource_set;
    Moss_PipelineState* pipeline;
};

struct SurfaceRectInstance {
    Color* color;
    Texture* texture;
    Rect* resource_set;
    Moss_PipelineState* pipeline;
    float transform[16]; /* column-major mat4 */
};

typedef struct Mesh {
    Material3D material;
    Moss_GPUBuffer* vertex_buffer;
    Moss_GPUBuffer* index_buffer;

    uint32_t vertex_count;
    uint32_t index_count;

    uint32_t vertex_stride;
};
typedef struct MeshInstance {
    Mesh* mesh;
    float transform[16]; /* column-major mat4 */
    Moss_VisibleMask visibility;
};

typedef struct Model {
    Mesh** meshes;
    uint32_t mesh_count;
};

typedef struct ModelInstance {
    Model* model;
    float transform[16];
    Moss_VisibleMask visibility;
};

typedef struct SkyBox {
    Moss_Texture* cubemap;
    Moss_GPUSampler* sampler;
    Moss_PipelineState* pipeline;
    Moss_ResourceSet* resource_set;
};

typedef struct FogVolume {
    float transform[16];
    float density;
    float falloff;
    float height;
    Color* color;
    Moss_CullMask cull_mask;
};

typedef struct Sprite2D {
    Texture* texture;
    Rect* rect;
    Moss_PipelineState* pipeline;
    float rotation;
    Float2 uv_min;
    Float2 uv_max;

    Material2D material;

    Moss_VisibleMask visibility;
};

typedef struct Sprite3D {
    Mesh* sampler;

    float transform[16];  /* world matrix */

    float size[2];
    Color* color;

    Moss_VisibleMask visibility;
};

typedef struct Decal { 
    Texture* Albedo, 
    Texture* Normal, 
    Texture* Orm, 
    Texture* Emission; 
    Color color; 
    float emission_energy, 
    float blendFactor = 1.0f; 
    Mat44 model; 
    CullFilter filter; 
};

MOSS_API Moss_PipelineState* Moss_RendererCreatePipeline(Moss_Renderer* renderer,const Moss_PipelineDesc* desc);
MOSS_API void Moss_PipelineBind(Moss_PipelineState* pipeline);
MOSS_API void Moss_PipelineUnbind(Moss_PipelineState* pipeline);
MOSS_API void Moss_PipelineSetUniform(Moss_PipelineState* pipeline,const char* name, const void* data, uint32_t size);
MOSS_API void Moss_PipelineFlush(Moss_PipelineState* pipeline);
MOSS_API int Moss_PipelineValidate(Moss_PipelineState* pipeline);

MOSS_API Moss_ResourceSet* Moss_ResourceSetCreate(Moss_Renderer* renderer, const Moss_ResourceSetLayoutDesc* desc);
MOSS_API void Moss_ResourceSetDestroy(Moss_ResourceSet* set);
MOSS_API void Moss_ResourceSetBindUniformBuffer(Moss_ResourceSet* set, uint32_t binding, Moss_GPUBuffer* buffer);
MOSS_API void Moss_ResourceSetBindUniformBufferRange(Moss_ResourceSet* set, uint32_t binding, Moss_GPUBuffer* buffer, size_t offset, size_t size);
MOSS_API void Moss_ResourceSetBindStorageBuffer(Moss_ResourceSet* set, uint32_t binding, Moss_GPUBuffer* buffer);
MOSS_API void Moss_ResourceSetBindTexture(Moss_ResourceSet* set,uint32_t binding, void* texture);
MOSS_API void Moss_ResourceSetBindSampler(Moss_ResourceSet* set, uint32_t binding, void* sampler);
MOSS_API void Moss_ResourceSetBind(Moss_Renderer* renderer, Moss_PipelineState* pipeline, uint32_t set_index, Moss_ResourceSet* set);

// Dont forget the FrameBuffer for post processing
MOSS_API Moss_Framebuffer* Moss_FramebufferCreate(Moss_Renderer* renderer, const Moss_FramebufferDesc* desc);
MOSS_API void Moss_FramebufferDestroy(Moss_Framebuffer* framebuffer);
MOSS_API void Moss_FramebufferResize(Moss_Framebuffer* framebuffer, uint32_t width, uint32_t height);
MOSS_API void Moss_PostProcessExecute(Moss_Renderer* renderer, const Moss_PostProcessPass* pass);
MOSS_API void Moss_FramebufferBegin(Moss_Renderer* renderer, Moss_Framebuffer* framebuffer);
MOSS_API void Moss_FramebufferEnd(Moss_Renderer* renderer, Moss_Framebuffer* framebuffer);
MOSS_API void Moss_TextureBarrier(Moss_Renderer* renderer, Moss_Texture* texture, EResourceState old_state, EResourceState new_state);
MOSS_API Moss_Framebuffer* Moss_RendererGetBackbuffer(Moss_Renderer* renderer);



void Moss_RenderSubViewport(Moss_GPUCommandBuffer* cmd, const SubViewport* sv, Moss_SubViewportRecordFn record, void* user_data) {
    Moss_CmdSetFramebuffer(cmd, sv->target);
    Moss_CmdSetViewport(cmd, &sv->viewport);

    record(cmd, user_data);

    Moss_CmdSetFramebuffer(cmd, NULL); /* backbuffer */
}


MOSS_API Moss_Renderer* Moss_CreateRenderer(Moss_Window* window, Color* bg_color, int* window_width, int* window_height, int* virt_res_wdith, int* virt_res_hight);
MOSS_API void Moss_RendererDestroy(Moss_Renderer* renderer);
MOSS_API void Moss_RendererBeginFrame(Moss_Renderer* renderer);
MOSS_API void Moss_RendererEndFrame(Moss_Renderer* renderer);

MOSS_API Mesh* Moss_MeshLoadFBX(const char* file_path);
MOSS_API Mesh* Moss_MeshLoadOBJ(const char* file_path);
MOSS_API Mesh* Moss_MeshLoadGLB(const char* file_path);

MOSS_API Model* Moss_ModelLoadFBX(const char* file_path);
MOSS_API Model* Moss_ModelLoadOBJ(const char* file_path);
MOSS_API Model* Moss_ModelLoadGLB(const char* file_path);

MOSS_API void Moss_MeshDraw(Mesh* mesh);
MOSS_API void Moss_ModelDraw(Model* model);

MOSS_API void Moss_MeshRemove(Mesh* mesh);
MOSS_API void Moss_ModelRemove(Model* model);

#if defined(MOSS_DEBUG_RENDERER)
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API char* Moss_GPUGetPhysicalDeviceName();
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API char* Moss_GPUAPIGetName();
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API char* Moss_GPUAPIGetVersion();



//===========================================================
// 2D Debug/Utils Rendering
//===========================================================
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawLine2D(Moss_Renderer* renderer, const Vec2* from, const Vec2* to, Color color, float thickness);
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawLines2D(Moss_Renderer* renderer, const TArray<Vec2>* positions, Color color, float thickness);
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawBox2D(Moss_Renderer* renderer, const Vec2* center, const Vec2* size, float rotation, Color color);
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawCircle2D(Moss_Renderer* renderer, const Vec2* center, float radius, Color color);
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawCylinder2D(Moss_Renderer* renderer, const RMatrix4x4* matrix, float halfHeight, float radius, Color color);
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawTriangle2D(Moss_Renderer* renderer, const Vec2* p0, const Vec2* p1, const Vec2* p2, Color color);
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawArrow2D(Moss_Renderer* renderer, const Vec2* from, const Vec2* to, Color color, float headSize);
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawMarker2D(Moss_Renderer* renderer, const Vec2* position, Color color, float size);
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawBone2D(Moss_Renderer* renderer, );
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawCoordinateSystem2D(Moss_Renderer* renderer, const RMatrix4x4* matrix, float size);
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawCapsule2D(Moss_Renderer* renderer, const RMatrix4x4* matrix, float halfHeightOfCylinder, float radius, Color color);
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawConvex2D(Moss_Renderer* renderer, );
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawConvexHull2D(Moss_Renderer* renderer, );
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawPolygonShape3D(Moss_Renderer* renderer, );
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawGizmo2D(Moss_Renderer* renderer, const RMat4x4* transform, float size);

//===========================================================
// 3D Debug/Utils Rendering
//===========================================================
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawLine3D(Moss_Renderer* renderer, const Vec3* from, const Vec3* to, Color color, float thickness);
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawLines3D(Moss_Renderer* renderer, const TArray<Vec3>* positions, Color color, float thickness);
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawBox3D(Moss_Renderer* renderer, const Vec3* center, const Vec3* size, const RMat4x4* rotation, Color color);
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawSphere3D(Moss_Renderer* renderer, const Vec3* center, float radius, Color color);
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawCylinder3D(Moss_Renderer* renderer, const Vec3* base, const Vec3* top, float radius, Color color);
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawCone3D(Moss_Renderer* renderer, );
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawTriangle3D(Moss_Renderer* renderer, );
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawArrow3D(Moss_Renderer* renderer, const Vec3* from, const Vec3* to, Color color, float headSize);
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawMarker3D(Moss_Renderer* renderer, const Vec3* position, Color color, float size);
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawPlane3D(Moss_Renderer* renderer, );
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawBone3D(Moss_Renderer* renderer, );
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawCoordinateSystem3D(Moss_Renderer* renderer, const RMat4x4* matrix, float size);
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawCapsule3D(Moss_Renderer* renderer, const RMat4x4* matrix, float halfHeightOfCylinder, float radius, Color color);
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawConvex3D(Moss_Renderer* renderer, );
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawConvexHull3D(Moss_Renderer* renderer, );
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawPolygonShape3D(Moss_Renderer* renderer, );
/*! @brief X @param X X @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawGizmo3D(Moss_Renderer* renderer, const RMat4x4* transform, float size);
#endif

typedef enum Moss_Upscaler {
    MOSS_UPSCALER_NONE,
    MOSS_UPSCALER_FSR1,
    MOSS_UPSCALER_FSR2
};

void Moss_RendererSetUpscaler(Moss_Renderer* renderer, Moss_Upscaler upscaler);
MOSS_API Moss_GPUDevice* Moss_RendererGetGPUDevice(Moss_Renderer* renderer) const;






#if defined(MOSS_GRAPHICS_VULKAN) || defined(MOSS_GRAPHICS_DIRECTX)
/* ======================================================
 * Meshlet / Clustered Mesh Support
 * =================================================== */
// A single meshlet holds a small cluster of vertices and primitives
struct alignas(16) Moss_Meshlet {
    Vec3 center;
    float radius;

    int8_t cone_axis[3];   // normalized direction in [-127,127]
    int8_t cone_cutoff;    // cos(cone_angle) packed into 0..127

    uint32_t dataOffset;   // offset to packed vertex indices
    uint32_t baseVertex;   // offset for base vertex
    uint8_t vertexCount;
    uint8_t triangleCount;
    uint8_t shortRefs;     // 4-bit packed indices flag
    uint8_t padding;       // pad to 16 bytes
};
// Optional meshlet cluster info, useful for indirect rendering
typedef struct Moss_MeshletCluster {
    Moss_Meshlet* meshlets;
    uint32_t meshlet_count;

    uint32_t* draw_commands;   // optional multi-draw
    uint32_t draw_command_count;
} Moss_MeshletCluster;

// GPU-side buffers to hold meshlet data
typedef struct Moss_MeshletBuffers {
    Moss_GPUBuffer* vertex_indices;  // packed vertex indices buffer
    Moss_GPUBuffer* primitives;      // triangle indices buffer
    Moss_GPUBuffer* meshlets;        // buffer of Moss_Meshlet structs
    uint32_t meshlet_count;

    // Optional: indirect draw buffer for GPU
    Moss_GPUBuffer* indirect_draws;
} Moss_MeshletBuffers;

//#define MOSS_ENABLE_MESHLETS

#endif // MOSS_GRAPHICS_VULKAN || MOSS_GRAPHICS_DIRECTX


// GPU APIs / Tools
#if defined(MOSS_GRAPHICS_OPENGL) || defined(MOSS_GRAPHICS_OPENGLES)
#endif // MOSS_GRAPHICS_OPENGL || MOSS_GRAPHICS_OPENGLES


#if defined(MOSS_GRAPHICS_VULKAN)
MOSS_API VkDevice						GetDevice(Moss_Renderer* renderer) const												{ return mDevice; }
MOSS_API VkDescriptorPool				GetDescriptorPool(Moss_Renderer* renderer) const										{ return mDescriptorPool; }
MOSS_API VkDescriptorSetLayout			GetDescriptorSetLayoutTexture(Moss_Renderer* renderer) const							{ return mDescriptorSetLayoutTexture; }
MOSS_API VkSampler						GetTextureSamplerRepeat(Moss_Renderer* renderer) const									{ return mTextureSamplerRepeat; }
MOSS_API VkSampler						GetTextureSamplerShadow(Moss_Renderer* renderer) const									{ return mTextureSamplerShadow; }
MOSS_API VkRenderPass					GetRenderPassShadow(Moss_Renderer* renderer) const										{ return mRenderPassShadow; }
MOSS_API VkRenderPass					GetRenderPass(Moss_Renderer* renderer) const											{ return mRenderPass; }
MOSS_API VkPipelineLayout				GetPipelineLayout(Moss_Renderer* renderer) const										{ return mPipelineLayout; }
MOSS_API VkCommandBuffer				GetCommandBuffer(Moss_Renderer* renderer)												{ MOSS_ASSERT(mInFrame); return mCommandBuffers[mFrameIndex]; }
MOSS_API VkCommandBuffer				StartTempCommandBuffer(Moss_Renderer* renderer);
MOSS_API void							EndTempCommandBuffer(VkCommandBuffer inCommandBuffer);
MOSS_API void							AllocateMemory(VkDeviceSize inSize, uint32 inMemoryTypeBits, VkMemoryPropertyFlags inProperties, VkDeviceMemory &outMemory);
MOSS_API void							FreeMemory(VkDeviceMemory inMemory, VkDeviceSize inSize);
MOSS_API void							CreateBuffer(VkDeviceSize inSize, VkBufferUsageFlags inUsage, VkMemoryPropertyFlags inProperties, BufferVK &outBuffer);
MOSS_API void							CopyBuffer(VkBuffer inSrc, VkBuffer inDst, VkDeviceSize inSize);
MOSS_API void							CreateDeviceLocalBuffer(const void *inData, VkDeviceSize inSize, VkBufferUsageFlags inUsage, BufferVK &outBuffer);
MOSS_API void							FreeBuffer(BufferVK &ioBuffer);
MOSS_API unique_ptr<ConstantBufferVK>	CreateConstantBuffer(VkDeviceSize inBufferSize);
MOSS_API void							CreateImage(uint32 inWidth, uint32 inHeight, VkFormat inFormat, VkImageTiling inTiling, VkImageUsageFlags inUsage, VkMemoryPropertyFlags inProperties, VkImage &outImage, VkDeviceMemory &outMemory);
MOSS_API void							DestroyImage(VkImage inImage, VkDeviceMemory inMemory);
MOSS_API VkImageView					CreateImageView(VkImage inImage, VkFormat inFormat, VkImageAspectFlags inAspectFlags);
MOSS_API VkFormat						FindDepthFormat();

MOSS_API void Moss_Vulkan_EncodeSPIRV(const char* path);
MOSS_API void Moss_Vulkan_DecodeSPIRV(const char* path);
MOSS_API void Moss_VK_WindowResize(Moss_Renderer* handle, uint32_t width, uint32_t height); // Resize window
#endif // MOSS_GRAPHICS_VULKAN

#if defined(MOSS_GRAPHICS_DIRECTX)
MOSS_API ID3D12Device*					GetDevice(Moss_Renderer* renderer);
MOSS_API ID3D12RootSignature*			GetRootSignature(Moss_Renderer* renderer);
MOSS_API ID3D12GraphicsCommandList*		GetCommandList(Moss_Renderer* renderer);
MOSS_API CommandQueueDX12&				GetUploadQueue(Moss_Renderer* renderer);
MOSS_API DescriptorHeapDX12&			GetDSVHeap(Moss_Renderer* renderer);
MOSS_API DescriptorHeapDX12&			GetSRVHeap(Moss_Renderer* renderer);


MOSS_API bool Moss_DX_CompileHLSL(const char* inputPath, const char* outputPath, const char* entryPoint, const char* target);
MOSS_API void Moss_DX_EnableDebugLayer(bool enable);	/* Debug */
MOSS_API uint32_t Moss_DX_GetShaderModel(void);		/* Info */
#endif // MOSS_GRAPHICS_DIRECTX

#if defined(MOSS_GRAPHICS_METAL)
MOSS_API MTKView*						GetView(Moss_Renderer* renderer) const;
MOSS_API id<MTLDevice>					GetDevice(Moss_Renderer* renderer) const;
MOSS_API id<MTLRenderCommandEncoder>	GetRenderEncoder(Moss_Renderer* renderer) const;


MOSS_API bool Moss_Metal_CompileMSL(const char* inputPath, const char* outputPath);	/* Shader tools */
MOSS_API bool Moss_Metal_SupportsFamily(uint32_t family);	/* Capabilities */
#endif // MOSS_GRAPHICS_METAL

// Debug / Utils
typedef void* Moss_Result;

MOSS_API void Moss_GPUFatalError(Moss_Result result);

#endif // MOSS_RENDERER_H
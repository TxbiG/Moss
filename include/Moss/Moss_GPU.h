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
 * @file Moss_GPU.h
 * @brief Core GPU interface for the Moss Framework.
 *
 * The renderer module provides the abstraction layer for all graphics
 * operations. It serves as a unified, API-agnostic rendering backend capable of
 * targeting multiple graphics APIs (Vulkan, OpenGL, OpenGL ES, Metal, and DirectX 12).
 * 
 * ### Macros
 * - MOSS_GRAPHICS_OPENGL 	- OpenGL v3.3
 * - MOSS_GRAPHICS_OPENGLES - OpenGL ES v2.0 or v3.0
 * - MOSS_GRAPHICS_VULKAN 	- Vulkan
 * - MOSS_GRAPHICS_DIRECTX 	- DirectX 12
 * - MOSS_GRAPHICS_METAL 	- Metal
 * 
 */

 // Mine

#ifndef MOSS_GPU_H
#define MOSS_GPU_H

#if defined(MOSS_USE_OPENGL) && !defined(MOSS_GRAPHICS_OPENGL)
#define MOSS_GRAPHICS_OPENGL 1
#endif
#if defined(MOSS_USE_OPENGLES) && !defined(MOSS_GRAPHICS_OPENGLES)
#define MOSS_GRAPHICS_OPENGLES 1
#endif
#if defined(MOSS_USE_VULKAN) && !defined(MOSS_GRAPHICS_VULKAN)
#define MOSS_GRAPHICS_VULKAN 1
#endif
#if defined(MOSS_USE_DIRECTX12) && !defined(MOSS_GRAPHICS_DIRECTX)
#define MOSS_GRAPHICS_DIRECTX 1
#endif
#if defined(MOSS_USE_METAL) && !defined(MOSS_GRAPHICS_METAL)
#define MOSS_GRAPHICS_METAL 1
#endif
#ifdef MOSS_GRAPHICS_VULKAN
#include <vulkan/vulkan.h>
#endif // MOSS_GRAPHICS_VULKAN

#include <Moss/Moss_stdinc.h>
#include <Moss/Moss_Platform.h>



struct Moss_GPUDevice;
struct Moss_GPUDeviceDesc;
struct Moss_GPUCommandBuffer;
struct Moss_GPUBuffer;
struct Moss_GPUTexture;
struct Moss_GPUTextureView;
struct Moss_GPUSampler;
struct Moss_GPUShader;
struct Moss_GPUTransferBuffer;
struct Moss_GPUQueryPool;
struct Moss_Texture;
struct Moss_TextureView;
struct Moss_Shader;
struct Moss_PipelineState;
struct Moss_ComputePipelineState;
struct Moss_PipelineDesc;
struct Moss_ResourceSet;
struct Moss_ResourceSetLayout;
struct Moss_Framebuffer;
struct Moss_GPUFence;
struct Moss_ShaderDesc;
struct Moss_RenderGraph;
struct Moss_RGPass;
struct Moss_RGTexture;
struct Moss_RGBuffer;

using Moss_BindlessHandle = uint32_t;



struct VertexShader;
struct PixelShader;
struct ComputeShader;
struct Compute_Queue;
struct Compute_Buffer;

struct Moss_Rect {
    int32_t x = 0;
    int32_t y = 0;
    int32_t w = 0;
    int32_t h = 0;
};

struct Moss_GPUViewport {
    float x = 0.0f;
    float y = 0.0f;
    float w = 0.0f;
    float h = 0.0f;
    float min_depth = 0.0f;
    float max_depth = 1.0f;
};

#define MOSS_GPU_COMMON_RENDER_STATE_DEFINED 1

enum class EDrawPass {
    SHADOW,
    DEPTH,
    OPAQUE,
    LIGHTING,
    TRANSPARENT,
    POSTPROCESS
};

enum class ETopology {
    POINT,
    LINE,
    TRIANGLE
};

enum class EFillMode {
    SOLID,
    WIREFRAME
};

enum class ECullMode {
    NONE,
    BACKFACE,
    FRONTFACE
};

enum class EFrontFace {
    CLOCKWISE,
    COUNTER_CLOCKWISE
};

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

enum class EBlendOp {
    OP_ADD,
    OP_SUBTRACT,
    OP_REVERSE_SUBTRACT,
    OP_MIN,
    OP_MAX
};

enum class EBlendMode {
    OPAQUE,
    ALPHA,
    ADDITIVE,
    MULTIPLY,
    CUSTOM
};

enum class EGPUIndexType : uint8_t {
    UINT16,
    UINT32
};

enum class EPixelFormat {
    UNKNOWN,
    R8,
    RG8,
    RGB8,
    RGBA8,
    BGRA8,
    SRGB8,
    SRGBA8,
    DEPTH16,
    DEPTH24,
    DEPTH32F,
    DEPTH24_STENCIL8,
    DEPTH32F_STENCIL8
};


#if defined(MOSS_GRAPHICS_VULKAN) || defined(MOSS_GRAPHICS_DIRECTX)
struct MeshShader;
struct TaskShader;
#endif

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

enum class ECommandQueue : uint8_t {
    GRAPHICS,
    COMPUTE,
    TRANSFER
};

enum class EGPUPresentMode : uint8_t {
    VSYNC,
    IMMEDIATE,
    MAILBOX
};

enum class EGPUSwapchainResult : uint8_t {
    SUCCESS,
    TIMEOUT,
    NOT_READY,
    SUBOPTIMAL,
    OUT_OF_DATE,
    LOST,
    UNSUPPORTED,
    ERROR
};

enum Moss_GPUShaderFormat : uint32_t {
    MOSS_GPU_SHADERFORMAT_NONE      = 0,
    MOSS_GPU_SHADERFORMAT_GLSL      = 1u << 0,
    MOSS_GPU_SHADERFORMAT_GLSL_ES   = 1u << 1,
    MOSS_GPU_SHADERFORMAT_SPIRV     = 1u << 2,
    MOSS_GPU_SHADERFORMAT_DXBC      = 1u << 3,
    MOSS_GPU_SHADERFORMAT_DXIL      = 1u << 4,
    MOSS_GPU_SHADERFORMAT_MSL       = 1u << 5,
    MOSS_GPU_SHADERFORMAT_METALLIB  = 1u << 6
};

enum Moss_GPUBackendType : uint8_t {
    MOSS_GPU_BACKEND_DEFAULT = 0,
    MOSS_GPU_BACKEND_OPENGL,
    MOSS_GPU_BACKEND_OPENGLES,
    MOSS_GPU_BACKEND_VULKAN,
    MOSS_GPU_BACKEND_DIRECTX12,
    MOSS_GPU_BACKEND_METAL,
    MOSS_GPU_BACKEND_FALLBACK
};

enum Moss_GPUBackendFlags : uint32_t {
    MOSS_GPU_BACKEND_FLAG_NONE = 0,
    MOSS_GPU_BACKEND_FLAG_COMPILED = 1u << 0,
    MOSS_GPU_BACKEND_FLAG_SUPPORTED_ON_PLATFORM = 1u << 1,
    MOSS_GPU_BACKEND_FLAG_REGISTERED_NATIVE = 1u << 2,
    MOSS_GPU_BACKEND_FLAG_FALLBACK_FUNCTION_TABLE = 1u << 3,
    MOSS_GPU_BACKEND_FLAG_WINDOW_PRESENT = 1u << 4,
    MOSS_GPU_BACKEND_FLAG_COMPUTE = 1u << 5,
    MOSS_GPU_BACKEND_FLAG_BINDLESS = 1u << 6,
    MOSS_GPU_BACKEND_FLAG_DEBUG_MARKERS = 1u << 7
};

struct Moss_GPUBackendInfo {
    Moss_GPUBackendType type = MOSS_GPU_BACKEND_DEFAULT;
    const char* name = nullptr;
    uint32_t shader_formats = MOSS_GPU_SHADERFORMAT_NONE;
    uint32_t flags = MOSS_GPU_BACKEND_FLAG_NONE;
};

enum class EShaderStage : uint32_t {
    NONE      = 0,
    VERTEX    = 1u << 0,
    FRAGMENT  = 1u << 1,
    COMPUTE   = 1u << 2,
    GEOMETRY  = 1u << 3,
    TESS_CTRL = 1u << 4,
    TESS_EVAL = 1u << 5,
    MESH      = 1u << 6,
    TASK      = 1u << 7
};

inline EShaderStage operator|(EShaderStage a, EShaderStage b) { return static_cast<EShaderStage>( static_cast<uint32_t>(a) | static_cast<uint32_t>(b)); }
inline EShaderStage operator&(EShaderStage a, EShaderStage b) { return static_cast<EShaderStage>( static_cast<uint32_t>(a) & static_cast<uint32_t>(b)); }

enum class EGPUBufferUsage : uint32_t {
    NONE        = 0,
    VERTEX      = 1u << 0,
    INDEX       = 1u << 1,
    UNIFORM     = 1u << 2,
    STORAGE     = 1u << 3,
    INDIRECT    = 1u << 4,
    TRANSFER_SRC= 1u << 5,
    TRANSFER_DST= 1u << 6
};
inline EGPUBufferUsage operator|(EGPUBufferUsage a, EGPUBufferUsage b) { return static_cast<EGPUBufferUsage>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b)); }

enum class ETextureUsage : uint32_t {
    NONE         = 0,
    SAMPLED      = 1u << 0,
    STORAGE      = 1u << 1,
    COLOR_TARGET = 1u << 2,
    DEPTH_TARGET = 1u << 3,
    TRANSFER_SRC = 1u << 4,
    TRANSFER_DST = 1u << 5
};
inline ETextureUsage operator|(ETextureUsage a, ETextureUsage b) { return static_cast<ETextureUsage>(static_cast<uint32_t>(a) | static_cast<uint32_t>(b)); }

enum class EColorWriteMask : uint8_t {
    NONE  = 0,
    RED   = 1 << 0,
    GREEN = 1 << 1,
    BLUE  = 1 << 2,
    ALPHA = 1 << 3,
    ALL   = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3)
};

//
enum class Moss_ShaderResourceType {
    UNIFORM_BUFFER,
    STORAGE_BUFFER,
    SAMPLED_TEXTURE,
    STORAGE_TEXTURE,
    SAMPLER,
    COMBINED_TEXTURE_SAMPLER,
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
    UNKNOWN = -1,
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
    LINEAR,
	POINT,
	ANISOTROPIC,
	LINEAR_MIPPOINT,
	POINT_MIPLINEAR,
	MINLINEAR_MAGPOINT_MIPLINEAR,
	MINLINEAR_MAGPOINT_MIPPOINT,
	MINPOINT_MAGLINEAR_MIPLINEAR,
	MINPOINT_MAGLINEAR_MIPPOINT
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
    UNDEFINED,
    PRESENT,
    COMMON,
    SHADER_READ,
    SHADER_WRITE,
    RENDER_TARGET,
    DEPTH_READ,
    DEPTH_WRITE,
    TRANSFER_SRC,
    TRANSFER_DST,
    VERTEX_BUFFER,
    INDEX_BUFFER,
    INDIRECT_ARGUMENT
};

enum class EResourceAccess : uint32_t {
    NONE = 0,
    READ = 1u << 0,
    WRITE = 1u << 1,
    READ_WRITE = (1u << 0) | (1u << 1)
};

enum class EGPUTextureViewType {
    DEFAULT,
    TEXTURE_2D,
    TEXTURE_2D_ARRAY,
    TEXTURE_3D,
    TEXTURE_CUBE,
    TEXTURE_CUBE_ARRAY
};

enum class EGPUQueryType {
    TIMESTAMP,
    OCCLUSION,
    PIPELINE_STATISTICS
};

enum class EGPUQueryResultFlags : uint32_t {
    NONE = 0,
    WAIT = 1u << 0,
    WITH_AVAILABILITY = 1u << 1,
    RESULT_64 = 1u << 2
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

enum class EVertexElementFormat {
	SINGLE,
	VECTOR2,
	VECTOR3,
	VECTOR4,
	COLOR,
	BYTE4,
	SHORT2,
	SHORT4,
	NORMALIZEDSHORT2,
	NORMALIZEDSHORT4,
	HALFVECTOR2,
	HALFVECTOR4
};

enum class EVertexElementUsage {
	POSITION,
	COLOR,
	TEXTURECOORDINATE,
	NORMAL,
	BINORMAL,
	TANGENT,
	BLENDINDICES,
	BLENDWEIGHT,
	DEPTH,
	FOG,
	POINTSIZE,
	SAMPLE,
	TESSELATEFACTOR
};

struct Moss_GPUStencilOpState {
    EStencilOp fail_op{};
    EStencilOp pass_op{};
    EStencilOp depth_fail_op{};
    ECompareOp compare_op{};
};

struct Moss_GPUBlitRegion {
    uint32_t src_mip_level = 0;
    uint32_t src_base_layer = 0;
    uint32_t src_layer_count = 1;

    uint32_t dst_mip_level = 0;
    uint32_t dst_base_layer = 0;
    uint32_t dst_layer_count = 1;

    uint32_t src_x = 0;
    uint32_t src_y = 0;
    uint32_t src_z = 0;

    uint32_t dst_x = 0;
    uint32_t dst_y = 0;
    uint32_t dst_z = 0;

    uint32_t width = 1;
    uint32_t height = 1;
    uint32_t depth = 1;
};

struct Moss_GPUBufferBinding {
    Moss_GPUBuffer* buffer = nullptr;
    uint64_t offset = 0;
};

struct Moss_GPUBufferLocation {
    Moss_GPUBuffer* buffer = nullptr;
    uint64_t offset = 0;
};

struct Moss_GPUBufferRegion {
    uint64_t offset = 0;
    uint64_t size = 0;
};

struct Moss_GPUBlitInfo {
    Moss_GPUTexture* src_texture = nullptr;
    Moss_GPUTexture* dst_texture = nullptr;
    Moss_GPUBlitRegion region{};
};

struct Moss_GPUColorTargetBlendState {
    bool enable_blend = false;

    EBlendFactor src_color_factor{};
    EBlendFactor dst_color_factor{};
    EBlendOp color_op{};

    EBlendFactor src_alpha_factor{};
    EBlendFactor dst_alpha_factor{};
    EBlendOp alpha_op{};
};

struct Moss_GPUColorTargetDescription {
    ETextureFormat format{};
    Moss_GPUColorTargetBlendState blend_state{};
};

struct Moss_GPUColorTargetInfo {
    Moss_GPUTexture* texture = nullptr;
    uint32_t mip_level = 0;
    uint32_t layer = 0;
};

struct Moss_GPUDepthStencilState {
    bool depth_test_enable = false;
    bool depth_write_enable = false;
    ECompareOp depth_compare_op{};

    bool stencil_test_enable = false;
    uint8_t stencil_read_mask = 0xFF;
    uint8_t stencil_write_mask = 0xFF;
    Moss_GPUStencilOpState front_face{};
    Moss_GPUStencilOpState back_face{};
};

struct Moss_GPUDepthStencilTargetInfo {
    Moss_GPUTexture* texture = nullptr;
    uint32_t mip_level = 0;
    uint32_t layer = 0;
};

struct Moss_GPUIndirectDrawCommand {
    uint32_t vertex_count = 0;
    uint32_t instance_count = 1;
    uint32_t first_vertex = 0;
    uint32_t first_instance = 0;
};

struct Moss_GPUIndexedIndirectDrawCommand {
    uint32_t index_count = 0;
    uint32_t instance_count = 1;
    uint32_t first_index = 0;
    int32_t  vertex_offset = 0;
    uint32_t first_instance = 0;
};

struct Moss_GPUIndirectDispatchCommand {
    uint32_t group_count_x = 0;
    uint32_t group_count_y = 0;
    uint32_t group_count_z = 0;
};

struct Moss_GPUMultisampleState {
    uint32_t sample_count = 1;
    uint32_t sample_mask = 0xFFFFFFFFu;
    bool alpha_to_coverage = false;
};

struct Moss_GPURasterizerState {
    EFillMode fill_mode{};
    ECullMode cull_mode{};
    EFrontFace front_face{};
    bool depth_clamp_enable = false;
};

struct Moss_GPUSamplerCreateInfo {
    ETextureFilter min_filter{};
    ETextureFilter mag_filter{};
    ETextureFilter mip_filter{};
    ETextureAddressMode address_u{};
    ETextureAddressMode address_v{};
    ETextureAddressMode address_w{};
    float mip_lod_bias = 0.0f;
    float min_lod = 0.0f;
    float max_lod = 1000.0f;
    bool enable_anisotropy = false;
    float max_anisotropy = 1.0f;
};

struct Moss_GPUShaderCreateInfo {
    EShaderStage stage{};
    const void* bytecode = nullptr;
    size_t bytecode_size = 0;
    uint32_t format = MOSS_GPU_SHADERFORMAT_NONE;
    const char* entry_point = "main";
    const char* debug_name = nullptr;
};

struct Moss_GPUComputePipelineCreateInfo {
    Moss_GPUShader* compute_shader = nullptr;
    const Moss_ResourceSetLayout* const* set_layouts = nullptr;
    uint32_t set_layout_count = 0;
    const char* debug_name = nullptr;
};

struct Moss_GPUTextureCreateInfo {
    ETextureType type{};
    ETextureFormat format{};
    ETextureUsage usage{};
    uint32_t width = 1;
    uint32_t height = 1;
    uint32_t depth = 1;
    uint32_t layers = 1;
    uint32_t mip_levels = 1;
    uint32_t sample_count = 1;
};

struct Moss_GPUTextureViewCreateInfo {
    Moss_GPUTexture* texture = nullptr;
    EGPUTextureViewType type = EGPUTextureViewType::DEFAULT;
    ETextureFormat format{};
    ETextureUsage usage = ETextureUsage::SAMPLED;
    uint32_t base_mip_level = 0;
    uint32_t mip_level_count = 1;
    uint32_t base_layer = 0;
    uint32_t layer_count = 1;
};

struct Moss_GPUTransferBufferCreateInfo {
    uint64_t size = 0;
};

struct Moss_GPUQueryPoolCreateInfo {
    EGPUQueryType type = EGPUQueryType::TIMESTAMP;
    uint32_t query_count = 1;
};

struct Moss_GPUTextureLocation {
    uint32_t mip_level = 0;
    uint32_t layer = 0;
    uint32_t x = 0;
    uint32_t y = 0;
    uint32_t z = 0;
};

struct Moss_GPUTextureRegion {
    uint32_t mip_level = 0;
    uint32_t base_layer = 0;
    uint32_t layer_count = 1;
    uint32_t x = 0;
    uint32_t y = 0;
    uint32_t z = 0;
    uint32_t width = 1;
    uint32_t height = 1;
    uint32_t depth = 1;
};

struct Moss_GPUTextureSubresourceRange {
    uint32_t base_mip_level = 0;
    uint32_t mip_level_count = 1;
    uint32_t base_layer = 0;
    uint32_t layer_count = 1;
};

struct Moss_GPUTextureSamplerBinding {
    Moss_GPUTexture* texture = nullptr;
    Moss_GPUSampler* sampler = nullptr;
};

struct Moss_GPUTextureTransferInfo {
    const void* data = nullptr;
    uint32_t row_pitch = 0;
    uint32_t slice_pitch = 0;
    Moss_GPUTextureRegion region{};
};

struct Moss_GPUTransferBufferLocation {
    Moss_GPUTransferBuffer* transfer_buffer = nullptr;
    uint64_t offset = 0;
};

struct Moss_GPUVertexAttribute {
    uint32_t location = 0;
    uint32_t binding = 0;
    ETextureFormat format{}; // if you already have a dedicated vertex format enum, use that instead
    uint32_t offset = 0;
};

struct Moss_GPUVertexBufferDescription {
    uint32_t binding = 0;
    uint32_t stride = 0;
    bool per_instance = false;
};

struct Moss_GPUVertexInputState {
    const Moss_GPUVertexAttribute* attributes = nullptr;
    uint32_t attribute_count = 0;

    const Moss_GPUVertexBufferDescription* buffers = nullptr;
    uint32_t buffer_count = 0;
};

struct GPUStorageBufferReadWriteBinding {
    Moss_GPUBuffer* buffer = nullptr;
    uint64_t offset = 0;
    uint64_t size = 0;
};

struct Moss_GPUStorageTextureReadWriteBinding {
    Moss_GPUTexture* texture = nullptr;
    uint32_t mip_level = 0;
    uint32_t layer = 0;
};

struct Moss_GPUTextureBinding {
    Moss_GPUTextureView* texture_view = nullptr;
};

struct Moss_GPUResourceBinding {
    uint32_t binding = 0;
    Moss_ShaderResourceType type = Moss_ShaderResourceType::UNIFORM_BUFFER;
    EShaderStage stage_mask = EShaderStage::NONE;
    Moss_GPUBufferBinding uniform_buffer{};
    GPUStorageBufferReadWriteBinding storage_buffer{};
    Moss_GPUTextureBinding sampled_texture{};
    GPUStorageBufferReadWriteBinding storage_texture{};
    Moss_GPUSampler* sampler = nullptr;
    Moss_GPUTextureSamplerBinding texture_sampler{};
};

struct Moss_GPUResourceSetLayoutBinding {
    uint32_t binding = 0;
    Moss_ShaderResourceType type = Moss_ShaderResourceType::UNIFORM_BUFFER;
    uint32_t count = 1;
    EShaderStage stage_mask = EShaderStage::NONE;
};

struct Moss_GPUResourceSetLayoutCreateInfo {
    uint32_t set_index = 0;
    const Moss_GPUResourceSetLayoutBinding* bindings = nullptr;
    uint32_t binding_count = 0;
    bool bindless = false;
};

struct Moss_GPUResourceSetCreateInfo {
    Moss_ResourceSetLayout* layout = nullptr;
    const Moss_GPUResourceBinding* bindings = nullptr;
    uint32_t binding_count = 0;
};

struct Moss_GPUFramebufferColorAttachment {
    Moss_GPUTextureView* texture_view = nullptr;
    ELoadOp load_op = ELoadOp::CLEAR;
    EStoreOp store_op = EStoreOp::STORE;
    float clear_color[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
};

struct Moss_GPUFramebufferDepthStencilAttachment {
    Moss_GPUTextureView* texture_view = nullptr;
    ELoadOp load_op = ELoadOp::CLEAR;
    EStoreOp store_op = EStoreOp::STORE;
    float clear_depth = 1.0f;
    uint32_t clear_stencil = 0;
};

struct Moss_GPUFramebufferCreateInfo {
    uint32_t width = 1;
    uint32_t height = 1;
    const Moss_GPUFramebufferColorAttachment* color_attachments = nullptr;
    uint32_t color_attachment_count = 0;
    const Moss_GPUFramebufferDepthStencilAttachment* depth_stencil_attachment = nullptr;
};

struct Moss_GPUTextureBarrier {
    Moss_GPUTexture* texture = nullptr;
    EResourceState old_state = EResourceState::UNDEFINED;
    EResourceState new_state = EResourceState::COMMON;
    EResourceAccess old_access = EResourceAccess::NONE;
    EResourceAccess new_access = EResourceAccess::READ_WRITE;
    Moss_GPUTextureSubresourceRange range{};
};

struct Moss_GPUBufferBarrier {
    Moss_GPUBuffer* buffer = nullptr;
    EResourceState old_state = EResourceState::UNDEFINED;
    EResourceState new_state = EResourceState::COMMON;
    EResourceAccess old_access = EResourceAccess::NONE;
    EResourceAccess new_access = EResourceAccess::READ_WRITE;
    uint64_t offset = 0;
    uint64_t size = 0;
};

struct Moss_GPUBarrierInfo {
    const Moss_GPUTextureBarrier* texture_barriers = nullptr;
    uint32_t texture_barrier_count = 0;
    const Moss_GPUBufferBarrier* buffer_barriers = nullptr;
    uint32_t buffer_barrier_count = 0;
};

struct Moss_GPUSwapchainCreateInfo {
    Moss_Window* window = nullptr;
    uint32_t width = 0;
    uint32_t height = 0;
    ETextureFormat format = ETextureFormat::RGBA8;
    EGPUPresentMode present_mode = EGPUPresentMode::VSYNC;
};

struct Moss_GPUSwapchainTexture {
    Moss_GPUTexture* texture = nullptr;
    EGPUSwapchainResult result = EGPUSwapchainResult::ERROR;
};

struct Moss_GPUOpenGLOptions {
    uint32_t major_version = 3;
    uint32_t minor_version = 3;
    bool core_profile = true;
    bool debug_context = false;
    bool forward_compatible = false;
};

struct Moss_GPUVulkanOptions {
    bool enable_validation = false;
    bool enable_synchronization_validation = false;
    bool enable_debug_markers = true;
    const char* const* required_instance_extensions = nullptr;
    uint32_t required_instance_extension_count = 0;
    const char* const* required_device_extensions = nullptr;
    uint32_t required_device_extension_count = 0;
};

struct Moss_GPUDirectX12Options {
    bool enable_debug_layer = false;
    bool enable_gpu_validation = false;
    bool enable_debug_markers = true;
    uint32_t node_mask = 0;
};

struct Moss_GPUMetalOptions {
    bool enable_validation = false;
    bool enable_debug_markers = true;
    bool prefer_low_power_device = false;
    bool use_argument_buffers = true;
};

struct Moss_GPUDeviceProperties {
    const char* driver_name = nullptr;
    const char* device_name = nullptr;
    const char* vendor_name = nullptr;
    uint32_t vendor_id = 0;
    uint32_t device_id = 0;

    uint64_t dedicated_video_memory = 0;
    uint64_t shared_system_memory = 0;

    uint32_t max_texture_dimension_2d = 0;
    uint32_t max_texture_array_layers = 0;
    uint32_t max_frames_in_flight = 0;

    bool supports_compute = true;
    bool supports_async_compute = false;
    bool supports_mesh_shaders = false;
    bool supports_bindless = false;
    bool supports_debug_markers = false;
};
struct Moss_GPUBufferDesc {
    uint64_t size;
    EGPUBufferUsage usage;
    int cpu_visible; /* staging vs device local */
};

struct Moss_VertexAttribute {
    uint32_t location;
    EVertexFormat format;
    uint32_t offset;
    EVertexInputRate rate;
};

struct Moss_ShaderResourceBinding {
    const char* name;

    Moss_ShaderResourceType type;
    EShaderStage stage_mask;

    uint32_t set;       /* Descriptor set / argument buffer */
    uint32_t binding;  /* Binding slot */
    uint32_t count;    /* Array size */

};

struct Moss_PipelineShaderStage {
    EShaderStage stage;
    Moss_Shader* shader;
};

struct Moss_ResourceSetLayoutBinding {
    Moss_ShaderResourceType type;
    uint32_t binding;
    uint32_t count;
    EShaderStage stage_mask;
};

struct Moss_TextureDesc {
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

struct Moss_GPUSamplerDesc {
    ETextureFilter min_filter;
    ETextureFilter mag_filter;
    ETextureFilter mip_filter;

    ETextureAddressMode address_u;
    ETextureAddressMode address_v;
    ETextureAddressMode address_w;

    float mip_lod_bias;
    float max_anisotropy;
};

struct Moss_TextureViewDesc {
    Moss_Texture* texture;
    ETextureFormat format;

    uint32_t base_mip;
    uint32_t mip_count;

    uint32_t base_layer;
    uint32_t layer_count;
};

struct Moss_TextureUploadDesc {
    const void* data;
    uint32_t width;
    uint32_t height;
    uint32_t depth;
    uint32_t mip_level;
    uint32_t base_layer;
    uint32_t layer_count;
    uint32_t row_pitch;
    uint32_t slice_pitch;
};

struct Moss_ResourceSetLayoutDesc {
    uint32_t set_index;

    const Moss_ResourceSetLayoutBinding* bindings;
    uint32_t binding_count;

    int bindless; /* enables bindless for this set */
};

struct Moss_RGTextureDesc {
    uint32_t width, height;
    ETextureFormat format;
    uint32_t mip_levels;
    uint32_t layers;
};

struct Moss_RGPassDesc {
    ERGPassType type;
    const char* name;

    void (*record)(Moss_GPUCommandBuffer* cmd, void* user_data);
    void* user_data;
};

struct Moss_StreamedTextureDesc {
    Moss_TextureDesc desc;
    uint32_t max_resident_mips;
};

struct Moss_FramebufferAttachmentDesc
{
    EAttachmentType type;
    ETextureFormat format;

    ELoadOp  load_op;
    EStoreOp store_op;

    float clear_color[4];   /* Only for color */
    float clear_depth;      /* Only for depth */
    uint32_t clear_stencil;

};

struct Moss_FramebufferDesc {
    uint32_t width;
    uint32_t height;

    const Moss_FramebufferAttachmentDesc* attachments;
    uint32_t attachment_count;

    int sampled; /* Can attachments be sampled in shaders? */
};

struct Moss_DepthStencilState {
    bool depth_test;
    bool depth_write;
    ECompareOp depth_compare;

    EStencilMode stencil;
};


using Moss_ShaderReloadCallback = void (*)(Moss_Shader* shader, void* user_data);
using Moss_SubViewportRecordFn = void (*)(Moss_GPUCommandBuffer* cmd, void* user_data);

MOSS_API Moss_GPUCommandBuffer* Moss_AcquireGPUCommandBuffer(Moss_GPUDevice* device);
MOSS_API void Moss_BeginGPUComputePass(Moss_GPUCommandBuffer* cmd);
MOSS_API void Moss_BeginGPUCopyPass(Moss_GPUCommandBuffer* cmd);
MOSS_API Moss_GPUBuffer* Moss_CreateGPUBuffer(Moss_GPUDevice* device, const Moss_GPUBufferDesc* desc);
MOSS_API void Moss_DestroyGPUDevice(Moss_GPUDevice* device);
MOSS_API void Moss_BeginGPURenderPass(Moss_GPUCommandBuffer* cmd, Moss_Framebuffer* framebuffer);
MOSS_API void Moss_BindGPUComputeSamplers(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, Moss_GPUSampler* const* samplers, uint32_t count);
MOSS_API void Moss_BindGPUComputeTextures(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, Moss_GPUTexture* const* textures, uint32_t texture_count);
MOSS_API void Moss_BindGPUComputeUniformBuffers(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, const Moss_GPUBufferBinding* bindings, uint32_t binding_count);
MOSS_API void Moss_BindGPUFragmentSamplers(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, Moss_GPUSampler* const* samplers, uint32_t sampler_count);
MOSS_API void Moss_BindGPUFragmentTextures(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, Moss_GPUTexture* const* textures, uint32_t texture_count);
MOSS_API void Moss_BindGPUFragmentUniformBuffers(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, const Moss_GPUBufferBinding* bindings, uint32_t binding_count);
MOSS_API void Moss_BindGPUIndexBuffer(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* buffer, uint64_t offset, EGPUIndexType index_type);
MOSS_API void Moss_BindGPUPipeline(Moss_GPUCommandBuffer* cmd, Moss_PipelineState* pipeline);
MOSS_API void Moss_BindGPUResourceSet(Moss_GPUCommandBuffer* cmd, uint32_t set_index, Moss_ResourceSet* set);
MOSS_API void Moss_BindGPUVertexBuffers(Moss_GPUCommandBuffer* cmd, uint32_t first_binding, const Moss_GPUBufferBinding* bindings, uint32_t binding_count);
MOSS_API void Moss_BindGPUVertexSamplers(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, Moss_GPUSampler* const* samplers, uint32_t sampler_count);
MOSS_API void Moss_BindGPUVertexTextures(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, Moss_GPUTexture* const* textures, uint32_t texture_count);
MOSS_API void Moss_BindGPUVertexUniformBuffers(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, const Moss_GPUBufferBinding* bindings, uint32_t binding_count);
MOSS_API void Moss_BlitGPUTexture(Moss_GPUCommandBuffer* cmd, Moss_Texture* src, Moss_Texture* dst, const Moss_GPUBlitRegion* region);
MOSS_API uint32_t Moss_CalculateGPUTextureFormatSize(ETextureFormat format, uint32_t width, uint32_t height, uint32_t depth);
MOSS_API void Moss_CancelGPUCommandBuffer(Moss_GPUCommandBuffer* cmd);
MOSS_API void Moss_CopyGPUBufferToBuffer(Moss_GPUCommandBuffer* cmd,Moss_GPUBuffer* src,Moss_GPUBuffer* dst,const Moss_GPUBufferRegion* src_region,const Moss_GPUBufferRegion* dst_region);
MOSS_API void Moss_CopyGPUBufferToTexture(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* src, Moss_GPUTexture* dst, const Moss_GPUBufferRegion* src_region, const Moss_GPUTextureRegion* dst_region);
MOSS_API void Moss_CopyGPUTextureToBuffer(Moss_GPUCommandBuffer* cmd, Moss_GPUTexture* src, Moss_GPUBuffer* dst, const Moss_GPUTextureRegion* src_region, const Moss_GPUBufferRegion* dst_region);
MOSS_API void Moss_CopyGPUTextureToTexture(Moss_GPUCommandBuffer* cmd, Moss_GPUTexture* src, Moss_GPUTexture* dst, const Moss_GPUTextureRegion* src_region, const Moss_GPUTextureRegion* dst_region);
MOSS_API Moss_GPUSampler* Moss_CreateGPUSampler( Moss_GPUDevice* device, const Moss_GPUSamplerCreateInfo* create_info);
MOSS_API Moss_GPUShader* Moss_CreateGPUShader(Moss_GPUDevice* device, const Moss_GPUShaderCreateInfo* create_info);
MOSS_API Moss_ComputePipelineState* Moss_CreateGPUComputePipeline(Moss_GPUDevice* device, const Moss_GPUComputePipelineCreateInfo* create_info);
MOSS_API Moss_GPUTexture* Moss_CreateGPUTexture(Moss_GPUDevice* device, const Moss_GPUTextureCreateInfo* create_info);
MOSS_API Moss_GPUTextureView* Moss_CreateGPUTextureView(Moss_GPUDevice* device, const Moss_GPUTextureViewCreateInfo* create_info);
MOSS_API Moss_GPUTransferBuffer* Moss_CreateGPUTransferBuffer(Moss_GPUDevice* device, const Moss_GPUTransferBufferCreateInfo* create_info);
MOSS_API Moss_Framebuffer* Moss_CreateGPUFramebuffer(Moss_GPUDevice* device, const Moss_GPUFramebufferCreateInfo* create_info);
MOSS_API Moss_ResourceSetLayout* Moss_CreateGPUResourceSetLayout(Moss_GPUDevice* device, const Moss_GPUResourceSetLayoutCreateInfo* create_info);
MOSS_API Moss_ResourceSet* Moss_CreateGPUResourceSet(Moss_GPUDevice* device, const Moss_GPUResourceSetCreateInfo* create_info);
MOSS_API Moss_GPUQueryPool* Moss_CreateGPUQueryPool(Moss_GPUDevice* device, const Moss_GPUQueryPoolCreateInfo* create_info);
MOSS_API void Moss_DispatchGPUCompute(Moss_GPUCommandBuffer* cmd, uint32_t group_count_x, uint32_t group_count_y, uint32_t group_count_z);
MOSS_API void Moss_DispatchGPUComputeIndirect(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* indirect_buffer, uint64_t offset);
MOSS_API void Moss_DownloadFromGPUBuffer(Moss_GPUDevice* device, Moss_GPUBuffer* src_buffer, void* dst_data, uint64_t size, uint64_t src_offset);
MOSS_API void Moss_DownloadFromGPUTexture(Moss_GPUDevice* device, Moss_GPUTexture* src_texture, const Moss_GPUTextureRegion* src_region, void* dst_data, uint32_t dst_row_pitch, uint32_t dst_slice_pitch);
MOSS_API void Moss_DrawGPUIndexedPrimitives(Moss_GPUCommandBuffer* cmd, uint32_t index_count, uint32_t instance_count, uint32_t first_index, int32_t vertex_offset, uint32_t first_instance);
MOSS_API void Moss_DrawGPUIndexedPrimitivesIndirect(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* indirect_buffer, uint64_t offset, uint32_t draw_count, uint32_t stride);
MOSS_API void Moss_DrawGPUPrimitives(Moss_GPUCommandBuffer* cmd, uint32_t vertex_count, uint32_t instance_count, uint32_t first_vertex, uint32_t first_instance);
MOSS_API void Moss_DrawGPUPrimitivesIndirect(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* indirect_buffer, uint64_t offset, uint32_t draw_count, uint32_t stride);
MOSS_API void Moss_EndGPUComputePass(Moss_GPUCommandBuffer* cmd);
MOSS_API void Moss_EndGPUCopyPass(Moss_GPUCommandBuffer* cmd);
MOSS_API void Moss_EndGPURenderPass(Moss_GPUCommandBuffer* cmd);
MOSS_API void Moss_GDKResumeGPU(Moss_GPUDevice* device);
MOSS_API void Moss_GDKSuspendGPU(Moss_GPUDevice* device);
MOSS_API void Moss_GenerateMipmapsForGPUTexture(Moss_GPUCommandBuffer* cmd, Moss_GPUTexture* texture);
MOSS_API const char* Moss_GetGPUDriver( uint32_t driver_index);
MOSS_API Moss_GPUBackendType Moss_GetCompiledGPUBackendType(void);
MOSS_API Moss_GPUBackendType Moss_GetGPUDeviceBackendType(Moss_GPUDevice* device);
MOSS_API const char* Moss_GPUBackendGetName(Moss_GPUBackendType backend);
MOSS_API uint32_t Moss_GPUBackendGetShaderFormats(Moss_GPUBackendType backend);
MOSS_API bool Moss_GPUBackendIsCompiled(Moss_GPUBackendType backend);
MOSS_API bool Moss_GPUBackendIsSupportedOnPlatform(Moss_GPUBackendType backend);
MOSS_API bool Moss_GetGPUBackendInfo(Moss_GPUBackendType backend, Moss_GPUBackendInfo* out_info);
/*! @brief Register a native backend function table for the selected API. Backend modules should call this during startup before device creation. @param backend Backend API to register. @param native_table Backend function table owned by the caller/static backend module. @return true when the table was accepted. */
MOSS_API bool Moss_RegisterNativeGPUBackend(Moss_GPUBackendType backend, const void* native_table);
/*! @brief Return whether a native backend table has been registered for an API. */
MOSS_API bool Moss_HasNativeGPUBackend(Moss_GPUBackendType backend);
MOSS_API uint32_t Moss_GetGPUShaderFormats( Moss_GPUDevice* device);
MOSS_API ETextureFormat Moss_GetGPUTextureFormatFromPixelFormat(EPixelFormat pixel_format);
MOSS_API uint32_t Moss_GetNumGPUDrivers(void);
MOSS_API EPixelFormat Moss_GetPixelFormatFromGPUTextureFormat(ETextureFormat texture_format);
MOSS_API bool Moss_GPUSupportsProperties(Moss_GPUDevice* device);
MOSS_API bool Moss_GPUSupportsShaderFormats(Moss_GPUDevice* device, uint32_t shader_format_mask);
MOSS_API uint32_t Moss_GPUTextureFormatTexelBlockSize( ETextureFormat format);
MOSS_API bool Moss_GPUTextureSupportsFormat(Moss_GPUDevice* device,ETextureFormat format, ETextureUsage usage);
MOSS_API bool Moss_GPUTextureSupportsSampleCount(Moss_GPUDevice* device,ETextureFormat format, uint32_t sample_count);
MOSS_API void Moss_InsertGPUDebugLabel(Moss_GPUCommandBuffer* cmd, const char* label);
MOSS_API void* Moss_MapGPUTransferBuffer(Moss_GPUDevice* device, Moss_GPUTransferBuffer* transfer_buffer);
MOSS_API void Moss_PopGPUDebugGroup(Moss_GPUCommandBuffer* cmd);
MOSS_API void Moss_PushGPUComputeUniformData(Moss_GPUCommandBuffer* cmd, uint32_t slot, const void* data, uint32_t size);
MOSS_API void Moss_PushGPUDebugGroup(Moss_GPUCommandBuffer* cmd, const char* label);
MOSS_API void Moss_PushGPUFragmentUniformData(Moss_GPUCommandBuffer* cmd, uint32_t slot, const void* data, uint32_t size);
MOSS_API void Moss_PushGPUVertexUniformData(Moss_GPUCommandBuffer* cmd, uint32_t slot, const void* data, uint32_t size);
MOSS_API bool Moss_QueryGPUFence(Moss_GPUDevice* device, Moss_GPUFence* fence);
MOSS_API void Moss_ReleaseGPUBuffer(Moss_GPUDevice* device, Moss_GPUBuffer* buffer);
MOSS_API void Moss_ReleaseGPUFence(Moss_GPUDevice* device, Moss_GPUFence* fence);
MOSS_API void Moss_ReleaseGPUSampler(Moss_GPUDevice* device, Moss_GPUSampler* sampler);
MOSS_API void Moss_ReleaseGPUShader(Moss_GPUDevice* device, Moss_GPUShader* shader);
MOSS_API void Moss_ReleaseGPUComputePipeline(Moss_GPUDevice* device, Moss_ComputePipelineState* pipeline);
MOSS_API void Moss_ReleaseGPUTexture(Moss_GPUDevice* device, Moss_GPUTexture* texture);
MOSS_API void Moss_ReleaseGPUTextureView(Moss_GPUDevice* device, Moss_GPUTextureView* texture_view);
MOSS_API void Moss_ReleaseGPUTransferBuffer(Moss_GPUDevice* device, Moss_GPUTransferBuffer* transfer_buffer);
MOSS_API void Moss_ReleaseGPUFramebuffer(Moss_GPUDevice* device, Moss_Framebuffer* framebuffer);
MOSS_API void Moss_ReleaseGPUResourceSetLayout(Moss_GPUDevice* device, Moss_ResourceSetLayout* layout);
MOSS_API void Moss_ReleaseGPUResourceSet(Moss_GPUDevice* device, Moss_ResourceSet* set);
MOSS_API void Moss_ReleaseGPUQueryPool(Moss_GPUDevice* device, Moss_GPUQueryPool* query_pool);
MOSS_API void Moss_SetGPUAllowedFramesInFlight(Moss_GPUDevice* device, uint32_t frames_in_flight);
MOSS_API void Moss_SetGPUBlendConstants(Moss_GPUCommandBuffer* cmd, float r, float g, float b, float a);
MOSS_API void Moss_SetGPUBufferName(Moss_GPUDevice* device, Moss_GPUBuffer* buffer, const char* name);
MOSS_API void Moss_SetGPUScissor(Moss_GPUCommandBuffer* cmd, const Moss_Rect* scissor);
MOSS_API void Moss_SetGPUStencilReference(Moss_GPUCommandBuffer* cmd,uint32_t reference);
MOSS_API void Moss_SetGPUTextureName(Moss_GPUDevice* device, Moss_GPUTexture* texture, const char* name);
MOSS_API void Moss_SetGPUViewport(Moss_GPUCommandBuffer* cmd, const Moss_GPUViewport* viewport);
MOSS_API void Moss_SubmitGPUCommandBuffer(Moss_GPUDevice* device, Moss_GPUCommandBuffer* cmd);
MOSS_API Moss_GPUFence* Moss_SubmitGPUCommandBufferAndAcquireFence(Moss_GPUDevice* device, Moss_GPUCommandBuffer* cmd);
MOSS_API void Moss_BindGPUComputePipeline(Moss_GPUCommandBuffer* cmd, Moss_ComputePipelineState* pipeline);
MOSS_API void Moss_BarrierGPUResources(Moss_GPUCommandBuffer* cmd, const Moss_GPUBarrierInfo* barrier_info);
MOSS_API void Moss_ResetGPUQueryPool(Moss_GPUCommandBuffer* cmd, Moss_GPUQueryPool* query_pool, uint32_t first_query, uint32_t query_count);
MOSS_API void Moss_WriteGPUTimestamp(Moss_GPUCommandBuffer* cmd, Moss_GPUQueryPool* query_pool, uint32_t query_index);
MOSS_API void Moss_BeginGPUQuery(Moss_GPUCommandBuffer* cmd, Moss_GPUQueryPool* query_pool, uint32_t query_index);
MOSS_API void Moss_EndGPUQuery(Moss_GPUCommandBuffer* cmd, Moss_GPUQueryPool* query_pool, uint32_t query_index);
MOSS_API bool Moss_GetGPUQueryResults(Moss_GPUDevice* device, Moss_GPUQueryPool* query_pool, uint32_t first_query, uint32_t query_count, void* data, uint64_t data_size, uint64_t stride, EGPUQueryResultFlags flags);
MOSS_API void Moss_TransitionGPUBuffer(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* buffer, EResourceState old_state, EResourceState new_state);
MOSS_API void Moss_TransitionGPUTexture(Moss_GPUCommandBuffer* cmd, Moss_GPUTexture* texture, EResourceState old_state, EResourceState new_state);
MOSS_API void Moss_UnmapGPUTransferBuffer(Moss_GPUDevice* device, Moss_GPUTransferBuffer* transfer_buffer);
MOSS_API void Moss_UploadToGPUBuffer(Moss_GPUDevice* device, Moss_GPUBuffer* dst_buffer, const void* src_data, uint64_t size, uint64_t dst_offset);
MOSS_API void Moss_UploadToGPUTexture(Moss_GPUDevice* device, Moss_GPUTexture* dst_texture, const Moss_GPUTextureTransferInfo* transfer_info);
MOSS_API void Moss_WaitForGPUFences(Moss_GPUDevice* device, Moss_GPUFence* const* fences, uint32_t fence_count, bool wait_all, uint64_t timeout_ns);
MOSS_API void Moss_WaitForGPUIdle(Moss_GPUDevice* device);
MOSS_API bool Moss_WindowSupportsGPUPresentMode(Moss_Window* window, EGPUPresentMode present_mode);

/*
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
*/

MOSS_API void Moss_BindGPUComputeStorageBuffers(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, const GPUStorageBufferReadWriteBinding* bindings, uint32_t binding_count);
MOSS_API void Moss_BindGPUComputeStorageTextures( Moss_GPUCommandBuffer* cmd, uint32_t first_slot, const Moss_GPUStorageTextureReadWriteBinding* bindings, uint32_t binding_count);
MOSS_API void Moss_BindGPUFragmentStorageBuffers(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, const GPUStorageBufferReadWriteBinding* bindings, uint32_t binding_count);
MOSS_API void Moss_BindGPUFragmentStorageTextures(Moss_GPUCommandBuffer* cmd, uint32_t first_slot,const Moss_GPUStorageTextureReadWriteBinding* bindings, uint32_t binding_count);
MOSS_API void Moss_BindGPUVertexStorageBuffers(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, const GPUStorageBufferReadWriteBinding* bindings, uint32_t binding_count);
MOSS_API void Moss_BindGPUVertexStorageTextures(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, const Moss_GPUStorageTextureReadWriteBinding* bindings, uint32_t binding_count);
//MOSS_API Moss_GPUStorageBufferReadWriteBinding();
//MOSS_API Moss_GPUStorageTextureReadWriteBinding();
MOSS_API const char* Moss_GetGPUDeviceDriver(Moss_GPUDevice* device);
MOSS_API const Moss_GPUDeviceProperties* Moss_GetGPUDeviceProperties(Moss_GPUDevice* device);
MOSS_API void Moss_ReleaseWindowFromGPUDevice(Moss_GPUDevice* device, Moss_Window* window);
MOSS_API bool Moss_ClaimWindowForGPUDevice(Moss_GPUDevice* device, Moss_Window* window);
MOSS_API void Moss_UpdateGPUResourceSet(Moss_GPUDevice* device, Moss_ResourceSet* set, const Moss_GPUResourceBinding* bindings, uint32_t binding_count);


struct Moss_GPUDeviceDesc {
    Moss_Window* window = nullptr;

    uint32_t backbuffer_width = 0;
    uint32_t backbuffer_height = 0;

    EGPUPresentMode present_mode = EGPUPresentMode::VSYNC;
    bool enable_validation = false;
#if defined(MOSS_DEBUG)
    bool enable_debug_markers = true;
#endif // MOSS_DEBUG
    Moss_GPUBackendType backend = MOSS_GPU_BACKEND_DEFAULT;
    const void* backend_options = nullptr;
};


MOSS_API Moss_GPUDevice* Moss_CreateGPUDevice(const Moss_GPUDeviceDesc* desc);
MOSS_API Moss_GPUDevice* Moss_CreateGPUDeviceWithProperties(const Moss_GPUDeviceDesc* desc, const Moss_GPUDeviceProperties* requested_properties);

MOSS_API Moss_GPUBuffer* Moss_GPUBufferCreate(Moss_GPUDevice* device, const Moss_GPUBufferDesc* desc);
MOSS_API void Moss_GPUBufferUpload(Moss_GPUDevice* device, Moss_GPUBuffer* buffer, const void* data, uint64_t size, uint64_t offset);
MOSS_API void* Moss_GPUBufferMap(Moss_GPUDevice* device, Moss_GPUBuffer* buffer);
MOSS_API void Moss_GPUBufferUnmap(Moss_GPUDevice* device, Moss_GPUBuffer* buffer);

MOSS_API void Moss_GPUBufferDestroy(Moss_GPUDevice* device, Moss_GPUBuffer* buffer);

MOSS_API bool Moss_AcquireGPUSwapchainTexture(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture** texture);
MOSS_API EGPUSwapchainResult Moss_AcquireGPUSwapchainTextureStatus(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture** texture);
MOSS_API ETextureFormat Moss_GetGPUSwapchainTextureFormat(Moss_GPUDevice* device, Moss_Window* window);
MOSS_API bool Moss_SetGPUSwapchainParameters(Moss_GPUDevice* device, Moss_Window* window, uint32_t width, uint32_t height, EGPUPresentMode present_mode);
MOSS_API EGPUSwapchainResult Moss_ResizeGPUSwapchain(Moss_GPUDevice* device, Moss_Window* window, uint32_t width, uint32_t height);
MOSS_API bool Moss_WaitAndAcquireGPUSwapchainTexture(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture** texture, uint64_t timeout_ns);
MOSS_API EGPUSwapchainResult Moss_WaitAndAcquireGPUSwapchainTextureStatus(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture** texture, uint64_t timeout_ns);
MOSS_API bool Moss_WaitForGPUSwapchain(Moss_GPUDevice* device, Moss_Window* window, uint64_t timeout_ns);
MOSS_API bool Moss_WindowSupportsGPUSwapchainComposition(Moss_Window* window);
MOSS_API bool Moss_PresentGPUSwapchain(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture* texture);
MOSS_API EGPUSwapchainResult Moss_PresentGPUSwapchainStatus(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture* texture);


MOSS_API void Moss_GPUClear(Moss_GPUCommandBuffer* cmd, float r, float g, float b, float a);


MOSS_API Moss_GPUSampler* Moss_GPUSamplerCreate(Moss_GPUDevice* renderer, const Moss_GPUSamplerDesc* desc);
MOSS_API void Moss_GPUSamplerDestroy(Moss_GPUDevice* device, Moss_GPUSampler* sampler);

MOSS_API Moss_Texture* Moss_TextureCreate(Moss_GPUDevice* device, const Moss_TextureDesc* desc);
MOSS_API void Moss_TextureDestroy(Moss_GPUDevice* device, Moss_Texture* texture);


MOSS_API void Moss_TextureUpload(Moss_GPUDevice* device, Moss_Texture* texture, const Moss_TextureUploadDesc* desc);
MOSS_API void Moss_TextureGenerateMips(Moss_GPUDevice* device, Moss_Texture* texture);

/* ======================================================
 * External GPU/Renderer Utility Libraries
 * =================================================== */
enum Moss_GPUExternalFeature : uint32_t {
    MOSS_GPU_EXTERNAL_FEATURE_FSR1 = 1u << 0,
    MOSS_GPU_EXTERNAL_FEATURE_FSR2 = 1u << 1,
    MOSS_GPU_EXTERNAL_FEATURE_MESHOPTIMIZER = 1u << 2,
    MOSS_GPU_EXTERNAL_FEATURE_SMOLV = 1u << 3,
    MOSS_GPU_EXTERNAL_FEATURE_VULKAN_MEMORY_ALLOCATOR = 1u << 4
};

struct Moss_GPUExternalFeatures {
    bool fsr1 = false;
    bool fsr2 = false;
    bool meshoptimizer = false;
    bool smolv = false;
    bool vulkan_memory_allocator = false;
};

enum Moss_FSR2QualityMode : uint32_t {
    MOSS_FSR2_QUALITY = 1,
    MOSS_FSR2_BALANCED = 2,
    MOSS_FSR2_PERFORMANCE = 3,
    MOSS_FSR2_ULTRA_PERFORMANCE = 4
};

struct Moss_FSR2RenderResolution {
    uint32_t width = 0;
    uint32_t height = 0;
};

MOSS_API uint32_t Moss_GPUGetExternalFeatureMask(void);
MOSS_API bool Moss_GPUExternalFeatureAvailable(Moss_GPUExternalFeature feature);
MOSS_API void Moss_GPUGetExternalFeatures(Moss_GPUExternalFeatures* out_features);
MOSS_API const char* Moss_GPUExternalFeatureName(Moss_GPUExternalFeature feature);

MOSS_API void Moss_FSR1BuildEasuConstants(uint32_t out_constants16[16], float input_viewport_width, float input_viewport_height, float input_width, float input_height, float output_width, float output_height);
MOSS_API float Moss_FSR2GetUpscaleRatio(Moss_FSR2QualityMode quality);
MOSS_API bool Moss_FSR2GetRenderResolution(uint32_t display_width, uint32_t display_height, Moss_FSR2QualityMode quality, Moss_FSR2RenderResolution* out_resolution);
MOSS_API int32_t Moss_FSR2GetJitterPhaseCount(int32_t render_width, int32_t display_width);
MOSS_API bool Moss_FSR2GetJitterOffset(float* out_x, float* out_y, int32_t index, int32_t phase_count);

MOSS_API size_t Moss_MeshGenerateVertexRemap(uint32_t* destination, const uint32_t* indices, size_t index_count, const void* vertices, size_t vertex_count, size_t vertex_size);
MOSS_API bool Moss_MeshOptimizeVertexCache(uint32_t* destination, const uint32_t* indices, size_t index_count, size_t vertex_count);
MOSS_API bool Moss_MeshOptimizeOverdraw(uint32_t* destination, const uint32_t* indices, size_t index_count, const float* vertex_positions, size_t vertex_count, size_t vertex_positions_stride, float threshold);
MOSS_API size_t Moss_MeshOptimizeVertexFetch(void* destination, uint32_t* indices, size_t index_count, const void* vertices, size_t vertex_count, size_t vertex_size);

MOSS_API bool Moss_SMOLVEncode(const void* spirv_data, size_t spirv_size, void* out_smolv_data, size_t* inout_smolv_size, uint32_t flags);
MOSS_API size_t Moss_SMOLVGetDecodedSize(const void* smolv_data, size_t smolv_size);
MOSS_API bool Moss_SMOLVDecode(const void* smolv_data, size_t smolv_size, void* out_spirv_data, size_t spirv_size, uint32_t flags);
MOSS_API bool Moss_GPUVulkanMemoryAllocatorAvailable(void);

struct Moss_GPUClothSolver;

struct Moss_GPUClothSolverDesc {
    Moss_ComputePipelineState* pipeline = nullptr;
    bool take_pipeline_ownership = false;
    Moss_GPUShaderCreateInfo shader{};
    Moss_GPUComputePipelineCreateInfo pipeline_info{};
    uint32_t group_size = 64;
};

struct Moss_GPUClothDispatchDesc {
    Moss_GPUBuffer* positions = nullptr;
    Moss_GPUBuffer* previous_positions = nullptr;
    Moss_GPUBuffer* velocities = nullptr;
    Moss_GPUBuffer* constraints = nullptr;
    uint64_t positions_size = 0;
    uint64_t previous_positions_size = 0;
    uint64_t velocities_size = 0;
    uint64_t constraints_size = 0;
    uint32_t particle_count = 0;
    uint32_t constraint_count = 0;
    uint32_t iterations = 1;
    float delta_time = 1.0f / 60.0f;
    float damping = 0.01f;
    float stiffness = 1.0f;
};

MOSS_API Moss_GPUClothSolver* Moss_GPUClothSolverCreate(Moss_GPUDevice* device, const Moss_GPUClothSolverDesc* desc);
MOSS_API void Moss_GPUClothSolverDestroy(Moss_GPUDevice* device, Moss_GPUClothSolver* solver);
MOSS_API bool Moss_GPUClothSolverDispatch(Moss_GPUCommandBuffer* cmd, Moss_GPUClothSolver* solver, const Moss_GPUClothDispatchDesc* desc);

struct Moss_TextureAssetDesc {
    const char* path = nullptr;
    ETextureFormat format = ETextureFormat::UNKNOWN;
    bool generate_mips = true;
};

struct Moss_ShaderAssetDesc {
    const char* path = nullptr;
    EShaderStage stage;
    const char* entry_point = nullptr;
};

//MOSS_API Moss_BindlessHandle Moss_GPUAssetLoadTexture(Moss_AssetManager* manager, Moss_GPUDevice* device, const Moss_TextureAssetDesc* desc);
//MOSS_API Moss_BindlessHandle Moss_GPUAssetLoadShader(Moss_AssetManager* manager, Moss_GPUDevice* device, const Moss_ShaderAssetDesc* desc);

//MOSS_API void Moss_DrawFullscreenTriangle(Moss_GPUDevice* device, Moss_GPUDevice*);



// GPU Commands

MOSS_API Moss_GPUCommandBuffer* Moss_GPUCommandBufferBegin(Moss_GPUDevice* device, ECommandQueue queue);
MOSS_API void Moss_GPUCommandBufferEnd(Moss_GPUCommandBuffer* cmd);
MOSS_API void Moss_GPUCommandBufferSubmit(Moss_GPUDevice* device, Moss_GPUCommandBuffer* cmd);
MOSS_API void Moss_CmdBeginRenderPass(Moss_GPUCommandBuffer* cmd, Moss_Framebuffer* fb);
MOSS_API void Moss_CmdEndRenderPass(Moss_GPUCommandBuffer* cmd);
MOSS_API void Moss_CmdBindPipeline(Moss_GPUCommandBuffer* cmd, Moss_PipelineState* pipeline);
MOSS_API void Moss_CmdBindResourceSet(Moss_GPUCommandBuffer* cmd, uint32_t set_index, Moss_ResourceSet* set);
MOSS_API void Moss_CmdBindVertexBuffer(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* buffer, uint64_t offset);
MOSS_API void Moss_CmdBindIndexBuffer(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* buffer, uint64_t offset);
MOSS_API void Moss_CmdDraw(Moss_GPUCommandBuffer* cmd, uint32_t vertex_count, uint32_t first_vertex);
MOSS_API void Moss_CmdDrawIndexed(Moss_GPUCommandBuffer* cmd, uint32_t index_count, uint32_t first_index, int32_t vertex_offset);
MOSS_API void Moss_CmdDispatch(Moss_GPUCommandBuffer* cmd, uint32_t x, uint32_t y, uint32_t z);



MOSS_API Moss_RGTexture* Moss_RGPassReadTexture(Moss_RGPass* pass, Moss_RGTexture* texture, EResourceState state);
MOSS_API Moss_RGTexture* Moss_RGPassWriteTexture(Moss_RGPass* pass, Moss_RGTexture* texture, EResourceState state);


MOSS_API int Moss_GPUDeviceSupportsAsyncCompute(Moss_GPUDevice* renderer);
MOSS_API void Moss_TextureSetResidency(Moss_Texture* texture, EResidencyState state);

// Shaders
struct Moss_ShaderDesc {
    EShaderStage stage;
    const char* path;
    const char* entry_point;
};

MOSS_API Moss_Shader* Moss_ShaderCreate(Moss_GPUDevice* device, const Moss_ShaderDesc* desc);
MOSS_API void Moss_ShaderDestroy(Moss_GPUDevice* device, Moss_Shader* shader);

// Bindless Textures
MOSS_API Moss_BindlessHandle Moss_BindlessRegisterTexture(Moss_GPUDevice* renderer, void* texture);
MOSS_API Moss_BindlessHandle Moss_BindlessRegisterBuffer(Moss_GPUDevice* renderer, Moss_GPUBuffer* buffer);
MOSS_API void Moss_BindlessUnregister(Moss_GPUDevice* renderer, Moss_BindlessHandle handle);


MOSS_API Moss_PipelineState* Moss_PipelineCreate(Moss_GPUDevice* device, const Moss_PipelineDesc* desc);
MOSS_API void Moss_PipelineDestroy(Moss_GPUDevice* device, Moss_PipelineState* pipeline);



#if defined(MOSS_ENABLE_MESHLETS) && (defined(MOSS_GRAPHICS_VULKAN) || defined(MOSS_GRAPHICS_DIRECTX))
/* ======================================================
 * Meshlet / Clustered Mesh Support
 * =================================================== */
// A single meshlet holds a small cluster of vertices and primitives
struct alignas(16) Moss_Meshlet {
    float center[3];
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
struct Moss_MeshletCluster {
    Moss_Meshlet* meshlets;
    uint32_t meshlet_count;

    uint32_t* draw_commands;   // optional multi-draw
    uint32_t draw_command_count;
};

// GPU-side buffers to hold meshlet data
struct Moss_MeshletBuffers {
    Moss_GPUBuffer* vertex_indices;  // packed vertex indices buffer
    Moss_GPUBuffer* primitives;      // triangle indices buffer
    Moss_GPUBuffer* meshlets;        // buffer of Moss_Meshlet structs
    uint32_t meshlet_count;

    // Optional: indirect draw buffer for GPU
    Moss_GPUBuffer* indirect_draws;
};
#endif // MOSS_GRAPHICS_VULKAN || MOSS_GRAPHICS_DIRECTX


#if defined(MOSS_GRAPHICS_VULKAN)
/*! @brief Window Resize for vulkan*/
MOSS_API void Moss_WindowResizeVK(VkPhysicalDevice physicalDevice, VkDevice device, VkSwapchainKHR& swapchain, VkExtent2D& swapchainExtent, 
    std::vector<VkFramebuffer>& swapchainFramebuffers, std::vector<VkImageView>& swapchainImageViews, VkImage& depthImage, VkDeviceMemory& depthImageMemory, VkImageView& depthImageView);
    //MOSS_API Moss_GPUVulkanOptions();
    //MOSS_API void Moss_GPUDeviceResizeSwapchain(Moss_GPUDevice* device, uint32_t width, uint32_t height);
#endif

// Debug / Utils
void* Moss_Result;
MOSS_API void Moss_GPUFatalError(Moss_Result result);


#endif // MOSS_GPU_H
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


using Moss_CullMask = uint32_t;
using Moss_LightMask = uint32_t;
using Moss_VisibleMask = uint32_t;


/* ======================================================
 * Forward Declerations
 * =================================================== */

struct Moss_Renderer;

struct Font;
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

struct Frustum2D;
struct Frustum3D;

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

#endif // MOSS_RENDERER_H
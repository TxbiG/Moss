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
 * Moss's low-level GPU abstractions and helper utilities.
 *
 * ### Forward-Looking Features:
 * - **Multithreaded Render Submission** - Asynchronous job-based rendering pipeline (future optimization).
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

#include <Moss/Moss_Platform.h>
#include <Moss/Moss_GPU.h>
#include <Moss/Variants/Color.h>
#include <Moss/Variants/Rect.h>
#include <Moss/Variants/Vector/Float2.h>
#include <Moss/Variants/Vector/Float3.h>
#include <Moss/Variants/Vector/Vec2.h>
#include <Moss/Variants/Vector/Vec3.h>
#include <Moss/Variants/Math/Real.h>

/* ======================================================
 * Forward Declerations
 * =================================================== */

struct Moss_Renderer;

struct Font;
struct SkyBox;
struct Viewport;
struct SubViewport;
struct FogVolume;
struct SurfaceInstance;
struct Texture;
struct Moss_Mesh;
struct Moss_Font;
struct Moss_Model;

struct Frustum2D { };
struct Frustum3D { };


using CullMask = uint32_t;
using LightMask = uint32_t;
using VisibleLayer = uint32_t;


struct Moss_RendererDesc {
    Moss_Window* window = nullptr;
    Moss_GPUDevice* gpu_device = nullptr;
    Color clear_color = Color(0.0f, 0.0f, 0.0f, 1.0f);
    uint32_t backbuffer_width = 0;
    uint32_t backbuffer_height = 0;
    uint32_t virtual_width = 0;
    uint32_t virtual_height = 0;
    EGPUPresentMode present_mode = EGPUPresentMode::VSYNC;
    bool enable_debug = false;
};

struct Moss_RenderFrameDesc {
    const void* camera = nullptr;
    float delta_time = 0.0f;
    float world_scale = 1.0f;
    Color clear_color = Color(0.0f, 0.0f, 0.0f, 1.0f);
};

struct Moss_DrawMeshDesc {
    Moss_Mesh* mesh = nullptr;
    Moss_PipelineState* pipeline = nullptr;
    Moss_ResourceSet* resource_set = nullptr;
    float transform[16];
    VisibleLayer visibility = 0xFFFFFFFFu;
};

struct Moss_DrawModelDesc {
    Moss_Model* model = nullptr;
    Moss_PipelineState* pipeline = nullptr;
    Moss_ResourceSet* resource_set = nullptr;
    float transform[16];
    VisibleLayer visibility = 0xFFFFFFFFu;
};

enum MaterialTextureType;

struct Moss_Material;
struct Moss_SpriteBatch;
struct Moss_DebugDrawList;
struct Moss_Camera;
struct Moss_ShadowMap;

struct Moss_MaterialTextureDesc {
    MaterialTextureType type;
    Texture* texture;
    Moss_GPUSampler* sampler = nullptr;
    uint32_t binding = 0;
};

struct Moss_MaterialUniformDesc {
    const char* name = nullptr;
    const void* data = nullptr;
    uint32_t size = 0;
    uint32_t binding = 0;
};

struct Moss_ShaderVariantDesc {
    const char* name = nullptr;
    const char* vertex_shader = nullptr;
    const char* fragment_shader = nullptr;
    const char* compute_shader = nullptr;
    uint32_t flags = 0;
};

struct Moss_MaterialDesc {
    const char* name = nullptr;
    const Moss_MaterialTextureDesc* textures = nullptr;
    uint32_t texture_count = 0;
    const Moss_MaterialUniformDesc* uniforms = nullptr;
    uint32_t uniform_count = 0;
    const Moss_ShaderVariantDesc* variants = nullptr;
    uint32_t variant_count = 0;
    Moss_PipelineState* pipeline = nullptr;
    Moss_ResourceSet* resource_set = nullptr;
};

struct Moss_PipelineCacheDesc {
    uint32_t max_pipelines = 256;
    bool allow_hot_reload = false;
};

struct Moss_FontGlyph {
    uint32_t codepoint = 0;
    float x0 = 0.0f;
    float y0 = 0.0f;
    float x1 = 0.0f;
    float y1 = 0.0f;
    float u0 = 0.0f;
    float v0 = 0.0f;
    float u1 = 0.0f;
    float v1 = 0.0f;
    float xadvance = 0.0f;
};

struct Moss_FontDesc {
    const char* path = nullptr;
    const void* data = nullptr;
    size_t data_size = 0;
    float pixel_size = 16.0f;
    uint32_t atlas_width = 1024;
    uint32_t atlas_height = 1024;
    uint32_t first_codepoint = 32;
    uint32_t codepoint_count = 96;
    bool upload_to_gpu = true;
};

struct Moss_FontMetrics {
    float pixel_size = 0.0f;
    float ascent = 0.0f;
    float descent = 0.0f;
    float line_gap = 0.0f;
    float line_height = 0.0f;
    float baseline = 0.0f;
};

struct Moss_TextMeasure {
    float width = 0.0f;
    float height = 0.0f;
};

struct Moss_DrawTextDesc {
    Moss_Font* font = nullptr;
    const char* text = nullptr;
    Float2 position = Float2(0.0f, 0.0f);
    Color color = Color(1.0f, 1.0f, 1.0f, 1.0f);
    float scale = 1.0f;
    float depth = 0.0f;
    VisibleLayer visibility = 0xFFFFFFFFu;
};

struct Moss_FontVertex {
    Float2 position = Float2(0.0f, 0.0f);
    Float2 uv = Float2(0.0f, 0.0f);
    Color color = Color(1.0f, 1.0f, 1.0f, 1.0f);
};

struct Moss_SpriteDesc {
    Texture* texture;
    Moss_Material* material = nullptr;
    Float2 position = Float2(0.0f, 0.0f);
    Float2 size = Float2(1.0f, 1.0f);
    Float2 origin = Float2(0.5f, 0.5f);
    Float2 uv_min = Float2(0.0f, 0.0f);
    Float2 uv_max = Float2(1.0f, 1.0f);
    Color color = Color(1.0f, 1.0f, 1.0f, 1.0f);
    float rotation = 0.0f;
    float depth = 0.0f;
    VisibleLayer visibility = 0xFFFFFFFFu;
};

struct Moss_DebugLineDesc {
    Float3 from = Float3(0.0f, 0.0f, 0.0f);
    Float3 to = Float3(0.0f, 0.0f, 0.0f);
    Color color = Color(1.0f, 1.0f, 1.0f, 1.0f);
    float thickness = 1.0f;
};

struct Moss_DebugBoxDesc {
    Float3 center = Float3(0.0f, 0.0f, 0.0f);
    Float3 size = Float3(1.0f, 1.0f, 1.0f);
    const float* transform = nullptr;
    Color color = Color(1.0f, 1.0f, 1.0f, 1.0f);
};

struct Moss_DebugSphereDesc {
    Float3 center = Float3(0.0f, 0.0f, 0.0f);
    float radius = 1.0f;
    Color color = Color(1.0f, 1.0f, 1.0f, 1.0f);
    uint32_t segments = 24;
};

struct Moss_DebugNavmeshDesc {
    const Float3* vertices = nullptr;
    uint32_t vertex_count = 0;
    const uint32_t* indices = nullptr;
    uint32_t index_count = 0;
    Color color = Color(0.1f, 0.8f, 0.3f, 1.0f);
};

struct Moss_DebugPhysicsContactDesc {
    Float3 position = Float3(0.0f, 0.0f, 0.0f);
    Float3 normal = Float3(0.0f, 1.0f, 0.0f);
    float impulse = 0.0f;
    Color color = Color(1.0f, 0.6f, 0.1f, 1.0f);
};

struct Moss_Camera2DDesc {
    Float2 position = Float2(0.0f, 0.0f);
    Float2 offset = Float2(0.0f, 0.0f);
    float zoom = 1.0f;
    float rotation = 0.0f;
    float viewport_width = 1.0f;
    float viewport_height = 1.0f;
};

struct Moss_Camera3DDesc {
    Float3 position = Float3(0.0f, 0.0f, 0.0f);
    Float3 target = Float3(0.0f, 0.0f, -1.0f);
    Float3 up = Float3(0.0f, 1.0f, 0.0f);
    float fov_degrees = 45.0f;
    float aspect_ratio = 1.0f;
    float near_plane = 0.1f;
    float far_plane = 1000.0f;
};

struct Moss_EditorCameraDesc {
    Moss_Camera3DDesc camera;
    float move_speed = 5.0f;
    float look_sensitivity = 0.2f;
    bool orbit_mode = false;
};

struct Moss_ShadowMapDesc {
    uint32_t width = 2048;
    uint32_t height = 2048;
    ETextureFormat depth_format = ETextureFormat::Depth32F;
    bool enable_cascades = false;
    uint32_t cascade_count = 1;
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


struct Moss_PostProcessPass {
    Moss_PipelineState* pipeline;
    Moss_ResourceSet* resource_set;
    Moss_Framebuffer* target;
};

struct Viewport {
    float x;
    float y;
    float width;
    float height;

    float min_depth;
    float max_depth;
};

struct SubViewport {
    Viewport viewport;
    Moss_Framebuffer* target;
};

struct Camera2D {
    RVec2 position;
	RVec2 offset;
    float zoom = 1.0f;
    float rotation = 0.0f;

    RMat44 u_viewProj;
	Frustum2D m_frustum;

    LightMask light_mask;
	VisibleLayer visible_layer;
};

struct Camera3D {
    RVec3 position;
	Vec3 up = Vec3(0.0f, 1.0f, 0.0f);    // Y is up
    Vec3 target;
	Quat Orientation = Quat::Identity();

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

struct Material2D {
    Color albedo;
    Texture* albedoMap;
    Texture* normalMap;
};

struct Material3D {
    Color albedo;
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
    Color color; 
    LightMask filter; 
};
// DirectionalLight2D
struct DirectionalLight2D { 
    float intensity, rotation; 
    Color color; 
    LightMask filter; 
};
// PointLight2D
struct PointLight2D { 
    float intensity, rotation, radius; 
    Float2 position; 
    Color color; 
    LightMask filter; 
};

struct TextureLight3D { 
    float intensity; 
    Texture* texture;
    Float3 position, rotation; 
    Color color; 
    LightMask filter;
};
struct DirectionalLight3D { 
    float intensity;
    Float3 rotation;
    Color color;
    LightMask filter;
};
struct SpotLight3D { 
    float intensity, radius, angle, penumbra; 
    Float3 position, rotation; 
    Color color; 
    LightMask filter; 
};
struct OmniLight3D { 
    float intensity, radius; 
    Float3 position; 
    Color color; 
    LightMask filter; 
};

struct SurfaceColor {
    Color color;
    Rect resource_set;
    Moss_PipelineState* pipeline;
};
struct SurfaceRect {
    Color color;
    Texture* texture;
    Rect resource_set;
    Moss_PipelineState* pipeline;
};

struct SurfaceRectInstance {
    Color color;
    Texture* texture;
    Rect resource_set;
    Moss_PipelineState* pipeline;
    float transform[16]; /* column-major mat4 */
};

struct Moss_Mesh {
    Material3D material;
    Moss_GPUBuffer* vertex_buffer;
    Moss_GPUBuffer* index_buffer;

    uint32_t vertex_count;
    uint32_t index_count;

    uint32_t vertex_stride;
};
struct MeshInstance {
    Moss_Mesh* mesh;
    float transform[16]; /* column-major mat4 */
    VisibleLayer visibility;
};

struct Moss_ModelNode { char name[128]{}; int32_t parent = -1; float local_transform[16]{}; float world_transform[16]{}; };
struct Moss_ModelBone { char name[128]{}; int32_t node_index = -1; float inverse_bind_matrix[16]{}; float pose_transform[16]{}; };
struct Moss_MorphTarget { char name[128]{}; float weight = 0.0f; };
struct Moss_ModelAnimation { char name[128]{}; float duration_seconds = 0.0f; float ticks_per_second = 0.0f; };
enum class Moss_ModelLoadError { None, InvalidArgument, FileNotFound, UnsupportedFormat, ImportFailed, OutOfMemory };
struct Moss_Model {
    Moss_Mesh** meshes = nullptr; uint32_t mesh_count = 0;
    Moss_ModelNode* nodes = nullptr; uint32_t node_count = 0;
    Moss_ModelBone* bones = nullptr; uint32_t bone_count = 0;
    Moss_MorphTarget* morph_targets = nullptr; uint32_t morph_target_count = 0;
    Moss_ModelAnimation* animations = nullptr; uint32_t animation_count = 0;
};

struct ModelInstance {
    Moss_Model* model;
    float transform[16];
    VisibleLayer visibility;
};

using Mesh = Moss_Mesh;
using Model = Moss_Model;

struct SkyBox {
    Texture* cubemap;
    Moss_GPUSampler* sampler;
    Moss_PipelineState* pipeline;
    Moss_ResourceSet* resource_set;
};

struct FogVolume {
    float transform[16];
    float density;
    float falloff;
    float height;
    Color color;
    CullMask cull_mask;
};

struct Sprite2D {
    Texture* texture;
    Rect rect;
    Moss_PipelineState* pipeline;
    float rotation;
    Float2 uv_min;
    Float2 uv_max;

    Material2D material;

    VisibleLayer visibility;
};

struct Sprite3D {
    Moss_Mesh* sampler;

    float transform[16];  /* world matrix */

    float size[2];
    Color color;

    VisibleLayer visibility;
};


struct Decal {
    Texture* Albedo;
    Texture* Normal;
    Texture* Orm;
    Texture* Emission;
    Color color;
    float emission_energy;
    float blendFactor = 1.0f;
    Mat44 model;
    CullMask filter;
};

MOSS_API Moss_Material* Moss_RendererCreateMaterial(Moss_Renderer* renderer, const Moss_MaterialDesc* desc);
MOSS_API void Moss_RendererDestroyMaterial(Moss_Renderer* renderer, Moss_Material* material);
MOSS_API int Moss_MaterialSetUniform(Moss_Material* material, const char* name, const void* data, uint32_t size);
MOSS_API int Moss_MaterialSetTexture(Moss_Material* material, MaterialTextureType type, Texture* texture, Moss_GPUSampler* sampler);

MOSS_API Moss_PipelineState* Moss_RendererCreatePipeline(Moss_Renderer* renderer, const Moss_PipelineDesc* desc);
MOSS_API void Moss_PipelineBind(Moss_PipelineState* pipeline);
MOSS_API void Moss_PipelineUnbind(Moss_PipelineState* pipeline);
MOSS_API void Moss_PipelineSetUniform(Moss_PipelineState* pipeline, const char* name, const void* data, uint32_t size);
MOSS_API void Moss_PipelineFlush(Moss_PipelineState* pipeline);
MOSS_API int Moss_PipelineValidate(Moss_PipelineState* pipeline);

MOSS_API Moss_ResourceSet* Moss_RendererCreateResourceSet(Moss_Renderer* renderer, const Moss_ResourceSetLayoutDesc* desc);
MOSS_API void Moss_RendererDestroyResourceSet(Moss_Renderer* renderer, Moss_ResourceSet* set);
MOSS_API void Moss_ResourceSetBindUniformBuffer(Moss_ResourceSet* set, uint32_t binding, Moss_GPUBuffer* buffer);
MOSS_API void Moss_ResourceSetBindUniformBufferRange(Moss_ResourceSet* set, uint32_t binding, Moss_GPUBuffer* buffer, size_t offset, size_t size);
MOSS_API void Moss_ResourceSetBindStorageBuffer(Moss_ResourceSet* set, uint32_t binding, Moss_GPUBuffer* buffer);
MOSS_API void Moss_ResourceSetBindTexture(Moss_ResourceSet* set, uint32_t binding, Texture* texture);
MOSS_API void Moss_ResourceSetBindSampler(Moss_ResourceSet* set, uint32_t binding, Moss_GPUSampler* sampler);
MOSS_API void Moss_ResourceSetBind(Moss_Renderer* renderer, Moss_PipelineState* pipeline, uint32_t set_index, Moss_ResourceSet* set);

MOSS_API Moss_Framebuffer* Moss_RendererCreateFramebuffer(Moss_Renderer* renderer, const Moss_FramebufferDesc* desc);
MOSS_API void Moss_RendererDestroyFramebuffer(Moss_Renderer* renderer, Moss_Framebuffer* framebuffer);
MOSS_API void Moss_FramebufferResize(Moss_Framebuffer* framebuffer, uint32_t width, uint32_t height);
MOSS_API void Moss_PostProcessExecute(Moss_Renderer* renderer, const Moss_PostProcessPass* pass);
MOSS_API void Moss_FramebufferBegin(Moss_Renderer* renderer, Moss_Framebuffer* framebuffer);
MOSS_API void Moss_FramebufferEnd(Moss_Renderer* renderer, Moss_Framebuffer* framebuffer);
MOSS_API void Moss_TextureBarrier(Moss_Renderer* renderer, Texture* texture, EResourceState old_state, EResourceState new_state);
MOSS_API Moss_Framebuffer* Moss_RendererGetBackbuffer(Moss_Renderer* renderer);
MOSS_API void Moss_RenderSubViewport(Moss_GPUCommandBuffer* cmd, const SubViewport* sub_viewport, Moss_SubViewportRecordFn record, void* user_data);

MOSS_API Moss_Renderer* Moss_RendererCreate(const Moss_RendererDesc* desc);
MOSS_API Moss_Renderer* Moss_CreateRenderer(Moss_Window* window);
MOSS_API void Moss_RendererDestroy(Moss_Renderer* renderer);
MOSS_API void Moss_TerminateRenderer(Moss_Renderer* renderer);
MOSS_API void Moss_RendererBeginFrame(Moss_Renderer* renderer);
MOSS_API void Moss_RendererBeginFrameEx(Moss_Renderer* renderer, const Moss_RenderFrameDesc* desc);
MOSS_API void Moss_RendererEndFrame(Moss_Renderer* renderer);
MOSS_API Moss_GPUDevice* Moss_RendererGetGPUDevice(Moss_Renderer* renderer);

MOSS_API Moss_Mesh* Moss_MeshLoadFBX(const char* file_path);
MOSS_API Moss_Mesh* Moss_MeshLoadOBJ(const char* file_path);
MOSS_API Moss_Mesh* Moss_MeshLoadGLB(const char* file_path);

MOSS_API Moss_Model* Moss_ModelLoadFBX(const char* file_path);
MOSS_API Moss_Model* Moss_ModelLoadOBJ(const char* file_path);
MOSS_API Moss_Model* Moss_ModelLoadGLB(const char* file_path);
MOSS_API Moss_Model* Moss_ModelLoad(const char* file_path);
MOSS_API Moss_ModelLoadError Moss_ModelGetLastLoadError();
MOSS_API const char* Moss_ModelGetLastLoadErrorMessage();
MOSS_API int Moss_ModelFindBone(const Moss_Model* model, const char* name);
MOSS_API int Moss_ModelFindMorphTarget(const Moss_Model* model, const char* name);
MOSS_API bool Moss_ModelSetBoneTransform(Moss_Model* model, uint32_t bone_index, const float* transform16);
MOSS_API bool Moss_ModelSetMorphWeight(Moss_Model* model, uint32_t morph_index, float weight);
struct Moss_MeshAssetDesc {
    const char* path = nullptr;
};

struct Moss_FontAssetDesc {
    const char* path = nullptr;
    float pixel_size = 16.0f;
};

//MOSS_API Moss_AssetHandle Moss_RendererAssetLoadMesh(Moss_AssetManager* manager, Moss_Renderer* renderer, const Moss_MeshAssetDesc* desc);
//MOSS_API Moss_Mesh* Moss_RendererAssetGetMesh(Moss_AssetManager* manager, Moss_AssetHandle handle);
//MOSS_API Moss_AssetHandle Moss_RendererAssetLoadFont(Moss_AssetManager* manager, Moss_Renderer* renderer, const Moss_FontAssetDesc* desc);
//MOSS_API Moss_Font* Moss_RendererAssetGetFont(Moss_AssetManager* manager, Moss_AssetHandle handle);

MOSS_API void Moss_RendererDrawMesh(Moss_Renderer* renderer, const Moss_DrawMeshDesc* desc);
MOSS_API void Moss_RendererDrawModelDesc(Moss_Renderer* renderer, const Moss_DrawModelDesc* desc);
MOSS_API void Moss_RendererDrawModel(Moss_Renderer* renderer, const Moss_Model* model, const float* transform);
MOSS_API Moss_Font* Moss_RendererCreateFont(Moss_Renderer* renderer, const Moss_FontDesc* desc);
MOSS_API void Moss_RendererDestroyFont(Moss_Renderer* renderer, Moss_Font* font);
MOSS_API Moss_Font* Moss_FontLoad(const char* path, float pixel_size);
MOSS_API Moss_Font* Moss_FontCreateFromMemory(const void* data, size_t data_size, float pixel_size);
MOSS_API void Moss_FontDestroy(Moss_Font* font);
MOSS_API const Moss_FontMetrics* Moss_FontGetMetrics(const Moss_Font* font);
MOSS_API const Moss_FontGlyph* Moss_FontGetGlyph(const Moss_Font* font, uint32_t codepoint);
MOSS_API const unsigned char* Moss_FontGetAtlasPixels(const Moss_Font* font, uint32_t* width, uint32_t* height, uint32_t* stride);
MOSS_API Texture* Moss_FontGetAtlasTexture(Moss_Font* font);
MOSS_API bool Moss_FontUploadAtlas(Moss_Renderer* renderer, Moss_Font* font);
MOSS_API Moss_TextMeasure Moss_FontMeasureText(const Moss_Font* font, const char* text, float scale);
MOSS_API size_t Moss_FontBuildTextQuads(const Moss_DrawTextDesc* desc, Moss_FontVertex* out_vertices, size_t vertex_capacity);
MOSS_API void Moss_RendererDrawText(Moss_Renderer* renderer, const Moss_DrawTextDesc* desc);
MOSS_API uint32_t Moss_RendererGetFontBackendMask(void);
MOSS_API bool Moss_RendererBackendSupportsFonts(Moss_GPUBackendType backend);
MOSS_API void Moss_RendererDrawSprite(Moss_Renderer* renderer, const Moss_SpriteDesc* desc);
MOSS_API Moss_SpriteBatch* Moss_RendererCreateSpriteBatch(Moss_Renderer* renderer, uint32_t capacity);
MOSS_API void Moss_RendererDestroySpriteBatch(Moss_Renderer* renderer, Moss_SpriteBatch* batch);
MOSS_API void Moss_SpriteBatchClear(Moss_SpriteBatch* batch);
MOSS_API int Moss_SpriteBatchAdd(Moss_SpriteBatch* batch, const Moss_SpriteDesc* desc);
MOSS_API void Moss_RendererDrawSpriteBatch(Moss_Renderer* renderer, const Moss_SpriteBatch* batch);

MOSS_API Moss_DebugDrawList* Moss_RendererCreateDebugDrawList(Moss_Renderer* renderer, uint32_t capacity);
MOSS_API void Moss_RendererDestroyDebugDrawList(Moss_Renderer* renderer, Moss_DebugDrawList* list);
MOSS_API void Moss_DebugDrawListClear(Moss_DebugDrawList* list);
MOSS_API int Moss_DebugDrawLine(Moss_DebugDrawList* list, const Moss_DebugLineDesc* desc);
MOSS_API int Moss_DebugDrawBox(Moss_DebugDrawList* list, const Moss_DebugBoxDesc* desc);
MOSS_API int Moss_DebugDrawSphere(Moss_DebugDrawList* list, const Moss_DebugSphereDesc* desc);
MOSS_API int Moss_DebugDrawNavmesh(Moss_DebugDrawList* list, const Moss_DebugNavmeshDesc* desc);
MOSS_API int Moss_DebugDrawPhysicsContact(Moss_DebugDrawList* list, const Moss_DebugPhysicsContactDesc* desc);
MOSS_API void Moss_RendererDrawDebugList(Moss_Renderer* renderer, const Moss_DebugDrawList* list);

MOSS_API Moss_Camera* Moss_CameraCreate2D(const Moss_Camera2DDesc* desc);
MOSS_API Moss_Camera* Moss_CameraCreate3D(const Moss_Camera3DDesc* desc);
MOSS_API Moss_Camera* Moss_CameraCreateEditor(const Moss_EditorCameraDesc* desc);
MOSS_API void Moss_CameraDestroy(Moss_Camera* camera);
MOSS_API void Moss_RendererSetCamera(Moss_Renderer* renderer, Moss_Camera* camera);

MOSS_API Moss_ShadowMap* Moss_RendererCreateShadowMap(Moss_Renderer* renderer, const Moss_ShadowMapDesc* desc);
MOSS_API void Moss_RendererDestroyShadowMap(Moss_Renderer* renderer, Moss_ShadowMap* shadow_map);
MOSS_API void Moss_RendererBeginShadowPass(Moss_Renderer* renderer, Moss_ShadowMap* shadow_map);
MOSS_API void Moss_RendererEndShadowPass(Moss_Renderer* renderer, Moss_ShadowMap* shadow_map);
MOSS_API void Moss_MeshDraw(Moss_Mesh* mesh);
MOSS_API void Moss_ModelDraw(Moss_Model* model);

MOSS_API void Moss_MeshRemove(Moss_Mesh* mesh);
MOSS_API void Moss_ModelRemove(Moss_Model* model);

// Compatibility aliases for the older renderer header names.
MOSS_API Moss_ResourceSet* Moss_ResourceSetCreate(Moss_Renderer* renderer, const Moss_ResourceSetLayoutDesc* desc);
MOSS_API void Moss_ResourceSetDestroy(Moss_ResourceSet* set);
MOSS_API Moss_Framebuffer* Moss_FramebufferCreate(Moss_Renderer* renderer, const Moss_FramebufferDesc* desc);
MOSS_API void Moss_FramebufferDestroy(Moss_Framebuffer* framebuffer);


enum Moss_Upscaler {
    MOSS_UPSCALER_NONE,
    MOSS_UPSCALER_FSR1,
    MOSS_UPSCALER_FSR2
};


struct Moss_RendererUpscalerDesc {
    Moss_Upscaler upscaler = MOSS_UPSCALER_NONE;
    Moss_FSR2QualityMode fsr2_quality = MOSS_FSR2_QUALITY;
    bool enable_sharpening = false;
    float sharpness = 0.0f;
};

MOSS_API void Moss_RendererSetUpscaler(Moss_Renderer* renderer, Moss_Upscaler upscaler);
MOSS_API void Moss_RendererSetUpscalerDesc(Moss_Renderer* renderer, const Moss_RendererUpscalerDesc* desc);


#if defined(MOSS_DEBUG_RENDERER) || !defined(NDEBUG)
/*! @brief Get the active physical GPU name for debug overlays. @return Backend-owned string, or null if unavailable. @ingroup Renderer Debug/Utils. */
MOSS_API char* Moss_GPUGetPhysicalDeviceName();
/*! @brief Get the active graphics API name for debug overlays. @return Backend-owned string, or null if unavailable. @ingroup Renderer Debug/Utils. */
MOSS_API char* Moss_GPUAPIGetName();
/*! @brief Get the active graphics API version string for debug overlays. @return Backend-owned string, or null if unavailable. @ingroup Renderer Debug/Utils. */
MOSS_API char* Moss_GPUAPIGetVersion();


/// Note that this class is meant to be a quick start for implementing a debug renderer, it is not the most efficient way to implement a debug renderer.
class MOSS_DEBUG_RENDERER_EXPORT DebugRendererSimple : public DebugRenderer
{
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
								DebugRendererSimple();

	/// Should be called every frame by the application to provide the camera position.
	/// This is used to determine the correct LOD for rendering.
	void						SetCameraPos(RVec3Arg inCameraPos)
	{
		mCameraPos = inCameraPos;
		mCameraPosSet = true;
	}

	/// Fallback implementation that uses DrawLine to draw a triangle (override this if you have a version that renders solid triangles)
	virtual void				DrawTriangle(RVec3Arg inV1, RVec3Arg inV2, RVec3Arg inV3, ColorArg inColor, ECastShadow inCastShadow) override
	{
		DrawLine(inV1, inV2, inColor);
		DrawLine(inV2, inV3, inColor);
		DrawLine(inV3, inV1, inColor);
	}

protected:
	/// Implementation of DebugRenderer interface
	virtual Batch				CreateTriangleBatch(const Triangle *inTriangles, int inTriangleCount) override;
	virtual Batch				CreateTriangleBatch(const Vertex *inVertices, int inVertexCount, const uint32 *inIndices, int inIndexCount) override;
	virtual void				DrawGeometry(RMat44Arg inModelMatrix, const AABox &inWorldSpaceBounds, float inLODScaleSq, ColorArg inModelColor, const GeometryRef &inGeometry, ECullMode inCullMode, ECastShadow inCastShadow, EDrawMode inDrawMode) override;

private:
	/// Implementation specific batch object
	class BatchImpl : public RefTargetVirtual
	{
	public:
		MOSS_OVERRIDE_NEW_DELETE

		virtual void			AddRef() override			{ ++mRefCount; }
		virtual void			Release() override			{ if (--mRefCount == 0) delete this; }

		TArray<Triangle>			mTriangles;

	private:
		atomic<uint32>			mRefCount = 0;
	};

	/// Last provided camera position
	RVec3						mCameraPos;
	bool						mCameraPosSet = false;
};

/// Implementation of DebugRenderer that records the API invocations to be played back later
class MOSS_EXPORT DebugRendererRecorder final : public DebugRenderer {
public:
	MOSS_OVERRIDE_NEW_DELETE

	/// Constructor
										DebugRendererRecorder(StreamOut &inStream) : mStream(inStream) { Initialize(); }

	/// Implementation of DebugRenderer interface
	virtual void						DrawLine(RVec3Arg inFrom, RVec3Arg inTo, ColorArg inColor) override;
	virtual void						DrawTriangle(RVec3Arg inV1, RVec3Arg inV2, RVec3Arg inV3, ColorArg inColor, ECastShadow inCastShadow) override;
	virtual Batch						CreateTriangleBatch(const Triangle *inTriangles, int inTriangleCount) override;
	virtual Batch						CreateTriangleBatch(const Vertex *inVertices, int inVertexCount, const uint32 *inIndices, int inIndexCount) override;
	virtual void						DrawGeometry(RMat44Arg inModelMatrix, const AABox &inWorldSpaceBounds, float inLODScaleSq, ColorArg inModelColor, const GeometryRef &inGeometry, ECullMode inCullMode, ECastShadow inCastShadow, EDrawMode inDrawMode) override;
	virtual void						DrawText3D(RVec3Arg inPosition, const string_view &inString, ColorArg inColor, float inHeight) override;

	/// Mark the end of a frame
	void								EndFrame();

	/// Control commands written into the stream
	enum class ECommand : uint8
	{
		CreateBatch,
		CreateBatchIndexed,
		CreateGeometry,
		EndFrame
	};

	/// Holds a single line segment
	struct LineBlob
	{
		RVec3							mFrom;
		RVec3							mTo;
		Color							mColor;
	};

	/// Holds a single triangle
	struct TriangleBlob
	{
		RVec3							mV1;
		RVec3							mV2;
		RVec3							mV3;
		Color							mColor;
		ECastShadow						mCastShadow;
	};

	/// Holds a single text entry
	struct TextBlob
	{
										TextBlob() = default;
										TextBlob(RVec3Arg inPosition, const string_view &inString, ColorArg inColor, float inHeight) : mPosition(inPosition), mString(inString), mColor(inColor), mHeight(inHeight) { }

		RVec3							mPosition;
		String							mString;
		Color							mColor;
		float							mHeight;
	};

	/// Holds a single geometry draw call
	struct GeometryBlob
	{
		RMat44							mModelMatrix;
		Color							mModelColor;
		uint32							mGeometryID;
		ECullMode						mCullMode;
		ECastShadow						mCastShadow;
		EDrawMode						mDrawMode;
	};

	/// All information for a single frame
	struct Frame
	{
		TArray<LineBlob>					mLines;
		TArray<TriangleBlob>				mTriangles;
		TArray<TextBlob>					mTexts;
		TArray<GeometryBlob>				mGeometries;
	};

private:
	/// Implementation specific batch object
	class BatchImpl : public RefTargetVirtual
	{
	public:
		MOSS_OVERRIDE_NEW_DELETE

										BatchImpl(uint32 inID)		: mID(inID) {  }

		virtual void					AddRef() override			{ ++mRefCount; }
		virtual void					Release() override			{ if (--mRefCount == 0) delete this; }

		atomic<uint32>					mRefCount = 0;
		uint32							mID;
	};

	/// Lock that prevents concurrent access to the internal structures
	Mutex								mMutex;

	/// Stream that recorded data will be sent to
	StreamOut &							mStream;

	/// Next available ID
	uint32								mNextBatchID = 1;
	uint32								mNextGeometryID = 1;

	/// Cached geometries and their IDs
	TMap<GeometryRef, uint32>	mGeometries;

	/// Data that is being accumulated for the current frame
	Frame								mCurrentFrame;
};

class MOSS_DEBUG_RENDERER_EXPORT DebugRendererPlayback
{
public:
	/// Constructor
										DebugRendererPlayback(DebugRenderer &inRenderer) : mRenderer(inRenderer) { }

	/// Parse a stream of frames
	void								Parse(StreamIn &inStream);

	/// Get the number of parsed frames
	uint32								GetNumFrames() const				{ return (uint32)mFrames.size(); }

	/// Draw a frame
	void								DrawFrame(uint32 inFrameNumber) const;

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

//===========================================================
// 2D Debug/Utils Rendering
//===========================================================
/*! @brief Queue a 2D debug line. @param renderer Renderer receiving the line. @param from Start point. @param to End point. @param color Line color. @param thickness Line thickness in pixels. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawLine2D(Moss_Renderer* renderer, const Vec2* from, const Vec2* to, Color color, float thickness);
/*! @brief Queue connected 2D debug line segments from a point array. @param renderer Renderer receiving the lines. @param positions Points interpreted as pairs or a strip by the backend. @param color Line color. @param thickness Line thickness in pixels. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawLines2D(Moss_Renderer* renderer, const TArray<Vec2>* positions, Color color, float thickness);
/*! @brief Queue a 2D debug box. @param renderer Renderer receiving the box. @param center Box center. @param size Box width and height. @param rotation Rotation in radians. @param color Line color. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawBox2D(Moss_Renderer* renderer, const Vec2* center, const Vec2* size, float rotation, Color color);
/*! @brief Queue a 2D debug circle. @param renderer Renderer receiving the circle. @param center Circle center. @param radius Circle radius. @param color Line color. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawCircle2D(Moss_Renderer* renderer, const Vec2* center, float radius, Color color);
/*! @brief Queue a 2D capsule/cylinder outline from a transform. @param renderer Renderer receiving the shape. @param matrix Shape transform. @param halfHeight Half cylinder height. @param radius End radius. @param color Line color. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawCylinder2D(Moss_Renderer* renderer, const RMat44* matrix, float halfHeight, float radius, Color color);
/*! @brief Queue a 2D debug triangle outline. @param renderer Renderer receiving the triangle. @param p0 First point. @param p1 Second point. @param p2 Third point. @param color Line color. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawTriangle2D(Moss_Renderer* renderer, const Vec2* p0, const Vec2* p1, const Vec2* p2, Color color);
/*! @brief Queue a 2D debug arrow. @param renderer Renderer receiving the arrow. @param from Tail point. @param to Tip point. @param color Line color. @param headSize Arrow head size. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawArrow2D(Moss_Renderer* renderer, const Vec2* from, const Vec2* to, Color color, float headSize);
/*! @brief Queue a 2D cross marker. @param renderer Renderer receiving the marker. @param position Marker center. @param color Line color. @param size Marker size. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawMarker2D(Moss_Renderer* renderer, const Vec2* position, Color color, float size);
/*! @brief Queue a 2D bone axis line from a transform. @param renderer Renderer receiving the bone. @param transform Bone transform. @param length Bone length. @param color Line color. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawBone2D(Moss_Renderer* renderer, const RMat44* transform, float length, Color color);
/*! @brief Queue 2D coordinate axes from a transform. @param renderer Renderer receiving the axes. @param matrix Axis transform. @param size Axis length. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawCoordinateSystem2D(Moss_Renderer* renderer, const RMat44* matrix, float size);
/*! @brief Queue a 2D capsule outline. @param renderer Renderer receiving the capsule. @param matrix Capsule transform. @param halfHeightOfCylinder Half height of the straight section. @param radius Capsule radius. @param color Line color. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawCapsule2D(Moss_Renderer* renderer, const RMat44* matrix, float halfHeightOfCylinder, float radius, Color color);
/*! @brief Queue a 2D convex polygon outline. @param renderer Renderer receiving the polygon. @param points Polygon points. @param point_count Number of points. @param color Line color. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawConvex2D(Moss_Renderer* renderer, const Vec2* points, uint32_t point_count, Color color);
/*! @brief Queue a 2D convex hull outline. @param renderer Renderer receiving the hull. @param points Hull points. @param point_count Number of points. @param color Line color. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawConvexHull2D(Moss_Renderer* renderer, const Vec2* points, uint32_t point_count, Color color);
/*! @brief Queue a 2D polygon outline. @param renderer Renderer receiving the polygon. @param points Polygon points. @param point_count Number of points. @param color Line color. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawPolygonShape2D(Moss_Renderer* renderer, const Vec2* points, uint32_t point_count, Color color);
/*! @brief Queue a 2D transform gizmo. @param renderer Renderer receiving the gizmo. @param transform Gizmo transform. @param size Axis size. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawGizmo2D(Moss_Renderer* renderer, const RMat44* transform, float size);

//===========================================================
// 3D Debug/Utils Rendering
//===========================================================
/*! @brief Queue a 3D debug line. @param renderer Renderer receiving the line. @param from Start point. @param to End point. @param color Line color. @param thickness Line thickness in pixels. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawLine3D(Moss_Renderer* renderer, const Vec3* from, const Vec3* to, Color color, float thickness);
/*! @brief Queue connected 3D debug line segments from a point array. @param renderer Renderer receiving the lines. @param positions Points interpreted as pairs or a strip by the backend. @param color Line color. @param thickness Line thickness in pixels. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawLines3D(Moss_Renderer* renderer, const TArray<Vec3>* positions, Color color, float thickness);
/*! @brief Queue a 3D debug box. @param renderer Renderer receiving the box. @param center Box center. @param size Box dimensions. @param rotation Optional orientation transform. @param color Line color. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawBox3D(Moss_Renderer* renderer, const Vec3* center, const Vec3* size, const RMat44* rotation, Color color);
/*! @brief Queue a 3D debug sphere. @param renderer Renderer receiving the sphere. @param center Sphere center. @param radius Sphere radius. @param color Line color. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawSphere3D(Moss_Renderer* renderer, const Vec3* center, float radius, Color color);
/*! @brief Queue a 3D cylinder outline. @param renderer Renderer receiving the cylinder. @param base Base center. @param top Top center. @param radius Cylinder radius. @param color Line color. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawCylinder3D(Moss_Renderer* renderer, const Vec3* base, const Vec3* top, float radius, Color color);
/*! @brief Queue a 3D cone outline. @param renderer Renderer receiving the cone. @param base Base center. @param tip Cone tip. @param radius Base radius. @param color Line color. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawCone3D(Moss_Renderer* renderer, const Vec3* base, const Vec3* tip, float radius, Color color);
/*! @brief Queue a 3D debug triangle outline. @param renderer Renderer receiving the triangle. @param p0 First point. @param p1 Second point. @param p2 Third point. @param color Line color. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawTriangle3D(Moss_Renderer* renderer, const Vec3* p0, const Vec3* p1, const Vec3* p2, Color color);
/*! @brief Queue a 3D debug arrow. @param renderer Renderer receiving the arrow. @param from Tail point. @param to Tip point. @param color Line color. @param headSize Arrow head size. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawArrow3D(Moss_Renderer* renderer, const Vec3* from, const Vec3* to, Color color, float headSize);
/*! @brief Queue a 3D cross marker. @param renderer Renderer receiving the marker. @param position Marker center. @param color Line color. @param size Marker size. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawMarker3D(Moss_Renderer* renderer, const Vec3* position, Color color, float size);
/*! @brief Queue a 3D debug plane outline. @param renderer Renderer receiving the plane. @param center Plane center. @param normal Plane normal. @param size Plane size. @param color Line color. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawPlane3D(Moss_Renderer* renderer, const Vec3* center, const Vec3* normal, float size, Color color);
/*! @brief Queue a 3D bone axis line from a transform. @param renderer Renderer receiving the bone. @param transform Bone transform. @param length Bone length. @param color Line color. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawBone3D(Moss_Renderer* renderer, const RMat44* transform, float length, Color color);
/*! @brief Queue 3D coordinate axes from a transform. @param renderer Renderer receiving the axes. @param matrix Axis transform. @param size Axis length. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawCoordinateSystem3D(Moss_Renderer* renderer, const RMat44* matrix, float size);
/*! @brief Queue a 3D capsule outline. @param renderer Renderer receiving the capsule. @param matrix Capsule transform. @param halfHeightOfCylinder Half height of the straight section. @param radius Capsule radius. @param color Line color. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawCapsule3D(Moss_Renderer* renderer, const RMat44* matrix, float halfHeightOfCylinder, float radius, Color color);
/*! @brief Queue a 3D convex polygon outline. @param renderer Renderer receiving the polygon. @param points Polygon points. @param point_count Number of points. @param color Line color. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawConvex3D(Moss_Renderer* renderer, const Vec3* points, uint32_t point_count, Color color);
/*! @brief Queue a 3D convex hull outline. @param renderer Renderer receiving the hull. @param points Hull points. @param point_count Number of points. @param color Line color. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawConvexHull3D(Moss_Renderer* renderer, const Vec3* points, uint32_t point_count, Color color);
/*! @brief Queue a 3D polygon outline. @param renderer Renderer receiving the polygon. @param points Polygon points. @param point_count Number of points. @param color Line color. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawPolygonShape3D(Moss_Renderer* renderer, const Vec3* points, uint32_t point_count, Color color);
/*! @brief Queue a 3D transform gizmo. @param renderer Renderer receiving the gizmo. @param transform Gizmo transform. @param size Axis size. @ingroup Renderer Debug/Utils. */
MOSS_API void Moss_RendererDrawGizmo3D(Moss_Renderer* renderer, const RMat44* transform, float size);
#endif


// GPU APIs / Tools
#if defined(MOSS_GRAPHICS_OPENGL) || defined(MOSS_GRAPHICS_OPENGLES)
#endif // MOSS_GRAPHICS_OPENGL || MOSS_GRAPHICS_OPENGLES


#if defined(MOSS_GRAPHICS_VULKAN)
struct BufferVK;
struct ConstantBufferVK;

MOSS_API VkDevice                       Moss_VulkanGetDevice(Moss_Renderer* renderer);
MOSS_API VkDescriptorPool               Moss_VulkanGetDescriptorPool(Moss_Renderer* renderer);
MOSS_API VkDescriptorSetLayout          Moss_VulkanGetDescriptorSetLayoutTexture(Moss_Renderer* renderer);
MOSS_API VkSampler                      Moss_VulkanGetTextureSamplerRepeat(Moss_Renderer* renderer);
MOSS_API VkSampler                      Moss_VulkanGetTextureSamplerShadow(Moss_Renderer* renderer);
MOSS_API VkRenderPass                   Moss_VulkanGetRenderPassShadow(Moss_Renderer* renderer);
MOSS_API VkRenderPass                   Moss_VulkanGetRenderPass(Moss_Renderer* renderer);
MOSS_API VkPipelineLayout               Moss_VulkanGetPipelineLayout(Moss_Renderer* renderer);
MOSS_API VkCommandBuffer                Moss_VulkanGetCommandBuffer(Moss_Renderer* renderer);
MOSS_API VkCommandBuffer                Moss_VulkanStartTempCommandBuffer(Moss_Renderer* renderer);
MOSS_API void                           Moss_VulkanEndTempCommandBuffer(Moss_Renderer* renderer, VkCommandBuffer command_buffer);
MOSS_API void                           Moss_VulkanAllocateMemory(Moss_Renderer* renderer, VkDeviceSize size, uint32_t memory_type_bits, VkMemoryPropertyFlags properties, VkDeviceMemory* out_memory);
MOSS_API void                           Moss_VulkanFreeMemory(Moss_Renderer* renderer, VkDeviceMemory memory, VkDeviceSize size);
MOSS_API void                           Moss_VulkanCreateBuffer(Moss_Renderer* renderer, VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties, BufferVK* out_buffer);
MOSS_API void                           Moss_VulkanCopyBuffer(Moss_Renderer* renderer, VkBuffer src, VkBuffer dst, VkDeviceSize size);
MOSS_API void                           Moss_VulkanCreateDeviceLocalBuffer(Moss_Renderer* renderer, const void* data, VkDeviceSize size, VkBufferUsageFlags usage, BufferVK* out_buffer);
MOSS_API void                           Moss_VulkanFreeBuffer(Moss_Renderer* renderer, BufferVK* buffer);
MOSS_API ConstantBufferVK*              Moss_VulkanCreateConstantBuffer(Moss_Renderer* renderer, VkDeviceSize buffer_size);
MOSS_API void                           Moss_VulkanCreateImage(Moss_Renderer* renderer, uint32_t width, uint32_t height, VkFormat format, VkImageTiling tiling, VkImageUsageFlags usage, VkMemoryPropertyFlags properties, VkImage* out_image, VkDeviceMemory* out_memory);
MOSS_API void                           Moss_VulkanDestroyImage(Moss_Renderer* renderer, VkImage image, VkDeviceMemory memory);
MOSS_API VkImageView                    Moss_VulkanCreateImageView(Moss_Renderer* renderer, VkImage image, VkFormat format, VkImageAspectFlags aspect_flags);
MOSS_API VkFormat                       Moss_VulkanFindDepthFormat(Moss_Renderer* renderer);

MOSS_API void Moss_Vulkan_EncodeSPIRV(const char* path);
MOSS_API void Moss_Vulkan_DecodeSPIRV(const char* path);
#endif // MOSS_GRAPHICS_VULKAN

#if defined(MOSS_GRAPHICS_DIRECTX)
struct ID3D12Device;
struct ID3D12RootSignature;
struct ID3D12GraphicsCommandList;
struct CommandQueueDX12;
struct DescriptorHeapDX12;

MOSS_API ID3D12Device*                  Moss_DX12GetDevice(Moss_Renderer* renderer);
MOSS_API ID3D12RootSignature*           Moss_DX12GetRootSignature(Moss_Renderer* renderer);
MOSS_API ID3D12GraphicsCommandList*     Moss_DX12GetCommandList(Moss_Renderer* renderer);
MOSS_API CommandQueueDX12*              Moss_DX12GetUploadQueue(Moss_Renderer* renderer);
MOSS_API DescriptorHeapDX12*            Moss_DX12GetDSVHeap(Moss_Renderer* renderer);
MOSS_API DescriptorHeapDX12*            Moss_DX12GetSRVHeap(Moss_Renderer* renderer);

MOSS_API bool Moss_DX_CompileHLSL(const char* inputPath, const char* outputPath, const char* entryPoint, const char* target);
MOSS_API void Moss_DX_EnableDebugLayer(bool enable); /* Debug */
MOSS_API uint32_t Moss_DX_GetShaderModel(void);      /* Info */
#endif // MOSS_GRAPHICS_DIRECTX

#if defined(MOSS_GRAPHICS_METAL)
MOSS_API MTKView*                       Moss_MetalGetView(Moss_Renderer* renderer);
MOSS_API id<MTLDevice>                  Moss_MetalGetDevice(Moss_Renderer* renderer);
MOSS_API id<MTLRenderCommandEncoder>    Moss_MetalGetRenderEncoder(Moss_Renderer* renderer);

MOSS_API bool Moss_Metal_CompileMSL(const char* inputPath, const char* outputPath); /* Shader tools */
MOSS_API bool Moss_Metal_SupportsFamily(uint32_t family); /* Capabilities */
#endif // MOSS_GRAPHICS_METAL

#endif // MOSS_RENDERER_H
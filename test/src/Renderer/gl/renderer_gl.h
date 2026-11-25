#ifndef MOSS_RENDERER_INTERNAL_H
#define MOSS_RENDERER_INTERNAL_H

#ifdef MOSS_GRAPHICS_OPENGL
#include <gl/glad.h>
#elif MOSS_GRAPHICS_OPENGLES
#include <gles/glad.h>
#endif
#include "../renderer_intern.h"

struct Moss_GPUBuffer {
    Moss_GPUDevice* device;
    GLuint handle;
    GLenum target;     // GL_ARRAY_BUFFER, GL_UNIFORM_BUFFER, etc
    uint64_t size;
    bool dynamic;
    void* mapped;
};

struct Moss_GPUDevice {
    GraphicsBackend backend;   // OPENGL / OPENGLES
    uint32_t frameIndex;
    uint32_t framesInFlight;
    bool supportsCompute;
    bool supportsInstancing;
    bool supportsMSAA;
    uint32_t maxTextureUnits;
    uint32_t maxUniformBuffers;
    // OpenGL context info
    void* context;     // EGLContext / GLFWwindow*
};


struct Moss_Shader {
    Moss_GPUDevice* device;
    Moss_ShaderStage stage;
    GLuint shaderHandle;   // GL_VERTEX_SHADER, etc
    const char* entryPoint; // unused in GL
};

// ---------------------------------------------------------------------------
// Pipeline (see renderer_gl_pipeline.cpp)
// ---------------------------------------------------------------------------
#include <string>
#include <vector>
#include <unordered_map>

// Vertex / instance inputs a pipeline can consume. Attribute locations are
// assigned in declaration order starting at 0 (the matrix inputs take 4), so
// shaders must use matching `layout(location = N)` indices. Per-vertex data is
// tightly packed in declaration order:
//   Position float3 (12B) | Normal float3 (12B) | TexCoord float2 (8B) | Color ubyte4 normalised (4B)
// and per-instance data likewise:
//   InstanceColor ubyte4 (4B) | InstanceTransform mat4 (64B) | InstanceInvTransform mat4 (64B)
enum class Moss_GLInput : uint8_t {
    Position, Normal, TexCoord, Color,
    InstanceColor, InstanceTransform, InstanceInvTransform
};

struct Moss_GLVertexAttrib {
    GLuint     location   = 0;
    GLint      components = 0;
    GLenum     type       = GL_FLOAT;
    GLboolean  normalized = GL_FALSE;
    uint32_t   offset     = 0;      // byte offset inside its stream
    GLuint     divisor    = 0;      // 0 = per vertex, 1 = per instance
};

struct Moss_GLUniform {
    GLint     location = -1;
    GLenum    type     = 0;
    GLint     count    = 1;         // array length
    uint32_t  elemSize = 0;         // bytes per element
    uint32_t  offset   = 0;         // into Moss_PipelineState::shadow
    bool      dirty    = false;
    bool      written  = false;     // false until first SetUniform
};

// Everything needed to create a pipeline. Defaults give opaque, depth-tested,
// back-face-culled triangles.
struct Moss_GLPipelineInfo {
    GLuint vertexShader   = 0;      // compiled GL shader objects, not owned
    GLuint fragmentShader = 0;
    const Moss_GLInput* inputs = nullptr;
    uint32_t inputCount   = 0;

    GLenum topology       = GL_TRIANGLES;

    bool   depthTest      = true;
    bool   depthWrite     = true;
    GLenum depthFunc      = GL_LEQUAL;

    bool   blendEnable    = false;
    GLenum blendSrcColor  = GL_SRC_ALPHA;
    GLenum blendDstColor  = GL_ONE_MINUS_SRC_ALPHA;
    GLenum blendSrcAlpha  = GL_ONE;
    GLenum blendDstAlpha  = GL_ONE_MINUS_SRC_ALPHA;
    GLenum blendEquation  = GL_FUNC_ADD;

    bool   cullEnable     = true;
    GLenum cullFace       = GL_BACK;
    GLenum frontFace      = GL_CCW;

    bool   wireframe      = false;  // desktop GL only; ignored on GLES
    bool   colorWrite     = true;
};

struct Moss_PipelineState {
    Moss_GPUDevice* device = nullptr;
    GLuint program = 0;

    // Fixed-function state, re-applied in full by Moss_PipelineBind
    GLenum topology = GL_TRIANGLES;
    bool   depthTest = true, depthWrite = true;
    GLenum depthFunc = GL_LEQUAL;
    bool   blending = false;
    GLenum blendSrcColor = GL_SRC_ALPHA, blendDstColor = GL_ONE_MINUS_SRC_ALPHA;
    GLenum blendSrcAlpha = GL_ONE,       blendDstAlpha = GL_ONE_MINUS_SRC_ALPHA;
    GLenum blendEquation = GL_FUNC_ADD;
    bool   cullFace = true;
    GLenum cullMode = GL_BACK, frontFace = GL_CCW;
    bool   wireframe = false;
    bool   colorWrite = true;

    // Vertex layout
    Moss_GLVertexAttrib attribs[16];
    uint32_t attribCount = 0;
    uint32_t vertexStride = 0;      // bytes, packed per-vertex stream
    uint32_t instanceStride = 0;    // bytes, packed per-instance stream

    // Uniform reflection + CPU shadow copy (uploaded on Bind / Flush)
    std::vector<Moss_GLUniform> uniforms;
    std::unordered_map<std::string, uint32_t> uniformLookup;
    std::vector<uint8_t> shadow;
    bool anyDirty = false;
};

// Internal GL-backend pipeline API
GLuint MossGL_CompileShader(GLenum stage, const char* source);   // 0 on failure, logs the info log
Moss_PipelineState* MossGL_CreatePipeline(Moss_GPUDevice* device, const Moss_GLPipelineInfo* info);
void   MossGL_DestroyPipeline(Moss_PipelineState* pipeline);
GLenum MossGL_PipelineGetTopology(const Moss_PipelineState* pipeline);
// Call with the target VAO bound; wires the pipeline's attributes to the buffers.
// vertexStride == 0 uses the packed stride computed from the input list.
void   MossGL_PipelineApplyVertexLayout(const Moss_PipelineState* pipeline, GLuint vertexBuffer, GLsizei vertexStride, GLuint instanceBuffer);

struct Moss_Framebuffer {
    Moss_GPUDevice* device;

    GLuint fbo;

    uint32_t width;
    uint32_t height;

    GLuint colorAttachments[8];
    uint32_t colorCount;

    GLuint depthAttachment;
};

#define MOSS_MAX_BINDINGS 16

struct Moss_ResourceSet {
    Moss_GPUDevice* device;

    uint32_t bufferCount;
    Moss_GPUBuffer* buffers[MOSS_MAX_BINDINGS];
    uint32_t bufferBindings[MOSS_MAX_BINDINGS];

    uint32_t textureCount;
    Moss_TextureView* textures[MOSS_MAX_BINDINGS];
    Moss_GPUSampler* samplers[MOSS_MAX_BINDINGS];
};

struct Moss_TextureView {
    Moss_GPUDevice* device;

    GLuint texture;
    GLenum target;     // GL_TEXTURE_2D, GL_TEXTURE_2D_ARRAY

    uint32_t width;
    uint32_t height;
    uint32_t mipLevels;

    GLenum format;
};

struct Moss_GPUSampler {
    GLuint sampler; // 0 if embedded

    GLenum minFilter;
    GLenum magFilter;
    GLenum wrapU;
    GLenum wrapV;

    float max_anisotropy;
    float lod_bias;
};


struct Moss_RenderGraph {
    Moss_GPUDevice* device;
    Moss_RGPass** passes;
    uint32_t passCount;
};

struct Moss_RGPass {
    const char* name;
    Moss_Framebuffer* framebuffer;
    void (*execute)(void*);
};

typedef struct Moss_RGTexture Moss_RGTexture;
typedef struct Moss_RGBuffer Moss_RGBuffer;
struct Moss_GPUCommandBuffer {
    Moss_GPUDevice* device;
    bool recording;
};
/*
struct Moss_Renderer {
    Moss_Window* window;

    // -------------------------------------------------
    // Renderer identity
    // -------------------------------------------------
    RendererType rendererType;        // Mobile
    GraphicsBackend backend;          // GLES / Vulkan / Metal
    uint32_t frameIndex;
    uint32_t framesInFlight;

    // -------------------------------------------------
    // Resolution & scaling
    // -------------------------------------------------
    uint32_t outputWidth;
    uint32_t outputHeight;

    uint32_t internalWidth;           // For upscaling
    uint32_t internalHeight;

    // -------------------------------------------------
    // Anti-aliasing / Upscaling
    // -------------------------------------------------
    AntiAliasing aaMode;
    bool enableFSR1;
    float fsrSharpness;

    // History index (used by TAA / FSR2 later)
    uint32_t historyIndex;

    // -------------------------------------------------
    // Render targets (API-agnostic)
    // -------------------------------------------------
    Moss_Framebuffer* mainFramebuffer;        // Scene render target
    Moss_Framebuffer* resolveFramebuffer;     // MSAA resolve
    Moss_Framebuffer* fsrFramebuffer;         // FSR intermediate

    Moss_TextureView* mainColor;
    Moss_TextureView* mainDepth;
    Moss_TextureView* motionVectors;          // optional
    Moss_TextureView* historyColor[2];        // ping-pong

    // -------------------------------------------------
    // Pipelines
    // -------------------------------------------------
    Moss_PipelineState* fullscreenPipeline;   // blit / post
    Moss_PipelineState* fsrEASUPipeline;
    Moss_PipelineState* fsrRCASPipeline;

    // -------------------------------------------------
    // Resource binding
    // -------------------------------------------------
    Moss_ResourceSet* globalSet;     // camera, time, frame data
    Moss_ResourceSet* materialSet;   // per-material bindings

    // -------------------------------------------------
    // Post-processing
    // -------------------------------------------------
    PostProcessingPipeline postPipeline;

    // -------------------------------------------------
    // Render graph (optional but recommended)
    // -------------------------------------------------
    Moss_RenderGraph* renderGraph;

    // -------------------------------------------------
    // Render state defaults
    // -------------------------------------------------
    bool enableDepthTest;
    bool enableAlphaBlending;
    bool enableCullFace;

    // -------------------------------------------------
    // Backend-private pointer
    // -------------------------------------------------
    void* backendData; // points to GLES/Vulkan/Metal renderer impl
};
*/
// Vertex / instance inputs a pipeline can consume. Attribute locations are
// assigned in declaration order starting at 0 (the matrix inputs take 4), so
// shaders must use matching `layout(location = N)` indices. Per-vertex data is
// tightly packed in declaration order:
//   Position float3 (12B) | Normal float3 (12B) | TexCoord float2 (8B) | Color ubyte4 normalised (4B)
// and per-instance data likewise:
//   InstanceColor ubyte4 (4B) | InstanceTransform mat4 (64B) | InstanceInvTransform mat4 (64B)
enum class Moss_GLInput : uint8_t {
    Position, Normal, TexCoord, Color,
    InstanceColor, InstanceTransform, InstanceInvTransform
};

struct Moss_GLVertexAttrib {
    GLuint     location   = 0;
    GLint      components = 0;
    GLenum     type       = GL_FLOAT;
    GLboolean  normalized = GL_FALSE;
    uint32_t   offset     = 0;      // byte offset inside its stream
    GLuint     divisor    = 0;      // 0 = per vertex, 1 = per instance
};

struct Moss_GLUniform {
    GLint     location = -1;
    GLenum    type     = 0;
    GLint     count    = 1;         // array length
    uint32_t  elemSize = 0;         // bytes per element
    uint32_t  offset   = 0;         // into Moss_PipelineState::shadow
    bool      dirty    = false;
    bool      written  = false;     // false until first SetUniform
};

// Everything needed to create a pipeline. Defaults give opaque, depth-tested,
// back-face-culled triangles.
struct Moss_GLPipelineInfo {
    GLuint vertexShader   = 0;      // compiled GL shader objects, not owned
    GLuint fragmentShader = 0;
    const Moss_GLInput* inputs = nullptr;
    uint32_t inputCount   = 0;

    GLenum topology       = GL_TRIANGLES;

    bool   depthTest      = true;
    bool   depthWrite     = true;
    GLenum depthFunc      = GL_LEQUAL;

    bool   blendEnable    = false;
    GLenum blendSrcColor  = GL_SRC_ALPHA;
    GLenum blendDstColor  = GL_ONE_MINUS_SRC_ALPHA;
    GLenum blendSrcAlpha  = GL_ONE;
    GLenum blendDstAlpha  = GL_ONE_MINUS_SRC_ALPHA;
    GLenum blendEquation  = GL_FUNC_ADD;

    bool   cullEnable     = true;
    GLenum cullFace       = GL_BACK;
    GLenum frontFace      = GL_CCW;

    bool   wireframe      = false;  // desktop GL only; ignored on GLES
    bool   colorWrite     = true;
};

// Internal GL-backend pipeline API
GLuint MossGL_CompileShader(GLenum stage, const char* source);   // 0 on failure, logs the info log
Moss_PipelineState* MossGL_CreatePipeline(Moss_GPUDevice* device, const Moss_GLPipelineInfo* info);
void   MossGL_DestroyPipeline(Moss_PipelineState* pipeline);
GLenum MossGL_PipelineGetTopology(const Moss_PipelineState* pipeline);
// Call with the target VAO bound; wires the pipeline's attributes to the buffers.
// vertexStride == 0 uses the packed stride computed from the input list.
void   MossGL_PipelineApplyVertexLayout(const Moss_PipelineState* pipeline, GLuint vertexBuffer, GLsizei vertexStride, GLuint instanceBuffer);

struct Moss_Framebuffer {
    Moss_GPUDevice* device;

    GLuint fbo;

    uint32_t width;
    uint32_t height;

    GLuint colorAttachments[8];
    uint32_t colorCount;

    GLuint depthAttachment;
};

#define MOSS_MAX_BINDINGS 16

struct Moss_ResourceSet {
    Moss_GPUDevice* device;

    uint32_t bufferCount;
    Moss_GPUBuffer* buffers[MOSS_MAX_BINDINGS];
    uint32_t bufferBindings[MOSS_MAX_BINDINGS];

    uint32_t textureCount;
    Moss_TextureView* textures[MOSS_MAX_BINDINGS];
    Moss_GPUSampler* samplers[MOSS_MAX_BINDINGS];
};

// GPU texture abstraction
struct Moss_Texture {
    GLuint texture_id;
    GLenum target;           // GL_TEXTURE_2D, GL_TEXTURE_CUBE_MAP, etc.
    uint32_t width, height, depth;
    uint32_t mip_levels;
    GLenum internal_format;  // GL_RGBA8, GL_RGB16F, etc.
};


struct Moss_TextureView {
    Moss_GPUDevice* device;

    GLuint texture;
    GLenum target;     // GL_TEXTURE_2D, GL_TEXTURE_2D_ARRAY

    uint32_t width;
    uint32_t height;
    uint32_t mipLevels;

    GLenum format;
};

struct Moss_GPUSampler {
    GLuint sampler; // 0 if embedded

    GLenum minFilter;
    GLenum magFilter;
    GLenum wrapU;
    GLenum wrapV;
};


struct Moss_RenderGraph {
    Moss_GPUDevice* device;
    Moss_RGPass** passes;
    uint32_t passCount;
};

struct Moss_RGPass {
    const char* name;
    Moss_Framebuffer* framebuffer;
    void (*execute)(void*);
};

typedef struct Moss_RGTexture Moss_RGTexture;
typedef struct Moss_RGBuffer Moss_RGBuffer;






struct Moss_Renderer {
    Moss_Window* window;

    // FSR1 / upscaling
    bool enableFSR1;
    float fsrSharpness;
    GLuint fsrEASUProgram;
    GLuint fsrRCASProgram;
    GLuint fsrIntermediateTex;
    GLuint fsrIntermediateFBO;
    int internalWidth;
    int internalHeight;


    PostProcessingPipeline postPipeline;

    // Main scene framebuffer
    GLuint mainSceneFBO = 0;
    GLuint mainSceneTexture = 0;
    GLuint mainSceneRBO = 0;

    // State flags
    bool enableDepthTest = false;
    bool enableAlphaBlending = true;
    bool enableCullFace = false;
    AntiAliasing aaMode = AntiAliasing::None;
    int msaaSamples = 4; // default 4x MSAA

    // History textures for TAA
    GLuint taaHistory[2];
    int taaCurrentHistory = 0;
    GLuint motionVectorTex = 0;  // optional

    //Camera2* m_camera2 = nullptr;  // MainViewport
    //Camera3* m_camera3 = nullptr;  // MainViewport

    // Fullscreen / debug
    GLuint fullscreenVAO = 0;


    // -------------------------------------------------
    // Resolution & scaling
    // -------------------------------------------------
    uint32_t outputWidth;
    uint32_t outputHeight;

    uint32_t internalWidth;           // For upscaling
    uint32_t internalHeight;

    // -------------------------------------------------
    // Anti-aliasing / Upscaling
    // -------------------------------------------------

    bool enableFSR1;
    float fsrSharpness;

    // History index (used by TAA / FSR2 later)
    uint32_t historyIndex;

    // -------------------------------------------------
    // Render targets (API-agnostic)
    // -------------------------------------------------
    Moss_Framebuffer* mainFramebuffer;        // Scene render target
    Moss_Framebuffer* resolveFramebuffer;     // MSAA resolve
    Moss_Framebuffer* fsrFramebuffer;         // FSR intermediate

    Moss_TextureView* mainColor;
    Moss_TextureView* mainDepth;
    Moss_TextureView* motionVectors;          // optional
    Moss_TextureView* historyColor[2];        // ping-pong

    // -------------------------------------------------
    // Pipelines
    // -------------------------------------------------
    Moss_PipelineState* fullscreenPipeline;   // blit / post
    Moss_PipelineState* fsrEASUPipeline;
    Moss_PipelineState* fsrRCASPipeline;

    // -------------------------------------------------
    // Resource binding
    // -------------------------------------------------
    Moss_ResourceSet* globalSet;     // camera, time, frame data
    Moss_ResourceSet* materialSet;   // per-material bindings

    // -------------------------------------------------
    // Post-processing
    // -------------------------------------------------
    PostProcessingPipeline postPipeline;

    // -------------------------------------------------
    // Render graph (optional but recommended)
    // -------------------------------------------------
    Moss_RenderGraph* renderGraph;

    // -------------------------------------------------
    // Render state defaults
    // -------------------------------------------------
    bool enableDepthTest;
    bool enableAlphaBlending;
    bool enableCullFace;

    // -------------------------------------------------
    // Backend
    // -------------------------------------------------
#ifdef MOSS_GRAPHICS_OPENGLES
    EGLContext context;
    EGLSurface surface;
#endif
    GLuint defaultVAO;

    struct {
        GLuint fbo;
        GLuint color;
        GLuint depth;
    } main;

    struct {
        GLuint fbo;
        GLuint color;
    } fsr;

    GLuint fsrEASUProgram;
    GLuint fsrRCASProgram;

#ifdef MOSS_DEBUG
#endif
};

#endif // MOSS_RENDERER_INTERNAL_H
#ifndef MOSS_RENDERER_INTERNAL_H
#define MOSS_RENDERER_INTERNAL_H
#include <Moss/Moss_Renderer.h>

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
    // Backend
    // -------------------------------------------------
    EGLContext context;
    EGLSurface surface;

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
};

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

struct Moss_PipelineState {
    Moss_GPUDevice* device;

    GLuint program;

    bool depthTest;
    bool depthWrite;
    bool blending;
    bool cullFace;

    GLenum depthFunc;
    GLenum blendSrc;
    GLenum blendDst;

    uint32_t msaaSamples;
};

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

#endif // MOSS_RENDERER_INTERNAL_H
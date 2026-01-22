#ifndef MOSS_RENDERER_OPENGL_H
#define MOSS_RENDERER_OPENGL_H


// Renderer https://youtu.be/NPnQF4yABwg?si=b1e4HnDKHBsWUAhZ

// Supported version history: OpenGL 3.3-OpenGL 4.6
/*              Renderer lifecycle          */

/* OpenGL 3.3 Renderer
 * Supports FSR1 (NOT FSR2)
 *
 */


#include <memory>

#include <Moss/Renderer/GL/glad.h>

#include <Moss/Core/Core.h>
#include <Moss/Moss_Platform.h>

#include <Moss/Renderer/Camera2.h>
#include <Moss/Renderer/Camera3.h>

//#include <Moss/Renderer/Frustum.h>

#include <Moss/Renderer/GL/PostProcess.h>
struct Moss_Renderer;

enum class AntiAliasing {
    None = 0,
    MSAA,
    TAA,
    FXAA,
    SMAA,
    SSAA
};

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
};

typedef struct Moss_GPUBuffer Moss_GPUBuffer;
typedef struct Moss_GPUDevice Moss_GPUDevice;
typedef struct Moss_Shader Moss_Shader;
typedef struct Moss_PipelineState Moss_PipelineState;
typedef struct Moss_Framebuffer Moss_Framebuffer;
typedef struct Moss_ResourceSet Moss_ResourceSet;
typedef struct Moss_TextureView Moss_TextureView;
typedef struct Moss_GPUSampler Moss_GPUSampler;

typedef struct Moss_RenderGraph Moss_RenderGraph;
typedef struct Moss_RGPass Moss_RGPass;
typedef struct Moss_RGTexture Moss_RGTexture;
typedef struct Moss_RGBuffer Moss_RGBuffer;
typedef struct Moss_GPUCommandBuffer Moss_GPUCommandBuffer;

#endif // MOSS_RENDERER_OPENGL_H
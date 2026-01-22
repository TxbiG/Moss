#define GLAD_IMPLEMENTATION
#include <Moss/Renderer/GL/glad.h>  // Leave this here

#ifdef MOSS_PLATFORM_WINDOWS
#include <Moss/Platform/Windows/win32_platform.h>
#endif
#include <Moss/Renderer/GL/Renderer_GL.h>


#define VIRTUAL_WIDTH 780
#define VIRTUAL_HEIGHT 420


enum DepthPass { NONE, LESS, LEQUAL, GREATER, GEQUAL, EQUAL, NOTEQUAL, ALWAYS };


// GL_POLYGON_OFFSET_FILL - Geometry-related, best in Scene or draw call level
// GL_SCISSOR_TEST - UI-specific, better handled in UI system
// GL_MULTISAMPLE - Usually stays enabled globally
// GL_PROGRAM_POINT_SIZE - Used with GL_POINTS; not post-processing relevant
// GL_TEXTURE_CUBE_MAP_SEAMLESS - 	One-time init in renderer startup
// GL_DEBUG_OUTPUT - 	One-time init in renderer startup
// GL_SAMPLE_MASK
// GL_STENCIL_TEST


////////////////////////////////////////////////////////////////////////
// Helper: Fullscreen triangle (used by FSR1 passes)
static GLuint gFullscreenVAO = 0;
static void DrawFullscreenTriangle() {
    if (!gFullscreenVAO) glGenVertexArrays(1, &gFullscreenVAO);
    glBindVertexArray(gFullscreenVAO);
    glDrawArrays(GL_TRIANGLES, 0, 3);
}

////////////////////////////////////////////////////////////////////////
// FSR1 Passes
void Moss_RenderFSR1_EASU(Moss_Renderer* r, GLuint inputTex, int inW, int inH) {
    glBindFramebuffer(GL_FRAMEBUFFER, r->fsrIntermediateFBO);
    glViewport(0, 0, r->window->width, r->window->height);

    glUseProgram(r->fsrEASUProgram);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, inputTex);

    glUniform2f(glGetUniformLocation(r->fsrEASUProgram, "uInputRes"), (float)inW, (float)inH);
    glUniform2f(glGetUniformLocation(r->fsrEASUProgram, "uOutputRes"), (float)r->window->width, (float)r->window->height);

    DrawFullscreenTriangle();
}

void Moss_RenderFSR1_RCAS(Moss_Renderer* r) {
    glBindFramebuffer(GL_FRAMEBUFFER, 0); // backbuffer
    glViewport(0, 0, r->window->width, r->window->height);

    glUseProgram(r->fsrRCASProgram);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, r->fsrIntermediateTex);

    glUniform1f(glGetUniformLocation(r->fsrRCASProgram, "uSharpness"), r->fsrSharpness);

    DrawFullscreenTriangle();
}


////////////////////////////////////////////////////////////////////////
// Renderer Creation
Moss_Renderer* Moss_CreateRenderer(Moss_Window* window) {
    if (!gladLoadGL()) { MOSS_ERROR("Failed to initialise OpenGL."); return nullptr; }

    Moss_Renderer* r = (Moss_Renderer*)malloc(sizeof(Moss_Renderer));
    memset(r, 0, sizeof(*r));
    r->window = window;

    // --- Internal FSR1 resolution ---
    r->internalWidth  = (int)(window->width  * 0.67f);
    r->internalHeight = (int)(window->height * 0.67f);

    // --- Main Scene FBO ---
    glGenFramebuffers(1, &r->mainSceneFBO);
    glBindFramebuffer(GL_FRAMEBUFFER, r->mainSceneFBO);

    if (r->aaMode == AntiAliasing::MSAA) {
        glGenTextures(1, &r->mainSceneTexture);
        glBindTexture(GL_TEXTURE_2D_MULTISAMPLE, r->mainSceneTexture);
        glTexImage2DMultisample(GL_TEXTURE_2D_MULTISAMPLE, r->msaaSamples, GL_RGB8, window->width, window->height, GL_TRUE);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D_MULTISAMPLE, r->mainSceneTexture, 0);

        // MSAA resolve FBO
        glGenFramebuffers(1, &r->resolveFBO);
        glBindFramebuffer(GL_FRAMEBUFFER, r->resolveFBO);
        glGenTextures(1, &r->resolveTexture);
        glBindTexture(GL_TEXTURE_2D, r->resolveTexture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, window->width, window->height, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, r->resolveTexture, 0);
    } else {
        glGenTextures(1, &r->mainSceneTexture);
        glBindTexture(GL_TEXTURE_2D, r->mainSceneTexture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, window->width, window->height, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, r->mainSceneTexture, 0);
    }

    // Depth/stencil
    glGenRenderbuffers(1, &r->mainSceneRBO);
    glBindRenderbuffer(GL_RENDERBUFFER, r->mainSceneRBO);
    if (r->aaMode == AntiAliasing::MSAA) 
        glRenderbufferStorageMultisample(GL_RENDERBUFFER, r->msaaSamples, GL_DEPTH24_STENCIL8, window->width, window->height);
    else 
        glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, window->width, window->height);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, r->mainSceneRBO);

    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
        MOSS_ERROR("Main scene FBO incomplete!");

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // --- FSR1 Intermediate FBO ---
    if (r->enableFSR1) {
        glGenTextures(1, &r->fsrIntermediateTex);
        glBindTexture(GL_TEXTURE_2D, r->fsrIntermediateTex);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, window->width, window->height, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        glGenFramebuffers(1, &r->fsrIntermediateFBO);
        glBindFramebuffer(GL_FRAMEBUFFER, r->fsrIntermediateFBO);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, r->fsrIntermediateTex, 0);

        if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
            MOSS_ERROR("FSR intermediate FBO incomplete");
    }

    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    if (r->enableCullFace) {
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        glFrontFace(GL_CCW);
    } else {
        glDisable(GL_CULL_FACE);
    }

    glEnable(GL_STENCIL_TEST);
    glStencilFunc(GL_ALWAYS, 1, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
    glStencilMask(0xFF);

    MOSS_INFO("Renderer: %s", (const char*)glGetString(GL_RENDERER));
    MOSS_INFO("OpenGL version: %s", (const char*)glGetString(GL_VERSION));

    return r;
}

////////////////////////////////////////////////////////////////////////
// Frame
void Moss_RendererBeginFrame(Moss_Renderer* r) {
    glBindFramebuffer(GL_FRAMEBUFFER, r->mainSceneFBO);

    glViewport(0, 0, r->internalWidth, r->internalHeight);

    if (r->enableDepthTest) glEnable(GL_DEPTH_TEST);
    else glDisable(GL_DEPTH_TEST);

    if (r->enableAlphaBlending) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    } else {
        glDisable(GL_BLEND);
    }

    glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
    if (r->enableDepthTest) 
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    else 
        glClear(GL_COLOR_BUFFER_BIT);
}

void Moss_RendererEndFrame(Moss_Renderer* r) {
    GLuint inputTex = (r->aaMode == AntiAliasing::MSAA) ? r->resolveTexture : r->mainSceneTexture;

    // MSAA resolve
    if (r->aaMode == AntiAliasing::MSAA) {
        glBindFramebuffer(GL_READ_FRAMEBUFFER, r->mainSceneFBO);
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, r->resolveFBO);
        glBlitFramebuffer(0, 0, r->internalWidth, r->internalHeight, 0, 0, r->internalWidth, r->internalHeight, GL_COLOR_BUFFER_BIT, GL_NEAREST);
        inputTex = r->resolveTexture;
    }

    // FSR1 upscale
    if (r->enableFSR1) {
        Moss_RenderFSR1_EASU(r, inputTex, r->internalWidth, r->internalHeight);
        Moss_RenderFSR1_RCAS(r);
    } else {
        if (!r->postPipeline.passes.empty())
            r->postPipeline.apply(inputTex);
        else {
            glBindFramebuffer(GL_FRAMEBUFFER, 0);
            glBindTexture(GL_TEXTURE_2D, inputTex);
            DrawFullscreenTriangle();
        }
    }

    Moss_SwapBuffers();
}

////////////////////////////////////////////////////////////////////////
// Termination
void Moss_TerminateRenderer(Moss_Renderer* r) {
    if (!r) return;

    if (r->enableFSR1) {
        glDeleteTextures(1, &r->fsrIntermediateTex);
        glDeleteFramebuffers(1, &r->fsrIntermediateFBO);
        glDeleteProgram(r->fsrEASUProgram);
        glDeleteProgram(r->fsrRCASProgram);
    }

    glDeleteFramebuffers(1, &r->mainSceneFBO);
    glDeleteTextures(1, &r->mainSceneTexture);
    glDeleteRenderbuffers(1, &r->mainSceneRBO);

    free(r);
}


void Moss_AddPostProcessor(Moss_Renderer* renderer, PostProcess* pass) { renderer->postPipeline.add(pass); }
void Moss_ClearPostProcessors(Moss_Renderer* renderer) { renderer->postPipeline.clear(); }

//void Moss_RendererActivateCamera2(Moss_Renderer* renderer, Camera2& cam2) { renderer->m_camera2 = &cam2; }
//void Moss_RendererActivateCamera3(Moss_Renderer* renderer, Camera3& cam3) { renderer->m_camera3 = &cam3; }

// ==========================================
//          Forward declorations
// ==========================================

// Combine VertexBuffers and Texture with unions
// Add Sprite, SpriteAnimations, Model Animations
/*
struct VertexBuffer { 
    union { Vec2 Position; Vec2 UV; Vec2 Normal; Color m_Color;};                                           // Vertex Buffer for 2D
    union { Vec3 Position; Vec3 UV; Vec3 Normal; Vec3 Tangent; };                             // Vertex Buffer for 3D Mesh
    union { Vec3 Position; Vec3 UV; Vec3 Normal; Vec3 Tangent; Vec4 joint; Vec4 weight; };    // Vertex Buffer for 3D Model
};

struct Viewport {
    ivec2 position;
    int width, height;
}

struct Framebuffer { GLuint Handle = 0; Texture ColorAttachment; };

struct Line2D;
struct ParallaxBackground2;

struct Material {};

struct Mesh {float* vertices; unsigned int* indecies; GLuint VBO, EBO, VAO; size_t vertexCount, indexCount; Shader& shader; }
struct Model {Mesh* meshes; size_t meshCount; Texture* textures; size_t textureCount; Shader& shader;};

//typedef struct MeshInstance2 {Rect* mesh; TVector<Vec2> position; float rotation; Shader& shader;};
//typedef struct MeshInstance3 {Mesh* mesh; TVector<Vec3> position, rotation; Shader& shader;};

struct Decal { GLuint textureID; Mat44 modelMatrix; Shader& shader; }


struct ShaderManager
{
    TVector<Shader*> shaders;
}*/

// Life Cycle
//struct Moss_Renderer {
    /*
    DirectionalLighting2* directionalLight2;
    DirectionalLighting3* directionalLight3;

    TVector<Rect*> rect;
    TVector<Recti*> recti;
    TVector<Surface*> Surface;
    TVector<MeshInstance2*> meshInstance2;

    TVector<Mesh*> mesh;
    TVector<Model*> model;
    TVector<MeshInstance3*> meshInstance3;

    TVector<Decal*> decal;

    TVector<FogVolume*> fogVolume;
    TVector<TextureLighting2*> textureLight2;
    TVector<PointLight2*> pointLight2;
    TVector<OmniLight3*> omniLight;
    TVector<SpotLight3*> spotLight;
    TVector<TextureLight3*> textureLight3;

    TVector<Shader*> shaders;
    TVector<Material*> materials;
    TVector<Shader*> posprocessing;
    TVector<Shader*> compositors;


    // Viewports
    Viewport m_viewport;
    TVector<Viewport> m_viewports;

    // Callbacks
    Moss_RendererGetCamera2D m_camera2D;
    Moss_RendererGetCamera3D m_camera3D;
    */
//}

/*      Buffer Functions        */
/*
// @brief Tell OpenGL to create a buffer used for Vertex/indecies. @param ID VAO/EBO ID. @param type GL_ARRAY_BUFFER for vertex / GL_ELEMENT_ARRAY_BUFFER for indicates. @param size Vertex/Indecie amount. @param usage GL_STATIC_DRAW / GL_DYNAMIC_DRAW / GL_STREAM_DRAW.
void Buffer_Create(GLuint& ID, GLenum type, const size_t size, const void* data, GLenum usage) { glGenBuffers(1, &ID); glBindBuffer(type, ID); glBufferData(type, size, data, usage); }
// @brief Tell OpenGL to bind Buffer. @param ID VBO/EBO ID @param type GL_ARRAY_BUFFER for vertex / GL_ELEMENT_ARRAY_BUFFER for indicates.
void Buffer_Bind(const GLuint& ID, GLenum type) { glBindBuffer(type, ID); }
// @brief Tell OpenGL to unbind Buffer. @param type GL_ARRAY_BUFFER for vertex / GL_ELEMENT_ARRAY_BUFFER for indicates.
void Buffer_Unbind(GLenum type) { glBindBuffer(type, 0); }
void Buffer_Delete(GLuint& ID) { glDeleteBuffers(1, &ID); ID = 0; }

void VAO_Create(GLuint& ID) { glGenVertexArrays(1, &ID); }
void VAO_Bind(const GLuint& ID) { glBindVertexArray(ID); }
void VAO_Unbind() { glBindVertexArray(0); }
void VAO_LinkAttrib(const unsigned int index, GLsizei size, const unsigned int stride, const void* data) { glEnableVertexAttribArray(index); glVertexAttribPointer(index, size, GL_FLOAT, GL_FALSE, stride, data); }
void VAO_Delete(GLuint& ID) { glDeleteVertexArrays(1, &ID); ID = 0; }
*/
/*               2D drawing                */

/*! @brief X. @param X X. @ingroup Rect. */
/*
Rect* Moss_CreateRect(Moss_Renderer* renderer, Color color, Vec2 size)
{
    Rect* rect = (Rect*)malloc(sizeof(Rect));
    float vertices[] = {
        // Positions       
        -0.5f, -0.5f, 0.0f, 
         0.5f, -0.5f, 0.0f, 
         0.5f,  0.5f, 0.0f, 
        -0.5f,  0.5f, 0.0f, 
    };

    unsigned int indices[] = { 0, 1, 2, 2, 3, 0 };

    return Mesh_Create(vertices, indices, sizeof(vertices) / sizeof(float), sizeof(indices) / sizeof(unsigned int));
}*/
/*! @brief X. @param X X. @ingroup Rect. */
//void Moss_PresentRect();
/*! @brief X. @param X X. @ingroup Rect. */
//void Moss_RemoveRect();


/*! @brief X. @param X X. @ingroup Texture Rect. */
//Rect* Moss_CreateTextureRect(Moss_Renderer* renderer, Moss_Texture2 texture, Vec2 size);
/*! @brief X. @param X X. @ingroup Texture Rect. */
//void Moss_PresentTextureRect();
/*! @brief X. @param X X. @ingroup Texture Rect. */
//void Moss_RemoveTextureRect(Moss_Renderer* renderer, Rect* rect) { free(rect); }

// Mesh / Geometry
/*! @brief X. @param X X. @ingroup Mesh. */
/*
Mesh* Moss_CreateMesh(Moss_Renderer* renderer, Moss_Texture2 texture, const char* path)
{
    Mesh* mesh = (Mesh*)malloc(sizeof(Mesh));
    VAO_Create(mesh->VAO);
    VAO_Bind(mesh->VAO);

    Buffer_Create(VBO, GL_ARRAY_BUFFER, vertexCount * sizeof(float), vertices, usage);
    Buffer_Create(EBO, GL_ELEMENT_ARRAY_BUFFER, indexCount * sizeof(unsigned int), indices, usage);

    // Link vertex attributes (assuming position only for simplicity, adjust as needed)
    VAO_LinkAttrib(0, 3, sizeof(float) * 3, (void*)0); // Position: layout(location = 0)
    VAO_Unbind();

    mesh.vertices = vertices;
    mesh.indecies = indices;

    return mesh;
}*/
/*! @brief X. @param X X. @ingroup Mesh. */
//void Moss_PresentMesh(Moss_Renderer* renderer, Mesh* mesh);
/*! @brief X. @param X X. @ingroup Mesh. */
//void Moss_RemoveMesh(Moss_Renderer* renderer, Mesh* mesh) { Buffer_Delete(&mesh.vbo); Buffer_Delete(&mesh.ebo); VAO_Delete(mesh.vao); free(mesh); }


// Instancing
/*! @brief X. @param X X. @ingroup X. */
//void Moss_CreateMeshInstances(Moss_Renderer* renderer, Moss_Texture2 texture, const char* path);
/*! @brief X. @param X X. @ingroup X. */
//void Moss_PresentMeshInstances(Moss_Renderer* renderer, Mesh* mesh);
/*! @brief X. @param X X. @ingroup X. */
//void Moss_RemoveMeshInstances(Moss_Renderer* renderer, Mesh* mesh) { free(mesh); }

/*! @brief X. @param X X. @ingroup Rect Instances. */
//void Moss_CreateRectInstances(Moss_Renderer* renderer, const char* path);
/*! @brief X. @param X X. @ingroup Rect Instances. */
//void Moss_PresentRectInstances(Moss_Renderer* renderer, Rect* mesh);
/*! @brief X. @param X X. @ingroup Rect Instances. */
//void Moss_RemoveRectInstances(Moss_Renderer* renderer, Rect* mesh) { free(mesh); }

// Primitives
/*! @brief X. @param X X. @ingroup X. */
/*
Mesh* Moss_CreateCube()
{
    float vertices[] = {
        // Positions          
        -0.5f, -0.5f, -0.5f, 
         0.5f, -0.5f, -0.5f, 
         0.5f,  0.5f, -0.5f, 
         0.5f,  0.5f, -0.5f, 
        -0.5f,  0.5f, -0.5f, 
        -0.5f, -0.5f, -0.5f, 

        -0.5f, -0.5f,  0.5f, 
         0.5f, -0.5f,  0.5f, 
         0.5f,  0.5f,  0.5f, 
         0.5f,  0.5f,  0.5f, 
        -0.5f,  0.5f,  0.5f, 
        -0.5f, -0.5f,  0.5f, 

        -0.5f,  0.5f,  0.5f, 
        -0.5f,  0.5f, -0.5f, 
        -0.5f, -0.5f, -0.5f, 
        -0.5f, -0.5f, -0.5f, 
        -0.5f, -0.5f,  0.5f, 
        -0.5f,  0.5f,  0.5f, 

         0.5f,  0.5f,  0.5f, 
         0.5f,  0.5f, -0.5f, 
         0.5f, -0.5f, -0.5f, 
         0.5f, -0.5f, -0.5f, 
         0.5f, -0.5f,  0.5f, 
         0.5f,  0.5f,  0.5f, 

        -0.5f, -0.5f, -0.5f, 
         0.5f, -0.5f, -0.5f, 
         0.5f, -0.5f,  0.5f, 
         0.5f, -0.5f,  0.5f, 
        -0.5f, -0.5f,  0.5f, 
        -0.5f, -0.5f, -0.5f, 

        -0.5f,  0.5f, -0.5f, 
         0.5f,  0.5f, -0.5f, 
         0.5f,  0.5f,  0.5f, 
         0.5f,  0.5f,  0.5f, 
        -0.5f,  0.5f,  0.5f, 
        -0.5f,  0.5f, -0.5f, 
    };

    unsigned int indices[] = {
         0, 1, 2, 2, 3, 0, // Back face
         4, 5, 6, 6, 7, 4, // Front face
         8, 9,10,10,11, 8, // Left face
        12,13,14,14,15,12, // Right face
        16,17,18,18,19,16, // Bottom face
        20,21,22,22,23,20  // Top face
    };

    return Mesh_Create(vertices, indices, sizeof(vertices) / sizeof(float), sizeof(indices) / sizeof(unsigned int));
}*/
/*! @brief X. @param X X. @ingroup X. */
/*
Mesh* Moss_CreateSphere()
{
    const unsigned int X_SEGMENTS = 64;
    const unsigned int Y_SEGMENTS = 64;
    const float PI = 3.14159265359f;
    std::vector<float> vertices;
    std::vector<unsigned int> indices;
    for (unsigned int y = 0; y <= Y_SEGMENTS; ++y) {
        for (unsigned int x = 0; x <= X_SEGMENTS; ++x) {
            float xSegment = (float)x / (float)X_SEGMENTS;
            float ySegment = (float)y / (float)Y_SEGMENTS;
            float xPos = cos(xSegment * 2.0f * PI) * sin(ySegment * PI);
            float yPos = cos(ySegment * PI);
            float zPos = sin(xSegment * 2.0f * PI) * sin(ySegment * PI);

            vertices.push_back(xPos);
            vertices.push_back(yPos);
            vertices.push_back(zPos);
        }
    }
    for (unsigned int y = 0; y < Y_SEGMENTS; ++y) {
        for (unsigned int x = 0; x < X_SEGMENTS; ++x) {
            indices.push_back(y * (X_SEGMENTS + 1) + x);
            indices.push_back((y + 1) * (X_SEGMENTS + 1) + x);
            indices.push_back((y + 1) * (X_SEGMENTS + 1) + x + 1);

            indices.push_back(y * (X_SEGMENTS + 1) + x);
            indices.push_back((y + 1) * (X_SEGMENTS + 1) + x + 1);
            indices.push_back(y * (X_SEGMENTS + 1) + x + 1);
        }
    }

    return Mesh_Create(vertices.data(), indices.data(), vertices.size(), indices.size());
}*/
/*! @brief X. @param X X. @ingroup X. */
/*
Mesh* Moss_CreateCylinder()
{
    std::vector<float> vertices;
    std::vector<unsigned int> indices;

    // Top circle
    vertices.insert(vertices.end(), {0.0f, height / 2.0f, 0.0f}); // Center
    for (int i = 0; i < segments; ++i) {
        float theta = i * 2 * M_PI / segments;
        float x = radius * cos(theta);
        float z = radius * sin(theta);
        vertices.insert(vertices.end(), {x, height / 2.0f, z});
    }

    // Bottom circle
    vertices.insert(vertices.end(), {0.0f, -height / 2.0f, 0.0f}); // Center
    for (int i = 0; i < segments; ++i) {
        float theta = i * 2 * M_PI / segments;
        float x = radius * cos(theta);
        float z = radius * sin(theta);
        vertices.insert(vertices.end(), {x, -height / 2.0f, z});
    }

    // Generate indices for top and bottom caps
    for (int i = 1; i <= segments; ++i) {
        int next = (i % segments) + 1;

        // Top cap
        indices.insert(indices.end(), {0, i, next});

        // Bottom cap
        int offset = segments + 1;
        indices.insert(indices.end(), {offset, offset + i, offset + next});
    }

    // Generate indices for side walls
    for (int i = 1; i <= segments; ++i) {
        int next = (i % segments) + 1;
        int offset = segments + 1;

        indices.insert(indices.end(), {i, offset + i, next});
        indices.insert(indices.end(), {next, offset + i, offset + next});
    }

    GLuint vao, vbo, ebo;
    Buffer_Create(vao, GL_ARRAY_BUFFER, sizeof(float) * vertices.size(), vertices.data(), GL_STATIC_DRAW);
    Buffer_Create(ebo, GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * indices.size(), indices.data(), GL_STATIC_DRAW);

    return {vao, vbo, ebo};
}*/
/*! @brief X. @param X X. @ingroup X. */
//Mesh* Moss_CreateCone();

/*
Mesh Moss_CreateCapsule()
{
    std::vector<float> vertices;
    std::vector<unsigned int> indices;

    // Top hemisphere
    for (int i = 0; i <= segments; ++i) {
        float phi = M_PI / 2 - (i * M_PI / segments); // From pi/2 to 0
        float y = radius * sin(phi);
        float r = radius * cos(phi);

        for (int j = 0; j <= segments; ++j) {
            float theta = j * 2 * M_PI / segments;
            float x = r * cos(theta);
            float z = r * sin(theta);

            vertices.insert(vertices.end(), {x, y + height / 2.0f, z}); // Shift vertically
        }
    }

    // Cylinder
    for (int i = 0; i <= 1; ++i) {
        float y = height / 2.0f - (i * height);

        for (int j = 0; j <= segments; ++j) {
            float theta = j * 2 * M_PI / segments;
            float x = radius * cos(theta);
            float z = radius * sin(theta);

            vertices.insert(vertices.end(), {x, y, z});
        }
    }

    // Bottom hemisphere
    for (int i = 0; i <= segments; ++i) {
        float phi = M_PI - (i * M_PI / segments); // From pi to pi/2
        float y = radius * sin(phi);
        float r = radius * cos(phi);

        for (int j = 0; j <= segments; ++j) {
            float theta = j * 2 * M_PI / segments;
            float x = r * cos(theta);
            float z = r * sin(theta);

            vertices.insert(vertices.end(), {x, y - height / 2.0f, z}); // Shift vertically
        }
    }

    // Generate indices for triangles
    // (For simplicity, generating an index buffer for glDrawElements)
    int rows = segments + 1;
    for (int i = 0; i < 2 * rows; ++i) { // Hemisphere and cylinder rows
        for (int j = 0; j < segments; ++j) {
            int next = (j + 1) % segments;

            indices.insert(indices.end(), {i * rows + j, i * rows + next, (i + 1) * rows + j});
            indices.insert(indices.end(), {i * rows + next, (i + 1) * rows + next, (i + 1) * rows + j});
        }
    }

    GLuint vao, vbo, ebo;
    Buffer_Create(vao, GL_ARRAY_BUFFER, sizeof(float) * vertices.size(), vertices.data(), GL_STATIC_DRAW);
    Buffer_Create(ebo, GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * indices.size(), indices.data(), GL_STATIC_DRAW);

    return {vao, vbo, ebo};
}
*/

/*           Textures        */
/*! @brief X. @param X X. @ingroup Texture 2D. */
//Moss_Texture2* Moss_CreateTexture(const char* path);
/*! @brief X. @param X X. @ingroup Texture 2D. */
//void Moss_PresentTexture(Moss_Renderer* renderer, Moss_Texture2* texture);
/*! @brief X. @param X X. @ingroup Texture 2D. */
//void Moss_RemoveTexture(Moss_Renderer* renderer, Moss_Texture2* texture);

/*! @brief X. @param X X. @ingroup Texture 3D. */
//Moss_Texture3* Moss_CreateTexture(const char* path);
/*! @brief X. @param X X. @ingroup Texture 3D. */
//void Moss_PresentTexture(Moss_Renderer* renderer, Moss_Texture3* texture);
/*! @brief X. @param X X. @ingroup Texture 3D. */
//void Moss_RemoveTexture(Moss_Renderer* renderer, Moss_Texture3* texture);

/*          Lighting           */
/*

DirectionalLighting2* Moss_CreateDirectionalLighting2(Moss_Renderer* renderer, float intensity, float directional_rotation) { DirectionalLighting2* light = (DirectionalLighting2*)malloc(sizeof(DirectionalLighting2)); return light; }
void Moss_PresentDirectionalLighting2(Moss_Renderer* renderer) {}
void Moss_RemoveDirectionalLighting2(Moss_Renderer* renderer, DirectionalLighting2* light) { free(light); }


TextureLighting2* Moss_CreateTextureLighting2(Moss_Renderer* renderer, Moss_Texture2* texture, float intensity, Vec2 position, float directional_rotation) { TextureLighting2* light = (TextureLighting2*)malloc(sizeof(DirectionalLighting2)); return light; }
void Moss_PresentTextureLighting2(Moss_Renderer* renderer) {}
void Moss_RemoveTextureLighting2(Moss_Renderer* renderer, Moss_Texture2* light) { free(light); }


PointLight2* Moss_CreatePointLight2(Moss_Renderer* renderer, Vec2 position, float intensity, Vec2 position, float directional_rotation) { PointLight2* light = (PointLight2*)malloc(sizeof(PointLight2)); return light; }
void Moss_PresentPointLighting2(Moss_Renderer* renderer) {}
void Moss_RemovePointLighting2(Moss_Renderer* renderer, PointLight2* light) { free(light); }


DirectionalLighting3* Moss_CreateDirectionalLighting3(Moss_Renderer* renderer, float intensity, Vec3 directionl_rotation);
void Moss_PresentPointLighting2(Moss_Renderer* renderer) {}
void Moss_RemovePointLighting2(Moss_Renderer* renderer, PointLight2* light) { free(light); }


OmniLight3* Moss_CreateOmniLight3(Moss_Renderer* renderer, float intensity, Vec3 position, Vec3 directional_rotation);
void Moss_PresentOmniLighting3(Moss_Renderer* renderer) {}
void Moss_RemoveOmniLighting3(Moss_Renderer* renderer, PointLight2* light) { free(light); }


SpotLight3* Moss_CreateSpotLight3(Moss_Renderer* renderer, float intensity, Vec3 position, Vec3 directional_rotation);
void Moss_PresentSpotLighting3(Moss_Renderer* renderer) {}
void Moss_RemoveSpotLighting3(Moss_Renderer* renderer, PointLight2* light) { free(light); }


TextureLight3* Moss_CreateTextureLight3(Moss_Renderer* renderer, float intensity, Moss_Texture3* texture, Vec2 position, Vec3 directional_rotation) {}
void Moss_PresentTextureLighting3(Moss_Renderer* renderer) {}
void Moss_RemoveTextureLighting3(Moss_Renderer* renderer, PointLight2* light) { free(light); }
*/

/*       Post-processing        */
/*
Shader* Moss_CreatePostProcess(Moss_Renderer* renderer) {}
void Moss_PresentPostProcess(Moss_Renderer* renderer, Shader* postprocess) {}
void Moss_RemovePostProcess(Moss_Renderer* renderer, Shader* postprocess) {}
*/
/*           Compositors      */
/*
Shader* Moss_CreateCompositorEffect(Moss_Renderer* renderer, const char* path)  {}
void Moss_PresentCompositorEffect(Moss_Renderer* renderer, Shader* compositor)  {}
void Moss_RemoveCompositorEffect(Moss_Renderer* renderer, Shader* compositor)  { free(compositor); }
*/

//void Moss_TerminateRenderer(Moss_Renderer* renderer) { if (!renderer) { return; } free(renderer); }
//void SubViewport(int x, int y, int width, int height, int scalex, int scaley) { glMatrixMode(GL_PROJECTION); glLoadIdentity(); glortho(x, y, scalex, scaley); glViewport(x,y,width,height); }


/*
TODO: ADD SHADOWMAPPING
Per-Frame Rendering
├─Setup lights
├─ Draw meshes / models
├─ Draw decals
├─ Apply fog (fragment shader or fullscreen pass)
├─ Compositors (optional)
├─ Post-process effects
└─ Moss_PresentRenderer (final screen output)
*/
/*
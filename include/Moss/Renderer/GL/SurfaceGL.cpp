#include <Moss/Renderer/GL/SurfaceGL.h>
#include <Moss/Renderer/GL/glad.h>

// Vertex shader for color-only surface (uses uniform color)
const char* vertexShaderSourceColor = R"(
#version 330 core
layout (location = 0) in vec3 aPos;

uniform vec4 u_Color;

uniform mat4 u_Model;
uniform mat4 u_ViewProjection;

out vec4 VertexColor;

void main() {
    gl_Position = u_ViewProjection * u_Model * vec4(aPos, 1.0);
    VertexColor = u_Color;
})";

// Vertex shader for textured surface (uses tex coords)
const char* vertexShaderSourceTexture = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec2 aTexCoord;

uniform mat4 u_Model;
uniform mat4 u_ViewProjection;

out vec2 TexCoord;

void main() {
    gl_Position = u_ViewProjection * u_Model * vec4(aPos, 1.0);
    TexCoord = aTexCoord;
})";

// Fragment shader for color-only surface
const char* fragmentShaderSourceColor = R"(
#version 330 core
in vec4 VertexColor;
out vec4 FragColor;

void main() {
    FragColor = VertexColor;
})";

// Fragment shader for textured surface
const char* fragmentShaderSourceTexture = R"(
#version 330 core
in vec2 TexCoord;
out vec4 FragColor;

uniform sampler2D u_Texture;
uniform vec2 u_UVOffset;
uniform vec2 u_UVScale;

void main() {
    vec2 atlasUV = u_UVOffset + TexCoord * u_UVScale;
    vec4 texColor = texture(u_Texture, atlasUV);
    if (texColor.a == 0.0) discard;
    FragColor = texColor;
})";

// Constructor for color surface (no texture)
// Texture-based constructor with optional UVs
SurfaceRect::SurfaceRect(float x, float y, float width, float height, const Texture& tex, Float2 uvOffset, Float2 uvScale)
    : position(x,y), width(width), height(height), color(), texture(&tex),
      uvOffset(uvOffset), uvScale(uvScale), mode(SurfaceMode::Texture),
      m_shader(vertexShaderSourceTexture, fragmentShaderSourceTexture, true), model(Mat44::sIdentity())
{ initBuffers(); }

// Color-based constructor
SurfaceRect::SurfaceRect(float x, float y, float width, float height, const Color& col)
    : position(x,y), width(width), height(height), color(col), texture(nullptr),
      uvOffset(0.0f, 0.0f), uvScale(1.0f, 1.0f), mode(SurfaceMode::Color),
      m_shader(vertexShaderSourceColor, fragmentShaderSourceColor, true), model(Mat44::sIdentity())
{ initBuffers(); }
SurfaceRect::~SurfaceRect() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
}

void SurfaceRect::initBuffers() {
    // Define indices (same for both textured and colored)
    unsigned int indices[] = {0, 1, 2, 2, 3, 0};

    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    if (mode == SurfaceMode::Texture && texture != nullptr) {
        // Apply uvOffset and uvScale to calculate atlas-specific texcoords
        float u0 = uvOffset.x;
        float v0 = uvOffset.y;
        float u1 = u0 + uvScale.x;
        float v1 = v0 + uvScale.y;

        float vertices[] = {
            // x    y    z     u     v
            0.0f, 0.0f, 0.0f, u0, v0,  // bottom-left
            1.0f, 0.0f, 0.0f, u1, v0,  // bottom-right
            1.0f, 1.0f, 0.0f, u1, v1,  // top-right
            0.0f, 1.0f, 0.0f, u0, v1   // top-left
        };

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

        // Position attribute
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        // TexCoord attribute
        glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);
    } else {
        // Color-only surface: positions only
        float vertices[] = {
            // x    y    z
            0.0f, 0.0f, 0.0f,
            1.0f, 0.0f, 0.0f,
            1.0f, 1.0f, 0.0f,
            0.0f, 1.0f, 0.0f
        };

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
    }

    glBindVertexArray(0); // Cleanup
}


void SurfaceRect::update() {
    model = Mat44::sTranslation(Vec3(position.x, position.y, 0.0f)) * Mat44::sRotationZ(rotation) * Mat44::sScale(Vec3(width, height, 1.0f));
}

void SurfaceRect::draw(const Mat44& viewProjectionMatrix) {
    m_shader.bind();

    m_shader.SetUniform("u_Model", model);
    m_shader.SetUniform("u_ViewProjection", viewProjectionMatrix);

    if (mode == SurfaceMode::Texture && texture) {
        texture->bind(0);
        m_shader.SetUniform("u_Texture", 0);
        m_shader.SetUniform("u_UVOffset", uvOffset);
        m_shader.SetUniform("u_UVScale", uvScale);
    } else {
        m_shader.SetUniform("u_Color", color);
    }

    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);

    m_shader.unbind();
}
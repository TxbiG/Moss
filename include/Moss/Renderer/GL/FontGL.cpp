#include <Moss/Renderer/GL/FontGL.h>
#include <Moss/Core/Profiler.h>
#include <fstream>
#include <vector>
#include <iostream>

#define STB_TRUETYPE_IMPLEMENTATION
#include <Moss/External/stb_truetype.h>

const char* vertexfont = R"(
#version 330 core
layout(location = 0) in vec2 aPos;
layout(location = 1) in vec2 aTexCoord;

uniform mat4 u_Model;
uniform mat4 u_ViewProjection;

out vec2 TexCoord;

void main() {
    gl_Position = u_ViewProjection * u_Model * vec4(aPos, 0.0, 1.0);
    TexCoord = aTexCoord;
})";

const char* fragfont = R"(
                    #version 330 core
                    in vec2 TexCoord;
                    out vec4 FragColor;

                    uniform sampler2D uFontTexture;

                    void main() {
                        float alpha = texture(uFontTexture, TexCoord).r;
                        FragColor = vec4(1.0, 0.0, 0.0, alpha);
                    }
                    )";

Font::Font() : textureID(0), bitmapWidth(1024), bitmapHeight(1024), m_shader(vertexfont, fragfont, true) {}

Font::~Font() {
    if (textureID) { glDeleteTextures(1, &textureID); }
    if (vbo) { glDeleteBuffers(1, &vbo); }
    if (vao) { glDeleteVertexArrays(1, &vao); }
}
bool Font::load(const std::string& path, float pixelHeight) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) {
        MOSS_ERROR("Failed to open font: %s\n", path.c_str());
        return false;
    }

    std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);

    if (size <= 0) {
        MOSS_ERROR("Font file is empty!\n");
        return false;
    }

    std::vector<unsigned char> fontBuffer(size);
    if (!file.read((char*)fontBuffer.data(), size)) {
        MOSS_ERROR("Failed to read font file: %s\n", path.c_str());
        return false;
    }

    stbtt_fontinfo fontInfo;
    if (!stbtt_InitFont(&fontInfo, fontBuffer.data(), 0)) {
        MOSS_ERROR("stbtt_InitFont failed!");
        return false;
    }

    int ascent, descent, lineGap;
    stbtt_GetFontVMetrics(&fontInfo, &ascent, &descent, &lineGap);

    float scale = stbtt_ScaleForPixelHeight(&fontInfo, pixelHeight);
    baseline = ascent * scale;

    // Create a larger bitmap if needed
    bitmapWidth = 1024;
    bitmapHeight = 1024;

    std::vector<unsigned char> bitmap(bitmapWidth * bitmapHeight, 0);

    int bakeResult = stbtt_BakeFontBitmap(
        fontBuffer.data(),       // font data
        0,                       // font index
        pixelHeight,             // font size
        bitmap.data(),           // output bitmap
        bitmapWidth, bitmapHeight,
        32,                      // first char
        96,                      // char count
        charData                 // output baked char data
    );

    if (bakeResult <= 0) { MOSS_ERROR("stbtt_BakeFontBitmap() failed! result = %d\n", bakeResult); return false; }

    // OpenGL texture creation
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    glTexImage2D(GL_TEXTURE_2D, 0, GL_RED, bitmapWidth, bitmapHeight, 0, GL_RED, GL_UNSIGNED_BYTE, bitmap.data());

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);


    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    glGenBuffers(1, &vbo);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 6 * 4, nullptr, GL_DYNAMIC_DRAW);

    // Vertex layout: pos (vec2) + uv (vec2)
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 4, (void*)0);

    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(float) * 4, (void*)(sizeof(float) * 2));

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    return true;
}


void Font::bind() {
    glEnable(GL_TEXTURE_2D);
    glBindTexture(GL_TEXTURE_2D, textureID);
}

void Font::unbind() {
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Font::renderText(const std::string& text, float x, float y, const Mat44& viewProjection) {

    struct Vertex {
        float x, y;
        float s, t;
    };

    std::vector<Vertex> vertices;
    vertices.reserve(text.size() * 6);

    float xpos = 0.0f; // Start at 0 because translation will move whole text

    for (char c : text) {
        if (c < 32 || c >= 128) continue;
        stbtt_bakedchar* b = &charData[c - 32];

        float x0 = xpos + b->xoff;
        float y0 = baseline + b->yoff;
        float x1 = x0 + (b->x1 - b->x0);
        float y1 = y0 + (b->y1 - b->y0);

        float s0 = b->x0 / (float)bitmapWidth;
        float t0 = b->y0 / (float)bitmapHeight;
        float s1 = b->x1 / (float)bitmapWidth;
        float t1 = b->y1 / (float)bitmapHeight;

        vertices.push_back({ x0, y0, s0, t0 });
        vertices.push_back({ x1, y0, s1, t0 });
        vertices.push_back({ x1, y1, s1, t1 });

        vertices.push_back({ x0, y0, s0, t0 });
        vertices.push_back({ x1, y1, s1, t1 });
        vertices.push_back({ x0, y1, s0, t1 });

        xpos += b->xadvance;
    }

    m_shader.bind();

    // Textue bind
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, textureID);

    m_shader.SetUniform("uFontTexture", 0);
    m_shader.SetUniform("u_ViewProjection", viewProjection);

    Mat44 model = Mat44::sTranslation(Vec3(x, y, -1.0f)); 
    m_shader.SetUniform("u_Model", model);


    // Draw
    glBindVertexArray(vao);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_DYNAMIC_DRAW);

    glDrawArrays(GL_TRIANGLES, 0, (GLsizei)vertices.size());

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);

    m_shader.unbind();
}
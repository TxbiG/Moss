// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Moss/Renderer/GL/Renderer_GL.h>

#include <string>
#include <Moss/External/stb_truetype.h>

#include <Moss/Core/Reference.h>
#include <Moss/Core/Variants/Color.h>
#include <Moss/Core/Variants/Vector/Float2.h>
#include <Moss/Core/Variants/Vector/Float3.h>
#include <Moss/Renderer/GL/SurfaceGL.h>
#include <Moss/Renderer/GL/TextureGL.h>
#include <Moss/Renderer/PipelineState.h>
#include <Moss/Renderer/GL/ShaderGL.h>
#include <string_view>
#include <memory>

class Moss_Renderer;
class Texture;

/// Font class for rendering variable width fonts with kerning, suitable for 3D rendering.
#ifndef MOSS_FONT_H
#define MOSS_FONT_H

class MOSS_API Font {
public:
    Font();
    ~Font();

    bool load(const std::string& path, float pixelHeight = 32.0f);
    void renderText(const std::string& text, float x, float y, const Mat44& mvp);
    void bind();
    void unbind();

private:
    GLuint textureID;
    ShaderGL m_shader;
    GLuint vao = 0;
    GLuint vbo = 0;

    stbtt_bakedchar charData[96]; // ASCII 32..126
    int bitmapWidth;
    int bitmapHeight;
    float baseline = 0.0f;
};
#endif // MOSS_FONT_H

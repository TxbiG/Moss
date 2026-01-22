#ifndef MOSS_MATERIAL_H
#define MOSS_MATERIAL_H
#include <Moss/Core/Core.h>
#include <Moss/Core/Variants/Color.h>
#include <Moss/Renderer/gl/TextureGL.h>
#include <Moss/Moss_Platform.h>
//#include <Moss/Renderer/Shader.h>


class [[nodiscard]] MOSS_API Material {
public:
    Material(const Color& color);
    Material(Texture& texture);
    ~Material();

    Color& color;
    Texture& texture = nullptr;
private:
    Moss_Renderer m_Renderer;
    Shader m_shader;
#ifdef MOSS_USE_OPENGL || MOSS_USE_OPENGLES
    unsigned int shaderProgram;
    void initShader();
    bool useTexture;
#endif // MOSS_USE_OPENGL || MOSS_USE_OPENGL
};

#endif // MOSS_MATERIAL_H
#ifndef MOSS_SURFACE_GL_H
#define MOSS_SURFACE_GL_H

#include <Moss/Renderer/GL/Renderer_GL.h>
#include <Moss/Core/Core.h>
#include <Moss/Core/Variants/Color.h>
#include <Moss/Core/Variants/Rect.h>
#include <Moss/Core/Variants/Vector/Float2.h>
#include <Moss/Moss_Platform.h>
#include <Moss/Renderer/GL/ShaderGL.h>
#include <Moss/Renderer/GL/TextureGL.h>

class Mat44;

enum class SurfaceMode {
    Color,
    Texture
};


class [[nodiscard]] MOSS_API SurfaceRect {
public:
    // Constructor for colored surface
    SurfaceRect(float x, float y, float width, float height, const Texture& tex, Float2 uvOffset = Float2(0.0f, 0.0f), Float2 uvScale = Float2(1.0f, 1.0f));

    // Constructor for textured surface with optional uvOffset and uvScale
    SurfaceRect(float x, float y, float width, float height, const Color& col);


    //void instance();

    ~SurfaceRect();

    void setRotation(float degrees) { rotation = degrees; }

    void update();
    void draw(const Mat44& viewProjectionMatrix);

    //void setPosition(Float2 _position);
    //Float2 getPosition();
    //void setRotation(float _rotation);
    //float getRotation();
    //void setScale(Float2 _scale);
    //Float2 getScale();
    //void setWidth(int _width);
    //int getWidth();
    //void setHeight(int _height);
    //int getHeight();

    Color color;
    float rotation = 0.0f;
    Float2 position;
    int width, height;
private:
/*
    struct Vertex {
		Float3 Position;
		Float3 Color;
		Float3 Normal;
		Float2 Texcoord;
	}*/
    //Moss_Renderer& renderer;
    unsigned int VAO, VBO, EBO;

    ShaderGL m_shader;
    SurfaceMode mode;
    Mat44 model;

    const Texture* texture = nullptr;
    Float2 uvOffset = {0.0f, 0.0f};
    Float2 uvScale = {1.0f, 1.0f};

    void initBuffers();
};

/*
struct [[nodiscard]] MOSS_API SurfaceRect {
    virtual ~SurfaceRect() = default;

    //virtual void setPosition(Float2 _position) const = 0;
    //virtual Float2 getPosition() const = 0;
    //virtual void setRotation(float _rotation) const = 0;
    //virtual float getRotation() const = 0;
    //virtual void setScale(Float2 _scale) const = 0;
    //virtual Float2 getScale() const = 0;
    //virtual void setWidth(int _width) const = 0;
    //virtual int getWidth() const = 0;
    //virtual void setHeight(int _height) const = 0;
    //virtual int getHeight() const = 0;

protected:
    unsigned int VAO, VBO, EBO;
    Mat44 model;
    Shader m_shader;
}
struct [[nodiscard]] MOSS_API TextureRect : public SurfaceRect {
    TextureRect(float x, float y, float width, float height, Texture texture);
    TextureRect(Rect rect, Texture texture);

    Texture texture;    // Texture2D
};

struct [[nodiscard]] MOSS_API ColorRect : public SurfaceRect {
    ColorRect(float x, float y, float width, float height, Color color);
    ColorRect(Rect rect, Color color);

    Color color;
};


// Can be used for sprite atlases
struct [[nodiscard]] MOSS_API Sprite2D : public SurfaceRect {
    Sprite2D(Rect rect, Texture texture);
    Sprite2D(Rect rect, Texture texture, Texture normal);

    Texture2D texture;
    Texture2D normal;
};


// Can be used for particle effects
class [[nodiscard]] MOSS_API MeshInstace2 {};

// Can be used for wave spawners
class [[nodiscard]] MOSS_API MultiMeshInstace2 {};

// can be used for large amounts of static props in environments
class [[nodiscard]] MOSS_API  MeshBatch2 {};
*/


#endif // MOSS_SURFACE_GL_H

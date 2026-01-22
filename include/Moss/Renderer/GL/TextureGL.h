#ifndef MOSS_TEXTURE_GL_H
#define MOSS_TEXTURE_GL_H

#include <Moss/Core/Core.h>
#include <Moss/Renderer/GL/Renderer_GL.h>
#include <Moss/Renderer/GL/ShaderGL.h>

enum class TextureType {
    Texture1D,
    Texture2D,
    Texture3D,
    TextureCubeMap,
    Texture2DArray
};

class [[nodiscard]] MOSS_API Texture {
public:
    Texture() = default;
    Texture(const char* path, TextureType type = TextureType::Texture2D);   // 1D and 3D texture: path to a single image (height ignored))
    Texture(const std::vector<std::string>& array, TextureType type);       // 2D texture array constructor for Texture2DArray / TextureCubeMap / Texture3D

    //~Texture();

    void bind(unsigned int slot = 0) const;
    void unbind() const;

    unsigned int GetTexture() const { return textureID; }
    int GetWidth() const { return width; }
    int GetHeight() const { return height; }
    int GetDepth() const { return depth; }
    int GetLayers() const { return layers; }
    TextureType GetType() const { return type; }

    void CreateFromMemory(int w, int h,
                               GLenum glInternalFmt, GLenum glFormat,
                               const void* pixels,
                               bool generateMips = false);

private:
    GLuint textureID = 0;
    int width = 0, height = 0, depth = 0, layers = 0, channels = 0;
    TextureType type = TextureType::Texture2D;

    void load1DTexture(const char* path);
    void load2DTexture(const char* path);
    void load3DTexture(const std::vector<std::string>& slices);
    void loadCubeMap(const std::vector<std::string>& faces);
    void load2DArray(const std::vector<std::string>& layersPaths);
};
/*
// Texture abstract
class [[nodiscard]] MOSS_API Texture {
public:
    virtual ~Texture();

    virtual void bind(GLuint unit = 0) const = 0;
    virtual void unbind() const = 0;

    GLuint getID() const { return textureID; }
    GLenum getTarget() const { return target; }

protected:
    GLuint textureID = 0;
    GLenum target = 0; // e.g. GL_TEXTURE_2D, GL_TEXTURE_3D, etc.
};

//          Texture Dimentions
class [[nodiscard]] MOSS_API Texture2D : public Texture {
public:
    Texture2D(int width, int height, GLenum format, const void* data = nullptr);

    void bind(GLuint unit = 0) const override;
    void unbind() const override;

private:
    int width, height;
};

class [[nodiscard]] MOSS_API Texture3D : public Texture {
public:
    Texture3D(int width, int height, int depth, GLenum format, const void* data = nullptr);
    void bind(GLuint unit = 0) const override;
    void unbind() const override;
private:
    int width, height;
    int depth = 0;
};



//class [[nodiscard]] MOSS_API TextureLayered {};
class [[nodiscard]] MOSS_API Cubemap : public Texture {
public:
    Cubemap(int size, GLenum format);
    void loadFace(GLenum face, const void* data);

    void bind(GLuint unit = 0) const override;
    void unbind() const override;
};

//class [[nodiscard]] MOSS_API CubemapArray {};
//class [[nodiscard]] MOSS_API Texture1DArray {};
//class [[nodiscard]] MOSS_API Texture2DArray {};


//          Noise Textures
class [[nodiscard]] MOSS_API NoiseTexture1D : public Texture {
public:
    GradientTexture1D(const std::vector<Color>& gradient);
};
class [[nodiscard]] MOSS_API NoiseTexture2D : public Texture2D {
public:
    NoiseTexture2D(int width, int height); // internally generates Perlin/White noise
};


//          Gradient Textures
class [[nodiscard]] MOSS_API GradientTexture1D : public Texture {
public:
    GradientTexture1D(const std::vector<Color>& gradient);
};
class [[nodiscard]] MOSS_API GradientTexture2D : public Texture {
public:
    GradientTexture1D(const std::vector<Color>& gradient);
};



// Allocates to cpu ram
class CPUTexture2D {
public:
    std::vector<Color> pixels;
    int width, height;

    // Methods: generateGradient(), generateNoise(), etc.
    GPUTexture2D uploadToGPU() const;
};

// Allocates to VRAM
class GPUTexture2D : public Texture2D {
    using Texture2D::Texture2D;
};
*/
#endif // MOSS_TEXTURE_GL_H
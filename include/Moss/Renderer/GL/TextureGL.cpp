#define STB_IMAGE_IMPLEMENTATION
#include <Moss/External/stb_image.h>
#include <Moss/Renderer/GL/TextureGL.h>



// 1D Texture loading
void Texture::load1DTexture(const char* path) {
    stbi_set_flip_vertically_on_load(true);
    unsigned char* data = stbi_load(path, &width, &height, &channels, 0);
    if (!data) {
        MOSS_ERROR("Failed to load 1D texture");
        return;
    }
    // Height is ignored in 1D textures, use width from image width, height must be 1 or ignored
    height = 1;
    depth = 0;

    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_1D, textureID);

    GLenum format = (channels == 4) ? GL_RGBA : GL_RGB;

    glTexImage1D(GL_TEXTURE_1D, 0, format, width, 0, format, GL_UNSIGNED_BYTE, data);

    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    stbi_image_free(data);
    glBindTexture(GL_TEXTURE_1D, 0);
}

// 2D Texture loading (same as before)
void Texture::load2DTexture(const char* path) {
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);


    stbi_set_flip_vertically_on_load(true);
    unsigned char* data = stbi_load(path, &width, &height, &channels, 0);
    if (data) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
        glGenerateMipmap(GL_TEXTURE_2D);
    } else {
        MOSS_ERROR(path, "Failed to load 2D texture: ");
    }
    stbi_image_free(data);
    glBindTexture(GL_TEXTURE_2D, 0);
}

// 3D Texture loading: slices are stacked depth-wise
void Texture::load3DTexture(const std::vector<std::string>& slices) {
    if (slices.empty()) {
        MOSS_ERROR("3D texture requires at least one slice");
        return;
    }
    stbi_set_flip_vertically_on_load(true);

    std::vector<unsigned char*> imagesData;
    int w = 0, h = 0, ch = 0;

    for (const auto& path : slices) {
        int tw, th, tch;
        unsigned char* data = stbi_load(path.c_str(), &tw, &th, &tch, 0);
        if (!data) {
            MOSS_ERROR("Failed to load 3D texture slice: ");
            for (auto d : imagesData) stbi_image_free(d);
            return;
        }
        if (w == 0 && h == 0) {
            w = tw; h = th; ch = tch;
        } else if (tw != w || th != h || tch != ch) {
            MOSS_ERROR("All 3D texture slices must have the same size and channels");
            for (auto d : imagesData) stbi_image_free(d);
            stbi_image_free(data);
            return;
        }
        imagesData.push_back(data);
    }

    width = w; height = h; depth = (int)slices.size(); channels = ch;

    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_3D, textureID);

    GLenum format = (channels == 4) ? GL_RGBA : GL_RGB;

    // Allocate 3D texture storage
    glTexImage3D(GL_TEXTURE_3D, 0, format, width, height, depth, 0, format, GL_UNSIGNED_BYTE, nullptr);

    // Upload each slice
    for (int i = 0; i < depth; i++) {
        glTexSubImage3D(GL_TEXTURE_3D, 0, 0, 0, i, width, height, 1, format, GL_UNSIGNED_BYTE, imagesData[i]);
        stbi_image_free(imagesData[i]);
    }

    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    glBindTexture(GL_TEXTURE_3D, 0);
}

void Texture::loadCubeMap(const std::vector<std::string>& faces) {
    if (faces.size() != 6) {
        MOSS_ERROR("Cube map requires exactly 6 faces");
        return;
    }
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_CUBE_MAP, textureID);

    stbi_set_flip_vertically_on_load(false); // usually cube maps are not flipped

    for (unsigned int i = 0; i < 6; i++) {
        unsigned char* data = stbi_load(faces[i].c_str(), &width, &height, &channels, 0);
        if (data) {
            GLenum format = channels == 4 ? GL_RGBA : GL_RGB;
            glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
            stbi_image_free(data);
        } else {
            MOSS_ERROR("Failed to load cube map face: ");
            stbi_image_free(data);
            glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
            return;
        }
    }

    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

    glBindTexture(GL_TEXTURE_CUBE_MAP, 0);
}


void Texture::load2DArray(const std::vector<std::string>& layersPaths)
{
    if (layersPaths.empty()) {
        MOSS_ERROR("2D texture array needs at least one layer");
        return;
    }

    stbi_set_flip_vertically_on_load(true);

    std::vector<unsigned char*> imagesData;
    int w = 0, h = 0, ch = 0;

    // ── 1. Load every layer, verify identical size & format ────────────────
    for (const auto& path : layersPaths) {
        int tw, th, tch;
        unsigned char* data = stbi_load(path.c_str(), &tw, &th, &tch, 0);
        if (!data) {
            MOSS_ERROR("Failed to load texture array layer ");
            for (auto d : imagesData) stbi_image_free(d);
            return;
        }

        if (w == 0 && h == 0) {          // first image sets the reference size/format
            w = tw; h = th; ch = tch;
        } else if (tw != w || th != h || tch != ch) {
            MOSS_ERROR("All 2D‑array layers must have the same size & channel count");
            for (auto d : imagesData) stbi_image_free(d);
            stbi_image_free(data);
            return;
        }
        imagesData.push_back(data);
    }

    // ── 2. Create the GL texture ───────────────────────────────────────────
    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D_ARRAY, textureID);

    GLenum format;
    switch (ch) {               // handle 1/3/4‑channel images
        case 1:  format = GL_RED;  break;
        case 3:  format = GL_RGB;  break;
        case 4:  format = GL_RGBA; break;
        default: format = GL_RGB;  break;
    }

    // Allocate immutable storage (1 mip level here; add more if you need mipmaps)
    glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, GL_RGBA8, w, h, layersPaths.size(), 0, format, GL_UNSIGNED_BYTE, nullptr);


    // ── 3. Upload each layer ───────────────────────────────────────────────
    for (size_t i = 0; i < layersPaths.size(); ++i) {
        glTexSubImage3D(GL_TEXTURE_2D_ARRAY,
                        0,                 // mip level
                        0, 0, static_cast<GLint>(i),
                        w, h, 1,
                        format,
                        GL_UNSIGNED_BYTE,
                        imagesData[i]);
        stbi_image_free(imagesData[i]);
    }

    // ── 4. Set parameters & (optional) generate mipmaps ───────────────────
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    glGenerateMipmap(GL_TEXTURE_2D_ARRAY);   // comment out if you don’t want mipmaps

    glBindTexture(GL_TEXTURE_2D_ARRAY, 0);

    // ── 5. Save dimensions for later queries ──────────────────────────────
    width  = w;
    height = h;
    depth  = static_cast<int>(layersPaths.size());
}

// 1D or 2D (default) constructor
Texture::Texture(const char* path, TextureType t) : type(t) {
    if (type == TextureType::Texture1D)
        load1DTexture(path);
    else if (type == TextureType::Texture2D)
        load2DTexture(path);
    else
        MOSS_ERROR("Invalid texture type for this constructor");
}

// 2D texture array constructor for Texture2DArray / TextureCubeMap / Texture3D
Texture::Texture(const std::vector<std::string>& array, TextureType t) : type(t) {
    if (type == TextureType::Texture2DArray) { load2DArray(array); }
    else if (type == TextureType::TextureCubeMap) { loadCubeMap(array); }
    else if (type == TextureType::Texture3D) { load3DTexture(array); }
    else { MOSS_ERROR("Invalid texture type for this constructor"); }
}

void Texture::bind(unsigned int slot) const {
    glActiveTexture(GL_TEXTURE0 + slot);
    switch(type) {
        case TextureType::Texture1D: glBindTexture(GL_TEXTURE_1D, textureID); break;
        case TextureType::Texture2D: glBindTexture(GL_TEXTURE_2D, textureID); break;
        case TextureType::Texture3D: glBindTexture(GL_TEXTURE_3D, textureID); break;
        case TextureType::TextureCubeMap: glBindTexture(GL_TEXTURE_CUBE_MAP, textureID); break;
        case TextureType::Texture2DArray: glBindTexture(GL_TEXTURE_2D_ARRAY, textureID); break;
    };
}

void Texture::unbind() const {
    switch(type) {
        case TextureType::Texture1D: glBindTexture(GL_TEXTURE_1D, 0); break;
        case TextureType::Texture2D: glBindTexture(GL_TEXTURE_2D, 0); break;
        case TextureType::Texture3D: glBindTexture(GL_TEXTURE_3D, 0); break;
        case TextureType::TextureCubeMap: glBindTexture(GL_TEXTURE_CUBE_MAP, 0); break;
        case TextureType::Texture2DArray: glBindTexture(GL_TEXTURE_2D_ARRAY, 0); break;
    }
}


void Texture::CreateFromMemory(int w, int h,
                               GLenum glInternalFmt, GLenum glFormat,
                               const void* pixels,
                               bool generateMips)
{
    type   = TextureType::Texture2D;
    width  = w;
    height = h;
    depth  = 1;
    layers = 1;

    glGenTextures(1, &textureID);
    glBindTexture(GL_TEXTURE_2D, textureID);

    glTexImage2D(GL_TEXTURE_2D, 0, glInternalFmt,
                 w, h, 0, glFormat, GL_UNSIGNED_BYTE, pixels);

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                    generateMips ? GL_LINEAR_MIPMAP_LINEAR : GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

    if (generateMips) glGenerateMipmap(GL_TEXTURE_2D);

    glBindTexture(GL_TEXTURE_2D, 0);
}
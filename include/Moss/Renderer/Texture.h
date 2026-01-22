#pragma once

#include <Jolt/Core/Reference.h>

enum class TextureDimension {
    Texture1D,
    Texture2D,
    Texture3D,
};

typedef enum {
    PIXELFORMAT_UNCOMPRESSED_GRAYSCALE = 1, // 8 bit per pixel (no alpha)
    PIXELFORMAT_UNCOMPRESSED_GRAY_ALPHA,    // 8*2 bpp (2 channels)
    PIXELFORMAT_UNCOMPRESSED_R5G6B5,        // 16 bpp
    PIXELFORMAT_UNCOMPRESSED_R8G8B8,        // 24 bpp
    PIXELFORMAT_UNCOMPRESSED_R5G5B5A1,      // 16 bpp (1 bit alpha)
    PIXELFORMAT_UNCOMPRESSED_R4G4B4A4,      // 16 bpp (4 bit alpha)
    PIXELFORMAT_UNCOMPRESSED_R8G8B8A8,      // 32 bpp
    PIXELFORMAT_UNCOMPRESSED_R32,           // 32 bpp (1 channel - float)
    PIXELFORMAT_UNCOMPRESSED_R32G32B32,     // 32*3 bpp (3 channels - float)
    PIXELFORMAT_UNCOMPRESSED_R32G32B32A32,  // 32*4 bpp (4 channels - float)
    PIXELFORMAT_UNCOMPRESSED_R16,           // 16 bpp (1 channel - half float)
    PIXELFORMAT_UNCOMPRESSED_R16G16B16,     // 16*3 bpp (3 channels - half float)
    PIXELFORMAT_UNCOMPRESSED_R16G16B16A16,  // 16*4 bpp (4 channels - half float)
    PIXELFORMAT_COMPRESSED_DXT1_RGB,        // 4 bpp (no alpha)
    PIXELFORMAT_COMPRESSED_DXT1_RGBA,       // 4 bpp (1 bit alpha)
    PIXELFORMAT_COMPRESSED_DXT3_RGBA,       // 8 bpp
    PIXELFORMAT_COMPRESSED_DXT5_RGBA,       // 8 bpp
    PIXELFORMAT_COMPRESSED_ETC1_RGB,        // 4 bpp
    PIXELFORMAT_COMPRESSED_ETC2_RGB,        // 4 bpp
    PIXELFORMAT_COMPRESSED_ETC2_EAC_RGBA,   // 8 bpp
    PIXELFORMAT_COMPRESSED_PVRT_RGB,        // 4 bpp
    PIXELFORMAT_COMPRESSED_PVRT_RGBA,       // 4 bpp
    PIXELFORMAT_COMPRESSED_ASTC_4x4_RGBA,   // 8 bpp
    PIXELFORMAT_COMPRESSED_ASTC_8x8_RGBA    // 2 bpp
} PixelFormat;

// Texture parameters: filter mode
// NOTE 1: Filtering considers mipmaps if available in the texture
// NOTE 2: Filter is accordingly set for minification and magnification
typedef enum {
    TEXTURE_FILTER_POINT = 0,               // No filter, just pixel approximation
    TEXTURE_FILTER_BILINEAR,                // Linear filtering
    TEXTURE_FILTER_TRILINEAR,               // Trilinear filtering (linear with mipmaps)
    TEXTURE_FILTER_ANISOTROPIC_4X,          // Anisotropic filtering 4x
    TEXTURE_FILTER_ANISOTROPIC_8X,          // Anisotropic filtering 8x
    TEXTURE_FILTER_ANISOTROPIC_16X,         // Anisotropic filtering 16x
} TextureFilter;

// Texture parameters: wrap mode
typedef enum {
    TEXTURE_WRAP_REPEAT = 0,                // Repeats texture in tiled mode
    TEXTURE_WRAP_CLAMP,                     // Clamps texture to edge pixel in tiled mode
    TEXTURE_WRAP_MIRROR_REPEAT,             // Mirrors and repeats the texture in tiled mode
    TEXTURE_WRAP_MIRROR_CLAMP               // Mirrors and clamps to border the texture in tiled mode
} TextureWrap;

// Cubemap layouts
typedef enum {
    CUBEMAP_LAYOUT_AUTO_DETECT = 0,         // Automatically detect layout type
    CUBEMAP_LAYOUT_LINE_VERTICAL,           // Layout is defined by a vertical line with faces
    CUBEMAP_LAYOUT_LINE_HORIZONTAL,         // Layout is defined by a horizontal line with faces
    CUBEMAP_LAYOUT_CROSS_THREE_BY_FOUR,     // Layout is defined by a 3x4 cross with cubemap faces
    CUBEMAP_LAYOUT_CROSS_FOUR_BY_THREE     // Layout is defined by a 4x3 cross with cubemap faces
} CubemapLayout;

enum class TextureFormat {
    // Unsigned normalized color formats
    R8, RG8, RGB8, RGBA8,

    // Signed normalized formats
    R8_SNORM, RG8_SNORM, RGB8_SNORM, RGBA8_SNORM,

    // Floating-point formats
    R16F, RG16F, RGB16F, RGBA16F, R32F, RG32F, RGB32F, RGBA32F,

    // Integer formats
    R8UI, RG8UI, RGBA8UI, R16UI, RG16UI, RGBA16UI, R32UI, RG32UI, RGBA32UI,

    // Depth formats
    Depth16, Depth24, Depth32F, Depth24Stencil8, Depth32FStencil8,

    // Compressed (optional support)
    DXT1, DXT3, DXT5, BC4, BC5, BC6H, BC7,

    // sRGB formats
    SRGB8, SRGBA8
};

enum class TextureUsage {
    Default,
    RenderTarget,
    DepthStencil,
};


struct TextureCreateInfo {
    TextureDimension dimension = TextureDimension::Texture2D;
    TextureFormat format = TextureFormat::RGBA8;
    TextureUsage usage = TextureUsage::Default;

    uint32_t width = 1;
    uint32_t height = 1;
    uint32_t depth = 1; // Only used for 3D

    bool generateMipmaps = false;
    bool sRGB = false;

    const void* initialData = nullptr;
    size_t dataSize = 0;

    std::string debugName;
};

class [[nodiscard]] MOSS_API Texture {
public:
    virtual ~Texture() = default;

    virtual void bind(uint32_t unit = 0) const = 0;
    virtual void unbind() const = 0;

    virtual uint32_t getWidth() const = 0;
    virtual uint32_t getHeight() const = 0;
    virtual uint32_t getDepth() const { return 1; } // Default for 2D

    virtual TextureFormat getFormat() const = 0;
    virtual TextureDimension getDimension() const = 0;

    const std::string& getDebugName() const { return debugName; }

protected:
    std::string debugName;
};


class [[nodiscard]] MOSS_API Texture2D : public Texture {
public:
    static std::unique_ptr<Texture2D> Create(const TextureCreateInfo& info);

    // inherit interface
};

class [[nodiscard]] MOSS_API Texture3D : public Texture {
public:
    static std::unique_ptr<Texture3D> Create(const TextureCreateInfo& info);

    virtual uint32_t getDepth() const override = 0;
};

std::unique_ptr<Texture2D> Texture2D::Create(const TextureCreateInfo& info) {
    switch (CurrentGraphicsAPI()) {
        case GraphicsAPI::OpenGL:
            return std::make_unique<OpenGLTexture2D>(info);
        case GraphicsAPI::Vulkan:
            return std::make_unique<VulkanTexture2D>(info);
        // ...
        default:
            throw std::runtime_error("Unsupported Graphics API");
    }
}



class Surface;

class Texture : public RefTarget<Texture>
{
public:
	/// Constructor
										Texture(int inWidth, int inHeight) : mWidth(inWidth), mHeight(inHeight) { }
	virtual								~Texture() = default;

	/// Get dimensions of texture
	inline int							GetWidth() const		{ return mWidth; }
	inline int							GetHeight() const		{ return mHeight; }

	/// Bind texture to the pixel shader
	virtual void						Bind() const = 0;

protected:
	int									mWidth;
	int									mHeight;
};
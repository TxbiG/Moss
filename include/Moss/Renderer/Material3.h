#ifndef MOSS_SURFACE_H
#define MOSS_SURFACE_H
#include <Moss/Core/Core.h>
#include <Moss/Core/Variants/Color.h>
#include <Moss/Core/Variants/Vector/Float2.h>
#include <Moss/Renderer/gl/TextureGL.h>
#include <Moss/Moss_Platform.h>
//#include <Moss/Renderer/Shader.h>

class Mat44;

enum class TextureSlot {
    Albedo,
    Specular,
    Metallic,
    Roughness,
    Normal,
    Diffuse,
    Displacement,
    Emission,
    Transparency,
    Reflection,
    Shadows,
    UV1,
    UV2,
    Custom0,
    Custom1
};

struct Texture {
    std::string name;
    uint32_t gpuHandle;     // API-specific
    int width, height;
    bool isSRGB;
};

enum class ShadingModel {
    PBR,
    Phong,
    BlinnPhong,
    Toon,
    Unlit
};

struct MaterialRenderState {
    bool depthTest = true;
    bool depthWrite = true;
    bool blending = false;
    bool cullBackFace = true;
    bool receiveShadows = true;
    bool castShadows = true;
};

class [[nodiscard]] MOSS_API StandardMaterial {
public:
    StandardMaterial();
    virtual ~StandardMaterial();

    // -------------------------
    // Setters
    // -------------------------
    void setAlbedo(const Vec4& color);
    void setSpecular(const Vec3& color);
    void setMetallic(float value);
    void setRoughness(float value);
    void setTransparency(float value);
    void setEmission(const Vec3& color, float intensity = 1.0f);
    void setReflection(float value);

    void setUVScale(const Vec2& scale);
    void setUVOffset(const Vec2& offset);

    void setTexture(TextureSlot slot, std::shared_ptr<Texture> texture);

    void setShadingModel(ShadingModel model);
    void setRenderState(const MaterialRenderState& state);


    void setPipelineState(PipelineState pipeline);
    // -------------------------
    // Getters
    // -------------------------
    Vec4 getAlbedo() const;
    Vec3 getSpecular() const;
    float getMetallic() const;
    float getRoughness() const;
    float getTransparency() const;
    Vec3 getEmission() const;
    float getReflection() const;
    Vec2 getUVScale() const;
    Vec2 getUVOffset() const;

    std::shared_ptr<Texture> getTexture(TextureSlot slot) const;
    ShadingModel getShadingModel() const;
    const MaterialRenderState& getRenderState() const;

    // -------------------------
    // Binding to GPU
    // -------------------------
    virtual void bind(const Mat44& viewProjectionMatrix, const Mat44& modelMatrix) = 0;

protected:
    // Core PBR/Phong Parameters
    Vec4 albedo {1.0f, 1.0f, 1.0f, 1.0f};
    Vec3 specular {0.5f, 0.5f, 0.5f};
    float metallic = 0.0f;
    float roughness = 0.5f;
    float transparency = 1.0f;
    Vec3 emission {0.0f, 0.0f, 0.0f};
    float emissionIntensity = 0.0f;
    float reflection = 0.0f;

    // UV Controls
    Vec2 uvScale {1.0f, 1.0f};
    Vec2 uvOffset {0.0f, 0.0f};

    // GPU Resources
    std::unordered_map<TextureSlot, std::shared_ptr<Texture>> textures;

    // Rendering Model
    ShadingModel shadingModel = ShadingModel::PBR;
    MaterialRenderState renderState;
};

#endif // MOSS_SURFACE_H
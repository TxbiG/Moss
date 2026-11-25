
#include <Moss/Renderer/Shader.h>
#include <Moss/Renderer/StorageBuffer.h>
#include <Moss/Renderer/PipelineState.h>

enum class LightFilter : uint8_t {
    Layer_1        = 1<<0,
    Layer_2        = 2<<0,
    Layer_3        = 3<<0,
    Layer_4        = 4<<0,
    Layer_5        = 5<<0,
    Layer_6        = 6<<0,
    Layer_7        = 7<<0,
    Layer_8        = 8<<0,
};

struct TextureLight2       { float intensity, rotation; Texture* texture; Float2 position; Color color; LightFilter filter; };
struct DirectionalLight2   { float intensity, rotation; Color color; LightFilter filter; };
struct PointLight2         { float intensity, radius; Float2 position; Color color; LightFilter filter; };

struct TextureLight3 { float intensity; Texture* texture; Float3 position, rotation; Color color; LightFilter filter; };
struct DirectionalLight3 { float intensity; Float3 rotation; Color color; };
struct SpotLight3 { float intensity, radius, angle, penumbra; Float3 position, rotation; Color color; LightFilter filter; };
struct OmniLight3 { float intensity, radius; Float3 position; Color color; LightFilter filter; };


class LightManager2D {
public:
    explicit LightManager2D(Renderer* renderer);
    ~LightManager2D() = default;

    void setDirectionalLight(const DirectionalLight2& light);
    void addPointLight(const PointLight2& light);
    void addTextureLight(const TextureLight2& light);

    void clear();

    // Should upload light data to active shader
    void draw() const;

private:
    struct LightData {
        DirectionalLight2 directionalLight;
        int hasDirectional = 0;
        std::vector<PointLight2> pointLights;
        std::vector<TextureLight2> textureLights;
    };
    Renderer* m_renderer;
    Ref<StorageBuffer> m_lightBuffer;
    LightData m_lightData;
    unique_ptr<PipelineState> mPipeline;
};



class LightManager3D {
public:
    LightManager3D(Renderer* renderer);
    ~LightManager3D() = default;

    void setDirectionalLight(const DirectionalLight3& light);
    void addSpotLight(const SpotLight3& light);
    void addOmniLight(const OmniLight3& light);
    void addTextureLight(const TextureLight3& light);

    void clear();

    // Upload light and shadow data to a backend-specific shader
    void draw() const;

    // Optional for engines that support shadows
    void renderShadows(const class Scene& scene);
private:
    struct LightData {
        DirectionalLight3 directionalLight;
        int hasDirectional = 0;
        std::vector<SpotLight3> spotLights;
        std::vector<OmniLight3> omniLights;
        std::vector<TextureLight3> textureLights;
    };

    Renderer* m_renderer;
    Ref<StorageBuffer> m_lightBuffer;
    LightData m_lightData;
    unique_ptr<PipelineState> mPipeline;
};

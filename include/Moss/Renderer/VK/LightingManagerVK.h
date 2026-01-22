#ifndef MOSS_LIGHTING_GL_H
#define MOSS_LIGHTING_GL_H

#include <Moss/Core/Core.h>
#include <Moss/Renderer/GL/ShaderGL.h>


/* Lighting Managers
 * Seperated from the renderer to reduce bloat
 * Includes the ability to Create 2D and 3D lights and shadows
 *
*/

struct [[nodiscard]] MOSS_API TextureLight2       { float intensity, rotation; Texture* texture; Float2 position; Color color; };
struct [[nodiscard]] MOSS_API DirectionalLight2   { float intensity, rotation; Color color; };
struct [[nodiscard]] MOSS_API PointLight2         { float intensity, rotation, radius; Float2 position; Color color; };

struct [[nodiscard]] MOSS_API TextureLight3 { float intensity; Texture* texture;Float3 position, rotation; Color color; };
struct [[nodiscard]] MOSS_API DirectionalLight3 { float intensity; Float3 rotation; Color color; };
struct [[nodiscard]] MOSS_API SpotLight3 { float intensity, radius, angle, penumbra; Float3 position, rotation; Color color; };
struct [[nodiscard]] MOSS_API OmniLight3 { float intensity, radius; Float3 position; Color color; };

class [[nodiscard]] MOSS_API LightManager3 {
public:
    LightManager3();
    ~LightManager3();

    void set(float intensity, Float3 rotation, Color color;)                                                                                { directionalLight = light; }
    SpotLight3* CreateSpotLight3(float intensity, float radius, float angle, float penumbra, Float3 position, Float3 rotation, Color color) { spotLights.push_back(light); }
    OmniLight3* CreateOmniLight3(float intensity, float radius, Float3 position, Color color)                                              { omniLights.push_back(light); }
    TextureLight3* CreateTextureLight3(float intensity, Texture* texture, Float3 position, rotation, Color color)                           { textureLights.push_back(light); }

    void clear() { spotLights.clear(); omniLights.clear(); textureLights.clear(); }

    // Uniform upload
    void draw() const {
    }

private:
    std::vector<DirectionalLight3*> directionalLight;
    std::vector<SpotLight3*>        spotLights;
    std::vector<OmniLight3*>        omniLights;
    std::vector<TextureLight3*>     textureLights;

    // Shadow map resources

    void initShadowResources();
    void renderDirectionalShadow(const Scene& scene);
    void renderSpotShadow(const SpotLight3& light, int index, const Scene& scene);
};


class [[nodiscard]] MOSS_API LightManager2 {
public:
    LightManager2();
    ~LightManager2();

    void CreateDirectionalLight2(float intensity, float rotation, Color color)                                  { directionalLight = light; }
    void CreatePointLight2(float intensity, float radius, float rotation, Float2 position, Color color)         { pointLights.push_back(light); }
    void CreateTextureLight2(float intensity, float rotation, Texture* texture, Float2 position, Color color)   { textureLights.push_back(light); }

    void clear() { pointLights.clear(); textureLights.clear(); }

    // Uniform upload
    void draw(ShaderGL* shader) const {
        shader.unbind();
    }

private:
    std::vector<DirectionalLight3*> directionalLight;
    std::vector<SpotLight3*>        pointLights;
    std::vector<TextureLight3*>     textureLights;


    // Shadow map resources
};

#endif // MOSS_LIGHTING_GL_H
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
        shader.bind();

        shader.setUniform1i("u_ShadowMap", shadowMapSlot); // e.g. 5
        glActiveTexture(GL_TEXTURE0 + shadowMapSlot);
        glBindTexture(GL_TEXTURE_2D, dirShadowMap);

        // Set light space matrix
        shader.setUniformMat4("u_LightSpaceMatrix", lightSpaceMatrix);

        // Directional Light
        shader.SetUniformVec3f("u_DirectionalLight.rotation", directionalLight.rotation);
        shader.SetUniformVec3f("u_DirectionalLight.color", directionalLight.color.toVec3());
        shader.SetUniformf("u_DirectionalLight.intensity", directionalLight.intensity);

        shader.SetUniformi("u_NumSpotLights", (int)spotLights.size());
        for (size_t i = 0; i < spotLights.size(); ++i) {
            const auto& l = spotLights[i];
            shader.SetUniformVec3f("u_SpotLights[" + std::to_string(i) + "].position", l.position);
            shader.SetUniformVec3f("u_SpotLights[" + std::to_string(i) + "].rotation", l.rotation);
            shader.SetUniformVec3f("u_SpotLights[" + std::to_string(i) + "].color", l.color.toVec3());
            shader.SetUniformf("u_SpotLights[" + std::to_string(i) + "].intensity", l.intensity);
            shader.SetUniformf("u_SpotLights[" + std::to_string(i) + "].radius", l.radius);
            shader.SetUniformf("u_SpotLights[" + std::to_string(i) + "].angle", l.angle);
            shader.SetUniformf("u_SpotLights[" + std::to_string(i) + "].penumbra", l.penumbra);
        }

        // OmniLighting
        shader.SetUniformi("u_NumOmniLights", (int)omniLights.size());
        for (size_t i = 0; i < omniLights.size(); ++i) {
            const auto& l = omniLights[i];
            shader.SetUniformVec3f("u_OmniLights[" + std::to_string(i) + "].position", l.position);
            shader.SetUniformVec3f("u_OmniLights[" + std::to_string(i) + "].rotation", l.rotation);
            shader.SetUniformVec3f("u_OmniLights[" + std::to_string(i) + "].color", l.color.toVec3());
            shader.SetUniformf("u_OmniLights[" + std::to_string(i) + "].intensity", l.intensity);
            shader.SetUniformf("u_OmniLights[" + std::to_string(i) + "].radius", l.radius);
            shader.SetUniformf("u_OmniLights[" + std::to_string(i) + "].angle", l.angle);
            shader.SetUniformf("u_OmniLights[" + std::to_string(i) + "].penumbra", l.penumbra);
        }

        // Texture Lighting
        shader.SetUniformi("u_NumTexLights", (int)omniLights.size());
        for (size_t i = 0; i < textureLights.size(); ++i) {
            const auto& l = textureLights[i];
            shader.SetUniformVec3f("u_TextureLights[" + std::to_string(i) + "].position", l.position);
            shader.SetUniformVec3f("u_TextureLights[" + std::to_string(i) + "].rotation", l.rotation);
            shader.SetUniformVec3f("u_TextureLights[" + std::to_string(i) + "].color", l.color.toVec3());
            shader.SetUniformf("u_TextureLights[" + std::to_string(i) + "].intensity", l.intensity);
        }
        shader.unbind();
    }

private:
    ShaderGL shader;
    std::vector<DirectionalLight3*> directionalLight;
    std::vector<SpotLight3*>        spotLights;
    std::vector<OmniLight3*>        omniLights;
    std::vector<TextureLight3*>     textureLights;

    // Shadow map resources
    ShaderGL shadowShader;
    GLuint dirShadowFBO = 0;
    GLuint dirShadowMap = 0;
    std::vector<GLuint> spotShadowFBOs;
    std::vector<GLuint> spotShadowMaps;

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
        shader.bind();

        shader.SetUniformi("u_NumSpotLights", (int)pointLights.size());
        shader.SetUniformi("u_NumOmniLights", (int)omniLights.size());

        shader.SetUniformVec3f("u_DirectionalLights[" + std::to_string(i) + "].rotation", directionalLight.rotation);
        shader.SetUniformVec3f("u_DirectionalLights[" + std::to_string(i) + "].color", directionalLight.color.toVec3());
        shader.SetUniformf("u_DirectionalLights[" + std::to_string(i) + "].intensity", directionalLight.intensity);

        for (size_t i = 0; i < pointLights.size(); ++i) {
            const auto& l = spotLights[i];
            shader.SetUniformVec3f("u_SpotLights[" + std::to_string(i) + "].position", l.position);
            shader.SetUniformVec3f("u_SpotLights[" + std::to_string(i) + "].rotation", l.rotation);
            shader.SetUniformVec3f("u_SpotLights[" + std::to_string(i) + "].color", l.color.toVec3());
            shader.SetUniformf("u_SpotLights[" + std::to_string(i) + "].intensity", l.intensity);
            shader.SetUniformf("u_SpotLights[" + std::to_string(i) + "].radius", l.radius);
            shader.SetUniformf("u_SpotLights[" + std::to_string(i) + "].angle", l.angle);
            shader.SetUniformf("u_SpotLights[" + std::to_string(i) + "].penumbra", l.penumbra);
        }

        for (size_t i = 0; i < textureLights.size(); ++i) {
            const auto& l = textureLights[i];
            shader.SetUniformVec3f("u_SpotLights[" + std::to_string(i) + "].position", l.position);
            shader.SetUniformVec3f("u_SpotLights[" + std::to_string(i) + "].rotation", l.rotation);
            shader.SetUniformVec3f("u_SpotLights[" + std::to_string(i) + "].color", l.color.toVec3());
            shader.SetUniformf("u_SpotLights[" + std::to_string(i) + "].intensity", l.intensity);
            shader.SetUniformf("u_SpotLights[" + std::to_string(i) + "].radius", l.radius);
            shader.SetUniformf("u_SpotLights[" + std::to_string(i) + "].angle", l.angle);
            shader.SetUniformf("u_SpotLights[" + std::to_string(i) + "].penumbra", l.penumbra);
        }
        shader.unbind();
    }

private:
    ShaderGL shader;
    std::vector<DirectionalLight3*> directionalLight;
    std::vector<SpotLight3*>        pointLights;
    std::vector<TextureLight3*>     textureLights;


    // Shadow map resources
    ShaderGL shadowShader;
    GLuint dirShadowFBO = 0;
    GLuint dirShadowMap = 0;
    std::vector<GLuint> spotShadowFBOs;
    std::vector<GLuint> spotShadowMaps;
};

#endif // MOSS_LIGHTING_GL_H
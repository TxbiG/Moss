#include <Moss/Renderer/GL/LightingGL.h>

const char* vertlighting2 = R"(
#version 330 core

layout (location = 0) in vec2 aPos;
layout (location = 1) in vec2 aTexCoord;

out vec2 TexCoords;
out vec2 FragPos;

uniform mat4 u_ViewProj;

void main()
{
    FragPos = aPos;
    TexCoords = aTexCoord;
    gl_Position = u_ViewProj * vec4(aPos, 0.0, 1.0);
})";

const char* fraglighting2 = R"(
#version 330 core

in vec2 TexCoords;
in vec2 FragPos;

out vec4 FragColor;

uniform sampler2D u_Texture;
uniform vec4 u_BaseColor;

struct DirectionalLight2 {
    float intensity;
    float rotation;
    vec3 color;
};

struct PointLight2 {
    float intensity;
    float radius;
    vec2 position;
    vec3 color;
};

struct TextureLight2 {
    float intensity;
    float rotation;
    vec2 position;
    vec3 color;
};

const int MAX_POINT_LIGHTS = 8;
uniform DirectionalLight2 u_DirLight2;
uniform PointLight2 u_PointLights2[MAX_POINT_LIGHTS];
uniform int u_NumPointLights2;

// Texture projection light
uniform TextureLight2 u_TextureLight2;
uniform sampler2D u_LightTexture2;
uniform vec2 u_Resolution2;

// Shadow
uniform sampler2D u_ShadowMap;
uniform mat4 u_LightSpaceMatrix;

void main()
{
    vec3 baseColor = texture(u_Texture, TexCoords).rgb * u_BaseColor.rgb;
    vec3 normal = vec3(0.0, 0.0, 1.0);
    vec3 lighting = vec3(0.0);

    // Directional
    vec2 dir = vec2(cos(u_DirLight2.rotation), sin(u_DirLight2.rotation));
    float diff = max(dot(normal.xy, -dir), 0.0);
    lighting += diff * u_DirLight2.color * u_DirLight2.intensity;

    // Point lights
    for (int i = 0; i < u_NumPointLights2; ++i)
    {
        vec2 lightDir = u_PointLights2[i].position - FragPos;
        float dist = length(lightDir);
        float attenuation = 1.0 - clamp(dist / u_PointLights2[i].radius, 0.0, 1.0);
        float d = max(dot(normal.xy, normalize(lightDir)), 0.0);
        lighting += d * u_PointLights2[i].color * u_PointLights2[i].intensity * attenuation;
    }

    // TextureLight2 projection
    vec2 rel = (FragPos - u_TextureLight2.position) / u_Resolution2;

    float s = sin(u_TextureLight2.rotation);
    float c = cos(u_TextureLight2.rotation);
    rel = mat2(c, -s, s, c) * rel + 0.5;

    vec4 texLight = texture(u_LightTexture2, rel);
    lighting += texLight.rgb * u_TextureLight2.color * u_TextureLight2.intensity * texLight.a;

    FragColor = vec4(baseColor * lighting, u_BaseColor.a);
})";

const char* vertlighting3 = R"(
#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec2 aTexCoord;

out vec3 FragPos;
out vec3 Normal;
out vec2 TexCoords;

uniform mat4 u_Model;
uniform mat4 u_View;
uniform mat4 u_Projection;

void main()
{
    FragPos = vec3(u_Model * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(u_Model))) * aNormal;
    TexCoords = aTexCoord;
    gl_Position = u_Projection * u_View * vec4(FragPos, 1.0);
})";

const char* fraglighting3 = R"(
#version 330 core

in vec3 FragPos;
in vec3 Normal;
in vec2 TexCoords;

out vec4 FragColor;

uniform sampler2D u_Diffuse;
uniform vec4 u_BaseColor;

struct DirectionalLight3 {
    vec3 rotation;
    float intensity;
    vec3 color;
};

struct SpotLight3 {
    vec3 position;
    vec3 rotation;
    float intensity;
    float radius;
    float angle;
    float penumbra;
    vec3 color;
};

struct OmniLight3 {
    vec3 position;
    float intensity;
    float radius;
    vec3 color;
};

struct TextureLight3 {
    vec3 position;
    vec3 rotation;
    float intensity;
    vec3 color;
};

/ === Constants ===
const int MAX_OMNI_LIGHTS    = 8;
const int MAX_SPOT_LIGHTS    = 4;
const int MAX_TEXTURE_LIGHTS = 4;

// === Light Uniforms ===
uniform DirectionalLight3 u_DirLight3;
uniform SpotLight3 u_SpotLights[MAX_SPOT_LIGHTS];
uniform OmniLight3 u_OmniLights[MAX_OMNI_LIGHTS];
uniform TextureLight3 u_TextureLights[MAX_TEXTURE_LIGHTS];
uniform sampler2D u_LightTexture3;

uniform int u_NumSpotLights;
uniform int u_NumOmniLights;
uniform int u_NumTexLights;

// === Shadow Uniforms ===
uniform mat4 u_LightSpaceMatrix;
uniform sampler2D u_ShadowMap;  // Directional lighting

uniform mat4 u_SpotLightSpaceMatrices[MAX_SPOT_LIGHTS];
uniform sampler2D u_SpotShadowMaps[MAX_SPOT_LIGHTS];

// shadow calc
// Directional Shadow
float calculateShadow(vec4 fragPosLightSpace, vec3 normal, vec3 lightDir)
{
    vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
    projCoords = projCoords * 0.5 + 0.5;

    float closestDepth = texture(u_ShadowMap, projCoords.xy).r;
    float currentDepth = projCoords.z;

    float bias = max(0.005 * (1.0 - dot(normal, lightDir)), 0.001);

    return currentDepth - bias > closestDepth ? 1.0 : 0.0;
}

// Spotlight Shadow
float calculateSpotShadow(int i, vec4 fragPosLightSpace, vec3 normal, vec3 lightDir)
{
    vec3 projCoords = fragPosLightSpace.xyz / fragPosLightSpace.w;
    projCoords = projCoords * 0.5 + 0.5;

    float closestDepth = texture(u_SpotShadowMaps[i], projCoords.xy).r;
    float currentDepth = projCoords.z;

    float bias = max(0.005 * (1.0 - dot(normal, lightDir)), 0.001);

    return currentDepth - bias > closestDepth ? 1.0 : 0.0;
}


void main()
{
    vec3 albedo = texture(u_Diffuse, TexCoords).rgb * u_BaseColor.rgb;
    vec3 norm = normalize(Normal);
    vec3 lighting = vec3(0.0);

    // === Directional Light ===
    vec3 dirLightDir = normalize(-u_DirLight3.rotation);
    float diff = max(dot(norm, dirLightDir), 0.0);
    vec4 fragPosLightSpace = u_LightSpaceMatrix * vec4(FragPos, 1.0);
    float shadow = calculateShadow(fragPosLightSpace, norm, dirLightDir);
    lighting += (1.0 - shadow) * diff * u_DirLight3.color * u_DirLight3.intensity;

    // === Omni Lights ===
    for (int i = 0; i < u_NumOmniLights; ++i)
    {
        vec3 toLight = u_OmniLights[i].position - FragPos;
        float distance = length(toLight);
        vec3 lightDir = normalize(toLight);
        float attenuation = 1.0 - clamp(distance / u_OmniLights[i].radius, 0.0, 1.0);
        float d = max(dot(norm, lightDir), 0.0);
        lighting += d * u_OmniLights[i].color * u_OmniLights[i].intensity * attenuation;
    }

    // === Spot Lights ===
    for (int i = 0; i < u_NumSpotLights; ++i)
    {
        vec3 toLight = u_SpotLights[i].position - FragPos;
        float distance = length(toLight);
        vec3 lightDir = normalize(toLight);
        float attenuation = 1.0 - clamp(distance / u_SpotLights[i].radius, 0.0, 1.0);

        float theta = dot(lightDir, normalize(-u_SpotLights[i].rotation));
        float epsilon = max(u_SpotLights[i].angle - u_SpotLights[i].penumbra, 0.001);
        float spotEffect = clamp((theta - u_SpotLights[i].penumbra) / epsilon, 0.0, 1.0);

        float d = max(dot(norm, lightDir), 0.0);

        // Shadow calculation
        vec4 spotLightSpacePos = u_SpotLightSpaceMatrices[i] * vec4(FragPos, 1.0);
        float shadow = calculateSpotShadow(i, spotLightSpacePos, norm, lightDir);

        lighting += (1.0 - shadow) * d * u_SpotLights[i].color * u_SpotLights[i].intensity * attenuation * spotEffect;
    }

    // === Texture Projector Lights ===
    for (int i = 0; i < u_NumTexLights; ++i)
    {
        vec3 toLight = FragPos - u_TextureLights[i].position;
        vec2 uv = toLight.xy * 0.5 + 0.5; // naive planar projection

        vec4 texSample = texture(u_LightTexture3, uv);

        float d = max(dot(norm, normalize(-u_TextureLights[i].rotation)), 0.0);
        vec3 contribution = texSample.rgb * u_TextureLights[i].color * u_TextureLights[i].intensity * d * texSample.a;
        lighting += contribution;
    }

    FragColor = vec4(albedo * lighting, u_BaseColor.a);
})";


LightManager3::LightManager3() : shader(vertlighting3, fraglighting3, true) {}
LightManager3::~LightManager3() {
    spotLights.clear();
    omniLights.clear();
    textureLights.clear();
}

void LightManager3::initShadowResources() {
    // --- Directional Light Shadow Map ---
    const int SHADOW_RES = 2048;

    glGenFramebuffers(1, &dirShadowFBO);
    glGenTextures(1, &dirShadowMap);
    glBindTexture(GL_TEXTURE_2D, dirShadowMap);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, SHADOW_RES, SHADOW_RES, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
    float borderColor[] = {1.0, 1.0, 1.0, 1.0};
    glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);

    glBindFramebuffer(GL_FRAMEBUFFER, dirShadowFBO);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, dirShadowMap, 0);
    glDrawBuffer(GL_NONE);
    glReadBuffer(GL_NONE);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // --- SpotLight Shadow Maps ---
    for (int i = 0; i < spotLights.size(); ++i) {
        GLuint fbo, map;
        glGenFramebuffers(1, &fbo);
        glGenTextures(1, &map);
        glBindTexture(GL_TEXTURE_2D, map);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, SHADOW_RES, SHADOW_RES, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
        glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);

        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, map, 0);
        glDrawBuffer(GL_NONE);
        glReadBuffer(GL_NONE);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);

        spotShadowFBOs.push_back(fbo);
        spotShadowMaps.push_back(map);
    }
}


void LightManager3::renderShadowMaps(const Scene& scene) {
    renderDirectionalShadow(scene);
    for (size_t i = 0; i < spotLights.size(); ++i) { renderSpotShadow(spotLights[i], i, scene); }
}

void LightManager3::renderDirectionalShadow(const Scene& scene) {
    glViewport(0, 0, 2048, 2048);
    glBindFramebuffer(GL_FRAMEBUFFER, dirShadowFBO);
    glClear(GL_DEPTH_BUFFER_BIT);

    // Create light view/projection matrix
    Mat44 lightProjection = Mat4::Orthographic(-20, 20, -20, 20, 0.1f, 100.f);
    Mat44 lightView = Mat4::LookAt(
        -directionalLight.rotation * 20.0f, // Light position (far along rotation dir)
        Float3(0, 0, 0),                     // Look at origin
        Float3(0, 1, 0)
    );

    Mat44 lightSpaceMatrix = lightProjection * lightView;

    // Send matrix to shadow shader
    shadowShader.bind();
    shadowShader.SetUniformMat4("u_LightSpaceMatrix", lightSpaceMatrix);

    scene.renderDepthOnly(shadowShader);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void LightManager3::renderSpotShadow(const SpotLight3& light, int index, const Scene& scene) {
    glViewport(0, 0, 2048, 2048);
    glBindFramebuffer(GL_FRAMEBUFFER, spotShadowFBOs[index]);
    glClear(GL_DEPTH_BUFFER_BIT);

    Mat44 lightProjection = Mat4::Perspective(light.angle, 1.0f, 0.1f, light.radius);
    Mat44 lightView = Mat4::LookAt(light.position, light.position + light.rotation, Float3(0, 1, 0));
    Mat44 lightSpaceMatrix = lightProjection * lightView;

    shadowShader.bind();
    shadowShader.SetUniformMat4("u_LightSpaceMatrix", lightSpaceMatrix);

    scene.renderDepthOnly(shadowShader);

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}



LightManager2::LightManager2() : shader(vertlighting2, fraglighting2, true) {}
LightManager2::~LightManager2() {
    pointLights.clear();
    textureLights.clear();
}
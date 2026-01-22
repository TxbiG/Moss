


class [[nodiscard]] MOSS_API ReflectionProbe {
public:
    Vec3 position;
    float radius = 10.0f;

    bool dynamic = false;
    GLuint cubemapTexture = 0;     // 6-face texture
    GLuint framebuffer = 0;        // Per-face FBO for rendering

    void Init(uint32 size = 256);  // Allocate cubemap and FBO
    void RenderSceneToCubemap(Scene& scene);  // Render from 6 directions

    void Bind(GLuint unit);  // Bind for use in shaders
};







class SubViewport {
public:
    virtual ~SubViewport() = default;

    /// Set viewport size in pixels
    virtual void setSize(int width, int height) = 0;

    /// Set screen-space position (used for scissor or output layout)
    virtual void setPosition(int x, int y) = 0;

    /// Prepare rendering to this viewport (bind FBO, set viewport, etc.)
    virtual void beginRender() = 0;

    /// End rendering and unbind any state (restore previous FBO, viewport, etc.)
    virtual void endRender() = 0;

    /// Get the texture containing the rendered content (may be nullptr if not used)
    virtual Ref<Texture> getRenderTexture() const = 0;
};
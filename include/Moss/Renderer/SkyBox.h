

#include <Moss/Renderer/Renderer.h>
#include <Moss/Renderer/PipelineState.h>

class SkyBox : public RefTarget<SkyBox> {
public:
    SkyBox(Renderer* inRenderer, Ref<Texture> inTexture) : mRenderer(inRenderer), m_texture(std::move(inTexture))  {
        const PipelineState::EInputDescription vertex_desc[] = { PipelineState::EInputDescription::Position };

        // Load vertex and pixel shaders
        Ref<VertexShader> vtx = mRenderer->CreateVertexShader("SkyBoxVertexShader");
        Ref<PixelShader> pix = mRenderer->CreatePixelShader("SkyBoxPixelShader");

        // Create pipeline state
        pipeline = mRenderer->CreatePipelineState(
            vtx,
            vertex_desc,
            std::size(vertex_desc),
            pix,
            PipelineState::EDrawPass::Normal,
            PipelineState::EFillMode::Solid,
            PipelineState::ETopology::Triangle,
            PipelineState::EDepthTest::On,
            PipelineState::EBlendMode::AlphaBlend,
            PipelineState::ECullMode::FrontFace  // Skybox usually uses FrontFace to render inside of cube
        );

        // Create cube primitive for the skybox
        primitive = mRenderer->CreateRenderPrimitive(PipelineState::ETopology::Triangle);
        // Assume primitive is filled elsewhere or uses a cube mesh
    }

    ~SkyBox() = default;

    void Draw(const Mat44& matrixProjection) {
        m_texture->Bind();            // You may need to bind to a slot
        pipeline->Activate();
        primitive->Draw();            // You may need to set transform before this
        m_texture->Unbind();
    }

private:
    Renderer* mRenderer = nullptr;
    unique_ptr<PipelineState> pipeline;
    Ref<Mesh> primitive;
    Ref<Texture> m_texture;
};
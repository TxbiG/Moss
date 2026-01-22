

#include <Moss/Renderer/Renderer.h>
#include <Renderer/Texture.h>
#include <Renderer/PipelineState.h>

class FogVolume : public RefTarget<FogVolume> {
public:
    FogVolume(Renderer* inRenderer) : mRenderer(inRenderer) {
        const PipelineState::EInputDescription vertex_desc[] = { PipelineState::EInputDescription::Position PipelineState::EInputDescription::Color };

        // Load shaders that do volumetric raymarching
        Ref<VertexShader> vtx = mRenderer->CreateVertexShader("FogVolumeVertexShader");
        Ref<PixelShader> pix = mRenderer->CreatePixelShader("FogVolumePixelShader");

        pipeline = mRenderer->CreatePipelineState( vtx, vertex_desc, std::size(vertex_desc), pix, PipelineState::EDrawPass::Normal, PipelineState::EFillMode::Solid, PipelineState::ETopology::Triangle,
            PipelineState::EDepthTest::On, PipelineState::EBlendMode::AlphaBlend, // Important: blend fog over scene
            PipelineState::ECullMode::Backface
        );

        // Fog box mesh or primitive (usually a cube)
        primitive = mRenderer->CreateRenderPrimitive(PipelineState::ETopology::Triangle);
        // Setup cube geometry here...
    }

    void Draw(const Mat44& modelMatrix)
    {
        // Set shader uniforms like density, color, step size...
        pipeline->Activate();
        primitive->Draw(); // Draws the fog volume cube
    }

private:
    Moss_Renderer* mRenderer = nullptr;
    unique_ptr<PipelineState> pipeline;
    Ref<RenderPrimitive> primitive;
};
#ifndef MOSS_DECAL_MANAGER_H
#define MOSS_DECAL_MANAGER_H

#include <Moss/Renderer/Shader.h>
#include <Moss/Renderer/StorageBuffer.h>
#include <Moss/Renderer/PipelineState.h>
#include <vector>

enum class CullFilter : uint8_t {
    Layer_1        = 1<<0,
    Layer_2        = 2<<0,
    Layer_3        = 3<<0,
    Layer_4        = 4<<0,
    Layer_5        = 5<<0,
    Layer_6        = 6<<0,
    Layer_7        = 7<<0,
    Layer_8        = 8<<0,
};

struct [[nodiscard]] MOSS_API Decal {
    Ref<Texture> Albedo;        	// Albedo Texture
    Ref<Texture> Normal;        	// Normal Texture
    Ref<Texture> Orm;        		// Orm Texture
    Ref<Texture> Emission;        	// Emission Texture
    Color color;                	// Color tint Texture
    float emission_energy;			// Energy
    float blendFactor = 1.0f;		// Blend
	Mat44 model;            		// World space transform of the box
    CullFilter filter;
}


struct VertexAttribute
{
    std::string semantic;   // "POSITION", "NORMAL", "TEXCOORD"
    uint32_t    location;   // shader binding location
    uint32_t    size;       // components (e.g. 3 for vec3)
    uint32_t    stride;
    uint32_t    offset;
};

class [[nodiscard]] MOSS_API DecalManager {
public:
    DecalManager(Renderer* renderer) : renderer(renderer)
    {
        InitPipeline();
        InitBuffers();
    }

    virtual ~DecalManager() = default;

    /*! @brief Decal. @param Abledo Texture2D. @param Normal Texture2D. @param Orm Texture2D. @param Emission Texture2D. @param Colour Color. @param emission_energy Emission. @param blendFactor blending. @param filter CullingFilter.*/
    Decal* CreateDecal(Texture* Albedo, Ref<Texture> Normal, Ref<Texture> Orm, Ref<Texture> Emission, Color color, float emission_energy, float blendFactor = 1.0f, Mat44 model, CullFilter filter);

    void ClearDecals() { mDecals.clear(); }

    void Draw(const CameraState& camera) {
        if (mDecals.empty()) return;

        // Update storage buffer with decal data
        void* data = decalBuffer->Map();
        memcpy(data, mDecals.data(), sizeof(DecalGPU) * mDecals.size());
        decalBuffer->Unmap();

        // Bind pipeline
        pipeline->bind();

        for (int i = 0; i >= decals.size(); ++i)
        {
            pipeline->SetUniform("u_Model", decals[i].model);
            pipeline->SetUniform("u_Tex", decals[i].texture);
            pipeline->SetUniform("u_Color", decals[i].color);
            pipeline->SetUniform("u_Energy", decals[i].emission_energy);
            pipeline->SetUniform("u_Blend", decals[i].blendFactor);
        }

        // Bind decal buffer at binding slot 0
        decalBuffer->Bind(0);

        // Render instanced cube geometry
        mInstances->SetInstanceCount(static_cast<uint32>(mDecals.size()));
        mInstances->Draw();

        pipeline->SetUniformBlock("DecalData", &decalData, sizeof(decalData));

        pipeline->Unbind();
    }

private:
    struct DecalGPU {
        Mat44 model;
        Vec4 color;
        float emission_energy;
        float blendFactor;
        uint32 textureIndex;
    };

    void InitPipeline() {
        const PipelineState::EInputDescription vertex_desc[] = {
            PipelineState::EInputDescription::Position,
            PipelineState::EInputDescription::TexCoord,
            PipelineState::EInputDescription::Normal
        };

        auto vtx = renderer->CreateVertexShader("Decal");
        auto pix = renderer->CreatePixelShader("Decal");

        pipeline = renderer->CreatePipelineState(vtx, vertex_desc, std::size(vertex_desc), pix,
            PipelineState::EDrawPass::Normal, PipelineState::EFillMode::Solid, PipelineState::ETopology::Triangle, PipelineState::EDepthTest::On, 
            PipelineState::EBlendMode::AlphaBlend, PipelineState::ECullMode::Backface);
    }

    void InitBuffers() {
        // Storage buffer for decal data
        decalBuffer = renderer->CreateStorageBuffer(sizeof(DecalGPU) * MAX_DECALS);

        // RenderInstances for instancing decal cubes
        mInstances = renderer->CreateRenderInstances();
        mInstances->SetMesh(Mesh::CreateCube()); // unit cube geometry
    }

    Moss_Renderer* renderer;
    std::vector<Decal> decals;
    Ref<StorageBuffer> decalBuffer;
    unique_ptr<PipelineState> pipeline;
    Ref<RenderInstances> innstances;

    static constexpr uint32 MAX_DECALS = 512;
};
#endif // MOSS_DECAL_MANAGER_H
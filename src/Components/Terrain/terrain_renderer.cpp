#include "Terrain.h"


class TerrainRenderer final {
public:
    TerrainRenderer();
    ~TerrainRenderer();

    void Initialize();
    void Render(const Terrain& terrain);

private:
    // GPU-driven culling + LOD
    Shader m_cullCompute;

    // Terrain rendering pipeline
    Shader m_vertex;
    Shader m_tessControl;
    Shader m_tessEval;
    Shader m_fragment;

    // GPU buffers
    Buffer m_chunkSSBO;
    Buffer m_indirectBuffer;

    Texture m_heightmap;
    Texture m_splatMap;

private:
    void RunCullingPass(const Terrain& terrain);
};

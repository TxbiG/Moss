//                        MIT License
//
//                  Copyright (c) 2025 Toby
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/*!
 * @file Landscape.h
 * @brief Open-world, chunked, GPU-driven terrain system.
 *
 * Architecture:
 *  - TerrainWorld   : streaming, paging, camera-based logic
 *  - TerrainChunk   : lightweight chunk data (no rendering)
 *  - TerrainRenderer: compute + tessellation + fragment rendering
 *  - TerrainPhysics : Jolt heightfield collision
 *  - FoliageSystem  : GPU-instanced vegetation
 */

#ifndef MOSS_LANDSCAPE3D_H
#define MOSS_LANDSCAPE3D_H

#include <vector>
#include <memory>
#include <unordered_map>
#include <string>
#include <cstdint>

#include <Moss/Moss_stdinc.h>
#include <Moss/Moss_Renderer.h>
#include <Moss/Moss_Physics.h>

#include <Moss/Renderer/Shader.h>
#include <Moss/Renderer/Buffer.h>
#include <Moss/Renderer/Texture.h>
#include <Moss/Renderer/Frustum.h>

#include <Moss/Math/AABox.h>
#include <Moss/Core/Variants/Vector/Vec2.h>
#include <Moss/Core/Variants/Vector/Vec3.h>

#include "Camera3.h"

//
// =======================================
// Terrain Chunk (DATA ONLY)
// =======================================
//

struct TerrainChunkCoord
{
    int32_t x;
    int32_t y;

    bool operator==(const TerrainChunkCoord& other) const noexcept
    {
        return x == other.x && y == other.y;
    }
};

struct TerrainChunkCoordHash
{
    size_t operator()(const TerrainChunkCoord& c) const noexcept
    {
        return std::hash<int32_t>()(c.x) ^ (std::hash<int32_t>()(c.y) << 1);
    }
};

/**
 * @brief Lightweight terrain chunk representation.
 *        Contains NO rendering or physics resources.
 */
class [[nodiscard]] TerrainChunk final
{
public:
    TerrainChunk(const TerrainChunkCoord& coord, uint32_t chunkSize);

    const TerrainChunkCoord& GetCoord() const noexcept { return m_coord; }
    const AABox& GetBounds() const noexcept { return m_bounds; }

    uint32_t GetLOD() const noexcept { return m_lod; }
    void SetLOD(uint32_t lod) noexcept { m_lod = lod; }

    bool HasPhysics() const noexcept { return m_hasPhysics; }
    bool HasFoliage() const noexcept { return m_hasFoliage; }

    void SetPhysicsEnabled(bool enabled) noexcept { m_hasPhysics = enabled; }
    void SetFoliageEnabled(bool enabled) noexcept { m_hasFoliage = enabled; }

private:
    TerrainChunkCoord m_coord;
    AABox m_bounds;

    uint32_t m_lod = 0;

    bool m_hasPhysics = false;
    bool m_hasFoliage = false;
};

//
// =======================================
// Terrain World (STREAMING / PAGING)
// =======================================
//

class [[nodiscard]] TerrainWorld final
{
public:
    TerrainWorld(uint32_t chunkSize,
                 uint32_t worldSize,
                 float maxHeight,
                 Camera3& camera);

    ~TerrainWorld();

    bool LoadHeightmap(const std::string& filePath);

    void Update();

    const std::unordered_map<TerrainChunkCoord,
                             std::unique_ptr<TerrainChunk>,
                             TerrainChunkCoordHash>&
    GetChunks() const noexcept;

    float GetHeightAt(uint32_t x, uint32_t y) const;

    uint32_t GetChunkSize() const noexcept { return m_chunkSize; }
    uint32_t GetWorldSize() const noexcept { return m_worldSize; }
    float    GetMaxHeight() const noexcept { return m_maxHeight; }

private:
    void StreamChunks();
    TerrainChunkCoord GetCameraChunk() const;

private:
    uint32_t m_chunkSize;
    uint32_t m_worldSize;
    float    m_maxHeight;

    std::vector<float> m_heightData;

    std::unordered_map<TerrainChunkCoord,
                       std::unique_ptr<TerrainChunk>,
                       TerrainChunkCoordHash> m_chunks;

    Camera3& m_camera;
};

//
// =======================================
// Terrain Renderer (GPU-DRIVEN)
// =======================================
//

class [[nodiscard]] TerrainRenderer final
{
public:
    TerrainRenderer();
    ~TerrainRenderer();

    void Initialize();
    void Render(const TerrainWorld& world, const Camera3& camera);

private:
    void RunCullingCompute(const TerrainWorld& world, const Camera3& camera);

private:
    // Compute shader for culling + LOD
    Shader m_cullCompute;

    // Terrain pipeline
    Shader m_vertexShader;
    Shader m_tessControlShader;
    Shader m_tessEvalShader;
    Shader m_fragmentShader;

    // GPU buffers
    Buffer m_chunkSSBO;
    Buffer m_indirectDrawBuffer;

    // Terrain textures
    Texture m_heightmap;
    Texture m_splatMap;
};

//
// =======================================
// Foliage System (GPU INSTANCING)
// =======================================
//

struct FoliageInstance
{
    Vec3 position;
    float scale;
    float rotation;
};

class [[nodiscard]] FoliageSystem final
{
public:
    FoliageSystem();
    ~FoliageSystem();

    void GenerateForChunk(const TerrainChunk& chunk,
                          const TerrainWorld& world);

    void RemoveChunk(const TerrainChunk& chunk);

    void Render(const Camera3& camera);

private:
    Buffer m_instanceBuffer;
    uint32_t m_instanceCount = 0;
};

//
// =======================================
// Terrain Physics (JOLT HEIGHTFIELD)
// =======================================
//

class [[nodiscard]] TerrainPhysics final
{
public:
    TerrainPhysics(PhysicsSystem& physicsSystem);
    ~TerrainPhysics();

    void CreateChunkCollider(const TerrainChunk& chunk,
                             const TerrainWorld& world);

    void RemoveChunkCollider(const TerrainChunk& chunk);

private:
    PhysicsSystem& m_physicsSystem;
};

//
// =======================================
// Open World Terrain Facade
// =======================================
//

/**
 * @brief High-level terrain system tying together
 *        world, renderer, physics, and foliage.
 */
class [[nodiscard]] OpenWorldTerrain3D final
{
public:
    OpenWorldTerrain3D(uint32_t chunkSize,
                       uint32_t worldSize,
                       float maxHeight,
                       Camera3& camera,
                       PhysicsSystem& physicsSystem);

    ~OpenWorldTerrain3D();

    bool LoadHeightmap(const std::string& filePath);

    void Update();
    void Render();

private:
    TerrainWorld    m_world;
    TerrainRenderer m_renderer;
    TerrainPhysics  m_physics;
    FoliageSystem   m_foliage;

    Camera3& m_camera;
};

#endif // MOSS_LANDSCAPE3D_H
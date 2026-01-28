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
 * @file Terrain3D.h
 * @brief Chunked heightmap-based 3D terrain system with LOD and frustum culling.
 *
 * Responsibilities:
 *  - Load heightmap data
 *  - Divide world into terrain chunks
 *  - Handle LOD selection and streaming
 *  - Perform frustum culling
 */

#ifndef MOSS_TERRAIN_H
#define MOSS_TERRAIN_H

//World-to-Chunk Mapping or Paging
// Actual LOD Mesh Adjustment
// Normal/Tangent Generation

// Frustum Debugging

#include <vector>
#include <memory>
#include <string>
#include <cstdint>

#include <Moss/Core/Variants/Vector/Vec3.h>
#include <Moss/Renderer/Shader.h>
#include <Moss/Renderer/Texture.h>
#include "Camera3.h"
#include <Moss/Renderer/Frustum.h>

#include <Moss/Moss_stdinc.h>
#include <Moss/Moss_Renderer>
#include <Moss/Moss_Physics.h>

class TerrainChunk;

/*! @class Terrain. @brief Manages terrain chunks and world-level terrain behavior. */
class [[nodiscard]] MOSS_API Terrain final
{
public:
    Terrain(uint32_t chunkSize, uint32_t worldSize, float maxHeight, Camera3& camera);
    ~Terrain();

    Terrain(const Terrain&)            = delete;
    Terrain& operator=(const Terrain&) = delete;

    /*! @brief Loads terrain height data from an image heightmap. @param filePath Path to heightmap image. @return True on success. */
    bool LoadFromHeightmap(const std::string& filePath);

    /*! @brief Updates chunk LODs and streaming based on camera position. */
    void Update();

    /*! @brief Renders all visible terrain chunks. */
    void Render(Moss_Renderer renderer);

    /*! @brief Gets world height at a given integer coordinate. */
    float GetHeightAt(uint32_t x, uint32_t y) const;

    /*! @brief Returns terrain world size in vertices. */
    uint32_t GetWorldSize() const noexcept { return m_worldSize; }

    /*! @brief Returns terrain chunk size. */
    uint32_t GetChunkSize() const noexcept { return m_chunkSize; }

    /*! @brief Returns maximum terrain height. */
    float GetMaxHeight() const noexcept { return m_maxHeight; }

private:
    friend class TerrainChunk;

    void GenerateChunks();

    uint32_t m_chunkSize;
    uint32_t m_worldSize;
    float    m_maxHeight;

    std::vector<float> m_heightData;
    std::vector<std::unique_ptr<TerrainChunk>> m_chunks;

    Camera3& m_camera;
};


// ================================
// TerrainChunk
// ================================

/*! @class TerrainChunk. @brief Represents a single chunk of terrain. */
class [[nodiscard]] TerrainChunk final
{
public:
    TerrainChunk(Terrain* parent, uint32_t chunkX, uint32_t chunkY, uint32_t chunkSize);
    ~TerrainChunk();

    TerrainChunk(const TerrainChunk&)            = delete;
    TerrainChunk& operator=(const TerrainChunk&) = delete;

    /*!
     * @brief Updates LOD state based on camera position.
     */
    void Update(const Vec3& cameraPosition);

    /*! @brief Renders this terrain chunk.
     */
    void Render();

    /*!
     * @brief Performs frustum visibility test.
     */
    bool IsVisible(const Frustum& frustum) const;

private:
    /*! @brief Generates the chunk mesh at current LOD. */
    void GenerateMesh();

    /*! @brief Calculates world-space bounding box. */
    void CalculateBounds();

    /*! @brief Updates mesh LOD based on distance. */
    void UpdateLOD(float distanceToCamera);

private:
    Terrain* m_parent;

    uint32_t m_chunkX;
    uint32_t m_chunkY;
    uint32_t m_chunkSize;

    Vec3 m_worldPosition;

    float m_lodFactor;
    uint32_t m_currentLOD;

    AABox m_bounds;

    // CPU mesh data
    std::vector<Vec3> m_positions;
    std::vector<Vec3> m_normals;
    std::vector<Vec2> m_uvs;
    std::vector<uint32_t> m_indices;

    // GPU resources
    uint32_t m_vao;
    uint32_t m_vbo;
    uint32_t m_ebo;

    Shader  m_shader;
    Texture m_texture;
};

#endif // MOSS_TERRAIN_H
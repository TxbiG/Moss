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
 * @brief X.
 */

#ifndef MOSS_OCEAN_H
#define MOSS_OCEAN_H

//World-to-Chunk Mapping or Paging
// Actual LOD Mesh Adjustment
// Normal/Tangent Generation

// Frustum Debugging

#include <vector>
#include <memory>
#include <string>
#include <Moss/Core/Variants/Vector/Vec3.h>
#include <Moss/Renderer/Shader.h>
#include <Moss/Renderer/Texture.h>
#include "Camera3.h"
#include <Moss/Renderer/Frustum.h>

#include <Moss/Moss_stdinc.h>
#include <Moss/Moss_Renderer>
#include <Moss/Moss_Physics.h>

class TerrainChunk;
struct Foliage;

class [[nodiscard]] Ocean
{
public:
    Terrain(uint32_t chunkSize, uint32_t worldSize, Camera3& camera);
    ~Terrain();

    bool LoadFromHeightmap(const std::string& filePath, float maxHeight = 100.0f);

    void Update();                          // Handles LOD switching, streaming
    void Render(Moss_Renderer renderer);    // Render visible chunks

private:
    friend class TerrainChunk;

    void GenerateChunks();
    float GetHeightAt(int x, int y) const;

    uint32_t m_chunkSize;
    uint32_t m_worldSize;
    float m_maxHeight;

    std::vector<float> m_heightData; // From heightmap
    std::vector<std::unique_ptr<TerrainChunk>> m_chunks;

    Camera3& m_camera;
};

// ==========================
// TerrainChunk Inner Class
// ==========================

class [[nodiscard]] TerrainChunk
{
public:
    TerrainChunk(Terrain* parent, int chunkX, int chunkY, uint32_t chunkSize);
    ~TerrainChunk();

    void Update(const Vec3& cameraPosition);
    void Render();
    bool IsChunkVisible(const Frustum& frustum) const;

private:
    void GenerateMesh();  // Generates vertices/indices from height data
    void CalculateBounds();

    Terrain* m_parent;
    int m_chunkX, m_chunkY;
    uint32_t m_chunkSize;

    Vec3 m_position;
    float m_lodFactor;

    AABox m_bounds;  // Bounding box

    std::vector<float> vertices;
    std::vector<unsigned int> indices;

    uint32_t vertexCount;

    Shader m_shader;    // Own shader instance
    Texture m_texture;  // Optional chunk-specific texture (or region in atlas)
};
#endif // MOSS_OCEAN_H
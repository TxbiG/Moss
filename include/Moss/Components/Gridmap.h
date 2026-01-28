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
 * @file Gridmap3D.h
 * @brief X.
 */

#ifndef MOSS_GRIDMAP_H
#define MOSS_GRIDMAP_H


#include <unordered_map>
#include <memory>
#include <string>

#include <Moss/Moss_stdinc.h>
#include <Moss/Moss_Renderer.h>
#include <Moss/Moss_Physics.h>

class MeshInstance;


// Todo:
// Support LODs and batching meshes per chunk
// Add Collision data

struct GridCell {
    std::shared_ptr<MeshInstance> instance;
    uint32_t meshId;    // Identifier to use with mesh registry
};

class GridMap {
public:
    GridMap(float cellSize = 1.0f);

    void SetCell(Float3 position, uint32_t& meshId);
    void RemoveCell(Float3 position);
    GridCell* GetCell(Float3 position);
    bool HasCell(Float3 position) const;

    Vec3 GridToWorld(int x, int y, int z) const;
    iVec3 WorldToGrid(const Vec3& worldPos) const;

private:
    float m_cellSize;

    // Key = linearized position or hash of (x,y,z)
    std::unordered_map<std::string, GridCell> m_cells;

    std::string MakeKey(Float3 position) const;
};

#endif // MOSS_GRIDMAP_H
#include "Terrain.h"
#include <cmath>



// --- Constructor ---
Terrain::Terrain(uint32_t chunkSize, uint32_t worldSize, Camera3& camera)
    : m_chunkSize(chunkSize), m_worldSize(worldSize), m_camera(camera)
{
    GenerateChunks();
}

// --- Destructor ---
Terrain::~Terrain() {
    // Smart pointers clean up chunks automatically
}

// --- Load From Heightmap ---
bool Terrain::LoadFromHeightmap(const std::string& filePath, float maxHeight) {
    int width, height, channels;
    unsigned char* data = stbi_load(filePath.c_str(), &width, &height, &channels, 1);
    if (!data) {
        std::cerr << "Failed to load heightmap: " << filePath << "\n";
        return false;
    }

    m_maxHeight = maxHeight;
    m_heightData.resize(width * height);

    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x)
            m_heightData[y * width + x] = (data[y * width + x] / 255.0f) * m_maxHeight;

    stbi_image_free(data);
    return true;
}

// --- Get Height At World Coordinate ---
float Terrain::GetHeightAt(int x, int y) const {
    int width = m_worldSize;
    if (x < 0 || y < 0 || x >= width || y >= width)
        return 0.0f;
    return m_heightData[y * width + x];
}

// --- Generate Chunks ---
void Terrain::GenerateChunks() {
    int chunksPerAxis = m_worldSize / m_chunkSize;
    m_chunks.reserve(chunksPerAxis * chunksPerAxis);

    for (int y = 0; y < chunksPerAxis; ++y)
        for (int x = 0; x < chunksPerAxis; ++x)
            m_chunks.emplace_back(std::make_unique<TerrainChunk>(this, x, y, m_chunkSize));
}

// --- Update All Chunks ---
void Terrain::Update() {
    Frustum frustum = m_camera.GetFrustum();
    Vec3 camPos = m_camera.GetPosition();

    int culledChunks = 0;
    for (auto& chunk : m_chunks) {
        if (chunk->IsChunkVisible(frustum)) {
            chunk->Update(camPos);
        } else {
            culledChunks++;
        }
    }
}

// --- Render All Visible Chunks ---
void Terrain::Render() {
    Frustum frustum = m_camera.GetFrustum();

    int culledChunks = 0;
    for (auto& chunk : m_chunks) {
        if (chunk->IsChunkVisible(frustum)) {
            chunk->Render();
        } else {
            culledChunks++;
        }
    }
}

// --- Constructor ---
TerrainChunk::TerrainChunk(Terrain* parent, int chunkX, int chunkY, uint32_t chunkSize) : m_parent(parent), m_chunkX(chunkX), m_chunkY(chunkY), m_chunkSize(chunkSize), 
m_position(chunkX * chunkSize, 0.0f, chunkY * chunkSize), m_shader(Shader("terrain.vert", "terrain.frag")), m_texture(Texture("ground_diffuse.png")) {
    GenerateMesh();
    CalculateBounds();
}

// --- Destructor ---
TerrainChunk::~TerrainChunk() {
    // Free GPU buffers
    glDeleteBuffers(1, &m_vbo);
    glDeleteVertexArrays(1, &m_vao);
}

// --- Mesh Generation ---
void TerrainChunk::GenerateMesh() {
    vertices.clear();
    indices.clear();

    this->resolution = m_chunkSize;
    vertexCount = (resolution + 1) * (resolution + 1);
    float scale = 1.0f;

    for (int z = 0; z <= resolution; ++z) {
        for (int x = 0; x <= resolution; ++x) {
            int worldX = x + m_chunkX * resolution;
            int worldZ = z + m_chunkY * resolution;
            float height = m_parent->GetHeightAt(worldX, worldZ);

            vertices.push_back(worldX * scale); // X
            vertices.push_back(height);         // Y
            vertices.push_back(worldZ * scale); // Z
        }
    }

    for (int z = 0; z < resolution; ++z) {
        for (int x = 0; x < resolution; ++x) {
            int topLeft = z * (resolution + 1) + x;
            int topRight = topLeft + 1;
            int bottomLeft = (z + 1) * (resolution + 1) + x;
            int bottomRight = bottomLeft + 1;

            indices.push_back(topLeft);
            indices.push_back(bottomLeft);
            indices.push_back(topRight);

            indices.push_back(topRight);
            indices.push_back(bottomLeft);
            indices.push_back(bottomRight);
        }
    }

    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glBindVertexArray(m_vao);
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glBindVertexArray(0);
}

// --- AABB Calculation ---
void TerrainChunk::CalculateBounds() {
    float minX = m_position.x;
    float maxX = m_position.x + m_chunkSize;
    float minZ = m_position.z;
    float maxZ = m_position.z + m_chunkSize;

    float minY = 0.0f;
    float maxY = m_parent->m_maxHeight;

    m_bounds = { Vec3(minX, minY, minZ), Vec3(maxX, maxY, maxZ) };
}

// --- Update (LOD can be added later) ---
void TerrainChunk::Update(const Vec3& cameraPosition) {
    float dist = (cameraPosition - m_position).Length();
    m_lodFactor = dist < 100.0f ? 1.0f : (dist < 300.0f ? 0.5f : 0.25f);
}

// --- Frustum Visibility ---
bool TerrainChunk::IsChunkVisible(const Frustum& frustum) const {
    return frustum.Overlaps(m_bounds);
}

// --- Rendering ---
void TerrainChunk::Render() {
    m_shader.Bind();
    m_texture.Bind();
    m_shader.SetUniform("model", Mat4::Translate(m_position));


    glBindVertexArray(m_vao);
    glPatchParameteri(GL_PATCH_VERTICES, 4);
    glDrawArrays(GL_PATCHES, 0, vertexCount); // vertexCount = resolution * resolution
    glBindVertexArray(0);





    glDrawElements(GL_PATCHES, indices.size(), GL_UNSIGNED_INT, 0);
}

#ifndef MOSS_MESH_GL_H
#define MOSS_MESH_GL_H

#include <Moss/Core/Core.h>
#include <Moss/Renderer/GL/Renderer_GL.h>
#include <Moss/Renderer/GL/glad.h>
#include <vector>
#include <memory>

// Mesh uses AOS
struct Vertex {
    Vec3 position;
    Vec3 normal;
    Vec2 texCoords; // Fixed: use correct field name `texCoords`, not `uv`
};

// Mesh uses AOS
class [[nodiscard]] MOSS_API Mesh {
public:
    Mesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices)
        : vertices(vertices), indices(indices), useInstancing(false) {}

    ~Mesh();

    void instance() { useInstancing = true; }
    void setupMesh();
    void setInstanceTransforms(const std::vector<Mat44>& transforms); // For instancing
    void draw();

    GLuint getVAO() const { return VAO; }
    size_t getIndexCount() const { return indices.size(); }

    float distanceThreshold;

private:
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;

    GLuint VAO = 0, VBO = 0, EBO = 0;
    GLuint instanceVBO = 0;
    GLsizei instanceCount = 0;
    bool useInstancing = false;
};

class [[nodiscard]] MOSS_API MeshInstance3 {
public:
    MeshInstance3(std::shared_ptr<Mesh> mesh)
        : mesh(std::move(mesh)) {}

    void setTransform(const Mat44& transform) { this->transform = transform; }
    void draw() { if (mesh) mesh->draw(); }

    const Mat44& getTransform() const { return transform; }

private:
    std::shared_ptr<Mesh> mesh;
    Mat44 transform = Mat44::sIdentity();
};

class [[nodiscard]] MOSS_API MultiMeshInstance3 {
public:
    MultiMeshInstance3(std::shared_ptr<Mesh> mesh)
        : mesh(std::move(mesh)) {}

    void setTransforms(const std::vector<Mat44>& transforms) {
        this->transforms = transforms;
        if (mesh) mesh->setInstanceTransforms(transforms);
    }

    void addTransform(const Mat44& transform) {
        transforms.push_back(transform);
        if (mesh) mesh->setInstanceTransforms(transforms);
    }

    void draw() { if (mesh) mesh->draw(); }

    const std::vector<Mat44>& getTransforms() const { return transforms; }

private:
    std::shared_ptr<Mesh> mesh;
    std::vector<Mat44> transforms;
};

struct MeshPart {
    std::vector<Vertex> vertices;
    std::vector<unsigned int> indices;
};

class MeshBatch3 {
public:
    void addMesh(const MeshPart& mesh) {
        unsigned int baseIndex = static_cast<unsigned int>(batchedVertices.size());
        batchedVertices.insert(batchedVertices.end(), mesh.vertices.begin(), mesh.vertices.end());
        for (unsigned int idx : mesh.indices)
            batchedIndices.push_back(baseIndex + idx);
    }

    void build() {
        glGenVertexArrays(1, &vao);
        glGenBuffers(1, &vbo);
        glGenBuffers(1, &ebo);

        glBindVertexArray(vao);

        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, batchedVertices.size() * sizeof(Vertex), batchedVertices.data(), GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, batchedIndices.size() * sizeof(unsigned int), batchedIndices.data(), GL_STATIC_DRAW);

        // Position
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, position));
        glEnableVertexAttribArray(0);

        // Normal
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, normal));
        glEnableVertexAttribArray(1);

        // TexCoords
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, texCoords));
        glEnableVertexAttribArray(2);

        glBindVertexArray(0);
    }

    void render() {
        glBindVertexArray(vao);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(batchedIndices.size()), GL_UNSIGNED_INT, 0);
    }

private:
    std::vector<Vertex> batchedVertices;
    std::vector<unsigned int> batchedIndices;
    GLuint vao = 0, vbo = 0, ebo = 0;
};

#endif // MOSS_MESH_GL_H

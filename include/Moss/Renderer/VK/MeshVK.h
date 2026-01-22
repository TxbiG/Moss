#pragma once


// Optomize your Meshes (OpenGL) https://youtu.be/9HO1dl0zcxg?si=JbOUCshCPg6GpJ8S
// Batching (OpenGL) https://youtu.be/SUMUtevioe4?si=cNQFcEzrD5zi0wxC
// Instancing https://youtu.be/TOPvFvL_GRY?si=3cqY1wgru2aoXm4-
// Tessellation https://youtu.be/4MvX5VeQWKA?si=jxluQ7TUEVP5_lUI
#include <Moss/Core/Core.h>
#include <vector>

// Mesh uses AOS
class [[nodiscard]] MOSS_API Mesh {
public:
    Mesh(VkDevice device, VkPhysicalDevice physicalDevice, VkCommandPool cmdPool, VkQueue queue,
           const std::vector<Vertex>& vertices, const std::vector<uint32_t>& indices,
           const Texture* texture = nullptr);
    Mesh(VkDevice device, VkPhysicalDevice physicalDevice, VkCommandPool cmdPool, VkQueue queue, const char* path, const Texture* texture = nullptr);
    ~Mesh();

    void Draw(VkCommandBuffer cmd, const Mat44& viewProj, const Mat44& modelMatrix);

    struct Vertex {
        Float3 position;
        Float2 texCoord;
        Float3 normal;

        static VkVertexInputBindingDescription GetBindingDescription();
        static std::array<VkVertexInputAttributeDescription, 3> GetAttributeDescriptions();
    };

private:
    void CreateVertexBuffer(const std::vector<Vertex>& vertices);
    void CreateIndexBuffer(const std::vector<uint32_t>& indices);
    void CreateUniforms();
    void UpdateUniforms(const Mat44& viewProj, const Mat44& modelMatrix);

    VkDevice mDevice;
    VkPhysicalDevice mPhysicalDevice;
    VkCommandPool mCommandPool;
    VkQueue mQueue;

    // GPU-resident buffers
    VkBuffer mVertexBuffer = VK_NULL_HANDLE;
    VkDeviceMemory mVertexBufferMemory = VK_NULL_HANDLE;

    VkBuffer mIndexBuffer = VK_NULL_HANDLE;
    VkDeviceMemory mIndexBufferMemory = VK_NULL_HANDLE;

    VkBuffer mUniformBuffer = VK_NULL_HANDLE;
    VkDeviceMemory mUniformBufferMemory = VK_NULL_HANDLE;

    VkDescriptorSet mDescriptorSet = VK_NULL_HANDLE;
    VkDescriptorSetLayout mDescriptorSetLayout = VK_NULL_HANDLE;

    Texture* mTexture = nullptr;

    uint32_t mIndexCount = 0;

    VkPipeline mPipeline = VK_NULL_HANDLE;
    VkPipelineLayout mPipelineLayout = VK_NULL_HANDLE;

    struct alignas(16) MeshUniforms {
        Mat44 model;
        Mat44 viewProj;
    };
};


class [[nodiscard]] MOSS_API Mesh {
public:
    Mesh(VulkanContext* context, const std::vector<Vertex>& vertices, const std::vector<uint32_t>& indices);

    ~Mesh();

    void draw(VkCommandBuffer commandBuffer) const;

    size_t getIndexCount() const { return indexCount; }

private:
    void createVertexBuffer();
    void createIndexBuffer();

    VulkanContext* context;

    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;

    VkBuffer vertexBuffer = VK_NULL_HANDLE;
    VkDeviceMemory vertexMemory = VK_NULL_HANDLE;

    VkBuffer indexBuffer = VK_NULL_HANDLE;
    VkDeviceMemory indexMemory = VK_NULL_HANDLE;

    size_t indexCount = 0;
};
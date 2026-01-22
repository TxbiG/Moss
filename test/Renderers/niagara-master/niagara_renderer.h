#pragma once



#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include <volk.h>

#include <vector>

#define VK_CHECK(call) \
	do \
	{ \
		VkResult result_ = call; \
		assert(result_ == VK_SUCCESS); \
	} while (0)

#define VK_CHECK_FORCE(call) \
	do \
	{ \
		VkResult result_ = call; \
		if (result_ != VK_SUCCESS) \
		{ \
			fprintf(stderr, "%s:%d: %s failed with error %d\n", __FILE__, __LINE__, #call, result_); \
			abort(); \
		} \
	} while (0)

#define VK_CHECK_SWAPCHAIN(call) \
	do \
	{ \
		VkResult result_ = call; \
		assert(result_ == VK_SUCCESS || result_ == VK_SUBOPTIMAL_KHR || result_ == VK_ERROR_OUT_OF_DATE_KHR); \
	} while (0)

#define VK_CHECK_QUERY(call) \
	do \
	{ \
		VkResult result_ = call; \
		assert(result_ == VK_SUCCESS || result_ == VK_NOT_READY); \
	} while (0)

template <typename T, size_t Size>
char (*countof_helper(T (&_Array)[Size]))[Size];

#define COUNTOF(array) (sizeof(*countof_helper(array)) + 0)


struct Image;
struct Buffer;

bool loadImage(Image& image, VkDevice device, VkCommandPool commandPool, VkCommandBuffer commandBuffer, VkQueue queue, const VkPhysicalDeviceMemoryProperties& memoryProperties, const Buffer& scratch, const char* path);

struct Swapchain
{
	VkSwapchainKHR swapchain;

	std::vector<VkImage> images;

	uint32_t width, height;
	uint32_t imageCount;
};

typedef struct GLFWwindow GLFWwindow;

const char** getSwapchainExtensions(uint32_t* count);

VkSurfaceKHR createSurface(VkInstance instance, GLFWwindow* window);
VkFormat getSwapchainFormat(VkPhysicalDevice physicalDevice, VkSurfaceKHR surface);
void createSwapchain(Swapchain& result, VkPhysicalDevice physicalDevice, VkDevice device, VkSurfaceKHR surface, uint32_t familyIndex, GLFWwindow* window, VkFormat format, VkSwapchainKHR oldSwapchain = 0);
void destroySwapchain(VkDevice device, const Swapchain& swapchain);

enum SwapchainStatus
{
	Swapchain_Ready,
	Swapchain_Resized,
	Swapchain_NotReady,
};

SwapchainStatus updateSwapchain(Swapchain& result, VkPhysicalDevice physicalDevice, VkDevice device, VkSurfaceKHR surface, uint32_t familyIndex, GLFWwindow* window, VkFormat format);








#include <string>

struct Shader
{
	std::string name;

	std::vector<char> spirv;
	VkShaderStageFlagBits stage;

	VkDescriptorType resourceTypes[32];
	uint32_t resourceMask;

	uint32_t localSizeX;
	uint32_t localSizeY;
	uint32_t localSizeZ;

	bool usesPushConstants;
	bool usesDescriptorArray;
};

struct ShaderSet
{
	std::vector<Shader> shaders;

	const Shader& operator[](const char* name) const;
};

struct Program
{
	VkPipelineBindPoint bindPoint;
	VkPipelineLayout layout;
	VkDescriptorSetLayout setLayout;
	VkDescriptorUpdateTemplate updateTemplate;

	VkShaderStageFlags pushConstantStages;
	uint32_t pushConstantSize;
	uint32_t pushDescriptorCount;

	uint32_t localSizeX;
	uint32_t localSizeY;
	uint32_t localSizeZ;

	const Shader* shaders[8];
	size_t shaderCount;
};

bool loadShader(Shader& shader, const char* path);
bool loadShader(Shader& shader, const char* base, const char* path);
bool loadShaders(ShaderSet& shaders, const char* base, const char* path);

using Shaders = std::initializer_list<const Shader*>;
using Constants = std::initializer_list<int>;

VkPipeline createGraphicsPipeline(VkDevice device, VkPipelineCache pipelineCache, const VkPipelineRenderingCreateInfo& renderingInfo, const Program& program, Constants constants = {});
VkPipeline createComputePipeline(VkDevice device, VkPipelineCache pipelineCache, const Program& program, Constants constants = {});

Program createProgram(VkDevice device, VkPipelineBindPoint bindPoint, Shaders shaders, size_t pushConstantSize, VkDescriptorSetLayout arrayLayout = nullptr);
void destroyProgram(VkDevice device, const Program& program);

VkDescriptorSetLayout createDescriptorArrayLayout(VkDevice device);
std::pair<VkDescriptorPool, VkDescriptorSet> createDescriptorArray(VkDevice device, VkDescriptorSetLayout layout, uint32_t descriptorCount);

inline uint32_t getGroupCount(uint32_t threadCount, uint32_t localSize)
{
	return (threadCount + localSize - 1) / localSize;
}

struct DescriptorInfo
{
	union
	{
		VkDescriptorImageInfo image;
		VkDescriptorBufferInfo buffer;
		VkAccelerationStructureKHR accelerationStructure;
	};

	DescriptorInfo()
	{
	}

	DescriptorInfo(VkAccelerationStructureKHR structure)
	{
		accelerationStructure = structure;
	}

	DescriptorInfo(VkImageView imageView, VkImageLayout imageLayout = VK_IMAGE_LAYOUT_GENERAL)
	{
		image.sampler = VK_NULL_HANDLE;
		image.imageView = imageView;
		image.imageLayout = imageLayout;
	}

	DescriptorInfo(VkSampler sampler)
	{
		image.sampler = sampler;
		image.imageView = VK_NULL_HANDLE;
		image.imageLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	}

	DescriptorInfo(VkSampler sampler, VkImageView imageView, VkImageLayout imageLayout = VK_IMAGE_LAYOUT_GENERAL)
	{
		image.sampler = sampler;
		image.imageView = imageView;
		image.imageLayout = imageLayout;
	}

	DescriptorInfo(VkBuffer buffer_, VkDeviceSize offset, VkDeviceSize range)
	{
		buffer.buffer = buffer_;
		buffer.offset = offset;
		buffer.range = range;
	}

	DescriptorInfo(VkBuffer buffer_)
	{
		buffer.buffer = buffer_;
		buffer.offset = 0;
		buffer.range = VK_WHOLE_SIZE;
	}
};




struct Buffer;

struct Mesh;
struct MeshDraw;
struct Meshlet;

void buildBLAS(VkDevice device, const std::vector<Mesh>& meshes, const Buffer& vb, const Buffer& ib, std::vector<VkAccelerationStructureKHR>& blas, std::vector<VkDeviceSize>& compactedSizes, Buffer& blasBuffer, VkCommandPool commandPool, VkCommandBuffer commandBuffer, VkQueue queue, const VkPhysicalDeviceMemoryProperties& memoryProperties);
void compactBLAS(VkDevice device, std::vector<VkAccelerationStructureKHR>& blas, const std::vector<VkDeviceSize>& compactedSizes, Buffer& blasBuffer, VkCommandPool commandPool, VkCommandBuffer commandBuffer, VkQueue queue, const VkPhysicalDeviceMemoryProperties& memoryProperties);

void buildCBLAS(VkDevice device, const std::vector<Mesh>& meshes, const std::vector<Meshlet>& meshlets, const Buffer& vxb, const Buffer& mdb, std::vector<VkAccelerationStructureKHR>& blas, Buffer& blasBuffer, VkCommandPool commandPool, VkCommandBuffer commandBuffer, VkQueue queue, const VkPhysicalDeviceMemoryProperties& memoryProperties);

void fillInstanceRT(VkAccelerationStructureInstanceKHR& instance, const MeshDraw& draw, uint32_t instanceIndex, VkDeviceAddress blas);

VkAccelerationStructureKHR createTLAS(VkDevice device, Buffer& tlasBuffer, Buffer& scratchBuffer, const Buffer& instanceBuffer, uint32_t primitiveCount, const VkPhysicalDeviceMemoryProperties& memoryProperties);

void buildTLAS(VkDevice device, VkCommandBuffer commandBuffer, VkAccelerationStructureKHR tlas, const Buffer& tlasBuffer, const Buffer& scratchBuffer, const Buffer& instanceBuffer, uint32_t primitiveCount, VkBuildAccelerationStructureModeKHR mode);



#include "math.h"

#include <stdint.h>

#include <string>
#include <vector>

struct alignas(16) Meshlet
{
	vec3 center;
	float radius;
	int8_t cone_axis[3];
	int8_t cone_cutoff;

	uint32_t dataOffset; // dataOffset..dataOffset+vertexCount-1 stores vertex indices, we store indices packed in 4b units after that
	uint32_t baseVertex;
	uint8_t vertexCount;
	uint8_t triangleCount;
	uint8_t shortRefs;
	uint8_t padding;
};

struct alignas(16) Material
{
	int albedoTexture;
	int normalTexture;
	int specularTexture;
	int emissiveTexture;

	vec4 diffuseFactor;
	vec4 specularFactor;
	vec3 emissiveFactor;
};

struct alignas(16) MeshDraw
{
	vec3 position;
	float scale;
	quat orientation;

	uint32_t meshIndex;
	uint32_t meshletVisibilityOffset;
	uint32_t postPass;
	uint32_t materialIndex;
};

struct Vertex
{
	uint16_t vx, vy, vz;
	uint16_t tp; // packed tangent: 8-8 octahedral
	uint32_t np; // packed normal: 10-10-10-2 vector + bitangent sign
	uint16_t tu, tv;
};

struct MeshLod
{
	uint32_t indexOffset;
	uint32_t indexCount;
	uint32_t meshletOffset;
	uint32_t meshletCount;
	float error;
};

struct alignas(16) Mesh
{
	vec3 center;
	float radius;

	uint32_t vertexOffset;
	uint32_t vertexCount;

	uint32_t lodCount;
	MeshLod lods[8];
};

struct Geometry
{
	// TODO: remove these vectors - they are just scratch copies that waste space
	std::vector<Vertex> vertices;
	std::vector<uint32_t> indices;
	std::vector<Meshlet> meshlets;
	std::vector<uint32_t> meshletdata;
	std::vector<uint16_t> meshletvtx0; // 4 position components per vertex referenced by meshlets in lod 0, packed tightly
	std::vector<Mesh> meshes;
};

struct Camera
{
	vec3 position;
	quat orientation;
	float fovY;
	float znear;
};

struct Keyframe
{
	vec3 translation;
	float scale;
	quat rotation;
};

struct Animation
{
	uint32_t drawIndex;

	float startTime;
	float period;
	std::vector<Keyframe> keyframes;
};

bool loadMesh(Geometry& geometry, const char* path, bool buildMeshlets, bool fast = false, bool clrt = false);
bool loadScene(Geometry& geometry, std::vector<Material>& materials, std::vector<MeshDraw>& draws, std::vector<std::string>& texturePaths, std::vector<Animation>& animations, Camera& camera, vec3& sunDirection, const char* path, bool buildMeshlets, bool fast = false, bool clrt = false);




struct Buffer
{
	VkBuffer buffer;
	VkDeviceMemory memory;
	void* data;
	size_t size;
};

struct Image
{
	VkImage image;
	VkImageView imageView;
	VkDeviceMemory memory;
};

VkImageMemoryBarrier2 imageBarrier(VkImage image, VkPipelineStageFlags2 srcStageMask, VkAccessFlags2 srcAccessMask, VkImageLayout oldLayout, VkPipelineStageFlags2 dstStageMask, VkAccessFlags2 dstAccessMask, VkImageLayout newLayout, VkImageAspectFlags aspectMask = VK_IMAGE_ASPECT_COLOR_BIT, uint32_t baseMipLevel = 0, uint32_t levelCount = VK_REMAINING_MIP_LEVELS);
VkBufferMemoryBarrier2 bufferBarrier(VkBuffer buffer, VkPipelineStageFlags2 srcStageMask, VkAccessFlags2 srcAccessMask, VkPipelineStageFlags2 dstStageMask, VkAccessFlags2 dstAccessMask);

void pipelineBarrier(VkCommandBuffer commandBuffer, VkDependencyFlags dependencyFlags, size_t bufferBarrierCount, const VkBufferMemoryBarrier2* bufferBarriers, size_t imageBarrierCount, const VkImageMemoryBarrier2* imageBarriers);

void invalidateBarrier(VkCommandBuffer commandBuffer, VkPipelineStageFlags2 stageMask, std::initializer_list<VkImage> colorImages, std::initializer_list<VkImage> depthImages = {});

void stageBarrier(VkCommandBuffer commandBuffer, VkPipelineStageFlags2 srcStageMask, VkAccessFlags2 srcAccessMask, VkPipelineStageFlags2 dstStageMask, VkAccessFlags2 dstAccessMask);
void stageBarrier(VkCommandBuffer commandBuffer, VkPipelineStageFlags2 srcStageMask, VkPipelineStageFlags2 dstStageMask);
void stageBarrier(VkCommandBuffer commandBuffer, VkPipelineStageFlags2 stageMask);

void createBuffer(Buffer& result, VkDevice device, const VkPhysicalDeviceMemoryProperties& memoryProperties, size_t size, VkBufferUsageFlags usage, VkMemoryPropertyFlags memoryFlags);
void uploadBuffer(VkDevice device, VkCommandPool commandPool, VkCommandBuffer commandBuffer, VkQueue queue, const Buffer& buffer, const Buffer& scratch, const void* data, size_t size);
void destroyBuffer(const Buffer& buffer, VkDevice device);

VkDeviceAddress getBufferAddress(const Buffer& buffer, VkDevice device);

VkImageView createImageView(VkDevice device, VkImage image, VkFormat format, uint32_t mipLevel, uint32_t levelCount);

void createImage(Image& result, VkDevice device, const VkPhysicalDeviceMemoryProperties& memoryProperties, uint32_t width, uint32_t height, uint32_t mipLevels, VkFormat format, VkImageUsageFlags usage);
void destroyImage(const Image& image, VkDevice device);

uint32_t getImageMipLevels(uint32_t width, uint32_t height);

VkSampler createSampler(VkDevice device, VkFilter filter, VkSamplerMipmapMode mipmapMode, VkSamplerAddressMode addressMode, VkSamplerReductionModeEXT reductionMode = VK_SAMPLER_REDUCTION_MODE_WEIGHTED_AVERAGE_EXT);


bool isInstanceExtensionSupported(const char* name);

VkInstance createInstance();
VkDebugReportCallbackEXT registerDebugCallback(VkInstance instance);

uint32_t getGraphicsFamilyIndex(VkPhysicalDevice physicalDevice);
VkPhysicalDevice pickPhysicalDevice(VkPhysicalDevice* physicalDevices, uint32_t physicalDeviceCount);

VkDevice createDevice(VkInstance instance, VkPhysicalDevice physicalDevice, uint32_t familyIndex, bool meshShadingSupported, bool raytracingSupported, bool clusterrtSupported);

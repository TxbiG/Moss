#ifndef MOSS_RENDERER_INTERNAL_H
#define MOSS_RENDERER_INTERNAL_H

#include <vulkan/vulkan.h>

#include <Moss/Moss_Renderer.h>

inline static const uint32_t cFrameCount = 2;


struct PhysicalDevice {
    VkPhysicalDevice physicalDevice;                // Physical device representation
    VkPhysicalDeviceProperties properties;          // Properties of the physical device including limits that the application can check against
	VkPhysicalDeviceMemoryProperties memoryProperties;              // Memory types and heaps of the physical device
    std::vector<VkQueueFamilyProperties> queueFamilyProperties;     // Queue family properties of the physical device
    std::vector<VkBool32> supports;
    std::vector<VkSurfaceFormatKHR> surfaceFormats;
    std::vector<VkPresentModeKHR> presentModes;

    VkSurfaceCapabilitiesKHR surfaceCapabilities;
	VkPhysicalDeviceFeatures PhysDevicefeatures;
	VkFormat m_depthFormat;
	struct {
		int Variant = 0;
		int Major = 0;
		int Minor = 0;
		int Patch = 0;
	} m_apiVersion;
	std::vector<VkExtensionProperties> m_extensions;
};

struct Moss_Renderer {
    Moss_Window& m_window;

    // Vulkan
    VkInstance m_instance   = VK_NULL_HANDLE;          // An object that contains all the information the implementation needs to work.
    VkSurfaceKHR m_surface  = VK_NULL_HANDLE;          // An instance extension which abstract native platform surface or window objects for use with Vulkan.
    VkDevice m_device       = VK_NULL_HANDLE;          // The “logical” GPU context that you actually execute things on.
    PhysicalDevice m_physicalDevice;                   // Holds GPU Physical device data.

	uint32							m_GraphicsQueueIndex = 0;
	uint32							m_PresentQueueIndex = 0;
	VkQueue							m_GraphicsQueue = VK_NULL_HANDLE;
	VkQueue							m_PresentQueue = VK_NULL_HANDLE;
	VkSwapchainKHR					m_SwapChain = VK_NULL_HANDLE;
	bool							m_SubOptimalSwapChain = false;
	std::vector<VkImage>			m_SwapChainImages;
	VkFormat						m_SwapChainImageFormat;
	VkExtent2D						m_SwapChainExtent;
	std::vector<VkImageView>				m_SwapChainImageViews;
	VkImage							m_DepthImage = VK_NULL_HANDLE;
	VkDeviceMemory					m_DepthImageMemory = VK_NULL_HANDLE;
	VkImageView						m_DepthImageView = VK_NULL_HANDLE;
	VkDescriptorSetLayout			m_DescriptorSetLayoutUBO = VK_NULL_HANDLE;
	VkDescriptorSetLayout			m_DescriptorSetLayoutTexture = VK_NULL_HANDLE;
	VkDescriptorPool				m_DescriptorPool = VK_NULL_HANDLE;
	VkDescriptorSet					m_DescriptorSets[cFrameCount];
	VkDescriptorSet					m_DescriptorSetsOrtho[cFrameCount];
	VkSampler						m_TextureSamplerShadow = VK_NULL_HANDLE;	
	VkSampler						m_TextureSamplerRepeat = VK_NULL_HANDLE;	
	VkRenderPass					m_RenderPassShadow = VK_NULL_HANDLE;
	VkRenderPass					m_RenderPass = VK_NULL_HANDLE;
	VkPipelineLayout				m_PipelineLayout = VK_NULL_HANDLE;
	VkFramebuffer					m_ShadowFrameBuffer = VK_NULL_HANDLE;
	std::vector<VkFramebuffer>			m_SwapChainFramebuffers;
	uint32							m_ImageIndex = 0;
	uint32							m_CurrentFrame = 0;
	VkCommandPool					m_CommandPool = VK_NULL_HANDLE;
	VkCommandBuffer					m_CommandBuffers[cFrameCount];
	std::vector<VkSemaphore>				m_AvailableSemaphores;
	std::vector<VkSemaphore>				m_ImageAvailableSemaphores;
	std::vector<VkSemaphore>				m_RenderFinishedSemaphores;
	VkFence							m_InFlightFences[cFrameCount];
	Ref<TextureVK>					m_ShadowMap;
	std::unique_ptr<ConstantBufferVK>	m_VertexShaderConstantBufferProjection[cFrameCount];
	std::unique_ptr<ConstantBufferVK>	m_VertexShaderConstantBufferOrtho[cFrameCount];
	std::unique_ptr<ConstantBufferVK>	m_PixelShaderConstantBuffer[cFrameCount];

	struct Key {
		bool operator == (const Key &inRHS) const { return m_Size == inRHS.m_Size && m_Usage == inRHS.m_Usage && m_Properties == inRHS.m_Properties; }

		VkDeviceSize				m_Size;
		VkBufferUsageFlags			m_Usage;
		VkMemoryPropertyFlags		m_Properties;
	};

	MOSS_MAKE_HASH_STRUCT(Key, KeyHasher, t.m_Size, t.m_Usage, t.m_Properties)

	// We try to recycle buffers from frame to frame
	using BufferCache = TMap<Key, TArray<BufferVK>, KeyHasher>;

	BufferCache						m_FreedBuffers[cFrameCount];
	BufferCache						m_BufferCache;

	// Smaller allocations (from cMinAllocSize to cMaxAllocSize) will be done in blocks of cBlockSize bytes.
	// We do this because there is a limit to the number of allocations that we can make in Vulkan.
	static constexpr VkDeviceSize	cMinAllocSize = 512;
	static constexpr VkDeviceSize	cMaxAllocSize = 65536;
	static constexpr VkDeviceSize	cBlockSize = 524288;

	//MOSS_MAKE_HASH_STRUCT(Key, MemKeyHasher, t.mUsage, t.mProperties, t.mSize)

	struct Memory {
		VkDeviceMemory				mMemory;
		VkDeviceSize				mOffset;
	};

	using MemoryCache = TMap<Key, TArray<Memory>, KeyHasher>;

	MemoryCache						m_MemoryCache;
	uint32							m_NumAllocations = 0;
	uint32							m_MaxNumAllocations = 0;
	VkDeviceSize					m_TotalAllocated = 0;
	VkDeviceSize					m_MaxTotalAllocated = 0;

	// FSR1
	VkImage         fsrIntermediateImage = VK_NULL_HANDLE;
	VkDeviceMemory  fsrIntermediateMemory = VK_NULL_HANDLE;
	VkImageView     fsrIntermediateView = VK_NULL_HANDLE;
	VkFramebuffer   fsrIntermediateFBO = VK_NULL_HANDLE;
	VkPipeline      fsrEASUPipeline = VK_NULL_HANDLE;
	VkPipelineLayout fsrEASUPipelineLayout = VK_NULL_HANDLE;
	VkPipeline      fsrRCASPipeline = VK_NULL_HANDLE;
	VkPipelineLayout fsrRCASPipelineLayout = VK_NULL_HANDLE;
	VkDescriptorSetLayout fsrDescriptorSetLayout = VK_NULL_HANDLE;
	VkDescriptorSet       fsrDescriptorSet = VK_NULL_HANDLE;
	int internalWidth = 0;
	int internalHeight = 0;
	float fsrSharpness = 0.2f;

	VkPhysicalDeviceDynamicRenderingFeaturesKHR dynRendering;

#if defined(MOSS_RENDERER_MOBILE)
	bool lowPowerMode;
	bool thermalThrottling;
#endif

	// FSR2
#if defined(MOSS_RENDERER_FORWARD)
	bool enableFSR2 = false;
	VkImage         fsrHistoryImages[2] = { VK_NULL_HANDLE, VK_NULL_HANDLE };
	VkDeviceMemory  fsrHistoryMemory[2] = { VK_NULL_HANDLE, VK_NULL_HANDLE };
	VkImageView     fsrHistoryViews[2] = { VK_NULL_HANDLE, VK_NULL_HANDLE };
	int             fsrHistoryIndex = 0;

	VkQueue m_ComputeQueue;
	uint32  m_ComputeQueueIndex;


	VkImage         fsrMotionVectorsImage = VK_NULL_HANDLE;
	VkDeviceMemory  fsrMotionVectorsMemory = VK_NULL_HANDLE;
	VkImageView     fsrMotionVectorsView = VK_NULL_HANDLE;

	VkImage         fsrDepthImage = VK_NULL_HANDLE;
	VkDeviceMemory  fsrDepthMemory = VK_NULL_HANDLE;
	VkImageView     fsrDepthView = VK_NULL_HANDLE;

	VkPipeline      fsr2Pipeline = VK_NULL_HANDLE;
	VkPipelineLayout fsr2PipelineLayout = VK_NULL_HANDLE;
	VkDescriptorSet fsr2DescriptorSet = VK_NULL_HANDLE;

	Mat4x4 previousViewProj = Mat4x4(1.0f);
	Mat4x4 currentViewProj = Mat4x4(1.0f);
#endif

#ifdef MOSS_DEBUG
    VkDebugUtilsMessengerEXT m_debugMessenger = VK_NULL_HANDLE;
#endif // MOSS_DEBUG

	bool IsDeviceSuitable(VkPhysicalDevice device);

};

struct Moss_GPUDevice {
    VkInstance instance;
    VkPhysicalDevice physicalDevice;
    VkDevice device;

    VkPhysicalDeviceProperties properties;
    VkPhysicalDeviceMemoryProperties memoryProperties;
    VkPhysicalDeviceFeatures features;

    uint32_t graphicsQueueIndex;
    uint32_t computeQueueIndex;
    uint32_t transferQueueIndex;

    VkQueue graphicsQueue;
    VkQueue computeQueue;
    VkQueue transferQueue;

    VkCommandPool commandPool;

    uint32_t framesInFlight;
    uint32_t frameIndex;
};

typedef struct Moss_Shader Moss_Shader;

// Add blend factors, stencil state, dynamic states
struct Moss_PipelineState {
    Moss_GPUDevice* device;

    VkPipeline pipeline;
    VkPipelineLayout layout;

    VkRenderPass compatibleRenderPass;

    bool depthTest;
    bool depthWrite;
    bool blending;
    bool cullFace;

    VkCullModeFlags cullMode;
    VkFrontFace frontFace;
};

#define MOSS_MAX_COLOR_ATTACHMENTS 8

struct Moss_Framebuffer {
    Moss_GPUDevice* device;

    VkFramebuffer framebuffer;
    VkRenderPass renderPass;

    Moss_TextureView* colorAttachments[MOSS_MAX_COLOR_ATTACHMENTS];
    uint32_t colorCount;

    Moss_TextureView* depthAttachment;

    uint32_t width, height;
};

#define MOSS_MAX_BINDINGS 16

struct Moss_ResourceSet {
    Moss_GPUDevice* device;

    VkDescriptorSetLayout layout;
    VkDescriptorSet descriptorSet;

    VkDescriptorPool pool;

    uint32_t bindingCount;
};

struct Moss_TextureView {
    Moss_GPUDevice* device;

    VkImage image;
    VkDeviceMemory memory;
    VkImageView view;

    uint32_t width;
    uint32_t height;
    uint32_t mipLevels;

    VkFormat format;
    VkImageLayout currentLayout;
};

struct Moss_GPUSampler {
    Moss_GPUDevice* device;

    VkSampler sampler;

    VkFilter minFilter;
    VkFilter magFilter;

    VkSamplerAddressMode addressU;
    VkSamplerAddressMode addressV;
    VkSamplerAddressMode addressW;
};


struct Moss_RenderGraph {
    Moss_GPUDevice* device;

    Moss_RGPass** passes;
    uint32_t passCount;
};

struct Moss_RGPass {
    const char* name;

    Moss_Framebuffer* framebuffer;

    VkPipelineStageFlags srcStage;
    VkPipelineStageFlags dstStage;

    void (*execute)(Moss_GPUCommandBuffer*, void* userData);
};

struct Moss_RGTexture {
    Moss_TextureView* texture;
};

struct Moss_RGBuffer {
    Moss_GPUBuffer* buffer;
};

struct Moss_GPUCommandBuffer {
    Moss_GPUDevice* device;

    VkCommandBuffer commandBuffer;

    VkFence fence;
    VkSemaphore acquireSemaphore;
    VkSemaphore releaseSemaphore;

    bool recording;
};


#endif // MOSS_RENDERER_INTERNAL_H
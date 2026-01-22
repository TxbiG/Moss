#ifndef MOSS_RENDERER_VK_H
#define MOSS_RENDERER_VK_H


// https://youtu.be/x2FHHU50ktQ?si=1zYR1lmBug518mDj
// https://youtu.be/oETErEkFICE?si=hiUsH7HcI1ES35mm
// https://youtu.be/Uc8FEI5bg3w?si=bPz9i05X0zCh9Dz2

/* Vulkan Renderer
 * Supports FSR1 and FSR2
 *
 */


#include <vulkan/vulkan.h>
#include <Moss/Moss_Platform.h>
//#include <Moss/Renderer/VK/Moss_Impli_Vulkan.h>
#include <Moss/Core/Variants/Color.h>
#include <Moss/Core/Variants/Vector/Float2.h>
#include <Moss/Core/Variants/Vector/Vec2.h>
#include <Moss/Core/Variants/Vector/Vec3.h>
#include <Moss/Core/Variants/TMap.h>
#include <Moss/Core/Variants/TSet.h>

//#include <Moss/Renderer/Shader.h>
//#include <Moss/Renderer/Texture.h>
#include <Moss/Renderer/VK/ConstantBufferVK.h>
//#include <Renderer/VK/TextureVK.h>
//#include <vector>
inline static const uint		cFrameCount = 2;

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

	// FSR2
#if defined(MOSS_RENDERER_FORWARD)
	bool enableFSR2 = false;
	VkImage         fsrHistoryImages[2] = { VK_NULL_HANDLE, VK_NULL_HANDLE };
	VkDeviceMemory  fsrHistoryMemory[2] = { VK_NULL_HANDLE, VK_NULL_HANDLE };
	VkImageView     fsrHistoryViews[2] = { VK_NULL_HANDLE, VK_NULL_HANDLE };
	int             fsrHistoryIndex = 0;

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


/*              Renderer lifecycle          */

/*! @brief Initalise Renderer. @param Moss_Window window. @ingroup Renderer. */
MOSS_API Moss_Renderer* Moss_CreateRenderer(Moss_Window* window);
/*! @brief Begin Frame Clears the renderer ready to capture the next frame. @param X X. @ingroup Renderer. */
MOSS_API void Moss_RendererBeginFrame(Moss_Renderer* renderer);
/*! @brief X. @param X X. @ingroup Renderer. */
MOSS_API void Moss_RendererEndFrame(Moss_Renderer* renderer);
/*! @brief X. @param X X. @ingroup Renderer. */
MOSS_API void Moss_TerminateRenderer(Moss_Renderer* renderer);
/*! @brief X. @param X X. @ingroup Renderer. */

//Surface SubViewport(int x, int y, int width, int height);


#endif // MOSS_RENDERER_VK_H
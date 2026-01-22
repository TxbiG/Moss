#ifndef MOSS_TEST_VK
#define MOSS_TEXT_VK

#include <vulkan/vulkan.h>
#include <Moss/Renderer/VK/ConstantBufferVK.h>
#include <Moss/Renderer/VK/TextureVK.h>
#include <Moss/Core/Variants/TMap.h>

class RendererVK
{
public:
	/// Destructor
							~RendererVK() override;

	// See: Renderer
	void					Initialize(Moss_Window* window);
	bool					BeginFrame(const CameraState &inCamera, float inWorldScale);
	void					EndShadowPass() override;
	void					EndFrame() override;
	void					SetProjectionMode() override;
	void					SetOrthoMode() override;
	//Ref<Texture>			CreateTexture(const Surface *inSurface) override;
	Ref<VertexShader>		CreateVertexShader(const char *inName) override;
	Ref<PixelShader>		CreatePixelShader(const char *inName) override;
	unique_ptr<PipelineState> CreatePipelineState(const VertexShader *inVertexShader, const PipelineState::EInputDescription *inInputDescription, uint inInputDescriptionCount, const PixelShader *inPixelShader, PipelineState::EDrawPass inDrawPass, PipelineState::EFillMode inFillMode, PipelineState::ETopology inTopology, PipelineState::EDepthTest inDepthTest, PipelineState::EBlendMode inBlendMode, PipelineState::ECullMode inCullMode) override;
	RenderPrimitive *		CreateRenderPrimitive(PipelineState::ETopology inType) override;
	RenderInstances *		CreateRenderInstances() override;
	Texture *				GetShadowMap() const override									{ return mShadowMap.GetPtr(); }
	void					OnWindowResize() override;

	VkDevice						GetDevice() const												{ return mDevice; }
	VkDescriptorPool				GetDescriptorPool() const										{ return mDescriptorPool; }
	VkDescriptorSetLayout			GetDescriptorSetLayoutTexture() const							{ return mDescriptorSetLayoutTexture; }
	VkSampler						GetTextureSamplerRepeat() const									{ return mTextureSamplerRepeat; }
	VkSampler						GetTextureSamplerShadow() const									{ return mTextureSamplerShadow; }
	VkRenderPass					GetRenderPassShadow() const										{ return mRenderPassShadow; }
	VkRenderPass					GetRenderPass() const											{ return mRenderPass; }
	VkPipelineLayout				GetPipelineLayout() const										{ return mPipelineLayout; }
	VkCommandBuffer					GetCommandBuffer()												{ MOSS_ASSERT(mInFrame); return mCommandBuffers[mFrameIndex]; }
	VkCommandBuffer					StartTempCommandBuffer();
	void							EndTempCommandBuffer(VkCommandBuffer inCommandBuffer);
	void							AllocateMemory(VkDeviceSize inSize, uint32 inMemoryTypeBits, VkMemoryPropertyFlags inProperties, VkDeviceMemory &outMemory);
	void							FreeMemory(VkDeviceMemory inMemory, VkDeviceSize inSize);
	void							CreateBuffer(VkDeviceSize inSize, VkBufferUsageFlags inUsage, VkMemoryPropertyFlags inProperties, BufferVK &outBuffer);
	void							CopyBuffer(VkBuffer inSrc, VkBuffer inDst, VkDeviceSize inSize);
	void							CreateDeviceLocalBuffer(const void *inData, VkDeviceSize inSize, VkBufferUsageFlags inUsage, BufferVK &outBuffer);
	void							FreeBuffer(BufferVK &ioBuffer);
	unique_ptr<ConstantBufferVK>	CreateConstantBuffer(VkDeviceSize inBufferSize);
	void							CreateImage(uint32 inWidth, uint32 inHeight, VkFormat inFormat, VkImageTiling inTiling, VkImageUsageFlags inUsage, VkMemoryPropertyFlags inProperties, VkImage &outImage, VkDeviceMemory &outMemory);
	void							DestroyImage(VkImage inImage, VkDeviceMemory inMemory);
	VkImageView						CreateImageView(VkImage inImage, VkFormat inFormat, VkImageAspectFlags inAspectFlags);
	VkFormat						FindDepthFormat();

private:
	uint32							FindMemoryType(uint32 inTypeFilter, VkMemoryPropertyFlags inProperties);
	void							FreeBufferInternal(BufferVK &ioBuffer);
	VkSurfaceFormatKHR				SelectFormat(VkPhysicalDevice inDevice);
	void							CreateSwapChain(VkPhysicalDevice inDevice);
	void							DestroySwapChain();
	void							UpdateViewPortAndScissorRect(uint32 inWidth, uint32 inHeight);
	VkSemaphore						AllocateSemaphore();
	void							FreeSemaphore(VkSemaphore inSemaphore);

	VkInstance						mInstance = VK_NULL_HANDLE;
#ifdef MOSS_DEBUG
	VkDebugUtilsMessengerEXT		mDebugMessenger = VK_NULL_HANDLE;
#endif  // MOSS_DEBUG
	VkPhysicalDevice				mPhysicalDevice = VK_NULL_HANDLE;
	VkPhysicalDeviceMemoryProperties mMemoryProperties;
	VkDevice						mDevice = VK_NULL_HANDLE;
	uint32							mGraphicsQueueIndex = 0;
	uint32							mPresentQueueIndex = 0;
	VkQueue							mGraphicsQueue = VK_NULL_HANDLE;
	VkQueue							mPresentQueue = VK_NULL_HANDLE;
	VkSurfaceKHR					mSurface = VK_NULL_HANDLE;
	VkSwapchainKHR					mSwapChain = VK_NULL_HANDLE;
	bool							mSubOptimalSwapChain = false;
	TArray<VkImage>					mSwapChainImages;
	VkFormat						mSwapChainImageFormat;
	VkExtent2D						mSwapChainExtent;
	TArray<VkImageView>				mSwapChainImageViews;
	VkImage							mDepthImage = VK_NULL_HANDLE;
	VkDeviceMemory					mDepthImageMemory = VK_NULL_HANDLE;
	VkImageView						mDepthImageView = VK_NULL_HANDLE;
	VkDescriptorSetLayout			mDescriptorSetLayoutUBO = VK_NULL_HANDLE;
	VkDescriptorSetLayout			mDescriptorSetLayoutTexture = VK_NULL_HANDLE;
	VkDescriptorPool				mDescriptorPool = VK_NULL_HANDLE;
	VkDescriptorSet					mDescriptorSets[cFrameCount];
	VkDescriptorSet					mDescriptorSetsOrtho[cFrameCount];
	VkSampler						mTextureSamplerShadow = VK_NULL_HANDLE;	
	VkSampler						mTextureSamplerRepeat = VK_NULL_HANDLE;	
	VkRenderPass					mRenderPassShadow = VK_NULL_HANDLE;
	VkRenderPass					mRenderPass = VK_NULL_HANDLE;
	VkPipelineLayout				mPipelineLayout = VK_NULL_HANDLE;
	VkFramebuffer					mShadowFrameBuffer = VK_NULL_HANDLE;
	TArray<VkFramebuffer>			mSwapChainFramebuffers;
	uint32							mImageIndex = 0;
	VkCommandPool					mCommandPool = VK_NULL_HANDLE;
	VkCommandBuffer					mCommandBuffers[cFrameCount];
	TArray<VkSemaphore>				mAvailableSemaphores;
	TArray<VkSemaphore>				mImageAvailableSemaphores;
	TArray<VkSemaphore>				mRenderFinishedSemaphores;
	VkFence							mInFlightFences[cFrameCount];
	Ref<TextureVK>					mShadowMap;
	unique_ptr<ConstantBufferVK>	mVertexShaderConstantBufferProjection[cFrameCount];
	unique_ptr<ConstantBufferVK>	mVertexShaderConstantBufferOrtho[cFrameCount];
	unique_ptr<ConstantBufferVK>	mPixelShaderConstantBuffer[cFrameCount];

	struct Key
	{
		bool						operator == (const Key &inRHS) const
		{
			return mSize == inRHS.mSize && mUsage == inRHS.mUsage && mProperties == inRHS.mProperties;
		}

		VkDeviceSize				mSize;
		VkBufferUsageFlags			mUsage;
		VkMemoryPropertyFlags		mProperties;
	};

	MOSS_MAKE_HASH_STRUCT(Key, KeyHasher, t.mSize, t.mUsage, t.mProperties)

	// We try to recycle buffers from frame to frame
	using BufferCache = TMap<Key, TArray<BufferVK>, KeyHasher>;

	BufferCache						mFreedBuffers[cFrameCount];
	BufferCache						mBufferCache;

	// Smaller allocations (from cMinAllocSize to cMaxAllocSize) will be done in blocks of cBlockSize bytes.
	// We do this because there is a limit to the number of allocations that we can make in Vulkan.
	static constexpr VkDeviceSize	cMinAllocSize = 512;
	static constexpr VkDeviceSize	cMaxAllocSize = 65536;
	static constexpr VkDeviceSize	cBlockSize = 524288;

	MOSS_MAKE_HASH_STRUCT(Key, MemKeyHasher, t.mUsage, t.mProperties, t.mSize)

	struct Memory
	{
		VkDeviceMemory				mMemory;
		VkDeviceSize				mOffset;
	};

	using MemoryCache = TMap<Key, TArray<Memory>, KeyHasher>;

	MemoryCache						mMemoryCache;
	uint32							mNumAllocations = 0;
	uint32							mMaxNumAllocations = 0;
	VkDeviceSize					mTotalAllocated = 0;
	VkDeviceSize					mMaxTotalAllocated = 0;
};

#endif // MOSS_TEXT_VK
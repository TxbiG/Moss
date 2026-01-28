#include <Moss/Renderer/Renderer_intern.h>


// Forward+ and Mobile

void Moss_GPUFatalError(Moss_Result inVkResult) { if (inVkResult != VK_SUCCESS) MOSS_FATAL("Vulkan error returned: %d", inVkResult); }


void Moss_TerminateRenderer(Moss_Renderer* renderer) {
	vkDeviceWaitIdle(mDevice);

	// Trace allocation stats
	Trace("VK: Max allocations: %u, max size: %u MB", mMaxNumAllocations, uint32(mMaxTotalAllocated >> 20));

	// Destroy the shadow map
	mShadowMap = nullptr;
	vkDestroyFramebuffer(mDevice, mShadowFrameBuffer, nullptr);

	// Release constant buffers
	for (unique_ptr<ConstantBufferVK> &cb : mVertexShaderConstantBufferProjection) cb = nullptr;
	for (unique_ptr<ConstantBufferVK> &cb : mVertexShaderConstantBufferOrtho) cb = nullptr;
	for (unique_ptr<ConstantBufferVK> &cb : mPixelShaderConstantBuffer) cb = nullptr;
	
	// Free all buffers
	for (BufferCache &bc : mFreedBuffers)
		for (BufferCache::value_type &vt : bc)
			for (BufferVK &bvk : vt.second)
				FreeBufferInternal(bvk);
	for (BufferCache::value_type &vt : mBufferCache)
		for (BufferVK &bvk : vt.second)
			FreeBufferInternal(bvk);

	// Free all blocks in the memory cache
	for (MemoryCache::value_type &mc : mMemoryCache)
		for (Memory &m : mc.second)
			if (m.mOffset == 0)
				vkFreeMemory(mDevice, m.mMemory, nullptr); // Don't care about memory tracking anymore
	
	for (VkFence fence : mInFlightFences)
		vkDestroyFence(mDevice, fence, nullptr);

	vkDestroyCommandPool(mDevice, mCommandPool, nullptr);

	vkDestroyPipelineLayout(mDevice, mPipelineLayout, nullptr);

	vkDestroyRenderPass(mDevice, mRenderPassShadow, nullptr);
	vkDestroyRenderPass(mDevice, mRenderPass, nullptr);

	vkDestroyDescriptorPool(mDevice, mDescriptorPool, nullptr);

	vkDestroySampler(mDevice, mTextureSamplerShadow, nullptr);
	vkDestroySampler(mDevice, mTextureSamplerRepeat, nullptr);

	vkDestroyDescriptorSetLayout(mDevice, mDescriptorSetLayoutUBO, nullptr);
	vkDestroyDescriptorSetLayout(mDevice, mDescriptorSetLayoutTexture, nullptr);

	DestroySwapChain();

	vkDestroySurfaceKHR(mInstance, mSurface, nullptr);

	vkDestroyDevice(mDevice, nullptr);

#ifdef MOSS_DEBUG
	PFN_vkDestroyDebugUtilsMessengerEXT vkDestroyDebugUtilsMessengerEXT = (PFN_vkDestroyDebugUtilsMessengerEXT)(void *)vkGetInstanceProcAddr(mInstance, "vkDestroyDebugUtilsMessengerEXT");
	if (vkDestroyDebugUtilsMessengerEXT != nullptr)
		vkDestroyDebugUtilsMessengerEXT(mInstance, mDebugMessenger, nullptr);
#endif

	vkDestroyInstance(mInstance, nullptr);
}


Moss_Renderer* Moss_CreateRenderer(Moss_Window* window) {
    Moss_Renderer* renderer = (Moss_Renderer*)malloc(sizeof(Moss_Renderer));
    memset(renderer, 0, sizeof(Moss_Renderer));

    // 1. Create Vulkan Instance
    std::vector<const char*> layers;
    std::vector<const char*> extensions = { VK_KHR_SURFACE_EXTENSION_NAME };

#ifdef MOSS_PLATFORM_WINDOWS
    extensions.push_back("VK_KHR_win32_surface");
#elif defined(MOSS_PLATFORM_MACOS) && defined(MOSS_GRAPHICS_MOLTENVK)
    extensions.push_back("VK_MVK_macos_surface");
#elif defined(MOSS_PLATFORM_LINUX)
    extensions.push_back("VK_KHR_xcb_surface");
#endif

#ifdef MOSS_DEBUG
    layers.push_back("VK_LAYER_KHRONOS_validation");
    extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
#endif

    VkApplicationInfo appInfo{};
    appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    appInfo.pApplicationName = "Moss";
    appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
    appInfo.pEngineName = "Moss Framework";
    appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
    appInfo.apiVersion = VK_API_VERSION_1_2;

    VkInstanceCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    createInfo.pApplicationInfo = &appInfo;
    createInfo.enabledExtensionCount = (uint32_t)extensions.size();
    createInfo.ppEnabledExtensionNames = extensions.data();
    createInfo.enabledLayerCount = (uint32_t)layers.size();
    createInfo.ppEnabledLayerNames = layers.data();

    VkResult res = vkCreateInstance(&createInfo, nullptr, &renderer->m_instance);
    MOSS_ASSERT(res == VK_SUCCESS, "Failed to create Vulkan instance");

    // 2. Create Surface
    res = Moss_CreateWindowSurface(window, renderer->m_instance, nullptr, &renderer->m_surface);
    MOSS_ASSERT(res == VK_SUCCESS, "Failed to create window surface");

    // 3. Choose Physical Device
    uint32_t deviceCount = 0;
    vkEnumeratePhysicalDevices(renderer->m_instance, &deviceCount, nullptr);
    if (deviceCount == 0) return nullptr; // false -> nullptr, since returning pointer

    std::vector<VkPhysicalDevice> devices(deviceCount);
    vkEnumeratePhysicalDevices(renderer->m_instance, &deviceCount, devices.data());

    for (const auto& device : devices)
    {
        if (renderer->IsDeviceSuitable(device))
        {
            renderer->m_physicalDevice.physicalDevice = device;
            vkGetPhysicalDeviceProperties(device, &renderer->m_physicalDevice.properties);
            vkGetPhysicalDeviceMemoryProperties(device, &renderer->m_physicalDevice.memoryProperties);
            vkGetPhysicalDeviceFeatures(device, &renderer->m_physicalDevice.PhysDevicefeatures);
            vkGetPhysicalDeviceSurfaceCapabilitiesKHR(device, renderer->m_surface, &renderer->m_physicalDevice.surfaceCapabilities);
            // You probably want to also get surfaceFormats and presentModes here into vectors
        }
    }

    // 4. Create Logical Device
    float queuePriority = 1.0f;
    std::vector<VkDeviceQueueCreateInfo> queueCreateInfos;

    std::set<uint32_t> uniqueQueueFamilies = { renderer->m_GraphicsQueueIndex, renderer->m_PresentQueueIndex };
    for (uint32_t queueFamily : uniqueQueueFamilies)
    {
        VkDeviceQueueCreateInfo queueCreateInfo{};
        queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
        queueCreateInfo.queueFamilyIndex = queueFamily;
        queueCreateInfo.queueCount = 1;
        queueCreateInfo.pQueuePriorities = &queuePriority;
        queueCreateInfos.push_back(queueCreateInfo);
    }

    VkDeviceCreateInfo deviceCreateInfo{};
    deviceCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    deviceCreateInfo.queueCreateInfoCount = static_cast<uint32_t>(queueCreateInfos.size());
    deviceCreateInfo.pQueueCreateInfos = queueCreateInfos.data();

    VkPhysicalDeviceFeatures deviceFeatures{};
    deviceFeatures.samplerAnisotropy = VK_TRUE;
    deviceCreateInfo.pEnabledFeatures = &deviceFeatures;

    std::vector<const char*> deviceExtensions = {
        VK_KHR_SWAPCHAIN_EXTENSION_NAME
    };

    deviceCreateInfo.enabledExtensionCount = static_cast<uint32_t>(deviceExtensions.size());
    deviceCreateInfo.ppEnabledExtensionNames = deviceExtensions.data();

    if (vkCreateDevice(renderer->m_physicalDevice.physicalDevice, &deviceCreateInfo, nullptr, &renderer->m_device) != VK_SUCCESS)
        return nullptr;

    vkGetDeviceQueue(renderer->m_device, renderer->m_GraphicsQueueIndex, 0, &renderer->m_GraphicsQueue);
    vkGetDeviceQueue(renderer->m_device, renderer->m_PresentQueueIndex, 0, &renderer->m_PresentQueue);

    // 5. Swapchain
    const auto& capabilities = renderer->m_physicalDevice.surfaceCapabilities;

    VkSurfaceFormatKHR chosenFormat = renderer->m_physicalDevice.surfaceFormats[0];
    for (const auto& format : renderer->m_physicalDevice.surfaceFormats)
    {
        if (format.format == VK_FORMAT_B8G8R8A8_UNORM &&
            format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR)
        {
            chosenFormat = format;
            break;
        }
    }

    VkPresentModeKHR chosenPresentMode = VK_PRESENT_MODE_FIFO_KHR;
    for (const auto& mode : renderer->m_physicalDevice.presentModes)
    {
        if (mode == VK_PRESENT_MODE_MAILBOX_KHR)
        {
            chosenPresentMode = mode;
            break;
        }
    }

    VkExtent2D extent = {};
    if (capabilities.currentExtent.width != UINT32_MAX)
    {
        extent = capabilities.currentExtent;
    }
    else
    {
        extent.width = std::clamp<uint32_t>(
            static_cast<uint32_t>(Moss_GetWindowWidth()), 
            capabilities.minImageExtent.width, 
            capabilities.maxImageExtent.width);

        extent.height = std::clamp<uint32_t>(
            static_cast<uint32_t>(Moss_GetWindowHeight()), 
            capabilities.minImageExtent.height, 
            capabilities.maxImageExtent.height);
    }

    uint32_t imageCount = capabilities.minImageCount + 1;
    if (capabilities.maxImageCount > 0 && imageCount > capabilities.maxImageCount)
        imageCount = capabilities.maxImageCount;

    VkSwapchainCreateInfoKHR swapchainCreateInfo{};
    swapchainCreateInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
    swapchainCreateInfo.surface = renderer->m_surface;
    swapchainCreateInfo.minImageCount = imageCount;
    swapchainCreateInfo.imageFormat = chosenFormat.format;
    swapchainCreateInfo.imageColorSpace = chosenFormat.colorSpace;
    swapchainCreateInfo.imageExtent = extent;
    swapchainCreateInfo.imageArrayLayers = 1;
    swapchainCreateInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

    uint32_t queueFamilyIndices[] = { renderer->m_GraphicsQueueIndex, renderer->m_PresentQueueIndex };
    if (renderer->m_GraphicsQueueIndex != renderer->m_PresentQueueIndex)
    {
        swapchainCreateInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
        swapchainCreateInfo.queueFamilyIndexCount = 2;
        swapchainCreateInfo.pQueueFamilyIndices = queueFamilyIndices;
    }
    else
    {
        swapchainCreateInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    }

    swapchainCreateInfo.preTransform = capabilities.currentTransform;
    swapchainCreateInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    swapchainCreateInfo.presentMode = chosenPresentMode;
    swapchainCreateInfo.clipped = VK_TRUE;
    swapchainCreateInfo.oldSwapchain = VK_NULL_HANDLE;

    if (vkCreateSwapchainKHR(renderer->m_device, &swapchainCreateInfo, nullptr, &renderer->m_SwapChain) != VK_SUCCESS)
    {
        return nullptr;
    }

    uint32_t actualImageCount = 0;
    vkGetSwapchainImagesKHR(renderer->m_device, renderer->m_SwapChain, &actualImageCount, nullptr);

    renderer->m_SwapChainImages.resize(actualImageCount);
    vkGetSwapchainImagesKHR(renderer->m_device, renderer->m_SwapChain, &actualImageCount, renderer->m_SwapChainImages.data());

    renderer->m_SwapChainImageFormat = chosenFormat.format;
    renderer->m_SwapChainExtent = extent;

    renderer->m_SwapChainImageViews.resize(actualImageCount);

    for (uint32_t i = 0; i < actualImageCount; ++i) {
        VkImageViewCreateInfo viewInfo{};
        viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        viewInfo.image = renderer->m_SwapChainImages[i];
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = renderer->m_SwapChainImageFormat;
        viewInfo.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
        viewInfo.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
        viewInfo.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
        viewInfo.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
        viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        viewInfo.subresourceRange.baseMipLevel = 0;
        viewInfo.subresourceRange.levelCount = 1;
        viewInfo.subresourceRange.baseArrayLayer = 0;
        viewInfo.subresourceRange.layerCount = 1;

        if (vkCreateImageView(renderer->m_device, &viewInfo, nullptr, &renderer->m_SwapChainImageViews[i]) != VK_SUCCESS) {
            throw std::runtime_error("failed to create image views!");
        }
    }

    // Create Command Pool
    VkCommandPoolCreateInfo poolInfo{};
    poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    poolInfo.queueFamilyIndex = renderer->m_GraphicsQueueIndex;
    poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;

    if (vkCreateCommandPool(renderer->m_device, &poolInfo, nullptr, &renderer->m_CommandPool) != VK_SUCCESS) { return nullptr; }

    // Create Command Buffers
    VkCommandBufferAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocInfo.commandPool = renderer->m_CommandPool;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandBufferCount = cFrameCount;

    if (vkAllocateCommandBuffers(renderer->m_device, &allocInfo, renderer->m_CommandBuffers) != VK_SUCCESS) { return nullptr; }

    // Vulkan Frames - semaphores and fences setup
    VkSemaphoreCreateInfo semaphoreInfo{};
    semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

    VkFenceCreateInfo fenceInfo{};
    fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
    fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

    renderer->m_ImageAvailableSemaphores.resize(cFrameCount);
    renderer->m_RenderFinishedSemaphores.resize(cFrameCount);
    for (uint32_t i = 0; i < cFrameCount; ++i)
    {
        if (vkCreateSemaphore(renderer->m_device, &semaphoreInfo, nullptr, &renderer->m_ImageAvailableSemaphores[i]) != VK_SUCCESS ||
            vkCreateSemaphore(renderer->m_device, &semaphoreInfo, nullptr, &renderer->m_RenderFinishedSemaphores[i]) != VK_SUCCESS ||
            vkCreateFence(renderer->m_device, &fenceInfo, nullptr, &renderer->m_InFlightFences[i]) != VK_SUCCESS)
        {
            return nullptr;
        }
    }

    MOSS_INFO("%s.", renderer->m_physicalDevice.properties.deviceName);
    MOSS_INFO("Vulkan API version: %u.%u.%u",
        renderer->m_physicalDevice.m_apiVersion.Major,
        renderer->m_physicalDevice.m_apiVersion.Minor,
        renderer->m_physicalDevice.m_apiVersion.Patch);

    return renderer;
}


void CreateSwapChain(VkPhysicalDevice inDevice) {
	// Select the format
	VkSurfaceFormatKHR format = SelectFormat(inDevice);
	mSwapChainImageFormat = format.format;

	// Determine swap chain extent
	VkSurfaceCapabilitiesKHR capabilities;
	vkGetPhysicalDeviceSurfaceCapabilitiesKHR(inDevice, mSurface, &capabilities);
	mSwapChainExtent = capabilities.currentExtent;
	if (mSwapChainExtent.width == UINT32_MAX || mSwapChainExtent.height == UINT32_MAX)
		mSwapChainExtent = { uint32(mWindow->GetWindowWidth()), uint32(mWindow->GetWindowHeight()) };
	mSwapChainExtent.width = Clamp(mSwapChainExtent.width, capabilities.minImageExtent.width, capabilities.maxImageExtent.width);
	mSwapChainExtent.height = Clamp(mSwapChainExtent.height, capabilities.minImageExtent.height, capabilities.maxImageExtent.height);
	Trace("VK: Create swap chain %ux%u", mSwapChainExtent.width, mSwapChainExtent.height);

	// Early out if our window has been minimized
	if (mSwapChainExtent.width == 0 || mSwapChainExtent.height == 0)
		return;

	// Create the swap chain
	uint32 desired_image_count = max(min(cFrameCount, capabilities.maxImageCount), capabilities.minImageCount);
	VkSwapchainCreateInfoKHR swapchain_create_info = {};
	swapchain_create_info.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
	swapchain_create_info.surface = mSurface;
	swapchain_create_info.minImageCount = desired_image_count;
	swapchain_create_info.imageFormat = format.format;
	swapchain_create_info.imageColorSpace = format.colorSpace;
	swapchain_create_info.imageExtent = mSwapChainExtent;
	swapchain_create_info.imageTArrayLayers = 1;
	swapchain_create_info.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
	uint32 queue_family_indices[] = { mGraphicsQueueIndex, mPresentQueueIndex };
	if (mGraphicsQueueIndex != mPresentQueueIndex)
	{
		swapchain_create_info.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
		swapchain_create_info.queueFamilyIndexCount = 2;
		swapchain_create_info.pQueueFamilyIndices = queue_family_indices;
	}
	else
	{
		swapchain_create_info.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
	}
	swapchain_create_info.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
	swapchain_create_info.preTransform = capabilities.currentTransform;
	swapchain_create_info.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
	swapchain_create_info.presentMode = VK_PRESENT_MODE_FIFO_KHR;
	swapchain_create_info.clipped = VK_TRUE;
	//FatalErrorIfFailed(vkCreateSwapchainKHR(mDevice, &swapchain_create_info, nullptr, &mSwapChain));

	// Get the actual swap chain image count
	uint32 image_count;
	//FatalErrorIfFailed(vkGetSwapchainImagesKHR(mDevice, mSwapChain, &image_count, nullptr));

	// Get the swap chain images
	mSwapChainImages.resize(image_count);
	//FatalErrorIfFailed(vkGetSwapchainImagesKHR(mDevice, mSwapChain, &image_count, mSwapChainImages.data()));

	// Create image views
	mSwapChainImageViews.resize(image_count);
	for (uint32 i = 0; i < image_count; ++i)
		mSwapChainImageViews[i] = CreateImageView(mSwapChainImages[i], mSwapChainImageFormat, VK_IMAGE_ASPECT_COLOR_BIT);

	// Create depth buffer
	VkFormat depth_format = FindDepthFormat();
	VkImageUsageFlags depth_usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
	VkMemoryPropertyFlags depth_memory_properties = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;

	// Test and utilize support for transient memory for the depth buffer
	VkImageFormatProperties depth_transient_properties = {};
	VkResult depth_transient_support = vkGetPhysicalDeviceImageFormatProperties(mPhysicalDevice, depth_format, VK_IMAGE_TYPE_2D, VK_IMAGE_TILING_OPTIMAL, depth_usage | VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT, 0, &depth_transient_properties);
	if (depth_transient_support == VK_SUCCESS)
	{
		depth_usage |= VK_IMAGE_USAGE_TRANSIENT_ATTACHMENT_BIT;

		// Test and utilize lazily allocated memory for the depth buffer
		for (size_t i = 0; i < mMemoryProperties.memoryTypeCount; i++)
			if (mMemoryProperties.memoryTypes[i].propertyFlags & VK_MEMORY_PROPERTY_LAZILY_ALLOCATED_BIT)
			{
				depth_memory_properties = VK_MEMORY_PROPERTY_LAZILY_ALLOCATED_BIT;
				break;
			}
	}

	CreateImage(mSwapChainExtent.width, mSwapChainExtent.height, depth_format, VK_IMAGE_TILING_OPTIMAL, depth_usage, depth_memory_properties, mDepthImage, mDepthImageMemory);
	mDepthImageView = CreateImageView(mDepthImage, depth_format, VK_IMAGE_ASPECT_DEPTH_BIT);

	// Create frame buffers for the normal pass
	mSwapChainFramebuffers.resize(image_count);
	for (size_t i = 0; i < mSwapChainFramebuffers.size(); i++)
	{
		VkImageView attachments[] = { mSwapChainImageViews[i], mDepthImageView };
		VkFramebufferCreateInfo frame_buffer_info = {};
		frame_buffer_info.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
		frame_buffer_info.renderPass = mRenderPass;
		frame_buffer_info.attachmentCount = std::size(attachments);
		frame_buffer_info.pAttachments = attachments;
		frame_buffer_info.width = mSwapChainExtent.width;
		frame_buffer_info.height = mSwapChainExtent.height;
		frame_buffer_info.layers = 1;
		//FatalErrorIfFailed(vkCreateFramebuffer(mDevice, &frame_buffer_info, nullptr, &mSwapChainFramebuffers[i]));
	}

	// Allocate space to remember the image available semaphores
	mImageAvailableSemaphores.resize(image_count, VK_NULL_HANDLE);

	// Allocate the render finished semaphores
	mRenderFinishedSemaphores.resize(image_count, VK_NULL_HANDLE);
	for (uint32 i = 0; i < image_count; ++i)
		mRenderFinishedSemaphores[i] = AllocateSemaphore();
}

void RendererVK::DestroySwapChain()
{
	// Destroy semaphores
	for (VkSemaphore semaphore : mImageAvailableSemaphores) 
		vkDestroySemaphore(mDevice, semaphore, nullptr);
	mImageAvailableSemaphores.clear();

	for (VkSemaphore semaphore : mRenderFinishedSemaphores) 
		vkDestroySemaphore(mDevice, semaphore, nullptr);
	mRenderFinishedSemaphores.clear();

	for (VkSemaphore semaphore : mAvailableSemaphores) 
		vkDestroySemaphore(mDevice, semaphore, nullptr);
	mAvailableSemaphores.clear();

	// Destroy depth buffer
	if (mDepthImageView != VK_NULL_HANDLE)
	{
		vkDestroyImageView(mDevice, mDepthImageView, nullptr);
		mDepthImageView = VK_NULL_HANDLE;

		DestroyImage(mDepthImage, mDepthImageMemory);
		mDepthImage = VK_NULL_HANDLE;
		mDepthImageMemory = VK_NULL_HANDLE;
	}

	for (VkFramebuffer frame_buffer : mSwapChainFramebuffers)
		vkDestroyFramebuffer(mDevice, frame_buffer, nullptr);
	mSwapChainFramebuffers.clear();

	for (VkImageView view : mSwapChainImageViews)
		vkDestroyImageView(mDevice, view, nullptr);
	mSwapChainImageViews.clear();

	mSwapChainImages.clear();

	if (mSwapChain != VK_NULL_HANDLE)
	{
		vkDestroySwapchainKHR(mDevice, mSwapChain, nullptr);
		mSwapChain = VK_NULL_HANDLE;
	}
}

void Moss_RendererBeginFrame(Moss_Renderer* renderer) {
    // Wait for previous frame fence to ensure GPU is done with that frame
    vkWaitForFences(renderer->m_device, 1, &renderer->m_InFlightFences[renderer->m_CurrentFrame], VK_TRUE, UINT64_MAX);
    vkResetFences(renderer->m_device, 1, &renderer->m_InFlightFences[renderer->m_CurrentFrame]);

    // Acquire next image from swapchain
    VkResult result = vkAcquireNextImageKHR(
        renderer->m_device,
        renderer->m_SwapChain,
        UINT64_MAX,
        renderer->m_ImageAvailableSemaphores[renderer->m_CurrentFrame],  // semaphore signaled when image is ready
        VK_NULL_HANDLE,
        &renderer->m_ImageIndex
    );

    if (result == VK_ERROR_OUT_OF_DATE_KHR) {
        CreateSwapChain(renderer);
        return;
    } else if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR) {
        throw std::runtime_error("Failed to acquire swap chain image!");
    }

    if (renderer->m_ImageIndex >= renderer->m_SwapChainFramebuffers.size()) {
        // This shouldn't happen, so recreate the swapchain just in case
        CreateSwapChain(renderer);
        return;
    }

    // Begin command buffer recording for the acquired image
    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

    vkResetCommandBuffer(renderer->m_CommandBuffers[renderer->m_ImageIndex], 0);
    if (vkBeginCommandBuffer(renderer->m_CommandBuffers[renderer->m_ImageIndex], &beginInfo) != VK_SUCCESS) {
        throw std::runtime_error("Failed to begin recording command buffer!");
    }

    // --- START RENDER PASS HERE ---
    VkRenderPassBeginInfo renderPassInfo{};
    renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    renderPassInfo.renderPass = renderer->m_RenderPass;
    renderPassInfo.framebuffer = renderer->m_SwapChainFramebuffers[renderer->m_ImageIndex];
    renderPassInfo.renderArea.offset = {0, 0};
    renderPassInfo.renderArea.extent = renderer->m_SwapChainExtent;

    VkClearValue clearColor = {{{0.0f, 0.0f, 0.0f, 1.0f}}};
    renderPassInfo.clearValueCount = 1;
    renderPassInfo.pClearValues = &clearColor;

    vkCmdBeginRenderPass(renderer->m_CommandBuffers[renderer->m_ImageIndex], &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);

    // TODO: Record your draw commands here

    // --- END RENDER PASS ---
    vkCmdEndRenderPass(renderer->m_CommandBuffers[renderer->m_ImageIndex]);

    // Then end command buffer recording here or in EndFrame as you prefer
}



void Moss_RendererEndFrame(Moss_Renderer* renderer) {
    if (vkEndCommandBuffer(renderer->m_CommandBuffers[renderer->m_ImageIndex]) != VK_SUCCESS) { throw std::runtime_error("Failed to record command buffer!"); }

    VkSubmitInfo submitInfo{};
    submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;

    VkSemaphore waitSemaphores[] = {renderer->m_ImageAvailableSemaphores[renderer->m_CurrentFrame]};
    VkPipelineStageFlags waitStages[] = {VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT};
    submitInfo.waitSemaphoreCount = 1;
    submitInfo.pWaitSemaphores = waitSemaphores;
    submitInfo.pWaitDstStageMask = waitStages;

    submitInfo.commandBufferCount = 1;
    submitInfo.pCommandBuffers = &renderer->m_CommandBuffers[renderer->m_ImageIndex];

    VkSemaphore signalSemaphores[] = {renderer->m_RenderFinishedSemaphores[renderer->m_CurrentFrame]};
    submitInfo.signalSemaphoreCount = 1;
    submitInfo.pSignalSemaphores = signalSemaphores;

    if (vkQueueSubmit(renderer->m_GraphicsQueue, 1, &submitInfo, renderer->m_InFlightFences[renderer->m_CurrentFrame]) != VK_SUCCESS) { throw std::runtime_error("Failed to submit draw command buffer!"); }

    VkPresentInfoKHR presentInfo{};
    presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;

    presentInfo.waitSemaphoreCount = 1;
    presentInfo.pWaitSemaphores = signalSemaphores;

    VkSwapchainKHR swapChains[] = {renderer->m_SwapChain};
    presentInfo.swapchainCount = 1;
    presentInfo.pSwapchains = swapChains;
    presentInfo.pImageIndices = &renderer->m_ImageIndex;

    VkResult result = vkQueuePresentKHR(renderer->m_PresentQueue, &presentInfo);

    if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR) { Moss_RecreateSwapchain(renderer); } 
    else if (result != VK_SUCCESS) { throw std::runtime_error("Failed to present swap chain image!"); }

    renderer->m_CurrentFrame = (renderer->m_CurrentFrame + 1) % cFrameCount;


    VkCommandBuffer command_buffer = GetCommandBuffer();
	vkCmdEndRenderPass(command_buffer);

	//FatalErrorIfFailed(vkEndCommandBuffer(command_buffer));

	VkSemaphore wait_semaphores[] = { mImageAvailableSemaphores[mImageIndex] };
	VkSemaphore signal_semaphores[] = { mRenderFinishedSemaphores[mImageIndex] };
	VkPipelineStageFlags wait_stages[] = { VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT };
	VkSubmitInfo submit_info = {};
	submit_info.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submit_info.waitSemaphoreCount = 1;
	submit_info.pWaitSemaphores = wait_semaphores;
	submit_info.pWaitDstStageMask = wait_stages;
	submit_info.commandBufferCount = 1;
	submit_info.pCommandBuffers = &command_buffer;
	submit_info.signalSemaphoreCount = 1;
	submit_info.pSignalSemaphores = signal_semaphores;
	//FatalErrorIfFailed(vkQueueSubmit(mGraphicsQueue, 1, &submit_info, mInFlightFences[mFrameIndex]));

	VkSwapchainKHR swap_chains[] = { mSwapChain };
	VkPresentInfoKHR present_info = {};
	present_info.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
	present_info.waitSemaphoreCount = 1;
	present_info.pWaitSemaphores = signal_semaphores;
	present_info.swapchainCount = 1;
	present_info.pSwapchains = swap_chains;
	present_info.pImageIndices = &mImageIndex;
	vkQueuePresentKHR(mPresentQueue, &present_info);
}





void CreateDepthResources() {
    VkFormat depthFormat = FindDepthFormat();

    CreateImage(
        swapchainExtent.width,
        swapchainExtent.height,
        depthFormat,
        VK_IMAGE_TILING_OPTIMAL,
        VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
        depthImage,
        depthImageMemory
    );

    depthImageView = CreateImageView(depthImage, depthFormat, VK_IMAGE_ASPECT_DEPTH_BIT);
}

void CreateFramebuffers() {
    swapchainFramebuffers.resize(swapchainImageViews.size());

    for (size_t i = 0; i < swapchainImageViews.size(); i++) {
        std::array<VkImageView, 2> attachments = { swapchainImageViews[i], depthImageView };

        VkFramebufferCreateInfo framebufferInfo{};
        framebufferInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        framebufferInfo.renderPass = renderPass;
        framebufferInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
        framebufferInfo.pAttachments = attachments.data();
        framebufferInfo.width  = swapchainExtent.width;
        framebufferInfo.height = swapchainExtent.height;
        framebufferInfo.layers = 1;

        if (vkCreateFramebuffer(device, &framebufferInfo, nullptr, &swapchainFramebuffers[i]) != VK_SUCCESS) {
            throw std::runtime_error("failed to create framebuffer!");
        }
    }
}


void Moss_WindowResizeVK(VkPhysicalDevice physicalDevice, VkDevice device, VkSwapchainKHR& swapchain, VkExtent2D& swapchainExtent, 
    std::vector<VkFramebuffer>& swapchainFramebuffers, std::vector<VkImageView>& swapchainImageViews, VkImage& depthImage, VkDeviceMemory& depthImageMemory, VkImageView& depthImageView) {

    vkDeviceWaitIdle(device);

    for (VkFramebuffer fb : swapchainFramebuffers)
        vkDestroyFramebuffer(device, fb, nullptr);
    swapchainFramebuffers.clear();

    vkDestroyImageView(device, depthImageView, nullptr);
    vkDestroyImage(device, depthImage, nullptr);
    vkFreeMemory(device, depthImageMemory, nullptr);

    depthImageView = VK_NULL_HANDLE;
    depthImage = VK_NULL_HANDLE;
    depthImageMemory = VK_NULL_HANDLE;

    for (VkImageView view : swapchainImageViews)
        vkDestroyImageView(device, view, nullptr);
    swapchainImageViews.clear();

    vkDestroySwapchainKHR(device, swapchain, nullptr);
    swapchain = VK_NULL_HANDLE;

    if (swapchainExtent.width == 0 || swapchainExtent.height == 0)
        return;

    CreateSwapChain(physicalDevice);
}
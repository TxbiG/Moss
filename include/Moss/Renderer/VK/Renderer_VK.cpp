#include <Moss/Renderer/VK/Renderer_VK.h>

#ifdef MOSS_PLATFORM_WINDOW
#include <Moss/Platform/Windows/win32_platform.h>
#elif defined(MOSS_PLATFORM_LINUX)
    #ifdef MOSS_USE_WAYLAND
        #include <Moss/Platform/Linux/wl_platform.h>
    #else
        #include <Moss/Platform/Linux/x11_platform.h>
    #endif // MOSS_USE_WAYLAND
#endif
#include <vulkan/vulkan.h>
#include <set>
#include <cassert>
// Supported version history: Vulkan Version 1.0-1.4

// Look into Global semaphores for compute or graphics sync across frames.
// Finish this https://vulkan-tutorial.com/Drawing_a_triangle/Setup/Validation_layers


// in release builds make sure to disable validation layers.
// VulkanMemoryAllocator makes memory management easier.
// Use Dynamic rendering and Attachment Options
// Enable processing shaders in the background, or pre-compile them to avoid stuttering during gameplay. 
// Choose graphics settings that match your hardware and desired performance. 
// Consider using a 24-bit depth map format (VK_FORMAT_D24_UNORM_S8_UINT) instead of a 32-bit format for potential performance gains. 
// Use double buffering (swapping buffers) to improve performance.

#include <set>

VkFormat FindSupportedFormat( VkPhysicalDevice physicalDevice, const std::vector<VkFormat>& candidates, VkImageTiling tiling, VkFormatFeatureFlags features) {
    for (VkFormat format : candidates) {
        VkFormatProperties props;
        vkGetPhysicalDeviceFormatProperties(physicalDevice, format, &props);

        if (tiling == VK_IMAGE_TILING_LINEAR && (props.linearTilingFeatures & features) == features) {
            return format;
        }
        else if (tiling == VK_IMAGE_TILING_OPTIMAL && (props.optimalTilingFeatures & features) == features) {
            return format;
        }
    }

    throw std::runtime_error("Failed to find supported format!");
}
VkFormat FindDepthFormat(VkPhysicalDevice physicalDevice) {
    return FindSupportedFormat(
        physicalDevice, 
        { VK_FORMAT_D32_SFLOAT_S8_UINT, VK_FORMAT_D24_UNORM_S8_UINT, VK_FORMAT_D32_SFLOAT },
        VK_IMAGE_TILING_OPTIMAL,
        VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT);
}

template <typename Container>
bool IsExtensionSupported(const Container& extensions, const char* extensionName)
{
    for (size_t i = 0; i < extensions.size(); ++i)
    {
        if (strcmp(extensions[i].extensionName, extensionName) == 0)
            return true;
    }
    return false;
}



/*              Renderer lifecycle          */
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



void Moss_TerminateRenderer(Moss_Renderer* renderer) {
    if (!renderer) return;

    VkDevice device = renderer->m_device;

    // Wait until GPU is done with all work
    if (device != VK_NULL_HANDLE) {
        vkDeviceWaitIdle(device);
    }

    // --- Destroy Constant Buffers ---
    for (int i = 0; i < cFrameCount; ++i) {
        renderer->m_VertexShaderConstantBufferProjection[i].reset();
        renderer->m_VertexShaderConstantBufferOrtho[i].reset();
        renderer->m_PixelShaderConstantBuffer[i].reset();
    }

    // --- Destroy Frame Fences ---
    for (int i = 0; i < cFrameCount; ++i) {
        if (renderer->m_InFlightFences[i]) { vkDestroyFence(device, renderer->m_InFlightFences[i], nullptr); }
    }

    // --- Destroy Semaphores ---
    for (VkSemaphore sem : renderer->m_AvailableSemaphores) { vkDestroySemaphore(device, sem, nullptr); }
    renderer->m_AvailableSemaphores.clear();
    for (VkSemaphore sem : renderer->m_ImageAvailableSemaphores) { vkDestroySemaphore(device, sem, nullptr); }
    renderer->m_ImageAvailableSemaphores.clear();
    for (VkSemaphore sem : renderer->m_RenderFinishedSemaphores) { vkDestroySemaphore(device, sem, nullptr); }
    renderer->m_RenderFinishedSemaphores.clear();

    // --- Destroy Command Buffers and Pool ---
    if (renderer->m_CommandPool) {
        vkFreeCommandBuffers(device, renderer->m_CommandPool, cFrameCount, renderer->m_CommandBuffers);
        
        vkDestroyCommandPool(device, renderer->m_CommandPool, nullptr);
    }

    // --- Destroy Framebuffers ---
    for (VkFramebuffer fb : renderer->m_SwapChainFramebuffers) { vkDestroyFramebuffer(device, fb, nullptr); }
    renderer->m_SwapChainFramebuffers.clear();

    if (renderer->m_ShadowFrameBuffer) { vkDestroyFramebuffer(device, renderer->m_ShadowFrameBuffer, nullptr); }

    // --- Destroy Render Passes ---
    if (renderer->m_RenderPass) { vkDestroyRenderPass(device, renderer->m_RenderPass, nullptr); }
    if (renderer->m_RenderPassShadow) { vkDestroyRenderPass(device, renderer->m_RenderPassShadow, nullptr); }

    // --- Destroy Pipeline Layout ---
    if (renderer->m_PipelineLayout) { vkDestroyPipelineLayout(device, renderer->m_PipelineLayout, nullptr); }

    // --- Destroy Samplers ---
    if (renderer->m_TextureSamplerShadow) { vkDestroySampler(device, renderer->m_TextureSamplerShadow, nullptr); }
    if (renderer->m_TextureSamplerRepeat) { vkDestroySampler(device, renderer->m_TextureSamplerRepeat, nullptr); }

    // --- Destroy Descriptor Sets ---
    // (You don’t destroy descriptor sets manually if they’re allocated from a pool, just destroy the pool)
    renderer->m_BufferCache.clear();
    for (int i = 0; i < cFrameCount; ++i) {
        renderer->m_FreedBuffers[i].clear();
    }

    // --- Destroy Descriptor Set Layouts ---
    if (renderer->m_DescriptorSetLayoutUBO) { vkDestroyDescriptorSetLayout(device, renderer->m_DescriptorSetLayoutUBO, nullptr);}
    if (renderer->m_DescriptorSetLayoutTexture) { vkDestroyDescriptorSetLayout(device, renderer->m_DescriptorSetLayoutTexture, nullptr); }

    // --- Destroy Descriptor Pool ---
    if (renderer->m_DescriptorPool) { vkDestroyDescriptorPool(device, renderer->m_DescriptorPool, nullptr); }

    // --- Destroy Depth Image + View ---
    if (renderer->m_DepthImageView) { vkDestroyImageView(device, renderer->m_DepthImageView, nullptr); }
    if (renderer->m_DepthImage) { vkDestroyImage(device, renderer->m_DepthImage, nullptr); }
    if (renderer->m_DepthImageMemory) { vkFreeMemory(device, renderer->m_DepthImageMemory, nullptr); }

    // --- Destroy Swapchain Image Views ---
    for (VkImageView view : renderer->m_SwapChainImageViews) { vkDestroyImageView(device, view, nullptr); }
    renderer->m_SwapChainImageViews.clear();

    // --- Destroy Swapchain ---
    if (renderer->m_SwapChain) { vkDestroySwapchainKHR(device, renderer->m_SwapChain, nullptr); }

    // --- Destroy Device ---
    if (device != VK_NULL_HANDLE) { vkDestroyDevice(device, nullptr); }

    // --- Destroy Surface ---
    if (renderer->m_surface) { vkDestroySurfaceKHR(renderer->m_instance, renderer->m_surface, nullptr); }

#ifdef MOSS_DEBUG
    if (renderer->m_debugMessenger) {
        auto func = (PFN_vkDestroyDebugUtilsMessengerEXT)
            vkGetInstanceProcAddr(renderer->m_instance, "vkDestroyDebugUtilsMessengerEXT");
        if (func) { func(renderer->m_instance, renderer->m_debugMessenger, nullptr); }
    }
#endif
    if (renderer->m_instance) { vkDestroyInstance(renderer->m_instance, nullptr); }

    free(renderer);
}


void Moss_RecreateSwapchain(Moss_Renderer* renderer) {

    vkDeviceWaitIdle(renderer->m_device);
    // Destroy old framebuffers first
    for (auto framebuffer : renderer->m_SwapChainFramebuffers) {
        vkDestroyFramebuffer(renderer->m_device, framebuffer, nullptr);
    }
    renderer->m_SwapChainFramebuffers.clear();

    const auto& capabilities = renderer->m_physicalDevice.surfaceCapabilities;

    // Choose surface format
    VkSurfaceFormatKHR chosenFormat = renderer->m_physicalDevice.surfaceFormats[0];
    for (const auto& format : renderer->m_physicalDevice.surfaceFormats) {
        if (format.format == VK_FORMAT_B8G8R8A8_UNORM &&
            format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) {
            chosenFormat = format;
            break;
        }
    }

    // Choose present mode
    VkPresentModeKHR chosenPresentMode = VK_PRESENT_MODE_FIFO_KHR;
    for (const auto& mode : renderer->m_physicalDevice.presentModes) {
        if (mode == VK_PRESENT_MODE_MAILBOX_KHR) {
            chosenPresentMode = mode;
            break;
        }
    }

    // Choose extent (size)
    VkExtent2D extent = {};
    if (capabilities.currentExtent.width != UINT32_MAX) {
        extent = capabilities.currentExtent;
    } else {
        extent.width = std::clamp<uint32_t>(Moss_GetWindowWidth(),
            capabilities.minImageExtent.width, capabilities.maxImageExtent.width);
        extent.height = std::clamp<uint32_t>(Moss_GetWindowHeight(),
            capabilities.minImageExtent.height, capabilities.maxImageExtent.height);
    }

    // Choose image count
    uint32_t imageCount = capabilities.minImageCount + 1;
    if (capabilities.maxImageCount > 0 && imageCount > capabilities.maxImageCount)
        imageCount = capabilities.maxImageCount;

    // Fill in swapchain create info
    VkSwapchainKHR oldSwapChain = renderer->m_SwapChain;
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
    if (renderer->m_GraphicsQueueIndex != renderer->m_PresentQueueIndex) {
        swapchainCreateInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
        swapchainCreateInfo.queueFamilyIndexCount = 2;
        swapchainCreateInfo.pQueueFamilyIndices = queueFamilyIndices;
    } else {
        swapchainCreateInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
        swapchainCreateInfo.queueFamilyIndexCount = 0;
        swapchainCreateInfo.pQueueFamilyIndices = nullptr;
    }

    swapchainCreateInfo.preTransform = capabilities.currentTransform;
    swapchainCreateInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    swapchainCreateInfo.presentMode = chosenPresentMode;
    swapchainCreateInfo.clipped = VK_TRUE;
    swapchainCreateInfo.oldSwapchain = oldSwapChain;

    VkSwapchainKHR newSwapChain;
    if (vkCreateSwapchainKHR(renderer->m_device, &swapchainCreateInfo, nullptr, &newSwapChain) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create swapchain!");
    }

    // Update swapchain handle immediately
    renderer->m_SwapChain = newSwapChain;

    // Destroy old swapchain handle if it exists
    if (oldSwapChain != VK_NULL_HANDLE) {
        vkDestroySwapchainKHR(renderer->m_device, oldSwapChain, nullptr);
    }

    // Query swapchain images
    uint32_t actualImageCount = 0;
    vkGetSwapchainImagesKHR(renderer->m_device, renderer->m_SwapChain, &actualImageCount, nullptr);

    renderer->m_SwapChainImages.resize(actualImageCount);
    vkGetSwapchainImagesKHR(renderer->m_device, renderer->m_SwapChain, &actualImageCount, renderer->m_SwapChainImages.data());

    renderer->m_SwapChainImageViews.resize(actualImageCount);

    // Create image views for each swapchain image
    for (uint32_t i = 0; i < actualImageCount; ++i) {
        VkImageViewCreateInfo viewInfo{};
        viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        viewInfo.image = renderer->m_SwapChainImages[i];
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
        viewInfo.format = chosenFormat.format;
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
            throw std::runtime_error("Failed to create image views!");
        }
    }

    renderer->m_SwapChainImageFormat = chosenFormat.format;
    renderer->m_SwapChainExtent = extent;

     if (renderer->m_RenderPass != VK_NULL_HANDLE) {
        vkDestroyRenderPass(renderer->m_device, renderer->m_RenderPass, nullptr);
    }

    VkAttachmentDescription colorAttachment{};
    colorAttachment.format = renderer->m_SwapChainImageFormat;
    colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
    colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    colorAttachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    VkAttachmentReference colorAttachmentRef{};
    colorAttachmentRef.attachment = 0;
    colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount = 1;
    subpass.pColorAttachments = &colorAttachmentRef;

    VkSubpassDependency dependency{};
    dependency.srcSubpass = VK_SUBPASS_EXTERNAL;
    dependency.dstSubpass = 0;
    dependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependency.srcAccessMask = 0;
    dependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;

    VkSubpassDependency dependency2{};
    dependency2.srcSubpass = 0;
    dependency2.dstSubpass = VK_SUBPASS_EXTERNAL;
    dependency2.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependency2.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependency2.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependency2.dstAccessMask = 0;

    VkSubpassDependency dependencies[] = { dependency, dependency2 };

    VkRenderPassCreateInfo renderPassInfo{};
    renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    renderPassInfo.attachmentCount = 1;
    renderPassInfo.pAttachments = &colorAttachment;
    renderPassInfo.subpassCount = 1;
    renderPassInfo.pSubpasses = &subpass;
    renderPassInfo.dependencyCount = 2;
    renderPassInfo.pDependencies = dependencies;

    if (vkCreateRenderPass(renderer->m_device, &renderPassInfo, nullptr, &renderer->m_RenderPass) != VK_SUCCESS) {
        throw std::runtime_error("Failed to create render pass!");
    }

    for (auto& imageView : renderer->m_SwapChainImageViews) {
        vkDestroyImageView(renderer->m_device, imageView, nullptr);
    }
    renderer->m_SwapChainImageViews.clear();
    

    renderer->m_SwapChainFramebuffers.resize(renderer->m_SwapChainImageViews.size());

    for (size_t i = 0; i < renderer->m_SwapChainImageViews.size(); i++) {
        VkImageView attachments[] = {
            renderer->m_SwapChainImageViews[i]
        };

        VkFramebufferCreateInfo framebufferInfo{};
        framebufferInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        framebufferInfo.renderPass = renderer->m_RenderPass;
        framebufferInfo.attachmentCount = 1;
        framebufferInfo.pAttachments = attachments;
        framebufferInfo.width = renderer->m_SwapChainExtent.width;
        framebufferInfo.height = renderer->m_SwapChainExtent.height;
        framebufferInfo.layers = 1;

        if (vkCreateFramebuffer(renderer->m_device, &framebufferInfo, nullptr, &renderer->m_SwapChainFramebuffers[i]) != VK_SUCCESS) {
            throw std::runtime_error("Failed to create framebuffer!");
        }
    }

    // Free old command buffers if you allocated them dynamically
    vkFreeCommandBuffers(renderer->m_device, renderer->m_CommandPool, cFrameCount, renderer->m_CommandBuffers);
    
    // Allocate new command buffers
    VkCommandBufferAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    allocInfo.commandPool = renderer->m_CommandPool;
    allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    allocInfo.commandBufferCount = cFrameCount;

    if (vkAllocateCommandBuffers(renderer->m_device, &allocInfo, renderer->m_CommandBuffers) != VK_SUCCESS) {
        throw std::runtime_error("Failed to allocate command buffers!");
    }

    VkCommandBuffer cmdBuf = renderer->m_CommandBuffers[renderer->m_ImageIndex];

    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    vkBeginCommandBuffer(cmdBuf, &beginInfo);

    VkImageMemoryBarrier barrier{};
    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
    barrier.oldLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
    barrier.newLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    barrier.image = renderer->m_SwapChainImages[renderer->m_ImageIndex];
    barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    barrier.subresourceRange.baseMipLevel = 0;
    barrier.subresourceRange.levelCount = 1;
    barrier.subresourceRange.baseArrayLayer = 0;
    barrier.subresourceRange.layerCount = 1;
    barrier.srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    barrier.dstAccessMask = 0; // presentation doesn't need dstAccessMask

    vkCmdPipelineBarrier(
        renderer->m_CommandBuffers[renderer->m_ImageIndex],
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT,
        VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT,
        0,
        0, nullptr,
        0, nullptr,
        1, &barrier);
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
        Moss_RecreateSwapchain(renderer);
        return;
    } else if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR) {
        throw std::runtime_error("Failed to acquire swap chain image!");
    }

    if (renderer->m_ImageIndex >= renderer->m_SwapChainFramebuffers.size()) {
        // This shouldn't happen, so recreate the swapchain just in case
        Moss_RecreateSwapchain(renderer);
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
    if (vkEndCommandBuffer(renderer->m_CommandBuffers[renderer->m_ImageIndex]) != VK_SUCCESS) {
        throw std::runtime_error("Failed to record command buffer!");
    }

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

    if (vkQueueSubmit(renderer->m_GraphicsQueue, 1, &submitInfo, renderer->m_InFlightFences[renderer->m_CurrentFrame]) != VK_SUCCESS) {
        throw std::runtime_error("Failed to submit draw command buffer!");
    }

    VkPresentInfoKHR presentInfo{};
    presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;

    presentInfo.waitSemaphoreCount = 1;
    presentInfo.pWaitSemaphores = signalSemaphores;

    VkSwapchainKHR swapChains[] = {renderer->m_SwapChain};
    presentInfo.swapchainCount = 1;
    presentInfo.pSwapchains = swapChains;
    presentInfo.pImageIndices = &renderer->m_ImageIndex;

    VkResult result = vkQueuePresentKHR(renderer->m_PresentQueue, &presentInfo);

    if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR) {
        Moss_RecreateSwapchain(renderer);
    } else if (result != VK_SUCCESS) {
        throw std::runtime_error("Failed to present swap chain image!");
    }

    renderer->m_CurrentFrame = (renderer->m_CurrentFrame + 1) % cFrameCount;
}

bool Moss_Renderer::IsDeviceSuitable(VkPhysicalDevice device)
{
    // 1. Queue Family Support (Graphics + Present)
    uint32_t queueFamilyCount = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, nullptr);
    std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
    vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyCount, queueFamilies.data());

    int graphicsIndex = -1;
    int presentIndex = -1;

    for (uint32_t i = 0; i < queueFamilyCount; ++i)
    {
        if (queueFamilies[i].queueFlags & VK_QUEUE_GRAPHICS_BIT)
            graphicsIndex = i;

        VkBool32 presentSupport = false;
        vkGetPhysicalDeviceSurfaceSupportKHR(device, i, m_surface, &presentSupport);
        if (presentSupport)
            presentIndex = i;

        if (graphicsIndex >= 0 && presentIndex >= 0)
            break;
    }

    if (graphicsIndex == -1 || presentIndex == -1)
        return false;

    // Store the queue indices
    m_GraphicsQueueIndex = static_cast<uint32_t>(graphicsIndex);
    m_PresentQueueIndex = static_cast<uint32_t>(presentIndex);
    m_physicalDevice.queueFamilyProperties = queueFamilies;

    // 2. Device Extension Support
    uint32_t extensionCount;
    vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount, nullptr);
    std::vector<VkExtensionProperties> availableExtensions(extensionCount);
    vkEnumerateDeviceExtensionProperties(device, nullptr, &extensionCount, availableExtensions.data());

    std::vector<const char*> requiredExtensions = {
        VK_KHR_SWAPCHAIN_EXTENSION_NAME
    };

    // Check for each required extension
    for (const char* required : requiredExtensions)
    {
        bool found = false;
        for (const auto& ext : availableExtensions)
        {
            if (strcmp(ext.extensionName, required) == 0)
            {
                found = true;
                break;
            }
        }
        if (!found)
            return false;
    }

    m_physicalDevice.m_extensions = availableExtensions;

    // 3. Swapchain Support (Surface formats & Present modes)
    uint32_t formatCount = 0, presentModeCount = 0;
    vkGetPhysicalDeviceSurfaceFormatsKHR(device, m_surface, &formatCount, nullptr);
    vkGetPhysicalDeviceSurfacePresentModesKHR(device, m_surface, &presentModeCount, nullptr);

    if (formatCount == 0 || presentModeCount == 0)
        return false;

    m_physicalDevice.surfaceFormats.resize(formatCount);
    vkGetPhysicalDeviceSurfaceFormatsKHR(device, m_surface, &formatCount, m_physicalDevice.surfaceFormats.data());

    m_physicalDevice.presentModes.resize(presentModeCount);
    vkGetPhysicalDeviceSurfacePresentModesKHR(device, m_surface, &presentModeCount, m_physicalDevice.presentModes.data());

    // 4. (Optional) Feature check
    VkPhysicalDeviceFeatures supportedFeatures;
    vkGetPhysicalDeviceFeatures(device, &supportedFeatures);

    if (!supportedFeatures.samplerAnisotropy)
        return false;

    return true;
}
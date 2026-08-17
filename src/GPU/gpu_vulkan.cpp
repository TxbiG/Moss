//                        MIT License
//
//                  Copyright (c) 2026 Toby

#include "gpu_backend.h"


const Moss_GPUBackend* g_vulkan_backend = nullptr;

bool supports_spirv_shader_formats(Moss_GPUDevice*, uint32_t shader_format_mask) {
    return (shader_format_mask & MOSS_GPU_SHADERFORMAT_SPIRV) != 0 || shader_format_mask == MOSS_GPU_SHADERFORMAT_NONE;
}

const Moss_GPUBackend* Moss_GetVulkanFallbackGPUBackend() {
    static const Moss_GPUBackend* backend = Moss_CreateNamedFallbackGPUBackend(MOSS_GPU_BACKEND_VULKAN, "Vulkan", MOSS_GPU_SHADERFORMAT_SPIRV, supports_spirv_shader_formats);
    return backend ? backend : Moss_GetFallbackGPUBackendTable();
}

const Moss_GPUBackend* Moss_GetVulkanGPUBackend() {
    return g_vulkan_backend ? g_vulkan_backend : Moss_GetVulkanFallbackGPUBackend();
}

void Moss_SetVulkanGPUBackend(const Moss_GPUBackend* backend) {
    g_vulkan_backend = backend;
}

bool Moss_VulkanGPUBackendIsRegistered() {
    return g_vulkan_backend != nullptr;
}
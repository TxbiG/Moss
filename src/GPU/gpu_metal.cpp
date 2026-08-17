//                        MIT License
//
//                  Copyright (c) 2026 Toby

#include "gpu_backend.h"


const Moss_GPUBackend* g_metal_backend = nullptr;

bool supports_metal_shader_formats(Moss_GPUDevice*, uint32_t shader_format_mask) {
    return (shader_format_mask & (MOSS_GPU_SHADERFORMAT_MSL | MOSS_GPU_SHADERFORMAT_METALLIB)) != 0 || shader_format_mask == MOSS_GPU_SHADERFORMAT_NONE;
}

const Moss_GPUBackend* Moss_GetMetalFallbackGPUBackend() {
    static const Moss_GPUBackend* backend = Moss_CreateNamedFallbackGPUBackend(MOSS_GPU_BACKEND_METAL, "Metal", MOSS_GPU_SHADERFORMAT_MSL | MOSS_GPU_SHADERFORMAT_METALLIB, supports_metal_shader_formats);
    return backend ? backend : Moss_GetFallbackGPUBackendTable();
}

} // namespace

const Moss_GPUBackend* Moss_GetMetalGPUBackend() {
    return g_metal_backend ? g_metal_backend : Moss_GetMetalFallbackGPUBackend();
}

void Moss_SetMetalGPUBackend(const Moss_GPUBackend* backend) {
    g_metal_backend = backend;
}

bool Moss_MetalGPUBackendIsRegistered() {
    return g_metal_backend != nullptr;
}
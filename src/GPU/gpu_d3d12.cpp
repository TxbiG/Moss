//                        MIT License
//
//                  Copyright (c) 2026 Toby

#include "gpu_backend.h"

namespace {

const Moss_GPUBackend* g_d3d12_backend = nullptr;

bool supports_d3d_shader_formats(Moss_GPUDevice*, uint32_t shader_format_mask) {
    return (shader_format_mask & (MOSS_GPU_SHADERFORMAT_DXIL | MOSS_GPU_SHADERFORMAT_DXBC)) != 0 || shader_format_mask == MOSS_GPU_SHADERFORMAT_NONE;
}

const Moss_GPUBackend* Moss_GetD3D12FallbackGPUBackend() {
    static const Moss_GPUBackend* backend = Moss_CreateNamedFallbackGPUBackend(MOSS_GPU_BACKEND_DIRECTX12, "DirectX 12", MOSS_GPU_SHADERFORMAT_DXIL | MOSS_GPU_SHADERFORMAT_DXBC, supports_d3d_shader_formats);
    return backend ? backend : Moss_GetFallbackGPUBackendTable();
}

} // namespace

const Moss_GPUBackend* Moss_GetD3D12GPUBackend() {
    return g_d3d12_backend ? g_d3d12_backend : Moss_GetD3D12FallbackGPUBackend();
}

void Moss_SetD3D12GPUBackend(const Moss_GPUBackend* backend) {
    g_d3d12_backend = backend;
}

bool Moss_D3D12GPUBackendIsRegistered() {
    return g_d3d12_backend != nullptr;
}
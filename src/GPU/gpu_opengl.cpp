//                        MIT License
//
//                  Copyright (c) 2026 Toby

#include "gpu_backend.h"



const Moss_GPUBackend* g_opengl_backend = nullptr;
const Moss_GPUBackend* g_opengles_backend = nullptr;

bool supports_glsl_shader_formats(Moss_GPUDevice*, uint32_t shader_format_mask) {
    return (shader_format_mask & MOSS_GPU_SHADERFORMAT_GLSL) != 0 || shader_format_mask == MOSS_GPU_SHADERFORMAT_NONE;
}

bool supports_gles_shader_formats(Moss_GPUDevice*, uint32_t shader_format_mask) {
    return (shader_format_mask & MOSS_GPU_SHADERFORMAT_GLSL_ES) != 0 || shader_format_mask == MOSS_GPU_SHADERFORMAT_NONE;
}

const Moss_GPUBackend* Moss_GetOpenGLFallbackGPUBackend() {
    static const Moss_GPUBackend* backend = Moss_CreateNamedFallbackGPUBackend(MOSS_GPU_BACKEND_OPENGL, "OpenGL", MOSS_GPU_SHADERFORMAT_GLSL, supports_glsl_shader_formats);
    return backend ? backend : Moss_GetFallbackGPUBackendTable();
}

const Moss_GPUBackend* Moss_GetOpenGLESFallbackGPUBackend() {
    static const Moss_GPUBackend* backend = Moss_CreateNamedFallbackGPUBackend(MOSS_GPU_BACKEND_OPENGLES, "OpenGL ES", MOSS_GPU_SHADERFORMAT_GLSL_ES, supports_gles_shader_formats);
    return backend ? backend : Moss_GetFallbackGPUBackendTable();
}

bool Moss_OpenGLGPUBackendIsRegistered() { return g_opengl_backend != nullptr; }
bool Moss_OpenGLESGPUBackendIsRegistered() { return g_opengles_backend != nullptr; }

bool Moss_VulkanGPUBackendIsRegistered();
bool Moss_D3D12GPUBackendIsRegistered();
bool Moss_MetalGPUBackendIsRegistered();

const Moss_GPUBackend* Moss_GetOpenGLGPUBackend() {
    return g_opengl_backend ? g_opengl_backend : Moss_GetOpenGLFallbackGPUBackend();
}

const Moss_GPUBackend* Moss_GetOpenGLESGPUBackend() {
    return g_opengles_backend ? g_opengles_backend : Moss_GetOpenGLESFallbackGPUBackend();
}

void Moss_SetOpenGLGPUBackend(const Moss_GPUBackend* backend) {
    g_opengl_backend = backend;
}

void Moss_SetOpenGLESGPUBackend(const Moss_GPUBackend* backend) {
    g_opengles_backend = backend;
}

bool Moss_GPUBackendHasRegisteredNativeTable(Moss_GPUBackendType type) {
    switch (type) {
        case MOSS_GPU_BACKEND_OPENGL: return Moss_OpenGLGPUBackendIsRegistered();
        case MOSS_GPU_BACKEND_OPENGLES: return Moss_OpenGLESGPUBackendIsRegistered();
        case MOSS_GPU_BACKEND_VULKAN: return Moss_VulkanGPUBackendIsRegistered();
        case MOSS_GPU_BACKEND_DIRECTX12: return Moss_D3D12GPUBackendIsRegistered();
        case MOSS_GPU_BACKEND_METAL: return Moss_MetalGPUBackendIsRegistered();
        default: return false;
    }
}
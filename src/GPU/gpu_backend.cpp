//                        MIT License
//
//                  Copyright (c) 2026 Toby

#include "gpu_backend.h"

#include <algorithm>
#include <cstring>
#include <new>
#include <string>
#include <vector>

#if defined(__APPLE__)
#include <TargetConditionals.h>
#endif

struct Moss_GPUTexture;

struct Moss_GPUDevice {
    Moss_GPUDeviceDesc desc{};
    Moss_GPUDeviceProperties properties{};
    uint32_t frames_in_flight = 2;
    bool validation_enabled = false;
    Moss_GPUTexture* fallback_swapchain_texture = nullptr;
};

struct Moss_GPUBuffer {
    Moss_GPUBufferDesc desc{};
    std::vector<uint8_t> bytes;
    EResourceState state = EResourceState::COMMON;
    std::string name;
};

struct Moss_GPUTexture {
    Moss_GPUTextureCreateInfo desc{};
    std::vector<uint8_t> bytes;
    EResourceState state = EResourceState::COMMON;
    std::string name;
    bool mipmaps_generated = false;
};

struct Moss_GPUTextureView {
    Moss_GPUTextureViewCreateInfo desc{};
};

struct Moss_GPUTransferBuffer {
    Moss_GPUTransferBufferCreateInfo desc{};
    std::vector<uint8_t> bytes;
};

struct Moss_GPUSampler {
    Moss_GPUSamplerCreateInfo desc{};
};

struct Moss_GPUShader {
    Moss_GPUShaderCreateInfo desc{};
    std::vector<uint8_t> bytecode;
};

struct Moss_ComputePipelineState {
    Moss_GPUComputePipelineCreateInfo desc{};
};

struct Moss_Framebuffer {
    Moss_GPUFramebufferCreateInfo desc{};
    std::vector<Moss_GPUFramebufferColorAttachment> color_attachments;
};

struct Moss_ResourceSetLayout {
    Moss_GPUResourceSetLayoutCreateInfo desc{};
    std::vector<Moss_GPUResourceSetLayoutBinding> bindings;
};

struct Moss_ResourceSet {
    Moss_GPUResourceSetCreateInfo desc{};
    std::vector<Moss_GPUResourceBinding> bindings;
};

struct Moss_GPUQueryPool {
    Moss_GPUQueryPoolCreateInfo desc{};
    std::vector<uint64_t> values;
};

struct Moss_Shader {
    Moss_ShaderDesc desc{};
    Moss_GPUShader* gpu_shader = nullptr;
};

struct Moss_PipelineState {
    const Moss_PipelineDesc* desc = nullptr;
};

struct Moss_GPUFence {
    bool signaled = true;
};

struct Moss_GPUCommandBuffer {
    ECommandQueue queue = ECommandQueue::GRAPHICS;
    bool recording = true;
    bool in_render_pass = false;
    bool in_compute_pass = false;
    bool in_copy_pass = false;
    uint32_t draw_count = 0;
    uint32_t dispatch_count = 0;
    uint32_t barrier_count = 0;
    Moss_PipelineState* pipeline = nullptr;
    Moss_ComputePipelineState* compute_pipeline = nullptr;
    Moss_GPUBuffer* index_buffer = nullptr;
    EGPUIndexType index_type = EGPUIndexType::UINT32;
    Moss_GPUViewport viewport{};
    Moss_Rect scissor{};
    std::vector<std::string> debug_stack;
};

namespace {

constexpr uint32_t MOSS_MAX_TRACKED_GPU_OBJECTS = 256;

struct BackendBinding {
    void* object = nullptr;
    const Moss_GPUBackend* backend = nullptr;
};

BackendBinding g_device_backends[MOSS_MAX_TRACKED_GPU_OBJECTS];
BackendBinding g_command_buffer_backends[MOSS_MAX_TRACKED_GPU_OBJECTS];

enum class BindlessResourceKind : uint8_t {
    Texture,
    Buffer
};

struct BindlessBinding {
    Moss_GPUDevice* device = nullptr;
    void* resource = nullptr;
    Moss_BindlessHandle handle = 0;
    BindlessResourceKind kind = BindlessResourceKind::Texture;
};

BindlessBinding g_bindless_bindings[MOSS_MAX_TRACKED_GPU_OBJECTS];
Moss_BindlessHandle g_next_bindless_handle = 1;

void register_binding(BackendBinding* bindings, void* object, const Moss_GPUBackend* backend) {
    if (!object || !backend) {
        return;
    }

    for (uint32_t i = 0; i < MOSS_MAX_TRACKED_GPU_OBJECTS; ++i) {
        if (bindings[i].object == object) {
            bindings[i].backend = backend;
            return;
        }
    }

    for (uint32_t i = 0; i < MOSS_MAX_TRACKED_GPU_OBJECTS; ++i) {
        if (!bindings[i].object) {
            bindings[i].object = object;
            bindings[i].backend = backend;
            return;
        }
    }
}

void unregister_binding(BackendBinding* bindings, void* object) {
    if (!object) {
        return;
    }

    for (uint32_t i = 0; i < MOSS_MAX_TRACKED_GPU_OBJECTS; ++i) {
        if (bindings[i].object == object) {
            bindings[i] = {};
            return;
        }
    }
}

void unregister_bindings_for_backend(BackendBinding* bindings, const Moss_GPUBackend* backend) {
    if (!backend) {
        return;
    }

    for (uint32_t i = 0; i < MOSS_MAX_TRACKED_GPU_OBJECTS; ++i) {
        if (bindings[i].backend == backend) {
            bindings[i] = {};
        }
    }
}

Moss_BindlessHandle allocate_bindless_handle() {
    for (uint32_t attempt = 0; attempt < 0xFFFFFFFEu; ++attempt) {
        if (g_next_bindless_handle == 0) {
            g_next_bindless_handle = 1;
        }

        Moss_BindlessHandle handle = g_next_bindless_handle++;
        bool in_use = false;
        for (uint32_t i = 0; i < MOSS_MAX_TRACKED_GPU_OBJECTS; ++i) {
            if (g_bindless_bindings[i].handle == handle) {
                in_use = true;
                break;
            }
        }
        if (!in_use) {
            return handle;
        }
    }

    return 0;
}

Moss_BindlessHandle register_bindless_resource(Moss_GPUDevice* device, void* resource, BindlessResourceKind kind) {
    if (!device || !resource) {
        return 0;
    }

    for (uint32_t i = 0; i < MOSS_MAX_TRACKED_GPU_OBJECTS; ++i) {
        if (g_bindless_bindings[i].device == device &&
            g_bindless_bindings[i].resource == resource &&
            g_bindless_bindings[i].kind == kind) {
            return g_bindless_bindings[i].handle;
        }
    }

    for (uint32_t i = 0; i < MOSS_MAX_TRACKED_GPU_OBJECTS; ++i) {
        if (g_bindless_bindings[i].handle == 0) {
            Moss_BindlessHandle handle = allocate_bindless_handle();
            if (handle == 0) {
                return 0;
            }
            g_bindless_bindings[i].device = device;
            g_bindless_bindings[i].resource = resource;
            g_bindless_bindings[i].kind = kind;
            g_bindless_bindings[i].handle = handle;
            return handle;
        }
    }

    return 0;
}

void unregister_bindless_resource(Moss_GPUDevice* device, Moss_BindlessHandle handle) {
    if (!device || handle == 0) {
        return;
    }

    for (uint32_t i = 0; i < MOSS_MAX_TRACKED_GPU_OBJECTS; ++i) {
        if (g_bindless_bindings[i].device == device && g_bindless_bindings[i].handle == handle) {
            g_bindless_bindings[i] = {};
            return;
        }
    }
}

void unregister_bindless_resources_for_device(Moss_GPUDevice* device) {
    if (!device) {
        return;
    }

    for (uint32_t i = 0; i < MOSS_MAX_TRACKED_GPU_OBJECTS; ++i) {
        if (g_bindless_bindings[i].device == device) {
            g_bindless_bindings[i] = {};
        }
    }
}

const Moss_GPUBackend* find_binding(const BackendBinding* bindings, void* object) {
    if (!object) {
        return nullptr;
    }

    for (uint32_t i = 0; i < MOSS_MAX_TRACKED_GPU_OBJECTS; ++i) {
        if (bindings[i].object == object) {
            return bindings[i].backend;
        }
    }

    return nullptr;
}

void register_command_buffer(Moss_GPUCommandBuffer* cmd, const Moss_GPUBackend* backend) {
    register_binding(g_command_buffer_backends, cmd, backend);
}

const Moss_GPUBackend* Moss_GetFallbackGPUBackend();

const Moss_GPUBackend* select_compiled_backend() {
#if defined(MOSS_USE_OPENGL) || defined(MOSS_GRAPHICS_OPENGL)
    return ::Moss_GetOpenGLGPUBackend();
#elif defined(MOSS_USE_OPENGLES) || defined(MOSS_GRAPHICS_OPENGLES)
    return ::Moss_GetOpenGLESGPUBackend();
#elif defined(MOSS_USE_VULKAN) || defined(MOSS_GRAPHICS_VULKAN)
    return ::Moss_GetVulkanGPUBackend();
#elif defined(MOSS_USE_DIRECTX12) || defined(MOSS_GRAPHICS_DIRECTX)
    return ::Moss_GetD3D12GPUBackend();
#elif defined(MOSS_USE_METAL) || defined(MOSS_GRAPHICS_METAL)
    return ::Moss_GetMetalGPUBackend();
#else
    return Moss_GetFallbackGPUBackend();
#endif
}

uint32_t texel_block_size(ETextureFormat format) {
    switch (format) {
        case ETextureFormat::R8:
        case ETextureFormat::R8_SNORM:
        case ETextureFormat::R8UI:
            return 1;
        case ETextureFormat::RG8:
        case ETextureFormat::RG8_SNORM:
        case ETextureFormat::R16F:
        case ETextureFormat::R16UI:
        case ETextureFormat::Depth16:
            return 2;
        case ETextureFormat::RGB8:
        case ETextureFormat::RGB8_SNORM:
        case ETextureFormat::SRGB8:
        case ETextureFormat::Depth24:
            return 3;
        case ETextureFormat::RGBA8:
        case ETextureFormat::RGBA8_SNORM:
        case ETextureFormat::R32F:
        case ETextureFormat::R32UI:
        case ETextureFormat::RG16F:
        case ETextureFormat::RG16UI:
        case ETextureFormat::RGBA8UI:
        case ETextureFormat::Depth32F:
        case ETextureFormat::Depth24Stencil8:
        case ETextureFormat::SRGBA8:
            return 4;
        case ETextureFormat::RGB16F:
            return 6;
        case ETextureFormat::RG32F:
        case ETextureFormat::RG32UI:
        case ETextureFormat::RGBA16F:
        case ETextureFormat::RGBA16UI:
        case ETextureFormat::Depth32FStencil8:
            return 8;
        case ETextureFormat::RGB32F:
            return 12;
        case ETextureFormat::RGBA32F:
        case ETextureFormat::RGBA32UI:
            return 16;
        case ETextureFormat::DXT1:
        case ETextureFormat::BC4:
            return 8;
        case ETextureFormat::DXT3:
        case ETextureFormat::DXT5:
        case ETextureFormat::BC5:
        case ETextureFormat::BC6H:
        case ETextureFormat::BC7:
            return 16;
    }

    return 0;
}

bool is_compressed_format(ETextureFormat format) {
    return format == ETextureFormat::DXT1 ||
           format == ETextureFormat::DXT3 ||
           format == ETextureFormat::DXT5 ||
           format == ETextureFormat::BC4 ||
           format == ETextureFormat::BC5 ||
           format == ETextureFormat::BC6H ||
           format == ETextureFormat::BC7;
}

Moss_GPUTextureCreateInfo to_gpu_texture_create_info(const Moss_TextureDesc* desc) {
    Moss_GPUTextureCreateInfo info{};
    if (!desc) {
        return info;
    }

    info.type = desc->type;
    info.format = desc->format;
    info.usage = desc->usage;
    info.width = desc->width;
    info.height = desc->height;
    info.depth = desc->depth;
    info.layers = desc->layers;
    info.mip_levels = desc->mip_levels;
    return info;
}

Moss_GPUSamplerCreateInfo to_gpu_sampler_create_info(const Moss_GPUSamplerDesc* desc) {
    Moss_GPUSamplerCreateInfo info{};
    if (!desc) {
        return info;
    }

    info.min_filter = desc->min_filter;
    info.mag_filter = desc->mag_filter;
    info.mip_filter = desc->mip_filter;
    info.address_u = desc->address_u;
    info.address_v = desc->address_v;
    info.address_w = desc->address_w;
    info.mip_lod_bias = desc->mip_lod_bias;
    info.max_anisotropy = desc->max_anisotropy;
    info.enable_anisotropy = desc->max_anisotropy > 1.0f;
    return info;
}


uint64_t fallback_texture_size(const Moss_GPUTextureCreateInfo& desc) {
    uint64_t total = 0;
    uint32_t w = std::max(1u, desc.width);
    uint32_t h = std::max(1u, desc.height);
    uint32_t d = std::max(1u, desc.depth);
    const uint32_t layers = std::max(1u, desc.layers);
    const uint32_t mip_count = std::max(1u, desc.mip_levels);
    const uint32_t block = std::max(1u, texel_block_size(desc.format));
    const bool compressed = is_compressed_format(desc.format);
    for (uint32_t mip = 0; mip < mip_count; ++mip) {
        total += compressed ? uint64_t((w + 3) / 4) * uint64_t((h + 3) / 4) * d * layers * block : uint64_t(w) * h * d * layers * block;
        w = std::max(1u, w >> 1);
        h = std::max(1u, h >> 1);
        d = std::max(1u, d >> 1);
    }
    return total;
}

uint64_t fallback_texture_mip_offset(const Moss_GPUTextureCreateInfo& desc, uint32_t target_mip) {
    uint64_t offset = 0;
    uint32_t w = std::max(1u, desc.width);
    uint32_t h = std::max(1u, desc.height);
    uint32_t d = std::max(1u, desc.depth);
    const uint32_t layers = std::max(1u, desc.layers);
    const uint32_t block = std::max(1u, texel_block_size(desc.format));
    const bool compressed = is_compressed_format(desc.format);
    for (uint32_t mip = 0; mip < target_mip; ++mip) {
        offset += compressed ? uint64_t((w + 3) / 4) * uint64_t((h + 3) / 4) * d * layers * block : uint64_t(w) * h * d * layers * block;
        w = std::max(1u, w >> 1);
        h = std::max(1u, h >> 1);
        d = std::max(1u, d >> 1);
    }
    return offset;
}

uint64_t fallback_texture_region_size(const Moss_GPUTextureCreateInfo& desc, const Moss_GPUTextureRegion& region) {
    const uint32_t block = std::max(1u, texel_block_size(desc.format));
    if (is_compressed_format(desc.format)) {
        return uint64_t((region.width + 3) / 4) * uint64_t((region.height + 3) / 4) * std::max(1u, region.depth) * std::max(1u, region.layer_count) * block;
    }
    return uint64_t(std::max(1u, region.width)) * std::max(1u, region.height) * std::max(1u, region.depth) * std::max(1u, region.layer_count) * block;
}

Moss_GPUDevice* fallback_create_device(const Moss_GPUDeviceDesc* desc, const Moss_GPUDeviceProperties*) {
    Moss_GPUDevice* device = new (std::nothrow) Moss_GPUDevice();
    if (!device) return nullptr;
    if (desc) device->desc = *desc;
    device->validation_enabled = desc ? desc->enable_validation : false;
    device->properties.driver_name = "Moss fallback GPU";
    device->properties.device_name = "Moss fallback GPU";
    device->properties.supports_bindless = true;
    device->properties.supports_debug_markers = true;
    device->properties.max_texture_dimension_2d = 16384;
    device->properties.max_texture_array_layers = 2048;
    device->properties.max_frames_in_flight = 3;
    return device;
}

void fallback_destroy_device(Moss_GPUDevice* device) {
    if (!device) return;
    delete device->fallback_swapchain_texture;
    delete device;
}
const char* fallback_get_device_driver(Moss_GPUDevice*) { return "Moss fallback GPU"; }
const Moss_GPUDeviceProperties* fallback_get_device_properties(Moss_GPUDevice* device) { return device ? &device->properties : nullptr; }
bool fallback_true_device(Moss_GPUDevice*) { return true; }
bool fallback_supports_shader_formats(Moss_GPUDevice*, uint32_t shader_format_mask) { return (shader_format_mask & MOSS_GPU_SHADERFORMAT_GLSL) != 0 || shader_format_mask == MOSS_GPU_SHADERFORMAT_NONE; }
bool supports_glsl_shader_formats(Moss_GPUDevice*, uint32_t shader_format_mask) { return (shader_format_mask & MOSS_GPU_SHADERFORMAT_GLSL) != 0 || shader_format_mask == MOSS_GPU_SHADERFORMAT_NONE; }
bool supports_gles_shader_formats(Moss_GPUDevice*, uint32_t shader_format_mask) { return (shader_format_mask & MOSS_GPU_SHADERFORMAT_GLSL_ES) != 0 || shader_format_mask == MOSS_GPU_SHADERFORMAT_NONE; }
bool supports_spirv_shader_formats(Moss_GPUDevice*, uint32_t shader_format_mask) { return (shader_format_mask & MOSS_GPU_SHADERFORMAT_SPIRV) != 0 || shader_format_mask == MOSS_GPU_SHADERFORMAT_NONE; }
bool supports_d3d_shader_formats(Moss_GPUDevice*, uint32_t shader_format_mask) { return (shader_format_mask & (MOSS_GPU_SHADERFORMAT_DXIL | MOSS_GPU_SHADERFORMAT_DXBC)) != 0 || shader_format_mask == MOSS_GPU_SHADERFORMAT_NONE; }
bool supports_metal_shader_formats(Moss_GPUDevice*, uint32_t shader_format_mask) { return (shader_format_mask & (MOSS_GPU_SHADERFORMAT_MSL | MOSS_GPU_SHADERFORMAT_METALLIB)) != 0 || shader_format_mask == MOSS_GPU_SHADERFORMAT_NONE; }
bool fallback_texture_supports_format(Moss_GPUDevice*, ETextureFormat, ETextureUsage) { return true; }
bool fallback_texture_supports_sample_count(Moss_GPUDevice*, ETextureFormat, uint32_t sample_count) { return sample_count == 1; }
int fallback_supports_async_compute(Moss_GPUDevice*) { return 0; }
void fallback_wait_idle(Moss_GPUDevice*) {}
bool fallback_claim_window(Moss_GPUDevice*, Moss_Window*) { return true; }
void fallback_release_window(Moss_GPUDevice*, Moss_Window*) {}
bool fallback_window_supports_present_mode(Moss_Window*, EGPUPresentMode present_mode) { return present_mode == EGPUPresentMode::VSYNC || present_mode == EGPUPresentMode::IMMEDIATE; }
bool fallback_window_supports_swapchain_composition(Moss_Window*) { return false; }
Moss_GPUTexture* fallback_create_texture(Moss_GPUDevice*, const Moss_GPUTextureCreateInfo* info);
bool fallback_ensure_swapchain_texture(Moss_GPUDevice* device) {
    if (!device) return false;
    const uint32_t width = std::max(1u, device->desc.backbuffer_width);
    const uint32_t height = std::max(1u, device->desc.backbuffer_height);
    if (device->fallback_swapchain_texture &&
        device->fallback_swapchain_texture->desc.width == width &&
        device->fallback_swapchain_texture->desc.height == height) {
        return true;
    }

    delete device->fallback_swapchain_texture;
    device->fallback_swapchain_texture = nullptr;

    Moss_GPUTextureCreateInfo info{};
    info.type = ETextureType::TEXTURE_2D;
    info.format = ETextureFormat::RGBA8;
    info.usage = ETextureUsage::COLOR_TARGET | ETextureUsage::TRANSFER_SRC | ETextureUsage::TRANSFER_DST;
    info.width = width;
    info.height = height;
    info.depth = 1;
    info.layers = 1;
    info.mip_levels = 1;
    device->fallback_swapchain_texture = fallback_create_texture(device, &info);
    if (device->fallback_swapchain_texture) {
        device->fallback_swapchain_texture->state = EResourceState::PRESENT;
    }
    return device->fallback_swapchain_texture != nullptr;
}
bool fallback_acquire_swapchain_texture(Moss_GPUDevice* device, Moss_Window*, Moss_GPUTexture** texture) {
    if (texture) *texture = nullptr;
    if (!fallback_ensure_swapchain_texture(device)) return false;
    device->fallback_swapchain_texture->state = EResourceState::RENDER_TARGET;
    if (texture) *texture = device->fallback_swapchain_texture;
    return true;
}
EGPUSwapchainResult fallback_acquire_swapchain_texture_status(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture** texture) {
    return fallback_acquire_swapchain_texture(device, window, texture) ? EGPUSwapchainResult::SUCCESS : EGPUSwapchainResult::ERROR;
}
bool fallback_wait_and_acquire_swapchain_texture(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture** texture, uint64_t) {
    return fallback_acquire_swapchain_texture(device, window, texture);
}
EGPUSwapchainResult fallback_wait_and_acquire_swapchain_texture_status(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture** texture, uint64_t timeout_ns) {
    (void)timeout_ns;
    return fallback_acquire_swapchain_texture_status(device, window, texture);
}
bool fallback_wait_for_swapchain(Moss_GPUDevice* device, Moss_Window*, uint64_t) { return device != nullptr; }
bool fallback_set_swapchain_parameters(Moss_GPUDevice* device, Moss_Window*, uint32_t width, uint32_t height, EGPUPresentMode present_mode) { if (!device) return false; device->desc.backbuffer_width = width; device->desc.backbuffer_height = height; device->desc.present_mode = present_mode; return true; }
EGPUSwapchainResult fallback_resize_swapchain(Moss_GPUDevice* device, Moss_Window*, uint32_t width, uint32_t height) {
    if (!device || width == 0 || height == 0) return EGPUSwapchainResult::ERROR;
    device->desc.backbuffer_width = width;
    device->desc.backbuffer_height = height;
    delete device->fallback_swapchain_texture;
    device->fallback_swapchain_texture = nullptr;
    return fallback_ensure_swapchain_texture(device) ? EGPUSwapchainResult::SUCCESS : EGPUSwapchainResult::ERROR;
}
ETextureFormat fallback_get_swapchain_texture_format(Moss_GPUDevice*, Moss_Window*) { return ETextureFormat::RGBA8; }
bool fallback_present_swapchain(Moss_GPUDevice* device, Moss_Window*, Moss_GPUTexture* texture) {
    if (!device || !texture || texture != device->fallback_swapchain_texture) return false;
    texture->state = EResourceState::PRESENT;
    return true;
}
EGPUSwapchainResult fallback_present_swapchain_status(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture* texture) {
    return fallback_present_swapchain(device, window, texture) ? EGPUSwapchainResult::SUCCESS : EGPUSwapchainResult::ERROR;
}
Moss_GPUCommandBuffer* fallback_begin_command_buffer(Moss_GPUDevice*, ECommandQueue queue) { Moss_GPUCommandBuffer* cmd = new (std::nothrow) Moss_GPUCommandBuffer(); if (cmd) cmd->queue = queue; return cmd; }
Moss_GPUCommandBuffer* fallback_acquire_command_buffer(Moss_GPUDevice* device) { return fallback_begin_command_buffer(device, ECommandQueue::GRAPHICS); }
void fallback_end_command_buffer(Moss_GPUCommandBuffer* cmd) { if (cmd) cmd->recording = false; }
void fallback_submit_command_buffer(Moss_GPUDevice*, Moss_GPUCommandBuffer* cmd) { delete cmd; }
Moss_GPUFence* fallback_submit_command_buffer_and_acquire_fence(Moss_GPUDevice*, Moss_GPUCommandBuffer* cmd) { delete cmd; return new (std::nothrow) Moss_GPUFence(); }
void fallback_cancel_command_buffer(Moss_GPUCommandBuffer* cmd) { delete cmd; }
void fallback_begin_render_pass(Moss_GPUCommandBuffer* cmd, Moss_Framebuffer*) { if (cmd) cmd->in_render_pass = true; }
void fallback_end_render_pass(Moss_GPUCommandBuffer* cmd) { if (cmd) cmd->in_render_pass = false; }
void fallback_begin_compute_pass(Moss_GPUCommandBuffer* cmd) { if (cmd) cmd->in_compute_pass = true; }
void fallback_end_compute_pass(Moss_GPUCommandBuffer* cmd) { if (cmd) cmd->in_compute_pass = false; }
void fallback_begin_copy_pass(Moss_GPUCommandBuffer* cmd) { if (cmd) cmd->in_copy_pass = true; }
void fallback_end_copy_pass(Moss_GPUCommandBuffer* cmd) { if (cmd) cmd->in_copy_pass = false; }
Moss_GPUBuffer* fallback_create_buffer(Moss_GPUDevice*, const Moss_GPUBufferDesc* desc) { Moss_GPUBuffer* buffer = new (std::nothrow) Moss_GPUBuffer(); if (!buffer) return nullptr; if (desc) buffer->desc = *desc; buffer->bytes.resize(static_cast<size_t>(buffer->desc.size)); return buffer; }
void fallback_release_buffer(Moss_GPUDevice*, Moss_GPUBuffer* buffer) { delete buffer; }
void fallback_upload_to_buffer(Moss_GPUDevice*, Moss_GPUBuffer* dst, const void* src, uint64_t size, uint64_t offset) { if (!dst || !src || size == 0 || offset > dst->bytes.size()) return; const uint64_t copy_size = std::min<uint64_t>(size, dst->bytes.size() - offset); std::memcpy(dst->bytes.data() + offset, src, static_cast<size_t>(copy_size)); }
void fallback_download_from_buffer(Moss_GPUDevice*, Moss_GPUBuffer* src, void* dst, uint64_t size, uint64_t offset) { if (!src || !dst || size == 0 || offset > src->bytes.size()) return; const uint64_t copy_size = std::min<uint64_t>(size, src->bytes.size() - offset); std::memcpy(dst, src->bytes.data() + offset, static_cast<size_t>(copy_size)); }
void* fallback_map_buffer(Moss_GPUDevice*, Moss_GPUBuffer* buffer) { return buffer && !buffer->bytes.empty() ? buffer->bytes.data() : nullptr; }
void fallback_unmap_buffer(Moss_GPUDevice*, Moss_GPUBuffer*) {}
void fallback_set_buffer_name(Moss_GPUDevice*, Moss_GPUBuffer* buffer, const char* name) { if (buffer) buffer->name = name ? name : ""; }
Moss_GPUTexture* fallback_create_texture(Moss_GPUDevice*, const Moss_GPUTextureCreateInfo* info) { Moss_GPUTexture* texture = new (std::nothrow) Moss_GPUTexture(); if (!texture) return nullptr; if (info) texture->desc = *info; texture->desc.width = std::max(1u, texture->desc.width); texture->desc.height = std::max(1u, texture->desc.height); texture->desc.depth = std::max(1u, texture->desc.depth); texture->desc.layers = std::max(1u, texture->desc.layers); texture->desc.mip_levels = std::max(1u, texture->desc.mip_levels); texture->bytes.resize(static_cast<size_t>(fallback_texture_size(texture->desc))); return texture; }
void fallback_release_texture(Moss_GPUDevice*, Moss_GPUTexture* texture) { delete texture; }
Moss_GPUTextureView* fallback_create_texture_view(Moss_GPUDevice*, const Moss_GPUTextureViewCreateInfo* info) { Moss_GPUTextureView* view = new (std::nothrow) Moss_GPUTextureView(); if (view && info) view->desc = *info; return view; }
void fallback_release_texture_view(Moss_GPUDevice*, Moss_GPUTextureView* view) { delete view; }
void fallback_upload_to_texture(Moss_GPUDevice*, Moss_GPUTexture* dst, const Moss_GPUTextureTransferInfo* info) { if (!dst || !info || !info->data) return; const uint64_t offset = fallback_texture_mip_offset(dst->desc, info->region.mip_level); if (offset >= dst->bytes.size()) return; const uint64_t size = std::min<uint64_t>(fallback_texture_region_size(dst->desc, info->region), dst->bytes.size() - offset); std::memcpy(dst->bytes.data() + offset, info->data, static_cast<size_t>(size)); }
void fallback_download_from_texture(Moss_GPUDevice*, Moss_GPUTexture* src, const Moss_GPUTextureRegion* region, void* dst, uint32_t, uint32_t) { if (!src || !dst) return; Moss_GPUTextureRegion whole{}; whole.width = src->desc.width; whole.height = src->desc.height; whole.depth = src->desc.depth; const Moss_GPUTextureRegion& r = region ? *region : whole; const uint64_t offset = fallback_texture_mip_offset(src->desc, r.mip_level); if (offset >= src->bytes.size()) return; const uint64_t size = std::min<uint64_t>(fallback_texture_region_size(src->desc, r), src->bytes.size() - offset); std::memcpy(dst, src->bytes.data() + offset, static_cast<size_t>(size)); }
void fallback_generate_mipmaps(Moss_GPUCommandBuffer*, Moss_GPUTexture* texture) { if (texture) texture->mipmaps_generated = true; }
void fallback_set_texture_name(Moss_GPUDevice*, Moss_GPUTexture* texture, const char* name) { if (texture) texture->name = name ? name : ""; }
Moss_GPUTransferBuffer* fallback_create_transfer_buffer(Moss_GPUDevice*, const Moss_GPUTransferBufferCreateInfo* info) { Moss_GPUTransferBuffer* buffer = new (std::nothrow) Moss_GPUTransferBuffer(); if (!buffer) return nullptr; if (info) buffer->desc = *info; buffer->bytes.resize(static_cast<size_t>(buffer->desc.size)); return buffer; }
void fallback_release_transfer_buffer(Moss_GPUDevice*, Moss_GPUTransferBuffer* buffer) { delete buffer; }
void* fallback_map_transfer_buffer(Moss_GPUDevice*, Moss_GPUTransferBuffer* buffer) { return buffer && !buffer->bytes.empty() ? buffer->bytes.data() : nullptr; }
void fallback_unmap_transfer_buffer(Moss_GPUDevice*, Moss_GPUTransferBuffer*) {}
Moss_GPUSampler* fallback_create_sampler(Moss_GPUDevice*, const Moss_GPUSamplerCreateInfo* info) { Moss_GPUSampler* sampler = new (std::nothrow) Moss_GPUSampler(); if (sampler && info) sampler->desc = *info; return sampler; }
void fallback_release_sampler(Moss_GPUDevice*, Moss_GPUSampler* sampler) { delete sampler; }
Moss_GPUShader* fallback_create_shader(Moss_GPUDevice*, const Moss_GPUShaderCreateInfo* info) { Moss_GPUShader* shader = new (std::nothrow) Moss_GPUShader(); if (!shader) return nullptr; if (info) { shader->desc = *info; if (info->bytecode && info->bytecode_size) { const uint8_t* bytes = static_cast<const uint8_t*>(info->bytecode); shader->bytecode.assign(bytes, bytes + info->bytecode_size); shader->desc.bytecode = shader->bytecode.data(); } } return shader; }
void fallback_release_shader(Moss_GPUDevice*, Moss_GPUShader* shader) { delete shader; }
Moss_ComputePipelineState* fallback_create_compute_pipeline(Moss_GPUDevice*, const Moss_GPUComputePipelineCreateInfo* info) { Moss_ComputePipelineState* pipeline = new (std::nothrow) Moss_ComputePipelineState(); if (pipeline && info) pipeline->desc = *info; return pipeline; }
void fallback_release_compute_pipeline(Moss_GPUDevice*, Moss_ComputePipelineState* pipeline) { delete pipeline; }
Moss_Framebuffer* fallback_create_framebuffer(Moss_GPUDevice*, const Moss_GPUFramebufferCreateInfo* info) { Moss_Framebuffer* framebuffer = new (std::nothrow) Moss_Framebuffer(); if (!framebuffer) return nullptr; if (info) { framebuffer->desc = *info; if (info->color_attachments && info->color_attachment_count) framebuffer->color_attachments.assign(info->color_attachments, info->color_attachments + info->color_attachment_count); framebuffer->desc.color_attachments = framebuffer->color_attachments.data(); } return framebuffer; }
void fallback_release_framebuffer(Moss_GPUDevice*, Moss_Framebuffer* framebuffer) { delete framebuffer; }
Moss_ResourceSetLayout* fallback_create_resource_set_layout(Moss_GPUDevice*, const Moss_GPUResourceSetLayoutCreateInfo* info) { Moss_ResourceSetLayout* layout = new (std::nothrow) Moss_ResourceSetLayout(); if (!layout) return nullptr; if (info) { layout->desc = *info; if (info->bindings && info->binding_count) layout->bindings.assign(info->bindings, info->bindings + info->binding_count); layout->desc.bindings = layout->bindings.data(); } return layout; }
void fallback_release_resource_set_layout(Moss_GPUDevice*, Moss_ResourceSetLayout* layout) { delete layout; }
Moss_ResourceSet* fallback_create_resource_set(Moss_GPUDevice*, const Moss_GPUResourceSetCreateInfo* info) { Moss_ResourceSet* set = new (std::nothrow) Moss_ResourceSet(); if (!set) return nullptr; if (info) { set->desc = *info; if (info->bindings && info->binding_count) set->bindings.assign(info->bindings, info->bindings + info->binding_count); set->desc.bindings = set->bindings.data(); } return set; }
void fallback_release_resource_set(Moss_GPUDevice*, Moss_ResourceSet* set) { delete set; }
void fallback_update_resource_set(Moss_GPUDevice*, Moss_ResourceSet* set, const Moss_GPUResourceBinding* bindings, uint32_t count) { if (!set) return; set->bindings.clear(); if (bindings && count) set->bindings.assign(bindings, bindings + count); set->desc.bindings = set->bindings.data(); set->desc.binding_count = static_cast<uint32_t>(set->bindings.size()); }
Moss_GPUQueryPool* fallback_create_query_pool(Moss_GPUDevice*, const Moss_GPUQueryPoolCreateInfo* info) { Moss_GPUQueryPool* pool = new (std::nothrow) Moss_GPUQueryPool(); if (!pool) return nullptr; if (info) pool->desc = *info; pool->values.resize(std::max(1u, pool->desc.query_count)); return pool; }
void fallback_release_query_pool(Moss_GPUDevice*, Moss_GPUQueryPool* pool) { delete pool; }
bool fallback_get_query_results(Moss_GPUDevice*, Moss_GPUQueryPool* pool, uint32_t first, uint32_t count, void* data, uint64_t data_size, uint64_t stride, EGPUQueryResultFlags) { if (!pool || !data || first + count > pool->values.size()) return false; const uint64_t step = stride ? stride : sizeof(uint64_t); if (data_size < step * count) return false; for (uint32_t i = 0; i < count; ++i) std::memcpy(static_cast<uint8_t*>(data) + step * i, &pool->values[first + i], sizeof(uint64_t)); return true; }
Moss_Shader* fallback_create_shader_from_desc(Moss_GPUDevice*, const Moss_ShaderDesc* desc) { Moss_Shader* shader = new (std::nothrow) Moss_Shader(); if (shader && desc) shader->desc = *desc; return shader; }
void fallback_destroy_shader(Moss_GPUDevice*, Moss_Shader* shader) { delete shader; }
Moss_PipelineState* fallback_create_pipeline(Moss_GPUDevice*, const Moss_PipelineDesc* desc) { Moss_PipelineState* pipeline = new (std::nothrow) Moss_PipelineState(); if (pipeline) pipeline->desc = desc; return pipeline; }
void fallback_destroy_pipeline(Moss_GPUDevice*, Moss_PipelineState* pipeline) { delete pipeline; }
void fallback_copy_buffer_to_buffer(Moss_GPUCommandBuffer*, Moss_GPUBuffer* src, Moss_GPUBuffer* dst, const Moss_GPUBufferRegion* src_region, const Moss_GPUBufferRegion* dst_region) { if (!src || !dst) return; const uint64_t src_offset = src_region ? src_region->offset : 0; const uint64_t dst_offset = dst_region ? dst_region->offset : 0; const uint64_t requested = src_region ? src_region->size : src->bytes.size(); if (src_offset > src->bytes.size() || dst_offset > dst->bytes.size()) return; const uint64_t size = std::min<uint64_t>(requested, std::min<uint64_t>(src->bytes.size() - src_offset, dst->bytes.size() - dst_offset)); std::memmove(dst->bytes.data() + dst_offset, src->bytes.data() + src_offset, static_cast<size_t>(size)); }
void fallback_copy_buffer_to_texture(Moss_GPUCommandBuffer*, Moss_GPUBuffer* src, Moss_GPUTexture* dst, const Moss_GPUBufferRegion* src_region, const Moss_GPUTextureRegion* dst_region) { if (!src || !dst) return; const uint64_t src_offset = src_region ? src_region->offset : 0; if (src_offset >= src->bytes.size()) return; Moss_GPUTextureTransferInfo info{}; info.data = src->bytes.data() + src_offset; if (dst_region) info.region = *dst_region; else { info.region.width = dst->desc.width; info.region.height = dst->desc.height; info.region.depth = dst->desc.depth; } fallback_upload_to_texture(nullptr, dst, &info); }
void fallback_copy_texture_to_buffer(Moss_GPUCommandBuffer*, Moss_GPUTexture* src, Moss_GPUBuffer* dst, const Moss_GPUTextureRegion* src_region, const Moss_GPUBufferRegion* dst_region) { if (!src || !dst) return; const uint64_t dst_offset = dst_region ? dst_region->offset : 0; if (dst_offset >= dst->bytes.size()) return; Moss_GPUTextureRegion r{}; if (src_region) r = *src_region; else { r.width = src->desc.width; r.height = src->desc.height; r.depth = src->desc.depth; } std::vector<uint8_t> scratch(static_cast<size_t>(fallback_texture_region_size(src->desc, r))); fallback_download_from_texture(nullptr, src, &r, scratch.data(), 0, 0); const uint64_t copy_size = std::min<uint64_t>(scratch.size(), dst->bytes.size() - dst_offset); std::memcpy(dst->bytes.data() + dst_offset, scratch.data(), static_cast<size_t>(copy_size)); }
void fallback_copy_texture_to_texture(Moss_GPUCommandBuffer*, Moss_GPUTexture* src, Moss_GPUTexture* dst, const Moss_GPUTextureRegion* src_region, const Moss_GPUTextureRegion* dst_region) { if (!src || !dst) return; Moss_GPUTextureRegion r{}; if (src_region) r = *src_region; else { r.width = src->desc.width; r.height = src->desc.height; r.depth = src->desc.depth; } std::vector<uint8_t> scratch(static_cast<size_t>(fallback_texture_region_size(src->desc, r))); fallback_download_from_texture(nullptr, src, &r, scratch.data(), 0, 0); Moss_GPUTextureTransferInfo info{}; info.data = scratch.data(); if (dst_region) info.region = *dst_region; else { info.region.width = dst->desc.width; info.region.height = dst->desc.height; info.region.depth = dst->desc.depth; } fallback_upload_to_texture(nullptr, dst, &info); }
void fallback_blit_texture(Moss_GPUCommandBuffer*, Moss_Texture* src, Moss_Texture* dst, const Moss_GPUBlitRegion*) { if (src && dst) reinterpret_cast<Moss_GPUTexture*>(dst)->mipmaps_generated = reinterpret_cast<Moss_GPUTexture*>(src)->mipmaps_generated; }
void fallback_barrier_resources(Moss_GPUCommandBuffer* cmd, const Moss_GPUBarrierInfo* info) { if (cmd && info) cmd->barrier_count += info->buffer_barrier_count + info->texture_barrier_count; }
void fallback_transition_buffer(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* buffer, EResourceState, EResourceState state) { if (buffer) buffer->state = state; if (cmd) ++cmd->barrier_count; }
void fallback_transition_texture(Moss_GPUCommandBuffer* cmd, Moss_GPUTexture* texture, EResourceState, EResourceState state) { if (texture) texture->state = state; if (cmd) ++cmd->barrier_count; }
void fallback_bind_pipeline(Moss_GPUCommandBuffer* cmd, Moss_PipelineState* pipeline) { if (cmd) cmd->pipeline = pipeline; }
void fallback_bind_compute_pipeline(Moss_GPUCommandBuffer* cmd, Moss_ComputePipelineState* pipeline) { if (cmd) cmd->compute_pipeline = pipeline; }
void fallback_bind_resource_set(Moss_GPUCommandBuffer*, uint32_t, Moss_ResourceSet*) {}
void fallback_bind_vertex_buffers(Moss_GPUCommandBuffer*, uint32_t, const Moss_GPUBufferBinding*, uint32_t) {}
void fallback_bind_index_buffer(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* buffer, uint64_t, EGPUIndexType type) { if (cmd) { cmd->index_buffer = buffer; cmd->index_type = type; } }
void fallback_bind_samplers(Moss_GPUCommandBuffer*, EShaderStage, uint32_t, Moss_GPUSampler* const*, uint32_t) {}
void fallback_bind_textures(Moss_GPUCommandBuffer*, EShaderStage, uint32_t, Moss_GPUTexture* const*, uint32_t) {}
void fallback_bind_uniform_buffers(Moss_GPUCommandBuffer*, EShaderStage, uint32_t, const Moss_GPUBufferBinding*, uint32_t) {}
void fallback_bind_storage_buffers(Moss_GPUCommandBuffer*, EShaderStage, uint32_t, const Moss_GPUStorageBufferReadWriteBinding*, uint32_t) {}
void fallback_bind_storage_textures(Moss_GPUCommandBuffer*, EShaderStage, uint32_t, const Moss_GPUStorageTextureReadWriteBinding*, uint32_t) {}
void fallback_draw(Moss_GPUCommandBuffer* cmd, uint32_t vertex_count, uint32_t instance_count, uint32_t, uint32_t) { if (cmd && vertex_count && instance_count) ++cmd->draw_count; }
void fallback_draw_indexed(Moss_GPUCommandBuffer* cmd, uint32_t index_count, uint32_t instance_count, uint32_t, int32_t, uint32_t) { if (cmd && index_count && instance_count) ++cmd->draw_count; }
void fallback_draw_indirect(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer*, uint64_t, uint32_t draw_count, uint32_t) { if (cmd) cmd->draw_count += draw_count; }
void fallback_draw_indexed_indirect(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer*, uint64_t, uint32_t draw_count, uint32_t) { if (cmd) cmd->draw_count += draw_count; }
void fallback_dispatch(Moss_GPUCommandBuffer* cmd, uint32_t x, uint32_t y, uint32_t z) { if (cmd && x && y && z) ++cmd->dispatch_count; }
void fallback_dispatch_indirect(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer*, uint64_t) { if (cmd) ++cmd->dispatch_count; }
void fallback_reset_query_pool(Moss_GPUCommandBuffer*, Moss_GPUQueryPool* pool, uint32_t first, uint32_t count) { if (!pool || first >= pool->values.size()) return; for (uint32_t i = 0; i < count && first + i < pool->values.size(); ++i) pool->values[first + i] = 0; }
void fallback_write_timestamp(Moss_GPUCommandBuffer*, Moss_GPUQueryPool* pool, uint32_t index) { if (pool && index < pool->values.size()) pool->values[index] = 1; }
void fallback_begin_query(Moss_GPUCommandBuffer*, Moss_GPUQueryPool* pool, uint32_t index) { if (pool && index < pool->values.size()) pool->values[index] = 1; }
void fallback_end_query(Moss_GPUCommandBuffer*, Moss_GPUQueryPool* pool, uint32_t index) { if (pool && index < pool->values.size()) ++pool->values[index]; }
void fallback_push_uniform_data(Moss_GPUCommandBuffer*, EShaderStage, uint32_t, const void*, uint32_t) {}
void fallback_set_viewport(Moss_GPUCommandBuffer* cmd, const Moss_GPUViewport* viewport) { if (cmd && viewport) cmd->viewport = *viewport; }
void fallback_set_scissor(Moss_GPUCommandBuffer* cmd, const Moss_Rect* scissor) { if (cmd && scissor) cmd->scissor = *scissor; }
void fallback_set_blend_constants(Moss_GPUCommandBuffer*, float, float, float, float) {}
void fallback_set_stencil_reference(Moss_GPUCommandBuffer*, uint32_t) {}
void fallback_clear(Moss_GPUCommandBuffer*, float, float, float, float) {}
void fallback_push_debug_group(Moss_GPUCommandBuffer* cmd, const char* label) { if (cmd) cmd->debug_stack.emplace_back(label ? label : ""); }
void fallback_pop_debug_group(Moss_GPUCommandBuffer* cmd) { if (cmd && !cmd->debug_stack.empty()) cmd->debug_stack.pop_back(); }
void fallback_insert_debug_label(Moss_GPUCommandBuffer*, const char*) {}
bool fallback_query_fence(Moss_GPUDevice*, Moss_GPUFence* fence) { return fence ? fence->signaled : false; }
void fallback_release_fence(Moss_GPUDevice*, Moss_GPUFence* fence) { delete fence; }
void fallback_wait_for_fences(Moss_GPUDevice*, Moss_GPUFence* const* fences, uint32_t count, bool, uint64_t) { if (!fences) return; for (uint32_t i = 0; i < count; ++i) if (fences[i]) fences[i]->signaled = true; }

const Moss_GPUBackend* Moss_GetFallbackGPUBackend() {
    static Moss_GPUBackend backend{};
    static bool initialized = false;
    if (!initialized) {
        backend.name = "fallback";
        backend.type = MOSS_GPU_BACKEND_FALLBACK;
        backend.shader_formats = MOSS_GPU_SHADERFORMAT_GLSL;
        backend.flags = MOSS_GPU_BACKEND_FLAG_FALLBACK_FUNCTION_TABLE | MOSS_GPU_BACKEND_FLAG_COMPUTE | MOSS_GPU_BACKEND_FLAG_BINDLESS | MOSS_GPU_BACKEND_FLAG_DEBUG_MARKERS;
        backend.create_device = fallback_create_device;
        backend.destroy_device = fallback_destroy_device;
        backend.get_device_driver = fallback_get_device_driver;
        backend.get_device_properties = fallback_get_device_properties;
        backend.supports_properties = fallback_true_device;
        backend.supports_shader_formats = fallback_supports_shader_formats;
        backend.texture_supports_format = fallback_texture_supports_format;
        backend.texture_supports_sample_count = fallback_texture_supports_sample_count;
        backend.supports_async_compute = fallback_supports_async_compute;
        backend.wait_idle = fallback_wait_idle;
        backend.claim_window = fallback_claim_window;
        backend.release_window = fallback_release_window;
        backend.window_supports_present_mode = fallback_window_supports_present_mode;
        backend.window_supports_swapchain_composition = fallback_window_supports_swapchain_composition;
        backend.acquire_swapchain_texture = fallback_acquire_swapchain_texture;
        backend.acquire_swapchain_texture_status = fallback_acquire_swapchain_texture_status;
        backend.wait_and_acquire_swapchain_texture = fallback_wait_and_acquire_swapchain_texture;
        backend.wait_and_acquire_swapchain_texture_status = fallback_wait_and_acquire_swapchain_texture_status;
        backend.wait_for_swapchain = fallback_wait_for_swapchain;
        backend.set_swapchain_parameters = fallback_set_swapchain_parameters;
        backend.resize_swapchain = fallback_resize_swapchain;
        backend.get_swapchain_texture_format = fallback_get_swapchain_texture_format;
        backend.present_swapchain = fallback_present_swapchain;
        backend.present_swapchain_status = fallback_present_swapchain_status;
        backend.acquire_command_buffer = fallback_acquire_command_buffer;
        backend.begin_command_buffer = fallback_begin_command_buffer;
        backend.end_command_buffer = fallback_end_command_buffer;
        backend.submit_command_buffer = fallback_submit_command_buffer;
        backend.submit_command_buffer_and_acquire_fence = fallback_submit_command_buffer_and_acquire_fence;
        backend.cancel_command_buffer = fallback_cancel_command_buffer;
        backend.begin_render_pass = fallback_begin_render_pass;
        backend.end_render_pass = fallback_end_render_pass;
        backend.begin_compute_pass = fallback_begin_compute_pass;
        backend.end_compute_pass = fallback_end_compute_pass;
        backend.begin_copy_pass = fallback_begin_copy_pass;
        backend.end_copy_pass = fallback_end_copy_pass;
        backend.create_buffer = fallback_create_buffer;
        backend.release_buffer = fallback_release_buffer;
        backend.upload_to_buffer = fallback_upload_to_buffer;
        backend.download_from_buffer = fallback_download_from_buffer;
        backend.map_buffer = fallback_map_buffer;
        backend.unmap_buffer = fallback_unmap_buffer;
        backend.set_buffer_name = fallback_set_buffer_name;
        backend.create_texture = fallback_create_texture;
        backend.release_texture = fallback_release_texture;
        backend.create_texture_view = fallback_create_texture_view;
        backend.release_texture_view = fallback_release_texture_view;
        backend.upload_to_texture = fallback_upload_to_texture;
        backend.download_from_texture = fallback_download_from_texture;
        backend.generate_mipmaps = fallback_generate_mipmaps;
        backend.set_texture_name = fallback_set_texture_name;
        backend.create_transfer_buffer = fallback_create_transfer_buffer;
        backend.release_transfer_buffer = fallback_release_transfer_buffer;
        backend.map_transfer_buffer = fallback_map_transfer_buffer;
        backend.unmap_transfer_buffer = fallback_unmap_transfer_buffer;
        backend.create_sampler = fallback_create_sampler;
        backend.release_sampler = fallback_release_sampler;
        backend.create_shader = fallback_create_shader;
        backend.release_shader = fallback_release_shader;
        backend.create_compute_pipeline = fallback_create_compute_pipeline;
        backend.release_compute_pipeline = fallback_release_compute_pipeline;
        backend.create_framebuffer = fallback_create_framebuffer;
        backend.release_framebuffer = fallback_release_framebuffer;
        backend.create_resource_set_layout = fallback_create_resource_set_layout;
        backend.release_resource_set_layout = fallback_release_resource_set_layout;
        backend.create_resource_set = fallback_create_resource_set;
        backend.release_resource_set = fallback_release_resource_set;
        backend.update_resource_set = fallback_update_resource_set;
        backend.create_query_pool = fallback_create_query_pool;
        backend.release_query_pool = fallback_release_query_pool;
        backend.get_query_results = fallback_get_query_results;
        backend.create_shader_from_desc = fallback_create_shader_from_desc;
        backend.destroy_shader = fallback_destroy_shader;
        backend.create_pipeline = fallback_create_pipeline;
        backend.destroy_pipeline = fallback_destroy_pipeline;
        backend.copy_buffer_to_buffer = fallback_copy_buffer_to_buffer;
        backend.copy_buffer_to_texture = fallback_copy_buffer_to_texture;
        backend.copy_texture_to_buffer = fallback_copy_texture_to_buffer;
        backend.copy_texture_to_texture = fallback_copy_texture_to_texture;
        backend.blit_texture = fallback_blit_texture;
        backend.barrier_resources = fallback_barrier_resources;
        backend.transition_buffer = fallback_transition_buffer;
        backend.transition_texture = fallback_transition_texture;
        backend.bind_pipeline = fallback_bind_pipeline;
        backend.bind_compute_pipeline = fallback_bind_compute_pipeline;
        backend.bind_resource_set = fallback_bind_resource_set;
        backend.bind_vertex_buffers = fallback_bind_vertex_buffers;
        backend.bind_index_buffer = fallback_bind_index_buffer;
        backend.bind_samplers = fallback_bind_samplers;
        backend.bind_textures = fallback_bind_textures;
        backend.bind_uniform_buffers = fallback_bind_uniform_buffers;
        backend.bind_storage_buffers = fallback_bind_storage_buffers;
        backend.bind_storage_textures = fallback_bind_storage_textures;
        backend.draw = fallback_draw;
        backend.draw_indexed = fallback_draw_indexed;
        backend.draw_indirect = fallback_draw_indirect;
        backend.draw_indexed_indirect = fallback_draw_indexed_indirect;
        backend.dispatch = fallback_dispatch;
        backend.dispatch_indirect = fallback_dispatch_indirect;
        backend.reset_query_pool = fallback_reset_query_pool;
        backend.write_timestamp = fallback_write_timestamp;
        backend.begin_query = fallback_begin_query;
        backend.end_query = fallback_end_query;
        backend.push_uniform_data = fallback_push_uniform_data;
        backend.set_viewport = fallback_set_viewport;
        backend.set_scissor = fallback_set_scissor;
        backend.set_blend_constants = fallback_set_blend_constants;
        backend.set_stencil_reference = fallback_set_stencil_reference;
        backend.clear = fallback_clear;
        backend.push_debug_group = fallback_push_debug_group;
        backend.pop_debug_group = fallback_pop_debug_group;
        backend.insert_debug_label = fallback_insert_debug_label;
        backend.query_fence = fallback_query_fence;
        backend.release_fence = fallback_release_fence;
        backend.wait_for_fences = fallback_wait_for_fences;
        initialized = true;
    }
    return &backend;
}

} // namespace

const Moss_GPUBackend* Moss_GetFallbackGPUBackendTable() {
    return Moss_GetFallbackGPUBackend();
}

Moss_GPUBackend* Moss_CreateNamedFallbackGPUBackend(Moss_GPUBackendType type, const char* name, uint32_t shader_formats, bool (*supports_shader_formats)(Moss_GPUDevice*, uint32_t)) {
    Moss_GPUBackend* backend = new (std::nothrow) Moss_GPUBackend(*Moss_GetFallbackGPUBackend());
    if (!backend) {
        return nullptr;
    }
    backend->name = name;
    backend->type = type;
    backend->shader_formats = shader_formats;
    backend->flags = MOSS_GPU_BACKEND_FLAG_FALLBACK_FUNCTION_TABLE | MOSS_GPU_BACKEND_FLAG_COMPUTE | MOSS_GPU_BACKEND_FLAG_BINDLESS | MOSS_GPU_BACKEND_FLAG_DEBUG_MARKERS;
    backend->supports_shader_formats = supports_shader_formats;
    return backend;
}

void Moss_RegisterGPUBackendDevice(Moss_GPUDevice* device, const Moss_GPUBackend* backend) { register_binding(g_device_backends, device, backend); }
void Moss_UnregisterGPUBackendDevice(Moss_GPUDevice* device) { unregister_binding(g_device_backends, device); }
const Moss_GPUBackend* Moss_GetGPUBackend(Moss_GPUDevice* device) { return find_binding(g_device_backends, device); }
const Moss_GPUBackend* Moss_GetGPUBackendFromCommandBuffer(Moss_GPUCommandBuffer* cmd) { return find_binding(g_command_buffer_backends, cmd); }

namespace {

const Moss_GPUBackend* backend_from_type(Moss_GPUBackendType type) {
    switch (type) {
        case MOSS_GPU_BACKEND_DEFAULT: return select_compiled_backend();
        case MOSS_GPU_BACKEND_OPENGL: return Moss_GetOpenGLGPUBackend();
        case MOSS_GPU_BACKEND_OPENGLES: return Moss_GetOpenGLESGPUBackend();
        case MOSS_GPU_BACKEND_VULKAN: return Moss_GetVulkanGPUBackend();
        case MOSS_GPU_BACKEND_DIRECTX12: return Moss_GetD3D12GPUBackend();
        case MOSS_GPU_BACKEND_METAL: return Moss_GetMetalGPUBackend();
        case MOSS_GPU_BACKEND_FALLBACK: return Moss_GetFallbackGPUBackend();
    }
    return nullptr;
}

bool backend_has_registered_native_table(Moss_GPUBackendType type) {
    return Moss_GPUBackendHasRegisteredNativeTable(type);
}

uint32_t backend_default_shader_formats(Moss_GPUBackendType type) {
    switch (type) {
        case MOSS_GPU_BACKEND_OPENGL: return MOSS_GPU_SHADERFORMAT_GLSL;
        case MOSS_GPU_BACKEND_OPENGLES: return MOSS_GPU_SHADERFORMAT_GLSL_ES;
        case MOSS_GPU_BACKEND_VULKAN: return MOSS_GPU_SHADERFORMAT_SPIRV;
        case MOSS_GPU_BACKEND_DIRECTX12: return MOSS_GPU_SHADERFORMAT_DXIL | MOSS_GPU_SHADERFORMAT_DXBC;
        case MOSS_GPU_BACKEND_METAL: return MOSS_GPU_SHADERFORMAT_MSL | MOSS_GPU_SHADERFORMAT_METALLIB;
        case MOSS_GPU_BACKEND_FALLBACK: return MOSS_GPU_SHADERFORMAT_GLSL;
        default: return MOSS_GPU_SHADERFORMAT_NONE;
    }
}

} // namespace

Moss_GPUBackendType Moss_GetCompiledGPUBackendType(void) {
#if defined(MOSS_USE_OPENGL) || defined(MOSS_GRAPHICS_OPENGL)
    return MOSS_GPU_BACKEND_OPENGL;
#elif defined(MOSS_USE_OPENGLES) || defined(MOSS_GRAPHICS_OPENGLES)
    return MOSS_GPU_BACKEND_OPENGLES;
#elif defined(MOSS_USE_VULKAN) || defined(MOSS_GRAPHICS_VULKAN)
    return MOSS_GPU_BACKEND_VULKAN;
#elif defined(MOSS_USE_DIRECTX12) || defined(MOSS_GRAPHICS_DIRECTX)
    return MOSS_GPU_BACKEND_DIRECTX12;
#elif defined(MOSS_USE_METAL) || defined(MOSS_GRAPHICS_METAL)
    return MOSS_GPU_BACKEND_METAL;
#else
    return MOSS_GPU_BACKEND_FALLBACK;
#endif
}

Moss_GPUBackendType Moss_GetGPUDeviceBackendType(Moss_GPUDevice* device) {
    const Moss_GPUBackend* backend = Moss_GetGPUBackend(device);
    return backend ? backend->type : MOSS_GPU_BACKEND_DEFAULT;
}

const char* Moss_GPUBackendGetName(Moss_GPUBackendType type) {
    const Moss_GPUBackend* backend = backend_from_type(type);
    if (backend && backend->name) {
        return backend->name;
    }
    switch (type) {
        case MOSS_GPU_BACKEND_OPENGL: return "OpenGL";
        case MOSS_GPU_BACKEND_OPENGLES: return "OpenGL ES";
        case MOSS_GPU_BACKEND_VULKAN: return "Vulkan";
        case MOSS_GPU_BACKEND_DIRECTX12: return "DirectX 12";
        case MOSS_GPU_BACKEND_METAL: return "Metal";
        case MOSS_GPU_BACKEND_FALLBACK: return "fallback";
        default: return "default";
    }
}

uint32_t Moss_GPUBackendGetShaderFormats(Moss_GPUBackendType type) {
    const Moss_GPUBackend* backend = backend_from_type(type);
    return backend ? backend->shader_formats : backend_default_shader_formats(type);
}

bool Moss_GPUBackendIsCompiled(Moss_GPUBackendType type) {
    if (type == MOSS_GPU_BACKEND_DEFAULT) {
        return true;
    }
    return Moss_GetCompiledGPUBackendType() == type;
}

bool Moss_GPUBackendIsSupportedOnPlatform(Moss_GPUBackendType type) {
    switch (type) {
        case MOSS_GPU_BACKEND_DEFAULT:
        case MOSS_GPU_BACKEND_FALLBACK:
            return true;
        case MOSS_GPU_BACKEND_OPENGL:
#if defined(_WIN32) || defined(__linux__) || (defined(__APPLE__) && !(defined(TARGET_OS_IPHONE) && TARGET_OS_IPHONE))
            return true;
#else
            return false;
#endif
        case MOSS_GPU_BACKEND_OPENGLES:
#if defined(__ANDROID__) || defined(__linux__) || (defined(__APPLE__) && defined(TARGET_OS_IPHONE) && TARGET_OS_IPHONE)
            return true;
#else
            return false;
#endif
        case MOSS_GPU_BACKEND_VULKAN:
#if defined(_WIN32) || defined(__linux__) || defined(__ANDROID__)
            return true;
#else
            return false;
#endif
        case MOSS_GPU_BACKEND_DIRECTX12:
#if defined(_WIN32)
            return true;
#else
            return false;
#endif
        case MOSS_GPU_BACKEND_METAL:
#if defined(__APPLE__)
            return true;
#else
            return false;
#endif
    }
    return false;
}

bool Moss_RegisterNativeGPUBackend(Moss_GPUBackendType type, const void* native_table) {
    const Moss_GPUBackend* backend = static_cast<const Moss_GPUBackend*>(native_table);
    if (!backend) {
        return false;
    }

    switch (type) {
        case MOSS_GPU_BACKEND_OPENGL: Moss_SetOpenGLGPUBackend(backend); return true;
        case MOSS_GPU_BACKEND_OPENGLES: Moss_SetOpenGLESGPUBackend(backend); return true;
        case MOSS_GPU_BACKEND_VULKAN: Moss_SetVulkanGPUBackend(backend); return true;
        case MOSS_GPU_BACKEND_DIRECTX12: Moss_SetD3D12GPUBackend(backend); return true;
        case MOSS_GPU_BACKEND_METAL: Moss_SetMetalGPUBackend(backend); return true;
        default: return false;
    }
}

bool Moss_HasNativeGPUBackend(Moss_GPUBackendType type) {
    return backend_has_registered_native_table(type);
}
bool Moss_GetGPUBackendInfo(Moss_GPUBackendType type, Moss_GPUBackendInfo* out_info) {
    if (!out_info) {
        return false;
    }

    const Moss_GPUBackend* backend = backend_from_type(type);
    const Moss_GPUBackendType resolved_type = (type == MOSS_GPU_BACKEND_DEFAULT && backend) ? backend->type : type;
    out_info->type = resolved_type;
    out_info->name = Moss_GPUBackendGetName(resolved_type);
    out_info->shader_formats = backend ? backend->shader_formats : backend_default_shader_formats(resolved_type);
    out_info->flags = backend ? backend->flags : MOSS_GPU_BACKEND_FLAG_NONE;
    if (Moss_GPUBackendIsCompiled(resolved_type)) {
        out_info->flags |= MOSS_GPU_BACKEND_FLAG_COMPILED;
    }
    if (Moss_GPUBackendIsSupportedOnPlatform(resolved_type)) {
        out_info->flags |= MOSS_GPU_BACKEND_FLAG_SUPPORTED_ON_PLATFORM;
    }
    if (backend_has_registered_native_table(resolved_type)) {
        out_info->flags |= MOSS_GPU_BACKEND_FLAG_REGISTERED_NATIVE;
        out_info->flags &= ~MOSS_GPU_BACKEND_FLAG_FALLBACK_FUNCTION_TABLE;
    } else if (backend) {
        out_info->flags |= MOSS_GPU_BACKEND_FLAG_FALLBACK_FUNCTION_TABLE;
    }
    if (backend && backend->present_swapchain_status) {
        out_info->flags |= MOSS_GPU_BACKEND_FLAG_WINDOW_PRESENT;
    }
    if (backend && backend->supports_async_compute) {
        out_info->flags |= MOSS_GPU_BACKEND_FLAG_COMPUTE;
    }
    return true;
}

#define MOSS_DEVICE_BACKEND(device) const Moss_GPUBackend* backend = Moss_GetGPUBackend(device)
#define MOSS_CMD_BACKEND(cmd) const Moss_GPUBackend* backend = Moss_GetGPUBackendFromCommandBuffer(cmd)
#define MOSS_CALL_VOID(fn, ...) do { if (backend && backend->fn) backend->fn(__VA_ARGS__); } while (0)
#define MOSS_CALL_RET(fn, fallback, ...) ((backend && backend->fn) ? backend->fn(__VA_ARGS__) : (fallback))

Moss_GPUDevice* Moss_CreateGPUDevice(const Moss_GPUDeviceDesc* desc) {
    return Moss_CreateGPUDeviceWithProperties(desc, nullptr);
}

Moss_GPUDevice* Moss_CreateGPUDeviceWithProperties(const Moss_GPUDeviceDesc* desc, const Moss_GPUDeviceProperties* requested_properties) {
    const Moss_GPUBackendType requested_backend = desc ? desc->backend : MOSS_GPU_BACKEND_DEFAULT;
    const Moss_GPUBackend* backend = nullptr;
    if (requested_backend == MOSS_GPU_BACKEND_DEFAULT) {
        backend = select_compiled_backend();
    } else if (Moss_GPUBackendIsCompiled(requested_backend)) {
        backend = backend_from_type(requested_backend);
    }

    if (!backend || !backend->create_device) {
        return nullptr;
    }

    Moss_GPUDevice* device = backend->create_device(desc, requested_properties);
    Moss_RegisterGPUBackendDevice(device, backend);
    return device;
}

void Moss_DestroyGPUDevice(Moss_GPUDevice* device) {
    MOSS_DEVICE_BACKEND(device);
    unregister_bindless_resources_for_device(device);
    unregister_bindings_for_backend(g_command_buffer_backends, backend);
    if (backend && backend->destroy_device) {
        backend->destroy_device(device);
    }
    Moss_UnregisterGPUBackendDevice(device);
}

Moss_GPUCommandBuffer* Moss_AcquireGPUCommandBuffer(Moss_GPUDevice* device) {
    MOSS_DEVICE_BACKEND(device);
    Moss_GPUCommandBuffer* cmd = MOSS_CALL_RET(acquire_command_buffer, nullptr, device);
    register_command_buffer(cmd, backend);
    return cmd;
}

Moss_GPUCommandBuffer* Moss_GPUCommandBufferBegin(Moss_GPUDevice* device, ECommandQueue queue) {
    MOSS_DEVICE_BACKEND(device);
    Moss_GPUCommandBuffer* cmd = MOSS_CALL_RET(begin_command_buffer, nullptr, device, queue);
    register_command_buffer(cmd, backend);
    return cmd;
}

void Moss_GPUCommandBufferEnd(Moss_GPUCommandBuffer* cmd) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(end_command_buffer, cmd); }
void Moss_GPUCommandBufferSubmit(Moss_GPUDevice* device, Moss_GPUCommandBuffer* cmd) { Moss_SubmitGPUCommandBuffer(device, cmd); }
void Moss_SubmitGPUCommandBuffer(Moss_GPUDevice* device, Moss_GPUCommandBuffer* cmd) {
    MOSS_DEVICE_BACKEND(device);
    MOSS_CALL_VOID(submit_command_buffer, device, cmd);
    unregister_binding(g_command_buffer_backends, cmd);
}

Moss_GPUFence* Moss_SubmitGPUCommandBufferAndAcquireFence(Moss_GPUDevice* device, Moss_GPUCommandBuffer* cmd) {
    MOSS_DEVICE_BACKEND(device);
    Moss_GPUFence* fence = MOSS_CALL_RET(submit_command_buffer_and_acquire_fence, nullptr, device, cmd);
    unregister_binding(g_command_buffer_backends, cmd);
    return fence;
}
void Moss_CancelGPUCommandBuffer(Moss_GPUCommandBuffer* cmd) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(cancel_command_buffer, cmd); unregister_binding(g_command_buffer_backends, cmd); }

void Moss_BeginGPURenderPass(Moss_GPUCommandBuffer* cmd, Moss_Framebuffer* framebuffer) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(begin_render_pass, cmd, framebuffer); }
void Moss_EndGPURenderPass(Moss_GPUCommandBuffer* cmd) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(end_render_pass, cmd); }
void Moss_BeginGPUComputePass(Moss_GPUCommandBuffer* cmd) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(begin_compute_pass, cmd); }
void Moss_EndGPUComputePass(Moss_GPUCommandBuffer* cmd) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(end_compute_pass, cmd); }
void Moss_BeginGPUCopyPass(Moss_GPUCommandBuffer* cmd) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(begin_copy_pass, cmd); }
void Moss_EndGPUCopyPass(Moss_GPUCommandBuffer* cmd) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(end_copy_pass, cmd); }
void Moss_CmdBeginRenderPass(Moss_GPUCommandBuffer* cmd, Moss_Framebuffer* fb) { Moss_BeginGPURenderPass(cmd, fb); }
void Moss_CmdEndRenderPass(Moss_GPUCommandBuffer* cmd) { Moss_EndGPURenderPass(cmd); }

Moss_GPUBuffer* Moss_CreateGPUBuffer(Moss_GPUDevice* device, const Moss_GPUBufferDesc* desc) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(create_buffer, nullptr, device, desc); }
Moss_GPUBuffer* Moss_GPUBufferCreate(Moss_GPUDevice* device, const Moss_GPUBufferDesc* desc) { return Moss_CreateGPUBuffer(device, desc); }
void Moss_ReleaseGPUBuffer(Moss_GPUDevice* device, Moss_GPUBuffer* buffer) { MOSS_DEVICE_BACKEND(device); MOSS_CALL_VOID(release_buffer, device, buffer); }
void Moss_GPUBufferDestroy(Moss_GPUDevice* device, Moss_GPUBuffer* buffer) { Moss_ReleaseGPUBuffer(device, buffer); }
void Moss_UploadToGPUBuffer(Moss_GPUDevice* device, Moss_GPUBuffer* dst_buffer, const void* src_data, uint64_t size, uint64_t dst_offset) { MOSS_DEVICE_BACKEND(device); MOSS_CALL_VOID(upload_to_buffer, device, dst_buffer, src_data, size, dst_offset); }
void Moss_GPUBufferUpload(Moss_GPUDevice* device, Moss_GPUBuffer* buffer, const void* data, uint64_t size, uint64_t offset) { Moss_UploadToGPUBuffer(device, buffer, data, size, offset); }
void Moss_DownloadFromGPUBuffer(Moss_GPUDevice* device, Moss_GPUBuffer* src_buffer, void* dst_data, uint64_t size, uint64_t src_offset) { MOSS_DEVICE_BACKEND(device); MOSS_CALL_VOID(download_from_buffer, device, src_buffer, dst_data, size, src_offset); }
void* Moss_GPUBufferMap(Moss_GPUDevice* device, Moss_GPUBuffer* buffer) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(map_buffer, nullptr, device, buffer); }
void Moss_GPUBufferUnmap(Moss_GPUDevice* device, Moss_GPUBuffer* buffer) { MOSS_DEVICE_BACKEND(device); MOSS_CALL_VOID(unmap_buffer, device, buffer); }
void Moss_SetGPUBufferName(Moss_GPUDevice* device, Moss_GPUBuffer* buffer, const char* name) { MOSS_DEVICE_BACKEND(device); MOSS_CALL_VOID(set_buffer_name, device, buffer, name); }

Moss_GPUTexture* Moss_CreateGPUTexture(Moss_GPUDevice* device, const Moss_GPUTextureCreateInfo* create_info) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(create_texture, nullptr, device, create_info); }
void Moss_ReleaseGPUTexture(Moss_GPUDevice* device, Moss_GPUTexture* texture) { MOSS_DEVICE_BACKEND(device); MOSS_CALL_VOID(release_texture, device, texture); }
Moss_GPUTextureView* Moss_CreateGPUTextureView(Moss_GPUDevice* device, const Moss_GPUTextureViewCreateInfo* create_info) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(create_texture_view, nullptr, device, create_info); }
void Moss_ReleaseGPUTextureView(Moss_GPUDevice* device, Moss_GPUTextureView* texture_view) { MOSS_DEVICE_BACKEND(device); MOSS_CALL_VOID(release_texture_view, device, texture_view); }
void Moss_UploadToGPUTexture(Moss_GPUDevice* device, Moss_GPUTexture* dst_texture, const Moss_GPUTextureTransferInfo* transfer_info) { MOSS_DEVICE_BACKEND(device); MOSS_CALL_VOID(upload_to_texture, device, dst_texture, transfer_info); }
void Moss_DownloadFromGPUTexture(Moss_GPUDevice* device, Moss_GPUTexture* src_texture, const Moss_GPUTextureRegion* src_region, void* dst_data, uint32_t dst_row_pitch, uint32_t dst_slice_pitch) { MOSS_DEVICE_BACKEND(device); MOSS_CALL_VOID(download_from_texture, device, src_texture, src_region, dst_data, dst_row_pitch, dst_slice_pitch); }
void Moss_GenerateMipmapsForGPUTexture(Moss_GPUCommandBuffer* cmd, Moss_GPUTexture* texture) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(generate_mipmaps, cmd, texture); }
void Moss_SetGPUTextureName(Moss_GPUDevice* device, Moss_GPUTexture* texture, const char* name) { MOSS_DEVICE_BACKEND(device); MOSS_CALL_VOID(set_texture_name, device, texture, name); }
Moss_Texture* Moss_TextureCreate(Moss_GPUDevice* device, const Moss_TextureDesc* desc) { Moss_GPUTextureCreateInfo info = to_gpu_texture_create_info(desc); return reinterpret_cast<Moss_Texture*>(Moss_CreateGPUTexture(device, &info)); }
void Moss_TextureDestroy(Moss_GPUDevice* device, Moss_Texture* texture) { Moss_ReleaseGPUTexture(device, reinterpret_cast<Moss_GPUTexture*>(texture)); }
void Moss_TextureUpload(Moss_GPUDevice* device, Moss_Texture* texture, const Moss_TextureUploadDesc* desc) {
    if (!desc) return;
    Moss_GPUTextureTransferInfo info{};
    info.data = desc->data;
    info.row_pitch = desc->row_pitch;
    info.slice_pitch = desc->slice_pitch;
    info.region.mip_level = desc->mip_level;
    info.region.base_layer = desc->base_layer;
    info.region.layer_count = desc->layer_count;
    info.region.width = desc->width;
    info.region.height = desc->height;
    info.region.depth = desc->depth;
    Moss_UploadToGPUTexture(device, reinterpret_cast<Moss_GPUTexture*>(texture), &info);
}
void Moss_TextureGenerateMips(Moss_GPUDevice* device, Moss_Texture* texture) {
    if (!device || !texture) {
        return;
    }

    Moss_GPUCommandBuffer* cmd = Moss_GPUCommandBufferBegin(device, ECommandQueue::GRAPHICS);
    if (!cmd) {
        return;
    }

    Moss_GenerateMipmapsForGPUTexture(cmd, reinterpret_cast<Moss_GPUTexture*>(texture));
    Moss_GPUCommandBufferEnd(cmd);
    Moss_SubmitGPUCommandBuffer(device, cmd);
}

Moss_GPUTransferBuffer* Moss_CreateGPUTransferBuffer(Moss_GPUDevice* device, const Moss_GPUTransferBufferCreateInfo* create_info) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(create_transfer_buffer, nullptr, device, create_info); }
void Moss_ReleaseGPUTransferBuffer(Moss_GPUDevice* device, Moss_GPUTransferBuffer* transfer_buffer) { MOSS_DEVICE_BACKEND(device); MOSS_CALL_VOID(release_transfer_buffer, device, transfer_buffer); }
void* Moss_MapGPUTransferBuffer(Moss_GPUDevice* device, Moss_GPUTransferBuffer* transfer_buffer) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(map_transfer_buffer, nullptr, device, transfer_buffer); }
void Moss_UnmapGPUTransferBuffer(Moss_GPUDevice* device, Moss_GPUTransferBuffer* transfer_buffer) { MOSS_DEVICE_BACKEND(device); MOSS_CALL_VOID(unmap_transfer_buffer, device, transfer_buffer); }

Moss_GPUSampler* Moss_CreateGPUSampler(Moss_GPUDevice* device, const Moss_GPUSamplerCreateInfo* create_info) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(create_sampler, nullptr, device, create_info); }
void Moss_ReleaseGPUSampler(Moss_GPUDevice* device, Moss_GPUSampler* sampler) { MOSS_DEVICE_BACKEND(device); MOSS_CALL_VOID(release_sampler, device, sampler); }
Moss_GPUSampler* Moss_GPUSamplerCreate(Moss_GPUDevice* device, const Moss_GPUSamplerDesc* desc) { Moss_GPUSamplerCreateInfo info = to_gpu_sampler_create_info(desc); return Moss_CreateGPUSampler(device, &info); }
void Moss_GPUSamplerDestroy(Moss_GPUDevice* device, Moss_GPUSampler* sampler) { Moss_ReleaseGPUSampler(device, sampler); }
Moss_GPUShader* Moss_CreateGPUShader(Moss_GPUDevice* device, const Moss_GPUShaderCreateInfo* create_info) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(create_shader, nullptr, device, create_info); }
void Moss_ReleaseGPUShader(Moss_GPUDevice* device, Moss_GPUShader* shader) { MOSS_DEVICE_BACKEND(device); MOSS_CALL_VOID(release_shader, device, shader); }
Moss_ComputePipelineState* Moss_CreateGPUComputePipeline(Moss_GPUDevice* device, const Moss_GPUComputePipelineCreateInfo* create_info) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(create_compute_pipeline, nullptr, device, create_info); }
void Moss_ReleaseGPUComputePipeline(Moss_GPUDevice* device, Moss_ComputePipelineState* pipeline) { MOSS_DEVICE_BACKEND(device); MOSS_CALL_VOID(release_compute_pipeline, device, pipeline); }
Moss_Framebuffer* Moss_CreateGPUFramebuffer(Moss_GPUDevice* device, const Moss_GPUFramebufferCreateInfo* create_info) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(create_framebuffer, nullptr, device, create_info); }
void Moss_ReleaseGPUFramebuffer(Moss_GPUDevice* device, Moss_Framebuffer* framebuffer) { MOSS_DEVICE_BACKEND(device); MOSS_CALL_VOID(release_framebuffer, device, framebuffer); }
Moss_ResourceSetLayout* Moss_CreateGPUResourceSetLayout(Moss_GPUDevice* device, const Moss_GPUResourceSetLayoutCreateInfo* create_info) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(create_resource_set_layout, nullptr, device, create_info); }
void Moss_ReleaseGPUResourceSetLayout(Moss_GPUDevice* device, Moss_ResourceSetLayout* layout) { MOSS_DEVICE_BACKEND(device); MOSS_CALL_VOID(release_resource_set_layout, device, layout); }
Moss_ResourceSet* Moss_CreateGPUResourceSet(Moss_GPUDevice* device, const Moss_GPUResourceSetCreateInfo* create_info) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(create_resource_set, nullptr, device, create_info); }
void Moss_ReleaseGPUResourceSet(Moss_GPUDevice* device, Moss_ResourceSet* set) { MOSS_DEVICE_BACKEND(device); MOSS_CALL_VOID(release_resource_set, device, set); }
void Moss_UpdateGPUResourceSet(Moss_GPUDevice* device, Moss_ResourceSet* set, const Moss_GPUResourceBinding* bindings, uint32_t binding_count) { MOSS_DEVICE_BACKEND(device); MOSS_CALL_VOID(update_resource_set, device, set, bindings, binding_count); }
Moss_GPUQueryPool* Moss_CreateGPUQueryPool(Moss_GPUDevice* device, const Moss_GPUQueryPoolCreateInfo* create_info) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(create_query_pool, nullptr, device, create_info); }
void Moss_ReleaseGPUQueryPool(Moss_GPUDevice* device, Moss_GPUQueryPool* query_pool) { MOSS_DEVICE_BACKEND(device); MOSS_CALL_VOID(release_query_pool, device, query_pool); }
bool Moss_GetGPUQueryResults(Moss_GPUDevice* device, Moss_GPUQueryPool* query_pool, uint32_t first_query, uint32_t query_count, void* data, uint64_t data_size, uint64_t stride, EGPUQueryResultFlags flags) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(get_query_results, false, device, query_pool, first_query, query_count, data, data_size, stride, flags); }
Moss_Shader* Moss_ShaderCreate(Moss_GPUDevice* device, const Moss_ShaderDesc* desc) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(create_shader_from_desc, nullptr, device, desc); }
void Moss_ShaderDestroy(Moss_GPUDevice* device, Moss_Shader* shader) { MOSS_DEVICE_BACKEND(device); MOSS_CALL_VOID(destroy_shader, device, shader); }
Moss_PipelineState* Moss_PipelineCreate(Moss_GPUDevice* device, const Moss_PipelineDesc* desc) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(create_pipeline, nullptr, device, desc); }
void Moss_PipelineDestroy(Moss_GPUDevice* device, Moss_PipelineState* pipeline) { MOSS_DEVICE_BACKEND(device); MOSS_CALL_VOID(destroy_pipeline, device, pipeline); }

void Moss_CopyGPUBufferToBuffer(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* src, Moss_GPUBuffer* dst, const Moss_GPUBufferRegion* src_region, const Moss_GPUBufferRegion* dst_region) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(copy_buffer_to_buffer, cmd, src, dst, src_region, dst_region); }
void Moss_CopyGPUBufferToTexture(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* src, Moss_GPUTexture* dst, const Moss_GPUBufferRegion* src_region, const Moss_GPUTextureRegion* dst_region) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(copy_buffer_to_texture, cmd, src, dst, src_region, dst_region); }
void Moss_CopyGPUTextureToBuffer(Moss_GPUCommandBuffer* cmd, Moss_GPUTexture* src, Moss_GPUBuffer* dst, const Moss_GPUTextureRegion* src_region, const Moss_GPUBufferRegion* dst_region) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(copy_texture_to_buffer, cmd, src, dst, src_region, dst_region); }
void Moss_CopyGPUTextureToTexture(Moss_GPUCommandBuffer* cmd, Moss_GPUTexture* src, Moss_GPUTexture* dst, const Moss_GPUTextureRegion* src_region, const Moss_GPUTextureRegion* dst_region) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(copy_texture_to_texture, cmd, src, dst, src_region, dst_region); }
void Moss_BlitGPUTexture(Moss_GPUCommandBuffer* cmd, Moss_Texture* src, Moss_Texture* dst, const Moss_GPUBlitRegion* region) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(blit_texture, cmd, src, dst, region); }
void Moss_BarrierGPUResources(Moss_GPUCommandBuffer* cmd, const Moss_GPUBarrierInfo* barrier_info) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(barrier_resources, cmd, barrier_info); }
void Moss_TransitionGPUBuffer(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* buffer, EResourceState old_state, EResourceState new_state) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(transition_buffer, cmd, buffer, old_state, new_state); }
void Moss_TransitionGPUTexture(Moss_GPUCommandBuffer* cmd, Moss_GPUTexture* texture, EResourceState old_state, EResourceState new_state) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(transition_texture, cmd, texture, old_state, new_state); }

void Moss_BindGPUPipeline(Moss_GPUCommandBuffer* cmd, Moss_PipelineState* pipeline) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(bind_pipeline, cmd, pipeline); }
void Moss_CmdBindPipeline(Moss_GPUCommandBuffer* cmd, Moss_PipelineState* pipeline) { Moss_BindGPUPipeline(cmd, pipeline); }
void Moss_BindGPUComputePipeline(Moss_GPUCommandBuffer* cmd, Moss_ComputePipelineState* pipeline) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(bind_compute_pipeline, cmd, pipeline); }
void Moss_BindGPUResourceSet(Moss_GPUCommandBuffer* cmd, uint32_t set_index, Moss_ResourceSet* set) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(bind_resource_set, cmd, set_index, set); }
void Moss_CmdBindResourceSet(Moss_GPUCommandBuffer* cmd, uint32_t set_index, Moss_ResourceSet* set) { Moss_BindGPUResourceSet(cmd, set_index, set); }
void Moss_BindGPUVertexBuffers(Moss_GPUCommandBuffer* cmd, uint32_t first_binding, const Moss_GPUBufferBinding* bindings, uint32_t binding_count) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(bind_vertex_buffers, cmd, first_binding, bindings, binding_count); }
void Moss_CmdBindVertexBuffer(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* buffer, uint64_t offset) { Moss_GPUBufferBinding binding{ buffer, offset }; Moss_BindGPUVertexBuffers(cmd, 0, &binding, 1); }
void Moss_BindGPUIndexBuffer(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* buffer, uint64_t offset, EGPUIndexType index_type) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(bind_index_buffer, cmd, buffer, offset, index_type); }
void Moss_CmdBindIndexBuffer(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* buffer, uint64_t offset) { Moss_BindGPUIndexBuffer(cmd, buffer, offset, EGPUIndexType::UINT32); }

void Moss_BindGPUVertexSamplers(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, Moss_GPUSampler* const* samplers, uint32_t count) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(bind_samplers, cmd, EShaderStage::VERTEX, first_slot, samplers, count); }
void Moss_BindGPUFragmentSamplers(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, Moss_GPUSampler* const* samplers, uint32_t count) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(bind_samplers, cmd, EShaderStage::FRAGMENT, first_slot, samplers, count); }
void Moss_BindGPUComputeSamplers(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, Moss_GPUSampler* const* samplers, uint32_t count) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(bind_samplers, cmd, EShaderStage::COMPUTE, first_slot, samplers, count); }
void Moss_BindGPUVertexTextures(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, Moss_GPUTexture* const* textures, uint32_t count) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(bind_textures, cmd, EShaderStage::VERTEX, first_slot, textures, count); }
void Moss_BindGPUFragmentTextures(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, Moss_GPUTexture* const* textures, uint32_t count) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(bind_textures, cmd, EShaderStage::FRAGMENT, first_slot, textures, count); }
void Moss_BindGPUComputeTextures(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, Moss_GPUTexture* const* textures, uint32_t count) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(bind_textures, cmd, EShaderStage::COMPUTE, first_slot, textures, count); }
void Moss_BindGPUVertexUniformBuffers(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, const Moss_GPUBufferBinding* bindings, uint32_t count) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(bind_uniform_buffers, cmd, EShaderStage::VERTEX, first_slot, bindings, count); }
void Moss_BindGPUFragmentUniformBuffers(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, const Moss_GPUBufferBinding* bindings, uint32_t count) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(bind_uniform_buffers, cmd, EShaderStage::FRAGMENT, first_slot, bindings, count); }
void Moss_BindGPUComputeUniformBuffers(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, const Moss_GPUBufferBinding* bindings, uint32_t count) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(bind_uniform_buffers, cmd, EShaderStage::COMPUTE, first_slot, bindings, count); }
void Moss_BindGPUVertexStorageBuffers(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, const Moss_GPUStorageBufferReadWriteBinding* bindings, uint32_t count) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(bind_storage_buffers, cmd, EShaderStage::VERTEX, first_slot, bindings, count); }
void Moss_BindGPUFragmentStorageBuffers(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, const Moss_GPUStorageBufferReadWriteBinding* bindings, uint32_t count) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(bind_storage_buffers, cmd, EShaderStage::FRAGMENT, first_slot, bindings, count); }
void Moss_BindGPUComputeStorageBuffers(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, const Moss_GPUStorageBufferReadWriteBinding* bindings, uint32_t count) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(bind_storage_buffers, cmd, EShaderStage::COMPUTE, first_slot, bindings, count); }
void Moss_BindGPUVertexStorageTextures(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, const Moss_GPUStorageTextureReadWriteBinding* bindings, uint32_t count) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(bind_storage_textures, cmd, EShaderStage::VERTEX, first_slot, bindings, count); }
void Moss_BindGPUFragmentStorageTextures(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, const Moss_GPUStorageTextureReadWriteBinding* bindings, uint32_t count) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(bind_storage_textures, cmd, EShaderStage::FRAGMENT, first_slot, bindings, count); }
void Moss_BindGPUComputeStorageTextures(Moss_GPUCommandBuffer* cmd, uint32_t first_slot, const Moss_GPUStorageTextureReadWriteBinding* bindings, uint32_t count) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(bind_storage_textures, cmd, EShaderStage::COMPUTE, first_slot, bindings, count); }

void Moss_DrawGPUPrimitives(Moss_GPUCommandBuffer* cmd, uint32_t vertex_count, uint32_t instance_count, uint32_t first_vertex, uint32_t first_instance) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(draw, cmd, vertex_count, instance_count, first_vertex, first_instance); }
void Moss_CmdDraw(Moss_GPUCommandBuffer* cmd, uint32_t vertex_count, uint32_t first_vertex) { Moss_DrawGPUPrimitives(cmd, vertex_count, 1, first_vertex, 0); }
void Moss_DrawGPUIndexedPrimitives(Moss_GPUCommandBuffer* cmd, uint32_t index_count, uint32_t instance_count, uint32_t first_index, int32_t vertex_offset, uint32_t first_instance) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(draw_indexed, cmd, index_count, instance_count, first_index, vertex_offset, first_instance); }
void Moss_CmdDrawIndexed(Moss_GPUCommandBuffer* cmd, uint32_t index_count, uint32_t first_index, int32_t vertex_offset) { Moss_DrawGPUIndexedPrimitives(cmd, index_count, 1, first_index, vertex_offset, 0); }
void Moss_DrawGPUPrimitivesIndirect(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* indirect_buffer, uint64_t offset, uint32_t draw_count, uint32_t stride) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(draw_indirect, cmd, indirect_buffer, offset, draw_count, stride); }
void Moss_DrawGPUIndexedPrimitivesIndirect(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* indirect_buffer, uint64_t offset, uint32_t draw_count, uint32_t stride) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(draw_indexed_indirect, cmd, indirect_buffer, offset, draw_count, stride); }
void Moss_DispatchGPUCompute(Moss_GPUCommandBuffer* cmd, uint32_t x, uint32_t y, uint32_t z) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(dispatch, cmd, x, y, z); }
void Moss_CmdDispatch(Moss_GPUCommandBuffer* cmd, uint32_t x, uint32_t y, uint32_t z) { Moss_DispatchGPUCompute(cmd, x, y, z); }
void Moss_DispatchGPUComputeIndirect(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* indirect_buffer, uint64_t offset) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(dispatch_indirect, cmd, indirect_buffer, offset); }
void Moss_ResetGPUQueryPool(Moss_GPUCommandBuffer* cmd, Moss_GPUQueryPool* query_pool, uint32_t first_query, uint32_t query_count) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(reset_query_pool, cmd, query_pool, first_query, query_count); }
void Moss_WriteGPUTimestamp(Moss_GPUCommandBuffer* cmd, Moss_GPUQueryPool* query_pool, uint32_t query_index) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(write_timestamp, cmd, query_pool, query_index); }
void Moss_BeginGPUQuery(Moss_GPUCommandBuffer* cmd, Moss_GPUQueryPool* query_pool, uint32_t query_index) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(begin_query, cmd, query_pool, query_index); }
void Moss_EndGPUQuery(Moss_GPUCommandBuffer* cmd, Moss_GPUQueryPool* query_pool, uint32_t query_index) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(end_query, cmd, query_pool, query_index); }

void Moss_PushGPUVertexUniformData(Moss_GPUCommandBuffer* cmd, uint32_t slot, const void* data, uint32_t size) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(push_uniform_data, cmd, EShaderStage::VERTEX, slot, data, size); }
void Moss_PushGPUFragmentUniformData(Moss_GPUCommandBuffer* cmd, uint32_t slot, const void* data, uint32_t size) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(push_uniform_data, cmd, EShaderStage::FRAGMENT, slot, data, size); }
void Moss_PushGPUComputeUniformData(Moss_GPUCommandBuffer* cmd, uint32_t slot, const void* data, uint32_t size) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(push_uniform_data, cmd, EShaderStage::COMPUTE, slot, data, size); }
void Moss_SetGPUViewport(Moss_GPUCommandBuffer* cmd, const Moss_GPUViewport* viewport) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(set_viewport, cmd, viewport); }
void Moss_SetGPUScissor(Moss_GPUCommandBuffer* cmd, const Moss_Rect* scissor) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(set_scissor, cmd, scissor); }
void Moss_SetGPUBlendConstants(Moss_GPUCommandBuffer* cmd, float r, float g, float b, float a) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(set_blend_constants, cmd, r, g, b, a); }
void Moss_SetGPUStencilReference(Moss_GPUCommandBuffer* cmd, uint32_t reference) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(set_stencil_reference, cmd, reference); }
void Moss_GPUClear(Moss_GPUCommandBuffer* cmd, float r, float g, float b, float a) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(clear, cmd, r, g, b, a); }

void Moss_PushGPUDebugGroup(Moss_GPUCommandBuffer* cmd, const char* label) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(push_debug_group, cmd, label); }
void Moss_PopGPUDebugGroup(Moss_GPUCommandBuffer* cmd) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(pop_debug_group, cmd); }
void Moss_InsertGPUDebugLabel(Moss_GPUCommandBuffer* cmd, const char* label) { MOSS_CMD_BACKEND(cmd); MOSS_CALL_VOID(insert_debug_label, cmd, label); }

bool Moss_QueryGPUFence(Moss_GPUDevice* device, Moss_GPUFence* fence) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(query_fence, false, device, fence); }
void Moss_ReleaseGPUFence(Moss_GPUDevice* device, Moss_GPUFence* fence) { MOSS_DEVICE_BACKEND(device); MOSS_CALL_VOID(release_fence, device, fence); }
void Moss_WaitForGPUFences(Moss_GPUDevice* device, Moss_GPUFence* const* fences, uint32_t fence_count, bool wait_all, uint64_t timeout_ns) { MOSS_DEVICE_BACKEND(device); MOSS_CALL_VOID(wait_for_fences, device, fences, fence_count, wait_all, timeout_ns); }
void Moss_WaitForGPUIdle(Moss_GPUDevice* device) { MOSS_DEVICE_BACKEND(device); MOSS_CALL_VOID(wait_idle, device); }

const char* Moss_GetGPUDriver(uint32_t driver_index) {
    uint32_t visible_index = 0;
    auto visit = [&](const Moss_GPUBackend* backend) -> const char* {
        if (!backend) {
            return nullptr;
        }
        if (visible_index == driver_index) {
            return backend->name;
        }
        ++visible_index;
        return nullptr;
    };

#if defined(MOSS_USE_OPENGL) || defined(MOSS_GRAPHICS_OPENGL)
    if (const char* name = visit(Moss_GetOpenGLGPUBackend())) return name;
#endif
#if defined(MOSS_USE_OPENGLES) || defined(MOSS_GRAPHICS_OPENGLES)
    if (const char* name = visit(Moss_GetOpenGLESGPUBackend())) return name;
#endif
#if defined(MOSS_USE_VULKAN) || defined(MOSS_GRAPHICS_VULKAN)
    if (const char* name = visit(Moss_GetVulkanGPUBackend())) return name;
#endif
#if defined(MOSS_USE_DIRECTX12) || defined(MOSS_GRAPHICS_DIRECTX)
    if (const char* name = visit(Moss_GetD3D12GPUBackend())) return name;
#endif
#if defined(MOSS_USE_METAL) || defined(MOSS_GRAPHICS_METAL)
    if (const char* name = visit(Moss_GetMetalGPUBackend())) return name;
#endif

    if (visible_index == 0 && driver_index == 0) {
        return Moss_GetFallbackGPUBackend()->name;
    }
    return nullptr;
}

uint32_t Moss_GetNumGPUDrivers(void) {
    uint32_t count = 0;
#if defined(MOSS_USE_OPENGL) || defined(MOSS_GRAPHICS_OPENGL)
    if (Moss_GetOpenGLGPUBackend()) ++count;
#endif
#if defined(MOSS_USE_OPENGLES) || defined(MOSS_GRAPHICS_OPENGLES)
    if (Moss_GetOpenGLESGPUBackend()) ++count;
#endif
#if defined(MOSS_USE_VULKAN) || defined(MOSS_GRAPHICS_VULKAN)
    if (Moss_GetVulkanGPUBackend()) ++count;
#endif
#if defined(MOSS_USE_DIRECTX12) || defined(MOSS_GRAPHICS_DIRECTX)
    if (Moss_GetD3D12GPUBackend()) ++count;
#endif
#if defined(MOSS_USE_METAL) || defined(MOSS_GRAPHICS_METAL)
    if (Moss_GetMetalGPUBackend()) ++count;
#endif
    return count ? count : 1;
}

uint32_t Moss_GetGPUShaderFormats(Moss_GPUDevice* device) { MOSS_DEVICE_BACKEND(device); return backend ? backend->shader_formats : 0; }
const char* Moss_GetGPUDeviceDriver(Moss_GPUDevice* device) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(get_device_driver, nullptr, device); }
const Moss_GPUDeviceProperties* Moss_GetGPUDeviceProperties(Moss_GPUDevice* device) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(get_device_properties, nullptr, device); }
bool Moss_GPUSupportsProperties(Moss_GPUDevice* device) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(supports_properties, false, device); }
bool Moss_GPUSupportsShaderFormats(Moss_GPUDevice* device, uint32_t shader_format_mask) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(supports_shader_formats, false, device, shader_format_mask); }
bool Moss_GPUTextureSupportsFormat(Moss_GPUDevice* device, ETextureFormat format, ETextureUsage usage) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(texture_supports_format, false, device, format, usage); }
bool Moss_GPUTextureSupportsSampleCount(Moss_GPUDevice* device, ETextureFormat format, uint32_t sample_count) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(texture_supports_sample_count, sample_count == 1, device, format, sample_count); }
int Moss_GPUDeviceSupportsAsyncCompute(Moss_GPUDevice* device) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(supports_async_compute, 0, device); }

bool Moss_ClaimWindowForGPUDevice(Moss_GPUDevice* device, Moss_Window* window) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(claim_window, false, device, window); }
void Moss_ReleaseWindowFromGPUDevice(Moss_GPUDevice* device, Moss_Window* window) { MOSS_DEVICE_BACKEND(device); MOSS_CALL_VOID(release_window, device, window); }
bool Moss_WindowSupportsGPUPresentMode(Moss_Window* window, EGPUPresentMode present_mode) { const Moss_GPUBackend* backend = select_compiled_backend(); return MOSS_CALL_RET(window_supports_present_mode, false, window, present_mode); }
bool Moss_WindowSupportsGPUSwapchainComposition(Moss_Window* window) { const Moss_GPUBackend* backend = select_compiled_backend(); return MOSS_CALL_RET(window_supports_swapchain_composition, false, window); }
EGPUSwapchainResult Moss_AcquireGPUSwapchainTextureStatus(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture** texture) {
    MOSS_DEVICE_BACKEND(device);
    if (backend && backend->acquire_swapchain_texture_status) {
        return backend->acquire_swapchain_texture_status(device, window, texture);
    }
    return MOSS_CALL_RET(acquire_swapchain_texture, false, device, window, texture) ? EGPUSwapchainResult::SUCCESS : EGPUSwapchainResult::ERROR;
}
bool Moss_AcquireGPUSwapchainTexture(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture** texture) { return Moss_AcquireGPUSwapchainTextureStatus(device, window, texture) == EGPUSwapchainResult::SUCCESS; }
EGPUSwapchainResult Moss_WaitAndAcquireGPUSwapchainTextureStatus(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture** texture, uint64_t timeout_ns) {
    MOSS_DEVICE_BACKEND(device);
    if (backend && backend->wait_and_acquire_swapchain_texture_status) {
        return backend->wait_and_acquire_swapchain_texture_status(device, window, texture, timeout_ns);
    }
    return MOSS_CALL_RET(wait_and_acquire_swapchain_texture, false, device, window, texture, timeout_ns) ? EGPUSwapchainResult::SUCCESS : EGPUSwapchainResult::ERROR;
}
bool Moss_WaitAndAcquireGPUSwapchainTexture(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture** texture, uint64_t timeout_ns) { return Moss_WaitAndAcquireGPUSwapchainTextureStatus(device, window, texture, timeout_ns) == EGPUSwapchainResult::SUCCESS; }
bool Moss_WaitForGPUSwapchain(Moss_GPUDevice* device, Moss_Window* window, uint64_t timeout_ns) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(wait_for_swapchain, false, device, window, timeout_ns); }
bool Moss_SetGPUSwapchainParameters(Moss_GPUDevice* device, Moss_Window* window, uint32_t width, uint32_t height, EGPUPresentMode present_mode) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(set_swapchain_parameters, false, device, window, width, height, present_mode); }
EGPUSwapchainResult Moss_ResizeGPUSwapchain(Moss_GPUDevice* device, Moss_Window* window, uint32_t width, uint32_t height) {
    MOSS_DEVICE_BACKEND(device);
    if (backend && backend->resize_swapchain) {
        return backend->resize_swapchain(device, window, width, height);
    }
    return EGPUSwapchainResult::UNSUPPORTED;
}
ETextureFormat Moss_GetGPUSwapchainTextureFormat(Moss_GPUDevice* device, Moss_Window* window) { MOSS_DEVICE_BACKEND(device); return MOSS_CALL_RET(get_swapchain_texture_format, ETextureFormat::RGBA8, device, window); }
EGPUSwapchainResult Moss_PresentGPUSwapchainStatus(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture* texture) {
    MOSS_DEVICE_BACKEND(device);
    if (backend && backend->present_swapchain_status) {
        return backend->present_swapchain_status(device, window, texture);
    }
    return MOSS_CALL_RET(present_swapchain, false, device, window, texture) ? EGPUSwapchainResult::SUCCESS : EGPUSwapchainResult::ERROR;
}
bool Moss_PresentGPUSwapchain(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture* texture) { return Moss_PresentGPUSwapchainStatus(device, window, texture) == EGPUSwapchainResult::SUCCESS; }

uint32_t Moss_GPUTextureFormatTexelBlockSize(ETextureFormat format) { return texel_block_size(format); }

uint32_t Moss_CalculateGPUTextureFormatSize(ETextureFormat format, uint32_t width, uint32_t height, uint32_t depth) {
    const uint32_t block_size = texel_block_size(format);
    if (block_size == 0) {
        return 0;
    }

    if (is_compressed_format(format)) {
        const uint32_t blocks_x = (width + 3) / 4;
        const uint32_t blocks_y = (height + 3) / 4;
        return blocks_x * blocks_y * depth * block_size;
    }

    return width * height * depth * block_size;
}

ETextureFormat Moss_GetGPUTextureFormatFromPixelFormat(EPixelFormat pixel_format) {
    switch (pixel_format) {
        case EPixelFormat::R8: return ETextureFormat::R8;
        case EPixelFormat::RG8: return ETextureFormat::RG8;
        case EPixelFormat::RGB8: return ETextureFormat::RGB8;
        case EPixelFormat::RGBA8: return ETextureFormat::RGBA8;
        case EPixelFormat::SRGB8: return ETextureFormat::SRGB8;
        case EPixelFormat::SRGBA8: return ETextureFormat::SRGBA8;
        case EPixelFormat::DEPTH16: return ETextureFormat::Depth16;
        case EPixelFormat::DEPTH24: return ETextureFormat::Depth24;
        case EPixelFormat::DEPTH32F: return ETextureFormat::Depth32F;
        case EPixelFormat::DEPTH24_STENCIL8: return ETextureFormat::Depth24Stencil8;
        case EPixelFormat::DEPTH32F_STENCIL8: return ETextureFormat::Depth32FStencil8;
        case EPixelFormat::BGRA8:
        case EPixelFormat::UNKNOWN:
            return ETextureFormat::RGBA8;
    }

    return ETextureFormat::RGBA8;
}

EPixelFormat Moss_GetPixelFormatFromGPUTextureFormat(ETextureFormat texture_format) {
    switch (texture_format) {
        case ETextureFormat::R8: return EPixelFormat::R8;
        case ETextureFormat::RG8: return EPixelFormat::RG8;
        case ETextureFormat::RGB8: return EPixelFormat::RGB8;
        case ETextureFormat::RGBA8: return EPixelFormat::RGBA8;
        case ETextureFormat::SRGB8: return EPixelFormat::SRGB8;
        case ETextureFormat::SRGBA8: return EPixelFormat::SRGBA8;
        case ETextureFormat::Depth16: return EPixelFormat::DEPTH16;
        case ETextureFormat::Depth24: return EPixelFormat::DEPTH24;
        case ETextureFormat::Depth32F: return EPixelFormat::DEPTH32F;
        case ETextureFormat::Depth24Stencil8: return EPixelFormat::DEPTH24_STENCIL8;
        case ETextureFormat::Depth32FStencil8: return EPixelFormat::DEPTH32F_STENCIL8;
        default: return EPixelFormat::UNKNOWN;
    }
}

void Moss_SetGPUAllowedFramesInFlight(Moss_GPUDevice*, uint32_t) {}
void Moss_GDKResumeGPU(Moss_GPUDevice*) {}
void Moss_GDKSuspendGPU(Moss_GPUDevice*) {}
Moss_BindlessHandle Moss_BindlessRegisterTexture(Moss_GPUDevice* device, void* texture) {
    return register_bindless_resource(device, texture, BindlessResourceKind::Texture);
}

Moss_BindlessHandle Moss_BindlessRegisterBuffer(Moss_GPUDevice* device, Moss_GPUBuffer* buffer) {
    return register_bindless_resource(device, buffer, BindlessResourceKind::Buffer);
}

void Moss_BindlessUnregister(Moss_GPUDevice* device, Moss_BindlessHandle handle) {
    unregister_bindless_resource(device, handle);
}
Moss_RGTexture* Moss_RGPassReadTexture(Moss_RGPass*, Moss_RGTexture* texture, EResourceState) { return texture; }
Moss_RGTexture* Moss_RGPassWriteTexture(Moss_RGPass*, Moss_RGTexture* texture, EResourceState) { return texture; }
void Moss_TextureSetResidency(Moss_Texture*, EResidencyState) {}

#undef MOSS_DEVICE_BACKEND
#undef MOSS_CMD_BACKEND
#undef MOSS_CALL_VOID
#undef MOSS_CALL_RET










//                        MIT License
//
//                  Copyright (c) 2026 Toby

#ifndef MOSS_RENDERER_GPU_BACKEND_H
#define MOSS_RENDERER_GPU_BACKEND_H

#include <Moss/Moss_GPU.h>

struct Moss_GPUBackend {
    const char* name;
    Moss_GPUBackendType type;
    uint32_t shader_formats;
    uint32_t flags;

    Moss_GPUDevice* (*create_device)(const Moss_GPUDeviceDesc* desc, const Moss_GPUDeviceProperties* requested_properties);
    void (*destroy_device)(Moss_GPUDevice* device);

    const char* (*get_device_driver)(Moss_GPUDevice* device);
    const Moss_GPUDeviceProperties* (*get_device_properties)(Moss_GPUDevice* device);
    bool (*supports_properties)(Moss_GPUDevice* device);
    bool (*supports_shader_formats)(Moss_GPUDevice* device, uint32_t shader_format_mask);
    bool (*texture_supports_format)(Moss_GPUDevice* device, ETextureFormat format, ETextureUsage usage);
    bool (*texture_supports_sample_count)(Moss_GPUDevice* device, ETextureFormat format, uint32_t sample_count);
    int (*supports_async_compute)(Moss_GPUDevice* device);
    void (*wait_idle)(Moss_GPUDevice* device);

    bool (*claim_window)(Moss_GPUDevice* device, Moss_Window* window);
    void (*release_window)(Moss_GPUDevice* device, Moss_Window* window);
    bool (*window_supports_present_mode)(Moss_Window* window, EGPUPresentMode present_mode);
    bool (*window_supports_swapchain_composition)(Moss_Window* window);
    bool (*acquire_swapchain_texture)(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture** texture);
    EGPUSwapchainResult (*acquire_swapchain_texture_status)(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture** texture);
    bool (*wait_and_acquire_swapchain_texture)(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture** texture, uint64_t timeout_ns);
    EGPUSwapchainResult (*wait_and_acquire_swapchain_texture_status)(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture** texture, uint64_t timeout_ns);
    bool (*wait_for_swapchain)(Moss_GPUDevice* device, Moss_Window* window, uint64_t timeout_ns);
    bool (*set_swapchain_parameters)(Moss_GPUDevice* device, Moss_Window* window, uint32_t width, uint32_t height, EGPUPresentMode present_mode);
    EGPUSwapchainResult (*resize_swapchain)(Moss_GPUDevice* device, Moss_Window* window, uint32_t width, uint32_t height);
    ETextureFormat (*get_swapchain_texture_format)(Moss_GPUDevice* device, Moss_Window* window);
    bool (*present_swapchain)(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture* texture);
    EGPUSwapchainResult (*present_swapchain_status)(Moss_GPUDevice* device, Moss_Window* window, Moss_GPUTexture* texture);

    Moss_GPUCommandBuffer* (*acquire_command_buffer)(Moss_GPUDevice* device);
    Moss_GPUCommandBuffer* (*begin_command_buffer)(Moss_GPUDevice* device, ECommandQueue queue);
    void (*end_command_buffer)(Moss_GPUCommandBuffer* cmd);
    void (*submit_command_buffer)(Moss_GPUDevice* device, Moss_GPUCommandBuffer* cmd);
    Moss_GPUFence* (*submit_command_buffer_and_acquire_fence)(Moss_GPUDevice* device, Moss_GPUCommandBuffer* cmd);
    void (*cancel_command_buffer)(Moss_GPUCommandBuffer* cmd);

    void (*begin_render_pass)(Moss_GPUCommandBuffer* cmd, Moss_Framebuffer* framebuffer);
    void (*end_render_pass)(Moss_GPUCommandBuffer* cmd);
    void (*begin_compute_pass)(Moss_GPUCommandBuffer* cmd);
    void (*end_compute_pass)(Moss_GPUCommandBuffer* cmd);
    void (*begin_copy_pass)(Moss_GPUCommandBuffer* cmd);
    void (*end_copy_pass)(Moss_GPUCommandBuffer* cmd);

    Moss_GPUBuffer* (*create_buffer)(Moss_GPUDevice* device, const Moss_GPUBufferDesc* desc);
    void (*release_buffer)(Moss_GPUDevice* device, Moss_GPUBuffer* buffer);
    void (*upload_to_buffer)(Moss_GPUDevice* device, Moss_GPUBuffer* dst_buffer, const void* src_data, uint64_t size, uint64_t dst_offset);
    void (*download_from_buffer)(Moss_GPUDevice* device, Moss_GPUBuffer* src_buffer, void* dst_data, uint64_t size, uint64_t src_offset);
    void* (*map_buffer)(Moss_GPUDevice* device, Moss_GPUBuffer* buffer);
    void (*unmap_buffer)(Moss_GPUDevice* device, Moss_GPUBuffer* buffer);
    void (*set_buffer_name)(Moss_GPUDevice* device, Moss_GPUBuffer* buffer, const char* name);

    Moss_GPUTexture* (*create_texture)(Moss_GPUDevice* device, const Moss_GPUTextureCreateInfo* create_info);
    void (*release_texture)(Moss_GPUDevice* device, Moss_GPUTexture* texture);
    Moss_GPUTextureView* (*create_texture_view)(Moss_GPUDevice* device, const Moss_GPUTextureViewCreateInfo* create_info);
    void (*release_texture_view)(Moss_GPUDevice* device, Moss_GPUTextureView* texture_view);
    void (*upload_to_texture)(Moss_GPUDevice* device, Moss_GPUTexture* dst_texture, const Moss_GPUTextureTransferInfo* transfer_info);
    void (*download_from_texture)(Moss_GPUDevice* device, Moss_GPUTexture* src_texture, const Moss_GPUTextureRegion* src_region, void* dst_data, uint32_t dst_row_pitch, uint32_t dst_slice_pitch);
    void (*generate_mipmaps)(Moss_GPUCommandBuffer* cmd, Moss_GPUTexture* texture);
    void (*set_texture_name)(Moss_GPUDevice* device, Moss_GPUTexture* texture, const char* name);

    Moss_GPUTransferBuffer* (*create_transfer_buffer)(Moss_GPUDevice* device, const Moss_GPUTransferBufferCreateInfo* create_info);
    void (*release_transfer_buffer)(Moss_GPUDevice* device, Moss_GPUTransferBuffer* transfer_buffer);
    void* (*map_transfer_buffer)(Moss_GPUDevice* device, Moss_GPUTransferBuffer* transfer_buffer);
    void (*unmap_transfer_buffer)(Moss_GPUDevice* device, Moss_GPUTransferBuffer* transfer_buffer);

    Moss_GPUSampler* (*create_sampler)(Moss_GPUDevice* device, const Moss_GPUSamplerCreateInfo* create_info);
    void (*release_sampler)(Moss_GPUDevice* device, Moss_GPUSampler* sampler);
    Moss_GPUShader* (*create_shader)(Moss_GPUDevice* device, const Moss_GPUShaderCreateInfo* create_info);
    void (*release_shader)(Moss_GPUDevice* device, Moss_GPUShader* shader);
    Moss_ComputePipelineState* (*create_compute_pipeline)(Moss_GPUDevice* device, const Moss_GPUComputePipelineCreateInfo* create_info);
    void (*release_compute_pipeline)(Moss_GPUDevice* device, Moss_ComputePipelineState* pipeline);
    Moss_Framebuffer* (*create_framebuffer)(Moss_GPUDevice* device, const Moss_GPUFramebufferCreateInfo* create_info);
    void (*release_framebuffer)(Moss_GPUDevice* device, Moss_Framebuffer* framebuffer);
    Moss_ResourceSetLayout* (*create_resource_set_layout)(Moss_GPUDevice* device, const Moss_GPUResourceSetLayoutCreateInfo* create_info);
    void (*release_resource_set_layout)(Moss_GPUDevice* device, Moss_ResourceSetLayout* layout);
    Moss_ResourceSet* (*create_resource_set)(Moss_GPUDevice* device, const Moss_GPUResourceSetCreateInfo* create_info);
    void (*release_resource_set)(Moss_GPUDevice* device, Moss_ResourceSet* set);
    void (*update_resource_set)(Moss_GPUDevice* device, Moss_ResourceSet* set, const Moss_GPUResourceBinding* bindings, uint32_t binding_count);
    Moss_GPUQueryPool* (*create_query_pool)(Moss_GPUDevice* device, const Moss_GPUQueryPoolCreateInfo* create_info);
    void (*release_query_pool)(Moss_GPUDevice* device, Moss_GPUQueryPool* query_pool);
    bool (*get_query_results)(Moss_GPUDevice* device, Moss_GPUQueryPool* query_pool, uint32_t first_query, uint32_t query_count, void* data, uint64_t data_size, uint64_t stride, EGPUQueryResultFlags flags);
    Moss_Shader* (*create_shader_from_desc)(Moss_GPUDevice* device, const Moss_ShaderDesc* desc);
    void (*destroy_shader)(Moss_GPUDevice* device, Moss_Shader* shader);
    Moss_PipelineState* (*create_pipeline)(Moss_GPUDevice* device, const Moss_PipelineDesc* desc);
    void (*destroy_pipeline)(Moss_GPUDevice* device, Moss_PipelineState* pipeline);

    void (*copy_buffer_to_buffer)(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* src, Moss_GPUBuffer* dst, const Moss_GPUBufferRegion* src_region, const Moss_GPUBufferRegion* dst_region);
    void (*copy_buffer_to_texture)(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* src, Moss_GPUTexture* dst, const Moss_GPUBufferRegion* src_region, const Moss_GPUTextureRegion* dst_region);
    void (*copy_texture_to_buffer)(Moss_GPUCommandBuffer* cmd, Moss_GPUTexture* src, Moss_GPUBuffer* dst, const Moss_GPUTextureRegion* src_region, const Moss_GPUBufferRegion* dst_region);
    void (*copy_texture_to_texture)(Moss_GPUCommandBuffer* cmd, Moss_GPUTexture* src, Moss_GPUTexture* dst, const Moss_GPUTextureRegion* src_region, const Moss_GPUTextureRegion* dst_region);
    void (*blit_texture)(Moss_GPUCommandBuffer* cmd, Moss_Texture* src, Moss_Texture* dst, const Moss_GPUBlitRegion* region);
    void (*barrier_resources)(Moss_GPUCommandBuffer* cmd, const Moss_GPUBarrierInfo* barrier_info);
    void (*transition_buffer)(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* buffer, EResourceState old_state, EResourceState new_state);
    void (*transition_texture)(Moss_GPUCommandBuffer* cmd, Moss_GPUTexture* texture, EResourceState old_state, EResourceState new_state);

    void (*bind_pipeline)(Moss_GPUCommandBuffer* cmd, Moss_PipelineState* pipeline);
    void (*bind_compute_pipeline)(Moss_GPUCommandBuffer* cmd, Moss_ComputePipelineState* pipeline);
    void (*bind_resource_set)(Moss_GPUCommandBuffer* cmd, uint32_t set_index, Moss_ResourceSet* set);
    void (*bind_vertex_buffers)(Moss_GPUCommandBuffer* cmd, uint32_t first_binding, const Moss_GPUBufferBinding* bindings, uint32_t binding_count);
    void (*bind_index_buffer)(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* buffer, uint64_t offset, EGPUIndexType index_type);
    void (*bind_samplers)(Moss_GPUCommandBuffer* cmd, EShaderStage stage, uint32_t first_slot, Moss_GPUSampler* const* samplers, uint32_t sampler_count);
    void (*bind_textures)(Moss_GPUCommandBuffer* cmd, EShaderStage stage, uint32_t first_slot, Moss_GPUTexture* const* textures, uint32_t texture_count);
    void (*bind_uniform_buffers)(Moss_GPUCommandBuffer* cmd, EShaderStage stage, uint32_t first_slot, const Moss_GPUBufferBinding* bindings, uint32_t binding_count);
    void (*bind_storage_buffers)(Moss_GPUCommandBuffer* cmd, EShaderStage stage, uint32_t first_slot, const Moss_GPUStorageBufferReadWriteBinding* bindings, uint32_t binding_count);
    void (*bind_storage_textures)(Moss_GPUCommandBuffer* cmd, EShaderStage stage, uint32_t first_slot, const Moss_GPUStorageTextureReadWriteBinding* bindings, uint32_t binding_count);

    void (*draw)(Moss_GPUCommandBuffer* cmd, uint32_t vertex_count, uint32_t instance_count, uint32_t first_vertex, uint32_t first_instance);
    void (*draw_indexed)(Moss_GPUCommandBuffer* cmd, uint32_t index_count, uint32_t instance_count, uint32_t first_index, int32_t vertex_offset, uint32_t first_instance);
    void (*draw_indirect)(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* indirect_buffer, uint64_t offset, uint32_t draw_count, uint32_t stride);
    void (*draw_indexed_indirect)(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* indirect_buffer, uint64_t offset, uint32_t draw_count, uint32_t stride);
    void (*dispatch)(Moss_GPUCommandBuffer* cmd, uint32_t group_count_x, uint32_t group_count_y, uint32_t group_count_z);
    void (*dispatch_indirect)(Moss_GPUCommandBuffer* cmd, Moss_GPUBuffer* indirect_buffer, uint64_t offset);
    void (*reset_query_pool)(Moss_GPUCommandBuffer* cmd, Moss_GPUQueryPool* query_pool, uint32_t first_query, uint32_t query_count);
    void (*write_timestamp)(Moss_GPUCommandBuffer* cmd, Moss_GPUQueryPool* query_pool, uint32_t query_index);
    void (*begin_query)(Moss_GPUCommandBuffer* cmd, Moss_GPUQueryPool* query_pool, uint32_t query_index);
    void (*end_query)(Moss_GPUCommandBuffer* cmd, Moss_GPUQueryPool* query_pool, uint32_t query_index);

    void (*push_uniform_data)(Moss_GPUCommandBuffer* cmd, EShaderStage stage, uint32_t slot, const void* data, uint32_t size);
    void (*set_viewport)(Moss_GPUCommandBuffer* cmd, const Moss_GPUViewport* viewport);
    void (*set_scissor)(Moss_GPUCommandBuffer* cmd, const Moss_Rect* scissor);
    void (*set_blend_constants)(Moss_GPUCommandBuffer* cmd, float r, float g, float b, float a);
    void (*set_stencil_reference)(Moss_GPUCommandBuffer* cmd, uint32_t reference);
    void (*clear)(Moss_GPUCommandBuffer* cmd, float r, float g, float b, float a);

    void (*push_debug_group)(Moss_GPUCommandBuffer* cmd, const char* label);
    void (*pop_debug_group)(Moss_GPUCommandBuffer* cmd);
    void (*insert_debug_label)(Moss_GPUCommandBuffer* cmd, const char* label);

    bool (*query_fence)(Moss_GPUDevice* device, Moss_GPUFence* fence);
    void (*release_fence)(Moss_GPUDevice* device, Moss_GPUFence* fence);
    void (*wait_for_fences)(Moss_GPUDevice* device, Moss_GPUFence* const* fences, uint32_t fence_count, bool wait_all, uint64_t timeout_ns);
};

void Moss_RegisterGPUBackendDevice(Moss_GPUDevice* device, const Moss_GPUBackend* backend);
void Moss_UnregisterGPUBackendDevice(Moss_GPUDevice* device);
const Moss_GPUBackend* Moss_GetGPUBackend(Moss_GPUDevice* device);
const Moss_GPUBackend* Moss_GetGPUBackendFromCommandBuffer(Moss_GPUCommandBuffer* cmd);

const Moss_GPUBackend* Moss_GetOpenGLGPUBackend();
const Moss_GPUBackend* Moss_GetOpenGLESGPUBackend();
const Moss_GPUBackend* Moss_GetVulkanGPUBackend();
const Moss_GPUBackend* Moss_GetD3D12GPUBackend();
const Moss_GPUBackend* Moss_GetMetalGPUBackend();

void Moss_SetOpenGLGPUBackend(const Moss_GPUBackend* backend);
void Moss_SetOpenGLESGPUBackend(const Moss_GPUBackend* backend);
void Moss_SetVulkanGPUBackend(const Moss_GPUBackend* backend);
void Moss_SetD3D12GPUBackend(const Moss_GPUBackend* backend);
void Moss_SetMetalGPUBackend(const Moss_GPUBackend* backend);

const Moss_GPUBackend* Moss_GetFallbackGPUBackendTable();
Moss_GPUBackend* Moss_CreateNamedFallbackGPUBackend(Moss_GPUBackendType type, const char* name, uint32_t shader_formats, bool (*supports_shader_formats)(Moss_GPUDevice*, uint32_t));
bool Moss_GPUBackendHasRegisteredNativeTable(Moss_GPUBackendType type);

#endif // MOSS_RENDERER_GPU_BACKEND_H

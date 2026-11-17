#ifndef MOSS_RENDERER_INTERNAL_H
#define MOSS_RENDERER_INTERNAL_H

#include <Moss/Moss_GPU.h>
#include <Moss/Moss_Renderer.h>

// A small number of frame slots keeps swapchain image views and framebuffers
// alive until the GPU has completed the work which referenced them.
static constexpr uint32_t cFrameCount = 2;

struct Moss_RendererFrameResources {
    Moss_GPUFence* fence = nullptr;
    Moss_GPUTexture* acquired_swapchain_texture = nullptr; // borrowed
    Moss_GPUTextureView* backbuffer_view = nullptr;
    Moss_Framebuffer* backbuffer_framebuffer = nullptr;
};

// The public Moss_Renderer handle is backend-independent. Backend headers must
// not define this type or any opaque GPU handle declared by Moss_GPU.h. Each
// native driver owns its API objects through the GPU handle's private
// `backend_data` pointer; renderer state only refers to the common GPU API.
/*
struct Moss_Renderer {
    Moss_Window* window = nullptr;
    Moss_GPUDevice* gpu_device = nullptr;

    Moss_GPUCommandBuffer* command_buffer = nullptr;
    Moss_GPUTexture* acquired_swapchain_texture = nullptr; // borrowed alias
    Moss_GPUTextureView* backbuffer_view = nullptr;         // active-slot alias
    Moss_Framebuffer* backbuffer_framebuffer = nullptr;    // active-slot alias

    Moss_RendererFrameResources frame_resources[cFrameCount]{};
    uint32_t next_frame_slot = 0;
    uint32_t active_frame_slot = 0;

    uint32_t backbuffer_width = 0;
    uint32_t backbuffer_height = 0;
    uint32_t virtual_width = 0;
    uint32_t virtual_height = 0;
    EGPUPresentMode present_mode = EGPUPresentMode::VSYNC;

    Color clear_color = Color(0.0f, 0.0f, 0.0f, 1.0f);
    bool owns_gpu_device = false;
    bool frame_active = false;
    bool render_pass_active = false;
    bool swapchain_needs_resize = false;
    bool debug_enabled = false;
    EGPUSwapchainResult last_swapchain_result = EGPUSwapchainResult::SUCCESS;
};*/

#endif // MOSS_RENDERER_INTERNAL_H
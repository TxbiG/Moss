#include <Moss/Platform/Linux/wl_platform.h>

#include "wayland-client-protocol.h"
#include "xdg-shell-client-protocol.h"
#include "xdg-decoration-unstable-v1-client-protocol.h"
#include "viewporter-client-protocol.h"
#include "relative-pointer-unstable-v1-client-protocol.h"
#include "pointer-constraints-unstable-v1-client-protocol.h"
#include "xdg-activation-v1-client-protocol.h"
#include "idle-inhibit-unstable-v1-client-protocol.h"
#include "fractional-scale-v1-client-protocol.h"

#ifdef MOSS_USE_VULKAN
#include <vulkan/vulkan.h>
VkInstance m_instance = VK_NULL_HANDLE;
VkSurfaceKHR m_surface = VK_NULL_HANDLE;
#endif

extern librarywl wl;

static Moss_FramebufferResizeCallback g_framebufferResizeCallback = nullptr;
static Moss_WindowSizeCallback g_windowSizeCallback = nullptr;
static Moss_WindowPositionCallback g_windowPositionCallback = nullptr;
static Moss_WindowFocusCallback g_windowFocusCallback = nullptr;
static Moss_WindowContentScaleCallback g_windowContentScaleCallback = nullptr;
static Moss_WindowResizeCallback g_windowResizeCallback = nullptr;



Moss_Window* Moss_CreateWindow(const char* title, int width, int height, Moss_Monitor* monitor, Moss_Window* share) {
    Moss_Window* window = (Moss_Window*)calloc(1, sizeof(Moss_Window));
    if (!window) return NULL;

    window->width  = width;
    window->height = height;

    // Connect to Wayland display
    window->display = wl_display_connect(NULL);
    if (!window->display) return NULL;

    // Setup registry and collect global interfaces
    wl.registry = wl_display_get_registry(window->display);
    wl_registry_add_listener(wl.registry, &wl.registry_listener, &wl);
    wl_display_roundtrip(window->display);  // sync to ensure globals are ready

    // Create main Wayland surface
    window->surface = wl_compositor_create_surface(wl.compositor);
    if (!window->surface) return NULL;

    // Create xdg_surface and xdg_toplevel for window management
    window->xdg.surface = xdg_wm_base_get_xdg_surface(wl.wmBase, window->surface);
    if (!window->xdg.surface) return NULL;

    window->xdg.toplevel = xdg_surface_get_toplevel(window->xdg.surface);
    if (!window->xdg.toplevel) return NULL;

    // Set initial window properties
    xdg_toplevel_set_title(window->xdg.toplevel, title);
    xdg_surface_commit(window->xdg.surface);
    wl_display_roundtrip(window->display);  // wait for configure event

#ifdef MOSS_USE_OPENGL
    // Create native window wrapper for EGL
    window->egl.window = wl.egl.window_create(window->surface, width, height);
    if (!window->egl.window) return NULL;
#endif

    return window;
}


void Moss_PollEvents(void) {  if (wl.display) { wl_display_dispatch(wl.display); } }



void Moss_SetFramebufferResizeCallback(Moss_FramebufferResizeCallback callback) {

}

void Moss_SetWindowSizeCallback(Moss_WindowSizeCallback callback) {

}

void Moss_SetWindowResizeCallback(Moss_WindowResizeCallback callback) {

}

void Moss_SetWindowPositionCallback(Moss_WindowPositionCallback callback) {

}

void Moss_SetWindowFocusCallback(Moss_WindowFocusCallback callback) {

}

void Moss_SetWindowContentScaleCallback(Moss_WindowContentScaleCallback callback) {

}

#ifdef MOSS_USE_OPENGL
void Moss_MakeContextCurrent(Moss_Window* window) {

}

void Moss_SwapBuffers(Moss_Window* window) { eglSwapBuffers(wl.display, window->surface); }

void Moss_SwapBuffersInterval(int interval) {

}

void* Moss_GetProcAddress(const char* procname) {

}
#endif // MOSS_USE_OPENGL



#ifdef MOSS_USE_VULKAN
#include <wayland-client.h>
#include <wayland-egl.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <vulkan/vulkan.h>
#include <vulkan/vulkan_wayland.h>

VkResult Moss_CreateWindowSurface(Moss_Window* window, VkInstance vk_instance, const VkAllocationCallbacks* allocator, VkSurfaceKHR* surface) {
    if (!window || !surface || !window->surface || !window->display)
        return VK_ERROR_INITIALIZATION_FAILED;

    VkWaylandSurfaceCreateInfoKHR createInfo = {};
        createInfo.sType = VK_STRUCTURE_TYPE_WAYLAND_SURFACE_CREATE_INFO_KHR;
        createInfo.display = window->display;
        createInfo.surface = window->surface;

    return vkCreateWaylandSurfaceKHR(vk_instance, &createInfo, allocator, surface);
}

int Moss_VulkanSupported(void) {
    if (vulkanLib) return 1;

    vulkanLib = dlopen("libvulkan.so.1", RTLD_NOW | RTLD_LOCAL);
    if (!vulkanLib) return 0;

    vkGetInstanceProcAddr = (PFN_vkGetInstanceProcAddr)dlsym(vulkanLib, "vkGetInstanceProcAddr");
    return vkGetInstanceProcAddr != NULL;
}

void Moss_InitVulkanLoader(PFN_vkGetInstanceProcAddr loader) {
    if (loader) vkGetInstanceProcAddr = loader;
}

const char** Moss_GetRequiredInstanceExtensions(uint32_t* count) {
    static const char* extensions[] = {
        VK_KHR_SURFACE_EXTENSION_NAME,
        VK_KHR_WAYLAND_SURFACE_EXTENSION_NAME
    };

    if (count) *count = 2;
    return extensions;
}

void* Moss_GetInstanceProcAddress(VkInstance instance, const char* procname) {
    if (!vkGetInstanceProcAddr) return NULL;
    return (void*)vkGetInstanceProcAddr(instance, procname);
}

int Moss_GetPhysicalDevicePresentationSupport(Moss_Window& window, VkPhysicalDevice device, uint32_t queuefamily) {
    return vkGetPhysicalDeviceWaylandPresentationSupportKHR(device, queuefamily, window.display);
}

#endif // MOSS_USE_VULKAN
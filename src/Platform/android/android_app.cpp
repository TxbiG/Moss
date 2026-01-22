

#include <stdlib.h>
#include <string.h>
#include <android/native_window_jni.h>
#include <time.h>

// ------------------------------------------------------------
// Create / Terminate Mobile Window
Moss_Window* Moss_CreateMobileWindow(struct android_app* app) {
    if (!app || !app->window) return NULL;

    memset(&moss_android, 0, sizeof(Moss_AndroidState));
    moss_android.app = app;
    moss_android.running = true;
    moss_android.focused = true;
    moss_android.paused = false;
    moss_android.lastTimeNS = 0;
    moss_android.refreshRate = 60.0f;

    moss_android.vm = app->activity->vm;
    moss_android.env = app->activity->env;
    moss_android.activity = app->activity->clazz;

    moss_android.window = app->window;
    moss_android.windowReady = true;
    moss_android.width = ANativeWindow_getWidth(app->window);
    moss_android.height = ANativeWindow_getHeight(app->window);

    // --------------------------------------------------------
    // OpenGL ES
    // --------------------------------------------------------
#if defined(MOSS_GRAPHICS_OPENGLES_2) || defined(MOSS_GRAPHICS_OPENGLES_3)
    moss_android.eglDisplay = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    eglInitialize(moss_android.eglDisplay, NULL, NULL);

    EGLint configAttribs[] = {
        EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT, // Used for both 2.0 and 3.0
        EGL_BLUE_SIZE, 8,
        EGL_GREEN_SIZE, 8,
        EGL_RED_SIZE, 8,
        EGL_DEPTH_SIZE, 16,
        EGL_NONE
    };

    EGLConfig config;
    EGLint numConfigs;
    eglChooseConfig(moss_android.eglDisplay, configAttribs, &config, 1, &numConfigs);

    moss_android.eglSurface = eglCreateWindowSurface(moss_android.eglDisplay, config, moss_android.window, NULL);

#ifdef MOSS_GRAPHICS_OPENGLES_2
    EGLint contextAttribs[] = { EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE };
#elif defined(MOSS_GRAPHICS_OPENGLES_3)
    EGLint contextAttribs[] = { EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE };
#endif

    moss_android.eglContext = eglCreateContext(moss_android.eglDisplay, config, EGL_NO_CONTEXT, contextAttribs);
    eglMakeCurrent(moss_android.eglDisplay, moss_android.eglSurface, moss_android.eglSurface, moss_android.eglContext);
#endif

    return (Moss_Window*)&moss_android;
}



void Moss_Terminate_MobileWindow(void) {
    // --------------------------------------------------------
    // OpenGL ES cleanup
    // --------------------------------------------------------
#if defined(MOSS_GRAPHICS_OPENGLES_2) || defined(MOSS_GRAPHICS_OPENGLES_3)
    if (moss_android.eglDisplay != EGL_NO_DISPLAY) {
        eglMakeCurrent(moss_android.eglDisplay, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        if (moss_android.eglContext != EGL_NO_CONTEXT)
            eglDestroyContext(moss_android.eglDisplay, moss_android.eglContext);
        if (moss_android.eglSurface != EGL_NO_SURFACE)
            eglDestroySurface(moss_android.eglDisplay, moss_android.eglSurface);
        eglTerminate(moss_android.eglDisplay);

        moss_android.eglDisplay = EGL_NO_DISPLAY;
        moss_android.eglSurface = EGL_NO_SURFACE;
        moss_android.eglContext = EGL_NO_CONTEXT;
    }
#endif

    // --------------------------------------------------------
    // Release native window
    // --------------------------------------------------------
    if (moss_android.window) {
        ANativeWindow_release(moss_android.window);
        moss_android.window = NULL;
    }

    moss_android.windowReady = false;
    moss_android.running = false;

    if (moss_android.clipboardString) {
        free(moss_android.clipboardString);
        moss_android.clipboardString = NULL;
    }
}



#if defined(MOSS_GRAPHICS_OPENGL) || defined(MOSS_GRAPHICS_OPENGLES)

// ------------------------------------------------------------
// Make OpenGL ES context current
// ------------------------------------------------------------
void Moss_MakeContextCurrent(Moss_Window* window) {
    if (!window) return;

    Moss_AndroidState* state = (Moss_AndroidState*)window;
    if (state->eglDisplay != EGL_NO_DISPLAY && state->eglSurface != EGL_NO_SURFACE && state->eglContext != EGL_NO_CONTEXT) {
        eglMakeCurrent(state->eglDisplay, state->eglSurface, state->eglSurface, state->eglContext);
    }
}

// ------------------------------------------------------------
// Swap buffers
// ------------------------------------------------------------
void Moss_SwapBuffers() {
    if (moss_android.eglDisplay != EGL_NO_DISPLAY && moss_android.eglSurface != EGL_NO_SURFACE) {
        eglSwapBuffers(moss_android.eglDisplay, moss_android.eglSurface);
    }
}

// ------------------------------------------------------------
// Swap buffers with interval (vsync)
// ------------------------------------------------------------
void Moss_SwapBuffersInterval(int interval) {
    if (moss_android.eglDisplay != EGL_NO_DISPLAY) {
        eglSwapInterval(moss_android.eglDisplay, interval);
    }
    Moss_SwapBuffers();
}

// ------------------------------------------------------------
// Get OpenGL ES function pointer
// ------------------------------------------------------------
void* Moss_GetProcAddress(const char* procname) {
    if (!procname) return NULL;
    return (void*)eglGetProcAddress(procname);
}

#endif // MOSS_GRAPHICS_OPENGL || MOSS_GRAPHICS_OPENGLES

#ifdef MOSS_GRAPHICS_VULKAN
PFN_vkGetInstanceProcAddr my_vkGetInstanceProcAddr = NULL;

// On Android, the Vulkan loader is already linked via NDK
int Moss_VulkanSupported(void) {
    // vkGetInstanceProcAddr is always available in NDK
    my_vkGetInstanceProcAddr = &vkGetInstanceProcAddr;
    return my_vkGetInstanceProcAddr != NULL;
}

// Create Vulkan surface from ANativeWindow*
VkResult Moss_CreateWindowSurface(Moss_Window* window, VkInstance vk_instance, const VkAllocationCallbacks* allocator, VkSurfaceKHR* vk_surface) {
    if (!window || !window->window) return VK_ERROR_INITIALIZATION_FAILED;

    VkAndroidSurfaceCreateInfoKHR surfaceInfo = {};
    surfaceInfo.sType = VK_STRUCTURE_TYPE_ANDROID_SURFACE_CREATE_INFO_KHR;
    surfaceInfo.window = window->window;  // ANativeWindow*

    VkResult res = vkCreateAndroidSurfaceKHR(vk_instance, &surfaceInfo, allocator, vk_surface);
    return res;
}

// Get required instance extensions
const char** Moss_GetRequiredInstanceExtensions(uint32_t* count) {
    static const char* extensions[2];
    uint32_t extCount = 0;

    extensions[extCount++] = VK_KHR_SURFACE_EXTENSION_NAME;
    extensions[extCount++] = VK_KHR_ANDROID_SURFACE_EXTENSION_NAME;

    if (count) *count = extCount;
    return extensions;
}

// Wrapper for vkGetInstanceProcAddr
void* Moss_GetInstanceProcAddress(VkInstance instance, const char* procname) { if (!my_vkGetInstanceProcAddr) return NULL; return (void*)my_vkGetInstanceProcAddr(instance, procname); }

// On Android, all physical devices support presentation on any queue
int Moss_GetPhysicalDevicePresentationSupport(Moss_Window* window, VkPhysicalDevice device, uint32_t queuefamily) { return device != VK_NULL_HANDLE; }

#endif // MOSS_USE_VULKAN
#ifndef MOSS_PLATFORM_ANDROID_H
#define MOSS_PLATFORM_ANDROID_H

#include <Moss/Platform/platform_intern.h>

#include <stdbool.h>
#include <stdint.h>


#ifdef MOSS_GRAPHICS_OPENGLES_2 || MOSS_GRAPHICS_OPENGLES_3
#include <EGL/egl.h>
#endif

#ifdef MOSS_GRAPHICS_VULKAN
#include <vulkan/vulkan.h>
#include <vulkan/vulkan_android.h>
#endif

// ------------------------------------------------------------
// Android Global Platform State (SDL/GLFW equivalent)
// ------------------------------------------------------------

typedef struct Moss_Window {
    struct android_app* app;

    // Window
    struct ANativeWindow* window;
    int32_t width;
    int32_t height;
    bool windowReady;

    // App lifecycle
    bool running;
    bool focused;
    bool paused;

    // Input
    struct AInputQueue* inputQueue;

    // EGL (OpenGL ES only — Vulkan uses surface directly)
    EGLDisplay eglDisplay;
    EGLSurface eglSurface;
    EGLContext eglContext;

    // JNI
    JavaVM* vm;
    JNIEnv* env;
    jobject activity;

    // Timing
    int64_t lastTimeNS;
    float refreshRate;

    // Clipboard
    char* clipboardString;

} Moss_AndroidState;

// Global Android state (defined in .c)
extern Moss_AndroidState moss_android;

extern struct android_poll_source* source;
// ------------------------------------------------------------
// Lifecycle / Entry
// ------------------------------------------------------------
Moss_Window* Moss_CreateMobileWindow(struct android_app* app);
void Moss_Terminate_MobileWindow(void);
void Moss_Android_WindowResized(int32_t width, int32_t height);


void Moss_PollEvents(void);

// ------------------------------------------------------------
// android_native_app_glue callbacks
// ------------------------------------------------------------

void Moss_Android_HandleCmd(struct android_app* app, int32_t cmd);
int32_t Moss_Android_HandleInput(struct android_app* app, struct AInputEvent* event);

// ------------------------------------------------------------
// Input handling
// ------------------------------------------------------------

void Moss_Android_HandleKey(struct AInputEvent* event);
void Moss_Android_HandleTouch(struct AInputEvent* event);
void Moss_Android_HandleGamepad(struct AInputEvent* event);

// ------------------------------------------------------------
// JNI helpers
// ------------------------------------------------------------

JNIEnv* Moss_Android_AttachThread(void);
void Moss_Android_DetachThread(void);
jobject Moss_Android_GetActivity(void);

// ------------------------------------------------------------
// Time
// ------------------------------------------------------------

uint64_t Moss_Android_GetTicksNS(void);
void Moss_Android_Sleep(uint32_t ms);

// Camera

#endif // MOSS_PLATFORM_ANDROID_H
#include <Moss/Platform/Linux/x11_platform.h>

#include <X11/cursorfont.h>
#include <X11/Xmd.h>
#include <poll.h>


#include <X11/Xlib.h>
#include <X11/Xutil.h>

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <errno.h>
#include <assert.h>


#ifdef MOSS_GRAPHICS_VULKAN
#include <vulkan/vulkan.h>
VkInstance m_instance = VK_NULL_HANDLE;
VkSurfaceKHR m_surface = VK_NULL_HANDLE;
#endif // MOSS_GRAPHICS_VULKAN

extern libraryX11 x11;

static Moss_FramebufferResizeCallback g_framebufferResizeCallback = NULL;
static Moss_WindowSizeCallback g_windowSizeCallback = NULL;
static Moss_WindowPositionCallback g_windowPositionCallback = NULL;
static Moss_WindowFocusCallback g_windowFocusCallback = NULL;
static Moss_WindowContentScaleCallback g_windowContentScaleCallback = NULL;
static Moss_WindowResizeCallback g_windowResizeCallback = NULL;


Moss_Window* Moss_CreateWindow(const char* title, int width, int height, Moss_Monitor* monitor, Moss_Window* share) {
    Moss_Window* window = (Moss_Window*)calloc(1, sizeof(Moss_Window));
    if (!window) return NULL;

    window->width = width;
    window->height = height;

    // Use the shared global X11 display
    Display* dpy = x11.display;
    int screen = x11.screen;
    Window root = x11.root;

#ifdef MOSS_GRAPHICS_OPENGL
    int fbcount;
    int attribs[] = {
        GLX_X_RENDERABLE, True,
        GLX_DRAWABLE_TYPE, GLX_WINDOW_BIT,
        GLX_RENDER_TYPE, GLX_RGBA_BIT,
        GLX_X_VISUAL_TYPE, GLX_TRUE_COLOR,
        GLX_RED_SIZE, 8,
        GLX_GREEN_SIZE, 8,
        GLX_BLUE_SIZE, 8,
        GLX_ALPHA_SIZE, 8,
        GLX_DEPTH_SIZE, 24,
        GLX_STENCIL_SIZE, 8,
        GLX_DOUBLEBUFFER, True,
        None
    };

    GLXFBConfig* fbc = x11.glx.ChooseFBConfig(dpy, screen, attribs, &fbcount);
    if (!fbc) return NULL;

    XVisualInfo* vi = x11.glx.GetVisualFromFBConfig(dpy, fbc[0]);
    if (!vi) return NULL;

    window->visual = vi;
    window->colormap = XCreateColormap(dpy, root, vi->visual, AllocNone);

    XSetWindowAttributes swa;
    swa.colormap = window->colormap;
    swa.event_mask = ExposureMask | KeyPressMask | KeyReleaseMask | ButtonPressMask | ButtonReleaseMask | StructureNotifyMask;

    window->handle = XCreateWindow(dpy, root, 0, 0, width, height, 0, vi->depth, InputOutput, vi->visual,
                                   CWColormap | CWEventMask, &swa);

    XStoreName(dpy, window->handle, title);

    x11.WM_DELETE_WINDOW = XInternAtom(dpy, "WM_DELETE_WINDOW", False);
    XSetWMProtocols(dpy, window->handle, &x11.WM_DELETE_WINDOW, 1);

    window->glxWindow = x11.glx.CreateWindow(dpy, fbc[0], window->handle, NULL);

    int contextAttribs[] = {
        GLX_CONTEXT_MAJOR_VERSION_ARB, 4,
        GLX_CONTEXT_MINOR_VERSION_ARB, 3,
        GLX_CONTEXT_PROFILE_MASK_ARB, GLX_CONTEXT_CORE_PROFILE_BIT_ARB,
        None
    };

    window->glxContext = x11.glx.CreateContextAttribsARB(dpy, fbc[0],
        share ? share->glxContext : NULL, True, contextAttribs);

    if (!window->glxContext) return NULL;

    x11.glx.MakeCurrent(dpy, window->glxWindow, window->glxContext);

    XMapWindow(dpy, window->handle);
    XFlush(dpy);

    return window;
#endif // MOSS_GRAPHICS_OPENGL
}


void Moss_TerminateWindow(Moss_Window* window) {
    if (!window)
        return;

#ifdef MOSS_GRAPHICS_OPENGL
    if (window->glxContext) {
        glXMakeCurrent(window->display, None, NULL);
        glXDestroyContext(window->display, window->glxContext);
        window->glxContext = NULL;
    }

    if (window->glxWindow) {
        glXDestroyWindow(window->display, window->glxWindow);
        window->glxWindow = 0;
    }
#endif // MOSS_GRAPHICS_OPENGL

    if (window->xwindow) {
        XDestroyWindow(window->display, window->xwindow);
        window->xwindow = 0;
    }

    XFlush(window->display);
    free(window);
}

bool Moss_ShouldWindowClose(Moss_Window* window) { return window ? window->shouldClose : true; }












void Moss_PollGamepads()
{
    struct js_event e;

    for (int i = 0; i < 4; i++) {
        if (pads_fd[i] == -1) continue;

        while (read(pads_fd[i], &e, sizeof(e)) > 0) {
            switch (e.type & ~JS_EVENT_INIT) {

                case JS_EVENT_BUTTON:
                    io.pads[i].buttons[e.number] = (e.value ? 1 : 0);
                    break;

                case JS_EVENT_AXIS:
                    io.pads[i].axes[e.number] = e.value / 32767.0f;
                    break;
            }
        }
    }
}

void Moss_PollEvents(void) {
    memcpy(io.keys_prev, io.keys, sizeof(io.keys));
    memcpy(io.mouse_buttons_prev, io.mouse_buttons, sizeof(io.mouse_buttons));


    while (XPending(display)) {
        XEvent e;
        XNextEvent(display, &e);

        switch (e.type) {

            case KeyPress:
            case KeyRelease: {
                KeySym sym = XLookupKeysym(&e.xkey, 0);
                bool pressed = (e.type == KeyPress);
                if (sym < 256)
                    io.keys[sym] = pressed;
            } break;

            case ButtonPress:
            case ButtonRelease: {
                bool pressed = (e.type == ButtonPress);

                switch (e.xbutton.button) {
                    case Button1: io.mouse_buttons[0] = pressed; break;
                    case Button2: io.mouse_buttons[1] = pressed; break;
                    case Button3: io.mouse_buttons[2] = pressed; break;

                    case Button4: if (pressed) io.mouse_scroll = +1; break;
                    case Button5: if (pressed) io.mouse_scroll = -1; break;
                }
            } break;

            case MotionNotify:
                io.mouse_x = e.xmotion.x;
                io.mouse_y = e.xmotion.y;
                break;
        }
    }

    Moss_PollGamepads();
}

typedef struct {
    int x, y, w, h;
    const char* label;
    int code;
} Moss_Button;

static bool point_in_rect(int px, int py, Moss_Button* b)
{
    return px >= b->x && px <= b->x + b->w &&
           py >= b->y && py <= b->y + b->h;
}

bool Moss_CreateMessageBox(const char* title,
                           const char* message,
                           Moss_MessageBoxFlags flags,
                           Moss_Window* ownerWindow)
{
    Display* dpy = XOpenDisplay(NULL);
    if (!dpy) return false;

    int screen = DefaultScreen(dpy);
    Window root = RootWindow(dpy, screen);

    int win_w = 360;
    int win_h = 160;

    Window win = XCreateSimpleWindow(
        dpy, root,
        300, 300,
        win_w, win_h,
        1,
        BlackPixel(dpy, screen),
        WhitePixel(dpy, screen)
    );

    XSelectInput(dpy, win, ExposureMask | ButtonPressMask);
    XStoreName(dpy, win, title);

    Atom deleteAtom = XInternAtom(dpy, "WM_DELETE_WINDOW", False);
    XSetWMProtocols(dpy, win, &deleteAtom, 1);

    XMapWindow(dpy, win);

    // ----------------------------------------------------
    // Create buttons based on messagebox flag
    // ----------------------------------------------------
    Moss_Button buttons[3];
    int buttonCount = 0;

    auto add_btn = [&](const char* text, int code) {
        buttons[buttonCount].label = text;
        buttons[buttonCount].code  = code;
        buttonCount++;
    };

    switch (flags)
    {
        case Moss_MessageBoxFlags::ABORT_ENTRY_IGNORE:
            add_btn("Abort",   1);
            add_btn("Retry",   2);
            add_btn("Ignore",  3);
            break;

        case Moss_MessageBoxFlags::CANCEL_TRY_CONTINUE:
            add_btn("Cancel",    1);
            add_btn("Try Again", 2);
            add_btn("Continue",  3);
            break;

        case Moss_MessageBoxFlags::OK:
            add_btn("OK", 1);
            break;

        case Moss_MessageBoxFlags::OK_CANCEL:
            add_btn("OK",     1);
            add_btn("Cancel", 2);
            break;

        case Moss_MessageBoxFlags::RETRY_CANCEL:
            add_btn("Retry",  1);
            add_btn("Cancel", 2);
            break;

        case Moss_MessageBoxFlags::YES_NO:
            add_btn("Yes", 1);
            add_btn("No",  2);
            break;

        case Moss_MessageBoxFlags::YES_NO_CANCEL:
            add_btn("Yes",    1);
            add_btn("No",     2);
            add_btn("Cancel", 3);
            break;

        case Moss_MessageBoxFlags::HELP:
            add_btn("Help", 1);
            break;

        default:
            add_btn("OK", 1);
            break;
    }

    // Button placement
    int btn_w = 90;
    int btn_h = 30;
    int spacing = 10;

    int total_btn_w = buttonCount * btn_w + (buttonCount - 1) * spacing;
    int start_x = (win_w - total_btn_w) / 2;

    for (int i = 0; i < buttonCount; i++)
    {
        buttons[i].x = start_x + i * (btn_w + spacing);
        buttons[i].y = win_h - btn_h - 20;
        buttons[i].w = btn_w;
        buttons[i].h = btn_h;
    }

    // ----------------------------------------------------
    // Message box loop
    // ----------------------------------------------------
    int returnCode = 0;

    XEvent ev;
    GC gc = XCreateGC(dpy, win, 0, NULL);

    while (true)
    {
        XNextEvent(dpy, &ev);

        if (ev.type == Expose)
        {
            // Draw message
            XDrawString(dpy, win, gc, 20, 40, message, strlen(message));

            // Draw buttons
            for (int i = 0; i < buttonCount; i++)
            {
                Moss_Button* b = &buttons[i];

                XDrawRectangle(dpy, win, gc, b->x, b->y, b->w, b->h);

                int text_x = b->x + 10;
                int text_y = b->y + b->h / 2 + 5;

                XDrawString(dpy, win, gc, text_x, text_y,
                            b->label, strlen(b->label));
            }
        }

        else if (ev.type == ButtonPress)
        {
            int mx = ev.xbutton.x;
            int my = ev.xbutton.y;

            for (int i = 0; i < buttonCount; i++)
            {
                if (point_in_rect(mx, my, &buttons[i]))
                {
                    returnCode = buttons[i].code;
                    goto done;
                }
            }
        }

        else if (ev.type == ClientMessage)
        {
            if ((Atom)ev.xclient.data.l[0] == deleteAtom)
            {
                returnCode = 0;
                break;
            }
        }
    }

done:
    XFreeGC(dpy, gc);
    XDestroyWindow(dpy, win);
    XCloseDisplay(dpy);

    // ----------------------------------------------------
    // Convert return code into boolean
    // ----------------------------------------------------
    switch (flags)
    {
        case Moss_MessageBoxFlags::OK:
            return true;

        case Moss_MessageBoxFlags::OK_CANCEL:
            return (returnCode == 1);

        case Moss_MessageBoxFlags::YES_NO:
            return (returnCode == 1); // Yes

        case Moss_MessageBoxFlags::RETRY_CANCEL:
            return (returnCode == 1); // Retry

        case Moss_MessageBoxFlags::CANCEL_TRY_CONTINUE:
            return (returnCode == 3); // Continue

        case Moss_MessageBoxFlags::ABORT_ENTRY_IGNORE:
            return (returnCode == 2); // Retry

        case Moss_MessageBoxFlags::YES_NO_CANCEL:
            return (returnCode == 1); // Yes

        default:
            return true;
    }
}


void Moss_SetWindowTitle(Moss_Window* window, const char* title) {}
void Moss_SetWindowIcon(Moss_Window* window, Moss_Image image) {}
void Moss_CloseWindow(Moss_Window* window) {}

void Moss_SetFramebufferResizeCallback(Moss_FramebufferResizeCallback callback) { g_framebufferResizeCallback = callback; }
void Moss_SetWindowSizeCallback(Moss_WindowSizeCallback callback) { g_windowSizeCallback = callback; }
void Moss_SetWindowResizeCallback(Moss_WindowResizeCallback callback) { g_windowResizeCallback = callback; }
void Moss_SetWindowPositionCallback(Moss_WindowPositionCallback callback) { g_windowPositionCallback = callback; }
void Moss_SetWindowFocusCallback(Moss_WindowFocusCallback callback) { g_windowFocusCallback = callback; }
void Moss_SetWindowContentScaleCallback(Moss_WindowContentScaleCallback callback) { g_windowContentScaleCallback = callback; }


#ifdef MOSS_GRAPHICS_OPENGL
void Moss_MakeContextCurrent(Moss_Window* window) {
    if (!window) { glXMakeCurrent(x11.display, None, NULL); return; }
    glXMakeCurrent(window->display, window->glxWindow, window->glxContext);
}

void Moss_SwapBuffers(Moss_Window* window) { x11.glx.SwapBuffers(x11.display, window->glxWindow); }

void Moss_SwapBuffersInterval(int interval) {
    // Try GLX EXT_swap_control
    if (glXSwapIntervalEXT) {
        glXSwapIntervalEXT(x11.display, DefaultRootWindow(x11.display), interval);
        return;
    }

    // Try GLX MESA_swap_control
    if (glXSwapIntervalMESA) {
        glXSwapIntervalMESA(interval);
        return;
    }

    // Try GLX SGI_swap_control
    if (glXSwapIntervalSGI) {
        glXSwapIntervalSGI(interval);
        return;
    }
}

void* Moss_GetProcAddress(const char* procname) {
    // Try GLX function loader
    void* p = (void*)glXGetProcAddressARB((const GLubyte*)procname);
    if (p) return p;

    // Fallback to dlsym
    void* handle = dlopen(NULL, RTLD_LAZY);
    if (handle) {
        p = dlsym(handle, procname);
        dlclose(handle);
    }

    return p;
}
#endif // MOSS_GRAPHICS_OPENGL

#ifdef MOSS_GRAPHICS_VULKAN
#include <dlfcn.h>
#include <vulkan/vulkan.h>
#include <vulkan/vulkan_xlib.h>

// Forward declarations for global Vulkan loader
static void* vulkanLib = nullptr;
static PFN_vkGetInstanceProcAddr vkGetInstanceProcAddr = nullptr;


VkResult Moss_CreateWindowSurface(Moss_Window* window, VkInstance instance, const VkAllocationCallbacks* allocator, VkSurfaceKHR* surface) {
    if (!window || !surface) return VK_ERROR_INITIALIZATION_FAILED;

    VkXlibSurfaceCreateInfoKHR surfaceCreateInfo = {};
    surfaceCreateInfo.sType = VK_STRUCTURE_TYPE_XLIB_SURFACE_CREATE_INFO_KHR;
    surfaceCreateInfo.dpy = x11.display;
    surfaceCreateInfo.window = window->handle;

    return vkCreateXlibSurfaceKHR(instance, &surfaceCreateInfo, allocator, surface);
}

int Moss_VulkanSupported(void) {
    if (vulkanLib) return 1;

    vulkanLib = dlopen("libvulkan.so.1", RTLD_NOW | RTLD_LOCAL);
    if (!vulkanLib) return 0;

    vkGetInstanceProcAddr = (PFN_vkGetInstanceProcAddr)dlsym(vulkanLib, "vkGetInstanceProcAddr");
    return vkGetInstanceProcAddr != NULL;
}

void Moss_InitVulkanLoader(PFN_vkGetInstanceProcAddr loader) { if (loader) vkGetInstanceProcAddr = loader; }

void Moss_ShutdownVulkanLoader() {
    if (vulkanLib) {
        dlclose(vulkanLib);
        vulkanLib = nullptr;
        vkGetInstanceProcAddr = nullptr;
    }
}

void* Moss_GetInstanceProcAddress(VkInstance instance, const char* procname) {
    if (!vkGetInstanceProcAddr) return nullptr; return (void*)vkGetInstanceProcAddr(instance, procname);
}

const char** Moss_GetRequiredInstanceExtensions(uint32_t* count) {
    static const char* extensions[] = { VK_KHR_SURFACE_EXTENSION_NAME, VK_KHR_XLIB_SURFACE_EXTENSION_NAME };

    if (count) *count = 2;
    return extensions;
}

int Moss_GetPhysicalDevicePresentationSupport(Moss_Window& window, VkPhysicalDevice device, uint32_t queueFamily) {
    return vkGetPhysicalDeviceXlibPresentationSupportKHR(device, queueFamily, x11.display, DefaultVisual(x11.display, x11.screen));
}

#endif // MOSS_GRAPHICS_VULKAN
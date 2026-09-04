#include "win32_platform.h"
#include <wingdi.h>

// TODO: convert YUYV (Linux) or RGB32 (Windows) into consistent format (like RGB24)
// Video capture frames report their native pixel layout through Moss_VideoCaptureFrame::format.

HINSTANCE hInstance;
HWND handle;
static bool isRunning = true;

#ifdef MOSS_USE_OPENGL

typedef HGLRC (WINAPI *PFN_wglCreateContext)(HDC hdc);
typedef BOOL  (WINAPI *PFN_wglDeleteContext)(HGLRC hglrc);
typedef PROC  (WINAPI *PFN_wglGetProcAddress)(LPCSTR lpszProc);
typedef HDC   (WINAPI *PFN_wglGetCurrentDC)(void);
typedef HGLRC (WINAPI *PFN_wglGetCurrentContext)(void);
typedef BOOL  (WINAPI *PFN_wglMakeCurrent)(HDC hdc, HGLRC hglrc);
typedef BOOL  (WINAPI *PFN_wglShareLists)(HGLRC hglrc1, HGLRC hglrc2);


typedef struct _libraryWGL
{
    HINSTANCE                           instance;
    PFN_wglCreateContext                CreateContext;
    PFN_wglDeleteContext                DeleteContext;
    PFN_wglGetProcAddress               GetProcAddress;
    PFN_wglGetCurrentDC                 GetCurrentDC;
    PFN_wglGetCurrentContext            GetCurrentContext;
    PFN_wglMakeCurrent                  MakeCurrent;
    PFN_wglShareLists                   ShareLists;

    PFNWGLSWAPINTERVALEXTPROC           SwapIntervalEXT;
    PFNWGLGETPIXELFORMATATTRIBIVARBPROC GetPixelFormatAttribivARB;
    PFNWGLGETEXTENSIONSSTRINGEXTPROC    GetExtensionsStringEXT;
    PFNWGLGETEXTENSIONSSTRINGARBPROC    GetExtensionsStringARB;
    PFNWGLCREATECONTEXTATTRIBSARBPROC   CreateContextAttribsARB;
    bool                            EXT_swap_control;
    bool                            EXT_colorspace;
    bool                            ARB_multisample;
    bool                            ARB_framebuffer_sRGB;
    bool                            EXT_framebuffer_sRGB;
    bool                            ARB_pixel_format;
    bool                            ARB_create_context;
    bool                            ARB_create_context_profile;
    bool                            EXT_create_context_es2_profile;
    bool                            ARB_create_context_robustness;
    bool                            ARB_create_context_no_error;
    bool                            ARB_context_flush_control;
} _libraryWGL;

#define WGL_DRAW_TO_WINDOW_ARB           0x2001
#define WGL_SUPPORT_OPENGL_ARB           0x2010
#define WGL_DOUBLE_BUFFER_ARB            0x2011
#define WGL_SWAP_METHOD_ARB              0x2007
#define WGL_SWAP_COPY_ARB                0x2029
#define WGL_PIXEL_TYPE_ARB               0x2013
#define WGL_TYPE_RGBA_ARB                0x202B
#define WGL_ACCELERATION_ARB             0x2003
#define WGL_FULL_ACCELERATION_ARB        0x2027
#define WGL_COLOR_BITS_ARB               0x2014
#define WGL_ALPHA_BITS_ARB               0x201B
#define WGL_DEPTH_BITS_ARB               0x2022

#define WGL_CONTEXT_MAJOR_VERSION_ARB    0x2091
#define WGL_CONTEXT_MINOR_VERSION_ARB    0x2092
#define WGL_CONTEXT_PROFILE_MASK_ARB     0x9126
#define WGL_CONTEXT_CORE_PROFILE_BIT_ARB 0x00000001
#define WGL_CONTEXT_FLAGS_ARB            0x2094
#define WGL_CONTEXT_DEBUG_BIT_ARB        0x00000001

typedef HGLRC (WINAPI *PFNWGLCREATECONTEXTATTRIBSARBPROC)(HDC hDC, HGLRC hShareContext, const int *attribList);
typedef BOOL (WINAPI *PFNWGLCHOOSEPIXELFORMATARBPROC)(HDC hdc, const int *piAttribIList, const FLOAT *pfAttribFList, UINT nMaxFormats, int *piFormats, UINT *nNumFormats);
#endif // MOSS_USE_OPENGL
// Declare the function pointer type and variable (you can put these in your internal header)
#ifndef PFNWGLSWAPINTERVALEXTPROC
typedef BOOL (APIENTRY *PFNWGLSWAPINTERVALEXTPROC)(int interval);
static PFNWGLSWAPINTERVALEXTPROC wglSwapIntervalEXT = NULL;

#endif // PFNWGLSWAPINTERVALEXTPROC

static HDC dc;

static wchar_t g_highSurrogate = 0;

#ifdef MOSS_USE_VULKAN
#include <vulkan/vulkan.h>
VkInstance m_instance = VK_NULL_HANDLE;
VkSurfaceKHR m_surface = VK_NULL_HANDLE;
#endif
#ifdef MOSS_USE_DIRECTX
#include <d3d11.h>
ID3D11Device* device;
ID3D11DeviceContext* context;
IDXGISwapChain* swapChain;
#endif

RECT rect;


static void RegisterRawMouse(HWND hWnd)
{
    RAWINPUTDEVICE rid = {0};
    rid.usUsagePage = 0x01;  // Generic desktop controls
    rid.usUsage     = 0x02;  // Mouse
    rid.dwFlags     = RIDEV_INPUTSINK; // receive even when not focused
    rid.hwndTarget  = hWnd;
    RegisterRawInputDevices(&rid,1,sizeof(rid));
}

struct Moss_Pointer {
    UINT32 id = 0;
    bool down = false;
    float x = 0, y = 0;
    float pressure = 0, tilt_x = 0, tilt_y = 0, rotation = 0;
};

static std::vector<Moss_Pointer> g_pointers;

Moss_Pointer* FindOrCreatePointer(UINT32 id, POINTER_INPUT_TYPE /*type*/) {
    for (auto& p : g_pointers) if (p.id == id) return &p;
    g_pointers.push_back({ id });
    return &g_pointers.back();
}
void RemovePointer(UINT32 id) {
    g_pointers.erase(std::remove_if(g_pointers.begin(), g_pointers.end(),
        [id](const Moss_Pointer& p) { return p.id == id; }), g_pointers.end());
}

static Moss_FramebufferResizeCallback g_framebufferResizeCallback = nullptr;
static Moss_WindowSizeCallback g_windowSizeCallback = nullptr;
static Moss_WindowPositionCallback g_windowPositionCallback = nullptr;
static Moss_WindowFocusCallback g_windowFocusCallback = nullptr;
static Moss_WindowContentScaleCallback g_windowContentScaleCallback = nullptr;
static Moss_WindowResizeCallback g_windowResizeCallback = nullptr;

int g_width;
int g_height;
static float g_mouseWheelX = 0.0f;
static float g_mouseWheelY = 0.0f;


static void QueueTextCharacter(wchar_t value)
{
    if (value >= 0xD800 && value <= 0xDBFF) {
        g_highSurrogate = value;
        return;
    }

    uint32_t codepoint = static_cast<uint32_t>(value);
    if (value >= 0xDC00 && value <= 0xDFFF && g_highSurrogate) {
        codepoint = 0x10000u + ((static_cast<uint32_t>(g_highSurrogate) - 0xD800u) << 10u)
            + (static_cast<uint32_t>(value) - 0xDC00u);
    }
    g_highSurrogate = 0;
    if (codepoint != 0u) g_textInput.push_back(codepoint);
}

static LRESULT CALLBACK WndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	PAINTSTRUCT ps;

	switch (message)
	{
    case WM_CLOSE:
        isRunning = false;
        DestroyWindow(hWnd);
        return 0;
    case WM_DESTROY:
		PostQuitMessage(0);
		break;
    case WM_SIZE: {
        const int width = LOWORD(lParam);
        const int height = HIWORD(lParam);
        if (g_framebufferResizeCallback) g_framebufferResizeCallback(width, height);
        if (g_windowSizeCallback) g_windowSizeCallback(width, height);
        g_width = width;
        g_height = height;
        return 0;
    }
    case WM_SETFOCUS:
        if (g_windowFocusCallback) g_windowFocusCallback(true);
        return 0;
    case WM_KILLFOCUS:
        if (g_windowFocusCallback) g_windowFocusCallback(false);
        return 0;
    case WM_CHAR:
        QueueTextCharacter(static_cast<wchar_t>(wParam));
        return 0;
    case WM_UNICHAR:
        if (wParam == UNICODE_NOCHAR) return TRUE;
        g_textInput.push_back(static_cast<uint32_t>(wParam));
        return 0;
    case WM_MOUSEWHEEL:
        g_mouseWheelY += static_cast<short>(HIWORD(wParam)) / static_cast<float>(WHEEL_DELTA);
        return 0;
    case WM_MOUSEHWHEEL:
        g_mouseWheelX += static_cast<short>(HIWORD(wParam)) / static_cast<float>(WHEEL_DELTA);
        return 0;
    case WM_INPUT: {
        UINT dwSize = 0;
        GetRawInputData((HRAWINPUT)lParam, RID_INPUT, nullptr, &dwSize, sizeof(RAWINPUTHEADER));

        std::vector<uint8_t> lpb(dwSize);
        if (GetRawInputData((HRAWINPUT)lParam, RID_INPUT, lpb.data(), &dwSize, sizeof(RAWINPUTHEADER)) == dwSize) {
            RAWINPUT* raw = (RAWINPUT*)lpb.data();
            if (raw->header.dwType == RIM_TYPEHID) { HandleHIDInput(raw); } }
        break;
    }

    case WM_MOUSEWHEEL:
        g_frame.wheel += (short)HIWORD(wParam) / (float)WHEEL_DELTA;
        return 0;
    case WM_INPUT: {
        UINT size = 0;
        GetRawInputData(reinterpret_cast<HRAWINPUT>(lParam), RID_INPUT, nullptr, &size, sizeof(RAWINPUTHEADER));
        std::vector<uint8_t> data(size);
        if (size != 0 && GetRawInputData(reinterpret_cast<HRAWINPUT>(lParam), RID_INPUT, data.data(), &size, sizeof(RAWINPUTHEADER)) == size) {
            RAWINPUT* raw = reinterpret_cast<RAWINPUT*>(data.data());
            if (raw->header.dwType == RIM_TYPEHID) {
                HandleHIDInput(raw);
            } else if (raw->header.dwType == RIM_TYPEMOUSE) {
                g_frame.mx += raw->data.mouse.lLastX;
                g_frame.my += raw->data.mouse.lLastY;
            }
        }
        return 0;
    }
    case WM_POINTERDOWN:
    case WM_POINTERUPDATE:
    case WM_POINTERUP: {
        const UINT32 pointerId = GET_POINTERID_WPARAM(wParam);
        POINTER_INPUT_TYPE type;
        if (!GetPointerType(pointerId, &type)) return 0;
        POINTER_INFO pointerInfo = {};
        if (!GetPointerInfo(pointerId, &pointerInfo)) return 0;
        Moss_Pointer* pointer = FindOrCreatePointer(pointerId, type);
        pointer->down = message != WM_POINTERUP;
        POINT point = pointerInfo.ptPixelLocation;
        ScreenToClient(hWnd, &point);
        pointer->x = static_cast<float>(point.x);
        pointer->y = static_cast<float>(point.y);

        if (type == PT_PEN) {
            POINTER_PEN_INFO pen = {};
            if (GetPointerPenInfo(pointerId, &pen)) {
                pointer->pressure = pen.pressure / 1024.0f;
                pointer->tilt_x = pen.tiltX / 90.0f;
                pointer->tilt_y = pen.tiltY / 90.0f;
                pointer->rotation = static_cast<float>(pen.rotation);
                pointer->eraser = (pen.penFlags & PEN_FLAG_ERASER) != 0;
            }
        }

        if (message == WM_POINTERUP) RemovePointer(pointerId);

        return 0;
    }

	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
}


Moss_Window* Moss_CreateWindow(const char* title, int width, int height, Moss_Monitor* monitor, Moss_Window* share) {
    hInstance = GetModuleHandleA(nullptr);
    WNDCLASS wc = {0};
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInstance;
    wc.hIcon = LoadIcon(hInstance, IDI_APPLICATION); 
    wc.hCursor = LoadCursor(NULL, IDC_ARROW);
    wc.lpszClassName = title;
    if (!RegisterClassA(&wc)) { MOSS_ERROR("Failed to register window class"); return NULL; }

    Moss_Window* window = (Moss_Window*) malloc(sizeof(Moss_Window));

    #if defined(MOSS_GRAPHICS_DIRECTX) || defined(MOSS_GRAPHICS_VULKAN)
    if (!window) { return NULL; }
    HWND handle = CreateWindowExA(0, wc.lpszClassName, title, WS_OVERLAPPEDWINDOW, 100, 100, width, height, NULL, NULL, hInstance, NULL);
    window->handle = handle;
    #endif // MOSS_USE_DIRECTX

    #ifdef MOSS_USE_OPENGL
    HWND fakeHWND = CreateWindowExA(0, title, title, WS_OVERLAPPEDWINDOW, 100, 100, width, height, NULL, NULL, hInstance, NULL);
    if (!fakeHWND) { MOSS_ERROR("Failed to create fake window"); return NULL; }
    HDC fakeDC = GetDC(fakeHWND);
    if (!fakeDC) { MOSS_ERROR("Failed to get fake HDC"); return NULL; }

    PIXELFORMATDESCRIPTOR fakePfd = {0};
    fakePfd.nSize = sizeof(fakePfd);
    fakePfd.nVersion = 1;
    fakePfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    fakePfd.iPixelType = PFD_TYPE_RGBA;
    fakePfd.cColorBits = 32;
    fakePfd.cAlphaBits = 8;
    fakePfd.cDepthBits = 24;

    int fakePixelFormat = ChoosePixelFormat(fakeDC, &fakePfd);
    if (!fakePixelFormat) { return NULL; }
    if (!SetPixelFormat(fakeDC, fakePixelFormat, &fakePfd)) {
        printf("Failed to set pixel format for fake DC");
        return NULL;
    }

    HGLRC fakeRC = wglCreateContext(fakeDC);
    if (!fakeRC) {
        printf("Failed to create or set fake RC");
        ReleaseDC(fakeHWND, fakeDC);
        DestroyWindow(fakeHWND);
        return NULL;
    }

    wglMakeCurrent(fakeDC, fakeRC);

    PFNWGLCHOOSEPIXELFORMATARBPROC wglChoosePixelFormatARB =
    (PFNWGLCHOOSEPIXELFORMATARBPROC)wglGetProcAddress("wglChoosePixelFormatARB");
    PFNWGLCREATECONTEXTATTRIBSARBPROC wglCreateContextAttribsARB =
    (PFNWGLCREATECONTEXTATTRIBSARBPROC)wglGetProcAddress("wglCreateContextAttribsARB");
    if (!wglCreateContextAttribsARB || !wglChoosePixelFormatARB) {
        MOSS_ERROR("Failed to load OpenGL functions");
        return NULL;
    }

    // Clean up the take stuff
    wglDeleteContext(fakeRC);
    ReleaseDC(fakeHWND, fakeDC);

    // Can't reuse the same (Device)Context, 
    // because we already called "SetPixelFormat"
    DestroyWindow(fakeHWND);

    // Real init for opengl window
    if (!window) { return NULL; }

    RECT borderRect = {0};
    AdjustWindowRectEx(&borderRect, WS_OVERLAPPEDWINDOW, 0, 0);
    width += borderRect.right - borderRect.left;
    height += borderRect.bottom - borderRect.top;

    window->handle = CreateWindowExA(0, wc.lpszClassName, title, WS_OVERLAPPEDWINDOW, 100, 100, width, height, NULL, NULL, wc.hInstance, NULL);
    if (!window->handle) {
        MOSS_ERROR("Failed to create real window");
        return NULL;
    }

    dc = GetDC(window->handle);
    if (!dc) { MOSS_ERROR("Failed to getDC"); return NULL; }

    // Set attribs only once here
    const int pixelAttribs[] =
    {
      WGL_DRAW_TO_WINDOW_ARB, 1,
      WGL_SUPPORT_OPENGL_ARB, 1,
      WGL_DOUBLE_BUFFER_ARB,  1,
      WGL_SWAP_METHOD_ARB,    WGL_SWAP_COPY_ARB,
      WGL_PIXEL_TYPE_ARB,     WGL_TYPE_RGBA_ARB,
      WGL_ACCELERATION_ARB,   WGL_FULL_ACCELERATION_ARB,
      WGL_COLOR_BITS_ARB,     32,
      WGL_ALPHA_BITS_ARB,     8,
      WGL_DEPTH_BITS_ARB,     24,
      0 // Terminate with 0, otherwise OpenGL will throw an Error!
    };

    UINT numPixelFormats;
    int pixelFormat = 0;
    if(!wglChoosePixelFormatARB(dc, pixelAttribs,
                                0, // Float List
                                1, // Max Formats
                                &pixelFormat,
                                &numPixelFormats))
    {
      MOSS_ERROR(0, "Failed to wglChoosePixelFormatARB");
      return NULL;
    }

    PIXELFORMATDESCRIPTOR pfd = {0};
    DescribePixelFormat(dc, pixelFormat, sizeof(PIXELFORMATDESCRIPTOR), &pfd);

    if(!SetPixelFormat(dc, pixelFormat, &pfd))
    {
      MOSS_ERROR(0, "Failed to SetPixelFormat");
      return NULL;
    }

    const int contextAttribs[] = {
      WGL_CONTEXT_MAJOR_VERSION_ARB, 4,
      WGL_CONTEXT_MINOR_VERSION_ARB, 3,
      WGL_CONTEXT_PROFILE_MASK_ARB, WGL_CONTEXT_CORE_PROFILE_BIT_ARB,
      WGL_CONTEXT_FLAGS_ARB, WGL_CONTEXT_DEBUG_BIT_ARB,
      0 // Terminate the Array
    };

    HGLRC rc = wglCreateContextAttribsARB(dc, 0, contextAttribs);
    if(!rc) {
      MOSS_ERROR(0, "Failed to crate Render Context for OpenGL");
      return NULL;
    }

    if(!wglMakeCurrent(dc, rc)) {
      MOSS_ERROR(0, "Faield to wglMakeCurrent");
      return NULL;
    }
    #endif // MOSS_USE_OPENGL

    window->width = width;
    window->height = height;

    g_width = width;
    g_height = height;

    RegisterRawMouse(window->handle);

    ShowWindow(window->handle, SW_SHOW);
    io = {};

    return window;
}

/*! @brief Sets window as current.
*  @ingroup window
*/
void Moss_PollEvents(void) {
    MSG msg;
    while (PeekMessage(&msg, nullptr, 0, 0, PM_REMOVE) > 0) {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }
    Input_Poll(&io);
}

void platform_set_vsync(bool vSync) { wglSwapIntervalEXT(vSync); }

void Moss_TerminateWindow(Moss_Window* window) { if (!window) return; if (IsWindow(window->handle)) { DestroyWindow(window->handle); } free(window);  }

bool Moss_ShouldWindowClose(Moss_Window* window) { return !isRunning; }
void Moss_CloseWindow() { isRunning = false; }


/*! @brief Change Window Mode. e.g. MINAMIZED, MAXIMIZED, FULLSCREEN, BORDERLESS. @param X X. @ingroup window */
void setWindowMode(int mode) {}

/*! @brief Change Window Flags. @param X X. @ingroup window */
void setWindowFlag(int mode) {}

void Moss_SetWindowTitle(Moss_Window* window, const char* title) {
    const wchar_t* tmp_title = convertCharToWchar(title);
    SetWindowTextW(window->handle, tmp_title);
}

void Moss_SetWindowIcon(Moss_Window* window, Moss_Image image) {
    // Create a bitmap header
    HBITMAP hBitmap = NULL;
    HICON hIcon = NULL;

    BITMAPV5HEADER bi = { 0 };
    bi.bV5Size = sizeof(BITMAPV5HEADER);
    bi.bV5Width = image.width;
    bi.bV5Height = -image.height; // Negative means top-down
    bi.bV5Planes = 1;
    bi.bV5BitCount = 32;
    bi.bV5Compression = BI_BITFIELDS;
    bi.bV5RedMask   = 0x00FF0000;
    bi.bV5GreenMask = 0x0000FF00;
    bi.bV5BlueMask  = 0x000000FF;
    bi.bV5AlphaMask = 0xFF000000;

    void* lpBits = NULL;
    hBitmap = CreateDIBSection(dc, (BITMAPINFO*)&bi, DIB_RGB_COLORS, &lpBits, NULL, 0);
    ReleaseDC(NULL, dc);

    if (!hBitmap) return;

    memcpy(lpBits, image.pixels, image.width * image.height * 4); // RGBA → BGRA works as-is with masks

    ICONINFO ii = { 0 };
    ii.fIcon = TRUE;
    ii.hbmColor = hBitmap;
    ii.hbmMask = hBitmap;

    hIcon = CreateIconIndirect(&ii);

    if (hIcon) {
        SendMessage(window->handle, WM_SETICON, ICON_SMALL, (LPARAM)hIcon);
        SendMessage(window->handle, WM_SETICON, ICON_BIG, (LPARAM)hIcon);
    }

    DeleteObject(hBitmap);
}

/*
void Moss_SetWindowSizeLimits(Moss_Window* window, int minWidth, int minHeight, int maxWidth, int maxHeight) {
    
}

*/


void Moss_WindowMode(Moss_WindowFlag );

void Moss_SwapBuffers() { SwapBuffers(dc); }


int Moss_GetWindowWidth() { return g_width; }
int Moss_GetWindowHeight() { return g_height; }

int Moss_GetAvailableCPUCores(void) {
    SYSTEM_INFO sysinfo;
    GetSystemInfo(&sysinfo);
    return (int)sysinfo.dwNumberOfProcessors;
}

int Moss_GetCPUCacheLineSize(void) {
    DWORD bufferSize = 0;
    GetLogicalProcessorInformation(NULL, &bufferSize);

    SYSTEM_LOGICAL_PROCESSOR_INFORMATION* buffer =
        (SYSTEM_LOGICAL_PROCESSOR_INFORMATION*)malloc(bufferSize);

    if (!GetLogicalProcessorInformation(buffer, &bufferSize)) {
        free(buffer);
        return -1;
    }

    int lineSize = 0;
    DWORD count = bufferSize / sizeof(SYSTEM_LOGICAL_PROCESSOR_INFORMATION);

    for (DWORD i = 0; i < count; i++) {
        if (buffer[i].Relationship == RelationCache &&
            buffer[i].Cache.Level == 1) {
            lineSize = buffer[i].Cache.LineSize;
            break;
        }
    }

    free(buffer);
    return lineSize;  // in bytes
}


void Moss_SwapBuffersInterval(int interval) {
#ifdef MOSS_USE_OPENGL
    // Load function pointer if not loaded yet
    if (!wglSwapIntervalEXT) {
        wglSwapIntervalEXT = (PFNWGLSWAPINTERVALEXTPROC)wglGetProcAddress("wglSwapIntervalEXT");
        }
    if (wglSwapIntervalEXT) {
        wglSwapIntervalEXT(interval);
    }
#endif // MOSS_USE_OPENGL
    (void)interval;
}

void* Moss_GetProcAdress(const char* procname)
{
#ifdef MOSS_USE_OPENGL
    PROC proc = wglGetProcAddress(procname);         // First, try wglGetProcAddress
    if (!proc)
    {
        static HMODULE openglDLL = LoadLibraryA("opengl32.dll");
        proc = GetProcAddress(openglDLL, procname);
        if (!proc)
        {
            MOSS_TRACE("Failed to load gl function glCreateProgram");
            return (void*)proc;
        }
    }
    return (void*)proc;
#else
    (void)procname; // suppress unused parameter warning
    return nullptr;
#endif
}




#if defined(MOSS_GRAPHICS_VULKAN) && defined(MOSS_PLATFORM_WINDOWS)
#define VK_USE_PLATFORM_WIN32_KHR
#include <vulkan/vulkan_win32.h>

HMODULE vulkanLib = NULL;
PFN_vkGetInstanceProcAddr my_vkGetInstanceProcAddr = NULL;

int Moss_VulkanSupported(void) {
    vulkanLib = LoadLibraryA("vulkan-1.dll");
    if (!vulkanLib) return 0;

    my_vkGetInstanceProcAddr = (PFN_vkGetInstanceProcAddr)GetProcAddress(vulkanLib, "vkGetInstanceProcAddr");
    return my_vkGetInstanceProcAddr != NULL;
}

VkResult Moss_CreateWindowSurface(Moss_Window* window, VkInstance vk_instance, const VkAllocationCallbacks *allocator, VkSurfaceKHR* vk_surface) {
    VkWin32SurfaceCreateInfoKHR windowSurfaceInfo = {};
    windowSurfaceInfo.sType = VK_STRUCTURE_TYPE_WIN32_SURFACE_CREATE_INFO_KHR;
    windowSurfaceInfo.hinstance = hInstance;
    windowSurfaceInfo.hwnd = window->handle;

    if(!Moss_VulkanSupported()) { return VK_ERROR_INITIALIZATION_FAILED; }

    PFN_vkCreateWin32SurfaceKHR vkCreateWin32SurfaceKHR = (PFN_vkCreateWin32SurfaceKHR)my_vkGetInstanceProcAddr(vk_instance, "vkCreateWin32SurfaceKHR");
    if (!vkCreateWin32SurfaceKHR) { return VK_ERROR_EXTENSION_NOT_PRESENT; }

    VkResult res = vkCreateWin32SurfaceKHR(vk_instance, &windowSurfaceInfo, allocator, vk_surface);
    return res;
}

//void Moss_InitVulkanLoader(PFN_vkGetInstanceProcAddr loader) { my_vkGetInstanceProcAddr = loader; }

const char** Moss_GetRequiredInstanceExtensions(uint32_t* count) {
    static const char* extensions[2]; // 3 max (surface, xlib/wayland/win32)
    uint32_t extCount = 0;

    extensions[extCount++] = VK_KHR_SURFACE_EXTENSION_NAME;
    extensions[extCount++] = VK_KHR_WIN32_SURFACE_EXTENSION_NAME;

    if (count) { *count = extCount; }
    return extensions;

}

void* Moss_GetInstanceProcAddress(VkInstance instance, const char* procname) {
    if (!my_vkGetInstanceProcAddr) return NULL;
    return (void*)my_vkGetInstanceProcAddr(instance, procname);
}


int Moss_GetPhysicalDevicePresentationSupport(Moss_Window* window, VkPhysicalDevice device, uint32_t queuefamily) {
    PFN_vkGetPhysicalDeviceWin32PresentationSupportKHR vkGetPhysicalDeviceWin32PresentationSupportKHR =
        (PFN_vkGetPhysicalDeviceWin32PresentationSupportKHR)my_vkGetInstanceProcAddr(NULL, "vkGetPhysicalDeviceWin32PresentationSupportKHR");

    if (!vkGetPhysicalDeviceWin32PresentationSupportKHR) { return 0; }

    return vkGetPhysicalDeviceWin32PresentationSupportKHR(device, queuefamily);
}
#endif // MOSS_USE_VULKAN


//===============================
/*          Callbacks          */
//===============================

void Moss_SetFramebufferReSizeCallback(FramebufferResizeCallback callback) { g_framebufferResizeCallback = callback; }
void Moss_SetWindowResizeCallback(void (*callback)(int width, int height)) { windowResizeCallback = callback; }
void Moss_SetWindowContentScaleCallback(Moss_Window* window, int width, int height) { if (windowContentScaleCallback) { windowContentScaleCallback(width, height); } }
void Moss_SetWindowPositionCallback(Moss_Window* window, int x, int y) { if (windowPositionCallback) { windowPositionCallback(x, y); } }
void Moss_SetWindowFocusCallback(Moss_Window* window) { if (windowFocusCallback) { windowFocusCallback(true); } }
void Moss_SetWindowSizeCallback(void (*callback)(int width, int height)) { windowSizeCallback = callback; }



void Moss_SetWindowMode(Moss_Window* window, Moss_WindowFlags flags) {
    if (!window || !window->handle) return;
 
    const HWND hwnd = window->handle;

    switch (flags)
    {
    case Moss_WindowFlags::NOTITLEBAR :
        LONG_PTR style = GetWindowLongPtrW(hwnd, GWL_STYLE);
        style &= ~(WS_CAPTION | WS_SYSMENU);
        SetWindowLongPtrW(hwnd, GWL_STYLE, style);
        SetWindowPos(hwnd, nullptr, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE | SWP_FRAMECHANGED);
        break;
    case Moss_WindowFlags::RESIZE_DISABLED :
        LONG_PTR style = GetWindowLongPtrW(hwnd, GWL_STYLE);
        style &= ~(WS_THICKFRAME | WS_MAXIMIZEBOX);
        SetWindowLongPtrW(hwnd, GWL_STYLE, style);
        SetWindowPos(hwnd, nullptr, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE | SWP_FRAMECHANGED);
        break;
    case Moss_WindowFlags::TRANSPARENT :
        LONG_PTR exStyle = GetWindowLongPtrW(hwnd, GWL_EXSTYLE);
        SetWindowLongPtrW(hwnd, GWL_EXSTYLE, exStyle | WS_EX_LAYERED);
        SetLayeredWindowAttributes(hwnd, 0, 230, LWA_ALPHA);
        break;
    case Moss_WindowFlags::NO_FOCUS :
        /* code */
        break;
    case Moss_WindowFlags::POPUP :
        LONG_PTR style = GetWindowLongPtrW(hwnd, GWL_STYLE);
        style &= ~WS_OVERLAPPEDWINDOW;
        style |= WS_POPUP;
        SetWindowLongPtrW(hwnd, GWL_STYLE, style);
        SetWindowPos(hwnd, nullptr, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE | SWP_FRAMECHANGED);
        break;
    case Moss_WindowFlags::NO_FOCUS :
        LONG_PTR exStyle = GetWindowLongPtrW(hwnd, GWL_EXSTYLE);
        SetWindowLongPtrW(hwnd, GWL_EXSTYLE, exStyle | WS_EX_NOACTIVATE);
        break;
    case Moss_WindowFlags::POPUP :
        LONG_PTR style = GetWindowLongPtrW(hwnd, GWL_STYLE);
        style &= ~WS_OVERLAPPEDWINDOW;
        style |= WS_POPUP;
        SetWindowLongPtrW(hwnd, GWL_STYLE, style);
        SetWindowPos(hwnd, nullptr, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE | SWP_FRAMECHANGED);
        break;
    case Moss_WindowFlags::EXTEND_TO_TITLE :
        MARGINS margins{ -1, -1, -1, -1 };
        DwmExtendFrameIntoClientArea(hwnd, &margins);
        break;
    case Moss_WindowFlags::MOUSE_PASSTHROUGH :
        LONG_PTR exStyle = GetWindowLongPtrW(hwnd, GWL_EXSTYLE);
        SetWindowLongPtrW(hwnd, GWL_EXSTYLE, exStyle | WS_EX_LAYERED | WS_EX_TRANSPARENT);
        SetLayeredWindowAttributes(hwnd, 0, 255, LWA_ALPHA);
        break;
    case Moss_WindowFlags::SHARP_CORNERS :
        DWORD preference = DWMWCP_DONOTROUND;
        DwmSetWindowAttribute(hwnd, DWMWA_WINDOW_CORNER_PREFERENCE, &preference, sizeof(preference));
        break;
    case Moss_WindowFlags::EXCLUDE_FROM_CAPTURE :
        SetWindowDisplayAffinity(hwnd, WDA_EXCLUDEFROMCAPTURE);
        break;
    case Moss_WindowFlags::HIDDEN :
        ShowWindow(hwnd, SW_HIDE);
        break;
    case Moss_WindowFlags::SHOWN :
        ShowWindow(hwnd, SW_SHOW);
        break;
    case Moss_WindowFlags::BORDERLESS :
        LONG_PTR style = GetWindowLongPtrW(hwnd, GWL_STYLE);
        style &= ~(WS_CAPTION | WS_THICKFRAME | WS_MINIMIZEBOX | WS_MAXIMIZEBOX | WS_SYSMENU);
        SetWindowLongPtrW(hwnd, GWL_STYLE, style);
        SetWindowPos(hwnd, nullptr, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE | SWP_FRAMECHANGED);
        break;
    case Moss_WindowFlags::RESIZABLE :
        LONG_PTR style = GetWindowLongPtrW(hwnd, GWL_STYLE);
        style |= (WS_THICKFRAME | WS_MAXIMIZEBOX);
        SetWindowLongPtrW(hwnd, GWL_STYLE, style);
        SetWindowPos(hwnd, nullptr, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE | SWP_FRAMECHANGED);
        break;
    case Moss_WindowFlags::MAXIMIZED :
        ShowWindow(hwnd, SW_MAXIMIZE);
        break;
    case Moss_WindowFlags::MINIMIZED :
        ShowWindow(hwnd, SW_RESTORE);
        break;
    case Moss_WindowFlags::MOUSE_GRABBED :
        RECT rect;
        GetClientRect(hwnd, &rect);
        POINT topLeft{ rect.left, rect.top };
        POINT bottomRight{ rect.right, rect.bottom };
        ClientToScreen(hwnd, &topLeft);
        ClientToScreen(hwnd, &bottomRight);
        RECT screenRect{ topLeft.x, topLeft.y, bottomRight.x, bottomRight.y };
        ClipCursor(&screenRect);
        break;
    case Moss_WindowFlags::INPUT_FOCUS :
        SetFocus(hwnd);
        break;
    case Moss_WindowFlags::MOUSE_FOCUS :
        SetCapture(hwnd);
        break;
    case Moss_WindowFlags::ALWAYS_ON_TOP :
        SetWindowPos(hwnd, HWND_TOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_NOACTIVATE);
        break;
    case Moss_WindowFlags::KEYBOARD_GRABBED :
        if (g_keyboardHook) {
            UnhookWindowsHookEx(g_keyboardHook);
        }
        g_keyboardGrabWindow = hwnd;
        g_keyboardHook = SetWindowsHookExW(WH_KEYBOARD_LL, KeyboardGrabProc, GetModuleHandleW(nullptr), 0);
        break;
    default:
        break;
    }
}

/*
static std::vector<uint32_t> g_textInput;

void Moss_GetMouseWheelDelta(float* x, float* y) {
    if (x) *x = g_mouseWheelX;
    if (y) *y = g_mouseWheelY;
    g_mouseWheelX = 0.0f;
    g_mouseWheelY = 0.0f;
}

uint32_t Moss_GetTextInput(uint32_t* codepoints, uint32_t capacity) {
    if (!codepoints || capacity == 0) return 0;
    const uint32_t count = static_cast<uint32_t>(std::min<size_t>(g_textInput.size(), capacity));
    std::copy_n(g_textInput.begin(), count, codepoints);
    g_textInput.erase(g_textInput.begin(), g_textInput.begin() + count);
    return count;
}

bool Moss_IsWindowFocused(Moss_Window* window) {
    const HWND target = window ? window->handle : handle;
    return target != nullptr && GetFocus() == target;
}

bool Moss_SetWindowAlwaysOnTop(Moss_Window* window, bool enabled) {
    if (!window || !window->handle) return false;
    return SetWindowPos(window->handle, enabled ? HWND_TOPMOST : HWND_NOTOPMOST, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_NOACTIVATE) != FALSE;
}

bool Moss_SetWindowBorderless(Moss_Window* window, bool enabled) {
    if (!window || !window->handle) return false;
    LONG_PTR style = GetWindowLongPtrW(window->handle, GWL_STYLE);
    if (enabled) style &= ~(WS_CAPTION | WS_THICKFRAME | WS_MINIMIZEBOX | WS_MAXIMIZEBOX | WS_SYSMENU);
    else style |= WS_OVERLAPPEDWINDOW;
    SetWindowLongPtrW(window->handle, GWL_STYLE, style);
    return SetWindowPos(window->handle, nullptr, 0, 0, 0, 0, SWP_NOMOVE | SWP_NOSIZE | SWP_NOZORDER | SWP_NOACTIVATE | SWP_FRAMECHANGED) != FALSE;
}
*/
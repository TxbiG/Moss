#include <Moss/Platform/Windows/win32_platform.h>
#include <wingdi.h>

// TODO: convert YUYV (Linux) or RGB32 (Windows) into consistent format (like RGB24)

HINSTANCE hInstance;
HWND handle;
static bool isRunning = true;

#ifdef MOSS_USE_OPENGL

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

static Moss_FramebufferResizeCallback g_framebufferResizeCallback = nullptr;
static Moss_WindowSizeCallback g_windowSizeCallback = nullptr;
static Moss_WindowPositionCallback g_windowPositionCallback = nullptr;
static Moss_WindowFocusCallback g_windowFocusCallback = nullptr;
static Moss_WindowContentScaleCallback g_windowContentScaleCallback = nullptr;
static Moss_WindowResizeCallback g_windowResizeCallback = nullptr;

int g_width;
int g_height;
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
	//case WM_PAINT:
		//BeginPaint(hWnd, &ps);
		//EndPaint(hWnd, &ps);
		//break;

	case WM_SIZE: {
        int width = LOWORD(lParam);
        int height = HIWORD(lParam);
        if (g_framebufferResizeCallback) {
            g_framebufferResizeCallback(width, height);
        }
        g_width = width;
        g_height = height;
        return 0;
    }
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
        // Mouse Input
        UINT sz = 0;
        GetRawInputData(reinterpret_cast<HRAWINPUT>(lParam), RID_INPUT, nullptr, &sz, sizeof(RAWINPUTHEADER));

        uint8_t buf[sizeof(RAWINPUT) + 64]{};

        if (GetRawInputData(reinterpret_cast<HRAWINPUT>(lParam), RID_INPUT, buf, &sz, sizeof(RAWINPUTHEADER)) == sz)
        {
            RAWINPUT* ri = reinterpret_cast<RAWINPUT*>(buf);
            if (ri->header.dwType == RIM_TYPEMOUSE)
            {
                g_frame.mx += ri->data.mouse.lLastX;
                g_frame.my += ri->data.mouse.lLastY;
            }
        }
        return 0;
    }

	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}

	return 0;
}

Moss_Window* Moss_CreateWindow(const char* title, int width, int height, Moss_Monitor* monitor, Moss_Window* share) {
    FreeConsole();
    //HWND cons = GetConsoleWindow();
    //ShowWindow(cons, SW_HIDE);

    hInstance = GetModuleHandleA(nullptr);
    WNDCLASS wc = {0};
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInstance;
    wc.hIcon = LoadIcon(hInstance, IDI_APPLICATION); 
    wc.hCursor = LoadCursor(NULL, IDC_ARROW);
    wc.lpszClassName = title;
    if (!RegisterClassA(&wc)) { MOSS_ERROR("Failed to register window class"); return NULL; }

    Moss_Window* window = (Moss_Window*) malloc(sizeof(Moss_Window));

    #if defined(MOSS_USE_DIRECTX) || defined(MOSS_USE_VULKAN)
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

    const int contextAttribs[] =
    {
      WGL_CONTEXT_MAJOR_VERSION_ARB, 4,
      WGL_CONTEXT_MINOR_VERSION_ARB, 3,
      WGL_CONTEXT_PROFILE_MASK_ARB, WGL_CONTEXT_CORE_PROFILE_BIT_ARB,
      WGL_CONTEXT_FLAGS_ARB, WGL_CONTEXT_DEBUG_BIT_ARB,
      0 // Terminate the Array
    };

    HGLRC rc = wglCreateContextAttribsARB(dc, 0, contextAttribs);
    if(!rc)
    {
      MOSS_ERROR(0, "Failed to crate Render Context for OpenGL");
      return NULL;
    }

    if(!wglMakeCurrent(dc, rc))
    {
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

void* Moss_LoadDynamicLibrary(const char* dll)
{
  HMODULE result = LoadLibraryA(dll);

  return result;
}

void* Moss_LoadDynamicFunction(void* dll, const char* funName)
{
  FARPROC proc = GetProcAddress((HMODULE)dll, funName);
  MOSS_ERROR("Failed to load function: %s from DLL", funName);

  return (void*)proc;
}

bool Moss_RemoveDynamicLibrary(void* dll)
{
  BOOL freeResult = FreeLibrary((HMODULE)dll);
  MOSS_ASSERT(freeResult, "Failed to FreeLibrary");

  return (bool)freeResult;
}

/*! @brief Change Window Mode. e.g. MINAMIZED, MAXIMIZED, FULLSCREEN, BORDERLESS. @param X X. @ingroup window */
void setWindowMode(int mode) {}

/*! @brief Change Window Flags. @param X X. @ingroup window */
void setWindowFlag(int mode) {}

const wchar_t* convertCharToWchar(const char* str) {
    static std::wstring wstr;
    int size_needed = MultiByteToWideChar(CP_UTF8, 0, str, -1, nullptr, 0);
    wstr.resize(size_needed - 1);
    MultiByteToWideChar(CP_UTF8, 0, str, -1, &wstr[0], size_needed);
    return wstr.c_str();
}

void Moss_SetWindowTitle(Moss_Window* window, const char* title)
{
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


int Moss_GetSystemRAM(void) {
    MEMORYSTATUSEX statex;
    statex.dwLength = sizeof(statex);
    if (GlobalMemoryStatusEx(&statex)) {
        return (int)(statex.ullTotalPhys / 1024 / 1024); // Convert to MB
    }
    return -1;
}

bool Moss_OpenURL(const char* url) {
    if (!url) {  MOSS_ERROR("URL is null.");  return false; }

    const wchar_t* wurl = convertCharToWchar(url);

    // Initialize COM as per MSDN recommendation
    HRESULT hr = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
    bool comInitialized = SUCCEEDED(hr);

    HINSTANCE rc = ShellExecuteW(nullptr, L"open", wurl, nullptr, nullptr, SW_SHOWNORMAL);

    if (comInitialized)
        CoUninitialize();

    if ((INT_PTR)rc <= 32) {
        MOSS_ERROR("Couldn't open given URL.");
        return false;
    }

    return true;
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

#ifdef MOSS_USE_VULKAN
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
/*
void Moss_SetWindowResizeCallback(void (*callback)(int width, int height)) {
    windowResizeCallback = callback;
}

void Moss_SetWindowContentScaleCallback(Moss_Window* window, int width, int height) {
    if (windowContentScaleCallback) {
        windowContentScaleCallback(width, height);
    }
}

void Moss_SetWindowPositionCallback(Moss_Window* window, int x, int y) {
    if (windowPositionCallback) {
        windowPositionCallback(x, y);
    }
}

void Moss_SetWindowFocusCallback(Moss_Window* window) {
    if (windowFocusCallback) {
        windowFocusCallback(true);
    }
}

void Moss_SetWindowSizeCallback(void (*callback)(int width, int height)) {
    windowSizeCallback = callback;
}
*/
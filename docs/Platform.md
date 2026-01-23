# Platform
## Overview


Platforms supported ```Windows```, ```Linux```, ```MacOS```, ```FreeBSD```, ```Android```, ```IOS```.

> [!NOTE]  
Working with VR vendors or any other platform not specified here will require manual creation, such as creating a window and input.

### Input Support
| Input Devices | Windows | MacOS | Linux  | IOS  | Android  |
| ------------- | ------------- | ------------- | ------------- | ------------- | ------------- |
| Keyboard      | ✅ | ✅ | ✅ | ❌ | ❌  |
| Mouse         | ✅ | ✅ | ✅ | ❌ | ❌ |
| Touch         | ✅ | ✅ | ✅ | ✅ | ✅ |
| Pen           | ✅ |✅  | ✅ | ✅ | ✅ |
| Xbox 360 Controllers  | ✅ | ✅ | ✅ | ❌ | ❌ |
| Xbox One Controllers  | ✅ | ✅ | ✅ | ❌ | ❌ |
| Xbox Series S Controllers | ✅ | ✅ | ✅ | ❌ | ❌ |
| PlayStation 4 Controllers | ✅ | ✅ | ✅ | ❌ | ❌ |
| PlayStation 5 Controllers | ✅ | ✅ | ✅ | ❌ | ❌ |

### Haptic Feedback support
| Haptic Feedback | Playstation (DualSense) | Xbox Impulse Triggers | Mouses        | Keyboard      | IOS Haptic  | Android Haptic |
| -------------   | -------------           | -------------         | ------------- | ------------- | ------------- | ------------- |
| Supported       | ✅                      | ✅                   | ❌            | ❌           | ✅           | ✅            |

### Graphics API Support
| Platforms     |Windows                       | MacOS  | Linux          | IOS              | Android               |
| ------------- | -------------                | -------| -------------  | -------------    | -------------         |
| Supported     | OpenGL, Vulkan, DirectX 12 or 12 | Metal  | OpenGL, Vulkan | OpenGL ES, Metal | OpenGL ES, Vulkan | 

## Macros
```cpp
// Provided by Moss
#define MOSS_PLATFORM_WINDOWS
#define MOSS_PLATFORM_WINDOWS_UWP
#define MOSS_PLATFORM_XBOXONE
#define MOSS_PLATFORM_XBOXSCARLETT
#define MOSS_PLATFORM_PS4
#define MOSS_PLATFORM_PS5
#define MOSS_PLATFORM_LINUX
#define MOSS_PLATFORM_UNIX
#define MOSS_PLATFORM_BSD
#define MOSS_PLATFORM_WAYLAND
#define MOSS_PLATFORM_X11
#define MOSS_PLATFORM_ANDROID
#define MOSS_PLATFORM_MACOS
#define MOSS_PLATFORM_IOS
#define MOSS_PLATFORM_TVOS
#define MOSS_PLATFORM_WASM
```

## Enums
```cpp
// Provided by Moss
enum Keyboard;
enum Mouse;
enum Gamepad
enum Joystick
enum Moss_WindowFlags;
enum Moss_MessageBoxFlags;
enum Moss_WindowMode;
enum Moss_CursorMode;
enum Moss_CursorShape;
```

## Structs
```cpp
// Provided by Moss
struct Moss_Window;
struct Moss_Monitor;
struct Moss_Cursor;
struct Moss_GammaRamp;
struct Moss_VideoMode;
struct Moss_Image;
struct Moss_Event;
struct Moss_Locale {
    char* country;
    char* language;
};
struct Moss_HapticFeedback;
```

## Functions
```cpp
Moss_Window* Moss_CreateWindow(const char* Title);
void Moss_TerminateWindow(Moss_Window* window);
bool ShouldWindowClose(Moss_Window* window);
void Moss_PollEvents();
bool Moss_CreateMessageBox();

void Moss_GetMonitorPhysicalSize(Moss_Monitor monitor, int* width_mm, int* height_mm);
void Moss_GetMonitorContentScale(Moss_Monitor monitor, float* xscale, float* yscale);
void Moss_GetMonitorPosition(Moss_Monitor monitor, int* x, int* y);
const char* Moss_GetMonitorName(Moss_Monitor monitor);
void Moss_SetGammaRamp(Moss_Monitor monitor, const Moss_GammaRamp* gammaRamp);
void Moss_SetGamma(Moss_Monitor monitor, float gamma);
Moss_GammaRamp* Moss_GetGammaRamp(Moss_Monitor monitor);

inline bool IsPressed(Keyboard k);
inline bool IsReleased(Keyboard k);
inline bool IsJustPressed(Keyboard k);
inline bool IsJustReleased(Keyboard k);

inline bool IsPressed(Mouse b);
inline bool IsReleased(Mouse b);
inline bool IsJustPressed(Mouse b);
inline bool IsJustReleased(Mouse b);

inline bool IsPressed(Gamepad b);
inline bool IsReleased(Gamepad b);
inline bool IsJustPressed(Gamepad b);
inline bool IsJustReleased(Gamepad b);
float GetJoyAxis(Joystick j);

Moss_Locale* Moss_GetLocale();
bool Moss_OpenURL(const char* url);
```

## Examples
### Window
```cpp
#pragma once
#include <Moss.h>

int main()
{

  Moss_Window* m_window = Moss_CreateWindow("Game", CENTER_POSITION, CENTER_POSITION, 700, 600, NULL, NULL);
  if (!m_window) { return 0; }

  while(!Moss_ShouldWindowClose(m_window))
  {
      PollEvents();
      /*    Code    */
      SwapBuffers(m_window);
  }

  Moss_TerminateWindow(m_window);
  return 0;
}
```

Callback for framebuffer resize events. @param width  New framebuffer width, in pixels. @param height New framebuffer height, in pixels.
```cpp
typedef void (*Moss_FramebufferResizeCallback)(int width, int height);
```
Sets the framebuffer resize callback. callback Pointer to a function to be invoked when framebuffer size changes.
```cpp
MOSS_API void Moss_SetFramebufferResizeCallback(Moss_FramebufferResizeCallback callback);
```

Callback for logical window size changes. @param width  New window width, in screen coordinates. @param height New window height, in screen coordinates.
```cpp
typedef void (*Moss_WindowSizeCallback)(int width, int height);
```
Sets the window size callback. Callback Pointer to a function invoked when the window size changes.
```cpp
MOSS_API void Moss_SetWindowSizeCallback(Moss_WindowSizeCallback callback);
```

Callback for window position changes on screen. @param xpos New X coordinate of the window’s top-left corner. @param ypos New Y coordinate of the window’s top-left corner.
```cpp
typedef void (*Moss_WindowPositionCallback)(int xpos, int ypos);
```
Sets the window resize callback (platform-level, e.g. minimize/maximize). @param callback Pointer to a function invoked when window resizing events occur.
```cpp
MOSS_API void Moss_SetWindowResizeCallback(Moss_WindowResizeCallback callback);
```

Callback for window focus events. @param focused True if the window gained focus; false if it lost focus.
```cpp
typedef void (*Moss_WindowFocusCallback)(bool focused);
```
Sets the window position callback. Callback Pointer to a function invoked when the window position changes.
```cpp
MOSS_API void Moss_SetWindowPositionCallback(Moss_WindowPositionCallback callback);
```

Callback for content scale changes (e.g., HiDPI scaling). @param xscale X-axis content scale factor. @param yscale Y-axis content scale factor.
```cpp
typedef void (*Moss_WindowContentScaleCallback)(float xscale, float yscale);
```
Sets the window focus callback. Callback Pointer to a function invoked when the window focus changes.
```cpp
MOSS_API void Moss_SetWindowFocusCallback(Moss_WindowFocusCallback callback);
```

Callback for general window resize notifications (platform-driven). @param width  New window width in pixels. @param height New window height in pixels.
```cpp
typedef void (*Moss_WindowResizeCallback)(int width, int height);
```
Sets the window content scale callback (for HiDPI / Retina support). @param callback Pointer to a function invoked when the content scale changes
```cpp
MOSS_API void Moss_SetWindowContentScaleCallback(Moss_WindowContentScaleCallback callback);
```

## Embedded window Specific
These are used for creating windows for mobile and console-specific
```cpp
MOSS_API void Moss_CreateEmbeddedWindow();
MOSS_API void Moss_Terminate_EmbeddedWindow();
MOSS_API void Moss_Embedded_OnOrientationChanged(bool focused);
MOSS_API void Moss_Embedded_OnResume();
MOSS_API void Moss_Embedded_OnPause();
```

### Monitor
```cpp
```
Callback for monitor configuration changes (e.g. hotplug events). @param monitorName Name or ID of the monitor that changed. connected True if the monitor was connected; false if disconnected.
```cpp
typedef void (*Moss_MonitorCallback)(const char* monitorName, bool connected);
```
Sets the monitor connection or configuration callback. Callback pointer to a function invoked when a monitor is connected or disconnected
```cpp
MOSS_API void Moss_SetMonitorCallback(Moss_MonitorCallback callback);
```

### Input
```cpp
```
### Haptic Feedback
```cpp
```
### Video Capture
```cpp
```

## Graphics API Specific
### OpenGL / OpenGL ES
```cpp
void Moss_MakeContextCurrent(Moss_Window* window);
```
```cpp
void Moss_SwapBuffers(); // < Used in Moss_Renderer so only call if you are making your own renderer
```
Moss_SwapBuffersInterval is used for V-Sync. However, it may cause the game to lag and introduce input lag.
```cpp
void Moss_SwapBuffersInterval(int interval);
```

```cpp
void* Moss_GetProcAddress(const char* procname);
```

### Vulkan
```cpp
VkResult Moss_CreateWindowSurface(Moss_Window* window, VkInstance vk_instance,
  const VkAllocationCallbacks *allocator, VkSurfaceKHR* vk_surface);
```
Moss_VulkanSupported is used to help check if the Vulkan driver is installed.
```cpp
int Moss_VulkanSupported(void);
```
```cpp
void Moss_InitVulkanLoader(PFN_vkGetInstanceProcAddr loader);
```
```cpp
const char** Moss_GetRequiredInstanceExtensions(uint32_t* count);
```
```cpp
void* Moss_GetInstanceProcAddress(VkInstance instance, const char* procname);
```
```cpp
int Moss_GetPhysicalDevicePresentationSupport(Moss_Window& window, VkPhysicalDevice device, uint32_t queuefamily);
```

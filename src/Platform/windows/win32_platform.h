#ifndef MOSS_PLATFORM_WIN32_H
#define MOSS_PLATFORM_WIN32_H

#include <Moss/Moss_stdinc.h>    // Defines Uint64, bool types, etc.
#include <Moss/Moss_Platform.h>  // Defines GamepadAxis, etc.

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <sysinfoapi.h>

#include "../platform_intern.h"
/*
#include <wctype.h>
#include <windows.h>
#include <dwmapi.h>
#include <dinput.h>
#include <xinput.h>
#include <dbt.h>
*/

// TODO: Test Support for Xbox and Playstation (4 & 5) Controllers

#ifndef XINPUT_CAPS_WIRELESS
    #define XINPUT_CAPS_WIRELESS 0x0002
#endif
#ifndef XINPUT_DEVSUBTYPE_WHEEL
    #define XINPUT_DEVSUBTYPE_WHEEL 0x02
#endif
#ifndef XINPUT_DEVSUBTYPE_ARCADE_STICK
    #define XINPUT_DEVSUBTYPE_ARCADE_STICK 0x03
#endif
#ifndef XINPUT_DEVSUBTYPE_FLIGHT_STICK
    #define XINPUT_DEVSUBTYPE_FLIGHT_STICK 0x04
#endif
#ifndef XINPUT_DEVSUBTYPE_DANCE_PAD
    #define XINPUT_DEVSUBTYPE_DANCE_PAD 0x05
#endif
#ifndef XINPUT_DEVSUBTYPE_GUITAR
    #define XINPUT_DEVSUBTYPE_GUITAR 0x06
#endif
#ifndef XINPUT_DEVSUBTYPE_DRUM_KIT
    #define XINPUT_DEVSUBTYPE_DRUM_KIT 0x08
#endif
#ifndef XINPUT_DEVSUBTYPE_ARCADE_PAD
    #define XINPUT_DEVSUBTYPE_ARCADE_PAD 0x13
#endif
#ifndef XUSER_MAX_COUNT
    #define XUSER_MAX_COUNT 4
#endif

#define SONY_PS 0x054C
#define SONY_PS_DUALSHOCK_PID 0x05C4
#define SONY_PS_DUALSENSE_PID 0x0CE6


#define MOD_DOWN(state, flag)  (((state) & (flag)) != 0)

extern HWND handle;

struct Moss_Window {
    HWND        handle;
    HICON       bigIcon;
    HICON       smallIcon;

    bool        cursorTracked;
    bool        frameAction;
    bool        iconified;
    bool        maximized;
    // Whether to enable framebuffer transparency on DWM
    bool        transparent;
    bool        scaleToMonitor;
    bool        keymenu;
    bool        showDefault;

    // Cached size used to filter out duplicate events
    int         width, height;

    // The last received cursor position, regardless of source
    int         lastCursorPosX, lastCursorPosY;
    // The last received high surrogate when decoding pairs of UTF-16 messages
    WCHAR       highSurrogate;
};

/*
struct _libraryWin32 {
    HINSTANCE           instance;
    HWND                helperWindowHandle;
    ATOM                helperWindowClass;
    ATOM                mainWindowClass;
    HDEVNOTIFY          deviceNotificationHandle;
    int                 acquiredMonitorCount;
    char*               clipboardString;
    short int           keycodes[512];
    short int           scancodes[Keyboard::MOSS_LAST_KEY + 1];
    char                keynames[Keyboard::MOSS_LAST_KEY + 1][5];
    // Where to place the cursor when re-enabled
    double              restoreCursorPosX, restoreCursorPosY;
    // The window whose disabled cursor mode is active
    _window*        disabledCursorWindow;
    // The window the cursor is captured in
    _window*        capturedCursorWindow;
    RAWINPUT*           rawInput;
    int                 rawInputSize;
    UINT                mouseTrailSize;
    // The cursor handle to use to hide the cursor (NULL or a transparent cursor)
    HCURSOR             blankCursor;

    struct {
        HINSTANCE                       instance;
        PFN_DirectInput8Create          Create;
        IDirectInput8W*                 api;
    } dinput8;

    struct {
        HINSTANCE                       instance;
        PFN_XInputGetCapabilities       GetCapabilities;
        PFN_XInputGetState              GetState;
    } xinput;

    struct {
        HINSTANCE                       instance;
        PFN_EnableNonClientDpiScaling   EnableNonClientDpiScaling_;
        PFN_SetProcessDpiAwarenessContext SetProcessDpiAwarenessContext_;
        PFN_GetDpiForWindow             GetDpiForWindow_;
        PFN_AdjustWindowRectExForDpi    AdjustWindowRectExForDpi_;
        PFN_GetSystemMetricsForDpi      GetSystemMetricsForDpi_;
    } user32;

    struct {
        HINSTANCE                       instance;
        PFN_DwmIsCompositionEnabled     IsCompositionEnabled;
        PFN_DwmFlush                    Flush;
        PFN_DwmEnableBlurBehindWindow   EnableBlurBehindWindow;
        PFN_DwmGetColorizationColor     GetColorizationColor;
    } dwmapi;

    struct {
        HINSTANCE                       instance;
        PFN_SetProcessDpiAwareness      SetProcessDpiAwareness_;
        PFN_GetDpiForMonitor            GetDpiForMonitor_;
    } shcore;

    struct {
        HINSTANCE                       instance;
        PFN_RtlVerifyVersionInfo        RtlVerifyVersionInfo_;
    } ntdll;
} _libraryWin32;
*/

// Win32-specific per-monitor data
struct _monitorWin32 {
    HMONITOR            handle;
    // This size matches the static size of DISPLAY_DEVICE.DeviceName
    WCHAR               adapterName[32];
    WCHAR               displayName[32];
    char                publicAdapterName[32];
    char                publicDisplayName[32];
    bool            modesPruned;
    bool            modeChanged;
};

// Win32-specific per-cursor data
struct _cursorWin32 { HCURSOR handle; };


// Modifiers
enum class KeyMod : uint8_t {
    SHIFT       = 1 << 0,
    CONTROL     = 1 << 1,
    ALT         = 1 << 2,
    SUPER       = 1 << 3,
    CAPS_LOCK   = 1 << 4,
    NUM_LOCK    = 1 << 5
};

extern int VirtualMouseButtonMap[static_cast<int>(Mouse::COUNT)];

struct _frame {
    int32_t mx, my;
    float   wheel;
};

extern _frame g_frame;

struct GamepadAxisConfig {
    float deadzone;     // e.g. 0.15f
    bool  invert;       // axis inversion
};

MOSS_API void PollKeyboard(INPUT_STATE* io);
MOSS_API void PollGamepads(INPUT_STATE* input);
MOSS_API void PollMouse(INPUT_STATE* io);
MOSS_API void Input_Poll(INPUT_STATE* io_state);



static char* strdup_safe(const char* s);

void RegisterRawInput(HWND hwnd);

void ParseDS4(RAWINPUT* raw);
void ParseDS5(RAWINPUT* raw);

void HandleHIDInput(RAWINPUT* raw);
static wchar_t* convertCharToWchar(const char* str);
#endif // MOSS_PLATFORM_WIN32_H
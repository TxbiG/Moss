#ifndef MOSS_PLATFORM_LINUX_X11_H
#define MOSS_PLATFORM_LINUX_X11_H

#include <Moss/Moss_Platform.h>

#include <unistd.h>
#include <signal.h>
#include <stdint.h>

#include <X11/Xlib.h>
#include <X11/keysym.h>
#include <X11/Xatom.h>
#include <X11/Xresource.h>
#include <X11/Xcursor/Xcursor.h>

//#include <X11/extensions/Xrandr.h>      // The XRandR extension provides mode setting and gamma control
#include <X11/XKBlib.h>                 // The Xkb extension provides improved keyboard support
//#include <X11/extensions/Xinerama.h>    // The Xinerama extension provides legacy monitor indices
#include <X11/extensions/XInput2.h>     // The XInput extension provides raw mouse motion input
#include <X11/extensions/shape.h>       // The Shape extension provides custom window shapes


// X11-specific per-window data
struct Moss_Window {
    Window        handle;               // Native X11 window handle
    Colormap      colormap;             // Colormap tied to GLX visual
    XIC           ic;                   // Input context
    bool          overrideRedirect;   // Used for borderless windows

    // Window state flags
    bool            iconified;
    bool            maximized;
    bool            transparent;

    // Size and position tracking
    int             width, height;
    int             xpos, ypos;

    // Cursor tracking
    int             lastCursorPosX, lastCursorPosY;
    int             warpCursorPosX, warpCursorPosY;

    // Key repeat tracking for ibus quirks
    Time          keyPressTimes[256];

    // GLX context (OpenGL only)
    GLXContext      glxContext;
    GLXWindow       glxWindow;
    XVisualInfo*    visual;
};


typedef struct libraryX11 {
    Display*        display;
    int             screen;
    Window          root;

    // Input context
    XIM             im;
    XContext        context;

    // Cursor
    Cursor          hiddenCursorHandle;

    // Clipboard and keyboard mappings
    char*           clipboardString;
    char            keynames[Moss_Keyboard::MOSS_LAST_KEY + 1][5];
    short int       keycodes[256];
    short int       scancodes[Moss_Keyboard::MOSS_LAST_KEY + 1];

    // Restore cursor pos (for raw mouse)
    double          restoreCursorPosX, restoreCursorPosY;
    Moss_Window*    disabledCursorWindow;

    // X error tracking
    XErrorHandler   errorHandler;
    int             errorCode;

    // Event pipe for wake-up signaling
    int             emptyEventPipe[2];

    // Atoms for window management
    Atom            WM_PROTOCOLS;
    Atom            WM_DELETE_WINDOW;
    Atom            NET_WM_NAME;
    Atom            NET_WM_STATE;
    Atom            NET_WM_STATE_FULLSCREEN;
    Atom            NET_WM_BYPASS_COMPOSITOR;

    // GLX functions (OpenGL only)
#if defined(MOSS_GRAPHICS_OPENGL)
    struct {
        void*           handle;
        PFNGLXGETFBCONFIGSPROC           GetFBConfigs;
        PFNGLXGETFBCONFIGATTRIBPROC      GetFBConfigAttrib;
        PFNGLXGETVISUALFROMFBCONFIGPROC  GetVisualFromFBConfig;
        PFNGLXCREATECONTEXTATTRIBSARBPROC CreateContextAttribsARB;
        PFNGLXSWAPBUFFERSPROC            SwapBuffers;
        PFNGLXDESTROYCONTEXTPROC         DestroyContext;
        PFNGLXMAKECURRENTPROC            MakeCurrent;
        PFNGLXCHOOSEFBCONFIGPROC         ChooseFBConfig;
        PFNGLXCREATEWINDOWPROC           CreateWindow;
        PFNGLXDESTROYWINDOWPROC          DestroyWindow;
    } glx;
#endif // MOSS_GRAPHICS_OPENGL
};

// X11-specific per-monitor data
typedef struct Moss_Monitor {
    RROutput        output;
    RRCrtc          crtc;
    RRMode          oldMode;

    // Index of corresponding Xinerama screen,
    // for EWMH full screen window placement
    int             index;
};

struct Moss_Monitor {
    RROutput id;
    char*    name;
    int      x, y;
    int      width_mm, height_mm;
    float    scale_x, scale_y;
};

// X11-specific per-cursor data
typedef struct Moss_Cursor { Cursor handle; };


struct Moss_InputState {
    uint8_t keys[256];
    uint8_t keys_prev[256];

    uint8_t mouse_buttons[8];
    uint8_t mouse_buttons_prev[8];

    int mouse_x, mouse_y;
    int mouse_scroll;

    struct {
        uint8_t buttons[32];
        float   axes[8];
    } pads[4];
};

#endif // MOSS_PLATFORM_LINUX_X11_H
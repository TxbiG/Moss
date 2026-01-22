#ifndef MOSS_PLATFORM_LINUX_WL_H
#define MOSS_PLATFORM_LINUX_WL_H

#include <Moss/Moss_Platform.h>

#include <wayland-client-core.h>
#include <xkbcommon/xkbcommon.h>
#include <xkbcommon/xkbcommon-compose.h>

typedef struct scaleWayland
{
    struct wl_output*           output;
    int32_t                     factor;
};

typedef struct fallbackEdgeWayland
{
    struct wl_surface*          surface;
    struct wl_subsurface*       subsurface;
    struct wp_viewport*         viewport;
};

// Wayland window
struct Moss_Window {
    int                         width, height;
    int                         fbWidth, fbHeight;
    bool                        visible;
    bool                        maximized;
    bool                        activated;
    bool                        fullscreen;
    bool                        hovered;
    bool                        transparent;
    bool                        scaleFramebuffer;

    struct wl_surface*          surface;
    struct wl_callback*         callback;

    struct {
        struct wl_egl_window*   window;
    } egl;

    struct {
        int                 width, height;
        bool                maximized;
        bool                iconified;
        bool                activated;
        bool                fullscreen;
    } pending;

    struct {
        struct xdg_surface*     surface;
        struct xdg_toplevel*    toplevel;
    } xdg;

    struct {
        struct libdecor_frame*  frame;
    } libdecor;

    Moss_Cursor*                currentCursor;
    double                      cursorPosX, cursorPosY;

    char*                       appId;

    // We need to track the monitors the window spans on to calculate the
    // optimal scaling factor.
    int32_t                     bufferScale;
    scaleWayland*               outputScales;
    size_t                      outputScaleCount;
    size_t                      outputScaleSize;

    struct wp_viewport*             scalingViewport;
    uint32_t                        scalingNumerator;
    struct wp_fractional_scale_v1*  fractionalScale;

    struct zwp_relative_pointer_v1* relativePointer;
    struct zwp_locked_pointer_v1*   lockedPointer;
    struct zwp_confined_pointer_v1* confinedPointer;

    struct zwp_idle_inhibitor_v1*   idleInhibitor;
    struct xdg_activation_token_v1* activationToken;

    struct {
        bool                    decorations;
        struct wl_buffer*           buffer;
        fallbackEdgeWayland    top, left, right, bottom;
        struct wl_surface*          focus;
    } fallback;
};

// Wayland-specific global data
typedef struct librarywl
{
    struct wl_display*          display;
    struct wl_registry*         registry;
    struct wl_compositor*       compositor;
    struct wl_seat*             seat;
    struct wl_pointer*          pointer;
    struct wl_keyboard*         keyboard;

    const wl_registry_listener* registry_listener;

    struct xdg_wm_base*         wmBase;
    struct wp_viewporter*       viewporter;
    struct wp_fractional_scale_manager_v1* fractionalScaleManager;

    struct zwp_relative_pointer_manager_v1* relativePointerManager;
    struct zwp_pointer_constraints_v1* pointerConstraints;

    // Cursor support
    struct wl_cursor_theme*     cursorTheme;
    struct wl_surface*          cursorSurface;

    // EGL for OpenGL
    struct {
        void*                       handle;
        PFN_wl_egl_window_create    window_create;
        PFN_wl_egl_window_destroy   window_destroy;
        PFN_wl_egl_window_resize    window_resize;
    } egl;

    // Only for tracking focus, input
    Moss_Window*                pointerFocus;
    Moss_Window*                keyboardFocus;

    const char*                 tag;
};

// Wayland-specific per-monitor data
typedef struct Moss_Monitor
{
    struct wl_output*           output;
    uint32_t                    name;
    int                         currentMode;

    int                         x;
    int                         y;
    int32_t                     scale;
};

// Wayland-specific per-cursor data
typedef struct Moss_Cursor {
    struct wl_cursor*           cursor;
    struct wl_cursor*           cursorHiDPI;
    struct wl_buffer*           buffer;
    int                         width, height;
    int                         xhot, yhot;
    int                         currentImage;
};

#endif // MOSS_PLATFORM_LINUX_WL_H
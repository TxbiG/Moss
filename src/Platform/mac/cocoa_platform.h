#ifndef MOSS_PLATFORM_MACOS_H
#define MOSS_PLATFORM_MACOS_H

#include <Moss/Platform/platform_intern.h>

#include <Carbon/Carbon.h>
#include <IOKit/hid/IOHIDLib.h>

// TODO: Add Support for Xbox and Playstation (4 & 5) Controllers

// NOTE: All of NSGL was deprecated in the 10.14 SDK. This disables the pointless warnings
#ifndef GL_SILENCE_DEPRECATION
#define GL_SILENCE_DEPRECATION
#endif


#include <unistd.h>
#include <dlfcn.h>
#include <sys/sysinfo.h>
#include <sys/types.h>
#include <sys/param.h>
#include <sys/sysctl.h>

#if defined(__OBJC__)
#import <Cocoa/Cocoa.h>

@class NSWindow;
#else
typedef void* id;
typedef void NSWindow;
#endif

// TODO: Add modifier key detection

struct Moss_Window {
    id              object;
    id              delegate;
    id              view;
    id              layer;

    bool            maximized;
    bool            occluded;
    bool            scaleFramebuffer;

    // Cached window properties to filter out duplicate events
    int             width, height;
    int             fbWidth, fbHeight;
    float           xscale, yscale;

    // The total sum of the distances the cursor has been warped
    // since the last cursor motion event was processed
    // This is kept to counteract Cocoa doing the same internally
    double          cursorWarpDeltaX, cursorWarpDeltaY;
};

// Cocoa-specific global data
//
typedef struct _libraryNS {
    CGEventSourceRef    eventSource;
    id                  delegate;
    bool            cursorHidden;
    TISInputSourceRef   inputSource;
    IOHIDManagerRef     hidManager;
    id                  unicodeData;
    id                  helper;
    id                  keyUpMonitor;
    id                  nibObjects;

    char                keynames[Moss_Keyboard::MOSS_LAST_KEY + 1][17];
    short int           keycodes[256];
    short int           scancodes[Moss_Keyboard::MOSS_LAST_KEY + 1];
    char*               clipboardString;
    CGPoint             cascadePoint;
    // Where to place the cursor when re-enabled
    double              restoreCursorPosX, restoreCursorPosY;
    // The window whose disabled cursor mode is active
    Moss_Window*        disabledCursorWindow;

    struct {
        CFBundleRef     bundle;
        PFN_TISCopyCurrentKeyboardLayoutInputSource CopyCurrentKeyboardLayoutInputSource;
        PFN_TISGetInputSourceProperty GetInputSourceProperty;
        PFN_LMGetKbdType GetKbdType;
        CFStringRef     kPropertyUnicodeKeyLayoutData;
    } tis;
};

// Cocoa-specific per-monitor data
//
typedef struct Moss_Monitor {
    CGDirectDisplayID   displayID;
    CGDisplayModeRef    previousMode;
    uint32_t            unitNumber;
    id                  screen;
    double              fallbackRefreshRate;
};

// Cocoa-specific per-cursor data
//
typedef struct Moss_Cursor {
    id              object;
};



static std::array<bool, MAX_KEYS> currentKeys = {};
static std::array<bool, MAX_KEYS> previousKeys = {};

static std::array<bool, MAX_MOUSE> currentMouse = {};
static std::array<bool, MAX_MOUSE> previousMouse = {};

static std::array<bool, MAX_GAMEPAD> currentGamepad = {};
static std::array<bool, MAX_GAMEPAD> previousGamepad = {};

@interface MossInputView : NSView
@end

@implementation MossInputView

- (BOOL)acceptsFirstResponder {
    return YES;
}

- (void)keyDown:(NSEvent*)event {
    Keyboard key = MapMacKeyCode(event.keyCode);
    currentKeys[static_cast<size_t>(key)] = true;
}
- (void)keyUp:(NSEvent*)event {
    Keyboard key = MapMacKeyCode(event.keyCode);
    currentKeys[static_cast<size_t>(key)] = false;
}

- (void)mouseDown:(NSEvent*)event {
    Mouse btn = static_cast<Mouse>(event.buttonNumber);
    if ((size_t)btn < MAX_MOUSE) {
        currentMouse[static_cast<size_t>(btn)] = true;
    }
}
- (void)mouseUp:(NSEvent*)event {
    Mouse btn = static_cast<Mouse>(event.buttonNumber);
    if ((size_t)btn < MAX_MOUSE) {
        currentMouse[static_cast<size_t>(btn)] = false;
    }
}
@end


static char* strdup_safe(const char* s) {
    if (!s) return NULL;
    size_t len = strlen(s);
    char* r = (char*)malloc(len + 1);
    if (r) memcpy(r, s, len + 1);
    return r;
}

Moss_Locale* Moss_GetLocale() {
    CFLocaleRef loc = CFLocaleCopyCurrent();
    if (!loc) {
        locale->language = strdup_safe("en");
        locale->country  = strdup_safe("US");
        return locale;
    }

    // Language
    CFStringRef lang = (CFStringRef)CFLocaleGetValue(loc, kCFLocaleLanguageCode);
    char buf[16] = {0};
    if (lang && CFStringGetCString(lang, buf, sizeof(buf), kCFStringEncodingUTF8)) {
        locale->language = strdup_safe(buf);
    } else {
        locale->language = strdup_safe("en");
    }

    // Country
    CFStringRef country = (CFStringRef)CFLocaleGetValue(loc, kCFLocaleCountryCode);
    memset(buf, 0, sizeof(buf));
    if (country && CFStringGetCString(country, buf, sizeof(buf), kCFStringEncodingUTF8)) {
        locale->country = strdup_safe(buf);
    } else {
        locale->country = strdup_safe("US");
    }

    CFRelease(loc);
}


int Moss_GetAvailableCPUCores(void) {
    int mib[2] = {CTL_HW, HW_AVAILCPU};
    int cores = 0;
    size_t len = sizeof(cores);
    sysctl(mib, 2, &cores, &len, NULL, 0);
    if (cores < 1)
    {
        mib[1] = HW_NCPU;
        sysctl(mib, 2, &cores, &len, NULL, 0);
        if (cores < 1) cores = 1;
    }
    return cores;
}

/**
 * @brief Determine the L1 cache line size of the CPU (in bytes).
 * @return Cache line size, or 64 as a safe default.
 */
MOSS_API int Moss_GetCPUCacheLineSize(void) {
    size_t line_size = 0;
    size_t size = sizeof(line_size);
    sysctlbyname("hw.cachelinesize", &line_size, &size, 0, 0);
    return (int)((line_size > 0) ? line_size : 64);
}

/**
 * @brief Get the total amount of system RAM (in MB).
 * @return Total memory in megabytes.
 */
int Moss_GetSystemRAM(void) {
    int mib[2] = {CTL_HW, HW_MEMSIZE};
    uint64_t memsize = 0;
    size_t len = sizeof(memsize);
    sysctl(mib, 2, &memsize, &len, NULL, 0);
    return (int)(memsize / (1024 * 1024));
}

// ---------------------------------------------------------------------------
// Dynamic Library Management
// ---------------------------------------------------------------------------
void* Moss_LoadDynamicLibrary(const char* lib_path) { return dlopen(lib_path, RTLD_NOW | RTLD_LOCAL); }
void* Moss_GetLibrarySymbol(void* handle, const char* symbol_name) { return dlsym(handle, symbol_name); }
void Moss_UnloadDynamicLibrary(void* handle) { if (handle) dlclose(handle); }



void Moss_UpdateInputStates();
#endif // MOSS_PLATFORM_MACOS_H
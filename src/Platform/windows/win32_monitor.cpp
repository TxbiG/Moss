#include <Moss/Moss_Platform.h>
#include <Moss/Moss_stdinc.h>

#include "win32_platform.h"

#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <math.h>
#include <highlevelmonitorconfigurationapi.h>
#include <physicalmonitorenumerationapi.h>  // Get Monitor links with "Dxva2.lib"
#include <shtypes.h>
#include <ShellScalingApi.h>

#pragma comment(lib, "Dxva2.lib")
#pragma comment(lib, "Shcore.lib")


struct Moss_Monitor {
    DISPLAY_DEVICEA displayDevice;  // DISPLAY_DEVICE info
    HMONITOR handle;                // Monitor handle
    MONITORINFOEXA monitorInfo;     // Monitor rectangle, flags, device name
    /*
    WCHAR               adapterName[32];
    WCHAR               displayName[32];
    char                publicAdapterName[32];
    char                publicDisplayName[32];
    bool            modesPruned;
    bool            modeChanged;
    */
};

static Moss_Monitor primaryMonitor = {};
static Moss_Monitor secondaryMonitor = {};
static bool monitorsInitialized = false;

static Moss_MonitorCallback g_monitorCallback = nullptr;


BOOL CALLBACK MonitorEnumProc(HMONITOR handle, HDC hdc, LPRECT rect, LPARAM data) {
    Moss_Monitor** monitors = (Moss_Monitor**)data;
    MONITORINFOEXA mi = {};
    mi.cbSize = sizeof(MONITORINFOEXA);
    if (!GetMonitorInfoA(handle, (LPMONITORINFO)&mi)) return TRUE;

    DISPLAY_DEVICEA dd = {};
    dd.cb = sizeof(DISPLAY_DEVICEA);
    EnumDisplayDevicesA(NULL, 0, &dd, 0);

    if (mi.dwFlags & MONITORINFOF_PRIMARY) {
        monitors[0]->handle = handle;
        monitors[0]->monitorInfo = mi;
        monitors[0]->displayDevice = dd;
    } else if (monitors[1]->handle == NULL) {
        monitors[1]->handle = handle;
        monitors[1]->monitorInfo = mi;
        monitors[1]->displayDevice = dd;
    }
    return (monitors[0]->handle && monitors[1]->handle) ? FALSE : TRUE;
}

void Moss_InitMonitors() {
    Moss_Monitor* monitors[2] = { &primaryMonitor, &secondaryMonitor };
    EnumDisplayMonitors(NULL, NULL, MonitorEnumProc, (LPARAM)monitors);
    monitorsInitialized = true;
}


Moss_Monitor* Moss_GetPrimaryMonitor() {
    if (!monitorsInitialized) Moss_InitMonitors();
    return &primaryMonitor;
}
Moss_Monitor* Moss_GetSecondaryMonitor() {
    if (!monitorsInitialized) Moss_InitMonitors();
    return &secondaryMonitor;
}

// Monitor
void Moss_GetMonitorPhysicalSize(Moss_Monitor* monitor, int* width_mm, int* height_mm) {
    DWORD count = 0;
    if (!GetNumberOfPhysicalMonitorsFromHMONITOR(monitor->handle, &count) || count == 0) { return; }

    PHYSICAL_MONITOR* physicalMonitors = (PHYSICAL_MONITOR*)malloc(sizeof(PHYSICAL_MONITOR) * count);
    if (!physicalMonitors) return;

    if (GetPhysicalMonitorsFromHMONITOR(monitor->handle, count, physicalMonitors)) {
        for (DWORD i = 0; i < count; i++) {
            DWORD minSize = 0, maxSize = 0, displaySize = 0;
            
            // Get width in millimeters
            if (!GetMonitorDisplayAreaSize(physicalMonitors[i].hPhysicalMonitor, MC_WIDTH, &minSize, &maxSize, &displaySize)) {
                continue; // Failed, try next monitor
            }
            *width_mm = (int)displaySize;

            // Get height in millimeters
            if (!GetMonitorDisplayAreaSize(physicalMonitors[i].hPhysicalMonitor, MC_HEIGHT, &minSize, &maxSize, &displaySize)) {
                continue; // Failed, try next monitor
            }
            *height_mm = (int)displaySize;

            break; // Got size successfully, exit loop
        }
        DestroyPhysicalMonitors(count, physicalMonitors);
    }
    free(physicalMonitors);
}

void Moss_GetMonitorContentScale(Moss_Monitor* monitor, float* xscale, float* yscale) {
    UINT dpiX = 96, dpiY = 96;  // default DPI (100% scaling)

    // Check if GetDpiForMonitor is available (Windows 8.1+)
    HMODULE shcore = LoadLibraryA("Shcore.dll");
    if (shcore) {
        typedef HRESULT(WINAPI *GetDpiForMonitorFunc)(HMONITOR, int, UINT*, UINT*);
        GetDpiForMonitorFunc getDpiForMonitor = 
            (GetDpiForMonitorFunc)GetProcAddress(shcore, "GetDpiForMonitor");

        if (getDpiForMonitor) {
            HRESULT hr = getDpiForMonitor(monitor->handle, MDT_EFFECTIVE_DPI, &dpiX, &dpiY);
            if (FAILED(hr)) {
                dpiX = dpiY = 96;  // fallback to default if call fails
            }
        }
        FreeLibrary(shcore);
    }

    *xscale = dpiX / 96.0f;
    *yscale = dpiY / 96.0f;
}

void Moss_GetMonitorPosition(Moss_Monitor* monitor, int* x, int* y) {
    MONITORINFO mi = {};
    mi.cbSize = sizeof(mi);
    if (GetMonitorInfo(monitor->handle, &mi)) {
        *x = mi.rcMonitor.left;
        *y = mi.rcMonitor.top;
    }
}

const char* Moss_GetMonitorName(Moss_Monitor* monitor) { return monitor->displayDevice.DeviceName; }

Moss_GammaRamp* Moss_GetGammaRamp(Moss_Monitor* monitor) {
    if (!monitor) { return NULL; }

    Moss_GammaRamp* outRamp = new (std::nothrow) Moss_GammaRamp{};
    if (!outRamp) { return NULL; }

    HDC hdc = CreateDCA(NULL, monitor->displayDevice.DeviceName, NULL, NULL);
    if (!hdc) { delete outRamp; return NULL; }

    WORD tempRamp[3 * 256]{};

    if (!GetDeviceGammaRamp(hdc, tempRamp)) {
        DeleteDC(hdc);
        delete outRamp;
        return NULL;
    }

    outRamp->size = 256;

    outRamp->red   = new (std::nothrow) uint8_t[outRamp->size];
    outRamp->green = new (std::nothrow) uint8_t[outRamp->size];
    outRamp->blue  = new (std::nothrow) uint8_t[outRamp->size];

    if (!outRamp->red || !outRamp->green || !outRamp->blue) {
        delete[] outRamp->red;
        delete[] outRamp->green;
        delete[] outRamp->blue;
        delete outRamp;
        DeleteDC(hdc);
        return NULL;
    }

    for (uint32_t i = 0; i < outRamp->size; ++i) {
        // Convert Windows' 16-bit gamma values to Moss' 8-bit range.
        outRamp->red[i]   = static_cast<uint8_t>(tempRamp[i] >> 8);
        outRamp->green[i] = static_cast<uint8_t>(tempRamp[256 + i] >> 8);
        outRamp->blue[i]  = static_cast<uint8_t>(tempRamp[512 + i] >> 8);
    }

    DeleteDC(hdc);
    return outRamp;
}

void Moss_SetGammaRamp(Moss_Monitor* monitor, const Moss_GammaRamp* gammaRamp) {
    if (!monitor || !gammaRamp) { return; }
    if (gammaRamp->size != 256U) { return; }
    if (!gammaRamp->red || !gammaRamp->green || !gammaRamp->blue) { return; }

    HDC hdc = CreateDCA(NULL, monitor->displayDevice.DeviceName, NULL, NULL);

    if (!hdc) { return; }

    WORD ramp[3][256]{};

    for (uint32_t i = 0; i < 256; ++i) {
        ramp[0][i] = static_cast<WORD>(gammaRamp->red[i]) * 257;
        ramp[1][i] = static_cast<WORD>(gammaRamp->green[i]) * 257;
        ramp[2][i] = static_cast<WORD>(gammaRamp->blue[i]) * 257;
    }

    const BOOL result = SetDeviceGammaRamp(hdc, ramp);

    DeleteDC(hdc);
    if (!result) { return; }
}

void Moss_SetGamma(Moss_Monitor* monitor, float gamma) {
    if (!monitor || gamma <= 0.0f) {  return; }
    HDC hdc = CreateDCA(NULL, monitor->displayDevice.DeviceName, NULL, NULL);
    if (!hdc)
        return;

    WORD gammaRamp[3][256]{};

    for (int i = 0; i < 256; ++i) {
        const float normalized = static_cast<float>(i) / 255.0f;
        const float corrected = powf(normalized, 1.0f / gamma);
        const int value = static_cast<int>(corrected * 65535.0f + 0.5f);
        const WORD rampValue = static_cast<WORD>(value < 0 ? 0 : value > 65535 ? 65535 : value);

        gammaRamp[0][i] = rampValue;
        gammaRamp[1][i] = rampValue;
        gammaRamp[2][i] = rampValue;
    }

    SetDeviceGammaRamp(hdc, gammaRamp);

    DeleteDC(hdc);
}

Moss_Monitor* Moss_MonitorGetPrimary() { return Moss_GetPrimaryMonitor(); }

Moss_Monitor* Moss_MonitorGetSecondary() {
    Moss_Monitor* monitor = Moss_GetSecondaryMonitor();
    return monitor && monitor->handle ? monitor : nullptr;
}

void Moss_MonitorGetPhysicalSize(Moss_Monitor* monitor, int* width_mm, int* height_mm) {
    if (!monitor) return;
    Moss_GetMonitorPhysicalSize(monitor, width_mm, height_mm); // passing pointer to by-value param!
}

void Moss_MonitorGetContentScale(Moss_Monitor* monitor, float* xscale, float* yscale) {
    if (!monitor) return;
    Moss_GetMonitorContentScale(monitor, xscale, yscale);
}

void Moss_MonitorGetPosition(Moss_Monitor* monitor, int* x, int* y) {
    if (!monitor) return;
    Moss_GetMonitorPosition(monitor, x, y);
}

const char* Moss_MonitorGetName(Moss_Monitor* monitor) {
    return monitor ? Moss_GetMonitorName(monitor) : nullptr;
}

void Moss_MonitorSetGammaRamp(Moss_Monitor* monitor, const Moss_GammaRamp* gammaRamp) {
    if (!monitor) return;
    Moss_SetGammaRamp(monitor, gammaRamp);
}

Moss_GammaRamp* Moss_MonitorGetGammaRamp(Moss_Monitor* monitor) {
    return monitor ? Moss_GetGammaRamp(monitor) : nullptr;
}

void Moss_MonitorSetGamma(Moss_Monitor* monitor, float gamma) {
    if (!monitor) return;
    Moss_SetGamma(monitor, gamma);
}

static int CountVideoModes(const char* deviceName) {
    int count = 0;
    DEVMODEA dm = {};
    dm.dmSize = sizeof(DEVMODEA);
    for (int i = 0; EnumDisplaySettingsA(deviceName, i, &dm); i++) { count++; }
    return count;
}

Moss_VideoMode* Moss_GetVideoModes(Moss_Monitor* monitor, int* outCount) {
    int count = CountVideoModes(monitor->displayDevice.DeviceName);
    if (outCount) *outCount = count;

    if (count == 0) { return NULL; }

    Moss_VideoMode* modes = (Moss_VideoMode*)malloc(sizeof(Moss_VideoMode) * count);
    if (!modes) return NULL;

    DEVMODEA devMode = {0};
    devMode.dmSize = sizeof(DEVMODEA);
    for (int i = 0, j = 0; EnumDisplaySettingsA(monitor->displayDevice.DeviceName, i, &devMode); i++) {
        Moss_VideoMode* m = &modes[j++];
        m->width = devMode.dmPelsWidth;
        m->height = devMode.dmPelsHeight;
        m->refreshRate = devMode.dmDisplayFrequency;
        // Bit depth logic
        if (devMode.dmBitsPerPel == 16) { m->redBits = 5; m->greenBits = 6; m->blueBits = 5;  } 
        else if (devMode.dmBitsPerPel == 24 || devMode.dmBitsPerPel == 32) {  m->redBits = m->greenBits = m->blueBits = 8;  } 
        else {  m->redBits = m->greenBits = m->blueBits = devMode.dmBitsPerPel / 3;  }
    }

    return modes;
}

void Moss_SetMonitorCallback(Moss_MonitorCallback callback) { g_monitorCallback = callback; }
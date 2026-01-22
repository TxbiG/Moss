#include <Moss/Moss_Platform.h>
#include <Moss/Moss_stdinc.h>

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

//TODO: void Moss_GetMonitorPos(Monitor, xpos, ypos), Moss_GetMonitorWorkarea(monitor, xpos, ypos);

struct Moss_Monitor {
    DISPLAY_DEVICEA displayDevice;  // DISPLAY_DEVICE info
    HMONITOR handle;              // Monitor handle
    MONITORINFOEXA monitorInfo;     // Monitor rectangle, flags, device name
};

/*
typedef struct _monitorWin32
{
    HMONITOR            handle;
    // This size matches the static size of DISPLAY_DEVICE.DeviceName
    WCHAR               adapterName[32];
    WCHAR               displayName[32];
    char                publicAdapterName[32];
    char                publicDisplayName[32];
    bool            modesPruned;
    bool            modeChanged;
} _monitorWin32;
*/

static Moss_Monitor primaryMonitor = { 0 };
static Moss_Monitor secondaryMonitor = { 0 };

static Moss_MonitorCallback g_monitorCallback = nullptr;

BOOL CALLBACK MonitorEnumProc(HMONITOR handle, HDC hdc, LPRECT rect, LPARAM data) {
    Moss_Monitor* monitors = (Moss_Monitor*)data;

    MONITORINFOEXA mi = { .cbSize = sizeof(MONITORINFOEXA) };
    if (!GetMonitorInfoA(handle, (LPMONITORINFO)&mi)) {
        return TRUE; // continue enumeration
    }

    DISPLAY_DEVICEA dd = { .cb = sizeof(DISPLAY_DEVICEA) };
    EnumDisplayDevicesA(NULL, 0, &dd, 0);

    if (mi.dwFlags & MONITORINFOF_PRIMARY) {
        monitors[0].handle = handle;
        monitors[0].monitorInfo = mi;
        monitors[0].displayDevice = dd;
    } else if (monitors[1].handle == NULL) {
        monitors[1].handle = handle;
        monitors[1].monitorInfo = mi;
        monitors[1].displayDevice = dd;
    }

    return monitors[0].handle && monitors[1].handle ? FALSE : TRUE;
}

void Moss_InitMonitors() {
    Moss_Monitor monitors[2] = { 0 };
    EnumDisplayMonitors(NULL, NULL, MonitorEnumProc, (LPARAM)monitors);

    primaryMonitor = monitors[0];
    secondaryMonitor = monitors[1];
}

Moss_Monitor* Moss_GetPrimaryMonitor() {
    if (primaryMonitor.handle == NULL) {
        Moss_InitMonitors();
    }
    return &primaryMonitor;
}

Moss_Monitor* Moss_GetSecondaryMonitor() {
    if (secondaryMonitor.handle == NULL) {
        Moss_InitMonitors();
    }
    return &secondaryMonitor;
}

// Monitor
void Moss_GetMonitorPhysicalSize(Moss_Monitor monitor, int* width_mm, int* height_mm)
{
    DWORD count = 0;
    if (!GetNumberOfPhysicalMonitorsFromHMONITOR(monitor.handle, &count) || count == 0) { return; }

    PHYSICAL_MONITOR* physicalMonitors = (PHYSICAL_MONITOR*)malloc(sizeof(PHYSICAL_MONITOR) * count);
    if (!physicalMonitors) return;

    if (GetPhysicalMonitorsFromHMONITOR(monitor.handle, count, physicalMonitors)) {
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

void Moss_GetMonitorContentScale(Moss_Monitor monitor, float* xscale, float* yscale)
{
    UINT dpiX = 96, dpiY = 96;  // default DPI (100% scaling)

    // Check if GetDpiForMonitor is available (Windows 8.1+)
    HMODULE shcore = LoadLibraryA("Shcore.dll");
    if (shcore) {
        typedef HRESULT(WINAPI *GetDpiForMonitorFunc)(HMONITOR, int, UINT*, UINT*);
        GetDpiForMonitorFunc getDpiForMonitor = 
            (GetDpiForMonitorFunc)GetProcAddress(shcore, "GetDpiForMonitor");

        if (getDpiForMonitor) {
            HRESULT hr = getDpiForMonitor(monitor.handle, MDT_EFFECTIVE_DPI, &dpiX, &dpiY);
            if (FAILED(hr)) {
                dpiX = dpiY = 96;  // fallback to default if call fails
            }
        }
        FreeLibrary(shcore);
    }

    *xscale = dpiX / 96.0f;
    *yscale = dpiY / 96.0f;
}

void Moss_GetMonitorPosition(Moss_Monitor monitor, int* x, int* y)
{
    MONITORINFO mi = { .cbSize = sizeof(mi) };
    if (GetMonitorInfo(monitor.handle, &mi)) {
        *x = mi.rcMonitor.left;
        *y = mi.rcMonitor.top;
    }
}

const char* Moss_GetMonitorName(Moss_Monitor monitor) { return monitor.monitorInfo.szDevice; }

Moss_GammaRamp* Moss_GetGammaRamp(Moss_Monitor monitor)
{
    Moss_GammaRamp* outRamp = malloc(sizeof(Moss_GammaRamp));
    if (!outRamp) return NULL;  // allocation failed

    HDC hdc = CreateDC(NULL, monitor.monitorInfo.szDevice, NULL, NULL);
    if (!hdc) {
        free(outRamp);
        return NULL;
    }

    WORD tempRamp[3 * 256];
    if (!GetDeviceGammaRamp(hdc, tempRamp)) {
        free(outRamp);
        DeleteDC(hdc);
        return NULL;
    }

    outRamp->size = 256;
    for (int i = 0; i < 256; i++) {
        outRamp->red[i] = tempRamp[i];
        outRamp->green[i] = tempRamp[256 + i];
        outRamp->blue[i] = tempRamp[512 + i];
    }

    DeleteDC(hdc);
    return outRamp;
}

void Moss_SetGammaRamp(Moss_Monitor monitor, const Moss_GammaRamp* gammaRamp) {
    if (!gammaRamp || gammaRamp->size != 256 ||
        !gammaRamp->red || !gammaRamp->green || !gammaRamp->blue) {
        return; // Invalid input
    }

    HDC hdc = CreateDCA("DISPLAY", monitor.monitorInfo.szDevice, NULL, NULL);
    if (!hdc) return;

    WORD ramp[3 * 256]; // Windows expects RGB WORD arrays packed together

    for (int i = 0; i < 256; ++i) {
        ramp[i]         = gammaRamp->red[i];
        ramp[256 + i]   = gammaRamp->green[i];
        ramp[512 + i]   = gammaRamp->blue[i];
    }

    SetDeviceGammaRamp(hdc, ramp);

    DeleteDC(hdc);
}

void Moss_SetGamma(Moss_Monitor monitor, float gamma)
{
    if (gamma <= 0.0f) return; // Invalid gamma

    // Get device context for the monitor's device name
    HDC hdc = CreateDCA("DISPLAY", monitor.displayDevice.DeviceName, NULL, NULL);
    if (!hdc) return;

    WORD gammaRamp[3][256];

    // Fill the gamma ramp
    for (int i = 0; i < 256; i++) {
        // Normalize i to [0,1]
        float normalized = i / 255.0f;
        // Apply gamma correction
        int val = (int)(powf(normalized, 1.0f / gamma) * 65535.0f + 0.5f);

        if (val > 65535) val = 65535;
        if (val < 0) val = 0;

        gammaRamp[0][i] = (WORD)val; // Red
        gammaRamp[1][i] = (WORD)val; // Green
        gammaRamp[2][i] = (WORD)val; // Blue
    }

    // Set the gamma ramp
    SetDeviceGammaRamp(hdc, gammaRamp);

    DeleteDC(hdc);
}

static int CountVideoModes(const char* deviceName) {
    int count = 0;
    DEVMODEA  dm = { .dmSize = sizeof(DEVMODEA) };
    for (int i = 0; EnumDisplaySettingsA(deviceName, i, &dm); i++) { count++; }
    return count;
}

Moss_VideoMode* Moss_GetVideoModes(Moss_Monitor monitor, int* outCount) {
    int count = CountVideoModes(monitor.displayDevice.DeviceName);
    if (outCount) *outCount = count;

    if (count == 0) return NULL;

    Moss_VideoMode* modes = (Moss_VideoMode*)malloc(sizeof(Moss_VideoMode) * count);
    if (!modes) return NULL;

    DEVMODEA devMode = {0};
    devMode.dmSize = sizeof(DEVMODEA);
    for (int i = 0, j = 0; EnumDisplaySettingsA(monitor.displayDevice.DeviceName, i, &devMode); i++) {
        Moss_VideoMode* m = &modes[j++];
        m->width = devMode.dmPelsWidth;
        m->height = devMode.dmPelsHeight;
        m->refreshRate = devMode.dmDisplayFrequency;
        // Bit depth logic
        if (devMode.dmBitsPerPel == 16) { 
            m->redBits = 5; m->greenBits = 6; m->blueBits = 5; 
        } else if (devMode.dmBitsPerPel == 24 || devMode.dmBitsPerPel == 32) { 
            m->redBits = m->greenBits = m->blueBits = 8; 
        } else { 
            m->redBits = m->greenBits = m->blueBits = devMode.dmBitsPerPel / 3; 
        }
    }

    return modes;
}

Moss_VideoMode Moss_GetCurrentVideoMode(Moss_Monitor monitor) {
    Moss_VideoMode mode = {0};
    DEVMODEA dm = { .dmSize = sizeof(DEVMODEA) };

    if (EnumDisplaySettingsA(monitor.monitorInfo.szDevice, ENUM_CURRENT_SETTINGS, &dm)) {
        mode.width = dm.dmPelsWidth;
        mode.height = dm.dmPelsHeight;
        mode.refreshRate = dm.dmDisplayFrequency;

        if (dm.dmBitsPerPel == 16) { mode.redBits = 5; mode.greenBits = 6; mode.blueBits = 5; } 
        else if (dm.dmBitsPerPel == 24 || dm.dmBitsPerPel == 32) { mode.redBits = mode.greenBits = mode.blueBits = 8; } 
        else { mode.redBits = mode.greenBits = mode.blueBits = dm.dmBitsPerPel / 3; }
    }

    return mode;
}

void Moss_SetMonitorCallback(Moss_MonitorCallback callback) {

}
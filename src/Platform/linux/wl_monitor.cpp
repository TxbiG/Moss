#include <Moss/Platform/Linux/wl_platform.h>
#include "wayland-client-protocol.h"

struct Moss_Monitor {
    int x;
    int y;
    int width;
    int height;
    int width_mm;
    int height_mm;
    float scale_x;
    float scale_y;
    const char* name;
};

static Moss_Monitor g_primary = {0, 0, 0, 0, 0, 0, 1.0f, 1.0f, "Wayland Primary Monitor"};
static Moss_MonitorCallback g_monitorCallback = nullptr;

Moss_Monitor* Moss_GetPrimaryMonitor() { return &g_primary; }
Moss_Monitor* Moss_GetSecondaryMonitor() { return nullptr; }
void Moss_GetMonitorPhysicalSize(Moss_Monitor* monitor, int* width_mm, int* height_mm) { if (width_mm) *width_mm = monitor ? monitor->width_mm : 0; if (height_mm) *height_mm = monitor ? monitor->height_mm : 0; }
void Moss_GetMonitorContentScale(Moss_Monitor* monitor, float* xscale, float* yscale) { if (xscale) *xscale = monitor ? monitor->scale_x : 1.0f; if (yscale) *yscale = monitor ? monitor->scale_y : 1.0f; }
void Moss_GetMonitorPosition(Moss_Monitor* monitor, int* x, int* y) { if (x) *x = monitor ? monitor->x : 0; if (y) *y = monitor ? monitor->y : 0; }
const char* Moss_GetMonitorName(Moss_Monitor* monitor) { return monitor && monitor->name ? monitor->name : "Wayland Monitor"; }
void Moss_SetGammaRamp(Moss_Monitor* monitor, const Moss_GammaRamp* gammaRamp) { (void)monitor; (void)gammaRamp; }
Moss_GammaRamp* Moss_GetGammaRamp(Moss_Monitor* monitor) { (void)monitor; return nullptr; }
void Moss_SetGamma(Moss_Monitor* monitor, float gamma) { (void)monitor; (void)gamma; }
void Moss_SetMonitorCallback(Moss_MonitorCallback callback) { g_monitorCallback = callback; }

Moss_Monitor* Moss_MonitorGetPrimary() { return Moss_GetPrimaryMonitor(); }
Moss_Monitor* Moss_MonitorGetSecondary() { return Moss_GetSecondaryMonitor(); }
void Moss_MonitorGetPhysicalSize(Moss_Monitor* monitor, int* width_mm, int* height_mm) { Moss_GetMonitorPhysicalSize(monitor, width_mm, height_mm); }
void Moss_MonitorGetContentScale(Moss_Monitor* monitor, float* xscale, float* yscale) { Moss_GetMonitorContentScale(monitor, xscale, yscale); }
void Moss_MonitorGetPosition(Moss_Monitor* monitor, int* x, int* y) { Moss_GetMonitorPosition(monitor, x, y); }
bool Moss_MonitorGetRect(Moss_Monitor* monitor, Moss_MonitorRect* out_rect) { if (!monitor || !out_rect) return false; out_rect->x = monitor->x; out_rect->y = monitor->y; out_rect->width = monitor->width; out_rect->height = monitor->height; return true; }
bool Moss_MonitorGetWorkArea(Moss_Monitor* monitor, Moss_MonitorRect* out_rect) { return Moss_MonitorGetRect(monitor, out_rect); }
const char* Moss_MonitorGetName(Moss_Monitor* monitor) { return Moss_GetMonitorName(monitor); }
void Moss_MonitorSetGammaRamp(Moss_Monitor* monitor, const Moss_GammaRamp* gammaRamp) { Moss_SetGammaRamp(monitor, gammaRamp); }
Moss_GammaRamp* Moss_MonitorGetGammaRamp(Moss_Monitor* monitor) { return Moss_GetGammaRamp(monitor); }
void Moss_MonitorSetGamma(Moss_Monitor* monitor, float gamma) { Moss_SetGamma(monitor, gamma); }
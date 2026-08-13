#include <Moss/Moss_Platform.h>

#include <ApplicationServices/ApplicationServices.h>

struct Moss_Monitor {
    CGDirectDisplayID display;
    CGRect bounds;
    const char* name;
};

static Moss_Monitor g_primary;
static Moss_MonitorCallback g_monitorCallback = nullptr;

static Moss_Monitor* Moss_MacPrimaryMonitor() {
    g_primary.display = CGMainDisplayID();
    g_primary.bounds = CGDisplayBounds(g_primary.display);
    g_primary.name = "Main Display";
    return &g_primary;
}

Moss_Monitor* Moss_GetPrimaryMonitor() { return Moss_MacPrimaryMonitor(); }
Moss_Monitor* Moss_GetSecondaryMonitor() { return nullptr; }
void Moss_GetMonitorPhysicalSize(Moss_Monitor* monitor, int* width_mm, int* height_mm) { if (width_mm) *width_mm = 0; if (height_mm) *height_mm = 0; (void)monitor; }
void Moss_GetMonitorContentScale(Moss_Monitor* monitor, float* xscale, float* yscale) { if (xscale) *xscale = 1.0f; if (yscale) *yscale = 1.0f; (void)monitor; }
void Moss_GetMonitorPosition(Moss_Monitor* monitor, int* x, int* y) { if (!monitor) monitor = Moss_MacPrimaryMonitor(); if (x) *x = (int)monitor->bounds.origin.x; if (y) *y = (int)monitor->bounds.origin.y; }
const char* Moss_GetMonitorName(Moss_Monitor* monitor) { (void)monitor; return "Main Display"; }
void Moss_SetGammaRamp(Moss_Monitor* monitor, const Moss_GammaRamp* gammaRamp) { (void)monitor; (void)gammaRamp; }
Moss_GammaRamp* Moss_GetGammaRamp(Moss_Monitor* monitor) { (void)monitor; return nullptr; }
void Moss_SetGamma(Moss_Monitor* monitor, float gamma) { (void)monitor; (void)gamma; }
void Moss_SetMonitorCallback(Moss_MonitorCallback callback) { g_monitorCallback = callback; }

Moss_Monitor* Moss_MonitorGetPrimary() { return Moss_GetPrimaryMonitor(); }
Moss_Monitor* Moss_MonitorGetSecondary() { return Moss_GetSecondaryMonitor(); }
void Moss_MonitorGetPhysicalSize(Moss_Monitor* monitor, int* width_mm, int* height_mm) { Moss_GetMonitorPhysicalSize(monitor, width_mm, height_mm); }
void Moss_MonitorGetContentScale(Moss_Monitor* monitor, float* xscale, float* yscale) { Moss_GetMonitorContentScale(monitor, xscale, yscale); }
void Moss_MonitorGetPosition(Moss_Monitor* monitor, int* x, int* y) { Moss_GetMonitorPosition(monitor, x, y); }
bool Moss_MonitorGetRect(Moss_Monitor* monitor, Moss_MonitorRect* out_rect) { if (!out_rect) return false; if (!monitor) monitor = Moss_MacPrimaryMonitor(); out_rect->x = (int)monitor->bounds.origin.x; out_rect->y = (int)monitor->bounds.origin.y; out_rect->width = (int)monitor->bounds.size.width; out_rect->height = (int)monitor->bounds.size.height; return true; }
bool Moss_MonitorGetWorkArea(Moss_Monitor* monitor, Moss_MonitorRect* out_rect) { return Moss_MonitorGetRect(monitor, out_rect); }
const char* Moss_MonitorGetName(Moss_Monitor* monitor) { return Moss_GetMonitorName(monitor); }
void Moss_MonitorSetGammaRamp(Moss_Monitor* monitor, const Moss_GammaRamp* gammaRamp) { Moss_SetGammaRamp(monitor, gammaRamp); }
Moss_GammaRamp* Moss_MonitorGetGammaRamp(Moss_Monitor* monitor) { return Moss_GetGammaRamp(monitor); }
void Moss_MonitorSetGamma(Moss_Monitor* monitor, float gamma) { Moss_SetGamma(monitor, gamma); }
#include <Moss/Platform/Linux/x11_platform.h>

#include <X11/Xlib.h>
#include <X11/extensions/Xrandr.h>

#include <limits.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <assert.h>

static Display* g_display = NULL;
static Window   g_root;
static Moss_MonitorCallback g_monitorCallback = NULL;

// Helper: Load display if not opened yet
static void ensure_display() {
    if (!g_display)
        g_display = XOpenDisplay(NULL);

    g_root = DefaultRootWindow(g_display);
}

// Helper: Create Moss_Monitor struct from XRandR output info
static Moss_Monitor* create_monitor(RROutput output, XRROutputInfo* oinfo, XRRCrtcInfo* cinfo) {
    Moss_Monitor* m = (Moss_Monitor*)calloc(1, sizeof(Moss_Monitor));
    m->id = output;

    m->name = strdup(oinfo->name);

    m->x = cinfo->x;
    m->y = cinfo->y;
    m->width_mm  = oinfo->mm_width;
    m->height_mm = oinfo->mm_height;

    // crude scaling approximation:
    // Assume DPI ≈ 96 means scale = 1.0
    float dpi_x = (oinfo->mm_width > 0)  ? (cinfo->width / (oinfo->mm_width / 25.4f)) : 96.0f;
    float dpi_y = (oinfo->mm_height > 0) ? (cinfo->height / (oinfo->mm_height / 25.4f)) : 96.0f;

    m->scale_x = dpi_x / 96.0f;
    m->scale_y = dpi_y / 96.0f;

    return m;
}

// enumerate monitors
static Moss_Monitor** enumerate_monitors(int* count) {
    ensure_display();

    XRRScreenResources* res = XRRGetScreenResources(g_display, g_root);
    Moss_Monitor** list = (Moss_Monitor**)calloc(res->noutput, sizeof(Moss_Monitor*));
    int index = 0;

    for (int i = 0; i < res->noutput; i++) {
        XRROutputInfo* oinfo = XRRGetOutputInfo(g_display, res, res->outputs[i]);
        if (oinfo->connection == RR_Connected && oinfo->crtc != 0)
        {
            XRRCrtcInfo* cinfo = XRRGetCrtcInfo(g_display, res, oinfo->crtc);
            list[index++] = create_monitor(res->outputs[i], oinfo, cinfo);
            XRRFreeCrtcInfo(cinfo);
        }
        XRRFreeOutputInfo(oinfo);
    }

    XRRFreeScreenResources(res);
    *count = index;
    return list;
}


Moss_Monitor* Moss_GetPrimaryMonitor() {
    int count;
    Moss_Monitor** mons = enumerate_monitors(&count);

    Moss_Monitor* primary = NULL;

    for (int i = 0; i < count; i++)
    {
        if (mons[i]->x == 0 && mons[i]->y == 0)
        {
            primary = mons[i];
            continue;
        }
        free(mons[i]);
    }

    free(mons);
    return primary;
}

Moss_Monitor* Moss_GetSecondaryMonitor() {
    int count;
    Moss_Monitor** mons = enumerate_monitors(&count);

    Moss_Monitor* result = NULL;

    for (int i = 0; i < count; i++)
    {
        if (!(mons[i]->x == 0 && mons[i]->y == 0))
        {
            result = mons[i];
            continue;
        }
        free(mons[i]);
    }

    free(mons);
    return result;
}


void Moss_GetMonitorPhysicalSize(Moss_Monitor* monitor, int* w, int* h) { if (w) *w = monitor->width_mm; if (h) *h = monitor->height_mm; }
void Moss_GetMonitorContentScale(Moss_Monitor* monitor, float* x, float* y) { if (x) *x = monitor->scale_x; if (y) *y = monitor->scale_y; }
void Moss_GetMonitorPosition(Moss_Monitor* monitor, int* x, int* y) { if (x) *x = monitor->x; if (y) *y = monitor->y; }
const char* Moss_GetMonitorName(Moss_Monitor* monitor) { return monitor->name; }

void Moss_SetGammaRamp(Moss_Monitor* monitor, const Moss_GammaRamp* gamma) {
    ensure_display();
    XRRSetCrtcGamma(g_display, monitor->id, (XRRCrtcGamma*)gamma);
}

Moss_GammaRamp* Moss_GetGammaRamp(Moss_Monitor* monitor) { ensure_display(); return (Moss_GammaRamp*)XRRGetCrtcGamma(g_display, monitor->id); }

void Moss_SetGamma(Moss_Monitor* monitor, float gamma) {
    ensure_display();
    int size = XRRGetCrtcGammaSize(g_display, monitor->id);
    XRRCrtcGamma* g = XRRAllocGamma(size);

    for (int i = 0; i < size; i++) {
        float v = powf((float)i / (size - 1), 1.0f / gamma) * 65535.0f;
        unsigned short value = (unsigned short)v;
        g->red[i] = g->green[i] = g->blue[i] = value;
    }

    XRRSetCrtcGamma(g_display, monitor->id, g);
    XRRFreeGamma(g);
}

void Moss_SetMonitorCallback(Moss_MonitorCallback callback) {
    ensure_display();
    g_monitorCallback = callback;

    // listen to RRScreenChangeNotify events
    XRRSelectInput(g_display, g_root, RRScreenChangeNotifyMask);
}
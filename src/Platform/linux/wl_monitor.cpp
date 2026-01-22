#include <Moss/Platform/Linux/wl_platform.h>
#include "wayland-client-protocol.h"



struct Moss_Monitor {

};

static Moss_MonitorCallback g_monitorCallback = nullptr;


Moss_Monitor* Moss_GetPrimaryMonitor() {

}

Moss_Monitor* Moss_GetSecondaryMonitor() {

}

void Moss_GetMonitorPhysicalSize(Moss_Monitor monitor, int* width_mm, int* height_mm) {

}

void Moss_GetMonitorContentScale(Moss_Monitor monitor, float* xscale, float* yscale) {

}

void Moss_GetMonitorPosition(Moss_Monitor monitor, int* x, int* y) {

}

const char* Moss_GetMonitorName(Moss_Monitor monitor) {

}

void Moss_SetGammaRamp(Moss_Monitor monitor, const Moss_GammaRamp* gammaRamp) {

}

Moss_GammaRamp* Moss_GetGammaRamp(Moss_Monitor monitor) {

}

void Moss_SetGamma(Moss_Monitor monitor, float gamma) {

}


void Moss_SetMonitorCallback(Moss_MonitorCallback callback) {

}
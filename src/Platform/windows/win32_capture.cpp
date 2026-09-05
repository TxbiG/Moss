#include <Moss/Moss_Platform.h>
#include <Moss/Moss_stdinc.h>

#include "win32_platform.h"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <dshow.h>
#include <objbase.h>
#include <strmif.h>

#pragma comment(lib, "strmiids.lib")
#pragma comment(lib, "ole32.lib")

typedef struct Moss_VideoCapture {
    IGraphBuilder* graph;
    ICaptureGraphBuilder2* captureBuilder;
    IMediaControl* mediaControl;
    IBaseFilter* videoCaptureFilter;
    IAMStreamConfig* streamConfig;
    IAMVideoProcAmp* videoProcAmp;

    unsigned char* buffer;
    long bufferSize;

    CRITICAL_SECTION lock;

    unsigned char* frameBuffer;  // Captured frame data
    long frameSize;
    bool frameReady;
} Moss_VideoCapture;


HRESULT STDMETHODCALLTYPE BufferCB(double Time, char* pBuffer, long Len) {
    Moss_VideoCapture* cap;

    EnterCriticalSection(&cap->lock);
    if (cap->frameBuffer && Len <= cap->frameSize) {
        memcpy(cap->frameBuffer, pBuffer, Len);
        cap->frameReady = true;
    }
    LeaveCriticalSection(&cap->lock);

    return S_OK;
}


Moss_CameraID* Moss_GetCameras(int* count) {}
const char* Moss_GetCameraName(Moss_CameraID camera_id) {}
Moss_CameraPosition Moss_GetCameraPosition(Moss_CameraID camera_id) {}
const char* Moss_GetCurrentCameraDriver(void) {}
int Moss_GetNumCameraDrivers(void) {}
const Moss_CameraSpec* Moss_GetCameraSupportedFormats(Moss_CameraID camera_id, int* count) {}
Moss_Surface* Moss_AcquireCameraFrame(Moss_Camera* camera, uint64_t* timestamp_ns) {}
void Moss_ReleaseCameraFrame(Moss_Camera* camera, Moss_Surface* frame) {}
bool Moss_GetCameraFormat(Moss_Camera* camera, Moss_CameraSpec* out_spec) {}
Moss_CameraPermissionState Moss_GetCameraPermissionState(Moss_Camera* camera) {}
Moss_PropertiesID Moss_GetCameraProperties(Moss_Camera* camera) {}
void Moss_CloseCamera(Moss_Camera *camera) {}
Moss_CameraID Moss_GetCameraID(Moss_Camera *camera) {}
//Moss_PropertiesID Moss_GetCameraProperties(Moss_Camera *camera) {}
Moss_Camera* Moss_OpenCamera(Moss_CameraID id, const Moss_CameraSpec *spec) {
    // 1. Initialize COM
    HRESULT hr = CoInitializeEx(NULL, COINIT_APARTMENTTHREADED); // Or COINIT_MULTITHREADED depending on Moss's architecture
    if (FAILED(hr)) return nullptr;

    // 2. Initialize your Critical Section for thread-safe buffer swapping
    Moss_Camera* camera = new Moss_Camera();
    InitializeCriticalSection(&camera->lock);

    // 3. Create the Capture Graph Builder
    hr = CoCreateInstance(CLSID_CaptureGraphBuilder2, NULL, CLSCTX_INPROC_SERVER, IID_ICaptureGraphBuilder2, (void**)&camera->captureBuilder);
    if (FAILED(hr)) {
        // Clean up and return nullptr
    }
    
    // ... continue building graph ...
    return camera;
}

Moss_VideoCapture* Moss_OpenCapture(Moss_CameraID captureID) {
    Moss_VideoCapture* cap = (Moss_VideoCapture*)calloc(1, sizeof(Moss_VideoCapture));
    if (!cap) { return NULL; }

    CoInitializeEx(NULL, COINIT_MULTITHREADED);

    // Create Filter Graph
    HRESULT hr = CoCreateInstance(CLSID_FilterGraph, NULL, CLSCTX_INPROC_SERVER, IID_IGraphBuilder, (void**)&cap->graph);
    if (FAILED(hr)) {return NULL;}

    // Capture Graph Builder
    hr = CoCreateInstance(CLSID_CaptureGraphBuilder2, NULL, CLSCTX_INPROC_SERVER, IID_ICaptureGraphBuilder2, (void**)&cap->captureBuilder);
    if (FAILED(hr)) {return NULL;}

    cap->captureBuilder->SetFiltergraph(cap->graph);

    // Get System Device Enumerator
    ICreateDevEnum* devEnum = NULL;
    IEnumMoniker* enumMoniker = NULL;
    CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER, IID_ICreateDevEnum, (void**)&devEnum);;
    devEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &enumMoniker, 0);

    IMoniker* moniker = NULL;
    if (enumMoniker->Next(1, &moniker, NULL) == S_OK) {
        moniker->BindToObject(NULL, NULL, IID_IBaseFilter, (void**)&cap->videoCaptureFilter);
        cap->graph->AddFilter(cap->videoCaptureFilter, L"Video Capture");
        cap->captureBuilder->RenderStream(&PIN_CATEGORY_PREVIEW, &MEDIATYPE_Video, cap->videoCaptureFilter, NULL, NULL);
    }

    cap->graph->QueryInterface(IID_IMediaControl, (void**)&cap->mediaControl);
    cap->videoCaptureFilter->QueryInterface(IID_IAMStreamConfig, (void**)&cap->streamConfig);
    cap->videoCaptureFilter->QueryInterface(IID_IAMVideoProcAmp, (void**)&cap->videoProcAmp);

    cap->mediaControl->Run();

    return cap;
}

void Moss_CloseCapture(Moss_VideoCapture* cap) {
    if (!cap) return;

    if (cap->mediaControl) cap->mediaControl->Stop();
    if (cap->videoProcAmp) cap->videoProcAmp->Release();
    if (cap->streamConfig) cap->streamConfig->Release();
    if (cap->videoCaptureFilter) cap->videoCaptureFilter->Release();
    if (cap->captureBuilder) cap->captureBuilder->Release();
    if (cap->graph) cap->graph->Release();

    CoUninitialize();
    free(cap);
}

unsigned char* Moss_VideoCaptureReadFrame(Moss_VideoCapture* cap)
{
    unsigned char* data = NULL;
    EnterCriticalSection(&cap->lock);
    if (cap->frameReady) {
        data = cap->frameBuffer;
        cap->frameReady = FALSE;
    }
    LeaveCriticalSection(&cap->lock);
    return data;
}


// Sets
void Moss_VideoCaptureSetBrightness(Moss_VideoCapture* cap, int brightness) {
    if (cap && cap->videoProcAmp) {
        cap->videoProcAmp->Set(VideoProcAmp_Brightness, brightness, VideoProcAmp_Flags_Manual);
    }
}

void Moss_VideoCaptureSetContrast(Moss_VideoCapture* cap, int contrast) {
    if (cap && cap->videoProcAmp) {
        cap->videoProcAmp->Set(VideoProcAmp_Contrast, contrast, VideoProcAmp_Flags_Manual);
    }
}
void Moss_VideoCaptureSetHUE(Moss_VideoCapture* cap, int hue) {
    if (cap && cap->videoProcAmp) {
        cap->videoProcAmp->Set(VideoProcAmp_Hue, hue, VideoProcAmp_Flags_Manual);
    }
}
void Moss_VideoCaptureSetSaturation(Moss_VideoCapture* cap, int saturation) {
    if (cap && cap->videoProcAmp) {
        cap->videoProcAmp->Set(VideoProcAmp_Saturation, saturation, VideoProcAmp_Flags_Manual);
    }
}

// Gets
int Moss_VideoCaptureGetBrightness(Moss_VideoCapture* cap, long value = 0, long flags = 0) {
    if (cap && cap->videoProcAmp) {
        cap->videoProcAmp->Get(VideoProcAmp_Brightness, &value, &flags);
    }
    return (int)value;
}

int Moss_VideoCaptureGetContrast(Moss_VideoCapture* cap, long value = 0, long flags = 0) {
    if (cap && cap->videoProcAmp) {
        cap->videoProcAmp->Get(VideoProcAmp_Contrast, &value, &flags);
    }
    return (int)value;
}
int Moss_VideoCaptureGetHUE(Moss_VideoCapture* cap, long value = 0, long flags = 0) {
    if (cap && cap->videoProcAmp) {
        cap->videoProcAmp->Get(VideoProcAmp_Hue, &value, &flags);
    }
    return (int)value;
}
int Moss_VideoCaptureGetSaturation(Moss_VideoCapture* cap, long value = 0, long flags = 0) {
    if (cap && cap->videoProcAmp) {
        cap->videoProcAmp->Get(VideoProcAmp_Saturation, &value, &flags);
    }
    return (int)value;
}
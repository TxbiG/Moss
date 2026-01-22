#include <Moss/Moss_Platform.h>
#include <Moss/Moss_stdinc.h>

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <dshow.h>
#include <objbase.h>
#include <strmif.h>

/*
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
    Moss_VideoCapture* cap = cb->cap;

    EnterCriticalSection(&cap->lock);
    if (cap->frameData && Len <= cap->frameSize) {
        memcpy(cap->frameData, pBuffer, Len);
        cap->frameReady = true;
    }
    LeaveCriticalSection(&cap->lock);

    return S_OK;
}


MOSS_API Moss_VideoCapture* Moss_OpenCapture() {
    Moss_VideoCapture* cap = calloc(1, sizeof(Moss_VideoCapture));
    if (!cap) {return NULL;}

    CoInitializeEx(NULL, COINIT_MULTITHREADED);

    // Create Filter Graph
    HRESULT hr = CoCreateInstance(&CLSID_FilterGraph, NULL, CLSCTX_INPROC_SERVER, &IID_IGraphBuilder, (void**)&cap->graph);
    if (FAILED(hr)) {return NULL;}

    // Capture Graph Builder
    hr = CoCreateInstance(&CLSID_CaptureGraphBuilder2, NULL, CLSCTX_INPROC_SERVER, &IID_ICaptureGraphBuilder2, (void**)&cap->captureBuilder);
    if (FAILED(hr)) {return NULL;}

    cap->captureBuilder->lpVtbl->SetFiltergraph(cap->captureBuilder, cap->graph);

    // Get System Device Enumerator
    ICreateDevEnum* devEnum = NULL;
    IEnumMoniker* enumMoniker = NULL;
    CoCreateInstance(&CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER, &IID_ICreateDevEnum, (void**)&devEnum);
    devEnum->lpVtbl->CreateClassEnumerator(devEnum, &CLSID_VideoInputDeviceCategory, &enumMoniker, 0);

    IMoniker* moniker = NULL;
    if (enumMoniker->lpVtbl->Next(enumMoniker, 1, &moniker, NULL) == S_OK) {
        moniker->lpVtbl->BindToObject(moniker, NULL, NULL, &IID_IBaseFilter, (void**)&cap->videoCaptureFilter);
        cap->graph->lpVtbl->AddFilter(cap->graph, cap->videoCaptureFilter, L"Video Capture");

        cap->captureBuilder->lpVtbl->RenderStream(
            cap->captureBuilder, &PIN_CATEGORY_PREVIEW, &MEDIATYPE_Video,
            cap->videoCaptureFilter, NULL, NULL);
    }

    cap->graph->lpVtbl->QueryInterface(cap->graph, &IID_IMediaControl, (void**)&cap->mediaControl);
    cap->videoCaptureFilter->lpVtbl->QueryInterface(cap->videoCaptureFilter, &IID_IAMStreamConfig, (void**)&cap->streamConfig);
    cap->videoCaptureFilter->lpVtbl->QueryInterface(cap->videoCaptureFilter, &IID_IAMVideoProcAmp, (void**)&cap->videoProcAmp);

    cap->mediaControl->lpVtbl->Run(cap->mediaControl);

    return cap;
}


MOSS_API unsigned char* Moss_VideoCaptureReadFrame(Moss_VideoCapture* cap)
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
MOSS_API void Moss_VideoCaptureSetBrightness(Moss_VideoCapture* cap, int brightness) {
    if (cap && cap->videoProcAmp) {
        cap->videoProcAmp->lpVtbl->Set(cap->videoProcAmp, VideoProcAmp_Brightness, brightness, VideoProcAmp_Flags_Manual);
    }
}

MOSS_API void Moss_VideoCaptureSetContrast(Moss_VideoCapture* cap, int contrast)
{
    if (cap && cap->videoProcAmp) {
        cap->videoProcAmp->lpVtbl->Set(cap->videoProcAmp, VideoProcAmp_Contrast, contrast, VideoProcAmp_Flags_Manual);
    }
}
MOSS_API void Moss_VideoCaptureSetHUE(Moss_VideoCapture* cap, int hue) {
    if (cap && cap->videoProcAmp) {
        cap->videoProcAmp->lpVtbl->Set(cap->videoProcAmp, VideoProcAmp_Hue, hue, VideoProcAmp_Flags_Manual);
    }
}
MOSS_API void Moss_VideoCaptureSetSaturation(Moss_VideoCapture* cap, int saturation) {
    if (cap && cap->videoProcAmp) {
        cap->videoProcAmp->lpVtbl->Set(cap->videoProcAmp, VideoProcAmp_Saturation, saturation, VideoProcAmp_Flags_Manual);
    }
}


// Gets

MOSS_API int Moss_VideoCaptureGetBrightness(Moss_VideoCapture* cap) {
    long value = 0, flags = 0;
    if (cap && cap->videoProcAmp) {
        cap->videoProcAmp->lpVtbl->Get(cap->videoProcAmp, VideoProcAmp_Brightness, &value, &flags);
    }
    return (int)value;
}

MOSS_API int Moss_VideoCaptureGetContrast(Moss_VideoCapture* cap)
{
    long value = 0, flags = 0;
    if (cap && cap->videoProcAmp) {
        cap->videoProcAmp->lpVtbl->Get(cap->videoProcAmp, VideoProcAmp_Contrast, &value, &flags);
    }
    return (int)value;
}
MOSS_API int Moss_VideoCaptureGetHUE(Moss_VideoCapture* cap)
{
    long value = 0, flags = 0;
    if (cap && cap->videoProcAmp) {
        cap->videoProcAmp->lpVtbl->Get(cap->videoProcAmp, VideoProcAmp_Hue, &value, &flags);
    }
    return (int)value;
}
MOSS_API int Moss_VideoCaptureGetSaturation(Moss_VideoCapture* cap)
{
    long value = 0, flags = 0;
    if (cap && cap->videoProcAmp) {
        cap->videoProcAmp->lpVtbl->Get(cap->videoProcAmp, VideoProcAmp_Saturation, &value, &flags);
    }
    return (int)value;
}

MOSS_API void Moss_CloseCapture(Moss_VideoCapture* cap) {
    if (!cap) return;

    if (cap->mediaControl) cap->mediaControl->lpVtbl->Stop(cap->mediaControl);
    if (cap->videoProcAmp) cap->videoProcAmp->lpVtbl->Release(cap->videoProcAmp);
    if (cap->streamConfig) cap->streamConfig->lpVtbl->Release(cap->streamConfig);
    if (cap->videoCaptureFilter) cap->videoCaptureFilter->lpVtbl->Release(cap->videoCaptureFilter);
    if (cap->captureBuilder) cap->captureBuilder->lpVtbl->Release(cap->captureBuilder);
    if (cap->graph) cap->graph->lpVtbl->Release(cap->graph);

    CoUninitialize();
    free(cap);
}*/




Moss_CameraID* Moss_GetCameras(int* count);
const char* Moss_GetCameraName(Moss_CameraID camera_id);
Moss_CameraPosition Moss_GetCameraPosition(Moss_CameraID camera_id);
const char* Moss_GetCurrentCameraDriver(void);
int Moss_GetNumCameraDrivers(void);
const Moss_CameraSpec* Moss_GetCameraSupportedFormats(Moss_CameraID camera_id, int* count);
Moss_Surface* Moss_AcquireCameraFrame(Moss_Camera* camera, uint64_t* timestamp_ns);
void Moss_ReleaseCameraFrame(Moss_Camera* camera, Moss_Surface* frame);
bool Moss_GetCameraFormat(Moss_Camera* camera, Moss_CameraSpec* out_spec);
Moss_CameraPermissionState Moss_GetCameraPermissionState(Moss_Camera* camera);
Moss_PropertiesID Moss_GetCameraProperties(Moss_Camera* camera);
void Moss_CloseCamera(Moss_Camera *camera);
Moss_CameraID Moss_GetCameraID(Moss_Camera *camera);
Moss_PropertiesID Moss_GetCameraProperties(Moss_Camera *camera);
Moss_Camera * Moss_OpenCamera(Moss_CameraID instance_id, const Moss_CameraSpec *spec);
Moss_VideoCapture* Moss_OpenVideoCapture(Moss_VideoCaptureID captureID);

void Moss_CloseVideoCapture(Moss_VideoCapture* cap);

uint8_t* Moss_VideoCaptureReadFrame(Moss_VideoCapture* cap);
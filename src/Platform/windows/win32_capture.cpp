#include <Moss/Moss_Platform.h>
#include <Moss/Moss_stdinc.h>

#include "win32_platform.h"

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <dshow.h>
#include <strmif.h>
#include <uuids.h>
#include <strmif.h>

#pragma comment(lib, "strmiids.lib")
#pragma comment(lib, "ole32.lib")

struct Moss_Capture {
    IGraphBuilder* graph;
    ICaptureGraphBuilder2* captureBuilder = nullptr;
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
};


/*
struct Moss_Capture {
    // A mutex for locking
    Mutex *lock;

    // Human-readable device name.
    char *name;

    // Position of camera (front-facing, back-facing, etc).
    Moss_CameraPosition position;

    // When refcount hits zero, we destroy the device object.
    Moss_AtomicInt refcount;

    // These are, initially, set from camera_driver, but we might swap them out with Zombie versions on disconnect/failure.
    bool (*WaitDevice)(Moss_Capture *device);
    AcquireFrameFunc AcquireFrame;
    void (*ReleaseFrame)(Moss_Capture *device, Moss_Surface *frame);

    // All supported formats/dimensions for this device.
    Moss_CameraSpec *all_specs;

    // Elements in all_specs.
    int num_specs;

    // The device's actual specification that the camera is outputting, before conversion.
    Moss_CameraSpec actual_spec;

    // The device's current camera specification, after conversions.
    Moss_CameraSpec spec;

    // Unique value assigned at creation time.
    Moss_CameraID instance_id;

    // Driver-specific hardware data on how to open device (`hidden` is driver-specific data _when opened_).
    void *handle;

    // Dropping the first frame(s) after open seems to help timing on some platforms.
    int drop_frames;

    // Backend timestamp of first acquired frame, so we can keep these meaningful regardless of epoch.
    uint64_t base_timestamp;

    // Moss timestamp of first acquired frame, so we can roughly convert to Moss ticks.
    uint64_t adjust_timestamp;

    // Pixel data flows from the driver into these, then gets converted for the app if necessary.
    Moss_Surface *acquire_surface;

    // acquire_surface converts or scales to this surface before landing in output_surfaces, if necessary.
    Moss_Surface *conversion_surface;

    // A queue of surfaces that buffer converted/scaled frames of video until the app claims them.
    SurfaceList output_surfaces[8];
    SurfaceList filled_output_surfaces;        // this is FIFO
    SurfaceList empty_output_surfaces;         // this is LIFO
    SurfaceList app_held_output_surfaces;

    // A fake video frame we allocate if the camera fails/disconnects.
    uint8_t *zombie_pixels;

    // non-zero if acquire_surface needs to be scaled for final output.
    int needs_scaling;  // -1: downscale, 0: no scaling, 1: upscale

    // true if acquire_surface needs to be converted for final output.
    bool needs_conversion;

    // Current state flags
    Moss_AtomicInt shutdown;
    Moss_AtomicInt zombie;

    // A thread to feed the camera device
    Moss_Thread *thread;

    // Optional properties.
    Moss_PropertiesID props;

    // Current state of user permission check.
    Moss_CameraPermissionState permission;

    // Data private to this driver, used when device is opened and running.
    struct Moss_PrivateCameraData *hidden;
};
*/

HRESULT STDMETHODCALLTYPE BufferCB(double Time, char* pBuffer, long Len) {
    Moss_Capture* cap;

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
Moss_Surface* Moss_AcquireCameraFrame(Moss_Capture* camera, uint64_t* timestamp_ns) {}
void Moss_ReleaseCameraFrame(Moss_Capture* camera, Moss_Surface* frame) {}
bool Moss_GetCameraFormat(Moss_Capture* camera, Moss_CameraSpec* out_spec) {}
Moss_CameraPermissionState Moss_GetCameraPermissionState(Moss_Capture* camera) {}
Moss_PropertiesID Moss_GetCameraProperties(Moss_Capture* camera) {}
void Moss_CloseCamera(Moss_Capture *camera) {}
Moss_CameraID Moss_GetCameraID(Moss_Capture *camera) {}
//Moss_PropertiesID Moss_GetCameraProperties(Moss_Capture *camera) {}

Moss_Capture* Moss_OpenCapture(Moss_CameraID captureID, const Moss_CameraSpec* spec) {
    (void)captureID;
    (void)spec;

    Moss_Capture* cap = static_cast<Moss_Capture*>(calloc(1, sizeof(Moss_Capture)));
    if (!cap) { return NULL; }

    HRESULT hr = CoInitializeEx(NULL, COINIT_MULTITHREADED);

    if (FAILED(hr) && hr != RPC_E_CHANGED_MODE) { free(cap); return NULL; }

    // Create Filter Graph.
    hr = CoCreateInstance(CLSID_FilterGraph, NULL, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&cap->graph));

    if (FAILED(hr) || !cap->graph) { free(cap); return NULL; }

    // Create Capture Graph Builder.
    hr = CoCreateInstance(CLSID_CaptureGraphBuilder2, NULL, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&cap->captureBuilder));

    if (FAILED(hr) || !cap->captureBuilder) {
        cap->graph->Release();
        free(cap);
        return NULL;
    }

    // Connect the Capture Graph Builder to the Filter Graph.
    hr = cap->captureBuilder->SetFiltergraph(cap->graph);

    if (FAILED(hr)) {
        cap->captureBuilder->Release();
        cap->graph->Release();
        free(cap);
        return NULL;
    }

    // Create System Device Enumerator.
    ICreateDevEnum* devEnum = NULL;

    hr = CoCreateInstance(CLSID_SystemDeviceEnum, NULL, CLSCTX_INPROC_SERVER, IID_PPV_ARGS(&devEnum));

    if (FAILED(hr) || !devEnum) {
        cap->captureBuilder->Release();
        cap->graph->Release();
        free(cap);
        return NULL;
    }

    // Enumerate video capture devices.
    IEnumMoniker* enumMoniker = NULL;

    hr = devEnum->CreateClassEnumerator(CLSID_VideoInputDeviceCategory, &enumMoniker, 0);

    if (FAILED(hr) || !enumMoniker) {
        devEnum->Release();
        cap->captureBuilder->Release();
        cap->graph->Release();
        free(cap);
        return NULL;
    }

    // Get the first camera.
    IMoniker* moniker = NULL;

    hr = enumMoniker->Next(1, &moniker, NULL);

    if (hr != S_OK || !moniker) {
        enumMoniker->Release();
        devEnum->Release();
        cap->captureBuilder->Release();
        cap->graph->Release();
        free(cap);
        return NULL;
    }

    // Convert the moniker into a capture filter.
    hr = moniker->BindToObject(NULL, NULL, IID_PPV_ARGS(&cap->videoCaptureFilter));

    moniker->Release();
    enumMoniker->Release();
    devEnum->Release();

    if (FAILED(hr) || !cap->videoCaptureFilter) {
        cap->captureBuilder->Release();
        cap->graph->Release();
        free(cap);
        return NULL;
    }

    // Add the camera to the graph.
    hr = cap->graph->AddFilter(cap->videoCaptureFilter, L"Video Capture");

    if (FAILED(hr)) {
        cap->videoCaptureFilter->Release();
        cap->captureBuilder->Release();
        cap->graph->Release();
        free(cap);
        return NULL;
    }

    // Render the camera preview stream.
    hr = cap->captureBuilder->RenderStream(&PIN_CATEGORY_PREVIEW, &MEDIATYPE_Video, cap->videoCaptureFilter, NULL, NULL);

    if (FAILED(hr)) {
        cap->videoCaptureFilter->Release();
        cap->captureBuilder->Release();
        cap->graph->Release();
        free(cap);
        return NULL;
    }

    // Get media control.
    hr = cap->graph->QueryInterface(IID_PPV_ARGS(&cap->mediaControl));

    if (FAILED(hr) || !cap->mediaControl) {
        cap->videoCaptureFilter->Release();
        cap->captureBuilder->Release();
        cap->graph->Release();
        free(cap);
        return NULL;
    }

    // Get stream configuration.
    cap->videoCaptureFilter->QueryInterface(IID_PPV_ARGS(&cap->streamConfig));

    // Get video processing controls.
    cap->videoCaptureFilter->QueryInterface(IID_PPV_ARGS(&cap->videoProcAmp));

    // Start capture.
    hr = cap->mediaControl->Run();

    if (FAILED(hr)) {
        if (cap->videoProcAmp) { cap->videoProcAmp->Release(); }

        if (cap->streamConfig) { cap->streamConfig->Release(); }

        cap->mediaControl->Release();
        cap->videoCaptureFilter->Release();
        cap->captureBuilder->Release();
        cap->graph->Release();
        free(cap);
        return NULL;
    }

    return cap;
}

void Moss_CloseCapture(Moss_Capture* cap) {
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

unsigned char* Moss_CaptureReadFrame(Moss_Capture* cap)
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
void Moss_CaptureSetBrightness(Moss_Capture* cap, int brightness) {
    if (cap && cap->videoProcAmp) {
        cap->videoProcAmp->Set(VideoProcAmp_Brightness, brightness, VideoProcAmp_Flags_Manual);
    }
}

void Moss_CaptureSetContrast(Moss_Capture* cap, int contrast) {
    if (cap && cap->videoProcAmp) {
        cap->videoProcAmp->Set(VideoProcAmp_Contrast, contrast, VideoProcAmp_Flags_Manual);
    }
}
void Moss_CaptureSetHUE(Moss_Capture* cap, int hue) {
    if (cap && cap->videoProcAmp) {
        cap->videoProcAmp->Set(VideoProcAmp_Hue, hue, VideoProcAmp_Flags_Manual);
    }
}
void Moss_CaptureSetSaturation(Moss_Capture* cap, int saturation) {
    if (cap && cap->videoProcAmp) {
        cap->videoProcAmp->Set(VideoProcAmp_Saturation, saturation, VideoProcAmp_Flags_Manual);
    }
}

// Gets
int Moss_CaptureGetBrightness(Moss_Capture* cap, long value = 0, long flags = 0) {
    if (cap && cap->videoProcAmp) {
        cap->videoProcAmp->Get(VideoProcAmp_Brightness, &value, &flags);
    }
    return (int)value;
}

int Moss_CaptureGetContrast(Moss_Capture* cap, long value = 0, long flags = 0) {
    if (cap && cap->videoProcAmp) {
        cap->videoProcAmp->Get(VideoProcAmp_Contrast, &value, &flags);
    }
    return (int)value;
}
int Moss_CaptureGetHUE(Moss_Capture* cap, long value = 0, long flags = 0) {
    if (cap && cap->videoProcAmp) {
        cap->videoProcAmp->Get(VideoProcAmp_Hue, &value, &flags);
    }
    return (int)value;
}
int Moss_CaptureGetSaturation(Moss_Capture* cap, long value = 0, long flags = 0) {
    if (cap && cap->videoProcAmp) {
        cap->videoProcAmp->Get(VideoProcAmp_Saturation, &value, &flags);
    }
    return (int)value;
}
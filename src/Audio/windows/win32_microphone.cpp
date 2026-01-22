// win32_microphone.cpp
#include <Moss/Audio/xaudio/win32_audio.h>

#include <mmdeviceapi.h>
#include <audioclient.h>
#include <functiondiscoverykeys_devpkey.h>
#include <comdef.h>
#include <thread>
#include <atomic>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>

// -----------------------------
// Microphone helper functions
// -----------------------------

Moss_Microphone mic;

bool Moss_IsMicrophoneDeviceReady() {
    return mic.inputDevice != nullptr && mic.audioClient != nullptr;
}

bool Moss_AudioEnumerateMicrophones() {
    mic.microphoneIds.clear();
    mic.microphoneNames.clear();

    IMMDeviceEnumerator* enumerator = nullptr;
    IMMDeviceCollection* devices = nullptr;

    if (FAILED(CoCreateInstance(
        __uuidof(MMDeviceEnumerator),
        nullptr,
        CLSCTX_ALL,
        __uuidof(IMMDeviceEnumerator),
        (void**)&enumerator)))
        return false;

    if (FAILED(enumerator->EnumAudioEndpoints(eCapture, DEVICE_STATE_ACTIVE, &devices))) {
        enumerator->Release();
        return false;
    }

    UINT count = 0;
    devices->GetCount(&count);

    for (UINT i = 0; i < count; ++i) {
        IMMDevice* device = nullptr;
        if (FAILED(devices->Item(i, &device)))
            continue;

        LPWSTR id = nullptr;
        device->GetId(&id);

        IPropertyStore* props = nullptr;
        device->OpenPropertyStore(STGM_READ, &props);

        PROPVARIANT name;
        PropVariantInit(&name);
        props->GetValue(PKEY_Device_FriendlyName, &name);

        mic.microphoneIds.push_back(id);
        mic.microphoneNames.emplace_back(_bstr_t(name.pwszVal));

        PropVariantClear(&name);
        props->Release();
        device->Release();
        CoTaskMemFree(id);
    }

    devices->Release();
    enumerator->Release();
    return true;
}



// Open default microphone and initialize capture
bool Moss_AudioMicrophoneOpen() {
    if (Moss_IsMicrophoneDeviceReady())
        return true;

    HRESULT hr;
    IMMDeviceEnumerator* enumerator = nullptr;

    hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr, CLSCTX_ALL, __uuidof(IMMDeviceEnumerator), (void**)&enumerator);
    if (FAILED(hr)) return false;

    hr = enumerator->GetDefaultAudioEndpoint(eCapture, eConsole, &mic.inputDevice);
    enumerator->Release();
    if (FAILED(hr)) return false;

    hr = mic.inputDevice->Activate(__uuidof(IAudioClient), CLSCTX_ALL, nullptr, (void**)&mic.audioClient);
    if (FAILED(hr)) return false;

    hr = mic.audioClient->GetMixFormat(&mic.format);
    if (FAILED(hr)) return false;

    // 1-second buffer in shared mode
    hr = mic.audioClient->Initialize(
        AUDCLNT_SHAREMODE_SHARED,
        0,
        10000000,
        0,
        mic.format,
        nullptr
    );
    if (FAILED(hr)) return false;

    hr = mic.audioClient->GetService(__uuidof(IAudioCaptureClient), (void**)&mic.captureClient);
    if (FAILED(hr)) return false;

    return true;
}

void Moss_AudioMicrophoneClose() {
    if (mic.captureClient) { mic.captureClient->Release(); }
    if (mic.audioClient) { mic.audioClient->Stop(); mic.audioClient->Release(); }
    if (mic.inputDevice) { mic.inputDevice->Release(); }
    if (mic.format) { CoTaskMemFree(mic.format); }
}

// Internal capture processing
static void Moss_CaptureMicrophone() {
    UINT32 packetLength = 0;
    mic.captureClient->GetNextPacketSize(&packetLength);

    while (packetLength) {
        BYTE* data = nullptr;
        UINT32 frames = 0;
        DWORD flags = 0;

        if (FAILED(mic.captureClient->GetBuffer(&data, &frames, &flags, nullptr, nullptr)))
            break;

        if (!(flags & AUDCLNT_BUFFERFLAGS_SILENT) && mic.micCallback) {
            const uint32_t channels = mic.format->nChannels;
            const uint32_t sampleCount = frames * channels;

            std::vector<float> buffer(sampleCount);

            if (mic.format->wFormatTag == WAVE_FORMAT_IEEE_FLOAT) {
                float* in = (float*)data;
                for (uint32_t i = 0; i < sampleCount; ++i)
                    buffer[i] = in[i] * mic.micGain;
            }
            else {
                int16_t* in = (int16_t*)data;
                for (uint32_t i = 0; i < sampleCount; ++i)
                    buffer[i] = (in[i] / 32768.0f) * mic.micGain;
            }

            mic.micCallback(buffer.data(), frames, mic.micUserData);
        }

        mic.captureClient->ReleaseBuffer(frames);
        mic.captureClient->GetNextPacketSize(&packetLength);
    }
}

// Start capturing microphone
void Moss_AudioMicrophonePlay() {
    if (!Moss_IsMicrophoneDeviceReady() || mic.capturing)
        return;

    mic.capturing = true;
    mic.audioClient->Start();

    mic.captureThread = std::thread([] {
        while (mic.capturing) {
            Moss_CaptureMicrophone();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    });
}

// Stop capturing microphone

void Moss_AudioMicrophoneStop() {
    if (!mic.capturing)
        return;

    mic.capturing = false;
    if (mic.captureThread.joinable())
        mic.captureThread.join();

    mic.audioClient->Stop();
}

// Default microphone ID
int Moss_AudioMicrophoneID() {
    return Moss_IsMicrophoneDeviceReady() ? 0 : -1;
}

// Placeholder for type (e.g., USB, built-in)
void Moss_AudioMicrophoneType() {}

// -----------------------------
// Microphone enumeration
// -----------------------------
int Moss_ListMicrophoneDevices() {
    return static_cast<int>(mic.microphoneNames.size());
}

const char* Moss_GetMicrophoneDeviceName(int index) {
    if (index < 0 || index >= static_cast<int>(mic.microphoneNames.size()))
        return nullptr;
    return mic.microphoneNames[index].c_str();
}



bool Moss_AudioSelectMicrophoneDevice(int id) {
    if (id < 0 || id >= (int)mic.microphoneIds.size())
        return false;

    Moss_AudioMicrophoneClose();

    IMMDeviceEnumerator* enumerator = nullptr;
    if (FAILED(CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr, CLSCTX_ALL, __uuidof(IMMDeviceEnumerator), (void**)&enumerator))) 
        return false;
    
    if (FAILED(enumerator->GetDevice(
        mic.microphoneIds[id].c_str(),
        &mic.inputDevice))) {
        enumerator->Release();
        return false;
    }

    enumerator->Release();
    return Moss_AudioMicrophoneOpen();
}


void Moss_AudioMicrophoneSetGain(Microphone*, float gain) { mic.micGain = std::max(0.0f, gain); }

int Moss_AudioMicrophoneGetSampleRate(Microphone*) { return mic.format ? mic.format->nSamplesPerSec : 0; }

int Moss_AudioMicrophoneGetChannels(Microphone*) { return mic.format ? mic.format->nChannels : 0; }



void Moss_AudioMicrophoneSetCallback(Microphone*, MicrophoneCallback callback, void* userData) {
    mic.micCallback = callback;
    mic.micUserData = userData;
}

// win32_microphone.cpp
#include "win32_audio.h"

#include <mmdeviceapi.h>
#include <audioclient.h>
#include <functiondiscoverykeys_devpkey.h>
#include <comdef.h>
#include <ksmedia.h>
#include <chrono>
#include <cmath>
#include <algorithm>

// -----------------------------
// Microphone helper functions
// -----------------------------

Moss_Microphone mic;
static Moss_Microphone* g_legacyMicrophone = nullptr;

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





/*

static bool Moss_Win32EnsureCOM() {
    const HRESULT hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    return SUCCEEDED(hr) || hr == RPC_E_CHANGED_MODE;
}

static bool Moss_IsFloatFormat(const WAVEFORMATEX* format) {
    if (!format)
        return false;

    if (format->wFormatTag == WAVE_FORMAT_IEEE_FLOAT)
        return true;

    if (format->wFormatTag == WAVE_FORMAT_EXTENSIBLE) {
        const auto* extensible = reinterpret_cast<const WAVEFORMATEXTENSIBLE*>(format);
        return IsEqualGUID(extensible->SubFormat, KSDATAFORMAT_SUBTYPE_IEEE_FLOAT);
    }

    return false;
}

static float Moss_ReadSampleAsFloat(const BYTE* data, const WAVEFORMATEX* format, uint32_t sampleIndex) {
    if (!data || !format)
        return 0.0f;

    if (Moss_IsFloatFormat(format) && format->wBitsPerSample == 32)
        return reinterpret_cast<const float*>(data)[sampleIndex];

    if (format->wBitsPerSample == 16)
        return reinterpret_cast<const int16_t*>(data)[sampleIndex] / 32768.0f;

    if (format->wBitsPerSample == 32)
        return reinterpret_cast<const int32_t*>(data)[sampleIndex] / 2147483648.0f;

    return 0.0f;
}

static void Moss_ResetMicrophoneBuffer(Moss_Microphone* micHandle, uint32_t ringFrames) {
    if (!micHandle)
        return;

    std::lock_guard<std::mutex> lock(micHandle->bufferMutex);
    micHandle->ringFrameCapacity = std::max<uint32_t>(ringFrames, 1);
    micHandle->ringBuffer.assign(micHandle->ringFrameCapacity * micHandle->channels, 0.0f);
    micHandle->readFrame = 0;
    micHandle->writeFrame = 0;
    micHandle->availableFrames = 0;
}

static void Moss_UpdateMicrophoneLevels(Moss_Microphone* micHandle, const float* samples, uint32_t sampleCount) {
    if (!micHandle || !micHandle->voiceMetricsEnabled || !samples || sampleCount == 0)
        return;

    float peak = 0.0f;
    double sumSquares = 0.0;

    for (uint32_t i = 0; i < sampleCount; ++i) {
        const float value = std::clamp(samples[i], -1.0f, 1.0f);
        const float absValue = std::fabs(value);
        peak = std::max(peak, absValue);
        sumSquares += static_cast<double>(value) * static_cast<double>(value);
    }

    const float rms = std::sqrt(static_cast<float>(sumSquares / sampleCount));
    micHandle->levels.rms = rms;
    micHandle->levels.peak = peak;
    micHandle->levels.smoothed_volume = micHandle->levels.smoothed_volume * 0.85f + rms * 0.15f;
    micHandle->levels.voice_activity = std::clamp((micHandle->levels.smoothed_volume - 0.015f) * 16.0f, 0.0f, 1.0f);
}

static void Moss_PushMicrophoneFrames(Moss_Microphone* micHandle, const float* frames, uint32_t frameCount) {
    if (!micHandle || !frames || frameCount == 0 || micHandle->ringFrameCapacity == 0 || micHandle->channels == 0)
        return;

    std::lock_guard<std::mutex> lock(micHandle->bufferMutex);

    for (uint32_t frame = 0; frame < frameCount; ++frame) {
        if (micHandle->availableFrames == micHandle->ringFrameCapacity) {
            micHandle->readFrame = (micHandle->readFrame + 1) % micHandle->ringFrameCapacity;
            micHandle->availableFrames--;
        }

        const uint32_t writeOffset = micHandle->writeFrame * micHandle->channels;
        const uint32_t sourceOffset = frame * micHandle->channels;
        for (uint32_t channel = 0; channel < micHandle->channels; ++channel)
            micHandle->ringBuffer[writeOffset + channel] = frames[sourceOffset + channel];

        micHandle->writeFrame = (micHandle->writeFrame + 1) % micHandle->ringFrameCapacity;
        micHandle->availableFrames++;
    }
}

static void Moss_ReleaseMicrophoneDevice(Moss_Microphone* micHandle) {
    if (!micHandle)
        return;

    if (micHandle->captureClient) {
        micHandle->captureClient->Release();
        micHandle->captureClient = nullptr;
    }

    if (micHandle->audioClient) {
        micHandle->audioClient->Stop();
        micHandle->audioClient->Release();
        micHandle->audioClient = nullptr;
    }

    if (micHandle->inputDevice) {
        micHandle->inputDevice->Release();
        micHandle->inputDevice = nullptr;
    }

    if (micHandle->format) {
        CoTaskMemFree(micHandle->format);
        micHandle->format = nullptr;
    }
}

static bool Moss_EnumerateMicrophonesInto(Moss_Microphone* micHandle) {
    if (!micHandle || !Moss_Win32EnsureCOM())
        return false;

    micHandle->microphoneIds.clear();
    micHandle->microphoneNames.clear();

    IMMDeviceEnumerator* enumerator = nullptr;
    IMMDeviceCollection* devices = nullptr;

    if (FAILED(CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr, CLSCTX_ALL, __uuidof(IMMDeviceEnumerator), (void**)&enumerator)))
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
        if (FAILED(device->GetId(&id))) {
            device->Release();
            continue;
        }

        IPropertyStore* props = nullptr;
        if (SUCCEEDED(device->OpenPropertyStore(STGM_READ, &props))) {
            PROPVARIANT name;
            PropVariantInit(&name);

            if (SUCCEEDED(props->GetValue(PKEY_Device_FriendlyName, &name)) && name.pwszVal)
                micHandle->microphoneNames.emplace_back(_bstr_t(name.pwszVal));
            else
                micHandle->microphoneNames.emplace_back("Microphone");

            PropVariantClear(&name);
            props->Release();
        }
        else {
            micHandle->microphoneNames.emplace_back("Microphone");
        }

        micHandle->microphoneIds.emplace_back(id);
        CoTaskMemFree(id);
        device->Release();
    }

    devices->Release();
    enumerator->Release();
    return true;
}

bool Moss_IsMicrophoneDeviceReady() {
    Moss_Microphone* legacy = g_legacyMicrophone ? g_legacyMicrophone : &mic;
    return legacy->inputDevice != nullptr && legacy->audioClient != nullptr && legacy->captureClient != nullptr;
}

bool Moss_AudioEnumerateMicrophones() {
    return Moss_EnumerateMicrophonesInto(&mic);
}

uint32_t Moss_MicrophoneGetDeviceCount() {
    Moss_AudioEnumerateMicrophones();
    return static_cast<uint32_t>(mic.microphoneNames.size());
}

const char* Moss_MicrophoneGetDeviceName(uint32_t device_index) {
    Moss_AudioEnumerateMicrophones();
    if (device_index >= mic.microphoneNames.size())
        return nullptr;
    return mic.microphoneNames[device_index].c_str();
}

Moss_Microphone* Moss_MicrophoneOpen(const Moss_MicrophoneDesc* desc) {
    Moss_MicrophoneDesc defaultDesc{};
    if (!desc)
        desc = &defaultDesc;

    if (!Moss_Win32EnsureCOM())
        return nullptr;

    Moss_Microphone* micHandle = new Moss_Microphone();
    micHandle->sampleRate = desc->sample_rate ? desc->sample_rate : 48000;
    micHandle->channels = desc->channels ? desc->channels : 1;
    micHandle->bufferFrames = desc->buffer_frames ? desc->buffer_frames : 480;
    micHandle->voiceMetricsEnabled = desc->enable_voice_metrics;

    if (!Moss_EnumerateMicrophonesInto(micHandle)) {
        delete micHandle;
        return nullptr;
    }

    HRESULT hr;
    IMMDeviceEnumerator* enumerator = nullptr;
    hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr, CLSCTX_ALL, __uuidof(IMMDeviceEnumerator), (void**)&enumerator);
    if (FAILED(hr)) {
        delete micHandle;
        return nullptr;
    }

    if (desc->device_index < micHandle->microphoneIds.size()) {
        hr = enumerator->GetDevice(micHandle->microphoneIds[desc->device_index].c_str(), &micHandle->inputDevice);
    }
    else {
        hr = enumerator->GetDefaultAudioEndpoint(eCapture, eConsole, &micHandle->inputDevice);
    }

    enumerator->Release();
    if (FAILED(hr)) {
        delete micHandle;
        return nullptr;
    }

    hr = micHandle->inputDevice->Activate(__uuidof(IAudioClient), CLSCTX_ALL, nullptr, (void**)&micHandle->audioClient);
    if (FAILED(hr)) {
        Moss_ReleaseMicrophoneDevice(micHandle);
        delete micHandle;
        return nullptr;
    }

    hr = micHandle->audioClient->GetMixFormat(&micHandle->format);
    if (FAILED(hr)) {
        Moss_ReleaseMicrophoneDevice(micHandle);
        delete micHandle;
        return nullptr;
    }

    micHandle->sampleRate = micHandle->format->nSamplesPerSec;
    const uint32_t sourceChannels = std::max<uint16_t>(micHandle->format->nChannels, 1);
    micHandle->channels = desc->channels ? std::max<uint32_t>(desc->channels, 1) : sourceChannels;

    const REFERENCE_TIME bufferDuration = static_cast<REFERENCE_TIME>(
        (static_cast<double>(std::max<uint32_t>(micHandle->bufferFrames, 1)) / micHandle->sampleRate) * 10000000.0);

    hr = micHandle->audioClient->Initialize(AUDCLNT_SHAREMODE_SHARED, 0, bufferDuration, 0, micHandle->format, nullptr);
    if (FAILED(hr)) {
        Moss_ReleaseMicrophoneDevice(micHandle);
        delete micHandle;
        return nullptr;
    }

    hr = micHandle->audioClient->GetService(__uuidof(IAudioCaptureClient), (void**)&micHandle->captureClient);
    if (FAILED(hr)) {
        Moss_ReleaseMicrophoneDevice(micHandle);
        delete micHandle;
        return nullptr;
    }

    Moss_ResetMicrophoneBuffer(micHandle, desc->ring_buffer_frames ? desc->ring_buffer_frames : micHandle->sampleRate);

    if (desc->start_immediately && !Moss_MicrophoneStart(micHandle)) {
        Moss_MicrophoneClose(micHandle);
        return nullptr;
    }

    return micHandle;
}

void Moss_MicrophoneClose(Moss_Microphone* micHandle) {
    if (!micHandle)
        return;

    Moss_MicrophoneStop(micHandle);
    Moss_ReleaseMicrophoneDevice(micHandle);

    if (micHandle != &mic)
        delete micHandle;
}

static void Moss_CaptureMicrophone(Moss_Microphone* micHandle) {
    if (!micHandle || !micHandle->captureClient || !micHandle->format)
        return;

    UINT32 packetLength = 0;
    micHandle->captureClient->GetNextPacketSize(&packetLength);

    while (packetLength) {
        BYTE* data = nullptr;
        UINT32 frames = 0;
        DWORD flags = 0;

        if (FAILED(micHandle->captureClient->GetBuffer(&data, &frames, &flags, nullptr, nullptr)))
            break;

        const uint32_t sourceChannels = std::max<uint16_t>(micHandle->format->nChannels, 1);
        const uint32_t outputChannels = std::max<uint32_t>(micHandle->channels, 1);
        std::vector<float> buffer(frames * outputChannels, 0.0f);

        if (!(flags & AUDCLNT_BUFFERFLAGS_SILENT)) {
            for (uint32_t frame = 0; frame < frames; ++frame) {
                for (uint32_t outChannel = 0; outChannel < outputChannels; ++outChannel) {
                    float sample = 0.0f;

                    if (outputChannels == 1 && sourceChannels > 1) {
                        for (uint32_t sourceChannel = 0; sourceChannel < sourceChannels; ++sourceChannel)
                            sample += Moss_ReadSampleAsFloat(data, micHandle->format, frame * sourceChannels + sourceChannel);
                        sample /= static_cast<float>(sourceChannels);
                    }
                    else {
                        const uint32_t sourceChannel = std::min(outChannel, sourceChannels - 1);
                        sample = Moss_ReadSampleAsFloat(data, micHandle->format, frame * sourceChannels + sourceChannel);
                    }

                    buffer[frame * outputChannels + outChannel] = std::clamp(sample * micHandle->micGain, -1.0f, 1.0f);
                }
            }
        }

        Moss_UpdateMicrophoneLevels(micHandle, buffer.data(), static_cast<uint32_t>(buffer.size()));
        Moss_PushMicrophoneFrames(micHandle, buffer.data(), frames);

        if (micHandle->micCallback)
            micHandle->micCallback(buffer.data(), static_cast<int>(frames), micHandle->micUserData);

        micHandle->captureClient->ReleaseBuffer(frames);
        micHandle->captureClient->GetNextPacketSize(&packetLength);
    }
}

bool Moss_MicrophoneStart(Moss_Microphone* micHandle) {
    if (!micHandle || !micHandle->audioClient || !micHandle->captureClient || micHandle->capturing)
        return false;

    if (FAILED(micHandle->audioClient->Start()))
        return false;

    micHandle->capturing = true;
    micHandle->captureThread = std::thread([micHandle] {
        while (micHandle->capturing) {
            Moss_CaptureMicrophone(micHandle);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    });

    return true;
}

void Moss_MicrophoneStop(Moss_Microphone* micHandle) {
    if (!micHandle || !micHandle->capturing)
        return;

    micHandle->capturing = false;
    if (micHandle->captureThread.joinable())
        micHandle->captureThread.join();

    if (micHandle->audioClient)
        micHandle->audioClient->Stop();
}

uint32_t Moss_MicrophoneRead(Moss_Microphone* micHandle, float* out_samples, uint32_t max_frames) {
    if (!micHandle || !out_samples || max_frames == 0 || micHandle->channels == 0)
        return 0;

    std::lock_guard<std::mutex> lock(micHandle->bufferMutex);
    const uint32_t framesToRead = std::min(max_frames, micHandle->availableFrames);

    for (uint32_t frame = 0; frame < framesToRead; ++frame) {
        const uint32_t readOffset = micHandle->readFrame * micHandle->channels;
        const uint32_t outputOffset = frame * micHandle->channels;

        for (uint32_t channel = 0; channel < micHandle->channels; ++channel)
            out_samples[outputOffset + channel] = micHandle->ringBuffer[readOffset + channel];

        micHandle->readFrame = (micHandle->readFrame + 1) % micHandle->ringFrameCapacity;
    }

    micHandle->availableFrames -= framesToRead;
    return framesToRead;
}

void Moss_MicrophoneSetGain(Moss_Microphone* micHandle, float gain) {
    if (!micHandle)
        return;
    micHandle->micGain = std::max(0.0f, gain);
}

uint32_t Moss_MicrophoneGetSampleRate(Moss_Microphone* micHandle) {
    return micHandle ? micHandle->sampleRate : 0;
}

uint32_t Moss_MicrophoneGetChannels(Moss_Microphone* micHandle) {
    return micHandle ? micHandle->channels : 0;
}

void Moss_MicrophoneSetCallback(Moss_Microphone* micHandle, MicrophoneCallback callback, void* userData) {
    if (!micHandle)
        return;
    micHandle->micCallback = callback;
    micHandle->micUserData = userData;
}

Moss_MicrophoneLevels Moss_MicrophoneGetLevels(Moss_Microphone* micHandle) {
    return micHandle ? micHandle->levels : Moss_MicrophoneLevels{};
}

float Moss_MicrophoneGetLevelRMS(Moss_Microphone* micHandle) {
    return Moss_MicrophoneGetLevels(micHandle).rms;
}

float Moss_MicrophoneGetLevelPeak(Moss_Microphone* micHandle) {
    return Moss_MicrophoneGetLevels(micHandle).peak;
}

float Moss_MicrophoneGetSmoothedVolume(Moss_Microphone* micHandle) {
    return Moss_MicrophoneGetLevels(micHandle).smoothed_volume;
}

float Moss_MicrophoneGetVoiceActivity(Moss_Microphone* micHandle) {
    return Moss_MicrophoneGetLevels(micHandle).voice_activity;
}

// -----------------------------
// Legacy microphone API wrappers
// -----------------------------

int Moss_AudioMicrophoneOpen() {
    if (Moss_IsMicrophoneDeviceReady())
        return 1;

    Moss_MicrophoneDesc desc{};
    desc.start_immediately = false;

    g_legacyMicrophone = Moss_MicrophoneOpen(&desc);
    return g_legacyMicrophone ? 1 : 0;
}

void Moss_AudioMicrophoneClose() {
    if (!g_legacyMicrophone)
        return;

    Moss_MicrophoneClose(g_legacyMicrophone);
    g_legacyMicrophone = nullptr;
}

void Moss_AudioMicrophonePlay() {
    Moss_MicrophoneStart(g_legacyMicrophone);
}

void Moss_AudioMicrophoneStop() {
    Moss_MicrophoneStop(g_legacyMicrophone);
}

int Moss_AudioMicrophoneID() {
    return Moss_IsMicrophoneDeviceReady() ? 0 : -1;
}

// Placeholder for type (e.g., USB, built-in)
void Moss_AudioMicrophoneType() {}

// -----------------------------
// Microphone enumeration
// -----------------------------
int Moss_ListMicrophoneDevices() {
    Moss_AudioEnumerateMicrophones();
    return static_cast<int>(mic.microphoneNames.size());
}

const char* Moss_GetMicrophoneDeviceName(int index) {
    Moss_AudioEnumerateMicrophones();
    if (index < 0 || index >= static_cast<int>(mic.microphoneNames.size()))
        return nullptr;
    return mic.microphoneNames[index].c_str();
}

bool Moss_AudioSelectMicrophoneDevice(int id) {
    if (id < 0)
        return false;

    Moss_AudioMicrophoneClose();

    Moss_MicrophoneDesc desc{};
    desc.device_index = static_cast<uint32_t>(id);
    desc.start_immediately = false;

    g_legacyMicrophone = Moss_MicrophoneOpen(&desc);
    return g_legacyMicrophone != nullptr;
}

void Moss_AudioMicrophoneSetGain(Microphone*, float gain) {
    Moss_MicrophoneSetGain(g_legacyMicrophone, gain);
}

int Moss_AudioMicrophoneGetSampleRate(Microphone*) {
    return static_cast<int>(Moss_MicrophoneGetSampleRate(g_legacyMicrophone));
}

int Moss_AudioMicrophoneGetChannels(Microphone*) {
    return static_cast<int>(Moss_MicrophoneGetChannels(g_legacyMicrophone));
}

void Moss_AudioMicrophoneSetCallback(Microphone*, MicrophoneCallback callback, void* userData) {
    Moss_MicrophoneSetCallback(g_legacyMicrophone, callback, userData);
}
*/
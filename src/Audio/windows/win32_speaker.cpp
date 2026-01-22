// win32_speaker.cpp
#include <Moss/Audio/xaudio/win32_audio.h>

#include <mmdeviceapi.h>
#include <Functiondiscoverykeys_devpkey.h>
#include <xaudio2.h>
#include <string>
#include <vector>

#pragma comment(lib, "winmm.lib")

// -----------------------------
// Speaker device representation
// -----------------------------
struct Moss_SpeakerDevice {
    int id;
    std::string name;
    std::wstring deviceId; // WASAPI ID
};

static std::vector<Moss_SpeakerDevice> g_SpeakerDevices;
static int g_CurrentSpeakerID = -1;
extern IXAudio2* g_xaudio2;
extern IXAudio2MasteringVoice* g_masteringVoice;
extern IXAudio2SourceVoice* g_SourceVoice;
extern bool g_SpeakerPaused;

// -----------------------------
// Basic speaker functions
// -----------------------------
bool Moss_IsSpeakerDeviceReady() {
    return g_xaudio2 && g_masteringVoice;
}

void Moss_AudioSpeakerOpen() {
    if (Moss_IsSpeakerDeviceReady()) return;

    HRESULT hr = XAudio2Create(&g_xaudio2, 0, XAUDIO2_DEFAULT_PROCESSOR);
    if (FAILED(hr)) return;

    hr = g_xaudio2->CreateMasteringVoice(&g_masteringVoice);
    if (FAILED(hr)) {
        g_xaudio2->Release();
        g_xaudio2 = nullptr;
    }
}

void Moss_AudioSpeakerPause() {
    if (g_SourceVoice) {
        g_SourceVoice->Stop();
        g_SourceVoice->FlushSourceBuffers();
        g_SpeakerPaused = true;
    }
}

void Moss_AudioSpeakerResume() {
    if (g_SourceVoice && g_SpeakerPaused) {
        g_SourceVoice->Start();
        g_SpeakerPaused = false;
    }
}

bool Moss_AudioSpeakerIsPaused() {
    return g_SpeakerPaused;
}

// -----------------------------
// Device enumeration
// -----------------------------
void Moss_EnumerateSpeakers() {
    g_SpeakerDevices.clear();

    IMMDeviceEnumerator* enumerator = nullptr;
    IMMDeviceCollection* collection = nullptr;

    if (FAILED(CoCreateInstance(__uuidof(MMDeviceEnumerator),nullptr, CLSCTX_ALL, __uuidof(IMMDeviceEnumerator), (void**)&enumerator)))
        return;

    if (FAILED(enumerator->EnumAudioEndpoints(eRender, DEVICE_STATE_ACTIVE, &collection))) {
        enumerator->Release();
        return;
    }

    UINT count = 0;
    collection->GetCount(&count);

    for (UINT i = 0; i < count; ++i) {
        IMMDevice* device = nullptr;
        collection->Item(i, &device);

        LPWSTR id = nullptr;
        device->GetId(&id);

        IPropertyStore* props = nullptr;
        device->OpenPropertyStore(STGM_READ, &props);

        PROPVARIANT varName;
        PropVariantInit(&varName);
        props->GetValue(PKEY_Device_FriendlyName, &varName);

        Moss_SpeakerDevice speaker{};
        speaker.id = static_cast<int>(i);
        speaker.name = std::string(varName.pwszVal, varName.pwszVal + wcslen(varName.pwszVal));
        speaker.deviceId = id;

        g_SpeakerDevices.push_back(speaker);

        PropVariantClear(&varName);
        props->Release();
        device->Release();
    }

    collection->Release();
    enumerator->Release();
}

// -----------------------------
// Device selection
// -----------------------------
bool Moss_AudioSelectSpeakerDevice(int id) {
    if (id < 0 || id >= static_cast<int>(g_SpeakerDevices.size()))
        return false;

    if (g_masteringVoice)
        g_masteringVoice->DestroyVoice();

    const wchar_t* deviceId = g_SpeakerDevices[id].deviceId.c_str();

    HRESULT hr = g_xaudio2->CreateMasteringVoice(&g_masteringVoice, XAUDIO2_DEFAULT_CHANNELS, XAUDIO2_DEFAULT_SAMPLERATE, 0, deviceId);

    if (FAILED(hr)) return false;

    g_CurrentSpeakerID = id;
    return true;
}

int Moss_GetCurrentSpeakerDeviceID() {
    return g_CurrentSpeakerID;
}

// -----------------------------
// Speaker helper functions
// -----------------------------
const char* Moss_GetSpeakerDeviceName(int id) {
    if (id < 0 || id >= static_cast<int>(g_SpeakerDevices.size()))
        return nullptr;
    return g_SpeakerDevices[id].name.c_str();
}
// Returns the number of enumerated speaker devices
int Moss_ListSpeakerDevices() {
    return static_cast<int>(g_SpeakerDevices.size());
}
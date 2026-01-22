#ifndef MOSS_WIN32_AUDIO_H
#define MOSS_WIN32_AUDIO_H

#include <Moss/Audio/audio_intern.h>

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <mmsystem.h>
#include <xaudio2.h>
#include <mmdeviceapi.h>
#include <audioclient.h>

// XAudio2
static IXAudio2* g_xaudio2 = nullptr;
static IXAudio2MasteringVoice* g_masteringVoice = nullptr;

// WASAPI
static IMMDeviceEnumerator* g_deviceEnumerator = nullptr;
static IMMDevice* g_microphoneDevice = nullptr;
static IAudioClient* g_audioClient = nullptr;
static IAudioCaptureClient* g_captureClient = nullptr;


extern std::vector<AudioStream*> g_activeStreams;

struct Moss_Microphone{
    IMMDevice* inputDevice = nullptr;
    IAudioClient* audioClient = nullptr;
    IAudioCaptureClient* captureClient = nullptr;
    WAVEFORMATEX* format = nullptr;

    std::vector<std::wstring> microphoneIds;
    std::vector<std::string>  microphoneNames;

    std::atomic<bool> capturing = false;
    std::thread captureThread;

    float micGain = 1.0f;

    MicrophoneCallback micCallback = nullptr;
    void* micUserData = nullptr;
};

extern Moss_Microphone mic;

// AudioStreamMicrophone
struct AudioStream {
    IXAudio2SourceVoice* sourceVoice = nullptr;

    Wav* wav = nullptr;
    Moss_Microphone* mic = nullptr;

    bool playing = false;
    bool loop = false;
    float pitch = 1.0f;
    float volume = 1.0f;
    ChannelID channel = 0;

    AudioStreamCallback callback = nullptr;
    void* callbackUser = nullptr;
};

struct AudioStream2D {
    AudioStream stream;
    float pan;
    float maxDistance = 100.0f;

    Float2 position {0, 0};

    DistanceModel distanceModel = DistanceModel::Linear;
};

struct AudioStream3D { 
    AudioStream stream;

    Vec3 position {0, 0, 0};
    Vec3 velocity {0, 0, 0};

    float pan;
    float dopplerScale;
    float maxDistance = 100.0f;
    DistanceModel distanceModel = DistanceModel::Linear;
};


struct StreamVoiceCallback : public IXAudio2VoiceCallback {
    AudioStream* stream = nullptr;

    void OnBufferEnd(void*) override {
        if (!stream) return;

        if (stream->loop && stream->wav) {
            XAUDIO2_BUFFER buf{};
            buf.AudioBytes = stream->wav->dataChunkSize;
            buf.pAudioData = (BYTE*)stream->wav->dataBegin;
            buf.Flags = XAUDIO2_END_OF_STREAM;
            stream->sourceVoice->SubmitSourceBuffer(&buf);
        }
        else {
            stream->playing = false;
        }
    }

    // Unused but required
    void OnVoiceProcessingPassStart(UINT32) override {}
    void OnVoiceProcessingPassEnd() override {}
    void OnStreamEnd() override {}
    void OnBufferStart(void*) override {}
    void OnLoopEnd(void*) override {}
    void OnVoiceError(void*, HRESULT) override {}
};


///////////////////////////////////////////////////////



struct Win32Speaker {
    HWAVEOUT hWaveOut = nullptr;
    WAVEFORMATEX format = {};
    WAVEHDR header = {};
    bool isPlaying = false;

    void* audioBuffer = nullptr;
    size_t bufferSize = 0;
};

#endif // MOSS_WIN32_AUDIO_H
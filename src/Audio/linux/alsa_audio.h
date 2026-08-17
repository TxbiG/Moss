#ifndef MOSS_LINUX_ALSA_H
#define MOSS_LINUX_ALSA_H

#include <Moss/Audio/audio_intern.h>
#include <alsa/asoundlib.h>

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct Moss_Microphone {
    snd_pcm_t* captureHandle = nullptr;

    std::vector<std::string> microphoneNames;

    std::atomic<bool> capturing = false;
    std::thread captureThread;

    std::mutex bufferMutex;
    std::vector<float> ringBuffer;
    uint32_t ringFrameCapacity = 0;
    uint32_t readFrame = 0;
    uint32_t writeFrame = 0;
    uint32_t availableFrames = 0;

    uint32_t sampleRate = 48000;
    uint32_t channels = 1;
    uint32_t bufferFrames = 480;
    bool voiceMetricsEnabled = true;

    float micGain = 1.0f;
    Moss_MicrophoneLevels levels{};

    MicrophoneCallback micCallback = nullptr;
    void* micUserData = nullptr;
};

extern Moss_Microphone mic;

struct AudioStream {
    snd_pcm_t* pcm = nullptr;
    Wav* wav = nullptr;

    std::atomic<bool> playing = false;
    std::atomic<bool> stopRequested = false;
    std::thread playbackThread;
    bool loop = false;
    float pitch = 1.0f;
    float playbackRate = 1.0f;
    float pan = 0.0f;
    float volume = 1.0f;
    ChannelID channel = 0;

    ~AudioStream();
    void play();
    void stop();
    void setVolume(float value);
    void setPitch(float value);
    void setPlaybackRate(float value);
    void setPan(float value);
    void setLooping(bool value);
};

struct AudioStream2D {
    AudioStream stream;
    Vec2 position = Vec2::Zero();
    Vec2 velocity = Vec2::Zero();
    float maxDistance = 100.0f;
    float pan = 0.0f;
    DistanceModel distanceModel = DistanceModel::LINEAR;

    void play();
    void stop();
};

struct AudioStream3D {
    AudioStream stream;
    Vec3 position = Vec3::sZero();
    Vec3 velocity = Vec3::sZero();
    float maxDistance = 100.0f;
    float pan = 0.0f;
    float dopplerScale = 1.0f;
    DistanceModel distanceModel = DistanceModel::LINEAR;

    void play();
    void stop();
};

#endif // MOSS_LINUX_ALSA_AUDIO_H
#ifndef MOSS_MAC_COREAUDIO_H
#define MOSS_MAC_COREAUDIO_H


#include <Moss/Audio/audio_intern.h>

#include <AudioToolbox/AudioToolbox.h>
#include <CoreAudio/CoreAudioTypes.h>
#include <CoreAudio/CoreAudio.h>

#include <vector>
#include <mutex>

static AudioUnit outputUnit = nullptr;
std::vector<AudioStream*> g_activeStreams;
std::mutex audioMutex;

struct Moss_Microphone {
    void* platform_data;          // OS-specific data
    Moss_MicrophoneCallback on_data;   // Callback when data is available
    void* user_data;
    int (*start)(struct Moss_Microphone*);
    int (*stop)(struct Moss_Microphone*);
};


struct AudioStream::AudioStream_t {
    AudioQueueRef queue;
    AudioQueueBufferRef buffers[3];
    Wav* wav;

    bool playing = false;
    bool loop;
    float pitch;
    float volume;
    ChannelID channel;
};

#endif // MOSS_MAC_COREAUDIO_H
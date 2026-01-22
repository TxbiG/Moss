#ifndef MOSS_LINUX_ALSA_H
#define MOSS_LINUX_ALSA_H

#include <Moss/Audio/audio_intern.h>
#include <alsa/asoundlib.h>



struct Moss_Microphone {
    void* platform_data;          // OS-specific data
    Moss_MicrophoneCallback on_data;   // Callback when data is available
    void* user_data;
    int (*start)(struct Moss_Microphone*);
    int (*stop)(struct Moss_Microphone*);
};


struct AudioStream::AudioStream_t {
    snd_pcm_t* pcm;
    Wav* wav;

    bool playing = false;
    bool loop;
    float pitch;
    float volume;
    ChannelID channel;
};

#endif // MOSS_LINUX_ALSA_AUDIO_H
// alsa_microphone.cpp
#include "audio_intern.h"

#include <alsa/asoundlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <stdio.h>
#include <vector>
#include <string>

// -----------------------------
// Moss_Microphone platform data
// -----------------------------
struct LinuxMicData {
    int fd;
    int recording;
    char buffer[4096];
};

// -----------------------------
// Check if default microphone is ready
// -----------------------------
static bool Moss_IsMicrophoneDeviceReady() {
    snd_pcm_t* handle;
    int err = snd_pcm_open(&handle, "default", SND_PCM_STREAM_CAPTURE, 0);
    if (err < 0) return false;

    snd_pcm_close(handle);
    return true;
}

// -----------------------------
// Start / stop capture
// -----------------------------
static int mic_start(Moss_Microphone* mic) {
    LinuxMicData* data = (LinuxMicData*)mic->platform_data;
    data->recording = 1;

    while (data->recording) {
        ssize_t len = read(data->fd, data->buffer, sizeof(data->buffer));
        if (len > 0 && mic->on_data) {
            mic->on_data(mic->user_data, data->buffer, len);
        } else if (len < 0 && errno != EAGAIN) {
            perror("read");
            return -1;
        }
    }
    return 0;
}

static int mic_stop(Moss_Microphone* mic) {
    LinuxMicData* data = (LinuxMicData*)mic->platform_data;
    data->recording = 0;
    return 0;
}

// -----------------------------
// Initialize microphone
// -----------------------------
int microphone_init(Moss_Microphone* mic, MicrophoneCallback callback, void* user_data) {
    if (!mic) return -1;

    memset(mic, 0, sizeof(Moss_Microphone));
    mic->on_data = callback;
    mic->user_data = user_data;
    mic->start = mic_start;
    mic->stop = mic_stop;

    LinuxMicData* data = (LinuxMicData*)calloc(1, sizeof(LinuxMicData));
    if (!data) return -1;

    data->fd = open("/dev/dsp", O_RDONLY);
    if (data->fd < 0) {
        perror("open /dev/dsp");
        free(data);
        return -1;
    }

    int format = AFMT_S16_LE;
    int channels = 1;
    int rate = 44100;

    ioctl(data->fd, SNDCTL_DSP_SETFMT, &format);
    ioctl(data->fd, SNDCTL_DSP_CHANNELS, &channels);
    ioctl(data->fd, SNDCTL_DSP_SPEED, &rate);

    mic->platform_data = data;
    return 0;
}

// -----------------------------
// Free microphone resources
// -----------------------------
int microphone_free(Moss_Microphone* mic) {
    if (mic->platform_data) {
        LinuxMicData* data = (LinuxMicData*)mic->platform_data;
        if (data->fd >= 0) close(data->fd);
        free(data);
        mic->platform_data = nullptr;
    }
    return 0;
}

// -----------------------------
// List microphone devices (ALSA)
// -----------------------------
int Moss_MicrophoneListDevices(char*** device_names, int* count) {
    if (!device_names || !count) return -1;

    *device_names = nullptr;
    *count = 0;

    std::vector<std::string> names;
    int card = -1;

    while (snd_card_next(&card) >= 0 && card >= 0) {
        snd_ctl_t* handle;
        snd_ctl_card_info_t* info;
        snd_ctl_card_info_alloca(&info);

        if (snd_ctl_open(&handle, card, 0) < 0) continue;
        if (snd_ctl_card_info(handle, info) >= 0) {
            const char* name = snd_ctl_card_info_get_name(info);
            if (name) names.push_back(name);
        }
        snd_ctl_close(handle);
    }

    *count = static_cast<int>(names.size());
    if (*count == 0) return 0;

    *device_names = (char**)calloc(*count, sizeof(char*));
    for (int i = 0; i < *count; ++i)
        (*device_names)[i] = strdup(names[i].c_str());

    return 0;
}

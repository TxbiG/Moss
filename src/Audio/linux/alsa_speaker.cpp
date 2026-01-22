// alsa_speaker.cpp
#include "audio_intern.h"
#include <alsa/asoundlib.h>
#include <vector>
#include <string>
#include <cstring>

// -----------------------------
// Speaker device representation
// -----------------------------
struct Moss_SpeakerDevice {
    int id;
    std::string name;
    std::string alsaName;
};

static std::vector<Moss_SpeakerDevice> g_SpeakerDevices;
static int g_CurrentSpeakerID = -1;
static snd_pcm_t* g_PCM = nullptr;
static bool g_SpeakerPaused = false;

// -----------------------------
// Basic speaker functions
// -----------------------------
bool Moss_IsSpeakerDeviceReady() {
    return g_PCM != nullptr;
}

bool Moss_AudioSpeakerOpen() {
    if (g_PCM)
        return true;

    if (snd_pcm_open(&g_PCM, "default", SND_PCM_STREAM_PLAYBACK, 0) < 0)
        return false;

    snd_pcm_set_params(
        g_PCM,
        SND_PCM_FORMAT_S16_LE,
        SND_PCM_ACCESS_RW_INTERLEAVED,
        2,        // channels
        48000,    // sample rate
        1,
        500000    // latency in µs
    );

    return true;
}

void Moss_AudioSpeakerPause() {
    if (g_PCM) {
        snd_pcm_pause(g_PCM, 1);
        g_SpeakerPaused = true;
    }
}

void Moss_AudioSpeakerResume() {
    if (g_PCM && g_SpeakerPaused) {
        snd_pcm_pause(g_PCM, 0);
        g_SpeakerPaused = false;
    }
}

bool Moss_AudioSpeakerIsPaused() {
    return g_SpeakerPaused;
}

int Moss_AudioSpeakerID() {
    return g_PCM ? 0 : -1;
}

// -----------------------------
// Device enumeration
// -----------------------------
void Moss_EnumerateSpeakers() {
    g_SpeakerDevices.clear();

    void** hints;
    if (snd_device_name_hint(-1, "pcm", &hints) < 0) return;

    for (void** n = hints; *n != nullptr; ++n) {
        char* name = snd_device_name_get_hint(*n, "NAME");
        char* desc = snd_device_name_get_hint(*n, "DESC");

        if (!name || strcmp(name, "null") == 0) {
            if (name) free(name);
            if (desc) free(desc);
            continue;
        }

        Moss_SpeakerDevice speaker{};
        speaker.id = static_cast<int>(g_SpeakerDevices.size());
        speaker.alsaName = name;
        speaker.name = desc ? desc : name;

        g_SpeakerDevices.push_back(speaker);

        free(name);
        if (desc) free(desc);
    }

    snd_device_name_free_hint(hints);
}

// -----------------------------
// Device selection
// -----------------------------
bool Moss_SelectSpeakerDevice(int id) {
    if (id < 0 || id >= static_cast<int>(g_SpeakerDevices.size()))
        return false;

    if (g_PCM) {
        snd_pcm_close(g_PCM);
        g_PCM = nullptr;
    }

    const char* deviceName = g_SpeakerDevices[id].alsaName.c_str();

    if (snd_pcm_open(&g_PCM, deviceName, SND_PCM_STREAM_PLAYBACK, 0) < 0) {
        g_PCM = nullptr;
        return false;
    }

    snd_pcm_set_params(
        g_PCM,
        SND_PCM_FORMAT_S16_LE,
        SND_PCM_ACCESS_RW_INTERLEAVED,
        2,
        48000,
        1,
        500000
    );

    g_CurrentSpeakerID = id;
    return true;
}

int Moss_GetCurrentSpeakerDeviceID() {
    return g_CurrentSpeakerID;
}

// -----------------------------
// Helper functions
// -----------------------------
int Moss_ListSpeakerDevices() {
    return static_cast<int>(g_SpeakerDevices.size());
}

const char* Moss_GetSpeakerDeviceName(int id) {
    if (id < 0 || id >= static_cast<int>(g_SpeakerDevices.size()))
        return nullptr;
    return g_SpeakerDevices[id].name.c_str();
}

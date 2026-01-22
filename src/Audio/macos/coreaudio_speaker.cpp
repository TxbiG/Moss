// coreaudio_speaker.cpp
#include "audio_intern.h"

#include <AudioToolbox/AudioToolbox.h>
#include <CoreAudio/CoreAudio.h>
#include <vector>
#include <string>
#include <cstring>
#include <cstdlib>

// -----------------------------
// Speaker device representation
// -----------------------------
struct Moss_SpeakerDevice {
    int id;
    std::string name;
    AudioDeviceID deviceId;
};

static std::vector<Moss_SpeakerDevice> g_SpeakerDevices;
static int g_CurrentSpeakerID = -1;
static AudioQueueRef g_AudioQueue = nullptr;
static bool g_SpeakerPaused = false;

// -----------------------------
// Basic speaker functions
// -----------------------------
bool Moss_IsSpeakerDeviceReady() {
    return g_AudioQueue != nullptr;
}

bool Moss_AudioSpeakerOpen() {
    if (g_AudioQueue) return true;

    AudioStreamBasicDescription format{};
    format.mSampleRate = 48000;
    format.mFormatID = kAudioFormatLinearPCM;
    format.mFormatFlags = kAudioFormatFlagIsSignedInteger | kAudioFormatFlagIsPacked;
    format.mBitsPerChannel = 16;
    format.mChannelsPerFrame = 2;
    format.mBytesPerFrame = 4;
    format.mFramesPerPacket = 1;
    format.mBytesPerPacket = 4;

    OSStatus status = AudioQueueNewOutput(
        &format,
        nullptr, // callback optional
        nullptr,
        nullptr,
        nullptr,
        0,
        &g_AudioQueue
    );

    return status == noErr;
}

void Moss_AudioSpeakerPause() {
    if (g_AudioQueue) {
        AudioQueuePause(g_AudioQueue);
        g_SpeakerPaused = true;
    }
}

void Moss_AudioSpeakerResume() {
    if (g_AudioQueue && g_SpeakerPaused) {
        AudioQueueStart(g_AudioQueue, nullptr);
        g_SpeakerPaused = false;
    }
}

bool Moss_AudioSpeakerIsPaused() {
    return g_SpeakerPaused;
}

// -----------------------------
// Speaker device enumeration
// -----------------------------
void Moss_EnumerateSpeakers() {
    g_SpeakerDevices.clear();

    UInt32 size = 0;
    AudioObjectPropertyAddress addr{
        kAudioHardwarePropertyDevices,
        kAudioObjectPropertyScopeGlobal,
        kAudioObjectPropertyElementMaster
    };

    if (AudioObjectGetPropertyDataSize(kAudioObjectSystemObject, &addr, 0, nullptr, &size) != noErr)
        return;

    int count = size / sizeof(AudioDeviceID);
    std::vector<AudioDeviceID> devices(count);

    if (AudioObjectGetPropertyData(kAudioObjectSystemObject, &addr, 0, nullptr, &size, devices.data()) != noErr)
        return;

    for (AudioDeviceID dev : devices) {
        UInt32 outputChannels = 0;
        UInt32 chSize = sizeof(UInt32);
        AudioObjectPropertyAddress chAddr{
            kAudioDevicePropertyStreamConfiguration,
            kAudioDevicePropertyScopeOutput,
            kAudioObjectPropertyElementMaster
        };

        AudioObjectGetPropertyData(dev, &chAddr, 0, nullptr, &chSize, &outputChannels);
        if (outputChannels == 0) continue; // skip non-output devices

        CFStringRef name = nullptr;
        UInt32 nameSize = sizeof(name);
        AudioObjectPropertyAddress nameAddr{
            kAudioObjectPropertyName,
            kAudioObjectPropertyScopeGlobal,
            kAudioObjectPropertyElementMaster
        };

        if (AudioObjectGetPropertyData(dev, &nameAddr, 0, nullptr, &nameSize, &name) == noErr && name) {
            char cname[256];
            CFStringGetCString(name, cname, sizeof(cname), kCFStringEncodingUTF8);
            CFRelease(name);

            Moss_SpeakerDevice speaker{};
            speaker.id = static_cast<int>(g_SpeakerDevices.size());
            speaker.name = cname;
            speaker.deviceId = dev;

            g_SpeakerDevices.push_back(speaker);
        }
    }
}

// -----------------------------
// Speaker device selection
// -----------------------------
bool Moss_SelectSpeakerDevice(int id) {
    if (id < 0 || id >= static_cast<int>(g_SpeakerDevices.size()))
        return false;

    if (g_AudioQueue) {
        AudioQueueStop(g_AudioQueue, true);
        AudioQueueDispose(g_AudioQueue, true);
        g_AudioQueue = nullptr;
    }

    AudioDeviceID deviceID = g_SpeakerDevices[id].deviceId;

    AudioStreamBasicDescription format{};
    format.mSampleRate = 48000;
    format.mFormatID = kAudioFormatLinearPCM;
    format.mFormatFlags = kAudioFormatFlagIsSignedInteger | kAudioFormatFlagIsPacked;
    format.mBitsPerChannel = 16;
    format.mChannelsPerFrame = 2;
    format.mBytesPerFrame = 4;
    format.mFramesPerPacket = 1;
    format.mBytesPerPacket = 4;

    AudioQueueRef queue = nullptr;
    OSStatus status = AudioQueueNewOutput(&format, nullptr, nullptr, nullptr, nullptr, 0, &queue);
    if (status != noErr) return false;

    status = AudioQueueSetProperty(queue, kAudioQueueProperty_CurrentDevice, &deviceID, sizeof(deviceID));
    if (status != noErr) {
        AudioQueueDispose(queue, true);
        return false;
    }

    g_AudioQueue = queue;
    g_CurrentSpeakerID = id;
    return true;
}

int Moss_AudioSpeakerID() {
    return g_AudioQueue ? 0 : -1;
}

int Moss_GetCurrentSpeakerDeviceID() {
    return g_CurrentSpeakerID;
}


// -----------------------------
// Speaker helper functions
// -----------------------------

// Returns the number of enumerated speaker devices
int Moss_ListSpeakerDevices() {
    return static_cast<int>(g_SpeakerDevices.size());
}

// Returns the name of the speaker device by ID
// Returns nullptr if the ID is invalid
const char* Moss_GetSpeakerDeviceName(int id) {
    if (id < 0 || id >= static_cast<int>(g_SpeakerDevices.size()))
        return nullptr;
    return g_SpeakerDevices[id].name.c_str();
}
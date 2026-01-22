// coreaudio_microphone.cpp
#include "audio_intern.h"
#include <AudioToolbox/AudioToolbox.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <vector>
#include <string>

// -----------------------------
// Moss_Microphone platform data
// -----------------------------
struct MacMicData {
    AudioQueueRef queue;
    AudioQueueBufferRef buffer;
    int recording;
    Moss_Microphone* mic;
};

// -----------------------------
// AudioQueue callback
// -----------------------------
static void AQInputCallback(void* userData, AudioQueueRef inQ, AudioQueueBufferRef inBuffer,
                            const AudioTimeStamp* inStartTime, UInt32 inNumPackets,
                            const AudioStreamPacketDescription* inPacketDesc) {
    MacMicData* data = (MacMicData*)userData;
    if (data->recording && data->mic->on_data) {
        data->mic->on_data(data->mic->user_data, inBuffer->mAudioData, inBuffer->mAudioDataByteSize);
    }
    AudioQueueEnqueueBuffer(inQ, inBuffer, 0, NULL);
}

// -----------------------------
// Check if microphone is ready
// -----------------------------
static bool Moss_IsMicrophoneDeviceReady() {
    AudioObjectPropertyAddress address = {
        kAudioHardwarePropertyDevices,
        kAudioObjectPropertyScopeInput,
        kAudioObjectPropertyElementMaster
    };

    UInt32 dataSize = 0;
    if (AudioObjectGetPropertyDataSize(kAudioObjectSystemObject, &address, 0, NULL, &dataSize) != noErr)
        return false;

    int deviceCount = dataSize / sizeof(AudioDeviceID);
    AudioDeviceID* devices = (AudioDeviceID*)malloc(dataSize);
    if (!devices) return false;

    if (AudioObjectGetPropertyData(kAudioObjectSystemObject, &address, 0, NULL, &dataSize, devices) != noErr) {
        free(devices);
        return false;
    }

    for (int i = 0; i < deviceCount; i++) {
        AudioObjectPropertyAddress deviceAddress = {
            kAudioDevicePropertyDeviceIsRunning,
            kAudioObjectPropertyScopeInput,
            kAudioObjectPropertyElementMaster
        };
        UInt32 isRunning = 0;
        UInt32 size = sizeof(isRunning);
        if (AudioObjectGetPropertyData(devices[i], &deviceAddress, 0, NULL, &size, &isRunning) == noErr && isRunning) {
            free(devices);
            return true;
        }
    }

    free(devices);
    return false;
}

// -----------------------------
// Start / Stop capture
// -----------------------------
static int mic_start(Moss_Microphone* mic) {
    MacMicData* data = (MacMicData*)mic->platform_data;
    data->recording = 1;
    return AudioQueueStart(data->queue, NULL);
}

static int mic_stop(Moss_Microphone* mic) {
    MacMicData* data = (MacMicData*)mic->platform_data;
    data->recording = 0;
    return AudioQueueStop(data->queue, false);
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

    MacMicData* data = (MacMicData*)calloc(1, sizeof(MacMicData));
    if (!data) return -1;

    mic->platform_data = data;
    data->mic = mic;

    AudioStreamBasicDescription format = {0};
    format.mSampleRate = 44100;
    format.mFormatID = kAudioFormatLinearPCM;
    format.mFormatFlags = kLinearPCMFormatFlagIsSignedInteger | kLinearPCMFormatFlagIsPacked;
    format.mBitsPerChannel = 16;
    format.mChannelsPerFrame = 1;
    format.mBytesPerFrame = 2;
    format.mFramesPerPacket = 1;
    format.mBytesPerPacket = 2;

    OSStatus status = AudioQueueNewInput(&format, AQInputCallback, data, NULL, kCFRunLoopCommonModes, 0, &data->queue);
    if (status != noErr) {
        free(data);
        mic->platform_data = NULL;
        return -1;
    }

    AudioQueueAllocateBuffer(data->queue, 4096, &data->buffer);
    AudioQueueEnqueueBuffer(data->queue, data->buffer, 0, NULL);

    return 0;
}

// -----------------------------
// Free microphone resources
// -----------------------------
int microphone_free(Moss_Microphone* mic) {
    if (mic->platform_data) {
        MacMicData* data = (MacMicData*)mic->platform_data;
        AudioQueueStop(data->queue, true);
        AudioQueueDispose(data->queue, true);
        free(data);
        mic->platform_data = NULL;
    }
    return 0;
}

// -----------------------------
// List microphone devices
// -----------------------------
int Moss_MicrophoneListDevices(char*** device_names, int* count) {
    if (!device_names || !count) return -1;

    UInt32 size = 0;
    AudioObjectPropertyAddress address = {
        kAudioHardwarePropertyDevices,
        kAudioObjectPropertyScopeGlobal,
        kAudioObjectPropertyElementMaster
    };

    if (AudioObjectGetPropertyDataSize(kAudioObjectSystemObject, &address, 0, NULL, &size) != noErr)
        return -1;

    int deviceCount = size / sizeof(AudioDeviceID);
    AudioDeviceID* devices = (AudioDeviceID*)malloc(size);
    if (!devices) return -1;

    if (AudioObjectGetPropertyData(kAudioObjectSystemObject, &address, 0, NULL, &size, devices) != noErr) {
        free(devices);
        return -1;
    }

    *device_names = (char**)calloc(deviceCount, sizeof(char*));
    *count = 0;

    for (int i = 0; i < deviceCount; i++) {
        CFStringRef name = NULL;
        AudioObjectPropertyAddress nameAddr = {
            kAudioObjectPropertyName,
            kAudioObjectPropertyScopeGlobal,
            kAudioObjectPropertyElementMaster
        };

        UInt32 nameSize = sizeof(name);
        if (AudioObjectGetPropertyData(devices[i], &nameAddr, 0, NULL, &nameSize, &name) == noErr && name) {
            char buf[256];
            if (CFStringGetCString(name, buf, sizeof(buf), kCFStringEncodingUTF8)) {
                (*device_names)[*count] = strdup(buf);
                (*count)++;
            }
            CFRelease(name);
        }
    }

    free(devices);
    return 0;
}

// coreaudio_microphone.cpp
#include "coreaudio_audio.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>


Moss_Microphone mic;
static Moss_Microphone* g_legacyMicrophone = nullptr;

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


/*

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
        peak = std::max(peak, std::fabs(value));
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

static void Moss_AudioQueueInputCallback(void* userData, AudioQueueRef queue, AudioQueueBufferRef buffer, const AudioTimeStamp*, UInt32, const AudioStreamPacketDescription*) {
    Moss_Microphone* micHandle = static_cast<Moss_Microphone*>(userData);
    if (!micHandle || !micHandle->capturing) {
        AudioQueueEnqueueBuffer(queue, buffer, 0, nullptr);
        return;
    }

    const uint32_t sampleCount = static_cast<uint32_t>(buffer->mAudioDataByteSize / sizeof(float));
    const uint32_t frameCount = micHandle->channels ? sampleCount / micHandle->channels : 0;
    float* samples = static_cast<float*>(buffer->mAudioData);

    for (uint32_t i = 0; i < sampleCount; ++i)
        samples[i] = std::clamp(samples[i] * micHandle->micGain, -1.0f, 1.0f);

    Moss_UpdateMicrophoneLevels(micHandle, samples, sampleCount);
    Moss_PushMicrophoneFrames(micHandle, samples, frameCount);
    if (micHandle->micCallback)
        micHandle->micCallback(samples, static_cast<int>(frameCount), micHandle->micUserData);

    AudioQueueEnqueueBuffer(queue, buffer, 0, nullptr);
}

static void Moss_EnumerateMicrophonesInto(Moss_Microphone* micHandle) {
    if (!micHandle)
        return;

    micHandle->microphoneNames.clear();
    AudioObjectPropertyAddress address = { kAudioHardwarePropertyDevices, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMaster };
    UInt32 size = 0;
    if (AudioObjectGetPropertyDataSize(kAudioObjectSystemObject, &address, 0, nullptr, &size) != noErr) {
        micHandle->microphoneNames.emplace_back("default");
        return;
    }

    std::vector<AudioDeviceID> devices(size / sizeof(AudioDeviceID));
    if (AudioObjectGetPropertyData(kAudioObjectSystemObject, &address, 0, nullptr, &size, devices.data()) != noErr) {
        micHandle->microphoneNames.emplace_back("default");
        return;
    }

    for (AudioDeviceID device : devices) {
        AudioObjectPropertyAddress streamAddress = { kAudioDevicePropertyStreamConfiguration, kAudioObjectPropertyScopeInput, kAudioObjectPropertyElementMaster };
        UInt32 streamSize = 0;
        if (AudioObjectGetPropertyDataSize(device, &streamAddress, 0, nullptr, &streamSize) != noErr || streamSize == 0)
            continue;

        std::vector<uint8_t> storage(streamSize);
        AudioBufferList* buffers = reinterpret_cast<AudioBufferList*>(storage.data());
        if (AudioObjectGetPropertyData(device, &streamAddress, 0, nullptr, &streamSize, buffers) != noErr)
            continue;

        UInt32 channels = 0;
        for (UInt32 i = 0; i < buffers->mNumberBuffers; ++i)
            channels += buffers->mBuffers[i].mNumberChannels;
        if (channels == 0)
            continue;

        CFStringRef name = nullptr;
        AudioObjectPropertyAddress nameAddress = { kAudioObjectPropertyName, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMaster };
        UInt32 nameSize = sizeof(name);
        if (AudioObjectGetPropertyData(device, &nameAddress, 0, nullptr, &nameSize, &name) == noErr && name) {
            char text[256] = {};
            if (CFStringGetCString(name, text, sizeof(text), kCFStringEncodingUTF8))
                micHandle->microphoneNames.emplace_back(text);
            CFRelease(name);
        }
    }

    if (micHandle->microphoneNames.empty())
        micHandle->microphoneNames.emplace_back("default");
}

uint32_t Moss_MicrophoneGetDeviceCount() {
    Moss_EnumerateMicrophonesInto(&mic);
    return static_cast<uint32_t>(mic.microphoneNames.size());
}

const char* Moss_MicrophoneGetDeviceName(uint32_t device_index) {
    Moss_EnumerateMicrophonesInto(&mic);
    if (device_index >= mic.microphoneNames.size())
        return nullptr;
    return mic.microphoneNames[device_index].c_str();
}

Moss_Microphone* Moss_MicrophoneOpen(const Moss_MicrophoneDesc* desc) {
    Moss_MicrophoneDesc defaultDesc{};
    if (!desc)
        desc = &defaultDesc;

    Moss_Microphone* micHandle = new Moss_Microphone();
    micHandle->sampleRate = desc->sample_rate ? desc->sample_rate : 48000;
    micHandle->channels = desc->channels ? desc->channels : 1;
    micHandle->bufferFrames = desc->buffer_frames ? desc->buffer_frames : 480;
    micHandle->voiceMetricsEnabled = desc->enable_voice_metrics;
    Moss_EnumerateMicrophonesInto(micHandle);
    Moss_ResetMicrophoneBuffer(micHandle, desc->ring_buffer_frames ? desc->ring_buffer_frames : micHandle->sampleRate);

    micHandle->inputFormat.mSampleRate = micHandle->sampleRate;
    micHandle->inputFormat.mFormatID = kAudioFormatLinearPCM;
    micHandle->inputFormat.mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked;
    micHandle->inputFormat.mBitsPerChannel = 32;
    micHandle->inputFormat.mChannelsPerFrame = micHandle->channels;
    micHandle->inputFormat.mFramesPerPacket = 1;
    micHandle->inputFormat.mBytesPerFrame = micHandle->channels * sizeof(float);
    micHandle->inputFormat.mBytesPerPacket = micHandle->inputFormat.mBytesPerFrame;

    OSStatus status = AudioQueueNewInput(&micHandle->inputFormat, Moss_AudioQueueInputCallback, micHandle, nullptr, kCFRunLoopCommonModes, 0, &micHandle->inputQueue);
    if (status != noErr) {
        delete micHandle;
        return nullptr;
    }

    const UInt32 bufferBytes = micHandle->bufferFrames * micHandle->channels * sizeof(float);
    for (AudioQueueBufferRef& inputBuffer : micHandle->inputBuffers) {
        if (AudioQueueAllocateBuffer(micHandle->inputQueue, bufferBytes, &inputBuffer) != noErr) {
            Moss_MicrophoneClose(micHandle);
            return nullptr;
        }
        AudioQueueEnqueueBuffer(micHandle->inputQueue, inputBuffer, 0, nullptr);
    }

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
    if (micHandle->inputQueue) {
        AudioQueueDispose(micHandle->inputQueue, true);
        micHandle->inputQueue = nullptr;
    }
    if (micHandle != &mic)
        delete micHandle;
}

bool Moss_MicrophoneStart(Moss_Microphone* micHandle) {
    if (!micHandle || !micHandle->inputQueue || micHandle->capturing)
        return false;
    micHandle->capturing = true;
    return AudioQueueStart(micHandle->inputQueue, nullptr) == noErr;
}

void Moss_MicrophoneStop(Moss_Microphone* micHandle) {
    if (!micHandle || !micHandle->capturing)
        return;
    micHandle->capturing = false;
    if (micHandle->inputQueue)
        AudioQueueStop(micHandle->inputQueue, true);
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

void Moss_MicrophoneSetGain(Moss_Microphone* micHandle, float gain) { if (micHandle) micHandle->micGain = std::max(0.0f, gain); }
uint32_t Moss_MicrophoneGetSampleRate(Moss_Microphone* micHandle) { return micHandle ? micHandle->sampleRate : 0; }
uint32_t Moss_MicrophoneGetChannels(Moss_Microphone* micHandle) { return micHandle ? micHandle->channels : 0; }
void Moss_MicrophoneSetCallback(Moss_Microphone* micHandle, MicrophoneCallback callback, void* userData) { if (micHandle) { micHandle->micCallback = callback; micHandle->micUserData = userData; } }
Moss_MicrophoneLevels Moss_MicrophoneGetLevels(Moss_Microphone* micHandle) { return micHandle ? micHandle->levels : Moss_MicrophoneLevels{}; }
float Moss_MicrophoneGetLevelRMS(Moss_Microphone* micHandle) { return Moss_MicrophoneGetLevels(micHandle).rms; }
float Moss_MicrophoneGetLevelPeak(Moss_Microphone* micHandle) { return Moss_MicrophoneGetLevels(micHandle).peak; }
float Moss_MicrophoneGetSmoothedVolume(Moss_Microphone* micHandle) { return Moss_MicrophoneGetLevels(micHandle).smoothed_volume; }
float Moss_MicrophoneGetVoiceActivity(Moss_Microphone* micHandle) { return Moss_MicrophoneGetLevels(micHandle).voice_activity; }

bool Moss_IsMicrophoneDeviceReady() { return Moss_MicrophoneGetDeviceCount() > 0; }
int Moss_AudioMicrophoneOpen() { Moss_MicrophoneDesc desc{}; desc.start_immediately = false; g_legacyMicrophone = Moss_MicrophoneOpen(&desc); return g_legacyMicrophone ? 1 : 0; }
void Moss_AudioMicrophoneClose() { Moss_MicrophoneClose(g_legacyMicrophone); g_legacyMicrophone = nullptr; }
void Moss_AudioMicrophonePlay() { Moss_MicrophoneStart(g_legacyMicrophone); }
void Moss_AudioMicrophoneStop() { Moss_MicrophoneStop(g_legacyMicrophone); }
int Moss_AudioMicrophoneID() { return 0; }
bool Moss_AudioSelectMicrophoneDevice(int id) { return id == 0; }
const char* Moss_GetMicrophoneDeviceName(int index) { return Moss_MicrophoneGetDeviceName(static_cast<uint32_t>(index)); }
int Moss_ListMicrophoneDevices() { return static_cast<int>(Moss_MicrophoneGetDeviceCount()); }
void Moss_AudioMicrophoneSetGain(Microphone*, float gain) { Moss_MicrophoneSetGain(g_legacyMicrophone, gain); }
int Moss_AudioMicrophoneGetSampleRate(Microphone*) { return static_cast<int>(Moss_MicrophoneGetSampleRate(g_legacyMicrophone)); }
int Moss_AudioMicrophoneGetChannels(Microphone*) { return static_cast<int>(Moss_MicrophoneGetChannels(g_legacyMicrophone)); }
void Moss_AudioMicrophoneSetCallback(Microphone*, MicrophoneCallback callback, void* userData) { Moss_MicrophoneSetCallback(g_legacyMicrophone, callback, userData); }
static uint32_t Moss_AudioSourceReadMossMicrophone(Moss_AudioSource* src, float* out_samples, uint32_t frames) {
    if (!src || !src->userdata || !out_samples || frames == 0)
        return 0;
    return Moss_MicrophoneRead(static_cast<Moss_Microphone*>(src->userdata), out_samples, frames);
}

static void Moss_AudioSourceDestroyMossMicrophone(Moss_AudioSource* src) {
    std::free(src);
}

Moss_AudioSource* Moss_AudioCaptureMossMicrophone(Moss_Microphone* sourceMic) {
    if (!sourceMic)
        return nullptr;
    Moss_AudioSource* source = static_cast<Moss_AudioSource*>(std::calloc(1, sizeof(Moss_AudioSource)));
    if (!source)
        return nullptr;
    source->type = MOSS_AUDIO_SOURCE_MICROPHONE;
    source->userdata = sourceMic;
    source->read = Moss_AudioSourceReadMossMicrophone;
    source->destroy = Moss_AudioSourceDestroyMossMicrophone;
    return source;
}

Moss_AudioSource* Moss_AudioCaptureMicrophone(Microphone*) {
    return Moss_AudioCaptureMossMicrophone(g_legacyMicrophone);
}
*/
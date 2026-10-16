// alsa_microphone.cpp
#include "alsa_audio.h"
 
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/soundcard.h>
 
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <string>
#include <mutex>
#include <thread>
#include <cerrno>
 
 
Moss_Microphone mic;
static Moss_Microphone* g_legacyMicrophone = nullptr;
 
// -----------------------------
// Internal Helper Devices Enumeration
// -----------------------------
static void Moss_ResetMicrophoneBuffer(Moss_Microphone* micHandle, uint32_t ringFrames) {
    if (!micHandle) return;
 
    std::lock_guard<std::mutex> lock(micHandle->bufferMutex);
    micHandle->ringFrameCapacity = std::max<uint32_t>(ringFrames, 1);
    micHandle->ringBuffer.assign(micHandle->ringFrameCapacity * micHandle->channels, 0.0f);
    micHandle->readFrame = 0;
    micHandle->writeFrame = 0;
    micHandle->availableFrames = 0;
}
 
static void Moss_UpdateMicrophoneLevels(Moss_Microphone* micHandle, const float* samples, uint32_t sampleCount) {
    if (!micHandle || !micHandle->voiceMetricsEnabled || !samples || sampleCount == 0) return;
 
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
    if (!micHandle || !frames || frameCount == 0 || micHandle->ringFrameCapacity == 0 || micHandle->channels == 0) return;
 
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
 
static void Moss_EnumerateMicrophonesInto(Moss_Microphone* micHandle) {
    if (!micHandle) return;
 
    micHandle->microphoneNames.clear();
    int card = -1;
    while (snd_card_next(&card) >= 0 && card >= 0) {
        snd_ctl_t* handle = nullptr;
        snd_ctl_card_info_t* info = nullptr;
        snd_ctl_card_info_alloca(&info);
        
        std::string card_name = "hw:" + std::to_string(card);
        if (snd_ctl_open(&handle, card_name.c_str(), 0) < 0) continue;
        if (snd_ctl_card_info(handle, info) >= 0) {
            const char* name = snd_ctl_card_info_get_name(info);
            if (name) micHandle->microphoneNames.emplace_back(name);
        }
        snd_ctl_close(handle);
    }
 
    if (micHandle->microphoneNames.empty()) { micHandle->microphoneNames.emplace_back("default"); }
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
        snd_ctl_t* handle = nullptr;
        snd_ctl_card_info_t* info = nullptr;
        snd_ctl_card_info_alloca(&info);
        
        std::string card_name = "hw:" + std::to_string(card);
        if (snd_ctl_open(&handle, card_name.c_str(), 0) < 0) continue;
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
 
const char* Moss_MicrophoneGetDeviceName(uint32_t device_index) {
    Moss_EnumerateMicrophonesInto(&mic);
    if (device_index >= mic.microphoneNames.size()) { return nullptr; }
    return mic.microphoneNames[device_index].c_str();
}
 
Moss_Microphone* Moss_MicrophoneOpen(const Moss_MicrophoneDesc* desc) {
    Moss_MicrophoneDesc defaultDesc{};
    if (!desc) desc = &defaultDesc;
 
    Moss_Microphone* micHandle = new Moss_Microphone();
    micHandle->sampleRate = desc->sample_rate ? desc->sample_rate : 48000;
    micHandle->channels = desc->channels ? desc->channels : 1;
    micHandle->bufferFrames = desc->buffer_frames ? desc->buffer_frames : 480;
    micHandle->voiceMetricsEnabled = desc->enable_voice_metrics;
    Moss_EnumerateMicrophonesInto(micHandle);
    Moss_ResetMicrophoneBuffer(micHandle, desc->ring_buffer_frames ? desc->ring_buffer_frames : micHandle->sampleRate);
 
    const char* deviceName = "default";
    int err = snd_pcm_open(&micHandle->captureHandle, deviceName, SND_PCM_STREAM_CAPTURE, 0);
    if (err < 0) {
        delete micHandle;
        return nullptr;
    }
 
    snd_pcm_hw_params_t* params = nullptr;
    snd_pcm_hw_params_alloca(&params);
    snd_pcm_hw_params_any(micHandle->captureHandle, params);
    snd_pcm_hw_params_set_access(micHandle->captureHandle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(micHandle->captureHandle, params, SND_PCM_FORMAT_FLOAT_LE);
    snd_pcm_hw_params_set_channels(micHandle->captureHandle, params, micHandle->channels);
    unsigned int rate = micHandle->sampleRate;
    snd_pcm_hw_params_set_rate_near(micHandle->captureHandle, params, &rate, nullptr);
    micHandle->sampleRate = rate;
    snd_pcm_uframes_t periodFrames = static_cast<snd_pcm_uframes_t>(micHandle->bufferFrames);
    snd_pcm_hw_params_set_period_size_near(micHandle->captureHandle, params, &periodFrames, nullptr);
    micHandle->bufferFrames = static_cast<uint32_t>(periodFrames);
 
    if (snd_pcm_hw_params(micHandle->captureHandle, params) < 0) {
        snd_pcm_close(micHandle->captureHandle);
        delete micHandle;
        return nullptr;
    }
 
    if (desc->start_immediately && !Moss_MicrophoneStart(micHandle)) { Moss_MicrophoneClose(micHandle); return nullptr; }
 
    return micHandle;
}
 
void Moss_MicrophoneClose(Moss_Microphone* micHandle) {
    if (!micHandle) return;
    Moss_MicrophoneStop(micHandle);
    if (micHandle->captureHandle) { snd_pcm_close(micHandle->captureHandle); micHandle->captureHandle = nullptr; }
    if (micHandle != &mic) delete micHandle;
}
 
void Moss_MicrophoneStop(Moss_Microphone* micHandle) {
    if (!micHandle || !micHandle->capturing) return;
    micHandle->capturing = false;
    if (micHandle->captureHandle) snd_pcm_drop(micHandle->captureHandle);
    if (micHandle->captureThread.joinable()) micHandle->captureThread.join();
}
 
bool Moss_MicrophoneStart(Moss_Microphone* micHandle) {
    if (!micHandle || !micHandle->captureHandle || micHandle->capturing) return false;
 
    micHandle->capturing = true;
    snd_pcm_prepare(micHandle->captureHandle);
    micHandle->captureThread = std::thread([micHandle] {
        std::vector<float> buffer(micHandle->bufferFrames * micHandle->channels, 0.0f);
        while (micHandle->capturing) {
            snd_pcm_sframes_t frames = snd_pcm_readi(micHandle->captureHandle, buffer.data(), micHandle->bufferFrames);
            if (frames == -EPIPE) { snd_pcm_prepare(micHandle->captureHandle); continue; }
            if (frames < 0) { snd_pcm_prepare(micHandle->captureHandle); continue; }
            if (frames == 0) { continue; }
 
            const uint32_t frameCount = static_cast<uint32_t>(frames);
            const uint32_t sampleCount = frameCount * micHandle->channels;
            for (uint32_t i = 0; i < sampleCount; ++i)
                buffer[i] = std::clamp(buffer[i] * micHandle->micGain, -1.0f, 1.0f);
 
            Moss_UpdateMicrophoneLevels(micHandle, buffer.data(), sampleCount);
            Moss_PushMicrophoneFrames(micHandle, buffer.data(), frameCount);
            if (micHandle->micCallback)
                micHandle->micCallback(buffer.data(), static_cast<int>(frameCount), micHandle->micUserData);
        }
    });
 
    return true;
}
 
 
uint32_t Moss_MicrophoneRead(Moss_Microphone* micHandle, float* out_samples, uint32_t max_frames) {
    if (!micHandle || !out_samples || max_frames == 0 || micHandle->channels == 0) { return 0; }
 
    std::lock_guard<std::mutex> lock(micHandle->bufferMutex);
    const uint32_t framesToRead = std::min(max_frames, micHandle->availableFrames);
    for (uint32_t frame = 0; frame < framesToRead; ++frame) {
        const uint32_t readOffset = micHandle->readFrame * micHandle->channels;
        const uint32_t outputOffset = frame * micHandle->channels;
        for (uint32_t channel = 0; channel < micHandle->channels; ++channel) { out_samples[outputOffset + channel] = micHandle->ringBuffer[readOffset + channel]; }
 
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
 
// Legacy Framework API Implementations
 
bool Moss_IsMicrophoneDeviceReady() {
    snd_pcm_t* handle = nullptr;
    const int err = snd_pcm_open(&handle, "default", SND_PCM_STREAM_CAPTURE, 0);
    if (err < 0) return false;
    snd_pcm_close(handle);
    return true;
}
 
int Moss_AudioMicrophoneOpen() { Moss_MicrophoneDesc desc{}; desc.start_immediately = false; g_legacyMicrophone = Moss_MicrophoneOpen(&desc); return g_legacyMicrophone ? 1 : 0; }
void Moss_AudioMicrophoneClose() { Moss_MicrophoneClose(g_legacyMicrophone); g_legacyMicrophone = nullptr; }
void Moss_AudioMicrophonePlay() { Moss_MicrophoneStart(g_legacyMicrophone); }
void Moss_AudioMicrophoneStop() { Moss_MicrophoneStop(g_legacyMicrophone); }
int Moss_AudioMicrophoneID() { return 0; }
bool Moss_AudioSelectMicrophoneDevice(int id) { return id == 0; }
const char* Moss_GetMicrophoneDeviceName(int index) { return Moss_MicrophoneGetDeviceName(static_cast<uint32_t>(index)); }
uint32_t Moss_MicrophoneGetDeviceCount() { Moss_EnumerateMicrophonesInto(&mic); return static_cast<uint32_t>(mic.microphoneNames.size()); }
void Moss_AudioMicrophoneSetGain(Moss_Microphone*, float gain) { Moss_MicrophoneSetGain(g_legacyMicrophone, gain); }
int Moss_AudioMicrophoneGetSampleRate(Moss_Microphone*) { return static_cast<int>(Moss_MicrophoneGetSampleRate(g_legacyMicrophone)); }
int Moss_AudioMicrophoneGetChannels(Moss_Microphone*) { return static_cast<int>(Moss_MicrophoneGetChannels(g_legacyMicrophone)); }
void Moss_AudioMicrophoneSetCallback(Moss_Microphone*, MicrophoneCallback callback, void* userData) { Moss_MicrophoneSetCallback(g_legacyMicrophone, callback, userData); }
 
 
 
 
 
 
static uint32_t Moss_AudioSourceReadMossMicrophone(Moss_AudioSource* src, float* out_samples, uint32_t frames) {
    if (!src || !src->userdata || !out_samples || frames == 0) { return 0; }

    return Moss_MicrophoneRead(static_cast<Moss_Microphone*>(src->userdata), out_samples, frames);
}
 
static void Moss_AudioSourceDestroyMossMicrophone(Moss_AudioSource* src) { std::free(src); }
 
Moss_AudioSource* _AudioCaptureMicrophone(Moss_Microphone* sourceMic) {
    if (!sourceMic) { return nullptr; }
    Moss_AudioSource* source = static_cast<Moss_AudioSource*>(std::calloc(1, sizeof(Moss_AudioSource)));
    if (!source) { return nullptr; }
    source->type = MOSS_AUDIO_SOURCE_MICROPHONE;
    source->userdata = sourceMic;
    source->read = Moss_AudioSourceReadMossMicrophone;
    source->destroy = Moss_AudioSourceDestroyMossMicrophone;
    return source;
}
 
Moss_AudioSource* Moss_AudioCaptureMicrophone(Moss_Microphone*) { return _AudioCaptureMicrophone(g_legacyMicrophone);
}
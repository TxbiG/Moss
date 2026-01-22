#ifndef MOSS_COCOA_AUDIO_H
#define MOSS_COCOA_AUDIO_H

#include <Moss/Audio/coreaudio/coreaudio_audio.h>


double GetDefaultOutputDeviceSampleRate() {
    AudioDeviceID deviceID = 0;
    UInt32 size = sizeof(deviceID);
    AudioObjectPropertyAddress addr = { kAudioHardwarePropertyDefaultOutputDevice, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMaster };

    OSStatus status = AudioObjectGetPropertyData(kAudioObjectSystemObject, &addr, 0, nullptr, &size, &deviceID);
    if (status != noErr) return 44100.0; // fallback

    Float64 sampleRate = 0;
    size = sizeof(sampleRate);
    addr.mSelector = kAudioDevicePropertyNominalSampleRate;
    addr.mScope = kAudioObjectPropertyScopeGlobal;

    status = AudioObjectGetPropertyData(deviceID, &addr, 0, nullptr, &size, &sampleRate);
    if (status != noErr) return 44100.0; // fallback

    return static_cast<double>(sampleRate);
}

UInt32 GetDefaultOutputDeviceChannelCount() {
    AudioDeviceID deviceID = 0;
    UInt32 size = sizeof(deviceID);
    AudioObjectPropertyAddress addr = { kAudioHardwarePropertyDefaultOutputDevice, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMaster };

    OSStatus status = AudioObjectGetPropertyData(kAudioObjectSystemObject, &addr, 0, nullptr, &size, &deviceID);
    if (status != noErr) return 2; // fallback

    UInt32 channels = 0;
    size = sizeof(channels);
    addr.mSelector = kAudioDevicePropertyStreamConfiguration;
    addr.mScope = kAudioObjectPropertyScopeOutput;

    AudioBufferList* bufferList = nullptr;
    status = AudioObjectGetPropertyDataSize(deviceID, &addr, 0, nullptr, &size);
    if (status != noErr) return 2;

    bufferList = (AudioBufferList*)malloc(size);
    if (!bufferList) return 2;

    status = AudioObjectGetPropertyData(deviceID, &addr, 0, nullptr, &size, bufferList);
    if (status != noErr) {
        free(bufferList);
        return 2;
    }

    UInt32 channelCount = 0;
    for (UInt32 i = 0; i < bufferList->mNumberBuffers; ++i) {
        channelCount += bufferList->mBuffers[i].mNumberChannels;
    }

    free(bufferList);
    return channelCount;
}

OSStatus AudioRenderCallback(void* inRefCon, AudioUnitRenderActionFlags* ioActionFlags, const AudioTimeStamp* inTimeStamp, UInt32 inBusNumber, UInt32 inNumberFrames, AudioBufferList* ioData) {
    AudioStream* stream = static_cast<AudioStream*>(inRefCon); // Use your custom class or context

    if (!stream || !stream->playing) {
        // Fill silence
        for (UInt32 i = 0; i < ioData->mNumberBuffers; ++i)
            std::memset(ioData->mBuffers[i].mData, 0, ioData->mBuffers[i].mDataByteSize);
        return noErr;
    }

    // Assume 32-bit float, stereo, non-interleaved
    float* left = static_cast<float*>(ioData->mBuffers[0].mData);
    float* right = static_cast<float*>(ioData->mBuffers[1].mData);

    // Fill `inNumberFrames` frames from the Wav's PCM data
    for (UInt32 i = 0; i < inNumberFrames; ++i) {
        float sample = stream->getNextSample(); // You write this based on your PCM logic
        left[i] = sample;
        right[i] = sample;
    }

    return noErr;
}

int Moss_Init_Audio() {
    AudioComponentDescription desc = {};
    desc.componentType = kAudioUnitType_Output;
    desc.componentSubType = kAudioUnitSubType_DefaultOutput; // Use HAL for lower-level access
    desc.componentManufacturer = kAudioUnitManufacturer_Apple;

    AudioComponent defaultOutput = AudioComponentFindNext(nullptr, &desc);
    if (!defaultOutput) { fprintf(stderr, "Failed to find default output AudioComponent.\n"); return 0; }

    OSStatus status = AudioComponentInstanceNew(defaultOutput, &outputUnit);
    if (status != noErr || outputUnit == nullptr) { fprintf(stderr, "Failed to create AudioUnit instance.\n"); return 0; }

    double sampleRate = GetDefaultOutputDeviceSampleRate();
    UInt32 channels = GetDefaultOutputDeviceChannelCount();

    AudioStreamBasicDescription format = {};
    format.mSampleRate = sampleRate;
    format.mFormatID = kAudioFormatLinearPCM;
    format.mFormatFlags = kAudioFormatFlagsNativeFloatPacked | kAudioFormatFlagIsNonInterleaved;
    format.mFramesPerPacket = 1;
    format.mChannelsPerFrame = channels;
    format.mBitsPerChannel = 32;
    format.mBytesPerFrame = format.mChannelsPerFrame * sizeof(float);
    format.mBytesPerPacket = format.mBytesPerFrame * format.mFramesPerPacket;

    OSStatus status = AudioUnitSetProperty(outputUnit, kAudioUnitProperty_StreamFormat, kAudioUnitScope_Input, 0, &format, sizeof(format));
    if (status != noErr) { fprintf(stderr, "Failed to set stream format.\n"); }

    // 2. Set the render callback property
    AURenderCallbackStruct callbackStruct = {};
    callbackStruct.inputProc = AudioRenderCallback;
    callbackStruct.inputProcRefCon = nullptr;  // Or your audio context pointer

    status = AudioUnitSetProperty(outputUnit, kAudioUnitProperty_SetRenderCallback, kAudioUnitScope_Input, 0, &callbackStruct, sizeof(callbackStruct));
    if (status != noErr) { fprintf(stderr, "Failed to set render callback.\n"); }

    status = AudioUnitInitialize(outputUnit);
    if (status != noErr) { fprintf(stderr, "Failed to initialize AudioUnit.\n"); return 0; }

    AudioOutputUnitStart(outputUnit);

    return 1;
}

void Moss_Terminate_Audio() { if (outputUnit) { AudioOutputUnitStop(outputUnit); AudioUnitUninitialize(outputUnit); AudioComponentInstanceDispose(outputUnit); outputUnit = nullptr; } }

size_t streamPCM(float* out, size_t frames, int numChannels) {
    if (!playing || !wav.dataBegin) return 0;

    size_t bytesPerSample = wav.bitsPerSample / 8;
    size_t samplesToRead = frames * wav.numChannels;
    size_t bytesAvailable = wav.dataChunkSize - positionInBytes;
    size_t samplesAvailable = bytesAvailable / bytesPerSample;
    size_t samplesToCopy = std::min(samplesToRead, samplesAvailable);

    const char* data = wav.dataBegin + positionInBytes;

    for (size_t i = 0; i < samplesToCopy; ++i) {
        int16_t sample = *reinterpret_cast<const int16_t*>(data + i * bytesPerSample); // assuming S16_LE
        float fsample = (sample / 32768.0f) * volume;

        // Apply pan (simple L/R balance)
        if (numChannels == 2) {
            float left = fsample * (1.0f - pan);
            float right = fsample * (1.0f + pan);
            out[i * 2 + 0] += left * 0.5f;
            out[i * 2 + 1] += right * 0.5f;
        } else {
            out[i] += fsample;
        }
    }

    positionInBytes += samplesToCopy * bytesPerSample;

    if (positionInBytes >= wav.dataChunkSize) {
        if (loop) {
            positionInBytes = 0;
        } else {
            stop();
        }
    }

    return samplesToCopy;
}


void AudioStream::play() {
    std::lock_guard<std::mutex> lock(audioMutex);
    if (!playing) {
        playing = true;
        activeStreams.push_back(this);
    }
}

void AudioStream::stop() {
    std::lock_guard<std::mutex> lock(audioMutex);
    playing = false;
    activeStreams.erase(std::remove(activeStreams.begin(), activeStreams.end(), this), activeStreams.end());
}






static MicrophoneCallback = nullptr;
static AudioStreamCallback = nullptr;



int Moss_Init_Audio();

void Moss_Terminate_Audio();

void Moss_AudioUpdate(float deltaTime);


Moss_AudioSource* Moss_AudioLoadWav();

Moss_AudioSource* Moss_AudioLoadOgg(const char* filename, AudioLoadType type);

Moss_AudioSource* Moss_AudioLoadMP3(const char* filename);

Moss_AudioSource* Moss_AudioCaptureMicrophone(Microphone* mic);

// Effects

AudioEffect Moss_AudioCreateEffect(AudioEffectType type);

void Moss_AudioCreateEffect(AudioEffect* effect, const char* paramName, float value);

void Moss_AudioRemoveEffect(AudioEffect* effect);

// Channel

ChannelID Moss_AudioCreateChannel(ChannelID channel);
void Audio_RemoveChannel(ChannelID channel);

ChannelID Moss_AudioGetMasterChannel();

void Moss_AudioSetChannelVolume(ChannelID channel, float volume);

void Moss_AudioSetChannelMute(ChannelID channel, bool mute);

void Moss_AudioAddChannelEffect(ChannelID channel, AudioEffect* effect);

void Moss_AudioRemoveChannelEffect(ChannelID channel, AudioEffect* effect);

void Moss_AudioRemoveAllChannelEffects(ChannelID channel);


AudioStream* Moss_AudioStreamCreate();

void Moss_AudioStreamPlay(AudioStream* audiostream);

void Moss_AudioStreamStop(AudioStream* audiostream);

void Moss_AudioStreamSetVolume(AudioStream* audiostream, float volume);

void Moss_AudioStreamSetPitch(AudioStream* audiostream, float pitch);

void Moss_AudioStreamSetLoop(AudioStream* audiostream, bool loop);

void Moss_AudioStreamRemove(AudioStream* audiostream);


AudioStream2D* Moss_AudioStream2DCreate();

void Moss_AudioStream2DPlay(AudioStream2D* audiostream);

void Moss_AudioStream2DStop(AudioStream2D* audiostream);

void Moss_AudioStream2DSetVolume(AudioStream2D* audiostream, float volume);

void Moss_AudioStream2DSetPitch(AudioStream2D* audiostream, float pitch);

void Moss_AudioStream2DSetLoop(AudioStream2D* audiostream, bool loop);

void Moss_AudioStream3DSetPosition(AudioStream2D* audiostream);

void Moss_AudioStream3DSetVelocity(AudioStream2D* audiostream);

void Moss_AudioStream3DSetMaxDistance(AudioStream2D* audiostream);

void Moss_AudioStream2DRemove(AudioStream2D* audiostream);


AudioStream3D* Moss_AudioStream3DCreate();

void Moss_AudioStream3DPlay(AudioStream3D* audiostream);

void Moss_AudioStream3DStop(AudioStream3D* audiostream);

void Moss_AudioStream3DSetVolume(AudioStream3D* audiostream, float volume);

void Moss_AudioStream3DSetPitch(AudioStream3D* audiostream, float pitch);

void Moss_AudioStream3DSetLoop(AudioStream3D* audiostream, bool loop);

void Moss_AudioStream3DSetPosition(AudioStream3D* audiostream);

void Moss_AudioStream3DSetVelocity(AudioStream3D* audiostream);

void Moss_AudioStream3DSetMaxDistance(AudioStream3D* audiostream);

void Moss_AudioStream3DSetDistanceModel(AudioStream3D* stream, DistanceModel model);

void Moss_AudioStream3DRemove(AudioStream3D* audiostream);

// Listeners

AudioListener2D* Moss_AudioCreateAudioListener2D();

AudioListener3D* Moss_AudioCreateAudioListener3D();

RayAudioListener2D* Moss_AudioCreateRayAudioListener2D();

RayAudioListener3D* Moss_AudioCreateRayAudioListener3D();


void Moss_AudioRemoveAudioListener2D(AudioListener2D* listener);

void Moss_AudioRemoveAudioListener3D(AudioListener3D* listener);

void Moss_AudioRemoveRayAudioListener2D(RayAudioListener2D* listener);

void Moss_AudioRemoveRayAudioListener3D(RayAudioListener3D* listener);


void Moss_AudioActivateAudioListener2D(AudioListener2D* listener, bool activate);

void Moss_AudioActivateAudioListener3D(AudioListener3D* listener, bool activate);

void Moss_AudioActivateRayAudioListener2D(RayAudioListener2D* listener, bool activate);

void Moss_AudioActivateRayAudioListener3D(RayAudioListener3D* listener, bool activate);

void Moss_AudioListenerSetOrientation(AudioListener3D* listener, const Vec3& forward, const Vec3& up);

// Speakers

bool Moss_IsSpeakerDeviceReady();

void Moss_AudioSpeakerOpen();

void Moss_AudioSpeakerPause();

void Moss_AudioSpeakerResume();

bool Moss_AudioSpeakerIsPaused();
/*! @brief X. @param X X. */
bool Moss_AudioSelectSpeakerDevice(int id);

int Moss_GetCurrentSpeakerDeviceID();

const char* Moss_GetSpeakerDeviceName(int id);

int Moss_ListSpeakerDevices();

// Microphone

bool Moss_IsMicrophoneDeviceReady();

int Moss_AudioMicrophoneOpen();

void Moss_AudioMicrophoneClose();

void Moss_AudioMicrophonePlay();

void Moss_AudioMicrophoneStop();

int Moss_AudioMicrophoneID();

bool Moss_AudioSelectMicrophoneDevice(int id);

const char* Moss_GetMicrophoneDeviceName(int index);

int Moss_ListMicrophoneDevices();


void Moss_AudioMicrophoneSetGain(Microphone* mic, float gain);

int Moss_AudioMicrophoneGetSampleRate(Microphone* mic);

int Moss_AudioMicrophoneGetChannels(Microphone* mic);


void Moss_AudioStreamSetCallback(AudioStream* stream, AudioStreamCallback callback, void* userData);

void Moss_AudioMicrophoneSetCallback(Microphone* mic, MicrophoneCallback callback, void* userData);

#endif // MOSS_COCOA_AUDIO_H
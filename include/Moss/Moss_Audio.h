//                        MIT License
//
//                  Copyright (c) 2026 Toby
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/*!
 * @file Moss_Audio.h
 * @brief Cross-platform audio subsystem for the Moss Framework.
 *
 * The audio module provides a unified, high-performance abstraction over
 * multiple platform-specific backends. It is designed to support both
 * **real-time 3D spatial audio** and **2D streamed playback** for games,
 * XR experiences, and simulation systems.
 *
 * ---
 *
 * ### Supported Backends:
 * - **Windows:** XAudio2 / WASAPI
 * - **macOS:** CoreAudio *(under testing)*
 * - **Linux:** ALSA *(under testing)*
 *
 * ---
 *
 * ### Core Capabilities:
 * - **2D / 3D Audio Streams** — Play, pause, and stop streamed or preloaded sounds.
 * - **Audio Effects Pipeline** — Supports a broad range of DSP effects:
 *   - Lowpass / Highpass filters
 *   - Echo, Flange, Distortion
 *   - Normalize, Parametric EQ
 *   - Pitch Shifting, Chorus, Compressor
 *   - Reverb and Delay
 * - **Dynamic Audio Control** — Real-time adjustment of pitch, gain, and pan per stream or channel.
 * - **Hardware Abstraction** — Unified interface for **Speakers** and **Microphones**, with support for enumeration, selection, and control.
 * - **Audio Listeners** — Spatial representation for 2D and 3D listener transforms, integrated with camera and XR systems.
 * - **Ray-Traced Audio Listeners** - CPU tracing for real-time occlusion and first-order reflections.
 * - **Wav Files** - .Wav files are supported
 *
 * ---
 *
 * ### Design Goals:
 * - Low-latency, high-fidelity cross-platform audio.
 * - Thread-safe mixing and real-time streaming.
 * - Integration with Moss Engine’s physics and rendering systems for synchronized A/V effects.
 * - Modular extension for third-party DSPs and audio middleware.
 */



/*
Audio thread separation
Lock-free ring buffers
Explicit update tick
Deterministic mixing order
HRTF plugin interface
Edit the audio raw
 */
#ifndef MOSS_AUDIO_H
#define MOSS_AUDIO_H

#include <Moss/Moss_stdinc.h>
//#include <Moss/Moss_Physics.h>
#include <Moss/Variants/Vector/Vec2.h>
#include <Moss/Variants/Vector/Vec3.h>

// AudioStream player set as signal for calling
#define CHANNEL_INVALID 0U

// Forward declarations
class PhysicsSystem;
struct AudioEffect;
struct AudioStream;
struct AudioStream2D;
struct AudioStream3D;
struct AudioListener2D;
struct AudioListener3D;
struct RayAudioListener2D;
struct RayAudioListener3D;
struct Moss_AudioSource;
struct Moss_Microphone;
struct Wav;

using ChannelID = uint32_t;

enum class AudioEffectType {
    LOWPASS                     = 0, 
    HIGHPASS                    = 1 << 0, 
    ECHO                        = 1 << 1, 
    FLANGE                      = 1 << 2, 
    DISTORTION                  = 1 << 3,
    NORMALIZE                   = 1 << 4, 
    PARAMEQ                     = 1 << 5, 
    PITCHSHIFTER                = 1 << 6, 
    CHORUS                      = 1 << 7,
    COMPRESSOR                  = 1 << 8, 
    REVERB                      = 1 << 9, 
    DELAY                       = 1 << 10,
    DOPPLER                     = 1 << 11, 
    PANNING                     = 1 << 12, 
    DISTANCE_ATTENUATION        = 1 << 13
};

enum class DistanceModel { 
    LINEAR, 
    INVERSE, 
    EXPONENTIAL 
};

enum class AudioLoadType { 
    FULLY_LOADED, 
    STREAMING 
};
struct Moss_AudioDecodedData {
    float* samples = nullptr;
    uint32_t frame_count = 0;
    uint32_t channels = 0;
    uint32_t sample_rate = 0;
    void (*free_samples)(void* userdata, float* samples) = nullptr;
    void* userdata = nullptr;
};
typedef bool (*Moss_AudioDecodeCallback)(const char* filename, AudioLoadType type, Moss_AudioDecodedData* out_data, void* userdata);

struct Moss_MicrophoneDesc {
    uint32_t device_index = 0;
    uint32_t sample_rate = 48000;
    uint32_t channels = 1;
    uint32_t buffer_frames = 480;
    uint32_t ring_buffer_frames = 48000;
    bool start_immediately = true;
    bool enable_voice_metrics = true;
};

struct Moss_MicrophoneLevels {
    float rms = 0.0f;
    float peak = 0.0f;
    float smoothed_volume = 0.0f;
    float voice_activity = 0.0f;
};

struct Moss_AudioRayHit2D {
    bool hit = false;
    float fraction = 1.0f;
    Vec2 position = Vec2::Zero();
    Vec2 normal = Vec2::Zero();
    float absorption = 0.5f;
    float transmission = 0.25f;
    uint32_t material_id = 0;
};

struct Moss_AudioRayHit3D {
    bool hit = false;
    float fraction = 1.0f;
    Vec3 position = Vec3::sZero();
    Vec3 normal = Vec3::sZero();
    float absorption = 0.5f;
    float transmission = 0.25f;
    uint32_t material_id = 0;
};

struct Moss_AudioRayTrace2DDesc {
    PhysicsSystem* physics = nullptr;
    Moss_AudioRaycast2DCallback raycast = nullptr;
    void* user_data = nullptr;
    Vec2 listener_position = Vec2::Zero();
    Vec2 source_position = Vec2::Zero();
    float max_distance = 100.0f;
    uint32_t reflection_rays = 0;
    float direct_occlusion_strength = 1.0f;
    float reflection_strength = 0.35f;
    float air_absorption = 0.02f;
};

struct Moss_AudioRayTrace3DDesc {
    PhysicsSystem* physics = nullptr;
    Moss_AudioRaycast3DCallback raycast = nullptr;
    void* user_data = nullptr;
    Vec3 listener_position = Vec3::sZero();
    Vec3 source_position = Vec3::sZero();
    float max_distance = 100.0f;
    uint32_t reflection_rays = 0;
    float direct_occlusion_strength = 1.0f;
    float reflection_strength = 0.35f;
    float air_absorption = 0.02f;
};

struct Moss_AudioRayTraceResult {
    bool audible = false;
    bool occluded = false;
    float distance = 0.0f;
    float attenuation = 1.0f;
    float occlusion = 0.0f;
    float transmission_gain = 1.0f;
    float lowpass = 1.0f;
    float reflection_gain = 0.0f;
    float reflection_delay_seconds = 0.0f;
    float delay_seconds = 0.0f;
};

typedef void (*MicrophoneCallback)(const float* buffer, int samples, void* userData);
typedef void (*AudioStreamCallback)(float* buffer, int frames, void* userData);

typedef bool (*Moss_AudioRaycast2DCallback)(PhysicsSystem* physics, const Vec2* origin, const Vec2* direction, float distance, Moss_AudioRayHit2D* out_hit, void* user_data);
typedef bool (*Moss_AudioRaycast3DCallback)(PhysicsSystem* physics, const Vec3* origin, const Vec3* direction, float distance, Moss_AudioRayHit3D* out_hit, void* user_data);

/*!  @brief Initialize the Moss Audio system. Must be called before using any audio functionality. Initializes the audio backend, mixer, and device interfaces. @return 0 on success, non-zero on failure. */
MOSS_API int Moss_Init_Audio();
/*! @brief Shut down the Moss Audio system. Stops all playback, releases devices, and frees internal resources. No audio functions may be called after this. */
MOSS_API void Moss_Terminate_Audio();
/*! @brief Update the audio system.  Must be called once per frame. Handles spatialization, streaming, callbacks, and effect updates. @param deltaTime Time elapsed since last update (seconds). */
MOSS_API void Moss_AudioUpdate(float deltaTime);
/*! @brief Load a WAV audio source from a file path. @param filename Path to the WAV file. @return Pointer to a loaded audio source, or null on failure. */
MOSS_API Moss_AudioSource* Moss_AudioLoadWavFile(const char* filename);
/*! @brief Compatibility wrapper for older code. Prefer Moss_AudioLoadWavFile. @return Always null because no file path is supplied. */
MOSS_API Moss_AudioSource* Moss_AudioLoadWav();
/*! @brief Load an OGG audio source. @param filename Path to the OGG file. @param type Load type (fully loaded or streaming). @return Pointer to a loaded audio source. */
MOSS_API Moss_AudioSource* Moss_AudioLoadOgg(const char* filename, AudioLoadType type);
/*! @brief Load an MP3 audio source. @param filename Path to the MP3 file. @return Pointer to a loaded audio source. */
MOSS_API Moss_AudioSource* Moss_AudioLoadMP3(const char* filename);
/*! @brief Register a decoder for encoded audio formats such as OGG/Vorbis or MP3. @param format Encoded format handled by the callback. @param callback Decode callback. @param userdata User pointer passed to the callback. @return true when registered. */
//MOSS_API bool Moss_AudioRegisterDecoder(Moss_AudioEncodedFormat format, Moss_AudioDecodeCallback callback, void* userdata);
/*! @brief Destroy an audio source created by Moss_AudioLoad* or Moss_AudioCaptureMicrophone. @param source Audio source to release. */
MOSS_API void Moss_AudioSourceDestroy(Moss_AudioSource* source);
/*! @brief Create an audio source from a microphone device. @param mic Microphone device handle. @return Pointer to an audio source capturing microphone input. */
MOSS_API Moss_AudioSource* Moss_AudioCaptureMicrophone(Moss_Microphone* mic);

/*! @brief Create an audio effect. @param type Type of effect to create. @return Created audio effect. */
MOSS_API AudioEffect Moss_AudioCreateEffect(AudioEffectType type);
/*! @brief Set a parameter on an audio effect. @param effect Effect to modify. @param paramName Name of the parameter. @param value Parameter value. */
MOSS_API void Moss_AudioCreateEffect(AudioEffect* effect, const char* paramName, float value);
/*! @brief Remove and destroy an audio effect. @param effect Effect to remove. */
MOSS_API void Moss_AudioRemoveEffect(AudioEffect* effect);

// Channel
/*! @brief Create or retrieve an audio channel. @param channel Channel ID. @return Channel ID. */
MOSS_API ChannelID Moss_AudioCreateChannel(ChannelID channel);
/*! @brief Remove an audio channel. @param channel Channel ID. */
MOSS_API void Audio_RemoveChannel(ChannelID channel);
/*! @brief Get the master audio channel. @return Master channel ID. */
MOSS_API ChannelID Moss_AudioGetMasterChannel();
/*! @brief Set channel volume. @param channel Channel ID. @param volume Volume (0.0 – 1.0). */
MOSS_API void Moss_AudioSetChannelVolume(ChannelID channel, float volume);
/*! @brief Mute or unmute a channel. @param channel Channel ID. @param mute True to mute. */
MOSS_API void Moss_AudioSetChannelMute(ChannelID channel, bool mute);
/*! @brief X. */
MOSS_API void Moss_AudioAddChannelEffect(ChannelID channel, AudioEffect* effect);
/*! @brief X. */
MOSS_API void Moss_AudioRemoveChannelEffect(ChannelID channel, AudioEffect* effect);
/*! @brief X. */
MOSS_API void Moss_AudioRemoveAllChannelEffects(ChannelID channel);

/*! @brief Create a non-spatial audio stream. @return Pointer to the created stream. */
MOSS_API AudioStream* Moss_AudioStreamCreate();
/*! @brief Start playback of an audio stream. @param audiostream Stream to play. */
MOSS_API void Moss_AudioStreamPlay(AudioStream* audiostream);
/*! @brief Stop playback of an audio stream. @param audiostream Stream to stop. */
MOSS_API void Moss_AudioStreamStop(AudioStream* audiostream);
/*! @brief Set stream volume. @param audiostream Stream to modify. @param volume Volume (0.0 – 1.0). */
MOSS_API void Moss_AudioStreamSetVolume(AudioStream* audiostream, float volume);
/*! @brief X. */
MOSS_API void Moss_AudioStreamSetPitch(AudioStream* audiostream, float pitch);
/*! @brief X. */
MOSS_API void Moss_AudioStreamSetPlaybackRate(AudioStream* audiostream, float rate);
/*! @brief X. */
MOSS_API void Moss_AudioStreamSetPan(AudioStream* audiostream, float pan);
/*! @brief X. */
MOSS_API void Moss_AudioStreamSetLoop(AudioStream* audiostream, bool loop);
/*! @brief X. */
MOSS_API void Moss_AudioStreamRemove(AudioStream* audiostream);

/*! @brief X. */
MOSS_API AudioStream2D* Moss_AudioStream2DCreate();
/*! @brief X. */
MOSS_API void Moss_AudioStream2DPlay(AudioStream2D* audiostream);
/*! @brief X. */
MOSS_API void Moss_AudioStream2DStop(AudioStream2D* audiostream);
/*! @brief X. */
MOSS_API void Moss_AudioStream2DSetVolume(AudioStream2D* audiostream, float volume);
/*! @brief X. */
MOSS_API void Moss_AudioStream2DSetPitch(AudioStream2D* audiostream, float pitch);
void Moss_AudioStream2DSetPlaybackRate(AudioStream2D* audiostream, float rate);
MOSS_API void Moss_AudioStream2DSetPan(AudioStream2D* audiostream, float pan);
/*! @brief X. */
MOSS_API void Moss_AudioStream2DSetLoop(AudioStream2D* audiostream, bool loop);
/*! @brief X. */
MOSS_API void Moss_AudioStream3DSetPosition(AudioStream2D* audiostream);
/*! @brief X. */
MOSS_API void Moss_AudioStream3DSetVelocity(AudioStream2D* audiostream);
/*! @brief X. */
MOSS_API void Moss_AudioStream3DSetMaxDistance(AudioStream2D* audiostream);
/*! @brief X. */
MOSS_API void Moss_AudioStream2DRemove(AudioStream2D* audiostream);

/*! @brief X. */
MOSS_API AudioStream3D* Moss_AudioStream3DCreate();
/*! @brief X. */
MOSS_API void Moss_AudioStream3DPlay(AudioStream3D* audiostream);
/*! @brief X. */
MOSS_API void Moss_AudioStream3DStop(AudioStream3D* audiostream);
/*! @brief X. */
MOSS_API void Moss_AudioStream3DSetVolume(AudioStream3D* audiostream, float volume);
/*! @brief X. */
MOSS_API void Moss_AudioStream3DSetPitch(AudioStream3D* audiostream, float pitch);
/*! @brief X. */
MOSS_API void Moss_AudioStream3DSetPlaybackRate(AudioStream3D* audiostream, float rate);
/*! @brief X. */
MOSS_API void Moss_AudioStream3DSetPan(AudioStream3D* audiostream, float pan);
/*! @brief X. */
MOSS_API void Moss_AudioStream3DSetLoop(AudioStream3D* audiostream, bool loop);
/*! @brief X. */
MOSS_API void Moss_AudioStream3DSetPosition(AudioStream3D* audiostream);
/*! @brief X. */
MOSS_API void Moss_AudioStream3DSetVelocity(AudioStream3D* audiostream);
/*! @brief X. */
MOSS_API void Moss_AudioStream3DSetMaxDistance(AudioStream3D* audiostream);
/*! @brief X. */
MOSS_API void Moss_AudioStream3DSetDistanceModel(AudioStream3D* stream, DistanceModel model);
/*! @brief X. */
MOSS_API void Moss_AudioStream3DRemove(AudioStream3D* audiostream);


/*! @brief X. */
MOSS_API AudioListener2D* Moss_AudioCreateAudioListener2D(Float2 position);
/*! @brief Create a 3D audio listener. @return Pointer to the created listener. */
MOSS_API AudioListener3D* Moss_AudioCreateAudioListener3D(Float3 position);
/*! @brief  ray-audio listener. Requires ray-audio support from the backend/application.. */
MOSS_API RayAudioListener2D* Moss_AudioCreateRayAudioListener2D(Float2 position);
/*! @brief  ray-audio listener. Requires ray-audio support from the backend/application. */
MOSS_API RayAudioListener3D* Moss_AudioCreateRayAudioListener3D(Float3 position);

/*! @brief X. */
MOSS_API void Moss_AudioRemoveAudioListener2D(AudioListener2D* listener);
/*! @brief X. */
MOSS_API void Moss_AudioRemoveAudioListener3D(AudioListener3D* listener);
/*! @brief X. */
MOSS_API void Moss_AudioRemoveRayAudioListener2D(RayAudioListener2D* listener);
/*! @brief X. */
MOSS_API void Moss_AudioRemoveRayAudioListener3D(RayAudioListener3D* listener);

/*! @brief X. */
MOSS_API void Moss_AudioActivateAudioListener2D(AudioListener2D* listener, bool activate);
/*! @brief X. */
MOSS_API void Moss_AudioActivateAudioListener3D(AudioListener3D* listener, bool activate);
/*! @brief X. */
MOSS_API void Moss_AudioActivateRayAudioListener2D(RayAudioListener2D* listener, bool activate);
/*! @brief X. */
MOSS_API void Moss_AudioActivateRayAudioListener3D(RayAudioListener3D* listener, bool activate);

/*! @brief Set listener orientation. @param listener Listener to modify. @param forward Forward direction vector. @param up Up direction vector. */
MOSS_API void Moss_AudioListenerSetOrientation(AudioListener3D* listener, const Vec3& forward, const Vec3& up);
/*! @brief X. */
MOSS_API void Moss_AudioRayListener2DSetPhysics(RayAudioListener2D* listener, PhysicsSystem* physics, Moss_AudioRaycast2DCallback raycast, void* user_data);
/*! @brief X. */
MOSS_API void Moss_AudioRayListener3DSetPhysics(RayAudioListener3D* listener, PhysicsSystem* physics, Moss_AudioRaycast3DCallback raycast, void* user_data);
/*! @brief X. */
MOSS_API void Moss_AudioRayListener2DSetTraceSettings(RayAudioListener2D* listener, float max_distance, uint32_t reflection_rays);
/*! @brief X. */
MOSS_API void Moss_AudioRayListener3DSetTraceSettings(RayAudioListener3D* listener, float max_distance, uint32_t reflection_rays);
/*! @brief X. */
MOSS_API bool Moss_AudioRayTrace2D(const Moss_AudioRayTrace2DDesc* desc, Moss_AudioRayTraceResult* out_result);
/*! @brief X. */
MOSS_API bool Moss_AudioRayTrace3D(const Moss_AudioRayTrace3DDesc* desc, Moss_AudioRayTraceResult* out_result);
/*! @brief X. */
MOSS_API bool Moss_AudioRayTraceFromListener2D(RayAudioListener2D* listener, const Vec2* source_position, Moss_AudioRayTraceResult* out_result);
/*! @brief X. */
MOSS_API bool Moss_AudioRayTraceFromListener3D(RayAudioListener3D* listener, const Vec3* source_position, Moss_AudioRayTraceResult* out_result);
/*! @brief X. */
MOSS_API float Moss_AudioRayTraceComputeGain(const Moss_AudioRayTraceResult* result);


/*! @brief Check if a speaker device is available. @return True if a device is ready. */
MOSS_API bool Moss_IsSpeakerDeviceReady();
/*! @brief Open the current speaker device. */
MOSS_API void Moss_AudioSpeakerOpen();

/*! @brief Pause audio output. */
MOSS_API void Moss_AudioSpeakerPause();
/*! @brief Resume audio output. */
MOSS_API void Moss_AudioSpeakerResume();
/*! @brief X. */
MOSS_API bool Moss_AudioSpeakerIsPaused();
/*! @brief X. @param X X. */
MOSS_API bool Moss_AudioSelectSpeakerDevice(int id);
/*! @brief X. */
MOSS_API int Moss_GetCurrentSpeakerDeviceID();
/*! @brief Get the display name for a speaker device. */
MOSS_API const char* Moss_GetSpeakerDeviceName(int id);
/*! @brief Return number of speakers. */
MOSS_API int Moss_ListSpeakerDevices();

// Microphone
/*! @brief Check if microphone is ready. */
MOSS_API uint32_t Moss_MicrophoneGetDeviceCount();
/*! @brief Check if microphone is ready. */
MOSS_API const char* Moss_MicrophoneGetDeviceName(uint32_t device_index);
/*! @brief Check if microphone is ready. */
MOSS_API Moss_Microphone* Moss_MicrophoneOpen(const Moss_MicrophoneDesc* desc);
/*! @brief Check if microphone is ready. */
MOSS_API void Moss_MicrophoneClose(Moss_Microphone* microphone);
/*! @brief Check if microphone is ready. */
MOSS_API bool Moss_MicrophoneStart(Moss_Microphone* microphone);
/*! @brief Check if microphone is ready. */
MOSS_API void Moss_MicrophoneStop(Moss_Microphone* microphone);
/*! @brief Check if microphone is ready. */
MOSS_API uint32_t Moss_MicrophoneRead(Moss_Microphone* microphone, float* out_samples, uint32_t max_frames);
/*! @brief Check if microphone is ready. */
MOSS_API void Moss_MicrophoneSetGain(Moss_Microphone* microphone, float gain);
/*! @brief Check if microphone is ready. */
MOSS_API uint32_t Moss_MicrophoneGetSampleRate(Moss_Microphone* microphone);
/*! @brief Check if microphone is ready. */
MOSS_API uint32_t Moss_MicrophoneGetChannels(Moss_Microphone* microphone);
/*! @brief Check if microphone is ready. */
MOSS_API void Moss_MicrophoneSetCallback(Moss_Microphone* microphone, MicrophoneCallback callback, void* userData);
/*! @brief Check if microphone is ready. */
MOSS_API Moss_MicrophoneLevels Moss_MicrophoneGetLevels(Moss_Microphone* microphone);
/*! @brief Check if microphone is ready. */
MOSS_API float Moss_MicrophoneGetLevelRMS(Moss_Microphone* microphone);
/*! @brief Check if microphone is ready. */
MOSS_API float Moss_MicrophoneGetLevelPeak(Moss_Microphone* microphone);
/*! @brief Check if microphone is ready. */
MOSS_API float Moss_MicrophoneGetSmoothedVolume(Moss_Microphone* microphone);
/*! @brief Check if microphone is ready. */
MOSS_API float Moss_MicrophoneGetVoiceActivity(Moss_Microphone* microphone);
/*! @brief Check if microphone is ready. */
MOSS_API bool Moss_IsMicrophoneDeviceReady();
/*! @brief Initialize microphone. */
MOSS_API int Moss_AudioMicrophoneOpen();
/*! @brief Terminate microphone. */
MOSS_API void Moss_AudioMicrophoneClose();
/*! @brief Start capture. */
MOSS_API void Moss_AudioMicrophonePlay();
/*! @brief Stop capture. */
MOSS_API void Moss_AudioMicrophoneStop();
/*! @brief Get default microphone ID. */
MOSS_API int Moss_AudioMicrophoneID();
/*! @brief X. @param X X. */
MOSS_API bool Moss_AudioSelectMicrophoneDevice(int id);
/*! @brief Get microphone name. */
MOSS_API const char* Moss_GetMicrophoneDeviceName(int index);
/*! @brief Return number of microphones. */
MOSS_API int Moss_ListMicrophoneDevices();
/*! @brief X. */
MOSS_API void Moss_AudioMicrophoneSetGain(Moss_Microphone* microphone, float gain);
/*! @brief X. */
MOSS_API int Moss_AudioMicrophoneGetSampleRate(Moss_Microphone* microphone);
/*! @brief X. */
MOSS_API int Moss_AudioMicrophoneGetChannels(Moss_Microphone* microphone);

/*! @brief Open the microphone device. @return 0 on success, non-zero on failure. */
MOSS_API void Moss_AudioStreamSetCallback(AudioStream* stream, AudioStreamCallback callback, void* userData);
/*! @brief Set a microphone capture callback. @param mic Microphone device. @param callback Callback function. @param userData User-defined pointer. */
MOSS_API void Moss_AudioMicrophoneSetCallback(Microphone* mic, MicrophoneCallback callback, void* userData);


#endif // MOSS_AUDIO_H
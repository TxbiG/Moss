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
struct Moss_AudioRayHit2D;
struct Moss_AudioRayHit3D;

using ChannelID = uint32_t;

typedef void (*MicrophoneCallback)(const float* buffer, int samples, void* userData);
typedef void (*AudioStreamCallback)(float* buffer, int frames, void* userData);

typedef bool (*Moss_AudioRaycast2DCallback)(PhysicsSystem* physics, const Vec2* origin, const Vec2* direction, float distance, Moss_AudioRayHit2D* out_hit, void* user_data);
typedef bool (*Moss_AudioRaycast3DCallback)(PhysicsSystem* physics, const Vec3* origin, const Vec3* direction, float distance, Moss_AudioRayHit3D* out_hit, void* user_data);


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
    Vec3 position = Vec3::Zero();
    Vec3 normal = Vec3::Zero();
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
    Vec3 listener_position = Vec3::Zero();
    Vec3 source_position = Vec3::Zero();
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

/**
 * @brief Initializes the Moss audio subsystem, including its backend and device interfaces.
 * @return `0` on success; a non-zero value on failure.
 */
MOSS_API int Moss_Init_Audio();
/**
 * @brief Shuts down the Moss audio subsystem and releases its resources. Stop using audio objects after termination.
 */
MOSS_API void Moss_Terminate_Audio();
/**
 * @brief Advances audio-system processing for the current frame.
 * @param deltaTime Elapsed time since the previous update, in seconds.
 */
MOSS_API void Moss_AudioUpdate(float deltaTime);
/**
 * @brief Loads a WAV file and creates an audio source for it.
 * @param filename Path to the WAV file.
 * @return Pointer to the loaded source, or `nullptr` if loading fails.
 */
MOSS_API Moss_AudioSource* Moss_AudioLoadWavFile(const char* filename);
/**
 * @brief Legacy compatibility entry point for WAV loading; it accepts no filename.
 * @return The header's existing comment indicates this returns `nullptr` because no path is provided.
 */
MOSS_API Moss_AudioSource* Moss_AudioLoadWav();
/**
 * @brief Loads an OGG audio file using the requested loading mode.
 * @param filename Path to the OGG file.
 * @param type Whether audio is fully loaded or streamed.
 * @return Pointer to the loaded source, or `nullptr` if loading fails.
 */
MOSS_API Moss_AudioSource* Moss_AudioLoadOgg(const char* filename, AudioLoadType type);
/**
 * @brief Loads an MP3 file and creates an audio source for it.
 * @param filename Path to the MP3 file.
 * @return Pointer to the loaded source, or `nullptr` if loading fails.
 */
MOSS_API Moss_AudioSource* Moss_AudioLoadMP3(const char* filename);
/*! @brief Register a decoder for encoded audio formats such as OGG/Vorbis or MP3. @param format Encoded format handled by the callback. @param callback Decode callback. @param userdata User pointer passed to the callback. @return true when registered. */
//MOSS_API bool Moss_AudioRegisterDecoder(Moss_AudioEncodedFormat format, Moss_AudioDecodeCallback callback, void* userdata);
/**
 * @brief Destroys an audio source and releases its associated resources.
 * @param source Source to destroy.
 */
MOSS_API void Moss_AudioSourceDestroy(Moss_AudioSource* source);
/**
 * @brief Creates an audio source from a microphone handle.
 * @param mic Microphone to capture from.
 * @return Pointer to the capture source, or `nullptr` if creation fails.
 */
MOSS_API Moss_AudioSource* Moss_AudioCaptureMicrophone(Moss_Microphone* mic);

/**
 * @brief Creates an audio effect of the specified type.
 * @param type Effect type to create.
 * @return The created effect value.
 */
MOSS_API AudioEffect Moss_AudioCreateEffect(AudioEffectType type);
/**
 * @brief Sets a named floating-point parameter on an existing audio effect.
 * @param effect Effect to modify.
 * @param paramName Name of the parameter to set.
 * @param value Value to assign to the parameter.
 */
MOSS_API void Moss_AudioCreateEffect(AudioEffect* effect, const char* paramName, float value);
/**
 * @brief Removes an audio effect and releases any resources it owns.
 * @param effect Effect to remove.
 */
MOSS_API void Moss_AudioRemoveEffect(AudioEffect* effect);

// Channel
/**
 * @brief Creates or retrieves an audio channel identified by the supplied ID.
 * @param channel Requested channel identifier.
 * @return Channel identifier returned by the audio system.
 */
MOSS_API ChannelID Moss_AudioCreateChannel(ChannelID channel);
/**
 * @brief Removes the specified audio channel.
 * @param channel Identifier of the channel to remove.
 */
MOSS_API void Audio_RemoveChannel(ChannelID channel);
/**
 * @brief Returns the identifier of the master audio channel.
 * @return Master channel identifier.
 */
MOSS_API ChannelID Moss_AudioGetMasterChannel();
/**
 * @brief Sets the gain of an audio channel.
 * @param channel Identifier of the channel to modify.
 * @param volume Requested volume, conventionally in the range `0.0` to `1.0`.
 */
MOSS_API void Moss_AudioSetChannelVolume(ChannelID channel, float volume);
/**
 * @brief Mutes or unmutes an audio channel.
 * @param channel Identifier of the channel to modify.
 * @param mute `true` to mute the channel; `false` to unmute it.
 */
MOSS_API void Moss_AudioSetChannelMute(ChannelID channel, bool mute);
/**
 * @brief Adds an effect to an audio channel's effect chain.
 * @param channel Identifier of the target channel.
 * @param effect Effect to add.
 */
MOSS_API void Moss_AudioAddChannelEffect(ChannelID channel, AudioEffect* effect);
/**
 * @brief Removes an effect from an audio channel's effect chain.
 * @param channel Identifier of the target channel.
 * @param effect Effect to remove.
 */
MOSS_API void Moss_AudioRemoveChannelEffect(ChannelID channel, AudioEffect* effect);
/**
 * @brief Removes all effects from an audio channel.
 * @param channel Identifier of the channel to clear.
 */
MOSS_API void Moss_AudioRemoveAllChannelEffects(ChannelID channel);

/**
 * @brief Creates a non-spatial audio stream.
 * @return Pointer to the stream, or `nullptr` if creation fails.
 */
MOSS_API AudioStream* Moss_AudioStreamCreate();
/**
 * @brief Starts playback of an audio stream.
 * @param audiostream Stream to play.
 */
MOSS_API void Moss_AudioStreamPlay(AudioStream* audiostream);
/**
 * @brief Stops playback of an audio stream.
 * @param audiostream Stream to stop.
 */
MOSS_API void Moss_AudioStreamStop(AudioStream* audiostream);
/**
 * @brief Sets the playback volume of an audio stream.
 * @param audiostream Stream to modify.
 * @param volume Requested volume, conventionally in the range `0.0` to `1.0`.
 */
MOSS_API void Moss_AudioStreamSetVolume(AudioStream* audiostream, float volume);
/**
 * @brief Sets the pitch multiplier for an audio stream.
 * @param audiostream Stream to modify.
 * @param pitch Pitch multiplier; `1.0` conventionally represents the original pitch.
 */
MOSS_API void Moss_AudioStreamSetPitch(AudioStream* audiostream, float pitch);
/**
 * @brief Sets the playback-rate multiplier for an audio stream.
 * @param audiostream Stream to modify.
 * @param rate Playback-rate multiplier; `1.0` conventionally represents the original rate.
 */
MOSS_API void Moss_AudioStreamSetPlaybackRate(AudioStream* audiostream, float rate);
/**
 * @brief Sets the stereo pan position of an audio stream.
 * @param audiostream Stream to modify.
 * @param pan Pan value; the exact accepted range is implementation-defined.
 */
MOSS_API void Moss_AudioStreamSetPan(AudioStream* audiostream, float pan);
/**
 * @brief Enables or disables looping for an audio stream.
 * @param audiostream Stream to modify.
 * @param loop `true` to loop playback; `false` to play without looping.
 */
MOSS_API void Moss_AudioStreamSetLoop(AudioStream* audiostream, bool loop);
/**
 * @brief Removes an audio stream and releases its resources.
 * @param audiostream Stream to remove.
 */
MOSS_API void Moss_AudioStreamRemove(AudioStream* audiostream);

/**
 * @brief Creates a 2D audio stream.
 * @return Pointer to the stream, or `nullptr` if creation fails.
 */
MOSS_API AudioStream2D* Moss_AudioStream2DCreate();
/**
 * @brief Starts playback of a 2D audio stream.
 * @param audiostream Stream to play.
 */
MOSS_API void Moss_AudioStream2DPlay(AudioStream2D* audiostream);
/**
 * @brief Stops playback of a 2D audio stream.
 * @param audiostream Stream to stop.
 */
MOSS_API void Moss_AudioStream2DStop(AudioStream2D* audiostream);
/**
 * @brief Sets the playback volume of a 2D audio stream.
 * @param audiostream Stream to modify.
 * @param volume Requested volume, conventionally in the range `0.0` to `1.0`.
 */
MOSS_API void Moss_AudioStream2DSetVolume(AudioStream2D* audiostream, float volume);
/**
 * @brief Sets the pitch multiplier for a 2D audio stream.
 * @param audiostream Stream to modify.
 * @param pitch Pitch multiplier; `1.0` conventionally represents the original pitch.
 */
MOSS_API void Moss_AudioStream2DSetPitch(AudioStream2D* audiostream, float pitch);
/**
 * @brief Sets the playback-rate multiplier for a 2D audio stream.
 * @param audiostream Stream to modify.
 * @param rate Playback-rate multiplier; `1.0` conventionally represents the original rate.
 */
void Moss_AudioStream2DSetPlaybackRate(AudioStream2D* audiostream, float rate);
/**
 * @brief Sets the stereo pan position of a 2D audio stream.
 * @param audiostream Stream to modify.
 * @param pan Pan value; the exact accepted range is implementation-defined.
 */
MOSS_API void Moss_AudioStream2DSetPan(AudioStream2D* audiostream, float pan);
/**
 * @brief Enables or disables looping for a 2D audio stream.
 * @param audiostream Stream to modify.
 * @param loop `true` to loop playback; `false` to play without looping.
 */
MOSS_API void Moss_AudioStream2DSetLoop(AudioStream2D* audiostream, bool loop);
/**
 * @brief Declaration named for 3D positioning, but currently accepts an `AudioStream2D*` and no position argument.
 * @param audiostream Stream handle as declared. This signature does not expose a position value; verify whether the parameter type and signature are intended.
 */
MOSS_API void Moss_AudioStream3DSetPosition(AudioStream2D* audiostream);
/**
 * @brief Declaration named for 3D velocity, but currently accepts an `AudioStream2D*` and no velocity argument.
 * @param audiostream Stream handle as declared. This signature does not expose a velocity value; verify whether the parameter type and signature are intended.
 */
MOSS_API void Moss_AudioStream3DSetVelocity(AudioStream2D* audiostream);
/**
 * @brief Declaration named for 3D maximum distance, but currently accepts an `AudioStream2D*` and no distance argument.
 * @param audiostream Stream handle as declared. This signature does not expose a distance value; verify whether the parameter type and signature are intended.
 */
MOSS_API void Moss_AudioStream3DSetMaxDistance(AudioStream2D* audiostream);
/**
 * @brief Removes a 2D audio stream and releases its resources.
 * @param audiostream Stream to remove.
 */
MOSS_API void Moss_AudioStream2DRemove(AudioStream2D* audiostream);

/**
 * @brief Creates a 3D spatial audio stream.
 * @return Pointer to the stream, or `nullptr` if creation fails.
 */
MOSS_API AudioStream3D* Moss_AudioStream3DCreate();
/**
 * @brief Starts playback of a 3D spatial audio stream.
 * @param audiostream Stream to play.
 */
MOSS_API void Moss_AudioStream3DPlay(AudioStream3D* audiostream);
/**
 * @brief Stops playback of a 3D spatial audio stream.
 * @param audiostream Stream to stop.
 */
MOSS_API void Moss_AudioStream3DStop(AudioStream3D* audiostream);
/**
 * @brief Sets the playback volume of a 3D audio stream.
 * @param audiostream Stream to modify.
 * @param volume Requested volume, conventionally in the range `0.0` to `1.0`.
 */
MOSS_API void Moss_AudioStream3DSetVolume(AudioStream3D* audiostream, float volume);
/**
 * @brief Sets the pitch multiplier for a 3D audio stream.
 * @param audiostream Stream to modify.
 * @param pitch Pitch multiplier; `1.0` conventionally represents the original pitch.
 */
MOSS_API void Moss_AudioStream3DSetPitch(AudioStream3D* audiostream, float pitch);
/**
 * @brief Sets the playback-rate multiplier for a 3D audio stream.
 * @param audiostream Stream to modify.
 * @param rate Playback-rate multiplier; `1.0` conventionally represents the original rate.
 */
MOSS_API void Moss_AudioStream3DSetPlaybackRate(AudioStream3D* audiostream, float rate);
/**
 * @brief Sets the pan value for a 3D audio stream.
 * @param audiostream Stream to modify.
 * @param pan Pan value; the exact accepted range is implementation-defined.
 */
MOSS_API void Moss_AudioStream3DSetPan(AudioStream3D* audiostream, float pan);
/** @brief Enables or disables looping for a 3D audio stream. @param audiostream Stream to modify. @param loop `true` to loop playback; `false` to play without looping.
 */
MOSS_API void Moss_AudioStream3DSetLoop(AudioStream3D* audiostream, bool loop);
/**
 * @brief Updates the position of a 3D audio stream. @param audiostream Stream handle. Note: this declaration currently exposes no position argument, so a position cannot be supplied through this signature.
 */
MOSS_API void Moss_AudioStream3DSetPosition(AudioStream3D* audiostream);
/**
 * @brief Updates the velocity of a 3D audio stream. @param audiostream Stream handle. Note: this declaration currently exposes no velocity argument, so a velocity cannot be supplied through this signature.
 */
MOSS_API void Moss_AudioStream3DSetVelocity(AudioStream3D* audiostream);
/** @brief Updates the maximum audible distance of a 3D audio stream. @param audiostream Stream handle. Note: this declaration currently exposes no distance argument, so a distance cannot be supplied through this signature.
 */
MOSS_API void Moss_AudioStream3DSetMaxDistance(AudioStream3D* audiostream);
/** @brief Sets the distance-attenuation model used by a 3D audio stream. @param stream Stream to modify. @param model Distance model to apply.
 */
MOSS_API void Moss_AudioStream3DSetDistanceModel(AudioStream3D* stream, DistanceModel model);
/** @brief Removes a 3D audio stream and releases its resources. @param audiostream Stream to remove.
 */
MOSS_API void Moss_AudioStream3DRemove(AudioStream3D* audiostream);


/** @brief Creates a 2D audio listener at the supplied position. @param position Initial listener position. @return Pointer to the listener, or `nullptr` if creation fails.
 */
MOSS_API AudioListener2D* Moss_AudioCreateAudioListener2D(Float2 position);
/** @brief Creates a 3D audio listener at the supplied position. @param position Initial listener position. @return Pointer to the listener, or `nullptr` if creation fails.
 */
MOSS_API AudioListener3D* Moss_AudioCreateAudioListener3D(Float3 position);
/** @brief Creates a 2D ray-audio listener at the supplied position. @param position Initial listener position. @return Pointer to the listener, or `nullptr` if creation fails. Requires ray-audio support.
 */
MOSS_API RayAudioListener2D* Moss_AudioCreateRayAudioListener2D(Float2 position);
/** @brief Creates a 3D ray-audio listener at the supplied position. @param position Initial listener position. @return Pointer to the listener, or `nullptr` if creation fails. Requires ray-audio support.
 */
MOSS_API RayAudioListener3D* Moss_AudioCreateRayAudioListener3D(Float3 position);

/** @brief Destroys a 2D audio listener. @param listener Listener to remove.
 */
MOSS_API void Moss_AudioRemoveAudioListener2D(AudioListener2D* listener);
/** @brief Destroys a 3D audio listener. @param listener Listener to remove.
 */
MOSS_API void Moss_AudioRemoveAudioListener3D(AudioListener3D* listener);
/** @brief Destroys a 2D ray-audio listener. @param listener Listener to remove.
 */
MOSS_API void Moss_AudioRemoveRayAudioListener2D(RayAudioListener2D* listener);
/** @brief Destroys a 3D ray-audio listener. @param listener Listener to remove.
 */
MOSS_API void Moss_AudioRemoveRayAudioListener3D(RayAudioListener3D* listener);

/** @brief Activates or deactivates a 2D audio listener. @param listener Listener to update. @param activate `true` to activate it; `false` to deactivate it.
 */
MOSS_API void Moss_AudioActivateAudioListener2D(AudioListener2D* listener, bool activate);
/** @brief Activates or deactivates a 3D audio listener. @param listener Listener to update. @param activate `true` to activate it; `false` to deactivate it.
 */
MOSS_API void Moss_AudioActivateAudioListener3D(AudioListener3D* listener, bool activate);
/** @brief Activates or deactivates a 2D ray-audio listener. @param listener Listener to update. @param activate `true` to activate it; `false` to deactivate it.
 */
MOSS_API void Moss_AudioActivateRayAudioListener2D(RayAudioListener2D* listener, bool activate);
/** @brief Activates or deactivates a 3D ray-audio listener. @param listener Listener to update. @param activate `true` to activate it; `false` to deactivate it.
 */
MOSS_API void Moss_AudioActivateRayAudioListener3D(RayAudioListener3D* listener, bool activate);

/** @brief Sets the orientation vectors for a 3D audio listener. @param listener Listener to modify. @param forward Forward direction vector. @param up Up direction vector.
 */
MOSS_API void Moss_AudioListenerSetOrientation(AudioListener3D* listener, const Vec3& forward, const Vec3& up);
/** @brief Configures the physics system and raycast callback used by a 2D ray-audio listener. @param listener Listener to configure. @param physics Physics system supplied to the callback. @param raycast Callback used to query 2D ray intersections. @param user_data User pointer forwarded to the callback.
 */
MOSS_API void Moss_AudioRayListener2DSetPhysics(RayAudioListener2D* listener, PhysicsSystem* physics, Moss_AudioRaycast2DCallback raycast, void* user_data);
/** @brief Configures the physics system and raycast callback used by a 3D ray-audio listener. @param listener Listener to configure. @param physics Physics system supplied to the callback. @param raycast Callback used to query 3D ray intersections. @param user_data User pointer forwarded to the callback.
 */
MOSS_API void Moss_AudioRayListener3DSetPhysics(RayAudioListener3D* listener, PhysicsSystem* physics, Moss_AudioRaycast3DCallback raycast, void* user_data);
/** @brief Sets ray-tracing limits for a 2D ray-audio listener. @param listener Listener to configure. @param max_distance Maximum ray-tracing distance. @param reflection_rays Number of rays to use for reflection tracing.
 */
MOSS_API void Moss_AudioRayListener2DSetTraceSettings(RayAudioListener2D* listener, float max_distance, uint32_t reflection_rays);
/** @brief Sets ray-tracing limits for a 3D ray-audio listener. @param listener Listener to configure. @param max_distance Maximum ray-tracing distance. @param reflection_rays Number of rays to use for reflection tracing.
 */
MOSS_API void Moss_AudioRayListener3DSetTraceSettings(RayAudioListener3D* listener, float max_distance, uint32_t reflection_rays);
/** @brief Traces audio paths in 2D using the supplied description and callback. @param desc Trace configuration, including physics/raycast callback and source/listener positions. @param[out] out_result Receives the computed audibility, occlusion, attenuation, and reflection data. @return `true` if tracing produced a result; otherwise `false`.
 */
MOSS_API bool Moss_AudioRayTrace2D(const Moss_AudioRayTrace2DDesc* desc, Moss_AudioRayTraceResult* out_result);
/** @brief Traces audio paths in 3D using the supplied description and callback. @param desc Trace configuration, including physics/raycast callback and source/listener positions. @param[out] out_result Receives the computed audibility, occlusion, attenuation, and reflection data. @return `true` if tracing produced a result; otherwise `false`.
 */
MOSS_API bool Moss_AudioRayTrace3D(const Moss_AudioRayTrace3DDesc* desc, Moss_AudioRayTraceResult* out_result);
/** @brief Traces an audio path from a configured 2D ray-audio listener to a source position. @param listener Listener providing the trace configuration. @param source_position Position of the audio source. @param[out] out_result Receives the trace result. @return `true` if tracing produced a result; otherwise `false`.
 */
MOSS_API bool Moss_AudioRayTraceFromListener2D(RayAudioListener2D* listener, const Vec2* source_position, Moss_AudioRayTraceResult* out_result);
/** @brief Traces an audio path from a configured 3D ray-audio listener to a source position. @param listener Listener providing the trace configuration. @param source_position Position of the audio source. @param[out] out_result Receives the trace result. @return `true` if tracing produced a result; otherwise `false`.
 */
MOSS_API bool Moss_AudioRayTraceFromListener3D(RayAudioListener3D* listener, const Vec3* source_position, Moss_AudioRayTraceResult* out_result);
/** @brief Computes an audio gain value from a ray-trace result. @param result Trace result to evaluate. @return Computed gain; the exact combination of result fields is implementation-defined.
 */
MOSS_API float Moss_AudioRayTraceComputeGain(const Moss_AudioRayTraceResult* result);


/** @brief Reports whether a speaker output device is ready. @return `true` if speaker output is ready; otherwise `false`.
 */
MOSS_API bool Moss_IsSpeakerDeviceReady();
/** @brief Opens or initializes the selected speaker output device.
 */
MOSS_API void Moss_AudioSpeakerOpen();

/** @brief Pauses speaker audio output.
 */
MOSS_API void Moss_AudioSpeakerPause();
/** @brief Resumes speaker audio output.
 */
MOSS_API void Moss_AudioSpeakerResume();
/** @brief Reports whether speaker output is paused. @return `true` if output is paused; otherwise `false`.
 */
MOSS_API bool Moss_AudioSpeakerIsPaused();
/** @brief Selects the speaker device identified by the supplied index or ID. @param id Speaker device identifier. @return `true` if selection succeeds; otherwise `false`.
 */
MOSS_API bool Moss_AudioSelectSpeakerDevice(int id);
/** @brief Returns the identifier of the currently selected speaker device. @return Current speaker device ID, or the implementation-defined sentinel when none is selected.
 */
MOSS_API int Moss_GetCurrentSpeakerDeviceID();
/** @brief Returns the display name of a speaker device. @param id Speaker device identifier. @return Device name, or `nullptr` if the ID is invalid or no name is available.
 */
MOSS_API const char* Moss_GetSpeakerDeviceName(int id);
/** @brief Returns the number of available speaker devices. @return Number of enumerated speaker devices.
 */
MOSS_API int Moss_ListSpeakerDevices();

// Microphone
/** @brief Returns the number of enumerated microphone input devices. @return Number of microphone devices.
 */
MOSS_API uint32_t Moss_MicrophoneGetDeviceCount();
/** @brief Returns the display name of a microphone input device. @param device_index Zero-based microphone device index. @return Device name, or `nullptr` if the index is invalid or no name is available.
 */
MOSS_API const char* Moss_MicrophoneGetDeviceName(uint32_t device_index);
/**
 * @brief Opens a microphone using the supplied capture configuration. @param desc Microphone configuration; must point to a valid descriptor. @return Pointer to the opened microphone, or `nullptr` if opening fails.
 */
MOSS_API Moss_Microphone* Moss_MicrophoneOpen(const Moss_MicrophoneDesc* desc);
/** @brief Closes a microphone and releases its capture resources. @param microphone Microphone to close.
 */
MOSS_API void Moss_MicrophoneClose(Moss_Microphone* microphone);
/** @brief Starts microphone capture. @param microphone Microphone to start. @return `true` if capture starts successfully; otherwise `false`.
 */
MOSS_API bool Moss_MicrophoneStart(Moss_Microphone* microphone);
/** @brief Stops microphone capture. @param microphone Microphone to stop.
 */
MOSS_API void Moss_MicrophoneStop(Moss_Microphone* microphone);
/** @brief Reads captured microphone samples into the supplied buffer. @param microphone Microphone to read from. @param[out] out_samples Destination sample buffer. @param max_frames Maximum number of frames to read, according to the API's frame convention. @return Number of frames read.
 */
MOSS_API uint32_t Moss_MicrophoneRead(Moss_Microphone* microphone, float* out_samples, uint32_t max_frames);
/** @brief Sets the input gain for a microphone. @param microphone Microphone to modify. @param gain Requested input gain.
 */
MOSS_API void Moss_MicrophoneSetGain(Moss_Microphone* microphone, float gain);
/** @brief Returns the configured sample rate of a microphone. @param microphone Microphone to query. @return Sample rate in samples per second.
 */
MOSS_API uint32_t Moss_MicrophoneGetSampleRate(Moss_Microphone* microphone);
/** @brief Returns the number of audio channels configured for a microphone. @param microphone Microphone to query. @return Number of channels.
 */
MOSS_API uint32_t Moss_MicrophoneGetChannels(Moss_Microphone* microphone);
/** @brief Sets the callback invoked with captured microphone samples. @param microphone Microphone to configure. @param callback Callback to receive sample data, or `nullptr` to clear it if supported. @param userData User pointer passed to the callback.
 */
MOSS_API void Moss_MicrophoneSetCallback(Moss_Microphone* microphone, MicrophoneCallback callback, void* userData);
/** @brief Returns the current microphone level metrics. @param microphone Microphone to query. @return RMS, peak, smoothed-volume, and voice-activity measurements.
 */
MOSS_API Moss_MicrophoneLevels Moss_MicrophoneGetLevels(Moss_Microphone* microphone);
/** @brief Returns the microphone's root-mean-square level. @param microphone Microphone to query. @return RMS level.
 */
MOSS_API float Moss_MicrophoneGetLevelRMS(Moss_Microphone* microphone);
/** @brief Returns the microphone's peak level. @param microphone Microphone to query. @return Peak level.
 */
MOSS_API float Moss_MicrophoneGetLevelPeak(Moss_Microphone* microphone);
/**
 * @brief Returns the microphone's smoothed volume measurement. @param microphone Microphone to query. @return Smoothed volume value.
 */
MOSS_API float Moss_MicrophoneGetSmoothedVolume(Moss_Microphone* microphone);
/** @brief Returns the microphone's voice-activity measurement. @param microphone Microphone to query. @return Voice-activity value.
 */
MOSS_API float Moss_MicrophoneGetVoiceActivity(Moss_Microphone* microphone);
/** @brief Reports whether a microphone input device is ready. @return `true` if microphone input is ready; otherwise `false`.
 */
MOSS_API bool Moss_IsMicrophoneDeviceReady();
/** @brief Opens the default or currently selected microphone through the legacy audio interface. @return The existing header does not specify a return value convention; verify the implementation before relying on a status code.
 */
MOSS_API int Moss_AudioMicrophoneOpen();
/** @brief Closes the microphone opened through the legacy audio interface.
 */
MOSS_API void Moss_AudioMicrophoneClose();
/** @brief Starts microphone capture through the legacy audio interface.
 */
MOSS_API void Moss_AudioMicrophonePlay();
/** @brief Stops microphone capture through the legacy audio interface.
 */
MOSS_API void Moss_AudioMicrophoneStop();
/** @brief Returns the ID of the default or currently selected microphone. @return Microphone device ID, or an implementation-defined sentinel when unavailable.
 */
MOSS_API int Moss_AudioMicrophoneID();
/** @brief Selects the microphone device identified by the supplied ID. @param id Microphone device identifier. @return `true` if selection succeeds; otherwise `false`.
 */
MOSS_API bool Moss_AudioSelectMicrophoneDevice(int id);
/** @brief Returns the display name of a microphone device. @param index Microphone device index. @return Device name, or `nullptr` if the index is invalid or no name is available.
 */
MOSS_API const char* Moss_GetMicrophoneDeviceName(int index);
/** @brief Returns the number of available microphone devices. @return Number of enumerated microphone devices.
 */
MOSS_API int Moss_ListMicrophoneDevices();
/** @brief Sets the gain for the specified microphone. @param microphone Microphone to modify. @param gain Requested input gain.
 */
MOSS_API void Moss_AudioMicrophoneSetGain(Moss_Microphone* microphone, float gain);
/**
 * @brief Returns the sample rate configured for the specified microphone. @param microphone Microphone to query. @return Sample rate in samples per second.
 */
MOSS_API int Moss_AudioMicrophoneGetSampleRate(Moss_Microphone* microphone);
/** @brief Returns the number of channels configured for the specified microphone. @param microphone Microphone to query. @return Number of channels.
 */
MOSS_API int Moss_AudioMicrophoneGetChannels(Moss_Microphone* microphone);

/**
 * @brief Sets the callback used to supply audio-stream samples. @param stream Stream to configure. @param callback Callback that fills the requested audio buffer, or `nullptr` to clear it if supported. @param userData User pointer passed to the callback.
 */
MOSS_API void Moss_AudioStreamSetCallback(AudioStream* stream, AudioStreamCallback callback, void* userData);
/*! @brief Set a microphone capture callback. @param mic Microphone device. @param callback Callback function. @param userData User-defined pointer. */


#endif // MOSS_AUDIO_H

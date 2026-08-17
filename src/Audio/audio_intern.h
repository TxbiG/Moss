#pragma once

#include <Moss/Moss_Audio.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <mutex>
#include <fstream>
#include <string>

#define SOUND_SPEED 2043.0f

enum class AudioFormat : uint32_t{
    UNKNOWN = 0x0000u,  // Unspecified audio format.
    U8      = 0x0008u,  // Unsigned 8-bit samples.
    S8      = 0x8008u,  // Signed 8-bit samples.
    S16LE   = 0x8010u,  // Signed 16-bit samples.
    S16BE   = 0x9010u,  // As above, but big-endian byte order.
    S24LE   = 0x8020u,  // Add this
    S32LE   = 0x8030u,  // 32-bit integer samples.
    S32BE   = 0x9030u,  // As above, but big-endian byte order.
    F32LE   = 0x8120u,  // 32-bit floating point samples.
    F32BE   = 0x9120u,  // As above, but big-endian byte order.
    F64LE   = 0x8140u,  // 64-bit float (double)
    F64BE   = 0x9140u
};

struct AudioSpec_t {
    AudioFormat format;
    int channels;
    int frequency;
};

struct AudioListener2D {
    Vec2 position;
    Vec2 velocity;
    bool active;
    float currentPan = 0.0f;      // left/right balance
    float currentDoppler = 0.0f;  // simulated doppler
};

struct AudioListener3D {
    Vec3 position;
    Vec3 velocity;
    bool active;
    float currentPan = 0.0f;
    float currentDoppler = 0.0f;
};

struct AudioChannel {
    ChannelID id;
    ChannelID parent;        // CHANNEL_INVALID = master output

    float volume;            // 0..1
    uint8_t muted;

    AudioEffect* effects;    // linked list
};


typedef void (*AudioEffectProcess)(float* samples, uint32_t frames, uint32_t channels, void* userdata);

struct AudioEffect{
    AudioEffectType type;
    AudioEffectProcess process;
    void* state;

    AudioEffect* next;
};

struct Wav::Wav_t {
    // Metadata
    uint32 riffChunkId;
    uint32 riffChunkSize;
    uint32 format;
    uint32 formatChunkId;
    uint32 formatChunkSize;
    uint16 audioFormat;
    uint16 numChannels;
    uint32 sampleRate;
    uint32 byteRate;
    uint16 blockAlign;
    uint16 bitsPerSample;
    uint8 dataChunkId[4];
    uint32 dataChunkSize;

    // Raw PCM data
    char* dataBegin;
};


typedef enum Moss_AudioSourceType {
    MOSS_AUDIO_SOURCE_WAV,
    MOSS_AUDIO_SOURCE_MICROPHONE,
    MOSS_AUDIO_SOURCE_CUSTOM
} Moss_AudioSourceType;

typedef struct Moss_AudioSource {
    Moss_AudioSourceType type;
    void* userdata;

    // Fill buffer with PCM frames
    uint32_t (*read)(struct Moss_AudioSource* src, float* out_samples, uint32_t frames);

    // Optional
    void (*reset)(struct Moss_AudioSource* src);
    void (*destroy)(struct Moss_AudioSource* src);
} Moss_AudioSource;

typedef struct {
    Moss_Microphone* mic;
    float ringbuffer[4096];
    uint32_t read_pos;
    uint32_t write_pos;
} MicSourceData;


uint32_t mic_read(
    Moss_AudioSource* src,
    float* out,
    uint32_t frames
) {
    MicSourceData* data = (MicSourceData*)src->userdata;

    uint32_t available =
        (data->write_pos - data->read_pos) & 4095;

    uint32_t to_copy = frames < available ? frames : available;

    for (uint32_t i = 0; i < to_copy; ++i) {
        out[i] = data->ringbuffer[data->read_pos & 4095];
        data->read_pos++;
    }

    return to_copy;
}

void mic_callback(void* user, const void* input, size_t bytes) {
    MicSourceData* data = (MicSourceData*)user;
    const float* samples = (const float*)input;

    size_t frames = bytes / sizeof(float);
    for (size_t i = 0; i < frames; ++i) {
        data->ringbuffer[data->write_pos & 4095] = samples[i];
        data->write_pos++;
    }
}

struct AudioStream2D::AudioStream2D_t {
    AudioStream stream;
    Vec2 position;
    Vec3 velocity;
    float maxDistance;
};

struct AudioStream2D::AudioStream2D_t {
    AudioStream stream;
    Vec2 position;
    Vec3 velocity;
    float maxDistance;
    float currentPan = 0.0f;
};

struct RayAudioListener2D : public AudioListener2D {
    float occlusion = 0.0f;
    float reflectionGain = 0.0f;
    float reflectionDelay = 0.0f;
    uint32_t rayCount = 0;
    float maxRayDistance = 100.0f;
    PhysicsSystem* physicsScene = nullptr;
    Moss_AudioRaycast2DCallback raycast = nullptr;
    void* raycastUserData = nullptr;
};

struct RayAudioListener3D : public AudioListener3D {
    float occlusion = 0.0f;
    float reflectionGain = 0.0f;
    float reflectionDelay = 0.0f;
    uint32_t rayCount = 0;
    float maxRayDistance = 100.0f;
    PhysicsSystem* physicsScene = nullptr;
    Moss_AudioRaycast3DCallback raycast = nullptr;
    void* raycastUserData = nullptr;
};


typedef void (*Moss_MicrophoneCallback)(void* user_data, const void* data, size_t size);

Microphone* Moss_CreateMicrophone(Moss_MicrophoneCallback callback, void* user_data);
void Moss_RemoveMicrophone(Microphone* mic);


void Audio_MixChannel(ChannelID id, float* buffer, uint32_t frames, uint32_t channels) {
    AudioChannel* ch = GetChannel(id);
    if (!ch || ch->muted) return;

    // 1. Effects
    for (AudioEffect::AudioEffect_t* fx = ch->effects; fx; fx = fx->next)
        fx->process(buffer, frames, channels, fx->state);

    // 2. Volume
    for (uint32_t i = 0; i < frames * channels; i++)
        buffer[i] *= ch->volume;

    // 3. Send to parent
    if (ch->parent != CHANNEL_INVALID)
        Audio_MixChannel(ch->parent, buffer, frames, channels);
}


// Populate internal microphone list.
void Moss_EnumerateMicrophone();
// Populate internal speaker list.
void Moss_EnumerateSpeakers();

static uint32 ReadU32LE(const uint8* data) { return data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24); }
static uint16 ReadU16LE(const uint8* data) { return data[0] | (data[1] << 8); }

Wav* CreateWav(const char* path) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file.is_open()) return nullptr;

    size_t fileSize = static_cast<size_t>(file.tellg());
    file.seekg(0, std::ios::beg);

    std::vector<uint8> buffer(fileSize);
    if (!file.read(reinterpret_cast<char*>(buffer.data()), fileSize)) return nullptr;

    size_t pos = 0;
    Wav* wav = new Wav();

    // Parse RIFF header
    wav->riffChunkId   = ReadU32LE(&buffer[pos]); pos += 4;
    wav->riffChunkSize = ReadU32LE(&buffer[pos]); pos += 4;
    wav->format        = ReadU32LE(&buffer[pos]); pos += 4;

    // "fmt " subchunk
    wav->formatChunkId   = ReadU32LE(&buffer[pos]); pos += 4;
    wav->formatChunkSize = ReadU32LE(&buffer[pos]); pos += 4;
    wav->audioFormat     = ReadU16LE(&buffer[pos]); pos += 2;
    wav->numChannels     = ReadU16LE(&buffer[pos]); pos += 2;
    wav->sampleRate      = ReadU32LE(&buffer[pos]); pos += 4;
    wav->byteRate        = ReadU32LE(&buffer[pos]); pos += 4;
    wav->blockAlign      = ReadU16LE(&buffer[pos]); pos += 2;
    wav->bitsPerSample   = ReadU16LE(&buffer[pos]); pos += 2;

    // Search for "data" chunk (may come after other chunks like "fact")
    while (pos + 8 < buffer.size()) {
        const char* chunkID = reinterpret_cast<const char*>(&buffer[pos]);
        uint32 chunkSize = ReadU32LE(&buffer[pos + 4]);

        if (std::memcmp(chunkID, "data", 4) == 0) {
            std::memcpy(wav->dataChunkId, chunkID, 4);
            wav->dataChunkSize = chunkSize;
            wav->dataBegin = new char[chunkSize];
            std::memcpy(wav->dataBegin, &buffer[pos + 8], chunkSize);
            break;
        }

        pos += 8 + chunkSize; // skip this chunk
    }

    if (!wav->dataBegin) { delete wav; return nullptr; }
    return wav;
}

void RemoveWav(Wav* wav) { if (wav) { delete[] wav->dataBegin;  delete wav; } }


// This is used to help build the pan Matrix
std::vector<float> AudioPan(int channels, float pan) {
    pan = std::clamp(pan, -1.f, 1.f);
    std::vector<float> gains(channels, 0.f);

    switch(channels) {
        case 1: // Mono
            gains[0] = 1.0f;  // no pan, just full volume
            break;
        case 2: // Stereo
            // Simple linear pan law
            gains[0] = (pan <= 0) ? 1.0f : 1.0f - pan; // Left
            gains[1] = (pan >= 0) ? 1.0f : 1.0f + pan; // Right
            break;
        case 3: // 2.1 (L, R, Subwoofer)
            gains[0] = (pan <= 0) ? 1.0f : 1.0f - pan; // Left
            gains[1] = (pan >= 0) ? 1.0f : 1.0f + pan; // Right
            gains[2] = 1.0f;  // Subwoofer always full
            break;
        case 4: // Quad/Surround (L, R, Rear L, Rear R)
            // Front pan balance, rear speakers get half volume
            gains[0] = (pan <= 0) ? 1.0f : 1.0f - pan; // Front Left
            gains[1] = (pan >= 0) ? 1.0f : 1.0f + pan; // Front Right
            gains[2] = gains[0] * 0.5f;                // Rear Left
            gains[3] = gains[1] * 0.5f;                // Rear Right
            break;
        case 5: // 4.1 (4 + Subwoofer)
            gains[0] = (pan <= 0) ? 1.0f : 1.0f - pan; // Front Left
            gains[1] = (pan >= 0) ? 1.0f : 1.0f + pan; // Front Right
            gains[2] = gains[0] * 0.5f;                // Rear Left
            gains[3] = gains[1] * 0.5f;                // Rear Right
            gains[4] = 1.0f;                           // Subwoofer
            break;
        case 6: // 5.1 (L, R, Center, LFE, Rear L, Rear R)
            // Pan affects Left, Right, Rear L, Rear R
            // Center and LFE full volume
            gains[0] = (pan <= 0) ? 1.0f : 1.0f - pan; // Left
            gains[1] = (pan >= 0) ? 1.0f : 1.0f + pan; // Right
            gains[2] = 1.0f;                           // Center
            gains[3] = 1.0f;                           // LFE (subwoofer)
            gains[4] = gains[0] * 0.5f;                // Rear Left
            gains[5] = gains[1] * 0.5f;                // Rear Right
            break;
        case 8: // 7.1 (L, R, Center, LFE, Rear L, Rear R, Side L, Side R)
            gains[0] = (pan <= 0) ? 1.0f : 1.0f - pan; // Left
            gains[1] = (pan >= 0) ? 1.0f : 1.0f + pan; // Right
            gains[2] = 1.0f;                           // Center
            gains[3] = 1.0f;                           // LFE
            gains[4] = gains[0] * 0.5f;                // Rear Left
            gains[5] = gains[1] * 0.5f;                // Rear Right
            gains[6] = gains[0] * 0.7f;                // Side Left
            gains[7] = gains[1] * 0.7f;                // Side Right
            break;
        default:
            // Unknown config, just full volume on all channels
            for (int i = 0; i < channels; ++i) { gains[i] = 1.0f; }
            break;
    }

    return gains;
}

// This is used to help add a 3D audio effect
float DopplerPitch(const Vec3& listenerPos, const Vec3& listenerVel,const Vec3& sourcePos,const Vec3& sourceVel,float speedOfSound = 343.0f, float dopplerFactor = 1.0f) {
    Vec3 relativePos = sourcePos - listenerPos;
    Vec3 relativeDir = (sourcePos - listenerPos).Normalized();

    float listenerSpeed = listenerVel.Dot(relativeDir); // how fast the listener is moving toward the source
    float sourceSpeed   = sourceVel.Dot(relativeDir);   // how fast the so

    float adjustedSpeedOfSound = speedOfSound / dopplerFactor;

    float numerator = adjustedSpeedOfSound + listenerSpeed;
    float denominator = adjustedSpeedOfSound + sourceSpeed;

    if (denominator <= 0.01f) denominator = 0.01f; // prevent divide by zero

    float pitch = numerator / denominator;
    return std::clamp(pitch, 0.5f, 2.0f); // typical safe audio range
}






// ============================
// DSP Implementations
// ============================

static float Moss_AudioClamp(float value, float minValue, float maxValue) {
    return value < minValue ? minValue : (value > maxValue ? maxValue : value);
}

static bool Moss_AudioParamEquals(const char* a, const char* b) {
    return a && b && std::strcmp(a, b) == 0;
}

// --- Filters ---
static void AudioEffectProcess_Lowpass(float* samples, uint32_t frames, uint32_t channels, void* userdata) {
    float* state = (float*)userdata;
    if (!samples || !state || channels == 0) return;
    for (uint32_t i = 0; i < frames * channels; i++) {
        float x = samples[i];
        samples[i] = 0.5f * x + 0.5f * state[0];
        state[0] = samples[i];
    }
}

static void AudioEffectProcess_Highpass(float* samples, uint32_t frames, uint32_t channels, void* userdata) {
    float* state = (float*)userdata;
    if (!samples || !state || channels == 0) return;
    for (uint32_t i = 0; i < frames * channels; i++) {
        float x = samples[i];
        samples[i] = x - 0.5f * state[0];
        state[0] = x;
    }
}


// --- Echo / Delay ---
typedef struct {
    float* buffer;
    uint32_t writePos;
    uint32_t size;
    float feedback;
} DelayState;

static void AudioEffectProcess_Delay(float* samples, uint32_t frames, uint32_t channels, void* userdata) {
    DelayState* state = (DelayState*)userdata;
    if (!samples || !state || !state->buffer || state->size == 0 || channels == 0) return;
    for (uint32_t i = 0; i < frames * channels; i++) {
        float delayed = state->buffer[state->writePos];
        state->buffer[state->writePos] = Moss_AudioClamp(samples[i] + delayed * state->feedback, -1.0f, 1.0f);
        samples[i] = Moss_AudioClamp(samples[i] + delayed * state->mix, -1.0f, 1.0f);
        state->writePos = (state->writePos + 1) % state->size;
    }
}


// --- Normalize ---
static void AudioEffectProcess_Normalize(float* samples, uint32_t frames, uint32_t channels, void* userdata) {
    float gain = *(float*)userdata;
    for (uint32_t i = 0; i < frames * channels; i++)
        samples[i] *= gain;
}

// --- Distortion ---
static void AudioEffectProcess_Distortion(float* samples, uint32_t frames, uint32_t channels, void* userdata) {
    float gain = *(float*)userdata;
    for (uint32_t i = 0; i < frames * channels; i++) {
        samples[i] *= gain;
        if (samples[i] > 1.0f) samples[i] = 1.0f;
        if (samples[i] < -1.0f) samples[i] = -1.0f;
    }
}

// --- Chorus (very simple) ---
typedef struct { float depth, phase; } ChorusState;
static void AudioEffectProcess_Chorus(float* samples, uint32_t frames, uint32_t channels, void* userdata) {
    // Simplified: just phase modulation without delay buffer
    ChorusState* state = (ChorusState*)userdata;
    for (uint32_t i = 0; i < frames * channels; i++) {
        samples[i] *= sinf(state->phase);
        state->phase += state->depth;
        if (state->phase > 2.0f * M_PI) state->phase -= 2.0f * M_PI;
    }
}

// --- Parametric EQ placeholder ---
typedef struct {
    float sampleRate;
    float frequency;
    float gainDb;
    float q;
    float z1[8];
    float z2[8];
} ParamEqState;


static void AudioEffectProcess_ParamEq(float* samples, uint32_t frames, uint32_t channels, void* userdata) {
    ParamEqState* state = (ParamEqState*)userdata;
    if (!samples || !state || channels == 0) return;
    const uint32_t usedChannels = channels > 8 ? 8 : channels;
    const float frequency = Moss_AudioClamp(state->frequency, 20.0f, state->sampleRate * 0.45f);
    const float q = Moss_AudioClamp(state->q, 0.1f, 18.0f);
    const float a = std::pow(10.0f, Moss_AudioClamp(state->gainDb, -24.0f, 24.0f) / 40.0f);
    const float w0 = 6.28318530718f * frequency / state->sampleRate;
    const float alpha = std::sin(w0) / (2.0f * q);
    const float cosw = std::cos(w0);
    float b0 = 1.0f + alpha * a;
    float b1 = -2.0f * cosw;
    float b2 = 1.0f - alpha * a;
    float a0 = 1.0f + alpha / a;
    float a1 = -2.0f * cosw;
    float a2 = 1.0f - alpha / a;
    b0 /= a0; b1 /= a0; b2 /= a0; a1 /= a0; a2 /= a0;

    for (uint32_t frame = 0; frame < frames; ++frame) {
        for (uint32_t channel = 0; channel < channels; ++channel) {
            const uint32_t idx = frame * channels + channel;
            if (channel >= usedChannels) continue;
            const float input = samples[idx];
            const float output = b0 * input + state->z1[channel];
            state->z1[channel] = b1 * input - a1 * output + state->z2[channel];
            state->z2[channel] = b2 * input - a2 * output;
            samples[idx] = Moss_AudioClamp(output, -1.0f, 1.0f);
        }
    }
}

typedef struct {
    float thresholdDb;
    float ratio;
    float attack;
    float release;
    float makeupGain;
    float envelope;
} CompressorState;


// --- Compressor placeholder ---
static void AudioEffectProcess_Compressor(float* samples, uint32_t frames, uint32_t channels, void* userdata) {
    CompressorState* state = (CompressorState*)userdata;
    if (!samples || !state || channels == 0) return;
    const float threshold = std::pow(10.0f, state->thresholdDb / 20.0f);
    const float ratio = Moss_AudioClamp(state->ratio, 1.0f, 40.0f);
    for (uint32_t i = 0; i < frames * channels; ++i) {
        const float level = std::fabs(samples[i]);
        const float coeff = level > state->envelope ? state->attack : state->release;
        state->envelope = state->envelope * coeff + level * (1.0f - coeff);
        float gain = 1.0f;
        if (state->envelope > threshold) {
            const float compressed = threshold + (state->envelope - threshold) / ratio;
            gain = compressed / state->envelope;
        }
        samples[i] = Moss_AudioClamp(samples[i] * gain * state->makeupGain, -1.0f, 1.0f);
    }
}

// --- Reverb: small multi-tap feedback network ---
typedef struct {
    float* buffers[4];
    uint32_t positions[4];
    uint32_t sizes[4];
    float decay;
    float mix;
} ReverbState;

static void AudioEffectProcess_Reverb(float* samples, uint32_t frames, uint32_t channels, void* userdata) {
    ReverbState* state = (ReverbState*)userdata;
    if (!samples || !state || channels == 0) return;
    for (uint32_t i = 0; i < frames * channels; ++i) {
        float wet = 0.0f;
        for (uint32_t tap = 0; tap < 4; ++tap) {
            if (!state->buffers[tap] || state->sizes[tap] == 0) continue;
            float delayed = state->buffers[tap][state->positions[tap]];
            state->buffers[tap][state->positions[tap]] = Moss_AudioClamp(samples[i] + delayed * state->decay, -1.0f, 1.0f);
            state->positions[tap] = (state->positions[tap] + 1) % state->sizes[tap];
            wet += delayed * 0.25f;
        }
        samples[i] = Moss_AudioClamp(samples[i] * (1.0f - state->mix) + wet * state->mix, -1.0f, 1.0f);
    }
}

// --- Pitch Shifter: lightweight in-place resampler ---
typedef struct {
    float ratio;
    float phase;
} PitchShiftState;

static void AudioEffectProcess_PitchShifter(float* samples, uint32_t frames, uint32_t channels, void* userdata) {
    PitchShiftState* state = (PitchShiftState*)userdata;
    if (!samples || !state || frames < 2 || channels == 0) return;
    const float ratio = Moss_AudioClamp(state->ratio, 0.5f, 2.0f);
    for (uint32_t channel = 0; channel < channels; ++channel) {
        float phase = state->phase;
        for (uint32_t frame = 0; frame < frames; ++frame) {
            const uint32_t base = (uint32_t)phase;
            const uint32_t next = base + 1 < frames ? base + 1 : base;
            const float frac = phase - (float)base;
            const float a = samples[base * channels + channel];
            const float b = samples[next * channels + channel];
            samples[frame * channels + channel] = a + (b - a) * frac;
            phase += ratio;
            while (phase >= (float)(frames - 1)) phase -= (float)(frames - 1);
        }
        state->phase = phase;
    }
}

// --- Flange ---
typedef struct {
    float* buffer;
    uint32_t size;
    uint32_t writePos;
    float phase;
    float rate;
    float depthSamples;
    float feedback;
    float mix;
} FlangeState;

static void AudioEffectProcess_Flange(float* samples, uint32_t frames, uint32_t channels, void* userdata) {
    FlangeState* state = (FlangeState*)userdata;
    if (!samples || !state || !state->buffer || state->size == 0 || channels == 0) return;
    for (uint32_t i = 0; i < frames * channels; ++i) {
        const float lfo = 0.5f + 0.5f * std::sin(state->phase);
        const uint32_t delay = (uint32_t)Moss_AudioClamp(1.0f + lfo * state->depthSamples, 1.0f, (float)(state->size - 1));
        const uint32_t readPos = (state->writePos + state->size - delay) % state->size;
        const float delayed = state->buffer[readPos];
        const float input = samples[i];
        samples[i] = Moss_AudioClamp(input * (1.0f - state->mix) + delayed * state->mix, -1.0f, 1.0f);
        state->buffer[state->writePos] = Moss_AudioClamp(input + delayed * state->feedback, -1.0f, 1.0f);
        state->writePos = (state->writePos + 1) % state->size;
        state->phase += state->rate;
        if (state->phase > 6.28318530718f) state->phase -= 6.28318530718f;
    }
}

// ============================
// State allocation per effect
// ============================
static void* AudioEffect_AllocateState(AudioEffectType type) {
    void* state = NULL;
    switch(type) {
        case AudioEffectType::LOWPASS:
        case AudioEffectType::HIGHPASS:
            state = calloc(1, sizeof(float));
            break;
        case AudioEffectType::NORMALIZE:
        case AudioEffectType::DISTORTION:
            state = malloc(sizeof(float));
            if (state) *(float*)state = 1.0f;
            break;
        case AudioEffectType::CHORUS:
            state = calloc(1, sizeof(ChorusState));
            if (state) ((ChorusState*)state)->depth = 0.05f;
            break;
        case AudioEffectType::DELAY:
        case AudioEffectType::ECHO: {
            DelayState* d = (DelayState*)calloc(1, sizeof(DelayState));
            if (!d) break;
            d->size = 44100;
            d->buffer = (float*)calloc(d->size, sizeof(float));
            d->feedback = 0.35f;
            d->mix = 0.5f;
            state = d;
            break;
        }
        case AudioEffectType::PARAMEQ: {
            ParamEqState* eq = (ParamEqState*)calloc(1, sizeof(ParamEqState));
            if (!eq) break;
            eq->sampleRate = 48000.0f;
            eq->frequency = 1000.0f;
            eq->gainDb = 0.0f;
            eq->q = 0.707f;
            state = eq;
            break;
        }
        case AudioEffectType::COMPRESSOR: {
            CompressorState* c = (CompressorState*)calloc(1, sizeof(CompressorState));
            if (!c) break;
            c->thresholdDb = -18.0f;
            c->ratio = 4.0f;
            c->attack = 0.90f;
            c->release = 0.995f;
            c->makeupGain = 1.0f;
            state = c;
            break;
        }
        case AudioEffectType::REVERB: {
            ReverbState* r = (ReverbState*)calloc(1, sizeof(ReverbState));
            if (!r) break;
            const uint32_t defaults[4] = { 1499, 2111, 2633, 3331 };
            for (uint32_t i = 0; i < 4; ++i) {
                r->sizes[i] = defaults[i];
                r->buffers[i] = (float*)calloc(r->sizes[i], sizeof(float));
            }
            r->decay = 0.45f;
            r->mix = 0.25f;
            state = r;
            break;
        }
        case AudioEffectType::PITCHSHIFTER: {
            PitchShiftState* p = (PitchShiftState*)calloc(1, sizeof(PitchShiftState));
            if (!p) break;
            p->ratio = 1.0f;
            state = p;
            break;
        }
        case AudioEffectType::FLANGE: {
            FlangeState* f = (FlangeState*)calloc(1, sizeof(FlangeState));
            if (!f) break;
            f->size = 2048;
            f->buffer = (float*)calloc(f->size, sizeof(float));
            f->rate = 0.0015f;
            f->depthSamples = 256.0f;
            f->feedback = 0.35f;
            f->mix = 0.5f;
            state = f;
            break;
        }
        default: break;
    }
    return state;
}

// ============================
// Process mapping
// ============================
static AudioEffectProcess AudioEffect_GetProcess(AudioEffectType type) {
    switch(type) {
        case AudioEffectType::LOWPASS:     return AudioEffectProcess_Lowpass;
        case AudioEffectType::HIGHTPASS:   return AudioEffectProcess_Highpass;
        case AudioEffectType::NORMALIZE:   return AudioEffectProcess_Normalize;
        case AudioEffectType::DISTORTION:  return AudioEffectProcess_Distortion;
        case AudioEffectType::CHORUS:      return AudioEffectProcess_Chorus;
        case AudioEffectType::ECHO:        return AudioEffectProcess_Delay;
        case AudioEffectType::DELAY:       return AudioEffectProcess_Delay;
        case AudioEffectType::FLANGE:      return AudioEffectProcess_Flange;
        case AudioEffectType::PARAMEQ:     return AudioEffectProcess_ParamEq;
        case AudioEffectType::REVERB:      return AudioEffectProcess_Reverb;
        case AudioEffectType::COMPRESSOR:  return AudioEffectProcess_Compressor;
        case AudioEffectType::PITCHSHIFTER:return AudioEffectProcess_PitchShifter;
        default:                           return NULL;
    }
}

// ============================
// Public helpers
// ============================
AudioEffect* AudioEffect_Create(AudioEffectType type) {
    AudioEffect* fx = (AudioEffect*)malloc(sizeof(AudioEffect));
    if (!fx) return NULL;

    fx->type = type;
    fx->process = AudioEffect_GetProcess(type);
    fx->state = AudioEffect_AllocateState(type);
    fx->next = NULL;

    return fx;
}

void AudioEffect_Destroy(AudioEffect* fx) {
    if (!fx) return;

    if (fx->state) {
        switch(fx->type) {
            case AudioEffectType::DELAY:
            case AudioEffectType::ECHO: {
                DelayState* d = (DelayState*)fx->state;
                free(d->buffer);
                break;
            }
            default: break;
        }
        free(fx->state);
    }

    free(fx);
}

// ============================
// Parameter API
// ============================
void Moss_AudioEffectSetParameter(AudioEffect* fx, const char* paramName, float value) {
    if (!fx || !fx->state || !paramName) return;

    switch(fx->type) {
        case AudioEffectType::NORMALIZE:
        case AudioEffectType::DISTORTION:
            if (Moss_AudioParamEquals(paramName, "gain")) *(float*)fx->state = Moss_AudioClamp(value, 0.0f, 32.0f);
            break;
        case AudioEffectType::CHORUS:
            if (Moss_AudioParamEquals(paramName, "depth")) ((ChorusState*)fx->state)->depth = Moss_AudioClamp(value, 0.0f, 0.25f);
            break;
        case AudioEffectType::DELAY:
        case AudioEffectType::ECHO:
            if (Moss_AudioParamEquals(paramName, "feedback")) ((DelayState*)fx->state)->feedback = Moss_AudioClamp(value, 0.0f, 0.95f);
            else if (Moss_AudioParamEquals(paramName, "mix")) ((DelayState*)fx->state)->mix = Moss_AudioClamp(value, 0.0f, 1.0f);
            break;
        case AudioEffectType::PARAMEQ: {
            ParamEqState* eq = (ParamEqState*)fx->state;
            if (Moss_AudioParamEquals(paramName, "frequency")) eq->frequency = Moss_AudioClamp(value, 20.0f, eq->sampleRate * 0.45f);
            else if (Moss_AudioParamEquals(paramName, "gain_db")) eq->gainDb = Moss_AudioClamp(value, -24.0f, 24.0f);
            else if (Moss_AudioParamEquals(paramName, "q")) eq->q = Moss_AudioClamp(value, 0.1f, 18.0f);
            else if (Moss_AudioParamEquals(paramName, "sample_rate")) eq->sampleRate = Moss_AudioClamp(value, 8000.0f, 384000.0f);
            break;
        }
        case AudioEffectType::COMPRESSOR: {
            CompressorState* c = (CompressorState*)fx->state;
            if (Moss_AudioParamEquals(paramName, "threshold_db")) c->thresholdDb = Moss_AudioClamp(value, -80.0f, 0.0f);
            else if (Moss_AudioParamEquals(paramName, "ratio")) c->ratio = Moss_AudioClamp(value, 1.0f, 40.0f);
            else if (Moss_AudioParamEquals(paramName, "attack")) c->attack = Moss_AudioClamp(value, 0.0f, 0.999f);
            else if (Moss_AudioParamEquals(paramName, "release")) c->release = Moss_AudioClamp(value, 0.0f, 0.9999f);
            else if (Moss_AudioParamEquals(paramName, "makeup_gain")) c->makeupGain = Moss_AudioClamp(value, 0.0f, 16.0f);
            break;
        }
        case AudioEffectType::REVERB: {
            ReverbState* r = (ReverbState*)fx->state;
            if (Moss_AudioParamEquals(paramName, "decay")) r->decay = Moss_AudioClamp(value, 0.0f, 0.98f);
            else if (Moss_AudioParamEquals(paramName, "mix")) r->mix = Moss_AudioClamp(value, 0.0f, 1.0f);
            break;
        }
        case AudioEffectType::PITCHSHIFTER:
            if (Moss_AudioParamEquals(paramName, "ratio")) ((PitchShiftState*)fx->state)->ratio = Moss_AudioClamp(value, 0.5f, 2.0f);
            break;
        case AudioEffectType::FLANGE: {
            FlangeState* f = (FlangeState*)fx->state;
            if (Moss_AudioParamEquals(paramName, "rate")) f->rate = Moss_AudioClamp(value, 0.0f, 0.1f);
            else if (Moss_AudioParamEquals(paramName, "depth_samples")) f->depthSamples = Moss_AudioClamp(value, 1.0f, (float)(f->size - 1));
            else if (Moss_AudioParamEquals(paramName, "feedback")) f->feedback = Moss_AudioClamp(value, -0.95f, 0.95f);
            else if (Moss_AudioParamEquals(paramName, "mix")) f->mix = Moss_AudioClamp(value, 0.0f, 1.0f);
            break;
        }
        default: break;
    }
}

// ============================================
// RayListener2D
// ============================================
struct Ray2D {
    Vec2 origin;
    Vec2 dir; // normalized
};

struct Wall {
    Vec2 a, b;
};

struct Listener2D {
    Vec2 position;
    Vec2 forward; // normalized
};

struct SoundSource2D {
    Vec2 position;
    float baseVolume;
};

struct AudioResult {
    float left;
    float right;
};
struct ReflectionHit2D {
    Vec2 position;
    Vec2 direction;
    float attenuation;
};

// Helpers
// 
inline float DistanceAttenuation(float distance) { return 1.0f / (1.0f + distance * 0.2f); }
// 
inline Vec2 Right(const Vec2& forward) { return Vec2{ forward.y, -forward.x }; }
//
inline Vec2 WallNormal(const Wall& w) { Vec2 edge = w.b - w.a; return Vec2{ -edge.y, edge.x }.Normalized(); // perpendicular
}
//
inline Vec2 Reflect(const Vec2& dir, const Vec2& normal) { return dir - 2.0f * dir.Dot(normal) * normal; }

inline bool RayIntersectsSegment(const Ray2D& ray, const Wall& seg, float& outT) {
    Vec2 v1 = ray.origin - seg.a;
    Vec2 v2 = seg.b - seg.a;
    Vec2 v3 = Vec2{ -ray.dir.y, ray.dir.x }; // perpendicular

    float dot = v2.Dot(v3);
    if (fabs(dot) < 1e-6f) return false;

    float t1 = v2.Cross(v1) / dot;
    float t2 = v1.Dot(v3) / dot;

    if (t1 >= 0.0f && t2 >= 0.0f && t2 <= 1.0f)
    {
        outT = t1;
        return true;
    }
    return false;
}

inline bool IsOccluded(const SoundSource2D& src, const Listener2D& listener, const std::vector<Wall>& walls) {
    Vec2 dir = (listener.position - src.position).Normalized();
    Ray2D ray{ src.position, dir };
    float maxDist = (listener.position - src.position).Length();

    for (const Wall& w : walls)
    {
        float t;
        if (RayIntersectsSegment(ray, w, t))
        {
            if (t < maxDist)
                return true;
        }
    }
    return false;
};

inline float ComputePan(const SoundSource2D& src, const Listener2D& listener) {
    Vec2 toSound = (src.position - listener.position).Normalized();
    Vec2 right = Right(listener.forward);

    float pan = toSound.Dot(right);
    return std::clamp(pan, -1.0f, 1.0f);
}

void CastMultiBounceRays2D(const SoundSource2D& src, const Listener2D& listener, const std::vector<Wall>& walls, std::vector<ReflectionHit2D>& outReflections, int maxBounces=3, float attenuation=0.3f)
{
    struct RayBounce { Vec2 origin, dir; float gain; int depth; };
    std::vector<RayBounce> queue;
    queue.push_back({src.position, (listener.position - src.position).Normalized(), 1.0f, 0});

    while(!queue.empty())
    {
        RayBounce ray = queue.back(); queue.pop_back();
        float nearestT = FLT_MAX;
        Wall nearestWall;
        bool hitWall=false;

        for(const Wall& w: walls)
        {
            float t;
            if(RayIntersectsSegment({ray.origin, ray.dir}, w, t) && t<nearestT)
            {
                nearestT=t;
                nearestWall=w;
                hitWall=true;
            }
        }

        if(!hitWall) continue;

        Vec2 hitPos = ray.origin + ray.dir*nearestT;
        Vec2 normal = WallNormal(nearestWall);
        Vec2 reflectedDir = Reflect(ray.dir, normal);

        if(ray.depth < maxBounces)
            queue.push_back({hitPos + reflectedDir*0.01f, reflectedDir, ray.gain*attenuation, ray.depth+1});

        // Only accept reflections reaching listener (approx)
        if((listener.position - hitPos).Length() < 0.5f)
            outReflections.push_back({hitPos, reflectedDir, ray.gain});
    }
}


/*! */
void CastReflectionRays(const PhysicsSystem& physics, const SoundSource3D& src, const Listener3D& listener, std::vector<ReflectionHit>& outReflections) {
    RayCastResult hit;
    if (!CastAudioRay(physics, src.position, listener.position, hit)) return;

    Vec3 hitPos = src.position + (listener.position - src.position) * hit.mFraction;
    Vec3 normal = physics.GetBodyInterface().GetTransformedShape(hit.mBodyID).GetSurfaceNormal(hit.mSubShapeID2, hitPos);
    Vec3 inDir = (listener.position - src.position).Normalized();
    Vec3 reflected = Reflect(inDir, normal);

    RayCastResult bounce;
    Vec3 bounceEnd = hitPos + reflected * 50.0f;
    if (!CastAudioRay(physics, hitPos + reflected * 0.01f, bounceEnd, bounce)) return;

    outReflections.push_back({ hitPos, reflected, 0.3f });
}

inline void CastReflectionRays(const SoundSource2D& src, const Listener2D& listener, const std::vector<Wall>& walls, std::vector<ReflectionHit2D>& outReflections) {
    Vec2 dir = (listener.position - src.position).Normalized();
    Ray2D directRay{ src.position, dir };
    float maxDist = (listener.position - src.position).Length();

    for (const Wall& w : walls)
    {
        float t;
        if (!RayIntersectsSegment(directRay, w, t))
            continue;

        if (t >= maxDist)
            continue;

        Vec2 hitPos = directRay.origin + dir * t;
        Vec2 normal = WallNormal(w);

        Vec2 reflectedDir = Reflect(dir, normal);

        // Cast reflection toward listener
        Ray2D reflRay{ hitPos + reflectedDir * 0.01f, reflectedDir };
        float tRefl = 0.0f;
        for (const Wall& w2 : walls) {
            if (RayIntersectsSegment(reflRay, w2, tRefl))
            {
                // Reflection hits another wall before listener, skip
                if (tRefl < (listener.position - hitPos).Length())
                    reflectedDir = Vec2{0,0};
            }
        }

        if (reflectedDir != Vec2{0,0}) {
            outReflections.push_back({ hitPos, reflectedDir, 0.3f });
        }
    }
}

inline void Apply2DHRTF(const Vec2& dir, const Listener2D& listener, float volume, float* outL, float* outR) {
    float pan = ComputePan({ listener.position + dir, 0.0f }, listener); // simulate direction
    float angle = (pan + 1.0f) * 0.25f * MOSS_PI;

    *outL += volume * std::cos(angle);
    *outR += volume * std::sin(angle);
}

void ApplyLateReverb2D(float* outL, float* outR, int samples, float decay=0.5f)
{
    static std::vector<float> delayL(44100,0.0f), delayR(44100,0.0f);
    int delaySamples = 4410;
    for(int i=0;i<samples;i++)
    {
        float dl = delayL[i%delayL.size()]*decay;
        float dr = delayR[i%delayR.size()]*decay;
        outL[i]+=dl; outR[i]+=dr;
        delayL[i%delayL.size()]=outL[i];
        delayR[i%delayR.size()]=outR[i];
    }
}

void SmoothAudio2D(float* current, float* previous, int samples, float alpha=0.2f)
{
    for(int i=0;i<samples;i++)
        current[i] = previous[i]*(1.0f-alpha) + current[i]*alpha;
}


void ProcessSound2D(const SoundSource2D& src, const Listener2D& listener, const std::vector<Wall>& walls, const float* monoInput, int samples, float* outL, float* outR)
{
    float distance = (listener.position - src.position).Length();
    float volume = src.baseVolume * DistanceAttenuation(distance);

    if(IsOccluded(src, listener, walls)) { volume *= 0.3f; }

    // Direct sound
    Apply2DHRTF((listener.position - src.position).Normalized(), listener, volume, outL, outR);

    // Reflections
    std::vector<ReflectionHit2D> reflections;
    CastMultiBounceRays2D(src, listener, walls, reflections, 3, 0.3f);

    for(auto& r: reflections)
    {
        float reflDist = (listener.position - r.position).Length();
        float reflVolume = volume * r.attenuation * DistanceAttenuation(reflDist);
        Apply2DHRTF(r.direction, listener, reflVolume, outL, outR);
    }

    // Late reverb
    ApplyLateReverb2D(outL, outR, samples);
}

inline AudioResult Compute2DAudioWithReflections(const SoundSource2D& src, const Listener2D& listener, const std::vector<Wall>& walls) {
    AudioResult result{0,0};

    float dist = (listener.position - src.position).Length();
    float volume = src.baseVolume * DistanceAttenuation(dist);

    if (IsOccluded(src, listener, walls)) { volume *= 0.3f; }

    // Direct sound
    Apply2DHRTF((listener.position - src.position).Normalized(), listener, volume, &result.left, &result.right);

    // Reflections
    std::vector<ReflectionHit2D> reflections;
    CastReflectionRays(src, listener, walls, reflections);

    for (auto& r : reflections) {
        float reflDist = (listener.position - r.position).Length();
        float reflVolume = volume * r.attenuation * DistanceAttenuation(reflDist);
        Apply2DHRTF(r.direction, listener, reflVolume, &result.left, &result.right);
    }

    return result;
}


// ============================================
// RayListener3D
// ============================================
struct Triangle {
    Vec3 a, b, c;
};

struct Listener3D {
    Vec3 position;
    Vec3 forward;
    Vec3 up;
};

struct SoundSource3D {
    Vec3 position;
    float baseVolume;
};

struct AudioResult
{
    float left;
    float right;
};

struct ReflectionHit {
    Vec3 position;
    Vec3 direction;
    float attenuation;
};
struct Angles {
    float azimuth;
    float elevation;
};
struct HRTF {
    std::vector<float> left;
    std::vector<float> right;
};

/*! */
inline Vec3 Right(const Listener3D& l) { return l.forward.Cross(l.up).Normalized(); }
/*! */
inline Vec3 Reflect(const Vec3& dir, const Vec3& normal) { return dir - 2.0f * dir.Dot(normal) * normal; }
/*! */
inline float NormalizeAzimuth(float az) { while (az < -180.0f) az += 360.0f; while (az > 180.0f) az -= 360.0f; return az; }
/*! */
inline float ClampElevation(float el) { return std::clamp(el, -45.0f, 90.0f); }
/*! */
inline int Quantize(float angle, int step) { return int(std::round(angle / step) * step); }
/*! */
bool RayIntersectsTriangle(const Vec3& rayOrigin, const Vec3& rayDir, const Triangle& tri, float& outT) {
    constexpr float EPS = 1e-6f;
    Vec3 edge1 = tri.b - tri.a;
    Vec3 edge2 = tri.c - tri.a;

    Vec3 pvec = rayDir.Cross(edge2);
    float det = edge1.Dot(pvec);
    if (fabs(det) < EPS) return false;

    float invDet = 1.0f / det;
    Vec3 tvec = rayOrigin - tri.a;
    float u = tvec.Dot(pvec) * invDet;
    if (u < 0.0f || u > 1.0f) return false;

    Vec3 qvec = tvec.Cross(edge1);
    float v = rayDir.Dot(qvec) * invDet;
    if (v < 0.0f || u + v > 1.0f) return false;

    float t = edge2.Dot(qvec) * invDet;
    if (t > EPS) { outT = t; return true; }
    return false;
}
/*! */
bool IsOccluded(const SoundSource3D& src, const Listener3D& listener, const std::vector<Triangle>& world) {
    Vec3 toListener = listener.position - src.position;
    float maxDist = toListener.Length();
    if (maxDist < 0.001f) return false;

    Vec3 dir = toListener.Normalized();
    constexpr float RayBias = 0.01f;
    Vec3 origin = src.position + dir * RayBias;

    for (const Triangle& tri : world) {
        float t;
        if (RayIntersectsTriangle(origin, dir, tri, t)) {
            if (t > RayBias && t < maxDist - RayBias) return true;
        }
    }
    return false;
}
/*! */
float ComputePan(const SoundSource3D& src, const Listener3D& listener) {
    Vec3 toSound = (src.position - listener.position).Normalized();
    Vec3 right = Right(listener);
    return std::clamp(toSound.Dot(right), -1.0f, 1.0f);
}
/*! */
float ComputeFrontBack(const SoundSource3D& src, const Listener3D& listener) {
    Vec3 toSound = (src.position - listener.position).Normalized();
    return std::clamp(toSound.Dot(listener.forward), -1.0f, 1.0f);
}
/*! */
inline float ApplyFrontBackAttenuation(float volume, float fb) { if (fb < 0.0f) volume *= 0.7f; return volume; }
/*! */
inline float ApplyBackStereoNarrowing(float pan, float fb) { if (fb < 0.0f) pan *= 0.5f; return pan; }
/*! */
float ComputeDoppler(const Vec3& srcVel, const Vec3& listenerVel, const SoundSource3D& src, const Listener3D& listener) {
    Vec3 dir = (src.position - listener.position).Normalized();
    float vls = std::clamp(listenerVel.Dot(dir), -300.0f, 300.0f);
    float vss = std::clamp(srcVel.Dot(dir), -300.0f, 300.0f);
    constexpr float speedOfSound = 343.0f;
    return std::clamp((speedOfSound + vls) / (speedOfSound + vss), 0.5f, 2.0f);
}

/*! */
bool CastAudioRay(const PhysicsSystem& physics, const Vec3& from, const Vec3& to, RayCastResult& outHit) {
    RRayCast ray(from, to - from);
    return physics.GetNarrowPhaseQuery().CastRay(ray, outHit);
}
/*! */
float ComputeOcclusion(const PhysicsSystem& physics, const SoundSource3D& src, const Listener3D& listener) {
    RayCastResult hit;
    if (CastAudioRay(physics, src.position, listener.position, hit))
        if (hit.mFraction < 1.0f) return 0.25f;
    return 1.0f;
}

/*! */
Angles ComputeHRTFAngles(const Vec3& dir, const Listener3D& listener) {
    Vec3 right = listener.forward.Cross(listener.up).Normalized();
    float azimuth = atan2f(dir.Dot(right), dir.Dot(listener.forward));
    float elevation = asinf(dir.Dot(listener.up));
    return { azimuth * 180.0f / MOSS_PI, elevation * 180.0f / MOSS_PI };
}
/*! */
const HRTF& GetHRTF(float azimuthDeg, float elevationDeg) {
    static HRTF dummy;
    azimuthDeg  = NormalizeAzimuth(azimuthDeg);
    elevationDeg = ClampElevation(elevationDeg);
    constexpr int AZ_STEP = 5, EL_STEP = 5;
    int az = Quantize(azimuthDeg, AZ_STEP);
    int el = Quantize(elevationDeg, EL_STEP);

    auto elIt = gHRTFs.find(el);
    if (elIt == gHRTFs.end()) return dummy;
    auto azIt = elIt->second.find(az);
    return azIt != elIt->second.end() ? azIt->second : dummy;
}

/*! */
void Convolve(const float* input, int samples, const std::vector<float>& ir, float* output) {
    for (int i = 0; i < samples; ++i) {
        output[i] = 0.0f;
        for (int k = 0; k < (int)ir.size(); ++k)
            if (i - k >= 0) output[i] += input[i - k] * ir[k];
    }
}
/*! */
void ApplyHRTF(const float* mono, int samples, const HRTF& hrtf, float* outL, float* outR, float gain) {
    std::vector<float> l(samples), r(samples);
    Convolve(mono, samples, hrtf.left,  l.data());
    Convolve(mono, samples, hrtf.right, r.data());
    for (int i = 0; i < samples; ++i) {
        outL[i] += l[i] * gain;
        outR[i] += r[i] * gain;
    }
}

/*! */
void ProcessSoundDirect(const PhysicsSystem& physics, const SoundSource3D& src, const Listener3D& listener, const Vec3& srcVel, const Vec3& listenerVel, 
    const float* monoInput, int samples, float* outL, float* outR, float& leftGain, float& rightGain) {
    // Reset output buffers
    std::fill(outL, outL + samples, 0.0f);
    std::fill(outR, outR + samples, 0.0f);

    float distance = (listener.position - src.position).Length();
    float volume = src.baseVolume / (1.0f + distance * distance * 0.05f);

    // Apply occlusion
    volume *= ComputeOcclusion(physics, src, listener);

    // Doppler factor (optional: could resample)
    float doppler = ComputeDoppler(srcVel, listenerVel, src, listener);

    // --- Direct sound ---
    Vec3 dir = (src.position - listener.position).Normalized();
    Angles a = ComputeHRTFAngles(dir, listener);
    const HRTF& hrtfDirect = GetHRTF(a.azimuth, a.elevation);
    ApplyHRTF(monoInput, samples, hrtfDirect, outL, outR, volume);

    // --- Reflections ---
    std::vector<ReflectionHit> reflections;
    CastReflectionRays(physics, src, listener, reflections);

    for (auto& r : reflections) {
        Angles ra = ComputeHRTFAngles(r.direction, listener);
        const HRTF& hrtfRef = GetHRTF(ra.azimuth, ra.elevation);
        ApplyHRTF(monoInput, samples, hrtfRef, outL, outR, volume * r.attenuation);
    }

    // Compute final gains for matrix
    // Take first sample as representative gain
    leftGain  = samples > 0 ? outL[0] : volume * 0.707f;
    rightGain = samples > 0 ? outR[0] : volume * 0.707f;
}

/* RayTrace is lacking 
Multi-bounce reflections
Late reverberation / decay
Frequency-dependent occlusion and diffraction
Temporal smoothing and optimized multi-source mixing
*/





void CastMultiBounce2DRays(const SoundSource2D& src, const Listener2D& listener, const std::vector<Wall>& walls, std::vector<ReflectionHit2D>& outReflections, int maxBounces = 3, float attenuation = 0.3f) {
    struct RayBounce {
        Vec2 origin;
        Vec2 dir;
        float gain;
        int depth;
    };

    std::vector<RayBounce> queue;
    queue.push_back({src.position, (listener.position - src.position).Normalized(), 1.0f, 0});

    while (!queue.empty()) {
        RayBounce ray = queue.back();
        queue.pop_back();

        float closestT = FLT_MAX;
        Wall hitWall{};
        bool hit = false;

        for (const Wall& w : walls) {
            float t;
            if (RayIntersectsSegment({ray.origin, ray.dir}, w, t)) {
                if (t < closestT) { closestT = t; hitWall = w; hit = true; }
            }
        }

        Vec2 hitPos = ray.origin + ray.dir * closestT;

        // Check if hits listener
        if ((listener.position - ray.origin).Dot(ray.dir) > 0.0f &&
            (listener.position - ray.origin).Length() <= closestT)
        {
            outReflections.push_back({hitPos, ray.dir, ray.gain});
        }

        if (hit && ray.depth < maxBounces) {
            Vec2 normal = WallNormal(hitWall);
            Vec2 reflDir = Reflect(ray.dir, normal);
            queue.push_back({hitPos + reflDir * 0.01f, reflDir, ray.gain * attenuation, ray.depth + 1});
        }
    }
}


// Late reverb
void ApplyLateReverb(float* outL, float* outR, int samples, float decay = 0.5f) {
    static std::vector<float> delayL(44100, 0.0f); // 1s buffer
    static std::vector<float> delayR(44100, 0.0f);
    int delaySamples = 4410; // 100ms delay

    for (int i = 0; i < samples; ++i) {
        float delayedL = delayL[i % delayL.size()] * decay;
        float delayedR = delayR[i % delayR.size()] * decay;

        outL[i] += delayedL;
        outR[i] += delayedR;

        delayL[i % delayL.size()] = outL[i];
        delayR[i % delayR.size()] = outR[i];
    }
}
// Temporal smoothing
void SmoothAudio(float* current, float* previous, int samples, float alpha=0.2f)
{
    for(int i=0;i<samples;i++)
        current[i] = previous[i]*(1.0f-alpha) + current[i]*alpha;
}

void ApplyFrequencyOcclusion(float* outL, float* outR, int samples, float cutoff = 2000.0f, float sampleRate = 44100.0f) {
    float RC = 1.0f / (2.0f * 3.14159f * cutoff);
    float dt = 1.0f / sampleRate;
    float alpha = dt / (RC + dt);

    float prevL = 0, prevR = 0;
    for (int i = 0; i < samples; ++i) {
        prevL = prevL + alpha * (outL[i] - prevL);
        prevR = prevR + alpha * (outR[i] - prevR);
        outL[i] = prevL;
        outR[i] = prevR;
    }
}


void ProcessSound(const PhysicsSystem& physics, const SoundSource3D& src, const Listener3D& listener, const Vec3& srcVel, const Vec3& listenerVel, const float* monoInput, int samples,
    float* outL, float* outR) {
    float distance = (listener.position - src.position).Length();
    float volume = src.baseVolume * DistanceAttenuation(distance);
    volume *= ComputeOcclusion(physics, src, listener);

    float doppler = ComputeDoppler(srcVel, listenerVel, src, listener);

    // Direct sound
    Angles a = ComputeHRTFAngles((src.position-listener.position).Normalized(), listener);
    const HRTF& hrtf = GetHRTF(a.azimuth, a.elevation);
    ApplyHRTF(monoInput, samples, hrtf, outL, outR, volume);

    // Multi-bounce reflections
    std::vector<ReflectionHit> reflections;
    CastMultiBounceRays(physics, src, listener, reflections, 3, 0.3f);
    for(auto& r: reflections) {
        Angles ra = ComputeHRTFAngles(r.direction, listener);
        const HRTF& rh = GetHRTF(ra.azimuth, ra.elevation);
        ApplyHRTF(monoInput, samples, rh, outL, outR, volume * r.attenuation);
    }

    // Late reverb
    ApplyLateReverb(outL, outR, samples);

    // Temporal smoothing (optional, requires previous buffer)
}



















































/*

used for culling
for (auto& src : allSources) {
    if ((src.position - listener.position).Length() > maxAudibleDistance) continue;
    // process early reflections & HRTF
}
*/







/*
struct Moss_SharedWavSourceData {
    Wav* wav = nullptr;
    uint32_t byte_offset = 0;
};

static bool Moss_AudioFileLooksLikeWaveShared(const char* filename) {
    if (!filename) {
        return false;
    }
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        return false;
    }
    char header[12]{};
    file.read(header, sizeof(header));
    return file.gcount() == sizeof(header) &&
           std::memcmp(header, "RIFF", 4) == 0 &&
           std::memcmp(header + 8, "WAVE", 4) == 0;
}

static uint32_t Moss_AudioSourceReadWavShared(Moss_AudioSource* src, float* out_samples, uint32_t frames) {
    if (!src || !src->userdata || !out_samples || frames == 0) {
        return 0;
    }

    Moss_SharedWavSourceData* data = static_cast<Moss_SharedWavSourceData*>(src->userdata);
    Wav* wav = data->wav;
    if (!wav || !wav->dataBegin || wav->numChannels == 0 || wav->bitsPerSample == 0) {
        return 0;
    }

    const uint32_t bytes_per_sample = wav->bitsPerSample / 8u;
    const uint32_t frame_bytes = bytes_per_sample * wav->numChannels;
    if (frame_bytes == 0 || data->byte_offset >= wav->dataChunkSize) {
        return 0;
    }

    const uint32_t available_frames = (wav->dataChunkSize - data->byte_offset) / frame_bytes;
    const uint32_t frames_to_read = frames < available_frames ? frames : available_frames;
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(wav->dataBegin + data->byte_offset);

    for (uint32_t frame = 0; frame < frames_to_read; ++frame) {
        for (uint32_t channel = 0; channel < wav->numChannels; ++channel) {
            const uint8_t* sample = bytes + frame * frame_bytes + channel * bytes_per_sample;
            float value = 0.0f;
            if (wav->audioFormat == 3 && wav->bitsPerSample == 32) {
                std::memcpy(&value, sample, sizeof(float));
            } else if (wav->bitsPerSample == 8) {
                value = (static_cast<int>(*sample) - 128) / 128.0f;
            } else if (wav->bitsPerSample == 16) {
                int16_t s = 0;
                std::memcpy(&s, sample, sizeof(s));
                value = static_cast<float>(s) / 32768.0f;
            } else if (wav->bitsPerSample == 24) {
                int32_t s = (static_cast<int32_t>(sample[0]) |
                            (static_cast<int32_t>(sample[1]) << 8) |
                            (static_cast<int32_t>(sample[2]) << 16));
                if (s & 0x00800000) {
                    s |= 0xFF000000;
                }
                value = static_cast<float>(s) / 8388608.0f;
            } else if (wav->bitsPerSample == 32) {
                int32_t s = 0;
                std::memcpy(&s, sample, sizeof(s));
                value = static_cast<float>(s) / 2147483648.0f;
            }
            out_samples[frame * wav->numChannels + channel] = value;
        }
    }

    data->byte_offset += frames_to_read * frame_bytes;
    return frames_to_read;
}

static void Moss_AudioSourceResetWavShared(Moss_AudioSource* src) {
    if (!src || !src->userdata) {
        return;
    }
    static_cast<Moss_SharedWavSourceData*>(src->userdata)->byte_offset = 0;
}

static void Moss_AudioSourceDestroyWavShared(Moss_AudioSource* src) {
    if (!src) {
        return;
    }
    Moss_SharedWavSourceData* data = static_cast<Moss_SharedWavSourceData*>(src->userdata);
    if (data) {
        RemoveWav(data->wav);
        delete data;
    }
    std::free(src);
}

struct Moss_DecodedAudioSourceData {
    std::vector<float> samples;
    uint32_t channels = 0;
    uint32_t sample_rate = 0;
    uint32_t frame_offset = 0;
};

struct Moss_RegisteredAudioDecoder {
    Moss_AudioEncodedFormat format = Moss_AudioEncodedFormat::WAV;
    Moss_AudioDecodeCallback callback = nullptr;
    void* userdata = nullptr;
};

static Moss_RegisteredAudioDecoder g_audio_decoders[8];

static bool Moss_AudioRegisterDecoderShared(Moss_AudioEncodedFormat format, Moss_AudioDecodeCallback callback, void* userdata) {
    if (!callback) {
        return false;
    }

    for (Moss_RegisteredAudioDecoder& decoder : g_audio_decoders) {
        if (decoder.callback && decoder.format == format) {
            decoder.callback = callback;
            decoder.userdata = userdata;
            return true;
        }
    }

    for (Moss_RegisteredAudioDecoder& decoder : g_audio_decoders) {
        if (!decoder.callback) {
            decoder.format = format;
            decoder.callback = callback;
            decoder.userdata = userdata;
            return true;
        }
    }

    return false;
}

static uint32_t Moss_AudioSourceReadDecodedShared(Moss_AudioSource* src, float* out_samples, uint32_t frames) {
    if (!src || !src->userdata || !out_samples || frames == 0) {
        return 0;
    }

    Moss_DecodedAudioSourceData* data = static_cast<Moss_DecodedAudioSourceData*>(src->userdata);
    if (data->channels == 0 || data->frame_offset >= data->samples.size() / data->channels) {
        return 0;
    }

    const uint32_t available_frames = static_cast<uint32_t>(data->samples.size() / data->channels) - data->frame_offset;
    const uint32_t frames_to_read = frames < available_frames ? frames : available_frames;
    const size_t sample_offset = static_cast<size_t>(data->frame_offset) * data->channels;
    const size_t sample_count = static_cast<size_t>(frames_to_read) * data->channels;
    std::memcpy(out_samples, data->samples.data() + sample_offset, sample_count * sizeof(float));
    data->frame_offset += frames_to_read;
    return frames_to_read;
}

static void Moss_AudioSourceResetDecodedShared(Moss_AudioSource* src) {
    if (src && src->userdata) {
        static_cast<Moss_DecodedAudioSourceData*>(src->userdata)->frame_offset = 0;
    }
}

static void Moss_AudioSourceDestroyDecodedShared(Moss_AudioSource* src) {
    if (!src) {
        return;
    }
    delete static_cast<Moss_DecodedAudioSourceData*>(src->userdata);
    std::free(src);
}

static Moss_AudioSource* Moss_AudioSourceCreateDecodedShared(const Moss_AudioDecodedData& decoded) {
    if (!decoded.samples || decoded.frame_count == 0 || decoded.channels == 0) {
        return nullptr;
    }

    Moss_AudioSource* source = static_cast<Moss_AudioSource*>(std::calloc(1, sizeof(Moss_AudioSource)));
    Moss_DecodedAudioSourceData* data = new (std::nothrow) Moss_DecodedAudioSourceData();
    if (!source || !data) {
        delete data;
        std::free(source);
        return nullptr;
    }

    const size_t sample_count = static_cast<size_t>(decoded.frame_count) * decoded.channels;
    data->samples.assign(decoded.samples, decoded.samples + sample_count);
    data->channels = decoded.channels;
    data->sample_rate = decoded.sample_rate;

    source->type = MOSS_AUDIO_SOURCE_CUSTOM;
    source->userdata = data;
    source->read = Moss_AudioSourceReadDecodedShared;
    source->reset = Moss_AudioSourceResetDecodedShared;
    source->destroy = Moss_AudioSourceDestroyDecodedShared;
    return source;
}

static Moss_AudioSource* Moss_AudioDecodeWithRegisteredDecoderShared(Moss_AudioEncodedFormat format, const char* filename, AudioLoadType type) {
    if (!filename || !filename[0]) {
        return nullptr;
    }

    for (const Moss_RegisteredAudioDecoder& decoder : g_audio_decoders) {
        if (!decoder.callback || decoder.format != format) {
            continue;
        }

        Moss_AudioDecodedData decoded{};
        if (!decoder.callback(filename, type, &decoded, decoder.userdata)) {
            continue;
        }

        Moss_AudioSource* source = Moss_AudioSourceCreateDecodedShared(decoded);
        if (decoded.free_samples) {
            decoded.free_samples(decoded.userdata, decoded.samples);
        }
        if (source) {
            return source;
        }
    }

    return nullptr;
}
static Moss_AudioSource* Moss_AudioLoadWaveSourceShared(const char* filename) {
    if (!Moss_AudioFileLooksLikeWaveShared(filename)) {
        return nullptr;
    }

    Wav* wav = CreateWav(filename);
    if (!wav) {
        return nullptr;
    }

    Moss_AudioSource* source = static_cast<Moss_AudioSource*>(std::calloc(1, sizeof(Moss_AudioSource)));
    Moss_SharedWavSourceData* data = new (std::nothrow) Moss_SharedWavSourceData();
    if (!source || !data) {
        RemoveWav(wav);
        delete data;
        std::free(source);
        return nullptr;
    }

    data->wav = wav;
    source->type = MOSS_AUDIO_SOURCE_WAV;
    source->userdata = data;
    source->read = Moss_AudioSourceReadWavShared;
    source->reset = Moss_AudioSourceResetWavShared;
    source->destroy = Moss_AudioSourceDestroyWavShared;
    return source;
}

static void AudioEffect_Init(AudioEffect* fx, AudioEffectType type) {
    if (!fx) return;
    fx->type = type;
    fx->process = AudioEffect_GetProcess(type);
    fx->state = AudioEffect_AllocateState(type);
    fx->next = NULL;
}

static void AudioEffect_FreeState(AudioEffect* fx) {
    if (!fx || !fx->state) return;
    switch(fx->type) {
        case AudioEffectType::DELAY:
        case AudioEffectType::ECHO:
            free(((DelayState*)fx->state)->buffer);
            break;
        case AudioEffectType::REVERB: {
            ReverbState* r = (ReverbState*)fx->state;
            for (uint32_t i = 0; i < 4; ++i) free(r->buffers[i]);
            break;
        }
        case AudioEffectType::FLANGE:
            free(((FlangeState*)fx->state)->buffer);
            break;
        default: break;
    }
    free(fx->state);
    fx->state = NULL;
    fx->process = NULL;
    fx->next = NULL;
}

AudioEffect* AudioEffect_Create(AudioEffectType type) {
    AudioEffect* fx = (AudioEffect*)malloc(sizeof(AudioEffect));
    if (!fx) return NULL;
    AudioEffect_Init(fx, type);
    return fx;
}

void AudioEffect_Destroy(AudioEffect* fx) {
    if (!fx) return;
    AudioEffect_FreeState(fx);
    free(fx);
}

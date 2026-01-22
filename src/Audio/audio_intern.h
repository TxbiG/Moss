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

/*
class RayAudioListener2D : public AudioListener2D {

    float occlusion;
    float reflectionGain;
    float reflectionDelay;
    uint32_t rayCount;
    float maxRayDistance;

    PhysicsSystem* physicsScene;
};
class RayAudioListener3D : public AudioListener3D {
    float occlusion;              // 0..1
    float reflectionGain;         // energy from reflections
    float reflectionDelay;        // seconds
    uint32_t rayCount;
    float maxRayDistance;

    PhysicsSystem* physicsScene;           // opaque handle to scene for ray tracing
};
*/


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

// --- Filters ---
static void AudioEffectProcess_Lowpass(float* samples, uint32_t frames, uint32_t channels, void* userdata) {
    float* state = (float*)userdata;
    for (uint32_t i = 0; i < frames * channels; i++) {
        float x = samples[i];
        samples[i] = 0.5f * x + 0.5f * state[0];
        state[0] = samples[i];
    }
}

static void AudioEffectProcess_Highpass(float* samples, uint32_t frames, uint32_t channels, void* userdata) {
    float* state = (float*)userdata;
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
    for (uint32_t i = 0; i < frames * channels; i++) {
        float delayed = state->buffer[state->writePos];
        state->buffer[state->writePos] = samples[i] + delayed * state->feedback;
        samples[i] += delayed;
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
static void AudioEffectProcess_ParamEq(float* samples, uint32_t frames, uint32_t channels, void* userdata) {
    (void)samples; (void)frames; (void)channels; (void)userdata;
    // Advanced: implement filters later
}

// --- Compressor placeholder ---
static void AudioEffectProcess_Compressor(float* samples, uint32_t frames, uint32_t channels, void* userdata) {
    (void)samples; (void)frames; (void)channels; (void)userdata;
    // Advanced: implement threshold/ratio compressor
}

// --- Reverb placeholder ---
static void AudioEffectProcess_Reverb(float* samples, uint32_t frames, uint32_t channels, void* userdata) {
    (void)samples; (void)frames; (void)channels; (void)userdata;
    // Advanced: implement multi-tap delay reverb
}

// --- Pitch Shifter placeholder ---
static void AudioEffectProcess_PitchShifter(float* samples, uint32_t frames, uint32_t channels, void* userdata) {
    (void)samples; (void)frames; (void)channels; (void)userdata;
    // Advanced: implement phase vocoder or granular pitch shift
}

// --- Flange placeholder ---
static void AudioEffectProcess_Flange(float* samples, uint32_t frames, uint32_t channels, void* userdata) {
    (void)samples; (void)frames; (void)channels; (void)userdata;
    // Advanced: implement small delay + LFO
}

// ============================
// State allocation per effect
// ============================
static void* AudioEffect_AllocateState(AudioEffectType type) {
    void* state = NULL;
    switch(type) {
        case AudioEffectType::LOWPASS:
        case AudioEffectType::HIGHTPASS:
            state = calloc(1, sizeof(float));
            break;

        case AudioEffectType::NORMALIZE:
        case AudioEffectType::DISTORTION: {
            state = malloc(sizeof(float));
            *(float*)state = 1.0f;
            break;
        }

        case AudioEffectType::CHORUS:
            state = calloc(1, sizeof(ChorusState));
            ((ChorusState*)state)->depth = 0.05f;
            break;

        case AudioEffectType::DELAY:
        case AudioEffectType::ECHO: {
            DelayState* d = (DelayState*)malloc(sizeof(DelayState));
            d->size = 44100; // 1 second at 44.1kHz
            d->buffer = (float*)calloc(d->size, sizeof(float));
            d->writePos = 0;
            d->feedback = 0.5f;
            state = d;
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
    if (!fx || !fx->state) return;

    switch(fx->type) {
        case AudioEffectType::NORMALIZE:
        case AudioEffectType::DISTORTION:
            if (strcmp(paramName, "gain") == 0)
                *(float*)fx->state = value;
            break;
        case AudioEffectType::CHORUS: {
            if (strcmp(paramName, "depth") == 0)
                ((ChorusState*)fx->state)->depth = value;
            break;
        }
        case AudioEffectType::DELAY:
        case AudioEffectType::ECHO: {
            if (strcmp(paramName, "feedback") == 0)
                ((DelayState*)fx->state)->feedback = value;
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
    float angle = (pan + 1.0f) * 0.25f * JPH_PI;

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
    return { azimuth * 180.0f / JPH_PI, elevation * 180.0f / JPH_PI };
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
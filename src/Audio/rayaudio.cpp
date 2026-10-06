// rayaudio.cpp
#include "audio_intern.h"
#include <Moss/Moss_Physics.h>

#include <algorithm>
#include <cfloat>
#include <cmath>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
constexpr float kSpeedOfSound          = 343.0f;   // m/s
constexpr float kPi                    = 3.14159265358979323846f;
constexpr float kGoldenAngle           = 2.39996322972865332f;
constexpr float kEpsilon               = 0.00001f;
constexpr float kDefaultReflectionRange = 50.0f;   // used when max_distance == 0 (unlimited)

constexpr uint32_t kMaxReflectionRays2D = 64;
constexpr uint32_t kMaxReflectionRays3D = 128;

// Defaults used when tracing from a listener (the listener has no per-trace tuning).
constexpr float kListenerDirectOcclusionStrength = 1.0f;
constexpr float kListenerReflectionStrength      = 1.0f;
constexpr float kListenerAirAbsorption           = 0.0f;

// ---------------------------------------------------------------------------
// Small math helpers
// ---------------------------------------------------------------------------
inline float clamp01(float v) { return std::max(0.0f, std::min(1.0f, v)); }

inline Vec2 sub(const Vec2& a, const Vec2& b) { return Vec2(a.GetX() - b.GetX(), a.GetY() - b.GetY()); }
inline Vec3 sub(const Vec3& a, const Vec3& b) { return Vec3(a.GetX() - b.GetX(), a.GetY() - b.GetY(), a.GetZ() - b.GetZ()); }

inline float length(const Vec2& v) { return std::sqrt(v.GetX() * v.GetX() + v.GetY() * v.GetY()); }
inline float length(const Vec3& v) { return std::sqrt(v.GetX() * v.GetX() + v.GetY() * v.GetY() + v.GetZ() * v.GetZ()); }

inline Vec2 normalized(const Vec2& v, float len) {
    return len <= kEpsilon ? Vec2(1.0f, 0.0f) : Vec2(v.GetX() / len, v.GetY() / len);
}
inline Vec3 normalized(const Vec3& v, float len) {
    return len <= kEpsilon ? Vec3(1.0f, 0.0f, 0.0f) : Vec3(v.GetX() / len, v.GetY() / len, v.GetZ() / len);
}

inline float distance_attenuation(float distance, float max_distance) {
    if (max_distance <= kEpsilon) {
        return 1.0f;
    }
    return clamp01(1.0f - distance / max_distance);
}

// ---------------------------------------------------------------------------
// Dimension traits: everything that differs between 2D and 3D lives here.
// ---------------------------------------------------------------------------
struct Traits2D {
    using Vec  = Vec2;
    using Hit  = Moss_AudioRayHit2D;
    using Desc = Moss_AudioRayTrace2DDesc;
    static constexpr uint32_t kMaxRays = kMaxReflectionRays2D;

    // Evenly spaced directions around a circle.
    static Vec reflection_direction(uint32_t i, uint32_t count) {
        const float a = (static_cast<float>(i) / static_cast<float>(count)) * 2.0f * kPi;
        return Vec(std::cos(a), std::sin(a));
    }
};

struct Traits3D {
    using Vec  = Vec3;
    using Hit  = Moss_AudioRayHit3D;
    using Desc = Moss_AudioRayTrace3DDesc;
    static constexpr uint32_t kMaxRays = kMaxReflectionRays3D;

    // Fibonacci sphere: near-uniform coverage for any ray count.
    static Vec reflection_direction(uint32_t i, uint32_t count) {
        const float u     = (static_cast<float>(i) + 0.5f) / static_cast<float>(count);
        const float z     = 1.0f - 2.0f * u;
        const float r     = std::sqrt(std::max(0.0f, 1.0f - z * z));
        const float theta = kGoldenAngle * static_cast<float>(i);
        return Vec(r * std::cos(theta), r * std::sin(theta), z);
    }
};

// ---------------------------------------------------------------------------
// Result building
// ---------------------------------------------------------------------------
void apply_hit(Moss_AudioRayTraceResult* out, float absorption, float transmission, float strength) {
    out->occluded = true;
    out->occlusion = clamp01(out->occlusion + clamp01(absorption) * std::max(0.0f, strength));
    out->transmission_gain *= clamp01(transmission);
}

void finish_direct_result(Moss_AudioRayTraceResult* out, float distance, float max_distance, float air_absorption) {
    out->distance           = distance;
    out->attenuation        = distance_attenuation(distance, max_distance);
    out->delay_seconds      = distance / kSpeedOfSound;
    out->transmission_gain  = clamp01(out->transmission_gain);
    out->occlusion          = clamp01(out->occlusion);
    out->lowpass            = clamp01(1.0f - out->occlusion * 0.75f - distance * std::max(0.0f, air_absorption) * 0.01f);
    out->audible            = out->attenuation > 0.0f && out->transmission_gain > 0.001f;
}

// ---------------------------------------------------------------------------
// Shared trace implementation
// ---------------------------------------------------------------------------
template <class T>
bool trace_impl(const typename T::Desc* desc, Moss_AudioRayTraceResult* out_result) {
    if (!desc || !out_result) {
        return false;
    }

    Moss_AudioRayTraceResult result{};
    result.transmission_gain = 1.0f;
    result.lowpass = 1.0f;

    const typename T::Vec to_source  = sub(desc->source_position, desc->listener_position);
    const float           distance   = length(to_source);
    const typename T::Vec direction  = normalized(to_source, distance);
    const float trace_distance       = desc->max_distance > 0.0f ? std::min(distance, desc->max_distance) : distance;

    // Direct path: listener -> source.
    if (desc->raycast && trace_distance > 0.0f) {
        typename T::Hit hit{};
        if (desc->raycast(desc->physics, &desc->listener_position, &direction, trace_distance, &hit, desc->user_data) && hit.hit) {
            apply_hit(&result, hit.absorption, hit.transmission, desc->direct_occlusion_strength);
        }
    }

    // Reflections: rays fired outward from the source.
    const uint32_t rays = std::min(desc->reflection_rays, T::kMaxRays);
    if (desc->raycast && rays > 0 && desc->reflection_strength > 0.0f) {
        const float range = desc->max_distance > 0.0f ? desc->max_distance : kDefaultReflectionRange;

        float reflected = 0.0f;
        float nearest   = FLT_MAX;

        for (uint32_t i = 0; i < rays; ++i) {
            const typename T::Vec ray_dir = T::reflection_direction(i, rays);
            typename T::Hit hit{};
            if (!desc->raycast(desc->physics, &desc->source_position, &ray_dir, range, &hit, desc->user_data) || !hit.hit) {
                continue;
            }
            const float hit_distance = clamp01(hit.fraction) * range;
            reflected += (1.0f - clamp01(hit.absorption)) * distance_attenuation(hit_distance + distance, range * 2.0f);
            nearest = std::min(nearest, hit_distance);
        }

        result.reflection_gain = clamp01((reflected / static_cast<float>(rays)) * desc->reflection_strength);
        // Approximation: first bounce path length ~= source->wall + source->listener.
        result.reflection_delay_seconds = nearest < FLT_MAX ? (nearest + distance) / kSpeedOfSound : 0.0f;
    }

    finish_direct_result(&result, distance, desc->max_distance, desc->air_absorption);
    *out_result = result;
    return true;
}

template <class T, class Listener>
bool trace_from_listener(Listener* listener, const typename T::Vec* source_position, Moss_AudioRayTraceResult* out_result) {
    if (!listener || !source_position || !out_result) {
        return false;
    }

    typename T::Desc desc{};
    desc.physics                  = listener->physicsScene;
    desc.raycast                  = listener->raycast;
    desc.user_data                = listener->raycastUserData;
    desc.listener_position        = listener->position;
    desc.source_position          = *source_position;
    desc.max_distance             = listener->maxRayDistance;
    desc.reflection_rays          = listener->rayCount;
    desc.direct_occlusion_strength = kListenerDirectOcclusionStrength;
    desc.reflection_strength      = kListenerReflectionStrength;
    desc.air_absorption           = kListenerAirAbsorption;

    if (!trace_impl<T>(&desc, out_result)) {
        return false;
    }

    listener->occlusion       = out_result->occlusion;
    listener->reflectionGain  = out_result->reflection_gain;
    listener->reflectionDelay = out_result->reflection_delay_seconds;
    return true;
}

template <class Listener, class Callback>
void set_physics(Listener* listener, PhysicsSystem* physics, Callback raycast, void* user_data) {
    if (!listener) {
        return;
    }
    listener->physicsScene     = physics;
    listener->raycast          = raycast;
    listener->raycastUserData  = user_data;
}

template <class T, class Listener>
void set_trace_settings(Listener* listener, float max_distance, uint32_t reflection_rays) {
    if (!listener) {
        return;
    }
    listener->maxRayDistance = std::max(0.0f, max_distance);
    listener->rayCount       = std::min(reflection_rays, T::kMaxRays);
}

// ---------------------------------------------------------------------------
// Public C API (unchanged signatures)
// ---------------------------------------------------------------------------
void Moss_AudioRayListener2DSetPhysics(RayAudioListener2D* listener, PhysicsSystem* physics, Moss_AudioRaycast2DCallback raycast, void* user_data) {
    set_physics(listener, physics, raycast, user_data);
}

void Moss_AudioRayListener3DSetPhysics(RayAudioListener3D* listener, PhysicsSystem* physics, Moss_AudioRaycast3DCallback raycast, void* user_data) {
    set_physics(listener, physics, raycast, user_data);
}

void Moss_AudioRayListener2DSetTraceSettings(RayAudioListener2D* listener, float max_distance, uint32_t reflection_rays) {
    set_trace_settings<Traits2D>(listener, max_distance, reflection_rays);
}

void Moss_AudioRayListener3DSetTraceSettings(RayAudioListener3D* listener, float max_distance, uint32_t reflection_rays) {
    set_trace_settings<Traits3D>(listener, max_distance, reflection_rays);
}

bool Moss_AudioRayTrace2D(const Moss_AudioRayTrace2DDesc* desc, Moss_AudioRayTraceResult* out_result) {
    return trace_impl<Traits2D>(desc, out_result);
}

bool Moss_AudioRayTrace3D(const Moss_AudioRayTrace3DDesc* desc, Moss_AudioRayTraceResult* out_result) {
    return trace_impl<Traits3D>(desc, out_result);
}

bool Moss_AudioRayTraceFromListener2D(RayAudioListener2D* listener, const Vec2* source_position, Moss_AudioRayTraceResult* out_result) {
    return trace_from_listener<Traits2D>(listener, source_position, out_result);
}

bool Moss_AudioRayTraceFromListener3D(RayAudioListener3D* listener, const Vec3* source_position, Moss_AudioRayTraceResult* out_result) {
    return trace_from_listener<Traits3D>(listener, source_position, out_result);
}

float Moss_AudioRayTraceComputeGain(const Moss_AudioRayTraceResult* result) {
    if (!result || !result->audible) {
        return 0.0f;
    }
    const float direct = result->attenuation * result->transmission_gain * (1.0f - result->occlusion * 0.35f);
    return clamp01(direct + result->reflection_gain * 0.25f);
}

// rayaudio.h
#include "audio_intern.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr float kMossSpeedOfSoundMetersPerSecond = 343.0f;
constexpr float kPi = 3.14159265358979323846f;

float clamp01(float value) {
    return std::max(0.0f, std::min(1.0f, value));
}

float length2(const Vec2& v) {
    return std::sqrt(v.GetX() * v.GetX() + v.GetY() * v.GetY());
}

float length3(const Vec3& v) {
    return std::sqrt(v.GetX() * v.GetX() + v.GetY() * v.GetY() + v.GetZ() * v.GetZ());
}

Vec2 normalize2(const Vec2& v, float len) {
    if (len <= 0.00001f) {
        return Vec2(1.0f, 0.0f);
    }
    return Vec2(v.GetX() / len, v.GetY() / len);
}

Vec3 normalize3(const Vec3& v, float len) {
    if (len <= 0.00001f) {
        return Vec3(1.0f, 0.0f, 0.0f);
    }
    return Vec3(v.GetX() / len, v.GetY() / len, v.GetZ() / len);
}

float distance_attenuation(float distance, float max_distance) {
    if (max_distance <= 0.00001f) {
        return 1.0f;
    }
    return clamp01(1.0f - distance / max_distance);
}

void finish_direct_result(Moss_AudioRayTraceResult* out, float distance, float max_distance, float air_absorption) {
    out->distance = distance;
    out->attenuation = distance_attenuation(distance, max_distance);
    out->delay_seconds = distance / kMossSpeedOfSoundMetersPerSecond;
    out->transmission_gain = clamp01(out->transmission_gain);
    out->occlusion = clamp01(out->occlusion);
    out->lowpass = clamp01(1.0f - out->occlusion * 0.75f - distance * std::max(0.0f, air_absorption) * 0.01f);
    out->audible = out->attenuation > 0.0f && out->transmission_gain > 0.001f;
}

void apply_hit(Moss_AudioRayTraceResult* out, bool hit, float absorption, float transmission, float strength) {
    if (!hit) {
        return;
    }
    const float safe_absorption = clamp01(absorption);
    const float safe_transmission = clamp01(transmission);
    const float safe_strength = std::max(0.0f, strength);
    out->occluded = true;
    out->occlusion = clamp01(out->occlusion + safe_absorption * safe_strength);
    out->transmission_gain *= safe_transmission;
}

} // namespace

void Moss_AudioRayListener2DSetPhysics(RayAudioListener2D* listener, PhysicsSystem* physics, Moss_AudioRaycast2DCallback raycast, void* user_data) {
    if (!listener) {
        return;
    }
    listener->physicsScene = physics;
    listener->raycast = raycast;
    listener->raycastUserData = user_data;
}

void Moss_AudioRayListener3DSetPhysics(RayAudioListener3D* listener, PhysicsSystem* physics, Moss_AudioRaycast3DCallback raycast, void* user_data) {
    if (!listener) {
        return;
    }
    listener->physicsScene = physics;
    listener->raycast = raycast;
    listener->raycastUserData = user_data;
}


void Moss_AudioRayListener2DSetTraceSettings(RayAudioListener2D* listener, float max_distance, uint32_t reflection_rays) {
    if (!listener) {
        return;
    }
    listener->maxRayDistance = std::max(0.0f, max_distance);
    listener->rayCount = std::min(reflection_rays, 64u);
}

void Moss_AudioRayListener3DSetTraceSettings(RayAudioListener3D* listener, float max_distance, uint32_t reflection_rays) {
    if (!listener) {
        return;
    }
    listener->maxRayDistance = std::max(0.0f, max_distance);
    listener->rayCount = std::min(reflection_rays, 128u);
}
bool Moss_AudioRayTrace2D(const Moss_AudioRayTrace2DDesc* desc, Moss_AudioRayTraceResult* out_result) {
    if (!desc || !out_result) {
        return false;
    }

    Moss_AudioRayTraceResult result{};
    result.transmission_gain = 1.0f;
    result.lowpass = 1.0f;

    const Vec2 to_source(desc->source_position.GetX() - desc->listener_position.GetX(), desc->source_position.GetY() - desc->listener_position.GetY());
    const float distance = length2(to_source);
    const Vec2 direction = normalize2(to_source, distance);
    const float trace_distance = desc->max_distance > 0.0f ? std::min(distance, desc->max_distance) : distance;

    if (desc->raycast && trace_distance > 0.0f) {
        Moss_AudioRayHit2D hit{};
        if (desc->raycast(desc->physics, &desc->listener_position, &direction, trace_distance, &hit, desc->user_data) && hit.hit) {
            apply_hit(&result, true, hit.absorption, hit.transmission, desc->direct_occlusion_strength);
        }
    }

    const uint32_t rays = std::min(desc->reflection_rays, 64u);
    if (desc->raycast && rays > 0 && desc->reflection_strength > 0.0f) {
        float reflected = 0.0f;
        float first_delay = 0.0f;
        for (uint32_t i = 0; i < rays; ++i) {
            const float a = (static_cast<float>(i) / static_cast<float>(rays)) * 2.0f * kPi;
            const Vec2 ray_dir(std::cos(a), std::sin(a));
            Moss_AudioRayHit2D hit{};
            if (desc->raycast(desc->physics, &desc->source_position, &ray_dir, desc->max_distance, &hit, desc->user_data) && hit.hit) {
                const float hit_distance = std::max(0.0f, hit.fraction) * desc->max_distance;
                const float energy = (1.0f - clamp01(hit.absorption)) * distance_attenuation(hit_distance + distance, desc->max_distance * 2.0f);
                reflected += energy;
                if (first_delay == 0.0f || hit_distance < first_delay) {
                    first_delay = hit_distance;
                }
            }
        }
        result.reflection_gain = clamp01((reflected / static_cast<float>(rays)) * desc->reflection_strength);
        result.reflection_delay_seconds = first_delay > 0.0f ? (first_delay + distance) / kMossSpeedOfSoundMetersPerSecond : 0.0f;
    }

    finish_direct_result(&result, distance, desc->max_distance, desc->air_absorption);
    *out_result = result;
    return true;
}

bool Moss_AudioRayTrace3D(const Moss_AudioRayTrace3DDesc* desc, Moss_AudioRayTraceResult* out_result) {
    if (!desc || !out_result) {
        return false;
    }

    Moss_AudioRayTraceResult result{};
    result.transmission_gain = 1.0f;
    result.lowpass = 1.0f;

    const Vec3 to_source(desc->source_position.GetX() - desc->listener_position.GetX(), desc->source_position.GetY() - desc->listener_position.GetY(), desc->source_position.GetZ() - desc->listener_position.GetZ());
    const float distance = length3(to_source);
    const Vec3 direction = normalize3(to_source, distance);
    const float trace_distance = desc->max_distance > 0.0f ? std::min(distance, desc->max_distance) : distance;

    if (desc->raycast && trace_distance > 0.0f) {
        Moss_AudioRayHit3D hit{};
        if (desc->raycast(desc->physics, &desc->listener_position, &direction, trace_distance, &hit, desc->user_data) && hit.hit) {
            apply_hit(&result, true, hit.absorption, hit.transmission, desc->direct_occlusion_strength);
        }
    }

    const uint32_t rays = std::min(desc->reflection_rays, 128u);
    if (desc->raycast && rays > 0 && desc->reflection_strength > 0.0f) {
        float reflected = 0.0f;
        float first_delay = 0.0f;
        for (uint32_t i = 0; i < rays; ++i) {
            const float u = (static_cast<float>(i) + 0.5f) / static_cast<float>(rays);
            const float v = static_cast<float>((i * 37u) % rays) / static_cast<float>(rays);
            const float z = 1.0f - 2.0f * u;
            const float r = std::sqrt(std::max(0.0f, 1.0f - z * z));
            const float theta = 2.0f * kPi * v;
            const Vec3 ray_dir(r * std::cos(theta), r * std::sin(theta), z);
            Moss_AudioRayHit3D hit{};
            if (desc->raycast(desc->physics, &desc->source_position, &ray_dir, desc->max_distance, &hit, desc->user_data) && hit.hit) {
                const float hit_distance = std::max(0.0f, hit.fraction) * desc->max_distance;
                const float energy = (1.0f - clamp01(hit.absorption)) * distance_attenuation(hit_distance + distance, desc->max_distance * 2.0f);
                reflected += energy;
                if (first_delay == 0.0f || hit_distance < first_delay) {
                    first_delay = hit_distance;
                }
            }
        }
        result.reflection_gain = clamp01((reflected / static_cast<float>(rays)) * desc->reflection_strength);
        result.reflection_delay_seconds = first_delay > 0.0f ? (first_delay + distance) / kMossSpeedOfSoundMetersPerSecond : 0.0f;
    }

    finish_direct_result(&result, distance, desc->max_distance, desc->air_absorption);
    *out_result = result;
    return true;
}


bool Moss_AudioRayTraceFromListener2D(RayAudioListener2D* listener, const Vec2* source_position, Moss_AudioRayTraceResult* out_result) {
    if (!listener || !source_position || !out_result) {
        return false;
    }

    Moss_AudioRayTrace2DDesc desc{};
    desc.physics = listener->physicsScene;
    desc.raycast = listener->raycast;
    desc.user_data = listener->raycastUserData;
    desc.listener_position = listener->position;
    desc.source_position = *source_position;
    desc.max_distance = listener->maxRayDistance;
    desc.reflection_rays = listener->rayCount;

    if (!Moss_AudioRayTrace2D(&desc, out_result)) {
        return false;
    }

    listener->occlusion = out_result->occlusion;
    listener->reflectionGain = out_result->reflection_gain;
    listener->reflectionDelay = out_result->reflection_delay_seconds;
    return true;
}

bool Moss_AudioRayTraceFromListener3D(RayAudioListener3D* listener, const Vec3* source_position, Moss_AudioRayTraceResult* out_result) {
    if (!listener || !source_position || !out_result) {
        return false;
    }

    Moss_AudioRayTrace3DDesc desc{};
    desc.physics = listener->physicsScene;
    desc.raycast = listener->raycast;
    desc.user_data = listener->raycastUserData;
    desc.listener_position = listener->position;
    desc.source_position = *source_position;
    desc.max_distance = listener->maxRayDistance;
    desc.reflection_rays = listener->rayCount;

    if (!Moss_AudioRayTrace3D(&desc, out_result)) {
        return false;
    }

    listener->occlusion = out_result->occlusion;
    listener->reflectionGain = out_result->reflection_gain;
    listener->reflectionDelay = out_result->reflection_delay_seconds;
    return true;
}
float Moss_AudioRayTraceComputeGain(const Moss_AudioRayTraceResult* result) {
    if (!result || !result->audible) {
        return 0.0f;
    }
    const float direct = result->attenuation * result->transmission_gain * (1.0f - result->occlusion * 0.35f);
    return clamp01(direct + result->reflection_gain * 0.25f);
}
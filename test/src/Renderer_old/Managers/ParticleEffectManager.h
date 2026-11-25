#ifndef MOSS_PARTICLE_EFFECT_H
#define MOSS_PARTICLE_EFFECT_H

#include <Moss/Moss_stdinc.h>
#include <Moss/Variants/Vector/Float3.h>
#include <Moss/Variants/Vector/Float2.h>
#include <Moss/Variants/TMap.h>
#include <Moss/Variants/TArray.h>

typedef uint32_t ParticleEmitterID;


// ======================================
// ======================================

class ParticleModifier2;

struct [[nodiscard]] MOSS_API Particle2 {
    Vec2 position, velocity;
    uint32_t lifespan;   // total time to live
    uint32_t age = 0;    // how long it has existed
    float rotation;
    Float2 scale = Float(1.0f, 1.0f);
};

struct [[nodiscard]] MOSS_API ParticleEffect2 { Mat44 m_transform; uint32_t spawnTime, uint32_t doomTime; TArray<ParticleEmitterID> ActiveEmitters; };

class [[nodiscard]] MOSS_API ParticleSystem2 {
public:
    ParticleSystem2() = default;
    ~ParticleSystem2() = default;

    void Update(uint32_t currentTime, uint32_t deltaTime);

    uint32_t CreateParticleEffect(const Transform2& transform);
    void DoomParticleEffect(uint32_t effectID);
    void KillParticleEffect(uint32_t effectID);

    ParticleEmitterID CreateParticleEmitter();
    void DoomParticleEmitter(ParticleEmitterID id);

private:
    TMap<uint32_t, ParticleEffectsBucket2> m_activeEffectBuckets;
    TMap<uint32_t, ParticleEmitterBucket2> m_activeEmitterBuckets;

    uint32_t nextEffectID = 1;
    uint32_t nextEmitterID = 1;
};



class [[nodiscard]] MOSS_API ParticleEffectsBucket2 {
public:
    ParticleEffectsBucket2(ParticleSystem2& system, ParticleEffect2& effectResource) : particleSystem(system), effectResource(effectResource) {}

    void Update(uint32_t currentTime, uint32_t deltaTime);

    uint32_t CreateParticleEffect(const Transform2& transform);
    void DoomParticleEffect(uint32_t effectID);

private:
    void UpdateEffectEmitter(ParticleEffect2* effect, uint32_t currentTime, uint32_t deltaTime);
    void RemoveParticleEffect(uint32_t index);

    ParticleSystem2& particleSystem;
    ParticleEffect2& effectResource;
    TArray<ParticleEffect2> activeEffects;
};

struct [[nodiscard]] MOSS_API ParticleEmitter2 {
    Mat44 m_transform;
    uint32_t spawnTimer;
    int amount;
    float spawnRemainer = 0.1f;
};

class [[nodiscard]] MOSS_API ParticleEmitterBucket2 {
public:
    ParticleEmitterBucket2(const ParticleEmitter2& resource);
    uint32_t CreateParticleEmitter();
    void Update(uint32_t currentTime, uint32_t deltaTime);
    void DoomParticleEmitter(ParticleEmitterID id);

    void draw();

private:
    void SpawnParticles(uint32_t deltaTime);
    void KillDeadParticles() return { m_particles.clear(); }
    uint32_t GetMaxParticleCount() const;

    Vec2 GetParticleStartingPosition();
    Vec2 GetParticleStartingVelocity();

    const ParticleEmitter2& m_emitterResource;

    TArray<Particle2> m_particles;
    TArray<ParticleModifier2*> m_modifiers;
    TArray<ParticleEmitter2> m_activeEmitters;



    GLuint vao = 0;
    GLuint quadVBO = 0;
    GLuint instanceVBO = 0;
    ShaderGL shader;
};


class [[nodiscard]] MOSS_API ParticleModifier2 {};


// ======================================
// ======================================
class ParticleModifier3;

struct [[nodiscard]] MOSS_API Particle3 {
    Vec3 position, Vec3 velocity;
    uint32_t lifespan;   // total time to live
    uint32_t age = 0;    // how long it has existed
};

struct [[nodiscard]] MOSS_API ParticleEffect3 { Mat44 m_transform; uint32_t spawnTime; uint32_t doomTime; TArray<ParticleEmitterID> ActiveEmitters; };

class [[nodiscard]] MOSS_API ParticleSystem3 {
public:
    ParticleSystem2() = default;
    ~ParticleSystem2() = default;

    void Update(uint32_t currentTime, uint32_t deltaTime);

    uint32_t CreateParticleEffect(const Transform3& transform);
    void DoomParticleEffect(uint32_t effectID);
    void KillParticleEffect(uint32_t effectID);

    ParticleEmitterID CreateParticleEmitter();
    void DoomParticleEmitter(ParticleEmitterID id);

private:
    TMap<uint32_t, ParticleEffectsBucket2> m_activeEffectBuckets;
    TMap<uint32_t, ParticleEmitterBucket2> m_activeEmitterBuckets;

    uint32_t nextEffectID = 1;
    uint32_t nextEmitterID = 1;
};


class [[nodiscard]] MOSS_API ParticleEffectsBucket3 {
public:
    ParticleEffectsBucket3(ParticleSystem3& system, ParticleEffect3& effectResource) : particleSystem(system), effectResource(effectResource) {}

    void Update(uint32_t currentTime, uint32_t deltaTime);
    uint32_t CreateParticleEffect(const Transform2& transform);
    void DoomParticleEffect(uint32_t effectID);

private:
    void UpdateEffectEmitter(ParticleEffect3* effect, uint32_t currentTime, uint32_t deltaTime);
    void RemoveParticleEffect(uint32_t index);

    ParticleSystem3& particleSystem;
    ParticleEffect3& effectResource;
    TArray<ParticleEffect2> activeEffects;
};

struct ParticleEmitter3
{
    Mat44 m_transform;
    uint32_t spawnTimer;
    int amount;
    float spawnRemainer = 0.1f;
};

class [[nodiscard]] MOSS_API ParticleEmitterBucket3 {
public:
    ParticleEmitterBucket2(const ParticleEmitter3& resource) : m_emitterResource(resource) {}

    void Update(uint32_t currentTime, uint32_t deltaTime);
    uint32_t CreateParticleEmitter();
    void DoomParticleEmitter(ParticleEmitterID id);

private:
    void SpawnParticles(uint32_t deltaTime);
    void KillDeadParticles();
    uint32_t GetMaxParticleCount() const;

    Vec2 GetParticleStartingPosition();
    Vec2 GetParticleStartingVelocity();

    const ParticleEmitter3& m_emitterResource;

    TArray<Particle3> m_particles;
    TArray<ParticleModifier3*> m_modifiers;
    TArray<ParticleEmitter3> m_activeEmitters;
};

class [[nodiscard]] MOSS_API ParticleModifier3 {};
#endif // MOSS_PARTICLE_EFFECT_3_GL_H
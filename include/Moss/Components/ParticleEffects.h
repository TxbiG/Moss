//                        MIT License
//
//                  Copyright (c) 2025 Toby
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
 * @file ParticleEffects.h
 * @brief X.
 */

 // Add Attractors and Collisions for Particles
 
#ifndef MOSS_PARTICLE_EFFECTS_H
#define MOSS_PARTICLE_EFFECTS_H

#include <Moss/Moss_stdinc.h>
#include <Moss/Moss_Renderer.h>
#include <Moss/Moss_Physics.h>

using ParticleEmitterID = uint32;

class CPUParticleModifier2D;
class CPUParticleEffectsBucket2D;

class CPUParticleModifier3;
class CPUParticleEffectsBucket3;

class GPUParticleModifier2D;
class GPUParticleEffectsBucket2D;

class GPUParticleModifier3;
class GPUParticleEffectsBucket3;


/*! @brief X. @param position X. @param velocity X. @param lifespan Total time to live. @param age How long it has existed. @param rotation X. @param scale X.*/
struct [[nodiscard]] MOSS_API Particle2 { Vec2 position, velocity; uint32 lifespan, age = 0; Float2 scale = Float(1.0f, 1.0f); };
struct [[nodiscard]] MOSS_API ParticleEffect2 { Mat3x2 m_transform; uint32 spawnTime, uint32 doomTime; TArray<ParticleEmitterID> ActiveEmitters; };
struct [[nodiscard]] MOSS_API ParticleEmitter2 { Mat3x2 m_transform; uint32 spawnTimer; int amount; float spawnRemainer = 0.1f; };

/*! @brief X. @param position X. @param velocity X. @param lifespan Total time to live. @param age How long it has existed. @param rotation X. @param scale X.*/
struct [[nodiscard]] MOSS_API Particle3 { Vec3 position, velocity, Float3 rotation, scale; uint32 lifespan, age = 0, };
struct [[nodiscard]] MOSS_API ParticleEffect3 { Mat44 m_transform; uint32 spawnTime; uint32 doomTime; TArray<ParticleEmitterID> ActiveEmitters; };
struct [[nodiscard]] MOSS_API ParticleEmitter3 { Mat44 m_transform; uint32 spawnTimer; int amount; float spawnRemainer = 0.1f; };

/* ================================
 * CPU Particles
 * ================================ */
class [[nodiscard]] MOSS_API CPUParticleSystem2D {
public:
    CPUParticleSystem2(Moss_Renderer* renderer) = default;
    ~CPUParticleSystem2() = default;

    void Update(uint32 currentTime, uint32 deltaTime);
	void draw();
    uint32 CreateParticleEffect(const Transform2& transform);
    void DoomParticleEffect(uint32 effectID);
    void KillParticleEffect(uint32 effectID);

    ParticleEmitterID CreateParticleEmitter();
    void DoomParticleEmitter(ParticleEmitterID id);

private:
	Moss_Renderer* renderer;
    TMap<uint32, ParticleEffectsBucket2> m_activeEffectBuckets;
    TMap<uint32, ParticleEmitterBucket2> m_activeEmitterBuckets;

    uint32 nextEffectID = 1;
    uint32 nextEmitterID = 1;
};

class [[nodiscard]] MOSS_API CPUParticleEmitterBucket2D {
public:
    CPUParticleEmitterBucket2(const ParticleEmitter2D& resource);
    uint32 CreateParticleEmitter();
    void Update(uint32 currentTime, uint32 deltaTime);
    void DoomParticleEmitter(ParticleEmitterID id);

    void draw();

private:
    void SpawnParticles(uint32 deltaTime);
    void KillDeadParticles() return { m_particles.clear(); }
    uint32 GetMaxParticleCount() const;

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
    // PipelineState m_pipeline;
};

class [[nodiscard]] MOSS_API CPUParticleSystem3D {
public:
    CPUParticleSystem2(Moss_Renderer* renderer) = default;
    ~CPUParticleSystem2() = default;

    void Update(uint32 currentTime, uint32 deltaTime);
	void draw();
    uint32 CreateParticleEffect(const Transform3& transform);
    void DoomParticleEffect(uint32 effectID);
    void KillParticleEffect(uint32 effectID);

    ParticleEmitterID CreateParticleEmitter();
    void DoomParticleEmitter(ParticleEmitterID id);

private:
	Moss_Renderer* m_renderer;
    TMap<uint32, ParticleEffectsBucket3> m_activeEffectBuckets;
    TMap<uint32, ParticleEmitterBucket3> m_activeEmitterBuckets;

    uint32 nextEffectID = 1;
    uint32 nextEmitterID = 1;
};

class [[nodiscard]] MOSS_API CPUParticleEmitterBucket3 {
public:
    CPUParticleEmitterBucket2(const ParticleEmitter3& resource) : m_emitterResource(resource) {}

    void Update(uint32 currentTime, uint32 deltaTime);
    uint32 CreateParticleEmitter();
    void DoomParticleEmitter(ParticleEmitterID id);

private:
    void SpawnParticles(uint32 deltaTime);
    void KillDeadParticles();
    uint32 GetMaxParticleCount() const;

    Vec2 GetParticleStartingPosition();
    Vec2 GetParticleStartingVelocity();

    const CPUParticleEmitter3& m_emitterResource;

    TArray<Particle3> m_particles;
    TArray<ParticleModifier3*> m_modifiers;
    TArray<ParticleEmitter3> m_activeEmitters;
};

class [[nodiscard]] MOSS_API ParticleModifier3 {};





class [[nodiscard]] MOSS_API GPUParticleSystem2D {
public:
    GPUParticleSystem2(Moss_Renderer* renderer) = default;
    ~GPUParticleSystem2() = default;

    void Update(uint32 currentTime, uint32 deltaTime);
	void draw();
    uint32 CreateParticleEffect(const Transform2& transform);
    void DoomParticleEffect(uint32 effectID);
    void KillParticleEffect(uint32 effectID);

    ParticleEmitterID CreateParticleEmitter();
    void DoomParticleEmitter(ParticleEmitterID id);

private:
	Moss_Renderer* renderer;
    TMap<uint32, ParticleEffectsBucket2> m_activeEffectBuckets;
    TMap<uint32, ParticleEmitterBucket2> m_activeEmitterBuckets;

    uint32 nextEffectID = 1;
    uint32 nextEmitterID = 1;
};

class [[nodiscard]] MOSS_API GPUParticleEmitterBucket2D {
public:
    GPUParticleEmitterBucket2(const ParticleEmitter2D& resource);
    uint32 CreateParticleEmitter();
    void Update(uint32 currentTime, uint32 deltaTime);
    void DoomParticleEmitter(ParticleEmitterID id);

    void draw();

private:
    void SpawnParticles(uint32 deltaTime);
    void KillDeadParticles() return { m_particles.clear(); }
    uint32 GetMaxParticleCount() const;

    Vec2 GetParticleStartingPosition();
    Vec2 GetParticleStartingVelocity();

    const ParticleEmitter2& m_emitterResource;

    ComputeShader m_spawnCS;
    ComputeShader m_updateCS;

    TArray<Particle2> m_particles;
    TArray<ParticleModifier2*> m_modifiers;
    TArray<ParticleEmitter2> m_activeEmitters;



    GLuint vao = 0;
    GLuint quadVBO = 0;
    GLuint instanceVBO = 0;
    ShaderGL shader;
};

class [[nodiscard]] MOSS_API ParticleModifier2 {};


/* ================================
 * GPU Particles
 * ================================ */

class [[nodiscard]] MOSS_API GPUParticleSystem3D {
public:
    GPUParticleSystem2(Moss_Renderer* renderer) = default;
    ~GPUParticleSystem2() = default;

    void Update(uint32 currentTime, uint32 deltaTime);
	void draw();
    uint32 CreateParticleEffect(const Transform3& transform);
    void DoomParticleEffect(uint32 effectID);
    void KillParticleEffect(uint32 effectID);

    ParticleEmitterID CreateParticleEmitter();
    void DoomParticleEmitter(ParticleEmitterID id);

private:
	Moss_Renderer* m_renderer;
    TMap<uint32, ParticleEffectsBucket3> m_activeEffectBuckets;
    TMap<uint32, ParticleEmitterBucket3> m_activeEmitterBuckets;

    uint32 nextEffectID = 1;
    uint32 nextEmitterID = 1;
};

class [[nodiscard]] MOSS_API GPUParticleEmitterBucket3D {
public:
    GPUParticleEmitterBucket2(const ParticleEmitter3& resource) : m_emitterResource(resource) {}

    void Update(uint32 currentTime, uint32 deltaTime);
    uint32 CreateParticleEmitter();
    void DoomParticleEmitter(ParticleEmitterID id);

private:
    void SpawnParticles(uint32 deltaTime);
    void KillDeadParticles();
    uint32 GetMaxParticleCount() const;

    Vec2 GetParticleStartingPosition();
    Vec2 GetParticleStartingVelocity();

    const GPUParticleEmitter3& m_emitterResource;

    ComputeShader m_spawnCS;
    ComputeShader m_updateCS;

    TArray<GPUParticle3> m_particles;
    TArray<GPUParticleModifier3*> m_modifiers;
    TArray<GPUParticleEmitter3> m_activeEmitters;
};

class [[nodiscard]] MOSS_API GPUParticleModifier3 {};

#endif // MOSS_PARTICLE_EFFECTS_H
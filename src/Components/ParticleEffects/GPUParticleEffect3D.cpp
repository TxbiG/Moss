

class [[nodiscard]] MOSS_API ParticleEffectsBucket3 {
public:
    ParticleEffectsBucket3(ParticleSystem3& system, ParticleEffect3& effectResource) : particleSystem(system), effectResource(effectResource) {}

    void Update(uint32 currentTime, uint32 deltaTime);
    uint32 CreateParticleEffect(const Transform2& transform);
    void DoomParticleEffect(uint32 effectID);

private:
    void UpdateEffectEmitter(ParticleEffect3* effect, uint32 currentTime, uint32 deltaTime);
    void RemoveParticleEffect(uint32 index);

    ParticleSystem3& particleSystem;
    ParticleEffect3& effectResource;
    TArray<ParticleEffect2> activeEffects;
};

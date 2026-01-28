




class [[nodiscard]] MOSS_API ParticleEffectsBucket2 {
public:
    ParticleEffectsBucket2(ParticleSystem2& system, ParticleEffect2& effectResource) : particleSystem(system), effectResource(effectResource) {}

    void Update(uint32 currentTime, uint32 deltaTime);

    uint32 CreateParticleEffect(const Transform2& transform);
    void DoomParticleEffect(uint32 effectID);

private:
    void UpdateEffectEmitter(ParticleEffect2* effect, uint32 currentTime, uint32 deltaTime);
    void RemoveParticleEffect(uint32 index);

    ParticleSystem2& particleSystem;
    ParticleEffect2& effectResource;
    TArray<ParticleEffect2> activeEffects;
};
#include <Moss/Renderer/GL/ParticleEffect2.h>


const char* vertparticle = R"(
#version 330 core
layout(location = 0) in vec2 in_pos;
layout(location = 1) in vec2 instance_pos;
layout(location = 2) in vec2 instance_scale;

void main() {
    vec2 scaled = in_pos * instance_scale;
    gl_Position = vec4(scaled + instance_pos, 0.0, 1.0);
})";

const char* fragparticle = R"(
#version 330 core
out vec4 FragColor;

void main() {
    FragColor = vec4(1.0, 1.0, 1.0, 1.0);
})";


ParticleEmitterBucket2::ParticleEmitterBucket2(const ParticleEmitter2& resource) : m_emitterResource(resource), shader(vertparticle, fragparticle, true) {
    // Simple quad
    float quadVerts[] = {
        -0.5f, -0.5f,
         0.5f, -0.5f,
        -0.5f,  0.5f,
         0.5f,  0.5f
    };

    glGenVertexArrays(1, &vao);
    glBindVertexArray(vao);

    // Quad VBO
    glGenBuffers(1, &quadVBO);
    glBindBuffer(GL_ARRAY_BUFFER, quadVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quadVerts), quadVerts, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0); // layout(location = 0)
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);

    // Instance data
    glGenBuffers(1, &instanceVBO);
    glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Particle2) * 1000, nullptr, GL_DYNAMIC_DRAW); // adjust capacity

    // Layouts: position.xy, scale.xy
    glEnableVertexAttribArray(1); // position
    glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, sizeof(Particle2), (void*)offsetof(Particle2, position));
    glVertexAttribDivisor(1, 1);

    glEnableVertexAttribArray(2); // scale
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Particle2), (void*)offsetof(Particle2, scale));
    glVertexAttribDivisor(2, 1);

    glBindVertexArray(0);
}

void ParticleEmitterBucket2::Update(uint32 currentTime, uint32 deltaTime) {
    SpawnParticles(deltaTime);
    for (auto& p : m_particles) {
        if (p.age < p.lifespan) {
            p.position += p.velocity * (deltaTime / 1000.0f); // simple motion
            p.age += deltaTime;
        }
    }
    KillDeadParticles();
}

void ParticleEmitterBucket2::draw() {
    if (m_particles.Empty()) return;

    glBindBuffer(GL_ARRAY_BUFFER, instanceVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(Particle2) * m_particles.Size(), m_particles.Data());

    shader.bind();
    glBindVertexArray(vao);
    glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 4, m_particles.Size());
    glBindVertexArray(0);
    shader.unbind();

}

void ParticleEmitterBucket2::SpawnParticles(uint32 deltaTime) {
    int numToSpawn = 10; // for example
    for (int i = 0; i < numToSpawn; ++i) {
        Particle2 p;
        p.position = GetParticleStartingPosition();
        p.velocity = GetParticleStartingVelocity();
        p.lifespan = 2000; // 2 seconds
        m_particles.Add(p);
    }
}

void ParticleEmitterBucket2::KillDeadParticles() {
    m_particles.RemoveIf([](const Particle2& p) {
        return p.age >= p.lifespan;
    });
}

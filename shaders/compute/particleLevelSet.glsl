#version 430 core

layout(local_size_x = 256) in;

struct Particle {
    vec4 pos;
    vec4 vel;
};

layout(std430, binding = 0) buffer ParticleBuffer { Particle particles[]; };
layout(std430, binding = 16) coherent buffer ParticlesLevelSet { int particlesLevelSet[]; };

uniform int size;
uniform int numParticles;

int gridIdx(int x, int y, int z) {
    return (z * size * size) + (y * size) + x;
}

void main() {
    uint idx = gl_GlobalInvocationID.x;
    if (idx >= numParticles) return;

    Particle p = particles[idx];
    vec3 pPos = p.pos.xyz;

    ivec3 cellIdx = ivec3(pPos * float(size));

    int range = 2;

    for (int k = -range; k <= range; ++k) {
        for (int j = -range; j <= range; ++j) {
            for (int i = -range; i <= range; ++i) {
                ivec3 neighbor = cellIdx + ivec3(i, j, k);

                if (neighbor.x < 0 || neighbor.x >= size ||
                    neighbor.y < 0 || neighbor.y >= size ||
                    neighbor.z < 0 || neighbor.z >= size) continue;

                vec3 cellCenter = (vec3(neighbor) + 0.5) / float(size);

                float dist = distance(pPos, cellCenter);
                float radius = 1.0f / size; // Particle radius
                float signedDist = radius - dist; // We inverse caus we want a negative distance inside the fluid

                // Atomic Min
                int intVal = int(signedDist * 1000000.0);
                atomicMax(particlesLevelSet[gridIdx(neighbor.x, neighbor.y, neighbor.z)], intVal);
            }
        }
    }

}

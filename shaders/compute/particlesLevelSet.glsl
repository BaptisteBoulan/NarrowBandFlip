#version 430 core

layout(local_size_x = 256) in;

struct Particle {
    vec4 pos;
    vec4 vel;
};

layout (std430, binding = 0) buffer ParticleBuffer { Particle particles[]; };
layout (std430, binding = 16) buffer ParticlesLevelSetBuffer { int[] particlesLevelSet; };

uniform int size;
uniform int numParticles;

int getIdx(ivec3 pos) {
    return (pos.z * size * size) + (pos.y * size) + pos.x;
}

void main() {
    uint idx = gl_GlobalInvocationID.x;
    if (idx >= numParticles) return;

    vec3 pPos = particles[idx].pos.xyz;
    
    ivec3 base = ivec3(pPos * size - vec3(1.0)); 

    for (int z = 0; z <= 2; z++) {
        for (int y = 0; y <= 2; y++) {
            for (int x = 0; x <= 2; x++) {
                ivec3 cellPos = base + ivec3(x, y, z);
                if (cellPos.x < 0 || cellPos.y < 0 || cellPos.z < 0 || 
                    cellPos.x >= size || cellPos.y >= size || cellPos.z >= size) continue;

                vec3 gridWorldPos = (vec3(cellPos) + 0.5f) / size;
                
                float dist = length(pPos - gridWorldPos) - (1.0f / size);

                // Scale to int for atomicMin (1000.0 provides 3 decimal places of precision)
                int intDist = int(dist * 1000.0);
                atomicMin(particlesLevelSet[getIdx(cellPos)], intDist);
            }
        }
    }
}
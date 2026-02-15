#version 450 core

struct Particle {
    vec4 pos;
    vec4 vel;
};

layout (local_size_x = 256) in;

layout (std430, binding = 0) readonly buffer ParticlesBuffer { Particle[] particles; };
layout (std430, binding = 25) coherent buffer DensityBuffer { int[] densities; };


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
                float density = max(0.0, 1.0 - dist * size);
                density = density * density * (3.0 - 2.0 * density);



                // Atomic add
                int intVal = int(density * 1000000.0);
                atomicAdd(densities[gridIdx(neighbor.x, neighbor.y, neighbor.z)], intVal);
                
                // densities[gridIdx(neighbor.x, neighbor.y, neighbor.z)] += intVal;
            }
        }
    }

}

#version 430 core

layout(local_size_x = 256) in;

struct Particle {
    vec4 pos;
    vec4 vel;
};

// Buffers
layout(std430, binding = 0) buffer ParticleBuffer { Particle particles[]; };
layout(std430, binding = 8) coherent buffer CellType   { uint cellType[]; };

uniform int size;
uniform int numParticles;

int gridIdx(int x, int y, int z) {return z * size *size + y * size + x;}

void main() {

    uint idx = gl_GlobalInvocationID.x;
    if (idx >= numParticles) return;

    Particle p = particles[idx];

    int i = int(p.pos.x * size);
    int j = int(p.pos.y * size);
    int k = int(p.pos.z * size);

    int cellIdx = gridIdx(i,j,k);

    uint type = cellType[cellIdx];

    if (type != 0) cellType[cellIdx] = 2; // If not SOLID FLUID
}
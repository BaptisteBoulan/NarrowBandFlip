#version 430 core

layout(local_size_x = 256) in;

struct Particle {
    vec2 pos;
    vec2 vel;
};

// Buffers
layout(std430, binding = 0) buffer ParticleBuffer { Particle particles[]; };
layout(std430, binding = 8) coherent buffer CellType   { uint cellType[]; };

uniform int size;
uniform int numParticles;

int getIdx(int i, int j) { return j * size + i; }

void main() {

    uint idx = gl_GlobalInvocationID.x;
    if (idx >= numParticles) return;

    Particle p = particles[idx];

    int i = int(p.pos.x * size);
    int j = int(p.pos.y * size);

    int cellIdx = getIdx(i,j);

    uint type = cellType[cellIdx];

    if (type != 0) cellType[cellIdx] = 2; // If not SOLID FLUID
}
#version 430 core
layout(local_size_x = 256) in;

struct Particle {
    vec2 pos;
    vec2 vel;
};

layout(std430, binding = 0) buffer ParticleBuffer { Particle particles[]; };
layout(std430, binding = 1) buffer UBuffer { float us[]; };
layout(std430, binding = 2) buffer VBuffer { float vs[]; };
layout(std430, binding = 3) buffer UMassBuffer { float uMasses[]; };
layout(std430, binding = 4) buffer VMassBuffer { float vMasses[]; };

uniform int size;
uniform int numParticles;

// Helper to handle atomic float addition
void atomicAddFloat(int index, buffer float data[], float val) {
    uint newVal;
    uint prevVal;
    do {
        prevVal = floatBitsToUint(data[index]);
        newVal = floatBitsToUint(uintBitsToFloat(prevVal) + val);
    } while (atomicCompSwap(floatBitsToUint(data[index]), prevVal, newVal) != prevVal);
}

void main() {
    uint idx = gl_GlobalInvocationID.x;
    if (idx >= numParticles) return;

    Particle p = particles[idx];
    float px = p.pos.x * size;
    float py = p.pos.y * size;

    // U-Velocity Transfer Logic (Simplified for brevity)
    // Same logic as your C++ code, but using atomicAddFloat
    // ... calculate weights and indices ...
    // atomicAddFloat(u_idx, us, weight * p.vel.x);
    // atomicAddFloat(u_idx, uMasses, weight);
}
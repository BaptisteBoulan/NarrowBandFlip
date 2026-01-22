#version 430 core

layout(local_size_x = 256) in;

struct Particle {
    vec2 pos;
    vec2 vel;
};

// Buffers
layout(std430, binding = 0) buffer ParticleBuffer { Particle particles[]; };
layout(std430, binding = 1) coherent buffer UBuffer         { uint us[]; };
layout(std430, binding = 2) coherent buffer VBuffer         { uint vs[]; };
layout(std430, binding = 3) coherent buffer UMassBuffer    { uint uMasses[]; };
layout(std430, binding = 4) coherent buffer VMassBuffer    { uint vMasses[]; };

uniform int size;
uniform int numParticles;

// Helper to perform atomic addition on floats using uint buffers
void atomicAddFloat(uint index, float val, int bufferType) {
    uint expectedVal, newVal, oldVal;
    
    // We must initialize 'oldVal' by reading the current state of the buffer
    if (bufferType == 0)      oldVal = us[index];
    else if (bufferType == 1) oldVal = vs[index];
    else if (bufferType == 2) oldVal = uMasses[index];
    else                      oldVal = vMasses[index];

    do {
        expectedVal = oldVal;
        // Convert bits to float, add, then convert back to uint bits
        float currentFloatVal = uintBitsToFloat(expectedVal);
        newVal = floatBitsToUint(currentFloatVal + val);
        
        // atomicCompSwap returns the value that was in memory BEFORE the operation
        if (bufferType == 0)      oldVal = atomicCompSwap(us[index], expectedVal, newVal);
        else if (bufferType == 1) oldVal = atomicCompSwap(vs[index], expectedVal, newVal);
        else if (bufferType == 2) oldVal = atomicCompSwap(uMasses[index], expectedVal, newVal);
        else                      oldVal = atomicCompSwap(vMasses[index], expectedVal, newVal);

    } while (oldVal != expectedVal); // If someone else changed it, try again
}

// Indexing helpers matching the CPU logic
int getUIdx(int i, int j) { return j * (size + 1) + i; }
int getVIdx(int i, int j) { return j * size + i; }

void main() {

    uint idx = gl_GlobalInvocationID.x;
    if (idx >= numParticles) return;

    Particle p = particles[idx];

    float px = p.pos.x * size;
    float py = p.pos.y * size;

    // U-VELOCITIES
    float ux = px;
    float uy = py - 0.5f;
    
    int ui = int(ux);
    int uj = int(uy);
    float uwx = ux - ui;
    float uwy = uy - uj;

    // Bilinear weights for U
    float weights_u[4] = {
        (1 - uwx) * (1 - uwy), // (i, j)
        uwx * (1 - uwy),       // (i+1, j)
        (1 - uwx) * uwy,       // (i, j+1)
        uwx * uwy              // (i+1, j+1)
    };

    // Transfer U
    int u_indices[4] = { getUIdx(ui, uj), getUIdx(ui+1, uj), getUIdx(ui, uj+1), getUIdx(ui+1, uj+1) };
    for(int k=0; k<4; ++k) {
        if (u_indices[k] < 0 || u_indices[k] >= size*(size+1)) continue;
        atomicAddFloat(u_indices[k], weights_u[k] * p.vel.x, 0);
        atomicAddFloat(u_indices[k], weights_u[k], 2);
    }

    // V-VELOCITIES
    float vx = px - 0.5f;
    float vy = py;

    int vi = int(vx);
    int vj = int(vy);
    float vwx = vx - vi;
    float vwy = vy - vj;

    // Bilinear weights for V
    float weights_v[4] = {
        (1 - vwx) * (1 - vwy), // (i, j)
        vwx * (1 - vwy),       // (i+1, j)
        (1 - vwx) * vwy,       // (i, j+1)
        vwx * vwy              // (i+1, j+1)
    };

    // Transfer V
    int v_indices[4] = { getVIdx(vi, vj), getVIdx(vi+1, vj), getVIdx(vi, vj+1), getVIdx(vi+1, vj+1) };
    for(int k=0; k<4; ++k) {
        if (v_indices[k] < 0 || v_indices[k] >= size*(size+1)) continue;
        atomicAddFloat(v_indices[k], weights_v[k] * p.vel.y, 1);
        atomicAddFloat(v_indices[k], weights_v[k], 3);
    }
}
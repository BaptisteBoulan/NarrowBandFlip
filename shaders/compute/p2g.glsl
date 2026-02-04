#version 430 core

layout(local_size_x = 256) in;

struct Particle {
    vec4 pos;
    vec4 vel;
};

layout(std430, binding = 0) buffer ParticleBuffer { Particle particles[]; };

// Velocity Buffers
layout(std430, binding = 1) coherent buffer UBuffer { uint us[]; };
layout(std430, binding = 2) coherent buffer VBuffer { uint vs[]; };
layout(std430, binding = 3) coherent buffer WBuffer { uint ws[]; };

// Mass/Weight Buffers
layout(std430, binding = 4) coherent buffer UMassBuffer { uint uMasses[]; };
layout(std430, binding = 5) coherent buffer VMassBuffer { uint vMasses[]; };
layout(std430, binding = 6) coherent buffer WMassBuffer { uint wMasses[]; };

uniform int size;
uniform int numParticles;

void atomicAddFloat(uint index, float val, int bufferType) {
    uint expectedVal, newVal, oldVal;
    
    // Select the correct buffer based on type
    if (bufferType == 0)      oldVal = us[index];
    else if (bufferType == 1) oldVal = vs[index];
    else if (bufferType == 2) oldVal = ws[index];
    else if (bufferType == 3) oldVal = uMasses[index];
    else if (bufferType == 4) oldVal = vMasses[index];
    else                      oldVal = wMasses[index];

    do {
        expectedVal = oldVal;
        float currentFloatVal = uintBitsToFloat(expectedVal);
        newVal = floatBitsToUint(currentFloatVal + val);
        
        if (bufferType == 0)      oldVal = atomicCompSwap(us[index], expectedVal, newVal);
        else if (bufferType == 1) oldVal = atomicCompSwap(vs[index], expectedVal, newVal);
        else if (bufferType == 2) oldVal = atomicCompSwap(ws[index], expectedVal, newVal);
        else if (bufferType == 3) oldVal = atomicCompSwap(uMasses[index], expectedVal, newVal);
        else if (bufferType == 4) oldVal = atomicCompSwap(vMasses[index], expectedVal, newVal);
        else                      oldVal = atomicCompSwap(wMasses[index], expectedVal, newVal);

    } while (oldVal != expectedVal);
}

// 3D Staggered Grid Indexing
int getUIdx(int i, int j, int k) { return k * size * (size + 1) + j * (size + 1) + i; }
int getVIdx(int i, int j, int k) { return k * (size + 1) * size + j * size + i; }
int getWIdx(int i, int j, int k) { return k * size * size + j * size + i; }

void main() {
    uint idx = gl_GlobalInvocationID.x;
    if (idx >= numParticles) return;

    Particle p = particles[idx];
    vec3 pPos = p.pos.xyz * float(size);

    // --- Transfer U (X-velocity) ---
    // Staggered: centered at (i, j+0.5, k+0.5)
    vec3 uPos = vec3(pPos.x, pPos.y - 0.5, pPos.z - 0.5);
    ivec3 ui0 = ivec3(floor(uPos));
    vec3 uf = uPos - vec3(ui0);

    for (int k = 0; k <= 1; ++k) {
        for (int j = 0; j <= 1; ++j) {
            for (int i = 0; i <= 1; ++i) {
                float weight = (i == 0 ? 1.0 - uf.x : uf.x) *
                               (j == 0 ? 1.0 - uf.y : uf.y) *
                               (k == 0 ? 1.0 - uf.z : uf.z);
                
                int gx = ui0.x + i;
                int gy = ui0.y + j;
                int gz = ui0.z + k;

                if (gx >= 0 && gx < size + 1 && gy >= 0 && gy < size && gz >= 0 && gz < size) {
                    int gIdx = getUIdx(gx, gy, gz);
                    atomicAddFloat(gIdx, weight * p.vel.x, 0);
                    atomicAddFloat(gIdx, weight, 3);
                }
            }
        }
    }

    // --- Transfer V (Y-velocity) ---
    // Staggered: centered at (i+0.5, j, k+0.5)
    vec3 vPos = vec3(pPos.x - 0.5, pPos.y, pPos.z - 0.5);
    ivec3 vi0 = ivec3(floor(vPos));
    vec3 vf = vPos - vec3(vi0);

    for (int k = 0; k <= 1; ++k) {
        for (int j = 0; j <= 1; ++j) {
            for (int i = 0; i <= 1; ++i) {
                float weight = (i == 0 ? 1.0 - vf.x : vf.x) *
                               (j == 0 ? 1.0 - vf.y : vf.y) *
                               (k == 0 ? 1.0 - vf.z : vf.z);
                
                int gx = vi0.x + i;
                int gy = vi0.y + j;
                int gz = vi0.z + k;

                if (gx >= 0 && gx < size && gy >= 0 && gy < size + 1 && gz >= 0 && gz < size) {
                    int gIdx = getVIdx(gx, gy, gz);
                    atomicAddFloat(gIdx, weight * p.vel.y, 1);
                    atomicAddFloat(gIdx, weight, 4);
                }
            }
        }
    }

    // --- Transfer W (Z-velocity) ---
    // Staggered: centered at (i+0.5, j+0.5, k)
    vec3 wPos = vec3(pPos.x - 0.5, pPos.y - 0.5, pPos.z);
    ivec3 wi0 = ivec3(floor(wPos));
    vec3 wf = wPos - vec3(wi0);

    for (int k = 0; k <= 1; ++k) {
        for (int j = 0; j <= 1; ++j) {
            for (int i = 0; i <= 1; ++i) {
                float weight = (i == 0 ? 1.0 - wf.x : wf.x) *
                               (j == 0 ? 1.0 - wf.y : wf.y) *
                               (k == 0 ? 1.0 - wf.z : wf.z);
                
                int gx = wi0.x + i;
                int gy = wi0.y + j;
                int gz = wi0.z + k;
                
                if (gx >= 0 && gx < size && gy >= 0 && gy < size && gz >= 0 && gz < size + 1) {
                    int gIdx = getWIdx(gx, gy, gz);
                    atomicAddFloat(gIdx, weight * p.vel.z, 2);
                    atomicAddFloat(gIdx, weight, 5);
                }
            }
        }
    }
}
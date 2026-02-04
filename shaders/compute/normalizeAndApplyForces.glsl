#version 430 core

layout(local_size_x = 256) in;

layout(std430, binding = 1) coherent buffer UBuffer { float[] us; };
layout(std430, binding = 2) coherent buffer VBuffer { float[] vs; };
layout(std430, binding = 4) coherent buffer UMassBuffer { float[] uMasses; };
layout(std430, binding = 5) coherent buffer VMassBuffer { float[] vMasses; };
layout(std430, binding = 7) coherent buffer NewUBuffer { float[] new_us; };
layout(std430, binding = 8) coherent buffer NewVBuffer { float[] new_vs; };

uniform int size;
uniform float gravity;
uniform float dt;

void main() {
    uint idx = gl_GlobalInvocationID.x;
    
    if (idx >= size * (size+1)) return;
    
    float um = uMasses[idx];
    if (um>0.0001f) us[idx] /= um;
    
    float vm = vMasses[idx];
    if (vm>0.0001f) vs[idx] /= vm;

    new_us[idx] = us[idx];
    new_vs[idx] = vs[idx] + gravity * dt;
}
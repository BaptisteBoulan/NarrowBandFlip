#version 430 core

layout(local_size_x = 256)in;

struct SolverParams {
    float dAd;
    float deltaNew;
    float deltaOld;
    float alpha;
};

layout(std430, binding = 12) coherent buffer PressureBuffer  { float[] pressure; };
layout(std430, binding = 15)  coherent buffer AdBuffer        { float[] Ad; };
layout(std430, binding = 14) coherent buffer DirectionBuffer { float[] direction; };
layout(std430, binding = 13) coherent buffer ResidualBuffer  { float[] residual; };
layout(std430, binding = 16) coherent buffer ParamsBuffer    { SolverParams params; };

uniform int size;

void main() {
    int idx = int(gl_GlobalInvocationID.x);
    if (idx >= size*size) return;
    
    pressure[idx] += params.alpha * direction[idx];
    residual[idx] -= params.alpha * Ad[idx];
}
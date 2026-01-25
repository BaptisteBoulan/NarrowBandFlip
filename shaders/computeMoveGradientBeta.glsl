#version 430 core

layout(local_size_x = 256)in;

struct SolverParams {
    float deltaOld;
    float deltaNew;
    float alpha;
    float dAd;
};

layout(std430, binding = 10) coherent buffer DirectionBuffer { float[] direction; };
layout(std430, binding = 12) readonly buffer ResidualBuffer  { float[] residual; };
layout(std430, binding = 13) coherent buffer ParamsBuffer    { SolverParams params; };

uniform int totalSize;

void main() {
    int idx = int(gl_GlobalInvocationID.x);
    if (idx >= totalSize) return;

    float beta = (params.deltaOld > 1e-10) ? params.deltaNew / params.deltaOld : 0.0;
    direction[idx] = residual[idx] + beta * direction[idx];
}
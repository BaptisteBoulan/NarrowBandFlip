#version 430 core

layout(local_size_x = 256)in;

struct SolverParams {
    float dAd;
    float deltaNew;
    float deltaOld;
    float alpha;
};

layout(std430, binding = 14) coherent buffer DirectionBuffer { float[] direction; };
layout(std430, binding = 13) readonly buffer ResidualBuffer  { float[] residual; };


layout(std430, binding = 16) coherent buffer ParamsBuffer    { SolverParams params; };

uniform int size;

void main() {
    int idx = int(gl_GlobalInvocationID.x);
    if (idx >= size*size*size) return;

    float beta = (params.deltaOld > 1e-10) ? params.deltaNew / params.deltaOld : 0.0;
    direction[idx] = residual[idx] + beta * direction[idx];
}
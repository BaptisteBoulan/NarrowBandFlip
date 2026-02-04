#version 430 core

layout(local_size_x = 1) in;

struct SolverParams {
    float dAd;
    float deltaNew;
    float deltaOld;
    float alpha;
};

layout(std430, binding = 16) coherent buffer ParamsBuffer { SolverParams params; };

void main() {
    params.alpha = (params.dAd < 1e-6) ? 0.0f : (params.deltaNew / params.dAd);
    params.deltaOld = params.deltaNew;

    params.deltaNew = 0.0f;
    params.dAd = 0.0f;
}
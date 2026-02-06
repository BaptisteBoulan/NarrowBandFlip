#version 430 core

layout(local_size_x = 8, local_size_y = 8, local_size_z = 8) in;

layout (std430, binding = 13) coherent buffer LevelSetBuffer { float[] levelSet; };
layout (std430, binding = 14) coherent buffer AdvectedLevelSetBuffer { float[] advectedLevelSet; };
layout (std430, binding = 16) coherent buffer ParticlesLevelSetBuffer { int[] particlesLevelSet; };
layout (std430, binding = 17) coherent buffer FinalLevelSetBuffer { float[] finalLevelSet; };

uniform int size;

int getIdx(ivec3 pos) {
    return (pos.z * size * size) + (pos.y * size) + pos.x;
}

void main() {
    ivec3 pos = ivec3(gl_GlobalInvocationID);
    if (pos.x >= size || pos.y >= size || pos.z >= size) return;

    int idx = getIdx(pos);
    float phi_prime = advectedLevelSet[idx];
    float phi_p = float(particlesLevelSet[idx]) / 1000.0f;
    float phi_final = min(phi_prime + 1.0f / size, phi_p);

    finalLevelSet[idx] = phi_final;
    levelSet[idx] = phi_final;
}
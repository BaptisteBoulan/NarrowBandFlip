#version 430 core

layout (local_size_x = 16, local_size_y = 8, local_size_z = 8) in;

layout (std430, binding = 7) coherent buffer NewUBuffer { float[] new_us; };
layout (std430, binding = 8) coherent buffer NewVBuffer { float[] new_vs; };
layout (std430, binding = 9) coherent buffer NewWBuffer { float[] new_ws; };

layout (std430, binding = 10) coherent buffer CellTypeBuffer { uint[] cellType; };
layout (std430, binding = 12) coherent buffer PressureBuffer { float[] pressure; };

uniform int size;
uniform float K;

int gridIdx(int x, int y, int z) {
    return z * size * size + y * size + x;
}

int getUIdx(int i, int j, int k) {
    return k * size * (size + 1) + j * (size + 1) + i;
}

int getVIdx(int i, int j, int k) {
    return k * (size + 1) * size + j * size + i;
}

int getWIdx(int i, int j, int k) {
    return j * size * size + k * size + i;
}

void main() {
    int i = int(gl_GlobalInvocationID.x);
    int j = int(gl_GlobalInvocationID.y);
    int k = int(gl_GlobalInvocationID.z);

    if (i >= size || j >= size || k >= size) return;

    // U VELOCITY (x-direction)
    int idxLeft = gridIdx(i - 1, j, k);
    int idxRight = gridIdx(i, j, k);
    int uIdx = getUIdx(i, j, k);

    if ((cellType[idxLeft] == 2 || cellType[idxRight] == 2) &&
        (cellType[idxLeft] != 0 && cellType[idxRight] != 0)) {
        new_us[uIdx] -= K * (pressure[idxRight] - pressure[idxLeft]);
    } else if (cellType[idxLeft] == 0 || cellType[idxRight] == 0) {
        new_us[uIdx] = 0.0f;
    }

    // V VELOCITY (y-direction)
    int idxBot = gridIdx(i, j - 1, k);
    int idxTop = gridIdx(i, j, k);
    int vIdx = getVIdx(i, j, k);

    if ((cellType[idxBot] == 2 || cellType[idxTop] == 2) &&
        (cellType[idxBot] != 0 && cellType[idxTop] != 0)) {
        new_vs[vIdx] -= K * (pressure[idxTop] - pressure[idxBot]);
    } else if (cellType[idxBot] == 0 || cellType[idxTop] == 0) {
        new_vs[vIdx] = 0.0f;
    }

    // W VELOCITY (z-direction)
    int idxBack = gridIdx(i, j, k - 1);
    int idxFront = gridIdx(i, j, k);
    int wIdx = getWIdx(i, j, k);

    if ((cellType[idxBack] == 2 || cellType[idxFront] == 2) &&
        (cellType[idxBack] != 0 && cellType[idxFront] != 0)) {
        new_ws[wIdx] -= K * (pressure[idxFront] - pressure[idxBack]);
    } else if (cellType[idxBack] == 0 || cellType[idxFront] == 0) {
        new_ws[wIdx] = 0.0f;
    }
}

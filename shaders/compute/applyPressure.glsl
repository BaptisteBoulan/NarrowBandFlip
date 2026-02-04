#version 430 core

layout (local_size_x = 16, local_size_y = 16) in;

layout (std430, binding = 7) coherent buffer NewUBuffer { float[] new_us; };
layout (std430, binding = 8) coherent buffer NewVBuffer { float[] new_vs; };
layout (std430, binding = 10) coherent buffer CellTypeBuffer { uint[] cellType; };
layout (std430, binding = 12) coherent buffer PressureBuffer { float[] pressure; };

uniform int size;
uniform float K;

int gridIdx(int x, int y, int z) {return z * size *size + y * size + x;}
int uIdx(int i, int j) { return j * (size+1) + i; }
int vIdx(int i, int j) { return j * size + i; }

void main() {
    int i = int(gl_GlobalInvocationID.x);
    int j = int(gl_GlobalInvocationID.y);

    if (i > size || j >= size) return;

    // U VELOCITY

    int idxLeft = gridIdx(i - 1, j, 0);
    int idxRight = gridIdx(i, j, 0);
    int uIdx = uIdx(i, j);

    if ((cellType[idxLeft] == 2 || cellType[idxRight] == 2) &&
        (cellType[idxLeft] != 0 && cellType[idxRight] != 0)) {

        new_us[uIdx] -= K * (pressure[idxRight] - pressure[idxLeft]);
    } else if (cellType[idxLeft] == 0 || cellType[idxRight] == 0) {
        new_us[uIdx] = 0.0f;
    }

    // V VELOCITY (swap i and j to keep the same bounds)
    int k = i;
    i = j;
    j = k;

    int idxBot = gridIdx(i, j - 1, 0);
    int idxTop = gridIdx(i, j, 0);
    int vIdx = vIdx(i, j);

    if ((cellType[idxBot] == 2 || cellType[idxTop] == 2) &&
        (cellType[idxBot] != 0 && cellType[idxTop] != 0)) {
        
        new_vs[vIdx] -= K * (pressure[idxTop] - pressure[idxBot]);
    }
    else if (cellType[idxBot] == 0 || cellType[idxTop] == 0) {
        new_vs[vIdx] = 0.0f;
    }
}

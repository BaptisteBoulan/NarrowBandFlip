#version 430 core

layout(local_size_x = 8, local_size_y = 8, local_size_z = 8) in;

layout(std430, binding = 7) coherent buffer NewUBuffer { float new_us[]; };
layout(std430, binding = 8) coherent buffer NewVBuffer { float new_vs[]; };
layout(std430, binding = 9) coherent buffer NewWBuffer { float new_ws[]; };
layout(std430, binding = 10) coherent buffer CellType   { uint cellType[]; };

layout(std430, binding = 13) buffer LevelSetBuffer { float levelSet[]; };

uniform int size;

int gridIdx(int x, int y, int z) {return z * size *size + y * size + x;}

int getUIdx(int i, int j, int k) { return k * size * (size + 1) + j * (size + 1) + i; }
int getVIdx(int i, int j, int k) { return k * (size + 1) * size + j * size + i; }
int getWIdx(int i, int j, int k) { return k * size * size + j * size + i; }

void main() {

    int i = int(gl_GlobalInvocationID.x);
    int j = int(gl_GlobalInvocationID.y);
    int k = int(gl_GlobalInvocationID.z);

    if (i >= size || j >= size || k >= size ) return;

    int cellIdx = gridIdx(i,j,k);

    int type = int(cellType[cellIdx]);

    if (type != 0) {
        cellType[cellIdx] = (levelSet[cellIdx] < 5.0f / size) ? 1 : 2; // If not SOLID then AIR/FLUID
    }
    else {
        new_us[getUIdx(i + 1, j, k)] = 0.0f;
        new_us[getUIdx(i, j, k)] = 0.0f;
        new_vs[getVIdx(i, j + 1, k)] = 0.0f;
        new_vs[getVIdx(i, j, k)] = 0.0f;
        new_ws[getWIdx(i, j, k + 1)] = 0.0f;
        new_ws[getWIdx(i, j, k)] = 0.0f;
    }
}
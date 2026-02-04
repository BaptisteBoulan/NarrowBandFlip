#version 430 core

layout(local_size_x = 16, local_size_y = 16) in;

layout(std430, binding = 5) coherent buffer NewUBuffer { float new_us[]; };
layout(std430, binding = 6) coherent buffer NewVBuffer { float new_vs[]; };
layout(std430, binding = 8) coherent buffer CellType   { uint cellType[]; };

uniform int size;

int gridIdx(int x, int y, int z) {return z * size *size + y * size + x;}
int uIdx(int i, int j) { return j * (size+1) + i; }
int vIdx(int i, int j) { return j * size + i; }

void main() {

    int i = int(gl_GlobalInvocationID.x);
    int j = int(gl_GlobalInvocationID.y);

    if (i >= size || j >= size) return;

    int cellIdx = gridIdx(i,j,0);

    int type = int(cellType[cellIdx]);

    if (type != 0) cellType[cellIdx] = 1; // If not SOLID then AIR
    else {
        new_us[uIdx(i + 1, j)] = 0.0f;
        new_us[uIdx(i, j)] = 0.0f;
        new_vs[vIdx(i, j + 1)] = 0.0f;
        new_vs[vIdx(i, j)] = 0.0f;
    }
}
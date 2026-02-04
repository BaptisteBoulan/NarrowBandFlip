#version 430 core

layout(local_size_x = 8, local_size_y = 8, local_size_z = 8) in;

layout(std430, binding = 10) buffer CellTypeBuffer { int cellType[]; };
layout(std430, binding = 14) buffer DirectionBuffer { float direction[]; };
layout(std430, binding = 15) buffer AdBuffer { float Ad[]; };

uniform int size;

int gridIdx(int x, int y, int z) {return z * size * size + y * size + x;}

void main() {
    int i = int(gl_GlobalInvocationID.x);
    int j = int(gl_GlobalInvocationID.y);
    int k = int(gl_GlobalInvocationID.z);

    if (i >= size || j >= size || k >= size) return;


    int idx = gridIdx(i,j,k);

    if (cellType[idx] != 2) {
        Ad[idx] = 0.0f;
        return;
    }

    float val = 0.0f;
    int neighborsCount = 0;

    int ni[6] = {i + 1, i - 1, i, i, i, i};
    int nj[6] = {j, j, j + 1, j - 1, j, j};
    int nk[6] = {k, k, k, k, k + 1, k - 1};

    for (int s = 0; s<6; s++) {
        int curI = ni[s];
        int curJ = nj[s];
        int curK = nk[s];
        // Boundary check
        if (curI >= 0 && curI < size && curJ >= 0 && curJ < size  && curK >= 0 && curK < size) {
            int nIdx = gridIdx(curI, curJ, curK);

            if (cellType[nIdx] != 0) {
                neighborsCount++;

                if (cellType[nIdx] == 2) {
                    val -= direction[nIdx];
                }
            }
        }
    }
    Ad[idx] = (float(neighborsCount) * direction[idx]) + val;
}
#version 430 core

layout(local_size_x = 16, local_size_y = 16) in;

layout(std430, binding = 8) buffer CellTypeBuffer { int cellType[]; };
layout(std430, binding = 9) buffer AdBuffer { float Ad[]; };
layout(std430, binding = 10) buffer DirectionBuffer { float direction[]; };

uniform int size;

int gridIdx(int x, int y, int z) {return z * size *size + y * size + x;}

void main() {
    int i = int(gl_GlobalInvocationID.x);
    int j = int(gl_GlobalInvocationID.y);

    if (i >= size || j >= size) return;


    int idx = gridIdx(i,j,0);

    if (cellType[idx] != 2) {
        Ad[idx] = 0.0f;
        return;
    }

    float val = 0.0f;
    int neighborsCount = 0;

    int ni[4] = {i + 1, i - 1, i, i};
    int nj[4] = {j, j, j + 1, j - 1};

    for (int k = 0; k<4; k++) {
        int curI = ni[k];
        int curJ = nj[k];
        // Boundary check
        if (curI >= 0 && curI < size && curJ >= 0 && curJ < size) {
            int nIdx = gridIdx(curI, curJ, 0);

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
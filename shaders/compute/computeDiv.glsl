#version 430 core

layout (local_size_x = 16, local_size_y = 16) in;

layout (std430, binding = 7) coherent buffer NewUBuffer { float[] new_us; };
layout (std430, binding = 8) coherent buffer NewVBuffer { float[] new_vs; };
layout (std430, binding = 10) coherent buffer CellTypeBuffer { uint[] cellType; };
layout (std430, binding = 11) coherent buffer DivBuffer { float[] div; };

uniform int size;
uniform float dt;
uniform float h;
uniform float rho;


int gridIdx(int x, int y, int z) {return z * size *size + y * size + x;}
int uIdx(int i, int j) { return j * (size+1) + i; }
int vIdx(int i, int j) { return j * size + i; }


void main() {

    int i = int(gl_GlobalInvocationID.x);
    int j = int(gl_GlobalInvocationID.y);

    if (i >= size || j >= size) return;

    int idx = gridIdx(i,j,0);


    if (cellType[idx] == 0) {
        div[idx] = 0.0f;;
        return;
    }

    float u_right = new_us[uIdx(i + 1, j)];
    float u_left  = new_us[uIdx(i, j)];
    float v_top   = new_vs[vIdx(i, j + 1)];
    float v_bot   = new_vs[vIdx(i, j)];

    float d = (u_right - u_left) + (v_top - v_bot);
    div[idx] = (d / h) * (rho / dt);
}
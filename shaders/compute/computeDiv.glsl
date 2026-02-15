#version 430 core

layout (local_size_x = 16, local_size_y = 8, local_size_z = 8) in;

layout (std430, binding = 7) coherent buffer NewUBuffer { float[] new_us; };
layout (std430, binding = 8) coherent buffer NewVBuffer { float[] new_vs; };
layout (std430, binding = 9) coherent buffer NewWBuffer { float[] new_ws; };

layout (std430, binding = 10) coherent buffer CellTypeBuffer { uint[] cellType; };
layout (std430, binding = 11) coherent buffer DivBuffer { float[] div; };

uniform int size;
uniform float dt;
uniform float h;
uniform float rho;

int gridIdx(int x, int y, int z) { return z * size * size + y * size + x; }

int getUIdx(int i, int j, int k) { return k * size * (size + 1) + j * (size + 1) + i; }
int getVIdx(int i, int j, int k) { return k * (size + 1) * size + j * size + i; }
int getWIdx(int i, int j, int k) { return k * size * size + j * size + i; }

void main() {
    int i = int(gl_GlobalInvocationID.x);
    int j = int(gl_GlobalInvocationID.y);
    int k = int(gl_GlobalInvocationID.z);

    if (i >= size || j >= size || k >= size) return;

    int idx = gridIdx(i, j, k);

    if (cellType[idx] == 0) {
        div[idx] = 0.0f;
        return;
    }

    float u_right = new_us[getUIdx(i + 1, j, k)];
    float u_left  = new_us[getUIdx(i, j, k)];

    float v_top   = new_vs[getVIdx(i, j + 1, k)];
    float v_bot   = new_vs[getVIdx(i, j, k)];

    float w_front = new_ws[getWIdx(i, j, k + 1)];
    float w_back  = new_ws[getWIdx(i, j, k)];

    float d = (u_right - u_left) + (v_top - v_bot) + (w_front - w_back);
    div[idx] = (d / h) * (rho / dt);
}

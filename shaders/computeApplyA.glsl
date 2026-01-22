#version 430 core
layout(local_size_x = 16, local_size_y = 16) in;

layout(std430, binding = 5) buffer PressureBuffer { float p[]; };
layout(std430, binding = 6) buffer AxBuffer { float Ax[]; };
layout(std430, binding = 7) buffer CellTypeBuffer { int types[]; };

uniform int size;

void main() {
    ivec2 curr = ivec2(gl_GlobalInvocationID.xy);
    if (curr.x >= size || curr.y >= size) return;

    int idx = curr.y * size + curr.x;
    if (types[idx] != 1) { // 1 = FLUID
        Ax[idx] = 0.0;
        return;
    }

    // Laplacian Operator logic
    // ... calculate neighbors and sum ...
}
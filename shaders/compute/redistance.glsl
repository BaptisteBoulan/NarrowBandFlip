#version 430 core
layout(local_size_x = 8, local_size_y = 8, local_size_z = 8) in;

layout(std430, binding = 10) readonly buffer CellTypeBuffer { uint cellType[]; };
layout(std430, binding = 17) readonly buffer LevelSetIn { float levelSetIn[]; };
layout(std430, binding = 14) writeonly buffer LevelSetOut { float levelSetOut[]; };

uniform int size;

int getIdx(ivec3 pos) {
    if (any(lessThan(pos, ivec3(0))) || any(greaterThanEqual(pos, ivec3(size)))) {
        // Out of bounds indices are invalid.
        return -1;
    }
    return (pos.z * size * size) + (pos.y * size) + pos.x;
}

void main() {
    ivec3 pos = ivec3(gl_GlobalInvocationID);
    if (pos.x >= size || pos.y >= size || pos.z >= size) return;

    int center_idx = getIdx(pos);
    
    // Only process FLUID cells (Type 2). AIR (1) and SOLID (0) stay at 0.0 distance.
    if (cellType[center_idx] != 2) {
        return;
    }

    float minPhi = levelSetIn[center_idx]; // Start with a large value

    for (int zk = -1; zk <= 1; zk++) {
        for (int yk = -1; yk <= 1; yk++) {
            for (int xk = -1; xk <= 1; xk++) {
                if (xk == 0 && yk == 0 && zk == 0) continue;
                
                ivec3 nPos = pos + ivec3(xk, yk, zk);
                int neighbor_idx = getIdx(nPos);
                if (neighbor_idx == -1) continue;
                if (cellType[neighbor_idx] == 0) continue;

                float d = length(vec3(xk, yk, zk)) / size;
                float neighborPhi = levelSetIn[neighbor_idx];
                
                minPhi = min(minPhi, neighborPhi + d);
            }
        }
    }
    levelSetOut[center_idx] = minPhi;
}

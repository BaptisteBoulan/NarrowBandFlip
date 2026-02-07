#version 430 core
layout(local_size_x = 8, local_size_y = 8, local_size_z = 8) in;

// We "ping-pong" by reading from one buffer and writing to the other.
// In our C++ code, LevelSetIn will be finalLevelSet (17) and LevelSetOut will be newLevelSet (14)
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
    float phi = levelSetIn[center_idx];
    // The distance between cell centers is h. In world units, this is 1.0/size.
    float h_dist = 1.0f / float(size);

    float newPhi = phi;

    ivec3 neighbors[6] = ivec3[6](
        ivec3(-1, 0, 0), ivec3(1, 0, 0),
        ivec3(0, -1, 0), ivec3(0, 1, 0),
        ivec3(0, 0, -1), ivec3(0, 0, 1)
    );

    // This loop propagates distance from neighbors.
    // It's a parallel version of a Jacobi iteration to solve the Eikonal equation (|grad(phi)|=1).
    for (int i = 0; i < 6; ++i) {
        int neighbor_idx = getIdx(pos + neighbors[i]);
        if (neighbor_idx == -1) continue;

        float neighborPhi = levelSetIn[neighbor_idx];

        // Only update from neighbors that have the same sign.
        // This is crucial to prevent the zero-crossing (the surface) from moving.
        if (phi * neighborPhi > 0) {
            if (phi > 0) {
                // We are in an AIR cell, looking for the closest distance to the surface.
                // If the neighbor is closer, our distance is at most neighbor's + h.
                newPhi = min(newPhi, neighborPhi + h_dist);
            } else {
                // We are in a FLUID cell, looking for the "deepest" distance from the surface.
                // If the neighbor is less deep, our distance is at most neighbor's - h.
                newPhi = max(newPhi, neighborPhi - h_dist);
            }
        }
    }
    
    levelSetOut[center_idx] = newPhi;
}

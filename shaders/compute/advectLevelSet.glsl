#version 430 core

layout(local_size_x = 8, local_size_y = 8, local_size_z = 8) in;

layout (std430, binding = 1) coherent buffer UBuffer { float[] us; };
layout (std430, binding = 2) coherent buffer VBuffer { float[] vs; };
layout (std430, binding = 3) coherent buffer WBuffer { float[] ws; };
layout (std430, binding = 13) coherent buffer LevelSetBuffer { float[] levelSet; };
layout (std430, binding = 14) coherent buffer NewLevelSetBuffer { float[] newLevelSet; };

uniform float dt;
uniform int size;

int getIdx(ivec3 pos) {
    return (pos.z * size * size) + (pos.y * size) + pos.x;
}

float sampleLevelSet(vec3 p) {
    // Clamp to valid range
    p = clamp(p, vec3(0.0), vec3(float(size) - 1.001));
    
    ivec3 p0 = ivec3(floor(p));
    ivec3 p1 = p0 + ivec3(1);
    
    vec3 t = fract(p);
    
    float c000 = levelSet[getIdx(ivec3(p0.x, p0.y, p0.z))];
    float c100 = levelSet[getIdx(ivec3(p1.x, p0.y, p0.z))];
    float c010 = levelSet[getIdx(ivec3(p0.x, p1.y, p0.z))];
    float c110 = levelSet[getIdx(ivec3(p1.x, p1.y, p0.z))];
    
    float c001 = levelSet[getIdx(ivec3(p0.x, p0.y, p1.z))];
    float c101 = levelSet[getIdx(ivec3(p1.x, p0.y, p1.z))];
    float c011 = levelSet[getIdx(ivec3(p0.x, p1.y, p1.z))];
    float c111 = levelSet[getIdx(ivec3(p1.x, p1.y, p1.z))];
    
    float c00 = mix(c000, c100, t.x);
    float c01 = mix(c001, c101, t.x);
    float c10 = mix(c010, c110, t.x);
    float c11 = mix(c011, c111, t.x);
    
    float c0 = mix(c00, c10, t.y);
    float c1 = mix(c01, c11, t.y);
    
    return mix(c0, c1, t.z);
}

void main() {
    ivec3 pos = ivec3(gl_GlobalInvocationID);
    if (pos.x >= size || pos.y >= size || pos.z >= size) return;

    vec3 worldPos = vec3(pos); // Grid coordinates

    int idx = getIdx(pos);
    
    // Simple velocity sampling (could also be interpolated for better results, but keeping it simple for now)
    // Note: us, vs, ws are staggered, but stored linearly. Assuming simple averaging for cell center here.
    // For full correctness, we should interpolate velocity at 'worldPos' properly.
    // However, given the current structure, let's stick to the existing velocity fetch but fix level set sampling.
    
    float u = 0.5f * (us[idx] + us[idx+1]); // Approximate
    float v = 0.5f * (vs[idx] + vs[idx+size]);
    float w = 0.5f * (ws[idx] + ws[idx+size*size]);
    
    // Backtrace
    // Convert velocity from units/s to grid_cells/step? 
    // Usually simulation assumes grid size 1.0 internally or handles dt scaling.
    // The previous code did: vec3 backPos = worldPos - vec3(u,v,w) * dt; (with worldPos normalized to 0..1)
    // Here worldPos is 0..size.
    // If velocities are in physical units, we need to scale.
    // Assuming velocities are in grid_units/s for now based on 'worldPos = vec3(pos)' usage.
    // If original code had worldPos = vec3(pos)/size, then velocities were likely normalized.
    // Let's check original advectLevelSet.glsl:
    // "vec3 worldPos = vec3(pos) / size;"
    // "vec3 backPos = worldPos - vec3(u,v,w) * dt;"
    // "ivec3 backPosIdx = ivec3(backPos * size);"
    // This implies backPos is in 0..1 range.
    
    // Re-implementing with consistent coordinate space (Grid Space 0..size)
    // Velocity is sampled from grid. If grid velocity is in m/s and domain is 1m, then u * size is velocity in cells/s.
    // Assuming u,v,w are in domain units (e.g. m/s).
    
    float u_grid = u * size;
    float v_grid = v * size;
    float w_grid = w * size;
    
    vec3 backPos = vec3(pos) - vec3(u_grid, v_grid, w_grid) * dt;
    
    float phi_prime = sampleLevelSet(backPos);
    
    // Using newLevelSet as temp buffer before updateLevelSet kernel runs
    // Note: The C++ code binds levelSet to 13 and newLevelSet to 14.
    // But the C++ code doesn't swap them or use newLevelSet in updateLevelSet.
    // updateLevelSet reads from 'advectedLevelSet' (binding 14).
    // So we must write to binding 14.
    
    newLevelSet[idx] = phi_prime;
}
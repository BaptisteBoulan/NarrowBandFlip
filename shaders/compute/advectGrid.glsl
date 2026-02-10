#version 430 core

layout(local_size_x = 8, local_size_y = 8, local_size_z = 8) in;

// We advect the velocity from the previous step (which is in new_us after projection)
layout(std430, binding = 7) readonly buffer OldUBuffer { float old_us[]; };
layout(std430, binding = 8) readonly buffer OldVBuffer { float old_vs[]; };
layout(std430, binding = 9) readonly buffer OldWBuffer { float old_ws[]; };

layout(std430, binding = 22) writeonly buffer UAdvBuffer { float u_adv[]; };
layout(std430, binding = 23) writeonly buffer VAdvBuffer { float v_adv[]; };
layout(std430, binding = 24) writeonly buffer WAdvBuffer { float w_adv[]; };

uniform int size;
uniform float dt;
uniform float h;

// Indexing helpers
int getUIdx(int i, int j, int k) { return k * size * (size + 1) + j * (size + 1) + i; }
int getVIdx(int i, int j, int k) { return k * (size + 1) * size + j * size + i; }
int getWIdx(int i, int j, int k) { return k * size * size + j * size + i; }

float sampleU(vec3 p) {
    float x = p.x;
    float y = p.y - 0.5;
    float z = p.z - 0.5;

    int i = int(floor(x));
    int j = int(floor(y));
    int k = int(floor(z));

    float fx = x - float(i);
    float fy = y - float(j);
    float fz = z - float(k);

    float val = 0.0;
    for (int dk = 0; dk <= 1; ++dk) {
        for (int dj = 0; dj <= 1; ++dj) {
            for (int di = 0; di <= 1; ++di) {
                int gx = clamp(i + di, 0, size);
                int gy = clamp(j + dj, 0, size - 1);
                int gz = clamp(k + dk, 0, size - 1);
                float weight = (di == 0 ? 1.0 - fx : fx) *
                               (dj == 0 ? 1.0 - fy : fy) *
                               (dk == 0 ? 1.0 - fz : fz);
                val += weight * old_us[getUIdx(gx, gy, gz)];
            }
        }
    }
    return val;
}

float sampleV(vec3 p) {
    float x = p.x - 0.5;
    float y = p.y;
    float z = p.z - 0.5;

    int i = int(floor(x));
    int j = int(floor(y));
    int k = int(floor(z));

    float fx = x - float(i);
    float fy = y - float(j);
    float fz = z - float(k);

    float val = 0.0;
    for (int dk = 0; dk <= 1; ++dk) {
        for (int dj = 0; dj <= 1; ++dj) {
            for (int di = 0; di <= 1; ++di) {
                int gx = clamp(i + di, 0, size - 1);
                int gy = clamp(j + dj, 0, size);
                int gz = clamp(k + dk, 0, size - 1);
                float weight = (di == 0 ? 1.0 - fx : fx) *
                               (dj == 0 ? 1.0 - fy : fy) *
                               (dk == 0 ? 1.0 - fz : fz);
                val += weight * old_vs[getVIdx(gx, gy, gz)];
            }
        }
    }
    return val;
}

float sampleW(vec3 p) {
    float x = p.x - 0.5;
    float y = p.y - 0.5;
    float z = p.z;

    int i = int(floor(x));
    int j = int(floor(y));
    int k = int(floor(z));

    float fx = x - float(i);
    float fy = y - float(j);
    float fz = z - float(k);

    float val = 0.0;
    for (int dk = 0; dk <= 1; ++dk) {
        for (int dj = 0; dj <= 1; ++dj) {
            for (int di = 0; di <= 1; ++di) {
                int gx = clamp(i + di, 0, size - 1);
                int gy = clamp(j + dj, 0, size - 1);
                int gz = clamp(k + dk, 0, size);
                float weight = (di == 0 ? 1.0 - fx : fx) *
                               (dj == 0 ? 1.0 - fy : fy) *
                               (dk == 0 ? 1.0 - fz : fz);
                val += weight * old_ws[getWIdx(gx, gy, gz)];
            }
        }
    }
    return val;
}

vec3 sampleVelocity(vec3 p) {
    return vec3(sampleU(p), sampleV(p), sampleW(p));
}

void main() {
    ivec3 pos = ivec3(gl_GlobalInvocationID);
    
    // U-velocity advection
    if (pos.x <= size && pos.y < size && pos.z < size) {
        vec3 worldPos = vec3(pos.x, float(pos.y) + 0.5, float(pos.z) + 0.5);
        vec3 v = sampleVelocity(worldPos);
        vec3 backtrackedPos = worldPos - v * (dt / h);
        u_adv[getUIdx(pos.x, pos.y, pos.z)] = sampleU(backtrackedPos);
    }

    // V-velocity advection
    if (pos.x < size && pos.y <= size && pos.z < size) {
        vec3 worldPos = vec3(float(pos.x) + 0.5, pos.y, float(pos.z) + 0.5);
        vec3 v = sampleVelocity(worldPos);
        vec3 backtrackedPos = worldPos - v * (dt / h);
        v_adv[getVIdx(pos.x, pos.y, pos.z)] = sampleV(backtrackedPos);
    }

    // W-velocity advection
    if (pos.x < size && pos.y < size && pos.z <= size) {
        vec3 worldPos = vec3(float(pos.x) + 0.5, float(pos.y) + 0.5, pos.z);
        vec3 v = sampleVelocity(worldPos);
        vec3 backtrackedPos = worldPos - v * (dt / h);
        w_adv[getWIdx(pos.x, pos.y, pos.z)] = sampleW(backtrackedPos);
    }
}

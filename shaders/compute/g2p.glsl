#version 430 core

layout(local_size_x = 256) in;

struct Particle {
    vec3 pos;
    vec3 vel;
};

// Buffers
layout(std430, binding = 0) buffer ParticleBuffer { Particle particles[]; };

layout(std430, binding = 1) coherent buffer UBuffer { uint us[]; };
layout(std430, binding = 2) coherent buffer VBuffer { uint vs[]; };
layout(std430, binding = 3) coherent buffer WBuffer { uint ws[]; };

layout(std430, binding = 7) coherent buffer NewUBuffer { float new_us[]; };
layout(std430, binding = 8) coherent buffer NewVBuffer { float new_vs[]; };
layout(std430, binding = 9) coherent buffer NewWBuffer { float new_ws[]; };

layout(std430, binding = 10) buffer CellTypeBuffer { int cellType[]; };

uniform int size;
uniform int numParticles;
uniform float dt;
uniform float h;
uniform float alpha;

// Indexing helpers
int gridIdx(int x, int y, int z) { return z * size * size + y * size + x; }
int getUIdx(int i, int j, int k) { return k * size * (size + 1) + j * (size + 1) + i; }
int getVIdx(int i, int j, int k) { return k * (size + 1) * size + j * size + i; }
int getWIdx(int i, int j, int k) { return k * size * size + j * size + i; }

void main() {

    uint idx = gl_GlobalInvocationID.x;
    if (idx >= numParticles) return;

    Particle p = particles[idx];

    float px = p.pos.x * size;
    float py = p.pos.y * size;
    float pz = p.pos.z * size;

    // --- U-VELOCITIES (Staggered in X) ---
    // Center: (0, -0.5, -0.5) relative to cell center
    float ux = px;
    float uy = py - 0.5f;
    float uz = pz - 0.5f;
    
    int ui = int(floor(ux));
    int uj = int(floor(uy));
    int uk = int(floor(uz));
    
    float uwx = ux - float(ui);
    float uwy = uy - float(uj);
    float uwz = uz - float(uk);

    // Trilinear Interpolation for U
    float pic_u = 0.0f;
    float flip_u = p.vel.x;

    for (int k = 0; k <= 1; k++) {
        for (int j = 0; j <= 1; j++) {
            for (int i = 0; i <= 1; i++) {
                int index = getUIdx(ui + i, uj + j, uk + k);
                if (index >= 0 && index < (size + 1) * size * size) {
                    float weight = (i == 0 ? (1.0 - uwx) : uwx) *
                                   (j == 0 ? (1.0 - uwy) : uwy) *
                                   (k == 0 ? (1.0 - uwz) : uwz);
                    
                    float new_val = new_us[index];
                    float old_val = us[index]; // Note: Ensure 'us' buffer contains floats or use uintbitsToFloat if strictly uint buffer
                    
                    pic_u += new_val * weight;
                    flip_u += (new_val - old_val) * weight;
                }
            }
        }
    }

    // --- V-VELOCITIES (Staggered in Y) ---
    // Center: (-0.5, 0, -0.5) relative to cell center
    float vx = px - 0.5f;
    float vy = py;
    float vz = pz - 0.5f;

    int vi = int(floor(vx));
    int vj = int(floor(vy));
    int vk = int(floor(vz));

    float vwx = vx - float(vi);
    float vwy = vy - float(vj);
    float vwz = vz - float(vk);

    float pic_v = 0.0f;
    float flip_v = p.vel.y;

    for (int k = 0; k <= 1; k++) {
        for (int j = 0; j <= 1; j++) {
            for (int i = 0; i <= 1; i++) {
                int index = getVIdx(vi + i, vj + j, vk + k);
                if (index >= 0 && index < size * (size + 1) * size) {
                    float weight = (i == 0 ? (1.0 - vwx) : vwx) *
                                   (j == 0 ? (1.0 - vwy) : vwy) *
                                   (k == 0 ? (1.0 - vwz) : vwz);

                    float new_val = new_vs[index];
                    float old_val = vs[index];

                    pic_v += new_val * weight;
                    flip_v += (new_val - old_val) * weight;
                }
            }
        }
    }

    // --- W-VELOCITIES (Staggered in Z) ---
    // Center: (-0.5, -0.5, 0) relative to cell center
    float wx = px - 0.5f;
    float wy = py - 0.5f;
    float wz = pz;

    int wi = int(floor(wx));
    int wj = int(floor(wy));
    int wk = int(floor(wz));

    float wwx = wx - float(wi);
    float wwy = wy - float(wj);
    float wwz = wz - float(wk);

    float pic_w = 0.0f;
    float flip_w = p.vel.z;

    for (int k = 0; k <= 1; k++) {
        for (int j = 0; j <= 1; j++) {
            for (int i = 0; i <= 1; i++) {
                int index = getWIdx(wi + i, wj + j, wk + k);
                // Check bounds (W grid usually has size+1 in Z)
                if (index >= 0 && index < size * size * (size + 1)) {
                    float weight = (i == 0 ? (1.0 - wwx) : wwx) *
                                   (j == 0 ? (1.0 - wwy) : wwy) *
                                   (k == 0 ? (1.0 - wwz) : wwz);

                    float new_val = new_ws[index];
                    float old_val = ws[index];

                    pic_w += new_val * weight;
                    flip_w += (new_val - old_val) * weight;
                }
            }
        }
    }

    // Update Velocity
    p.vel.x = (1.0f - alpha) * pic_u + alpha * flip_u;
    p.vel.y = (1.0f - alpha) * pic_v + alpha * flip_v;
    p.vel.z = (1.0f - alpha) * pic_w + alpha * flip_w;

    // Advect Position
    p.pos += p.vel * dt;

    // --- 3D COLLISION HANDLING ---
    int cellX = int(p.pos.x * size);
    int cellY = int(p.pos.y * size);
    int cellZ = int(p.pos.z * size);

    cellX = max(0, min(size - 1, cellX));
    cellY = max(0, min(size - 1, cellY));
    cellZ = max(0, min(size - 1, cellZ));

    // Check if current cell is solid (0 usually implies solid/boundary in this context)
    if (cellType[gridIdx(cellX, cellY, cellZ)] == 0) {
        float cellMinX = float(cellX) / size;
        float cellMaxX = float(cellX + 1) / size;
        float cellMinY = float(cellY) / size;
        float cellMaxY = float(cellY + 1) / size;
        float cellMinZ = float(cellZ) / size;
        float cellMaxZ = float(cellZ + 1) / size;

        // Calculate distance to 6 faces
        // If the neighbor is ALSO solid (or out of bounds), set distance to high value (2.0f) to prevent pushing that way
        float distLeft   = (p.pos.x < h          || cellType[gridIdx(max(0, cellX-1), cellY, cellZ)] == 0) ? 2.0f : p.pos.x - cellMinX;
        float distRight  = (p.pos.x > 1.0f - h   || cellType[gridIdx(min(size-1, cellX+1), cellY, cellZ)] == 0) ? 2.0f : cellMaxX - p.pos.x;
        
        float distBottom = (p.pos.y < h          || cellType[gridIdx(cellX, max(0, cellY-1), cellZ)] == 0) ? 2.0f : p.pos.y - cellMinY;
        float distTop    = (p.pos.y > 1.0f - h   || cellType[gridIdx(cellX, min(size-1, cellY+1), cellZ)] == 0) ? 2.0f : cellMaxY - p.pos.y;
        
        float distBack   = (p.pos.z < h          || cellType[gridIdx(cellX, cellY, max(0, cellZ-1))] == 0) ? 2.0f : p.pos.z - cellMinZ;
        float distFront  = (p.pos.z > 1.0f - h   || cellType[gridIdx(cellX, cellY, min(size-1, cellZ+1))] == 0) ? 2.0f : cellMaxZ - p.pos.z;

        float minDist = distLeft;
        int side = 0;

        if (distRight < minDist)  { minDist = distRight;  side = 1; }
        if (distBottom < minDist) { minDist = distBottom; side = 2; }
        if (distTop < minDist)    { minDist = distTop;    side = 3; }
        if (distBack < minDist)   { minDist = distBack;   side = 4; }
        if (distFront < minDist)  { minDist = distFront;  side = 5; }

        float eps = 1e-4f;

        switch (side) {
            case 0: // Left X
                p.pos.x = cellMinX - eps;
                p.vel.x = 0; 
                break;
            case 1: // Right X
                p.pos.x = cellMaxX + eps;
                p.vel.x = 0;
                break;
            case 2: // Bottom Y
                p.pos.y = cellMinY - eps;
                p.vel.y = 0;
                break;
            case 3: // Top Y
                p.pos.y = cellMaxY + eps;
                p.vel.y = 0;
                break;
            case 4: // Back Z
                p.pos.z = cellMinZ - eps;
                p.vel.z = 0;
                break;
            case 5: // Front Z
                p.pos.z = cellMaxZ + eps;
                p.vel.z = 0;
                break;
        }
        
        // Final clamp to ensure particle stays in domain
        p.pos.x = min(1.0f - h, max(h, p.pos.x));
        p.pos.y = min(1.0f - h, max(h, p.pos.y));
        p.pos.z = min(1.0f - h, max(h, p.pos.z));
    }

    particles[idx] = p;
}
#version 430 core

layout(local_size_x = 256) in;

struct Particle {
    vec2 pos;
    vec2 vel;
};

// Buffers
layout(std430, binding = 0) buffer ParticleBuffer { Particle particles[]; };
layout(std430, binding = 1) coherent buffer UBuffer         { float us[]; };
layout(std430, binding = 2) coherent buffer VBuffer         { float vs[]; };
layout(std430, binding = 5) coherent buffer NewUBuffer         { float new_us[]; };
layout(std430, binding = 6) coherent buffer NewVBuffer         { float new_vs[]; };
layout(std430, binding = 8) buffer CellTypeBuffer { int cellType[]; };

uniform int size;
uniform int numParticles;
uniform float dt;
uniform float h;

// Indexing helpers matching the CPU logic
int getUIdx(int i, int j) { return j * (size + 1) + i; }
int getVIdx(int i, int j) { return j * size + i; }
int gridIdx(int i, int j) { return j * size + i; }

float alpha = 0.95f;

void main() {

    uint idx = gl_GlobalInvocationID.x;
    if (idx >= numParticles) return;

    Particle p = particles[idx];

    float px = p.pos.x * size;
    float py = p.pos.y * size;

    // U-VELOCITIES
    float ux = px;
    float uy = py - 0.5f;
    
    int ui = int(ux);
    int uj = int(uy);
    float uwx = ux - ui;
    float uwy = uy - uj;

    // Bilinear weights for U
    float weights_u[4] = {
        (1 - uwx) * (1 - uwy), // (i, j)
        uwx * (1 - uwy),       // (i+1, j)
        (1 - uwx) * uwy,       // (i, j+1)
        uwx * uwy              // (i+1, j+1)
    };
    int u_indices[4] = { getUIdx(ui, uj), getUIdx(ui+1, uj), getUIdx(ui, uj+1), getUIdx(ui+1, uj+1) };
    // Transfer U
    float pic_u = 0.0f;
    float flip_u = p.vel.x;
    
    for(int k=0; k<4; ++k) {
        if (u_indices[k] < 0 || u_indices[k] >= size*(size+1)) continue;
        float new_u = new_us[u_indices[k]];
        float old_u = us[u_indices[k]];

        pic_u += new_u * weights_u[k];
        flip_u += (new_u - old_u) * weights_u[k];
    }

    // V-VELOCITIES
    float vx = px - 0.5f;
    float vy = py;

    int vi = int(vx);
    int vj = int(vy);
    float vwx = vx - vi;
    float vwy = vy - vj;

    // Bilinear weights for V
    float weights_v[4] = {
        (1 - vwx) * (1 - vwy), // (i, j)
        vwx * (1 - vwy),       // (i+1, j)
        (1 - vwx) * vwy,       // (i, j+1)
        vwx * vwy              // (i+1, j+1)
    };
    int v_indices[4] = { getVIdx(vi, vj), getVIdx(vi+1, vj), getVIdx(vi, vj+1), getVIdx(vi+1, vj+1) };
    
    // Transfer V
    float pic_v = 0.0f;
    float flip_v = p.vel.y;
    
    for(int k=0; k<4; ++k) {
        if (v_indices[k] < 0 || v_indices[k] >= size*(size+1)) continue;
        float new_v = new_vs[v_indices[k]];
        float old_v = vs[v_indices[k]];

        pic_v += new_v * weights_v[k];
        flip_v += (new_v - old_v) * weights_v[k];
    }

    p.vel.x = (1.0f - alpha) * pic_u + alpha * flip_u;
    p.vel.y = (1.0f - alpha) * pic_v + alpha * flip_v;

    p.pos += p.vel * dt;

    // Collision
    int cellX = int(p.pos.x * size);
    int cellY = int(p.pos.y * size);

    cellX = max(0, min(size - 1, cellX));
    cellY = max(0, min(size - 1, cellY));

    if (cellType[gridIdx(cellX, cellY)] == 0) {
        float cellMinX = cellX / size;
        float cellMaxX = (cellX + 1) / size;
        float cellMinY = cellY / size;
        float cellMaxY = (cellY + 1) / size;

        // The 2.0f is a safety to ensure the particle does not try to get out of the box
        float distLeft   = (p.pos.x < h        || cellType[gridIdx(cellX-1, cellY)] == 0) ? 2.0f : p.pos.x - cellMinX;
        float distRight  = (p.pos.x > 1.0f - h || cellType[gridIdx(cellX+1, cellY)] == 0) ? 2.0f : cellMaxX - p.pos.x;
        float distBottom = (p.pos.y < h        || cellType[gridIdx(cellX, cellY-1)] == 0) ? 2.0f : p.pos.y - cellMinY;
        float distTop    = (p.pos.y > 1.0f - h || cellType[gridIdx(cellX, cellY+1)] == 0) ? 2.0f : cellMaxY - p.pos.y;

        float minDist = distLeft;
        int side = 0;

        if (distRight < minDist)  { minDist = distRight;  side = 1; }
        if (distBottom < minDist) { minDist = distBottom; side = 2; }
        if (distTop < minDist)    { minDist = distTop;    side = 3; }

        float eps = 1e-4f;

        switch (side) {
            case 0: // Left edge
                p.pos.x = cellMinX - eps;
                p.vel.x = 0; 
                break;
            case 1: // Right edge
                p.pos.x = cellMaxX + eps;
                p.vel.x = 0;
                break;
            case 2: // Bottom edge
                p.pos.y = cellMinY - eps;
                p.vel.y = 0;
                break;
            case 3: // Top edge
                p.pos.y = cellMaxY + eps;
                p.vel.y = 0;
                break;
        }
        p.pos.x = min(1.0f - h, max(h, p.pos.x));
        p.pos.y = min(1.0f - h, max(h, p.pos.y));
    }

    particles[idx] = p;
}
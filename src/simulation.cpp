#include "simulation.h"


// === MAIN STEPS ===

void Simulation::p2g() {
    // RESET GRIDS
    grid.us = grid.new_us;
    grid.vs = grid.new_vs;

    grid.new_us.assign(size * (size + 1), 0.0f);
    grid.new_vs.assign((size + 1) * size, 0.0f); 

    grid.uMasses.assign(size * (size + 1), 0.0f);
    grid.vMasses.assign((size + 1) * size, 0.0f);

    for (auto& p : particles) {
        float px = p.pos.x * size;
        float py = p.pos.y * size;

        // U-VELOCITIES
        float ux = px;
        float uy = py - 0.5f;
        
        int ui = (int)ux;
        int uj = (int)uy;
        float uwx = ux - ui;
        float uwy = uy - uj;

        // Bilinear weights for U
        float weights_u[4] = {
            (1 - uwx) * (1 - uwy), // (i, j)
            uwx * (1 - uwy),       // (i+1, j)
            (1 - uwx) * uwy,       // (i, j+1)
            uwx * uwy              // (i+1, j+1)
        };

        // Transfer U
        int u_indices[4] = { grid.uIdx(ui, uj), grid.uIdx(ui+1, uj), grid.uIdx(ui, uj+1), grid.uIdx(ui+1, uj+1) };
        for(int k=0; k<4; ++k) {
            grid.new_us[u_indices[k]] += weights_u[k] * p.vel.x;
            grid.uMasses[u_indices[k]] += weights_u[k];
        }

        // V-VELOCITIES
        float vx = px - 0.5f;
        float vy = py;

        int vi = (int)vx;
        int vj = (int)vy;
        float vwx = vx - vi;
        float vwy = vy - vj;

        // Bilinear weights for V
        float weights_v[4] = {
            (1 - vwx) * (1 - vwy), // (i, j)
            vwx * (1 - vwy),       // (i+1, j)
            (1 - vwx) * vwy,       // (i, j+1)
            vwx * vwy              // (i+1, j+1)
        };

        // Transfer V
        int v_indices[4] = { grid.vIdx(vi, vj), grid.vIdx(vi+1, vj), grid.vIdx(vi, vj+1), grid.vIdx(vi+1, vj+1) };
        for(int k=0; k<4; ++k) {
            grid.new_vs[v_indices[k]] += weights_v[k] * p.vel.y;
            grid.vMasses[v_indices[k]] += weights_v[k];
        }
    }

    // Normalize
    for (int k = 0; k < grid.new_us.size(); k++) {
        if (grid.uMasses[k] > 1e-9f) grid.new_us[k] /= grid.uMasses[k];
    }
    for (int k = 0; k < grid.new_vs.size(); k++) {
        if (grid.vMasses[k] > 1e-9f) grid.new_vs[k] /= grid.vMasses[k];
    }
}

void Simulation::applyGravity(float dt) {
    for (int k = 0; k < (size+1)*size; k++) {
        grid.new_vs[k] += GRAVITY * dt;
    }
}

void Simulation::computeDivergences(float dt) {
    for (int k = 0; k < size; k++) {
        grid.new_us[grid.uIdx(0,k)] = 0;
        grid.new_us[grid.uIdx(size,k)] = 0;

        grid.new_vs[grid.vIdx(k,0)] = 0;
        grid.new_vs[grid.vIdx(k,size)] = 0;
    }

    for (int i = 1; i < size-1; i++) {
        for (int j = 1; j < size-1; j++) {
            float div = 0.0f;

            div += grid.new_us[grid.uIdx(i + 1, j)];
            div -= grid.new_us[grid.uIdx(i, j)];

            div += grid.new_vs[grid.vIdx(i, j + 1)];
            div -= grid.new_vs[grid.vIdx(i, j)];

            grid.divergence[grid.gridIdx(i, j)] = div/ h * RHO / dt;
        }
    }
}

void Simulation::computePressures(float dt) {
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {

            int activeFaces = 0;
            float sumPressures = 0.0f;
            

            // RIGHT
            if (i < size - 1) {
                sumPressures += grid.pressure[grid.gridIdx(i+1,j)];
                activeFaces++;
            }
            // LEFT
            if (i > 0) {
                sumPressures += grid.pressure[grid.gridIdx(i-1,j)];
                activeFaces++;
            }

            // TOP
            if (j < size - 1) {
                sumPressures += grid.pressure[grid.gridIdx(i,j+1)];
                activeFaces++;
            }
            // BOTTOM
            if (j > 0) {
                sumPressures += grid.pressure[grid.gridIdx(i,j-1)];
                activeFaces++;
            }

            int idx = grid.gridIdx(i,j);

            grid.pressure[idx] = (sumPressures - h*h*grid.divergence[idx]) / activeFaces;

        }
    }
}

void Simulation::solvePressure(float dt) {
    // Pressure solving iterations
    for (int k = 0; k < 30; k++) {
        computePressures(dt);
    }
}

void Simulation::applyPressure(float dt) {
    float K = dt / RHO / h;

    for (int i = 1; i < size; i++) {
        for (int j = 0; j < size; j ++) {
            grid.new_us[grid.uIdx(i,j)] -= K*(grid.pressure[grid.gridIdx(i,j)] - grid.pressure[grid.gridIdx(i-1,j)]);
        }
    }

    for (int i = 0; i < size; i++) {
        for (int j = 1; j < size; j ++) {
            grid.new_vs[grid.vIdx(i,j)] -= K*(grid.pressure[grid.gridIdx(i,j)] - grid.pressure[grid.gridIdx(i,j-1)]);
        }
    }
}

void Simulation::g2p(float dt) {
    float alpha = 0.95f; // Flip ratio

    for (auto& p : particles) {
        p.vel = glm::vec2(0.0f);

        float px = p.pos.x * size;
        float py = p.pos.y * size;

        // U-VELOCITIES
        float ux = px;
        float uy = py - 0.5f;
        
        int ui = (int)ux;
        int uj = (int)uy;
        float uwx = ux - ui;
        float uwy = uy - uj;

        // Bilinear weights for U
        float weights_u[4] = {
            (1 - uwx) * (1 - uwy), // (i, j)
            uwx * (1 - uwy),       // (i+1, j)
            (1 - uwx) * uwy,       // (i, j+1)
            uwx * uwy              // (i+1, j+1)
        };
        int u_indices[4] = { grid.uIdx(ui, uj), grid.uIdx(ui+1, uj), grid.uIdx(ui, uj+1), grid.uIdx(ui+1, uj+1) };
        
        // Transfer U
        float pic_u = 0.0f;
        float flip_u = p.vel.x;
        
        for(int k=0; k<4; ++k) {
            float new_u = grid.new_us[u_indices[k]];
            float old_u = grid.us[u_indices[k]];

            pic_u += new_u * weights_u[k];
            flip_u += (new_u - old_u) * weights_u[k];
        }

        // V-VELOCITIES
        float vx = px - 0.5f;
        float vy = py;

        int vi = (int)vx;
        int vj = (int)vy;
        float vwx = vx - vi;
        float vwy = vy - vj;

        // Bilinear weights for V
        float weights_v[4] = {
            (1 - vwx) * (1 - vwy), // (i, j)
            vwx * (1 - vwy),       // (i+1, j)
            (1 - vwx) * vwy,       // (i, j+1)
            vwx * vwy              // (i+1, j+1)
        };
        int v_indices[4] = { grid.vIdx(vi, vj), grid.vIdx(vi+1, vj), grid.vIdx(vi, vj+1), grid.vIdx(vi+1, vj+1) };
        
        // Transfer V
        float pic_v = 0.0f;
        float flip_v = p.vel.y;
        
        for(int k=0; k<4; ++k) {
            float new_v = grid.new_vs[v_indices[k]];
            float old_v = grid.vs[v_indices[k]];

            pic_v += new_v * weights_v[k];
            flip_v += (new_v - old_v) * weights_v[k];
        }

        p.vel.x = (1.0f - alpha) * pic_u + alpha * flip_u;
        p.vel.y = (1.0f - alpha) * pic_v + alpha * flip_v;
    }
}


void Simulation::advectParticles(float dt) {
    for (auto& p : particles) {
        glm::vec2 v1 = sampleVelocityFromGrid(p.pos);
        
        glm::vec2 midPos = p.pos + v1 * (dt * 0.5f);
        
        glm::vec2 v2 = sampleVelocityFromGrid(midPos);
        
        p.pos += v2 * dt;

        // Boundaries
        if (p.pos.x <= h) {
            p.pos.x = h+0.01f;
            p.vel.x *= -0.8f;
        }
        if (p.pos.x >= 1-h) {
            p.pos.x = 0.999f-h;
            p.vel.x *= -0.8f;
        }

        if (p.pos.y <= h) {
            p.pos.y = h+0.01f;
            p.vel.y *= -0.8f;
        }
        if (p.pos.y >= 1-h) {
            p.pos.y = 0.999f-h;
            p.vel.y *= -0.8f;
        }
    }
}


// === HELPERS ===

float Simulation::interpolate(float x, float y, const std::vector<float>& gridValues, bool isU) {
    int i = (int)std::floor(x);
    int j = (int)std::floor(y);
    
    i = std::max(0, std::min(i, (isU ? size : size - 1)));
    j = std::max(0, std::min(j, (isU ? size - 1 : size)));

    float wx = x - i;
    float wy = y - j;

    float v00 = gridValues[isU ? grid.uIdx(i, j) : grid.vIdx(i, j)];
    float v10 = gridValues[isU ? grid.uIdx(i+1, j) : grid.vIdx(i+1, j)];
    float v01 = gridValues[isU ? grid.uIdx(i, j+1) : grid.vIdx(i, j+1)];
    float v11 = gridValues[isU ? grid.uIdx(i+1, j+1) : grid.vIdx(i+1, j+1)];

    return (1-wx)*(1-wy)*v00 + wx*(1-wy)*v10 + (1-wx)*wy*v01 + wx*wy*v11;
}

glm::vec2 Simulation::sampleVelocityFromGrid(glm::vec2 pos) {
    float px = pos.x * size;
    float py = pos.y * size;

    float ux = px;
    float uy = py - 0.5f;
    float u = interpolate(ux, uy, grid.new_us, true); 

    float vx = px - 0.5f;
    float vy = py;
    float v = interpolate(vx, vy, grid.new_vs, false);

    return glm::vec2(u, v);
}
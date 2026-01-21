#include "simulation.h"


// === MAIN STEPS ===

void Simulation::p2g() {

    // DEBUG
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            grid.interpolatedVelocities[grid.gridIdx(i,j)].x = 0.5f * (grid.new_us[grid.uIdx(i,j)]+grid.new_us[grid.uIdx(i+1,j)]);
            grid.interpolatedVelocities[grid.gridIdx(i,j)].y = 0.5f * (grid.new_vs[grid.vIdx(i,j)]+grid.new_vs[grid.vIdx(i,j+1)]);
        }
    }

    // RESET GRIDS
    grid.us.assign(size * (size + 1), 0.0f);
    grid.vs.assign((size + 1) * size, 0.0f); 

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
            if (u_indices[k] < 0 || u_indices[k] >= grid.total_size) continue;
            grid.us[u_indices[k]] += weights_u[k] * p.vel.x;
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
            if (v_indices[k] < 0 || v_indices[k] >= grid.total_size) continue;
            grid.vs[v_indices[k]] += weights_v[k] * p.vel.y;
            grid.vMasses[v_indices[k]] += weights_v[k];
        }

    }

    // Normalize
    for (int k = 0; k < grid.us.size(); k++) {
        if (grid.uMasses[k] > 1e-9f) grid.us[k] /= grid.uMasses[k];
    }
    for (int k = 0; k < grid.vs.size(); k++) {
        if (grid.vMasses[k] > 1e-9f) grid.vs[k] /= grid.vMasses[k];
    }
}

void Simulation::applyGravity(float dt) {
    for (int k = 0; k < (size+1)*size; k++) {
        if (grid.vMasses[k] > 1e-2)
        grid.new_vs[k] = grid.vs[k] + GRAVITY * dt;
        grid.new_us[k] = grid.us[k];
    }
}

void Simulation::computeDivergences(float dt) {
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            if (grid.solidCells[grid.gridIdx(i, j)]) {
                // If a face borders a solid cell, the velocity is forced to 0
                grid.new_us[grid.uIdx(i + 1, j)] = 0.0f;
                grid.new_us[grid.uIdx(i, j)] = 0.0f;
                grid.new_vs[grid.vIdx(i, j + 1)] = 0.0f;
                grid.new_vs[grid.vIdx(i, j)] = 0.0f;
            }
        }
    }
    
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            if (grid.solidCells[grid.gridIdx(i, j)]) {
                grid.divergence[grid.gridIdx(i, j)] = 0;
                continue;
            }

            float u_right = grid.new_us[grid.uIdx(i + 1, j)];
            float u_left  = grid.new_us[grid.uIdx(i, j)];
            float v_top   = grid.new_vs[grid.vIdx(i, j + 1)];
            float v_bot   = grid.new_vs[grid.vIdx(i, j)];

            float div = (u_right - u_left) + (v_top - v_bot);
            grid.divergence[grid.gridIdx(i, j)] = (div / h) * (RHO / dt);
        }
    }
}

void Simulation::solvePressure(float dt) {
    int n = grid.total_size;
    std::vector<float> r(n, 0.0f);
    std::vector<float> d(n, 0.0f);
    std::vector<float> Ad(n, 0.0f);
    std::vector<float> b(n, 0.0f);

    // 1. Prepare RHS: b = -h^2 * divergence
    for (int i = 0; i < n; i++) {
        if (!grid.solidCells[i]) {
            b[i] = -h * h * grid.divergence[i];
        }
        grid.pressure[i] = 0.0f; // Initial guess p_0 = 0
    }

    // 2. Initial residual r_0 = b - A*p_0. Since p_0 is 0, r_0 = b
    r = b; 
    d = r; // Initial direction d_0 = r_0
    
    double deltaNew = dotProduct(r, r);
    double epsilon = 1e-6; // Convergence threshold
    int maxIter = 100;

    for (int k = 0; k < maxIter; k++) {
        if (deltaNew < epsilon)  {
            std::cout << "CG converged in " << k << " iterations. Residual: " << deltaNew << std::endl;
            break;
        }

        // Ad = A * d
        applyA(d, Ad);

        // alpha = r^T * r / (d^T * A * d)
        double dAd = dotProduct(d, Ad);
        float alpha = (dAd == 0) ? 0.0f : (float)(deltaNew / dAd);

        // p = p + alpha * d
        // r = r - alpha * Ad
        for (int i = 0; i < n; i++) {
            grid.pressure[i] += alpha * d[i];
            r[i] -= alpha * Ad[i];
        }

        double deltaOld = deltaNew;
        deltaNew = dotProduct(r, r);

        // beta = deltaNew / deltaOld
        float beta = (float)(deltaNew / deltaOld);

        // d = r + beta * d
        for (int i = 0; i < n; i++) {
            d[i] = r[i] + beta * d[i];
        }
    }
}

void Simulation::applyPressure(float dt) {
    float K = dt / RHO / h;

    // U VELOCITIES
    for (int i = 1; i < size; i++) {
        for (int j = 1; j < size - 1; j++) {
            if (!grid.solidCells[grid.gridIdx(i, j)] && !grid.solidCells[grid.gridIdx(i - 1, j)]) {
                grid.new_us[grid.uIdx(i, j)] -= K * (grid.pressure[grid.gridIdx(i, j)] - grid.pressure[grid.gridIdx(i - 1, j)]);
            } else {
                grid.new_us[grid.uIdx(i, j)] = 0.0f; // Solid boundary
            }
        }
    }

    // V VELOCITIES
    for (int i = 1; i < size - 1; i++) {
        for (int j = 1; j < size; j++) {
            if (!grid.solidCells[grid.gridIdx(i, j)] && !grid.solidCells[grid.gridIdx(i, j-1)]) {
                grid.new_vs[grid.vIdx(i, j)] -= K * (grid.pressure[grid.gridIdx(i, j)] - grid.pressure[grid.gridIdx(i, j-1)]);
            } else {
                grid.new_vs[grid.vIdx(i, j)] = 0.0f; // Solid boundary
            }
        }
    }

    computeDivergences(dt); // just to debug
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
            if (u_indices[k] < 0 || u_indices[k] >= grid.total_size) continue;
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
            if (v_indices[k] < 0 || v_indices[k] >= grid.total_size) continue;
            float new_v = grid.new_vs[v_indices[k]];
            float old_v = grid.vs[v_indices[k]];

            pic_v += new_v * weights_v[k];
            flip_v += (new_v - old_v) * weights_v[k];
        }

        p.vel.x = (1.0f - alpha) * pic_u + alpha * flip_u;
        p.vel.y = (1.0f - alpha) * pic_v + alpha * flip_v;

        p.pos += p.vel * dt;

        // Collision
        int cellX = (int)(p.pos.x * size);
        int cellY = (int)(p.pos.y * size);

        cellX = std::max(0, std::min(size - 1, cellX));
        cellY = std::max(0, std::min(size - 1, cellY));

        if (grid.solidCells[grid.gridIdx(cellX, cellY)]) {
            float fluidX = (float)cellX / size;
            float fluidY = (float)cellY / size;

            if (cellX == 0) { p.pos.x = h + 0.001f; p.vel.x = 0; }
            if (cellX == size - 1) { p.pos.x = 1.0f - h - 0.001f; p.vel.x = 0; }
            if (cellY == 0) { p.pos.y = h + 0.001f; p.vel.y = 0; }
            if (cellY == size - 1) { p.pos.y = 1.0f - h - 0.001f; p.vel.y = 0; }
        }
    }
}


void Simulation::applyA(const std::vector<float>& x, std::vector<float>& Ax) {
    for (int i = 0; i < size; i++) {
        for (int j = 0; j < size; j++) {
            int idx = grid.gridIdx(i, j);
            if (grid.solidCells[idx]) {
                Ax[idx] = 0.0f;
                continue;
            }

            float val = 0.0f;
            int fluidNeighbors = 0;
            int neighbors[4][2] = {{i+1, j}, {i-1, j}, {i, j+1}, {i, j-1}};

            for (auto& n : neighbors) {
                int ni = n[0], nj = n[1];
                // Boundary check
                if (ni >= 0 && ni < size && nj >= 0 && nj < size) {
                    if (!grid.solidCells[grid.gridIdx(ni, nj)]) {
                        val -= x[grid.gridIdx(ni, nj)];
                        fluidNeighbors++;
                    }
                }
            }
            Ax[idx] = (fluidNeighbors * x[idx]) + val;
        }
    }
}

float Simulation::dotProduct(const std::vector<float>& a, const std::vector<float>& b) {
    float result = 0.0;
    for (size_t i = 0; i < a.size(); i++) {
        result += a[i] * b[i];
    }
    return result;
}
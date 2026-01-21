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

void Simulation::computePressures(float dt) {
    for (int i = 1; i < size - 1; i++) {
        for (int j = 1; j < size - 1; j++) {
            if (grid.solidCells[grid.gridIdx(i, j)]) continue; // Skip solid cells

            float sumPressures = 0.0f;
            int fluidNeighbors = 0;

            int neighbors[4][2] = {{i+1, j}, {i-1, j}, {i, j+1}, {i, j-1}};
            
            for (auto& n : neighbors) {
                int ni = n[0];
                int nj = n[1];
                if (!grid.solidCells[grid.gridIdx(ni, nj)]) {
                    sumPressures += grid.pressure[grid.gridIdx(ni, nj)];
                    fluidNeighbors++;
                }
            }

            if (fluidNeighbors > 0) {
                if (overRelaxation) {
                    int idx = grid.gridIdx(i, j);     
                    float p_gs = (sumPressures - (h * h * grid.divergence[idx])) / fluidNeighbors;         
                    grid.pressure[idx] = (1.0f - OMEGA) * grid.pressure[idx] + OMEGA * p_gs;
                } else {
                    int idx = grid.gridIdx(i, j);
                    grid.pressure[idx] = (sumPressures - (h * h * grid.divergence[idx])) / fluidNeighbors;
                }
            }
        }
    }
}

void Simulation::solvePressure(float dt) {
    for (int k = 0; k < grid.total_size; k++) grid.pressure[k] = 0;

    // Pressure solving iterations
    for (int k = 0; k < 100; k++) {
        computePressures(dt);
    }
    computeR();

    
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

void Simulation::computeR() {
    double sumSquaredResiduals = 0.0;
    int activeCells = 0;

    for (int i = 1; i < size - 1; i++) {
        for (int j = 1; j < size - 1; j++) {
            if (grid.solidCells[grid.gridIdx(i, j)]) continue;

            float p_ij = grid.pressure[grid.gridIdx(i, j)];
            float sumPNeighbors = 0.0f;
            int fluidNeighbors = 0;

            int neighbors[4][2] = {{i+1, j}, {i-1, j}, {i, j+1}, {i, j-1}};
            for (auto& n : neighbors) {
                if (!grid.solidCells[grid.gridIdx(n[0], n[1])]) {
                    sumPNeighbors += grid.pressure[grid.gridIdx(n[0], n[1])];
                    fluidNeighbors++;
                }
            }

            if (fluidNeighbors > 0) {
                float targetP = (sumPNeighbors - (h * h * grid.divergence[grid.gridIdx(i, j)])) / fluidNeighbors;
                float localResidual = targetP - p_ij; 
                
                sumSquaredResiduals += localResidual * localResidual;
                activeCells++;
            }
        }
    }
    double residualNorm = std::sqrt(sumSquaredResiduals / (activeCells > 0 ? activeCells : 1));
    std::cout << "Residual Norm : " << residualNorm << std::endl;
}